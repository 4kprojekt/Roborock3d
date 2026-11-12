"""Decoder for the Roborock S5 lidar (LDS) byte stream captured by tap/lidartap.so, plus the live SLAM pose.

LDS packet = 22 bytes, Neato XV-11 style:
  0xFA | index 0xA0..0xF9 (4 degrees each) | speed u16 LE (rpm * 64) |
  4 x [dist u16 LE: bits 0-13 mm, bit 15 invalid, bit 14 strength warning ; strength u16 LE] | checksum u16 LE
Other bytes in the stream (0xAA info packets) are skipped by checksum resync.

SLAM pose = first 12 bytes of /run/shm/SharedMapInfo: u32 flags, i16 x_mm, i16 y_mm, f32 theta_rad.
Valetudo map coordinates: x_cm = 2560 + x_mm/10, y_cm = 2560 - y_mm/10, angle = 90 - degrees(theta)
(checked against Valetudo's robot_position on 2026-11-09, matched to the centimetre).
"""
import math
import struct
import time
from collections import deque

MAP_CENTER_CM = 2560


def checksum_ok(p):
    chk = 0
    for i in range(10):
        chk = (chk << 1) + p[2 * i] + (p[2 * i + 1] << 8)
    chk = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF
    return chk == p[20] | (p[21] << 8)


class LdsDecoder:
    """Feed raw bytes (with the robot-clock ms they arrived at); get full 360-degree scans back."""

    def __init__(self):
        self.buf = bytearray()
        self.points = [None] * 360  # angle -> (dist_mm, strength, valid, t_ms)
        self.last_index = -1
        self.rpm = 0.0
        self.good = self.bad = 0

    def feed(self, data, t_ms=0):
        """Returns completed scans: {"t", "rpm", "points": [[angle, mm, strength, valid, t_ms], ...]}"""
        self.buf += data
        scans, b, i = [], self.buf, 0
        while i + 22 <= len(b):
            if b[i] != 0xFA or not 0xA0 <= b[i + 1] <= 0xF9:
                i += 1
                continue
            p = b[i:i + 22]
            if not checksum_ok(p):
                self.bad += 1
                i += 1
                continue
            self.good += 1
            idx = p[1] - 0xA0
            if idx <= self.last_index:  # wrapped (or repeated) -> one rotation done
                scans.append(self._flush())
            self.last_index = idx
            self.rpm = struct.unpack_from("<H", p, 2)[0] / 64
            for j in range(4):
                raw, strength = struct.unpack_from("<HH", p, 4 + 4 * j)
                self.points[idx * 4 + j] = (raw & 0x3FFF, strength, not raw & 0x8000, t_ms)
            i += 22
        del b[:i]
        return scans

    def _flush(self):
        pts = [[a, *v] for a, v in enumerate(self.points) if v is not None]
        self.points = [None] * 360
        return {"t": time.time(), "rpm": round(self.rpm, 1), "points": pts}


def parse_pose(d):
    """24 bytes of SharedMapInfo -> (x_cm, y_cm, angle_deg) in Valetudo map coordinates."""
    _, x, y, th = struct.unpack_from("<Ihhf", d)
    return MAP_CENTER_CM + x / 10, MAP_CENTER_CM - y / 10, (90 - math.degrees(th)) % 360


class PoseTrack:
    """Recent poses (robot-clock ms) with interpolation, so each lidar point gets the pose it was taken at."""

    def __init__(self, keep=120):
        self.h = deque(maxlen=keep)

    def add(self, t_ms, x, y, angle):
        if self.h:
            _, px, py, _ = self.h[-1]
            if math.hypot(x - px, y - py) > 60:  # torn read of the shared file / relocalisation jump
                self.h.clear()
        self.h.append((t_ms, x, y, angle))

    def at(self, t_ms, max_gap_ms=400):
        h = self.h
        if not h or t_ms < h[0][0] - max_gap_ms or t_ms > h[-1][0] + max_gap_ms:
            return None
        prev = h[0]
        for cur in h:
            if cur[0] >= t_ms:
                if cur[0] == prev[0]:
                    return cur[1:]
                f = min(1.0, max(0.0, (t_ms - prev[0]) / (cur[0] - prev[0])))
                da = (cur[3] - prev[3] + 180) % 360 - 180
                return (prev[1] + f * (cur[1] - prev[1]), prev[2] + f * (cur[2] - prev[2]), (prev[3] + f * da) % 360)
            prev = cur
        return h[-1][1:]

    def last(self):
        return self.h[-1] if self.h else None

    def turn_rate(self, window_ms=400):
        """deg/s over the last window (scans taken while spinning fast are too smeared to map with)."""
        if len(self.h) < 2:
            return 0.0
        end = self.h[-1]
        start = next((p for p in self.h if p[0] >= end[0] - window_ms), end)
        dt = (end[0] - start[0]) / 1000
        return abs((end[3] - start[3] + 180) % 360 - 180) / dt if dt > 0 else 0.0


# Calibration, fitted 2026-11-09 on ~220 scans against the map's structural walls (server /local/debug):
# - lidar 0 deg sits 10 deg off Valetudo's robot angle (standing still: best 9-10 deg)
# - the SLAM pose in SharedMapInfo is published late: a point measured at t matches the pose stamped t + lag
# ponytail: calibration knobs - re-fit with /local/debug if scans look rotated or smeared in turns.
LDS_ANGLE_OFFSET_DEG = 10
POSE_LAG_MS = 200


def to_world(scan, x_cm, y_cm, angle_deg, offset_deg=LDS_ANGLE_OFFSET_DEG):
    """Valid scan points -> map coordinates [x_cm, y_cm, dist_mm, strength, lidar_angle] (map y grows downwards)."""
    out = []
    for a, mm, strength, valid, *_ in scan["points"]:
        if not valid or mm == 0:
            continue
        th = math.radians(angle_deg + offset_deg + a)
        out.append([round(x_cm + mm / 10 * math.cos(th)), round(y_cm + mm / 10 * math.sin(th)), mm, strength, a])
    return out


def rays(scan, track, offset_deg=LDS_ANGLE_OFFSET_DEG, lag_ms=POSE_LAG_MS):
    """Every scan direction with the pose it was measured from (motion-compensated):
    [ox_cm, oy_cm, dir_rad, dist_cm or None (no echo), strength]. Empty if the pose track doesn't cover it."""
    out = []
    for a, mm, strength, valid, t_ms in scan["points"]:
        pose = track.at(t_ms + lag_ms)
        if pose is None:
            continue
        th = math.radians(pose[2] + offset_deg + a)
        out.append([pose[0], pose[1], th, mm / 10 if valid and mm else None, strength])
    return out


if __name__ == "__main__":  # self-check: python lds.py [capture.lds]
    import sys
    # hand-made packet: index 0xA0, 300 rpm, 4 points of 1000/1001/1002/0x8000(invalid) mm
    body = bytearray([0xFA, 0xA0]) + struct.pack("<H", 300 * 64)
    for d in (1000, 1001, 1002, 0x8000):
        body += struct.pack("<HH", d, 50)
    chk = 0
    for i in range(10):
        chk = (chk << 1) + body[2 * i] + (body[2 * i + 1] << 8)
    body += struct.pack("<H", ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF)
    assert checksum_ok(body)
    dec = LdsDecoder()
    assert dec.feed(b"\x00\xaa" + body, 500) == [] and dec.points[0] == (1000, 50, True, 500) and dec.points[3][2] is False
    nxt = bytearray(body); nxt[1] = 0xA0
    done = dec.feed(bytes(nxt), 700)
    assert len(done) == 1  # same index again = new rotation
    # 1 m straight ahead at angle 0, robot at (100, 200) cm facing 90 deg (down the map), no offset -> (100, 300)
    assert to_world({"points": [[0, 1000, 9, True, 0], [1, 500, 9, False, 0]]}, 100, 200, 90, 0) == [[100, 300, 1000, 9, 0]]

    # real pose bytes: slam (-18, -217) mm, theta -1.610 rad == Valetudo (2558, 2582) cm, 182 deg
    x, y, a = parse_pose(struct.pack("<Ihhf", 0x101, -18, -217, -1.610) + bytes(12))
    assert (round(x), round(y), round(a)) == (2558, 2582, 182), (x, y, a)

    tr = PoseTrack()
    tr.add(1000, 100, 100, 350)
    tr.add(1100, 110, 100, 10)       # 20 deg turn through 0
    px, py, pa = tr.at(1050)
    assert (px, py) == (105, 100) and abs(pa - 0) < 1e-6, (px, py, pa)
    assert tr.at(5000) is None and round(tr.turn_rate()) == 200
    r = rays({"points": [[0, 1000, 9, True, 1050], [90, 0, 0, False, 1050]]}, tr, 0, 0)
    assert r[0][:2] == [105, 100] and r[0][3] == 100 and r[1][3] is None
    tr.add(1200, 900, 900, 10)       # jump -> history reset: nothing is interpolated across it
    assert len(tr.h) == 1 and tr.at(700) is None and tr.at(1150)[0] == 900

    if len(sys.argv) > 1:
        d = LdsDecoder()
        scans = d.feed(open(sys.argv[1], "rb").read())
        valid = [sum(1 for p in s["points"] if p[3]) for s in scans]
        print(f"packets ok={d.good} bad={d.bad} scans={len(scans)} rpm~{scans[-1]['rpm'] if scans else '-'} "
              f"valid points/scan median={sorted(valid)[len(valid) // 2] if valid else 0}")
    print("lds: ok")
