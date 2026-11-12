"""Decoder for the Roborock S5 MCU sensor stream captured by tap/lidartap.so (port 7 = get_package payloads).

A package is a TLV sequence [id u8][len u8][data]. IDs come from librr_plugin_chassis.so's jump table; field
layouts were worked out from a live capture on 2026-11-09 (see README). Self-check: python sensors.py
"""
import math
import re
import struct

ODO, BATTERY, DOCK_IR, DOCK, BUMPER, DROP, CLIFF, LDS_BUMPER, WALL, FAN, WHEEL, BRUSH, SWEEP, ERROR = \
    7, 8, 19, 32, 33, 34, 35, 37, 64, 80, 81, 82, 83, 4
LOG_IDS = (247, 248, 249, 250)

# ponytail: thresholds are heuristics tuned on one short run (brush idles ~300, wheels ~150 raw units);
# calibration knobs - tune after a few full cleanings with known carpets.
BRUSH_CARPET_RATIO = 1.5    # brush current this many times its baseline for BRUSH_HOLD_S -> carpet / more drag
BRUSH_JAM_RATIO = 2.6       # ... -> something wound into the brush (cable, sock)
BRUSH_HOLD_S = 1.0
WHEEL_LOAD_RATIO = 2.2      # wheel current vs baseline -> robot is pushing against something / stuck
WHEEL_HOLD_S = 0.7
TILT_DEG = 4.5              # climbing a threshold or a thick rug
COOLDOWN_S = 4.0


def tlv(payload):
    j = 0
    while j + 2 <= len(payload):
        n = payload[j + 1]
        if j + 2 + n > len(payload):
            return
        yield payload[j], payload[j + 2:j + 2 + n]
        j += 2 + n


class Level:
    """Fast vs slow average of a motor current; fires when fast stays above ratio * slow for hold seconds."""

    def __init__(self, floor):
        self.fast = self.slow = None
        self.floor = floor  # below this the motor is off: don't learn a baseline from it
        self.since = {}

    def update(self, v, t):
        if v < self.floor:
            self.fast = None
            self.since.clear()
            return
        self.fast = v if self.fast is None else self.fast + 0.2 * (v - self.fast)
        if self.slow is None:
            self.slow = v
        elif self.fast < 1.3 * self.slow:  # only learn the baseline from normal running
            self.slow += 0.004 * (v - self.slow)
        elif self.fast < self.slow:
            self.slow = self.fast

    def above(self, ratio, hold, t):
        if self.fast is None or self.slow is None or self.fast < ratio * self.slow:
            self.since.pop(ratio, None)
            return False
        return t - self.since.setdefault(ratio, t) >= hold


class Sensors:
    def __init__(self):
        self.v = {"bumper_l": 0, "bumper_r": 0, "cliff": 0, "drop": 0, "lds_bumper": 0, "dock": 0, "wall": None,
                  "wheel_l": 0, "wheel_r": 0, "brush": 0, "tilt": 0.0, "gyro_z": 0.0, "enc_l": 0, "enc_r": 0,
                  "mv": None, "ma": None, "temp": None, "soc": None, "fan_duty": None, "brush_duty": None,
                  "error": 0}
        self.brush_lvl, self.wheel_lvl = Level(60), Level(40)
        self.events = []
        self.last = {}      # event kind -> last time fired
        self.win = {}       # max values since the last window() call
        self.tilt_since = None
        self.packets = 0

    def _event(self, t, kind, cooldown=COOLDOWN_S, **extra):
        if t - self.last.get(kind, -1e9) < cooldown:
            return
        self.last[kind] = t
        self.events.append({"k": kind, "t": t, **extra})

    def _max(self, key, val):
        if val is not None and val > self.win.get(key, -1e9):
            self.win[key] = val

    def feed(self, payload, t):
        """payload = one get_package result, t = seconds (any monotonic-ish clock, used for holds/cooldowns)."""
        v = self.v
        self.packets += 1
        for tid, d in tlv(payload):
            if tid == BUMPER and len(d) >= 2:
                # ponytail: byte order L/R assumed (both fired together in every captured hit); MCU log lines
                # "L Bumper On" / "R Bumper On" can confirm it
                l, r = d[0], d[1]
                if (l and not v["bumper_l"]) or (r and not v["bumper_r"]):
                    self._event(t, "bumper", 0.3, side="L+P" if l and r else "L" if l else "P")
                v["bumper_l"], v["bumper_r"] = l, r
            elif tid == CLIFF and len(d) >= 2:
                m = d[0] | d[1] << 8
                if m and not v["cliff"]:
                    self._event(t, "cliff", 1.0, mask=m)
                v["cliff"] = m
            elif tid == DROP and len(d) >= 2:
                m = d[0] | d[1] << 8
                if m and not v["drop"]:
                    self._event(t, "drop", 1.0, mask=m)
                v["drop"] = m
            elif tid == LDS_BUMPER and len(d) >= 1:
                if d[0] and not v["lds_bumper"]:
                    self._event(t, "lds_bumper", 1.0)
                v["lds_bumper"] = d[0]
            elif tid == DOCK and len(d) >= 1:
                v["dock"] = d[0]
            elif tid == WALL and len(d) == 2:
                v["wall"] = struct.unpack("<h", d)[0]
                self._max("wall", v["wall"])
            elif tid == WHEEL and len(d) == 8:
                v["wheel_l"], v["wheel_r"] = struct.unpack_from("<HH", d)
                w = max(v["wheel_l"], v["wheel_r"])
                self._max("wheel", w)
                self.wheel_lvl.update(w, t)
                if self.wheel_lvl.above(WHEEL_LOAD_RATIO, WHEEL_HOLD_S, t):
                    self._event(t, "drive_load", value=w, base=round(self.wheel_lvl.slow))
            elif tid == BRUSH and len(d) == 6:
                v["brush"] = struct.unpack_from("<H", d, 2)[0]
                self._max("brush", v["brush"])
                self.brush_lvl.update(v["brush"], t)
                if self.brush_lvl.above(BRUSH_JAM_RATIO, BRUSH_HOLD_S, t):
                    self._event(t, "brush_jam", value=v["brush"], base=round(self.brush_lvl.slow))
                elif self.brush_lvl.above(BRUSH_CARPET_RATIO, BRUSH_HOLD_S, t):
                    self._event(t, "carpet", value=v["brush"], base=round(self.brush_lvl.slow))
            elif tid == ODO and len(d) == 64:
                ax, ay, az, _, _, gz = struct.unpack_from("<6f", d)
                v["enc_l"], v["enc_r"] = struct.unpack_from("<ii", d, 40)
                v["gyro_z"] = round(gz, 3)
                if az > 1:
                    v["tilt"] = round(math.degrees(math.atan2(math.hypot(ax, ay), az)), 1)
                    self._max("tilt", v["tilt"])
                    if v["tilt"] < TILT_DEG:
                        self.tilt_since = None
                    elif self.tilt_since is None:
                        self.tilt_since = t
                    elif t - self.tilt_since >= 0.25:
                        self._event(t, "tilt", value=v["tilt"])
            elif tid == BATTERY and len(d) >= 5:
                v["mv"], v["ma"], v["soc"] = struct.unpack_from("<HHB", d)
            elif tid == ERROR and len(d) == 4:
                e = struct.unpack("<I", d)[0]
                if e and e != v["error"]:
                    self._event(t, "mcu_error", 10, code=e)
                v["error"] = e
            elif tid in LOG_IDS and len(d) > 1:
                self._log_line(d[1:].split(b"\0")[0].decode("latin1").strip(), t)

    def _log_line(self, s, t):
        v = self.v
        if m := re.search(r"I:\s*(-?\d+) mA,V:\s*(\d+) mV,T:\s*(-?\d+) C", s):
            v["ma"], v["mv"], v["temp"] = int(m[1]), int(m[2]), int(m[3])
        elif m := re.search(r"soc=(\d+),V=(\d+),I=(-?\d+)", s):
            v["soc"], v["mv"], v["ma"] = int(m[1]), int(m[2]), int(m[3])
        elif m := re.match(r"(Fan|Brush|Sweep) Duty = (\d+)", s):
            v[m[1].lower() + "_duty"] = int(m[2])
        elif "Open Circuit" in s or "Over Current" in s or "Stall" in s:
            self._event(t, "mcu_warn", 120, text=s[:80])

    def drain(self):
        ev, self.events = self.events, []
        return ev

    def window(self):
        """Peak values since the previous call (for one history frame)."""
        w, self.win = self.win, {}
        return w

    def snapshot(self):
        s = dict(self.v)
        s["brush_base"] = round(self.brush_lvl.slow) if self.brush_lvl.slow else None
        s["wheel_base"] = round(self.wheel_lvl.slow) if self.wheel_lvl.slow else None
        s["watts"] = round(s["mv"] * s["ma"] / 1e6, 1) if s["mv"] and s["ma"] is not None else None
        s["packets"] = self.packets
        return s


EVENT_PL = {
    "bumper": "Uderzenie zderzakiem", "cliff": "Czujnik krawędzi (schody/próg)", "drop": "Koło opadło / robot podniesiony",
    "lds_bumper": "Uderzenie wieżyczką lidaru", "carpet": "Większy opór szczotki (dywan?)",
    "brush_jam": "Szczotka mocno hamowana (kabel? skarpetka?)", "drive_load": "Zwiększony opór jazdy",
    "tilt": "Przechył (próg / gruby dywan)", "mcu_warn": "Ostrzeżenie mikrokontrolera", "mcu_error": "Błąd mikrokontrolera",
}


if __name__ == "__main__":
    h = bytes.fromhex
    pkt = lambda tid, data: bytes([tid, len(data)]) + data

    # real samples from the 2026-11-09 capture
    assert list(tlv(h("2102" "0101" "2302" "0000"))) == [(33, h("0101")), (35, h("0000"))]
    assert list(tlv(h("2105" "01"))) == []  # truncated item is ignored, no exception

    s = Sensors()
    s.feed(pkt(BUMPER, h("0000")), 0.0)
    s.feed(pkt(BUMPER, h("0101")), 1.0)
    s.feed(pkt(BUMPER, h("0101")), 1.04)     # still pressed -> no second event
    s.feed(pkt(BUMPER, h("0000")), 1.2)
    s.feed(pkt(BUMPER, h("0100")), 3.0)
    ev = s.drain()
    assert [(e["k"], e["side"]) for e in ev] == [("bumper", "L+P"), ("bumper", "L")], ev

    s.feed(pkt(BATTERY, h("0e40ad025d0000000000")), 4.0)
    assert (s.v["mv"], s.v["ma"], s.v["soc"]) == (16398, 685, 93)
    s.feed(pkt(249, b"\x81ADC: 384,3578,1862.I:1931 mA,V:15380 mV,T: 30 C.\r\n\0"), 4.1)
    assert (s.v["ma"], s.v["mv"], s.v["temp"]) == (1931, 15380, 30) and s.snapshot()["watts"] == 29.7
    s.feed(pkt(250, b"\x81Sweep: Open Circuit Possibly\r\n\0"), 4.2)
    assert s.drain()[0]["k"] == "mcu_warn"

    odo = bytearray(64)
    struct.pack_into("<6f", odo, 0, 0.06, 0.13, 9.67, 0, 0, 1.598)
    struct.pack_into("<ii", odo, 40, 75851, 76944)
    s.feed(pkt(ODO, bytes(odo)), 5.0)
    assert s.v["tilt"] < 1.5 and s.v["gyro_z"] == 1.598 and s.v["enc_l"] == 75851

    # brush: 20 s of normal running at ~300, then a rug (x1.7), then a jam (x3)
    t = 10.0
    brush = lambda cur: pkt(BRUSH, struct.pack("<HHH", 0xFFFF, cur, 19456))
    for _ in range(1000):
        s.feed(brush(300), t); t += 0.02
    assert not s.drain()
    for _ in range(100):
        s.feed(brush(520), t); t += 0.02
    assert [e["k"] for e in s.drain()] == ["carpet"]
    t += 10
    for _ in range(150):
        s.feed(brush(950), t); t += 0.02
    assert "brush_jam" in [e["k"] for e in s.drain()]
    assert s.window()["brush"] == 950 and s.window() == {}
    print("sensors: ok")
