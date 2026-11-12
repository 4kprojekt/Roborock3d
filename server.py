#!/usr/bin/env python3
"""Roborock S5 / Valetudo control panel backend (stdlib only).

- serves static/ (the web UI); proxies GET /api/* and writes (PUT/POST/DELETE) per WRITE_RULES to the robot
- hotspots: places where the robot keeps hitting / climbing onto something (analysis.hotspots), live alerts
- program: procedural control - a list of steps (goto, forward/back/turn in manual control, look, clean...)
- poller: status / battery / Wi-Fi / map every second -> /local/state, /local/map, Wi-Fi coverage samples
- tap (SSH -> tap/lidartap_relay on the robot): raw lidar, MCU sensor packets and the 10 Hz SLAM pose
    -> live scan, self-updating obstacle cloud, mirror/glass detection, bumper/cliff/brush/wheel events
- recorder: every run is saved to data/sessions/<id>.jsonl (2 frames/s + events) for history and replay
- explore: drives through every room with GoTo (no cleaning) so the lidar sees the whole flat
- photos: room pictures pinned to a wall segment, shown in the 3D view

Usage:
  python server.py                                 # robot from ROBOROCK_URL (.env), tap via ROBOROCK_SSH_KEY
  python server.py --robot http://<robot-ip>       # or pass it directly
  python server.py --robot mock                    # fake robot, no hardware needed (no tap, no driving)
Settings: copy .env.example to .env (see README "Configuration").
"""
import argparse
import datetime
import json
import math
import os
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import Counter, deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import analysis
import lds
import sensors

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
DATA = ROOT / "data"
SAMPLES = DATA / "wifi_samples.jsonl"
SESSIONS = DATA / "sessions"
PHOTOS = DATA / "photos"
EVENTS_LOG = DATA / "events.jsonl"

CELL = 5                    # cm, obstacle cloud grid
CLOUD_MAX_CELLS = 300_000
HIT, CLEAR, CAP = 2, 1, 12  # a cell gains HIT per echo, loses CLEAR per ray passing through it
STABLE_S = 4                # an echo must stay this long before it counts as an obstacle ...
MOVING_MAX_S = 25           # ... and one that vanished sooner than this was something moving (person, pet, door leaf)
MIN_HITS = 3                # cells below this are not reported (noise / just removed)
MAX_TURN_DEG_S = 70         # scans taken while spinning faster are not mapped
ACTIVE = {"cleaning", "moving", "returning", "manual_control", "paused"}
SESSION_END_AFTER_S = 30
FRAME_SCAN_POINTS = 90
WIFI_CELL_CM = 25
WIFI_MIN_MOVE_CM = 15
PHOTO_TYPES = {"image/jpeg": ".jpg", "image/png": ".png", "image/webp": ".webp"}
PHOTO_MAX_BYTES = 15 * 1024 * 1024
# writes the panel/MCP may proxy to Valetudo: every robot capability + timers, Valetudo events, log level.
# Valetudo's own settings, updater, MQTT, NTP, system stay refused. Capabilities that can wipe the map, cut the robot
# off the network or change firmware behaviour also need the header "X-Confirm: tak" (the UI asks first).
WRITE_RULES = [(r"/api/v2/robot/capabilities/\w+(/[\w/.-]*)?", ("PUT",)),
               (r"/api/v2/timers", ("POST",)), (r"/api/v2/timers/[\w-]+(/action)?", ("PUT", "DELETE")),
               (r"/api/v2/events/[\w-]+/interact", ("PUT",)), (r"/api/v2/valetudo/log/level", ("PUT",))]
CONFIRM_CAPS = ("MapResetCapability", "WifiConfigurationCapability", "PersistentMapControlCapability",
                "MapSnapshotCapability", "QuirksCapability", "VoicePackManagementCapability")


def write_check(method, path, confirmed):
    """-> None if the write may go to the robot, else the reason it is refused."""
    if not any(re.fullmatch(p, path) and method in ms for p, ms in WRITE_RULES):
        return "ta operacja nie jest dozwolona z panelu"
    m = re.match(r"/api/v2/robot/capabilities/(\w+)", path)
    if m and m[1] in CONFIRM_CAPS and not confirmed:
        return f"{m[1]} wymaga potwierdzenia (nagłówek X-Confirm: tak)"
    return None

lock = threading.RLock()
BASE = None   # robot base URL (None in mock mode)
FETCH = None  # path -> (status, content_type, body_bytes)
MAPRAW = {"bytes": b"{}", "nonce": None}
MAP = {"ps": 5, "walls": set(), "structural": set(), "islands": [], "near_struct": set(), "near_wall": set(),
       "walk": set(), "rooms": []}
TRACK = lds.PoseTrack()
POSE = {"x": None, "y": None, "angle": 0, "t": 0, "src": None}
SENS = sensors.Sensors()
SCAN = {"scan": None, "world": [], "pose": None, "no_echo": 0}
CLOUD = {}                    # (cx, cy) -> hits (0..CAP)
PHANTOM = Counter()           # (cx, cy) -> echoes that came from behind a structural wall
THROUGH, SEEN, MISS = Counter(), Counter(), Counter()
FAR = {}                      # wall px -> deque of echoes seen through it
BORN = {}                     # cloud cell -> time of its first echo
MOVED = deque(maxlen=600)     # (t, x_cm, y_cm) of cells that appeared and were gone again within MOVING_MAX_S
PH_WALL = {}                  # phantom cell -> the wall pixel it was seen through
MIRROR = {"px": set()}
OPEN = {}                     # wall px -> -1..1: +1 = rays lately go THROUGH it (open door), -1 = echoes come FROM it
OPEN_AT = 0.3                 # above this the wall stretch is drawn as open
RC = "/api/v2/robot/capabilities/HighResolutionManualControlCapability"
RC_SIGN = {"turn": 1}         # +turn command = Valetudo angle grows (tested 2026-11-09); re-learned in rc_enable        # wall pixels of detected mirrors: echoes through them are dropped, not drawn
DOORS = {"list": [], "cells": {}, "hit": {}, "pass": {}}  # doorways of the map + newest echo / ray per position
REV = {"cloud": 0}
RAYS_DBG = deque(maxlen=60)   # every 4th scan, raw + pose history (for calibrating lidar angle offset / time lag)
DBG = {"n": 0}
PENDING = deque()             # scans waiting for the (late) SLAM pose of their moment
EVENTS = deque(maxlen=300)
WIFI = []                     # samples {t, x, y, signal}
STATUS = {"attrs": [], "value": None, "battery": None, "wifi": {}, "error": None}
SESSION = {"id": None, "last_active": 0}
EXPLORE = {"running": False, "stop": False, "route": [], "i": 0, "msg": "", "session": None}
TAP = {"connected": False, "last": 0}
_summary_cache = {}


def make_fetch(robot):
    if robot == "mock":
        from mock_robot import fake_api
        return fake_api
    base = robot.rstrip("/")

    def fetch(path):
        try:
            with urllib.request.urlopen(base + path, timeout=10) as r:
                return r.status, r.headers.get("Content-Type", "application/json"), r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers.get("Content-Type", "text/plain"), e.read()
    return fetch


def get_json(path):
    status, _, body = FETCH(path)
    if status != 200:
        raise RuntimeError(f"{path}: HTTP {status}")
    return json.loads(body)


def put_robot(path, body_bytes, method="PUT"):
    if not BASE:
        raise RuntimeError("tryb mock - brak robota")
    req = urllib.request.Request(BASE + path, data=body_bytes if method != "DELETE" else None, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def put_json(path, body):
    status, out = put_robot(path, json.dumps(body).encode())
    if status >= 300:
        raise RuntimeError(f"{path}: HTTP {status} {out[:120]!r}")


# ---------- events + sessions ----------

def append_jsonl(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, separators=(",", ":"), ensure_ascii=False) + "\n")


def add_event(kind, msg, **extra):
    ev = {**extra, "t": round(time.time(), 1), "k": kind, "msg": msg}
    if POSE["x"] is not None:
        ev.setdefault("x", round(POSE["x"]))
        ev.setdefault("y", round(POSE["y"]))
        ev.setdefault("a", round(POSE["angle"]))  # heading -> where the bumper touched (analysis.contact_point)
    with lock:
        EVENTS.append(ev)
        append_jsonl(EVENTS_LOG, ev)
        if SESSION["id"]:
            append_jsonl(SESSIONS / f"{SESSION['id']}.jsonl", {"ev": ev})
    if kind in analysis.HOT_KINDS and "x" in ev:
        hot_count(ev)


HOT = Counter()           # 50 cm cell of a contact point -> problem events there (all history)
HOT_ALERT = (3, 10, 25, 50, 100)


def hot_count(ev, live=True):
    """Count a problem event at its contact cell; when a place reaches 3/10/25... events say so (live only)."""
    x, y = analysis.contact_point(ev)
    k = (int(x // 50), int(y // 50))
    HOT[k] += 1
    if live and HOT[k] in HOT_ALERT:
        add_event("hotspot", f"Miejsce problemowe: {HOT[k]}× w tym samym miejscu ({analysis.HOT_KINDS[ev['k']]})",
                  x=round(x), y=round(y))


_hot_cache = {}


def hotspots_report():
    key = (EVENTS_LOG.stat().st_mtime if EVENTS_LOG.exists() else 0, MAP.get("key"))
    if _hot_cache.get("key") != key:
        evs = [json.loads(l) for l in EVENTS_LOG.read_text(encoding="utf-8").splitlines() if l.strip()] if EVENTS_LOG.exists() else []
        with lock:
            walls, ps = MAP["walls"], MAP["ps"]
        _hot_cache.update(key=key, list=analysis.hotspots(evs, walls, ps))
    return _hot_cache["list"]


def recorder():
    """2 frames/s of every run: pose, battery, status, Wi-Fi, peak sensor values, a thinned lidar scan."""
    while True:
        time.sleep(0.5)
        try:
            now, st = time.time(), STATUS["value"]
            if st in ACTIVE:
                SESSION["last_active"] = now
                if SESSION["id"] is None:
                    sid = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
                    kind = "explore" if EXPLORE["running"] else "clean"
                    if EXPLORE["running"]:
                        EXPLORE["session"] = sid
                    append_jsonl(SESSIONS / f"{sid}.jsonl", {"meta": {"id": sid, "type": kind, "start": round(now)}})
                    SESSION["id"] = sid
                    add_event("session", "Start: " + ("skanowanie mieszkania" if kind == "explore" else "sprzątanie"))
            elif SESSION["id"] and now - SESSION["last_active"] > SESSION_END_AFTER_S:
                add_event("session", f"Koniec przejazdu ({st})")
                SESSION["id"] = None
            win = SENS.window()  # always drained so peaks never leak into the next run
            if SESSION["id"] is None or POSE["x"] is None:
                continue
            with lock:
                fresh = SCAN["scan"] is not None and now - SCAN["scan"]["t"] < 2
                world = SCAN["world"] if fresh else []
            step = max(1, len(world) // FRAME_SCAN_POINTS)
            snap = SENS.snapshot()
            append_jsonl(SESSIONS / f"{SESSION['id']}.jsonl", {
                "t": round(now, 2), "x": round(POSE["x"]), "y": round(POSE["y"]), "a": round(POSE["angle"]),
                "bat": STATUS["battery"], "st": st, "wifi": (STATUS["wifi"] or {}).get("signal"),
                "s": {"br": win.get("brush"), "wh": win.get("wheel"), "tilt": win.get("tilt"), "wall": win.get("wall"),
                      "w": snap["watts"]},
                "scan": [p[:2] for p in world[::step]]})
        except Exception as e:
            print(f"[recorder] {e}")


def session_summary(path):
    key = (path.stat().st_mtime, path.stat().st_size)
    if _summary_cache.get(path.name, (None,))[0] == key:
        return _summary_cache[path.name][1]
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    meta = next((r["meta"] for r in rows if "meta" in r), {"id": path.stem, "type": "clean"})
    pts = [r for r in rows if "t" in r]
    out = None
    if pts:
        dist = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(pts, pts[1:]))
        bats = [r["bat"] for r in pts if r.get("bat") is not None]
        evs = Counter(r["ev"]["k"] for r in rows if "ev" in r)
        out = {**meta, "end": pts[-1]["t"], "duration_s": round(pts[-1]["t"] - pts[0]["t"]), "frames": len(pts),
               "bat_start": bats[0] if bats else None, "bat_end": bats[-1] if bats else None,
               "distance_m": round(dist / 100, 1), "events": dict(evs), "active": path.stem == SESSION["id"],
               "curve": [[round(r["t"] - pts[0]["t"]), r["bat"]] for r in pts[::max(1, len(pts) // 60)] if r.get("bat") is not None]}
    _summary_cache[path.name] = (key, out)
    return out


def sessions_list():
    out = []
    for p in sorted(SESSIONS.glob("*.jsonl"), reverse=True) if SESSIONS.exists() else []:
        s = session_summary(p)
        if s:
            out.append(s)
    return out


# ---------- poller: status, Wi-Fi, map ----------

def update_map(body):
    m = json.loads(body)
    nonce = (m.get("metaData") or {}).get("nonce")
    if nonce == MAPRAW["nonce"]:
        return m
    walls, walk, rooms = analysis.map_sets(m)
    key = (len(walls), len(walk), sum(x * 31 + y for x, y in walls))
    with lock:
        MAPRAW.update(bytes=body, nonce=nonce)
        if key != MAP.get("key"):  # the nonce changes with every robot move; redo the geometry only when walls do
            structural, islands = analysis.wall_parts(walls, walk)
            MAP.update(key=key, ps=m["pixelSize"], walls=walls, structural=structural, islands=islands,
                       near_struct=analysis.dilate(structural, 2), near_wall=analysis.dilate(walls, 1),
                       walk=walk, rooms=rooms)
            old = {(d["x1"], d["y1"], d["x2"], d["y2"]): d.get("state") for d in DOORS["list"]}
            doors = analysis.doorways(rooms, m["pixelSize"])
            for d in doors:
                d["state"] = old.get((d["x1"], d["y1"], d["x2"], d["y2"]))
            DOORS.update(list=doors, cells=analysis.door_cells(doors, CELL), hit={}, **{"pass": {}})
            REV["cloud"] += 1
    return m


def poller(interval):
    last_sample, last_status, tick, weak_at = None, None, 0, 0
    while True:
        now = time.time()
        try:
            attrs = get_json("/api/v2/robot/state/attributes")
            st = next((a for a in attrs if a["__class"] == "StatusStateAttribute"), {})
            bat = next((a for a in attrs if a["__class"] == "BatteryStateAttribute"), {})
            fresh = tick % 3 == 0 or st.get("value") in ACTIVE   # while driving: a reading every tick
            if fresh:
                STATUS["wifi"] = get_json("/api/v2/robot/capabilities/WifiConfigurationCapability").get("details") or {}
            status, _, body = FETCH("/api/v2/robot/state/map")
            m = update_map(body) if status == 200 else {}
            STATUS.update(attrs=attrs, value=st.get("value"), battery=bat.get("level"), error=st.get("error"))
            if st.get("value") != last_status:
                if last_status is not None:
                    err = (st.get("error") or {}).get("message")
                    add_event("error" if err else "status", f"Stan: {st.get('value')}" + (f" ({err})" if err else ""))
                last_status = st.get("value")
            pos = next((e for e in m.get("entities", []) if e.get("type") == "robot_position"), None)
            if pos and now - POSE["t"] > 1.5:  # the tap's 10 Hz SLAM pose wins while it is alive
                POSE.update(x=pos["points"][0], y=pos["points"][1],
                            angle=(pos.get("metaData") or {}).get("angle", 0), t=now, src="map")
            sig = STATUS["wifi"].get("signal")
            # only a reading taken this tick (a stale one would be pinned to the wrong place), and only after
            # the robot moved WIFI_MIN_MOVE_CM: parked on the dock = one sample, not thousands
            if fresh and POSE["x"] is not None and sig is not None:
                if last_sample is None or math.hypot(POSE["x"] - last_sample[0], POSE["y"] - last_sample[1]) >= WIFI_MIN_MOVE_CM:
                    last_sample = (POSE["x"], POSE["y"])
                    row = {"t": round(now), "x": round(POSE["x"]), "y": round(POSE["y"]), "signal": sig}
                    with lock:
                        WIFI.append(row)
                        append_jsonl(SAMPLES, row)
                if sig <= -80 and now - weak_at > 180 and st.get("value") in ACTIVE:
                    weak_at = now
                    add_event("wifi", f"Słaby sygnał Wi-Fi {sig} dBm")
        except Exception as e:  # robot offline / rebooting - keep trying
            print(f"[poller] {e}")
        tick += 1
        time.sleep(interval)


# ---------- tap: lidar, sensors, pose ----------

def process_scan(scan):
    """Place one lidar turn on the map (each point from the pose it was measured at) and update the cloud."""
    rays = lds.rays(scan, TRACK)
    last = TRACK.last()
    DBG["n"] += 1
    if DBG["n"] % 4 == 0:
        RAYS_DBG.append({"pts": [[a, mm, t] for a, mm, _, valid, t in scan["points"] if valid and mm],
                         "track": [list(h) for h in TRACK.h]})
    pose = {"x": last[1], "y": last[2], "angle": last[3]} if last else dict(POSE)
    world = [[round(ox + d * math.cos(th)), round(oy + d * math.sin(th)), round(d * 10), s]
             for ox, oy, th, d, s in rays if d]
    if not rays and POSE["x"] is not None and time.time() - POSE["t"] < 3:  # no SLAM pose: place by the map pose
        world = [p[:4] for p in lds.to_world(scan, POSE["x"], POSE["y"], POSE["angle"])]
    with lock:
        SCAN.update(scan=scan, world=world, raw=world, pose=pose, no_echo=sum(1 for r in rays if not r[3]))
        ps, structural, near = MAP["ps"], MAP["structural"], MAP["near_struct"]
        if not rays or not structural or TRACK.turn_rate() > MAX_TURN_DEG_S or len(CLOUD) > CLOUD_MAX_CELLS:
            return
        now, drop, dcells, dhit, dpass = time.time(), set(), DOORS["cells"], DOORS["hit"], DOORS["pass"]
        near_wall, mirror = MAP["near_wall"], MIRROR["px"]
        for ox, oy, th, dist, _ in rays:
            dx, dy = math.cos(th), math.sin(th)
            limit = dist if dist else 350
            d, wall, path = 10.0, None, []
            while d < limit - 15:  # walk the ray: stop at the first structural wall, remember the cells passed
                x, y = ox + dx * d, oy + dy * d
                p = (int(x // ps), int(y // ps))
                if p in structural:
                    wall = p
                    break
                path.append((int(x // CELL), int(y // CELL)))
                d += CELL
            if dist is None:
                if wall:
                    MISS[wall] += 1  # the map has a wall there, the lidar got nothing back
                continue
            hx, hy = ox + dx * dist, oy + dy * dist
            cell = (int(hx // CELL), int(hy // CELL))
            if wall and d < dist - 30 and (int(hx // ps), int(hy // ps)) not in near:
                THROUGH[wall] += 1  # echo from behind a wall: mirror image / through glass / doorway
                FAR.setdefault(wall, deque(maxlen=30)).append((hx, hy))
                o = OPEN.get(wall, 0.0)
                OPEN[wall] = o + 0.15 * (1 - o)
                if wall in mirror or o <= OPEN_AT:  # mirror image / not (yet) a known opening: don't draw it
                    drop.add((round(hx), round(hy)))
                if wall not in mirror:  # a mirror image is not a real thing behind the wall: not kept either
                    PHANTOM[cell] += 1
                    PH_WALL[cell] = wall
                continue
            if wall:
                SEEN[wall] += 1
                o = OPEN.get(wall, 0.0)
                OPEN[wall] = o + 0.15 * (-1 - o)  # the wall answered: it is there (door shut again)
            if cell not in CLOUD:
                BORN[cell] = now
            CLOUD[cell] = min(CAP, CLOUD.get(cell, 0) + HIT)
            if cell in dcells:
                dhit[dcells[cell]] = now
            for c in path:  # free space between robot and echo: things that used to be there are gone
                if c in dcells:
                    dpass[dcells[c]] = now
                n = CLOUD.get(c)
                if n is not None:
                    if n <= CLEAR:
                        del CLOUD[c]
                        age = now - BORN.pop(c, now)
                        if 0.4 < age < MOVING_MAX_S and (c[0] * CELL // ps, c[1] * CELL // ps) not in near_wall:
                            MOVED.append((now, c[0] * CELL + 2, c[1] * CELL + 2))
                    else:
                        CLOUD[c] = n - CLEAR
        if drop:
            SCAN["world"] = [q for q in world if (q[0], q[1]) not in drop]
        if DBG["n"] % 10 == 0:
            refresh_derived(now)
        REV["cloud"] += 1


def refresh_derived(now):
    """Every ~2 s while scanning (lock held): which wall pixels are mirrors, and did any door change state."""
    segs = [g for g in analysis.reflective_segments(THROUGH, FAR, SEEN, MISS, MAP["near_wall"], MAP["ps"]) if g["kind"] == "lustro"]
    ps = MAP["ps"]
    MIRROR["px"] = {w for w in THROUGH if any(analysis.seg_dist(w[0] * ps + ps / 2, w[1] * ps + ps / 2, g) <= 12 for g in segs)}
    for i, d in enumerate(DOORS["list"]):
        st = analysis.door_state(i, d["nk"], DOORS["hit"], DOORS["pass"], now)
        if st and st != d.get("state"):
            if d.get("state"):  # the first verdict is just the initial state, not a change
                add_event("door", f"Drzwi {' ↔ '.join(d['rooms'])}: {st}", x=(d["x1"] + d["x2"]) // 2, y=(d["y1"] + d["y2"]) // 2)
            d["state"] = st


def tap_reader(host, key):
    """Stream lidartap datagrams over SSH: "LT" port dir len(u16) t_ms(u32) payload.
    port 3 = lidar bytes, 7 = MCU sensor package, 8 = SLAM pose."""
    cmd = ["ssh", "-i", key, "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
           "-o", "ServerAliveInterval=10", "-o", "StrictHostKeyChecking=accept-new",
           "-o", f"UserKnownHostsFile={DATA / 'known_hosts'}", f"root@{host}",
           "killall lidartap_relay 2>/dev/null; exec /mnt/data/lidartap_relay"]
    DATA.mkdir(exist_ok=True)
    while True:
        dec, buf = lds.LdsDecoder(), bytearray()
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"[tap] connecting to {host}")
            while chunk := p.stdout.read1(65536):
                TAP.update(connected=True, last=time.time())
                buf += chunk
                lidar = bytearray()
                while len(buf) >= 10:
                    if buf[:2] != b"LT":
                        k = buf.find(b"LT", 1)
                        del buf[:k if k > 0 else len(buf)]
                        continue
                    n = int.from_bytes(buf[4:6], "little")
                    if len(buf) < 10 + n:
                        break
                    port, direction, t_ms = buf[2], buf[3], int.from_bytes(buf[6:10], "little")
                    data = bytes(buf[10:10 + n])
                    del buf[:10 + n]
                    if port == 3 and direction == 0:
                        lidar += data
                        for sc in dec.feed(data, t_ms):
                            sc["t_end"] = t_ms
                            PENDING.append(sc)
                        while PENDING and time.time() - PENDING[0]["t"] > 1.5:  # no SLAM pose arriving: don't wait
                            process_scan(PENDING.popleft())
                    elif port == 7:
                        SENS.feed(data, t_ms / 1000)
                        for ev in SENS.drain():
                            extra = {k: v for k, v in ev.items() if k not in ("k", "t")}
                            add_event(ev["k"], sensors.EVENT_PL.get(ev["k"], ev["k"]), **extra)
                    elif port == 8 and n >= 12:
                        x, y, a = lds.parse_pose(data)
                        TRACK.add(t_ms, x, y, a)
                        POSE.update(x=x, y=y, angle=a, t=time.time(), src="slam")
                        while PENDING and PENDING[0]["t_end"] + lds.POSE_LAG_MS + 150 <= t_ms:
                            process_scan(PENDING.popleft())
                if lidar:
                    with open(DATA / f"lidar_{datetime.date.today()}.bin", "ab") as f:
                        f.write(lidar)  # raw LDS bytes, decodable later with lds.py
            print(f"[tap] relay ended: {p.stderr.read().decode(errors='replace').strip()}")
        except Exception as e:
            print(f"[tap] {e}")
        TAP["connected"] = False
        time.sleep(5)


def cloud_report():
    with lock:
        ps, near, walk, near_all = MAP["ps"], MAP["near_struct"], MAP["walk"], MAP["near_wall"]
        now = time.time()  # young echoes wait: if they vanish again they were something moving, not an obstacle
        cells = {k: n for k, n in CLOUD.items() if n >= MIN_HITS and now - BORN.get(k, 0) >= STABLE_S}
        phantom = [[cx * CELL + 2, cy * CELL + 2, n] for (cx, cy), n in PHANTOM.items()
                   if n >= 2 and PH_WALL.get((cx, cy)) not in MIRROR["px"] and OPEN.get(PH_WALL.get((cx, cy)), 0) <= OPEN_AT]
        open_px = sorted(w for comp in analysis.components({w for w, o in OPEN.items() if o > OPEN_AT and w not in MIRROR["px"]}, gap=1)
                         if len(comp) >= 4 for w in comp)
        moving = [[x, y] for t, x, y in MOVED if now - t < 30]
        segs = analysis.reflective_segments(THROUGH, FAR, SEEN, MISS, near_all, ps)
        islands = [[x, y] for comp in MAP["islands"] for x, y in comp]
        doors = [dict(d) for d in DOORS["list"]]
        rev = REV["cloud"]
    classified = analysis.classify_cloud(cells, CELL, ps, near, walk)
    return {"rev": rev, "cells": classified, "phantom": phantom, "reflective": segs, "islands": islands, "doors": doors, "moving": moving, "open_px": open_px,
            "obstacles": analysis.obstacle_clusters(classified, CELL, MIN_HITS)}


def wifi_report():
    with lock:
        samples, rooms, ps = list(WIFI), MAP["rooms"], MAP["ps"]
    acc = {}
    for s in samples:
        k = (s["x"] // WIFI_CELL_CM, s["y"] // WIFI_CELL_CM)
        a = acc.setdefault(k, [0, 0])
        a[0] += s["signal"]
        a[1] += 1
    cells = [[kx * WIFI_CELL_CM, ky * WIFI_CELL_CM, round(v / n, 1), n] for (kx, ky), (v, n) in acc.items()]
    return {"n": len(samples), "cell": WIFI_CELL_CM, "cells": cells, "rooms": analysis.wifi_by_room(samples, rooms, ps)}


def state_report():
    now = time.time()
    with lock:
        s = SCAN["scan"]
        scan = None if s is None else {"age": round(now - s["t"], 2), "rpm": s["rpm"], "n": len(SCAN["world"]),
                                       "no_echo": SCAN["no_echo"]}
        events = list(EVENTS)[-40:]
        cloud = {"rev": REV["cloud"], "cells": len(CLOUD)}
    pose = None if POSE["x"] is None else {"x": round(POSE["x"], 1), "y": round(POSE["y"], 1),
                                           "angle": round(POSE["angle"], 1), "src": POSE["src"], "age": round(now - POSE["t"], 2)}
    return {"t": now, "attrs": STATUS["attrs"], "wifi": STATUS["wifi"], "pose": pose, "map_nonce": MAPRAW["nonce"],
            "cloud": cloud, "scan": scan, "sensors": SENS.snapshot() if TAP["connected"] else None,
            "tap": TAP["connected"], "explore": {k: EXPLORE[k] for k in ("running", "route", "i", "msg", "session")},
            "program": {"i": PROGRAM["i"], "n": len(PROGRAM["steps"]), "results": PROGRAM["results"][-20:]},
            "session": SESSION["id"], "events": events, "wifi_samples": len(WIFI), "robot": BASE or "mock"}


# ---------- exploration (drive everywhere, no cleaning) ----------

def drive_to(x, y, label, timeout=300):
    """GoTo and wait until the robot itself stops (arrived, or blocked by an obstacle). -> distance left [cm]"""
    put_json("/api/v2/robot/capabilities/GoToLocationCapability", {"action": "goto", "coordinates": {"x": x, "y": y}})
    t0, still = time.time(), 0
    time.sleep(4)
    while time.time() - t0 < timeout and not EXPLORE["stop"]:
        still = 0 if STATUS["value"] == "moving" else still + 1
        if still >= 3:  # really stopped, not a one-poll status flicker
            break
        time.sleep(1)
    if STATUS["value"] == "error":
        raise RuntimeError("robot zgłosił błąd")
    left = math.hypot((POSE["x"] or x) - x, (POSE["y"] or y) - y)
    if left > 40 and not EXPLORE["stop"]:
        add_event("explore", f"Cel „{label}” nieosiągalny: zatrzymanie {round(left)} cm przed nim (przeszkoda?)")
    return left


def heading():
    """Map direction the robot drives in (deg, map y down). Same convention as scene.js HEADING_OFFSET_DEG."""
    return (POSE["angle"] - 90) % 360


def ang_diff(a, b):
    return (a - b + 180) % 360 - 180


def bearing(x, y):
    return math.degrees(math.atan2(y - POSE["y"], x - POSE["x"])) % 360


def rc_move(velocity, turn):
    put_json(RC, {"action": "move", "vector": {"velocity": velocity, "angle": turn}})


def blocked_ahead(clear_cm=38, half_w=19):
    """Nearest lidar echo in the robot's path within clear_cm (from the lidar centre) -> distance cm or None."""
    with lock:
        pts, x0, y0 = list(SCAN.get("raw") or SCAN["world"]), POSE["x"], POSE["y"]  # all echoes, also through "walls"
    h = math.radians(heading())
    ux, uy = math.cos(h), math.sin(h)
    return min((round(a) for x, y, *_ in pts for a in [(x - x0) * ux + (y - y0) * uy]
                if 8 < a < clear_cm and abs((y - y0) * ux - (x - x0) * uy) < half_w), default=None)


def bumped_since(t):
    return any(e["t"] >= t and e["k"] in ("bumper", "lds_bumper", "cliff", "drop") for e in list(EVENTS))


def rc_enable():
    put_json(RC, {"action": "enable"})
    t0 = time.time()
    while STATUS["value"] != "manual_control" and time.time() - t0 < 8:
        time.sleep(0.3)
    # status flips at ~1.5 s, but moves are ignored for up to ~10 s more: nudge until the heading really moves
    h0, t0 = heading(), time.time()
    while abs(ang_diff(heading(), h0)) < 4:
        if time.time() - t0 > 20:
            raise RuntimeError("sterowanie ręczne nie obraca robota")
        rc_move(0, 20)
        time.sleep(0.4)
    RC_SIGN["turn"] = 1 if ang_diff(heading(), h0) > 0 else -1
    rc_move(0, 0)


def rc_turn_to(target, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout and not EXPLORE["stop"]:
        err = ang_diff(target, heading())
        if abs(err) < 7:
            break
        rc_move(0, RC_SIGN["turn"] * max(-35, min(35, err * 0.7)))  # each move lasts ~1.5 s: keep it gentle
        time.sleep(0.3)
    rc_move(0, 0)


def rc_drive(tx, ty, max_cm=400, arrive_cm=20, push=False):
    """Drive straight at (tx, ty) until there, an obstacle in front, a bumper hit, or stuck. -> reason
    push: an echo ahead only slows it down - the bumper decides (door frames, a half-open door leaf)."""
    t0, x0, y0 = time.time(), POSE["x"], POSE["y"]
    last, last_t = (x0, y0), t0
    while not EXPLORE["stop"]:
        if math.hypot(tx - POSE["x"], ty - POSE["y"]) < arrive_cm:
            reason = "cel"
            break
        if math.hypot(POSE["x"] - x0, POSE["y"] - y0) > max_cm:
            reason = "limit"
            break
        if bumped_since(t0):
            reason = "zderzak"
            break
        ahead = blocked_ahead()
        if ahead is not None and not push:
            reason = f"przeszkoda {ahead} cm przed robotem"
            break
        if time.time() - last_t > 2.5:
            if math.hypot(POSE["x"] - last[0], POSE["y"] - last[1]) < 4:
                reason = "utknął"
                break
            last, last_t = (POSE["x"], POSE["y"]), time.time()
        err = ang_diff(bearing(tx, ty), heading())
        rc_move((0.35 if ahead is not None else 0.75) if abs(err) < 30 else 0, RC_SIGN["turn"] * max(-30, min(30, err * 0.8)))
        time.sleep(0.25)
    else:
        reason = "stop"
    rc_move(0, 0)
    return reason


def probe(seg, target=None):
    """Try to drive through a wall stretch the lidar sees through (door that was shut on the map).
    Doesn't trust the map: approach, face it, drive straight on (towards target, if given) until something
    physically stops the robot."""
    ps, walk = MAP["ps"], MAP["walk"]
    mx, my = (seg["x1"] + seg["x2"]) / 2, (seg["y1"] + seg["y2"]) / 2
    bx, by = target or seg["beyond"]
    ux, uy = seg["x2"] - seg["x1"], seg["y2"] - seg["y1"]
    ul = math.hypot(ux, uy) or 1
    nx, ny = -uy / ul, ux / ul  # perpendicular to the door, pointing to the target's side
    if (bx - mx) * nx + (by - my) * ny < 0:
        nx, ny = -nx, -ny
    ax, ay = next(((mx - nx * k, my - ny * k) for k in (70, 55, 40) if (int((mx - nx * k) // ps), int((my - ny * k) // ps)) in walk),
                  (None, None))
    if ax is None:
        return "brak miejsca na podjazd"
    if drive_to(round(ax), round(ay), "przed " + seg["kind"]) > 50:
        return "nie dojechał przed przejście"
    rc_enable()
    try:
        rc_turn_to(math.degrees(math.atan2(ny, nx)) % 360)
        reason = rc_drive(mx + nx * 60, my + ny * 60, max_cm=250, arrive_cm=25, push=True)  # through the door first
        if reason == "cel":
            gx, gy = target or (mx + nx * 400, my + ny * 400)
            rc_turn_to(bearing(gx, gy))
            reason = rc_drive(gx, gy, max_cm=500, arrive_cm=30, push=True)
        past = (POSE["x"] - mx) * nx + (POSE["y"] - my) * ny
        time.sleep(2)  # let lidar and the robot's own SLAM look around
        rc_turn_to(bearing(ax, ay))
        rc_drive(ax, ay, max_cm=600, arrive_cm=25)
    finally:
        try:
            rc_move(0, 0)
        finally:
            put_json(RC, {"action": "disable"})
            time.sleep(1.5)
    return f"przejechał {round(past)} cm za linię drzwi ({reason})" if past > 25 else f"nie przejechał ({reason})"


def explore_run(target=None):
    try:
        st = STATUS["value"]
        if st not in ("docked", "idle"):
            raise RuntimeError(f"robot zajęty (status: {st})")
        m = json.loads(MAPRAW["bytes"])
        route = [[*target, "wskazany cel"]] if target else [list(r) for r in analysis.plan_waypoints(m, (POSE["x"] or 0, POSE["y"] or 0))]
        EXPLORE.update(route=route, msg=f"{len(route)} punktów")
        tried, i = set(), 0
        while i < len(route) and not EXPLORE["stop"]:
            x, y, name = route[i]
            EXPLORE.update(i=i, msg=f"jadę do punktu {i + 1}/{len(route)} ({name})")
            left = drive_to(x, y, name)
            if left > 60 and not EXPLORE["stop"]:  # the robot's own map says no (e.g. a door shut when it was made): try it
                with lock:
                    shut = analysis.shut_doors(MAP["walls"], MAP["walk"], (x, y), MAP["ps"], start=(POSE["x"], POSE["y"]))
                near = sorted((g for g in cloud_report()["reflective"] if g["kind"] in ("drzwi?", "szyba/otwór")),
                              key=lambda g: math.hypot((g["x1"] + g["x2"]) / 2 - x, (g["y1"] + g["y2"]) / 2 - y))
                near = shut[:2] + [g for g in near[:1] if math.hypot((g["x1"] + g["x2"]) / 2 - x, (g["y1"] + g["y2"]) / 2 - y) <= 350]
                if not near and not name.endswith("(ponownie)"):  # lidar hasn't looked through it yet: come back at the end
                    route.append([x, y, name + " (ponownie)"])
                for seg in near:
                    if EXPLORE["stop"]:
                        break
                    tried.add(tuple(v // 50 for v in seg.get("beyond", (seg["x1"], seg["y1"]))))
                    EXPLORE["msg"] = f"punkt {i + 1} ({name}) zablokowany na mapie robota: próbuję przez {seg['kind']}"
                    res = probe(seg, (x, y))
                    add_event("explore", f"Punkt {i + 1} ({name}) przez {seg['kind']}: {res}")
                    if res.startswith("przejechał"):
                        break
            i += 1
            if i == len(route) and not target:  # planned stops done: physically try every opening the lidar sees through
                ps, walk = MAP["ps"], MAP["walk"]
                segs = sorted((g for g in cloud_report()["reflective"] if g["kind"] in ("drzwi?", "szyba/otwór") and "beyond" in g
                               and (g["beyond"][0] // ps, g["beyond"][1] // ps) not in walk), key=lambda g: -g["n"])
                for seg in segs:
                    key = tuple(v // 50 for v in seg["beyond"])
                    if key in tried or len(tried) >= 4 or EXPLORE["stop"]:
                        continue
                    tried.add(key)
                    EXPLORE["msg"] = f"próbuję przejechać: {seg['kind']} przy ({seg['x1']}, {seg['y1']})"
                    add_event("explore", f"Próba przejazdu ({seg['kind']}): {probe(seg)}")
                if tried and not EXPLORE["stop"]:  # the robot's map grew behind the openings: visit what is new
                    m = json.loads(MAPRAW["bytes"])
                    seen = [(x, y) for x, y, _ in route]
                    new = [list(r) for r in analysis.plan_waypoints(m, (POSE["x"], POSE["y"]))
                           if all(math.hypot(r[0] - x, r[1] - y) > 150 for x, y in seen)]
                    route += new
                    if new:
                        add_event("explore", f"Mapa się powiększyła: {len(new)} nowych punktów")
                EXPLORE["route"] = route
        EXPLORE["msg"] = "powrót na stację"
        put_json("/api/v2/robot/capabilities/BasicControlCapability", {"action": "home"})
        EXPLORE["msg"] = "zakończone" if not EXPLORE["stop"] else "przerwane, powrót na stację"
    except Exception as e:
        EXPLORE["msg"] = f"błąd: {e}"
    finally:
        EXPLORE["running"] = False


# ---------- procedural control: a program = list of steps, run one by one with a report per step ----------
# Shares the EXPLORE slot (one job at a time, same stop flag, same banner in the UI).
PROGRAM = {"steps": [], "results": [], "i": 0}
BASIC = "/api/v2/robot/capabilities/BasicControlCapability"
MAX_STEPS, MAX_RC_CM = 200, 300
RC_OPS = ("forward", "back", "turn", "face", "face_point")
PROGRAM_OPS = ("goto", *RC_OPS, "look", "wait", "wait_done", "clean_zone", "clean_rooms", "fan", "locate", "home", "stop",
               "explore")
PROGRAMS = ROOT / "programs"  # saved programs (<name>.json: {"title", "description", "steps"})
PROGRAM_HELP = """Kroki programu (lista obiektów, jeden klucz = jedna operacja):
  {"goto": [x, y]}             jazda planerem robota do punktu (cm, mapa Valetudo)
  {"forward": cm} / {"back": cm}  jazda prosto w sterowaniu ręcznym (maks. 300 cm; do przodu staje przed przeszkodą)
  {"forward": cm, "push": true}   do przodu aż do zderzaka (przeszkoda z lidaru tylko zwalnia)
  {"turn": deg}                obrót o kąt (+ = w prawo / zgodnie z ruchem wskazówek na mapie)
  {"face": deg} / {"face_point": [x, y]}  obrót na kierunek mapy (0 = +x, 90 = +y w dół mapy) / w stronę punktu
  {"look": true}               pomiar lidarem: najbliższe echo przód / lewo / prawo / tył (cm)
  {"wait": s}                  czekaj (maks. 600 s)
  {"wait_done": s}             czekaj aż robot skończy zadanie (status docked/idle), maks. s
  {"clean_zone": [x1, y1, x2, y2], "iterations": n}   {"clean_rooms": ["1", "2"], "iterations": n}
  {"fan": "low|medium|high|max|off"}   {"locate": true}   {"home": true}   {"stop": true}
  {"explore": true}            pełny skan mieszkania lidarem (jak „Skanuj mieszkanie”), kończy się na stacji
Każdy krok zwraca: wynik, przesunięcie, pozycję i zdarzenia czujników (zderzak, przechył...) w jego trakcie."""


def rc_straight(cm, push=False):
    """Drive cm straight (negative = reverse) holding the current heading. -> (reason, cm actually moved)
    Forward stops in front of a lidar echo (unless push) or on a bumper hit; reverse has no bumper: limit + stuck."""
    sign, h0, x0, y0 = (1 if cm > 0 else -1), heading(), POSE["x"], POSE["y"]
    ux, uy = math.cos(math.radians(h0)), math.sin(math.radians(h0))
    t0, last, last_t, reason = time.time(), (x0, y0), time.time(), "stop"
    done = 0.0
    while not EXPLORE["stop"]:
        done = ((POSE["x"] - x0) * ux + (POSE["y"] - y0) * uy) * sign
        if done >= abs(cm) - 4:
            reason = "cel"
            break
        if time.time() - t0 > 10 + abs(cm) / 5:
            reason = "limit czasu"
            break
        ahead = blocked_ahead() if sign > 0 else None
        if sign > 0 and bumped_since(t0):
            reason = "zderzak"
            break
        if ahead is not None and not push:
            reason = f"przeszkoda {ahead} cm przed robotem"
            break
        if time.time() - last_t > 2.5:
            if math.hypot(POSE["x"] - last[0], POSE["y"] - last[1]) < 3:
                reason = "utknął"
                break
            last, last_t = (POSE["x"], POSE["y"]), time.time()
        err = ang_diff(h0, heading())
        speed = 0.3 if ahead is not None or abs(cm) - done < 20 else 0.6
        rc_move(sign * speed, RC_SIGN["turn"] * max(-20, min(20, err * 0.8)))
        time.sleep(0.25)
    rc_move(0, 0)
    return reason, round(done)


def look():
    """Nearest lidar echo in 4 sectors around the robot (cm from the lidar) + the obstacle count within 1 m."""
    with lock:
        pts, x0, y0, age = list(SCAN.get("raw") or SCAN["world"]), POSE["x"], POSE["y"], SCAN["scan"] and time.time() - SCAN["scan"]["t"]
    if not pts or age is None or age > 3:
        return {"error": "brak świeżego skanu lidaru (lidar się nie kręci?)"}
    h, out = heading(), {"front": None, "right": None, "back": None, "left": None}
    for x, y, *_ in pts:
        d = math.hypot(x - x0, y - y0)
        rel = ang_diff(math.degrees(math.atan2(y - y0, x - x0)), h)
        k = "front" if abs(rel) <= 45 else "back" if abs(rel) >= 135 else "right" if rel > 0 else "left"
        out[k] = round(d) if out[k] is None else min(out[k], round(d))
    out["echoes_within_100cm"] = sum(1 for x, y, *_ in pts if math.hypot(x - x0, y - y0) < 100)
    return out


def wait_done(max_s):
    t0 = time.time()
    time.sleep(3)
    while time.time() - t0 < max_s and not EXPLORE["stop"] and STATUS["value"] not in ("docked", "idle", "error"):
        time.sleep(1)
    return STATUS["value"]


def program_step(op, s):
    a = s[op]
    if op == "goto":
        left = drive_to(int(a[0]), int(a[1]), f"krok {PROGRAM['i'] + 1}")
        return {"result": "dojechał" if left <= 40 else "nie dojechał", "left_cm": round(left)}
    if op in ("forward", "back"):
        cm = max(-MAX_RC_CM, min(MAX_RC_CM, float(a))) * (1 if op == "forward" else -1)
        reason, moved = rc_straight(cm, push=bool(s.get("push")))
        return {"result": reason, "moved_cm": moved}
    if op in ("turn", "face", "face_point"):
        h0 = heading()
        target = (h0 + float(a)) % 360 if op == "turn" else float(a) % 360 if op == "face" else bearing(*a)
        rc_turn_to(target)
        return {"result": "obrócony" if abs(ang_diff(target, heading())) < 10 else "nie obrócił się do końca",
                "turned_deg": round(ang_diff(heading(), h0)), "heading": round(heading())}
    if op == "look":
        return look()
    if op == "wait":
        end = time.time() + min(600, float(a))
        while time.time() < end and not EXPLORE["stop"]:
            time.sleep(0.25)
        return {"result": "ok"}
    if op == "wait_done":
        return {"result": wait_done(min(7200, float(a)))}
    if op == "clean_zone":
        x1, y1, x2, y2 = (int(v) for v in a)
        x1, x2, y1, y2 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
        put_json("/api/v2/robot/capabilities/ZoneCleaningCapability", {"action": "clean", "iterations": int(s.get("iterations", 1)),
                 "zones": [{"points": {"pA": {"x": x1, "y": y1}, "pB": {"x": x2, "y": y1}, "pC": {"x": x2, "y": y2}, "pD": {"x": x1, "y": y2}}}]})
        return {"result": "start sprzątania strefy"}
    if op == "clean_rooms":
        put_json("/api/v2/robot/capabilities/MapSegmentationCapability", {"action": "start_segment_action", "segment_ids": [str(v) for v in a],
                 "iterations": int(s.get("iterations", 1)), "customOrder": True})
        return {"result": "start sprzątania pokoi"}
    if op == "fan":
        put_json("/api/v2/robot/capabilities/FanSpeedControlCapability/preset", {"name": str(a)})
        return {"result": "ok"}
    if op == "locate":
        put_json("/api/v2/robot/capabilities/LocateCapability", {"action": "locate"})
        return {"result": "ok"}
    if op in ("home", "stop"):
        put_json(BASIC, {"action": op})
        return {"result": "ok"}
    if op == "explore":
        t0 = time.time()  # leaving manual control takes a moment
        while STATUS["value"] not in ("docked", "idle") and time.time() - t0 < 15:
            time.sleep(0.5)
        explore_run()
        EXPLORE["running"] = True  # explore_run frees the slot when done; the program still owns it
        return {"result": EXPLORE["msg"]}
    raise ValueError(f"nieznana operacja {op!r}")


def program_validate(steps):
    if not isinstance(steps, list) or not 0 < len(steps) <= MAX_STEPS:
        raise ValueError(f"steps: lista 1..{MAX_STEPS} kroków")
    for i, s in enumerate(steps):
        op = next((k for k in s if k in PROGRAM_OPS), None) if isinstance(s, dict) else None
        if op is None:
            raise ValueError(f"krok {i + 1}: brak znanej operacji ({', '.join(PROGRAM_OPS)})")


def run_program(steps):
    manual = False
    PROGRAM.update(steps=steps, results=[], i=0)
    try:
        for i, s in enumerate(steps):
            if EXPLORE["stop"]:
                break
            op = next(k for k in s if k in PROGRAM_OPS)
            PROGRAM["i"] = i
            EXPLORE.update(i=i, msg=f"krok {i + 1}/{len(steps)}: {json.dumps(s, ensure_ascii=False)}")
            if op in RC_OPS and not manual:
                if STATUS["value"] not in ("docked", "idle", "manual_control"):
                    raise RuntimeError(f"sterowanie ręczne wymaga postoju robota (status: {STATUS['value']})")
                rc_enable()
                manual = True
            elif op not in RC_OPS and op not in ("look", "wait") and manual:
                put_json(RC, {"action": "disable"})
                manual = False
                time.sleep(1.5)
            t0 = time.time()
            try:
                res = program_step(op, s)
            except Exception as e:
                res = {"result": f"błąd: {e}"}
            evs = [f"{e['k']}{'(' + e['side'] + ')' if e.get('side') else ''}" for e in list(EVENTS)
                   if e["t"] >= t0 and e["k"] in analysis.HOT_KINDS]
            PROGRAM["results"].append({"step": i + 1, "op": s, **res, "events": evs,
                                       "pose": {"x": round(POSE["x"] or 0), "y": round(POSE["y"] or 0), "heading": round(heading())}})
        EXPLORE["msg"] = "program zakończony" if not EXPLORE["stop"] else "program przerwany"
    except Exception as e:
        EXPLORE["msg"] = f"błąd programu: {e}"
    finally:
        if manual:
            try:
                rc_move(0, 0)
                put_json(RC, {"action": "disable"})
            except Exception as e:
                print(f"[program] disable: {e}")
        EXPLORE["running"] = False


# ---------- photos ----------

def photos_index():
    f = PHOTOS / "index.json"
    return json.loads(f.read_text(encoding="utf-8")) if f.exists() else []


def photos_save(items):
    PHOTOS.mkdir(parents=True, exist_ok=True)
    (PHOTOS / "index.json").write_text(json.dumps(items, ensure_ascii=False, indent=1), encoding="utf-8")


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(STATIC), **kw)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")  # also for static files: edits show up on reload
        super().end_headers()

    def reply(self, status, ctype, body):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def json(self, obj, status=200):
        self.reply(status, "application/json", json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode())

    def body(self, limit):
        n = int(self.headers.get("Content-Length") or 0)
        if n > limit:
            raise ValueError("za duże")
        return self.rfile.read(n)

    def do_GET(self):
        path = urlparse(self.path).path
        if path.startswith("/api/"):
            try:
                return self.reply(*FETCH(self.path))
            except Exception as e:
                return self.reply(502, "text/plain; charset=utf-8", f"robot unreachable: {e}".encode())
        if path == "/local/state":
            return self.json(state_report())
        if path == "/local/map":
            return self.reply(200, "application/json", MAPRAW["bytes"])
        if path == "/local/scan":
            with lock:
                s = SCAN["scan"]
                body = {"age": None} if s is None else {
                    "age": round(time.time() - s["t"], 2), "rpm": s["rpm"], "pose": SCAN["pose"],
                    "points": SCAN["world"], "raw_n": len(s["points"]), "no_echo": SCAN["no_echo"]}
            return self.json(body)
        if path == "/local/cloud":
            return self.json(cloud_report())
        if path == "/local/debug":
            with lock:
                walls = sorted(set(THROUGH) | set(SEEN) | set(MISS))
                return self.json({"ps": MAP["ps"], "scans": list(RAYS_DBG), "structural": [list(p) for p in MAP["structural"]],
                                  "walls": [[x, y, THROUGH[(x, y)], SEEN[(x, y)], MISS[(x, y)]] for x, y in walls]})
        if path == "/local/wifi":
            return self.json(wifi_report())
        if path == "/local/hotspots":
            return self.json(hotspots_report())
        if path == "/local/programs":
            return self.json([{"name": f.stem, **json.loads(f.read_text(encoding="utf-8"))} for f in sorted(PROGRAMS.glob("*.json"))])
        if path == "/local/program":
            return self.json({**PROGRAM, "running": EXPLORE["running"], "msg": EXPLORE["msg"], "help": PROGRAM_HELP})
        if path == "/local/sessions":
            return self.json(sessions_list())
        if path.startswith("/local/sessions/"):
            f = SESSIONS / (Path(path).name + ".jsonl")  # Path().name strips any ../
            if not f.exists():
                return self.json({"error": "not found"}, 404)
            return self.reply(200, "application/x-ndjson", f.read_bytes())
        if path == "/local/photos":
            return self.json(photos_index())
        if path.startswith("/local/photos/"):
            f = PHOTOS / Path(path).name
            ctype = next((c for c, ext in PHOTO_TYPES.items() if f.suffix == ext), None)
            if not ctype or not f.exists():
                return self.json({"error": "not found"}, 404)
            return self.reply(200, ctype, f.read_bytes())
        return super().do_GET()

    def robot_write(self, method):
        path = urlparse(self.path).path
        why = write_check(method, path, self.headers.get("X-Confirm", "").lower() == "tak")
        if why:
            return self.json({"error": why}, 403)
        try:
            status, out = put_robot(path, self.body(65536) if method != "DELETE" else None, method)
            return self.reply(status, "application/json", out or b"{}")
        except Exception as e:
            return self.json({"error": str(e)}, 502)

    def do_PUT(self):
        return self.robot_write("PUT")

    def do_DELETE(self):
        return self.robot_write("DELETE")

    def do_POST(self):
        url = urlparse(self.path)
        if url.path.startswith("/api/"):
            return self.robot_write("POST")
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path == "/local/program/start":
            if EXPLORE["running"]:
                return self.json({"error": "trwa inne zadanie (skan lub program) - najpierw stop"}, 409)
            if not BASE:
                return self.json({"error": "tryb mock - brak robota"}, 400)
            try:
                steps = json.loads(self.body(262144) or b"null")
                steps = steps.get("steps") if isinstance(steps, dict) else steps
                program_validate(steps)
            except ValueError as e:
                return self.json({"error": str(e)}, 400)
            EXPLORE.update(running=True, stop=False, route=[s["goto"] for s in steps if "goto" in s], i=0, msg="program: start")
            threading.Thread(target=run_program, args=(steps,), daemon=True).start()
            return self.json({"ok": True, "steps": len(steps)})
        if url.path == "/local/wifi/clear":
            with lock:
                WIFI.clear()
                SAMPLES.unlink(missing_ok=True)
            return self.json({})
        if url.path == "/local/cloud/clear":
            with lock:
                for c in (CLOUD, PHANTOM, THROUGH, SEEN, MISS, FAR, BORN, MOVED, PH_WALL, MIRROR["px"], OPEN):
                    c.clear()
                REV["cloud"] += 1
            return self.json({})
        if url.path == "/local/explore/start":
            if EXPLORE["running"]:
                return self.json({"error": "już trwa"}, 409)
            if not BASE:
                return self.json({"error": "tryb mock - brak robota"}, 400)
            q = parse_qs(url.query)
            target = (int(q["x"][0]), int(q["y"][0])) if "x" in q and "y" in q else None
            EXPLORE.update(running=True, stop=False, route=[], i=0, msg="planowanie trasy")
            threading.Thread(target=explore_run, args=(target,), daemon=True).start()
            return self.json({"ok": True})
        if url.path == "/local/explore/stop":
            EXPLORE["stop"] = True
            return self.json({"ok": True})
        if url.path == "/local/photos":
            ext = PHOTO_TYPES.get((self.headers.get("Content-Type") or "").split(";")[0].strip())
            if not ext:
                return self.json({"error": "tylko JPEG, PNG lub WebP"}, 415)
            try:
                data = self.body(PHOTO_MAX_BYTES)
                seg = [float(q[k]) for k in ("x1", "y1", "x2", "y2")]
                aspect = min(4.0, max(0.25, float(q.get("aspect", 0.75))))  # height / width of the image
            except (KeyError, ValueError) as e:
                return self.json({"error": f"złe parametry: {e}"}, 400)
            if not data or any(not 0 <= v <= 10000 for v in seg):
                return self.json({"error": "pusty plik lub punkty poza mapą"}, 400)
            pid = uuid.uuid4().hex[:12]
            PHOTOS.mkdir(parents=True, exist_ok=True)
            (PHOTOS / (pid + ext)).write_bytes(data)
            item = {"id": pid, "file": pid + ext, "name": q.get("name", "")[:60], "x1": seg[0], "y1": seg[1],
                    "x2": seg[2], "y2": seg[3], "aspect": aspect, "t": round(time.time())}
            with lock:
                photos_save(photos_index() + [item])
            return self.json(item)
        if url.path == "/local/photos/delete":
            with lock:
                items = photos_index()
                gone = [p for p in items if p["id"] == q.get("id")]
                for p in gone:
                    (PHOTOS / Path(p["file"]).name).unlink(missing_ok=True)
                photos_save([p for p in items if p["id"] != q.get("id")])
            return self.json({"deleted": len(gone)})
        self.reply(404, "text/plain", b"not found")

    def log_message(self, *a):
        pass


def load_env(path=ROOT / ".env"):
    """KEY=value lines from .env; real environment variables win."""
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            k, sep, v = line.partition("=")
            if sep and not line.lstrip().startswith("#"):
                os.environ.setdefault(k.strip(), v.strip().strip('"'))


def main():
    global FETCH, BASE
    load_env()
    env = os.environ.get
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=env("ROBOROCK_URL"), help="Valetudo base URL or 'mock' (env ROBOROCK_URL)")
    ap.add_argument("--host", default=env("ROBOROCK_PANEL_HOST", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=int(env("ROBOROCK_PANEL_PORT", "8780")))
    ap.add_argument("--interval", type=float, default=1.0, help="status/map poll interval [s]")
    ap.add_argument("--ssh-key", default=env("ROBOROCK_SSH_KEY", str(Path.home() / ".ssh" / "roborock")),
                    help="key for the tap (env ROBOROCK_SSH_KEY)")
    ap.add_argument("--no-tap", action="store_true", default=env("ROBOROCK_TAP", "1") == "0",
                    help="don't read lidar/sensors/pose over SSH (env ROBOROCK_TAP=0)")
    args = ap.parse_args()
    if not args.robot:
        ap.error("set the robot address: ROBOROCK_URL in .env (copy .env.example) or --robot http://<robot-ip>")
    args.ssh_key = os.path.expanduser(args.ssh_key)

    FETCH = make_fetch(args.robot)
    BASE = None if args.robot == "mock" else args.robot.rstrip("/")
    if SAMPLES.exists():
        WIFI.extend(json.loads(l) for l in SAMPLES.read_text(encoding="utf-8").splitlines() if l.strip())
    if EVENTS_LOG.exists():
        old = [json.loads(l) for l in EVENTS_LOG.read_text(encoding="utf-8").splitlines() if l.strip()]
        EVENTS.extend(old[-300:])
        for ev in old:
            if ev.get("k") in analysis.HOT_KINDS and "x" in ev:
                hot_count(ev, live=False)
    threading.Thread(target=poller, args=(args.interval,), daemon=True).start()
    threading.Thread(target=recorder, daemon=True).start()
    if BASE and not args.no_tap:
        threading.Thread(target=tap_reader, args=(urlparse(args.robot).hostname, args.ssh_key), daemon=True).start()
    print(f"UI: http://{args.host}:{args.port}  ->  robot: {args.robot}")
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
