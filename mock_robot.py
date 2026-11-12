"""Fake Valetudo API for working without the robot: two open-plan rooms, a moving robot,
Wi-Fi signal falling off with distance from an access point. Same JSON shapes as Valetudo."""
import json
import math
import random
import time

PS = 5            # cm per map pixel (Roborock uses 5)
OFF = 400         # real maps sit far from (0,0)
ROOMS = [("16", "Salon", 0, 0, 120, 80), ("17", "Kuchnia", 120, 20, 60, 60)]  # id, name, x, y, w, h [px]
AP_CM = ((OFF + 10) * PS, (OFF + 10) * PS)
DOCK_CM = ((OFF + 5) * PS, (OFF + 40) * PS)


def compress(pixels):
    """(x, y) set -> Valetudo RLE [xStart, y, count, ...]"""
    out = []
    for x, y in sorted(pixels, key=lambda p: (p[1], p[0])):
        if out and out[-2] == y and out[-3] + out[-1] == x:
            out[-1] += 1
        else:
            out += [x, y, 1]
    return out


def room_pixels(x, y, w, h):
    return {(OFF + x + i, OFF + y + j) for i in range(w) for j in range(h)}


def robot_cm(t):
    a = t / 15
    return ((OFF + 90 + 70 * math.cos(a)) * PS, (OFF + 45 + 25 * math.sin(a)) * PS, math.degrees(a) + 90)


def signal_at(x, y):
    d = math.dist((x, y), AP_CM)
    return round(-30 - 30 * math.log10(1 + d / 50) + random.uniform(-2, 2))


def build_map():
    inside = set().union(*(room_pixels(*r[2:]) for r in ROOMS))
    walls = {(x + dx, y + dy) for x, y in inside for dx in (-1, 0, 1) for dy in (-1, 0, 1)} - inside
    layers = [{"__class": "MapLayer", "type": "segment", "pixels": [],
               "compressedPixels": compress(room_pixels(*r[2:])),
               "metaData": {"segmentId": r[0], "name": r[1], "active": False, "area": r[4] * r[5] * PS * PS}}
              for r in ROOMS]
    layers.append({"__class": "MapLayer", "type": "wall", "pixels": [], "compressedPixels": compress(walls),
                   "metaData": {}})
    t = time.time()
    x, y, ang = robot_cm(t)
    path = [round(v) for k in range(300, 0, -1) for v in robot_cm(t - k * 0.5)[:2]]
    return {
        "__class": "ValetudoMap", "metaData": {"version": 2, "nonce": str(t)},
        "size": {"x": 5120, "y": 5120}, "pixelSize": PS, "layers": layers,
        "entities": [
            {"__class": "PathMapEntity", "type": "path", "points": path, "metaData": {}},
            {"__class": "PointMapEntity", "type": "charger_location", "points": list(DOCK_CM), "metaData": {}},
            {"__class": "PointMapEntity", "type": "robot_position", "points": [round(x), round(y)],
             "metaData": {"angle": round(ang) % 360}},
            {"__class": "LineMapEntity", "type": "virtual_wall",
             "points": [(OFF + 150) * PS, (OFF + 25) * PS, (OFF + 175) * PS, (OFF + 25) * PS], "metaData": {}},
        ],
    }


def routes():
    x, y, _ = robot_cm(time.time())
    return {
        "/api/v2/robot": {"manufacturer": "Roborock", "modelName": "S5 (mock)",
                          "implementation": "RoborockS5ValetudoRobot"},
        "/api/v2/valetudo/version": {"release": "mock", "commit": "mock"},
        "/api/v2/robot/state/map": build_map(),
        "/api/v2/robot/state/attributes": [
            {"__class": "StatusStateAttribute", "value": "cleaning", "flag": "none"},
            {"__class": "BatteryStateAttribute", "level": 87, "flag": "discharging"},
            {"__class": "PresetSelectionStateAttribute", "type": "fan_speed", "value": "medium"},
            {"__class": "ConsumableStateAttribute", "type": "brush", "subType": "main",
             "remaining": {"value": 15420, "unit": "minutes"}},
            {"__class": "ConsumableStateAttribute", "type": "filter", "subType": "main",
             "remaining": {"value": 6100, "unit": "minutes"}},
        ],
        "/api/v2/robot/capabilities": ["WifiConfigurationCapability", "CurrentStatisticsCapability"],
        "/api/v2/robot/capabilities/WifiConfigurationCapability": {
            "__class": "ValetudoWifiStatus", "state": "connected",
            "details": {"ssid": "DomWiFi", "bssid": "aa:bb:cc:dd:ee:ff", "signal": signal_at(x, y),
                        "frequency": "2.4ghz", "ips": ["192.168.0.50"]}},
        "/api/v2/robot/capabilities/CurrentStatisticsCapability": [
            {"__class": "ValetudoDataPoint", "type": "time", "value": 1260},
            {"__class": "ValetudoDataPoint", "type": "area", "value": 184000}],
        "/api/v2/system/host/info": {"hostname": "mock", "arch": "arm", "uptime": round(time.monotonic()),
                                     "mem": {"total": 536870912, "free": 120000000}, "load": {"1": 0.8}},
    }


def fake_api(path):
    data = routes().get(path.split("?")[0])
    if data is None:
        return 404, "text/plain", b"not found (mock)"
    return 200, "application/json", json.dumps(data).encode()


if __name__ == "__main__":  # self-check of the RLE encoder against Valetudo's own example
    assert compress({(37, 5), (38, 5), (39, 5), (10, 6)}) == [37, 5, 3, 10, 6, 1]
    assert fake_api("/api/v2/robot/state/map")[0] == 200
    print("mock_robot: ok")
