"""MCP server (stdio, JSON-RPC 2.0) for the Roborock S5 - a thin layer over the panel server (server.py).

All robot writes go through the panel's proxy, so its WRITE_RULES apply (no updater / Valetudo settings; map reset,
Wi-Fi config, quirks... only with confirm=true). Panel address: env ROBOROCK_PANEL (default http://127.0.0.1:8780).
ponytail: hand-rolled stdio JSON-RPC instead of the `mcp` package - stdlib only, like server.py.
Self-check: python mcp_server.py --test
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request

PANEL = os.environ.get("ROBOROCK_PANEL", "http://127.0.0.1:8780").rstrip("/")
CAP = "/api/v2/robot/capabilities/"


def call(method, path, body=None, confirm=False):
    req = urllib.request.Request(PANEL + path, method=method,
                                 data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", **({"X-Confirm": "tak"} if confirm else {})})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            raw = r.read()
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"{method} {path}: HTTP {e.code} {e.read()[:300].decode(errors='replace')}")
    except urllib.error.URLError as e:
        raise RuntimeError(f"panel {PANEL} niedostępny ({e.reason}) - uruchom server.py")
    try:
        return json.loads(raw) if raw else {}
    except ValueError:
        return raw.decode(errors="replace")


def ts(t):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))


# ---------------- tools ----------------
def status(_):
    s = call("GET", "/local/state")
    a = {x["__class"]: x for x in s.get("attrs", [])}
    sen = s.get("sensors") or {}
    out = {
        "status": a.get("StatusStateAttribute", {}).get("value"),
        "status_flag": a.get("StatusStateAttribute", {}).get("flag"),
        "battery_pct": a.get("BatteryStateAttribute", {}).get("level"),
        "battery_flag": a.get("BatteryStateAttribute", {}).get("flag"),
        "fan_speed": a.get("PresetSelectionStateAttribute", {}).get("value"),
        "pose_cm": s.get("pose"),
        "wifi_dbm": (s.get("wifi") or {}).get("signal"),
        "explore": s.get("explore"),
        "lidar_age_s": (s.get("scan") or {}).get("age"),
    }
    if sen:  # raw MCU data from the lidar tap; the % above is the firmware's estimate (unreliable on a worn battery)
        out["power"] = {k: sen.get(k) for k in ("mv", "ma", "watts", "soc", "temp", "dock")}
        out["sensors"] = {k: sen.get(k) for k in ("bumper_l", "bumper_r", "cliff", "drop", "wheel_l", "wheel_r",
                                                  "tilt", "wall", "error")}
    return out


def control(a):
    act = a["action"]
    if act == "locate":
        return call("PUT", CAP + "LocateCapability", {"action": "locate"})
    if act not in ("start", "stop", "pause", "home"):
        raise ValueError("action: start|stop|pause|home|locate")
    return call("PUT", CAP + "BasicControlCapability", {"action": act})


def rooms(_):
    segs = call("GET", CAP + "MapSegmentationCapability")
    doors = call("GET", "/local/cloud").get("doors", [])
    return {"segments": [{"id": g["id"], "name": g.get("name")} for g in segs],
            "doors": [{k: d.get(k) for k in ("rooms", "state", "width", "x1", "y1", "x2", "y2")} for d in doors]}


def clean_rooms(a):
    return call("PUT", CAP + "MapSegmentationCapability",
                {"action": "start_segment_action", "segment_ids": [str(i) for i in a["ids"]],
                 "iterations": int(a.get("iterations", 1)), "customOrder": True})


def clean_zone(a):
    x1, y1, x2, y2 = (int(a[k]) for k in ("x1", "y1", "x2", "y2"))
    x1, x2, y1, y2 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
    pts = {"pA": {"x": x1, "y": y1}, "pB": {"x": x2, "y": y1}, "pC": {"x": x2, "y": y2}, "pD": {"x": x1, "y": y2}}
    return call("PUT", CAP + "ZoneCleaningCapability",
                {"action": "clean", "zones": [{"points": pts}], "iterations": int(a.get("iterations", 1))})


def go_to(a):
    return call("PUT", CAP + "GoToLocationCapability",
                {"action": "goto", "coordinates": {"x": int(a["x"]), "y": int(a["y"])}})


def fan_speed(a):
    if "preset" not in a:
        return {"presets": call("GET", CAP + "FanSpeedControlCapability/presets")}
    return call("PUT", CAP + "FanSpeedControlCapability/preset", {"name": a["preset"]})


def carpet_mode(a):
    if "enabled" not in a:
        return call("GET", CAP + "CarpetModeControlCapability")
    return call("PUT", CAP + "CarpetModeControlCapability", {"action": "enable" if a["enabled"] else "disable"})


def consumables(_):
    return [{"part": f'{c["type"]}/{c["subType"]}', "remaining_h": round(c["remaining"]["value"] / 60, 1)
             if c["remaining"]["unit"] == "minutes" else c["remaining"]}
            for c in call("GET", CAP + "ConsumableMonitoringCapability")]


def statistics(_):
    conv = {"time": ("time_min", 1 / 60), "area": ("area_m2", 1 / 10000), "count": ("count", 1)}
    out = {}
    for name, cap in (("total", "TotalStatisticsCapability"), ("current", "CurrentStatisticsCapability")):
        out[name] = {conv[p["type"]][0]: round(p["value"] * conv[p["type"]][1], 1)
                     for p in call("GET", CAP + cap) if p["type"] in conv}
    return out


def explore(a):
    if a["action"] == "stop":
        return call("POST", "/local/explore/stop")
    q = f'?x={int(a["x"])}&y={int(a["y"])}' if "x" in a and "y" in a else ""
    return call("POST", "/local/explore/start" + q)


def events(a):
    n = int(a.get("limit", 20))
    kind = a.get("kind")
    ev = [e for e in call("GET", "/local/state").get("events", []) if not kind or e["k"] == kind]
    return [{"time": ts(e["t"]), "kind": e["k"], "msg": e["msg"], "x": e.get("x"), "y": e.get("y")} for e in ev[-n:]]


def map_analysis(_):
    c = call("GET", "/local/cloud")
    return {"doors": c.get("doors"), "openings_mirrors": c.get("reflective"),
            "moving_objects": c.get("moving"), "counts": {k: len(c.get(k) or []) for k in
                                                          ("cells", "phantom", "islands", "obstacles", "open_px")}}


def sessions(a):
    out = []
    for s in call("GET", "/local/sessions")[: int(a.get("limit", 10))]:
        s = {k: v for k, v in s.items() if k != "curve"}
        s["start"], s["end"] = ts(s["start"]), s.get("end") and ts(s["end"])
        out.append(s)
    return out


def wifi(_):
    return call("GET", "/local/wifi")


def hotspots(a):
    out = []
    for h in call("GET", "/local/hotspots")[: int(a.get("limit", 10))]:
        h = dict(h)
        h["first"], h["last"] = ts(h["first"]), ts(h["last"])
        out.append(h)
    return out


RESTR = CAP + "CombinedVirtualRestrictionsCapability"


def rect(x1, y1, x2, y2):
    x1, x2, y1, y2 = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
    return {"pA": {"x": x1, "y": y1}, "pB": {"x": x2, "y": y1}, "pC": {"x": x2, "y": y2}, "pD": {"x": x1, "y": y2}}


def restrictions(a):
    cur = call("GET", RESTR)
    r = {"virtualWalls": cur.get("virtualWalls", []), "restrictedZones": cur.get("restrictedZones", [])}
    act = a.get("action", "list")
    if act == "list":
        return r
    if act == "add_zone":
        if "hotspot_id" in a:
            h = next((h for h in call("GET", "/local/hotspots") if h["id"] == int(a["hotspot_id"])), None)
            if not h:
                raise ValueError(f"brak miejsca problemowego {a['hotspot_id']}")
            box = h["zone"]
        else:
            box = [int(a[k]) for k in ("x1", "y1", "x2", "y2")]
        r["restrictedZones"].append({"type": "regular", "points": rect(*box)})
    elif act == "add_wall":
        r["virtualWalls"].append({"points": {"pA": {"x": int(a["x1"]), "y": int(a["y1"])}, "pB": {"x": int(a["x2"]), "y": int(a["y2"])}}})
    elif act == "remove":
        key = "virtualWalls" if a["kind"] == "wall" else "restrictedZones"
        r[key].pop(int(a["index"]))
    elif act == "clear":
        r = {"virtualWalls": [], "restrictedZones": []}
    else:
        raise ValueError("action: list|add_zone|add_wall|remove|clear")
    call("PUT", RESTR, r)
    return {"saved": r}


def program_start(steps):
    return call("POST", "/local/program/start", steps)


def run_program(a):
    if a.get("help"):
        return call("GET", "/local/program")["help"]
    if a.get("list"):
        return [{k: p.get(k) for k in ("name", "title", "description")} | {"steps": len(p["steps"])} for p in call("GET", "/local/programs")]
    if a.get("name"):
        p = next((p for p in call("GET", "/local/programs") if p["name"] == a["name"]), None)
        if not p:
            raise ValueError(f"brak zapisanego programu {a['name']!r} (list=true pokazuje dostępne)")
        return program_start(p["steps"])
    return program_start(a["steps"])


def run_route(a):
    steps = []
    for p in a["points"]:
        steps.append({"goto": [p[0], p[1]]})
        if a.get("look", True):
            steps.append({"look": True})
    if a.get("home", True):
        steps.append({"home": True})
    return program_start(steps)


def drive(a):
    step = {a["action"]: a.get("value", True)}
    if a.get("push"):
        step["push"] = True
    return program_start([step])


def job_status(a):
    p = call("GET", "/local/program")
    s = call("GET", "/local/state")["explore"]
    return {"running": p["running"], "msg": p["msg"], "step": p["i"] + 1, "steps": len(p["steps"]),
            "results": p["results"][-int(a.get("last", 20)):], "explore_route": s.get("route")}


def stop_job(_):
    return call("POST", "/local/explore/stop")


def rename_room(a):
    return call("PUT", CAP + "MapSegmentRenameCapability", {"action": "rename_segment", "segment_id": str(a["id"]), "name": a["name"]})


def schedule(a):
    act = a.get("action", "list")
    if act == "list":
        return call("GET", "/api/v2/timers")
    if act == "delete":
        return call("DELETE", f"/api/v2/timers/{a['id']}")
    if act == "run_now":
        return call("PUT", f"/api/v2/timers/{a['id']}/action", {"action": "execute_now"})
    if act == "add":  # local time in, Valetudo wants UTC
        h, m = (int(v) for v in a["time"].split(":"))
        days = a.get("days", [0, 1, 2, 3, 4, 5, 6])  # 0 = niedziela
        t = time.mktime((2024, 1, 7, h, m, 0, 0, 0, -1))  # 2024-01-07 = Sunday
        dshift = int((time.gmtime(t).tm_mday - 7))
        u = time.gmtime(t)
        rooms = a.get("rooms")
        action = {"type": "segment_cleanup", "params": {"segment_ids": [str(r) for r in rooms], "iterations": 1, "custom_order": True}} \
            if rooms else {"type": "full_cleanup", "params": {}}
        body = {"enabled": True, "label": a.get("label", "")[:24], "dow": sorted({(d + dshift) % 7 for d in days}),
                "hour": u.tm_hour, "minute": u.tm_min, "action": action,
                "pre_actions": [{"type": "fan_speed_control", "params": {"value": a["fan"]}}] if a.get("fan") else []}
        return call("POST", "/api/v2/timers", body)
    raise ValueError("action: list|add|delete|run_now")


def valetudo_capabilities(_):
    return call("GET", "/api/v2/robot/capabilities")


def valetudo_api(a):
    path = a["path"] if a["path"].startswith("/api/") else "/api/v2/" + a["path"].lstrip("/")
    return call(a.get("method", "GET").upper(), path, a.get("body"), confirm=bool(a.get("confirm")))


XY = {"x": {"type": "number", "description": "cm, współrzędne mapy Valetudo"},
      "y": {"type": "number", "description": "cm, współrzędne mapy Valetudo"}}
TOOLS = {
    "robot_status": (status, "Stan robota: status, bateria (% i surowe mV/mA/temp z MCU), pozycja, wentylator, Wi-Fi, "
                             "czujniki (zderzaki, klify, koła), postęp eksploracji.", {}, []),
    "robot_control": (control, "Podstawowe polecenia: start sprzątania całości, stop, pause, home (powrót do stacji), "
                               "locate (robot się odzywa).",
                      {"action": {"type": "string", "enum": ["start", "stop", "pause", "home", "locate"]}}, ["action"]),
    "list_rooms": (rooms, "Lista pokoi (segmentów) z ich id oraz wykryte drzwi między pokojami.", {}, []),
    "clean_rooms": (clean_rooms, "Sprzątaj wybrane pokoje (id z list_rooms) w podanej kolejności.",
                    {"ids": {"type": "array", "items": {"type": "string"}},
                     "iterations": {"type": "integer", "minimum": 1, "maximum": 3}}, ["ids"]),
    "clean_zone": (clean_zone, "Sprzątaj prostokątną strefę (cm, współrzędne mapy).",
                   {"x1": {"type": "number"}, "y1": {"type": "number"}, "x2": {"type": "number"},
                    "y2": {"type": "number"}, "iterations": {"type": "integer", "minimum": 1, "maximum": 3}},
                   ["x1", "y1", "x2", "y2"]),
    "go_to": (go_to, "Jedź do punktu (planer robota; nie przejedzie przez drzwi zamknięte na jego mapie - wtedy "
                     "użyj explore z x,y).", XY, ["x", "y"]),
    "explore": (explore, "Eksploracja/skan mieszkania (bez x,y) albo jazda do punktu z próbą fizycznego przejazdu "
                         "przez drzwi (z x,y). action=stop przerywa.",
                {"action": {"type": "string", "enum": ["start", "stop"]}, **XY}, ["action"]),
    "fan_speed": (fan_speed, "Moc ssania. Bez preset zwraca listę dostępnych.", {"preset": {"type": "string"}}, []),
    "carpet_mode": (carpet_mode, "Tryb dywanowy (zwiększanie ssania na dywanie). Bez enabled zwraca stan.",
                    {"enabled": {"type": "boolean"}}, []),
    "consumables": (consumables, "Pozostały czas pracy szczotek, filtra i czujników (godziny).", {}, []),
    "statistics": (statistics, "Statystyki sprzątania: bieżące i łączne (czas, powierzchnia, liczba).", {}, []),
    "events": (events, "Ostatnie zdarzenia czujników i eksploracji (zderzak, drzwi, błędy...).",
               {"limit": {"type": "integer"}, "kind": {"type": "string", "description": "np. bumper, door, explore, error"}},
               []),
    "map_analysis": (map_analysis, "Analiza mapy z lidaru: drzwi i ich stan, lustra/szyby/otwory, ruchome obiekty.",
                     {}, []),
    "sessions": (sessions, "Historia sesji (sprzątanie/eksploracja): czas, bateria start/koniec, dystans, zdarzenia.",
                 {"limit": {"type": "integer"}}, []),
    "wifi_report": (wifi, "Pomiary siły Wi-Fi zebrane podczas jazdy.", {}, []),
    "hotspots": (hotspots, "Miejsca, w których robot wielokrotnie na coś wpada lub najeżdża (zderzak, przechył, opór kół, "
                           "wciąganie przez szczotkę), z całej historii. Dla każdego: liczba, rodzaje, ile dni, czy przy ścianie "
                           "z mapy (normalne) czy poza mapą (przedmiot), proponowana strefa zakazana 'zone' [x1,y1,x2,y2].",
                 {"limit": {"type": "integer"}}, []),
    "restrictions": (restrictions, "Strefy zakazane i wirtualne ściany. list | add_zone (x1..y2 albo hotspot_id) | add_wall "
                                   "(x1,y1,x2,y2) | remove (kind=zone|wall, index) | clear. Zmieniaj tylko po zgodzie użytkownika.",
                     {"action": {"type": "string", "enum": ["list", "add_zone", "add_wall", "remove", "clear"]},
                      "hotspot_id": {"type": "integer"}, "x1": {"type": "number"}, "y1": {"type": "number"},
                      "x2": {"type": "number"}, "y2": {"type": "number"},
                      "kind": {"type": "string", "enum": ["zone", "wall"]}, "index": {"type": "integer"}}, []),
    "run_route": (run_route, "Przejazd przez listę punktów (planer robota) z pomiarem lidarem po każdym; raport per punkt "
                             "w job_status (dojechał/nie, zdarzenia czujników). Domyślnie na końcu powrót do stacji.",
                  {"points": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                   "look": {"type": "boolean"}, "home": {"type": "boolean"}}, ["points"]),
    "run_program": (run_program, "Sterowanie proceduralne: lista kroków wykonywana po kolei (goto, forward/back cm, turn/face "
                                 "deg, look, wait, wait_done, clean_zone, clean_rooms, fan, locate, home, stop, explore). help=true "
                                 "zwraca opis kroków, list=true zapisane programy (np. serwis_mapy), name=... uruchamia "
                                 "zapisany. Postęp i wyniki: job_status.",
                    {"steps": {"type": "array", "items": {"type": "object"}}, "help": {"type": "boolean"},
                     "list": {"type": "boolean"}, "name": {"type": "string"}}, []),
    "drive": (drive, "Pojedynczy ruch w sterowaniu ręcznym: forward/back (cm, maks. 300), turn (stopnie, + = w prawo), "
                     "face (kierunek mapy), look (pomiar odległości lidarem). Do przodu staje przed przeszkodą, push=true "
                     "jedzie aż do zderzaka.",
              {"action": {"type": "string", "enum": ["forward", "back", "turn", "face", "look"]},
               "value": {"type": "number"}, "push": {"type": "boolean"}}, ["action"]),
    "job_status": (job_status, "Postęp bieżącego zadania (program / trasa / skan) i wyniki kroków.",
                   {"last": {"type": "integer"}}, []),
    "stop_job": (stop_job, "Przerwij program / trasę / skan (robot zatrzymuje się w miejscu).", {}, []),
    "rename_room": (rename_room, "Nadaj nazwę pokojowi (segmentowi) o danym id.",
                    {"id": {"type": "string"}, "name": {"type": "string"}}, ["id", "name"]),
    "schedule": (schedule, "Harmonogram sprzątań: list | add (time 'HH:MM' czasu lokalnego, days 0=nd..6=sb, rooms "
                           "opcjonalnie, fan opcjonalnie, label) | delete (id) | run_now (id).",
                 {"action": {"type": "string", "enum": ["list", "add", "delete", "run_now"]}, "id": {"type": "string"},
                  "time": {"type": "string"}, "days": {"type": "array", "items": {"type": "integer"}},
                  "rooms": {"type": "array", "items": {"type": "string"}}, "fan": {"type": "string"},
                  "label": {"type": "string"}}, []),
    "valetudo_capabilities": (valetudo_capabilities, "Lista wszystkich funkcji (capabilities) robota w Valetudo.", {}, []),
    "valetudo_api": (valetudo_api, "Dowolne wywołanie API Valetudo (GET wszystko; zapis: capabilities, timers, events, "
                                   "log level). Ścieżka np. 'robot/capabilities/DoNotDisturbCapability'. Operacje "
                                   "ryzykowne (MapReset, WifiConfiguration, PersistentMapControl, MapSnapshot, Quirks, "
                                   "VoicePack) wymagają confirm=true - ustaw tylko po wyraźnej zgodzie użytkownika.",
                     {"method": {"type": "string", "enum": ["GET", "PUT", "POST", "DELETE"]}, "path": {"type": "string"},
                      "body": {"type": "object"}, "confirm": {"type": "boolean"}}, ["path"]),
}


def handle(msg):
    m, mid, p = msg.get("method"), msg.get("id"), msg.get("params") or {}
    if mid is None:  # notification (initialized, cancelled...)
        return None
    if m == "initialize":
        res = {"protocolVersion": p.get("protocolVersion", "2025-06-18"), "capabilities": {"tools": {}},
               "serverInfo": {"name": "roborock-s5", "version": "1.0"}}
    elif m == "ping":
        res = {}
    elif m == "tools/list":
        res = {"tools": [{"name": n, "description": d,
                          "inputSchema": {"type": "object", "properties": props, "required": req}}
                         for n, (_, d, props, req) in TOOLS.items()]}
    elif m == "tools/call":
        t = TOOLS.get(p.get("name"))
        if not t:
            return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32602, "message": f"nieznane narzędzie {p.get('name')}"}}
        try:
            out, err = t[0](p.get("arguments") or {}), False
        except Exception as e:  # tool errors go back to the model, not as protocol errors
            out, err = f"{type(e).__name__}: {e}", True
        text = out if isinstance(out, str) else json.dumps(out, ensure_ascii=False, indent=1)
        res = {"content": [{"type": "text", "text": text}], "isError": err}
    else:
        return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32601, "message": f"brak metody {m}"}}
    return {"jsonrpc": "2.0", "id": mid, "result": res}


def main():
    sys.stdin.reconfigure(encoding="utf-8")
    sys.stdout.reconfigure(encoding="utf-8")  # Windows console default (cp1250) would mangle Polish text
    for line in sys.stdin:
        if line.strip():
            out = handle(json.loads(line))
            if out is not None:
                sys.stdout.write(json.dumps(out, ensure_ascii=False) + "\n")
                sys.stdout.flush()


def selftest():
    global call
    sent = []
    call = lambda meth, path, body=None, confirm=False: sent.append((meth, path, body)) or (
        [{"id": "1"}] if path.endswith("MapSegmentationCapability") and meth == "GET" else {})
    assert handle({"jsonrpc": "2.0", "method": "notifications/initialized"}) is None
    assert handle({"id": 1, "method": "initialize", "params": {"protocolVersion": "X"}})["result"]["protocolVersion"] == "X"
    assert len(handle({"id": 2, "method": "tools/list"})["result"]["tools"]) == len(TOOLS)
    r = handle({"id": 3, "method": "tools/call", "params": {"name": "clean_zone",
                                                             "arguments": {"x1": 300, "y1": 50, "x2": 100, "y2": 200}}})
    assert not r["result"]["isError"] and sent[-1][2]["zones"][0]["points"]["pA"] == {"x": 100, "y": 50}, sent
    r = handle({"id": 4, "method": "tools/call", "params": {"name": "robot_control", "arguments": {"action": "fly"}}})
    assert r["result"]["isError"]
    handle({"id": 5, "method": "tools/call", "params": {"name": "explore", "arguments": {"action": "start", "x": 1, "y": 2}}})
    assert sent[-1][:2] == ("POST", "/local/explore/start?x=1&y=2")
    assert handle({"id": 6, "method": "nope"})["error"]["code"] == -32601
    tc = lambda name, args: handle({"id": 9, "method": "tools/call", "params": {"name": name, "arguments": args}})["result"]
    tc("run_route", {"points": [[1, 2], [3, 4]]})
    assert sent[-1][2] == [{"goto": [1, 2]}, {"look": True}, {"goto": [3, 4]}, {"look": True}, {"home": True}], sent[-1]
    tc("drive", {"action": "forward", "value": 50, "push": True})
    assert sent[-1][2] == [{"forward": 50, "push": True}]
    tc("restrictions", {"action": "add_zone", "x1": 10, "y1": 30, "x2": 0, "y2": 20})
    assert sent[-1][2]["restrictedZones"][0]["points"]["pC"] == {"x": 10, "y": 30}, sent[-1]
    tc("valetudo_api", {"method": "put", "path": "robot/capabilities/X", "body": {}})
    assert sent[-1][:2] == ("PUT", "/api/v2/robot/capabilities/X")
    tc("schedule", {"action": "add", "time": "10:30", "days": [1]})
    b = sent[-1][2]
    assert time.localtime(time.mktime((2024, 1, 8, 10, 30, 0, 0, 0, -1))).tm_hour == 10
    u = time.gmtime(time.mktime((2024, 1, 8, 10, 30, 0, 0, 0, -1)))  # Monday 10:30 local in UTC
    assert (b["hour"], b["minute"], b["dow"]) == (u.tm_hour, u.tm_min, [(u.tm_wday + 1) % 7]), b
    print("mcp_server: ok")


if __name__ == "__main__":
    selftest() if "--test" in sys.argv else main()
