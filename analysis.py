"""Pure map + lidar analysis (no I/O). Self-check: python analysis.py

Coordinates: map pixels (Valetudo layers) and cm (entities, lidar). px = cm // pixelSize.
"""
import math
import time
from collections import Counter, deque

WALL, OBSTACLE, PHANTOM = 0, 1, 2
SMALL_OBSTACLE_CM = 30


def map_sets(m):
    """-> walls, walkable (floor + segments), rooms [(id, name, pixel set)]"""
    walls, walk, rooms = set(), set(), []
    for layer in m.get("layers", []):
        px = set()
        c = layer.get("compressedPixels") or []
        for i in range(0, len(c), 3):
            px.update((c[i] + k, c[i + 1]) for k in range(c[i + 2]))
        if layer["type"] == "wall":
            walls |= px
        else:
            walk |= px
            if layer["type"] == "segment":
                md = layer.get("metaData") or {}
                rooms.append((str(md.get("segmentId")), md.get("name") or f"Pokój {md.get('segmentId')}", px))
    return walls, walk, rooms


def dilate(px, r):
    return {(x + dx, y + dy) for x, y in px for dx in range(-r, r + 1) for dy in range(-r, r + 1)}


def classify_cloud(cells, cell_cm, ps, near_wall, walk):
    """cells {(cx, cy): hits} -> [[x_cm, y_cm, hits, cls]]: on a mapped wall / inside the flat (obstacle) /
    outside the flat behind walls (phantom: mirror reflections, through glass or open doors)."""
    out = []
    for (cx, cy), n in cells.items():
        x, y = cx * cell_cm + cell_cm // 2, cy * cell_cm + cell_cm // 2
        p = (x // ps, y // ps)
        cls = WALL if p in near_wall else OBSTACLE if p in walk else PHANTOM
        out.append([x, y, n, cls])
    return out


def obstacle_clusters(classified, cell_cm, min_hits=3):
    """Connected obstacle cells -> boxes; small ones (chair legs, cables, toys) vs big (furniture)."""
    cells = {(x // cell_cm, y // cell_cm) for x, y, n, cls in classified if cls == OBSTACLE and n >= min_hits}
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        comp, q = [], deque([start])
        seen.add(start)
        while q:
            c = q.popleft()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (c[0] + dx, c[1] + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        q.append(nb)
        xs, ys = [c[0] for c in comp], [c[1] for c in comp]
        w, h = (max(xs) - min(xs) + 1) * cell_cm, (max(ys) - min(ys) + 1) * cell_cm
        out.append({"x": min(xs) * cell_cm, "y": min(ys) * cell_cm, "w": w, "h": h, "cells": len(comp),
                    "kind": "drobna" if max(w, h) <= SMALL_OBSTACLE_CM else "duża"})
    return out


def expected_wall(walls, ps, x_cm, y_cm, angle_deg, max_cm=350):
    """First mapped wall pixel along a ray, or None (used to spot glass: map says wall, lidar got nothing)."""
    th = math.radians(angle_deg)
    dx, dy = math.cos(th), math.sin(th)
    for d in range(10, max_cm, ps):
        p = (int((x_cm + dx * d) // ps), int((y_cm + dy * d) // ps))
        if p in walls:
            return p
    return None


def glass_candidates(seen, miss, ps, min_miss=8, ratio=0.6):
    """Wall pixels the lidar mostly passes through -> likely glass / mirror / very dark surface."""
    out = []
    for p, m in miss.items():
        r = m / (m + seen.get(p, 0))
        if m >= min_miss and r >= ratio:
            out.append([p[0] * ps + ps // 2, p[1] * ps + ps // 2, round(r, 2)])
    return out


N8 = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]


def components(px, gap=1):
    """8-connected components of a pixel set (gap=2 also joins pixels one apart)."""
    px, out = set(px), []
    nb = [(dx, dy) for dx in range(-gap, gap + 1) for dy in range(-gap, gap + 1) if dx or dy]
    while px:
        q = [px.pop()]
        comp = set(q)
        while q:
            x, y = q.pop()
            for dx, dy in nb:
                p = (x + dx, y + dy)
                if p in px:
                    px.discard(p)
                    comp.add(p)
                    q.append(p)
        out.append(comp)
    return out


def wall_parts(walls, walk):
    """Split the map's wall layer (which is "everything the lidar hit") into
    structural walls (components touching the unknown outside of the flat) and islands (furniture, table legs,
    things standing free on the floor) -> (structural set, [island sets])."""
    if not walls:
        return set(), []
    xs, ys = [p[0] for p in walls], [p[1] for p in walls]
    x0, x1, y0, y1 = min(xs) - 2, max(xs) + 2, min(ys) - 2, max(ys) + 2
    outside, q = {(x0, y0)}, deque([(x0, y0)])
    while q:  # flood the unknown area from outside the bounding box
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            p = (x + dx, y + dy)
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p not in outside and p not in walls and p not in walk:
                outside.add(p)
                q.append(p)
    structural, islands = set(), []
    for comp in components(walls):
        if any((x + dx, y + dy) in outside for x, y in comp for dx, dy in N8):
            structural |= comp
        else:
            islands.append(comp)
    return structural, islands


def ray_wall(walls, ps, ox, oy, th, dist_cm, margin_cm=30):
    """First wall pixel a ray crosses clearly before its echo (dist_cm), or None.
    A lidar can't see through a wall: an echo from beyond one came through glass, a mirror or a doorway."""
    dx, dy, d = math.cos(th), math.sin(th), 10
    while d < dist_cm - margin_cm:
        p = (int((ox + dx * d) // ps), int((oy + dy * d) // ps))
        if p in walls:
            return p
        d += ps
    return None


def fit_line(pts):
    """PCA line through points -> (cx, cy, ux, uy, half_length)."""
    n = len(pts)
    cx, cy = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
    sxx = sum((p[0] - cx) ** 2 for p in pts)
    syy = sum((p[1] - cy) ** 2 for p in pts)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in pts)
    th = 0.5 * math.atan2(2 * sxy, sxx - syy)
    ux, uy = math.cos(th), math.sin(th)
    proj = [(p[0] - cx) * ux + (p[1] - cy) * uy for p in pts]
    return cx, cy, ux, uy, (max(proj) - min(proj)) / 2, (max(proj) + min(proj)) / 2


DOOR_MIN_CM, DOOR_MAX_CM = 55, 115


def doorways(rooms, ps):
    """Open passages between rooms of the map: where pixels of two rooms touch with no wall in between.
    -> [{x1, y1, x2, y2, width, kind: "drzwi" | "przejście", rooms: [a, b]}] (cm)."""
    owner = {}
    for i, (_, _, px) in enumerate(rooms):
        for q in px:
            owner[q] = i
    border = {}
    for (x, y), i in owner.items():
        for dx, dy in ((1, 0), (0, 1)):
            j = owner.get((x + dx, y + dy))
            if j is not None and j != i:
                border.setdefault((min(i, j), max(i, j)), set()).add((x, y))
    out = []
    for (i, j), px in border.items():
        for comp in components(px, gap=2):
            pts = [(q[0] * ps + ps / 2, q[1] * ps + ps / 2) for q in comp]
            cx, cy, ux, uy, half, mid = fit_line(pts)
            width = round(2 * half + ps)
            if width < 40:
                continue
            out.append({"x1": round(cx + ux * (mid - half)), "y1": round(cy + uy * (mid - half)),
                        "x2": round(cx + ux * (mid + half)), "y2": round(cy + uy * (mid + half)), "width": width,
                        "kind": "drzwi" if width <= DOOR_MAX_CM + 15 else "przejście", "rooms": [rooms[i][1], rooms[j][1]]})
    return out


def seg_dist(px, py, s):
    """Distance from a point to the segment s = {x1, y1, x2, y2}."""
    vx, vy = s["x2"] - s["x1"], s["y2"] - s["y1"]
    f = max(0.0, min(1.0, ((px - s["x1"]) * vx + (py - s["y1"]) * vy) / ((vx * vx + vy * vy) or 1)))
    return math.hypot(px - s["x1"] - f * vx, py - s["y1"] - f * vy)


def walk_regions(walk, r):
    """Floor split into areas the robot can actually drive between: cores = pixels more than r px from the floor's
    edge (4-connected), each floor pixel then belongs to the core within r px of it. -> {px: region id}"""
    clear, q = {}, deque()
    for p in walk:
        if any(nb not in walk for nb in ((p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1))):
            clear[p] = 1
            q.append(p)
    while q:
        p = q.popleft()
        for nb in ((p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)):
            if nb in walk and nb not in clear:
                clear[nb] = clear[p] + 1
                q.append(nb)
    lab, q = {}, deque()
    for i, c in enumerate(components({p for p, d in clear.items() if d > r}, gap=1)):
        for p in c:
            lab[p] = i + 1
            q.append(p)
    while q:  # grow each core back over the edge pixels it owns
        p = q.popleft()
        for nb in ((p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)):
            if nb in walk and nb not in lab:
                lab[nb] = lab[p]
                q.append(nb)
    return lab


def shut_doors(walls, walk, target, ps, max_thick=3, robot_r=3, start=None):
    """Door-sized thin walls that separate the target's part of the floor from the rest of the flat
    (a door that was shut when the robot mapped it). target in cm. -> segments, nearest to target first."""
    t = (int(target[0] // ps), int(target[1] // ps))
    if t not in walk:
        t = min(walk, key=lambda q: (q[0] - t[0]) ** 2 + (q[1] - t[1]) ** 2, default=None)
    if t is None:
        return []
    lab = walk_regions(walk, robot_r)
    tl = lab.get(t) or next((lab[q] for q in sorted(lab, key=lambda q: (q[0] - t[0]) ** 2 + (q[1] - t[1]) ** 2)), None)
    comp = {p for p, v in lab.items() if v == tl}
    if start and (int(start[0] // ps), int(start[1] // ps)) in comp:
        return []  # robot is already on the target's side: no door in between
    size = Counter(lab.values())
    door = {}  # wall px -> region on the other side (only real rooms, >= 1 m2 - not map noise)
    for w in walls:
        for dx, dy in ((1, 0), (0, 1)):
            a, b, run = w, w, 1
            while run <= max_thick and (a[0] - dx, a[1] - dy) in walls:
                a, run = (a[0] - dx, a[1] - dy), run + 1
            while run <= max_thick and (b[0] + dx, b[1] + dy) in walls:
                b, run = (b[0] + dx, b[1] + dy), run + 1
            a, b = (a[0] - dx, a[1] - dy), (b[0] + dx, b[1] + dy)
            if run <= max_thick and a in lab and b in lab and (a in comp) != (b in comp):  # thin wall, target side | other side
                other = lab[b] if a in comp else lab[a]
                if size[other] * ps * ps >= 1e4:
                    door[w] = other
    out = []
    for c in (c for o in set(door.values()) for c in components({w for w, v in door.items() if v == o}, gap=2)):
        pts = [(x * ps + ps / 2, y * ps + ps / 2) for x, y in c]
        cx, cy, ux, uy, half, mid = fit_line(pts)
        length = 2 * half + ps
        if 35 <= length <= 150:
            out.append({"x1": round(cx + ux * (mid - half)), "y1": round(cy + uy * (mid - half)),
                        "x2": round(cx + ux * (mid + half)), "y2": round(cy + uy * (mid + half)),
                        "kind": "drzwi zamknięte na mapie", "n": len(c)})
    return sorted(out, key=lambda g: math.hypot((g["x1"] + g["x2"]) / 2 - target[0], (g["y1"] + g["y2"]) / 2 - target[1]))


def door_cells(doors, cell, band=2):
    """Grid cells lying on each doorway (+- band cells across it) -> {cell: (door index, position along the door)}.
    Also stores the number of positions in door["nk"]."""
    out = {}
    for i, d in enumerate(doors):
        length = math.hypot(d["x2"] - d["x1"], d["y2"] - d["y1"]) or 1
        ux, uy = (d["x2"] - d["x1"]) / length, (d["y2"] - d["y1"]) / length
        d["nk"] = max(1, round(length / cell))
        for k in range(d["nk"]):
            for off in range(-band, band + 1):
                x = d["x1"] + ux * (k + 0.5) * cell - uy * off * cell
                y = d["y1"] + uy * (k + 0.5) * cell + ux * off * cell
                out.setdefault((int(x // cell), int(y // cell)), (i, k))
    return out


def door_state(i, nk, hit, passed, now, fresh_s=20):
    """Is doorway i shut? Each position along it votes by its newest observation: an echo ON the line (hit) = shut,
    a ray THROUGH it (passed) = open. -> "zamknięte" | "otwarte" | None (not looked at lately / undecided)."""
    shut = opened = 0
    for k in range(nk):
        h, q = hit.get((i, k), 0), passed.get((i, k), 0)
        if now - max(h, q) < fresh_s:
            shut, opened = shut + (h > q), opened + (q >= h)
    if shut + opened < 0.5 * nk:
        return None
    return "zamknięte" if shut >= 0.6 * (shut + opened) else "otwarte" if opened >= 0.6 * (shut + opened) else None


def reflective_segments(through, far, seen, miss, near_wall, ps, min_through=10, min_px=4, min_ratio=0.5):
    """Wall stretches the lidar looks through, classified:
      lustro       - echoes from behind it, mirrored back, land on real walls (a virtual image of the room)
      drzwi?       - door-sized stretch with ordinary echoes behind it: a door that was shut when the map was made
      szyba/otwór  - echoes from behind it that are not a mirror image (window, glass door, wide opening)
      brak odbicia - the map has a wall but the lidar mostly gets no echo there (glass at an angle, black gloss)
    through/seen/miss: Counter wall px -> rays; far: wall px -> [(x_cm, y_cm)] echoes seen through that pixel."""
    out = []
    # a wall pixel counts only if rays mostly go THROUGH it (a solid wall also collects a few stray crossings)
    cand = {p for p, n in through.items() if n >= min_through and n / (n + seen.get(p, 0)) >= min_ratio}
    for comp in components(cand, gap=2):
        if len(comp) < min_px:
            continue
        pts = [(p[0] * ps + ps / 2, p[1] * ps + ps / 2) for p in comp]
        cx, cy, ux, uy, half, mid = fit_line(pts)
        nx, ny = -uy, ux
        fars = [f for p in comp for f in far.get(p, ())]
        on_wall = 0
        for fx, fy in fars:  # mirror the echo across the wall line
            k = 2 * ((fx - cx) * nx + (fy - cy) * ny)
            rx, ry = fx - k * nx, fy - k * ny
            on_wall += (int(rx // ps), int(ry // ps)) in near_wall
        score = on_wall / len(fars) if fars else 0.0
        length = 2 * half + ps
        kind = "lustro" if len(fars) >= 8 and score >= 0.45 else "drzwi?" if DOOR_MIN_CM <= length <= DOOR_MAX_CM else "szyba/otwór"
        seg = {"x1": round(cx + ux * (mid - half)), "y1": round(cy + uy * (mid - half)),
               "x2": round(cx + ux * (mid + half)), "y2": round(cy + uy * (mid + half)),
               "kind": kind, "score": round(score, 2), "n": sum(through[p] for p in comp)}
        if fars and kind != "lustro":  # where to drive to look behind it: 1 m past the middle, on the echoes' side
            mx, my = cx + ux * mid, cy + uy * mid
            side = 1 if sum((fx - mx) * nx + (fy - my) * ny for fx, fy in fars) > 0 else -1
            seg["beyond"] = [round(mx + side * nx * 100), round(my + side * ny * 100)]
        out.append(seg)
    dark = {p for p, m in miss.items() if m >= 40 and m / (m + seen.get(p, 0)) >= 0.9 and p not in cand}
    for comp in components(dark, gap=1):
        if len(comp) < 8:  # at least 40 cm of wall that practically never answers
            continue
        pts = [(p[0] * ps + ps / 2, p[1] * ps + ps / 2) for p in comp]
        cx, cy, ux, uy, half, mid = fit_line(pts)
        out.append({"x1": round(cx + ux * (mid - half)), "y1": round(cy + uy * (mid - half)),
                    "x2": round(cx + ux * (mid + half)), "y2": round(cy + uy * (mid + half)),
                    "kind": "brak odbicia", "score": 0.0, "n": sum(miss[p] for p in comp)})
    return out


def wifi_by_room(samples, rooms, ps):
    """[{name, avg, min, max, n}] of Wi-Fi dBm samples per room, weakest first."""
    out = []
    for _, name, px in rooms:
        v = [s["signal"] for s in samples if (int(s["x"] // ps), int(s["y"] // ps)) in px]
        if v:
            out.append({"name": name, "avg": round(sum(v) / len(v)), "min": min(v), "max": max(v), "n": len(v)})
    return sorted(out, key=lambda r: r["avg"])


def plan_waypoints(m, start_cm, room_m2=10, min_clear_px=6):
    """Exploration stops: per room ceil(area/room_m2) points, each as far from walls/edges as possible
    (lidar sees everything from the middle; no lane-by-lane cleaning). Ordered nearest-neighbour."""
    ps = m["pixelSize"]
    _, walk, rooms = map_sets(m)
    if not walk:
        return []
    clear, q = {}, deque()
    for p in walk:  # BFS distance from the walkable area's edge
        if any((p[0] + dx, p[1] + dy) not in walk for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
            clear[p] = 1
            q.append(p)
    while q:
        p = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (p[0] + dx, p[1] + dy)
            if nb in walk and nb not in clear:
                clear[nb] = clear[p] + 1
                q.append(nb)
    stops = []
    for _, name, px in rooms or [("0", "Mieszkanie", walk)]:
        pts = list(px)
        k = max(1, math.ceil(len(pts) * ps * ps / 1e4 / room_m2))
        seeds = [max(pts, key=lambda p: clear.get(p, 0))]
        while len(seeds) < k:  # farthest-point seeding spreads the stops over the room
            seeds.append(max(pts, key=lambda p: min((p[0] - s[0]) ** 2 + (p[1] - s[1]) ** 2 for s in seeds)))
        groups = [[] for _ in seeds]
        for p in pts:
            groups[min(range(k), key=lambda i: (p[0] - seeds[i][0]) ** 2 + (p[1] - seeds[i][1]) ** 2)].append(p)
        for g in groups:
            best = max(g, key=lambda p: clear.get(p, 0)) if g else None
            if best and clear.get(best, 0) >= min_clear_px:
                stops.append((best[0] * ps + ps // 2, best[1] * ps + ps // 2, name))
    route, cur = [], start_cm
    while stops:
        nxt = min(stops, key=lambda s: (s[0] - cur[0]) ** 2 + (s[1] - cur[1]) ** 2)
        stops.remove(nxt)
        route.append(nxt)
        cur = nxt
    return route


# ---------- places where the robot keeps running into / onto something ----------
HOT_KINDS = {"bumper": "uderzenie", "lds_bumper": "uderzenie wieżyczką", "tilt": "najazd / przechył",
             "drive_load": "opór kół", "brush_jam": "wciąganie przez szczotkę", "cliff": "krawędź", "drop": "uniesienie koła"}
ROBOT_R = 17  # cm, bumper radius


def contact_point(e):
    """Where the robot touched the thing: bumper hits ~at the front edge (side L/P = 35 deg off), the rest under it.
    ponytail: events logged before 2026-11-10 have no heading ("a") -> robot centre, ~17 cm off."""
    x, y = e["x"], e["y"]
    if e["k"] in ("bumper", "lds_bumper") and e.get("a") is not None:
        h = math.radians(e["a"] - 90 + {"L": -35, "P": 35}.get(e.get("side"), 0))  # map heading = Valetudo angle - 90
        return x + ROBOT_R * math.cos(h), y + ROBOT_R * math.sin(h)
    return x, y


def contact_known(e):
    return e["k"] not in ("bumper", "lds_bumper") or e.get("a") is not None


def hotspots(events, walls, ps, radius=30, min_n=3):
    """Group problem events (hits, climbing onto things, brush jams...) into places <= 2*radius across.
    -> list, worst first. near_wall: contact at a wall of the robot's map (normal edge cleaning), else off the map.
    ponytail: greedy densest-point grouping, O(n^2) - fine for the ~1000s of events a flat produces."""
    left = [(e, *contact_point(e)) for e in events if e.get("k") in HOT_KINDS and e.get("x") is not None]
    near, near_c = dilate(walls, 2), dilate(walls, 5)  # contact point known: 10 cm; robot centre only: 25 cm
    at_wall = lambda e, x, y: (int(x // ps), int(y // ps)) in (near if contact_known(e) else near_c)
    out = []
    while left:
        close = lambda p: [q for q in left if (q[1] - p[1]) ** 2 + (q[2] - p[2]) ** 2 <= radius ** 2]
        items = close(max(left, key=lambda p: len(close(p))))
        if len(items) < min_n:
            break
        ids = {id(q) for q in items}
        left = [q for q in left if id(q) not in ids]
        xs, ys = [x for _, x, _ in items], [y for _, _, y in items]
        kinds = Counter(e["k"] for e, _, _ in items)
        wall_share = sum(at_wall(e, x, y) for e, x, y in items) / len(items)
        days = len({time.strftime("%Y-%m-%d", time.localtime(e["t"])) for e, _, _ in items})
        top = kinds.most_common(1)[0][0]
        climb = kinds["tilt"] + kinds["drive_load"] + kinds["brush_jam"] + kinds["cliff"] + kinds["drop"]
        if climb >= len(items) / 3:
            what = "robot na coś najeżdża (próg, dywan, kabel, przedmiot pod robotem)"
        elif wall_share >= 0.6:
            what = "uderzenia w ścianę/mebel z mapy (zwykle normalne przy sprzątaniu krawędzi)"
        else:
            what = "przeszkoda, której nie ma na mapie (np. nogi krzesła, przedmiot na podłodze)"
        pad = 20
        out.append({"x": round(sum(xs) / len(xs)), "y": round(sum(ys) / len(ys)), "n": len(items), "days": days,
                    "kinds": {HOT_KINDS[k]: n for k, n in kinds.most_common()}, "main": HOT_KINDS[top],
                    "near_wall": round(wall_share, 2), "what": what,
                    "first": min(e["t"] for e, _, _ in items), "last": max(e["t"] for e, _, _ in items),
                    "zone": [round(min(xs)) - pad, round(min(ys)) - pad, round(max(xs)) + pad, round(max(ys)) + pad],
                    "score": round(len(items) * (1 - 0.7 * wall_share) * (1 + 0.5 * climb / len(items)), 1)})
    out.sort(key=lambda h: -h["score"])
    for i, h in enumerate(out):
        h["id"] = i + 1
    return out


if __name__ == "__main__":
    from collections import Counter
    from mock_robot import build_map, OFF, PS

    m = build_map()
    walls, walk, rooms = map_sets(m)
    assert len(rooms) == 2 and walls and not walls & walk

    route = plan_waypoints(m, ((OFF + 5) * PS, (OFF + 40) * PS))
    names = [r[2] for r in route]
    assert set(names) == {"Salon", "Kuchnia"}, names
    for x, y, _ in route:  # every stop inside the flat and >= 30 cm from its edge
        assert (x // PS, y // PS) in walk
    assert len(route) == 4, route  # Salon 24 m2 -> 3 stops, Kuchnia 9 m2 -> 1

    near = dilate(walls, 2)
    wall_cell = next(iter(walls))
    inside = ((OFF + 60) * PS // 5, (OFF + 40) * PS // 5)
    outside = ((OFF - 40) * PS // 5, (OFF - 40) * PS // 5)
    cells = {(wall_cell[0] * PS // 5, wall_cell[1] * PS // 5): 5, inside: 9, outside: 4}
    cls = {(x // 5, y // 5): c for x, y, n, c in classify_cloud(cells, 5, PS, near, walk)}
    assert cls[inside] == OBSTACLE and cls[outside] == PHANTOM and WALL in cls.values()

    blob = [[x * 5, y * 5, 5, OBSTACLE] for x in range(100, 104) for y in range(100, 103)]  # 20x15 cm
    big = [[x * 5, y * 5, 5, OBSTACLE] for x in range(200, 216) for y in range(200, 203)]  # 80 cm long
    kinds = sorted(c["kind"] for c in obstacle_clusters(blob + big, 5))
    assert kinds == ["drobna", "duża"], kinds

    wx, wy = (OFF + 60) * PS, (OFF + 40) * PS  # middle of Salon, looking left (180 deg) -> left wall
    assert expected_wall(walls, PS, wx, wy, 180) is not None
    g = glass_candidates(Counter({(1, 1): 2}), Counter({(1, 1): 10, (2, 2): 3}), PS)
    assert len(g) == 1 and g[0][2] == 0.83

    # structural walls vs islands: the mock flat's outline is structural, a free-standing box is an island
    box = {(OFF + 50 + i, OFF + 40 + j) for i in range(6) for j in range(6) if i in (0, 5) or j in (0, 5)}
    structural, islands = wall_parts(walls | box, walk - {(OFF + 50 + i, OFF + 40 + j) for i in range(6) for j in range(6)})
    assert structural == walls and len(islands) == 1 and islands[0] == box

    # a ray from the middle of Salon to an "echo" 2 m beyond the right wall of the flat crosses that wall
    rx = (OFF + 180) * PS                       # right outer wall of Kuchnia (x = OFF+180)
    w = ray_wall(walls, PS, (OFF + 150) * PS, (OFF + 50) * PS, 0.0, 30 * PS + 200)
    assert w is not None and w[0] == OFF + 180, w
    assert ray_wall(walls, PS, (OFF + 150) * PS, (OFF + 50) * PS, 0.0, 30 * PS) is None  # echo on the wall itself

    # mirror on that wall, y = OFF+40..OFF+60: echoes behind it are the mirror image of real wall points
    near_all = dilate(walls, 1)
    seg_px = [(OFF + 180, OFF + 40 + k) for k in range(20)]
    through = Counter({p: 12 for p in seg_px})
    real = [((OFF - 1) * PS + 2, (OFF + 10 + k) * PS) for k in range(0, 56, 4)]     # Salon's left outer wall
    wall_x = rx + PS / 2
    mirrored = [(2 * wall_x - x, y) for x, y in real]                                # virtual image behind the mirror
    far = {seg_px[i % 20]: [] for i in range(20)}
    for i, f in enumerate(mirrored):
        far[seg_px[i % 20]].append(f)
    segs = reflective_segments(through, far, Counter(), Counter(), near_all, PS)
    assert len(segs) == 1 and segs[0]["kind"] == "lustro" and segs[0]["score"] >= 0.8, segs
    assert abs(segs[0]["x1"] - wall_x) <= 3 and abs(abs(segs[0]["y2"] - segs[0]["y1"]) - 19 * PS) <= 6, segs

    # same wall stretch, but the echoes behind it are just a garden (no mirror symmetry) -> glass / opening
    garden = {p: [(rx + 150 + 7 * i, (OFF + 45) * PS + 11 * i) for i in range(3)] for p in seg_px}
    segs = reflective_segments(through, garden, Counter(), Counter(), near_all, PS)
    assert segs[0]["kind"] == "drzwi?" and segs[0]["score"] < 0.2, segs      # 100 cm wide -> door-sized
    assert segs[0]["beyond"][0] > rx + 80, segs                            # look-behind target is outside, past the wall
    wide = Counter({(OFF + 180, OFF + 22 + k): 12 for k in range(40)})        # 2 m of it -> window / glass wall
    segs = reflective_segments(wide, {p: garden[seg_px[0]] for p in wide}, Counter(), Counter(), near_all, PS)
    assert segs[0]["kind"] == "szyba/otwór", segs
    solid = Counter({p: 40 for p in seg_px})                                  # mostly seen as a wall -> not transparent
    assert reflective_segments(through, garden, solid, Counter(), near_all, PS) == []

    # a stretch that mostly returns nothing
    dark_px = [(OFF + 60 + k, OFF - 1) for k in range(10)]
    segs = reflective_segments(Counter(), {}, Counter({p: 2 for p in dark_px}), Counter({p: 60 for p in dark_px}), near_all, PS)
    assert [s["kind"] for s in segs] == ["brak odbicia"]

    stats = wifi_by_room([{"x": (OFF + 10) * PS, "y": (OFF + 10) * PS, "signal": -50},
                          {"x": (OFF + 20) * PS, "y": (OFF + 10) * PS, "signal": -60},
                          {"x": (OFF + 150) * PS, "y": (OFF + 40) * PS, "signal": -80}], rooms, PS)
    assert [(r["name"], r["avg"], r["n"]) for r in stats] == [("Kuchnia", -80, 1), ("Salon", -55, 2)]

    # the mock flat is open-plan: Salon and Kuchnia touch along x = OFF+120, y = OFF+20..OFF+79 (3 m) -> a passage
    d = doorways(rooms, PS)
    assert len(d) == 1 and d[0]["kind"] == "przejście" and abs(d[0]["width"] - 300) <= 10, d
    assert set(d[0]["rooms"]) == {"Salon", "Kuchnia"} and abs(d[0]["x1"] - (OFF + 119.5) * PS) <= 5, d
    # shrink the contact to 80 cm -> a door
    a = (rooms[0][0], rooms[0][1], {q for q in rooms[0][2] if q[0] < OFF + 119 or OFF + 40 <= q[1] < OFF + 56})
    d = doorways([a, rooms[1]], PS)
    assert len(d) == 1 and d[0]["kind"] == "drzwi" and abs(d[0]["width"] - 80) <= 6, d

    # door state: an 80 cm door = 16 positions; echoes on the line -> shut, later rays through it -> open again
    door = {"x1": 1000, "y1": 500, "x2": 1000, "y2": 580}
    cells = door_cells([door], 5)
    assert door["nk"] == 16 and cells[(200, 100)] == (0, 0) and cells[(198, 115)] == (0, 15) and (203, 100) not in cells
    hit = {(0, k): 100.0 for k in range(16)}
    assert door_state(0, 16, hit, {}, 105) == "zamknięte" and door_state(0, 16, hit, {}, 500) is None
    assert door_state(0, 16, hit, {(0, k): 110.0 for k in range(4, 16)}, 112) == "otwarte"   # a leaf swung open
    assert door_state(0, 16, {(0, 1): 100.0}, {}, 101) is None                             # barely seen: no verdict
    assert seg_dist(1010, 540, door) == 10 and seg_dist(1000, 590, door) == 10

    # shut door: floor 40x30 px split by a 4 px thick wall (rows 13-16) with a 1 px thin stretch at x 10..25 (80 cm)
    fl = {(x, y) for x in range(40) for y in range(30)}
    wl = {(x, y) for x in range(40) for y in range(13, 17) if not (10 <= x < 26 and y != 14)}
    sd = shut_doors(wl, fl - wl, (20 * PS, 5 * PS), PS)
    assert len(sd) == 1 and abs(math.hypot(sd[0]["x2"] - sd[0]["x1"], sd[0]["y2"] - sd[0]["y1"]) + PS - 80) <= 6, sd
    assert abs(sd[0]["y1"] - 14.5 * PS) <= 1, sd
    assert shut_doors(wl - {(x, 14) for x in range(10, 26)}, fl - wl, (100, 25), PS) == []  # open: nothing to try
    leak = {(35, y) for y in range(13, 17)}  # + a 5 cm slit elsewhere (map noise): the robot can't use it, door still found
    assert len(shut_doors(wl - leak, (fl - wl) | leak, (20 * PS, 5 * PS), PS)) == 1
    assert shut_doors(wl, fl - wl, (20 * PS, 5 * PS), PS, start=(30 * PS, 8 * PS)) == []  # same side: nothing in between

    # hotspots: 4 hits on a chair leg in the open (heading east -> contact 17 cm east), 3 tilts at a threshold,
    # 3 hits at a wall, 1 lone hit (ignored)
    wl = {(40, y) for y in range(0, 60)}  # wall at x = 200 cm
    T = 1794200000
    ev = [{"k": "bumper", "x": 100, "y": 100, "a": 90, "t": T + i} for i in range(4)]
    ev += [{"k": "tilt", "x": 300, "y": 50, "t": T + 86400 * i} for i in range(3)]
    ev += [{"k": "bumper", "x": 183, "y": 150 + i, "a": 90, "t": T} for i in range(3)]
    ev += [{"k": "bumper", "x": 50, "y": 250, "t": T}, {"k": "status", "x": 1, "y": 1, "t": T}]
    hs = hotspots(ev, wl, PS)
    assert len(hs) == 3, hs
    leg = next(h for h in hs if h["n"] == 4)
    assert leg["x"] == 117 and "nie ma na mapie" in leg["what"] and leg["zone"] == [97, 80, 137, 120], leg
    assert next(h for h in hs if h["main"] == "najazd / przechył")["days"] == 3
    assert "ścianę" in next(h for h in hs if h["near_wall"] > 0.5)["what"]
    assert hs[-1]["near_wall"] > 0.5  # wall hits (normal edge cleaning) rank last
    print("analysis: ok")
