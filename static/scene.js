// One three.js scene for every view (2D top-down, 3D, ISO, orbit, FPV) and every layer.
// Units: cm. Map x -> three x, map y (down) -> three z, height -> three y. Origin = centre of the first map.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { EVENT_STYLE } from "./history.js";
import { decodeLayer, mapBounds, floorGrid, wifiGrid, signalRGB, segmentColor, batteryColor, splitWallRuns } from "./mapdata.js";
const WIFI_CONTOUR_DB = 6;   // heatmap contour lines every 6 dBm

const ACC = "#C8FF3D", CYAN = "#38E1F0";
const HEADING_OFFSET_DEG = -90; // driving direction = Valetudo angle - 90 deg (checked against a recorded run)
const ISLAND_H = 35;            // furniture / free-standing things from the map: drawn low, the lidar can't see height
const SMALL_H = 10;             // small obstacles the lidar sees right now (cables, toys, chair legs)
const EYE_H = 14;               // FPV camera height: roughly the lidar turret
const EVENT_COLOR = { bumper: "#F08A24", lds_bumper: "#F08A24", cliff: "#D6334A", drop: "#D6334A", carpet: CYAN,
  brush_jam: "#D6334A", drive_load: "#F2D23C", tilt: "#B48CFF", mcu_warn: "#F2D23C", mcu_error: "#D6334A", hotspot: "#FF5A1F" };
const REFLECT_COLOR = { "lustro": "#B48CFF", "szyba/otwór": "#5DD8FF", "brak odbicia": "#F2D23C", "drzwi?": "#7CF29A", "drzwi": "#7CF29A", "przejście": "#4FB870" };
const SMOOTH_S = 0.3; // live pose arrives every 0.5 s: glide towards it with this time constant

// pose between two poses (angle the short way round) - used for missing frames in replay and live smoothing
export function mixPose(a, b, f) {
  const da = ((b.angle - a.angle + 540) % 360) - 180;
  return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, angle: (a.angle + da * f + 360) % 360 };
}

export function createScene(el, overlay) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(2, devicePixelRatio));
  el.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#090C0F");
  scene.add(new THREE.HemisphereLight(0xffffff, 0x25303a, 1.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(300, 900, 500);
  scene.add(sun);

  const persp = new THREE.PerspectiveCamera(48, 1, 2, 60000);
  const ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 60000);
  const ctlP = new OrbitControls(persp, renderer.domElement), ctlO = new OrbitControls(ortho, renderer.domElement);
  for (const c of [ctlP, ctlO]) { c.enableDamping = true; c.maxPolarAngle = Math.PI / 2 - 0.02; }
  let camera = persp, view = "3d", radius = 800;

  const G = {};
  for (const k of ["floor", "walls", "obst", "wifi", "cloud", "phantom", "scan", "path", "mirrors", "photos", "events", "route", "replay", "misc", "hot"]) {
    G[k] = new THREE.Group(); scene.add(G[k]);
  }
  const box = new THREE.BoxGeometry(1, 1, 1);
  const S = { O: null, map: null, ps: 5, bounds: null, wallH: 120, sig: "", islands: [], layers: {}, labels: new Map(),
    robot: null, replayPose: null, wifi: null, cloud: null, pickCb: null };

  const P = (x, y, h = 0) => new THREE.Vector3(x - S.O.x, h, y - S.O.y);
  const clear = g => { for (const o of [...g.children]) { g.remove(o); o.traverse(n => { n.material?.map?.dispose(); n.material?.dispose(); if (n.geometry && n.geometry !== box) n.geometry.dispose(); }); } };

  function boxes(items, color, opts = {}) { // items: [x, y, z, sx, sy, sz, cssColor?] -> one InstancedMesh
    const mesh = new THREE.InstancedMesh(box, new THREE.MeshStandardMaterial({ color, roughness: 0.85, ...opts }), items.length);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), p = new THREE.Vector3(), s = new THREE.Vector3();
    items.forEach((it, i) => {
      m.compose(p.set(it[0], it[1], it[2]), q, s.set(it[3], it[4], it[5]));
      mesh.setMatrixAt(i, m);
      if (it[6]) mesh.setColorAt(i, new THREE.Color(it[6]));
    });
    return mesh;
  }

  function mapPlane(canvas, y, filter, opacity = 1) { // a canvas stretched over the map bounds, lying on the floor
    const b = S.bounds, ps = S.ps, w = (b.maxX - b.minX + 1) * ps, h = (b.maxY - b.minY + 1) * ps;
    const tex = new THREE.CanvasTexture(canvas);
    tex.magFilter = tex.minFilter = filter; tex.generateMipmaps = false; tex.colorSpace = THREE.SRGBColorSpace;
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(w, h),
      new THREE.MeshBasicMaterial({ map: tex, transparent: true, opacity, depthWrite: false }));
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.copy(P(b.minX * ps + w / 2, b.minY * ps + h / 2, y));
    return mesh;
  }

  // ---------- robot ----------
  const robot = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(17.5, 17.5, 9.6, 8), new THREE.MeshStandardMaterial({ color: ACC, roughness: 0.6 }));
  body.position.y = 4.8;
  const turret = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 2.4, 16), new THREE.MeshStandardMaterial({ color: "#0A0D10" }));
  turret.position.y = 10.8;
  const nose = new THREE.Mesh(new THREE.BoxGeometry(5, 3, 5), new THREE.MeshStandardMaterial({ color: "#0A0D10" }));
  nose.position.set(13, 8.5, 0); // +x = forward
  const pulse = new THREE.Mesh(new THREE.RingGeometry(17, 19, 32), new THREE.MeshBasicMaterial({ color: ACC, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
  pulse.rotation.x = -Math.PI / 2; pulse.position.y = 1;
  robot.add(body, turret, nose, pulse);
  robot.visible = false;
  scene.add(robot);

  // ---------- live scan (reused buffers) ----------
  const scanPts = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: CYAN, size: 4, sizeAttenuation: false }));
  const scanRays = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: CYAN, transparent: true, opacity: 0.13 }));
  scanPts.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(360 * 3), 3));
  scanRays.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(360 * 6), 3));
  scanPts.frustumCulled = scanRays.frustumCulled = false;
  G.scan.add(scanPts, scanRays);

  function setScan(points, origin) {
    const n = points && origin && S.O ? Math.min(360, points.length) : 0;
    const a = scanPts.geometry.attributes.position, r = scanRays.geometry.attributes.position;
    for (let i = 0; i < n; i++) {
      const x = points[i][0] - S.O.x, z = points[i][1] - S.O.y;
      a.setXYZ(i, x, 9, z);
      r.setXYZ(i * 2, origin.x - S.O.x, 9, origin.y - S.O.y); r.setXYZ(i * 2 + 1, x, 9, z);
    }
    a.needsUpdate = r.needsUpdate = true;
    scanPts.geometry.setDrawRange(0, n); scanRays.geometry.setDrawRange(0, n * 2);
  }

  // ---------- map ----------
  function setMap(map, islandPx) {
    const b = mapBounds(map);
    if (!b) return;
    const ps = map.pixelSize, first = !S.O;
    S.map = map; S.ps = ps; S.bounds = b; S.islands = islandPx ?? S.islands;
    if (first) {
      S.O = { x: (b.minX + b.maxX + 1) / 2 * ps, y: (b.minY + b.maxY + 1) / 2 * ps };
      radius = Math.max(b.maxX - b.minX, b.maxY - b.minY) * ps / 2;
    }
    const sig = map.layers.map(l => l.type + (l.compressedPixels?.length ?? 0) + (l.metaData?.name ?? "")).join("|") + "#" + S.islands.length + "#" + S.wallH + "#" + (S.openKey ?? "");
    if (sig !== S.sig) { S.sig = sig; buildStatic(map, b, ps); }
    buildEntities(map);
    if (first) setView(view);
    if (S.wifi) setWifi(S.wifi);
  }

  function buildStatic(map, b, ps) {
    clear(G.floor); clear(G.walls);
    const w = b.maxX - b.minX + 1, h = b.maxY - b.minY + 1, k = Math.max(1, Math.min(4, Math.floor(2048 / Math.max(w, h))));
    const cv = document.createElement("canvas"); cv.width = w * k; cv.height = h * k;
    const ctx = cv.getContext("2d");
    for (const layer of map.layers) {
      if (layer.type === "wall") continue;
      ctx.fillStyle = "#121820";
      for (const [x, y, n] of decodeLayer(layer)) ctx.fillRect((x - b.minX) * k, (y - b.minY) * k, n * k, k);
      if (layer.type === "segment") { // faint room tint so rooms can be told apart
        ctx.fillStyle = segmentColor(layer.metaData?.segmentId) + "26";
        for (const [x, y, n] of decodeLayer(layer)) ctx.fillRect((x - b.minX) * k, (y - b.minY) * k, n * k, k);
      }
    }
    ctx.globalCompositeOperation = "source-atop"; // 0.5 m grid, only where there is floor
    ctx.fillStyle = "#1F2A34";
    const g = 50 / ps;
    for (let x = Math.ceil(b.minX / g) * g; x <= b.maxX; x += g) ctx.fillRect((x - b.minX) * k, 0, 1, cv.height);
    for (let y = Math.ceil(b.minY / g) * g; y <= b.maxY; y += g) ctx.fillRect(0, (y - b.minY) * k, cv.width, 1);
    G.floor.add(mapPlane(cv, 0, THREE.NearestFilter));

    const parts = splitWallRuns(map, S.islands, S.open ?? []);
    const run = H => ([x, y, n]) => [(x + n / 2) * ps - S.O.x, H / 2, (y + 0.5) * ps - S.O.y, n * ps, H, ps];
    if (parts.structural.length) G.walls.add(boxes(parts.structural.map(run(S.wallH)), "#BFC8D0"));
    clear(G.obst);
    if (parts.islands.length) { const m = boxes(parts.islands.map(run(ISLAND_H)), "#56626E"); m.name = "islands"; G.obst.add(m); }
    if (S.cloud) drawObstacles(S.cloud);
    S.rooms = map.layers.filter(l => l.type === "segment").map(l => {
      const runs = decodeLayer(l), n = runs.reduce((a, r) => a + r[2], 0) || 1;
      return { id: String(l.metaData?.segmentId), name: l.metaData?.name || "Pokój " + l.metaData?.segmentId,
        area: n * ps * ps / 1e4,
        x: runs.reduce((a, r) => a + (r[0] + r[2] / 2) * r[2], 0) / n * ps, y: runs.reduce((a, r) => a + (r[1] + 0.5) * r[2], 0) / n * ps };
    });
    refreshLabels();
  }

  function buildEntities(map) {
    clear(G.path); clear(G.misc);
    for (const e of map.entities ?? []) {
      const p = e.points ?? [];
      const line = (color, h, close) => {
        const pts = [];
        for (let i = 0; i < p.length; i += 2) pts.push(P(p[i], p[i + 1], h));
        if (close && pts.length) pts.push(pts[0].clone());
        return new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color }));
      };
      if (e.type === "path") G.path.add(line(ACC, 3));
      else if (e.type === "predicted_path") G.path.add(line("#6B7682", 3));
      else if (e.type === "virtual_wall") G.misc.add(line("#D6334A", 40));
      else if (e.type === "no_go_area" || e.type === "no_mop_area") G.misc.add(line("#D6334A", 4, true));
      else if (e.type === "active_zone") G.misc.add(line(CYAN, 4, true));
      else if (e.type === "charger_location") {
        const dock = new THREE.Mesh(new THREE.BoxGeometry(16, 12, 30), new THREE.MeshStandardMaterial({ color: "#2A333D" }));
        dock.position.copy(P(p[0], p[1], 6));
        const edge = new THREE.LineSegments(new THREE.EdgesGeometry(dock.geometry), new THREE.LineBasicMaterial({ color: ACC }));
        dock.add(edge);
        G.misc.add(dock);
      }
    }
  }

  // ---------- Wi-Fi heatmap ----------
  function setWifi(report) {
    S.wifi = report;
    clear(G.wifi);
    if (!S.map || !report?.cells?.length) return;
    const floor = floorGrid(S.map), hm = wifiGrid(report.cells, report.cell, floor, S.ps);
    const cv = document.createElement("canvas"); cv.width = hm.w; cv.height = hm.h;
    const ctx = cv.getContext("2d"), img = ctx.createImageData(hm.w, hm.h);
    const band = i => Math.floor(hm.values[i] / WIFI_CONTOUR_DB);
    for (let i = 0; i < hm.values.length; i++) {
      if (Number.isNaN(hm.values[i])) continue;
      const [r, g, b] = signalRGB(hm.values[i]);
      // contour line where the signal crosses a multiple of WIFI_CONTOUR_DB (NaN neighbours compare false)
      const line = ((i + 1) % hm.w && band(i + 1) < band(i)) || band(i - 1) < band(i) || band(i + hm.w) < band(i) || band(i - hm.w) < band(i);
      img.data.set(line ? [r * 0.45, g * 0.45, b * 0.45, 230] : [r, g, b, 170], i * 4);
    }
    ctx.putImageData(img, 0, 0);
    // the canvas covers ceil(w/step)*step pixels; stretch it over exactly that area
    const b = S.bounds, save = S.bounds;
    S.bounds = { minX: b.minX, minY: b.minY, maxX: b.minX + hm.w * hm.step - 1, maxY: b.minY + hm.h * hm.step - 1 };
    G.wifi.add(mapPlane(cv, 1.5, THREE.LinearFilter));
    S.bounds = save;
  }

  // ---------- lidar cloud, obstacles, mirrors ----------
  function points(list, colorOf, y, size, opacity = 1) {
    const pos = new Float32Array(list.length * 3), col = new Float32Array(list.length * 3), c = new THREE.Color();
    list.forEach((p, i) => { pos.set([p[0] - S.O.x, y, p[1] - S.O.y], i * 3); c.set(colorOf(p)); col.set([c.r, c.g, c.b], i * 3); });
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    g.setAttribute("color", new THREE.BufferAttribute(col, 3));
    return new THREE.Points(g, new THREE.PointsMaterial({ size, sizeAttenuation: false, vertexColors: true, transparent: opacity < 1, opacity, depthWrite: opacity === 1 }));
  }

  function drawObstacles(c) {
    for (const o of [...G.obst.children]) if (o.name !== "islands") { G.obst.remove(o); o.material.dispose(); }
    const small = c.obstacles.filter(o => o.kind === "drobna").map(o =>
      [o.x + o.w / 2 - S.O.x, SMALL_H / 2, o.y + o.h / 2 - S.O.y, Math.max(o.w, 6), SMALL_H, Math.max(o.h, 6)]);
    if (small.length) G.obst.add(boxes(small, "#F08A24"));
  }

  function setCloud(c) {
    S.cloud = c;
    if (!S.O) return;
    clear(G.cloud); clear(G.phantom); clear(G.mirrors);
    const walls = c.cells.filter(p => p[3] === 0), obst = c.cells.filter(p => p[3] === 1), out = c.cells.filter(p => p[3] === 2);
    if (walls.length) {
      [[9, 1], [0.33, 0.6], [0.66, 0.4], [1, 0.25]].forEach(([f, op], i) => // the design's stacked scan slices
        G.cloud.add(points(walls, () => CYAN, i ? f * S.wallH : f, i ? 2 : 3, op)));
    }
    if (obst.length) G.cloud.add(points(obst, () => "#F08A24", 9, 3));
    const ph = out.concat(c.phantom);
    if (ph.length) G.phantom.add(points(ph, () => "#E85DFF", 9, 2.5, 0.8));
    drawObstacles(c);
    for (const s of c.reflective) {
      const a = P(s.x1, s.y1), b = P(s.x2, s.y2), len = Math.max(10, a.distanceTo(b));
      const m = new THREE.Mesh(new THREE.PlaneGeometry(len, S.wallH + 6),
        new THREE.MeshBasicMaterial({ color: REFLECT_COLOR[s.kind] ?? "#fff", transparent: true, opacity: 0.45, side: THREE.DoubleSide, depthWrite: false }));
      m.position.set((a.x + b.x) / 2, (S.wallH + 6) / 2, (a.z + b.z) / 2);
      m.rotation.y = -Math.atan2(b.z - a.z, b.x - a.x);
      G.mirrors.add(m);
    }
    for (const d of c.doors ?? []) { // doorways: a strip on the floor
      const a = P(d.x1, d.y1), b = P(d.x2, d.y2);
      const m = new THREE.Mesh(new THREE.BoxGeometry(Math.max(10, a.distanceTo(b)), 1.5, 6), new THREE.MeshBasicMaterial({ color: d.state === "zamknięte" ? "#D6334A" : REFLECT_COLOR[d.kind] }));
      m.position.set((a.x + b.x) / 2, 1.5, (a.z + b.z) / 2);
      m.rotation.y = -Math.atan2(b.z - a.z, b.x - a.x);
      G.mirrors.add(m);
    }
    refreshLabels();
  }

  // ---------- events, route, photos, replay ----------
  function setEvents(list) {
    clear(G.events);
    const ev = (list ?? []).filter(e => e.x != null && EVENT_COLOR[e.k]);
    S.evShown = ev.slice(-40); // newest 40 get a text label (when the "Etykiety zdarzeń" layer is on)
    refreshLabels();
    if (!ev.length || !S.O) return;
    const geo = new THREE.OctahedronGeometry(5);
    const mesh = new THREE.InstancedMesh(geo, new THREE.MeshBasicMaterial(), ev.length);
    const m = new THREE.Matrix4();
    ev.forEach((e, i) => { mesh.setMatrixAt(i, m.makeTranslation(e.x - S.O.x, 16, e.y - S.O.y)); mesh.setColorAt(i, new THREE.Color(EVENT_COLOR[e.k])); });
    G.events.add(mesh);
  }

  function setHotspots(list) { // problem places: a ring sized like the proposed no-go zone
    clear(G.hot);
    S.hot = list ?? [];
    refreshLabels();
    if (!S.O) return;
    for (const h of S.hot) {
      const r = Math.max(15, (h.zone[2] - h.zone[0]) / 2), col = h.near_wall >= 0.6 ? "#A3AEB9" : "#F08A24";
      const m = new THREE.Mesh(new THREE.RingGeometry(r - 3, r, 40), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false }));
      m.rotation.x = -Math.PI / 2; m.position.copy(P(h.x, h.y, 2));
      const fill = new THREE.Mesh(new THREE.CircleGeometry(r - 3, 40), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: Math.min(0.45, 0.08 + h.n / 150), side: THREE.DoubleSide, depthWrite: false }));
      fill.rotation.x = -Math.PI / 2; fill.position.copy(P(h.x, h.y, 1.5));
      G.hot.add(m, fill);
    }
  }

  function setMarks(pts) { S.marks = pts; refreshLabels(); } // points being picked (route plan, zone corners)

  function setRoute(route, i) {
    S.route = route?.length ? { route, i } : null;
    refreshLabels();
  }

  function setPhotos(list) {
    clear(G.photos);
    S.photos = list ?? [];
    if (!S.O) return;
    const loader = new THREE.TextureLoader();
    for (const ph of S.photos) {
      const a = P(ph.x1, ph.y1), b = P(ph.x2, ph.y2), w = Math.max(20, a.distanceTo(b)), h = w * ph.aspect;
      const tex = loader.load("/local/photos/" + encodeURIComponent(ph.file));
      tex.colorSpace = THREE.SRGBColorSpace;
      const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }));
      m.position.set((a.x + b.x) / 2, h / 2, (a.z + b.z) / 2);
      m.rotation.y = -Math.atan2(b.z - a.z, b.x - a.x);
      G.photos.add(m);
    }
  }

  function setReplay(rows, idx, events) {
    if (!rows) { clear(G.replay); S.replayRows = null; S.replayPose = null; S.replayIdx = -1; G.path.visible = !!S.layers.path; setEvents(S.liveEvents); return; }
    if (S.replayRows !== rows) { // build once per session, then just move the draw range
      clear(G.replay);
      S.replayRows = rows; S.replayIdx = -1;
      const pos = new Float32Array(rows.length * 3), col = new Float32Array(rows.length * 3), c = new THREE.Color();
      rows.forEach((r, i) => { pos.set([r.x - S.O.x, 4, r.y - S.O.y], i * 3); c.set(batteryColor(r.bat)); col.set([c.r, c.g, c.b], i * 3); });
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      g.setAttribute("color", new THREE.BufferAttribute(col, 3));
      const ghost = new THREE.Line(g.clone(), new THREE.LineBasicMaterial({ color: "#3A4552" }));
      const done = new THREE.Line(g, new THREE.LineBasicMaterial({ vertexColors: true }));
      done.name = "done"; done.frustumCulled = false;
      const scan = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: CYAN, size: 3.5, sizeAttenuation: false }));
      scan.geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(200 * 3), 3));
      scan.name = "scan"; scan.frustumCulled = false;
      G.replay.add(ghost, done, scan);
    }
    G.path.visible = false;
    const i0 = Math.floor(idx), r = rows[i0], n = rows[Math.min(rows.length - 1, i0 + 1)];
    S.replayPose = mixPose({ x: r.x, y: r.y, angle: r.a }, { x: n.x, y: n.y, angle: n.a }, idx - i0);
    if (S.replayIdx === i0) return;
    S.replayIdx = i0;
    G.replay.getObjectByName("done").geometry.setDrawRange(0, i0 + 1);
    const sc = G.replay.getObjectByName("scan"), a = sc.geometry.attributes.position, pts = r.scan ?? [];
    pts.slice(0, 200).forEach((p, i) => a.setXYZ(i, p[0] - S.O.x, 9, p[1] - S.O.y));
    a.needsUpdate = true; sc.geometry.setDrawRange(0, Math.min(200, pts.length));
    setEvents((events ?? []).filter(e => e.t <= r.t), true);
  }

  // ---------- HTML labels projected onto the scene ----------
  function refreshLabels() {
    const want = new Map();
    if (S.layers.labels) for (const r of S.rooms ?? []) want.set("room" + r.id, { x: r.x, y: r.y, h: 2, cls: "lab", html: `${r.name}<br><span>${r.area.toFixed(1)} m²</span>` });
    if (S.layers.mirrors) (S.cloud?.reflective ?? []).forEach((s, i) => want.set("refl" + i, { x: (s.x1 + s.x2) / 2, y: (s.y1 + s.y2) / 2, h: S.wallH + 12, cls: "lab refl",
      html: `${s.kind}${s.kind === "lustro" ? " " + Math.round(s.score * 100) + "%" : ""}`, color: REFLECT_COLOR[s.kind] }));
    if (S.layers.mirrors) (S.cloud?.doors ?? []).forEach((d, i) => want.set("door" + i, { x: (d.x1 + d.x2) / 2, y: (d.y1 + d.y2) / 2, h: 14, cls: "lab refl",
      html: `${d.kind} ${d.width} cm${d.state ? " · " + d.state : ""}`, color: d.state === "zamknięte" ? "#D6334A" : REFLECT_COLOR[d.kind] }));
    if (S.layers.evlabels && S.layers.events) (S.evShown ?? []).forEach((e, i) => want.set("ev" + i, { x: e.x, y: e.y, h: 24, cls: "lab ev",
      html: (EVENT_STYLE[e.k]?.[1] ?? e.k) + (e.side ? ` (${e.side})` : ""), color: EVENT_COLOR[e.k] }));
    if (S.layers.photos) for (const p of S.photos ?? []) if (p.name) want.set("ph" + p.id, { x: (p.x1 + p.x2) / 2, y: (p.y1 + p.y2) / 2, h: 6, cls: "lab", html: p.name });
    if (S.layers.hot) (S.hot ?? []).filter(h => h.near_wall < 0.6 || h.id <= 6).forEach(h => want.set("hot" + h.id, { x: h.x, y: h.y, h: 30, cls: "lab ev",
      html: `⚠ ${h.id}: ${h.n}× ${h.main}`, color: h.near_wall >= 0.6 ? "#A3AEB9" : "#F08A24" }));
    (S.marks ?? []).forEach(([x, y], i) => want.set("mk" + i, { x, y, h: 6, cls: "lab wp cur", html: String.fromCharCode(65 + (i % 26)) }));
    if (S.route) S.route.route.forEach(([x, y], i) => want.set("wp" + i, { x, y, h: 6, cls: "lab wp " + (i < S.route.i ? "done" : i === S.route.i ? "cur" : ""), html: String(i + 1) }));
    for (const [k, l] of S.labels) if (!want.has(k)) { l.el.remove(); S.labels.delete(k); }
    for (const [k, w] of want) {
      let l = S.labels.get(k);
      if (!l) { l = { el: document.createElement("div") }; overlay.appendChild(l.el); S.labels.set(k, l); }
      Object.assign(l, w);
      l.el.className = w.cls; l.el.innerHTML = w.html; l.el.style.borderColor = w.color ?? ""; l.el.style.color = w.color ?? "";
    }
  }

  const v3 = new THREE.Vector3();
  function placeLabels() {
    const w = el.clientWidth, h = el.clientHeight;
    if (!S.O) return; // labels can be requested (route, mirrors) before the first map arrives
    for (const l of S.labels.values()) {
      v3.copy(P(l.x, l.y, l.h)).project(camera);
      const ok = v3.z > -1 && v3.z < 1 && Math.abs(v3.x) < 1.05 && Math.abs(v3.y) < 1.05;
      l.el.style.display = ok ? "" : "none";
      if (ok) l.el.style.transform = `translate(-50%,-50%) translate(${((v3.x + 1) / 2 * w).toFixed(1)}px,${((1 - v3.y) / 2 * h).toFixed(1)}px)`;
    }
  }

  // ---------- views ----------
  function fitOrtho() {
    const aspect = el.clientWidth / Math.max(1, el.clientHeight), b = S.bounds, ps = S.ps;
    const hw = b ? (b.maxX - b.minX + 1) * ps / 2 : radius, hh = b ? (b.maxY - b.minY + 1) * ps / 2 : radius;
    const half = Math.max(hh, hw / aspect) * (view === "iso" ? 1.25 : 1.08);
    Object.assign(ortho, { left: -half * aspect, right: half * aspect, top: half, bottom: -half });
    ortho.updateProjectionMatrix();
  }

  function setView(v) {
    view = v;
    const sph = (tilt, az, r) => new THREE.Vector3(Math.sin(tilt) * Math.sin(az) * r, Math.cos(tilt) * r, Math.sin(tilt) * Math.cos(az) * r);
    const d = Math.PI / 180;
    ctlP.enabled = ctlO.enabled = false; ctlP.autoRotate = false;
    if (v === "2d") {
      camera = ortho; ortho.up.set(0, 0, -1); ortho.position.set(0, radius * 6, 0); ortho.zoom = 1;
      ctlO.target.set(0, 0, 0); ctlO.enableRotate = false; ctlO.screenSpacePanning = true; ctlO.enabled = true;
    } else if (v === "iso") {
      camera = ortho; ortho.up.set(0, 1, 0); ortho.position.copy(sph(62 * d, 45 * d, radius * 6)); ortho.zoom = 1;
      ctlO.target.set(0, 0, 0); ctlO.enableRotate = true; ctlO.screenSpacePanning = false; ctlO.enabled = true;
    } else if (v === "fpv") {
      camera = persp; persp.fov = 82; persp.near = 1;
    } else { // 3d, orbit
      camera = persp; persp.fov = 48; persp.near = 2; persp.position.copy(sph(50 * d, 34 * d, radius * 1.9));
      ctlP.target.set(0, 0, 0); ctlP.enabled = true; ctlP.autoRotate = v === "orbit"; ctlP.autoRotateSpeed = 1.5;
    }
    resize();
    (camera === ortho ? ctlO : ctlP).update();
  }

  function resize() {
    const w = el.clientWidth, h = el.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h);
    persp.aspect = w / h; persp.updateProjectionMatrix();
    fitOrtho();
  }
  new ResizeObserver(resize).observe(el);

  function setLayers(layers) {
    S.layers = { ...layers };
    for (const k of ["walls", "obst", "wifi", "cloud", "phantom", "scan", "mirrors", "photos", "events", "hot"]) G[k].visible = !!layers[k];
    G.path.visible = !!layers.path && !S.replayRows;
    refreshLabels();
  }

  function setWallHeight(cm) { S.wallH = cm; S.sig = ""; if (S.map) setMap(S.map); if (S.cloud) setCloud(S.cloud); }

  function setRobot(pose, active) { S.robot = pose; S.active = active; }

  function setFollow(on) { S.follow = on; if (!on) setView(view); } // off: back to the whole flat

  function setLiveEvents(list) { S.liveEvents = list; if (!S.replayRows) setEvents(list); }

  // click on the floor -> map cm
  const ray = new THREE.Raycaster(), ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  let down = null;
  renderer.domElement.addEventListener("pointerdown", e => { down = [e.clientX, e.clientY]; });
  renderer.domElement.addEventListener("pointerup", e => {
    if (!down || !S.pickCb || !S.O || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
    const r = renderer.domElement.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
    const hit = ray.ray.intersectPlane(ground, new THREE.Vector3());
    if (hit) S.pickCb(Math.round(hit.x + S.O.x), Math.round(hit.z + S.O.y));
  });

  const clock = new THREE.Clock();
  renderer.setAnimationLoop(() => {
    const dt = Math.min(0.1, clock.getDelta()), target = S.replayPose ?? S.robot;
    // replay poses are already interpolated; the live pose glides (a jump over 1.5 m = relocalisation -> snap)
    if (!target) S.shown = null;
    else if (S.replayPose || !S.shown || Math.hypot(target.x - S.shown.x, target.y - S.shown.y) > 150) S.shown = { ...target };
    else S.shown = mixPose(S.shown, target, 1 - Math.exp(-dt / SMOOTH_S));
    const pose = S.shown;
    if (pose && S.O) {
      const phi = (pose.angle + HEADING_OFFSET_DEG) * Math.PI / 180;
      robot.visible = view !== "fpv";
      robot.position.set(pose.x - S.O.x, 0, pose.y - S.O.y);
      robot.rotation.y = -phi;
      const t = (clock.getElapsedTime() % 1.8) / 1.8;
      pulse.visible = !!S.active && !S.replayPose;
      pulse.scale.setScalar(1 + 2.2 * t); pulse.material.opacity = 0.9 * (1 - t);
      if (view === "fpv") {
        const fx = Math.cos(phi), fz = Math.sin(phi);
        persp.position.set(pose.x - S.O.x + fx * 6, EYE_H, pose.y - S.O.y + fz * 6);
        persp.lookAt(pose.x - S.O.x + fx * 300, EYE_H, pose.y - S.O.y + fz * 300);
      }
    }
    if (S.follow && pose && S.O && view !== "fpv" && view !== "2d") {
      const ctl = camera === ortho ? ctlO : ctlP, want = new THREE.Vector3(pose.x - S.O.x, 0, pose.y - S.O.y);
      const step = want.sub(ctl.target).multiplyScalar(Math.min(1, dt * 4));
      ctl.target.add(step); camera.position.add(step);
    }
    if (view !== "fpv") (camera === ortho ? ctlO : ctlP).update();
    renderer.render(scene, camera);
    placeLabels();
  });

  return { setMap, setView, setLayers, setWallHeight, setRobot, setScan, setCloud, setWifi, setPhotos, setReplay, setRoute,
    setLiveEvents, setFollow, setHotspots, setMarks, onPick: cb => { S.pickCb = cb; }, rooms: () => S.rooms ?? [], islands: px => { S.islands = px; S.sig = ""; if (S.map) setMap(S.map); },
    openings: (px, key) => { S.open = px; S.openKey = key; if (S.map) setMap(S.map); } };
}
