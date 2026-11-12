// Pure helpers for the Valetudo map format (no DOM, no three.js) - tested by test_mapdata.mjs.
// Layers: pixel coords, RLE `compressedPixels` = [xStart, y, count, ...] (old maps: flat `pixels` = [x, y, ...]).
// Entities: points in cm. 1 map pixel = map.pixelSize cm.

export function decodeLayer(layer) {
  const runs = [], c = layer.compressedPixels, p = layer.pixels;
  if (c?.length) for (let i = 0; i < c.length; i += 3) runs.push([c[i], c[i + 1], c[i + 2]]);
  else if (p?.length) for (let i = 0; i < p.length; i += 2) runs.push([p[i], p[i + 1], 1]);
  return runs;
}

export function mapBounds(map) {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const layer of map.layers ?? []) {
    for (const [x, y, n] of decodeLayer(layer)) {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x + n - 1);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
  }
  return minX === Infinity ? null : { minX, maxX, minY, maxY };
}

// Occupancy grid (1 byte per map pixel over the map bounds) of the layers accepted by `pick(layer)`.
export function layerGrid(map, pick) {
  const b = mapBounds(map);
  if (!b) return null;
  const w = b.maxX - b.minX + 1, h = b.maxY - b.minY + 1, data = new Uint8Array(w * h);
  for (const layer of map.layers) {
    if (!pick(layer)) continue;
    for (const [x, y, n] of decodeLayer(layer)) data.fill(1, (y - b.minY) * w + x - b.minX, (y - b.minY) * w + x - b.minX + n);
  }
  return { ...b, w, h, data };
}
export const wallGrid = map => layerGrid(map, l => l.type === "wall");
export const floorGrid = map => layerGrid(map, l => l.type !== "wall");

// 360° "lidar scan" reconstructed from the map: rays from origin (cm) to the first wall pixel.
// Not the robot's raw scan - what the lidar would see from here according to the map (used while it is parked).
export function raycast(grid, ps, origin, rays = 360, maxCm = 800) {
  const out = [], ox = origin[0] / ps - grid.minX, oy = origin[1] / ps - grid.minY, maxT = maxCm / ps;
  for (let i = 0; i < rays; i++) {
    const a = i / rays * 2 * Math.PI, dx = Math.cos(a), dy = Math.sin(a);
    let hit = false, t = 0;
    for (; t < maxT; t += 0.5) {
      const x = Math.floor(ox + dx * t), y = Math.floor(oy + dy * t);
      if (x < 0 || y < 0 || x >= grid.w || y >= grid.h) break;
      if (grid.data[y * grid.w + x]) { hit = true; break; }
    }
    out.push({ angle: i * 360 / rays, dist: t * ps, hit,
      x: (ox + dx * t + grid.minX) * ps, y: (oy + dy * t + grid.minY) * ps });
  }
  return out;
}

// Smooth Wi-Fi heatmap: inverse-distance-weighted dBm on a coarse grid over the floor.
// cells = [[x_cm, y_cm, dBm, n], ...] (server's 25 cm averages), floor = floorGrid(map), step = map pixels per
// heatmap cell. A heatmap cell gets a value only on the floor and within `reach` cm of a measurement
// (no guessing about rooms the robot never drove through). -> { w, h, step, values: Float32Array (NaN = none) }
export function idwGrid(cells, cellCm, floor, ps, step = 2, reach = 120) {
  const w = Math.ceil(floor.w / step), h = Math.ceil(floor.h / step), values = new Float32Array(w * h).fill(NaN);
  const pts = cells.map(c => [c[0] + cellCm / 2, c[1] + cellCm / 2, c[2]]);
  const reach2 = reach * reach;
  for (let gy = 0; gy < h; gy++) {
    for (let gx = 0; gx < w; gx++) {
      const px = Math.min(floor.w - 1, gx * step + (step >> 1)), py = Math.min(floor.h - 1, gy * step + (step >> 1));
      if (!floor.data[py * floor.w + px]) continue;
      const x = (floor.minX + px + 0.5) * ps, y = (floor.minY + py + 0.5) * ps;
      let sum = 0, wsum = 0, near = Infinity;
      for (const [sx, sy, v] of pts) {
        const d2 = (sx - x) ** 2 + (sy - y) ** 2;
        if (d2 < near) near = d2;
        const wgt = 1 / (d2 + 400); // +20 cm^2 softening: no spikes at the sample itself
        sum += v * wgt; wsum += wgt;
      }
      if (near <= reach2) values[gy * w + gx] = sum / wsum;
    }
  }
  return { w, h, step, values };
}

// Design palette: -90 red, -78 orange, -66 yellow, -52 green.
const WIFI_STOPS = [[-90, [214, 51, 74]], [-78, [240, 138, 36]], [-66, [242, 210, 60]], [-52, [123, 224, 74]]];
export function signalRGB(dbm) {
  const c = Math.max(-90, Math.min(-52, dbm));
  let i = 0;
  while (i < WIFI_STOPS.length - 2 && c > WIFI_STOPS[i + 1][0]) i++;
  const [v0, c0] = WIFI_STOPS[i], [v1, c1] = WIFI_STOPS[i + 1], t = (c - v0) / (v1 - v0);
  return c0.map((x, k) => Math.round(x + (c1[k] - x) * t));
}
export function signalColor(dbm, alpha = 1) {
  return `rgba(${signalRGB(dbm).join(",")},${alpha})`;
}

export function batteryColor(pct, alpha = 1) {
  return `hsla(${Math.round(Math.max(0, Math.min(100, pct ?? 0)) * 1.2)}, 85%, 52%, ${alpha})`;
}

const PALETTE = ["#4e79a7", "#f28e2b", "#59a14f", "#b07aa1", "#76b7b2", "#edc948", "#ff9da7", "#9c755f"];
export function segmentColor(id) {
  const h = [...String(id ?? "")].reduce((a, ch) => a * 31 + ch.charCodeAt(0), 7);
  return PALETTE[Math.abs(h) % PALETTE.length];
}

// Wall runs split into structural walls and islands (furniture): islandPx = [[x, y], ...] from the server.
// wall runs split into structural / island (furniture); openPx (lidar sees through them: open door) are left out
export function splitWallRuns(map, islandPx, openPx = []) {
  const key = (x, y) => x + "," + y, isl = new Set(islandPx.map(p => key(...p))), open = new Set(openPx.map(p => key(...p)));
  const kind = (x, y) => (open.has(key(x, y)) ? 2 : isl.has(key(x, y)) ? 1 : 0), structural = [], islands = [];
  for (const layer of map.layers) {
    if (layer.type !== "wall") continue;
    for (const [x, y, n] of decodeLayer(layer)) {
      let start = x, cur = kind(x, y);
      for (let i = 1; i <= n; i++) {
        const k = i < n ? kind(x + i, y) : -1;
        if (k !== cur) {
          if (cur < 2) (cur ? islands : structural).push([start, y, x + i - start]);
          start = x + i; cur = k;
        }
      }
    }
  }
  return { structural, islands };
}
