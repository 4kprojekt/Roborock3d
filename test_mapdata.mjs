// Run: node test_mapdata.mjs
import assert from "node:assert/strict";
import { decodeLayer, mapBounds, signalRGB, signalColor, segmentColor, wallGrid, floorGrid, raycast, idwGrid,
  splitWallRuns } from "./static/mapdata.js";

// RLE example straight from Valetudo's MapLayer.js: [37,5,3] = (37,5),(38,5),(39,5)
assert.deepEqual(decodeLayer({ compressedPixels: [37, 5, 3, 10, 6, 1] }), [[37, 5, 3], [10, 6, 1]]);
assert.deepEqual(decodeLayer({ pixels: [17, 24, 18, 25], compressedPixels: [] }), [[17, 24, 1], [18, 25, 1]]);
assert.deepEqual(decodeLayer({}), []);

assert.deepEqual(
  mapBounds({ layers: [{ compressedPixels: [37, 5, 3] }, { pixels: [10, 9] }] }),
  { minX: 10, maxX: 39, minY: 5, maxY: 9 });
assert.equal(mapBounds({ layers: [] }), null);

assert.deepEqual(signalRGB(-95), [214, 51, 74]);   // clamped to the red end
assert.deepEqual(signalRGB(-52), [123, 224, 74]);
assert.deepEqual(signalRGB(-78), [240, 138, 36]);  // exactly on a stop
assert.equal(signalColor(-40, 0.5), "rgba(123,224,74,0.5)");
assert.equal(segmentColor("3"), segmentColor(3));

// vertical wall at pixel x=20 (y 0..40), origin at pixel (10,20), 5 cm/px
const wallMap = { layers: [
  { type: "wall", compressedPixels: Array.from({ length: 41 }, (_, y) => [20, y, 1]).flat() },
  { type: "floor", compressedPixels: Array.from({ length: 41 }, (_, y) => [0, y, 20]).flat() }] };
const grid = wallGrid(wallMap);
assert.equal(grid.data[20 * grid.w + 20], 1);
assert.equal(grid.data[20 * grid.w + 19], 0);
const scan = raycast(grid, 5, [10 * 5 + 2.5, 20 * 5 + 2.5], 4);
assert.equal(scan[0].hit, true);                        // 0° = +x -> hits the wall
assert.ok(Math.abs(scan[0].dist - 47.5) <= 2.5, scan[0].dist); // 9.5 px * 5 cm
assert.equal(scan[2].hit, false);                       // 180° -> leaves the map, no wall

// heatmap: strong signal on the left (x = 5 cm), weak on the right (x = 95 cm) of a 100 x 205 cm floor
const floor = floorGrid(wallMap);
assert.equal(floor.data[0], 1);
assert.equal(floor.data[20], 0); // the wall column is not floor
const hm = idwGrid([[0, 100, -40, 5], [75, 100, -80, 5]], 25, floor, 5, 2, 80);
const at = (xcm, ycm) => hm.values[Math.floor(ycm / 5 / hm.step) * hm.w + Math.floor(xcm / 5 / hm.step)];
assert.ok(at(12, 110) > -50 && at(88, 110) < -70, `${at(12, 110)} ${at(88, 110)}`);
assert.ok(at(50, 110) < at(12, 110) && at(50, 110) > at(88, 110));  // smooth in between
assert.ok(Number.isNaN(at(102, 110)));                               // on the wall: nothing
assert.ok(Number.isNaN(at(12, 2)), "more than `reach` away from any measurement -> no guess");

// run [18..22] on row 3 where pixels 20,21 are furniture
const parts = splitWallRuns({ layers: [{ type: "wall", compressedPixels: [18, 3, 5] }] }, [[20, 3], [21, 3]]);
assert.deepEqual(parts, { structural: [[18, 3, 2], [22, 3, 1]], islands: [[20, 3, 2]] });
{ const o = splitWallRuns({ layers: [{ type: "wall", compressedPixels: [18, 3, 5] }] }, [[20, 3]], [[19, 3]]);
  assert.deepEqual(o, { structural: [[18, 3, 1], [21, 3, 2]], islands: [[20, 3, 1]] }); } // 19,3 open -> not drawn

console.log("mapdata: ok");
