// Run history (server: data/sessions/*.jsonl) and the canvas charts: battery sessions, 14 days, replay timeline, polar scan.
import { batteryColor } from "./mapdata.js";

const MONO = "10px 'JetBrains Mono', ui-monospace, monospace";
const WH_PER_PCT = 5200 * 14.4 / 1000 / 100; // S5 battery: 5200 mAh / 14.4 V = 74.9 Wh
export const EVENT_STYLE = {
  bumper: ["#F08A24", "Uderzenie zderzakiem"], lds_bumper: ["#F08A24", "Uderzenie wieżyczką lidaru"],
  cliff: ["#D6334A", "Czujnik krawędzi"], drop: ["#D6334A", "Koło opadło / podniesiony"],
  carpet: ["#38E1F0", "Większy opór szczotki (dywan?)"], brush_jam: ["#D6334A", "Szczotka hamowana (kabel?)"],
  drive_load: ["#F2D23C", "Zwiększony opór jazdy"], tilt: ["#B48CFF", "Przechył (próg / dywan)"],
  mcu_warn: ["#F2D23C", "Ostrzeżenie MCU"], mcu_error: ["#D6334A", "Błąd MCU"], wifi: ["#F08A24", "Słaby sygnał Wi-Fi"],
  error: ["#D6334A", "Błąd robota"], status: ["#A3AEB9", "Zmiana stanu"], session: ["#C8FF3D", "Przejazd"],
  door: ["#7CF29A", "Drzwi zmieniły stan"], explore: ["#C8FF3D", "Skanowanie mieszkania"],
  hotspot: ["#FF5A1F", "Miejsce problemowe"],
};
export const SERIES = { br: ["Prąd szczotki", "#38E1F0"], wh: ["Prąd kół", "#F2D23C"], tilt: ["Przechył °", "#B48CFF"],
  wifi: ["Wi-Fi dBm", "#7BE04A"], w: ["Pobór W", "#F08A24"] };

export const fmtDur = s => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, "0")}`;

export async function loadSession(id) {
  const text = await (await fetch("/local/sessions/" + encodeURIComponent(id))).text();
  const lines = text.split("\n").filter(Boolean).map(l => JSON.parse(l));
  return { meta: lines.find(l => l.meta)?.meta ?? { id }, rows: lines.filter(l => "t" in l), events: lines.filter(l => l.ev).map(l => l.ev) };
}

function setup(canvas) {
  const dpr = devicePixelRatio || 1, w = canvas.clientWidth, h = canvas.clientHeight;
  canvas.width = w * dpr; canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = MONO;
  return { ctx, w, h };
}

function axes(ctx, L, T, pw, ph, yTicks, Y, xTicks, X) {
  ctx.strokeStyle = "#1D242C"; ctx.fillStyle = "#8B96A3"; ctx.lineWidth = 1;
  for (const v of yTicks) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.textAlign = "right"; ctx.fillText(v, L - 6, Y(v) + 3); }
  ctx.textAlign = "center";
  for (const [v, label] of xTicks) ctx.fillText(label, X(v), T + ph + 14);
  ctx.textAlign = "left";
}

// Battery % vs minutes for the last runs; newest in the accent colour.
export function drawSessionsChart(canvas, sessions, accent) {
  const { ctx, w, h } = setup(canvas), L = 34, R = 10, T = 8, B = 20, pw = w - L - R, ph = h - T - B;
  const list = sessions.filter(s => s.curve?.length > 1).slice(0, 5);
  if (!list.length) { ctx.fillStyle = "#8B96A3"; ctx.fillText("Brak zapisanych przejazdów - pojawią się po pierwszym sprzątaniu.", L, h / 2); return; }
  const maxMin = Math.max(10, ...list.map(s => s.duration_s / 60)), minBat = Math.min(60, ...list.flatMap(s => s.curve.map(c => c[1]))) - 5;
  const X = m => L + m / maxMin * pw, Y = p => T + (100 - p) / (100 - minBat) * ph;
  const yT = []; for (let v = 100; v >= minBat; v -= 10) yT.push(v);
  const step = maxMin > 60 ? 20 : maxMin > 20 ? 10 : 2, xT = [];
  for (let m = 0; m <= maxMin; m += step) xT.push([m, m + (m + step > maxMin ? " min" : "")]);
  axes(ctx, L, T, pw, ph, yT, Y, xT, X);
  [...list].reverse().forEach((s, i, arr) => {
    const newest = i === arr.length - 1;
    ctx.strokeStyle = newest ? accent : "#3A4552"; ctx.lineWidth = newest ? 2.5 : 1.5;
    ctx.beginPath();
    s.curve.forEach(([t, b], k) => ctx[k ? "lineTo" : "moveTo"](X(t / 60), Y(b)));
    ctx.stroke();
    if (newest) { const [t, b] = s.curve.at(-1); ctx.fillStyle = accent; ctx.fillRect(X(t / 60) - 4, Y(b) - 4, 8, 8); }
  });
}

// Energy per day [Wh] for 14 days, estimated from battery % used (74.9 Wh pack).
export function drawDailyChart(canvas, sessions, accent) {
  const { ctx, w, h } = setup(canvas), T = 16, B = 20, ph = h - T - B, days = [];
  const today = new Date(); today.setHours(0, 0, 0, 0);
  for (let i = 13; i >= 0; i--) days.push({ d: new Date(today - i * 864e5), wh: 0 });
  for (const s of sessions) {
    const d = new Date(s.start * 1000); d.setHours(0, 0, 0, 0);
    const day = days.find(x => +x.d === +d);
    if (day && s.bat_start != null && s.bat_end != null) day.wh += Math.max(0, s.bat_start - s.bat_end) * WH_PER_PCT;
  }
  const max = Math.max(5, ...days.map(d => d.wh)), bw = w / 14;
  days.forEach((d, i) => {
    const bh = d.wh / max * ph;
    ctx.fillStyle = i === 13 ? accent : d.wh > max * 0.8 ? "#F08A24" : "#4A5663";
    ctx.fillRect(i * bw + 3, T + ph - Math.max(1, bh), bw - 6, Math.max(1, bh));
    ctx.fillStyle = "#A3AEB9"; ctx.textAlign = "center";
    ctx.fillText(d.wh ? d.wh.toFixed(1) : "—", i * bw + bw / 2, T + ph - bh - 4);
    ctx.fillStyle = "#8B96A3"; ctx.fillText(d.d.getDate(), i * bw + bw / 2, h - 5);
  });
  ctx.textAlign = "left";
  return days.reduce((a, d) => a + d.wh, 0);
}

// One run: battery line (coloured by level), an optional sensor series, event markers, status band, cursor.
export function drawReplayChart(canvas, rows, events, idx, series) {
  const { ctx, w, h } = setup(canvas), L = 34, R = 40, T = 22, B = 30, pw = w - L - R, ph = h - T - B;
  if (rows.length < 2) return;
  const t0 = rows[0].t, t1 = rows.at(-1).t || t0 + 1, X = t => L + (t - t0) / (t1 - t0) * pw;
  const bats = rows.map(r => r.bat).filter(b => b != null), minBat = Math.max(0, Math.min(...bats) - 3), maxBat = Math.min(100, Math.max(...bats) + 1);
  const Y = b => T + (maxBat - b) / Math.max(1, maxBat - minBat) * ph;
  const xT = []; for (let k = 0; k <= 4; k++) xT.push([t0 + (t1 - t0) * k / 4, fmtDur((t1 - t0) * k / 4)]);
  axes(ctx, L, T, pw, ph, [maxBat, Math.round((maxBat + minBat) / 2), minBat], Y, xT, X);

  const val = r => (series === "wifi" ? r.wifi : r.s?.[series]);
  const sv = rows.map(val).filter(v => v != null);
  if (series && sv.length > 1) {
    const lo = Math.min(...sv), hi = Math.max(...sv), Ys = v => T + ph - (v - lo) / Math.max(1e-6, hi - lo) * ph;
    ctx.strokeStyle = SERIES[series][1]; ctx.lineWidth = 1; ctx.globalAlpha = 0.85; ctx.beginPath();
    let pen = false;
    for (const r of rows) { const v = val(r); if (v == null) { pen = false; continue; } ctx[pen ? "lineTo" : "moveTo"](X(r.t), Ys(v)); pen = true; }
    ctx.stroke(); ctx.globalAlpha = 1;
    ctx.fillStyle = SERIES[series][1]; ctx.fillText(Math.round(hi), L + pw + 6, T + 4); ctx.fillText(Math.round(lo), L + pw + 6, T + ph);
  }
  ctx.lineWidth = 2.5;
  for (let i = 1; i < rows.length; i++) {
    if (rows[i].bat == null || rows[i - 1].bat == null) continue;
    ctx.strokeStyle = batteryColor(rows[i].bat);
    ctx.beginPath(); ctx.moveTo(X(rows[i - 1].t), Y(rows[i - 1].bat)); ctx.lineTo(X(rows[i].t), Y(rows[i].bat)); ctx.stroke();
  }
  for (const e of events) { // markers above the plot: where a sensor fired
    const st = EVENT_STYLE[e.k];
    if (!st || e.k === "session" || e.k === "status") continue;
    const x = X(e.t);
    ctx.strokeStyle = st[0] + "66"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
    ctx.fillStyle = st[0]; ctx.beginPath(); ctx.moveTo(x, T - 2); ctx.lineTo(x - 5, T - 11); ctx.lineTo(x + 5, T - 11); ctx.closePath(); ctx.fill();
  }
  const band = { cleaning: "#C8FF3D", moving: "#B48CFF", returning: "#F08A24", docked: "#7BE04A", idle: "#8B96A3", paused: "#F2D23C", error: "#D6334A" };
  for (let i = 1; i < rows.length; i++) { ctx.fillStyle = band[rows[i].st] ?? "#555"; ctx.fillRect(X(rows[i - 1].t), h - 8, X(rows[i].t) - X(rows[i - 1].t) + 0.5, 4); }
  const r = rows[idx];
  ctx.strokeStyle = "#E6EBF0"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(r.t), T - 12); ctx.lineTo(X(r.t), h - 4); ctx.stroke();
  canvas._seek = px => { // nearest frame to an x position
    const t = t0 + (px - L) / pw * (t1 - t0);
    let lo = 0, hi = rows.length - 1;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (rows[mid].t < t) lo = mid + 1; else hi = mid; }
    return lo;
  };
}

// Polar plot of one lidar turn around the robot. pts: [{angle_deg, dist_cm}], heading up.
export function drawPolar(canvas, pts, maxM = 8) {
  const { ctx, w, h } = setup(canvas), cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 12;
  ctx.strokeStyle = "#1D242C"; ctx.lineWidth = 1;
  ctx.strokeRect(cx - R, cy - R, 2 * R, 2 * R);
  for (let k = 1; k <= 4; k++) { ctx.beginPath(); ctx.arc(cx, cy, R * k / 4, 0, 2 * Math.PI); ctx.stroke(); }
  ctx.beginPath(); ctx.moveTo(cx, cy - R - 6); ctx.lineTo(cx, cy + R + 6); ctx.moveTo(cx - R - 6, cy); ctx.lineTo(cx + R + 6, cy); ctx.stroke();
  ctx.fillStyle = "#8B96A3";
  for (let k = 1; k <= 4; k++) ctx.fillText(`${maxM * k / 4}m`, cx + 3, cy - R * k / 4 + 10);
  const xy = p => { const a = (p.angle - 90) * Math.PI / 180, r = Math.min(1, p.dist / (maxM * 100)) * R; return [cx + Math.cos(a) * r, cy + Math.sin(a) * r]; };
  const sorted = [...pts].sort((a, b) => a.angle - b.angle);
  if (sorted.length > 2) {
    ctx.fillStyle = "#38E1F014"; ctx.beginPath();
    sorted.forEach((p, i) => { const [x, y] = xy(p); ctx[i ? "lineTo" : "moveTo"](x, y); });
    ctx.closePath(); ctx.fill();
  }
  ctx.fillStyle = "#38E1F0";
  for (const p of sorted) { const [x, y] = xy(p); ctx.fillRect(x - 1.2, y - 1.2, 2.4, 2.4); }
  ctx.fillStyle = getComputedStyle(canvas).getPropertyValue("--acc") || "#C8FF3D";
  ctx.fillRect(cx - 5, cy - 5, 10, 10);
}
