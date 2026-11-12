// Control panel logic: live state from the local server, robot controls, history + replay, photos.
import { createScene } from "./scene.js";
import { wallGrid, raycast, mapBounds, signalColor, batteryColor } from "./mapdata.js";
import { loadSession, drawSessionsChart, drawDailyChart, drawReplayChart, drawPolar, fmtDur, EVENT_STYLE, SERIES } from "./history.js";

const $ = s => document.querySelector(s);
const esc = v => String(v ?? "").replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const ACC = getComputedStyle(document.documentElement).getPropertyValue("--acc").trim();
const api = async (p, init) => {
  const r = await fetch(p, init);
  if (!r.ok) throw new Error(`${p}: HTTP ${r.status} ${(await r.text()).slice(0, 160)}`);
  return r.json().catch(() => ({}));
};
const put = (cap, body) => api("/api/v2/robot/capabilities/" + cap, { method: "PUT", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
const cell = (label, value, style = "") => `<div><div class="lbl s">${label}</div><div class="big" style="${style}">${value}</div></div>`;

const STATUS_PL = { cleaning: "SPRZĄTANIE", moving: "JAZDA", returning: "POWRÓT", docked: "W STACJI", idle: "POSTÓJ", paused: "PAUZA", manual_control: "RĘCZNE", error: "BŁĄD" };
const FAN_PL = { low: "Cicha", medium: "Balans", high: "Turbo", max: "Max", off: "Wył.", min: "Min" };
const CONS = { "brush/main": ["Szczotka główna", 300], "brush/side_right": ["Szczotka boczna", 200], "filter/main": ["Filtr", 150], "cleaning/sensor": ["Czujniki", 30], "sensor/all": ["Czujniki", 30] };
const VIEWS = [["2d", "2D"], ["3d", "3D"], ["iso", "ISO"], ["orbit", "Orbita"], ["fpv", "FPV"]];
const LAYERS = [["walls", "Ściany"], ["obst", "Meble i przeszkody"], ["wifi", "Heatmapa Wi-Fi"], ["cloud", "Chmura lidar"], ["scan", "Skan na żywo"],
  ["path", "Trasa"], ["events", "Zdarzenia czujników"], ["evlabels", "Etykiety zdarzeń"], ["labels", "Etykiety"], ["mirrors", "Lustra / szyby"], ["phantom", "Odbicia (błędne)"], ["photos", "Zdjęcia"], ["hot", "Miejsca problemowe"]];

const S = { state: null, map: null, mapNonce: null, cloudRev: -1, cloudAt: 0, cloud: null, wifi: null, sessions: [], view: "3d", chart: "sessions",
  layers: { walls: true, obst: true, wifi: true, cloud: true, scan: true, path: true, events: true, labels: true, evlabels: false, mirrors: true, phantom: false, photos: true, hot: true },
  rooms: new Set(), passes: 1, replay: null, info: {}, lastOk: 0, photo: null };

let scene;
try { scene = createScene($("#scene"), $("#labels")); } catch (e) { $("#scene").insertAdjacentHTML("beforeend", `<div class="hud" style="left:230px;top:16px">Nie udało się uruchomić widoku 3D: ${esc(e.message)}</div>`); }

// ---------- view + layers ----------
function drawViews() {
  $("#views").innerHTML = VIEWS.map(([id, l]) => `<button class="chip ${S.view === id ? "on" : ""}" aria-pressed="${S.view === id}" data-v="${id}">${l}</button>`).join("");
  $("#layers").innerHTML = LAYERS.map(([id, l]) => `<button class="chip ${S.layers[id] ? "on" : ""}" aria-pressed="${!!S.layers[id]}" data-l="${id}" style="justify-content:flex-start;width:100%;min-height:26px;font-size:10px;white-space:nowrap"><span class="sq"></span>${l}</button>`).join("");
  $("#r-fpv").classList.toggle("on", S.view === "fpv");
}
function setView(v) { S.view = v; scene?.setView(v); drawViews(); }
$("#follow").onchange = e => scene?.setFollow(e.target.checked);
$("#views").onclick = e => { const v = e.target.closest("[data-v]")?.dataset.v; if (v) setView(v); };
$("#layers").onclick = e => { const l = e.target.closest("[data-l]")?.dataset.l; if (l) { S.layers[l] = !S.layers[l]; scene?.setLayers(S.layers); drawViews(); } };
$("#wallh").oninput = e => { $("#wallh-v").textContent = e.target.value; scene?.setWallHeight(+e.target.value); };
scene?.setLayers(S.layers);
drawViews();

// ---------- top bar + telemetry ----------
const attr = cls => S.state?.attrs?.find(a => a.__class === cls);

function showState() {
  const st = S.state, status = attr("StatusStateAttribute"), bat = attr("BatteryStateAttribute"), wifi = st.wifi ?? {};
  const value = status?.value, active = ["cleaning", "moving", "returning", "manual_control"].includes(value);
  $("#h-status").textContent = (STATUS_PL[value] ?? value ?? "—") + (status?.error ? `: ${status.error.message}` : "");
  $("#h-dot").className = active ? "blink" : ""; $("#h-dot").style.background = value === "error" ? "var(--red)" : "var(--acc)";
  $("#h-battery").textContent = bat ? `${bat.level}%${bat.flag === "charging" ? " ⚡" : ""}` : "—";
  $("#h-wifi").textContent = wifi.signal != null ? `${wifi.signal} dBm` : "—"; $("#h-wifi").style.color = wifi.signal != null ? signalColor(wifi.signal) : "";
  const running = value === "cleaning" || value === "moving" || value === "returning";
  $("#b-run-label").textContent = running ? "Pauza" : value === "paused" ? "Wznów" : "Start";
  $("#b-run-icon").setAttribute("d", running ? "M3 2h3v10H3zM8 2h3v10H8z" : "M3 1.5l9 5.5-9 5.5z");
  $("#b-run").dataset.action = running ? "pause" : "start";
  $("#b-explore-label").textContent = st.explore.running ? "Stop skanowania" : "Skanuj mieszkanie";
  $("#banner").style.display = st.explore.msg && (st.explore.running || /błąd/.test(st.explore.msg)) ? "block" : "none";
  $("#banner").textContent = (/^(krok|program)/.test(st.explore.msg) ? "Program: " : "Skanowanie mieszkania: ") + st.explore.msg;
  $("#live").innerHTML = `<span class="blink" style="width:6px;height:6px;background:var(--acc)"></span>LIVE${st.tap ? " · LIDAR + CZUJNIKI" : " · tylko Valetudo"}`;

  $("#t-bat").textContent = bat?.level ?? "—";
  const on = Math.round((bat?.level ?? 0) / 5);
  $("#t-segs").innerHTML = Array.from({ length: 20 }, (_, i) => `<div class="bseg ${i < on - 1 ? "on" : i === on - 1 ? (active ? "last" : "on") : ""}"></div>`).join("");
  const sn = st.sensors;
  $("#t-power").innerHTML = sn ? `<div>${bat?.flag === "charging" ? "ładowanie" : "pobór"} <b>${sn.watts ?? "—"} W</b></div><div>napięcie <b>${sn.mv ? (sn.mv / 1000).toFixed(2) : "—"} V</b></div><div>prąd <b>${sn.ma ?? "—"} mA</b></div><div>temp. baterii <b>${sn.temp ?? "—"} °C</b></div>`
    : `<div>czujniki: brak połączenia z podsłuchem</div>`;
  const bars = wifi.signal == null ? 0 : wifi.signal > -55 ? 4 : wifi.signal > -67 ? 3 : wifi.signal > -75 ? 2 : 1, col = signalColor(wifi.signal ?? -90);
  $("#t-wifi").innerHTML = `
    <div><div class="lbl">Sygnał Wi-Fi</div><div style="display:flex;align-items:flex-end;gap:10px;margin-top:6px"><div style="display:flex;align-items:flex-end;gap:3px;height:22px" aria-hidden="true">
      ${[6, 11, 16, 22].map((h, i) => `<div style="width:5px;height:${h}px;background:${i < bars ? col : "#2A333D"}"></div>`).join("")}</div><span class="big">${wifi.signal ?? "—"} dBm</span></div></div>
    <div><div class="lbl">SSID · pasmo</div><div class="val" style="margin-top:8px">${esc(wifi.ssid ?? "—")} · ${esc((wifi.frequency ?? "").replace("ghz", " GHz"))}</div></div>
    <div><div class="lbl">Przepustowość</div><div class="val" style="margin-top:8px">${wifi.upspeed ?? "—"} Mbps ↑</div></div>
    <div><div class="lbl">BSSID</div><div class="val" style="margin-top:8px;font-size:12px">${esc(wifi.bssid ?? "—")}</div></div>`;

  const p = st.pose, sc = st.scan;
  $("#hud-pos").innerHTML = `<div>POS <b>${p ? `${Math.round(p.x)}, ${Math.round(p.y)} cm` : "—"}</b></div><div>KĄT <b>${p ? Math.round(p.angle) + "°" : "—"}</b></div>
    <div>POZYCJA <b>${p?.src === "slam" ? "SLAM 10 Hz" : "mapa"}</b></div><div>LDS <b style="color:var(--cyan)">${sc && sc.age < 3 ? sc.rpm + " RPM" : "stoi"}</b></div>
    <div>PRZECHYŁ <b>${sn?.tilt ?? "—"}°</b></div>`;
  $("#hud-stats").innerHTML = [["Pomiary Wi-Fi", st.wifi_samples, ""], ["Punkty lidar", st.cloud.cells, "color:var(--cyan)"],
    ["Przeszkody", S.cloud ? S.cloud.obstacles.length : "—", ""], ["Lustra/szyby", S.cloud ? S.cloud.reflective.length : "—", "color:#B48CFF"]]
    .map(([l, v, c], i) => `<div style="padding:8px 12px;${i % 2 ? "" : "border-right:1px solid var(--line);"}${i < 2 ? "border-bottom:1px solid var(--line)" : ""}"><div class="lbl s">${l}</div><div style="font-size:15px;font-weight:700;${c}">${v}</div></div>`).join("");
  if (!S.replay) scene?.setRobot(p, active);
  scene?.setRoute(st.explore.running ? st.explore.route : null, st.explore.i);
  showProgram(st);
  scene?.setLiveEvents(st.events.filter(e => st.t - e.t < 3600));
  showSensors(sn);
  showEvents(st.events);
  $("#f-sync").textContent = "Ostatnia synchronizacja " + new Date().toLocaleTimeString("pl-PL");
}

function showSensors(sn) {
  $("#sen-src").textContent = sn ? `${sn.packets} pakietów MCU` : "brak danych";
  if (!sn) { $("#sen").innerHTML = `<div style="grid-column:1/-1" class="kv">Dane czujników idą przez podsłuch na robocie (SSH). Teraz nie jest połączony.</div>`; return; }
  const onoff = (v, txt = "WCIŚNIĘTY") => (v ? `<span style="color:var(--orange)">${txt}</span>` : "—");
  const load = (v, base) => (base ? ` <span style="color:var(--muted);font-size:11px">/ ${base}</span>` : "");
  $("#sen").innerHTML = [
    ["Zderzak L / P", `${onoff(sn.bumper_l)} / ${onoff(sn.bumper_r)}`], ["Wieżyczka lidaru", onoff(sn.lds_bumper)],
    ["Czujniki krawędzi", onoff(sn.cliff, "KRAWĘDŹ " + sn.cliff)], ["Koła opadły", onoff(sn.drop, "TAK")],
    ["Prąd kół L / P", `${sn.wheel_l} / ${sn.wheel_r}${load(0, sn.wheel_base)}`], ["Prąd szczotki", `${sn.brush}${load(0, sn.brush_base)}`],
    ["Czujnik ściany", sn.wall ?? "—"], ["Przechył", `${sn.tilt}°`],
    ["Wentylator / szczotka", `${sn.fan_duty ?? "—"} / ${sn.brush_duty ?? "—"} %`], ["Żyroskop Z", `${sn.gyro_z} rad/s`],
  ].map(([l, v]) => `<div><div class="lbl s">${l}</div><div class="val">${v}</div></div>`).join("");
}

function showEvents(events) {
  $("#events").innerHTML = [...events].reverse().slice(0, 40).map(e => {
    const st = EVENT_STYLE[e.k] ?? ["#A3AEB9"];
    const extra = e.side ? ` (${e.side})` : e.value != null ? ` (${e.value}${e.base ? " / norma " + e.base : ""})` : e.text ? `: ${e.text}` : "";
    return `<li><span class="mono" style="font-size:12px;color:var(--muted)">${new Date(e.t * 1000).toLocaleTimeString("pl-PL")}</span>
      <span style="width:8px;height:8px;background:${st[0]};margin-top:5px"></span><span style="font-size:13px;line-height:1.45">${esc(e.msg)}${esc(extra)}</span></li>`;
  }).join("") || `<li><span></span><span></span><span class="kv">Brak zdarzeń.</span></li>`;
}

// ---------- map, cloud, Wi-Fi, scan ----------
async function loadMap() {
  S.map = await api("/local/map");
  if (!S.map.layers) return;
  scene?.setMap(S.map, S.cloud?.islands);
  S.grid = wallGrid(S.map);
  const b = mapBounds(S.map), ps = S.map.pixelSize;
  const floor = S.map.layers.filter(l => l.type !== "wall").reduce((a, l) => a + (l.metaData?.area ?? 0), 0);
  $("#f-map").textContent = b ? `Mapa z lidaru · ${ps} cm/piksel · ${((b.maxX - b.minX + 1) * ps / 100).toFixed(1)} × ${((b.maxY - b.minY + 1) * ps / 100).toFixed(1)} m · ${(floor / 1e4).toFixed(1)} m²` : "—";
  drawRooms();
}

async function loadCloud() {
  const c = await api("/local/cloud");
  const islandsChanged = c.islands.length !== (S.cloud?.islands.length ?? -1);
  S.cloud = c; S.cloudRev = c.rev; S.cloudAt = performance.now();
  if (islandsChanged) scene?.islands(c.islands);
  const ok = (c.open_px ?? []).join(";"); // walls the lidar now sees through (opened door) disappear from the 3D map
  if (ok !== S.openKey) { S.openKey = ok; scene?.openings(c.open_px ?? [], ok); }
  scene?.setCloud(c);
  const n = [0, 0, 0];
  for (const p of c.cells) n[p[3]]++;
  const small = c.obstacles.filter(o => o.kind === "drobna").length;
  $("#an").innerHTML = [["Punkty na ścianach", n[0]], ["Punkty przeszkód", n[1]], ["Przeszkody drobne ≤ 30 cm", small], ["Przeszkody duże", c.obstacles.length - small],
    ["Odbicia odrzucone", c.phantom.length + n[2]], ["Ruchome (pominięte, 30 s)", (c.moving ?? []).length], ["Meble z mapy (piksele)", c.islands.length]].map(([l, v]) => cell(l, v)).join("");
  $("#an-refl").innerHTML = c.reflective.length ? c.reflective.map(s =>
    `<div><b style="color:${{ "lustro": "#B48CFF", "szyba/otwór": "#5DD8FF", "drzwi?": "#7CF29A" }[s.kind] ?? "var(--yellow)"}">${esc(s.kind)}</b> · ${Math.round(Math.hypot(s.x2 - s.x1, s.y2 - s.y1))} cm · ${s.n} promieni${s.kind === "lustro" ? ` · zgodność odbicia ${Math.round(s.score * 100)}%` : ""}</div>`).join("")
    : "Nie wykryto jeszcze luster ani szyb. Potrzebny jest przejazd z włączonym lidarem (sprzątanie albo „Skanuj mieszkanie”).";
  $("#an-refl").innerHTML += (c.doors ?? []).map(d => `<div><b style="color:#7CF29A">${esc(d.kind)}</b> · ${d.width} cm · ${esc(d.rooms.join(" ↔ "))} · <b style="color:${d.state === "zamknięte" ? "#D6334A" : "inherit"}">${esc(d.state ?? "stan nieznany")}</b></div>`).join("");
}

async function loadWifi() {
  const w = S.wifi = await api("/local/wifi");
  scene?.setWifi(w);
  $("#w-rooms").innerHTML = w.rooms.map(r => `<div style="display:flex;flex-direction:column;gap:5px">
    <div style="display:flex;justify-content:space-between;font-size:13px"><span>${esc(r.name)}</span><span class="mono" style="color:${signalColor(r.avg)};font-weight:700">${r.avg} dBm</span></div>
    <div style="height:8px;background:#1A2129"><div class="hbar" style="height:8px;width:${Math.round(Math.max(4, Math.min(100, (r.avg + 95) / 55 * 100)))}%;background:${signalColor(r.avg)}"></div></div>
    <div class="mono" style="font-size:10px;color:var(--muted)">min ${r.min} · maks ${r.max} · ${r.n} pomiarów</div></div>`).join("") || `<div class="kv">Brak pomiarów. Zbierają się podczas jazdy robota.</div>`;
  const weak = w.rooms[0];
  $("#w-warn").style.display = weak ? "flex" : "none";
  if (weak) $("#w-warn").innerHTML = `<svg width="16" height="16" viewBox="0 0 16 16" fill="none" stroke="#F08A24" stroke-width="1.6" aria-hidden="true" style="flex:none;margin-top:2px"><path d="M8 1.5L15 14H1z"></path><path d="M8 6v4M8 11.5v1"></path></svg>
    <span>Najsłabszy zasięg: <b>${esc(weak.name)}</b> (średnio ${weak.avg} dBm, minimum ${weak.min} dBm).${weak.avg < -72 ? " Rozważ dodatkowy punkt dostępowy w pobliżu." : ""}</span>`;
}

async function loadScan() {
  const st = S.state;
  if (S.replay || !st) return;
  const live = st.scan && st.scan.age < 3;
  let pts = [], label = "z mapy (lidar stoi)";
  if (live) {
    const sc = await api("/local/scan");
    if (sc.points?.length && sc.pose) {
      scene?.setScan(sc.points, sc.pose);
      const head = sc.pose.angle - 90;
      pts = sc.points.map(p => ({ angle: ((Math.atan2(p[1] - sc.pose.y, p[0] - sc.pose.x) * 180 / Math.PI - head) % 360 + 360) % 360, dist: p[2] / 10 }));
      label = `na żywo · ${sc.rpm} RPM · bez echa: ${sc.no_echo}`;
    }
  } else {
    scene?.setScan(null);
    if (S.grid && st.pose) { // what the lidar would see from here according to the map
      const head = st.pose.angle - 90;
      pts = raycast(S.grid, S.map.pixelSize, [st.pose.x, st.pose.y]).filter(p => p.hit).map(p => ({ angle: ((p.angle - head) % 360 + 360) % 360, dist: p.dist }));
    }
  }
  polar(pts, label);
}

function polar(pts, label) {
  drawPolar($("#polar"), pts);
  const d = pts.map(p => p.dist);
  $("#s-count").textContent = `${pts.length} pkt`;
  $("#s-stats").innerHTML = `<span>Źródło <b style="color:var(--text)">${esc(label)}</b></span><span>Min. odl. <b style="color:var(--text)">${d.length ? (Math.min(...d) / 100).toFixed(2) : "—"} m</b></span>
    <span>Zasięg maks. <b style="color:var(--text)">${d.length ? (Math.max(...d) / 100).toFixed(1) : "—"} m</b></span><span>Rozdz. 1° · 5 Hz</span>`;
}

// ---------- cleaning parameters ----------
function drawRooms() {
  const rooms = scene?.rooms() ?? [];
  $("#p-rooms").innerHTML = rooms.map(r => `<button class="chip ${S.rooms.has(r.id) ? "on" : ""}" aria-pressed="${S.rooms.has(r.id)}" data-r="${esc(r.id)}">${esc(r.name)}</button>`).join("") || `<span class="kv">Mapa nie ma podziału na pokoje.</span>`;
  const area = rooms.filter(r => S.rooms.has(r.id)).reduce((a, r) => a + r.area, 0);
  const tot = S.info.total ?? {}, minPerM2 = tot.area ? tot.time / 60 / (tot.area / 1e4) : null;
  const runs = S.sessions.filter(s => s.type === "clean" && s.duration_s > 120 && s.bat_start != null);
  const pctPerMin = runs.length ? runs.reduce((a, s) => a + (s.bat_start - s.bat_end), 0) / runs.reduce((a, s) => a + s.duration_s / 60, 0) : null;
  const estMin = minPerM2 != null ? area * minPerM2 * S.passes : null, estBat = estMin != null && pctPerMin != null ? estMin * pctPerMin : null;
  $("#p-est").innerHTML = cell("Powierzchnia", area.toFixed(1) + " m²") + cell("Szac. czas", estMin != null ? Math.round(estMin) + " min" : "—")
    + cell("Szac. bateria", estBat != null ? "−" + Math.round(estBat) + "%" : "—", estBat > 70 ? "color:var(--orange)" : "");
  $("#p-start").disabled = !S.rooms.size;
  $("#p-pass").innerHTML = [1, 2, 3].filter(n => n <= (S.info.segProps?.iterationCount?.max ?? 1)).map(n => `<button class="seg ${S.passes === n ? "on" : ""}" aria-pressed="${S.passes === n}" data-n="${n}">${n}×</button>`).join("");
}
$("#p-rooms").onclick = e => { const id = e.target.closest("[data-r]")?.dataset.r; if (id) { S.rooms.has(id) ? S.rooms.delete(id) : S.rooms.add(id); drawRooms(); } };
$("#p-pass").onclick = e => { const n = e.target.closest("[data-n]")?.dataset.n; if (n) { S.passes = +n; drawRooms(); } };
$("#p-start").onclick = () => act(() => put("MapSegmentationCapability", { action: "start_segment_action", segment_ids: [...S.rooms], iterations: S.passes, customOrder: false }));

function drawParams() {
  const cur = S.state?.attrs?.find(a => a.__class === "PresetSelectionStateAttribute" && a.type === "fan_speed")?.value;
  $("#p-fan").innerHTML = (S.info.fan ?? []).map(n => `<button class="seg ${cur === n ? "on" : ""}" aria-pressed="${cur === n}" data-f="${esc(n)}">${FAN_PL[n] ?? esc(n)}</button>`).join("");
  const dnd = S.info.dnd, carpet = !!S.info.carpet?.enabled, two = n => String(n).padStart(2, "0");
  const local = t => { const d = new Date(); d.setUTCHours(t.hour, t.minute); return `${two(d.getHours())}:${two(d.getMinutes())}`; };
  $("#p-opts").innerHTML = `
    <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 0;border-bottom:1px solid var(--line)"><div style="display:flex;flex-direction:column"><span>Wzmocnienie na dywanie</span><span class="mono" style="font-size:11px;color:var(--muted)">robot sam zwiększa ssanie na dywanie</span></div>
      <button class="tgl ${carpet ? "on" : ""}" id="o-carpet" aria-pressed="${carpet}" aria-label="Wzmocnienie na dywanie"><span class="knob"></span></button></div>
    <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 0;border-bottom:1px solid var(--line)"><div style="display:flex;flex-direction:column"><span>Nie przeszkadzać</span><span class="mono" style="font-size:11px;color:var(--muted)">${dnd ? `${dnd.enabled ? "włączone" : "wyłączone"} · ${local(dnd.start)} – ${local(dnd.end)}` : "—"} · zmiana niżej, w „Valetudo · wszystkie funkcje”</span></div></div>`;
  $("#o-carpet").onclick = () => act(async () => { await put("CarpetModeControlCapability", { action: carpet ? "disable" : "enable" }); await loadInfo(); });
}
$("#p-fan").onclick = e => { const n = e.target.closest("[data-f]")?.dataset.f; if (n) act(() => put("FanSpeedControlCapability/preset", { name: n })); };

async function act(fn) {
  try { await fn(); await tick(); } catch (e) { alert("Nie udało się: " + e.message); }
}
$("#b-run").onclick = () => act(() => put("BasicControlCapability", { action: $("#b-run").dataset.action || "start" }));
$("#b-stop").onclick = () => act(() => put("BasicControlCapability", { action: "stop" }));
$("#b-home").onclick = () => act(() => put("BasicControlCapability", { action: "home" }));
$("#b-locate").onclick = () => act(() => put("LocateCapability", { action: "locate" }));
$("#b-explore").onclick = () => act(async () => {
  if (S.state.explore.running) return api("/local/explore/stop", { method: "POST" });
  if (confirm("Robot objedzie wszystkie pokoje bez sprzątania (punkty na środku pomieszczeń), a potem wróci na stację. Start?")) await api("/local/explore/start", { method: "POST" });
});
$("#w-clear").onclick = () => confirm("Usunąć wszystkie zapisane pomiary zasięgu Wi-Fi?") && act(async () => { await api("/local/wifi/clear", { method: "POST" }); await loadWifi(); });
$("#a-clear").onclick = () => confirm("Wyczyścić chmurę punktów lidaru i wykryte lustra/szyby?") && act(async () => { await api("/local/cloud/clear", { method: "POST" }); await loadCloud(); });

// ---------- slow info: consumables, timers, statistics ----------
async function loadInfo() {
  const g = p => api("/api/v2/" + p).catch(() => null), cap = "robot/capabilities/";
  let [robot, ver, fan, segProps, carpet, dnd, cons, cur, tot, timers] = await Promise.all([g("robot"), g("valetudo/version"), g(cap + "FanSpeedControlCapability/presets"),
    g(cap + "MapSegmentationCapability/properties"), g(cap + "CarpetModeControlCapability"), g(cap + "DoNotDisturbCapability"), g(cap + "ConsumableMonitoringCapability"),
    g(cap + "CurrentStatisticsCapability"), g(cap + "TotalStatisticsCapability"), g("timers")]);
  const pts = list => Object.fromEntries((list ?? []).map(d => [d.type, d.value]));
  // a failed request (robot briefly out of Wi-Fi range) keeps the previous value instead of blanking the panel
  const fresh = { fan, segProps, carpet, dnd, total: tot && pts(tot), current: cur && pts(cur), timers: timers && Object.values(timers), robot, ver, cons };
  for (const [k, v] of Object.entries(fresh)) if (v != null) S.info[k] = v;
  ({ robot, ver, cons } = S.info);
  if (robot) $("#h-model").textContent = `${robot.manufacturer} ${robot.modelName}`;
  const host = (S.state?.robot ?? "").replace(/^https?:\/\//, "");
  $("#h-sub").title = `Valetudo ${ver?.release ?? "?"} · ${host}`;
  $("#tm-edit").href = (S.state?.robot ?? "#") + "/#/timers";
  const c = S.info.current ?? {};
  $("#h-time").textContent = c.time != null ? fmtDur(c.time) : "—";
  $("#h-area").textContent = c.area != null ? (c.area / 1e4).toFixed(1) + " m²" : "—";
  $("#t-task").textContent = c.area != null ? `${(c.area / 1e4).toFixed(1)} m² · ${fmtDur(c.time ?? 0)}` : "—";
  const floor = S.map?.layers?.filter(l => l.type !== "wall").reduce((a, l) => a + (l.metaData?.area ?? 0), 0) || 0;
  $("#t-task-bar").style.width = floor && c.area ? Math.min(100, Math.round(c.area / floor * 100)) + "%" : "0";
  drawParams(); drawRooms(); drawTimers(); drawHistorySummary();
  $("#cons").innerHTML = (cons ?? []).map(x => {
    const [name, life] = CONS[`${x.type}/${x.subType}`] ?? [`${x.type} ${x.subType}`, null];
    const leftH = x.remaining.unit === "minutes" ? x.remaining.value / 60 : null;
    const pct = x.remaining.unit === "percent" ? x.remaining.value : life ? Math.max(0, Math.min(100, Math.round(leftH / life * 100))) : 0;
    const col = pct < 25 ? "var(--orange)" : "var(--text)", on = Math.round(pct / 5);
    return `<div style="display:flex;flex-direction:column;gap:6px;padding-bottom:12px;border-bottom:1px solid var(--line)">
      <div style="display:flex;justify-content:space-between;align-items:center;gap:10px"><span>${esc(name)}</span><span class="mono" style="font-size:12px;color:${col};font-weight:700">${pct}%</span></div>
      <div style="display:flex;gap:2px">${Array.from({ length: 20 }, (_, i) => `<div style="flex:1 1 0;height:10px;background:${i < on ? (pct < 25 ? "var(--orange)" : "var(--acc)") : "#1A2129"}"></div>`).join("")}</div>
      <div style="display:flex;justify-content:space-between;align-items:center"><span class="mono" style="font-size:11px;color:var(--muted)">${leftH != null ? `zostało ${leftH.toFixed(0)} h${life ? " z " + life + " h" : ""}` : ""}${pct === 0 ? " · do wymiany lub zresetowania licznika" : ""}</span>
      <button class="chip sm" data-cons="${esc(x.type)}/${esc(x.subType)}" data-name="${esc(name)}">Reset</button></div></div>`;
  }).join("") || `<div class="kv">Brak danych o materiałach eksploatacyjnych.</div>`;
}
$("#cons").onclick = e => {
  const b = e.target.closest("[data-cons]");
  if (b && confirm(`Zresetować licznik „${b.dataset.name}”? Zrób to tylko po faktycznej wymianie lub czyszczeniu.`))
    act(async () => { await put("ConsumableMonitoringCapability/" + b.dataset.cons, { action: "reset" }); await loadInfo(); });
};

function drawTimers() {
  const days = ["Pn", "Wt", "Śr", "Cz", "Pt", "Sb", "Nd"], today = (new Date().getDay() + 6) % 7, slots = days.map(() => []);
  for (const t of S.info.timers ?? []) {
    for (const dow of t.dow ?? []) { // Valetudo timers are UTC: shift to local time (and day)
      const d = new Date(Date.UTC(2024, 0, 7 + dow, t.hour, t.minute)); // 2024-01-07 was a Sunday (dow 0)
      slots[(d.getDay() + 6) % 7].push({ h: d.getHours() + d.getMinutes() / 60, label: `${String(d.getHours()).padStart(2, "0")}:${String(d.getMinutes()).padStart(2, "0")}`, what: t.label || t.action?.type || "", on: t.enabled });
    }
  }
  const dnd = S.info.dnd, lh = t => { const d = new Date(); d.setUTCHours(t.hour, t.minute); return d.getHours() + d.getMinutes() / 60; };
  const shade = dnd?.enabled ? (() => { const a = lh(dnd.start), b = lh(dnd.end); return a > b ? [[a, 24], [0, b]] : [[a, b]]; })() : [];
  $("#tm").innerHTML = `<div style="display:grid;grid-template-columns:40px repeat(7,minmax(0,1fr));gap:4px"><span></span>${days.map((d, i) =>
    `<span class="mono" style="font-size:11px;text-align:center;color:${i === today ? "var(--acc)" : "var(--mid)"};padding-bottom:6px;border-bottom:2px solid ${i === today ? "var(--acc)" : "var(--line)"}">${d}</span>`).join("")}</div>
    <div style="display:grid;grid-template-columns:40px repeat(7,minmax(0,1fr));gap:4px;margin-top:6px;height:200px">
    <div class="mono" style="display:flex;flex-direction:column;justify-content:space-between;font-size:10px;color:var(--muted)"><span>00</span><span>06</span><span>12</span><span>18</span><span>24</span></div>
    ${slots.map(list => `<div style="position:relative;background:#0A0D10;border:1px solid #1A2028">
      ${shade.map(([a, b]) => `<div style="position:absolute;left:0;right:0;top:${a / 24 * 100}%;height:${(b - a) / 24 * 100}%;background:repeating-linear-gradient(135deg,#141A20 0 4px,transparent 4px 8px)"></div>`).join("")}
      ${list.map(s => `<div style="position:absolute;left:3px;right:3px;top:${s.h / 24 * 100}%;min-height:16px;background:${s.on ? "var(--acc)" : "#3A4552"};color:#0A0D10;padding:2px 4px;overflow:hidden" title="${esc(s.what)}"><div class="mono" style="font-size:10px;font-weight:700">${s.label}</div></div>`).join("")}</div>`).join("")}</div>`;
  $("#tm-legend").innerHTML = `<span><span style="display:inline-block;width:10px;height:10px;background:var(--acc)"></span> zaplanowane (${(S.info.timers ?? []).length})</span>
    <span><span style="display:inline-block;width:10px;height:10px;background:repeating-linear-gradient(135deg,#3A4552 0 2px,transparent 2px 4px)"></span> nie przeszkadzać${dnd?.enabled ? "" : " (wyłączone)"}</span>
    ${(S.info.timers ?? []).length ? "" : "<span>Brak zaplanowanych sprzątań. Dodasz je niżej, w „Valetudo · wszystkie funkcje”.</span>"}`;
}

// ---------- history, charts, replay ----------
function drawHistorySummary() {
  const tot = S.info.total ?? {};
  $("#hi-sum").innerHTML = [["Zapisane tutaj", `${S.sessions.length} przejazdów`], ["Robot łącznie", tot.count != null ? `${tot.count} sprzątań` : "—"],
    ["Łączny czas", tot.time != null ? `${Math.round(tot.time / 3600)} h` : "—"], ["Powierzchnia", tot.area != null ? `${Math.round(tot.area / 1e4).toLocaleString("pl-PL")} m²` : "—"]]
    .map(([l, v]) => `<div style="padding:6px 14px"><div class="lbl s">${l}</div><div style="font-weight:700">${v}</div></div>`).join("");
}

async function loadSessions() {
  S.sessions = await api("/local/sessions");
  drawHistorySummary();
  $("#hi-rows").innerHTML = S.sessions.map(s => {
    const ev = Object.entries(s.events ?? {}).filter(([k]) => EVENT_STYLE[k] && !["session", "status"].includes(k));
    const used = s.bat_start != null ? s.bat_start - s.bat_end : null, col = s.active ? "var(--acc)" : "var(--green)";
    return `<tr class="rowh ${S.replay?.meta.id === s.id ? "sel" : ""}" data-id="${esc(s.id)}">
      <td class="mono">${new Date(s.start * 1000).toLocaleString("pl-PL", { day: "numeric", month: "numeric", hour: "2-digit", minute: "2-digit" })}</td>
      <td class="mono">${fmtDur(s.duration_s)}</td><td>${s.type === "explore" ? "Skanowanie" : "Sprzątanie"}</td><td class="mono">${s.distance_m} m</td>
      <td class="mono">${used != null ? `−${used}% <span style="color:var(--muted)">(${s.bat_start}→${s.bat_end})</span>` : "—"}</td>
      <td>${ev.map(([k, n]) => `<span class="tag" style="border-color:${EVENT_STYLE[k][0]};color:${EVENT_STYLE[k][0]};margin:0 4px 2px 0" title="${EVENT_STYLE[k][1]}">${n}× ${esc(EVENT_STYLE[k][1].split(" ")[0])}</span>`).join("") || `<span class="mono" style="color:var(--muted)">—</span>`}</td>
      <td><span class="tag" style="border-color:${col};color:${col}"><span class="${s.active ? "blink" : ""}" style="width:6px;height:6px;background:${col}"></span>${s.active ? "W toku" : "Zapisano"}</span></td></tr>`;
  }).join("") || `<tr><td colspan="7" class="kv">Brak zapisanych przejazdów. Każde sprzątanie i skanowanie zapisuje się automatycznie od teraz.</td></tr>`;
  if (!S.replay) drawChart();
}
$("#hi-rows").onclick = e => { const id = e.target.closest("[data-id]")?.dataset.id; if (id) openReplay(id); };

function drawChart() {
  const modes = S.replay ? [["replay", "Odtwarzanie"]] : [["sessions", "Sesje"], ["daily", "14 dni"]], mode = S.replay ? "replay" : S.chart;
  $("#c-modes").innerHTML = modes.map(([id, l]) => `<button class="chip ${mode === id ? "on" : ""}" aria-pressed="${mode === id}" data-c="${id}">${l}</button>`).join("");
  $("#replay-bar").style.display = S.replay ? "flex" : "none";
  const cv = $("#chart");
  if (S.replay) {
    const R = S.replay, r = R.rows[R.idx], series = $("#r-series").value;
    $("#c-title").textContent = "Przejazd " + new Date(R.rows[0].t * 1000).toLocaleString("pl-PL", { day: "numeric", month: "numeric", hour: "2-digit", minute: "2-digit" });
    drawReplayChart(cv, R.rows, R.events, R.idx, series);
    const kinds = [...new Set(R.events.map(e => e.k))].filter(k => EVENT_STYLE[k] && !["session", "status"].includes(k));
    $("#c-legend").innerHTML = `<span>linia: bateria (kolor = poziom)</span>${series ? `<span style="color:${SERIES[series][1]}">— ${SERIES[series][0]}</span>` : ""}`
      + kinds.map(k => `<span style="color:${EVENT_STYLE[k][0]}">▼ ${EVENT_STYLE[k][1]} (${R.events.filter(e => e.k === k).length})</span>`).join("") + `<span>kliknij wykres, żeby przewinąć</span>`;
    $("#c-stats").innerHTML = cell("Czas", fmtDur(r.t - R.rows[0].t)) + cell("Bateria", `${r.bat ?? "—"}%`, `color:${batteryColor(r.bat)}`)
      + cell("Wi-Fi", r.wifi != null ? r.wifi + " dBm" : "—") + cell("Szczotka / koła", `${r.s?.br ?? "—"} / ${r.s?.wh ?? "—"}`);
    return;
  }
  $("#c-title").textContent = "Zużycie baterii";
  const runs = S.sessions.filter(s => s.bat_start != null), used = runs.reduce((a, s) => a + Math.max(0, s.bat_start - s.bat_end), 0), mins = runs.reduce((a, s) => a + s.duration_s / 60, 0);
  if (S.chart === "daily") {
    const sum = drawDailyChart(cv, S.sessions, ACC);
    $("#c-legend").innerHTML = `<span>Energia na dzień [Wh] · 14 dni · łącznie <b style="color:var(--text)">${sum.toFixed(1)} Wh</b> · szacunek z % baterii (pakiet 5200 mAh / 14,4 V)</span>`;
  } else {
    drawSessionsChart(cv, S.sessions, ACC);
    $("#c-legend").innerHTML = `<span><span style="display:inline-block;width:14px;height:3px;background:var(--acc);vertical-align:middle"></span> ostatni przejazd</span><span><span style="display:inline-block;width:14px;height:3px;background:#3A4552;vertical-align:middle"></span> poprzednie</span><span>kliknij wiersz historii, żeby odtworzyć przejazd</span>`;
  }
  const sn = S.state?.sensors;
  $("#c-stats").innerHTML = cell("Śr. zużycie", mins > 1 ? (used / mins).toFixed(2) + " %/min" : "—") + cell("Zużyto łącznie", used + " %")
    + cell("Napięcie", sn?.mv ? (sn.mv / 1000).toFixed(2) + " V" : "—") + cell("Temp. baterii", sn?.temp != null ? sn.temp + " °C" : "—");
}
$("#c-modes").onclick = e => { const c = e.target.closest("[data-c]")?.dataset.c; if (c && c !== "replay") { S.chart = c; drawChart(); } };
$("#r-series").innerHTML = `<option value="">bez dodatkowej serii</option>` + Object.entries(SERIES).map(([k, v]) => `<option value="${k}">${v[0]}</option>`).join("");
$("#r-series").value = "br";
$("#r-series").onchange = drawChart;

let playTimer = null;
function stopPlay() { cancelAnimationFrame(playTimer); playTimer = null; $("#r-play").textContent = "▶ Odtwórz"; if (S.replay) S.replay.idx = Math.floor(S.replay.idx); }
function seek(i) { // i may be fractional while playing: the scene interpolates the pose, the rest follows whole frames
  const R = S.replay;
  R.pos = Math.max(0, Math.min(R.rows.length - 1, i));
  scene?.setReplay(R.rows, R.pos, R.events);
  if (Math.floor(R.pos) === R.idx && R.drawn) return;
  R.idx = Math.floor(R.pos); R.drawn = true;
  $("#r-pos").value = R.idx;
  const r = R.rows[R.idx];
  polar((r.scan ?? []).map(p => ({ angle: ((Math.atan2(p[1] - r.y, p[0] - r.x) * 180 / Math.PI - (r.a - 90)) % 360 + 360) % 360, dist: Math.hypot(p[0] - r.x, p[1] - r.y) })), "zapis przejazdu");
  drawChart();
}
async function openReplay(id) {
  const s = await loadSession(id);
  if (s.rows.length < 2) return alert("Ten przejazd jest za krótki, żeby go odtworzyć.");
  stopPlay();
  S.replay = { ...s, idx: 0 };
  $("#r-pos").max = s.rows.length - 1;
  scene?.setScan(null);
  loadSessions();
  seek(0);
  $("#chart").scrollIntoView({ behavior: "smooth", block: "center" });
}
$("#r-play").onclick = () => {
  if (playTimer) return stopPlay();
  if (S.replay.idx >= S.replay.rows.length - 1) seek(0);
  $("#r-play").textContent = "⏸ Pauza";
  let last = performance.now();
  const step = now => { // frames are 0.5 s apart: 10 * n frames per second = 5n x real time, drawn every display frame
    if (!S.replay || S.replay.pos >= S.replay.rows.length - 1) return stopPlay();
    seek(S.replay.pos + Math.min(0.1, (now - last) / 1000) * 10 * +$("#r-speed").value);
    last = now;
    playTimer = requestAnimationFrame(step);
  };
  playTimer = requestAnimationFrame(step);
};
$("#r-pos").oninput = e => seek(+e.target.value);
$("#r-fpv").onclick = () => setView(S.view === "fpv" ? "3d" : "fpv");
$("#r-close").onclick = () => { stopPlay(); S.replay = null; scene?.setReplay(null); loadSessions(); drawChart(); };
$("#chart").onpointerdown = e => {
  const c = $("#chart"), at = ev => S.replay && c._seek && seek(c._seek(ev.offsetX));
  at(e);
  c.onpointermove = ev => ev.buttons && at(ev);
};

// ---------- photos ----------
async function loadPhotos() {
  const list = await api("/local/photos");
  scene?.setPhotos(list);
  $("#ph-list").innerHTML = list.map(p => `<div style="display:flex;align-items:center;gap:10px;border:1px solid var(--line);padding:6px">
    <img src="/local/photos/${encodeURIComponent(p.file)}" alt="" style="width:64px;height:44px;object-fit:cover">
    <span style="flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis">${esc(p.name || "Zdjęcie")}<br><span class="mono" style="font-size:10px;color:var(--muted)">${Math.round(Math.hypot(p.x2 - p.x1, p.y2 - p.y1))} cm ściany</span></span>
    <button class="chip sm" data-del="${esc(p.id)}">Usuń</button></div>`).join("");
}
$("#ph-add").onclick = () => $("#ph-file").click();
$("#ph-file").onchange = async e => {
  const file = e.target.files[0];
  e.target.value = "";
  if (!file) return;
  const img = new Image();
  img.src = URL.createObjectURL(file);
  await img.decode().catch(() => null);
  if (!img.naturalWidth) return alert("Nie udało się odczytać obrazu.");
  const ph = { file, aspect: img.naturalHeight / img.naturalWidth, name: prompt("Podpis zdjęcia (np. Salon – ściana z oknem):", file.name.replace(/\.\w+$/, "")) ?? "" };
  URL.revokeObjectURL(img.src);
  startPick(["Kliknij na mapie POCZĄTEK ściany ze zdjęcia (lewy brzeg zdjęcia)", "Teraz kliknij KONIEC ściany (prawy brzeg zdjęcia)"], async ([[x1, y1], [x2, y2]]) => {
    const q = new URLSearchParams({ x1, y1, x2, y2, aspect: ph.aspect.toFixed(4), name: ph.name });
    await act(async () => { await api("/local/photos?" + q, { method: "POST", headers: { "Content-Type": ph.file.type }, body: ph.file }); await loadPhotos(); setView("3d"); });
  });
};

// clicking points on the map: photos, route plan, no-go zones, virtual walls, segment split
// hints: one per point; done(points) after the last one. hints = null -> open-ended (route), finished by its button
function startPick(hints, done, open = false) {
  S.pick = { hints, done, open, pts: [] };
  scene?.setMarks([]);
  if (S.view !== "2d") setView("2d");
  $("#pick-hint").textContent = hints[0] + " · Esc = anuluj";
  $("#scene").scrollIntoView({ behavior: "smooth", block: "center" });
}
function endPick() { S.pick = null; scene?.setMarks(S.route ?? []); $("#pick-hint").textContent = ""; }
addEventListener("keydown", e => { if (e.key === "Escape" && S.pick) endPick(); });
scene?.onPick(async (x, y) => {
  const pk = S.pick;
  if (!pk) return;
  pk.pts.push([x, y]);
  scene?.setMarks(pk.pts);
  if (pk.open) { pk.done(pk.pts); $("#pick-hint").textContent = `${pk.pts.length} pkt · ${pk.hints[0]}`; return; }
  if (pk.pts.length < pk.hints.length) { $("#pick-hint").textContent = pk.hints[pk.pts.length] + " · Esc = anuluj"; return; }
  endPick();
  await pk.done(pk.pts);
});
$("#ph-list").onclick = e => { const id = e.target.closest("[data-del]")?.dataset.del; if (id && confirm("Usunąć to zdjęcie?")) act(async () => { await api("/local/photos/delete?id=" + encodeURIComponent(id), { method: "POST" }); await loadPhotos(); }); };

// ---------- procedural control: manual pad, route, program ----------
const req = (method, path, body, confirm) => api(path, { method, headers: { "Content-Type": "application/json", ...(confirm ? { "X-Confirm": "tak" } : {}) },
  body: body === undefined ? undefined : JSON.stringify(body) });
const runProgram = steps => act(() => req("POST", "/local/program/start", steps));
const fmtStep = r => {
  const op = Object.entries(r.op).map(([k, v]) => k + (v === true ? "" : " " + JSON.stringify(v))).join(" ");
  const res = r.result ?? (r.front !== undefined ? `przód ${r.front ?? "—"} · prawo ${r.right ?? "—"} · tył ${r.back ?? "—"} · lewo ${r.left ?? "—"} cm` : r.error ?? "");
  const extra = r.moved_cm != null ? ` · ${r.moved_cm} cm` : r.turned_deg != null ? ` · ${r.turned_deg}°` : r.left_cm != null ? ` · ${r.left_cm} cm od celu` : "";
  return { op, txt: res + extra + (r.events.length ? ` · ⚠ ${r.events.join(", ")}` : ""), bad: !!r.events.length || /nie |błąd|utknął|zderzak|przeszkoda/.test(res) };
};
function showProgram(st) {
  const res = st.program?.results ?? [];
  const key = res.length + "|" + st.explore.msg;
  if (key === S.pgKey) return;
  S.pgKey = key;
  $("#pg-res").innerHTML = [...res].reverse().map(r => { const f = fmtStep(r); return `<li><span class="mono" style="font-size:12px;color:var(--muted)">krok ${r.step}</span>
    <span style="width:8px;height:8px;background:${f.bad ? "var(--orange)" : "var(--acc)"};margin-top:5px"></span><span style="font-size:12px"><b class="mono">${esc(f.op)}</b> → ${esc(f.txt)}</span></li>`; }).join("")
    || `<li><span></span><span></span><span class="kv">Tu pojawi się raport z każdego kroku.</span></li>`;
}
document.querySelectorAll("[data-rc]").forEach(b => { b.onclick = () => runProgram([JSON.parse(b.dataset.rc)]); });
$("#pg-stop").onclick = () => act(() => api("/local/explore/stop", { method: "POST" }));
$("#pg-run").onclick = () => { try { runProgram(JSON.parse($("#pg-text").value)); } catch (e) { alert("Błędny JSON: " + e.message); } };
api("/local/programs").then(list => {
  S.programs = list;
  $("#pg-saved").innerHTML += list.map(p => `<option value="${esc(p.name)}">${esc(p.title ?? p.name)}</option>`).join("");
}).catch(() => {});
$("#pg-saved").onchange = e => {
  const p = (S.programs ?? []).find(x => x.name === e.target.value);
  $("#pg-desc").textContent = p ? `${p.description ?? ""} (${p.steps.length} kroków)` : "";
  if (p) { $("#pg-text").value = JSON.stringify(p.steps); $("#pg-box").open = true; }
};
$("#pg-help").onclick = async () => { const d = $("#pg-doc"); d.style.display = d.style.display === "none" ? "block" : "none"; d.textContent = (await api("/local/program")).help; };
const rtInfo = () => { $("#rt-info").textContent = S.route?.length ? `${S.route.length} pkt` : ""; };
$("#rt-pick").onclick = () => { S.route = []; startPick(["klikaj kolejne punkty trasy, potem „Jedź trasą”"], pts => { S.route = [...pts]; rtInfo(); }, true); };
$("#rt-clear").onclick = () => { S.route = []; endPick(); rtInfo(); };
$("#rt-go").onclick = () => {
  if (!S.route?.length) return alert("Najpierw wskaż punkty trasy na mapie.");
  const steps = S.route.flatMap(([x, y]) => [{ goto: [x, y] }, { look: true }]).concat([{ home: true }]);
  endPick(); runProgram(steps);
};

// ---------- problem places + restrictions ----------
const RESTR = "/api/v2/robot/capabilities/CombinedVirtualRestrictionsCapability";
const rect = ([x1, y1, x2, y2]) => { [x1, x2] = [Math.min(x1, x2), Math.max(x1, x2)]; [y1, y2] = [Math.min(y1, y2), Math.max(y1, y2)];
  return { pA: { x: x1, y: y1 }, pB: { x: x2, y: y1 }, pC: { x: x2, y: y2 }, pD: { x: x1, y: y2 } }; };
async function saveRestr(fn) {
  const cur = await api(RESTR), r = { virtualWalls: cur.virtualWalls ?? [], restrictedZones: cur.restrictedZones ?? [] };
  fn(r);
  await req("PUT", RESTR, r);
  await loadHot();
}
async function loadHot() {
  const [hs, rs] = await Promise.all([api("/local/hotspots"), api(RESTR).catch(() => null)]);
  S.hot = hs; scene?.setHotspots(hs);
  const real = hs.filter(h => h.near_wall < 0.6);
  $("#hs-sum").textContent = `${real.length} poza mapą · ${hs.length - real.length} przy ścianach`;
  $("#hs-list").innerHTML = hs.slice(0, 25).map(h => `<div style="display:flex;gap:10px;align-items:center;border:1px solid var(--line);padding:6px 8px">
    <span class="mono" style="font-weight:700;color:${h.near_wall < 0.6 ? "var(--orange)" : "var(--mid)"};min-width:46px">⚠ ${h.id}</span>
    <span style="flex:1;min-width:0;font-size:12px">${h.n}× ${esc(Object.entries(h.kinds).map(([k, n]) => `${k} ${n}`).join(", "))} · ${h.days} ${h.days === 1 ? "dzień" : "dni"}<br>
      <span class="mono" style="font-size:10px;color:var(--muted)">${esc(h.what)} · (${h.x}, ${h.y}) · ostatnio ${new Date(h.last * 1000).toLocaleString("pl-PL", { day: "numeric", month: "numeric", hour: "2-digit", minute: "2-digit" })}</span></span>
    <button class="chip sm" data-go="${h.x},${h.y}" title="Wyślij robota w to miejsce">Jedź</button><button class="chip sm" data-block="${h.id}" title="Strefa zakazana wokół tego miejsca">Zablokuj</button></div>`).join("")
    || `<div class="kv">Brak powtarzających się problemów.</div>`;
  if (!rs) return;
  S.restr = rs;
  $("#rs-list").innerHTML = [...(rs.restrictedZones ?? []).map((z, i) => `<div>Strefa ${i + 1}: (${z.points.pA.x}, ${z.points.pA.y}) – (${z.points.pC.x}, ${z.points.pC.y}) <button class="chip sm" data-rdel="zone:${i}">Usuń</button></div>`),
    ...(rs.virtualWalls ?? []).map((w, i) => `<div>Ściana ${i + 1}: (${w.points.pA.x}, ${w.points.pA.y}) – (${w.points.pB.x}, ${w.points.pB.y}) <button class="chip sm" data-rdel="wall:${i}">Usuń</button></div>`)].join("")
    || "Brak stref zakazanych i wirtualnych ścian.";
}
$("#hs-list").onclick = e => {
  const g = e.target.closest("[data-go]")?.dataset.go, b = e.target.closest("[data-block]")?.dataset.block;
  if (g) { const [x, y] = g.split(",").map(Number); confirm(`Wysłać robota do (${x}, ${y})?`) && runProgram([{ goto: [x, y] }, { look: true }]); }
  if (b) { const h = S.hot.find(h => h.id === +b); confirm(`Dodać strefę zakazaną wokół miejsca ${b} (${h.zone[2] - h.zone[0]} × ${h.zone[3] - h.zone[1]} cm)? Robot przestanie tam wjeżdżać.`)
    && act(() => saveRestr(r => r.restrictedZones.push({ type: "regular", points: rect(h.zone) }))); }
};
$("#rs-list").onclick = e => { const d = e.target.closest("[data-rdel]")?.dataset.rdel; if (!d) return;
  const [k, i] = d.split(":"); act(() => saveRestr(r => (k === "zone" ? r.restrictedZones : r.virtualWalls).splice(+i, 1))); };
$("#rs-zone").onclick = () => startPick(["Kliknij JEDEN róg strefy zakazanej", "Kliknij PRZECIWLEGŁY róg"],
  ([[x1, y1], [x2, y2]]) => act(() => saveRestr(r => r.restrictedZones.push({ type: "regular", points: rect([x1, y1, x2, y2]) }))));
$("#rs-wall").onclick = () => startPick(["Kliknij POCZĄTEK wirtualnej ściany", "Kliknij KONIEC ściany"],
  ([[x1, y1], [x2, y2]]) => act(() => saveRestr(r => r.virtualWalls.push({ points: { pA: { x: x1, y: y1 }, pB: { x: x2, y: y2 } } }))));

// ---------- every Valetudo function ----------
const CAPP = "/api/v2/robot/capabilities/";
const two = n => String(n).padStart(2, "0");
const toLocal = t => { const d = new Date(); d.setUTCHours(t.hour, t.minute, 0, 0); return `${two(d.getHours())}:${two(d.getMinutes())}`; };
const toUTC = hm => { const [h, m] = hm.split(":").map(Number), d = new Date(); d.setHours(h, m, 0, 0); return { hour: d.getUTCHours(), minute: d.getUTCMinutes() }; };
const card = (title, html) => `<div style="border:1px solid var(--line);padding:12px;display:flex;flex-direction:column;gap:8px"><span class="lbl">${title}</span>${html}</div>`;
const inp = (id, v = "", w = "100%", type = "text") => `<input id="${id}" type="${type}" value="${esc(v)}" class="mono" style="width:${w};background:#0A0D10;color:var(--text);border:1px solid var(--line2);min-height:32px;padding:0 8px">`;
const DAYS = ["Nd", "Pn", "Wt", "Śr", "Cz", "Pt", "Sb"];

async function loadVF() {
  const g = p => api(p).catch(() => null);
  const caps = (await g("/api/v2/robot/capabilities")) ?? [];
  const has = c => caps.includes(c);
  const [dnd, vol, voice, segs, timers, tprops, snaps, pmap, quirks, vevents, loglvl, host, ver, props] = await Promise.all([
    has("DoNotDisturbCapability") && g(CAPP + "DoNotDisturbCapability"), has("SpeakerVolumeControlCapability") && g(CAPP + "SpeakerVolumeControlCapability"),
    has("VoicePackManagementCapability") && g(CAPP + "VoicePackManagementCapability"), g(CAPP + "MapSegmentationCapability"), g("/api/v2/timers"), g("/api/v2/timers/properties"),
    has("MapSnapshotCapability") && g(CAPP + "MapSnapshotCapability"), has("PersistentMapControlCapability") && g(CAPP + "PersistentMapControlCapability"),
    has("QuirksCapability") && g(CAPP + "QuirksCapability"), g("/api/v2/events"), g("/api/v2/valetudo/log/level"), g("/api/v2/system/host/info"), g("/api/v2/valetudo/version"), g("/api/v2/robot/properties")]);
  S.vf = { segs: segs ?? [], timers: Object.values(timers ?? {}), quirks: quirks ?? [] };
  const segOpts = (S.vf.segs).map(s => `<option value="${esc(s.id)}">${esc(s.name || "Pokój " + s.id)}</option>`).join("");
  const out = [];
  if (dnd) out.push(card("Nie przeszkadzać", `<label class="mono" style="font-size:12px"><input type="checkbox" id="vf-dnd-on" ${dnd.enabled ? "checked" : ""}> włączone</label>
    <div style="display:flex;gap:6px;align-items:center">od ${inp("vf-dnd-a", toLocal(dnd.start), "110px", "time")} do ${inp("vf-dnd-b", toLocal(dnd.end), "110px", "time")}</div>
    <button class="chip" id="vf-dnd-save">Zapisz</button>`));
  if (vol) out.push(card("Dźwięk", `<div style="display:flex;gap:8px;align-items:center"><input type="range" id="vf-vol" min="0" max="100" value="${vol.volume}" style="flex:1"><span class="mono" id="vf-vol-v">${vol.volume}%</span></div>
    <div style="display:flex;gap:6px"><button class="chip" id="vf-vol-save">Zapisz głośność</button>${has("SpeakerTestCapability") ? `<button class="chip" id="vf-test">Dźwięk testowy</button>` : ""}</div>
    ${voice ? `<span class="kv">Pakiet głosowy: <b>${esc(voice.currentLanguage)}</b> · ${esc(voice.operationStatus?.type)}${voice.operationStatus?.progress != null ? " " + voice.operationStatus.progress + "%" : ""}</span>
    <details><summary>Wgraj pakiet głosowy</summary>${inp("vf-vp-url", "", "100%")}<div style="display:flex;gap:6px;margin-top:4px">${inp("vf-vp-lang", "pl", "70px")}${inp("vf-vp-hash", "", "100%")}</div>
    <span class="kv">adres .tar.gz, kod języka, suma MD5</span><button class="chip" id="vf-vp-go">Pobierz i zainstaluj</button></details>` : ""}`));
  if (segs) out.push(card("Pokoje", `${S.vf.segs.map(s => `<div style="display:flex;gap:6px;align-items:center"><span class="mono" style="width:24px">${esc(s.id)}</span>${inp("vf-seg-" + s.id, s.name ?? "", "100%")}<button class="chip sm" data-ren="${esc(s.id)}">Zmień nazwę</button></div>`).join("")}
    ${has("MapSegmentEditCapability") ? `<div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap">Połącz <select id="vf-ja">${segOpts}</select> z <select id="vf-jb">${segOpts}</select><button class="chip sm" id="vf-join">Połącz</button></div>
    <div style="display:flex;gap:6px;align-items:center">Podziel <select id="vf-sp">${segOpts}</select><button class="chip sm" id="vf-split">Wskaż linię podziału</button></div>` : ""}`));
  out.push(card("Harmonogram", `${S.vf.timers.map(t => `<div style="display:flex;gap:6px;align-items:center;flex-wrap:wrap;font-size:12px"><input type="checkbox" data-ten="${esc(t.id)}" ${t.enabled ? "checked" : ""}>
      <b class="mono">${toLocal(t)}</b> ${t.dow.map(d => DAYS[new Date(Date.UTC(2024, 0, 7 + d, t.hour, t.minute)).getDay()]).join(" ")} · ${esc(t.label || t.action.type)}
      <button class="chip sm" data-trun="${esc(t.id)}">Teraz</button><button class="chip sm" data-tdel="${esc(t.id)}">Usuń</button></div>`).join("") || `<span class="kv">Brak zaplanowanych sprzątań.</span>`}
    <details><summary>Dodaj</summary><div style="display:flex;flex-direction:column;gap:6px;margin-top:6px">
      <div style="display:flex;gap:6px">${inp("vf-t-time", "10:00", "110px", "time")}${inp("vf-t-label", "", "100%")}</div>
      <div class="mono" style="font-size:11px;display:flex;flex-wrap:wrap;gap:6px">${[1, 2, 3, 4, 5, 6, 0].map(d => `<label><input type="checkbox" data-tday="${d}" checked> ${DAYS[d]}</label>`).join("")}</div>
      <div style="display:flex;gap:6px;flex-wrap:wrap"><select id="vf-t-rooms" multiple size="3" style="min-width:140px">${segOpts}</select>
        <select id="vf-t-fan"><option value="">moc: bez zmiany</option>${(S.info.fan ?? []).map(f => `<option>${esc(f)}</option>`).join("")}</select></div>
      <span class="kv">Bez zaznaczonych pokoi = całe mieszkanie.</span><button class="chip" id="vf-t-add">Dodaj sprzątanie</button></div></details>`));
  out.push(card("Mapa", `${pmap ? `<label class="mono" style="font-size:12px"><input type="checkbox" id="vf-pmap" ${pmap.enabled ? "checked" : ""}> trwała mapa (wyłączenie = mapa od zera przy każdym sprzątaniu)</label>` : ""}
    ${(snaps ?? []).map(sn => `<div style="font-size:12px">Kopia ${esc(sn.id)} · ${new Date(sn.timestamp).toLocaleString("pl-PL")} <button class="chip sm" data-snap="${esc(sn.id)}">Przywróć</button></div>`).join("")}
    ${has("MapResetCapability") ? `<button class="chip" id="vf-reset" style="color:var(--red);border-color:var(--red)">Usuń mapę (reset)</button>` : ""}`));
  out.push(card("Wi-Fi", `<span class="kv">${esc(S.state?.wifi?.ssid ?? "—")} · ${S.state?.wifi?.signal ?? "—"} dBm · ${esc((S.state?.wifi?.ips ?? [])[0] ?? "")}</span>
    <button class="chip" id="vf-scan">Skanuj sieci</button><div id="vf-scan-out" class="kv"></div>
    <details><summary>Zmień sieć (robot się rozłączy)</summary>${inp("vf-ssid", "", "100%")}<div style="margin-top:4px">${inp("vf-psk", "", "100%", "password")}</div>
      <span class="kv">SSID, hasło. Po zmianie robot dostanie nowy adres IP w nowej sieci.</span><button class="chip" id="vf-wifi-go">Zapisz</button></details>`));
  if (quirks?.length) out.push(card("Ustawienia firmware (quirks)", quirks.map((q, i) => `<div style="font-size:12px"><b>${esc(q.title)}</b><br><span class="kv">${esc(q.description)}</span><br>
    <select data-quirk="${i}">${q.options.map(o => `<option ${o === q.value ? "selected" : ""}>${esc(o)}</option>`).join("")}</select></div>`).join("")));
  out.push(card("Zdarzenia Valetudo", (vevents ?? []).filter(e => !e.processed).map(e => `<div style="font-size:12px">${esc(e.__class.replace("ValetudoEvent", ""))} · ${new Date(e.timestamp).toLocaleString("pl-PL")} ${esc(e.message ?? e.type ?? e.subType ?? "")}
    <button class="chip sm" data-vev="${esc(e.id)}">OK</button></div>`).join("") || `<span class="kv">Brak nowych zdarzeń.</span>`));
  out.push(card("Log i system", `<div style="display:flex;gap:6px;align-items:center">poziom logu <select id="vf-loglvl">${(loglvl?.presets ?? []).map(p => `<option ${p === loglvl.current ? "selected" : ""}>${p}</option>`).join("")}</select><button class="chip sm" id="vf-log">Pokaż log</button></div>
    <span class="kv">Valetudo <b>${esc(ver?.release ?? "?")}</b> · firmware <b>${esc(props?.firmwareVersion ?? "?")}</b><br>pamięć wolna <b>${host ? Math.round(host.mem.free / 1048576) : "—"} / ${host ? Math.round(host.mem.total / 1048576) : "—"} MB</b> · obciążenie <b>${host?.load?.["1"]?.toFixed(2) ?? "—"}</b> · działa <b>${host ? Math.round(host.uptime / 3600) : "—"} h</b></span>
    <pre id="vf-log-out" style="max-height:260px;overflow:auto;display:none"></pre>`));
  $("#vf").innerHTML = out.join("");
}

$("#vf-load").onclick = () => loadVF();
$("#vf").oninput = e => { if (e.target.id === "vf-vol") $("#vf-vol-v").textContent = e.target.value + "%"; };
$("#vf").onchange = e => {
  const t = e.target;
  if (t.dataset.ten) { const tm = S.vf.timers.find(x => x.id === t.dataset.ten); act(async () => { await req("PUT", "/api/v2/timers/" + tm.id, { ...tm, enabled: t.checked }); await loadVF(); }); }
  if (t.id === "vf-pmap") { if (confirm(t.checked ? "Włączyć trwałą mapę?" : "Wyłączyć trwałą mapę? Robot będzie budował mapę od zera przy każdym sprzątaniu, a strefy i pokoje przestaną działać."))
    act(async () => { await req("PUT", CAPP + "PersistentMapControlCapability", { action: t.checked ? "enable" : "disable" }, true); await loadVF(); }); else t.checked = !t.checked; }
  if (t.dataset.quirk) { const q = S.vf.quirks[+t.dataset.quirk];
    if (confirm(`Zmienić „${q.title}” na „${t.value}”? To ustawienie firmware robota.`)) act(async () => { await req("PUT", CAPP + "QuirksCapability", { id: q.id, value: t.value }, true); await loadVF(); }); else t.value = q.value; }
  if (t.id === "vf-loglvl") act(() => req("PUT", "/api/v2/valetudo/log/level", { level: t.value }));
};
$("#vf").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  const id = b.id, d = b.dataset;
  if (id === "vf-dnd-save") act(async () => { await req("PUT", CAPP + "DoNotDisturbCapability", { enabled: $("#vf-dnd-on").checked, start: toUTC($("#vf-dnd-a").value), end: toUTC($("#vf-dnd-b").value) }); await loadInfo(); await loadVF(); });
  if (id === "vf-vol-save") act(() => req("PUT", CAPP + "SpeakerVolumeControlCapability", { action: "set_volume", value: +$("#vf-vol").value }));
  if (id === "vf-test") act(() => req("PUT", CAPP + "SpeakerTestCapability", { action: "play_test_sound" }));
  if (id === "vf-vp-go" && confirm("Pobrać i zainstalować pakiet głosowy z podanego adresu?")) act(() => req("PUT", CAPP + "VoicePackManagementCapability", { action: "download", url: $("#vf-vp-url").value, language: $("#vf-vp-lang").value, hash: $("#vf-vp-hash").value }, true));
  if (d.ren) act(async () => { await req("PUT", CAPP + "MapSegmentRenameCapability", { action: "rename_segment", segment_id: d.ren, name: $("#vf-seg-" + d.ren).value }); await loadVF(); });
  if (id === "vf-join") { const a = $("#vf-ja").value, b2 = $("#vf-jb").value; if (a === b2) return alert("Wybierz dwa różne pokoje.");
    confirm("Połączyć te pokoje w jeden?") && act(async () => { await req("PUT", CAPP + "MapSegmentEditCapability", { action: "join_segments", segment_a_id: a, segment_b_id: b2 }); await loadVF(); }); }
  if (id === "vf-split") { const sid = $("#vf-sp").value;
    startPick(["Kliknij POCZĄTEK linii podziału pokoju", "Kliknij KONIEC linii podziału"], ([[x1, y1], [x2, y2]]) =>
      act(async () => { await req("PUT", CAPP + "MapSegmentEditCapability", { action: "split_segment", segment_id: sid, pA: { x: x1, y: y1 }, pB: { x: x2, y: y2 } }); await loadVF(); })); }
  if (d.trun) confirm("Uruchomić to sprzątanie teraz?") && act(() => req("PUT", `/api/v2/timers/${d.trun}/action`, { action: "execute_now" }));
  if (d.tdel) confirm("Usunąć to zaplanowane sprzątanie?") && act(async () => { await req("DELETE", "/api/v2/timers/" + d.tdel); await loadVF(); await loadInfo(); });
  if (id === "vf-t-add") {
    const [h, m] = $("#vf-t-time").value.split(":").map(Number), ref = new Date(2024, 0, 7, h, m); // 2024-01-07 = Sunday
    const shift = ref.getUTCDate() - 7, rooms = [...$("#vf-t-rooms").selectedOptions].map(o => o.value), fan = $("#vf-t-fan").value;
    const dow = [...document.querySelectorAll("[data-tday]:checked")].map(c => (+c.dataset.tday + shift + 7) % 7);
    if (!dow.length) return alert("Zaznacz co najmniej jeden dzień.");
    act(async () => { await req("POST", "/api/v2/timers", { enabled: true, label: $("#vf-t-label").value.slice(0, 24), dow, hour: ref.getUTCHours(), minute: ref.getUTCMinutes(),
      action: rooms.length ? { type: "segment_cleanup", params: { segment_ids: rooms, iterations: 1, custom_order: true } } : { type: "full_cleanup", params: {} },
      pre_actions: fan ? [{ type: "fan_speed_control", params: { value: fan } }] : [] }); await loadVF(); await loadInfo(); });
  }
  if (d.snap) confirm("Przywrócić tę kopię mapy? Obecna mapa zostanie zastąpiona.") && act(() => req("PUT", CAPP + "MapSnapshotCapability", { action: "restore", id: d.snap }, true));
  if (id === "vf-reset" && confirm("Usunąć mapę robota? Pokoje, strefy zakazane i nazwy pokoi znikną. Tego nie da się cofnąć.") && prompt("Wpisz USUŃ, żeby potwierdzić:") === "USUŃ")
    act(() => req("PUT", CAPP + "MapResetCapability", { action: "reset" }, true));
  if (id === "vf-scan") act(async () => { const l = await api(CAPP + "WifiScanCapability");
    $("#vf-scan-out").innerHTML = l.sort((a, b) => b.details.signal - a.details.signal).map(n => `<div>${esc(n.details.ssid || "(ukryta)")} · <b style="color:${signalColor(n.details.signal)}">${n.details.signal} dBm</b></div>`).join(""); });
  if (id === "vf-wifi-go" && confirm(`Przełączyć robota na sieć „${$("#vf-ssid").value}”? Jeśli hasło jest złe, robot zniknie z sieci i trzeba będzie go zresetować.`))
    act(() => req("PUT", CAPP + "WifiConfigurationCapability", { ssid: $("#vf-ssid").value, credentials: { type: "wpa2_psk", typeSpecificSettings: { password: $("#vf-psk").value } } }, true));
  if (d.vev) act(async () => { await req("PUT", `/api/v2/events/${d.vev}/interact`, { interaction: "ok" }); await loadVF(); });
  if (id === "vf-log") act(async () => { const r = await fetch("/api/v2/valetudo/log/content"), o = $("#vf-log-out");
    o.style.display = "block"; o.textContent = (await r.text()).split("\n").slice(-200).join("\n"); o.scrollTop = o.scrollHeight; });
};

// ---------- API console ----------
$("#cx-go").onclick = async () => {
  const m = $("#cx-m").value, body = $("#cx-b").value.trim();
  try {
    const r = await fetch($("#cx-p").value, { method: m, headers: { "Content-Type": "application/json", ...($("#cx-c").checked ? { "X-Confirm": "tak" } : {}) }, body: m === "GET" || m === "DELETE" ? undefined : body || "{}" });
    const t = await r.text();
    let pretty = t; try { pretty = JSON.stringify(JSON.parse(t), null, 2); } catch {}
    $("#cx-out").textContent = `HTTP ${r.status}\n` + pretty;
  } catch (e) { $("#cx-out").textContent = e.message; }
};

// ---------- raw API dump ----------
$("#api-load").onclick = async () => {
  const paths = ["robot", "valetudo/version", "system/host/info", "system/runtime/info", "robot/capabilities"];
  const caps = await api("/api/v2/robot/capabilities").catch(() => []);
  paths.push(...caps.map(c => "robot/capabilities/" + c));
  $("#api").innerHTML = "ładowanie…";
  const res = await Promise.all(paths.map(p => api("/api/v2/" + p).then(v => [p, v, null], e => [p, null, e.message])));
  $("#api").innerHTML = res.map(([p, v, err]) => `<details><summary>${esc(p)}${err ? " ⚠" : ""}</summary><pre>${esc(err ?? JSON.stringify(v, null, 2))}</pre></details>`).join("");
};

// ---------- loops ----------
let ticking = false;
async function tick() {
  if (ticking) return;
  ticking = true;
  try {
    S.state = await api("/local/state");
    S.lastOk = performance.now();
    $("#banner").dataset.offline = "";
    showState();
    if (S.state.map_nonce !== S.mapNonce) { S.mapNonce = S.state.map_nonce; await loadMap(); }
    if (S.state.cloud.rev !== S.cloudRev && performance.now() - S.cloudAt > 2500) await loadCloud();
  } catch (e) {
    console.error("tick:", e);
    if (performance.now() - S.lastOk > 4000) { $("#banner").style.display = "block"; $("#banner").textContent = "Brak połączenia z lokalnym serwerem panelu (server.py). Ponawiam… " + e.message; }
  } finally { ticking = false; }
}

await tick();
await Promise.allSettled([loadInfo(), loadCloud(), loadWifi(), loadSessions(), loadPhotos(), loadHot(), loadVF()]);
drawChart();
setInterval(tick, 500);
setInterval(() => loadScan().catch(() => {}), 400);
setInterval(() => loadWifi().catch(() => {}), 10000);
setInterval(() => loadInfo().then(drawParams).catch(() => {}), 20000);
setInterval(() => { if (!S.replay) loadSessions().catch(() => {}); }, 15000);
setInterval(drawParams, 2000);
setInterval(() => loadHot().catch(() => {}), 30000);
addEventListener("resize", drawChart);
