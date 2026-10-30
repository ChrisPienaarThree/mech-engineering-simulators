/* Cavitation Trainer — transparent pump with instrumented suction line, NPSH test, vibration,
   sound, and a damaged-impeller diagnosis exercise. Standalone, offline. Rig data illustrative. */
'use strict';

/* ------------------------------------------------------------------ physics */
const G = 9.81, PATM = 101325;
const PUMP = { N0: 2900, H0: 32, a1: 0.2, a2: 0.01, Qbep: 24, npshr: Q => 1.5 + 0.004 * Q * Q, vanes: 6 };
const D = 41.9e-3, A = Math.PI * D * D / 4;
const SUCTION = { L: 3, K: 2.5 };                 // fixed suction pipework
const SYSTEM = { Hs: 4, K: 6 };                    // discharge static head and fixed resistance
const DV = { Kv100: 25, R: 30 };                   // discharge valve, equal-percentage
function waterProps(T) { const rho = 1000 - 0.0035 * (T - 4) ** 2 * 0.65; const mu = 2.414e-5 * Math.pow(10, 247.8 / (T + 273.15 - 140)); const pv = 610.94 * Math.exp(17.625 * T / (T + 243.04)); return { rho, mu, pv }; }
function friction(Re) { if (Re < 2300) return Math.max(64 / Math.max(Re, 1), 0.02); return 0.25 / Math.pow(Math.log10(0.0015e-3 / (3.7 * D) + 5.74 / Math.pow(Re, 0.9)), 2); }
function pumpHead(Q, n) { if (n <= 0) return 0; const q = Q / n; return n * n * Math.max(0, PUMP.H0 - PUMP.a1 * q - PUMP.a2 * q * q); }
function runout(n) { const b = PUMP.a1, a = PUMP.a2; return n * (-b + Math.sqrt(b * b + 4 * a * PUMP.H0)) / (2 * a); }
function gateK(x) { const t = [[1, 0.2], [0.75, 1.15], [0.5, 5.6], [0.25, 24], [0.1, 100], [0.05, 400]]; x = Math.max(0.05, Math.min(1, x)); for (let i = 0; i < t.length - 1; i++) { const [x1, k1] = t[i], [x2, k2] = t[i + 1]; if (x <= x1 && x >= x2) { const u = (x1 - x) / (x1 - x2); return Math.exp(Math.log(k1) * (1 - u) + Math.log(k2) * u); } } return 400; }
function dvKv(x) { return DV.Kv100 * Math.pow(DV.R, x - 1); }
/* cavitation severity from NPSH margin: inception (audible, vibration) starts around 2.5 m of margin,
   3 % head drop at margin 0 (NPSH3 by definition), breakdown below that. */
function headFactor(m) { if (m >= 0.6) return 1; if (m >= 0) return 1 - 0.03 * (0.6 - m) / 0.6; return Math.max(0.25, 0.97 - 0.55 * Math.min(1, -m / 1.5)); }
function sevNoise(m) { return Math.max(0, Math.min(1, (2.5 - m) / 2.5)); }
function sevVib(m) { return Math.pow(Math.max(0, Math.min(1, (2.0 - m) / 2.6)), 1.4); }

function solve(s) {
  const fl = waterProps(s.temp), n = s.speedActual / 100;
  const Ks = SUCTION.K + gateK(s.throttle / 100), Kv = dvKv(s.dvalve / 100);
  const losses = Q => { const q = Q / 3600, v = q / A, Re = fl.rho * v * D / fl.mu, f = friction(Re); const hv = v * v / (2 * G); const hs = (f * SUCTION.L / D + Ks) * hv; const hd = (SYSTEM.K + f * 6 / D) * hv + Math.pow(Q / Kv, 2) * 1e5 / (fl.rho * G); return { hs, hd, v, hv }; };
  const npsha = Q => PATM / (fl.rho * G) + s.lift - losses(Q).hs - fl.pv / (fl.rho * G);
  const npshr = Q => n > 0 ? PUMP.npshr(Q / Math.max(n, 0.05)) * n * n : 0;
  let Q = 0, fac = 1;
  if (n > 0.02 && pumpHead(0, n) > SYSTEM.Hs) {
    for (let it = 0; it < 4; it++) {
      let lo = 0, hi = runout(n);
      for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; const L = losses(m); if (pumpHead(m, n) * fac > SYSTEM.Hs + L.hd) lo = m; else hi = m; }
      Q = (lo + hi) / 2; fac = headFactor(npsha(Q) - npshr(Q));
    }
  }
  const L = losses(Q), H = pumpHead(Q, n) * fac, Hclean = pumpHead(Q, n);
  const na = npsha(Q), nr = npshr(Q), margin = na - nr;
  const psuc = fl.rho * G * (s.lift - L.hs);                 // gauge Pa at the eye
  const sN = n > 0.05 ? sevNoise(margin) : 0, sV = n > 0.05 ? sevVib(margin) : 0;
  const vib = n > 0.02 ? 1.2 + 0.9 * n * n + 11 * sV : 0.1;         // mm/s RMS
  const dB = n > 0.02 ? 58 + 12 * n + 32 * sN : 32;
  return { fl, n, rpm: n * PUMP.N0, Q, H, Hclean, drop: Hclean > 0 ? (1 - H / Hclean) * 100 : 0, npsha: na, npshr: nr, margin, psuc, vib, dB, sN, sV, fac, Ks, Kv, v: L.v, hs: L.hs, pv: fl.pv,
           stage: n <= 0.05 ? 'off' : margin > 2.5 ? 'clean' : margin > 0.6 ? 'incipient' : margin > -1.2 ? 'developed' : 'severe' };
}

/* ------------------------------------------------------------------ state */
const state = { throttle: 100, temp: 20, lift: 0.5, speed: 100, speedActual: 100, dvalve: 70, log: [], sweep: null, hist: [], flags: {}, sound: false, diag: {} };
let R = solve(state);
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);

/* ------------------------------------------------------------------ rig svg */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent, anchor) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; if (anchor) t.style.textAnchor = anchor; return t; }
const rig = {};
const PC = { x: 560, y: 300, r: 118 };            // pump centre and volute radius
function gauge(svg, cx, cy, name, min, max, unit) { const g = el('g', { transform: `translate(${cx} ${cy})` }, svg); el('circle', { r: 26, class: 'gauge-face' }, g); for (let i = 0; i <= 8; i++) { const a = -135 + i * 33.75, r = Math.PI * a / 180; el('line', { x1: 18 * Math.sin(r), y1: -18 * Math.cos(r), x2: 22 * Math.sin(r), y2: -22 * Math.cos(r), class: 'gauge-tick' }, g); } const needle = el('line', { x1: 0, y1: 4, x2: 0, y2: -20, class: 'gauge-needle' }, g); el('circle', { r: 2.5, fill: 'var(--alarm)' }, g); txt(0, 38, name, 'gauge-name', g); const val = txt(0, 50, '', 'gauge-val', g); return { set(v) { const t = Math.max(0, Math.min(1, (v - min) / (max - min))); needle.setAttribute('transform', `rotate(${-135 + 270 * t})`); val.textContent = fmt(v, 1) + ' ' + unit; } }; }
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 560', role: 'img' }, $('#rig')); rig.svg = svg;
  // supply tank on the left; level relative to pump eye
  el('rect', { x: 40, y: 150, width: 180, height: 330, class: 'reservoir', rx: 4 }, svg);
  rig.fluid = el('rect', { x: 42, y: 300, width: 176, height: 178, class: 'fluid' }, svg);
  txt(130, 140, 'SUPPLY TANK · OPEN TO ATMOSPHERE', 'label', svg, 'middle');
  el('line', { x1: 40, y1: 300, x2: 220, y2: 300, class: 'tap', 'stroke-dasharray': '4 4' }, svg); txt(46, 296, 'pump-eye datum', 'scale-txt', svg);
  rig.levelTxt = txt(130, 168, '', 'gauge-val', svg, 'middle');
  // suction line: tank bottom → up/along to pump eye (pump eye at PC.x - PC.r)
  const eye = { x: PC.x - PC.r - 10, y: PC.y };
  const path = `M220 440 L300 440 L300 ${PC.y} L${eye.x} ${PC.y}`;
  el('path', { d: path, class: 'pipe-edge', 'stroke-width': 18 }, svg); el('path', { d: path, class: 'pipe', 'stroke-width': 14 }, svg); rig.sucF = el('path', { d: path, class: 'pipe-fluid', 'stroke-width': 10 }, svg);
  // suction throttle on the vertical run
  el('path', { d: 'M286 380 L314 400 L286 420 Z', class: 'valve-body' }, svg); el('path', { d: 'M314 380 L286 400 L314 420 Z', class: 'valve-body' }, svg);
  rig.gate = el('rect', { x: 298, y: 382, width: 4, height: 36, fill: 'var(--panel)' }, svg);
  el('line', { x1: 300, y1: 380, x2: 300, y2: 356, class: 'gauge-tick' }, svg); rig.stem = el('rect', { x: 292, y: 346, width: 16, height: 10, class: 'valve-body', rx: 2 }, svg);
  txt(322, 396, 'SUCTION THROTTLE', 'label', svg); rig.thTxt = txt(322, 410, '', 'gauge-val', svg, 'start');
  // vacuum gauge and thermometer on the suction line
  rig.gV = gauge(svg, 372, 230, 'VACUUM / SUCTION', -100, 50, 'kPa'); el('line', { x1: 372, y1: 258, x2: 372, y2: 292, class: 'tap' }, svg);
  el('rect', { x: 314, y: 236, width: 12, height: 40, class: 'sensor', rx: 6 }, svg); rig.therm = el('rect', { x: 317, y: 266, width: 6, height: 5, fill: 'var(--alarm)' }, svg); el('circle', { cx: 320, cy: 278, r: 6, fill: 'var(--alarm)' }, svg); el('line', { x1: 320, y1: 284, x2: 320, y2: 292, class: 'tap' }, svg);
  rig.tTxt = txt(320, 230, '', 'sensor-txt', svg); txt(320, 220, 'TEMPERATURE', 'gauge-name', svg);
  // transparent volute with impeller
  el('circle', { cx: PC.x, cy: PC.y, r: PC.r, class: 'volute' }, svg);
  el('circle', { cx: PC.x, cy: PC.y, r: PC.r - 12, fill: 'var(--water)', opacity: .18 }, svg);
  rig.imp = el('g', { transform: `translate(${PC.x} ${PC.y})` }, svg);
  for (let i = 0; i < PUMP.vanes; i++) el('path', { d: 'M18 0 Q 45 -20 88 -40', transform: `rotate(${i * 60})`, class: 'impeller', 'stroke-width': 4 }, rig.imp);
  el('circle', { cx: PC.x, cy: PC.y, r: 18, class: 'pump-body' }, svg);
  el('circle', { cx: PC.x, cy: PC.y, r: 40, fill: 'none', stroke: 'var(--alarm)', 'stroke-width': 1, 'stroke-dasharray': '3 3' }, svg); txt(PC.x, PC.y + 62, 'IMPELLER EYE', 'scale-txt', svg, 'middle');
  txt(PC.x, PC.y + PC.r + 28, 'TRANSPARENT VOLUTE · IMPELLER 6 VANES', 'label', svg, 'middle'); rig.rpmTxt = txt(PC.x, PC.y + PC.r + 42, '', 'gauge-val', svg, 'middle');
  // discharge: top of volute up and right → flowmeter → discharge valve
  const dis = `M${PC.x + 40} ${PC.y - PC.r + 8} L${PC.x + 40} 120 L860 120`;
  el('path', { d: dis, class: 'pipe-edge', 'stroke-width': 18 }, svg); el('path', { d: dis, class: 'pipe', 'stroke-width': 14 }, svg); rig.disF = el('path', { d: dis, class: 'pipe-fluid', 'stroke-width': 10 }, svg);
  el('rect', { x: 690, y: 98, width: 60, height: 44, class: 'flowmeter', rx: 5 }, svg); rig.rotor = el('g', { transform: 'translate(720 120)' }, svg); for (let i = 0; i < 4; i++) el('line', { x1: 0, y1: 0, x2: 13, y2: 0, transform: `rotate(${i * 90})`, class: 'rotor' }, rig.rotor);
  txt(720, 160, 'FLOWMETER', 'label', svg, 'middle'); rig.qTxt = txt(720, 174, '', 'sensor-txt', svg);
  el('path', { d: 'M796 106 L820 120 L796 134 Z', class: 'valve-body' }, svg); el('path', { d: 'M844 106 L820 120 L844 134 Z', class: 'valve-body' }, svg); txt(820, 160, 'DISCHARGE VALVE', 'label', svg, 'middle'); rig.dvTxt = txt(820, 174, '', 'sensor-txt', svg);
  // accelerometer on the bearing housing (right of volute) and microphone
  el('rect', { x: PC.x + PC.r + 6, y: PC.y - 14, width: 26, height: 28, class: 'sensor', rx: 3 }, svg); txt(PC.x + PC.r + 19, PC.y + 4, 'a', 'tag', svg, 'middle');
  txt(PC.x + PC.r + 60, PC.y - 24, 'ACCELEROMETER', 'label', svg); rig.vibTxt = txt(PC.x + PC.r + 60, PC.y - 10, '', 'sensor-txt', svg, 'start');
  rig.wave = el('polyline', { class: 'wave', points: '' }, svg); txt(PC.x + PC.r + 60, PC.y + 66, 'housing velocity, mm/s', 'scale-txt', svg);
  el('circle', { cx: 790, cy: 420, r: 12, class: 'sensor' }, svg); el('rect', { x: 788, y: 432, width: 4, height: 22, class: 'sensor' }, svg); txt(790, 406, 'MICROPHONE', 'label', svg, 'middle'); rig.dbTxt = txt(790, 474, '', 'sensor-txt', svg);
  rig.bars = []; for (let i = 0; i < 12; i++) rig.bars.push(el('rect', { x: 720 + i * 5, y: 500, width: 3, height: 2, fill: 'var(--good)' }, svg));
  txt(750, 520, 'sound level', 'scale-txt', svg, 'middle');
  // vapour bubbles at the eye and along the vane inlets
  rig.bub = el('g', {}, svg); for (let i = 0; i < 60; i++) el('circle', { class: 'vapour', r: 1.5, opacity: 0 }, rig.bub);
  // flow particles
  rig.parts = []; const pg = el('g', {}, svg); rig.sucPath = [[220, 440], [300, 440], [300, PC.y], [eye.x, PC.y]]; rig.disPath = [[PC.x + 40, PC.y - PC.r + 8], [PC.x + 40, 120], [860, 120]];
  for (let i = 0; i < 18; i++) rig.parts.push({ p: rig.sucPath, s: i / 18, e: el('circle', { r: 2.2, class: 'particle' }, pg) }); for (let i = 0; i < 18; i++) rig.parts.push({ p: rig.disPath, s: i / 18, e: el('circle', { r: 2.2, class: 'particle' }, pg) });
  txt(300, 470, 'SUCTION LINE 3 m DN40', 'label', svg, 'middle');
}
function pathLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); return L; }
function pointAt(pts, d) { for (let i = 1; i < pts.length; i++) { const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); if (d <= l) { const t = d / l; return [pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * t, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * t]; } d -= l; } return pts[pts.length - 1]; }
function updateRig(dt) {
  const s = state; const level = 300 - s.lift * 40; rig.fluid.setAttribute('y', Math.max(152, level)); rig.fluid.setAttribute('height', Math.max(0, 478 - Math.max(152, level))); rig.levelTxt.textContent = (s.lift >= 0 ? '+' : '') + s.lift.toFixed(1) + ' m to eye';
  const x = s.throttle / 100; rig.gate.setAttribute('height', 36 * x); rig.gate.setAttribute('y', 400 - 18 * x); rig.stem.setAttribute('y', 346 - 12 * x); rig.thTxt.textContent = s.throttle + ' % · K ' + fmt(R.Ks, 1);
  rig.gV.set(R.psuc / 1000); rig.tTxt.textContent = s.temp + ' °C'; rig.therm.setAttribute('height', 5 + (s.temp - 10) / 85 * 25); rig.therm.setAttribute('y', 271 - (5 + (s.temp - 10) / 85 * 25));
  rig.imAngle = (rig.imAngle || 0) + R.rpm / 60 * 360 * dt * 0.08; rig.imp.setAttribute('transform', `translate(${PC.x} ${PC.y}) rotate(${rig.imAngle % 360})`); rig.rpmTxt.textContent = Math.round(R.rpm) + ' rpm';
  rig.roAngle = (rig.roAngle || 0) + R.Q * 12 * dt; rig.rotor.setAttribute('transform', `translate(720 120) rotate(${rig.roAngle % 360})`); rig.qTxt.textContent = fmt(R.Q, 1) + ' m³/h'; rig.dvTxt.textContent = s.dvalve + ' % open';
  rig.vibTxt.textContent = fmt(R.vib, 1) + ' mm/s RMS'; rig.dbTxt.textContent = fmt(R.dB, 0) + ' dB';
  const flowing = R.Q > 0.1; [rig.sucF, rig.disF].forEach(e => e.style.opacity = flowing ? .75 : .25);
  for (const p of rig.parts) { const len = pathLen(p.p); p.s = (p.s + R.v * 60 * dt / len) % 1; const [px, py] = pointAt(p.p, p.s * len); p.e.setAttribute('cx', px); p.e.setAttribute('cy', py); p.e.style.opacity = flowing ? .85 : 0; }
  // vibration waveform
  const t0 = performance.now() / 1000, pts = []; const bx = PC.x + PC.r + 60, by = PC.y + 30;
  for (let i = 0; i < 90; i++) { const t = t0 + i * 0.0025; const base = Math.sin(2 * Math.PI * R.rpm / 60 * t) * 0.6 + Math.sin(2 * Math.PI * R.rpm / 60 * PUMP.vanes * t) * 0.25; const cav = (Math.random() - 0.5) * 2 * R.sV * 3 + (Math.random() < R.sV * 0.15 ? (Math.random() - 0.5) * 12 * R.sV : 0); const y = R.n > 0.02 ? (base * (0.6 + R.n) + cav) : 0; pts.push(`${bx + i * 1.5},${by - y * 3}`); } rig.wave.setAttribute('points', pts.join(' '));
  rig.bars.forEach((b, i) => { const lvl = (R.dB - 30) / 70; const h = Math.max(2, 40 * Math.min(1, lvl * (0.6 + 0.4 * Math.random()) * (1 - i * 0.03))); b.setAttribute('height', h); b.setAttribute('y', 500 - h); b.setAttribute('fill', i > 8 && lvl > 0.75 ? 'var(--alarm)' : i > 5 && lvl > 0.5 ? 'var(--warn)' : 'var(--good)'); });
  // vapour: bubbles born at the eye, carried along vanes, growing with severity
  const nb = Math.round(60 * Math.min(1, R.sN * 1.2)); rig.bub.querySelectorAll('circle').forEach((c, i) => { if (i >= nb) { c.setAttribute('opacity', 0); return; } const u = ((performance.now() / 700) + i * 0.37) % 1; const ang = (i * 61 + rig.imAngle) * Math.PI / 180; const rr = 22 + u * (R.stage === 'severe' ? 95 : 60); c.setAttribute('cx', PC.x + Math.cos(ang + u * 1.2) * rr); c.setAttribute('cy', PC.y + Math.sin(ang + u * 1.2) * rr); c.setAttribute('r', 1.2 + 3 * R.sN * (1 - u * 0.6)); c.setAttribute('opacity', 0.9 * (1 - u)); });
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement); const col = n => cs().getPropertyValue(n).trim();
function setup(c, ratio) { const dpr = window.devicePixelRatio || 1; const W = c.clientWidth || 900, H = W * ratio; if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); } const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H); return { ctx, W, H }; }
function drawNpshChart() {
  const { ctx, W, H } = setup($('#chart'), 360 / 900); const m = { l: 52, r: 16, t: 14, b: 36 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
  const Xmax = 12, Hmax = 40; const X = x => m.l + Math.max(0, Math.min(Xmax, x)) / Xmax * pw, Y = h => m.t + ph - Math.max(0, Math.min(Hmax, h)) / Hmax * ph;
  ctx.font = '11px ' + col('--mono'); ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.lineWidth = 1;
  ctx.textAlign = 'center'; for (let x = 0; x <= Xmax; x += 2) { ctx.beginPath(); ctx.moveTo(X(x), m.t); ctx.lineTo(X(x), m.t + ph); ctx.stroke(); ctx.fillText(x, X(x), H - m.b + 16); }
  ctx.textAlign = 'right'; for (let h = 0; h <= Hmax; h += 10) { ctx.beginPath(); ctx.moveTo(m.l, Y(h)); ctx.lineTo(m.l + pw, Y(h)); ctx.stroke(); ctx.fillText(h, m.l - 8, Y(h) + 4); }
  ctx.textAlign = 'center'; ctx.font = '600 12px ' + col('--body'); ctx.fillText('NPSH available at the eye  (m)', m.l + pw / 2, H - 4); ctx.save(); ctx.translate(14, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('Pump head  (m)', 0, 0); ctx.restore();
  if (R.n > 0.02) {
    const Hc = R.Hclean, nr = R.npshr; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 3; let f = true; for (let x = 0; x <= Xmax; x += 0.05) { const h = Hc * headFactor(x - nr); f ? ctx.moveTo(X(x), Y(h)) : ctx.lineTo(X(x), Y(h)); f = false; } ctx.stroke();
    ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 4]); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(X(nr), m.t); ctx.lineTo(X(nr), m.t + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.textAlign = 'left'; ctx.font = '600 11px ' + col('--mono'); ctx.fillText(`NPSH3 = NPSHr ${fmt(nr, 2)} m at ${fmt(R.Q, 1)} m³/h`, X(nr) + 6, m.t + 14);
    ctx.strokeStyle = col('--good'); ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(m.l, Y(Hc * 0.97)); ctx.lineTo(m.l + pw, Y(Hc * 0.97)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--good'); ctx.fillText('−3 % head', m.l + 6, Y(Hc * 0.97) - 4);
    for (const r of state.log) { ctx.fillStyle = col('--ink'); ctx.beginPath(); ctx.arc(X(r.npsha), Y(r.H), 4, 0, 7); ctx.fill(); }
    const pulse = 6 + 2 * Math.sin(performance.now() / 300); ctx.fillStyle = R.margin < 0 ? col('--alarm') : R.margin < 2.5 ? col('--warn') : col('--good'); ctx.beginPath(); ctx.arc(X(R.npsha), Y(R.H), pulse, 0, 7); ctx.fill(); ctx.fillStyle = col('--panel'); ctx.beginPath(); ctx.arc(X(R.npsha), Y(R.H), 2.5, 0, 7); ctx.fill();
    ctx.fillStyle = col('--ink'); ctx.textAlign = 'left'; ctx.fillText(`NPSHa ${fmt(R.npsha, 2)} m · H ${fmt(R.H, 1)} m · drop ${fmt(R.drop, 1)} %`, Math.min(X(R.npsha) + 12, m.l + pw - 220), Math.max(Y(R.H) - 12, m.t + 30));
  } else { ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.font = '600 14px ' + col('--body'); ctx.fillText('Pump stopped', m.l + pw / 2, m.t + ph / 2); }
}
function drawStrip() {
  const { ctx, W, H } = setup($('#strip'), 220 / 900); const m = { l: 40, r: 46, t: 10, b: 22 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const T = 40;
  ctx.strokeStyle = col('--grid'); ctx.lineWidth = 1; for (let i = 0; i <= 4; i++) { const y = m.t + ph * i / 4; ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(m.l + pw, y); ctx.stroke(); }
  ctx.fillStyle = col('--muted'); ctx.font = '10px ' + col('--mono'); ctx.textAlign = 'center'; for (let s = 0; s <= T; s += 10) ctx.fillText('−' + (T - s) + ' s', m.l + pw * s / T, H - 6);
  const now = performance.now() / 1000; const series = [['H', col('--water'), 0, 40, 'm'], ['vib', col('--brass'), 0, 16, 'mm/s'], ['dB', col('--good'), 30, 110, 'dB']];
  series.forEach(([k, c, lo, hi, u], si) => { ctx.beginPath(); ctx.strokeStyle = c; ctx.lineWidth = 2; let f = true; for (const h of state.hist) { const x = m.l + pw * (1 - (now - h.t) / T); if (x < m.l) continue; const y = m.t + ph - (h[k] - lo) / (hi - lo) * ph; f ? ctx.moveTo(x, y) : ctx.lineTo(x, y); f = false; } ctx.stroke(); ctx.fillStyle = c; ctx.textAlign = 'left'; ctx.fillText(`${k === 'H' ? 'H' : k === 'vib' ? 'vib' : 'dB'} ${fmt(R[k], k === 'dB' ? 0 : 1)} ${u}`, m.l + pw + 4, m.t + 12 + si * 14); });
}
function drawSpectrum() {
  const { ctx, W, H } = setup($('#spectrum'), 260 / 900); const m = { l: 44, r: 14, t: 12, b: 30 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const fmax = 2000;
  ctx.strokeStyle = col('--grid'); ctx.lineWidth = 1; ctx.fillStyle = col('--muted'); ctx.font = '10px ' + col('--mono'); ctx.textAlign = 'center';
  for (let f = 0; f <= fmax; f += 250) { const x = m.l + f / fmax * pw; ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke(); ctx.fillText(f + ' Hz', x, H - 8); }
  const f1 = R.rpm / 60, bins = 200, w = pw / bins; const peaks = [[f1, 0.55 * R.n], [2 * f1, 0.14 * R.n], [f1 * PUMP.vanes, 0.35 * R.n], [2 * f1 * PUMP.vanes, 0.12 * R.n]];
  for (let i = 0; i < bins; i++) { const f = (i + 0.5) / bins * fmax; let a = 0.02 + R.sV * (0.12 + 0.35 * Math.min(1, f / 900)) * (0.7 + 0.6 * Math.random()); for (const [pf, pa] of peaks) a += pa * Math.exp(-Math.pow((f - pf) / 14, 2)); a = Math.min(1, a); ctx.fillStyle = a > 0.5 ? col('--brass') : col('--water'); ctx.fillRect(m.l + i * w, m.t + ph - a * ph, Math.max(1, w - 1), a * ph); }
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px ' + col('--mono'); ctx.textAlign = 'left'; ctx.fillText(`1× ${fmt(f1, 1)} Hz · vane pass ${fmt(f1 * PUMP.vanes, 0)} Hz · broadband floor ${R.sV > 0.05 ? 'RISING — cavitation' : 'normal'}`, m.l + 6, m.t + 14);
}

/* ------------------------------------------------------------------ sound */
const audio = { ctx: null, gainHum: null, gainNoise: null, gainCrack: null, noise: null };
function initAudio() {
  const AC = window.AudioContext || window.webkitAudioContext; if (!AC) return false; const ac = new AC(); audio.ctx = ac;
  const master = ac.createGain(); master.gain.value = 0.9; master.connect(ac.destination);
  // hum: 1x and vane-pass tones
  const hum = ac.createGain(); hum.gain.value = 0; hum.connect(master); audio.gainHum = hum;
  audio.osc1 = ac.createOscillator(); audio.osc1.type = 'sawtooth'; audio.osc1.frequency.value = 48; const lp = ac.createBiquadFilter(); lp.type = 'lowpass'; lp.frequency.value = 400; audio.osc1.connect(lp); lp.connect(hum);
  audio.osc2 = ac.createOscillator(); audio.osc2.type = 'sine'; audio.osc2.frequency.value = 290; const g2 = ac.createGain(); g2.gain.value = 0.25; audio.osc2.connect(g2); g2.connect(hum); audio.osc1.start(); audio.osc2.start();
  // broadband cavitation noise
  const len = ac.sampleRate * 2, buf = ac.createBuffer(1, len, ac.sampleRate), d = buf.getChannelData(0); for (let i = 0; i < len; i++) d[i] = Math.random() * 2 - 1;
  const src = ac.createBufferSource(); src.buffer = buf; src.loop = true; const bp = ac.createBiquadFilter(); bp.type = 'bandpass'; bp.frequency.value = 2500; bp.Q.value = 0.6; const gn = ac.createGain(); gn.gain.value = 0; src.connect(bp); bp.connect(gn); gn.connect(master); src.start(); audio.gainNoise = gn;
  // crackle: random short bursts
  audio.crackBuf = buf; audio.master = master; return true;
}
function crackle() { if (!audio.ctx || !state.sound || R.sN < 0.05) return; const ac = audio.ctx; const n = Math.floor(R.sN * 6); for (let i = 0; i < n; i++) { const s = ac.createBufferSource(); s.buffer = audio.crackBuf; const g = ac.createGain(); const t = ac.currentTime + Math.random() * 0.1; g.gain.setValueAtTime(0, t); g.gain.linearRampToValueAtTime(0.25 * R.sN * (0.5 + Math.random()), t + 0.002); g.gain.exponentialRampToValueAtTime(0.001, t + 0.03 + Math.random() * 0.05); const hp = ac.createBiquadFilter(); hp.type = 'highpass'; hp.frequency.value = 1500 + Math.random() * 3000; s.connect(hp); hp.connect(g); g.connect(audio.master); s.start(t, Math.random() * 1.5, 0.12); } }
function updateAudio() { if (!audio.ctx) return; const on = state.sound && R.n > 0.02; const t = audio.ctx.currentTime; audio.gainHum.gain.setTargetAtTime(on ? 0.05 + 0.05 * R.n : 0, t, 0.1); audio.gainNoise.gain.setTargetAtTime(on ? 0.35 * Math.pow(R.sN, 1.5) : 0, t, 0.1); audio.osc1.frequency.setTargetAtTime(Math.max(10, R.rpm / 60), t, 0.1); audio.osc2.frequency.setTargetAtTime(Math.max(20, R.rpm / 60 * PUMP.vanes), t, 0.1); }
function toggleSound() { if (!audio.ctx) { if (!initAudio()) { alert('Web Audio is not available in this browser.'); return; } } if (audio.ctx.state === 'suspended') audio.ctx.resume(); state.sound = !state.sound; $('#btnSound').textContent = state.sound ? 'Sound on (M)' : 'Enable sound'; $('#btnSound').classList.toggle('on', state.sound); }

/* ------------------------------------------------------------------ readouts, status, log */
const READ = [
  ['npsha', 'NPSH available', () => fmt(R.npsha, 2), 'm'], ['npshr', 'NPSH required at this flow', () => fmt(R.npshr, 2), 'm'],
  ['margin', 'NPSH margin', () => fmt(R.margin, 2), 'm', () => R.n > 0.05 && R.margin < 0.6],
  ['psuc', 'Suction pressure at eye', () => fmt(R.psuc / 1000, 1), 'kPa gauge'], ['Q', 'Flow', () => fmt(R.Q, 1), 'm³/h'],
  ['H', 'Pump head', () => fmt(R.H, 1), 'm'], ['drop', 'Head loss from cavitation', () => fmt(R.drop, 1), '%', () => R.drop >= 3],
  ['vib', 'Vibration RMS', () => fmt(R.vib, 1), 'mm/s', () => R.vib > 7.1], ['dB', 'Sound level', () => fmt(R.dB, 0), 'dB'],
  ['pv', 'Vapour pressure', () => fmt(R.pv / 1000, 2), 'kPa abs'], ['pvh', 'Vapour pressure head', () => fmt(R.pv / (R.fl.rho * G), 2), 'm'],
  ['hs', 'Suction line losses', () => fmt(R.hs, 2), 'm'], ['Ks', 'Suction K total', () => fmt(R.Ks, 1), '—'], ['rpm', 'Pump speed', () => Math.round(R.rpm), 'rpm'],
  ['stage', 'Cavitation stage', () => ({ off: 'pump off', clean: 'none', incipient: 'incipient', developed: 'developed', severe: 'severe' }[R.stage]), '', () => R.stage === 'developed' || R.stage === 'severe'],
];
const TOP = ['npsha', 'npshr', 'margin', 'psuc', 'Q', 'H', 'vib', 'dB'];
function buildReadouts() { const mk = (box, keys) => { box.innerHTML = ''; for (const r of READ) { if (keys && !keys.includes(r[0])) continue; const d = document.createElement('div'); d.className = 'ro'; d.dataset.k = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u"></span>`; box.appendChild(d); } }; mk($('#readouts'), TOP); mk($('#dataGrid'), null); }
function updateReadouts() { document.querySelectorAll('.ro').forEach(d => { const r = READ.find(x => x[0] === d.dataset.k); d.querySelector('.v').textContent = r[2](); d.querySelector('.u').textContent = r[3]; d.classList.toggle('alarm', !!(r[4] && r[4]())); }); }
function updateStatus() {
  const run = R.n > 0.02; $('#pillRun').textContent = run ? `Pump running · ${Math.round(R.rpm)} rpm · ${fmt(R.Q, 1)} m³/h` : 'Pump stopped'; $('#pillRun').classList.toggle('on', run);
  $('#pillNpsh').textContent = run ? `NPSHa ${fmt(R.npsha, 2)} m · NPSHr ${fmt(R.npshr, 2)} m · margin ${fmt(R.margin, 2)} m` : 'NPSH margin —';
  $('#pillIncip').classList.toggle('hidden', R.stage !== 'incipient'); $('#pillCav').classList.toggle('hidden', R.stage !== 'developed'); $('#pillCav').textContent = `Developed cavitation · head down ${fmt(R.drop, 1)} %`; $('#pillSevere').classList.toggle('hidden', R.stage !== 'severe');
  state.flags.incip = state.flags.incip || R.stage === 'incipient'; state.flags.dev = state.flags.dev || R.stage === 'developed' || R.stage === 'severe';
}
function record(tag) { const r = { i: state.log.length + 1, th: state.throttle, T: state.temp, lift: state.lift, psuc: +(R.psuc / 1000).toFixed(1), npsha: +R.npsha.toFixed(2), npshr: +R.npshr.toFixed(2), Q: +R.Q.toFixed(2), H: +R.H.toFixed(2), drop: +R.drop.toFixed(1), vib: +R.vib.toFixed(1), dB: +R.dB.toFixed(0), tag }; state.log.push(r); renderLog(); checkTasks(); }
function renderLog() { const tb = $('#logTable tbody'); tb.innerHTML = ''; for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.th, r.T, r.lift, r.psuc, r.npsha, r.npshr, r.Q, r.H, r.drop, r.vib, r.dB].map(v => `<td>${v}</td>`).join(''); if (r.drop >= 3) tr.style.color = 'var(--alarm)'; tb.appendChild(tr); } $('#logCount').textContent = state.log.length; const csv = ['n,throttle_pct,T_C,level_m,suction_kPa_g,NPSHa_m,NPSHr_m,Q_m3h,H_m,head_drop_pct,vib_mms,sound_dB'].concat(state.log.map(r => [r.i, r.th, r.T, r.lift, r.psuc, r.npsha, r.npshr, r.Q, r.H, r.drop, r.vib, r.dB].join(','))).join('\n'); state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv); }

/* ------------------------------------------------------------------ damaged impeller samples */
function rng(seed) { let s = seed; return () => { s = (s * 1664525 + 1013904223) % 4294967296; return s / 4294967296; }; }
const MECH = { cavitation: 'Cavitation', erosion: 'Abrasive erosion', corrosion: 'Corrosion', recirculation: 'Suction recirculation' };
const SAMPLES = [
  { id: 'cav', mech: 'cavitation', evidence: { 'Where': 'Suction (low-pressure) face of each vane, just behind the leading edge, within 20 mm of the eye. Rest of the vane and the shrouds are clean.', 'Surface': 'Deep, sharp-edged, sponge-like craters; crystalline appearance; no directional pattern.', 'Fluid': 'Clean treated water, 65 °C, no solids.', 'History': 'Long suction line from a tank below the pump; operators reported a "gravel" noise for months; head had dropped about 4 %.' },
    fb: 'Sponge-like pitting concentrated on the suction side of the vane inlet, clean metal elsewhere, a low-margin suction arrangement and a noise history all point to classical cavitation. Recirculation would sit on the pressure side; erosion would be smooth and directional; corrosion would attack everywhere.' },
  { id: 'ero', mech: 'erosion', evidence: { 'Where': 'Vane tips and outer third of the pressure face, plus the shroud near the outlet; tips visibly thinned and rounded.', 'Surface': 'Smooth, polished, with shallow grooves running in the flow direction; edges rounded, not cratered.', 'Fluid': 'Mine water with 2–4 % fine silica sand.', 'History': 'NPSH margin always above 4 m; no unusual noise; wear rate tracked the sand content of the feed.' },
    fb: 'Smooth, polished, directional wear at the high-velocity outlet, thinned tips and an abrasive feed are abrasive erosion. Cavitation and recirculation pit rather than polish, and sit at the inlet; corrosion is uniform and not directional.' },
  { id: 'cor', mech: 'corrosion', evidence: { 'Where': 'Every wetted surface uniformly — vane faces, hub, both shrouds, even the back face where flow is stagnant.', 'Surface': 'Fine, shallow, uniform pitting under a brown-red deposit; metal dull and roughened; no relation to flow direction or pressure side.', 'Fluid': 'Untreated seawater with periodic acid dosing.', 'History': 'NPSH margin ample; vibration normal; the casing and the suction pipe show the same attack.' },
    fb: 'Uniform attack on all wetted surfaces, including stagnant ones the flow never scours, plus discolouration and an aggressive fluid, is corrosion. Flow-driven mechanisms leave a location signature; corrosion does not.' },
  { id: 'rec', mech: 'recirculation', evidence: { 'Where': 'Pressure face of the vanes at the inlet, and the front shroud right at the eye; the suction face is clean.', 'Surface': 'Sponge-like pitting, similar in texture to cavitation, but on the opposite side of the vane and extending onto the shroud at the eye.', 'Fluid': 'Clean process water, 25 °C.', 'History': 'Pump oversized: ran for two years at 25–35 % of BEP against a throttled discharge. NPSH margin measured at 5 m. Low-frequency pressure pulsations and a random knocking noise reported at low flow.' },
    fb: 'Cavitation-type pitting on the pressure side of the vane inlet and on the shroud at the eye, with ample NPSH margin but a long history of running far below BEP, is suction recirculation damage: vapour formed in the recirculating vortices at the eye, not from low suction pressure.' },
];
function impellerSVG(sample) {
  const svg = el('svg', { viewBox: '0 0 300 300' }); const g = el('g', { transform: 'translate(150 150)' }, svg);
  el('circle', { r: 130, class: 'metal' }, g); el('circle', { r: 36, class: 'metal' }, g);
  const vanes = []; for (let i = 0; i < 6; i++) { const p = el('path', { d: 'M38 0 Q 75 -28 126 -52', transform: `rotate(${i * 60})`, fill: 'none', stroke: '#6b7a8c', 'stroke-width': 9, 'stroke-linecap': 'round' }, g); vanes.push(p); }
  const r = rng(sample.id.charCodeAt(0) * 97 + sample.id.charCodeAt(2));
  const along = (i, u) => { const a = i * Math.PI / 3; const x = 38 + u * 88, y = -(u * 52 + 4 * Math.sin(u * Math.PI) * 6); return [x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)]; };
  const norm = (i, u, side) => { const a = i * Math.PI / 3 + Math.atan2(-52, 88) + (side === 'suction' ? -Math.PI / 2 : Math.PI / 2); return [Math.cos(a), Math.sin(a)]; };
  if (sample.mech === 'cavitation') for (let i = 0; i < 6; i++) for (let k = 0; k < 28; k++) { const u = 0.05 + r() * 0.3; const [x, y] = along(i, u), [nx, ny] = norm(i, u, 'suction'); const d = 5 + r() * 6; el('circle', { cx: x + nx * d, cy: y + ny * d, r: 1 + r() * 2.2, class: 'pit pit-cav' }, g); }
  if (sample.mech === 'recirculation') { for (let i = 0; i < 6; i++) for (let k = 0; k < 22; k++) { const u = 0.02 + r() * 0.25; const [x, y] = along(i, u), [nx, ny] = norm(i, u, 'pressure'); const d = 5 + r() * 6; el('circle', { cx: x + nx * d, cy: y + ny * d, r: 1 + r() * 2.2, class: 'pit pit-cav' }, g); } for (let k = 0; k < 90; k++) { const a = r() * Math.PI * 2, rr = 38 + r() * 14; el('circle', { cx: Math.cos(a) * rr, cy: Math.sin(a) * rr, r: 0.8 + r() * 1.8, class: 'pit' }, g); } }
  if (sample.mech === 'erosion') { for (let i = 0; i < 6; i++) { for (let k = 0; k < 5; k++) { const u1 = 0.45 + k * 0.1, u2 = 0.98; const [x1, y1] = along(i, u1), [x2, y2] = along(i, u2); const [nx, ny] = norm(i, 0.7, 'pressure'); const d = 1 + k * 1.2 - 3; el('line', { x1: x1 + nx * d, y1: y1 + ny * d, x2: x2 + nx * d, y2: y2 + ny * d, class: 'groove' }, g); } const [tx, ty] = along(i, 1); el('circle', { cx: tx, cy: ty, r: 7, fill: '#c9ced6', stroke: '#eef2f6', 'stroke-width': 2, 'stroke-dasharray': '2 2' }, g); } for (let k = 0; k < 40; k++) { const a = r() * Math.PI * 2, rr = 112 + r() * 16; el('line', { x1: Math.cos(a) * rr, y1: Math.sin(a) * rr, x2: Math.cos(a + 0.12) * (rr + 3), y2: Math.sin(a + 0.12) * (rr + 3), class: 'groove' }, g); } }
  if (sample.mech === 'corrosion') { el('circle', { r: 130, class: 'rust' }, g); for (let k = 0; k < 900; k++) { const a = r() * Math.PI * 2, rr = r() * 130; el('circle', { cx: Math.cos(a) * rr, cy: Math.sin(a) * rr, r: 0.5 + r() * 1.1, class: 'pit' }, g); } }
  txt(150, 292, 'eye at centre · flow outward · rotation clockwise', 'scale-txt', svg, 'middle');
  return svg;
}
function buildSamples() {
  const box = $('#samples'); box.innerHTML = ''; state.diag = {}; const order = SAMPLES.slice(); const r = rng(Date.now() % 100000); for (let i = order.length - 1; i > 0; i--) { const j = Math.floor(r() * (i + 1)); [order[i], order[j]] = [order[j], order[i]]; }
  order.forEach((smp, k) => { const d = document.createElement('div'); d.className = 'sample'; d.innerHTML = `<h3>Sample ${String.fromCharCode(65 + k)}</h3>`; d.appendChild(impellerSVG(smp)); const dl = document.createElement('dl'); dl.className = 'evid'; for (const [kk, v] of Object.entries(smp.evidence)) dl.innerHTML += `<dt>${kk}</dt><dd>${v}</dd>`; d.appendChild(dl); const ch = document.createElement('div'); ch.className = 'choices'; const fb = document.createElement('div'); fb.className = 'feedback'; fb.textContent = 'Name the mechanism from the evidence.';
    for (const [key, name] of Object.entries(MECH)) { const b = document.createElement('button'); b.textContent = name; b.onclick = () => { if (state.diag[smp.id] !== undefined) return; const ok = key === smp.mech; state.diag[smp.id] = ok; ch.querySelectorAll('button').forEach(x => { x.classList.toggle('right', x.textContent === MECH[smp.mech]); if (x === b && !ok) x.classList.add('wrong'); }); fb.className = 'feedback ' + (ok ? 'ok' : 'no'); fb.textContent = (ok ? 'Correct. ' : `Not this one — it is ${MECH[smp.mech].toLowerCase()}. `) + smp.fb; updateDiag(); checkTasks(); }; ch.appendChild(b); }
    d.appendChild(ch); d.appendChild(fb); box.appendChild(d); }); updateDiag();
}
function updateDiag() { const n = Object.keys(state.diag).length, ok = Object.values(state.diag).filter(Boolean).length; $('#diagScore').textContent = `${n} / 4 diagnosed · ${ok} correct`; }

/* ------------------------------------------------------------------ tasks */
const TASKS = [
  { id: 'hear', t: 'Hear it before you see it in the head', d: 'Enable sound. With the discharge valve at 70 %, close the suction throttle slowly. Note the throttle position where the crackle and the accelerometer floor start rising — the head has not moved yet.', ok: () => !!state.flags.incip && state.sound },
  { id: 'npsh3', t: 'Find NPSH3', d: 'Record points as you reduce NPSHa at a fixed discharge-valve setting until the head has dropped 3 %. Read NPSH3 off the chart and compare with NPSHr at that flow.', ok: () => state.log.some(r => r.drop >= 3) && state.log.some(r => r.drop < 1), st: () => { const a = state.log.find(r => r.drop >= 3); return a ? `first ≥3 % point at NPSHa ${a.npsha} m (NPSHr ${a.npshr} m)` : `${state.log.length} points so far`; } },
  { id: 'temp', t: 'Temperature alone can do it', d: 'Open the suction throttle fully, then raise the fluid temperature until cavitation develops. Nothing mechanical changed — only the vapour pressure.', ok: () => state.log.some(r => r.th >= 95 && r.T >= 60 && r.drop >= 3) || (state.throttle >= 95 && state.temp >= 60 && R.drop >= 3) },
  { id: 'lift', t: 'Suction lift', d: 'Cool the fluid, open the throttle, and lower the tank level below the pump eye until the margin goes negative. Read the vacuum gauge as it happens.', ok: () => state.lift < 0 && R.margin < 0 && state.temp <= 30 && state.throttle >= 95 },
  { id: 'recover', t: 'Recover it', d: 'From developed cavitation, restore a margin above 2.5 m by opening the throttle, cooling the fluid or raising the level. Watch the order in which sound, vibration and head come back.', ok: () => !!state.flags.dev && R.stage === 'clean' },
  { id: 'diag', t: 'Diagnose all four impellers', d: 'In the Damaged impellers tab, name the mechanism for every sample from the location and texture of the damage and the service history.', ok: () => Object.keys(state.diag).length === 4, st: () => Object.keys(state.diag).length ? `${Object.values(state.diag).filter(Boolean).length} of ${Object.keys(state.diag).length} correct` : '' },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (t.ok()) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0, crackTimer = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  const ramp = 40 * dt; state.speedActual += Math.max(-ramp, Math.min(ramp, state.speed - state.speedActual)); if (Math.abs(state.speed - state.speedActual) < 0.05) state.speedActual = state.speed;
  if (state.sweep) { const sw = state.sweep; sw.t += dt; const step = Math.floor(sw.t / 1.6); state.throttle = Math.max(8, Math.round(100 - step * 6)); $('#throttle').value = state.throttle; if (step > sw.rec && sw.t - step * 1.6 > 1.2) { record('auto'); sw.rec = step; if (R.drop >= 6 || state.throttle <= 8) { state.sweep = null; $('#btnSweep').textContent = 'Auto-run NPSH test'; } } }
  R = solve(state);
  state.hist.push({ t: now / 1000, H: R.H, vib: R.vib, dB: R.dB }); while (state.hist.length && now / 1000 - state.hist[0].t > 41) state.hist.shift();
  updateRig(dt); drawNpshChart(); drawStrip(); if ($('#tab-spec').classList.contains('on')) drawSpectrum();
  crackTimer += dt; if (crackTimer > 0.1) { crackTimer = 0; crackle(); } updateAudio();
  if (frame++ % 6 === 0) { updateReadouts(); updateStatus(); checkTasks(); $('#throttleOut').textContent = state.throttle + ' %'; $('#tempOut').textContent = state.temp + ' °C'; $('#liftOut').textContent = (state.lift >= 0 ? '+' : '') + state.lift.toFixed(1) + ' m'; $('#speedOut').textContent = state.speed + ' %'; $('#rpmOut').textContent = Math.round(R.rpm) + ' rpm'; $('#dvalveOut').textContent = state.dvalve + ' %'; }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function bind() {
  const on = (id, fn) => $('#' + id).addEventListener('input', e => fn(e.target.value));
  on('throttle', v => { state.throttle = +v; state.sweep = null; }); on('temp', v => state.temp = +v); on('lift', v => state.lift = +v); on('speed', v => state.speed = +v); on('dvalve', v => state.dvalve = +v);
  document.querySelectorAll('[data-th]').forEach(b => b.onclick = () => { state.throttle = +b.dataset.th; $('#throttle').value = state.throttle; state.sweep = null; });
  document.querySelectorAll('[data-temp]').forEach(b => b.onclick = () => { state.temp = +b.dataset.temp; $('#temp').value = state.temp; });
  document.querySelectorAll('[data-speed]').forEach(b => b.onclick = () => { state.speed = +b.dataset.speed; $('#speed').value = state.speed; });
  $('#btnRecord').onclick = () => record('manual'); $('#btnSound').onclick = toggleSound;
  $('#btnSweep').onclick = () => { if (state.sweep) { state.sweep = null; $('#btnSweep').textContent = 'Auto-run NPSH test'; return; } state.throttle = 100; $('#throttle').value = 100; if (state.speed < 5) { state.speed = 100; $('#speed').value = 100; } state.sweep = { t: 0, rec: -1 }; $('#btnSweep').textContent = 'Stop test'; document.querySelector('[data-tab="log"]').click(); };
  $('#btnReset').onclick = () => { Object.assign(state, { throttle: 100, temp: 20, lift: 0.5, speed: 100, dvalve: 70, sweep: null, flags: {} }); ['throttle', 'temp', 'lift', 'speed', 'dvalve'].forEach(id => $('#' + id).value = state[id]); document.querySelectorAll('.tasks li').forEach(l => l.classList.remove('done')); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); }; $('#btnShuffle').onclick = buildSamples;
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('ct_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT'].includes(e.target.tagName)) return; const st = e.shiftKey ? 10 : 2;
    if (e.key === 'ArrowLeft') { state.throttle = Math.max(8, state.throttle - st); $('#throttle').value = state.throttle; } else if (e.key === 'ArrowRight') { state.throttle = Math.min(100, state.throttle + st); $('#throttle').value = state.throttle; }
    else if (e.key === 'ArrowUp') { state.temp = Math.min(95, state.temp + st); $('#temp').value = state.temp; } else if (e.key === 'ArrowDown') { state.temp = Math.max(10, state.temp - st); $('#temp').value = state.temp; }
    else if (e.key === 'r' || e.key === 'R') record('manual'); else if (e.key === 's' || e.key === 'S') $('#btnSweep').click(); else if (e.key === 'm' || e.key === 'M') toggleSound(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawNpshChart(); drawStrip(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.CavitationTrainer = {
  getState: () => ({ inputs: { throttle: state.throttle, temp: state.temp, lift: state.lift, speed: state.speed, dvalve: state.dvalve }, results: { npsha: R.npsha, npshr: R.npshr, margin: R.margin, Q: R.Q, H: R.H, drop: R.drop, vib: R.vib, dB: R.dB, stage: R.stage }, diagnosis: state.diag, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['throttle', 'temp', 'lift', 'speed', 'dvalve']) if (inp[k] !== undefined) { state[k] = +inp[k]; $('#' + k).value = state[k]; } },
  meta: { id: 'cavitation-trainer', title: 'Cavitation Trainer', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.CavitationTrainer.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'cavitation-trainer', state: window.CavitationTrainer.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('ct_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildReadouts(); buildTasks(); buildSamples(); bind(); renderLog();
  R = solve(state); requestAnimationFrame(tick);
}
build();
