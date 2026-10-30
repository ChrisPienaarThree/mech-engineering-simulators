/* Hydraulic Loop Bench — transparent recirculating pump test rig simulator.
   Standalone, offline. Physics: affinity laws, Darcy–Weisbach, Kv valve model, NPSH.
   Pump data is illustrative (training only). */
'use strict';

/* ------------------------------------------------------------------ physics */
const G = 9.81;
const PUMP = {              // [Illustrative pump data]
  N0: 2900,                 // rated speed rpm
  H0: 32, a1: 0.2, a2: 0.01, // H = H0 - a1 Q - a2 Q^2  (Q in m3/h, H in m) at N0
  Qbep: 24, etaMax: 0.72,
  motorKW: 4.0, motorEta: 0.88, overloadKW: 4.4,
  npshr: Q => 1.5 + 0.004 * Q * Q, // m at rated speed
};
const PIPES = { 25: 27.3e-3, 32: 36.0e-3, 40: 41.9e-3, 50: 53.1e-3 }; // bore m
const VALVE = { Kv100: 25, R: 30 };  // equal-percentage globe, m3/h at 1 bar
const BASE = { L: 4, D: PIPES[40], Kfit: 6 };  // fixed pipework: 4 m DN40 + fittings
const RETURN_HEIGHT = 2.2; // return outlet height above pump centreline (m)
const ROUGH = 0.0015e-3;   // clear acrylic ~ smooth

function waterProps(T) {  // rho kg/m3, mu Pa.s, pv Pa (Antoine-ish fits, 10-85 C)
  const rho = 1000 - 0.0035 * (T - 4) ** 2 * 0.65;
  const mu = 2.414e-5 * Math.pow(10, 247.8 / (T + 273.15 - 140));
  const pv = 610.94 * Math.exp(17.625 * T / (T + 243.04));
  return { rho, mu, pv };
}
function friction(Re, D) {
  if (Re < 1) return 64;
  if (Re < 2300) return 64 / Re;
  const f = 0.25 / Math.pow(Math.log10(ROUGH / (3.7 * D) + 5.74 / Math.pow(Re, 0.9)), 2);
  if (Re < 4000) { const t = (Re - 2300) / 1700; return (64 / 2300) * (1 - t) + f * t; }
  return f;
}
function pumpHead(Q, n) { // n = N/N0
  if (n <= 0) return 0;
  const q = Q / n; return n * n * Math.max(0, PUMP.H0 - PUMP.a1 * q - PUMP.a2 * q * q);
}
function pumpEta(Q, n) {
  if (n <= 0 || Q <= 0) return 0;
  const q = Q / n; return Math.max(0.03, PUMP.etaMax * (1 - Math.pow((q - PUMP.Qbep) / PUMP.Qbep, 2)));
}
function runout(n) { const b = PUMP.a1, a = PUMP.a2, c = -PUMP.H0; return n * (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a); }
function valveKv(x) { return x < 0.02 ? 0 : VALVE.Kv100 * Math.pow(VALVE.R, x - 1); }

function systemLosses(Q, s, fl) { // returns {hf_test, hf_base, hv, hs, v_test, v_base, Re_test, f_test}
  const q = Q / 3600;
  const D = PIPES[s.pipe], A = Math.PI * D * D / 4, v = q / A;
  const Re = fl.rho * v * D / fl.mu, f = friction(Re, D);
  const hf_test = f * (s.length / D) * v * v / (2 * G);
  const Ab = Math.PI * BASE.D * BASE.D / 4, vb = q / Ab, Reb = fl.rho * vb * BASE.D / fl.mu, fb = friction(Reb, BASE.D);
  const hf_base = (fb * BASE.L / BASE.D + BASE.Kfit) * vb * vb / (2 * G);
  const Kv = valveKv(s.valve / 100);
  const hv = Kv > 0 ? Math.pow(Q / Kv, 2) * 1e5 / (fl.rho * G) : Infinity;
  const Ks = 2 + 60 * Math.pow(s.strainer / 100, 2); // strainer + suction fittings
  const hs = Ks * vb * vb / (2 * G);
  return { hf_test, hf_base, hv, hs, v_test: v, v_base: vb, Re_test: Re, f_test: f, Kv };
}
function staticHead(s) { return RETURN_HEIGHT - s.level; }

function solve(s) {
  const fl = waterProps(s.temp);
  const n = s.speedActual / 100;
  const Hs = staticHead(s);
  let Q = 0, H = 0, L = null;
  if (n > 0.02 && valveKv(s.valve / 100) > 0 && pumpHead(0, n) > Hs) {
    let lo = 0, hi = runout(n);
    for (let i = 0; i < 60; i++) {
      const mid = (lo + hi) / 2, l = systemLosses(mid, s, fl);
      const sys = Hs + l.hf_test + l.hf_base + l.hv + l.hs;
      if (pumpHead(mid, n) > sys) lo = mid; else hi = mid;
    }
    Q = (lo + hi) / 2; H = pumpHead(Q, n); L = systemLosses(Q, s, fl);
  } else { L = systemLosses(0, s, fl); L.hv = 0; }
  // NPSH
  const patm = 101325;
  const npsha = patm / (fl.rho * G) + s.level - L.hs - fl.pv / (fl.rho * G);
  const npshr = n > 0 ? PUMP.npshr(Q / Math.max(n, 0.05)) * n * n : 0;
  const margin = npsha - npshr;
  let cavFactor = 1;
  if (n > 0.05 && margin < 0) cavFactor = Math.max(0.35, 1 + margin * 0.3);
  if (cavFactor < 1) { H *= cavFactor; Q *= Math.sqrt(cavFactor); L = systemLosses(Q, s, fl); }
  const eta = pumpEta(Q, n);
  const Pshaft = Q > 0 ? fl.rho * G * (Q / 3600) * H / eta : (n > 0.02 ? 0.25 * PUMP.motorKW * 1000 * n * n * n : 0);
  const Pelec = Pshaft / PUMP.motorEta + (n > 0.02 ? 120 : 0);
  // pressures (gauge, Pa) at pump centreline datum
  const P1 = fl.rho * G * (s.level - L.hs);
  const P2 = P1 + fl.rho * G * H;
  const zTop = 3.0; // top run height above pump centreline (m)
  const P3 = P2 - fl.rho * G * (L.hf_base * 0.5 + L.hf_test + zTop);     // upstream valve
  const P4 = P3 - fl.rho * G * (Number.isFinite(L.hv) ? L.hv : 0);       // downstream valve
  const dpTest = fl.rho * G * L.hf_test;
  const dpValve = Number.isFinite(L.hv) ? fl.rho * G * L.hv : P3;
  return { fl, n, rpm: n * PUMP.N0, Q, H, Hs, L, npsha, npshr, margin, cavFactor, eta, Pshaft, Pelec, P1, P2, P3, P4, dpTest, dpValve,
           runoutQ: runout(n), specE: Q > 0 ? Pelec / 1000 / Q : 0 };
}

/* ------------------------------------------------------------------ state */
const state = {
  speed: 0, speedActual: 0, valve: 50, pipe: 40, length: 8, temp: 20, strainer: 0, level: 0.8,
  Qshown: 0, Hshown: 0, deadTime: 0, log: [], sweep: null, casingT: 20, flags: {},
};
let R = solve(state);
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';

/* ------------------------------------------------------------------ rig svg */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; return t; }

// loop geometry (px in a 900x560 viewBox)
const SUCTION = [[640, 505], [300, 505], [300, 470], [152, 470]];
const DISCHARGE = [[120, 440], [120, 130], [760, 130], [760, 393]];
const TEST = { x1: 200, x2: 480, y: 130 };
const rig = {};
function pathD(pts) { return pts.map((p, i) => (i ? 'L' : 'M') + p[0] + ' ' + p[1]).join(' '); }

function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 560', role: 'img' }, $('#rig'));
  rig.svg = svg;
  // reservoir
  el('rect', { x: 640, y: 380, width: 220, height: 150, class: 'reservoir', rx: 4 }, svg);
  rig.fluid = el('rect', { x: 642, y: 480, width: 216, height: 48, class: 'fluid' }, svg);
  txt(750, 548, 'CLEAR RESERVOIR · OPEN TO ATMOSPHERE', 'label', svg).setAttribute('text-anchor', 'middle');
  for (let i = 0; i <= 5; i++) { const y = 528 - i * 28; el('line', { x1: 852, y1: y, x2: 858, y2: y, class: 'gauge-tick' }, svg); }
  rig.levelTxt = txt(846, 400, '', 'gauge-val', svg); rig.levelTxt.style.textAnchor = 'end';
  // pipes: suction (fixed DN40), discharge riser + top + downcomer
  const w = 12;
  el('path', { d: pathD(SUCTION), class: 'pipe-edge', 'stroke-width': w + 4 }, svg);
  el('path', { d: pathD(SUCTION), class: 'pipe', 'stroke-width': w }, svg);
  rig.sucFluid = el('path', { d: pathD(SUCTION), class: 'pipe-fluid', 'stroke-width': w - 4 }, svg);
  el('path', { d: pathD(DISCHARGE), class: 'pipe-edge', 'stroke-width': w + 4 }, svg);
  el('path', { d: pathD(DISCHARGE), class: 'pipe', 'stroke-width': w }, svg);
  rig.disFluid = el('path', { d: pathD(DISCHARGE), class: 'pipe-fluid', 'stroke-width': w - 4 }, svg);
  // test section (variable bore) drawn over the top run
  rig.testEdge = el('line', { x1: TEST.x1, y1: TEST.y, x2: TEST.x2, y2: TEST.y, class: 'pipe-edge' }, svg);
  rig.testPipe = el('line', { x1: TEST.x1, y1: TEST.y, x2: TEST.x2, y2: TEST.y, class: 'pipe' }, svg);
  rig.testFluid = el('line', { x1: TEST.x1, y1: TEST.y, x2: TEST.x2, y2: TEST.y, class: 'pipe-fluid' }, svg);
  rig.testTag = txt(340, 168, '', 'tag', svg); rig.testTag.setAttribute('text-anchor', 'middle');
  txt(340, 182, 'INTERCHANGEABLE TEST SECTION', 'label', svg).setAttribute('text-anchor', 'middle');
  // DP taps on test section
  for (const x of [TEST.x1 + 12, TEST.x2 - 12]) { el('line', { x1: x, y1: TEST.y - 12, x2: x, y2: 88, class: 'tap' }, svg); el('circle', { cx: x, cy: TEST.y - 12, r: 3, class: 'tap-dot' }, svg); }
  el('rect', { x: 268, y: 62, width: 144, height: 34, class: 'dp-box' }, svg);
  el('line', { x1: TEST.x1 + 12, y1: 88, x2: 268, y2: 79, class: 'tap' }, svg); el('line', { x1: TEST.x2 - 12, y1: 88, x2: 412, y2: 79, class: 'tap' }, svg);
  txt(340, 75, 'ΔP TEST SECTION', 'gauge-name', svg); rig.dpTest = txt(340, 90, '—', 'dp-txt', svg);
  // flowmeter
  el('rect', { x: 540, y: 108, width: 64, height: 44, class: 'flowmeter', rx: 5 }, svg);
  rig.rotor = el('g', { transform: 'translate(572 130)' }, svg);
  for (let i = 0; i < 4; i++) el('line', { x1: 0, y1: 0, x2: 14, y2: 0, transform: `rotate(${i * 90})`, class: 'rotor' }, rig.rotor);
  txt(572, 172, 'FLOWMETER', 'label', svg).setAttribute('text-anchor', 'middle');
  rig.flowTxt = txt(572, 190, '', 'dp-txt', svg);
  // pump + motor
  el('rect', { x: 30, y: 447, width: 62, height: 46, class: 'motor', rx: 6 }, svg);
  txt(61, 476, 'M', 'tag', svg).setAttribute('text-anchor', 'middle');
  txt(30, 440, 'VARIABLE-SPEED MOTOR', 'label', svg);
  el('line', { x1: 92, y1: 470, x2: 98, y2: 470, class: 'gauge-tick' }, svg);
  el('circle', { cx: 120, cy: 470, r: 30, class: 'pump-body' }, svg);
  rig.impeller = el('g', { transform: 'translate(120 470)' }, svg);
  for (let i = 0; i < 6; i++) el('path', { d: 'M0 0 Q 10 -6 20 -14', transform: `rotate(${i * 60})`, class: 'impeller' }, rig.impeller);
  el('circle', { cx: 120, cy: 470, r: 4, class: 'impeller' }, svg);
  txt(120, 522, 'CENTRIFUGAL PUMP', 'label', svg).setAttribute('text-anchor', 'middle');
  rig.heat = el('circle', { cx: 120, cy: 470, r: 34, class: 'heat' }, svg);
  rig.bubbles = el('g', {}, svg);
  for (let i = 0; i < 9; i++) el('circle', { r: 2 + (i % 3), class: 'bubble', cx: 150 + i * 4, cy: 470, opacity: 0 }, rig.bubbles);
  // strainer
  el('rect', { x: 392, y: 496, width: 24, height: 18, class: 'flowmeter', rx: 2 }, svg);
  for (let i = 1; i < 4; i++) el('line', { x1: 392 + i * 6, y1: 496, x2: 392 + i * 6, y2: 514, class: 'gauge-tick' }, svg);
  rig.strainerFill = el('rect', { x: 392, y: 496, width: 24, height: 18, fill: 'var(--brass)', opacity: 0, rx: 2 }, svg);
  txt(404, 532, 'SUCTION STRAINER', 'label', svg).setAttribute('text-anchor', 'middle');
  // control valve (bow-tie) on downcomer
  el('path', { d: 'M746 220 L774 240 L746 260 Z', class: 'valve-body' }, svg);
  el('path', { d: 'M774 220 L746 240 L774 260 Z', class: 'valve-body' }, svg);
  rig.valveGate = el('rect', { x: 758, y: 222, width: 4, height: 36, fill: 'var(--panel)' }, svg);
  el('line', { x1: 760, y1: 220, x2: 760, y2: 196, class: 'gauge-tick' }, svg);
  rig.stem = el('rect', { x: 752, y: 186, width: 16, height: 10, class: 'valve-body', rx: 2 }, svg);
  txt(796, 244, 'CONTROL VALVE', 'label', svg);
  rig.valveTxt = txt(796, 258, '', 'dp-txt', svg); rig.valveTxt.style.textAnchor = 'start';
  // static head marker
  txt(772, 372, 'RETURN OUTLET', 'label', svg);
  // particles
  rig.particles = [];
  const mk = (path) => { const g = el('g', {}, svg); for (let i = 0; i < 26; i++) rig.particles.push({ path, s: i / 26, e: el('circle', { r: 2.2, class: 'particle' }, g) }); };
  mk(SUCTION); mk(DISCHARGE);
  // gauges
  rig.g1 = gauge(svg, 232, 416, 'P1 SUCTION', -0.5, 4);
  rig.g2 = gauge(svg, 172, 300, 'P2 DISCHARGE', -0.5, 4);
  rig.g3 = gauge(svg, 700, 178, 'P3 BEFORE VALVE', -0.5, 4);
  rig.g4 = gauge(svg, 700, 302, 'P4 AFTER VALVE', -0.5, 4);
  el('line', { x1: 232, y1: 438, x2: 232, y2: 464, class: 'tap' }, svg);
  el('line', { x1: 150, y1: 300, x2: 128, y2: 300, class: 'tap' }, svg);
  el('line', { x1: 722, y1: 178, x2: 752, y2: 178, class: 'tap' }, svg);
  el('line', { x1: 722, y1: 302, x2: 752, y2: 302, class: 'tap' }, svg);
  // motor speed label
  rig.rpmTxt = txt(61, 426, '', 'gauge-val', svg);
  // ΔP valve box
  el('rect', { x: 780, y: 268, width: 96, height: 30, class: 'dp-box' }, svg);
  txt(828, 280, 'ΔP VALVE', 'gauge-name', svg); rig.dpValve = txt(828, 293, '—', 'dp-txt', svg);
  // arrows for flow direction
  txt(120, 118, '▲', 'tag', svg).setAttribute('text-anchor', 'middle');
}
function gauge(svg, cx, cy, name, min, max) {
  const g = el('g', { transform: `translate(${cx} ${cy})` }, svg);
  el('circle', { r: 22, class: 'gauge-face' }, g);
  for (let i = 0; i <= 8; i++) { const a = -135 + i * 33.75, r = Math.PI * a / 180; el('line', { x1: 15 * Math.sin(r), y1: -15 * Math.cos(r), x2: 19 * Math.sin(r), y2: -19 * Math.cos(r), class: 'gauge-tick' }, g); }
  const needle = el('line', { x1: 0, y1: 3, x2: 0, y2: -17, class: 'gauge-needle' }, g);
  el('circle', { r: 2.5, fill: 'var(--alarm)' }, g);
  txt(0, 33, name, 'gauge-name', g);
  const val = txt(0, 44, '', 'gauge-val', g);
  return { needle, val, min, max, set(v) { const t = Math.max(0, Math.min(1, (v - min) / (max - min))); needle.setAttribute('transform', `rotate(${-135 + 270 * t})`); val.textContent = fmt(v, 2) + ' bar'; } };
}

/* ------------------------------------------------------------------ chart */
const chart = { c: $('#chart'), Qmax: 50, Hmax: 40 };
function drawChart() {
  const c = chart.c, ctx = c.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const W = c.clientWidth || 900, Hh = W * 520 / 900;
  if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(Hh * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cs = getComputedStyle(document.documentElement);
  const col = n => cs.getPropertyValue(n).trim();
  const m = { l: 52, r: 52, t: 18, b: 40 }, pw = W - m.l - m.r, ph = Hh - m.t - m.b;
  const X = q => m.l + q / chart.Qmax * pw, Y = h => m.t + ph - h / chart.Hmax * ph, Y2 = e => m.t + ph - e * ph;
  ctx.clearRect(0, 0, W, Hh);
  ctx.font = '11px ' + col('--mono');
  // grid
  ctx.strokeStyle = col('--grid'); ctx.lineWidth = 1; ctx.fillStyle = col('--muted'); ctx.textAlign = 'center';
  for (let q = 0; q <= chart.Qmax; q += 10) { ctx.beginPath(); ctx.moveTo(X(q), m.t); ctx.lineTo(X(q), m.t + ph); ctx.stroke(); ctx.fillText(q, X(q), Hh - m.b + 16); }
  ctx.textAlign = 'right';
  for (let h = 0; h <= chart.Hmax; h += 10) { ctx.beginPath(); ctx.moveTo(m.l, Y(h)); ctx.lineTo(m.l + pw, Y(h)); ctx.stroke(); ctx.fillText(h, m.l - 8, Y(h) + 4); }
  ctx.textAlign = 'left';
  for (let e = 0; e <= 1; e += 0.25) ctx.fillText(Math.round(e * 100) + '%', m.l + pw + 8, Y2(e) + 4);
  ctx.textAlign = 'center'; ctx.font = '600 12px ' + col('--body');
  ctx.fillText('Flow Q  (m³/h)', m.l + pw / 2, Hh - 6);
  ctx.save(); ctx.translate(14, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('Head H  (m)', 0, 0); ctx.restore();
  ctx.save(); ctx.translate(W - 8, m.t + ph / 2); ctx.rotate(Math.PI / 2); ctx.fillStyle = col('--good'); ctx.fillText('Pump efficiency η', 0, 0); ctx.restore();
  ctx.font = '11px ' + col('--mono');
  const plot = (fn, step, style, width, dash) => { ctx.beginPath(); ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash || []); let first = true; for (let q = 0; q <= chart.Qmax; q += step) { const y = fn(q); if (y === null || !Number.isFinite(y)) { first = true; continue; } if (y < -5 || y > chart.Hmax * 1.2) { first = true; continue; } first ? ctx.moveTo(X(q), Y(y)) : ctx.lineTo(X(q), Y(y)); first = false; } ctx.stroke(); ctx.setLineDash([]); };
  // rated pump curve (dim) and at set speed
  plot(q => q <= runout(1) ? pumpHead(q, 1) : null, 0.5, col('--water-soft'), 2, [6, 5]);
  const n = R.n;
  if (n > 0.02) plot(q => q <= runout(n) ? pumpHead(q, n) * R.cavFactor : null, 0.5, col('--water'), 3);
  // efficiency at set speed (right axis)
  if (n > 0.02) { ctx.beginPath(); ctx.strokeStyle = col('--good'); ctx.lineWidth = 2; ctx.setLineDash([2, 4]); let f = true; for (let q = 0.5; q <= runout(n); q += 0.5) { const e = pumpEta(q, n); f ? ctx.moveTo(X(q), Y2(e)) : ctx.lineTo(X(q), Y2(e)); f = false; } ctx.stroke(); ctx.setLineDash([]); }
  // system curve
  const s = state, fl = R.fl, Hs = R.Hs;
  plot(q => { const l = systemLosses(q, s, fl); const h = Hs + l.hf_test + l.hf_base + l.hs + (Number.isFinite(l.hv) ? l.hv : 1e9); return h; }, 0.25, col('--brass'), 3);
  // static head line
  ctx.strokeStyle = col('--brass'); ctx.lineWidth = 1; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(m.l, Y(Hs)); ctx.lineTo(m.l + pw, Y(Hs)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = col('--brass'); ctx.textAlign = 'left'; ctx.fillText('static head ' + fmt(Hs, 1) + ' m', m.l + 6, Y(Hs) - 5);
  // BEP marker
  if (n > 0.02) { const qb = PUMP.Qbep * n; ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(X(qb), Y(pumpHead(qb, n)), 4, 0, 7); ctx.fill(); ctx.fillText('BEP', X(qb) + 7, Y(pumpHead(qb, n)) - 6); }
  // operating point
  if (n > 0.02) {
    const q = state.Qshown, h = state.Hshown;
    ctx.strokeStyle = col('--steel'); ctx.setLineDash([2, 3]); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(q), Y(h)); ctx.lineTo(X(q), m.t + ph); ctx.moveTo(X(q), Y(h)); ctx.lineTo(m.l, Y(h)); ctx.stroke(); ctx.setLineDash([]);
    const pulse = 6 + 2 * Math.sin(performance.now() / 300);
    ctx.fillStyle = R.margin < 0 ? col('--alarm') : col('--ink'); ctx.beginPath(); ctx.arc(X(q), Y(h), pulse, 0, 7); ctx.fill();
    ctx.fillStyle = col('--panel'); ctx.beginPath(); ctx.arc(X(q), Y(h), 2.5, 0, 7); ctx.fill();
    ctx.fillStyle = col('--ink'); ctx.font = '600 12px ' + col('--mono'); ctx.textAlign = 'left';
    ctx.fillText(`Q ${fmt(q, 1)} m³/h · H ${fmt(h, 1)} m · η ${fmt(R.eta * 100, 0)} %`, Math.min(X(q) + 12, m.l + pw - 210), Math.max(Y(h) - 12, m.t + 12));
  } else { ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.font = '600 14px ' + col('--body'); ctx.fillText('Pump stopped — raise motor speed to start the loop', m.l + pw / 2, m.t + ph / 2); }
}

/* ------------------------------------------------------------------ ui */
const READ = [
  ['Q', 'Flow', v => fmt(R.Q, 1), 'm³/h'], ['Qlpm', 'Flow', v => fmt(R.Q * 1000 / 60, 0), 'L/min'],
  ['H', 'Pump head', v => fmt(R.H, 1), 'm'], ['v', 'Velocity in test pipe', v => fmt(R.L.v_test, 2), 'm/s'],
  ['Re', 'Reynolds number', v => Math.round(R.L.Re_test).toLocaleString(), R.L.Re_test < 2300 ? 'laminar' : R.L.Re_test < 4000 ? 'transitional' : 'turbulent'],
  ['f', 'Friction factor f', v => fmt(R.L.f_test, 4), 'Darcy'], ['hf', 'Test-section loss', v => fmt(R.L.hf_test, 2), 'm'],
  ['hv', 'Valve loss', v => Number.isFinite(R.L.hv) ? fmt(R.L.hv, 2) : 'shut', 'm'], ['Hs', 'Static head', v => fmt(R.Hs, 2), 'm'],
  ['npsha', 'NPSH available', v => fmt(R.npsha, 2), 'm'], ['npshr', 'NPSH required', v => fmt(R.npshr, 2), 'm'],
  ['margin', 'NPSH margin', v => fmt(R.margin, 2), 'm', () => R.n > 0.05 && R.margin < 0.5],
  ['eta', 'Pump efficiency', v => fmt(R.eta * 100, 1), '%'], ['Psh', 'Shaft power', v => fmt(R.Pshaft / 1000, 2), 'kW'],
  ['Pel', 'Electrical power', v => fmt(R.Pelec / 1000, 2), 'kW', () => R.Pelec > PUMP.overloadKW * 1000],
  ['spec', 'Specific energy', v => fmt(R.specE, 3), 'kWh/m³'], ['rpm', 'Pump speed', v => Math.round(R.rpm), 'rpm'],
  ['Kv', 'Valve Kv', v => fmt(R.L.Kv, 1), 'm³/h·bar⁻½'],
];
const TOP = ['Q', 'H', 'eta', 'Pel', 'v', 'npsha', 'margin', 'spec'];
function buildReadouts() {
  const mk = (box, keys) => { box.innerHTML = ''; for (const r of READ) { if (keys && !keys.includes(r[0])) continue; const d = document.createElement('div'); d.className = 'ro'; d.dataset.k = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u"></span>`; box.appendChild(d); } };
  mk($('#readouts'), TOP); mk($('#dataGrid'), null);
}
function updateReadouts() {
  document.querySelectorAll('.ro').forEach(d => { const r = READ.find(x => x[0] === d.dataset.k); d.querySelector('.v').textContent = r[2](); d.querySelector('.u').textContent = typeof r[3] === 'function' ? r[3]() : (r[0] === 'Re' ? (R.L.Re_test < 2300 ? 'laminar' : R.L.Re_test < 4000 ? 'transitional' : 'turbulent') : r[3]); d.classList.toggle('alarm', !!(r[4] && r[4]())); });
}

function updateRig(dt) {
  const s = state;
  // fluid level
  const hFluid = 20 + (s.level - 0.1) / 1.4 * 110; rig.fluid.setAttribute('y', 528 - hFluid); rig.fluid.setAttribute('height', hFluid);
  rig.levelTxt.textContent = s.level.toFixed(2) + ' m';
  // test pipe width
  const w = 8 + (s.pipe - 25) / 25 * 10; rig.testEdge.setAttribute('stroke-width', w + 4); rig.testPipe.setAttribute('stroke-width', w); rig.testFluid.setAttribute('stroke-width', w - 4);
  rig.testTag.textContent = `DN${s.pipe} · ${(PIPES[s.pipe] * 1000).toFixed(1)} mm bore · ${s.length} m`;
  // fluid opacity by flow
  const flowing = R.Q > 0.05; [rig.sucFluid, rig.disFluid, rig.testFluid].forEach(e => e.style.opacity = flowing ? 0.75 : 0.25);
  // impeller rotation
  rig.imAngle = (rig.imAngle || 0) + R.rpm / 60 * 360 * dt * 0.08; rig.impeller.setAttribute('transform', `translate(120 470) rotate(${rig.imAngle % 360})`);
  rig.rpmTxt.textContent = Math.round(R.rpm) + ' rpm';
  // flowmeter rotor
  rig.roAngle = (rig.roAngle || 0) + R.Q * 12 * dt; rig.rotor.setAttribute('transform', `translate(572 130) rotate(${rig.roAngle % 360})`);
  rig.flowTxt.textContent = fmt(R.Q, 1) + ' m³/h';
  // valve
  const x = s.valve / 100; rig.valveGate.setAttribute('height', 36 * x); rig.valveGate.setAttribute('y', 240 - 18 * x); rig.stem.setAttribute('y', 186 - 14 * x);
  rig.valveTxt.textContent = s.valve + ' % open · Kv ' + fmt(R.L.Kv, 1);
  rig.dpValve.textContent = Number.isFinite(R.dpValve) ? fmt(R.dpValve / 1000, 1) + ' kPa' : '—';
  rig.dpTest.textContent = fmt(R.dpTest / 1000, 2) + ' kPa';
  rig.strainerFill.setAttribute('opacity', s.strainer / 100 * 0.8);
  // gauges
  rig.g1.set(R.P1 / 1e5); rig.g2.set(R.P2 / 1e5); rig.g3.set(R.P3 / 1e5); rig.g4.set(R.P4 / 1e5);
  // particles
  const vTest = R.L.v_test, vBase = R.L.v_base;
  for (const p of rig.particles) {
    const isTest = p.path === DISCHARGE;
    const pxs = (isTest ? (vTest + vBase) / 2 : vBase) * 60; // px per second
    const len = pathLen(p.path); p.s = (p.s + pxs * dt / len) % 1;
    const [px, py] = pointAt(p.path, p.s * len); p.e.setAttribute('cx', px); p.e.setAttribute('cy', py);
    p.e.style.opacity = flowing ? 0.85 : 0;
  }
  // cavitation bubbles and dead-head heat
  const cav = R.n > 0.05 && R.margin < 0;
  rig.bubbles.querySelectorAll('circle').forEach((b, i) => { if (cav) { const t = (performance.now() / 400 + i * 0.7) % 3; b.setAttribute('cx', 100 + t * 30 + (i % 3) * 5); b.setAttribute('cy', 470 - 10 + (i % 4) * 6); b.setAttribute('opacity', 0.9 - t / 3); } else b.setAttribute('opacity', 0); });
  rig.heat.style.opacity = Math.min(0.5, (s.casingT - 20) / 120);
}
function pathLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); return L; }
function pointAt(pts, d) { for (let i = 1; i < pts.length; i++) { const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); if (d <= l) { const t = d / l; return [pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * t, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * t]; } d -= l; } return pts[pts.length - 1]; }

function updateStatus() {
  const run = R.n > 0.02; $('#pillRun').textContent = run ? `Pump running · ${Math.round(R.rpm)} rpm` : 'Pump stopped'; $('#pillRun').classList.toggle('on', run);
  $('#pillPoint').textContent = run ? `Operating point Q ${fmt(R.Q, 1)} m³/h · H ${fmt(R.H, 1)} m` : 'Operating point —';
  const dead = run && R.Q < 0.3 && state.deadTime > 2.5;
  $('#pillDead').classList.toggle('hidden', !dead); $('#pillDead').textContent = `Dead-head · casing ${Math.round(state.casingT)} °C`;
  const cav = run && R.margin < 0; $('#pillCav').classList.toggle('hidden', !cav); $('#pillCav').textContent = `Cavitation · NPSH margin ${fmt(R.margin, 1)} m`;
  const over = R.Pelec > PUMP.overloadKW * 1000; $('#pillOver').classList.toggle('hidden', !over); $('#pillOver').textContent = `Motor overload · ${fmt(R.Pelec / 1000, 2)} kW of ${PUMP.motorKW} kW`;
  const ro = run && R.Q > 0.93 * R.runoutQ; $('#pillRunout').classList.toggle('hidden', !ro); $('#pillRunout').textContent = 'Run-out · end of curve';
  state.flags.dead = state.flags.dead || dead; state.flags.cav = state.flags.cav || cav; state.flags.over = state.flags.over || over; state.flags.runout = state.flags.runout || ro;
}

/* ------------------------------------------------------------------ log */
function record(tag) {
  const row = { i: state.log.length + 1, speed: state.speed, rpm: Math.round(R.rpm), valve: state.valve, pipe: 'DN' + state.pipe, Q: +R.Q.toFixed(2), H: +R.H.toFixed(2), P1: +(R.P1 / 1e5).toFixed(3), P2: +(R.P2 / 1e5).toFixed(3), dpT: +(R.dpTest / 1000).toFixed(2), dpV: Number.isFinite(R.dpValve) ? +(R.dpValve / 1000).toFixed(1) : '', eta: +(R.eta * 100).toFixed(1), Pel: +(R.Pelec / 1000).toFixed(3), margin: +R.margin.toFixed(2), tag: tag || '' };
  state.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  const tb = $('#logTable tbody'); tb.innerHTML = '';
  for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.speed, r.rpm, r.valve, r.pipe, r.Q, r.H, r.P1, r.P2, r.dpT, r.dpV, r.eta, r.Pel, r.margin].map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); }
  $('#logCount').textContent = state.log.length;
  const csv = ['n,speed_pct,rpm,valve_pct,pipe,Q_m3h,H_m,P1_bar,P2_bar,dP_pipe_kPa,dP_valve_kPa,eta_pct,P_elec_kW,NPSH_margin_m'].concat(state.log.map(r => [r.i, r.speed, r.rpm, r.valve, r.pipe, r.Q, r.H, r.P1, r.P2, r.dpT, r.dpV, r.eta, r.Pel, r.margin].join(','))).join('\n');
  state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv);
}

/* ------------------------------------------------------------------ tasks */
const TASKS = [
  { id: 'start', t: 'Start the loop', d: 'Run the motor at 100 % with the valve half open and confirm flow on the meter.', ok: () => state.speed >= 99 && state.speedActual >= 98 && state.valve >= 45 && state.valve <= 55 && R.Q > 1 },
  { id: 'bep', t: 'Find the best-efficiency point', d: 'At 100 % speed, adjust the valve until the efficiency readout is within 2 % of its maximum. Note the flow and head.', ok: () => state.speedActual >= 98 && R.eta >= PUMP.etaMax - 0.02 },
  { id: 'throttle', t: 'Same flow, two ways', d: 'Deliver 12 m³/h (±0.5) by throttling at 100 % speed and record it. Then open the valve fully and reduce speed until you get 12 m³/h again; record that too. Compare electrical power.', ok: () => state.log.some(r => r.speed >= 99 && Math.abs(r.Q - 12) <= 0.5) && state.log.some(r => r.valve >= 95 && r.speed < 90 && Math.abs(r.Q - 12) <= 0.5), st: () => { const a = state.log.find(r => r.speed >= 99 && Math.abs(r.Q - 12) <= 0.5), b = state.log.find(r => r.valve >= 95 && r.speed < 90 && Math.abs(r.Q - 12) <= 0.5); return a && b ? `throttled ${a.Pel} kW vs speed-controlled ${b.Pel} kW — saving ${Math.round((1 - b.Pel / a.Pel) * 100)} %` : (a ? 'throttled point recorded — now the speed-controlled one' : 'no points yet'); } },
  { id: 'pipe', t: 'Swap the test section', d: 'With speed 100 % and the valve fully open, record a point with DN50 fitted, then with DN25. Watch the operating point slide up the curve and ΔP across the test section climb.', ok: () => state.log.some(r => r.pipe === 'DN50' && r.speed >= 99 && r.valve >= 95) && state.log.some(r => r.pipe === 'DN25' && r.speed >= 99 && r.valve >= 95), st: () => { const a = state.log.find(r => r.pipe === 'DN50' && r.speed >= 99 && r.valve >= 95), b = state.log.find(r => r.pipe === 'DN25' && r.speed >= 99 && r.valve >= 95); return a && b ? `DN50: ${a.Q} m³/h, ΔP ${a.dpT} kPa · DN25: ${b.Q} m³/h, ΔP ${b.dpT} kPa` : 'record both pipes'; } },
  { id: 'cav', t: 'Make it cavitate', d: 'Heat the fluid and foul the strainer with the pump running until the NPSH margin goes negative. Watch the head collapse on the chart and the bubbles at the impeller eye.', ok: () => !!state.flags.cav },
  { id: 'dead', t: 'Dead-head the pump', d: 'Shut the valve with the pump running for a few seconds. No flow, full head, and the casing starts to heat because all the power goes into the trapped fluid.', ok: () => !!state.flags.dead },
  { id: 'runout', t: 'Run it off the end of the curve', d: 'Fit DN50, open the valve fully at 100 % speed. The pump runs out to high flow and the motor is overloaded — the other end of the failure envelope.', ok: () => !!state.flags.runout && !!state.flags.over },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); const ok = t.ok(); if (ok) li.classList.add('done'); li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (t.st ? t.st() : (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  // motor ramp
  const ramp = 40 * dt; state.speedActual += Math.max(-ramp, Math.min(ramp, state.speed - state.speedActual));
  if (Math.abs(state.speed - state.speedActual) < 0.05) state.speedActual = state.speed;
  // sweep
  if (state.sweep) { state.sweep.t += dt; state.valve = Math.min(100, Math.round(state.sweep.t * 12)); $('#valve').value = state.valve; if (state.sweep.t - state.sweep.lastRec >= 0.8 && state.speedActual === state.speed) { record('sweep'); state.sweep.lastRec = state.sweep.t; } if (state.valve >= 100 && state.sweep.t - state.sweep.lastRec > 0.5) { state.sweep = null; $('#btnSweep').textContent = 'Auto-sweep valve'; } }
  R = solve(state);
  // flow lag
  const tau = 0.5; state.Qshown += (R.Q - state.Qshown) * Math.min(1, dt / tau); state.Hshown += (R.H - state.Hshown) * Math.min(1, dt / tau);
  // dead-head heating
  if (R.n > 0.05 && R.Q < 0.3) { state.deadTime += dt; state.casingT = Math.min(95, state.casingT + 4 * dt * R.n); } else { state.deadTime = 0; state.casingT = Math.max(state.temp, state.casingT - 8 * dt); }
  updateRig(dt); drawChart();
  if (frame++ % 6 === 0) { updateReadouts(); updateStatus(); checkTasks(); $('#speedOut').textContent = state.speed + ' %'; $('#rpmOut').textContent = Math.round(R.rpm) + ' rpm' + (state.speedActual !== state.speed ? ' (ramping)' : ''); $('#valveOut').textContent = state.valve + ' %'; $('#kvOut').textContent = 'Kv ' + fmt(R.L.Kv, 1); }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function bind() {
  const on = (id, fn) => $('#' + id).addEventListener('input', e => { fn(e.target.value); });
  on('speed', v => state.speed = +v); on('valve', v => { state.valve = +v; state.sweep = null; });
  on('pipe', v => state.pipe = +v); on('length', v => { state.length = +v; $('#lengthOut').textContent = v + ' m'; });
  on('temp', v => { state.temp = +v; $('#tempOut').textContent = v + ' °C'; }); on('strainer', v => { state.strainer = +v; $('#strainerOut').textContent = v + ' %'; });
  on('level', v => { state.level = +v; $('#levelOut').textContent = (+v).toFixed(2) + ' m'; });
  document.querySelectorAll('[data-speed]').forEach(b => b.onclick = () => { state.speed = +b.dataset.speed; $('#speed').value = state.speed; });
  document.querySelectorAll('[data-valve]').forEach(b => b.onclick = () => { state.valve = +b.dataset.valve; $('#valve').value = state.valve; state.sweep = null; });
  $('#btnRecord').onclick = () => record('manual');
  $('#btnSweep').onclick = () => { if (state.sweep) { state.sweep = null; $('#btnSweep').textContent = 'Auto-sweep valve'; return; } if (state.speed < 5) { state.speed = 100; $('#speed').value = 100; } state.valve = 0; $('#valve').value = 0; state.sweep = { t: 0, lastRec: -1 }; $('#btnSweep').textContent = 'Stop sweep'; document.querySelector('[data-tab="log"]').click(); };
  $('#btnReset').onclick = () => { Object.assign(state, { speed: 0, valve: 50, pipe: 40, length: 8, temp: 20, strainer: 0, level: 0.8, sweep: null, flags: {} }); ['speed', 'valve', 'pipe', 'length', 'temp', 'strainer', 'level'].forEach(id => $('#' + id).value = state[id]); $('#lengthOut').textContent = '8 m'; $('#tempOut').textContent = '20 °C'; $('#strainerOut').textContent = '0 %'; $('#levelOut').textContent = '0.80 m'; document.querySelectorAll('.tasks li').forEach(l => l.classList.remove('done')); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('hl_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden');
  $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const step = e.shiftKey ? 10 : 2;
    if (e.key === 'ArrowUp') { state.speed = Math.min(100, state.speed + step); $('#speed').value = state.speed; }
    else if (e.key === 'ArrowDown') { state.speed = Math.max(0, state.speed - step); $('#speed').value = state.speed; }
    else if (e.key === 'ArrowRight') { state.valve = Math.min(100, state.valve + step); $('#valve').value = state.valve; }
    else if (e.key === 'ArrowLeft') { state.valve = Math.max(0, state.valve - step); $('#valve').value = state.valve; }
    else if (e.key === 'r' || e.key === 'R') record('manual');
    else if (e.key === 's' || e.key === 'S') $('#btnSweep').click();
    else if (e.key === 'Escape') $('#help').classList.add('hidden');
    else return;
    e.preventDefault();
  });
  window.addEventListener('resize', drawChart);
}

/* ------------------------------------------------------------------ dashboard API */
window.HydraulicLoop = {
  getState: () => ({ inputs: { speed: state.speed, valve: state.valve, pipe: state.pipe, length: state.length, temp: state.temp, strainer: state.strainer, level: state.level }, results: { Q: R.Q, H: R.H, eta: R.eta, Pelec: R.Pelec, P1: R.P1, P2: R.P2, margin: R.margin, rpm: R.rpm }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: (inp) => { for (const k of ['speed', 'valve', 'pipe', 'length', 'temp', 'strainer', 'level']) if (inp[k] !== undefined) { state[k] = +inp[k]; const c = $('#' + k); if (c) c.value = state[k]; } },
  meta: { id: 'hydraulic-loop', title: 'Hydraulic Loop Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.HydraulicLoop.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'hydraulic-loop', state: window.HydraulicLoop.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('hl_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildReadouts(); buildTasks(); bind(); renderLog();
  // open in a realistic working state: pump running at 80 %, valve half open
  state.speed = 80; state.speedActual = 80; $('#speed').value = 80;
  R = solve(state); state.Qshown = R.Q; state.Hshown = R.H;
  requestAnimationFrame(tick);
}
build();
