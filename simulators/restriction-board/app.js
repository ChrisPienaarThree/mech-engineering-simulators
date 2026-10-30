/* Restriction Board — interchangeable pipe-element trainer with a piezometer bank.
   Standalone, offline. Physics: Bernoulli, Darcy–Weisbach, minor-loss K, ISO-style orifice/venturi.
   Rig data is illustrative (training only). */
'use strict';

/* ------------------------------------------------------------------ physics */
const G = 9.81, PATM = 101325;
const PUMP = { N0: 2900, H0: 32, a1: 0.2, a2: 0.01 };          // same illustrative pump as the loop bench
const D0 = 41.9e-3, A0 = Math.PI * D0 * D0 / 4;                   // main line DN40
const BORES = { 40: 41.9e-3, 50: 53.1e-3, 65: 70.3e-3, 80: 82.5e-3 };
const BOARD = 2.0, TAPS = [0.2, 0.8, 1.0, 1.2, 1.5, 1.8], ELEM = [0.9, 1.1];
const SUPPLY = { L: 3, K: 2, z: 0.5, Hs: 1.5 };                  // supply pipework, board height, return static head
const ROUGH = { smooth: 0.0015e-3, steel: 0.045e-3, rusty: 0.5e-3 };

function waterProps(T) { const rho = 1000 - 0.0035 * (T - 4) ** 2 * 0.65; const mu = 2.414e-5 * Math.pow(10, 247.8 / (T + 273.15 - 140)); const pv = 610.94 * Math.exp(17.625 * T / (T + 243.04)); return { rho, mu, pv }; }
function friction(Re, D, eps) { if (Re < 1) return 64; if (Re < 2300) return 64 / Re; const f = 0.25 / Math.pow(Math.log10(eps / (3.7 * D) + 5.74 / Math.pow(Re, 0.9)), 2); if (Re < 4000) { const t = (Re - 2300) / 1700; return (64 / 2300) * (1 - t) + f * t; } return f; }
function pumpHead(Q, n) { if (n <= 0) return 0; const q = Q / n; return n * n * Math.max(0, PUMP.H0 - PUMP.a1 * q - PUMP.a2 * q * q); }
function runout(n) { const b = PUMP.a1, a = PUMP.a2; return n * (-b + Math.sqrt(b * b + 4 * a * PUMP.H0)) / (2 * a); }
function throttleK(x) { return 0.2 + 400 * Math.pow(1 - x, 2.5); }
const VALVE_K = { gate: [[1, 0.2], [0.75, 1.15], [0.5, 5.6], [0.25, 24], [0.1, 100]], globe: [[1, 10], [0.75, 14], [0.5, 30], [0.25, 120], [0.1, 600]], ball: [[1, 0.05], [0.75, 0.9], [0.5, 5.5], [0.25, 60], [0.1, 400]] };
function valveK(type, x) { const t = VALVE_K[type]; x = Math.max(0.1, Math.min(1, x)); for (let i = 0; i < t.length - 1; i++) { const [x1, k1] = t[i], [x2, k2] = t[i + 1]; if (x <= x1 && x >= x2) { const u = (x1 - x) / (x1 - x2); return Math.exp(Math.log(k1) * (1 - u) + Math.log(k2) * u); } } return t[t.length - 1][1]; }
const ELBOW_K = { standard: 0.9, long: 0.3, mitre: 1.1 };

const ELEMENTS = {
  pipe: { name: 'Straight pipe', opts: { surface: ['smooth', 'steel', 'rusty'] }, def: { surface: 'smooth' } },
  orifice: { name: 'Orifice plate', opts: { beta: [0.4, 0.5, 0.6, 0.7] }, def: { beta: 0.6, cd: 0.61 } },
  venturi: { name: 'Venturi meter', opts: { beta: [0.5, 0.6, 0.7] }, def: { beta: 0.6, cd: 0.98 } },
  expansion: { name: 'Sudden expansion', opts: { d2: [50, 65, 80] }, def: { d2: 65 } },
  contraction: { name: 'Sudden contraction', opts: { d1: [80, 65, 50] }, def: { d1: 65 } },
  elbows: { name: 'Elbows', opts: { count: [1, 2, 3, 4], type: ['standard', 'long', 'mitre'] }, def: { count: 2, type: 'standard' } },
  valve: { name: 'Valve', opts: { type: ['gate', 'globe', 'ball'] }, def: { type: 'gate', open: 50 } },
};

/* Evaluate the board for a given main-line flow Q (m3/h). Returns loss terms and the tap profile. */
function board(Q, s, fl) {
  const q = Q / 3600, e = s.el, o = s.opt[e];
  const v0 = q / A0, Re0 = fl.rho * v0 * D0 / fl.mu;
  const eps = e === 'pipe' ? ROUGH[o.surface] : ROUGH.smooth;
  const f0 = friction(Re0, D0, eps), hv0 = v0 * v0 / (2 * G);
  const grad0 = f0 / D0 * hv0;                        // m head per m of main pipe
  // section geometry: upstream bore, downstream bore
  let Dup = D0, Ddn = D0, Dth = null;
  if (e === 'expansion') Ddn = BORES[o.d2];
  if (e === 'contraction') Dup = BORES[o.d1];
  if (e === 'orifice' || e === 'venturi') Dth = D0 * o.beta;
  const A = D => Math.PI * D * D / 4;
  const vup = q / A(Dup), vdn = q / A(Ddn), vth = Dth ? q / A(Dth) : null;
  const gradUp = friction(fl.rho * vup * Dup / fl.mu, Dup, eps) / Dup * vup * vup / (2 * G);
  const gradDn = friction(fl.rho * vdn * Ddn / fl.mu, Ddn, eps) / Ddn * vdn * vdn / (2 * G);
  // element: tap differential (T2-T3 for meters, T2-T4 otherwise), permanent loss, and local profile shape
  let dpMeter = 0, hperm = 0, hT3 = 0, Kref = 0, cdEff = 0, note = '';
  const hvUp = vup * vup / (2 * G), hvDn = vdn * vdn / (2 * G);
  if (e === 'pipe') { hperm = grad0 * (ELEM[1] - ELEM[0]); Kref = hperm / hv0 || 0; hT3 = -grad0 * (TAPS[2] - TAPS[1]); }
  else if (e === 'orifice' || e === 'venturi') {
    const beta = o.beta, At = A(Dth);
    cdEff = e === 'orifice' ? 0.61 + (Re0 < 10000 ? 0.03 * (10000 - Math.max(Re0, 1000)) / 9000 : 0) : 0.98;
    dpMeter = q > 0 ? Math.pow(q / (cdEff * At), 2) * fl.rho * (1 - Math.pow(beta, 4)) / 2 : 0; // Pa
    const hm = dpMeter / (fl.rho * G);
    if (e === 'orifice') { const r = Math.sqrt(1 - Math.pow(beta, 4) * (1 - cdEff * cdEff)); hperm = hm * (r - cdEff * beta * beta) / (r + cdEff * beta * beta); }
    else hperm = hm * (0.10 + 0.12 * (1 - beta));
    hT3 = -hm; Kref = hperm / hv0 || 0;
  }
  else if (e === 'expansion') { const K = Math.pow(1 - A(Dup) / A(Ddn), 2); hperm = K * hvUp; Kref = K; hT3 = (hvUp - hvDn) * 0.5 - hperm * 0.6; }
  else if (e === 'contraction') { const K = 0.42 * (1 - A(Ddn) / A(Dup)); hperm = K * hvDn; Kref = K; hT3 = (hvUp - hvDn) - hperm - 0.6 * hvDn; note = 'vena contracta'; }
  else if (e === 'elbows') { const K = o.count * ELBOW_K[o.type]; hperm = K * hv0; Kref = K; hT3 = -hperm * 0.5; }
  else if (e === 'valve') { const K = valveK(o.type, o.open / 100); hperm = K * hv0; Kref = K; hT3 = -hperm * 1.3; }
  // profile: static head relative to T2 at each tap (friction on straight runs + element)
  const upFr = x => -gradUp * (TAPS[1] - x);
  const dnFr = x => -gradDn * (x - TAPS[3]);
  const bern = (hvUp - hvDn);                          // static head change from velocity change (Bernoulli)
  const h = [upFr(TAPS[0]), 0, hT3, bern - hperm, bern - hperm + dnFr(TAPS[4]), bern - hperm + dnFr(TAPS[5])];
  if (e === 'pipe') { h[2] = -grad0 * 0.2; h[3] = -grad0 * 0.4; h[4] = -grad0 * 0.7; h[5] = -grad0 * 1.0; }
  const vel = [vup, vup, vth || (e === 'contraction' ? vdn : (e === 'expansion' ? vup : vup)), vdn, vdn, vdn];
  const totalBoard = -h[5] + upFr(TAPS[0]) * -1 + 0; // total head loss T1→T6 (static) later corrected by velocity heads
  return { v0, Re0, f0, hv0, Dup, Ddn, Dth, vup, vdn, vth, dpMeter, hperm, Kref, cdEff, h, vel, note, gradUp, gradDn, beta: o.beta || null };
}
function solve(s) {
  const fl = waterProps(s.temp), n = s.speedActual / 100;
  const Kth = throttleK(s.throttle / 100);
  const sysHead = Q => { const b = board(Q, s, fl); const hsup = (friction(b.Re0, D0, ROUGH.smooth) * SUPPLY.L / D0 + SUPPLY.K) * b.hv0; const hboard = b.gradUp * (ELEM[0]) + b.hperm + b.gradDn * (BOARD - ELEM[1]); return { H: SUPPLY.Hs + hsup + hboard + Kth * b.hv0, hsup, b }; };
  let Q = 0;
  if (n > 0.02 && pumpHead(0, n) > SUPPLY.Hs) { let lo = 0, hi = runout(n); for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; if (pumpHead(m, n) > sysHead(m).H) lo = m; else hi = m; } Q = (lo + hi) / 2; }
  const sh = sysHead(Q), b = sh.b, H = pumpHead(Q, n);
  // absolute static head at T2 (datum: board centreline) then all taps
  const hT2 = H - sh.hsup - SUPPLY.z - b.gradUp * (ELEM[0] - TAPS[1]) - b.gradUp * TAPS[1] + b.gradUp * (TAPS[1]) - b.gradUp * (TAPS[1] - 0) * 0; // = H - hsup - z - friction to T2
  const hT2c = H - sh.hsup - SUPPLY.z - b.gradUp * TAPS[1];
  const taps = TAPS.map((x, i) => { const hs = hT2c + b.h[i]; const hv = b.vel[i] * b.vel[i] / (2 * G); return { x, D: i === 2 && b.Dth ? b.Dth : (i < 2 ? b.Dup : b.Ddn), v: b.vel[i], hs, hv, ht: hs + hv, p: fl.rho * G * hs }; });
  // throat cavitation (meters): absolute pressure at T3 below vapour pressure
  let cav = false;
  if ((s.el === 'venturi' || s.el === 'orifice') && n > 0.05 && Q > 0) { const pabs = PATM + taps[2].p; if (pabs < fl.pv) { cav = true; taps[2].hs = (fl.pv - PATM) / (fl.rho * G); taps[2].p = fl.pv - PATM; taps[2].ht = taps[2].hs + taps[2].hv; } }
  // blocked tap: T3 frozen at last good value
  if (s.tapFault) { if (state.stale == null) state.stale = { hs: taps[2].hs, hv: taps[2].hv }; taps[2].hs = state.stale.hs; taps[2].ht = state.stale.hs + state.stale.hv; taps[2].p = fl.rho * G * taps[2].hs; } else state.stale = null;
  const dpTaps = (taps[1].hs - taps[3].hs) * fl.rho * G;         // across element T2–T4 (Pa)
  const dpMeter = (taps[1].hs - taps[2].hs) * fl.rho * G;        // T2–T3 (meter reading)
  const dpPerm = (taps[0].ht - taps[5].ht) * fl.rho * G;         // total head loss T1–T6
  const K = b.hv0 > 0 ? (taps[1].hs - taps[3].hs + (taps[1].hv - taps[3].hv)) / b.hv0 : 0;
  // meter-inferred flow with the learner's Cd
  let Qmeter = null, err = null;
  if (s.el === 'orifice' || s.el === 'venturi') { const beta = s.opt[s.el].beta, At = Math.PI * Math.pow(D0 * beta, 2) / 4; const dp = Math.max(0, dpMeter); Qmeter = s.opt[s.el].cd * At * Math.sqrt(2 * dp / (fl.rho * (1 - Math.pow(beta, 4)))) * 3600; err = Q > 0.1 ? (Qmeter - Q) / Q * 100 : null; }
  return { fl, n, rpm: n * PUMP.N0, Q, H, b, taps, dpTaps, dpMeter, dpPerm, K, Qmeter, err, cav, lowRe: s.el === 'orifice' && n > 0.05 && b.Re0 < 5000 && Q > 0.1, Kth, hsup: sh.hsup };
}

/* ------------------------------------------------------------------ state */
const state = { el: 'pipe', opt: JSON.parse(JSON.stringify(Object.fromEntries(Object.entries(ELEMENTS).map(([k, v]) => [k, v.def])))), speed: 0, speedActual: 0, throttle: 100, temp: 20, tapFault: false, stale: null, log: [], sweep: null, flags: {}, shown: null };
let R = solve(state);
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);
function setting(s = state) { const o = s.opt[s.el]; switch (s.el) { case 'pipe': return o.surface; case 'orifice': case 'venturi': return 'β ' + o.beta; case 'expansion': return 'DN40→DN' + o.d2; case 'contraction': return 'DN' + o.d1 + '→DN40'; case 'elbows': return o.count + '× ' + o.type; case 'valve': return o.type + ' ' + o.open + ' %'; } }

/* ------------------------------------------------------------------ rig svg */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent, anchor) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; if (anchor) t.style.textAnchor = anchor; return t; }
const X0 = 60, PXM = 368, YL = 400, HSCALE = 9.5, YTOP = 74;   // px per m along the board; head scale px per m
const px = x => X0 + x * PXM;
const rig = {};
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 560', role: 'img' }, $('#rig')); rig.svg = svg;
  // head scale on the left
  for (let m = 0; m <= 30; m += 5) { const y = YL - m * HSCALE; el('line', { x1: 30, y1: y, x2: 880, y2: y, class: 'scale' }, svg); txt(26, y + 3, m + ' m', 'scale-txt', svg, 'end'); }
  txt(30, 46, 'STATIC HEAD ABOVE BOARD CENTRELINE (PIEZOMETER COLUMNS)', 'label', svg);
  // pump + supply on the left, throttle + flowmeter on the right
  el('circle', { cx: 28, cy: YL, r: 16, class: 'pump-body' }, svg); rig.imp = el('g', { transform: `translate(28 ${YL})` }, svg);
  for (let i = 0; i < 6; i++) el('path', { d: 'M0 0 Q 5 -3 10 -8', transform: `rotate(${i * 60})`, class: 'impeller' }, rig.imp);
  txt(28, YL + 30, 'SUPPLY', 'label', svg, 'middle'); rig.rpmTxt = txt(28, YL + 42, '', 'gauge-val', svg, 'middle');
  // pipe runs: upstream (0..ELEM0), element block, downstream (ELEM1..BOARD)
  rig.upEdge = el('line', { x1: px(0), y1: YL, x2: px(ELEM[0]), y2: YL, class: 'pipe-edge' }, svg);
  rig.up = el('line', { x1: px(0), y1: YL, x2: px(ELEM[0]), y2: YL, class: 'pipe' }, svg);
  rig.upF = el('line', { x1: px(0), y1: YL, x2: px(ELEM[0]), y2: YL, class: 'pipe-fluid' }, svg);
  rig.dnEdge = el('line', { x1: px(ELEM[1]), y1: YL, x2: px(BOARD) + 20, y2: YL, class: 'pipe-edge' }, svg);
  rig.dn = el('line', { x1: px(ELEM[1]), y1: YL, x2: px(BOARD) + 20, y2: YL, class: 'pipe' }, svg);
  rig.dnF = el('line', { x1: px(ELEM[1]), y1: YL, x2: px(BOARD) + 20, y2: YL, class: 'pipe-fluid' }, svg);
  rig.elemG = el('g', {}, svg);
  // throttle valve and flowmeter after the board
  el('path', { d: `M${px(BOARD) + 30} ${YL - 12} L${px(BOARD) + 52} ${YL} L${px(BOARD) + 30} ${YL + 12} Z`, class: 'valve-body' }, svg);
  el('path', { d: `M${px(BOARD) + 74} ${YL - 12} L${px(BOARD) + 52} ${YL} L${px(BOARD) + 74} ${YL + 12} Z`, class: 'valve-body' }, svg);
  txt(px(BOARD) + 52, YL + 30, 'THROTTLE', 'label', svg, 'middle'); rig.thTxt = txt(px(BOARD) + 52, YL + 42, '', 'gauge-val', svg, 'middle');
  txt(px(BOARD) + 52, YL - 22, 'FLOWMETER', 'label', svg, 'middle'); rig.qTxt = txt(px(BOARD) + 52, YL - 34, '', 'dp-txt', svg);
  // tapping points with piezometer tubes
  rig.tubes = TAPS.map((x, i) => {
    const cx = px(x);
    el('line', { x1: cx, y1: YL - 8, x2: cx, y2: YTOP, class: 'tube' }, svg); el('line', { x1: cx, y1: YL - 8, x2: cx, y2: YTOP, class: 'tube', 'stroke-width': 10, opacity: .35 }, svg);
    const col = el('rect', { x: cx - 4, y: YL, width: 8, height: 0, class: 'column', rx: 2 }, svg);
    el('circle', { cx, cy: YL, r: 3, class: 'tap-dot' }, svg);
    txt(cx, YL + 56, 'T' + (i + 1), 'tag', svg, 'middle'); txt(cx, YL + 68, x.toFixed(1) + ' m', 'scale-txt', svg, 'middle');
    const val = txt(cx, YTOP - 8, '', 'gauge-val', svg, 'middle');
    return { col, val, cx };
  });
  rig.hgl = el('polyline', { class: 'hgl', points: '' }, svg); rig.egl = el('polyline', { class: 'egl', points: '' }, svg);
  rig.stale = el('rect', { x: px(TAPS[2]) - 6, y: YL - 8, width: 12, height: 10, class: 'stale', opacity: 0 }, svg);
  // ΔP boxes
  const box = (x, y, w, name) => { el('rect', { x, y, width: w, height: 32, class: 'dp-box' }, svg); txt(x + w / 2, y + 12, name, 'gauge-name', svg); return txt(x + w / 2, y + 26, '', 'dp-txt', svg); };
  rig.dpBox = box(px(1.0) - 70, YL + 84, 140, 'ΔP T2–T4 · ELEMENT');
  rig.dpM = box(px(1.0) - 230, YL + 84, 140, 'ΔP T2–T3 · METER TAPS');
  rig.dpP = box(px(1.0) + 90, YL + 84, 150, 'TOTAL LOSS T1–T6');
  // particles
  rig.parts = []; const g = el('g', {}, svg); for (let i = 0; i < 40; i++) rig.parts.push({ s: i / 40, e: el('circle', { r: 2.2, class: 'particle' }, g) });
  rig.bubbles = el('g', {}, svg); for (let i = 0; i < 8; i++) el('circle', { r: 2 + (i % 3), class: 'bubble', opacity: 0 }, rig.bubbles);
  drawElement();
}
function widthFor(D) { return 8 + (D - 0.0273) / (0.0825 - 0.0273) * 18; }
function drawElement() {
  const g = rig.elemG; g.innerHTML = ''; const e = state.el, o = state.opt[e], x1 = px(ELEM[0]), x2 = px(ELEM[1]), xm = px(1.0);
  const wUp = widthFor(R.b.Dup), wDn = widthFor(R.b.Ddn);
  [rig.upEdge, rig.up, rig.upF].forEach((l, i) => l.setAttribute('stroke-width', wUp + [4, 0, -4][i]));
  [rig.dnEdge, rig.dn, rig.dnF].forEach((l, i) => l.setAttribute('stroke-width', wDn + [4, 0, -4][i]));
  const body = (w1, w2) => el('path', { d: `M${x1} ${YL - w1 / 2} L${x2} ${YL - w2 / 2} L${x2} ${YL + w2 / 2} L${x1} ${YL + w1 / 2} Z`, class: 'pipe', stroke: 'var(--steel-light)', 'stroke-width': 2 }, g);
  if (e === 'pipe') { body(wUp, wUp); txt(xm, YL - 26, o.surface.toUpperCase() + ' BORE', 'label', g, 'middle'); }
  else if (e === 'orifice') { body(wUp, wUp); const t = widthFor(R.b.Dth) / 2; el('rect', { x: xm - 3, y: YL - wUp / 2 - 10, width: 6, height: wUp / 2 + 10 - t, class: 'element' }, g); el('rect', { x: xm - 3, y: YL + t, width: 6, height: wUp / 2 + 10 - t, class: 'element' }, g); txt(xm, YL - 30, 'ORIFICE β ' + o.beta, 'label', g, 'middle'); }
  else if (e === 'venturi') { const t = widthFor(R.b.Dth) / 2, h = wUp / 2; el('path', { d: `M${x1} ${YL - h} L${xm - 8} ${YL - t} L${xm + 8} ${YL - t} L${x2 + 30} ${YL - h} L${x2 + 30} ${YL + h} L${xm + 8} ${YL + t} L${xm - 8} ${YL + t} L${x1} ${YL + h} Z`, class: 'element' }, g); txt(xm, YL - 30, 'VENTURI β ' + o.beta, 'label', g, 'middle'); }
  else if (e === 'expansion') { el('path', { d: `M${x1} ${YL - wUp / 2} L${xm} ${YL - wUp / 2} L${xm} ${YL - wDn / 2} L${x2} ${YL - wDn / 2} L${x2} ${YL + wDn / 2} L${xm} ${YL + wDn / 2} L${xm} ${YL + wUp / 2} L${x1} ${YL + wUp / 2} Z`, class: 'element' }, g); txt(xm, YL - 34, 'DN40 → DN' + o.d2, 'label', g, 'middle'); }
  else if (e === 'contraction') { el('path', { d: `M${x1} ${YL - wUp / 2} L${xm} ${YL - wUp / 2} L${xm} ${YL - wDn / 2} L${x2} ${YL - wDn / 2} L${x2} ${YL + wDn / 2} L${xm} ${YL + wDn / 2} L${xm} ${YL + wUp / 2} L${x1} ${YL + wUp / 2} Z`, class: 'element' }, g); txt(xm, YL - 34, 'DN' + o.d1 + ' → DN40', 'label', g, 'middle'); }
  else if (e === 'elbows') { const n = o.count, step = (x2 - x1) / n; let d = `M${x1} ${YL}`; for (let i = 0; i < n; i++) { const up = i % 2 === 0 ? -28 : 28; d += ` L${x1 + i * step + step / 2} ${YL} L${x1 + i * step + step / 2} ${YL + up} L${x1 + (i + 1) * step} ${YL + up} L${x1 + (i + 1) * step} ${YL}`; } el('path', { d, class: 'element-line', 'stroke-width': wUp }, g); el('path', { d, class: 'pipe-fluid', 'stroke-width': wUp - 5 }, g); txt(xm, YL - 40, n + ' × 90° ' + o.type.toUpperCase(), 'label', g, 'middle'); }
  else if (e === 'valve') { body(wUp, wUp); el('path', { d: `M${xm - 16} ${YL - 14} L${xm + 16} ${YL + 14} L${xm + 16} ${YL - 14} L${xm - 16} ${YL + 14} Z`, class: 'valve-body' }, g); const x = o.open / 100; el('rect', { x: xm - 3, y: YL - 14 * x, width: 6, height: 28 * x, fill: 'var(--panel)' }, g); el('line', { x1: xm, y1: YL - 14, x2: xm, y2: YL - 30, class: 'gauge-tick' }, g); el('rect', { x: xm - 9, y: YL - 38, width: 18, height: 8, class: 'valve-body' }, g); txt(xm, YL - 46, o.type.toUpperCase() + ' VALVE ' + o.open + ' %', 'label', g, 'middle'); }
}
function updateRig(dt) {
  const t = R.taps;
  t.forEach((tp, i) => { const h = Math.max(0, Math.min(34, tp.hs)); rig.tubes[i].col.setAttribute('y', YL - h * HSCALE); rig.tubes[i].col.setAttribute('height', h * HSCALE); rig.tubes[i].val.textContent = fmt(tp.hs, 2) + ' m'; });
  rig.hgl.setAttribute('points', t.map((tp, i) => `${rig.tubes[i].cx},${YL - Math.max(-2, Math.min(34, tp.hs)) * HSCALE}`).join(' '));
  rig.egl.setAttribute('points', t.map((tp, i) => `${rig.tubes[i].cx},${YL - Math.max(-2, Math.min(34, tp.ht)) * HSCALE}`).join(' '));
  rig.stale.setAttribute('opacity', state.tapFault ? 1 : 0);
  rig.dpBox.textContent = fmt(R.dpTaps / 1000, 2) + ' kPa'; rig.dpM.textContent = fmt(R.dpMeter / 1000, 2) + ' kPa'; rig.dpP.textContent = fmt(R.dpPerm / 1000, 2) + ' kPa';
  rig.qTxt.textContent = fmt(R.Q, 1) + ' m³/h'; rig.thTxt.textContent = state.throttle + ' %'; rig.rpmTxt.textContent = Math.round(R.rpm) + ' rpm';
  rig.imAngle = (rig.imAngle || 0) + R.rpm / 60 * 360 * dt * 0.08; rig.imp.setAttribute('transform', `translate(28 ${YL}) rotate(${rig.imAngle % 360})`);
  const flowing = R.Q > 0.05; [rig.upF, rig.dnF].forEach(e => e.style.opacity = flowing ? .75 : .25);
  const len = px(BOARD) + 20 - px(0);
  for (const p of rig.parts) { p.s = (p.s + R.b.v0 * 60 * dt / len) % 1; const x = px(0) + p.s * len; p.e.setAttribute('cx', x); p.e.setAttribute('cy', YL); p.e.style.opacity = flowing && (x < px(ELEM[0]) || x > px(ELEM[1])) ? .85 : 0; }
  rig.bubbles.querySelectorAll('circle').forEach((b, i) => { if (R.cav) { const u = (performance.now() / 350 + i * .6) % 3; b.setAttribute('cx', px(1.0) + u * 30 + (i % 3) * 4); b.setAttribute('cy', YL - 8 + (i % 4) * 5); b.setAttribute('opacity', .9 - u / 3); } else b.setAttribute('opacity', 0); });
}

/* ------------------------------------------------------------------ chart */
const chart = { c: $('#chart'), Qmax: 40 };
function dpAt(Q, s) { const fl = waterProps(s.temp); const b = board(Q, s, fl); const hv1 = b.vup * b.vup / (2 * G), hv3 = b.vdn * b.vdn / (2 * G); return (b.h[1] - b.h[3]) * fl.rho * G; } // T2–T4 static ΔP
function drawChart() {
  const c = chart.c, ctx = c.getContext('2d'), dpr = window.devicePixelRatio || 1; const W = c.clientWidth || 900, Hh = W * 440 / 900;
  if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(Hh * dpr); } ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cs = getComputedStyle(document.documentElement), col = n => cs.getPropertyValue(n).trim();
  const m = { l: 58, r: 20, t: 16, b: 40 }, pw = W - m.l - m.r, ph = Hh - m.t - m.b;
  // y range from element curve at Qmax (kPa), min 20
  const ref = { ...state, el: 'pipe', opt: { ...state.opt, pipe: { surface: 'smooth' } } };
  let ymax = 20; for (let q = 5; q <= chart.Qmax; q += 5) ymax = Math.max(ymax, dpAt(q, state) / 1000); ymax = Math.min(ymax, 400);
  const nice = ymax <= 25 ? 25 : ymax <= 50 ? 50 : ymax <= 100 ? 100 : ymax <= 200 ? 200 : 400;
  const X = q => m.l + q / chart.Qmax * pw, Y = p => m.t + ph - Math.max(0, Math.min(p, nice * 1.05)) / nice * ph;
  ctx.clearRect(0, 0, W, Hh); ctx.font = '11px ' + col('--mono'); ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.lineWidth = 1;
  ctx.textAlign = 'center'; for (let q = 0; q <= chart.Qmax; q += 10) { ctx.beginPath(); ctx.moveTo(X(q), m.t); ctx.lineTo(X(q), m.t + ph); ctx.stroke(); ctx.fillText(q, X(q), Hh - m.b + 16); }
  ctx.textAlign = 'right'; for (let i = 0; i <= 5; i++) { const p = nice * i / 5; ctx.beginPath(); ctx.moveTo(m.l, Y(p)); ctx.lineTo(m.l + pw, Y(p)); ctx.stroke(); ctx.fillText(p, m.l - 8, Y(p) + 4); }
  ctx.textAlign = 'center'; ctx.font = '600 12px ' + col('--body'); ctx.fillText('Flow Q  (m³/h)', m.l + pw / 2, Hh - 6);
  ctx.save(); ctx.translate(14, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('ΔP across element, T2–T4  (kPa)', 0, 0); ctx.restore();
  const plot = (fn, style, width, dash) => { ctx.beginPath(); ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash || []); let first = true; for (let q = 0; q <= chart.Qmax; q += 0.5) { const y = fn(q) / 1000; if (y > nice * 1.05) break; first ? ctx.moveTo(X(q), Y(y)) : ctx.lineTo(X(q), Y(y)); first = false; } ctx.stroke(); ctx.setLineDash([]); };
  plot(q => dpAt(q, ref), col('--water-soft'), 2, [6, 5]);
  plot(q => dpAt(q, state), col('--brass'), 3);
  // recorded points
  for (const r of state.log) { const same = r.el === state.el && r.setting === setting(); ctx.fillStyle = same ? col('--ink') : col('--steel-light'); ctx.beginPath(); ctx.arc(X(r.Q), Y(r.dpT), same ? 4 : 3, 0, 7); ctx.fill(); }
  // current point
  if (R.n > 0.02 && state.shown) { const q = state.shown.Q, p = state.shown.dp / 1000; const pulse = 6 + 2 * Math.sin(performance.now() / 300); ctx.fillStyle = R.cav ? col('--alarm') : col('--brass'); ctx.beginPath(); ctx.arc(X(q), Y(p), pulse, 0, 7); ctx.fill(); ctx.fillStyle = col('--panel'); ctx.beginPath(); ctx.arc(X(q), Y(p), 2.5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.font = '600 12px ' + col('--mono'); ctx.textAlign = 'left'; ctx.fillText(`Q ${fmt(q, 1)} m³/h · ΔP ${fmt(p, 2)} kPa · K ${fmt(R.K, 2)}`, Math.min(X(q) + 12, m.l + pw - 230), Math.max(Y(p) - 12, m.t + 12)); }
  else if (R.n <= 0.02) { ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.font = '600 14px ' + col('--body'); ctx.fillText('Pump stopped — raise supply speed to start the flow', m.l + pw / 2, m.t + ph / 2); }
}

/* ------------------------------------------------------------------ ui */
const READ = [
  ['Q', 'Flow (reference meter)', () => fmt(R.Q, 1), 'm³/h'], ['v', 'Main-line velocity', () => fmt(R.b.v0, 2), 'm/s'],
  ['Re', 'Reynolds number', () => Math.round(R.b.Re0).toLocaleString(), () => R.b.Re0 < 2300 ? 'laminar' : R.b.Re0 < 4000 ? 'transitional' : 'turbulent'],
  ['dp', 'ΔP across element T2–T4', () => fmt(R.dpTaps / 1000, 2), 'kPa'], ['perm', 'Total head loss T1–T6', () => fmt(R.dpPerm / 1000, 2), 'kPa'],
  ['K', 'Loss coefficient K', () => fmt(R.K, 2), 'on main velocity head'], ['Qm', 'Meter-inferred flow', () => fmt(R.Qmeter, 2), 'm³/h'],
  ['err', 'Meter error', () => fmt(R.err, 1), '%', () => R.err !== null && Math.abs(R.err) > 2],
  ['f', 'Friction factor f', () => fmt(R.b.f0, 4), 'Darcy'], ['hv', 'Velocity head v²/2g', () => fmt(R.b.hv0, 3), 'm'],
  ['vth', 'Throat velocity', () => fmt(R.b.vth, 2), 'm/s'], ['pth', 'Absolute pressure at T3', () => fmt((PATM + R.taps[2].p) / 1000, 1), 'kPa abs', () => R.cav],
  ['H', 'Pump head', () => fmt(R.H, 1), 'm'], ['rpm', 'Pump speed', () => Math.round(R.rpm), 'rpm'],
  ['cd', 'Discharge coefficient in use', () => fmt(R.b.cdEff || null, 3), 'actual'], ['Kth', 'Throttle K', () => fmt(R.Kth, 1), '—'],
];
const TOP = ['Q', 'v', 'Re', 'dp', 'perm', 'K', 'Qm', 'err'];
function buildReadouts() { const mk = (box, keys) => { box.innerHTML = ''; for (const r of READ) { if (keys && !keys.includes(r[0])) continue; const d = document.createElement('div'); d.className = 'ro'; d.dataset.k = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u"></span>`; box.appendChild(d); } }; mk($('#readouts'), TOP); mk($('#dataGrid'), null); }
function updateReadouts() { document.querySelectorAll('.ro').forEach(d => { const r = READ.find(x => x[0] === d.dataset.k); d.querySelector('.v').textContent = r[2](); d.querySelector('.u').textContent = typeof r[3] === 'function' ? r[3]() : r[3]; d.classList.toggle('alarm', !!(r[4] && r[4]())); }); }
function updateTapTable() { const tb = $('#tapTable tbody'); tb.innerHTML = ''; R.taps.forEach((t, i) => { const tr = document.createElement('tr'); tr.innerHTML = `<td>T${i + 1}${i === 2 && R.b.note ? ' (' + R.b.note + ')' : ''}</td><td>${t.x.toFixed(1)}</td><td>${(t.D * 1000).toFixed(1)}</td><td>${fmt(t.v, 2)}</td><td>${fmt(t.hs, 2)}</td><td>${fmt(t.hv, 3)}</td><td>${fmt(t.ht, 2)}</td><td>${fmt(t.p / 1000, 1)}</td>`; tb.appendChild(tr); }); }
function updateStatus() {
  const run = R.n > 0.02; $('#pillRun').textContent = run ? `Supply pump · ${Math.round(R.rpm)} rpm` : 'Pump stopped'; $('#pillRun').classList.toggle('on', run);
  $('#pillElem').textContent = ELEMENTS[state.el].name + ' · ' + setting() + (run ? ` · Q ${fmt(R.Q, 1)} m³/h` : '');
  $('#pillCav').classList.toggle('hidden', !R.cav); $('#pillCav').textContent = `Throat cavitation · p₃ at vapour pressure — meter reading invalid`;
  $('#pillRe').classList.toggle('hidden', !R.lowRe); $('#pillRe').textContent = `Low Reynolds number · Re ${Math.round(R.b.Re0).toLocaleString()} — orifice Cd drifting`;
  $('#pillTap').classList.toggle('hidden', !state.tapFault); $('#pillTap').textContent = 'Tap T3 blocked · reading is stale';
  state.flags.cav = state.flags.cav || R.cav;
}
function buildOpts() {
  const box = $('#elemOpts'); box.innerHTML = ''; const e = state.el, def = ELEMENTS[e], o = state.opt[e];
  const wrap = document.createElement('div'); wrap.className = 'opts';
  const lab = document.createElement('label'); lab.textContent = def.name + ' options'; wrap.appendChild(lab);
  for (const [k, vals] of Object.entries(def.opts)) { const row = document.createElement('div'); row.className = 'row'; const name = { surface: 'Bore surface', beta: 'Throat ratio β', d2: 'Downstream bore', d1: 'Upstream bore', count: 'Number of elbows', type: 'Type' }[k]; row.innerHTML = `<span>${name}</span>`; const seg = document.createElement('div'); seg.className = 'seg'; vals.forEach(v => { const b = document.createElement('button'); b.textContent = k === 'd2' || k === 'd1' ? 'DN' + v : String(v); b.className = String(o[k]) === String(v) ? 'on' : ''; b.onclick = () => { o[k] = v; buildOpts(); R = solve(state); drawElement(); }; seg.appendChild(b); }); row.appendChild(seg); wrap.appendChild(row); }
  if (e === 'valve') { const row = document.createElement('div'); row.className = 'row'; row.innerHTML = `<span>Opening</span><input type="range" id="vopen" min="10" max="100" step="5" value="${o.open}" style="flex:1"><output id="vopenOut">${o.open} %</output>`; wrap.appendChild(row); row.querySelector('input').addEventListener('input', ev => { o.open = +ev.target.value; row.querySelector('output').textContent = o.open + ' %'; drawElement(); }); }
  if (e === 'orifice' || e === 'venturi') { const row = document.createElement('div'); row.className = 'row'; row.innerHTML = `<span>Your calibration C<sub>d</sub></span><input type="range" id="cd" min="0.50" max="1.00" step="0.005" value="${o.cd}" style="flex:1"><output id="cdOut">${o.cd.toFixed(3)}</output>`; wrap.appendChild(row); row.querySelector('input').addEventListener('input', ev => { o.cd = +ev.target.value; row.querySelector('output').textContent = o.cd.toFixed(3); }); }
  box.appendChild(wrap);
}
function selectElement(e) { state.el = e; document.querySelectorAll('#elemSeg button').forEach(b => b.classList.toggle('on', b.dataset.el === e)); state.stale = null; buildOpts(); R = solve(state); drawElement(); }

/* ------------------------------------------------------------------ log & tasks */
function record(tag) { const row = { i: state.log.length + 1, el: state.el, name: ELEMENTS[state.el].name, setting: setting(), Q: +R.Q.toFixed(2), v: +R.b.v0.toFixed(2), Re: Math.round(R.b.Re0), dpT: +(R.dpTaps / 1000).toFixed(2), perm: +(R.dpPerm / 1000).toFixed(2), K: +R.K.toFixed(2), Qm: R.Qmeter === null ? '' : +R.Qmeter.toFixed(2), err: R.err === null ? '' : +R.err.toFixed(1), tag }; state.log.push(row); renderLog(); checkTasks(); }
function renderLog() { const tb = $('#logTable tbody'); tb.innerHTML = ''; for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.name, r.setting, r.Q, r.v, r.Re, r.dpT, r.perm, r.K, r.Qm, r.err].map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); } $('#logCount').textContent = state.log.length; const csv = ['n,element,setting,Q_m3h,v_ms,Re,dP_T2T4_kPa,total_loss_kPa,K,meter_Q_m3h,meter_error_pct'].concat(state.log.map(r => [r.i, r.name, r.setting, r.Q, r.v, r.Re, r.dpT, r.perm, r.K, r.Qm, r.err].map(v => `"${v}"`).join(','))).join('\n'); state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv); }
const TASKS = [
  { id: 'bern', t: 'Bernoulli at the venturi throat', d: 'Fit the venturi and run at least 8 m³/h. Record a point. Check in the tapping-point table that the static head at T3 falls while the total head barely changes — that is Bernoulli, not a loss.', ok: () => state.log.some(r => r.el === 'venturi' && r.Q >= 8) },
  { id: 'recov', t: 'Pressure recovery in an expansion', d: 'Fit the sudden expansion and run at least 8 m³/h. The T4 column stands higher than T2: static pressure rises as the fluid slows, even though energy is lost. Record it.', ok: () => state.log.some(r => r.el === 'expansion' && r.Q >= 8), st: () => { const t = R.taps; return state.el === 'expansion' && R.Q > 1 ? `now: T2 ${fmt(t[1].hs, 2)} m → T4 ${fmt(t[3].hs, 2)} m, energy line ${fmt(t[1].ht - t[3].ht, 2)} m lower` : ''; } },
  { id: 'fric', t: 'Friction scales with Q²', d: 'Straight pipe: record a point, then double the flow (within 10 %) and record again. Compare the ΔP ratio with 4.', ok: () => { const p = state.log.filter(r => r.el === 'pipe'); return p.some(a => p.some(b => b.Q > 1.8 * a.Q && b.Q < 2.2 * a.Q && a.Q > 2)); }, st: () => { const p = state.log.filter(r => r.el === 'pipe'); for (const a of p) for (const b of p) if (b.Q > 1.8 * a.Q && b.Q < 2.2 * a.Q && a.Q > 2) return `ΔP ratio ${(b.dpT / a.dpT).toFixed(2)} for a flow ratio of ${(b.Q / a.Q).toFixed(2)}`; return 'record two straight-pipe points, one at double the flow'; } },
  { id: 'minor', t: 'Minor losses add up', d: 'Record one elbow and then four elbows of the same type at the same flow (within 5 %). The ΔP ratio should be close to 4; the K readout tells you the coefficient per elbow.', ok: () => { const e = state.log.filter(r => r.el === 'elbows'); return e.some(a => a.setting.startsWith('1×') && e.some(b => b.setting.startsWith('4×') && Math.abs(b.Q - a.Q) / a.Q < 0.05)); } },
  { id: 'cal', t: 'Calibrate the orifice', d: 'Fit the orifice, run at least 8 m³/h and adjust your Cd until the meter-inferred flow agrees with the reference flowmeter within 1 %.', ok: () => state.el === 'orifice' && R.Q >= 8 && R.err !== null && Math.abs(R.err) < 1, st: () => state.el === 'orifice' && R.err !== null ? `current error ${fmt(R.err, 1)} % with Cd ${state.opt.orifice.cd.toFixed(3)}` : '' },
  { id: 'sys', t: 'Trace an element curve', d: 'Auto-sweep any element. Six or more points trace the ΔP–Q parabola: this is the element\'s contribution to a system curve.', ok: () => Object.values(state.log.reduce((m, r) => { const k = r.el + r.setting; m[k] = (m[k] || 0) + 1; return m; }, {})).some(n => n >= 6), st: () => { const groups = state.log.reduce((m, r) => { (m[r.el + r.setting] = m[r.el + r.setting] || []).push(r); return m; }, {}); for (const g of Object.values(groups)) if (g.length >= 6) { const xs = g.map(r => Math.log(r.Q)), ys = g.map(r => Math.log(Math.max(r.dpT, 0.01))); const n = xs.length, mx = xs.reduce((a, b) => a + b) / n, my = ys.reduce((a, b) => a + b) / n; const sl = xs.reduce((a, x, i) => a + (x - mx) * (ys[i] - my), 0) / xs.reduce((a, x) => a + (x - mx) ** 2, 0); return `fitted exponent ΔP ∝ Q^${sl.toFixed(2)}`; } return ''; } },
  { id: 'cav', t: 'Cavitate the venturi throat', d: 'Venturi with β 0.5, fluid at 80 °C, everything open at 100 % speed. When the throat pressure reaches vapour pressure the reading stops following the flow.', ok: () => !!state.flags.cav },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (t.ok()) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  const ramp = 40 * dt; state.speedActual += Math.max(-ramp, Math.min(ramp, state.speed - state.speedActual)); if (Math.abs(state.speed - state.speedActual) < 0.05) state.speedActual = state.speed;
  if (state.sweep) { const sw = state.sweep; sw.t += dt; const steps = 8; const k = Math.min(steps - 1, Math.floor(sw.t / 1.2)); state.throttle = Math.round(8 + (100 - 8) * Math.pow((k + 1) / steps, 1.5)); $('#throttle').value = state.throttle; if (k > sw.rec && sw.t - k * 1.2 > 0.9 && state.speedActual === state.speed) { record('sweep'); sw.rec = k; } if (sw.rec >= steps - 1) { state.sweep = null; $('#btnSweep').textContent = 'Auto-sweep flow'; } }
  R = solve(state);
  if (!state.shown) state.shown = { Q: R.Q, dp: R.dpTaps }; const tau = 0.5; state.shown.Q += (R.Q - state.shown.Q) * Math.min(1, dt / tau); state.shown.dp += (R.dpTaps - state.shown.dp) * Math.min(1, dt / tau);
  updateRig(dt); drawChart();
  if (frame++ % 6 === 0) { updateReadouts(); updateStatus(); updateTapTable(); checkTasks(); $('#speedOut').textContent = state.speed + ' %'; $('#rpmOut').textContent = Math.round(R.rpm) + ' rpm' + (state.speedActual !== state.speed ? ' (ramping)' : ''); $('#throttleOut').textContent = state.throttle + ' %'; }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function bind() {
  $('#speed').addEventListener('input', e => state.speed = +e.target.value); $('#throttle').addEventListener('input', e => { state.throttle = +e.target.value; state.sweep = null; });
  $('#temp').addEventListener('input', e => { state.temp = +e.target.value; $('#tempOut').textContent = e.target.value + ' °C'; });
  $('#tapFault').addEventListener('change', e => { state.tapFault = e.target.checked; state.stale = null; });
  document.querySelectorAll('[data-speed]').forEach(b => b.onclick = () => { state.speed = +b.dataset.speed; $('#speed').value = state.speed; });
  document.querySelectorAll('#elemSeg button').forEach(b => b.onclick = () => selectElement(b.dataset.el));
  $('#btnRecord').onclick = () => record('manual');
  $('#btnSweep').onclick = () => { if (state.sweep) { state.sweep = null; $('#btnSweep').textContent = 'Auto-sweep flow'; return; } if (state.speed < 5) { state.speed = 100; $('#speed').value = 100; } state.sweep = { t: 0, rec: -1 }; $('#btnSweep').textContent = 'Stop sweep'; document.querySelector('[data-tab="log"]').click(); };
  $('#btnReset').onclick = () => { Object.assign(state, { speed: 0, throttle: 100, temp: 20, tapFault: false, stale: null, sweep: null, flags: {}, opt: JSON.parse(JSON.stringify(Object.fromEntries(Object.entries(ELEMENTS).map(([k, v]) => [k, v.def])))) }); $('#speed').value = 0; $('#throttle').value = 100; $('#temp').value = 20; $('#tempOut').textContent = '20 °C'; $('#tapFault').checked = false; selectElement('pipe'); document.querySelectorAll('.tasks li').forEach(l => l.classList.remove('done')); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('rb_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  const order = Object.keys(ELEMENTS);
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'BUTTON'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const step = e.shiftKey ? 10 : 2;
    if (e.key === 'ArrowUp') { state.speed = Math.min(100, state.speed + step); $('#speed').value = state.speed; } else if (e.key === 'ArrowDown') { state.speed = Math.max(0, state.speed - step); $('#speed').value = state.speed; }
    else if (e.key === 'ArrowRight') { state.throttle = Math.min(100, state.throttle + step); $('#throttle').value = state.throttle; } else if (e.key === 'ArrowLeft') { state.throttle = Math.max(5, state.throttle - step); $('#throttle').value = state.throttle; }
    else if (/^[1-7]$/.test(e.key)) selectElement(order[+e.key - 1]); else if (e.key === 'r' || e.key === 'R') record('manual'); else if (e.key === 's' || e.key === 'S') $('#btnSweep').click(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', drawChart);
}

/* ------------------------------------------------------------------ dashboard API */
window.RestrictionBoard = {
  getState: () => ({ inputs: { element: state.el, options: state.opt[state.el], speed: state.speed, throttle: state.throttle, temp: state.temp, tapFault: state.tapFault }, results: { Q: R.Q, dpTaps: R.dpTaps, dpPerm: R.dpPerm, K: R.K, Qmeter: R.Qmeter, err: R.err, taps: R.taps.map(t => ({ x: t.x, hs: t.hs, ht: t.ht })) }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.element && ELEMENTS[inp.element]) selectElement(inp.element); if (inp.options) Object.assign(state.opt[state.el], inp.options); for (const k of ['speed', 'throttle', 'temp']) if (inp[k] !== undefined) { state[k] = +inp[k]; $('#' + k).value = state[k]; } if (inp.tapFault !== undefined) { state.tapFault = !!inp.tapFault; $('#tapFault').checked = state.tapFault; } buildOpts(); drawElement(); },
  meta: { id: 'restriction-board', title: 'Restriction Board', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.RestrictionBoard.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'restriction-board', state: window.RestrictionBoard.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('rb_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  state.speed = 70; state.speedActual = 70; $('#speed').value = 70;
  R = solve(state);
  buildRig(); buildReadouts(); buildTasks(); buildOpts(); bind(); renderLog();
  selectElement('venturi');   // open on the most characteristic element: the Bernoulli demo
  requestAnimationFrame(tick);
}
build();
