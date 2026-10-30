/* Instrumented Pump Trainer — closed loop, two transparent pumps (single / parallel / series),
   full board instrumentation, thermal dynamics, and a curve-builder chart. Standalone, offline.
   Rig data illustrative (training only). */
'use strict';

/* ------------------------------------------------------------------ physics */
const G = 9.81, PATM = 101325;
const PUMP = { N0: 2900, H0: 32, a1: 0.2, a2: 0.01, Qbep: 24, etaMax: 0.72, npshr: Q => 1.5 + 0.004 * Q * Q, vanes: 6 };
const MOTOR = { kW: 4, V: 400, FLA: 8.2, etaFL: 0.88 };
const D = 41.9e-3, A = Math.PI * D * D / 4;
const LOOP = { Hs: 1.5, Kfix: 5, L: 12, Kinter: 1.5, tankM: 800, UA: 160, Tamb: 22 };
const FLUIDS = { water: { name: 'Water', rho0: 998, mu0: 1.0e-3, cp: 4186, pvScale: 1 }, glycol: { name: '30 % glycol', rho0: 1040, mu0: 2.6e-3, cp: 3700, pvScale: 0.85 } };
function props(fluidKey, T) { const f = FLUIDS[fluidKey]; const rho = f.rho0 - 0.0035 * (T - 4) ** 2 * 0.65; const mu = f.mu0 * Math.pow(10, 247.8 / (T + 273.15 - 140)) / Math.pow(10, 247.8 / (293.15 - 140)); const pv = f.pvScale * 610.94 * Math.exp(17.625 * T / (T + 243.04)); return { rho, mu, pv, cp: f.cp }; }
function friction(Re) { if (Re < 2300) return Math.max(64 / Math.max(Re, 1), 0.02); return 0.25 / Math.pow(Math.log10(0.0015e-3 / (3.7 * D) + 5.74 / Math.pow(Re, 0.9)), 2); }
function pumpHead(Q, n) { if (n <= 0.02) return 0; const q = Q / n; return n * n * Math.max(0, PUMP.H0 - PUMP.a1 * q - PUMP.a2 * q * q); }
function pumpEta(Q, n) { if (n <= 0.02 || Q <= 0) return 0; const q = Q / n; return Math.max(0.03, PUMP.etaMax * (1 - Math.pow((q - PUMP.Qbep) / PUMP.Qbep, 2))); }
function runout(n) { const b = PUMP.a1, a = PUMP.a2; return n * (-b + Math.sqrt(b * b + 4 * a * PUMP.H0)) / (2 * a); }
function pumpQatH(H, n) { if (n <= 0.02 || H >= pumpHead(0, n)) return 0; let lo = 0, hi = runout(n); for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (pumpHead(m, n) > H) lo = m; else hi = m; } return (lo + hi) / 2; }
function gateK(x) { const t = [[1, 0.2], [0.75, 1.15], [0.5, 5.6], [0.25, 24], [0.1, 100], [0.05, 400]]; x = Math.max(0.05, Math.min(1, x)); for (let i = 0; i < t.length - 1; i++) { const [x1, k1] = t[i], [x2, k2] = t[i + 1]; if (x <= x1 && x >= x2) { const u = (x1 - x) / (x1 - x2); return Math.exp(Math.log(k1) * (1 - u) + Math.log(k2) * u); } } return 400; }
function dKv(x) { return x < 0.02 ? 0 : 25 * Math.pow(30, x - 1); }
function headFactor(m) { if (m >= 0.6) return 1; if (m >= 0) return 1 - 0.03 * (0.6 - m) / 0.6; return Math.max(0.25, 0.97 - 0.55 * Math.min(1, -m / 1.5)); }

/* combined pump head for the arrangement at total flow Q (before cavitation factors) */
function combHead(Q, s) {
  const n1 = s.n1a / 100, n2 = s.n2a / 100;
  if (s.arr === 'single') return pumpHead(Q, n1);
  if (s.arr === 'series') return pumpHead(Q, n1) + pumpHead(Q, n2) - (n2 > 0.02 ? 0 : LOOP.Kinter * 0);
  // parallel: find H such that Q1(H)+Q2(H) = Q
  let lo = 0, hi = Math.max(pumpHead(0, n1), pumpHead(0, n2)); if (hi <= 0) return 0;
  for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (pumpQatH(m, n1) + pumpQatH(m, n2) > Q) lo = m; else hi = m; } return (lo + hi) / 2;
}
function solve(s) {
  const fl = props(s.fluid, s.Tf);
  const Ks = 2 + gateK(s.sthr / 100) + 60 * Math.pow(s.strainer / 100, 2), Kv = dKv(s.dthr / 100);
  const loss = Q => { const q = Q / 3600, v = q / A, Re = fl.rho * v * D / fl.mu, f = friction(Re), hv = v * v / (2 * G); const hs = (f * 3 / D + Ks) * hv; const hd = (LOOP.Kfix + f * LOOP.L / D) * hv + (Kv > 0 ? Math.pow(Q / Kv, 2) * 1e5 / (fl.rho * G) : Infinity); return { hs, hd, v, hv, Re }; };
  const anyOn = (s.n1a > 2) || (s.arr !== 'single' && s.n2a > 2);
  let Q = 0, fac1 = 1, fac2 = 1;
  if (anyOn && Kv > 0 && combHead(0, s) > LOOP.Hs) {
    for (let it = 0; it < 3; it++) {
      let lo = 0, hi = (s.arr === 'parallel' ? 2 : 1) * runout(Math.max(s.n1a, s.n2a) / 100) + 1;
      for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; const L = loss(m); if (combHeadFac(m, s, fac1, fac2) > LOOP.Hs + L.hd + (s.arr === 'series' ? 0 : L.hs) * 0) lo = m; else hi = m; }
      Q = (lo + hi) / 2;
      const sp = split(Q, s); const L = loss(Q);
      const npsha1 = PATM / (fl.rho * G) + s.level - lossS(sp.Q1 + (s.arr === 'parallel' ? sp.Q2 : 0), Ks, fl) - fl.pv / (fl.rho * G);
      fac1 = s.n1a > 2 ? headFactor(npsha1 - PUMP.npshr(sp.Q1 / Math.max(s.n1a / 100, 0.05)) * Math.pow(s.n1a / 100, 2)) : 1;
      if (s.arr === 'parallel') { fac2 = s.n2a > 2 ? headFactor(npsha1 - PUMP.npshr(sp.Q2 / Math.max(s.n2a / 100, 0.05)) * Math.pow(s.n2a / 100, 2)) : 1; }
      else if (s.arr === 'series') { const H1 = pumpHead(Q, s.n1a / 100) * fac1; const npsha2 = npsha1 + H1 - LOOP.Kinter * L.hv; fac2 = s.n2a > 2 ? headFactor(npsha2 - PUMP.npshr(Q / Math.max(s.n2a / 100, 0.05)) * Math.pow(s.n2a / 100, 2)) : 1; }
    }
  }
  const L = loss(Q), sp = split(Q, s);
  const n1 = s.n1a / 100, n2 = s.n2a / 100;
  const hsuc = lossS(s.arr === 'parallel' ? Q : (s.arr === 'single' ? Q : Q), Ks, fl);
  const npsha1 = PATM / (fl.rho * G) + s.level - hsuc - fl.pv / (fl.rho * G);
  // per-pump results
  const H1 = pumpHead(sp.Q1, n1) * fac1, H2 = s.arr === 'single' ? 0 : pumpHead(sp.Q2, n2) * fac2;
  const P1s = fl.rho * G * (s.level - hsuc);                                   // suction gauge pump 1 (Pa)
  const P1d = P1s + fl.rho * G * H1;
  const P2s = s.arr === 'series' ? P1d - fl.rho * G * LOOP.Kinter * L.hv : P1s;
  const P2d = P2s + fl.rho * G * H2;
  const Htot = s.arr === 'series' ? H1 + H2 - LOOP.Kinter * L.hv : (s.arr === 'parallel' ? Math.max(H1, H2) : H1);
  const pump = (Qp, n, H, fac, m) => {
    const eta = pumpEta(Qp, n), on = n > 0.02;
    const Phyd = fl.rho * G * (Qp / 3600) * H, Pshaft = on ? (Qp > 0.05 ? Phyd / eta : 0.28 * MOTOR.kW * 1000 * n * n * n) : 0;
    const load = Pshaft / (MOTOR.kW * 1000); const etaM = on ? MOTOR.etaFL * (load < 0.25 ? 0.6 + 1.1 * load : Math.min(1, 0.87 + 0.15 * load)) : 1; const pf = on ? 0.55 + 0.35 * Math.min(1, load) : 0;
    const Pelec = on ? Pshaft / etaM + 90 : 0; const I = on ? Pelec / (Math.sqrt(3) * MOTOR.V * pf) : 0;
    const npshr = on ? PUMP.npshr(Qp / Math.max(n, 0.05)) * n * n : 0, margin = m - npshr;
    const sV = on ? Math.pow(Math.max(0, Math.min(1, (2.0 - margin) / 2.6)), 1.4) : 0; const offCurve = on && Qp > 0 ? Math.abs(Qp / n - PUMP.Qbep) / PUMP.Qbep : 0;
    const vib = on ? 1.0 + 1.0 * n * n + 9 * sV + 2.5 * Math.pow(offCurve, 2) : 0.1;
    return { Q: Qp, H, eta, Phyd, Pshaft, Pelec, I, pf, etaM, load, npshr, margin, vib, on, cav: on && margin < 0.6, fac };
  };
  const p1 = pump(sp.Q1, n1, H1, fac1, npsha1);
  const p2 = pump(s.arr === 'single' ? 0 : sp.Q2, s.arr === 'single' ? 0 : n2, H2, fac2, s.arr === 'series' ? npsha1 + H1 - LOOP.Kinter * L.hv : npsha1);
  const Phyd = fl.rho * G * (Q / 3600) * Htot, Pel = p1.Pelec + p2.Pelec, Psh = p1.Pshaft + p2.Pshaft;
  return { fl, Q, Htot, L, p1, p2, P1s, P1d, P2s, P2d, npsha1, hsuc, Ks, Kv, Phyd, Pel, Psh, etaOverall: Pel > 0 ? Phyd / Pel : 0, etaPump: Psh > 0 ? Phyd / Psh : 0, anyOn };
}
function combHeadFac(Q, s, f1, f2) { const n1 = s.n1a / 100, n2 = s.n2a / 100; if (s.arr === 'single') return pumpHead(Q, n1) * f1; if (s.arr === 'series') return pumpHead(Q, n1) * f1 + pumpHead(Q, n2) * f2; let lo = 0, hi = Math.max(pumpHead(0, n1) * f1, pumpHead(0, n2) * f2); if (hi <= 0) return 0; const qAt = (H, n, f) => (n <= 0.02 || f <= 0) ? 0 : pumpQatH(H / f, n); for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (qAt(m, n1, f1) + qAt(m, n2, f2) > Q) lo = m; else hi = m; } return (lo + hi) / 2; }
function split(Q, s) { if (s.arr !== 'parallel') return { Q1: Q, Q2: s.arr === 'series' ? Q : 0 }; const n1 = s.n1a / 100, n2 = s.n2a / 100; const H = combHead(Q, s); const q1 = pumpQatH(H, n1), q2 = pumpQatH(H, n2); const t = q1 + q2 || 1; return { Q1: Q * q1 / t, Q2: Q * q2 / t }; }
function lossS(Q, Ks, fl) { const q = Q / 3600, v = q / A, Re = fl.rho * v * D / fl.mu; return (friction(Re) * 3 / D + Ks) * v * v / (2 * G); }

/* ------------------------------------------------------------------ state */
const state = { arr: 'single', n1: 0, n2: 0, n1a: 0, n2a: 0, dthr: 60, sthr: 100, level: 1.0, strainer: 0, fluid: 'water', Tf: 22, Tb1: 22, Tb2: 22, accel: 1, log: [], sweep: null, flags: {}, shown: { Q: 0, H: 0 } };
let R = solve(state);
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);

/* ------------------------------------------------------------------ rig svg */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent, anchor) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; if (anchor) t.style.textAnchor = anchor; return t; }
const rig = {}; const P1 = { x: 400, y: 400 }, P2 = { x: 640, y: 400 }, PR = 42, HY = 140, SY = 400;
function pipe(svg, d, w = 12, key) { el('path', { d, class: 'pipe-edge', 'stroke-width': w + 4 }, svg); el('path', { d, class: 'pipe', 'stroke-width': w }, svg); const f = el('path', { d, class: 'pipe-fluid', 'stroke-width': w - 4 }, svg); if (key) rig[key] = f; return f; }
function valveSym(svg, x, y, vert, key) { const g = el('g', { transform: `translate(${x} ${y}) rotate(${vert ? 90 : 0})` }, svg); el('path', { d: 'M-14 -10 L0 0 L-14 10 Z', class: 'valve-body' }, g); el('path', { d: 'M14 -10 L0 0 L14 10 Z', class: 'valve-body' }, g); const gate = el('rect', { x: -2, y: -9, width: 4, height: 18, fill: 'var(--panel)' }, g); if (key) rig[key] = gate; return g; }
function cross(svg, x, y) { const g = el('g', {}, svg); el('line', { x1: x - 8, y1: y - 8, x2: x + 8, y2: y + 8, class: 'tap' }, g); el('line', { x1: x - 8, y1: y + 8, x2: x + 8, y2: y - 8, class: 'tap' }, g); return g; }
function gauge(svg, cx, cy, name, min, max, unit, key) { const g = el('g', { transform: `translate(${cx} ${cy})` }, svg); el('circle', { r: 20, class: 'gauge-face' }, g); for (let i = 0; i <= 8; i++) { const a = -135 + i * 33.75, r = Math.PI * a / 180; el('line', { x1: 13 * Math.sin(r), y1: -13 * Math.cos(r), x2: 17 * Math.sin(r), y2: -17 * Math.cos(r), class: 'gauge-tick' }, g); } const needle = el('line', { x1: 0, y1: 3, x2: 0, y2: -15, class: 'gauge-needle' }, g); el('circle', { r: 2, fill: 'var(--alarm)' }, g); txt(0, 30, name, 'gauge-name', g); const val = txt(0, 41, '', 'gauge-val', g); rig[key] = { set(v) { const t = Math.max(0, Math.min(1, (v - min) / (max - min))); needle.setAttribute('transform', `rotate(${-135 + 270 * t})`); val.textContent = fmt(v, 0) + ' ' + unit; } }; }
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 560', role: 'img' }, $('#rig')); rig.svg = svg;
  // tank (return enters the top, suction leaves the bottom right)
  el('rect', { x: 40, y: 190, width: 170, height: 300, class: 'reservoir', rx: 4 }, svg); rig.fluid = el('rect', { x: 42, y: 330, width: 166, height: 158, class: 'fluid' }, svg);
  txt(125, 508, 'TANK · CLOSED LOOP · 0.8 m³', 'label', svg, 'middle'); el('line', { x1: 40, y1: SY, x2: 210, y2: SY, class: 'tap', 'stroke-dasharray': '4 4' }, svg); txt(46, SY - 4, 'pump-eye datum', 'scale-txt', svg); rig.levelTxt = txt(200, 206, '', 'gauge-val', svg, 'end');
  el('rect', { x: 60, y: 440, width: 10, height: 26, class: 'sensor', rx: 4 }, svg); el('circle', { cx: 65, cy: 470, r: 5, fill: 'var(--alarm)' }, svg); txt(65, 432, 'T fluid', 'gauge-name', svg); rig.tfTxt = txt(65, 422, '', 'gauge-val', svg, 'middle');
  // suction: tank → strainer → suction throttle → header at SY through P1 to P2
  pipe(svg, `M210 470 L300 470 L300 ${SY} L${P1.x - PR - 2} ${SY}`, 12, 'suc');
  rig.sucP2f = pipe(svg, `M${P1.x + PR + 2} ${SY} L${P2.x - PR - 2} ${SY}`, 12);
  el('rect', { x: 232, y: 460, width: 24, height: 20, class: 'flowmeter', rx: 2 }, svg); for (let i = 1; i < 4; i++) el('line', { x1: 232 + i * 6, y1: 460, x2: 232 + i * 6, y2: 480, class: 'gauge-tick' }, svg); rig.strFill = el('rect', { x: 232, y: 460, width: 24, height: 20, fill: 'var(--brass)', opacity: 0, rx: 2 }, svg); txt(244, 498, 'STRAINER', 'label', svg, 'middle');
  valveSym(svg, 300, 436, true, 'sthrGate'); txt(284, 432, 'SUCTION', 'label', svg, 'end'); txt(284, 444, 'THROTTLE', 'label', svg, 'end');
  // discharge risers to the top header; header returns left into the tank top via throttle and flowmeter
  rig.dis1f = pipe(svg, `M${P1.x} ${P1.y - PR} L${P1.x} ${HY}`, 12);
  rig.dis2f = pipe(svg, `M${P2.x} ${P2.y - PR} L${P2.x} ${HY} L${P1.x} ${HY}`, 12);
  pipe(svg, `M${P1.x} ${HY} L125 ${HY} L125 200`, 12, 'ret');
  valveSym(svg, 300, HY, false, 'dthrGate'); txt(300, HY - 24, 'DISCHARGE THROTTLE', 'label', svg, 'middle');
  el('rect', { x: 192, y: HY - 20, width: 56, height: 40, class: 'flowmeter', rx: 5 }, svg); rig.rotor = el('g', { transform: `translate(220 ${HY})` }, svg); for (let i = 0; i < 4; i++) el('line', { x1: 0, y1: 0, x2: 12, y2: 0, transform: `rotate(${i * 90})`, class: 'rotor' }, rig.rotor); txt(220, HY - 26, 'FLOW', 'label', svg, 'middle'); rig.qTxt = txt(220, HY + 36, '', 'gauge-val', svg, 'middle');
  // series link: P1 riser → P2 suction header (between isolation cross and P2)
  rig.serf = pipe(svg, `M${P1.x} 250 L520 250 L520 ${SY}`, 12);
  rig.isoSer = cross(svg, 460, 250); rig.isoP2 = cross(svg, 470, SY); rig.isoDis1 = cross(svg, P1.x, 200);
  // pumps
  for (const [P, key, name] of [[P1, 'p1', 'P1'], [P2, 'p2', 'P2']]) {
    const g = el('g', {}, svg); rig[key + 'g'] = g;
    el('rect', { x: P.x - 30, y: P.y + PR + 8, width: 60, height: 32, class: 'motor', rx: 5 }, g); txt(P.x, P.y + PR + 29, 'M', 'tag', g, 'middle');
    el('circle', { cx: P.x, cy: P.y, r: PR, class: 'pump-body' }, g); el('circle', { cx: P.x, cy: P.y, r: PR - 6, fill: 'var(--water)', opacity: .15 }, g);
    const imp = el('g', { transform: `translate(${P.x} ${P.y})` }, g); for (let i = 0; i < 6; i++) el('path', { d: 'M8 0 Q 18 -8 33 -15', transform: `rotate(${i * 60})`, class: 'impeller' }, imp); rig[key + 'imp'] = imp;
    el('circle', { cx: P.x, cy: P.y, r: 6, class: 'pump-body' }, g); txt(P.x - PR - 6, P.y - PR - 4, name, 'tag', g, 'end');
    el('rect', { x: P.x + PR + 3, y: P.y - 26, width: 16, height: 16, class: 'sensor', rx: 2 }, g); txt(P.x + PR + 11, P.y - 14, 'a', 'gauge-name', g);
    rig[key + 'bub'] = el('g', {}, g); for (let i = 0; i < 20; i++) el('circle', { r: 1.5, class: 'bubble', opacity: 0 }, rig[key + 'bub']);
    rig[key + 'rpm'] = txt(P.x, P.y + PR + 54, '', 'gauge-val', g, 'middle');
  }
  // pressure gauges: suction gauges below the header, discharge gauges beside the risers
  gauge(svg, P1.x - 74, 470, 'P1 SUCTION', -100, 500, 'kPa', 'g1s'); el('line', { x1: P1.x - 74, y1: 448, x2: P1.x - 74, y2: SY + 8, class: 'tap' }, svg);
  gauge(svg, P1.x + 62, 300, 'P1 DISCHARGE', -100, 700, 'kPa', 'g1d'); el('line', { x1: P1.x + 40, y1: 300, x2: P1.x + 8, y2: 300, class: 'tap' }, svg);
  gauge(svg, P2.x - 74, 470, 'P2 SUCTION', -100, 500, 'kPa', 'g2s'); el('line', { x1: P2.x - 74, y1: 448, x2: P2.x - 74, y2: SY + 8, class: 'tap' }, svg);
  gauge(svg, P2.x + 62, 300, 'P2 DISCHARGE', -100, 700, 'kPa', 'g2d'); el('line', { x1: P2.x + 40, y1: 300, x2: P2.x + 8, y2: 300, class: 'tap' }, svg);
  rig.arrTxt = txt(500, 70, '', 'tag', svg, 'middle'); txt(500, 88, 'ARRANGEMENT', 'label', svg, 'middle');
  rig.parts = []; const pg = el('g', {}, svg); for (let i = 0; i < 14; i++) rig.parts.push({ p: [[210, 470], [300, 470], [300, SY], [P1.x - PR, SY]], s: i / 14, e: el('circle', { r: 2.2, class: 'particle' }, pg) }); for (let i = 0; i < 22; i++) rig.parts.push({ p: [[P1.x, HY], [125, HY], [125, 200]], s: i / 22, e: el('circle', { r: 2.2, class: 'particle' }, pg) });
}
function pathLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); return L; }
function pointAt(pts, d) { for (let i = 1; i < pts.length; i++) { const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); if (d <= l) { const t = d / l; return [pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * t, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * t]; } d -= l; } return pts[pts.length - 1]; }
function updateRig(dt) {
  const s = state, par = s.arr === 'parallel', ser = s.arr === 'series';
  const lv = Math.max(196, Math.min(486, SY - s.level * 40)); rig.fluid.setAttribute('y', lv); rig.fluid.setAttribute('height', 488 - lv); rig.levelTxt.textContent = (s.level >= 0 ? '+' : '') + s.level.toFixed(1) + ' m'; rig.tfTxt.textContent = fmt(s.Tf, 1) + ' °C';
  rig.strFill.setAttribute('opacity', s.strainer / 100 * .8); rig.sthrGate.setAttribute('height', 18 * s.sthr / 100); rig.sthrGate.setAttribute('y', -9 * s.sthr / 100); rig.dthrGate.setAttribute('height', 18 * s.dthr / 100); rig.dthrGate.setAttribute('y', -9 * s.dthr / 100);
  rig.arrTxt.textContent = { single: 'SINGLE — P1 only, P2 isolated', parallel: 'PARALLEL — common suction and discharge headers', series: 'SERIES — P1 discharge feeds P2 suction' }[s.arr];
  rig.p2g.classList.toggle('pump-off', s.arr === 'single'); rig.isoP2.style.opacity = par ? 0 : 1; rig.isoSer.style.opacity = ser ? 0 : 1; rig.isoDis1.style.opacity = ser ? 1 : 0;
  rig.sucP2f.style.opacity = par && R.Q > 0.1 ? .75 : .15; rig.serf.style.opacity = ser && R.Q > 0.1 ? .75 : .15; rig.dis1f.style.opacity = !ser && R.Q > 0.1 ? .75 : .15; rig.dis2f.style.opacity = s.arr !== 'single' && R.Q > 0.1 ? .75 : .15; rig.suc.style.opacity = R.Q > 0.1 ? .75 : .25; rig.ret.style.opacity = R.Q > 0.1 ? .75 : .25;
  rig.a1 = (rig.a1 || 0) + s.n1a / 100 * PUMP.N0 / 60 * 360 * dt * .08; rig.a2 = (rig.a2 || 0) + (s.arr === 'single' ? 0 : s.n2a / 100 * PUMP.N0 / 60 * 360 * dt * .08); rig.p1imp.setAttribute('transform', `translate(${P1.x} ${P1.y}) rotate(${rig.a1 % 360})`); rig.p2imp.setAttribute('transform', `translate(${P2.x} ${P2.y}) rotate(${rig.a2 % 360})`);
  rig.p1rpm.textContent = Math.round(s.n1a / 100 * PUMP.N0) + ' rpm'; rig.p2rpm.textContent = s.arr === 'single' ? 'isolated' : Math.round(s.n2a / 100 * PUMP.N0) + ' rpm';
  rig.ro = (rig.ro || 0) + R.Q * 12 * dt; rig.rotor.setAttribute('transform', `translate(220 ${HY}) rotate(${rig.ro % 360})`); rig.qTxt.textContent = fmt(R.Q, 1) + ' m³/h';
  rig.g1s.set(R.P1s / 1000); rig.g1d.set(R.P1d / 1000); rig.g2s.set(s.arr === 'single' ? R.P1s / 1000 : R.P2s / 1000); rig.g2d.set(s.arr === 'single' ? R.P1s / 1000 : R.P2d / 1000);
  for (const p of rig.parts) { const len = pathLen(p.p); p.s = (p.s + R.L.v * 60 * dt / len) % 1; const [px, py] = pointAt(p.p, p.s * len); p.e.setAttribute('cx', px); p.e.setAttribute('cy', py); p.e.style.opacity = R.Q > 0.1 ? .85 : 0; }
  for (const [key, P, pr] of [['p1', P1, R.p1], ['p2', P2, R.p2]]) rig[key + 'bub'].querySelectorAll('circle').forEach((b, i) => { const sev = pr.on ? Math.max(0, Math.min(1, (2.5 - pr.margin) / 2.5)) : 0; if (i < Math.round(20 * sev)) { const u = ((performance.now() / 600) + i * .37) % 1; const ang = (i * 61 + rig.a1) * Math.PI / 180; const rr = 8 + u * 28; b.setAttribute('cx', P.x + Math.cos(ang + u) * rr); b.setAttribute('cy', P.y + Math.sin(ang + u) * rr); b.setAttribute('r', 1 + 2 * sev); b.setAttribute('opacity', .9 * (1 - u)); } else b.setAttribute('opacity', 0); });
}

/* ------------------------------------------------------------------ chart */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
const PALETTE = ['#1F8AD9', '#C7891E', '#2A8A5C', '#C6302C', '#7C4DFF', '#0F766E', '#ED8B00', '#6B7A8C'];
function keyOf(r, by) { return by === 'speed' ? `P1 ${r.n1} %${r.arr !== 'single' ? ' · P2 ' + r.n2 + ' %' : ''}` : by === 'arr' ? r.arr : `throttle ${r.dthr} %`; }
function drawChart() {
  const c = $('#chart'), dpr = window.devicePixelRatio || 1, W = c.clientWidth || 900, Hh = W * 500 / 900; if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(Hh * dpr); } const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, Hh);
  const m = { l: 50, r: 48, t: 14, b: 38 }, pw = W - m.l - m.r, ph = Hh - m.t - m.b; const Qmax = 80, Hmax = 70; const X = q => m.l + q / Qmax * pw, Y = h => m.t + ph - h / Hmax * ph, Y2 = e => m.t + ph - e * ph;
  ctx.font = '11px ' + col('--mono'); ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.lineWidth = 1; ctx.textAlign = 'center';
  for (let q = 0; q <= Qmax; q += 10) { ctx.beginPath(); ctx.moveTo(X(q), m.t); ctx.lineTo(X(q), m.t + ph); ctx.stroke(); ctx.fillText(q, X(q), Hh - m.b + 16); }
  ctx.textAlign = 'right'; for (let h = 0; h <= Hmax; h += 10) { ctx.beginPath(); ctx.moveTo(m.l, Y(h)); ctx.lineTo(m.l + pw, Y(h)); ctx.stroke(); ctx.fillText(h, m.l - 8, Y(h) + 4); }
  ctx.textAlign = 'left'; for (let e = 0; e <= 1; e += .25) ctx.fillText(Math.round(e * 100) + '%', m.l + pw + 8, Y2(e) + 4);
  ctx.textAlign = 'center'; ctx.font = '600 12px ' + col('--body'); ctx.fillText('Flow Q  (m³/h)', m.l + pw / 2, Hh - 4); ctx.save(); ctx.translate(14, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('Head H  (m)', 0, 0); ctx.restore(); ctx.save(); ctx.translate(W - 8, m.t + ph / 2); ctx.rotate(Math.PI / 2); ctx.fillStyle = col('--good'); ctx.fillText('Efficiency (points)', 0, 0); ctx.restore();
  const plot = (fn, style, width, dash, qmax) => { ctx.beginPath(); ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash || []); let first = true; for (let q = 0; q <= (qmax || Qmax); q += .5) { const y = fn(q); if (!Number.isFinite(y) || y < 0 || y > Hmax * 1.1) { first = true; continue; } first ? ctx.moveTo(X(q), Y(y)) : ctx.lineTo(X(q), Y(y)); first = false; } ctx.stroke(); ctx.setLineDash([]); };
  if ($('#showModel').checked) {
    const s = state; const n1 = s.n1a / 100, n2 = s.n2a / 100;
    for (const n of [1, .8, .6]) plot(q => q <= runout(n) ? pumpHead(q, n) : NaN, col('--water-soft'), 1.5, [5, 4]);
    if (n1 > .02) plot(q => q <= runout(n1) ? pumpHead(q, n1) : NaN, col('--water'), 2.5);
    if (s.arr === 'parallel' && (n1 > .02 || n2 > .02)) plot(q => combHead(q, s), col('--brass'), 2.5, [], 2 * runout(Math.max(n1, n2)));
    if (s.arr === 'series' && (n1 > .02 || n2 > .02)) plot(q => q <= runout(Math.max(n1, n2)) ? pumpHead(q, n1) + pumpHead(q, n2) : NaN, col('--brass'), 2.5);
    const fl = R.fl; plot(q => { const q3 = q / 3600, v = q3 / A, Re = fl.rho * v * D / fl.mu, hv = v * v / (2 * G); return LOOP.Hs + (LOOP.Kfix + friction(Re) * LOOP.L / D) * hv + (R.Kv > 0 ? Math.pow(q / R.Kv, 2) * 1e5 / (fl.rho * G) : NaN); }, col('--steel'), 2, [6, 4]);
    ctx.fillStyle = col('--muted'); ctx.textAlign = 'left'; ctx.font = '11px ' + col('--mono'); ctx.fillText('model: pump curves at 100 / 80 / 60 % (dashed), current arrangement (solid), system curve (grey dashed)', m.l + 6, m.t + 12);
  }
  // recorded points
  const by = $('#colourBy').value; const keys = []; const legend = $('#chartLegend'); legend.innerHTML = '';
  for (const r of state.log) { const k = keyOf(r, by); if (!keys.includes(k)) keys.push(k); const c2 = PALETTE[keys.indexOf(k) % PALETTE.length]; ctx.fillStyle = c2; ctx.beginPath(); ctx.arc(X(r.Q), Y(r.H), 4.5, 0, 7); ctx.fill(); if ($('#showEta').checked && r.Q > 0.2) { ctx.strokeStyle = c2; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(X(r.Q), Y2(r.etaP / 100), 4, 0, 7); ctx.stroke(); } }
  keys.forEach((k, i) => { const sp = document.createElement('span'); sp.innerHTML = `<i class="sw" style="background:${PALETTE[i % PALETTE.length]}"></i>${k}`; legend.appendChild(sp); });
  if (!state.log.length) { const sp = document.createElement('span'); sp.textContent = 'no points recorded yet — the curves are yours to build'; legend.appendChild(sp); }
  // current point
  if (R.anyOn) { const q = state.shown.Q, h = state.shown.H; ctx.strokeStyle = col('--steel'); ctx.setLineDash([2, 3]); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(q), Y(h)); ctx.lineTo(X(q), m.t + ph); ctx.moveTo(X(q), Y(h)); ctx.lineTo(m.l, Y(h)); ctx.stroke(); ctx.setLineDash([]); const pulse = 6 + 2 * Math.sin(performance.now() / 300); ctx.fillStyle = col('--ink'); ctx.beginPath(); ctx.arc(X(q), Y(h), pulse, 0, 7); ctx.fill(); ctx.fillStyle = col('--panel'); ctx.beginPath(); ctx.arc(X(q), Y(h), 2.5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.font = '600 12px ' + col('--mono'); ctx.textAlign = 'left'; ctx.fillText(`Q ${fmt(q, 1)} · H ${fmt(h, 1)} m · η pump ${fmt(R.etaPump * 100, 0)} % · P ${fmt(R.Pel / 1000, 2)} kW`, Math.min(X(q) + 12, m.l + pw - 260), Math.max(Y(h) - 12, m.t + 26)); }
}

/* ------------------------------------------------------------------ instruments, readouts */
const INSTR = [['Ps', 'Suction pressure', (p, k) => fmt((k === 'p1' ? R.P1s : R.P2s) / 1000, 1), 'kPa g'], ['Pd', 'Discharge pressure', (p, k) => fmt((k === 'p1' ? R.P1d : R.P2d) / 1000, 1), 'kPa g'], ['rpm', 'Speed', (p, k) => Math.round((k === 'p1' ? state.n1a : state.n2a) / 100 * PUMP.N0), 'rpm'], ['I', 'Motor current', p => fmt(p.I, 2), 'A', p => p.I > MOTOR.FLA * 1.1], ['P', 'Motor power', p => fmt(p.Pelec / 1000, 2), 'kW', p => p.Pelec > MOTOR.kW * 1100], ['vib', 'Bearing vibration', p => fmt(p.vib, 1), 'mm/s', p => p.vib > 7.1], ['Tb', 'Bearing temperature', (p, k) => fmt(k === 'p1' ? state.Tb1 : state.Tb2, 1), '°C', (p, k) => (k === 'p1' ? state.Tb1 : state.Tb2) > 85], ['Qp', 'Flow through pump', p => fmt(p.Q, 1), 'm³/h'], ['m', 'NPSH margin', p => fmt(p.margin, 2), 'm', p => p.on && p.margin < 0.6]];
function buildInstr() { const box = $('#instr'); box.innerHTML = ''; for (const [k, name] of [['p1', 'Pump P1'], ['p2', 'Pump P2']]) { const d = document.createElement('div'); d.className = 'ibox'; d.dataset.k = k; d.innerHTML = `<h4>${name}</h4><div class="row">${INSTR.map(i => `<div class="ro" data-i="${i[0]}"><span class="k">${i[1]}</span><span class="v">—</span><span class="u">${i[3]}</span></div>`).join('')}</div>`; box.appendChild(d); } }
function updateInstr() { document.querySelectorAll('.ibox').forEach(b => { const k = b.dataset.k, p = k === 'p1' ? R.p1 : R.p2; b.querySelectorAll('.ro').forEach(r => { const i = INSTR.find(x => x[0] === r.dataset.i); r.querySelector('.v').textContent = (k === 'p2' && state.arr === 'single' && !['Ps', 'Tb'].includes(i[0])) ? '—' : i[2](p, k); r.classList.toggle('alarm', !!(i[4] && i[4](p, k))); }); }); }
const READ = [['Q', 'Total flow', () => fmt(R.Q, 1), 'm³/h'], ['H', 'Total head (derived)', () => fmt(R.Htot, 1), 'm'], ['Phyd', 'Hydraulic power ρgQH', () => fmt(R.Phyd / 1000, 2), 'kW'], ['Pel', 'Total electrical power', () => fmt(R.Pel / 1000, 2), 'kW'], ['etaP', 'Pump efficiency (derived)', () => fmt(R.etaPump * 100, 1), '%'], ['etaO', 'Overall efficiency', () => fmt(R.etaOverall * 100, 1), '%'], ['Tf', 'Fluid temperature', () => fmt(state.Tf, 1), '°C', () => state.Tf > 60], ['rho', 'Fluid density', () => fmt(R.fl.rho, 0), 'kg/m³'], ['npsha', 'NPSHa at P1 eye', () => fmt(R.npsha1, 2), 'm'], ['v', 'Velocity in loop', () => fmt(R.L.v, 2), 'm/s'], ['Ks', 'Suction K total', () => fmt(R.Ks, 1), '—'], ['Kv', 'Discharge valve Kv', () => fmt(R.Kv, 1), 'm³/h·bar⁻½']];
function buildReadouts() { const box = $('#dataGrid'); box.innerHTML = ''; for (const r of READ) { const d = document.createElement('div'); d.className = 'ro'; d.dataset.k = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u">${r[3]}</span>`; box.appendChild(d); } }
function updateReadouts() { document.querySelectorAll('#dataGrid .ro').forEach(d => { const r = READ.find(x => x[0] === d.dataset.k); d.querySelector('.v').textContent = r[2](); d.classList.toggle('alarm', !!(r[4] && r[4]())); }); }
function updateStatus() {
  const on = R.anyOn; $('#pillRun').textContent = on ? `Running · ${state.arr} · ${fmt(R.Q, 1)} m³/h` : 'Pumps stopped'; $('#pillRun').classList.toggle('on', on);
  $('#pillPoint').textContent = on ? `H ${fmt(R.Htot, 1)} m · η pump ${fmt(R.etaPump * 100, 0)} % · ${fmt(R.Pel / 1000, 2)} kW` : '—';
  const cav = R.p1.cav || R.p2.cav; $('#pillCav').classList.toggle('hidden', !cav); $('#pillCav').textContent = `Cavitation · ${R.p1.cav ? 'P1 ' : ''}${R.p2.cav ? 'P2' : ''} margin ${fmt(Math.min(R.p1.margin, R.p2.on ? R.p2.margin : 99), 1)} m`;
  const over = R.p1.I > MOTOR.FLA * 1.1 || R.p2.I > MOTOR.FLA * 1.1; $('#pillOver').classList.toggle('hidden', !over);
  const brg = state.Tb1 > 85 || state.Tb2 > 85; $('#pillBrg').classList.toggle('hidden', !brg); $('#pillBrg').textContent = `Bearing hot · ${fmt(Math.max(state.Tb1, state.Tb2), 0)} °C`;
  const vib = R.p1.vib > 7.1 || R.p2.vib > 7.1; $('#pillVib').classList.toggle('hidden', !vib);
  $('#pillFluid').classList.toggle('hidden', state.Tf < 45); $('#pillFluid').textContent = `Fluid heating · ${fmt(state.Tf, 0)} °C`;
  Object.assign(state.flags, { cav: state.flags.cav || cav, over: state.flags.over || over, brg: state.flags.brg || brg });
}

/* ------------------------------------------------------------------ log & tasks */
function record(tag) { const r = { i: state.log.length + 1, arr: state.arr, n1: state.n1, n2: state.arr === 'single' ? 0 : state.n2, dthr: state.dthr, sthr: state.sthr, Q: +R.Q.toFixed(2), Ps: +(R.P1s / 1000).toFixed(1), Pd: +((state.arr === 'series' ? R.P2d : R.P1d) / 1000).toFixed(1), H: +R.Htot.toFixed(2), rpm: Math.round(state.n1a / 100 * PUMP.N0), I: +(R.p1.I + R.p2.I).toFixed(2), P: +(R.Pel / 1000).toFixed(3), Phyd: +(R.Phyd / 1000).toFixed(3), etaP: +(R.etaPump * 100).toFixed(1), etaO: +(R.etaOverall * 100).toFixed(1), vib: +Math.max(R.p1.vib, R.p2.vib).toFixed(1), Tb: +Math.max(state.Tb1, state.Tb2).toFixed(1), Tf: +state.Tf.toFixed(1), tag }; state.log.push(r); renderLog(); checkTasks(); }
function renderLog() { const tb = $('#logTable tbody'); tb.innerHTML = ''; for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.arr, r.n1, r.n2, r.dthr, r.sthr, r.Q, r.Ps, r.Pd, r.H, r.rpm, r.I, r.P, r.Phyd, r.etaP, r.etaO, r.vib, r.Tb, r.Tf].map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); } $('#logCount').textContent = state.log.length; const csv = ['n,arrangement,n1_pct,n2_pct,discharge_throttle_pct,suction_throttle_pct,Q_m3h,Ps_kPa,Pd_kPa,H_m,rpm,I_A,P_elec_kW,P_hyd_kW,eta_pump_pct,eta_overall_pct,vib_mms,bearing_C,fluid_C'].concat(state.log.map(r => [r.i, r.arr, r.n1, r.n2, r.dthr, r.sthr, r.Q, r.Ps, r.Pd, r.H, r.rpm, r.I, r.P, r.Phyd, r.etaP, r.etaO, r.vib, r.Tb, r.Tf].join(','))).join('\n'); state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv); }
const spread = pts => pts.length >= 6 && (Math.max(...pts.map(p => p.Q)) - Math.min(...pts.map(p => p.Q))) > 15;
const TASKS = [
  { id: 'hq', t: 'Construct the H–Q curve', d: 'Single pump at 100 %. Record at least six points across the discharge-throttle range (shut to open). Plot head against flow.', ok: () => spread(state.log.filter(r => r.arr === 'single' && r.n1 >= 99 && r.sthr >= 95)) },
  { id: 'eta', t: 'Construct the efficiency curve and find the BEP', d: 'From the same points, compute ρgQH / P_shaft (or read the derived column) and find the flow at peak efficiency.', ok: () => { const p = state.log.filter(r => r.arr === 'single' && r.n1 >= 99 && r.sthr >= 95); return p.length >= 6 && p.some(r => Math.abs(r.Q - PUMP.Qbep) < 4 && r.etaP >= 68); }, st: () => { const p = state.log.filter(r => r.arr === 'single' && r.n1 >= 99); if (!p.length) return ''; const b = p.reduce((a, r) => r.etaP > a.etaP ? r : a); return `best recorded η ${b.etaP} % at ${b.Q} m³/h`; } },
  { id: 'sys', t: 'Construct the system curve', d: 'Fix the discharge throttle (say 60 %) and record points at four or more speeds. These points lie on the system curve, not on a pump curve.', ok: () => { const p = state.log.filter(r => r.arr === 'single' && r.dthr === state.log.find(x => x.arr === 'single')?.dthr); const speeds = new Set(p.map(r => r.n1)); return speeds.size >= 4; }, st: () => { const byT = {}; state.log.filter(r => r.arr === 'single').forEach(r => { (byT[r.dthr] = byT[r.dthr] || new Set()).add(r.n1); }); const best = Object.entries(byT).sort((a, b) => b[1].size - a[1].size)[0]; return best ? `throttle ${best[0]} %: ${best[1].size} speed(s) recorded` : ''; } },
  { id: 'aff', t: 'Affinity-law curves', d: 'Record H–Q points at 100 %, 80 % and 60 % speed. Check that shut-off head scales with speed squared and run-out flow with speed.', ok: () => ['100', '80', '60'].every(n => state.log.filter(r => r.arr === 'single' && String(r.n1) === n).length >= 3), st: () => { const h = n => { const p = state.log.filter(r => r.arr === 'single' && r.n1 === n && r.dthr <= 5); return p.length ? p[0].H : null; }; const a = h(100), b = h(60); return a && b ? `shut-off 100 %: ${a} m, 60 %: ${b} m → ratio ${(b / a).toFixed(2)} (0.36 expected)` : 'record shut-off (throttle 0 %) at each speed for the check'; } },
  { id: 'par', t: 'Parallel curve', d: 'Parallel, both pumps at 100 %. Record at least five points across the throttle range and compare with the single-pump curve: flow adds at equal head.', ok: () => state.log.filter(r => r.arr === 'parallel' && r.n1 >= 99 && r.n2 >= 99).length >= 5 },
  { id: 'ser', t: 'Series curve', d: 'Series, both pumps at 100 %. Record at least five points: head adds at equal flow.', ok: () => state.log.filter(r => r.arr === 'series' && r.n1 >= 99 && r.n2 >= 99).length >= 5 },
  { id: 'op', t: 'Predict the operating point', d: 'From your H–Q curve and your system curve, predict the flow at 100 % speed with the throttle at 60 %, then set the rig to that state and record. Within 5 %?', ok: () => state.log.some(r => r.arr === 'single' && r.n1 >= 99 && r.dthr === 60 && r.tag === 'manual') && spread(state.log.filter(r => r.arr === 'single' && r.n1 >= 99)) && state.log.filter(r => r.arr === 'single').map(r => r.n1).filter((v, i, a) => a.indexOf(v) === i).length >= 3 },
  { id: 'mismatch', t: 'Mismatched pumps in parallel', d: 'Parallel with P2 at 70 % and P1 at 100 %, throttle 30 %. Watch P2\'s flow: a slower pump against the header pressure of a faster one can be pushed to shut-off.', ok: () => state.arr === 'parallel' && state.n1 >= 99 && state.n2 <= 72 && state.n2 >= 60 && state.dthr <= 35 && R.p2.on, st: () => state.arr === 'parallel' && R.p2.on ? `P2 is passing ${fmt(R.p2.Q, 1)} m³/h against ${fmt(R.Htot, 1)} m` : '' },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  const ramp = 40 * dt; for (const k of ['n1', 'n2']) { const a = k + 'a'; state[a] += Math.max(-ramp, Math.min(ramp, state[k] - state[a])); if (Math.abs(state[k] - state[a]) < 0.05) state[a] = state[k]; }
  if (state.sweep) { const sw = state.sweep; sw.t += dt; const steps = [0, 10, 20, 30, 45, 60, 75, 90, 100]; const k = Math.min(steps.length - 1, Math.floor(sw.t / 1.5)); state.dthr = steps[k]; $('#dthr').value = state.dthr; if (k > sw.rec && sw.t - k * 1.5 > 1.1 && state.n1a === state.n1 && state.n2a === state.n2) { record('sweep'); sw.rec = k; if (k === steps.length - 1) { state.sweep = null; $('#btnSweep').textContent = 'Sweep discharge throttle'; } } }
  R = solve(state);
  const tau = 0.5; state.shown.Q += (R.Q - state.shown.Q) * Math.min(1, dt / tau); state.shown.H += (R.Htot - state.shown.H) * Math.min(1, dt / tau);
  // thermal dynamics (accelerated clock)
  const dts = dt * state.accel; const fl = R.fl;
  state.Tf += ((R.Psh - LOOP.UA * (state.Tf - LOOP.Tamb)) / (LOOP.tankM * fl.cp)) * dts;
  for (const [k, p] of [['Tb1', R.p1], ['Tb2', R.p2]]) { const n = p.on ? (k === 'Tb1' ? state.n1a : state.n2a) / 100 : 0; const target = LOOP.Tamb + (p.on ? 18 * n * n + 22 * p.load + 2.2 * p.vib + 0.3 * (state.Tf - LOOP.Tamb) : 0); state[k] += (target - state[k]) / 90 * dts; }
  updateRig(dt); drawChart();
  if (frame++ % 6 === 0) { updateInstr(); updateReadouts(); updateStatus(); checkTasks(); $('#n1Out').textContent = state.n1 + ' %'; $('#n2Out').textContent = state.n2 + ' %'; $('#rpm1Out').textContent = Math.round(state.n1a / 100 * PUMP.N0) + ' rpm'; $('#rpm2Out').textContent = Math.round(state.n2a / 100 * PUMP.N0) + ' rpm'; $('#dthrOut').textContent = state.dthr + ' %'; $('#sthrOut').textContent = state.sthr + ' %'; $('#levelOut').textContent = state.level.toFixed(1) + ' m'; $('#strainerOut').textContent = state.strainer + ' %'; }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function setArr(a) { state.arr = a; document.querySelectorAll('#arrSeg button').forEach(b => b.classList.toggle('on', b.dataset.arr === a)); }
function bind() {
  const on = (id, fn) => $('#' + id).addEventListener('input', e => fn(e.target.value));
  on('n1', v => state.n1 = +v); on('n2', v => state.n2 = +v); on('dthr', v => { state.dthr = +v; state.sweep = null; }); on('sthr', v => state.sthr = +v); on('level', v => state.level = +v); on('strainer', v => state.strainer = +v);
  $('#fluid').addEventListener('change', e => state.fluid = e.target.value);
  for (const k of ['n1', 'n2', 'dthr']) document.querySelectorAll(`[data-${k}]`).forEach(b => b.onclick = () => { state[k] = +b.dataset[k]; $('#' + k).value = state[k]; if (k === 'dthr') state.sweep = null; });
  $('#btnMatch').onclick = () => { state.n2 = state.n1; $('#n2').value = state.n2; };
  document.querySelectorAll('#arrSeg button').forEach(b => b.onclick = () => setArr(b.dataset.arr));
  $('#btnRecord').onclick = () => record('manual');
  $('#btnSweep').onclick = () => { if (state.sweep) { state.sweep = null; $('#btnSweep').textContent = 'Sweep discharge throttle'; return; } if (state.n1 < 5) { state.n1 = 100; $('#n1').value = 100; } state.sweep = { t: 0, rec: -1 }; $('#btnSweep').textContent = 'Stop sweep'; document.querySelector('[data-tab="log"]').click(); };
  $('#btnReset').onclick = () => { Object.assign(state, { n1: 0, n2: 0, dthr: 60, sthr: 100, level: 1.0, strainer: 0, fluid: 'water', Tf: 22, Tb1: 22, Tb2: 22, sweep: null, flags: {} }); ['n1', 'n2', 'dthr', 'sthr', 'level', 'strainer', 'fluid'].forEach(id => $('#' + id).value = state[id]); setArr('single'); document.querySelectorAll('.tasks li').forEach(l => l.classList.remove('done')); };
  $('#btnAccel').onclick = () => { state.accel = state.accel === 1 ? 20 : 1; $('#btnAccel').textContent = 'Thermal ×' + state.accel; $('#btnAccel').classList.toggle('on', state.accel > 1); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  ['showModel', 'showEta', 'colourBy'].forEach(id => $('#' + id).addEventListener('change', drawChart));
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('pt_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT'].includes(e.target.tagName)) return; const st = e.shiftKey ? 10 : 2;
    if (e.key === 'ArrowUp') { state.n1 = Math.min(100, state.n1 + st); $('#n1').value = state.n1; } else if (e.key === 'ArrowDown') { state.n1 = Math.max(0, state.n1 - st); $('#n1').value = state.n1; }
    else if (e.key === 'ArrowRight') { state.dthr = Math.min(100, state.dthr + st); $('#dthr').value = state.dthr; } else if (e.key === 'ArrowLeft') { state.dthr = Math.max(0, state.dthr - st); $('#dthr').value = state.dthr; }
    else if (e.key === '1') setArr('single'); else if (e.key === '2') setArr('parallel'); else if (e.key === '3') setArr('series'); else if (e.key === 'r' || e.key === 'R') record('manual'); else if (e.key === 's' || e.key === 'S') $('#btnSweep').click(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', drawChart);
}

/* ------------------------------------------------------------------ dashboard API */
window.PumpTrainer = {
  getState: () => ({ inputs: { arr: state.arr, n1: state.n1, n2: state.n2, dthr: state.dthr, sthr: state.sthr, level: state.level, strainer: state.strainer, fluid: state.fluid }, results: { Q: R.Q, H: R.Htot, eta: R.etaPump, Pelec: R.Pel, P1: R.P1s, P2: R.P1d, margin: R.p1.margin, rpm: state.n1a / 100 * PUMP.N0, vib: Math.max(R.p1.vib, R.p2.vib), Tb: Math.max(state.Tb1, state.Tb2), Tf: state.Tf }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.arr) setArr(inp.arr); for (const k of ['n1', 'n2', 'dthr', 'sthr', 'level', 'strainer']) if (inp[k] !== undefined) { state[k] = +inp[k]; $('#' + k).value = state[k]; } if (inp.fluid && FLUIDS[inp.fluid]) { state.fluid = inp.fluid; $('#fluid').value = inp.fluid; } },
  meta: { id: 'pump-trainer', title: 'Instrumented Pump Trainer', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PumpTrainer.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'pump-trainer', state: window.PumpTrainer.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('pt_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildInstr(); buildReadouts(); buildTasks(); bind(); renderLog();
  state.n1 = 100; state.n1a = 100; $('#n1').value = 100; R = solve(state); state.shown = { Q: R.Q, H: R.Htot };
  requestAnimationFrame(tick);
}
build();
