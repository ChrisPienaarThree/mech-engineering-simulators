/* Hydraulic Power Demonstrator — fixed-displacement power pack, relief valve, 4/3 directional valve, meter-in and
   meter-out needle valves, and a vertical cylinder from a rack of bores and rods with a load cell on the rod.
   Quasi-static hydraulics solved each frame (flow, cap and rod pressures, pump pressure), position integrated in
   real time, oil temperature on a ×10 thermal clock. Standalone, offline. Rig data illustrative. */
'use strict';

/* ------------------------------------------------------------------ rig constants */
const PACK = { Vg: 8, etaV: 0.93, etaM: 0.90, kW: 4, tankL: 30, pt: 1.5, Cv: 7.8, closedCentreP: null, tandemP: 3 };   // cm³/rev, -, -, kW, L, bar return, needle valve L/min per √bar
const CYL = { stroke: 0.3, rated: 250, sealLimit: 320, burst: 400, frictionPct: 0.02, frictionN: 150, platformKg: 20 };   // seal friction: 2 % of the pressure force + 150 N
const kv = a => PACK.Cv * a * a;   // needle valves are progressive: flow coefficient ∝ opening²
const RODS = { 25: [12, 16], 32: [14, 18, 22], 40: [18, 22, 28], 50: [22, 28, 36], 63: [28, 36, 45], 80: [36, 45, 56] };
const BORES = [25, 32, 40, 50, 63, 80], G = 9.81, AMB = 25, OIL_C = PACK.tankL * 0.87 * 1900, COOL = 40, THERMAL = 10;
const $ = s => document.querySelector(s);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const area = D => Math.PI * D * D / 4e6, annulus = (D, d) => Math.PI * (D * D - d * d) / 4e6;   // m²
const LOADS = { weight: ['Mass', 0, 3000, 50, 1000, v => v.toLocaleString().replace(/,/g, ' ') + ' kg'], spring: ['Spring rate', 0, 200, 5, 60, v => v + ' kN/m'], brake: ['Brake force', 0, 100, 1, 30, v => v + ' kN'], none: ['—', 0, 1, 1, 0, () => '—'] };

/* ------------------------------------------------------------------ state */
const state = {
  bore: 50, rod: 28, loadType: 'weight', loadVal: 1000, relief: 150, rpm: 1450, mIn: 100, mOut: 100, closedCentre: false, dcv: 'neutral',
  run: false, tripped: null, blown: false, x: 0, v: 0, t: 0, Toil: AMB, overT: 0, gP1: 0, gP2: 0, gP3: 0, trace: [], traceAcc: 0, log: [], flags: {}, reliefHot: 0, stallT: 0, dash: 0,
};
const R = {};

/* ------------------------------------------------------------------ hydraulics */
function loadForce(dir) {   // N opposing the motion (negative = assisting)
  const w = CYL.platformKg * G;
  if (state.loadType === 'weight') return (state.loadVal * G + w) * (dir === 'extend' ? 1 : -1);
  if (state.loadType === 'spring') return (state.loadVal * 1000 * state.x + w) * (dir === 'extend' ? 1 : -1);
  if (state.loadType === 'brake') return state.loadVal * 1000 + w * (dir === 'extend' ? 1 : -1);
  return w * (dir === 'extend' ? 1 : -1);
}
function solve() {
  const A1 = area(state.bore), A2 = annulus(state.bore, state.rod), Qp = state.run && !state.tripped ? PACK.Vg * state.rpm * PACK.etaV / 1000 : 0;   // L/min
  const out = { A1, A2, Qp, Q: 0, p1: 0, p2: 0, pp: 0, v: 0, Frod: 0, Fpa: 0, Qr: 0, Pm: 0, Pu: 0, Pw: 0, stall: false, atEnd: false, cav: false, intens: false, dir: state.dcv };
  const dir = state.dcv, pt = PACK.pt;
  if (!Qp) { out.pp = 0; out.p1 = out.p2 = 0; out.Frod = state.loadType === 'weight' ? 0 : 0; return out; }
  if (dir === 'neutral' || state.blown) { out.pp = state.closedCentre ? state.relief : PACK.tandemP; out.Qr = state.closedCentre ? Qp : 0; out.p1 = out.p2 = state.blown ? 0 : pt; out.Pm = out.pp * Qp / 600 / PACK.etaM; out.Pw = state.closedCentre ? state.relief * Qp / 600 : PACK.tandemP * Qp / 600; return out; }
  const Ad = dir === 'extend' ? A1 : A2, Ar = dir === 'extend' ? A2 : A1, aIn = state.mIn / 100, aOut = state.mOut / 100;
  const atEnd = dir === 'extend' ? state.x >= CYL.stroke - 1e-6 : state.x <= 1e-6; out.atEnd = atEnd;
  const Fopp = loadForce(dir);
  const eff = 1 - CYL.frictionPct, fric = Q => CYL.frictionN;
  const pr = Q => aOut <= 0 ? Infinity : pt + Math.pow(Q * Ar / Ad / kv(aOut), 2);           // return-side pressure for cylinder flow Q (L/min into the driving side)
  const pd = Q => (Fopp + CYL.frictionN + pr(Q) * 1e5 * Ar) / Ad / eff / 1e5;                  // driving-side pressure needed, bar (2 % seal friction on the pressure force)
  const ppNeed = Q => aIn <= 0 ? Infinity : pd(Q) + Math.pow(Q / kv(aIn), 2);
  let Q = 0, p1, p2, pp;
  if (atEnd || aIn <= 0 || aOut <= 0) { Q = 0; }
  else if (Fopp + fric(0) + pt * 1e5 * Ar < 0) {   // overrunning: the load drives, the meter-out valve holds it
    const prun = (-Fopp - fric(0)) * eff / Ar / 1e5;    // return pressure if the driving side is at zero
    const Qrun = (Ad / Ar) * kv(aOut) * Math.sqrt(Math.max(0, prun - pt));
    if (Qrun <= Qp) { Q = Qrun; pp = Math.min(state.relief, Math.pow(Q / kv(aIn), 2) + 0.5); }   // pump keeps up; cap side barely pressurised
    else { Q = Qrun; out.cav = true; pp = PACK.tandemP; }
    p1 = 0.3; p2 = pt + Math.pow(Q * Ar / Ad / kv(aOut), 2);
    out.Q = Math.min(Q, Qp); out.v = Q / 60000 / Ad * (dir === 'extend' ? 1 : -1); out.pp = pp; out.Qr = Math.max(0, Qp - out.Q);
    out.p1 = dir === 'extend' ? p1 : p2; out.p2 = dir === 'extend' ? p2 : p1; out.p2 = dir === 'extend' ? p2 : p1;
    assign(out, dir, p1, p2, Fopp, fric(Q), Ad, Ar, Qp); return out;
  }
  if (Q === 0 && !(atEnd || aIn <= 0 || aOut <= 0)) {
    if (ppNeed(Qp) <= state.relief) Q = Qp;
    else if (pd(0) >= state.relief) Q = 0;
    else { let lo = 0, hi = Qp; for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (ppNeed(m) <= state.relief) lo = m; else hi = m; } Q = lo; }
  }
  if (Q > 1e-6) { p2 = pr(Q); p1 = pd(Q); pp = Q >= Qp - 1e-6 ? ppNeed(Q) : state.relief; }
  else { pp = state.relief; p1 = atEnd ? state.relief : Math.min(state.relief, pd(0)); if (aOut <= 0 && !atEnd) { p1 = state.relief; p2 = Math.max(pt, (state.relief * 1e5 * Ad - Fopp - fric(0)) / Ar / 1e5); } else p2 = pt; out.stall = !atEnd; }
  if (aIn <= 0) { pp = state.relief; p1 = 0; p2 = pt; }
  out.Q = Q; out.v = Q / 60000 / Ad * (dir === 'extend' ? 1 : -1); out.pp = pp; out.Qr = Qp - Q;
  assign(out, dir, p1, p2, Fopp, fric(Q), Ad, Ar, Qp); return out;
}
function assign(out, dir, pdv, prv, Fopp, fr, Ad, Ar, Qp) {
  out.p1 = dir === 'extend' ? pdv : prv; out.p2 = dir === 'extend' ? prv : pdv;   // p1 = cap side, p2 = rod side (naming on the rig: P2 cap, P3 rod)
  out.Fpa = pdv * 1e5 * Ad / 1000;                                                 // kN on the driving face
  const net = pdv * 1e5 * Ad * (1 - CYL.frictionPct) - prv * 1e5 * Ar - fr;         // N available at the rod
  out.Frod = (out.Q > 1e-6 || out.cav ? Fopp : Math.min(Fopp, Math.max(0, net))) / 1000;
  if (state.loadType === 'weight' && Fopp < 0) out.Frod = -Fopp / 1000;              // the weight always hangs on the rod
  out.Pm = out.pp * Qp / 600 / PACK.etaM; out.Pu = Math.max(0, Fopp * Math.abs(out.v)) / 1000;
  out.Pw = Math.max(0, out.pp * Qp / 600 - out.Pu);
  out.intens = out.p2 > state.relief + 5 && dir === 'extend' || out.p1 > state.relief + 5 && dir === 'retract';
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
const rig = {}, GEO = { cx: 660, top: 250, bot: 500, travel: 190, rodLen: 232 };
function gauge(svg, cx, cy, name, max) { const g = el('g', { transform: `translate(${cx},${cy})` }, svg); el('circle', { r: 24, class: 'gauge-face' }, g); for (let i = 0; i <= 10; i++) { const a = (-135 + i * 27) * Math.PI / 180; el('line', { x1: 19 * Math.sin(a), y1: -19 * Math.cos(a), x2: 23 * Math.sin(a), y2: -23 * Math.cos(a), class: 'gauge-tick' }, g); } const n = el('line', { x1: 0, y1: 0, x2: 0, y2: -20, class: 'gauge-needle' }, g); txt(0, 36, name, 'gauge-name', g); const v = txt(0, 12, '—', 'gauge-val', g); return { set: val => { n.setAttribute('transform', `rotate(${-135 + clamp(val / max, 0, 1.05) * 270})`); v.textContent = fmt(val, 0) + ' bar'; } }; }
function needle(svg, x, y, name, key) { const g = el('g', { transform: `translate(${x},${y})` }, svg); el('polygon', { points: '-14,-9 14,-9 0,0', class: 'needle-body' }, g); el('polygon', { points: '-14,9 14,9 0,0', class: 'needle-body' }, g); const stem = el('line', { x1: 0, y1: 0, x2: 0, y2: -26, class: 'needle-stem' }, g); const cap = el('rect', { x: -8, y: -32, width: 16, height: 6, class: 'valve-body' }, g); rig[key + 'Txt'] = txt(22, -12, name + ' 100 %', 'label', g); rig[key] = { set: pct => { stem.setAttribute('y2', -14 - 12 * pct / 100); cap.setAttribute('y', -20 - 12 * pct / 100); rig[key + 'Txt'].textContent = `${name} ${pct} %`; } }; }
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 520', role: 'img', 'aria-label': 'Hydraulic power pack driving a vertical cylinder with a load' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg);
  // tank and power pack
  el('rect', { x: 40, y: 400, width: 260, height: 100, rx: 4, class: 'tank' }, svg); rig.oil = el('rect', { x: 43, y: 430, width: 254, height: 67, class: 'oil' }, svg); rig.heat = el('rect', { x: 43, y: 430, width: 254, height: 67, class: 'heat' }, svg);
  txt(50, 494, 'TANK 30 L', 'label', svg); rig.tankT = txt(290, 494, '25 °C', 'tag', svg, 'end');
  el('circle', { cx: 90, cy: 360, r: 26, class: 'motor-body' }, svg); txt(90, 365, 'M', 'tag', svg, 'middle'); rig.rpmTxt = txt(90, 326, '0 rpm', 'label', svg, 'middle');
  el('line', { x1: 116, y1: 360, x2: 140, y2: 360, class: 'stem' }, svg); el('circle', { cx: 165, cy: 360, r: 24, class: 'pump-body' }, svg); el('polygon', { points: '165,340 179,360 151,360', class: 'arrowhead' }, svg); txt(165, 398, 'PUMP 8 cm³/rev', 'label', svg, 'middle');
  el('path', { d: 'M165,384 V400', class: 'hose' }, svg);
  // relief valve
  el('rect', { x: 210, y: 336, width: 34, height: 48, rx: 3, class: 'valve-box' }, svg); el('path', { d: 'M227,380 V344 M219,344 l8,-8 l8,8', class: 'arrow' }, svg); el('path', { d: 'M244,360 l6,-4 l-6,-4 l6,-4 l-6,-4', class: 'spring' }, svg); rig.reliefArrow = el('path', { d: 'M227,384 V400', class: 'relief-arrow relief' }, svg); txt(227, 330, 'RELIEF', 'label', svg, 'middle');
  el('path', { d: 'M189,360 H210', class: 'hose' }, svg);
  // pressure line to the directional valve
  el('path', { d: 'M165,336 V300 H340 V262', class: 'hose' }, svg); rig.hP = el('path', { d: 'M165,336 V300 H340 V262', class: 'hose-oil' }, svg);
  rig.g1 = gauge(svg, 270, 260, 'P1 PUMP', 250); el('path', { d: 'M270,284 V300', class: 'tap' }, svg);
  // 4/3 directional valve, return to a tank symbol beneath it
  rig.dcvBoxes = {}; for (const [bx, id] of [[300, 'retract'], [340, 'neutral'], [380, 'extend']]) rig.dcvBoxes[id] = el('rect', { x: bx, y: 220, width: 40, height: 42, class: 'valve-box' }, svg);
  el('path', { d: 'M310,258 L330,224 M330,258 L310,224', class: 'arrow' }, svg); el('path', { d: 'M350,258 V224 M370,258 V224', class: 'arrow' }, svg); el('path', { d: 'M390,224 V258 M410,224 V258', class: 'arrow' }, svg);
  el('polygon', { points: '386,232 390,222 394,232', class: 'arrowhead' }, svg); el('polygon', { points: '406,250 410,260 414,250', class: 'arrowhead' }, svg); el('polygon', { points: '326,232 330,222 334,232', class: 'arrowhead' }, svg); el('polygon', { points: '306,250 310,260 314,250', class: 'arrowhead' }, svg);
  txt(300, 210, '4/3 VALVE', 'label', svg); el('path', { d: 'M380,262 V306', class: 'hose' }, svg); el('path', { d: 'M366,306 H394 M370,306 V318 M390,306 V318 M370,318 H390', class: 'arrow' }, svg); txt(400, 318, 'T', 'label', svg);
  // A line (cap side) with meter-in; B line (rod side) with meter-out
  el('path', { d: 'M350,220 V130 H580 V494 H600', class: 'hose' }, svg); rig.hA = el('path', { d: 'M350,220 V130 H580 V494 H600', class: 'hose-oil' }, svg);
  el('path', { d: 'M370,220 V190 H560 V268 H600', class: 'hose' }, svg); rig.hB = el('path', { d: 'M370,220 V190 H560 V268 H600', class: 'hose-oil' }, svg);
  needle(svg, 470, 130, 'METER-IN', 'nIn'); needle(svg, 445, 190, 'METER-OUT', 'nOut');
  rig.g2 = gauge(svg, 520, 470, 'P2 CAP', 250); el('path', { d: 'M544,470 H580', class: 'tap' }, svg);
  rig.g3 = gauge(svg, 500, 240, 'P3 ROD', 400); el('path', { d: 'M500,216 V190', class: 'tap' }, svg);
  rig.cyl = el('g', {}, svg);
  rig.face = el('g', { transform: 'translate(820,412)' }, svg); txt(820, 505, 'PISTON FACE · TO SCALE', 'label', svg, 'middle');
}
function drawCylinder() {
  const g = rig.cyl; g.innerHTML = ''; const { cx, top, bot, travel, rodLen } = GEO, W = state.bore * 1.5, rw = state.rod * 1.5;
  const y = bot - 20 - state.x / CYL.stroke * travel, wallL = cx - W / 2 - 6, plat = y - 8 - rodLen;   // y = piston centre, plat = platform top
  el('line', { x1: wallL, y1: 494, x2: 600, y2: 494, class: 'hose' }, g); el('line', { x1: wallL, y1: 268, x2: 600, y2: 268, class: 'hose' }, g);
  el('rect', { x: wallL, y: top, width: W + 12, height: bot - top, rx: 4, class: 'cyl' }, g);
  const cap = el('rect', { x: cx - W / 2, y: y + 8, width: W, height: bot - 12 - y - 8, class: 'chamber' }, g); cap.style.opacity = 0.25 + 0.6 * clamp(R.p1 / 250, 0, 1);
  const rodC = el('rect', { x: cx - W / 2, y: top + 12, width: W, height: Math.max(0, y - 8 - top - 12), class: 'chamber rod' }, g); rodC.style.opacity = 0.2 + 0.6 * clamp(R.p2 / 250, 0, 1);
  el('rect', { x: wallL, y: top, width: W + 12, height: 12, class: 'cyl-cap' }, g); el('rect', { x: wallL, y: bot - 12, width: W + 12, height: 12, class: 'cyl-cap' }, g);
  el('rect', { x: cx - W / 2, y: y - 8, width: W, height: 16, class: 'piston' }, g);
  el('rect', { x: cx - rw / 2, y: plat + 10, width: rw, height: y - 8 - plat - 10, class: 'rod' }, g);
  el('rect', { x: cx - 70, y: plat, width: 140, height: 10, class: 'platform' }, g);
  el('rect', { x: cx - 16, y: plat + 14, width: 32, height: 18, rx: 3, class: 'lc' }, g); txt(cx, plat + 27, 'LC', 'gauge-name', g);
  rig.lcTxt = txt(cx + 80, plat + 28, `LOAD CELL  ${fmt(R.Frod, 2)} kN`, 'lc-txt', g);
  if (state.loadType === 'weight' && state.loadVal > 0) { const n = clamp(Math.round(state.loadVal / 600), 1, 5); for (let i = 0; i < n; i++) el('rect', { x: cx - 45 + (i % 2) * 8, y: plat - (i + 1) * 9, width: 90 - (i % 2) * 16, height: 8, rx: 2, class: 'weight' }, g); txt(cx + 80, plat - 2, `${state.loadVal.toLocaleString().replace(/,/g, ' ')} kg`, 'tag', g); }
  if (state.loadType === 'spring') { el('rect', { x: cx - 80, y: 14, width: 160, height: 8, class: 'crosshead' }, g); const nL = 8, h = (plat - 22) / nL; let d = `M${cx},${plat}`; for (let i = 0; i < nL; i++) d += ` l${i % 2 ? 18 : -18},${-h}`; d += ` L${cx},22`; el('path', { d, class: 'spring', style: `stroke-width:${1 + state.loadVal / 60}` }, g); txt(cx + 80, 30, `${state.loadVal} kN/m`, 'tag', g); }
  if (state.loadType === 'brake') { el('rect', { x: wallL - 6, y: top - 22, width: W + 24, height: 10, class: 'crosshead' }, g); el('rect', { x: cx - rw / 2 - 14, y: top - 20, width: 12, height: 8, class: 'brake' }, g); el('rect', { x: cx + rw / 2 + 2, y: top - 20, width: 12, height: 8, class: 'brake' }, g); txt(cx + 80, top - 14, `brake ${state.loadVal} kN`, 'tag', g); }
  txt(cx, bot + 16, `Ø${state.bore} / ${state.rod} · stroke 300 mm`, 'tag', g, 'middle');
  const f = rig.face; f.innerHTML = ''; const sc = 0.85; el('circle', { r: state.bore * sc / 2, class: 'face' }, f); el('circle', { r: state.rod * sc / 2, class: 'face-rod' }, f);
  const yb = state.bore * sc / 2; txt(0, yb + 14, `A cap ${fmt(R.A1 * 1e4)} cm²`, 'gauge-val', f); txt(0, yb + 26, `A rod side ${fmt(R.A2 * 1e4)} cm²`, 'gauge-val', f); txt(0, yb + 38, `ratio ${fmt(R.A1 / R.A2, 2)}`, 'gauge-val', f);
}
function updateRig(dt) {
  drawCylinder();
  rig.g1.set(state.gP1); rig.g2.set(state.gP2); rig.g3.set(state.gP3);
  rig.rpmTxt.textContent = (state.run && !state.tripped ? state.rpm : 0).toLocaleString().replace(/,/g, ' ') + ' rpm';
  rig.tankT.textContent = fmt(state.Toil, 0) + ' °C'; rig.heat.style.opacity = clamp((state.Toil - 40) / 60, 0, 0.5);
  rig.reliefArrow.classList.toggle('on', R.Qr > 0.2); rig.nIn.set(state.mIn); rig.nOut.set(state.mOut);
  for (const k in rig.dcvBoxes) rig.dcvBoxes[k].classList.toggle('on', k === state.dcv);
  state.dash -= (R.Qp || 0) * 1.5 * dt; rig.hP.style.strokeDashoffset = state.dash; rig.hP.classList.toggle('hi', R.pp >= state.relief - 1 && R.Qp > 0); rig.hP.classList.toggle('off', !R.Qp);
  const qa = state.dcv === 'extend' ? R.Q : state.dcv === 'retract' ? -R.Q * R.A2 / R.A1 : 0, qb = state.dcv === 'retract' ? R.Q : state.dcv === 'extend' ? -R.Q * R.A2 / R.A1 : 0;
  rig.hA.style.strokeDashoffset = (rig.dA = (rig.dA || 0) - qa * 1.5 * dt); rig.hB.style.strokeDashoffset = (rig.dB = (rig.dB || 0) - qb * 1.5 * dt); rig.hA.classList.toggle('off', !qa); rig.hB.classList.toggle('off', !qb); rig.hB.classList.toggle('hi', R.intens);
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawTradeoff() {
  const cv = $('#tradeoff'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H);
  const L = 60, Rr = 70, T = 16, B = 36, pw = W - L - Rr, ph = H - T - B, Qp = PACK.Vg * state.rpm * PACK.etaV / 1000;
  const fmax = state.relief * 1e5 * area(80) / 1000, vmax = Qp / 60000 / area(25) * 1000;
  const Yf = f => T + (1 - f / fmax) * ph, Yv = v => T + (1 - Math.log10(Math.max(1, v)) / Math.log10(Math.max(10, vmax))) * ph;
  ctx.font = '11px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.strokeStyle = col('--grid');
  for (let i = 0; i <= 4; i++) { const f = fmax * i / 4; ctx.beginPath(); ctx.moveTo(L, Yf(f)); ctx.lineTo(L + pw, Yf(f)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(fmt(f, 0), L - 6, Yf(f) + 4); }
  ctx.textAlign = 'left'; for (const v of [1, 10, 100, 1000]) if (v <= vmax * 1.2) ctx.fillText(v + ' mm/s', L + pw + 6, Yv(v) + 4);
  ctx.textAlign = 'center'; ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.fillText(`force, kN at ${state.relief} bar`, L, 10); ctx.fillText(`speed at ${fmt(Qp)} L/min`, L + pw + 30, 10);
  const bw = pw / BORES.length;
  BORES.forEach((D, i) => { const d = RODS[D][1] || RODS[D][0], A1 = area(D), A2 = annulus(D, D === state.bore ? state.rod : d), x0 = L + i * bw, cur = D === state.bore;
    ctx.globalAlpha = cur ? 1 : 0.55; ctx.fillStyle = col('--water'); ctx.fillRect(x0 + bw * 0.12, Yf(state.relief * 1e5 * A1 / 1000), bw * 0.3, T + ph - Yf(state.relief * 1e5 * A1 / 1000));
    ctx.fillStyle = col('--water-deep'); ctx.fillRect(x0 + bw * 0.44, Yf(state.relief * 1e5 * A2 / 1000), bw * 0.3, T + ph - Yf(state.relief * 1e5 * A2 / 1000));
    const ve = Qp / 60000 / A1 * 1000, vr = Qp / 60000 / A2 * 1000; ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(x0 + bw * 0.27, Yv(ve), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(x0 + bw * 0.59, Yv(vr), 5, 0, 7); ctx.fill();
    ctx.globalAlpha = 1; ctx.fillStyle = cur ? col('--ink') : col('--muted'); ctx.font = (cur ? '700 ' : '') + '12px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`Ø${D}`, x0 + bw / 2, T + ph + 16); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillText(`${fmt(state.relief * 1e5 * A1 / 1000, 1)} kN · ${fmt(ve, 0)} mm/s`, x0 + bw / 2, T + ph + 30);
    if (cur) { ctx.strokeStyle = col('--brass'); ctx.lineWidth = 1.5; ctx.strokeRect(x0 + 2, T, bw - 4, ph); } });
  ctx.strokeStyle = col('--line'); ctx.lineWidth = 1; ctx.strokeRect(L, T, pw, ph);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H);
  const L = 52, Rr = 40, T = 8, B = 18, span = 40, t0 = state.t - span, X = t => L + (t - t0) / span * (W - L - Rr), tr = state.trace;
  const strips = [[T, T + 84], [T + 92, T + 176], [T + 184, H - B]];
  const strip = (i, lo, hi, label, ticks) => { const [a, b] = strips[i]; ctx.strokeStyle = col('--line'); ctx.strokeRect(L, a, W - L - Rr, b - a); ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; for (const v of ticks) ctx.fillText(v, L - 4, a + (1 - (v - lo) / (hi - lo)) * (b - a) + 3); ctx.textAlign = 'left'; ctx.font = '600 11px Barlow Condensed, sans-serif'; ctx.fillStyle = col('--ink'); ctx.fillText(label, L + 5, a + 12); return v => a + (1 - clamp((v - lo) / (hi - lo), -0.02, 1.02)) * (b - a); };
  const series = (Y, key, color, w, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash || []); let f = true; for (const p of tr) { const v = p[key]; if (!Number.isFinite(v)) continue; f ? ctx.moveTo(X(p.t), Y(v)) : ctx.lineTo(X(p.t), Y(v)); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; for (let s = 0; s <= span; s += 10) ctx.fillText('−' + (span - s) + ' s', X(t0 + s), H - 5);
  ctx.save(); ctx.beginPath(); ctx.rect(L, T, W - L - Rr, H - T - B); ctx.clip();
  const pmax = Math.max(300, ...tr.map(p => p.p3)); const Y1 = strip(0, 0, pmax, 'Pressure, bar', [0, 150, 300]); series(Y1, () => 0, col('--line'), 1); series(Y1, 'pp', col('--brass'), 1.5); series(Y1, 'p1', col('--water'), 2); series(Y1, 'p3', col('--water-deep'), 1.5);
  const fm = Math.max(10, state.relief * 1e5 * area(80) / 1000); const Y2 = strip(1, 0, fm, 'Force, kN  ·  position 0–300 mm', [0, Math.round(fm / 2), Math.round(fm)]); const Y2b = v => strips[1][0] + (1 - v / CYL.stroke) * (strips[1][1] - strips[1][0]); series(Y2b, 'x', col('--good'), 1.5); series(Y2, 'F', col('--alarm'), 2);
  const Y3 = strip(2, 0, 6, 'Power, kW  ·  motor, useful, wasted', [0, 3, 6]); series(Y3, () => PACK.kW, col('--alarm'), 1, [4, 4]); series(Y3, 'Pm', col('--ink'), 1.5); series(Y3, 'Pu', col('--good'), 1.5); series(Y3, 'Pw', col('--warn'), 1.5);
  ctx.restore();
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  $('#roP1').textContent = fmt(state.gP1, 0); $('#roP2').textContent = fmt(state.gP2, 0); $('#roP3').textContent = fmt(state.gP3, 0); $('#roP3box').classList.toggle('alarm', state.gP3 > CYL.sealLimit || state.gP2 > CYL.sealLimit);
  $('#roF').textContent = fmt(R.Frod, 2); $('#roFpa').textContent = fmt(R.Fpa, 2); $('#roV').textContent = fmt(Math.abs(R.v) * 1000, 0); $('#roX').textContent = fmt(state.x * 1000, 0);
  $('#roA').textContent = fmt(R.A1 * 1e4); $('#roA2').textContent = fmt(R.A2 * 1e4); $('#roQ').textContent = fmt(R.Qp); $('#roQc').textContent = fmt(R.Q); $('#roQr').textContent = fmt(R.Qr);
  $('#roPm').textContent = fmt(R.Pm, 2); $('#roPu').textContent = fmt(R.Pu, 2); $('#roPbox').classList.toggle('alarm', R.Pm > PACK.kW); $('#roPw').textContent = fmt(R.Pw, 2); $('#roWbox').classList.toggle('warnv', R.Pw > 1); $('#roT').textContent = fmt(state.Toil, 0); $('#roTbox').classList.toggle('alarm', state.Toil > 70); $('#roTbox').classList.toggle('warnv', state.Toil > 55 && state.Toil <= 70);
  const run = $('#pillRun'); run.textContent = state.tripped ? 'Tripped — ' + state.tripped : state.run ? 'Pump running' : 'Pump stopped'; run.classList.toggle('on', state.run); run.classList.toggle('trip', !!state.tripped);
  const st = $('#pillState'); st.textContent = state.blown ? 'Rod seal blown — repair' : !state.run ? '—' : state.dcv === 'neutral' ? (state.closedCentre ? 'Neutral — pump dead-headed at relief' : 'Neutral — pump unloaded to tank') : R.atEnd ? (state.dcv === 'extend' ? 'Fully extended — over relief' : 'Fully retracted — over relief') : R.stall ? 'Stalled against the load at relief pressure' : R.cav ? 'Load running away' : `${state.dcv === 'extend' ? 'Extending' : 'Retracting'} ${fmt(Math.abs(R.v) * 1000, 0)} mm/s`;
  $('#pillRelief').classList.toggle('hidden', !(R.Qr > 0.2)); $('#pillIntens').classList.toggle('hidden', !R.intens); $('#pillCav').classList.toggle('hidden', !R.cav); $('#pillOver').classList.toggle('hidden', !(R.Pm > PACK.kW)); $('#pillHot').classList.toggle('hidden', state.Toil <= 60);
  $('#btnStart').disabled = state.run || !!state.tripped; $('#btnStop').disabled = !state.run;
}

/* ------------------------------------------------------------------ log */
function record(tag = '') {
  const r = { i: state.log.length + 1, tag, bore: state.bore, rod: state.rod, A1: +(R.A1 * 1e4).toFixed(2), A2: +(R.A2 * 1e4).toFixed(2), dcv: state.dcv, load: state.loadType + ' ' + state.loadVal, relief: state.relief, p1: +R.pp.toFixed(0), p2: +R.p1.toFixed(0), p3: +R.p2.toFixed(0), F: +R.Frod.toFixed(2), Fpa: +R.Fpa.toFixed(2), v: +(R.v * 1000).toFixed(0), Q: +R.Q.toFixed(2), Pm: +R.Pm.toFixed(2), Pw: +R.Pw.toFixed(2), rpm: state.rpm, stall: R.stall, atEnd: R.atEnd };
  state.log.push(r); renderLog(); checkTasks();
}
function renderLog() {
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr><td>${r.i}</td><td>${r.tag}</td><td>${r.bore}</td><td>${r.rod}</td><td>${r.A1}</td><td>${r.A2}</td><td>${r.dcv}</td><td>${r.load}</td><td>${r.relief}</td><td>${r.p1}</td><td>${r.p2}</td><td>${r.p3}</td><td>${r.F}</td><td>${r.Fpa}</td><td>${r.v}</td><td>${r.Q}</td><td>${r.Pm}</td><td>${r.Pw}</td></tr>`).join('');
  $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,bore_mm,rod_mm,A_cap_cm2,A_rod_cm2,valve,load,relief_bar,P1_pump,P2_cap,P3_rod,F_kN,pA_kN,v_mms,Q_cyl_Lmin,motor_kW,waste_kW,rpm\n' + state.log.map(r => [r.i, r.tag, r.bore, r.rod, r.A1, r.A2, r.dcv, r.load, r.relief, r.p1, r.p2, r.p3, r.F, r.Fpa, r.v, r.Q, r.Pm, r.Pw, r.rpm].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}

/* ------------------------------------------------------------------ learning tasks */
const TASKS = [
  { id: 'fpa', t: 'Force = pressure × area', d: 'Fit the brake (or a load the cylinder cannot move), stall it, and record the load-cell force at three different bores with the same relief setting. Plot force against area.', ok: () => new Set(state.log.filter(r => r.stall && r.dcv === 'extend').map(r => r.bore)).size >= 3 },
  { id: 'speed', t: 'Speed = flow ÷ area', d: 'Record the extending speed at three bores with the same pump speed and both valves open.', ok: () => new Set(state.log.filter(r => r.dcv === 'extend' && r.v > 0 && !r.stall).map(r => r.bore)).size >= 3 },
  { id: 'rod', t: 'Rod side: less force, more speed', d: 'For one cylinder, record a moving extend and a moving retract reading. Compare the speeds and the p·A forces with the area ratio.', ok: () => { const e = state.log.filter(r => r.dcv === 'extend' && r.v > 0 && !r.stall), re = state.log.filter(r => r.dcv === 'retract' && r.v < 0); return e.some(a => re.some(b => a.bore === b.bore && a.rod === b.rod)); } },
  { id: 'stall', t: 'Stall it', d: 'Raise the load until the cylinder stops. Where does P1 sit, and what does the load cell read compared with p·A?', ok: () => state.flags.stall },
  { id: 'intens', t: 'Intensify the rod side', d: 'Extend against a load with the meter-out valve almost closed. Watch P3 climb past the relief setting.', ok: () => state.flags.intens },
  { id: 'waste', t: 'Where the power goes', d: 'Throttle the meter-in valve to half speed or less while lifting. Read the wasted power and watch the tank temperature.', ok: () => state.flags.waste },
  { id: 'overload', t: 'Overload the motor', d: 'High relief setting, full pump speed, big load: push the motor past 4 kW until it trips.', ok: () => state.flags.overload },
  { id: 'size', t: 'Size the cylinder', d: 'Lift 2 000 kg to full stroke with the relief valve at 150 bar or less, using the smallest bore that can do it.', ok: () => state.flags.size },
  { id: 'overrun', t: 'Overrunning load', d: 'Lower a heavy weight with the meter-out valve wide open, then hold it back with the valve.', ok: () => state.flags.overrun && state.flags.held },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt;
  Object.assign(R, solve());
  if (!state.blown) state.x = clamp(state.x + R.v * dt, 0, CYL.stroke);
  const lag = Math.min(1, dt / 0.12); state.gP1 += (R.pp - state.gP1) * lag; state.gP2 += (R.p1 - state.gP2) * lag; state.gP3 += (R.p2 - state.gP3) * lag;
  state.Toil += ((R.Pw * 1000 - COOL * (state.Toil - AMB)) / OIL_C) * dt * THERMAL;
  if (R.Pm > PACK.kW * 1.15) { state.overT += dt; if (state.overT > 4 && !state.tripped) trip('motor overload — ' + fmt(R.Pm, 1) + ' kW'); } else state.overT = Math.max(0, state.overT - dt);
  if (state.Toil > 80 && !state.tripped) trip('oil over-temperature 80 °C');
  if (Math.max(R.p1, R.p2) > CYL.burst && !state.blown) { state.blown = true; record('seal blown'); $('#btnReset').classList.remove('hidden'); }
  if (state.run) { if (R.stall) state.flags.stall = true; if (R.intens && R.Q > 0.05) state.flags.intens = true; if (state.mIn <= 50 && R.Q > 0.1 && R.Pw >= 1 && R.pp >= state.relief - 1) state.flags.waste = true; if (R.cav) state.flags.overrun = true; if (state.flags.overrun && state.dcv === 'retract' && R.Q > 0.05 && !R.cav && state.loadType === 'weight' && state.loadVal >= 1000 && state.mOut < 100) state.flags.held = true;
    if (state.loadType === 'weight' && state.loadVal >= 2000 && state.relief <= 150 && state.bore === 50 && state.dcv === 'extend' && state.x >= CYL.stroke - 0.005) state.flags.size = true; }
  if (R.stall && state.run) { state.stallT += dt; if (state.stallT > 1.5 && !state.flags['stallrec' + state.bore + state.relief]) { state.flags['stallrec' + state.bore + state.relief] = true; record('stall'); } } else state.stallT = 0;
  state.traceAcc += dt; if (state.traceAcc >= 0.1) { state.traceAcc = 0; state.trace.push({ t: state.t, pp: R.pp, p1: R.p1, p3: R.p2, F: R.Frod, x: state.x, Pm: R.Pm, Pu: R.Pu, Pw: R.Pw }); if (state.trace.length > 400) state.trace.shift(); }
  updateRig(dt); frame++; if (frame % 2 === 0) { drawTrace(); updateReadouts(); if (frame % 6 === 0) { drawTradeoff(); checkTasks(); } }
  requestAnimationFrame(tick);
}
function trip(reason) { state.tripped = reason; state.run = false; state.flags.overload = state.flags.overload || reason.startsWith('motor'); record('trip'); $('#btnReset').classList.remove('hidden'); }

/* ------------------------------------------------------------------ wiring */
function fillRods() { const sel = $('#rod'); sel.innerHTML = RODS[state.bore].map(d => `<option value="${d}">${d} mm rod</option>`).join(''); if (!RODS[state.bore].includes(state.rod)) state.rod = RODS[state.bore][1] || RODS[state.bore][0]; sel.value = state.rod; }
function setLoadType(t) { state.loadType = t; const [name, min, max, step, def, f] = LOADS[t]; $('#loadName').textContent = name; const e = $('#loadVal'); e.min = min; e.max = max; e.step = step; e.value = def; state.loadVal = def; $('#loadValOut').textContent = f(def); e.disabled = t === 'none'; }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') { e.checked = !!v; state[id] = !!v; } else { e.value = v; state[id] = isNaN(+v) ? v : +v; } e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setDcv(d) { state.dcv = d; document.querySelectorAll('#dcvSeg button').forEach(b => b.classList.toggle('on', b.dataset.dcv === d)); }
function start() { if (state.tripped) return; state.run = true; }
function resetTrip() { state.tripped = null; state.blown = false; state.overT = 0; if (state.Toil > 60) state.Toil = 45; $('#btnReset').classList.add('hidden'); }
const PRESETS = {
  small: () => { setCtl('bore', 25); fillRods(); setCtl('rod', 12); $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 200); setCtl('relief', 150); setCtl('rpm', 1450); setCtl('mIn', 100); setCtl('mOut', 100); start(); },
  big: () => { setCtl('bore', 80); fillRods(); setCtl('rod', 45); $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 3000); setCtl('relief', 150); setCtl('rpm', 1450); setCtl('mIn', 100); setCtl('mOut', 100); start(); },
  stall: () => { $('#loadType').value = 'brake'; setLoadType('brake'); setCtl('loadVal', 100); setCtl('relief', 150); setCtl('mIn', 100); setCtl('mOut', 100); start(); },
  lift: () => { $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 2000); setCtl('relief', 150); setCtl('rpm', 1450); setCtl('mIn', 100); setCtl('mOut', 100); start(); },
  intens: () => { setCtl('bore', 50); fillRods(); setCtl('rod', 28); $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 500); setCtl('relief', 150); setCtl('mIn', 100); setCtl('mOut', 12); start(); },
  waste: () => { $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 1000); setCtl('relief', 200); setCtl('rpm', 1450); setCtl('mIn', 30); setCtl('mOut', 100); start(); },
  overrun: () => { setCtl('bore', 50); fillRods(); setCtl('rod', 28); $('#loadType').value = 'weight'; setLoadType('weight'); setCtl('loadVal', 2500); setCtl('relief', 150); setCtl('mIn', 100); setCtl('mOut', 100); state.x = CYL.stroke; start(); },
};
function bind() {
  $('#bore').addEventListener('change', e => { state.bore = +e.target.value; fillRods(); }); $('#rod').addEventListener('change', e => state.rod = +e.target.value); fillRods();
  $('#loadType').addEventListener('change', e => setLoadType(e.target.value)); $('#loadVal').addEventListener('input', e => { state.loadVal = +e.target.value; $('#loadValOut').textContent = LOADS[state.loadType][5](state.loadVal); });
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('relief', v => v + ' bar'); bindR('rpm', v => v.toLocaleString().replace(/,/g, ' ') + ' rpm'); bindR('mIn', v => v + ' %'); bindR('mOut', v => v + ' %');
  $('#closedCentre').addEventListener('change', e => state.closedCentre = e.target.checked);
  document.querySelectorAll('#dcvSeg button').forEach(b => b.onclick = () => setDcv(b.dataset.dcv));
  $('#btnStart').onclick = start; $('#btnStop').onclick = () => { state.run = false; }; $('#btnReset').onclick = resetTrip; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('hp_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const k = e.key.toUpperCase();
    if (e.key === 'ArrowUp') setDcv('extend'); else if (e.key === 'ArrowDown') setDcv('retract'); else if (k === 'S') state.run ? state.run = false : start(); else if (k === 'R') record(''); else if (/^[1-6]$/.test(k)) { setCtl('bore', BORES[+k - 1]); fillRods(); } else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  document.addEventListener('keyup', e => { if (e.key === 'ArrowUp' || e.key === 'ArrowDown') setDcv('neutral'); });
  window.addEventListener('resize', () => { drawTradeoff(); drawTrace(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.HydraulicPowerDemo = {
  getState: () => ({ inputs: { bore: state.bore, rod: state.rod, loadType: state.loadType, loadVal: state.loadVal, relief: state.relief, rpm: state.rpm, mIn: state.mIn, mOut: state.mOut, closedCentre: state.closedCentre, dcv: state.dcv, run: state.run }, results: { P1: R.pp, P2: R.p1, P3: R.p2, F: R.Frod, Fpa: R.Fpa, v: R.v * 1000, x: state.x * 1000, A1: R.A1 * 1e4, A2: R.A2 * 1e4, Qp: R.Qp, Q: R.Q, Pm: R.Pm, Pu: R.Pu, Pw: R.Pw, Toil: state.Toil, stall: R.stall, intens: R.intens, cav: R.cav, tripped: state.tripped }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.bore !== undefined) { setCtl('bore', +inp.bore); fillRods(); } if (inp.rod !== undefined) setCtl('rod', +inp.rod); if (inp.loadType) { $('#loadType').value = inp.loadType; setLoadType(inp.loadType); } if (inp.loadVal !== undefined) setCtl('loadVal', +inp.loadVal); for (const k of ['relief', 'rpm', 'mIn', 'mOut']) if (inp[k] !== undefined) setCtl(k, +inp[k]); if (inp.closedCentre !== undefined) setCtl('closedCentre', !!inp.closedCentre); if (inp.dcv) setDcv(inp.dcv); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.reset) resetTrip(); if (inp.run === true) start(); if (inp.run === false) state.run = false; },
  meta: { id: 'hydraulic-power-demonstrator', title: 'Hydraulic Power Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.HydraulicPowerDemo.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'hydraulic-power-demonstrator', state: window.HydraulicPowerDemo.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('hp_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); setLoadType('weight'); renderLog(); Object.assign(R, solve()); drawCylinder(); drawTradeoff(); drawTrace(); updateReadouts();
  requestAnimationFrame(tick);
}
build();
