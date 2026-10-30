/* API 650 Tank Design Demonstrator — a welded, atmospheric, flat-bottom storage tank sized course by course with the
   one-foot method, allowable stresses per material, corrosion allowance, standard plate rounding, minimum thicknesses,
   bottom and annular plates, roof plate (supported cone, self-supporting cone or dome), top and intermediate wind
   girders, wind overturning and anchorage, steel mass and a constant-capacity diameter sweep. Fill and hydrotest on a
   clock with the live hoop stress in every course. Standalone, offline. Teaching model — data and clause references
   are illustrative and must be verified against the edition of API 650 in force. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const PLATES = [5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40, 45];
const MAT = { // Sd design stress, St hydrostatic test stress (MPa) after Table 5-2 (SI); Fy min yield; material group after Table 4-4
  a283c: { name: 'A283M Gr C', Sd: 137, St: 154, Fy: 205, group: 'I' },
  a36: { name: 'A36M', Sd: 160, St: 171, Fy: 250, group: 'I' },
  a516_415: { name: 'A516M Gr 415', Sd: 147, St: 165, Fy: 220, group: 'II' },
  a516_485: { name: 'A516M Gr 485', Sd: 173, St: 195, Fy: 260, group: 'III' },
  a573_485: { name: 'A573M Gr 485', Sd: 193, St: 208, Fy: 290, group: 'IV' },
  a537_1: { name: 'A537M Cl 1', Sd: 194, St: 220, Fy: 345, group: 'VI' },
};
const ROOF = { open: 'Open top (external floating roof)', cone: 'Supported cone (rafters and columns)', selfcone: 'Self-supporting cone', dome: 'Self-supporting dome' };
const ANN = { bands: [190, 210, 220, 250], rows: [[19, [6, 6, 7, 9]], [25, [6, 7, 10, 11]], [32, [6, 9, 12, 14]], [40, [8, 11, 14, 17]], [45, [9, 13, 16, 19]]] }; // Table 5-1a (SI)
const RHO = 7850, ROOF_STRUCT = { open: 0, cone: 22, selfcone: 4, dome: 4 }; // kg/m² of framing / top ring allowance [illustrative]

/* ------------------------------------------------------------------ state */
const state = { D: 30, n: 8, hc: 1.8, H: 14.1, G: 0.85, CA: 1.5, mat: 'a283c', V: 190, roof: 'cone', slope: 3.6, rr: 1.0, Lr: 1.0, S: 0, fillRate: 1000, fluid: 'product', tscale: 300, run: true,
  level: 0, mode: 'idle', overrides: {}, sel: -1, t: 0, spill: false, log: [], flags: {}, csv: '' };
const R = {};

/* ------------------------------------------------------------------ model */
const roundUp = t => PLATES.find(p => p >= t - 1e-9) ?? Infinity;
function design(p) {
  const D = clamp(p.D, 1, 200), n = clamp(Math.round(p.n), 1, 24), hc = clamp(p.hc, 0.5, 4), Hs = n * hc, H = clamp(p.H, 0, Hs), G = clamp(p.G, 0.1, 3), CA = clamp(p.CA, 0, 10), m = MAT[p.mat] || MAT.a283c, V = clamp(p.V, 10, 400), ov = p.overrides || {};
  const tmin = D < 15 ? 5 : D < 36 ? 6 : D <= 60 ? 8 : 10, onefoot = D <= 61;
  const courses = [];
  for (let i = 0; i < n; i++) {
    const y = i * hc, Hi = Math.max(H - y, 0), h1 = Math.max(Hi - 0.3, 0);
    const td = 4.9 * D * h1 * G / m.Sd + CA, tt = 4.9 * D * h1 / m.St;
    const treq = Math.max(td, tt, tmin), tauto = roundUp(treq), gov = treq === tmin ? 'minimum' : td >= tt ? 'product' : 'test';
    courses.push({ i, y, Hi, td, tt, treq, tauto, gov, t: Number.isFinite(ov[i]) ? ov[i] : tauto, over: Number.isFinite(ov[i]) });
  }
  for (let i = n - 2; i >= 0; i--) if (!courses[i].over && !courses[i + 1].over) courses[i].tauto = courses[i].t = Math.max(courses[i].t, courses[i + 1].t);
  let allOk = true;
  for (const c of courses) {
    const h1 = Math.max(c.Hi - 0.3, 0);
    c.sProd = c.t - CA > 0 ? 4.9 * D * h1 * G / (c.t - CA) : Infinity; c.sTest = 4.9 * D * h1 / c.t;
    c.okProd = c.sProd <= m.Sd + 1e-9; c.okTest = c.sTest <= m.St + 1e-9; c.okMin = c.t >= tmin; c.okMax = c.t <= 45; c.okOrder = c.i === n - 1 || c.t >= courses[c.i + 1].t;
    c.ok = c.okProd && c.okTest && c.okMin && c.okMax && c.okOrder; if (!c.ok) allOk = false;
    c.mass = Math.PI * D * hc * c.t / 1000 * RHO;
  }
  const c1 = courses[0], massShell = courses.reduce((a, c) => a + c.mass, 0);
  // bottom and annular plates (5.4, 5.5, Table 5-1a)
  const tb = 6 + CA, s1 = Math.max(c1.sProd, c1.sTest), hiGroup = ['IV', 'IVA', 'V', 'VI'].includes(m.group);
  const annular = hiGroup || c1.sProd > 160 || c1.sTest > 171; let ta = tb, annOut = false;
  if (annular) { const bi = ANN.bands.findIndex(b => s1 <= b); const row = ANN.rows.find(r => c1.t <= r[0]) || ANN.rows[ANN.rows.length - 1]; if (bi < 0) { annOut = true; ta = row[1][3] + 2 + CA; } else ta = row[1][bi] + CA; }
  const La = Math.max(600, H * G > 0 ? 215 * (ta - CA) / Math.sqrt(H * G) : 600);
  const massBottom = Math.PI * (D / 2 + 0.05) ** 2 * tb / 1000 * RHO + (annular ? Math.PI * D * (La / 1000 + 0.05) * (ta - tb) / 1000 * RHO : 0);
  // roof (5.10)
  const theta = clamp(p.slope, 0.5, 60) * Math.PI / 180, rr = clamp(p.rr, 0.5, 2) * D, Lr = Math.max(p.Lr, 0), S = Math.max(p.S, 0), roof = ROOF[p.roof] ? p.roof : 'cone';
  let tr = 0, T = 0, area = 0, ringF = 0, roofOk = true, roofNote = '', hr = 0, phi = theta;
  if (roof !== 'open') {
    let tplate = 5;
    for (let k = 0; k < 3; k++) { T = Math.min(tplate, 13) * RHO * 9.81 / 1e6 + Math.max(Lr, S, 1.0); // kPa: plate dead load + live or snow (min 1.0 kPa)
      if (roof === 'cone') tplate = 5 + CA;
      else if (roof === 'selfcone') tplate = Math.max(5 + CA, D / (4.8 * Math.sin(theta)) * Math.sqrt(T / 2.2) + CA);
      else tplate = Math.max(5 + CA, rr / 2.4 * Math.sqrt(T / 2.2) + CA); }
    tr = tplate;
    if (roof === 'dome') { const Rr = D / 2; hr = rr - Math.sqrt(Math.max(rr * rr - Rr * Rr, 0)); area = 2 * Math.PI * rr * hr; phi = Math.asin(Math.min(Rr / rr, 1)); if (rr < 0.8 * D - 1e-9 || rr > 1.2 * D + 1e-9) { roofOk = false; roofNote = 'dome radius must be 0.8 D to 1.2 D'; } }
    else { hr = D / 2 * Math.tan(theta); area = Math.PI * (D / 2) ** 2 / Math.cos(theta); }
    if (roof === 'selfcone' && (theta < 9.5 * Math.PI / 180 - 1e-9 || theta > 37 * Math.PI / 180 + 1e-9)) { roofOk = false; roofNote = 'self-supporting cone slope must be 9.5° to 37°'; }
    if (roof === 'cone' && theta < Math.atan(1 / 16) - 1e-9) { roofOk = false; roofNote = 'supported cone slope must be at least 1:16'; }
    if (roof !== 'cone' && tr > 13 + 1e-9) { roofOk = false; roofNote = 'self-supporting roof plate exceeds 13 mm — use a supported roof or a dome'; }
    ringF = roof === 'cone' ? 0 : T * D * D / (8 * Math.tan(phi)); // kN hoop compression at the roof-to-shell junction of a self-supporting roof
    if (roof === 'cone' && roofOk) roofNote = 'top angle at least ' + (D <= 11 ? '51 × 51 × 4.8' : D <= 18 ? '51 × 51 × 6.4' : '76 × 76 × 9.5') + ' mm';
  }
  const ringA = ringF * 1000 / (0.6 * m.Fy), roofPlateMass = area * tr / 1000 * RHO, roofMass = roofPlateMass + area * ROOF_STRUCT[roof] + (roof === 'open' ? Math.PI * D * 12 : 0);
  // wind girders (5.9.6, 5.9.7)
  const q = (V / 190) ** 2, Pw = 0.86 * q, Pr = 1.44 * q, tTop = courses[n - 1].t;
  const H1 = 9.47 * tTop * Math.sqrt((tTop / D) ** 3) / q, Wtr = courses.map(c => hc * (tTop / c.t) ** 2.5), Htr = Wtr.reduce((a, b) => a + b, 0);
  const nGird = Htr > H1 ? Math.ceil(Htr / H1) - 1 : 0, girders = [];
  for (let j = 1; j <= nGird; j++) { let target = Htr * j / (nGird + 1), acc = 0, yA = Hs; for (let i = n - 1; i >= 0; i--) { if (acc + Wtr[i] >= target) { yA = Hs - (i + 1) * hc + hc * (1 - (target - acc) / Wtr[i]); break; } acc += Wtr[i]; yA = Hs - (i) * hc; } girders.push(yA); }
  const Ztop = roof === 'open' ? D * D * Hs / 17 * q : 0, Zint = nGird ? D * D * (Htr / (nGird + 1)) / 17 * q : 0;
  // overturning (5.11) — unanchored tank
  const Wshell = massShell * 9.81 / 1000, Wroof = roofMass * 9.81 / 1000, WroofShell = roof === 'cone' ? 0.5 * Wroof : Wroof, WroofPlate = (roof === 'cone' ? 0.5 : 1) * roofPlateMass * 9.81 / 1000;
  const Mw = Pw * D * Hs * Hs / 2 + (roof === 'open' ? 0 : Pr * Math.PI * D * D / 4 * D / 2), MDL = (Wshell + WroofShell) * D / 2, MDLR = WroofPlate * D / 2;
  const wL = Math.min(59 * (tb - CA) * Math.sqrt(m.Fy * Math.max(H, 0.01)), 70.4 * H * D), MF = wL / 1000 * Math.PI * D * D / 2;
  const crit1 = 0.6 * Mw < MDL / 1.5 + MDLR, crit2 = Mw < (MDL + MF) / 2 + MDLR, anchored = !(crit1 && crit2), Fslide = Pw * D * Hs, slideOk = Fslide < 0.4 * (Wshell + Wroof);
  const massTotal = massShell + massBottom + roofMass, Vnom = Math.PI * D * D / 4 * Hs, Vwork = Math.PI * D * D / 4 * H;
  return { D, n, hc, Hs, H, G, CA, m, V, tmin, onefoot, courses, allOk, massShell, tb, ta, La, annular, annOut, s1, massBottom, roof, theta, phi, rr, hr, tr, T, area, ringF, ringA, roofOk, roofNote, roofMass, roofPlateMass, Pw, Pr, H1, Htr, nGird, girders, Ztop, Zint, Mw, MDL, MDLR, MF, wL, crit1, crit2, anchored, Fslide, slideOk, massTotal, Vnom, Vwork, kgm3: massTotal / Math.max(Vwork, 1e-6) };
}
function sweep(s) { // same design rules at constant working capacity, freeboard and course height, no overrides
  const V = Math.PI * s.D * s.D / 4 * Math.min(s.H, s.n * s.hc), fb = s.n * s.hc - Math.min(s.H, s.n * s.hc), pts = [];
  for (let D = 6; D <= 90; D += 1) { const H = V / (Math.PI * D * D / 4); if (H < 1.5 || H + fb > 30) continue; const n = Math.ceil((H + fb) / s.hc - 1e-9); const r = design(Object.assign({}, s, { D, H, n, overrides: {} })); pts.push({ D, H, mass: r.massTotal, shell: r.massShell, anchored: r.anchored, gird: r.nGird, onefoot: r.onefoot }); }
  return pts;
}
function liveStress(r) { const s = state, Gf = s.fluid === 'water' ? 1 : r.G; return r.courses.map(c => { const h = Math.max(s.level - c.y - 0.3, 0); const t = s.fluid === 'water' ? c.t : c.t - r.CA; return t > 0 ? 4.9 * r.D * h * Gf / t : Infinity; }); }

/* ------------------------------------------------------------------ dynamics (fill, drain, gusts) */
function step(dt) {
  const s = state, r = R; s.t += dt * s.tscale; if (!s.run) return;
  const A = Math.PI * s.D * s.D / 4, dz = s.fillRate * s.tscale * dt / 3600 / A;
  if (s.mode === 'fill') { s.level += dz; if (s.level >= r.Hs) { s.level = r.Hs; s.spill = true; s.mode = 'idle'; } if (s.mode === 'fill' && s.target !== undefined && s.level >= s.target) { s.level = s.target; s.mode = 'idle'; if (s.fluid === 'water' && s.level >= 0.98 * r.H) s.flags.hydro = true; } }
  else if (s.mode === 'drain') { s.level -= dz; if (s.level <= 0) { s.level = 0; s.mode = 'idle'; s.spill = false; } }
  if (s.level > r.Hs) s.level = r.Hs;
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', E = {};
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); if (parent) parent.appendChild(e); return e; }
function buildRig() {
  const host = $('#rig'); host.innerHTML = ''; const svg = el('svg', { viewBox: '0 0 960 520', role: 'img', 'aria-label': 'Tank elevation' }, host); E.svg = svg;
  E.wind = el('g', {}, svg); E.found = el('g', {}, svg); E.fluid = el('g', {}, svg); E.shell = el('g', {}, svg); E.roof = el('g', {}, svg); E.extras = el('g', {}, svg); E.dims = el('g', {}, svg); E.txt = el('g', {}, svg);
  svg.addEventListener('click', e => { const c = e.target && e.target.closest ? e.target.closest('[data-i]') : null; if (c) { state.sel = +c.dataset.i; drawRig(); updateReadouts(); } });
}
function drawRig() {
  const s = state, r = R; if (!E.svg || !r.courses) return;
  for (const k of ['wind', 'found', 'fluid', 'shell', 'roof', 'extras', 'dims', 'txt']) E[k].innerHTML = '';
  const roofH = r.roof === 'open' ? 0 : r.hr, totalH = r.Hs + roofH, sc = Math.min(540 / r.D, 340 / Math.max(totalH, 1)), cx = 520, yb = 440, xL = cx - r.D / 2 * sc, xR = cx + r.D / 2 * sc, yTop = yb - r.Hs * sc;
  const wall = t => Math.min(4 + t * 0.55, 28);
  // foundation, bottom, annular
  el('rect', { x: xL - 30, y: yb, width: xR - xL + 60, height: 26, class: 'pad' }, E.found); el('rect', { x: xL - 8, y: yb - 4, width: xR - xL + 16, height: 4, class: 'bottomp' }, E.found);
  if (r.annular) { const w = Math.max(r.La / 1000 * sc, 14); el('rect', { x: xL - 10, y: yb - 6, width: w + 10, height: 6, class: 'annular' }, E.found); el('rect', { x: xR - w, y: yb - 6, width: w + 10, height: 6, class: 'annular' }, E.found); }
  // liquid
  if (s.level > 0.005) { const yl = yb - 4 - s.level * sc; el('rect', { x: xL + 2, y: yl, width: xR - xL - 4, height: yb - 4 - yl, class: 'liquid ' + (s.fluid === 'water' ? 'water' : 'product') }, E.fluid); el('line', { x1: xL + 2, y1: yl, x2: xR - 2, y2: yl, class: 'surface' }, E.fluid); el('text', { x: xL + 14, y: Math.min(yl + 15, yb - 10), class: 'val' }, E.fluid).textContent = `${s.fluid === 'water' ? 'water' : 'product G ' + fmt(r.G, 2)} · ${fmt(s.level, 2)} m`; }
  // courses
  const live = liveStress(r);
  r.courses.forEach((c, i) => { const y1 = yb - 4 - (c.y + r.hc) * sc, h = r.hc * sc, w = wall(c.t), bad = !c.ok, sel = s.sel === i, live1 = live[i], lim = s.fluid === 'water' ? r.m.St : r.m.Sd, hot = live1 > lim;
    el('rect', { x: xL - w / 2, y: y1, width: w, height: h, class: 'course' + (bad ? ' bad' : '') + (sel ? ' sel' : '') + (c.over ? ' over' : '') + (hot ? ' hot' : ''), 'data-i': i }, E.shell); el('rect', { x: xR - w / 2, y: y1, width: w, height: h, class: 'course' + (bad ? ' bad' : '') + (sel ? ' sel' : '') + (c.over ? ' over' : '') + (hot ? ' hot' : ''), 'data-i': i }, E.shell);
    el('rect', { x: xL + w / 2, y: y1, width: xR - xL - w, height: h, class: 'coursehit', 'data-i': i }, E.shell);
    el('line', { x1: xL - 14, y1: y1 + h, x2: xR + 14, y2: y1 + h, class: 'weld' }, E.shell);
    const tx = el('text', { x: xL - w / 2 - 8, y: y1 + h / 2 + 4, class: 'ctag' + (bad ? ' bad' : '') + (sel ? ' sel' : ''), 'text-anchor': 'end' }, E.txt); tx.textContent = `C${i + 1} · ${c.t} mm${c.over ? ' *' : ''}`;
    if (h > 14) { const t2 = el('text', { x: xR + w / 2 + 8, y: y1 + h / 2 + 4, class: 'cval' + (hot ? ' bad' : '') }, E.txt); t2.textContent = s.level > 0.01 && Number.isFinite(live1) ? `${fmt(live1, 0)} MPa` : `${c.gov} · req ${fmt(c.treq, 1)}`; } });
  // roof
  const rw = wall(r.tr || 5);
  if (r.roof === 'open') { el('path', { d: `M${xL - 12} ${yTop} h24 v6 h-16 z`, class: 'girder' }, E.roof); el('path', { d: `M${xR + 12} ${yTop} h-24 v6 h16 z`, class: 'girder' }, E.roof); el('text', { x: xR - 12, y: yTop - 8, class: 'lbl', 'text-anchor': 'end' }, E.txt).textContent = 'top wind girder'; }
  else if (r.roof === 'dome') { const Rr = r.rr * sc, yc = yTop + Rr - roofH * sc; el('path', { d: `M${xL} ${yTop} A${Rr} ${Rr} 0 0 1 ${xR} ${yTop}`, class: 'roofp' + (r.roofOk ? '' : ' bad'), 'stroke-width': rw }, E.roof); el('path', { d: `M${xL} ${yTop} A${Rr} ${Rr} 0 0 1 ${xR} ${yTop} Z`, class: 'roofin' }, E.roof); void yc; }
  else { const ya = yTop - roofH * sc; el('path', { d: `M${xL} ${yTop} L${cx} ${ya} L${xR} ${yTop} Z`, class: 'roofin' }, E.roof); el('path', { d: `M${xL} ${yTop} L${cx} ${ya} L${xR} ${yTop}`, class: 'roofp' + (r.roofOk ? '' : ' bad'), 'stroke-width': rw }, E.roof);
    if (r.roof === 'cone') { const nc = Math.max(1, Math.round(r.D / 12)); for (let k = 1; k < nc + 1; k++) { const x = xL + (xR - xL) * k / (nc + 1), yr = yTop - roofH * sc * (1 - Math.abs(x - cx) / ((xR - xL) / 2)); el('line', { x1: x, y1: yb - 4, x2: x, y2: yr + 2, class: 'column' }, E.roof); } } }
  if (r.roof !== 'open') { const t = el('text', { x: cx, y: yTop - roofH * sc - 12, class: 'lbl', 'text-anchor': 'middle' }, E.txt); t.textContent = `${ROOF[r.roof]} · plate ${fmt(r.tr, 1)} mm${r.roofOk ? '' : ' ✗'}`; }
  // ring at the junction
  el('rect', { x: xL - 8, y: yTop - 3, width: 16, height: 6, class: 'ring' }, E.roof); el('rect', { x: xR - 8, y: yTop - 3, width: 16, height: 6, class: 'ring' }, E.roof);
  // intermediate girders
  r.girders.forEach((yA, k) => { const y = yb - 4 - yA * sc; el('path', { d: `M${xL - 14} ${y} h14 v-8 h-6 z`, class: 'girder' }, E.extras); el('path', { d: `M${xR + 14} ${y} h-14 v-8 h6 z`, class: 'girder' }, E.extras); el('text', { x: xL + 14, y: y - 4, class: 'lbl' }, E.txt).textContent = `intermediate girder ${k + 1} · ${fmt(yA, 1)} m`; });
  // anchors
  if (r.anchored) { for (const x of [xL, xR]) { el('path', { d: `M${x - 12} ${yb} l12 -22 l12 22 z`, class: 'chair' }, E.extras); el('line', { x1: x, y1: yb - 22, x2: x, y2: yb + 24, class: 'bolt' }, E.extras); } el('text', { x: cx, y: yb + 42, class: 'lbl bad', 'text-anchor': 'middle' }, E.txt).textContent = 'ANCHORS REQUIRED — unanchored tank overturns in the design wind'; }
  else el('text', { x: cx, y: yb + 42, class: 'lbl', 'text-anchor': 'middle' }, E.txt).textContent = `unanchored · stable in ${r.V} km/h wind`;
  // design level marker
  const yH = yb - 4 - r.H * sc; el('line', { x1: xL, y1: yH, x2: xR, y2: yH, class: 'hline' }, E.dims); el('text', { x: xR - 12, y: yH + 14, class: 'lbl', 'text-anchor': 'end' }, E.txt).textContent = `design liquid level H ${fmt(r.H, 2)} m`;
  // dimensions
  const yd = Math.max(yTop - roofH * sc - 34, 16); el('line', { x1: xL, y1: yd, x2: xR, y2: yd, class: 'dim' }, E.dims); el('line', { x1: xL, y1: yd - 6, x2: xL, y2: yd + 6, class: 'dim' }, E.dims); el('line', { x1: xR, y1: yd - 6, x2: xR, y2: yd + 6, class: 'dim' }, E.dims); el('text', { x: cx, y: yd - 5, class: 'dimt', 'text-anchor': 'middle' }, E.txt).textContent = `D ${fmt(r.D, 1)} m`;
  const xd = xL - 92; el('line', { x1: xd, y1: yTop, x2: xd, y2: yb - 4, class: 'dim' }, E.dims); el('line', { x1: xd - 6, y1: yTop, x2: xd + 6, y2: yTop, class: 'dim' }, E.dims); el('line', { x1: xd - 6, y1: yb - 4, x2: xd + 6, y2: yb - 4, class: 'dim' }, E.dims); el('text', { x: xd - 8, y: (yTop + yb) / 2, class: 'dimt', 'text-anchor': 'middle', transform: `rotate(-90 ${xd - 8} ${(yTop + yb) / 2})` }, E.txt).textContent = `${fmt(r.Hs, 1)} m · ${r.n} × ${fmt(r.hc, 2)} m`;
  // wind arrows
  const q = (r.V / 190) ** 2, len = Math.min(30 + 50 * q, 95), wob = Math.sin(s.t * 2.2) * 5 * q, xe = xd - 16; for (let k = 0; k < 4; k++) { const y = yTop + (yb - yTop) * (k + 0.5) / 4, x0 = xe - len + wob; el('path', { d: `M${x0} ${y} h${len - 12} m-8 -6 l8 6 l-8 6`, class: 'warrow' }, E.wind); }
  el('text', { x: xe - len / 2, y: yTop - 8, class: 'lbl', 'text-anchor': 'middle' }, E.txt).textContent = `wind ${r.V} km/h · ${fmt(r.Pw, 2)} kPa`;
  if (s.spill) el('text', { x: cx, y: yd - 16, class: 'lbl bad', 'text-anchor': 'middle' }, E.txt).textContent = 'OVERFILL — liquid at the top of the shell';
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function drawThick() {
  const cv = $('#chart'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 70, r: 20, t: 14, b: 32 }, r = R; frame(ctx, W, H, pad);
  const n = r.courses.length, tmax = Math.max(48, ...r.courses.map(c => Math.max(c.t, c.treq)) ) * 1.02, x = t => pad.l + (W - pad.l - pad.r) * t / tmax, rowH = (H - pad.t - pad.b) / n;
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillStyle = css('--muted'); ctx.textAlign = 'center';
  for (let t = 0; t <= tmax; t += 5) { const X = x(t); ctx.strokeStyle = css('--grid'); ctx.beginPath(); ctx.moveTo(X, pad.t); ctx.lineTo(X, H - pad.b); ctx.stroke(); ctx.fillText(t + '', X, H - pad.b + 14); }
  ctx.fillText('plate thickness, mm', (pad.l + W - pad.r) / 2, H - 4);
  r.courses.slice().reverse().forEach((c, k) => { const y0 = pad.t + k * rowH + 3, hgt = rowH - 6;
    ctx.fillStyle = c.ok ? css('--water-soft') : css('--alarm'); ctx.fillRect(pad.l, y0, x(c.t) - pad.l, hgt);
    ctx.fillStyle = css('--brass'); ctx.fillRect(pad.l, y0 + hgt * 0.3, x(c.td) - pad.l, hgt * 0.2); ctx.fillStyle = css('--good'); ctx.fillRect(pad.l, y0 + hgt * 0.55, x(c.tt) - pad.l, hgt * 0.2);
    ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x(c.treq), y0); ctx.lineTo(x(c.treq), y0 + hgt); ctx.stroke();
    ctx.fillStyle = css('--ink'); ctx.textAlign = 'right'; ctx.fillText(`C${c.i + 1}`, pad.l - 6, y0 + hgt / 2 + 4); ctx.textAlign = 'left'; ctx.fillText(`${c.t} mm · ${c.gov}${c.over ? ' · manual' : ''}`, x(c.t) + 5, y0 + hgt / 2 + 4); });
  ctx.strokeStyle = css('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(x(r.tmin), pad.t); ctx.lineTo(x(r.tmin), H - pad.b); ctx.stroke(); ctx.beginPath(); ctx.moveTo(x(45), pad.t); ctx.lineTo(x(45), H - pad.b); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = css('--alarm'); ctx.textAlign = 'left'; ctx.fillText(`min ${r.tmin}`, x(r.tmin) + 3, pad.t + 10); ctx.textAlign = 'right'; ctx.fillText('max 45', x(45) - 3, pad.t + 10);
}
function drawStress() {
  const cv = $('#schart'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 70, r: 20, t: 14, b: 32 }, r = R; frame(ctx, W, H, pad);
  const n = r.courses.length, live = liveStress(r), umax = 130, x = u => pad.l + (W - pad.l - pad.r) * clamp(u, 0, umax) / umax, rowH = (H - pad.t - pad.b) / n;
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillStyle = css('--muted'); ctx.textAlign = 'center';
  for (let u = 0; u <= umax; u += 10) { const X = x(u); ctx.strokeStyle = u === 100 ? css('--alarm') : css('--grid'); ctx.beginPath(); ctx.moveTo(X, pad.t); ctx.lineTo(X, H - pad.b); ctx.stroke(); ctx.fillText(u + '', X, H - pad.b + 14); }
  ctx.fillText('stress as % of the allowable · product against Sd, hydrotest against St, live fill against the same', (pad.l + W - pad.r) / 2, H - 4);
  r.courses.slice().reverse().forEach((c, k) => { const y0 = pad.t + k * rowH + 3, hgt = rowH - 6, up = c.sProd / r.m.Sd * 100, ut = c.sTest / r.m.St * 100, ul = live[c.i] / (state.fluid === 'water' ? r.m.St : r.m.Sd) * 100;
    ctx.fillStyle = up > 100 ? css('--alarm') : css('--brass'); ctx.fillRect(pad.l, y0, x(up) - pad.l, hgt * 0.42); ctx.fillStyle = ut > 100 ? css('--alarm') : css('--good'); ctx.fillRect(pad.l, y0 + hgt * 0.5, x(ut) - pad.l, hgt * 0.42);
    if (state.level > 0.01 && Number.isFinite(ul)) { ctx.strokeStyle = ul > 100 ? css('--alarm') : css('--water'); ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(x(ul), y0 - 1); ctx.lineTo(x(ul), y0 + hgt + 1); ctx.stroke(); }
    ctx.fillStyle = css('--ink'); ctx.textAlign = 'right'; ctx.fillText(`C${c.i + 1}`, pad.l - 6, y0 + hgt / 2 + 4); ctx.textAlign = 'left'; ctx.fillText(`${fmt(c.sProd, 0)} / ${fmt(c.sTest, 0)} MPa`, Math.min(x(Math.max(up, ut)) + 5, W - pad.r - 92), y0 + hgt / 2 + 4); });
}
function drawSweep() {
  const cv = $('#trace'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 70, r: 20, t: 14, b: 32 }, r = R; frame(ctx, W, H, pad);
  const pts = R.sweep || []; if (!pts.length) return; const best0 = pts.reduce((a, b) => b.mass < a.mass ? b : a), mmax = Math.min(Math.max(...pts.map(p => p.mass)), 2.5 * Math.max(best0.mass, r.massTotal)) / 1000 * 1.08, Dmax = 90, x = D => pad.l + (W - pad.l - pad.r) * D / Dmax, y = m => H - pad.b - (H - pad.t - pad.b) * m / mmax;
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillStyle = css('--muted'); ctx.textAlign = 'center';
  for (let D = 0; D <= Dmax; D += 10) { ctx.strokeStyle = css('--grid'); ctx.beginPath(); ctx.moveTo(x(D), pad.t); ctx.lineTo(x(D), H - pad.b); ctx.stroke(); ctx.fillText(D + '', x(D), H - pad.b + 14); }
  const stepM = mmax > 400 ? 100 : mmax > 150 ? 50 : mmax > 60 ? 20 : mmax > 25 ? 10 : 5; ctx.textAlign = 'right'; for (let m = 0; m <= mmax; m += stepM) { ctx.strokeStyle = css('--grid'); ctx.beginPath(); ctx.moveTo(pad.l, y(m)); ctx.lineTo(W - pad.r, y(m)); ctx.stroke(); ctx.fillText(m + ' t', pad.l - 6, y(m) + 4); }
  ctx.textAlign = 'center'; ctx.fillText(`diameter, m · working capacity ${fmt(r.Vwork, 0)} m³ held constant, same freeboard, course height, material and roof`, (pad.l + W - pad.r) / 2, H - 4);
  ctx.fillStyle = css('--alarm'); ctx.globalAlpha = 0.08; ctx.fillRect(x(61), pad.t, x(Dmax) - x(61), H - pad.t - pad.b); ctx.globalAlpha = 1; ctx.fillStyle = css('--alarm'); ctx.fillText('D > 61 m: variable-design-point method', (x(61) + x(Dmax)) / 2, H - pad.b - 6);
  const line = (key, col, wd) => { ctx.strokeStyle = col; ctx.lineWidth = wd; ctx.beginPath(); let pen = false; pts.forEach(p => { const X = x(p.D), Y = y(p[key] / 1000); if (p[key] / 1000 > mmax) { pen = false; return; } if (!pen) ctx.moveTo(X, Y); else ctx.lineTo(X, Y); pen = true; }); ctx.stroke(); };
  line('mass', css('--water'), 2.5); line('shell', css('--steel'), 1.5);
  pts.forEach(p => { if (p.mass / 1000 > mmax) return; if (p.anchored) { ctx.fillStyle = css('--alarm'); ctx.fillRect(x(p.D) - 1.5, y(p.mass / 1000) - 8, 3, 3); } if (p.gird) { ctx.fillStyle = css('--brass'); ctx.fillRect(x(p.D) - 1.5, y(p.mass / 1000) + 5, 3, 3); } });
  const best = pts.reduce((a, b) => b.mass < a.mass ? b : a); ctx.fillStyle = css('--good'); ctx.beginPath(); ctx.arc(x(best.D), y(best.mass / 1000), 5, 0, 7); ctx.fill(); ctx.textAlign = 'left'; ctx.fillStyle = css('--ink'); ctx.fillText(`least steel: D ${best.D} m, H ${fmt(best.H, 1)} m, ${fmt(best.mass / 1000, 1)} t`, x(best.D) + 8, y(best.mass / 1000) - 6);
  ctx.fillStyle = css('--brass'); ctx.beginPath(); ctx.arc(x(r.D), y(r.massTotal / 1000), 6, 0, 7); ctx.fill(); ctx.fillStyle = css('--ink'); if (Math.abs(best.D - r.D) < 0.5) ctx.fillText('(this tank is at the minimum)', x(r.D) + 8, y(r.massTotal / 1000) + 14); else ctx.fillText(`this tank ${fmt(r.massTotal / 1000, 1)} t`, x(r.D) + 8, y(r.massTotal / 1000) + 14);
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const s = state, r = R, c1 = r.courses[0], live = liveStress(r);
  $('#roCap').textContent = fmt(r.Vwork, 0); $('#roNom').textContent = fmt(r.Vnom, 0); $('#roKg').textContent = fmt(r.kgm3, 1);
  $('#roT1').textContent = c1.t; $('#roT1req').textContent = fmt(c1.treq, 2); $('#roT1gov').textContent = c1.gov; $('#roT1Box').classList.toggle('alarm', !c1.ok);
  $('#roSched').textContent = r.courses.map(c => c.t).join(' / '); $('#roSchedBox').classList.toggle('alarm', !r.allOk); $('#roSchedNote').textContent = r.allOk ? `${r.n} courses, bottom to top · minimum ${r.tmin} mm for D ${fmt(r.D, 0)} m` : 'a course fails a check — see the chart';
  $('#roMassS').textContent = fmt(r.massShell / 1000, 1); $('#roMassT').textContent = fmt(r.massTotal / 1000, 1); $('#roMassB').textContent = fmt(r.massBottom / 1000, 1); $('#roMassR').textContent = fmt(r.roofMass / 1000, 1);
  $('#roBot').textContent = r.annular ? `${r.ta} mm annular` : `${r.tb} mm lap`; $('#roBotNote').textContent = r.annular ? `butt-welded ring ${fmt(r.La, 0)} mm wide · bottom ${r.tb} mm · first-course stress ${fmt(r.s1, 0)} MPa` : `sketch plates allowed · first-course stress ${fmt(r.s1, 0)} MPa ≤ 160 / 171`; $('#roBotBox').classList.toggle('warnv', r.annular); $('#roBotBox').classList.toggle('alarm', r.annOut);
  $('#roRoof').textContent = r.roof === 'open' ? 'open' : fmt(r.tr, 1) + ' mm'; $('#roRoofNote').textContent = r.roof === 'open' ? `top girder Z ${fmt(r.Ztop, 0)} cm³` : r.roofOk ? `${r.roof === 'dome' ? 'dome radius ' + fmt(r.rr, 1) + ' m' : 'slope ' + fmt(s.slope, 1) + '°'} · load ${fmt(r.T, 2)} kPa · ${r.roof === 'cone' ? r.roofNote : 'ring ' + fmt(r.ringF, 0) + ' kN, area ' + fmt(r.ringA, 0) + ' mm²'}` : r.roofNote; $('#roRoofBox').classList.toggle('alarm', !r.roofOk);
  $('#roGird').textContent = r.nGird; $('#roGirdNote').textContent = `H1 ${fmt(r.H1, 1)} m · transformed shell ${fmt(r.Htr, 1)} m${r.nGird ? ' · Z ' + fmt(r.Zint, 0) + ' cm³' : ''}`; $('#roGirdBox').classList.toggle('warnv', r.nGird > 0);
  $('#roAnch').textContent = r.anchored ? 'required' : 'not needed'; $('#roAnchNote').textContent = `Mw ${fmt(r.Mw / 1000, 1)} MN·m · MDL ${fmt(r.MDL / 1000, 1)} · MF ${fmt(r.MF / 1000, 1)} · ${r.crit1 ? 'empty ok' : 'empty FAILS'}, ${r.crit2 ? 'full ok' : 'full FAILS'}`; $('#roAnchBox').classList.toggle('alarm', r.anchored);
  const lim = s.fluid === 'water' ? r.m.St : r.m.Sd, l1 = live[0]; $('#roLive').textContent = s.level > 0.01 && Number.isFinite(l1) ? fmt(l1, 0) : '—'; $('#roLiveNote').textContent = `level ${fmt(s.level, 2)} m of ${fmt(r.H, 2)} · ${s.fluid === 'water' ? 'hydrotest against St ' + r.m.St : 'product against Sd ' + r.m.Sd} MPa`; $('#roLiveBox').classList.toggle('alarm', l1 > lim); $('#roLiveBox').classList.toggle('warnv', l1 <= lim && l1 > 0.95 * lim);
  const sel = r.courses[s.sel]; $('#roSel').textContent = sel ? `C${sel.i + 1} · ${sel.t} mm` : 'click a course'; $('#roSelNote').textContent = sel ? `required ${fmt(sel.treq, 2)} (product ${fmt(sel.td, 2)}, test ${fmt(sel.tt, 2)}, min ${r.tmin}) · ${sel.over ? 'manual' : 'auto'}${sel.ok ? '' : ' · FAILS: ' + [!sel.okProd && 'product stress', !sel.okTest && 'test stress', !sel.okMin && 'below minimum', !sel.okMax && 'over 45 mm', !sel.okOrder && 'thinner than course above'].filter(Boolean).join(', ')}` : 'then Thicker / Thinner / Auto'; $('#roSelBox').classList.toggle('alarm', !!sel && !sel.ok);
  const ps = $('#pillState'); const bad = !r.allOk || !r.roofOk || r.annOut || !r.onefoot; ps.textContent = !r.onefoot ? '1-foot method not permitted (D > 61 m)' : !r.allOk ? 'Shell course fails' : !r.roofOk ? 'Roof not feasible' : r.anchored ? 'Design ok · anchors required' : 'Design ok'; ps.className = 'pill ' + (bad ? 'warn' : r.anchored || r.nGird ? 'amber' : 'ok');
  $('#pillTank').textContent = `D ${fmt(r.D, 1)} m · H ${fmt(r.H, 1)} m · ${r.m.name} · ${fmt(r.massTotal / 1000, 0)} t`;
  $('#pillFill').textContent = s.mode === 'fill' ? `Filling ${s.fluid} ${fmt(s.level, 1)} m` : s.mode === 'drain' ? `Draining ${fmt(s.level, 1)} m` : s.level > 0.01 ? `${s.fluid} at ${fmt(s.level, 1)} m` : 'Empty'; $('#pillFill').className = 'pill ' + (s.mode !== 'idle' ? 'on run' : '');
  $('#pillOver').classList.toggle('hidden', !(l1 > lim)); $('#pillSpill').classList.toggle('hidden', !s.spill); $('#pillVDP').classList.toggle('hidden', r.onefoot); $('#pillGird').classList.toggle('hidden', !r.nGird); $('#pillAnch').classList.toggle('hidden', !r.anchored);
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start';
  $('#ctlSlope').classList.toggle('off', s.roof === 'open' || s.roof === 'dome'); $('#ctlRr').classList.toggle('off', s.roof !== 'dome'); $('#ctlLoads').classList.toggle('off', s.roof === 'open');
  $('#HOut').textContent = fmt(r.H, 1) + ' m'; $('#H').max = r.Hs;
  if (!r.allOk) s.flags.badCourse = true; if (s.flags.badCourse && r.allOk && Object.keys(s.overrides).length) s.flags.fixed = true; if (!r.onefoot) s.flags.vdp = true; if (s.sel >= 0) s.flags.select = true;
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, D: +r.D.toFixed(1), Hs: +r.Hs.toFixed(2), H: +r.H.toFixed(2), G: +r.G.toFixed(2), mat: s.mat, CA: r.CA, V: r.V, roof: r.roof, n: r.n, sched: r.courses.map(c => c.t).join('/'), gov1: r.courses[0].gov, t1: r.courses[0].t, s1p: +r.courses[0].sProd.toFixed(0), s1t: +r.courses[0].sTest.toFixed(0), ok: r.allOk ? 'ok' : 'FAIL', shell: +(r.massShell / 1000).toFixed(1), total: +(r.massTotal / 1000).toFixed(1), kgm3: +r.kgm3.toFixed(1), cap: +r.Vwork.toFixed(0), tr: r.roof === 'open' ? '' : +r.tr.toFixed(1), roofOk: r.roofOk ? 'ok' : 'no', gird: r.nGird, anch: r.anchored ? 'yes' : 'no', ann: r.annular ? r.ta : 'lap', level: +s.level.toFixed(2), fluid: s.fluid }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { const K = ['i', 'tag', 'D', 'Hs', 'H', 'G', 'mat', 'CA', 'V', 'roof', 'n', 'sched', 'gov1', 's1p', 's1t', 'ok', 'shell', 'total', 'kgm3', 'cap', 'tr', 'roofOk', 'gird', 'anch', 'ann', 'level', 'fluid']; $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${K.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,D_m,shell_height_m,design_level_m,G,material,CA_mm,wind_kmh,roof,courses,schedule_mm,course1_governs,course1_product_MPa,course1_test_MPa,shell_check,shell_t,total_t,kg_per_m3,capacity_m3,roof_plate_mm,roof_ok,intermediate_girders,anchors,bottom,level_m,fluid\n' + state.log.map(r => K.map(k => r[k]).join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'baseline', t: 'Read the baseline', d: 'Record the 30 m tank as it opens. Read the plate schedule from the bottom course up, the total steel and the kg per m³ of capacity.', ok: () => state.log.length > 0 },
  { id: 'select', t: 'The one-foot rule', d: 'Click the bottom course and the second course. Compare the required thickness with the height of liquid above each course: the design point sits 0.3 m above the bottom of the course, where the course above still helps.', ok: () => state.flags.select },
  { id: 'govern', t: 'Who governs?', d: 'With the light product (G 0.85) the hydrotest governs the bottom course. Raise G until the product case governs and record both readings.', ok: () => state.log.some(x => x.gov1 === 'test') && state.log.some(x => x.gov1 === 'product') },
  { id: 'material', t: 'Buy a better plate', d: 'Record the same tank in A283M Gr C and again in A516M Gr 485 or A537M. The shell steel falls by 10 % or more — and the annular plate appears.', ok: () => state.log.some(a => state.log.some(b => same(a, b, ['D', 'Hs', 'H', 'G', 'CA']) && a.mat !== b.mat && Math.abs(a.shell - b.shell) / Math.max(a.shell, b.shell) >= 0.10)) },
  { id: 'ca', t: 'Corrosion allowance', d: 'Record the tank with no corrosion allowance and again with 3 mm. Note that the allowance adds to the product case only, never to the hydrotest case.', ok: () => state.log.some(a => state.log.some(b => same(a, b, ['D', 'Hs', 'H', 'G', 'mat']) && a.CA === 0 && b.CA >= 3)) },
  { id: 'optimise', t: 'Squat or tall?', d: 'Use the sweep chart: at this capacity, find the diameter that needs the least steel. Record two tanks of the same capacity (within 3 %) at least 6 m apart in diameter.', ok: () => state.log.some(a => state.log.some(b => Math.abs(a.cap - b.cap) / Math.max(a.cap, b.cap) <= 0.03 && Math.abs(a.D - b.D) >= 6)) },
  { id: 'girder', t: 'Stiffen the shell', d: 'Make the tank tall and thin, or raise the wind speed, until an intermediate wind girder is required. Record it and read where it sits on the transformed shell.', ok: () => state.log.some(x => x.gird >= 1) },
  { id: 'anchor', t: 'Blow it over', d: 'Record a small, tall, empty tank in a strong wind that needs anchors; then change the design (lower, wider, heavier roof) until it stands unanchored and record that.', ok: () => state.log.some(x => x.anch === 'yes') && state.log.some(x => x.anch === 'no') },
  { id: 'override', t: 'Break a course and fix it', d: 'Select a course and make it thinner than required, or thinner than the course above it. See the check fail. Then bring it back to a passing schedule.', ok: () => state.flags.badCourse && state.flags.fixed },
  { id: 'hydro', t: 'Hydrotest', d: 'Press Hydrotest: the tank fills with water to the design level. Watch the live stress in the bottom course approach the test allowable St and record the reading at full.', ok: () => state.flags.hydro && state.log.some(x => x.fluid === 'water' && x.level >= 0.98 * x.H) },
  { id: 'roof', t: 'Roof choice', d: 'Put a self-supporting cone on the 30 m tank at 9.5°: the plate needs more than 13 mm. Record it, then record a feasible roof (supported cone or dome) on the same diameter.', ok: () => state.log.some(a => a.roofOk === 'no' && state.log.some(b => b.D === a.D && b.roofOk === 'ok' && b.roof !== 'open')) },
  { id: 'vdp', t: 'Beyond the one-foot method', d: 'Take the diameter past 61 m and read the warning: the one-foot method is no longer permitted and the variable-design-point method (or Annex A rules for small tanks) takes over.', ok: () => state.flags.vdp },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame_ = 0, dirty = true;
function recompute() { Object.assign(R, design(state)); R.sweep = sweep(state); if (state.level > R.Hs) state.level = R.Hs; dirty = false; }
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; if (dirty) recompute(); step(dt); drawRig(); frame_++; if (frame_ % 2 === 0) { updateReadouts(); drawStress(); if (frame_ % 6 === 0) { drawThick(); drawSweep(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSel(id, v) { const e = $('#' + id); e.value = v; state[id] = v; dirty = true; }
function fillTo(target, fluid) { state.fluid = fluid; state.target = target; state.mode = target > state.level ? 'fill' : 'drain'; state.spill = false; state.run = true; }
function adjust(dir) { const s = state, c = R.courses[s.sel]; if (!c) return; const k = PLATES.indexOf(c.t); const nk = clamp(k + dir, 0, PLATES.length - 1); s.overrides[s.sel] = PLATES[nk]; dirty = true; recompute(); }
function autoSel() { if (state.sel >= 0) delete state.overrides[state.sel]; dirty = true; recompute(); }
function resetOv() { state.overrides = {}; dirty = true; recompute(); }
const PRESETS = {
  base: () => { setSel('mat', 'a283c'); setSel('roof', 'cone'); setSel('fluid', 'product'); for (const [k, v] of Object.entries({ D: 30, n: 8, hc: 1.8, G: 0.85, CA: 1.5, V: 190, slope: 3.6, rr: 1.0, Lr: 1.0, S: 0, fillRate: 1000, tscale: 300 })) setCtl(k, v); recompute(); setCtl('H', 14.1); state.overrides = {}; state.sel = -1; state.level = 0; state.mode = 'idle'; state.spill = false; state.run = true; dirty = true; recompute(); },
  material: () => { PRESETS.base(); setSel('mat', 'a516_485'); recompute(); },
  ca: () => { PRESETS.base(); setCtl('CA', 3); recompute(); },
  heavy: () => { PRESETS.base(); setCtl('G', 1.25); recompute(); },
  hydro: () => { PRESETS.base(); setCtl('tscale', 600); setSel('fluid', 'water'); recompute(); fillTo(R.H, 'water'); },
  tall: () => { PRESETS.base(); setSel('roof', 'open'); for (const [k, v] of Object.entries({ D: 12, n: 10, hc: 2.4, V: 220 })) setCtl(k, v); recompute(); setCtl('H', 23.5); recompute(); },
  wind: () => { PRESETS.base(); setSel('roof', 'open'); for (const [k, v] of Object.entries({ D: 45, n: 8, hc: 2.4, V: 230, G: 0.8 })) setCtl(k, v); recompute(); setCtl('H', 18.5); recompute(); },
  anchor: () => { PRESETS.base(); setSel('roof', 'selfcone'); for (const [k, v] of Object.entries({ D: 6, n: 6, hc: 2.4, V: 220, slope: 9.5 })) setCtl(k, v); recompute(); setCtl('H', 14); recompute(); },
  selfroof: () => { PRESETS.base(); setSel('roof', 'selfcone'); setCtl('slope', 9.5); recompute(); },
  dome: () => { PRESETS.base(); setSel('roof', 'dome'); for (const [k, v] of Object.entries({ D: 30, n: 6, hc: 2.4, rr: 0.8 })) setCtl(k, v); recompute(); setCtl('H', 14); recompute(); },
  big: () => { PRESETS.base(); setSel('mat', 'a537_1'); for (const [k, v] of Object.entries({ D: 70, n: 8, hc: 2.4 })) setCtl(k, v); recompute(); setCtl('H', 18.9); recompute(); },
  small: () => { PRESETS.base(); setSel('mat', 'a36'); for (const [k, v] of Object.entries({ D: 10, n: 4, hc: 2.0, G: 1.0, CA: 0 })) setCtl(k, v); recompute(); setCtl('H', 7.7); recompute(); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); dirty = true; }; e.addEventListener('input', set); set(); };
  bindR('D', v => v.toFixed(1) + ' m'); bindR('n', v => v + ' courses'); bindR('hc', v => v.toFixed(2) + ' m'); bindR('H', v => v.toFixed(1) + ' m'); bindR('G', v => v.toFixed(2)); bindR('CA', v => v.toFixed(1) + ' mm'); bindR('V', v => v + ' km/h'); bindR('slope', v => v.toFixed(1) + '° (1:' + (1 / Math.tan(v * Math.PI / 180)).toFixed(1) + ')'); bindR('rr', v => v.toFixed(2) + ' × D'); bindR('Lr', v => v.toFixed(2) + ' kPa'); bindR('S', v => v.toFixed(2) + ' kPa'); bindR('fillRate', v => v + ' m³/h'); bindR('tscale', v => '×' + v);
  for (const id of ['mat', 'roof', 'fluid']) $('#' + id).addEventListener('change', e => { state[id] = e.target.value; dirty = true; });
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnFill').onclick = () => fillTo(R.H, state.fluid === 'water' ? 'water' : 'product'); $('#btnHydro').onclick = () => { setSel('fluid', 'water'); fillTo(R.H, 'water'); }; $('#btnDrain').onclick = () => fillTo(0, state.fluid); $('#btnRecord').onclick = () => record('');
  $('#btnThick').onclick = () => adjust(1); $('#btnThin').onclick = () => adjust(-1); $('#btnAuto').onclick = autoSel; $('#btnReset').onclick = resetOv;
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tk_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'F') $('#btnFill').click(); if (k === 'W') $('#btnHydro').click(); if (k === 'E') $('#btnDrain').click(); if (k === 'R') record(''); if (e.key === 'ArrowUp' && state.sel >= 0) { e.preventDefault(); adjust(1); } if (e.key === 'ArrowDown' && state.sel >= 0) { e.preventDefault(); adjust(-1); } if (k === 'A') autoSel(); if (e.key === 'Escape') { $('#help').classList.add('hidden'); state.sel = -1; } });
}

/* ------------------------------------------------------------------ dashboard API */
window.TankDesign = {
  getState: () => ({ inputs: { D: state.D, n: state.n, hc: state.hc, H: state.H, G: state.G, CA: state.CA, mat: state.mat, V: state.V, roof: state.roof, slope: state.slope, rr: state.rr, Lr: state.Lr, S: state.S, fluid: state.fluid, fillRate: state.fillRate, tscale: state.tscale, run: state.run, overrides: Object.assign({}, state.overrides) },
    results: { Hs: R.Hs, schedule: R.courses ? R.courses.map(c => c.t) : [], required: R.courses ? R.courses.map(c => +c.treq.toFixed(2)) : [], governs: R.courses ? R.courses.map(c => c.gov) : [], allOk: R.allOk, tmin: R.tmin, onefoot: R.onefoot, massShell: R.massShell, massTotal: R.massTotal, kgm3: R.kgm3, Vwork: R.Vwork, annular: R.annular, ta: R.ta, tb: R.tb, tr: R.tr, roofOk: R.roofOk, ringF: R.ringF, H1: R.H1, Htr: R.Htr, nGird: R.nGird, girders: R.girders, Mw: R.Mw, MDL: R.MDL, MF: R.MF, anchored: R.anchored, level: state.level, liveStress1: R.courses ? liveStress(R)[0] : null },
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }),
  setState: inp => { for (const k of ['mat', 'roof', 'fluid']) if (inp[k]) setSel(k, inp[k]); for (const k of ['D', 'n', 'hc', 'G', 'CA', 'V', 'slope', 'rr', 'Lr', 'S', 'fillRate', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.H !== undefined) { recompute(); setCtl('H', inp.H); } if (inp.overrides) state.overrides = Object.assign({}, inp.overrides); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.fill) fillTo(R.H, state.fluid); if (inp.hydrotest) { setSel('fluid', 'water'); fillTo(R.H, 'water'); } if (inp.drain) fillTo(0, state.fluid); if (inp.level !== undefined) state.level = clamp(+inp.level, 0, R.Hs || 0); if (inp.run !== undefined) state.run = !!inp.run; dirty = true; recompute(); },
  meta: { id: 'api650-tank', title: 'API 650 Tank Design Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.TankDesign.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'api650-tank', state: window.TankDesign.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('tk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); recompute(); step(0); drawRig(); updateReadouts(); drawThick(); drawStress(); drawSweep();
  requestAnimationFrame(tick);
}
build();
