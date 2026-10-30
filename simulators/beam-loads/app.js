/* Loads on Beams Demonstrator — Euler–Bernoulli beam finite elements (200 elements, banded solve) give the
   displacements and reactions for any support arrangement; shear and moment are then built by statics from the
   reactions so the diagrams are exact. Sections are parametric. Standalone, offline. Data illustrative. */
'use strict';

const NEL = 200, G = 9.81;
const MATS = { steel: { E: 210, fy: 355, rho: 7850, name: 'Steel S355' }, alu: { E: 70, fy: 240, rho: 2700, name: 'Aluminium 6082' }, timber: { E: 11, fy: 24, rho: 500, name: 'Timber C24' } };
const SECS = {
  rect: { name: 'Rectangle', dims: [['Width b', 20, 400, 100], ['Depth h', 20, 600, 200], null], props: (b, h) => { const I = b * h ** 3 / 12, A = b * h; return { I, Z: I / (h / 2), A, h, b }; } },
  isec: { name: 'I-section', dims: [['Flange width b', 40, 400, 100], ['Depth h', 60, 600, 200], ['Flange thickness t', 3, 30, 8]], props: (b, h, t) => { const tw = Math.max(3, t * 0.7), hw = h - 2 * t, I = b * h ** 3 / 12 - (b - tw) * hw ** 3 / 12, A = 2 * b * t + hw * tw; return { I, Z: I / (h / 2), A, h, b, t, tw }; } },
  rhs: { name: 'Rect. hollow', dims: [['Width b', 20, 400, 100], ['Depth h', 20, 600, 200], ['Wall t', 2, 25, 6]], props: (b, h, t) => { const I = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12, A = b * h - (b - 2 * t) * (h - 2 * t); return { I, Z: I / (h / 2), A, h, b, t }; } },
  circ: { name: 'Round bar', dims: [['Diameter d', 20, 400, 150], null, null], props: d => { const I = Math.PI * d ** 4 / 64, A = Math.PI * d * d / 4; return { I, Z: I / (d / 2), A, h: d, b: d }; } },
  chs: { name: 'Circ. hollow', dims: [['Diameter d', 20, 400, 150], null, ['Wall t', 2, 25, 6]], props: (d, _, t) => { const di = d - 2 * t, I = Math.PI * (d ** 4 - di ** 4) / 64, A = Math.PI * (d * d - di * di) / 4; return { I, Z: I / (d / 2), A, h: d, b: d, t }; } },
};
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';

/* ------------------------------------------------------------------ state */
const state = { L: 6, stype: 'ss', xa: 0, xb: 6, selfw: false, exag: 0, sec: 'isec', d1: 100, d2: 200, d3: 8, mat: 'steel', dlim: 250, loads: [], log: [], flags: {}, nextId: 1, drag: null };
const R = {};

/* ------------------------------------------------------------------ section and material */
function section() { const s = SECS[state.sec], p = s.props(state.d1, state.d2, state.d3); const m = MATS[state.mat]; return { ...p, I_m4: p.I * 1e-12, Z_m3: p.Z * 1e-9, EI: m.E * 1e6 * p.I * 1e-12, E: m.E, fy: m.fy, sw: p.A * 1e-6 * m.rho * G / 1000, name: s.name }; }   // EI in kN·m², sw kN/m

/* ------------------------------------------------------------------ finite elements */
function hermite(xi, l) { const x2 = xi * xi, x3 = x2 * xi; return [1 - 3 * x2 + 2 * x3, l * (xi - 2 * x2 + x3), 3 * x2 - 2 * x3, l * (-x2 + x3)]; }
function solve() {
  const L = state.L, l = L / NEL, n = NEL + 1, ndof = 2 * n, sec = section(), EI = sec.EI;
  const F = new Float64Array(ndof);
  const nodeAt = x => clamp(Math.round(x / l), 0, NEL);
  const loads = state.loads.slice(); if (state.selfw) loads.push({ type: 'w', w: sec.sw, a: 0, b: L, self: true });
  for (const ld of loads) {
    if (ld.type === 'p') { const e = clamp(Math.floor(ld.x / l), 0, NEL - 1), xi = ld.x / l - e, N = hermite(xi, l); for (let k = 0; k < 4; k++) F[2 * e + k] += -ld.P * N[k]; }
    else if (ld.type === 'm') { const e = clamp(Math.floor(ld.x / l), 0, NEL - 1), xi = ld.x / l - e, dN = [(-6 * xi + 6 * xi * xi) / l, 1 - 4 * xi + 3 * xi * xi, (6 * xi - 6 * xi * xi) / l, -2 * xi + 3 * xi * xi]; for (let k = 0; k < 4; k++) F[2 * e + k] += ld.M * dN[k]; }
    else if (ld.type === 'w') { const a = Math.min(ld.a, ld.b), b = Math.max(ld.a, ld.b); for (let e = 0; e < NEL; e++) { const x0 = e * l, x1 = x0 + l, c0 = Math.max(a, x0), c1 = Math.min(b, x1); if (c1 <= c0) continue; const gp = [-0.7745966692, 0, 0.7745966692], gw = [5 / 9, 8 / 9, 5 / 9]; for (let q = 0; q < 3; q++) { const x = (c0 + c1) / 2 + gp[q] * (c1 - c0) / 2, N = hermite((x - x0) / l, l); for (let k = 0; k < 4; k++) F[2 * e + k] += -ld.w * N[k] * gw[q] * (c1 - c0) / 2; } } }
  }
  // constraints
  const fixed = new Uint8Array(ndof); const supports = [];
  const nA = nodeAt(state.xa), nB = nodeAt(state.xb);
  if (state.stype === 'ss') { supports.push({ n: nA, name: 'A', kind: 'pin' }, { n: nB, name: 'B', kind: 'roller' }); }
  if (state.stype === 'cant') { supports.push({ n: nA, name: 'A', kind: 'fixed' }); }
  if (state.stype === 'prop') { supports.push({ n: nA, name: 'A', kind: 'fixed' }, { n: nB, name: 'B', kind: 'roller' }); }
  if (state.stype === 'ff') { supports.push({ n: nA, name: 'A', kind: 'fixed' }, { n: nB, name: 'B', kind: 'fixed' }); }
  for (const s of supports) { fixed[2 * s.n] = 1; if (s.kind === 'fixed') fixed[2 * s.n + 1] = 1; }
  const mech = state.stype === 'ss' && nA === nB;
  const map = new Int32Array(ndof).fill(-1); let nf = 0; for (let i = 0; i < ndof; i++) if (!fixed[i]) map[i] = nf++;
  const bw = 4, K = Array.from({ length: nf }, () => new Float64Array(bw + 1));   // K[i][j - i] for j ≥ i (upper band)
  const ke = (a => [[12, 6 * l, -12, 6 * l], [6 * l, 4 * l * l, -6 * l, 2 * l * l], [-12, -6 * l, 12, -6 * l], [6 * l, 2 * l * l, -6 * l, 4 * l * l]].map(r => r.map(v => v * a)))(EI / l ** 3);
  for (let e = 0; e < NEL; e++) for (let a = 0; a < 4; a++) { const gi = map[2 * e + a]; if (gi < 0) continue; for (let b = 0; b < 4; b++) { const gj = map[2 * e + b]; if (gj < gi) continue; K[gi][gj - gi] += ke[a][b]; } }
  const f = new Float64Array(nf); for (let i = 0; i < ndof; i++) if (map[i] >= 0) f[map[i]] = F[i];
  // banded Cholesky-free LDLᵀ (symmetric, upper band)
  let ok = !mech;
  if (ok) for (let i = 0; i < nf; i++) { const p = K[i][0]; if (p <= 1e-12) { ok = false; break; } for (let j = 1; j <= bw && i + j < nf; j++) { const r = K[i][j] / p; if (r === 0) continue; for (let k = j; k <= bw && i + k < nf; k++) K[i + j][k - j] -= r * K[i][k]; f[i + j] -= r * f[i]; } }
  const u = new Float64Array(ndof);
  if (ok) { const y = new Float64Array(nf); for (let i = nf - 1; i >= 0; i--) { let s = f[i]; for (let j = 1; j <= bw && i + j < nf; j++) s -= K[i][j] * y[i + j]; y[i] = s / K[i][0]; } for (let i = 0; i < ndof; i++) if (map[i] >= 0) u[i] = y[map[i]]; }
  // reactions at constrained dofs from element forces
  const react = new Float64Array(ndof);
  for (let e = 0; e < NEL; e++) for (let a = 0; a < 4; a++) { const gi = 2 * e + a; if (!fixed[gi]) continue; let s = 0; for (let b = 0; b < 4; b++) s += ke[a][b] * u[2 * e + b]; react[gi] += s; }
  for (let i = 0; i < ndof; i++) if (fixed[i]) react[i] -= F[i];
  for (const s of supports) { s.x = s.n * l; s.R = ok ? react[2 * s.n] : NaN; s.M = s.kind === 'fixed' && ok ? react[2 * s.n + 1] : 0; s.lift = s.kind !== 'fixed' && s.R < -1e-6; }
  // diagrams by statics (upward forces positive, sagging positive)
  const NS = 600, xs = new Float64Array(NS + 1), V = new Float64Array(NS + 1), M = new Float64Array(NS + 1), D = new Float64Array(NS + 1);
  for (let i = 0; i <= NS; i++) {
    const x = i * L / NS; xs[i] = x; let v = 0, m = 0;
    for (const s of supports) if (s.x < x - 1e-9 || (s.x <= 1e-9 && x > 0)) { v += s.R; m += s.R * (x - s.x) - s.M; }   // anticlockwise couples on the left segment reduce the sagging moment
    for (const ld of loads) {
      if (ld.type === 'p' && ld.x < x - 1e-9) { v -= ld.P; m -= ld.P * (x - ld.x); }
      else if (ld.type === 'm' && ld.x < x - 1e-9) m -= ld.M;
      else if (ld.type === 'w') { const a = Math.min(ld.a, ld.b), b = Math.min(Math.max(ld.a, ld.b), x); if (b > a) { const len = b - a; v -= ld.w * len; m -= ld.w * len * (x - (a + b) / 2); } }
    }
    V[i] = v; M[i] = m; const e = clamp(Math.floor(x / l), 0, NEL - 1), xi = x / l - e, N = hermite(xi, l); D[i] = ok ? (N[0] * u[2 * e] + N[1] * u[2 * e + 1] + N[2] * u[2 * e + 2] + N[3] * u[2 * e + 3]) * 1000 : 0;   // mm, up +
  }
  const imax = arr => { let k = 0; for (let i = 1; i < arr.length; i++) if (arr[i] > arr[k]) k = i; return k; }, imin = arr => { let k = 0; for (let i = 1; i < arr.length; i++) if (arr[i] < arr[k]) k = i; return k; }, iabs = arr => { let k = 0; for (let i = 1; i < arr.length; i++) if (Math.abs(arr[i]) > Math.abs(arr[k])) k = i; return k; };
  const kV = iabs(V), kMx = imax(M), kMn = imin(M), kD = iabs(D), kMa = Math.abs(M[kMx]) >= Math.abs(M[kMn]) ? kMx : kMn;
  let cf = 0; for (let i = 1; i <= NS; i++) if (M[i - 1] * M[i] < 0 && Math.abs(M[i]) > 1e-6) cf++;
  const totalP = loads.reduce((s, ld) => s + (ld.type === 'p' ? ld.P : ld.type === 'w' ? ld.w * Math.abs(ld.b - ld.a) : 0), 0);
  const sigma = Math.abs(M[kMa]) / sec.Z_m3 / 1000, dmax = Math.abs(D[kD]), ratio = dmax > 1e-9 ? L * 1000 / dmax : Infinity;
  return { ok, mech, supports, xs, V, M, D, kV, kMx, kMn, kMa, kD, cf, totalP, sec, sigma, dmax, ratio, xD: xs[kD], xM: xs[kMa], Mmax: M[kMx], Mmin: M[kMn], Vmax: V[kV], util: sigma / (sec.fy / 1.5), utilD: dmax / (L * 1000 / state.dlim), yield: sigma > sec.fy, lift: supports.some(s => s.lift), sw: state.selfw ? sec.sw * L : 0, loads };
}

/* ------------------------------------------------------------------ drawing */
const NSVG = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NSVG, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
const BX0 = 70, BX1 = 700, BY = 150;
const X = x => BX0 + x / state.L * (BX1 - BX0), xOf = px => clamp((px - BX0) / (BX1 - BX0) * state.L, 0, state.L);
function drawBeam() {
  const svg = $('#beam'); svg.innerHTML = ''; const r = R;
  // deflected shape
  const scale = state.exag > 0 ? state.exag / 10 : (r.dmax > 1e-6 ? 60 / r.dmax : 0);   // px per mm
  let d = ''; for (let i = 0; i <= 600; i++) d += (i ? 'L' : 'M') + X(r.xs[i]).toFixed(1) + ',' + (BY - r.D[i] * scale).toFixed(1) + ' '; el('path', { d, class: 'beam-def' + (r.yield ? ' yield' : '') }, svg);
  el('line', { x1: X(0), y1: BY, x2: X(state.L), y2: BY, class: 'beam-line' }, svg);
  // supports
  for (const s of r.supports) {
    const x = X(s.x), g = el('g', { class: 'sup' + (state.stype === 'ss' ? ' drag' : ''), 'data-sup': s.name }, svg);
    if (s.kind === 'fixed') { const side = s.x < state.L / 2 ? -1 : 1; el('rect', { x: x + (side < 0 ? -16 : 4), y: BY - 34, width: 12, height: 68, class: 'sup' }, g); for (let y = BY - 30; y < BY + 34; y += 8) el('line', { x1: x + (side < 0 ? -16 : 16), y1: y, x2: x + (side < 0 ? -26 : 26), y2: y + 8, class: 'sup-hatch' }, g); }
    else { el('polygon', { points: `${x},${BY + 7} ${x - 14},${BY + 30} ${x + 14},${BY + 30}`, class: 'sup' + (s.lift ? ' lift' : '') }, g); if (s.kind === 'roller') { el('circle', { cx: x - 8, cy: BY + 36, r: 4, class: 'sup' }, g); el('circle', { cx: x + 8, cy: BY + 36, r: 4, class: 'sup' }, g); } el('line', { x1: x - 22, y1: BY + 41, x2: x + 22, y2: BY + 41, class: 'sup-ground' }, g); for (let i = -20; i < 22; i += 8) el('line', { x1: x + i, y1: BY + 41, x2: x + i - 6, y2: BY + 48, class: 'sup-hatch' }, g); }
    // reaction arrow
    if (Number.isFinite(s.R)) { const up = s.R >= 0, y0 = BY + 62, y1 = BY + 92; el('line', { x1: x, y1: up ? y1 : y0, x2: x, y2: up ? y0 + 6 : y1 - 6, class: 'react' }, g); el('polygon', { points: up ? `${x},${y0} ${x - 5},${y0 + 9} ${x + 5},${y0 + 9}` : `${x},${y1} ${x - 5},${y1 - 9} ${x + 5},${y1 - 9}`, class: 'react-head' }, g); txt(x, BY + 106, `R${s.name} ${fmt(s.R)} kN${s.kind === 'fixed' ? ` · M ${fmt(s.M)} kNm` : ''}`, 'react-txt' + (s.lift ? ' bad' : ''), g, 'middle'); }
    txt(x, BY + 118, `${s.name} at ${fmt(s.x, 2)} m`, 'dim-txt', g);
  }
  // loads
  for (const ld of r.loads) {
    if (ld.type === 'p') { const x = X(ld.x), g = el('g', { class: 'pload-grab', 'data-load': ld.id }, svg); el('line', { x1: x, y1: BY - 62, x2: x, y2: BY - 14, class: 'pload' }, g); el('polygon', { points: `${x},${BY - 8} ${x - 6},${BY - 20} ${x + 6},${BY - 20}`, class: 'pload-head' }, g); el('rect', { x: x - 20, y: BY - 90, width: 40, height: 26, fill: 'transparent' }, g); txt(x, BY - 68, `${fmt(ld.P)} kN`, 'pload-txt', g); txt(x, BY - 80, `x ${fmt(ld.x, 2)}`, 'dim-txt', g); }
    else if (ld.type === 'w') { const a = X(Math.min(ld.a, ld.b)), b = X(Math.max(ld.a, ld.b)), hgt = 14 + 26 * clamp(ld.w / 50, 0.1, 1); el('rect', { x: a, y: BY - 8 - hgt, width: b - a, height: hgt, class: 'udl' }, svg); el('line', { x1: a, y1: BY - 8 - hgt, x2: b, y2: BY - 8 - hgt, class: 'udl-line' }, svg); for (let x = a; x <= b + 0.5; x += Math.max(12, (b - a) / 14)) { el('line', { x1: x, y1: BY - 8 - hgt, x2: x, y2: BY - 14, class: 'udl-line' }, svg); el('polygon', { points: `${x},${BY - 8} ${x - 3},${BY - 15} ${x + 3},${BY - 15}`, class: 'pload-head', style: 'fill:var(--water)' }, svg); } txt((a + b) / 2, BY - 14 - hgt, `${fmt(ld.w)} kN/m${ld.self ? ' self-weight' : ''}`, 'udl-txt', svg); }
    else if (ld.type === 'm') { const x = X(ld.x), rr = 22, cw = ld.M < 0; el('path', { d: `M${x + rr},${BY} A${rr},${rr} 0 1 ${cw ? 1 : 0} ${x},${BY - rr}`, class: 'mload' }, svg); el('circle', { cx: x, cy: BY - rr, r: 4, class: 'mload-head' }, svg); txt(x, BY - rr - 12, `${fmt(Math.abs(ld.M))} kNm ${ld.M >= 0 ? '↺' : '↻'}`, 'mload-txt', svg); }
  }
  // dimensions and section sketch
  el('line', { x1: X(0), y1: BY + 128, x2: X(state.L), y2: BY + 128, class: 'dim' }, svg); txt((X(0) + X(state.L)) / 2, BY + 140, `L = ${fmt(state.L)} m`, 'dim-txt', svg);
  const sx = 790, sy = 80, sec = r.sec, sc = 100 / Math.max(sec.h, sec.b); el('rect', { x: 725, y: 12, width: 165, height: 200, class: 'sec-box' }, svg); txt(730, 26, `${sec.name} · ${MATS[state.mat].name}`, 'sec-txt', svg);
  const gS = el('g', { transform: `translate(${sx},${sy + 60})` }, svg), hb = sec.h * sc / 2, wb = sec.b * sc / 2;
  if (state.sec === 'rect') el('rect', { x: -wb, y: -hb, width: 2 * wb, height: 2 * hb, class: 'sec' }, gS);
  else if (state.sec === 'isec') { const tf = sec.t * sc, tw = sec.tw * sc / 2; el('path', { d: `M${-wb},${-hb} H${wb} V${-hb + tf} H${tw} V${hb - tf} H${wb} V${hb} H${-wb} V${hb - tf} H${-tw} V${-hb + tf} H${-wb} Z`, class: 'sec' }, gS); }
  else if (state.sec === 'rhs') { el('rect', { x: -wb, y: -hb, width: 2 * wb, height: 2 * hb, class: 'sec' }, gS); el('rect', { x: -wb + sec.t * sc, y: -hb + sec.t * sc, width: 2 * wb - 2 * sec.t * sc, height: 2 * hb - 2 * sec.t * sc, fill: 'var(--panel2)', stroke: 'var(--steel)' }, gS); }
  else { el('circle', { r: hb, class: 'sec' }, gS); if (state.sec === 'chs') el('circle', { r: hb - sec.t * sc, fill: 'var(--panel2)', stroke: 'var(--steel)' }, gS); }
  txt(730, 178, `I ${fmt(sec.I / 1e4, 0)} cm⁴ · Z ${fmt(sec.Z / 1e3, 0)} cm³`, 'sec-txt', svg); txt(730, 192, `A ${fmt(sec.A / 100, 1)} cm² · ${fmt(sec.sw * 100 / G, 1)} kg/m`, 'sec-txt', svg); txt(730, 206, `E ${sec.E} GPa · fy ${sec.fy} MPa`, 'sec-txt', svg);
  if (r.dmax > 1e-6) { el('line', { x1: X(r.xD), y1: BY, x2: X(r.xD), y2: BY - r.D[r.kD] * scale, class: 'marker' }, svg); txt(X(r.xD) + 4, BY - r.D[r.kD] * scale / 2, `δ ${fmt(r.dmax, 2)} mm`, 'dim-txt', svg); }
  txt(BX0, 320, `deflection drawn ×${fmt(scale * 1000 / ((BX1 - BX0) / state.L), 0)} · sagging moment ${fmt(r.Mmax, 1)} kNm at ${fmt(r.xs[r.kMx], 2)} m · hogging ${fmt(r.Mmin, 1)} kNm at ${fmt(r.xs[r.kMn], 2)} m`, 'dim-txt', svg).setAttribute('text-anchor', 'start');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawCharts() {
  const cv = $('#charts'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const r = R, L = 70, Rr = 20, pw = W - L - Rr;
  const strips = [[14, 200, 'Shear force V, kN', r.V, '--alarm', 1], [216, 402, 'Bending moment M, kNm (sagging up)', r.M, '--water', 1], [418, 604, 'Deflection, mm', r.D, '--good', 1]];
  const Xp = x => L + x / state.L * pw;
  for (const [t, b, name, arr, color, sgn] of strips) {
    const amax = Math.max(1e-9, ...Array.from(arr, Math.abs)), mid = (t + b) / 2, half = (b - t) / 2 - 18, Y = v => mid - (v * sgn) / amax * half;
    ctx.fillStyle = col('--panel2'); ctx.fillRect(L, t, pw, b - t); ctx.strokeStyle = col('--grid'); ctx.lineWidth = 1;
    for (let x = 0; x <= state.L + 1e-9; x += state.L > 8 ? 1 : 0.5) { ctx.beginPath(); ctx.moveTo(Xp(x), t); ctx.lineTo(Xp(x), b); ctx.stroke(); }
    ctx.strokeStyle = col('--steel'); ctx.beginPath(); ctx.moveTo(L, mid); ctx.lineTo(L + pw, mid); ctx.stroke();
    ctx.beginPath(); ctx.fillStyle = col(color); ctx.globalAlpha = 0.2; ctx.moveTo(Xp(0), mid); for (let i = 0; i <= 600; i++) ctx.lineTo(Xp(r.xs[i]), Y(arr[i])); ctx.lineTo(Xp(state.L), mid); ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.strokeStyle = col(color); ctx.lineWidth = 2; for (let i = 0; i <= 600; i++) i ? ctx.lineTo(Xp(r.xs[i]), Y(arr[i])) : ctx.moveTo(Xp(r.xs[i]), Y(arr[i])); ctx.stroke();
    for (const s of r.supports) { ctx.strokeStyle = col('--muted'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(Xp(s.x), t); ctx.lineTo(Xp(s.x), b); ctx.stroke(); ctx.setLineDash([]); }
    ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.textAlign = 'left'; ctx.fillText(name, L + 6, t + 14);
    ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.fillText(fmt(amax * sgn, 1), L - 5, Y(amax) + 4); ctx.fillText(fmt(-amax * sgn, 1), L - 5, Y(-amax) + 4); ctx.fillText('0', L - 5, mid + 4);
    const kmax = arr === r.V ? r.kV : arr === r.M ? r.kMa : r.kD, kx = Xp(r.xs[kmax]), ky = Y(arr[kmax]); ctx.fillStyle = col(color); ctx.beginPath(); ctx.arc(kx, ky, 4, 0, 7); ctx.fill(); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = kx > L + pw * 0.7 ? 'right' : 'left'; ctx.fillText(`${fmt(arr[kmax], arr === r.D ? 2 : 1)} at ${fmt(r.xs[kmax], 2)} m`, kx + (kx > L + pw * 0.7 ? -8 : 8), ky + (arr[kmax] * sgn >= 0 ? -6 : 14));
    if (arr === r.M && Math.abs(r.Mmin) > 1e-6 && r.kMn !== r.kMa) { const kx2 = Xp(r.xs[r.kMn]), ky2 = Y(r.M[r.kMn]); ctx.beginPath(); ctx.arc(kx2, ky2, 3, 0, 7); ctx.fill(); ctx.textAlign = kx2 > L + pw * 0.7 ? 'right' : 'left'; ctx.fillText(`${fmt(r.Mmin, 1)} at ${fmt(r.xs[r.kMn], 2)} m`, kx2 + (kx2 > L + pw * 0.7 ? -8 : 8), ky2 + 14); }
    if (arr === r.M && Math.abs(r.Mmax) > 1e-6 && r.kMx !== r.kMa) { const kx2 = Xp(r.xs[r.kMx]), ky2 = Y(r.M[r.kMx]); ctx.beginPath(); ctx.arc(kx2, ky2, 3, 0, 7); ctx.fill(); ctx.textAlign = kx2 > L + pw * 0.7 ? 'right' : 'left'; ctx.fillText(`${fmt(r.Mmax, 1)} at ${fmt(r.xs[r.kMx], 2)} m`, kx2 + (kx2 > L + pw * 0.7 ? -8 : 8), ky2 - 6); }
  }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; for (let x = 0; x <= state.L + 1e-9; x += state.L > 8 ? 2 : 1) ctx.fillText(fmt(x, 0) + ' m', Xp(x), 616);
}

/* ------------------------------------------------------------------ readouts, log */
function updateReadouts() {
  const r = R, A = r.supports.find(s => s.name === 'A'), B = r.supports.find(s => s.name === 'B');
  $('#roRA').textContent = A ? fmt(A.R) : '—'; $('#roMA').textContent = A && A.kind === 'fixed' ? `· M ${fmt(A.M)} kNm` : ''; $('#roRB').textContent = B ? fmt(B.R) : '—'; $('#roMB').textContent = B && B.kind === 'fixed' ? `· M ${fmt(B.M)} kNm` : (B ? '' : 'no support B');
  $('#roV').textContent = fmt(Math.abs(r.Vmax)); $('#roVx').textContent = fmt(r.xs[r.kV], 2); $('#roM').textContent = fmt(r.M[r.kMa]); $('#roMx').textContent = fmt(r.xM, 2); $('#roMmin').textContent = fmt(r.Mmin);
  $('#roS').textContent = fmt(r.sigma, 0); $('#roFy').textContent = r.sec.fy; $('#roZ').textContent = fmt(r.sec.Z / 1e3, 0); $('#roSBox').classList.toggle('alarm', r.yield); $('#roSBox').classList.toggle('warnv', !r.yield && r.util > 1);
  $('#roD').textContent = fmt(r.dmax, 2); $('#roDx').textContent = fmt(r.xD, 2); $('#roDr').textContent = Number.isFinite(r.ratio) ? fmt(r.ratio, 0) : '∞'; $('#roDBox').classList.toggle('alarm', r.utilD > 1);
  $('#roI').textContent = fmt(r.sec.I / 1e4, 0); $('#roEI').textContent = fmt(r.sec.EI, 0); $('#roWt').textContent = fmt(r.totalP); $('#roSw').textContent = fmt(r.sw); $('#roCf').textContent = r.cf; $('#roU').textContent = fmt(r.util * 100, 0); $('#roUd').textContent = fmt(r.utilD * 100, 0);
  const st = $('#pillState'); st.textContent = r.mech ? 'Mechanism' : !r.ok ? 'Unstable' : r.yield ? 'Beam yields' : r.lift ? 'Support uplift' : r.util > 1 ? 'Over allowable stress' : r.utilD > 1 ? 'Too flexible' : 'Within limits'; st.className = 'pill ' + (r.mech || !r.ok || r.yield || r.lift || r.util > 1 ? 'warn' : r.utilD > 1 ? 'amber' : 'ok');
  $('#pillUtil').textContent = `stress ${fmt(r.sigma, 0)} MPa · ${fmt(r.util * 100, 0)} % of allowable`; $('#pillDef').textContent = `deflection ${fmt(r.dmax, 1)} mm · span/${Number.isFinite(r.ratio) ? fmt(r.ratio, 0) : '∞'}`;
  $('#pillYield').classList.toggle('hidden', !r.yield); $('#pillDefl').classList.toggle('hidden', !(r.utilD > 1)); $('#pillLift').classList.toggle('hidden', !r.lift); $('#pillMech').classList.toggle('hidden', !(r.mech || !r.ok));
}
function loadsText() { return state.loads.map(l => l.type === 'p' ? `P${l.P}@${l.x}` : l.type === 'w' ? `w${l.w}[${l.a}-${l.b}]` : `M${l.M}@${l.x}`).join(' ') + (state.selfw ? ' +sw' : ''); }
function record(tag = '') {
  const r = R, A = r.supports.find(s => s.name === 'A'), B = r.supports.find(s => s.name === 'B');
  const row = { i: state.log.length + 1, tag, st: state.stype, L: state.L, xa: state.xa, xb: state.stype === 'cant' ? '' : state.xb, sec: `${state.sec} ${state.d1}/${state.d2}/${state.d3} ${state.mat}`, I: +(r.sec.I / 1e4).toFixed(0), loads: loadsText(), RA: +(A ? A.R : 0).toFixed(2), RB: +(B ? B.R : 0).toFixed(2), V: +Math.abs(r.Vmax).toFixed(2), M: +r.M[r.kMa].toFixed(2), xM: +r.xM.toFixed(2), Mmin: +r.Mmin.toFixed(2), s: +r.sigma.toFixed(1), d: +r.dmax.toFixed(2), xD: +r.xD.toFixed(2), ratio: Number.isFinite(r.ratio) ? Math.round(r.ratio) : '', cf: r.cf, d2: state.d2, secType: state.sec, mat: state.mat };
  state.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.st, r.L, r.xa, r.xb, r.sec, r.I, r.loads, r.RA, r.RB, r.V, r.M, r.xM, r.Mmin, r.s, r.d, r.xD, r.ratio].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,supports,L_m,xA,xB,section,I_cm4,loads,RA_kN,RB_kN,Vmax_kN,Mmax_kNm,at_m,Mmin_kNm,sigma_MPa,defl_mm,at_m,span_over_defl\n' + state.log.map(r => [r.i, r.tag, r.st, r.L, r.xa, r.xb, r.sec, r.I, `"${r.loads}"`, r.RA, r.RB, r.V, r.M, r.xM, r.Mmin, r.s, r.d, r.xD, r.ratio].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}

/* ------------------------------------------------------------------ tasks */
const isFullUDL = () => state.loads.length === 1 && state.loads[0].type === 'w' && Math.min(state.loads[0].a, state.loads[0].b) <= 0.01 && Math.max(state.loads[0].a, state.loads[0].b) >= state.L - 0.01 && !state.selfw;
const endsSS = () => state.stype === 'ss' && state.xa <= 0.01 && state.xb >= state.L - 0.01;
const TASKS = [
  { id: 'udl', t: 'wL²/8 and wL/2', d: 'Simply supported at the ends with one uniform load over the whole span. Record it and check R = wL/2 and M = wL²/8.', ok: () => state.flags.udl },
  { id: 'pl4', t: 'PL/4', d: 'One point load at mid-span on a simply supported beam. Record it; check M = PL/4 and δ = PL³/48EI.', ok: () => state.flags.pl4 },
  { id: 'cant', t: 'Cantilever tip load', d: 'Fixed at A, one point load at the tip. Record it; check M = PL and δ = PL³/3EI.', ok: () => state.flags.cant },
  { id: 'over', t: 'Best overhang', d: 'Uniform load over the whole beam: slide both supports in until the sagging and hogging moments are equal (about 0.207 L).', ok: () => state.flags.over },
  { id: 'depth', t: 'Double the depth', d: 'Record the same beam and loads with a section depth h and then 2h. Deflection should drop eight times for a rectangle.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.secType === 'rect' && b.secType === 'rect' && a.loads === b.loads && a.L === b.L && Math.abs(b.d2 - 2 * a.d2) < 1e-6)); } },
  { id: 'fixed', t: 'Fixed ends', d: 'Record the full uniform load simply supported and then fixed at both ends: the peak moment falls from wL²/8 to wL²/12 and moves to the supports.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.st === 'ss' && b.st === 'ff' && a.loads === b.loads && a.L === b.L)); } },
  { id: 'uplift', t: 'Lift a support', d: 'Put a load on an overhang until the far roller reaction goes negative.', ok: () => state.flags.uplift },
  { id: 'contra', t: 'Point of contraflexure', d: 'Record a case where the moment changes sign (propped cantilever, fixed ends, or an overhang).', ok: () => state.log.some(r => r.cf > 0) },
  { id: 'design', t: 'Size the beam', d: '20 kN/m over a 6 m simply supported span: pick a section and material with stress at 60–100 % of allowable and deflection within the limit.', ok: () => state.flags.design },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() {
  const r = R, f = state.flags;
  if (isFullUDL() && endsSS() && state.log.length && state.log[state.log.length - 1].loads === loadsText()) f.udl = true;
  if (endsSS() && state.loads.length === 1 && state.loads[0].type === 'p' && Math.abs(state.loads[0].x - state.L / 2) < 0.03 && state.log.some(l => l.loads === loadsText() && l.st === 'ss')) f.pl4 = true;
  if (state.stype === 'cant' && state.xa <= 0.01 && state.loads.length === 1 && state.loads[0].type === 'p' && state.loads[0].x >= state.L - 0.03 && state.log.some(l => l.loads === loadsText() && l.st === 'cant')) f.cant = true;
  if (isFullUDL() && state.stype === 'ss' && state.xa > 0.15 * state.L && state.xa < 0.26 * state.L && state.L - state.xb > 0.15 * state.L && state.L - state.xb < 0.26 * state.L && Math.abs(Math.abs(r.Mmax) - Math.abs(r.Mmin)) < 0.08 * Math.abs(r.Mmax)) f.over = true;
  if (r.lift) f.uplift = true;
  if (endsSS() && Math.abs(state.L - 6) < 0.01 && state.loads.length === 1 && state.loads[0].type === 'w' && Math.abs(state.loads[0].w - 20) < 0.01 && isFullUDL() && r.util >= 0.6 && r.util <= 1 && r.utilD <= 1) f.design = true;
  for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; }
}

/* ------------------------------------------------------------------ loads UI */
function addLoad(ld) { ld.id = state.nextId++; state.loads.push(ld); renderLoads(); refresh(); }
function renderLoads() {
  const box = $('#loads'); box.innerHTML = '';
  for (const ld of state.loads) {
    const row = document.createElement('div'); row.className = 'lrow ' + ld.type; const L = state.L;
    if (ld.type === 'p') row.innerHTML = `<label>Point load<select data-k="type"><option value="p" selected>Point load</option><option value="w">Distributed</option><option value="m">Couple</option></select></label><label>kN<input type="number" data-k="P" value="${ld.P}" min="0" max="500" step="1"></label><label>position ${fmt(ld.x, 2)} m<input type="range" data-k="x" min="0" max="${L}" step="0.05" value="${ld.x}"></label><span></span><button class="del" title="remove">×</button>`;
    else if (ld.type === 'w') row.innerHTML = `<label>Distributed<select data-k="type"><option value="p">Point load</option><option value="w" selected>Distributed</option><option value="m">Couple</option></select></label><label>kN/m<input type="number" data-k="w" value="${ld.w}" min="0" max="200" step="1"></label><label>from ${fmt(ld.a, 2)} m<input type="range" data-k="a" min="0" max="${L}" step="0.05" value="${ld.a}"></label><label>to ${fmt(ld.b, 2)} m<input type="range" data-k="b" min="0" max="${L}" step="0.05" value="${ld.b}"></label><button class="del" title="remove">×</button>`;
    else row.innerHTML = `<label>Couple<select data-k="type"><option value="p">Point load</option><option value="w">Distributed</option><option value="m" selected>Couple</option></select></label><label>kNm (+ anticlockwise)<input type="number" data-k="M" value="${ld.M}" min="-500" max="500" step="1"></label><label>position ${fmt(ld.x, 2)} m<input type="range" data-k="x" min="0" max="${L}" step="0.05" value="${ld.x}"></label><span></span><button class="del" title="remove">×</button>`;
    row.querySelectorAll('[data-k]').forEach(inp => inp.addEventListener('input', e => { const k = e.target.dataset.k; if (k === 'type') { const t = e.target.value; if (t !== ld.type) { const x = ld.x ?? ld.a ?? 0; Object.keys(ld).forEach(kk => kk !== 'id' && delete ld[kk]); Object.assign(ld, t === 'p' ? { type: 'p', P: 10, x } : t === 'w' ? { type: 'w', w: 10, a: 0, b: state.L } : { type: 'm', M: 20, x }); renderLoads(); } } else { ld[k] = +e.target.value; const lab = e.target.closest('label'); if (e.target.type === 'range') lab.firstChild.textContent = (k === 'a' ? 'from ' : k === 'b' ? 'to ' : 'position ') + fmt(ld[k], 2) + ' m'; } refresh(); }));
    row.querySelector('.del').onclick = () => { state.loads = state.loads.filter(l => l !== ld); renderLoads(); refresh(); };
    box.appendChild(row);
  }
}
function refresh() { Object.assign(R, solve()); drawBeam(); drawCharts(); updateReadouts(); checkTasks(); }

/* ------------------------------------------------------------------ dragging on the drawing */
function bindDrag() {
  const svg = $('#beam'); const toX = e => { const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY; return xOf(pt.matrixTransform(svg.getScreenCTM().inverse()).x); };
  svg.addEventListener('pointerdown', e => { const g = e.target.closest('[data-load],[data-sup]'); if (!g) return; e.preventDefault(); svg.setPointerCapture(e.pointerId); state.drag = g.dataset.load ? { load: +g.dataset.load } : { sup: g.dataset.sup }; });
  svg.addEventListener('pointermove', e => { if (!state.drag) return; const x = Math.round(toX(e) * 20) / 20; if (state.drag.load) { const ld = state.loads.find(l => l.id === state.drag.load); if (ld) ld.x = x; } else if (state.stype === 'ss') { if (state.drag.sup === 'A') setCtl('xa', Math.min(x, state.xb - 0.1)); else setCtl('xb', Math.max(x, state.xa + 0.1)); } refresh(); });
  const up = () => { if (state.drag && state.drag.load) renderLoads(); state.drag = null; }; svg.addEventListener('pointerup', up); svg.addEventListener('pointercancel', up);
}

/* ------------------------------------------------------------------ wiring */
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') e.checked = !!v; else e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSection(type) { state.sec = type; const s = SECS[type]; ['d1', 'd2', 'd3'].forEach((id, i) => { const dd = s.dims[i], ctl = $('#' + id).closest('.ctl'); ctl.style.display = dd ? '' : 'none'; if (dd) { $('#' + id + 'Name').textContent = dd[0]; const e = $('#' + id); e.min = dd[1]; e.max = dd[2]; e.value = dd[3]; state[id] = dd[3]; $('#' + id + 'Out').textContent = dd[3] + ' mm'; } }); }
const PRESETS = {
  pl: () => { base('ss'); addLoad({ type: 'p', P: 20, x: 3 }); },
  udl: () => { base('ss'); addLoad({ type: 'w', w: 10, a: 0, b: 6 }); },
  four: () => { base('ss'); addLoad({ type: 'p', P: 15, x: 2 }); addLoad({ type: 'p', P: 15, x: 4 }); },
  cant: () => { base('cant'); addLoad({ type: 'p', P: 10, x: 6 }); },
  over: () => { base('ss'); setCtl('xa', 1.2); setCtl('xb', 4.8); addLoad({ type: 'w', w: 10, a: 0, b: 6 }); },
  prop: () => { base('prop'); addLoad({ type: 'w', w: 10, a: 0, b: 6 }); },
  ff: () => { base('ff'); addLoad({ type: 'w', w: 10, a: 0, b: 6 }); },
  couple: () => { base('ss'); addLoad({ type: 'm', M: 30, x: 3 }); },
  uplift: () => { base('ss'); setCtl('xa', 0); setCtl('xb', 4); addLoad({ type: 'p', P: 25, x: 6 }); },
};
function base(st) { state.loads = []; setCtl('L', 6); $('#stype').value = st; state.stype = st; setCtl('xa', 0); setCtl('xb', 6); setCtl('selfw', false); renderLoads(); }
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('L', v => v.toFixed(1) + ' m'); bindR('xa', v => v.toFixed(2) + ' m'); bindR('xb', v => v.toFixed(2) + ' m'); bindR('exag', v => v ? '×' + v : 'auto'); bindR('d1', v => v + ' mm'); bindR('d2', v => v + ' mm'); bindR('d3', v => v + ' mm'); bindR('dlim', v => String(v));
  $('#L').addEventListener('input', () => { $('#xa').max = state.L; $('#xb').max = state.L; if (state.xb > state.L) setCtl('xb', state.L); state.loads.forEach(l => { if (l.x !== undefined) l.x = Math.min(l.x, state.L); if (l.b !== undefined) l.b = Math.min(l.b, state.L); }); renderLoads(); });
  $('#stype').addEventListener('change', e => { state.stype = e.target.value; $('#xb').closest('.ctl').style.display = state.stype === 'cant' ? 'none' : ''; });
  $('#selfw').addEventListener('change', e => state.selfw = e.target.checked); $('#sec').addEventListener('change', e => setSection(e.target.value)); $('#mat').addEventListener('change', e => state.mat = e.target.value);
  ['L', 'xa', 'xb', 'exag', 'd1', 'd2', 'd3', 'dlim', 'stype', 'selfw', 'sec', 'mat'].forEach(id => { $('#' + id).addEventListener('input', refresh); $('#' + id).addEventListener('change', refresh); });
  $('#addP').onclick = () => addLoad({ type: 'p', P: 10, x: +(state.L / 2).toFixed(2) }); $('#addW').onclick = () => addLoad({ type: 'w', w: 10, a: 0, b: state.L }); $('#addM').onclick = () => addLoad({ type: 'm', M: 20, x: +(state.L / 2).toFixed(2) });
  $('#btnRecord').onclick = () => record(''); $('#btnClearLoads').onclick = () => { state.loads = []; renderLoads(); refresh(); };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => { PRESETS[b.dataset.preset](); refresh(); });
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('bm_theme', next); } catch (e) {} refresh(); };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (k === 'R') record(''); else if (k === 'P') $('#addP').click(); else if (k === 'W') $('#addW').click(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); });
  window.addEventListener('resize', drawCharts); bindDrag();
}

/* ------------------------------------------------------------------ dashboard API */
window.BeamLoads = {
  getState: () => ({ inputs: { L: state.L, stype: state.stype, xa: state.xa, xb: state.xb, selfw: state.selfw, sec: state.sec, d1: state.d1, d2: state.d2, d3: state.d3, mat: state.mat, dlim: state.dlim, loads: state.loads.map(({ id, ...l }) => l) }, results: { RA: R.supports.find(s => s.name === 'A')?.R, RB: R.supports.find(s => s.name === 'B')?.R, Vmax: R.Vmax, Mmax: R.Mmax, Mmin: R.Mmin, sigma: R.sigma, dmax: R.dmax, ratio: R.ratio, util: R.util, utilD: R.utilD, yield: R.yield, lift: R.lift, cf: R.cf }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.stype) { $('#stype').value = inp.stype; state.stype = inp.stype; } for (const k of ['L', 'xa', 'xb', 'd1', 'd2', 'd3', 'dlim', 'exag']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.sec) { $('#sec').value = inp.sec; setSection(inp.sec); } if (inp.mat) { $('#mat').value = inp.mat; state.mat = inp.mat; } if (inp.selfw !== undefined) setCtl('selfw', inp.selfw); if (Array.isArray(inp.loads)) { state.loads = []; inp.loads.forEach(l => addLoad({ ...l })); } refresh(); },
  meta: { id: 'beam-loads', title: 'Loads on Beams Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.BeamLoads.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'beam-loads', state: window.BeamLoads.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('bm_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildTasks(); bind(); setSection('isec'); renderLog(); PRESETS.udl(); refresh();
}
build();
