/* Bearing and Lubrication Rig — journal bearing (short-bearing load solution, viscosity–temperature iteration, λ-blended
   friction on the Stribeck curve, lumped thermal balance on a thermal clock) and rolling-element bearing (equivalent load,
   L10 and modified life, Palmgren friction, defect frequencies). Standalone, offline. Rig data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const RHO = 870, CP = 1900, F_BOUND = 0.10, P_LIMIT = 5, T_LIMIT = 95;
const OILS = { 22: [22, 4.3], 32: [32, 5.4], 46: [46, 6.8], 68: [68, 8.7], 100: [100, 11.4], 150: [150, 15.0], grease: [100, 11.4] };   // ν40, ν100 cSt
const BRG = {
  6205: { name: '6205', type: 'ball', d: 25, Do: 52, C: 14.0, C0: 7.8, n: 9, dw: 7.94, dm: 38.5, alpha: 0, nlim: 22000, f0: 2, f1k: 0.0009 },
  6208: { name: '6208', type: 'ball', d: 40, Do: 80, C: 32.5, C0: 19.0, n: 9, dw: 12.7, dm: 60, alpha: 0, nlim: 13000, f0: 2, f1k: 0.0009 },
  6310: { name: '6310', type: 'ball', d: 50, Do: 110, C: 65, C0: 38, n: 8, dw: 19.05, dm: 80, alpha: 0, nlim: 9500, f0: 2, f1k: 0.0009 },
  7208: { name: '7208', type: 'ac', d: 40, Do: 80, C: 36.5, C0: 24, n: 13, dw: 11.5, dm: 60, alpha: 40, nlim: 12000, f0: 2, f1k: 0.001 },
  NU208: { name: 'NU208', type: 'cyl', d: 40, Do: 80, C: 53, C0: 48, n: 14, dw: 10, dm: 60, alpha: 0, nlim: 12000, f0: 3, f1k: 0.0003 },
  22210: { name: '22210', type: 'sph', d: 50, Do: 90, C: 99, C0: 108, n: 30, dw: 10, dm: 70, alpha: 12, nlim: 6300, f0: 4, f1k: 0.0005 },
};
const DG_TABLE = [[0.014, 0.19, 2.30], [0.028, 0.22, 1.99], [0.056, 0.26, 1.71], [0.084, 0.28, 1.55], [0.11, 0.30, 1.45], [0.17, 0.34, 1.31], [0.28, 0.38, 1.15], [0.42, 0.42, 1.04], [0.56, 0.44, 1.00]];

/* ------------------------------------------------------------------ state */
const state = { mode: 'journal', W: 5, N: 1500, Fa: 0, D: 60, LD: 1, cr: 0.0015, Ra: 0.8, brg: '6208', cont: 0.5, rel: 1, oil: '46', Tin: 40, Qs: 100, tscale: 20, run: false, T: 40, wiped: false, wipeT: 0, t: 0, trace: [], traceAcc: 0, sweep: null, sweepPts: [], log: [], flags: {} };
const R = {};

/* ------------------------------------------------------------------ oil */
function viscosity(oil, T) {   // kinematic cSt at T °C (ASTM D341 Walther) → dynamic Pa·s
  const [v40, v100] = OILS[oil], Z = v => Math.log10(Math.log10(v + 0.7)), b = (Z(v40) - Z(v100)) / (Math.log10(373.15) - Math.log10(313.15)), a = Z(v40) + b * Math.log10(313.15);
  const zz = a - b * Math.log10(T + 273.15), nu = Math.pow(10, Math.pow(10, zz)) - 0.7; return { nu, mu: nu * 1e-6 * RHO };
}

/* ------------------------------------------------------------------ journal bearing */
function journal(Tfilm) {
  const s = state, Rr = s.D / 2000, c = s.cr * Rr, L = s.LD * s.D / 1000, W = s.W * 1000, w = s.N * 2 * Math.PI / 60, U = w * Rr, { mu, nu } = viscosity(s.oil, Tfilm);
  const cap = e => mu * U * L ** 3 / (4 * c * c) * e / (1 - e * e) ** 2 * Math.sqrt(16 * e * e + Math.PI * Math.PI * (1 - e * e));   // Ocvirk short-bearing load
  let lo = 1e-4, hi = 0.9995; for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; if (cap(m) < W) lo = m; else hi = m; } const e = (lo + hi) / 2;
  const h0 = c * (1 - e), phi = Math.atan(Math.PI * Math.sqrt(1 - e * e) / (4 * e)) * 180 / Math.PI, P = W / (L * s.D / 1000);
  const Tq = mu * w * Rr ** 3 * L / c * 2 * Math.PI / Math.sqrt(1 - e * e), fh = Tq / (W * Rr), lam = h0 / (s.Ra * 1e-6), k = 1 - Math.exp(-Math.pow(lam / 1.5, 2)), f = k * fh + (1 - k) * F_BOUND * (s.wiped ? 1.5 : 1);
  const Tfric = f * W * Rr, loss = Tfric * w, Qside = e * U * c * L * (s.Qs / 100), Qmin = 1e-7, Q = Math.max(Qside, Qmin);
  const S = (Rr / c) ** 2 * mu * (s.N / 60) / P, hersey = mu * (s.N / 60) / P, Tsteady = s.Tin + loss / (RHO * CP * Q) * (s.Qs > 0 ? 1 : 3);
  return { e, h0, phi, P: P / 1e6, Tq: Tfric, f, loss, Q: Qside * 6e4, Qs: s.Qs, S, hersey, lam, mu, nu, Tsteady: Math.min(Tsteady, 220), c, U, regime: lam < 1 ? 'boundary' : lam < 3 ? 'mixed' : 'hydro' };
}
/* ------------------------------------------------------------------ rolling bearing */
function rolling(T) {
  const s = state, b = BRG[s.brg], Fr = s.W, Fa = s.Fa; let X = 1, Y = 0, e = 0, note = '';
  if (b.type === 'ball') { const r = Fa / b.C0; let row = DG_TABLE[0]; for (const t of DG_TABLE) if (r >= t[0]) row = t; e = row[1]; if (Fa / Math.max(Fr, 1e-6) > e) { X = 0.56; Y = row[2]; } }
  else if (b.type === 'ac') { e = 1.14; if (Fa / Math.max(Fr, 1e-6) > e) { X = 0.35; Y = 0.57; } }
  else if (b.type === 'sph') { e = 0.31; if (Fa / Math.max(Fr, 1e-6) <= e) { X = 1; Y = 2.2; } else { X = 0.67; Y = 3.3; } }
  else { X = 1; Y = 0; if (Fa > 0.05) note = 'cylindrical roller: axial load not carried'; }
  const P = Math.max(1e-3, X * Fr + Y * Fa), p = b.type === 'ball' || b.type === 'ac' ? 3 : 10 / 3, L10 = Math.pow(b.C / P, p), L10h = L10 * 1e6 / (60 * s.N);
  const { nu } = viscosity(s.oil, T), nu1 = s.N < 1000 ? 45000 * Math.pow(s.N, -0.83) * Math.pow(b.dm, -0.5) : 4500 * Math.pow(s.N, -0.5) * Math.pow(b.dm, -0.5), kappa = clamp(nu / nu1, 0.05, 4);
  const Cu = b.C0 / (b.type === 'ball' || b.type === 'ac' ? 22 : 8), x = s.cont * Cu / P, aiso = clamp(10 * Math.pow(x * kappa, 0.7), 0.1, 50), Lnm = s.rel * aiso * L10h;
  const vn = nu * s.N, M = (vn >= 2000 ? b.f0 * Math.pow(vn, 2 / 3) : 160 * b.f0) * 1e-7 * b.dm ** 3 * (s.oil === 'grease' ? 1.3 : 1) + b.f1k * Math.pow(P / b.C0, 0.55) * P * 1000 * b.dm * 1e-3 * 1000 / 1000;
  const loss = M * 1e-3 * s.N * 2 * Math.PI / 60, Rth = 40 / b.dm, Tsteady = s.Tin + loss * Rth, fr = s.N / 60, g = b.dw / b.dm * Math.cos(b.alpha * Math.PI / 180);
  return { b, P, X, Y, e, L10, L10h, Lnm, kappa, nu, aiso, M, loss, Tsteady: Math.min(Tsteady, 200), BPFO: b.n / 2 * fr * (1 - g), BPFI: b.n / 2 * fr * (1 + g), BSF: b.dm / (2 * b.dw) * fr * (1 - g * g), FTF: 0.5 * fr * (1 - g), nlim: s.oil === 'grease' ? b.nlim * 0.8 : b.nlim, Tlim: s.oil === 'grease' ? 90 : 110, minOK: P >= 0.01 * b.C, note };
}

/* ------------------------------------------------------------------ frame */
function step(dt) {
  const s = state; if (s.sweep) { s.sweep.t += dt; s.N = Math.round(Math.exp(Math.log(20) + (Math.log(6000) - Math.log(20)) * clamp(s.sweep.t / 60, 0, 1)) / 10) * 10; $('#N').value = s.N; $('#NOut').textContent = s.N.toLocaleString().replace(/,/g, ' ') + ' rpm'; }
  if (s.mode === 'journal') { const j = journal(s.T); Object.assign(R, j); if (s.run) { const tau = 60; s.T += (j.Tsteady - s.T) * Math.min(1, dt * s.tscale / tau); if ((j.lam < 0.5 || s.T > 150) && s.run) { s.wipeT += dt * s.tscale; if (s.wipeT > 20 && !s.wiped) { s.wiped = true; record(j.lam < 0.5 ? 'wiped' : 'overheated'); $('#btnRepair').classList.remove('hidden'); } } else s.wipeT = Math.max(0, s.wipeT - dt * s.tscale); } else s.T += (s.Tin - s.T) * Math.min(1, dt * s.tscale / 120); }
  else { const r = rolling(s.T); Object.assign(R, r); const tau = 120; s.T += ((s.run ? r.Tsteady : s.Tin) - s.T) * Math.min(1, dt * s.tscale / tau); }
  if (s.sweep) { if (s.sweep.t > 60) s.sweep = null; else if (s.mode === 'journal' && s.t - (s.sweep.last || 0) > 1.2) { s.sweep.last = s.t; s.sweepPts.push({ h: R.hersey, f: R.f }); } }
  s.traceAcc += dt; if (s.traceAcc >= 0.25) { s.traceAcc = 0; s.trace.push({ t: s.t, T: s.T, a: s.mode === 'journal' ? R.h0 * 1e6 : R.Lnm, f: s.mode === 'journal' ? R.f : R.M }); if (s.trace.length > 1200) s.trace.shift(); }
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg, ang = 0, cageAng = 0;
function buildRig() { svg = el('svg', { viewBox: '0 0 900 420', role: 'img', 'aria-label': 'Bearing rig' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig(dt) {
  svg.innerHTML = ''; const s = state, cx = 300, cy = 215;
  if (s.mode === 'journal') {
    const Rb = 150, cgap = 22, Rj = Rb - cgap, e = R.e, phi = R.phi * Math.PI / 180;   // exaggerated clearance; eccentricity along the attitude angle from the load line (load downward)
    const ex = e * cgap * Math.sin(phi), ey = e * cgap * Math.cos(phi);   // journal displaced down and to the side of the load line
    el('circle', { cx, cy, r: Rb + 40, class: 'bush' }, svg); el('circle', { cx, cy, r: Rb, class: 'bore' }, svg);
    el('circle', { cx, cy, r: Rb - 1, class: 'film' }, svg);
    // pressure lobe on the converging side
    let d = ''; for (let a = 0; a <= 360; a += 4) { const th = a * Math.PI / 180, ang0 = th - (Math.PI / 2 + phi); let p = Math.max(0, Math.sin(ang0)) * e; const rr = Rb + 6 + p * 34; d += (a ? 'L' : 'M') + (cx + rr * Math.cos(th)).toFixed(1) + ',' + (cy + rr * Math.sin(th)).toFixed(1) + ' '; } el('path', { d: d + 'Z', class: 'pressure' }, svg);
    ang = (ang + s.N / 60 * 360 * dt * 0.05) % 360; const jx = cx + ex, jy = cy + ey;
    el('circle', { cx: jx, cy: jy, r: Rj, class: 'journal' }, svg); el('line', { x1: jx, y1: jy, x2: jx + Rj * 0.9 * Math.cos(ang * Math.PI / 180), y2: jy + Rj * 0.9 * Math.sin(ang * Math.PI / 180), class: 'keyline' }, svg);
    el('line', { x1: jx, y1: jy - Rj - 70, x2: jx, y2: jy - Rj - 14, class: 'loadarrow' }, svg); el('polygon', { points: `${jx},${jy - Rj - 6} ${jx - 7},${jy - Rj - 18} ${jx + 7},${jy - Rj - 18}`, class: 'loadhead' }, svg); txt(jx + 10, jy - Rj - 40, `W ${s.W} kN`, 'val', svg);
    const hx = cx + (Rb - cgap * (1 - e) / 2) * Math.sin(phi) * 1, hy = cy + (Rb - cgap * (1 - e) / 2) * Math.cos(phi); txt(hx + 14, hy + 4, `h₀ ${fmt(R.h0 * 1e6, 1)} µm`, 'val' + (R.lam < 1 ? ' bad' : ''), svg);
    txt(cx, cy + Rb + 62, `clearance exaggerated · ε ${fmt(e, 3)} · attitude ${fmt(R.phi, 0)}°`, 'lbl', svg, 'middle');
    el('rect', { x: 560, y: 60, width: 300, height: 18, fill: 'var(--panel2)' }, svg); el('rect', { x: 560, y: 60, width: 300 * clamp((s.T - 20) / 200, 0, 1), height: 18, class: 'oilbar' }, svg); txt(560, 52, `film temperature ${fmt(s.T, 0)} °C · viscosity ${fmt(R.mu * 1000, 1)} mPa·s`, 'lbl', svg);
    el('rect', { x: 560, y: 110, width: 300, height: 18, fill: 'var(--panel2)' }, svg); el('rect', { x: 560, y: 110, width: 300 * clamp(R.lam / 6, 0, 1), height: 18, fill: R.lam < 1 ? 'var(--alarm)' : R.lam < 3 ? 'var(--warn)' : 'var(--good)' }, svg); txt(560, 102, `λ = h₀ / Ra = ${fmt(R.lam, 2)} · ${R.regime === 'hydro' ? 'hydrodynamic' : R.regime} lubrication`, 'lbl', svg); for (const [v, l] of [[1, '1'], [3, '3']]) { el('line', { x1: 560 + 300 * v / 6, y1: 108, x2: 560 + 300 * v / 6, y2: 130, stroke: 'var(--ink)', 'stroke-width': 1 }, svg); txt(560 + 300 * v / 6, 142, l, 'lbl', svg, 'middle'); }
    txt(560, 180, 'FRICTION', 'lbl', svg); txt(560, 198, `f ${fmt(R.f, 4)} · torque ${fmt(R.Tq, 2)} N·m · loss ${fmt(R.loss, 0)} W`, 'val', svg);
    txt(560, 230, 'OIL', 'lbl', svg); txt(560, 248, `${s.oil === 'grease' ? 'grease' : 'ISO VG ' + s.oil} · supply ${s.Qs} % · side leakage ${fmt(R.Q, 2)} L/min`, 'val', svg);
    txt(560, 280, 'GEOMETRY', 'lbl', svg); txt(560, 298, `Ø${s.D} × L ${fmt(s.LD * s.D, 0)} mm · c ${fmt(R.c * 1e6, 0)} µm · Ra ${s.Ra} µm`, 'val', svg);
    if (s.wiped) txt(560, 340, 'BEARING FAILED — babbitt wiped / overheated, replace', 'val bad', svg);
  } else {
    const b = R.b, Ro = 160, Ri = Ro * b.d / b.Do, Rm = (Ro + Ri) / 2, rb = (Ro - Ri) * 0.42;
    el('circle', { cx, cy, r: Ro, class: 'race' }, svg); el('circle', { cx, cy, r: Ro - (Ro - Ri) * 0.22, fill: 'var(--panel)' }, svg); el('circle', { cx, cy, r: Ri + (Ro - Ri) * 0.22, class: 'race' }, svg); el('circle', { cx, cy, r: Ri, fill: 'var(--panel)', stroke: 'var(--steel)' }, svg);
    ang = (ang + s.N / 60 * 360 * dt * 0.05) % 360; cageAng = (cageAng + R.FTF * 360 * dt * 0.05) % 360;
    el('circle', { cx, cy, r: Rm, class: 'cage', transform: `rotate(${cageAng} ${cx} ${cy})` }, svg);
    const n = Math.min(b.n, 16); for (let i = 0; i < n; i++) { const a = (cageAng + i * 360 / n) * Math.PI / 180, bx = cx + Rm * Math.cos(a), by = cy + Rm * Math.sin(a); if (b.type === 'cyl' || b.type === 'sph') el('rect', { x: bx - rb * 0.8, y: by - rb, width: rb * 1.6, height: rb * 2, rx: 3, class: 'ball' + (!R.minOK ? ' bad' : ''), transform: `rotate(${a * 180 / Math.PI + 90} ${bx} ${by})` }, svg); else el('circle', { cx: bx, cy: by, r: rb, class: 'ball' + (!R.minOK ? ' bad' : '') }, svg); }
    el('line', { x1: cx, y1: cy, x2: cx + Ri * 0.8 * Math.cos(ang * Math.PI / 180), y2: cy + Ri * 0.8 * Math.sin(ang * Math.PI / 180), class: 'keyline' }, svg);
    el('line', { x1: cx, y1: cy - Ro - 70, x2: cx, y2: cy - Ro - 14, class: 'loadarrow' }, svg); el('polygon', { points: `${cx},${cy - Ro - 6} ${cx - 7},${cy - Ro - 18} ${cx + 7},${cy - Ro - 18}`, class: 'loadhead' }, svg); txt(cx + 10, cy - Ro - 40, `Fr ${s.W} kN · Fa ${s.Fa} kN`, 'val', svg);
    txt(cx, cy + Ro + 40, `${b.name} · d ${b.d} · D ${b.Do} mm · ${b.n} elements · C ${b.C} kN · C₀ ${b.C0} kN`, 'lbl', svg, 'middle');
    txt(560, 52, `bearing temperature ${fmt(s.T, 0)} °C · limit ${R.Tlim} °C`, 'lbl', svg); el('rect', { x: 560, y: 60, width: 300, height: 18, fill: 'var(--panel2)' }, svg); el('rect', { x: 560, y: 60, width: 300 * clamp((s.T - 20) / 180, 0, 1), height: 18, fill: s.T > R.Tlim ? 'var(--alarm)' : 'var(--brass)', opacity: .6 }, svg);
    txt(560, 110, 'LIFE', 'lbl', svg); txt(560, 128, `L10 ${fmt(R.L10h, 0)} h · modified ${fmt(R.Lnm, 0)} h · C/P ${fmt(b.C / R.P, 1)}`, 'val' + (R.L10h < 5000 ? ' bad' : ''), svg);
    txt(560, 160, 'FRICTION', 'lbl', svg); txt(560, 178, `M ${fmt(R.M, 0)} N·mm · loss ${fmt(R.loss, 0)} W · κ ${fmt(R.kappa, 2)}`, 'val', svg);
    txt(560, 210, 'DEFECT FREQUENCIES AT THIS SPEED', 'lbl', svg); txt(560, 228, `BPFO ${fmt(R.BPFO, 1)} · BPFI ${fmt(R.BPFI, 1)} · BSF ${fmt(R.BSF, 1)} · FTF ${fmt(R.FTF, 2)} Hz`, 'val', svg);
    txt(560, 260, 'CHECKS', 'lbl', svg); txt(560, 278, `${s.N > R.nlim ? 'over the speed limit' : 'speed OK'} · ${R.minOK ? 'minimum load OK' : 'load too light — elements may skid'}`, 'val' + (s.N > R.nlim || !R.minOK ? ' bad' : ''), svg); if (R.note) txt(560, 298, R.note, 'val bad', svg);
  }
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawChart() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 16, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  if (s.mode === 'journal') {
    const hmin = 1e-9, hmax = 1e-5, X = h => L + (Math.log10(h) - Math.log10(hmin)) / 4 * pw, fmax = 0.14, Y = f => T + (1 - clamp(f / fmax, 0, 1)) * ph;
    ctx.strokeStyle = col('--grid'); for (let d = -9; d <= -5; d++) { ctx.beginPath(); ctx.moveTo(X(10 ** d), T); ctx.lineTo(X(10 ** d), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText('1e' + d, X(10 ** d), H - 16); }
    for (const f of [0.02, 0.05, 0.1]) { ctx.beginPath(); ctx.moveTo(L, Y(f)); ctx.lineTo(L + pw, Y(f)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(f, L - 4, Y(f) + 3); }
    // curve: sweep speed at the current film temperature
    const saveN = s.N; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 2; let first = true, bnd = null, hyd = null;
    for (let i = 0; i <= 120; i++) { s.N = Math.exp(Math.log(5) + (Math.log(20000) - Math.log(5)) * i / 120); const j = journal(s.T); const x = X(j.hersey), y = Y(j.f); first ? ctx.moveTo(x, y) : ctx.lineTo(x, y); first = false; if (bnd === null && j.lam >= 1) bnd = x; if (hyd === null && j.lam >= 3) hyd = x; } ctx.stroke(); s.N = saveN;
    ctx.globalAlpha = 0.12; ctx.fillStyle = col('--alarm'); ctx.fillRect(L, T, (bnd || L) - L, ph); ctx.fillStyle = col('--warn'); ctx.fillRect(bnd || L, T, (hyd || bnd || L) - (bnd || L), ph); ctx.fillStyle = col('--good'); ctx.fillRect(hyd || L, T, L + pw - (hyd || L), ph); ctx.globalAlpha = 1;
    ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.font = '600 11px Barlow Condensed, sans-serif'; ctx.fillText('boundary', ((bnd || L) + L) / 2, T + 14); ctx.fillText('mixed', ((bnd || L) + (hyd || bnd || L)) / 2, T + 14); ctx.fillText('hydrodynamic', ((hyd || L) + L + pw) / 2, T + 14);
    for (const p of s.sweepPts) { ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(X(p.h), Y(p.f), 3, 0, 7); ctx.fill(); }
    ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(R.hersey), Y(R.f), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = X(R.hersey) > L + pw * 0.6 ? 'right' : 'left'; ctx.fillText(`f ${fmt(R.f, 4)} · λ ${fmt(R.lam, 2)} · ${s.N} rpm`, X(R.hersey) + (X(R.hersey) > L + pw * 0.6 ? -10 : 10), Y(R.f) - 10);
    ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('Hersey number μN/P (curve drawn at the present film temperature)', L + pw / 2, H - 4); ctx.save(); ctx.translate(12, T + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('friction coefficient f', 0, 0); ctx.restore();
  } else {
    const b = R.b, Pmin = b.C / 40, Pmax = b.C0 * 1.2, X = P => L + (Math.log10(P) - Math.log10(Pmin)) / (Math.log10(Pmax) - Math.log10(Pmin)) * pw, Y = h => T + (1 - clamp((Math.log10(h) - 2) / 5, 0, 1)) * ph;
    ctx.strokeStyle = col('--grid'); for (let d = 2; d <= 7; d++) { ctx.beginPath(); ctx.moveTo(L, Y(10 ** d)); ctx.lineTo(L + pw, Y(10 ** d)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(d >= 6 ? (10 ** (d - 6)) + 'M h' : (10 ** (d - 3)) + 'k h', L - 4, Y(10 ** d) + 3); }
    for (const P of [1, 2, 5, 10, 20, 50, 100]) if (P >= Pmin && P <= Pmax) { ctx.beginPath(); ctx.moveTo(X(P), T); ctx.lineTo(X(P), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(P + ' kN', X(P), H - 16); }
    const p = b.type === 'ball' || b.type === 'ac' ? 3 : 10 / 3, curve = (fn, color, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash(dash || []); let f = true; for (let i = 0; i <= 100; i++) { const P = Math.exp(Math.log(Pmin) + (Math.log(Pmax) - Math.log(Pmin)) * i / 100), h = fn(P); f ? ctx.moveTo(X(P), Y(h)) : ctx.lineTo(X(P), Y(h)); f = false; } ctx.stroke(); ctx.setLineDash([]); };
    const L10h = P => Math.pow(b.C / P, p) * 1e6 / (60 * s.N); curve(L10h, col('--water')); curve(P => { const Cu = b.C0 / (b.type === 'ball' || b.type === 'ac' ? 22 : 8); return s.rel * clamp(10 * Math.pow(s.cont * Cu / P * R.kappa, 0.7), 0.1, 50) * L10h(P); }, col('--good'), [5, 4]);
    ctx.strokeStyle = col('--alarm'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(b.C0), T); ctx.lineTo(X(b.C0), T + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.textAlign = 'right'; ctx.fillText('C₀ static limit', X(b.C0) - 4, T + 12);
    ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(R.P), Y(R.L10h), 6, 0, 7); ctx.fill(); ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(X(R.P), Y(R.Lnm), 5, 0, 7); ctx.fill();
    ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`P ${fmt(R.P, 2)} kN · L10 ${fmt(R.L10h, 0)} h · modified ${fmt(R.Lnm, 0)} h`, Math.min(X(R.P) + 10, L + pw - 300), Math.max(T + 26, Y(R.L10h) - 10));
    ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`equivalent load P (kN) · ${b.name} at ${s.N} rpm · solid L10, dashed modified life (a1·aISO)`, L + pw / 2, H - 4);
  }
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 56, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, span = 300 / state.tscale * 1, tr = state.trace.filter(p => p.t > state.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return;
  const X = t => L + (t - (state.t - span)) / span * pw, series = (key, color, lo, hi) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 1.6; let f = true; for (const p of tr) { const y = T + (1 - clamp((p[key] - lo) / (hi - lo), 0, 1)) * ph; f ? ctx.moveTo(X(p.t), y) : ctx.lineTo(X(p.t), y); f = false; } ctx.stroke(); };
  const amax = Math.max(1e-9, ...tr.map(p => p.a)), fmax = Math.max(1e-9, ...tr.map(p => p.f));
  series('T', col('--alarm'), 20, 220); series('a', col('--water'), 0, amax * 1.1); series('f', col('--brass'), 0, fmax * 1.1);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText('220 °C', L - 4, T + 10); ctx.fillText('20', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText(state.mode === 'journal' ? `${fmt(amax, 0)} µm` : `${fmt(amax, 0)} h`, L + pw + 4, T + 10); ctx.fillStyle = col('--brass'); ctx.fillText(state.mode === 'journal' ? `f ${fmt(fmax, 3)}` : `${fmt(fmax, 0)} N·mm`, L + pw + 4, T + 24);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`thermal clock ×${state.tscale} · window ${fmt(span * state.tscale / 60, 0)} min of bearing time`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, j = R; $('#pillRun').textContent = s.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', s.run); $('#pillT').textContent = `${s.mode === 'journal' ? 'oil film' : 'bearing'} ${fmt(s.T, 0)} °C`;
  if (s.mode === 'journal') {
    $('#roS').textContent = fmt(j.S, 3); $('#roE').textContent = fmt(j.e, 3); $('#roPhi').textContent = fmt(j.phi, 0); $('#roH0').textContent = fmt(j.h0 * 1e6, 1); $('#roLam').textContent = fmt(j.lam, 2); $('#roH0Box').classList.toggle('alarm', j.lam < 1); $('#roH0Box').classList.toggle('warnv', j.lam >= 1 && j.lam < 3);
    $('#roF').textContent = fmt(j.f, 4); $('#roTq').textContent = fmt(j.Tq, 2); $('#roPl').textContent = fmt(j.loss, 0); $('#roHz').textContent = j.hersey.toExponential(2); $('#roTo').textContent = fmt(s.T, 0); $('#roMu').textContent = fmt(j.mu * 1000, 1); $('#roTBox').classList.toggle('alarm', s.T > T_LIMIT); $('#roP').textContent = fmt(j.P, 2); $('#roPBox').classList.toggle('alarm', j.P > P_LIMIT); $('#roQ').textContent = fmt(j.Q, 2); $('#roQs').textContent = s.Qs + ' %'; $('#roC').textContent = fmt(j.c * 1e6, 0); $('#roCR').textContent = s.cr.toFixed(4); $('#roV').textContent = fmt(j.U, 1); $('#roN').textContent = s.N;
    const pr = $('#pillRegime'); pr.textContent = j.regime === 'hydro' ? 'hydrodynamic film' : j.regime + ' lubrication'; pr.className = 'pill regime ' + j.regime; $('#pillWipe').classList.toggle('hidden', !(j.lam < 0.5 && s.run) && !s.wiped); $('#pillHot').classList.toggle('hidden', s.T <= T_LIMIT); $('#pillPmax').classList.toggle('hidden', j.P <= P_LIMIT);
  } else {
    $('#rrP').textContent = fmt(j.P, 2); $('#rrX').textContent = j.X; $('#rrY').textContent = j.Y; $('#rrL10').textContent = fmt(j.L10h, 0); $('#rrL10r').textContent = fmt(j.L10, 1); $('#rrLBox').classList.toggle('warnv', j.L10h < 10000); $('#rrLnm').textContent = fmt(j.Lnm, 0); $('#rra1').textContent = s.rel; $('#rraiso').textContent = fmt(j.aiso, 2); $('#rrCP').textContent = fmt(j.b.C / j.P, 1); $('#rrC').textContent = j.b.C; $('#rrC0').textContent = j.b.C0; $('#rrK').textContent = fmt(j.kappa, 2);
    $('#rrM').textContent = fmt(j.M, 0); $('#rrPl').textContent = fmt(j.loss, 0); $('#rrT').textContent = fmt(s.T, 0); $('#rrTlim').textContent = j.Tlim; $('#rrTBox').classList.toggle('alarm', s.T > j.Tlim); $('#rrN').textContent = s.N; $('#rrNlim').textContent = j.nlim; $('#rrNBox').classList.toggle('alarm', s.N > j.nlim); $('#rrBPFO').textContent = fmt(j.BPFO, 1); $('#rrBPFI').textContent = fmt(j.BPFI, 1); $('#rrBSF').textContent = fmt(j.BSF, 1); $('#rrFTF').textContent = fmt(j.FTF, 2); $('#rrMin').textContent = j.minOK ? 'OK' : 'too light'; $('#rrMinBox').classList.toggle('alarm', !j.minOK);
    const pr = $('#pillRegime'); const reg = j.kappa < 0.4 ? 'boundary' : j.kappa < 1 ? 'mixed' : 'hydro'; pr.textContent = `κ ${fmt(j.kappa, 2)} · ${reg === 'hydro' ? 'full film' : reg} lubrication`; pr.className = 'pill regime ' + reg; $('#pillWipe').classList.add('hidden'); $('#pillHot').classList.toggle('hidden', s.T <= j.Tlim); $('#pillPmax').classList.toggle('hidden', j.P <= j.b.C0); $('#pillPmax').textContent = 'Load above static rating C₀';
  }
  $('#btnRun').textContent = s.run ? 'Running' : 'Start';
}
function record(tag = '') {
  const s = state, j = R, r = s.mode === 'journal' ? { i: s.log.length + 1, tag, mode: 'journal', brg: `Ø${s.D}×${fmt(s.LD * s.D, 0)} c/R ${s.cr}`, W: s.W, Fa: '', N: s.N, oil: s.oil, T: +s.T.toFixed(0), mu: +(j.mu * 1000).toFixed(1), a: +j.S.toFixed(3), b: +j.e.toFixed(3), c: +(j.h0 * 1e6).toFixed(1), d: +j.lam.toFixed(2), e: +j.f.toFixed(4), loss: +j.loss.toFixed(0), reg: j.regime, cr: s.cr } : { i: s.log.length + 1, tag, mode: 'rolling', brg: j.b.name, W: s.W, Fa: s.Fa, N: s.N, oil: s.oil, T: +s.T.toFixed(0), mu: +(j.nu).toFixed(1), a: +j.kappa.toFixed(2), b: +j.P.toFixed(2), c: +j.L10h.toFixed(0), d: +j.Lnm.toFixed(0), e: +j.M.toFixed(0), loss: +j.loss.toFixed(0), reg: 'κ ' + fmt(j.kappa, 2), cont: s.cont };
  s.log.push(r); renderLog(); checkTasks();
}
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.mode, r.brg, r.W, r.Fa, r.N, r.oil, r.T, r.mu, r.a, r.b, r.c, r.d, r.e, r.loss, r.reg].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,mode,bearing,W_kN,Fa_kN,rpm,oil,T_C,visc,S_or_kappa,eps_or_P,h0um_or_L10h,lambda_or_Lnm,f_or_M,loss_W,regime\n' + state.log.map(r => [r.i, r.tag, r.mode, r.brg, r.W, r.Fa, r.N, r.oil, r.T, r.mu, r.a, r.b, r.c, r.d, r.e, r.loss, r.reg].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'regimes', t: 'Cross the Stribeck curve', d: 'Journal mode: record one reading in boundary lubrication (λ < 1) and one with a full film (λ > 3). The speed sweep will take you across.', ok: () => state.log.some(r => r.mode === 'journal' && r.reg === 'boundary') && state.log.some(r => r.mode === 'journal' && r.reg === 'hydro') },
  { id: 'oil', t: 'Heavier oil, thicker film, hotter bearing', d: 'Record the same load and speed with two oil grades. Compare film thickness and film temperature.', ok: () => { const L = state.log.filter(r => r.mode === 'journal'); return L.some(a => L.some(b => a.oil !== b.oil && a.W === b.W && a.N === b.N)); } },
  { id: 'load', t: 'Double the load', d: 'Record a journal reading, double the load, record again: film thickness falls, friction coefficient falls, power loss rises.', ok: () => { const L = state.log.filter(r => r.mode === 'journal'); return L.some(a => L.some(b => Math.abs(b.W - 2 * a.W) < 0.3 && a.N === b.N && a.oil === b.oil)); } },
  { id: 'clear', t: 'Clearance', d: 'Record two clearance ratios at the same conditions. Tight clearance gives a thinner film and more heat; loose gives less load capacity.', ok: () => { const L = state.log.filter(r => r.mode === 'journal'); return L.some(a => L.some(b => a.cr !== b.cr && a.W === b.W && a.N === b.N && a.oil === b.oil)); } },
  { id: 'wipe', t: 'Lose the oil supply', d: 'Cut the oil supply while running: watch the film temperature run away and the bearing wipe.', ok: () => state.log.some(r => r.tag === 'wiped' || r.tag === 'overheated') },
  { id: 'cube', t: 'The cube law', d: 'Rolling mode: record a bearing at one radial load and at double the load. L10 should fall eight times for a ball bearing.', ok: () => { const L = state.log.filter(r => r.mode === 'rolling'); return L.some(a => L.some(b => a.brg === b.brg && Math.abs(b.W - 2 * a.W) < 0.3 && a.Fa === b.Fa && a.N === b.N)); } },
  { id: 'axial', t: 'Axial load on a ball bearing', d: 'Put an axial load on a deep-groove ball bearing until Fa/Fr exceeds e: the equivalent load jumps and life collapses. Record it.', ok: () => state.log.some(r => r.mode === 'rolling' && r.Fa > 0 && r.b > r.W + 0.05) },
  { id: 'contam', t: 'Clean versus dirty', d: 'Record the same rolling bearing with normal cleanliness and heavy contamination: compare the modified life.', ok: () => { const L = state.log.filter(r => r.mode === 'rolling'); return L.some(a => L.some(b => a.brg === b.brg && a.W === b.W && a.cont !== b.cont)); } },
  { id: 'speed', t: 'Speed and viscosity', d: 'Run a rolling bearing past its speed limit, or with too heavy an oil at high speed, until the temperature alarm shows.', ok: () => state.flags.hot },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { if (state.mode === 'rolling' && state.run && state.T > R.Tlim) state.flags.hot = true; for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt; step(dt); drawRig(dt); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawChart(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setMode(m) { state.mode = m; document.querySelectorAll('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === m)); document.querySelectorAll('.jrn').forEach(e => e.classList.toggle('hidden', m !== 'journal')); document.querySelectorAll('.rol').forEach(e => e.classList.toggle('hidden', m !== 'rolling')); $('#grpJ').textContent = m === 'journal' ? 'Journal bearing geometry' : 'Rolling bearing'; $('#ctlFa').style.display = m === 'rolling' ? '' : 'none'; $('#roJ').classList.toggle('hidden', m !== 'journal'); $('#roR').classList.toggle('hidden', m !== 'rolling'); $('#rigTitle').textContent = m === 'journal' ? 'Journal bearing · eccentric journal, oil film and pressure lobe' : 'Rolling-element bearing · races, elements and cage'; $('#chartTitle').textContent = m === 'journal' ? 'Stribeck curve · friction coefficient against μN/P' : 'Life against load · L10 and modified life for this bearing'; state.trace = []; state.sweepPts = []; state.sweep = null; state.wiped = false; $('#btnRepair').classList.add('hidden'); }
const PRESETS = {
  design: () => { setMode('journal'); for (const [k, v] of Object.entries({ W: 5, N: 1500, D: 60, LD: 1, cr: 0.0015, Ra: 0.8, Tin: 40, Qs: 100 })) setCtl(k, v); $('#oil').value = '46'; state.oil = '46'; state.run = true; },
  slow: () => { PRESETS.design(); setCtl('N', 30); setCtl('W', 40); $('#oil').value = '22'; state.oil = '22'; },
  thin: () => { PRESETS.design(); $('#oil').value = '22'; state.oil = '22'; setCtl('Tin', 70); },
  starve: () => { PRESETS.design(); setCtl('Qs', 0); },
  tight: () => { PRESETS.design(); setCtl('cr', 0.0006); setCtl('N', 4000); },
  roll: () => { setMode('rolling'); $('#brg').value = '6208'; state.brg = '6208'; for (const [k, v] of Object.entries({ W: 6.5, Fa: 0, N: 1500, Tin: 40 })) setCtl(k, v); $('#oil').value = '46'; state.oil = '46'; $('#cont').value = '0.5'; state.cont = 0.5; state.run = true; },
  axial: () => { PRESETS.roll(); setCtl('W', 3); setCtl('Fa', 3); },
  dirty: () => { PRESETS.roll(); $('#oil').value = 'grease'; state.oil = 'grease'; $('#cont').value = '0.05'; state.cont = 0.05; },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('W', v => v + ' kN'); bindR('N', v => v.toLocaleString().replace(/,/g, ' ') + ' rpm'); bindR('Fa', v => v + ' kN'); bindR('D', v => v + ' mm'); bindR('LD', v => v.toFixed(2)); bindR('cr', v => v.toFixed(4)); bindR('Ra', v => v.toFixed(1) + ' µm'); bindR('Tin', v => v + ' °C'); bindR('Qs', v => v + ' %'); bindR('tscale', v => '×' + v);
  $('#brg').addEventListener('change', e => state.brg = e.target.value); $('#cont').addEventListener('change', e => state.cont = +e.target.value); $('#rel').addEventListener('change', e => state.rel = +e.target.value); $('#oil').addEventListener('change', e => state.oil = e.target.value);
  document.querySelectorAll('#modes button').forEach(b => b.onclick = () => setMode(b.dataset.mode));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => { state.run = false; state.sweep = null; }; $('#btnSweep').onclick = () => { state.run = true; state.sweepPts = []; state.sweep = { t: 0 }; }; $('#btnRecord').onclick = () => record(''); $('#btnRepair').onclick = () => { state.wiped = false; state.wipeT = 0; state.T = state.Tin; $('#btnRepair').classList.add('hidden'); };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('br_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') state.run = !state.run; else if (k === 'R') record(''); else if (k === '1') setMode('journal'); else if (k === '2') setMode('rolling'); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawChart(); drawTrace(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.BearingRig = {
  getState: () => ({ inputs: { mode: state.mode, W: state.W, N: state.N, Fa: state.Fa, D: state.D, LD: state.LD, cr: state.cr, Ra: state.Ra, brg: state.brg, cont: state.cont, rel: state.rel, oil: state.oil, Tin: state.Tin, Qs: state.Qs, run: state.run }, results: state.mode === 'journal' ? { S: R.S, eps: R.e, h0_um: R.h0 * 1e6, lambda: R.lam, f: R.f, loss_W: R.loss, T: state.T, P_MPa: R.P, regime: R.regime, wiped: state.wiped } : { P: R.P, L10h: R.L10h, Lnm: R.Lnm, kappa: R.kappa, M: R.M, loss_W: R.loss, T: state.T, BPFO: R.BPFO, BPFI: R.BPFI, BSF: R.BSF, FTF: R.FTF }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.mode) setMode(inp.mode); for (const k of ['W', 'N', 'Fa', 'D', 'LD', 'cr', 'Ra', 'Tin', 'Qs', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); for (const k of ['brg', 'oil']) if (inp[k] !== undefined) { $('#' + k).value = inp[k]; state[k] = inp[k]; } for (const k of ['cont', 'rel']) if (inp[k] !== undefined) { $('#' + k).value = inp[k]; state[k] = +inp[k]; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.sweep) $('#btnSweep').click(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'bearing-lube-rig', title: 'Bearing and Lubrication Rig', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.BearingRig.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'bearing-lube-rig', state: window.BearingRig.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('br_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setMode('journal'); step(0); drawRig(0); updateReadouts(); drawChart(); drawTrace();
  requestAnimationFrame(tick);
}
build();
