/* Fatigue and Stress Concentration Bench — stepped shaft (fillet) in bending or axial load, or a plate with a hole,
   under cyclic load. Kt from curve fits, Peterson notch sensitivity to Kf, Marin factors to the endurance limit,
   S-N curve (Basquin between 10³ and 10⁶, knee for steel, none for aluminium), mean-stress criteria on the Goodman
   diagram, Miner damage on a cycle clock and Paris-law crack growth to fracture. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const fmtN = N => !Number.isFinite(N) ? '∞' : N < 1e3 ? '< 10³' : N >= 1e9 ? '> 10⁹' : N.toExponential(2).replace('e+', '×10^');
const MATS = {
  s1045: { name: 'Steel 1045 normalised', Su: 620, Sy: 400, al: false, Kc: 60, C: 6.9e-12, m: 3 },
  s4340: { name: 'Steel 4340 Q&T', Su: 1100, Sy: 950, al: false, Kc: 80, C: 1.36e-10, m: 2.25 },
  s304: { name: 'Stainless 304 annealed', Su: 580, Sy: 240, al: false, Kc: 150, C: 6.9e-12, m: 3 },
  al6061: { name: 'Aluminium 6061-T6', Su: 310, Sy: 275, al: true, Sf: 96, Kc: 30, C: 1.5e-11, m: 3 },
  al7075: { name: 'Aluminium 7075-T6', Su: 570, Sy: 500, al: true, Sf: 160, Kc: 25, C: 2.5e-11, m: 3 },
};
const FINISH = { ground: { name: 'Ground', a: 1.58, b: -0.085 }, machined: { name: 'Machined / cold drawn', a: 4.51, b: -0.265 }, hot: { name: 'Hot rolled', a: 57.7, b: -0.718 }, forged: { name: 'As forged', a: 272, b: -0.995 } };
const REL = { 50: 1, 90: 0.897, 95: 0.868, 99: 0.814, 99.9: 0.753, 99.99: 0.702 };
const KT_BEND = [[6, 0.87868, -0.33243], [3, 0.89334, -0.30860], [2, 0.90879, -0.28598], [1.5, 0.93836, -0.25759], [1.2, 0.97098, -0.21796], [1.1, 0.95120, -0.23757], [1.05, 0.98137, -0.19653], [1.02, 0.96048, -0.17711]];
const KT_AX = [[3, 1.0122, -0.3092], [2, 1.01470, -0.30035], [1.5, 0.99957, -0.28221], [1.2, 0.96272, -0.25548], [1.1, 0.98450, -0.20173], [1.05, 0.98470, -0.17451], [1.02, 0.97700, -0.13248]];
const AI = 0.25, YC = 1.12;

/* ------------------------------------------------------------------ state */
const state = { geo: 'bend', mat: 's1045', d: 25, Dd: 1.5, r: 0.5, w: 50, t: 8, dh: 10, finish: 'machined', rel: 50, crit: 'goodman', La: 300, Lm: 0, f: 10, tscale: 100, run: false,
  n: 0, D: 0, broken: false, t: 0, phase: 0, log: [], flags: {}, hist: [], histAcc: 0, specimens: 1 };
const R = {};

/* ------------------------------------------------------------------ model */
function ktFit(tab, Dd, rd) { let lo = tab[tab.length - 1], hi = tab[0]; for (let i = 0; i < tab.length - 1; i++) if (Dd <= tab[i][0] && Dd >= tab[i + 1][0]) { hi = tab[i]; lo = tab[i + 1]; } const f = hi[0] === lo[0] ? 0 : (Dd - lo[0]) / (hi[0] - lo[0]), A = lo[1] + f * (hi[1] - lo[1]), b = lo[2] + f * (hi[2] - lo[2]); return Math.max(1, A * Math.pow(clamp(rd, 0.005, 0.3), b)); }
function solve() {
  const s = state, m = MATS[s.mat], geo = s.geo; let Kt, rn, snomA, snomM, area, Se0, kb, kc, label, unit;
  if (geo === 'hole') { const dw = clamp(s.dh / s.w, 0.02, 0.8); Kt = 3.00 - 3.13 * dw + 3.66 * dw * dw - 1.53 * dw ** 3; rn = s.dh / 2; area = (s.w - s.dh) * s.t; snomA = s.La * 1000 / area; snomM = s.Lm * 1000 / area; kb = 1; kc = 0.85; label = 'P / ((w − d) t)'; unit = 'kN'; }
  else if (geo === 'axial') { Kt = ktFit(KT_AX, s.Dd, s.r / s.d); rn = s.r; area = Math.PI * s.d * s.d / 4; snomA = s.La * 1000 / area; snomM = s.Lm * 1000 / area; kb = 1; kc = 0.85; label = '4P / (π d²)'; unit = 'kN'; }
  else { Kt = ktFit(KT_BEND, s.Dd, s.r / s.d); rn = s.r; const Z = Math.PI * s.d ** 3 / 32; snomA = s.La * 1000 / Z; snomM = s.Lm * 1000 / Z; kb = s.d <= 51 ? 1.24 * Math.pow(s.d, -0.107) : 1.51 * Math.pow(s.d, -0.157); kc = 1; label = '32M / (π d³)'; unit = 'N·m'; }
  const aP = m.al ? 0.5 : 0.0254 * Math.pow(2070 / m.Su, 1.8), q = 1 / (1 + aP / Math.max(rn, 0.05)), Kf = 1 + q * (Kt - 1);
  const ka = Math.min(1, FINISH[s.finish].a * Math.pow(m.Su, FINISH[s.finish].b)), ke = REL[s.rel] || 1; Se0 = m.al ? m.Sf : Math.min(0.5 * m.Su, 700); const Se = ka * kb * kc * ke * Se0;
  let sa = Kf * snomA, sm = Kf * snomM, yieldNotch = false; if (Kf * (snomA + Math.abs(snomM)) > m.Sy) { yieldNotch = true; sm = Math.sign(snomM) * Math.max(0, m.Sy - Kf * snomA); }
  const Suk = m.Su / 6.895, f = Suk < 70 ? 0.9 : clamp(1.06 - 2.8e-3 * Suk + 6.9e-6 * Suk * Suk, 0.7, 0.9), Nk = m.al ? 5e8 : 1e6;
  const bS = Math.log10(Se / (f * m.Su)) / Math.log10(Nk / 1e3), aS = f * m.Su / Math.pow(1e3, bS);
  const smp = Math.max(sm, 0); let sar, nf, ny = (sa + Math.abs(sm)) > 0 ? m.Sy / (sa + Math.abs(sm)) : Infinity;
  const c = s.crit; const gm = smp / m.Su, gy = smp / m.Sy;
  if (c === 'goodman') { sar = gm >= 1 ? Infinity : sa / (1 - gm); nf = sa / Se + gm > 0 ? 1 / (sa / Se + gm) : Infinity; }
  else if (c === 'gerber') { sar = gm >= 1 ? Infinity : sa / (1 - gm * gm); nf = smp <= 0 ? (sa > 0 ? Se / sa : Infinity) : sa > 0 ? 0.5 * (m.Su / smp) ** 2 * (sa / Se) * (-1 + Math.sqrt(1 + (2 * smp * Se / (m.Su * sa)) ** 2)) : Infinity; }
  else if (c === 'soderberg') { sar = gy >= 1 ? Infinity : sa / (1 - gy); nf = sa / Se + gy > 0 ? 1 / (sa / Se + gy) : Infinity; }
  else { sar = gy >= 1 ? Infinity : sa / Math.sqrt(1 - gy * gy); nf = sa > 0 || smp > 0 ? 1 / Math.sqrt((sa / Se) ** 2 + gy * gy) : Infinity; }
  let N; if (!Number.isFinite(sar)) N = 0; else if (sar <= 0) N = Infinity; else if (!m.al && sar <= Se) N = Infinity; else if (sar >= f * m.Su) N = 1e3 * Math.pow(f * m.Su / sar, 6); else N = Math.pow(sar / aS, 1 / bS);
  const smax = snomM + snomA, dS = Kf * Math.max(0, Math.min(2 * snomA, smax)); // effective stress range at the notch: the tensile part of the cycle
  const ac = smax > 0 ? Math.min((m.Kc / (YC * smax)) ** 2 / Math.PI * 1000, geo === 'hole' ? (s.w - s.dh) / 2 : s.d / 2) : Infinity;
  const mm = m.m;
  const Np = dS > 0 && Number.isFinite(ac) && ac > AI ? (Math.pow(AI / 1000, 1 - mm / 2) - Math.pow(ac / 1000, 1 - mm / 2)) / ((mm / 2 - 1) * m.C * Math.pow(YC * dS * Math.sqrt(Math.PI), mm)) : Infinity;
  const D0 = Number.isFinite(N) && Number.isFinite(Np) ? clamp(1 - Np / N, 0, 0.95) : 0;
  return { Kt, q, Kf, aP, ka, kb, kc, ke, Se0, Se, f, aS, bS, snomA, snomM, sa, sm, sar, nf, ny, N, hours: Number.isFinite(N) ? N / s.f / 3600 : Infinity, yieldNotch, staticFail: smax * Kf >= m.Su || sar === Infinity, label, unit, area, rn, smax, dS, ac, Np, D0, m, Nk };
}
function crackAt(D, r) { if (!Number.isFinite(r.N) || D <= r.D0) return 0; const mm = r.m.m, n = Number.isFinite(r.Np) ? (D - r.D0) / (1 - r.D0) * r.Np : 0, base = Math.pow(AI / 1000, 1 - mm / 2) - (mm / 2 - 1) * r.m.C * Math.pow(YC * r.dS * Math.sqrt(Math.PI), mm) * n; if (base <= 0) return r.ac; return Math.min(r.ac, Math.pow(base, 1 / (1 - mm / 2)) * 1000); }
function step(dt) {
  const s = state; Object.assign(R, solve());
  if (s.run && !s.broken) { const dn = s.f * dt * s.tscale; s.n += dn; s.phase += s.f * dt * 2 * Math.PI * 0.3; if (Number.isFinite(R.N) && R.N > 0) s.D += dn / R.N; else if (R.N === 0) s.D = 1; if (s.D >= 1) { s.D = 1; s.broken = true; s.run = false; s.flags.fracture = true; record('fractured'); } }
  s.t += dt; R.a = crackAt(s.D, R); if (R.a >= 1) s.flags.crack = true; if (R.yieldNotch) s.flags.yield = true;
  s.histAcc += dt; if (s.histAcc >= 0.2) { s.histAcc = 0; s.hist.push({ n: s.n, a: R.a, D: s.D }); if (s.hist.length > 1200) s.hist.shift(); }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg;
function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Fatigue specimen' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); const d = el('defs', {}, svg); const g = el('linearGradient', { id: 'stressG', x1: 0, x2: 1, y1: 0, y2: 0 }, d); el('stop', { offset: '0', 'stop-color': 'var(--alarm)', 'stop-opacity': 0 }, g); el('stop', { offset: '0.5', 'stop-color': 'var(--alarm)', 'stop-opacity': 0.75 }, g); el('stop', { offset: '1', 'stop-color': 'var(--alarm)', 'stop-opacity': 0 }, g); }
const arrow = (x1, y1, x2, y2, cls, p) => { el('line', { x1, y1, x2, y2, class: cls }, p); const a = Math.atan2(y2 - y1, x2 - x1); el('polygon', { points: `${x2},${y2} ${x2 - 10 * Math.cos(a - 0.4)},${y2 - 10 * Math.sin(a - 0.4)} ${x2 - 10 * Math.cos(a + 0.4)},${y2 - 10 * Math.sin(a + 0.4)}`, class: cls.replace('arrow', 'head') }, p); };
function drawRig() {
  svg.innerHTML = ''; buildDefs(); const s = state, r = R, m = MATS[s.mat], cy = 170, ph = Math.sin(s.phase), load = s.Lm + s.La * ph, ampl = s.La > 0 ? load / s.La : 0, hot = clamp((r.sa + Math.abs(r.sm)) / m.Sy, 0, 1), ax = 3 * Math.max(1, Math.min(6, s.d / 10)); // crack drawn ×ax
  if (s.geo === 'hole') {
    const kk = Math.min(2.6, 140 / s.w), W = s.w * kk, L = 520, x0 = 190, hr = s.dh / 2 * kk, cx = x0 + L / 2;
    el('rect', { x: x0, y: cy - W / 2, width: L, height: W, class: 'spec' }, svg); el('circle', { cx, cy, r: hr, class: 'hole' }, svg);
    for (const sg of [-1, 1]) el('rect', { x: cx + sg * hr - (sg < 0 ? 40 : 0), y: cy - W / 2, width: 40, height: W, fill: 'url(#stressG)', opacity: hot }, svg);
    const ac = r.a * kk * ax; if (ac > 0) { el('line', { x1: cx + hr, y1: cy, x2: cx + hr + ac, y2: cy, class: 'crack' }, svg); el('line', { x1: cx - hr, y1: cy, x2: cx - hr - ac, y2: cy, class: 'crack' }, svg); }
    if (s.broken) { el('line', { x1: cx, y1: cy - W / 2 - 4, x2: cx, y2: cy + W / 2 + 4, class: 'fracture' }, svg); }
    const al = 40 + 50 * ampl; arrow(x0 - 10, cy, x0 - 10 - al, cy, 'farrow', svg); arrow(x0 + L + 10, cy, x0 + L + 10 + al, cy, 'farrow', svg); txt(x0 + L / 2, cy - W / 2 - 14, `P = ${fmt(load, 0)} kN now · Pa ${s.La} ± · Pm ${s.Lm}`, 'ftxt', svg, 'middle');
    txt(x0 + L / 2, cy + W / 2 + 22, `plate ${s.w} × ${s.t} mm · hole Ø${s.dh} · d/w ${fmt(s.dh / s.w, 2)} · Kt ${fmt(r.Kt, 2)} on the net section`, 'lbl', svg, 'middle');
  } else {
    const Dmm = s.d * s.Dd, k = Math.min(3, 150 / Dmm), hD = Dmm * k / 2, hd = s.d * k / 2, rf = Math.max(2, s.r * k), x0 = 150, xs = 430, x1 = 810;
    const path = `M${x0},${cy - hD} L${xs - rf},${cy - hD} L${xs - rf},${cy - hd - rf} A${rf},${rf} 0 0 0 ${xs},${cy - hd} L${x1},${cy - hd} L${x1},${cy + hd} L${xs},${cy + hd} A${rf},${rf} 0 0 0 ${xs - rf},${cy + hd + rf} L${xs - rf},${cy + hD} L${x0},${cy + hD} Z`;
    el('path', { d: path, class: 'spec' }, svg); el('rect', { x: xs - 40, y: cy - hD, width: 60, height: 2 * hD, fill: 'url(#stressG)', opacity: hot }, svg);
    el('line', { x1: x0 - 20, y1: cy, x2: x1 + 20, y2: cy, class: 'cl' }, svg);
    const ac = r.a * k * ax; if (ac > 0) { el('line', { x1: xs, y1: cy - hd, x2: xs, y2: cy - hd + ac, class: 'crack' }, svg); if (s.geo === 'bend') el('line', { x1: xs, y1: cy + hd, x2: xs, y2: cy + hd - ac * 0.4, class: 'crack' }, svg); }
    if (s.broken) el('line', { x1: xs, y1: cy - hD - 4, x2: xs, y2: cy + hD + 4, class: 'fracture' }, svg);
    if (s.geo === 'bend') { const mo = 24 + 30 * Math.abs(ampl), sg = ampl >= 0 ? 1 : -1; el('path', { d: `M${x1 + 30},${cy - mo} A${mo},${mo} 0 0 ${sg > 0 ? 1 : 0} ${x1 + 30},${cy + mo}`, class: 'farrow' }, svg); el('polygon', { points: sg > 0 ? `${x1 + 30},${cy + mo + 2} ${x1 + 22},${cy + mo - 8} ${x1 + 38},${cy + mo - 8}` : `${x1 + 30},${cy - mo - 2} ${x1 + 22},${cy - mo + 8} ${x1 + 38},${cy - mo + 8}`, class: 'fhead' }, svg); el('path', { d: `M${x0 - 30},${cy - mo} A${mo},${mo} 0 0 ${sg > 0 ? 0 : 1} ${x0 - 30},${cy + mo}`, class: 'farrow' }, svg); txt(480, cy - hD - 26, `M = ${fmt(load, 0)} N·m now · Ma ${s.La} ± · Mm ${s.Lm}`, 'ftxt', svg, 'middle'); }
    else { const al = 40 + 50 * ampl; arrow(x0 - 10, cy, x0 - 10 - al, cy, 'farrow', svg); arrow(x1 + 10, cy, x1 + 10 + al, cy, 'farrow', svg); txt(480, cy - hD - 26, `P = ${fmt(load, 0)} kN now · Pa ${s.La} ± · Pm ${s.Lm}`, 'ftxt', svg, 'middle'); }
    el('line', { x1: xs - rf - 14, y1: cy - hd - rf - 14, x2: xs - rf, y2: cy - hd - rf, class: 'dim' }, svg); txt(xs - rf - 18, cy - hd - rf - 18, `r ${s.r} mm`, 'val', svg, 'end');
    txt(x0 + 20, cy + hD + 20, `D ${fmt(Dmm, 1)} mm`, 'lbl', svg); txt(x1 - 20, cy + hd + 20, `d ${s.d} mm`, 'lbl', svg, 'end'); txt(480, cy + hD + 42, `stepped shaft · D/d ${s.Dd} · r/d ${fmt(s.r / s.d, 3)} · Kt ${fmt(r.Kt, 2)} · ${s.geo === 'bend' ? 'bending' : 'axial'}`, 'lbl', svg, 'middle');
  }
  txt(480, 300, `nominal ${fmt(r.snomA, 0)} ± at mean ${fmt(r.snomM, 0)} MPa → at the notch σa ${fmt(r.sa, 0)} · σm ${fmt(r.sm, 0)} MPa (Kf ${fmt(r.Kf, 2)}) · crack drawn ×${ax}`, 'val', svg, 'middle');
  const bx = 850; el('rect', { x: bx, y: 40, width: 26, height: 240, rx: 4, class: 'tbar' }, svg); const hD2 = clamp(s.D, 0, 1) * 240; el('rect', { x: bx, y: 280 - hD2, width: 26, height: hD2, class: 'tfill' + (s.D > 0.8 ? ' bad' : '') }, svg); txt(bx + 13, 30, 'damage', 'lbl', svg, 'middle'); txt(bx + 13, 298, `${fmt(s.D * 100, 0)} %`, 'ftxt', svg, 'middle');
  const yD0 = 280 - r.D0 * 240; el('line', { x1: bx - 6, y1: yD0, x2: bx + 32, y2: yD0, class: 'farrow', 'stroke-dasharray': '4 3' }, svg); txt(bx + 36, yD0 + 4, 'crack', 'lbl', svg);
  const cx2 = 910; el('rect', { x: cx2, y: 40, width: 26, height: 240, rx: 4, class: 'tbar' }, svg); const ha = Number.isFinite(r.ac) && r.ac > 0 ? clamp(r.a / r.ac, 0, 1) * 240 : 0; el('rect', { x: cx2, y: 280 - ha, width: 26, height: ha, fill: 'var(--alarm)', opacity: 0.8 }, svg); txt(cx2 + 13, 30, 'a / a c', 'lbl', svg, 'middle'); txt(cx2 + 13, 298, `${fmt(r.a, 2)} mm`, 'ftxt', svg, 'middle');
  txt(480, 340, `cycles ${Math.round(s.n).toLocaleString().replace(/,/g, ' ')} of ${Number.isFinite(r.N) ? Math.round(r.N).toLocaleString().replace(/,/g, ' ') : '∞'} · ${s.f} Hz × clock ${s.tscale} · specimen ${s.specimens}`, 'val', svg, 'middle');
  txt(480, 372, s.broken ? `FRACTURED after ${Math.round(s.n).toLocaleString().replace(/,/g, ' ')} cycles — crack grew from ${AI} mm to ${fmt(r.ac, 1)} mm then the ligament tore · New specimen to continue` : r.staticFail ? 'STATIC FAILURE — maximum stress at the notch reaches the tensile strength' : r.yieldNotch ? `NOTCH YIELDS — Kf·σmax exceeds Sy ${m.Sy} MPa: mean stress relaxes to ${fmt(r.sm, 0)} MPa` : !Number.isFinite(r.N) ? `INFINITE LIFE — equivalent reversed stress ${fmt(r.sar, 0)} MPa is below the endurance limit ${fmt(r.Se, 0)} MPa · factor ${fmt(r.nf, 2)}` : `FINITE LIFE ${fmtN(r.N)} cycles (${fmt(r.hours, 1)} h at ${s.f} Hz) · equivalent reversed ${fmt(r.sar, 0)} MPa against Se ${fmt(r.Se, 0)} · ${s.crit} factor ${fmt(r.nf, 2)}`, 'val' + (s.broken || r.staticFail || r.yieldNotch || (Number.isFinite(r.N) && r.N < 1e5) ? ' bad' : ''), svg, 'middle');
}
function buildDefs() { if (svg.querySelector('#stressG')) return; const d = el('defs', {}, svg); const g = el('linearGradient', { id: 'stressG', x1: 0, x2: 1, y1: 0, y2: 0 }, d); el('stop', { offset: '0', 'stop-color': 'var(--alarm)', 'stop-opacity': 0 }, g); el('stop', { offset: '0.5', 'stop-color': 'var(--alarm)', 'stop-opacity': 0.75 }, g); el('stop', { offset: '1', 'stop-color': 'var(--alarm)', 'stop-opacity': 0 }, g); }
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawSN() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 64, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R, m = MATS[s.mat];
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const lx0 = 3, lx1 = 9, X = N => L + (Math.log10(Math.max(N, 1)) - lx0) / (lx1 - lx0) * pw, ly0 = Math.log10(Math.max(20, r.Se * 0.25)), ly1 = Math.log10(m.Su * 1.2), Y = S => T + (1 - (Math.log10(Math.max(S, 1)) - ly0) / (ly1 - ly0)) * ph;
  ctx.strokeStyle = col('--grid'); for (let e = 3; e <= 9; e++) { ctx.beginPath(); ctx.moveTo(X(10 ** e), T); ctx.lineTo(X(10 ** e), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText('10^' + e, X(10 ** e), H - 18); }
  ctx.textAlign = 'right'; for (const S of [50, 100, 200, 300, 500, 700, 1000, 1400]) if (Math.log10(S) > ly0 && Math.log10(S) < ly1) { ctx.beginPath(); ctx.moveTo(L, Y(S)); ctx.lineTo(L + pw, Y(S)); ctx.stroke(); ctx.fillText(S, L - 4, Y(S) + 3); }
  const curve = (Se, color, w, dash, lbl) => { const b = Math.log10(Se / (r.f * m.Su)) / Math.log10(r.Nk / 1e3), a = r.f * m.Su / Math.pow(1e3, b); ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); ctx.moveTo(X(1e3), Y(r.f * m.Su)); if (m.al) { ctx.lineTo(X(1e9), Y(a * Math.pow(1e9, b))); } else { ctx.lineTo(X(1e6), Y(Se)); ctx.lineTo(X(1e9), Y(Se)); } ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = color; ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(lbl, X(1e9) - 4, Y(m.al ? a * Math.pow(1e9, b) : Se) - 5); };
  curve(r.Se0, col('--steel'), 1.4, [5, 4], `polished smooth specimen · Se′ ${fmt(r.Se0, 0)} MPa`); curve(r.Se, col('--water'), 2.4, [], `this part (Marin factors) · Se ${fmt(r.Se, 0)} MPa`);
  if (Number.isFinite(r.sar) && r.sar > 0) { const y = Y(r.sar); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 1.4; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke(); ctx.setLineDash([]);
    if (Number.isFinite(r.N)) { ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(X(r.N), y, 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); }
    if (s.n > 0) { ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(s.n), y, 5, 0, 7); ctx.fill(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(X(s.n), y); ctx.stroke(); }
    ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`σar ${fmt(r.sar, 0)} MPa → ${Number.isFinite(r.N) ? 'N ' + fmtN(r.N) : 'infinite life'}`, L + 6, y - 6); }
  for (const p of s.log) if (Number.isFinite(p.N) && p.N > 0 && p.sar > 0) { ctx.fillStyle = col('--muted'); ctx.beginPath(); ctx.arc(X(p.N), Y(p.sar), 3, 0, 7); ctx.fill(); }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('cycles to failure N (log) · completely reversed stress amplitude MPa (log) · red dot = cycle clock · brass = predicted failure', L + pw / 2, H - 4);
}
function drawGoodman() {
  const cv = $('#gchart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R, m = MATS[s.mat];
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const xm = m.Su * 1.08, ym = Math.max(r.Se * 1.3, m.Sy * 1.05, r.sa * 1.1), X = v => L + v / xm * pw, Y = v => T + (1 - clamp(v / ym, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); const gs = xm > 800 ? 200 : 100; for (let v = 0; v <= xm; v += gs) { ctx.beginPath(); ctx.moveTo(X(v), T); ctx.lineTo(X(v), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(v, X(v), H - 18); } for (let v = 0; v <= ym; v += gs) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(v, L - 4, Y(v) + 3); }
  const lineTo = (pts, color, w, dash, on) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = on ? w + 1.2 : w; ctx.setLineDash(dash); pts.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke(); ctx.setLineDash([]); };
  const ger = [], ell = []; for (let i = 0; i <= 40; i++) { const x = m.Su * i / 40; ger.push([x, r.Se * (1 - (x / m.Su) ** 2)]); if (x <= m.Sy) ell.push([x, r.Se * Math.sqrt(Math.max(0, 1 - (x / m.Sy) ** 2))]); }
  lineTo([[0, r.Se], [m.Su, 0]], col('--water'), 1.6, [], s.crit === 'goodman'); lineTo(ger, col('--good'), 1.4, [], s.crit === 'gerber'); lineTo([[0, r.Se], [m.Sy, 0]], col('--brass'), 1.4, [], s.crit === 'soderberg'); lineTo(ell, '#7a5a2a', 1.4, [], s.crit === 'asme'); lineTo([[0, m.Sy], [m.Sy, 0]], col('--alarm'), 1.4, [6, 3], false);
  ctx.font = '600 10px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('Goodman', X(m.Su * 0.55) + 4, Y(r.Se * 0.45) + 13); ctx.fillStyle = col('--good'); ctx.fillText('Gerber', X(m.Su * 0.3), Y(r.Se * (1 - 0.09)) - 6); ctx.fillStyle = col('--brass'); ctx.fillText('Soderberg', X(m.Sy * 0.55), Y(r.Se * 0.45) + 13); ctx.fillStyle = '#7a5a2a'; ctx.fillText('ASME-elliptic', X(m.Sy * 0.85) - 30, Y(r.Se * Math.sqrt(1 - 0.7225)) - 8); ctx.fillStyle = col('--alarm'); ctx.fillText('yield (Langer)', X(m.Sy * 0.6) + 4, Y(m.Sy * 0.4) - 4);
  ctx.strokeStyle = col('--steel'); ctx.lineWidth = 1; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(0), Y(0)); const sc = Math.max(r.sm, 0) + r.sa > 0 ? Math.min(xm / Math.max(r.sm, 1e-6), ym / Math.max(r.sa, 1e-6)) : 0; ctx.lineTo(X(Math.max(r.sm, 0) * sc), Y(r.sa * sc)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(Math.max(r.sm, 0)), Y(r.sa), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke();
  for (const p of s.log) { ctx.fillStyle = col('--muted'); ctx.beginPath(); ctx.arc(X(Math.max(p.sm, 0)), Y(p.sa), 3, 0, 7); ctx.fill(); }
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = X(Math.max(r.sm, 0)) > L + pw * 0.6 ? 'right' : 'left'; ctx.fillText(`σm ${fmt(r.sm, 0)} · σa ${fmt(r.sa, 0)} MPa · n_${s.crit} ${fmt(r.nf, 2)} · n_yield ${fmt(r.ny, 2)}`, X(Math.max(r.sm, 0)) + (X(Math.max(r.sm, 0)) > L + pw * 0.6 ? -10 : 10), Y(r.sa) - 10);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`mean stress σm MPa · alternating stress σa MPa at the notch · Se ${fmt(r.Se, 0)} · Sy ${m.Sy} · Su ${m.Su}`, L + pw / 2, H - 4);
}
function drawCrack() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  if (!Number.isFinite(r.N) || !Number.isFinite(r.ac)) { ctx.textAlign = 'center'; ctx.fillText(Number.isFinite(r.ac) ? 'infinite life at this load — no crack initiates' : 'no tensile part of the cycle — no crack growth', L + pw / 2, T + ph / 2); return; }
  const X = n => L + clamp(n / r.N, 0, 1) * pw, Y = a => T + (1 - clamp(a / r.ac, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); for (let f = 0; f <= 1; f += 0.25) { ctx.beginPath(); ctx.moveTo(X(r.N * f), T); ctx.lineTo(X(r.N * f), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(fmt(f * 100, 0) + ' % N', X(r.N * f), H - 6); }
  ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 2; for (let i = 0; i <= 200; i++) { const D = i / 200, a = crackAt(D, r); i ? ctx.lineTo(X(D * r.N), Y(a)) : ctx.moveTo(X(D * r.N), Y(a)); } ctx.stroke();
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(X(r.D0 * r.N), T); ctx.lineTo(X(r.D0 * r.N), T + ph); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(s.D * r.N), Y(r.a), 5, 0, 7); ctx.fill();
  ctx.textAlign = 'right'; ctx.fillStyle = col('--muted'); ctx.fillText(`${fmt(r.ac, 1)} mm`, L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('crack a', L + pw + 4, T + 10); ctx.fillStyle = col('--alarm'); ctx.fillText('initiation', L + pw + 4, T + 24);
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.fillText(`initiation ${fmt(r.D0 * 100, 0)} % of life · propagation ${fmtN(Math.min(r.Np, r.N))} cycles from ${AI} mm to a critical ${fmt(r.ac, 1)} mm · Paris C ${r.m.C.toExponential(1)}, m ${r.m.m}`, L + 6, T + 14);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, m = MATS[s.mat];
  $('#roKt').textContent = fmt(r.Kt, 2); $('#roRn').textContent = fmt(r.rn, 2); $('#roQ').textContent = fmt(r.q, 2); $('#roKf').textContent = fmt(r.Kf, 2); $('#roA').textContent = fmt(r.aP, 3);
  $('#roSe').textContent = fmt(r.Se, 0); $('#roSe0').textContent = fmt(r.Se0, 0); $('#roKa').textContent = fmt(r.ka, 2); $('#roKb').textContent = fmt(r.kb, 2); $('#roKc').textContent = fmt(r.kc, 2); $('#roKe').textContent = fmt(r.ke, 2);
  $('#roSa').textContent = fmt(r.sa, 0); $('#roSm').textContent = fmt(r.sm, 0); $('#roSnom').textContent = `${fmt(r.snomA, 0)} ± ${fmt(r.snomM, 0)}`; $('#roSaBox').classList.toggle('alarm', r.yieldNotch || r.staticFail);
  $('#roNf').textContent = fmt(r.nf, 2); $('#roNy').textContent = fmt(r.ny, 2); $('#roCrit').textContent = { goodman: 'Goodman', gerber: 'Gerber', soderberg: 'Soderberg', asme: 'ASME-elliptic' }[s.crit]; $('#roNfBox').classList.toggle('alarm', r.nf < 1 || r.ny < 1); $('#roNfBox').classList.toggle('warnv', r.nf >= 1 && r.nf < 1.5);
  $('#roSar').textContent = fmt(r.sar, 0); $('#roN').textContent = fmtN(r.N); $('#roHours').textContent = Number.isFinite(r.hours) ? (r.hours > 1e5 ? '> 10⁵' : fmt(r.hours, 1)) : '∞'; $('#roNBox').classList.toggle('warnv', Number.isFinite(r.N)); $('#roNBox').classList.toggle('alarm', Number.isFinite(r.N) && r.N < 1e4);
  $('#roCyc').textContent = s.n >= 1e6 ? (s.n / 1e6).toFixed(2) + ' M' : Math.round(s.n).toLocaleString().replace(/,/g, ' '); $('#roD').textContent = fmt(s.D * 100, 1); $('#roDBox').classList.toggle('alarm', s.D >= 1); $('#roDBox').classList.toggle('warnv', s.D > 0.5 && s.D < 1);
  $('#roCrack').textContent = fmt(r.a, 2); $('#roAc').textContent = Number.isFinite(r.ac) ? fmt(r.ac, 1) : '—'; $('#roCrackBox').classList.toggle('warnv', r.a > 0);
  $('#roRate').textContent = (s.f * s.tscale).toLocaleString().replace(/,/g, ' '); $('#roSpec').textContent = s.specimens; $('#roFrac').textContent = s.log.filter(x => x.tag === 'fractured').length;
  const ps = $('#pillState'); ps.textContent = s.broken ? 'Fractured' : r.staticFail ? 'Static failure' : r.yieldNotch ? 'Notch yielding' : !Number.isFinite(r.N) ? 'Infinite life' : r.a > 0 ? 'Crack growing' : 'Finite life'; ps.className = 'pill ' + (s.broken || r.staticFail ? 'warn' : r.yieldNotch || r.a > 0 ? 'amber' : !Number.isFinite(r.N) ? 'ok' : '');
  $('#pillKf').textContent = `Kt ${fmt(r.Kt, 2)} · Kf ${fmt(r.Kf, 2)} · Se ${fmt(r.Se, 0)} MPa`; $('#pillYield').classList.toggle('hidden', !r.yieldNotch); $('#pillCrack').classList.toggle('hidden', !(r.a > 0 && !s.broken)); $('#pillFrac').classList.toggle('hidden', !s.broken);
  $('#pillRun').textContent = s.run ? 'Cycling' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Cycling' : 'Start cycling';
  $('#rigTitle').textContent = { bend: 'Stepped shaft in bending · fillet at the shoulder', axial: 'Stepped shaft under axial load · fillet at the shoulder', hole: 'Flat plate with a central hole under axial load' }[s.geo]; $('#LaName').textContent = s.geo === 'bend' ? 'Moment amplitude Ma' : 'Load amplitude Pa'; $('#LmName').textContent = s.geo === 'bend' ? 'Mean moment Mm' : 'Mean load Pm';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, geo: s.geo, mat: s.mat, d: s.geo === 'hole' ? s.w + 'x' + s.t + '/' + s.dh : s.d + ' D/d ' + s.Dd, r: s.geo === 'hole' ? s.dh / 2 : s.r, finish: s.finish, rel: s.rel, La: s.La, Lm: s.Lm, Kt: +r.Kt.toFixed(2), q: +r.q.toFixed(2), Kf: +r.Kf.toFixed(2), Se: +r.Se.toFixed(0), sa: +r.sa.toFixed(0), sm: +r.sm.toFixed(0), sar: +r.sar.toFixed(0), crit: s.crit, nf: +r.nf.toFixed(2), ny: +r.ny.toFixed(2), N: r.N, Ntxt: fmtN(r.N), n: Math.round(s.n), a: +r.a.toFixed(2), snomA: +r.snomA.toFixed(0), snomM: +r.snomM.toFixed(0) }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.geo, r.mat, r.d, r.r, r.finish, r.La, r.Lm, r.Kt, r.q, r.Kf, r.Se, r.sa, r.sm, r.sar, r.nf, r.ny, r.Ntxt, r.n, r.a].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,geometry,material,size,notch_radius_mm,finish,amplitude,mean,Kt,q,Kf,Se_MPa,sigma_a_MPa,sigma_m_MPa,sigma_ar_MPa,criterion,n_fatigue,n_yield,N_cycles,cycles_run,crack_mm,nominal_a,nominal_m\n' + state.log.map(r => [r.i, r.tag, r.geo, r.mat, r.d, r.r, r.finish, r.La, r.Lm, r.Kt, r.q, r.Kf, r.Se, r.sa, r.sm, r.sar, r.crit, r.nf, r.ny, Number.isFinite(r.N) ? Math.round(r.N) : 'inf', r.n, r.a, r.snomA, r.snomM].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'fracture', t: 'Run one to fracture', d: 'Start cycling the baseline notched shaft and watch the damage bar, the crack initiate at the fillet and the specimen break. Compare the cycles run with the S-N prediction.', ok: () => state.flags.fracture },
  { id: 'radius', t: 'Fillet radius buys life', d: 'Record the same shaft and load with a fillet radius at least four times larger. The life should improve by more than three times, or go infinite.', ok: () => { const L = state.log.filter(x => x.geo !== 'hole'); return L.some(a => L.some(b => same(a, b, ['geo', 'mat', 'd', 'finish', 'La', 'Lm']) && b.r >= 4 * a.r && (b.N >= 3 * a.N || !Number.isFinite(b.N)) && Number.isFinite(a.N))); } },
  { id: 'finish', t: 'Surface finish', d: 'Record the same part hot-rolled or as-forged and ground. Read what the surface factor ka does to the endurance limit and the life.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['geo', 'mat', 'd', 'r', 'La', 'Lm']) && a.finish === 'ground' && (b.finish === 'hot' || b.finish === 'forged'))); } },
  { id: 'mean', t: 'Mean stress hurts', d: 'Record the same amplitude at zero mean and at a mean load that gives at least 100 MPa mean stress at the notch. Watch the point move right on the Goodman diagram.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['geo', 'mat', 'd', 'r', 'finish', 'La']) && a.sm === 0 && b.sm >= 100)); } },
  { id: 'infinite', t: 'Design for infinite life, efficiently', d: 'Find a load or geometry that gives infinite life with the equivalent reversed stress at least 85 % of Se, and record it.', ok: () => state.log.some(x => !Number.isFinite(x.N) && x.sar >= 0.85 * x.Se && x.sar > 0) },
  { id: 'sens', t: 'Notch sensitivity', d: 'Record the same geometry in two materials whose notch sensitivity q differs by 0.1 or more. The sharper, stronger steel feels the notch more fully.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['geo', 'd', 'r']) && a.mat !== b.mat && Math.abs(a.q - b.q) >= 0.1)); } },
  { id: 'crack', t: 'Watch the crack', d: 'Let a specimen run until the crack passes 1 mm, and note how little of the life is left once a crack is visible.', ok: () => state.flags.crack },
  { id: 'alu', t: 'No endurance limit', d: 'Record an aluminium part with a finite life above 10⁷ cycles: the S-N curve never goes flat, so there is no stress that is safe for ever.', ok: () => state.log.some(x => MATS[x.mat].al && Number.isFinite(x.N) && x.N > 1e7) },
  { id: 'hole', t: 'Hole in a plate', d: 'Record a plate whose hole gives Kt above 2.7 (a small hole) and one with Kt below 2.3 (a large hole) — the net section changes too.', ok: () => { const L = state.log.filter(x => x.geo === 'hole'); return L.some(x => x.Kt > 2.7) && L.some(x => x.Kt < 2.3); } },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawCrack(); if (frame % 6 === 0) { drawSN(); drawGoodman(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setGeo(g) { state.geo = g; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === g)); document.querySelectorAll('[data-m]').forEach(e => e.classList.toggle('off', !e.dataset.m.split(' ').includes(g))); const La = $('#La'), Lm = $('#Lm'); if (g === 'bend') { La.max = 1500; Lm.min = -500; Lm.max = 1500; La.step = 5; Lm.step = 5; } else { La.max = 200; Lm.min = -50; Lm.max = 200; La.step = 1; Lm.step = 1; } resetSpecimen(); }
function resetSpecimen() { const s = state; s.n = 0; s.D = 0; s.broken = false; s.hist = []; }
function newSpecimen() { resetSpecimen(); state.specimens++; }
const PRESETS = {
  base: () => { setGeo('bend'); $('#mat').value = 's1045'; state.mat = 's1045'; $('#finish').value = 'machined'; state.finish = 'machined'; $('#rel').value = '50'; state.rel = 50; $('#crit').value = 'goodman'; state.crit = 'goodman'; $('#Dd').value = '1.5'; state.Dd = 1.5; for (const [k, v] of Object.entries({ d: 25, r: 0.5, La: 220, Lm: 0, f: 10, tscale: 200 })) setCtl(k, v); resetSpecimen(); state.run = true; },
  bigr: () => { PRESETS.base(); setCtl('r', 4); },
  polish: () => { PRESETS.base(); $('#finish').value = 'ground'; state.finish = 'ground'; },
  mean: () => { PRESETS.base(); setCtl('Lm', 150); },
  strong: () => { PRESETS.base(); $('#mat').value = 's4340'; state.mat = 's4340'; },
  alu: () => { PRESETS.base(); $('#mat').value = 'al6061'; state.mat = 'al6061'; setCtl('La', 100); setCtl('r', 2); setCtl('tscale', 10000); },
  hole: () => { PRESETS.base(); setGeo('hole'); for (const [k, v] of Object.entries({ w: 50, t: 8, dh: 10, La: 35, Lm: 15 })) setCtl(k, v); state.run = true; },
  infinite: () => { PRESETS.base(); setCtl('r', 3); $('#finish').value = 'ground'; state.finish = 'ground'; setCtl('La', 210); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('d', v => v + ' mm'); bindR('r', v => v.toFixed(1) + ' mm'); bindR('w', v => v + ' mm'); bindR('t', v => v + ' mm'); bindR('dh', v => v + ' mm'); bindR('La', v => v + (state.geo === 'bend' ? ' N·m' : ' kN')); bindR('Lm', v => v + (state.geo === 'bend' ? ' N·m' : ' kN')); bindR('f', v => v + ' Hz'); $('#tscale').addEventListener('change', e => state.tscale = +e.target.value);
  for (const id of ['mat', 'finish', 'crit']) $('#' + id).addEventListener('change', e => { state[id] = e.target.value; }); $('#rel').addEventListener('change', e => state.rel = +e.target.value); $('#Dd').addEventListener('change', e => state.Dd = +e.target.value);
  document.querySelectorAll('#types button').forEach(b => b.onclick = () => setGeo(b.dataset.type));
  $('#btnRun').onclick = () => { if (state.broken) newSpecimen(); state.run = true; }; $('#btnStop').onclick = () => state.run = false; $('#btnNew').onclick = newSpecimen; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('fn_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); if (state.broken) newSpecimen(); state.run = !state.run; } if (k === 'N') newSpecimen(); if (k === 'R') record(''); if (k === '1') setGeo('bend'); if (k === '2') setGeo('axial'); if (k === '3') setGeo('hole'); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.FatigueBench = {
  getState: () => ({ inputs: { geo: state.geo, mat: state.mat, d: state.d, Dd: state.Dd, r: state.r, w: state.w, t: state.t, dh: state.dh, finish: state.finish, rel: state.rel, crit: state.crit, La: state.La, Lm: state.Lm, f: state.f, tscale: state.tscale, run: state.run }, results: { Kt: R.Kt, q: R.q, Kf: R.Kf, Se: R.Se, sa: R.sa, sm: R.sm, sar: R.sar, nf: R.nf, ny: R.ny, N: R.N, n: state.n, D: state.D, a: R.a, ac: R.ac, broken: state.broken, yieldNotch: R.yieldNotch }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.geo) setGeo(inp.geo); for (const k of ['d', 'r', 'w', 't', 'dh', 'La', 'Lm', 'f', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); for (const k of ['mat', 'finish', 'crit']) if (inp[k]) { $('#' + k).value = inp[k]; state[k] = inp[k]; } if (inp.rel) { $('#rel').value = inp.rel; state.rel = +inp.rel; } if (inp.Dd) { $('#Dd').value = inp.Dd; state.Dd = +inp.Dd; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.newSpecimen) newSpecimen(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'fatigue-notch', title: 'Fatigue and Stress Concentration Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.FatigueBench.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'fatigue-notch', state: window.FatigueBench.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('fn_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setGeo('bend'); step(0); drawRig(); updateReadouts(); drawSN(); drawGoodman(); drawCrack();
  requestAnimationFrame(tick);
}
build();
