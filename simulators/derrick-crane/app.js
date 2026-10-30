/* Derrick Crane Demonstrator — stiff-leg derrick solved as statics: moments about the boom foot give the topping
   lift, axial components give boom and mast compression, plan equilibrium at the mast head gives the two leg forces,
   leg tension gives anchor uplift. Every dimension is adjustable; the load chart is rebuilt from the same solver.
   Standalone, offline. Rig data illustrative. */
'use strict';

const G = 9.81, SF_ROPE = 5, LREF = 20, MAST_CAP = 1200, SLEW_MARGIN = 8, BLOCK_MIN = 1.0;
const $ = s => document.querySelector(s);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—', rad = d => d * Math.PI / 180;

/* ------------------------------------------------------------------ state */
const state = {
  Hm: 24, Lb: 30, hf: 1.5, Ls: 18, phi: 45, qb: 90, nh: 4, nt: 6, ropeH: 265, ropeT: 265, mh: 0.6, anc: 250, cb0: 900, cl0: 500, winch: 60,
  mload: 10, theta: 55, psi: 0, drop: 6, lifted: false, motion: {}, t: 0, log: [], flags: {}, tipped: false,
};
const R = {};

/* ------------------------------------------------------------------ statics */
function geometry(s = state) {
  const th = rad(s.theta), Px = s.Lb * Math.cos(th), Py = s.Lb * Math.sin(th);          // tip relative to the boom foot
  const Hrel = s.Hm - s.hf, d = Math.hypot(Px, Hrel - Py);                                 // topping-lift length
  const ux = -Px / d, uy = (Hrel - Py) / d;                                                 // unit vector tip → mast head
  const bx = Math.cos(th), by = Math.sin(th), Lleg = Math.hypot(s.Ls, s.Hm);
  return { th, Px, Py, Hrel, d, ux, uy, bx, by, Lleg, R: Px, tipH: s.hf + Py };
}
function forces(W, s = state, g = geometry(s)) {   // W = load on the hook incl. block, kN → all member forces (kN)
  const Wb = s.qb * s.Lb * G / 1000;
  const Tt = (W + Wb / 2) * g.d / g.Hrel;                                                   // topping lift, total
  const Cb = -Tt * (g.ux * g.bx + g.uy * g.by) + W * g.by + W / s.nh + Wb * g.by;            // boom compression at the foot
  const Htop = Tt * g.Px / g.d, Vtop = Tt * (g.Hrel - g.Py) / g.d;                          // pull on the mast head: horizontal toward the boom, vertical down (+)
  const a1 = rad(180 + s.phi), a2 = rad(180 - s.phi), k = s.Ls / g.Lleg, kv = s.Hm / g.Lleg, ps = rad(s.psi);
  const e1 = [Math.cos(a1) * k, Math.sin(a1) * k], e2 = [Math.cos(a2) * k, Math.sin(a2) * k], Hx = Htop * Math.cos(ps), Hy = Htop * Math.sin(ps);
  const det = e1[0] * e2[1] - e1[1] * e2[0];   // T1 e1 + T2 e2 + H = 0
  const T1 = (-Hx * e2[1] + Hy * e2[0]) / det, T2 = (-e1[0] * Hy + e1[1] * Hx) / det;       // tension positive
  const Cm = Vtop + Tt / s.nt + (T1 + T2) * kv;
  const U1 = T1 * kv, U2 = T2 * kv, base = W + Wb + Math.max(0, U1) + Math.max(0, U2);
  const cbCap = s.cb0 * Math.pow(LREF / s.Lb, 2), clCap = s.cl0 * Math.pow(LREF / g.Lleg, 2), ropeHcap = s.ropeH / SF_ROPE, ropeTcap = s.ropeT / SF_ROPE;
  const Th = W / s.nh, Ttp = Tt / s.nt;
  const util = { boom: Cb / cbCap, topping: Ttp / Math.min(ropeTcap, s.winch), hoist: Th / Math.min(ropeHcap, s.winch), mast: Cm / MAST_CAP, leg1: T1 < 0 ? -T1 / clCap : T1 / (clCap * 1.6), leg2: T2 < 0 ? -T2 / clCap : T2 / (clCap * 1.6), anchor: Math.max(U1, U2) / s.anc };
  let gov = 'boom', umax = -1; for (const k in util) if (util[k] > umax) { umax = util[k]; gov = k; }
  return { W, Wb, Tt, Ttp, Th, Cb, Cm, Htop, Vtop, T1, T2, U1, U2, base, cbCap, clCap, ropeHcap, ropeTcap, util, umax, gov };
}
function ratedLoad(s = state, g = geometry(s)) {   // largest W with every utilisation ≤ 1 (all forces are affine in W)
  const f0 = forces(0, s, g), f1 = forces(100, s, g); let best = Infinity, gov = 'boom';
  for (const k in f0.util) { const a = f0.util[k], b = (f1.util[k] - a) / 100; if (b > 1e-9) { const w = (1 - a) / b; if (w < best) { best = w; gov = k; } } }
  return { W: Math.max(0, best), gov };
}
const GOV_NAME = { boom: 'boom strut', topping: 'topping rope / winch', hoist: 'hoist rope / winch', mast: 'mast', leg1: 'leg 1', leg2: 'leg 2', anchor: 'anchor hold-down' }, GOV_COL = { boom: '--water', topping: '--brass', hoist: '--water-deep', mast: '--steel', leg1: '--steel', leg2: '--steel', anchor: '--warn' };

/* ------------------------------------------------------------------ frame */
function step(dt) {
  const m = state.motion;
  if (m.luffUp) state.theta = clamp(state.theta + 1.5 * dt, 10, 85); if (m.luffDown) state.theta = clamp(state.theta - 1.5 * dt, 10, 85);
  if (m.slewL) state.psi = clamp(state.psi - 4 * dt, -160, 160); if (m.slewR) state.psi = clamp(state.psi + 4 * dt, -160, 160);
  const g = geometry(); const maxDrop = g.tipH;
  if (m.hoistUp) state.drop = clamp(state.drop - 0.6 * dt, BLOCK_MIN, maxDrop); if (m.hoistDown) state.drop = clamp(state.drop + 0.6 * dt, BLOCK_MIN, maxDrop);
  state.drop = clamp(state.drop, BLOCK_MIN, maxDrop);
  const hookH = g.tipH - state.drop;
  if (hookH > 0.05) state.lifted = true; if (hookH <= 0.01) state.lifted = false;
  const W = (state.mh + (state.lifted ? state.mload : 0)) * G;
  Object.assign(R, g, forces(W), { hookH, rated: ratedLoad(), slewLimit: Math.abs(state.psi) > 180 - state.phi - SLEW_MARGIN, twoBlock: state.drop <= BLOCK_MIN + 1e-6 && m.hoistUp });
  R.tipped = R.util.anchor > 1 && state.lifted; R.overload = R.umax > 1;
  syncOutputs();
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
const ucls = u => u >= 1 ? 'u2' : u >= 0.7 ? 'u1' : 'u0';
let svg, gElev, gPlan;
function buildRig() { svg = el('svg', { viewBox: '0 0 900 520', role: 'img', 'aria-label': 'Derrick elevation and plan' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); gElev = el('g', {}, svg); gPlan = el('g', { transform: 'translate(10,14)' }, svg); }
function drawRig() {
  const s = state, g = R; gElev.innerHTML = '';
  const Lleft = s.Ls * Math.max(0.15, Math.cos(rad(s.phi))) + 2, hmax = Math.max(s.Hm, g.tipH) + 3, sc = Math.min(440 / hmax, 640 / (Lleft + s.Lb + 4));
  const X = x => 240 + (Lleft + x) * sc, Y = y => 470 - y * sc;   // ground at y = 470; legs projected to the left of the mast, plan inset top-left
  el('line', { x1: 20, y1: Y(0), x2: 880, y2: Y(0), class: 'ground' }, gElev); for (let x = 30; x < 880; x += 16) el('line', { x1: x, y1: Y(0), x2: x - 8, y2: Y(0) + 8, class: 'hatch' }, gElev);
  // legs (projected: both drawn to the rear at their plan-projected distance)
  const ps = rad(s.psi), legProj = i => { const a = rad(180 + (i === 1 ? s.phi : -s.phi)) - ps; return s.Ls * Math.cos(a); };
  for (const i of [1, 2]) { const xa = legProj(i), T = i === 1 ? g.T1 : g.T2, u = g.util['leg' + i]; el('line', { x1: X(0), y1: Y(s.Hm), x2: X(xa), y2: Y(0), class: 'member leg ' + ucls(u) }, gElev); const lifting = (i === 1 ? g.U1 : g.U2) > s.anc; el('rect', { x: X(xa) - 12, y: Y(0) - (lifting ? 8 : 0), width: 24, height: 10, class: 'anchor' + (lifting ? ' lift' : '') }, gElev); el('line', { x1: X(0), y1: Y(0), x2: X(xa), y2: Y(0), class: 'member sill' }, gElev); txt(X(xa), Y(0) + 18 + (i === 1 ? 0 : 13), `leg ${i} ${T >= 0 ? 'T' : 'C'} ${fmt(Math.abs(T), 0)} kN · uplift ${fmt(Math.max(0, i === 1 ? g.U1 : g.U2), 0)}`, 'ftag' + (lifting ? ' bad' : ''), gElev, 'middle'); }
  // mast
  el('line', { x1: X(0), y1: Y(0), x2: X(0), y2: Y(s.Hm), class: 'member mast ' + ucls(g.util.mast) }, gElev); for (let y = 2; y < s.Hm; y += 2) el('line', { x1: X(0) - 4, y1: Y(y), x2: X(0) + 4, y2: Y(y + 1.2), class: 'lattice' }, gElev);
  txt(X(0) - 10, Y(s.Hm / 2), `mast C ${fmt(g.Cm, 0)} kN`, 'ftag', gElev, 'end');
  // boom
  const tipX = X(g.Px), tipY = Y(g.tipH), footY = Y(s.hf);
  el('line', { x1: X(0), y1: footY, x2: tipX, y2: tipY, class: 'member boom ' + ucls(g.util.boom) }, gElev);
  for (let l = 1.5; l < s.Lb; l += 1.5) { const x = X(l * g.bx), y = Y(s.hf + l * g.by); el('line', { x1: x - 3 * g.by, y1: y - 3 * g.bx, x2: x + 3 * g.by + 5 * g.bx, y2: y + 3 * g.bx - 5 * g.by, class: 'lattice' }, gElev); }
  txt((X(0) + tipX) / 2 + 12, (footY + tipY) / 2 + 14, `boom C ${fmt(g.Cb, 0)} kN`, 'ftag' + (g.util.boom >= 1 ? ' bad' : ''), gElev);
  // topping lift (parts drawn as a narrow fan), hoist lead along the boom, hoist fall to the hook
  for (let i = 0; i < Math.min(s.nt, 4); i++) el('line', { x1: X(0) + (i - 1.5) * 1.6, y1: Y(s.Hm), x2: tipX + (i - 1.5) * 1.6, y2: tipY, class: 'rope' }, gElev);
  el('line', { x1: X(0) + 5, y1: Y(s.Hm), x2: X(0) + 5, y2: Y(0.8), class: 'rope' }, gElev);   // topping lead down the mast
  txt((X(0) + tipX) / 2 - 8, (Y(s.Hm) + tipY) / 2 - 8, `topping ${fmt(g.Tt, 0)} kN · ${fmt(g.Ttp, 0)}/part`, 'ftag' + (g.util.topping >= 1 ? ' bad' : ''), gElev, 'end');
  el('line', { x1: X(0) + 6 * g.by, y1: footY - 6 * g.bx, x2: tipX + 6 * g.by, y2: tipY - 6 * g.bx, class: 'rope hoist' }, gElev);
  const hookY = Y(g.hookH); for (let i = 0; i < Math.min(s.nh, 4); i++) el('line', { x1: tipX + (i - 1.5) * 2, y1: tipY, x2: tipX + (i - 1.5) * 2, y2: hookY, class: 'rope hoist' }, gElev);
  el('circle', { cx: tipX, cy: tipY, r: 5, class: 'sheave' }, gElev); el('circle', { cx: X(0), cy: Y(s.Hm), r: 5, class: 'sheave' }, gElev); el('circle', { cx: X(0), cy: footY, r: 4, class: 'pin' }, gElev);
  el('rect', { x: X(0) - 22, y: Y(0) - 14, width: 14, height: 12, class: 'winch' }, gElev); el('rect', { x: X(0) + 8, y: Y(0) - 14, width: 14, height: 12, class: 'winch' }, gElev);
  // hook block and load
  el('rect', { x: tipX - 7, y: hookY - 10, width: 14, height: 12, class: 'hook' }, gElev);
  const loadX = state.lifted ? tipX : tipX, loadY = state.lifted ? hookY + 2 : Y(0); const lw = 18 + Math.sqrt(s.mload) * 8;
  if (s.mload > 0) { el('rect', { x: loadX - lw / 2, y: loadY - (state.lifted ? 0 : lw * 0.6), width: lw, height: lw * 0.6, rx: 3, class: 'load' }, gElev); txt(loadX, loadY + (state.lifted ? lw * 0.36 : -lw * 0.6 + lw * 0.36), `${s.mload} t`, 'load-txt', gElev); if (!state.lifted && Math.abs(loadX - tipX) < 1) el('line', { x1: tipX, y1: hookY + 2, x2: tipX, y2: Y(0) - lw * 0.6, class: 'rope hoist slack' }, gElev); }
  // dimensions
  el('path', { d: `M${X(0)},${Y(0) + 36} H${tipX}`, class: 'dim' }, gElev); txt((X(0) + tipX) / 2, Y(0) + 46, `R = ${fmt(g.R)} m`, 'dim-txt', gElev, 'middle');
  el('path', { d: `M${X(0) - 40},${Y(0)} V${Y(s.Hm)}`, class: 'dim' }, gElev); txt(X(0) - 44, Y(s.Hm) + 10, `H ${s.Hm} m`, 'dim-txt', gElev, 'end');
  const near = tipX > 640; txt(tipX + (near ? -10 : 10), tipY - 8, `tip ${fmt(g.tipH)} m · hook ${fmt(g.hookH)} m · θ ${fmt(s.theta, 0)}°`, 'dim-txt', gElev, near ? 'end' : 'start');
  drawPlan();
}
function drawPlan() {
  const s = state, g = R; gPlan.innerHTML = ''; const r = 95, c = [100, 100], sc = r / Math.max(s.Ls, s.Lb);
  el('rect', { x: 0, y: 0, width: 200, height: 210, rx: 8, class: 'plan-bg' }, gPlan); txt(8, 14, 'Plan · legs and slew arc', 'plan-txt', gPlan);
  const P = (dist, ang) => [c[0] + dist * sc * Math.cos(rad(ang)), c[1] - dist * sc * Math.sin(rad(ang))];   // plan angle: 0 = boom zero-slew (right), + = anticlockwise
  const lim = 180 - s.phi - SLEW_MARGIN, arc = (a0, a1, cls) => { const p0 = P(r / sc, a0), p1 = P(r / sc, a1); el('path', { d: `M${c[0]},${c[1]} L${p0[0]},${p0[1]} A${r},${r} 0 ${Math.abs(a1 - a0) > 180 ? 1 : 0} 0 ${p1[0]},${p1[1]} Z`, class: cls }, gPlan); };
  arc(-lim, lim, 'arc-ok'); arc(lim, 360 - lim, 'arc-no');
  for (const [i, a] of [[1, 180 + s.phi], [2, 180 - s.phi]]) { const p = P(s.Ls, a); el('line', { x1: c[0], y1: c[1], x2: p[0], y2: p[1], class: 'plan-leg' }, gPlan); const lifting = (i === 1 ? g.U1 : g.U2) > s.anc; el('rect', { x: p[0] - 6, y: p[1] - 6, width: 12, height: 12, class: 'anchor' + (lifting ? ' lift' : '') }, gPlan); txt(p[0] + (i === 1 ? -9 : 9), p[1] + 4, `leg ${i}`, 'plan-txt', gPlan, i === 1 ? 'end' : 'start'); }
  const b = P(g.R, s.psi); el('line', { x1: c[0], y1: c[1], x2: b[0], y2: b[1], class: 'plan-boom' }, gPlan); el('circle', { cx: c[0], cy: c[1], r: 5, class: 'pin' }, gPlan);
  txt(100, 204, `slew ${fmt(s.psi, 0)}° · safe arc ±${fmt(lim, 0)}°`, 'plan-txt', gPlan, 'middle');
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawChart() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H);
  const L = 56, Rr = 16, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state;
  const pts = []; for (let th = 85; th >= 10; th -= 1) { const g = geometry({ ...s, theta: th }); const rl = ratedLoad({ ...s, theta: th }, g); pts.push({ R: g.R, t: rl.W / G - s.mh, gov: rl.gov }); }
  const rmax = Math.max(10, s.Lb * Math.cos(rad(10)) + 2), tmax = Math.max(5, Math.min(120, Math.max(...pts.map(p => p.t)) * 1.15), s.mload * 1.1);
  const X = r => L + r / rmax * pw, Y = t => T + (1 - clamp(t / tmax, 0, 1)) * ph;
  ctx.font = '11px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.strokeStyle = col('--grid');
  const rs = rmax > 30 ? 5 : 2; for (let r = 0; r <= rmax; r += rs) { ctx.beginPath(); ctx.moveTo(X(r), T); ctx.lineTo(X(r), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(r, X(r), T + ph + 14); }
  const ts = tmax > 50 ? 20 : tmax > 20 ? 10 : 5; for (let t = 0; t <= tmax; t += ts) { ctx.beginPath(); ctx.moveTo(L, Y(t)); ctx.lineTo(L + pw, Y(t)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(t, L - 6, Y(t) + 4); }
  ctx.textAlign = 'center'; ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.fillText('radius, m', L + pw / 2, H - 6); ctx.save(); ctx.translate(12, T + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('rated load, t (net of block)', 0, 0); ctx.restore();
  ctx.lineWidth = 3; for (let i = 1; i < pts.length; i++) { ctx.beginPath(); ctx.strokeStyle = col(GOV_COL[pts[i].gov]); ctx.moveTo(X(pts[i - 1].R), Y(Math.max(0, pts[i - 1].t))); ctx.lineTo(X(pts[i].R), Y(Math.max(0, pts[i].t))); ctx.stroke(); }
  ctx.lineWidth = 1; ctx.strokeStyle = col('--line'); ctx.strokeRect(L, T, pw, ph);
  const cur = state.lifted ? s.mload : 0; ctx.beginPath(); ctx.fillStyle = col('--alarm'); ctx.arc(X(R.R), Y(cur), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = col('--alarm'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`${fmt(cur)} t at ${fmt(R.R)} m · rated ${fmt(R.rated.W / G - s.mh)} t (${GOV_NAME[R.rated.gov]})`, Math.min(X(R.R) + 10, L + pw - 380), Math.max(Y(cur) - 10, T + 12));
  if (!state.lifted) { ctx.fillStyle = col('--muted'); ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillText('load on the ground — hoist to pick it up', L + 8, T + ph - 8); }
}
function drawBars() {
  const cv = $('#bars'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H);
  const items = [['Boom strut', R.util.boom, `${fmt(R.Cb, 0)} / ${fmt(R.cbCap, 0)} kN`], ['Topping rope / winch', R.util.topping, `${fmt(R.Ttp, 0)} / ${fmt(Math.min(R.ropeTcap, state.winch), 0)} kN per part`], ['Hoist rope / winch', R.util.hoist, `${fmt(R.Th, 0)} / ${fmt(Math.min(R.ropeHcap, state.winch), 0)} kN per part`], ['Mast', R.util.mast, `${fmt(R.Cm, 0)} / ${MAST_CAP} kN`], ['Leg 1', R.util.leg1, `${R.T1 >= 0 ? 'tension' : 'compression'} ${fmt(Math.abs(R.T1), 0)} kN`], ['Leg 2', R.util.leg2, `${R.T2 >= 0 ? 'tension' : 'compression'} ${fmt(Math.abs(R.T2), 0)} kN`], ['Anchor hold-down', R.util.anchor, `uplift ${fmt(Math.max(R.U1, R.U2, 0), 0)} / ${state.anc} kN`]];
  const L = 170, Rr = 230, T = 10, rowH = (H - T - 8) / items.length, pw = W - L - Rr;
  ctx.font = '12px IBM Plex Sans, sans-serif';
  items.forEach(([name, u, note], i) => { const y = T + i * rowH; ctx.fillStyle = col('--ink'); ctx.textAlign = 'right'; ctx.fillText(name, L - 10, y + rowH * 0.62); ctx.fillStyle = col('--panel2'); ctx.fillRect(L, y + 4, pw, rowH - 8); const uu = clamp(u, 0, 1.3); ctx.fillStyle = col(u >= 1 ? '--alarm' : u >= 0.7 ? '--warn' : '--good'); ctx.fillRect(L, y + 4, pw * uu / 1.3, rowH - 8); ctx.textAlign = 'left'; ctx.fillStyle = col('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.fillText(`${fmt(u * 100, 0)} %  ·  ${note}`, L + pw + 8, y + rowH * 0.62); ctx.font = '12px IBM Plex Sans, sans-serif'; });
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L + pw / 1.3, T); ctx.lineTo(L + pw / 1.3, H - 8); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('100 %', L + pw / 1.3, H - 1);
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const s = state, g = R, sgn = T => T >= 0 ? 'tension' : 'compression';
  $('#roR').textContent = fmt(g.R); $('#roH').textContent = fmt(g.hookH); $('#roW').textContent = fmt(g.W, 0); $('#roWt').textContent = fmt(g.W / G);
  $('#roTt').textContent = fmt(g.Tt, 0); $('#roTtp').textContent = fmt(g.Ttp, 0); $('#roTh').textContent = fmt(g.Th, 0); $('#roCb').textContent = fmt(g.Cb, 0); $('#roCbCap').textContent = fmt(g.cbCap, 0); $('#roCm').textContent = fmt(g.Cm, 0); $('#roCmCap').textContent = MAST_CAP;
  $('#roL1').textContent = fmt(Math.abs(g.T1), 0); $('#roL1s').textContent = sgn(g.T1); $('#roL2').textContent = fmt(Math.abs(g.T2), 0); $('#roL2s').textContent = sgn(g.T2); $('#roU').textContent = fmt(Math.max(0, g.U1, g.U2), 0); $('#roUcap').textContent = s.anc; $('#roB').textContent = fmt(g.base, 0); $('#roRated').textContent = fmt(g.rated.W / G - s.mh);
  const setBox = (id, u) => { const b = $('#' + id); b.classList.toggle('alarm', u >= 1); b.classList.toggle('warnv', u >= 0.7 && u < 1); };
  setBox('roTtBox', g.util.topping); setBox('roThBox', g.util.hoist); setBox('roCbBox', g.util.boom); setBox('roCmBox', g.util.mast); setBox('roL1Box', g.util.leg1); setBox('roL2Box', g.util.leg2); setBox('roUBox', g.util.anchor);
  const st = $('#pillState'); st.textContent = g.tipped ? 'OVERTURNING — anchor lifting' : g.overload ? `Overload — ${GOV_NAME[g.gov]}` : !state.lifted ? 'Load on the ground' : `Lifting ${s.mload} t at ${fmt(g.R)} m`; st.className = 'pill ' + (g.tipped || g.overload ? 'warn' : 'ok');
  $('#pillUtil').textContent = `utilisation ${fmt(g.umax * 100, 0)} %`; $('#pillGov').textContent = `governs: ${GOV_NAME[g.gov]}`;
  $('#pillOver').classList.toggle('hidden', !g.overload); $('#pillTip').classList.toggle('hidden', !g.tipped); $('#pillSlew').classList.toggle('hidden', !g.slewLimit); $('#pillBlock').classList.toggle('hidden', !g.twoBlock);
}
function syncOutputs() { $('#thetaOut').textContent = fmt(state.theta, 0) + '°'; $('#theta').value = state.theta; $('#psiOut').textContent = fmt(state.psi, 0) + '°'; $('#psi').value = state.psi; }

/* ------------------------------------------------------------------ log, tasks */
function record(tag = '') {
  const s = state, g = R, r = { i: s.log.length + 1, tag, Hm: s.Hm, Lb: s.Lb, Ls: s.Ls, phi: s.phi, th: +s.theta.toFixed(0), psi: +s.psi.toFixed(0), R: +g.R.toFixed(1), load: state.lifted ? s.mload : 0, parts: `${s.nh}/${s.nt}`, Tt: +g.Tt.toFixed(0), Th: +g.Th.toFixed(1), Cb: +g.Cb.toFixed(0), Cm: +g.Cm.toFixed(0), T1: +g.T1.toFixed(0), T2: +g.T2.toFixed(0), U: +Math.max(0, g.U1, g.U2).toFixed(0), util: +(g.umax * 100).toFixed(0), gov: GOV_NAME[g.gov] };
  s.log.push(r); renderLog(); checkTasks();
}
function renderLog() {
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.Hm, r.Lb, r.Ls, r.phi, r.th, r.psi, r.R, r.load, r.parts, r.Tt, r.Th, r.Cb, r.Cm, r.T1, r.T2, r.U, r.util, r.gov].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,mast_m,boom_m,sill_m,spread_deg,angle_deg,slew_deg,radius_m,load_t,parts_hoist/topping,topping_kN,hoist_per_part_kN,boom_kN,mast_kN,leg1_kN,leg2_kN,uplift_kN,util_pct,governs\n' + state.log.map(r => [r.i, r.tag, r.Hm, r.Lb, r.Ls, r.phi, r.th, r.psi, r.R, r.load, r.parts, r.Tt, r.Th, r.Cb, r.Cm, r.T1, r.T2, r.U, r.util, r.gov].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'reach', t: 'Find the radius limit', d: 'Lift 10 t or more and luff out until something reaches 100 %. Which limit governs, and at what radius?', ok: () => state.flags.reach },
  { id: 'reeve', t: 'Reeving halves the rope pull', d: 'Record the same lift with two different hoist reevings and compare the hoist rope tension.', ok: () => { const L = state.log.filter(r => r.load > 0); return L.some(a => L.some(b => a.parts.split('/')[0] !== b.parts.split('/')[0] && same(a, b, ['load', 'th', 'Lb', 'Hm']))); } },
  { id: 'topping', t: 'Topping lift against boom angle', d: 'Record the same lift at three boom angles. Plot topping-lift tension against radius.', ok: () => { const L = state.log.filter(r => r.load > 0); const grp = {}; for (const r of L) { const k = `${r.load}|${r.Lb}|${r.Hm}|${r.parts}`; (grp[k] = grp[k] || new Set()).add(r.th); } return Object.values(grp).some(s => s.size >= 3); } },
  { id: 'mast', t: 'A taller mast', d: 'Record the same lift (same boom, angle, load) with two mast heights. Where does the topping tension go, and the mast compression?', ok: () => { const L = state.log.filter(r => r.load > 0); return L.some(a => L.some(b => a.Hm !== b.Hm && same(a, b, ['load', 'th', 'Lb', 'parts']))); } },
  { id: 'legs', t: 'Slew toward a leg', d: 'With a load on, slew until one leg carries more than twice the other. Keep going: which leg goes into compression?', ok: () => state.flags.legs },
  { id: 'anchor', t: 'Lift an anchor', d: 'Reduce the hold-down or increase the load until an anchor lifts and the derrick would overturn.', ok: () => state.flags.anchor },
  { id: 'boomlen', t: 'Longer boom, same radius', d: 'Record the same load at the same radius (±1 m) with two boom lengths. Compare boom compression with its capacity.', ok: () => { const L = state.log.filter(r => r.load > 0); return L.some(a => L.some(b => a.Lb !== b.Lb && a.load === b.load && Math.abs(a.R - b.R) <= 1)); } },
  { id: 'design', t: 'Design a 10 t lift at 20 m', d: 'Choose dimensions and rigging so that 10 t at 20 m radius or more has every member below 90 %.', ok: () => state.flags.design },
  { id: 'twoblock', t: 'Two-block it', d: 'Hoist until the block meets the boom tip. What does the hoist rope see if the winch keeps pulling?', ok: () => state.flags.twoblock },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() {
  const g = R; if (state.lifted) { if (state.mload >= 10 && g.overload) state.flags.reach = true; if (Math.abs(g.T1) > 2 * Math.abs(g.T2) || Math.abs(g.T2) > 2 * Math.abs(g.T1)) state.flags.legs = true; if (g.tipped) state.flags.anchor = true; if (state.mload >= 10 && g.R >= 20 && g.umax <= 0.9) state.flags.design = true; }
  if (g.twoBlock) state.flags.twoblock = true;
  for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; }
}

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt; step(dt); drawRig(); frame++; if (frame % 2 === 0) { drawBars(); updateReadouts(); if (frame % 6 === 0) { drawChart(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
const PRESETS = {
  std: () => { for (const [k, v] of Object.entries({ Hm: 24, Lb: 30, hf: 1.5, Ls: 18, phi: 45, qb: 90, nh: 4, nt: 6, mh: 0.6, anc: 250, cb0: 900, cl0: 500, winch: 60, mload: 10, theta: 55, psi: 0 })) setCtl(k, v); setCtl('ropeH', 265); setCtl('ropeT', 265); },
  reach: () => { PRESETS.std(); setCtl('mload', 12); state.drop = 4; state.motion = { luffDown: true }; setTimeout(() => state.motion = {}, 25000); },
  tall: () => { PRESETS.std(); setCtl('Hm', 36); },
  narrow: () => { PRESETS.std(); setCtl('phi', 30); setCtl('psi', 120); state.drop = 4; },
  light: () => { PRESETS.std(); setCtl('anc', 80); setCtl('mload', 15); state.drop = 4; },
  single: () => { PRESETS.std(); setCtl('nh', 1); setCtl('mload', 8); state.drop = 4; },
  long: () => { PRESETS.std(); setCtl('Lb', 44); setCtl('theta', 40); state.drop = 4; },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('Hm', v => v + ' m'); bindR('Lb', v => v + ' m'); bindR('hf', v => v.toFixed(1) + ' m'); bindR('Ls', v => v + ' m'); bindR('phi', v => v + '°'); bindR('qb', v => v + ' kg/m'); bindR('nh', v => String(v)); bindR('nt', v => String(v)); bindR('mh', v => v.toFixed(1) + ' t'); bindR('anc', v => v + ' kN'); bindR('cb0', v => v + ' kN'); bindR('cl0', v => v + ' kN'); bindR('winch', v => v + ' kN'); bindR('mload', v => v + ' t'); bindR('theta', v => fmt(v, 0) + '°'); bindR('psi', v => fmt(v, 0) + '°');
  $('#ropeH').addEventListener('change', e => state.ropeH = +e.target.value); $('#ropeT').addEventListener('change', e => state.ropeT = +e.target.value);
  document.querySelectorAll('.motion [data-m]').forEach(b => { const on = e => { e.preventDefault(); state.motion[b.dataset.m] = true; b.classList.add('on'); }, off = () => { state.motion[b.dataset.m] = false; b.classList.remove('on'); }; b.addEventListener('pointerdown', on); b.addEventListener('pointerup', off); b.addEventListener('pointerleave', off); b.addEventListener('pointercancel', off); });
  $('#btnRecord').onclick = () => record(''); $('#btnLand').onclick = () => { state.drop = R.tipH; state.lifted = false; };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('dk_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  const KEYS = { Q: 'luffUp', A: 'luffDown', Z: 'slewL', X: 'slewR', ARROWUP: 'hoistUp', ARROWDOWN: 'hoistDown' };
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (KEYS[k]) { state.motion[KEYS[k]] = true; e.preventDefault(); } else if (k === 'R') record(''); else if (e.key === 'Escape') $('#help').classList.add('hidden'); });
  document.addEventListener('keyup', e => { const k = e.key.toUpperCase(); if (KEYS[k]) state.motion[KEYS[k]] = false; });
  window.addEventListener('resize', () => { drawChart(); drawBars(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.DerrickCrane = {
  getState: () => ({ inputs: { Hm: state.Hm, Lb: state.Lb, hf: state.hf, Ls: state.Ls, phi: state.phi, qb: state.qb, nh: state.nh, nt: state.nt, ropeH: state.ropeH, ropeT: state.ropeT, mh: state.mh, anc: state.anc, cb0: state.cb0, cl0: state.cl0, winch: state.winch, mload: state.mload, theta: state.theta, psi: state.psi, lifted: state.lifted }, results: { R: R.R, hookH: R.hookH, W: R.W, Tt: R.Tt, Th: R.Th, Cb: R.Cb, Cm: R.Cm, T1: R.T1, T2: R.T2, U1: R.U1, U2: R.U2, umax: R.umax, gov: R.gov, rated_t: R.rated.W / G - state.mh, overload: R.overload, tipped: R.tipped }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['Hm', 'Lb', 'hf', 'Ls', 'phi', 'qb', 'nh', 'nt', 'mh', 'anc', 'cb0', 'cl0', 'winch', 'mload', 'theta', 'psi', 'ropeH', 'ropeT']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.lift) state.drop = 4; if (inp.land) { state.drop = 1e9; state.lifted = false; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); },
  meta: { id: 'derrick-crane', title: 'Derrick Crane Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.DerrickCrane.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'derrick-crane', state: window.DerrickCrane.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('dk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); state.drop = geometry().tipH; step(0); drawRig(); drawChart(); drawBars(); updateReadouts();
  requestAnimationFrame(tick);
}
build();
