/* Belt and Chain Drive Bench — V-belt, flat belt, synchronous belt and roller chain on two pulleys. Euler–Eytelwein
   traction with installation tension and centrifugal tension, creep below the limit and gross slip above it, belt
   heating on a thermal clock, rated power per belt, tooth jump and chain chordal action. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const SEC = { SPZ: { k: 0.7, dmin: 63, m: 0.07, name: 'SPZ' }, SPA: { k: 1.25, dmin: 90, m: 0.12, name: 'SPA' }, SPB: { k: 2.2, dmin: 140, m: 0.2, name: 'SPB' }, SPC: { k: 4.0, dmin: 224, m: 0.37, name: 'SPC' } };
const CHAIN = { '08B': { p: 12.7, Fb: 17800, m: 0.7 }, '12B': { p: 19.05, Fb: 28900, m: 1.25 }, '16B': { p: 25.4, Fb: 60000, m: 2.7 } };
const SF = 1.3, BETA = 38 * Math.PI / 180, T_ALARM = 85, T_FAIL = 110;

/* ------------------------------------------------------------------ state */
const state = { type: 'v', d1: 125, d2: 250, C: 600, sec: 'SPA', z: 2, misal: 0, n1: 1450, Tload: 60, pre: 100, mu: 0.3, Tamb: 25, tscale: 50, run: false, T: 25, elong: 0, jumped: false, glazed: false, t: 0, trace: [], traceAcc: 0, log: [], flags: {}, ang1: 0, ang2: 0, dash: 0 };
const R = {};

/* ------------------------------------------------------------------ model */
function geom() {
  const s = state; if (s.type === 'chain') { const z1 = clamp(Math.round(s.d1 / 5), 9, 57), z2 = clamp(Math.round(s.d2 / 5), 9, 114), p = CHAIN[s.sec] ? CHAIN[s.sec].p : 12.7, d1 = p / Math.sin(Math.PI / z1), d2 = p / Math.sin(Math.PI / z2); return { d1, d2, z1, z2, p }; }
  return { d1: s.d1, d2: s.d2 };
}
function solve() {
  const s = state, g = geom(), d1 = g.d1, d2 = g.d2, C = Math.max(s.C, (d1 + d2) / 2 + 20), ratio = d2 / d1, w1 = s.n1 * 2 * Math.PI / 60, v = w1 * d1 / 2000;
  const theta = Math.PI - 2 * Math.asin(clamp((d2 - d1) / (2 * C), -0.99, 0.99)), Lb = 2 * C + Math.PI * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * C);
  const n2nom = s.n1 / ratio, TeReq = s.Tload / (d2 / 2000);   // N total effective tension needed at the driven pulley
  let out = { d1, d2, C, ratio, v, theta: theta * 180 / Math.PI, Lb, n2nom, TeReq, z1: g.z1, z2: g.z2 };
  if (s.type === 'v' || s.type === 'flat') {
    const sec = s.type === 'v' ? SEC[s.sec] : { k: 0.5, dmin: 40, m: 0.17, name: 'flat 50×3' }, mu = s.type === 'v' ? s.mu / Math.sin(BETA / 2) : s.mu, E = Math.exp(mu * theta), Tc = sec.m * v * v;
    const Pr = s.type === 'v' ? Math.max(0.05, sec.k * v * (1 - sec.dmin / d1) * (1 - 0.0003 * v * v)) : Math.max(0.05, (900 - Tc) * (1 - 1 / E) * v / 1000);
    const TeRated = Pr * 1000 / Math.max(v, 0.1), T0rec = s.type === 'v' ? 1.2 * TeRated / 2 * (E + 1) / (E - 1) + Tc : 900 * (1 + 1 / E) / 2, T0 = T0rec * s.pre / 100 * (1 - s.elong * 8), small = s.type === 'v' && d1 < sec.dmin;
    const TeMax = Math.max(0, 2 * (T0 - Tc) * (E - 1) / (E + 1)) * s.z, u = TeReq / Math.max(TeMax, 1e-6);
    let slip = u <= 1 ? 0.003 + 0.012 * u * u : 0.015 + 0.4 * (u - 1); slip = clamp(slip, 0, 1); const gross = u > 1;
    const Te = Math.min(TeReq, TeMax), Teb = Te / s.z, T1 = T0 + Teb / 2, T2 = T0 - Teb / 2, n2 = n2nom * (1 - slip), P = s.Tload * n2 * 2 * Math.PI / 60 / 1000, eta = (1 - slip) * (1 - (s.type === 'v' ? 0.03 : 0.02)) * 100;
    const Pslip = Te * v * slip, Tst = s.Tamb + 12 + 0.06 * Pslip / s.z + 2 * s.misal * s.misal + 0.6 * v, belts = Math.ceil(s.Tload * n2nom * 2 * Math.PI / 60 / 1000 * SF / Pr), brg = s.z * (T1 + T2) * Math.cos((Math.PI - theta) / 2);
    Object.assign(out, { small, dmin: sec.dmin, sec: sec.name, mu: mu, E, Tc, Pr, T0rec, T0, TeMax, TeMaxBelt: TeMax / s.z, u, slip, gross, Te, T1, T2, n2, P, eta, Pslip, Tst, belts, brg, life: Math.exp(-Math.max(0, s.T - 60) / 20) * (1 - 0.25 * s.misal) * (1 - s.elong * 10) * (s.pre > 150 ? 0.7 : 1), jump: false });
  } else if (s.type === 'timing') {
    const m = 0.16, Tc = m * v * v, Pr = Math.min(12, 0.9 * v), TeRated = Pr * 1000 / Math.max(v, 0.1), T0rec = 0.6 * TeRated + Tc, T0 = T0rec * s.pre / 100 * (1 - s.elong * 8), TeMax = 2.2 * Math.max(0, T0 - Tc), u = TeReq / Math.max(TeMax, 1e-6), jump = u > 1 || s.jumped;
    const Te = jump ? 0 : TeReq, T1 = T0 + Te / 2, T2 = Math.max(0, T0 - Te / 2), n2 = jump ? 0 : n2nom, P = s.Tload * n2 * 2 * Math.PI / 60 / 1000, teeth = Math.round(Math.PI * d1 / 8), zmesh = Math.round(teeth * theta / (2 * Math.PI));
    Object.assign(out, { sec: 'HTD 8M-30', mu: 0, E: Infinity, Tc, Pr, T0rec, T0, TeMax, TeMaxBelt: TeMax, u, slip: 0, gross: false, Te, T1, T2, n2, P, eta: jump ? 0 : 98, Pslip: 0, Tst: s.Tamb + 10 + 0.5 * v + 2 * s.misal * s.misal, belts: Math.ceil(s.Tload * n2nom * 2 * Math.PI / 60 / 1000 * SF / Pr), brg: (T1 + T2) * Math.cos((Math.PI - theta) / 2), life: (1 - 0.3 * s.misal) * (s.pre > 150 ? 0.6 : 1), jump, teeth, zmesh });
  } else {
    const ch = CHAIN[s.sec] || CHAIN['12B'], pull = TeReq, Fc = ch.m * v * v, sf = ch.Fb / Math.max(pull + Fc, 1), chordal = (1 - Math.cos(Math.PI / g.z1)) * 100, lube = v < 1 ? 'manual / drip' : v < 4 ? 'drip' : v < 10 ? 'oil bath' : 'forced stream', jump = s.elong >= 0.03 || s.jumped;
    const n2 = jump ? 0 : n2nom, P = s.Tload * n2 * 2 * Math.PI / 60 / 1000, eta = jump ? 0 : (97 - 0.4 * v - (s.elong > 0.015 ? 2 : 0));
    Object.assign(out, { sec: s.sec, mu: 0, E: Infinity, Tc: Fc, Pr: ch.Fb / 8 * v / 1000, T0rec: 0, T0: 0, TeMax: ch.Fb / 8, TeMaxBelt: ch.Fb / 8, u: (pull + Fc) / (ch.Fb / 8), slip: 0, gross: false, Te: pull, T1: pull + Fc, T2: Fc, n2, P, eta, Pslip: 0, Tst: s.Tamb + 8 + 1.2 * v, belts: 1, brg: pull + 2 * Fc, life: 1 - s.elong * 25, jump, sf, chordal, lube, pitch: ch.p });
  }
  out.PdesignKW = s.Tload * n2nom * 2 * Math.PI / 60 / 1000; return out;
}
function step(dt) {
  const s = state; Object.assign(R, solve());
  if (s.run) { const tau = 30; s.T += (R.Tst - s.T) * Math.min(1, dt * s.tscale / tau); const wear = s.type === 'chain' ? 4e-7 * (R.u + 0.2) * (R.v + 0.5) * (1 + 3 * s.misal) : 1.5e-7 * (R.slip * 40 + 0.3) * (1 + 3 * s.misal) * (R.v + 1); s.elong = Math.min(0.05, s.elong + wear * dt * s.tscale / 60); if (s.T > T_FAIL && !s.glazed) { s.glazed = true; record('belt failed'); } if (R.jump && !s.jumped && (s.type === 'timing' || s.type === 'chain')) { s.jumped = true; record('jumped'); } }
  else s.T += (s.Tamb - s.T) * Math.min(1, dt * s.tscale / 60);
  if (s.run && s.type !== 'chain' && s.type !== 'timing') { s.ang1 += s.n1 / 60 * 360 * dt * 0.05; s.ang2 += R.n2 / 60 * 360 * dt * 0.05; s.dash += R.v * 40 * dt * 0.05 * (1 - R.slip); } else if (s.run) { s.ang1 += s.n1 / 60 * 360 * dt * 0.05; s.ang2 += R.n2 / 60 * 360 * dt * 0.05; s.dash += R.v * 40 * dt * 0.05; }
  s.traceAcc += dt; if (s.traceAcc >= 0.2) { s.traceAcc = 0; s.trace.push({ t: s.t, n2: s.run ? R.n2 : 0, slip: s.run ? R.slip * 100 : 0, T: s.T, e: s.elong * 100 }); if (s.trace.length > 900) s.trace.shift(); }
  if (s.run) { if (R.gross) s.flags.slipload = true; if (s.T > T_ALARM) s.flags.hot = true; if (s.pre >= 160 && s.type !== 'chain') s.flags.tight = true; if (s.type === 'timing' && R.jump) s.flags.timing = true; }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg;
function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Belt drive' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, k = Math.min(0.9, 540 / r.C, 125 / (Math.max(r.d1, r.d2) / 2)), x1 = 190, y = 175, x2 = x1 + r.C * k, r1 = Math.max(16, r.d1 / 2 * k), r2 = Math.max(16, r.d2 / 2 * k);
  const a = Math.asin((r2 - r1) / (r.C * k)), tx1 = x1 - r1 * Math.sin(a), ty1 = y - r1 * Math.cos(a), tx2 = x2 - r2 * Math.sin(a), ty2 = y - r2 * Math.cos(a), bx1 = x1 - r1 * Math.sin(a), by1 = y + r1 * Math.cos(a), bx2 = x2 - r2 * Math.sin(a), by2 = y + r2 * Math.cos(a);
  el('rect', { x: x1 - 60, y: y + r1 + 24, width: 120, height: 34, rx: 5, class: 'motor' }, svg); txt(x1, y + r1 + 46, `MOTOR ${s.n1} rpm`, 'lbl', svg, 'middle'); el('rect', { x: x2 - 60, y: y + r2 + 24, width: 120, height: 34, rx: 5, class: 'load' }, svg); txt(x2, y + r2 + 46, `LOAD ${s.Tload} N·m`, 'lbl', svg, 'middle');
  const beltPath = `M${tx1},${ty1} L${tx2},${ty2} A${r2},${r2} 0 ${Math.PI + 2 * a > Math.PI ? 1 : 0} 1 ${bx2},${by2} L${bx1},${by1} A${r1},${r1} 0 ${Math.PI - 2 * a > Math.PI ? 1 : 0} 1 ${tx1},${ty1} Z`;
  const hot = s.T > T_ALARM || s.glazed; el('path', { d: beltPath, class: 'belt' + (hot ? ' hot' : '') + (s.type === 'chain' ? ' chain' : '') }, svg); el('line', { x1: bx1, y1: by1, x2: bx2, y2: by2, class: 'belt slack', stroke: 'var(--panel)' }, svg);
  el('path', { d: beltPath, class: 'belt-dash', style: `stroke-dashoffset:${-s.dash}` }, svg);
  for (const [cx, r0, ang, name, teeth] of [[x1, r1, s.ang1, 'driver', r.z1], [x2, r2, s.ang2, 'driven', r.z2]]) { el('circle', { cx, cy: y, r: r0, class: s.type === 'chain' ? 'sprocket' : 'pulley' }, svg); if (s.type === 'chain' || s.type === 'timing') { const n = teeth || Math.round(Math.PI * (cx === x1 ? r.d1 : r.d2) / 8); for (let i = 0; i < Math.min(n, 60); i++) { const t = ang * Math.PI / 180 + i * 2 * Math.PI / Math.min(n, 60); el('circle', { cx: cx + r0 * Math.cos(t), cy: y + r0 * Math.sin(t), r: 2.5, class: 'tooth' }, svg); } } el('circle', { cx, cy: y, r: Math.max(6, r0 * 0.25), class: 'pulley-hub' }, svg); el('line', { x1: cx, y1: y, x2: cx + r0 * 0.85 * Math.cos(ang * Math.PI / 180), y2: y + r0 * 0.85 * Math.sin(ang * Math.PI / 180), class: 'mark' }, svg); txt(cx, y - r0 - 26, `${name} · ${s.type === 'chain' ? teeth + ' teeth' : fmt(cx === x1 ? r.d1 : r.d2, 0) + ' mm'} · ${fmt(cx === x1 ? s.n1 : r.n2, 0)} rpm`, 'lbl', svg, 'middle'); }
  const wr = r1 + 10; el('path', { d: `M${x1 + wr * Math.cos(Math.PI / 2 + a)},${y - wr * Math.sin(Math.PI / 2 + a)} A${wr},${wr} 0 ${r.theta > 180 ? 1 : 0} 0 ${x1 + wr * Math.cos(-Math.PI / 2 - a)},${y - wr * Math.sin(-Math.PI / 2 - a)}`, class: 'arc' }, svg); txt(Math.max(x1 - wr - 8, 64), y + 4, `θ ${fmt(r.theta, 0)}°`, 'val', svg, 'end');
  txt((tx1 + tx2) / 2, (ty1 + ty2) / 2 - 12, `T₁ ${fmt(r.T1, 0)} N${s.type === 'v' && s.z > 1 ? ' per belt' : ''} · tight`, 'ftxt t1', svg, 'middle'); txt((bx1 + bx2) / 2, (by1 + by2) / 2 + 22, `T₂ ${fmt(r.T2, 0)} N · slack`, 'ftxt t2', svg, 'middle');
  const bl = clamp(r.brg / 40, 10, 80), ax = x2 + r2 + 4; el('line', { x1: ax, y1: y, x2: ax + bl, y2: y, class: 'brg' }, svg); el('polygon', { points: `${ax + bl + 8},${y} ${ax + bl},${y - 5} ${ax + bl},${y + 5}`, class: 'brghead' }, svg); if (ax + 118 < 958) txt(ax + 2, y - 8, `shaft load ${fmt(r.brg, 0)} N`, 'lbl', svg, 'start'); else txt(956, y + 20, `shaft load ${fmt(r.brg, 0)} N`, 'lbl', svg, 'end');
  txt(480, 370, `${r.sec} · ${s.type === 'chain' ? 'chain pull ' + fmt(r.Te, 0) + ' N · safety factor ' + fmt(r.sf, 1) + ' · chordal ' + fmt(r.chordal, 2) + ' % · ' + r.lube + ' lubrication' : 'belt speed ' + fmt(r.v, 1) + ' m/s · length ' + fmt(r.Lb, 0) + ' mm · installation tension ' + fmt(r.T0, 0) + ' N (' + s.pre + ' % of ' + fmt(r.T0rec, 0) + ')'} · temperature ${fmt(s.T, 0)} °C · wear ${fmt(s.elong * 100, 2)} %`, 'val' + (r.gross || r.jump || hot ? ' bad' : ''), svg, 'middle');
  txt(480, 391, r.small ? `PULLEY TOO SMALL for ${r.sec}: minimum ${r.dmin} mm — the belt bends too hard and its rating collapses` : r.gross ? `GROSS SLIP — transmitted ${fmt(r.Te, 0)} of ${fmt(r.TeReq, 0)} N needed · driven speed falling · belt heating` : r.jump ? (s.type === 'chain' ? 'CHAIN JUMPING THE SPROCKET — elongation past 3 %' : 'TOOTH JUMP — tension too low for the load') : s.glazed ? 'BELT GLAZED / FAILED — fit a new belt' : `creep ${fmt(r.slip * 100, 2)} % · ${fmt(r.P, 2)} kW delivered · ${r.belts} belt${r.belts > 1 ? 's' : ''} required for ${fmt(r.PdesignKW, 1)} kW × SF ${SF}`, 'val' + (r.gross || r.jump || s.glazed ? ' bad' : ''), svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawChart() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 60, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  if (s.type === 'chain') {
    const z = R.z1, X = f => L + f * pw, vmax = R.v * 1.05, vmin = R.v * Math.cos(Math.PI / z) * 0.95, Y = v => T + (1 - (v - vmin) / Math.max(1e-6, vmax - vmin)) * ph;
    ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 2.5; for (let i = 0; i <= 200; i++) { const f = i / 200, phase = (f * 3 % 1) * 2 * Math.PI / z - Math.PI / z, v = R.v * Math.cos(phase); i ? ctx.lineTo(X(f), Y(v)) : ctx.moveTo(X(f), Y(v)); } ctx.stroke();
    ctx.strokeStyle = col('--steel'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(L, Y(R.v)); ctx.lineTo(L + pw, Y(R.v)); ctx.stroke(); ctx.setLineDash([]); ctx.textAlign = 'right'; ctx.fillText(fmt(R.v, 3) + ' m/s', L - 4, Y(R.v) + 3); ctx.fillText(fmt(R.v * Math.cos(Math.PI / z), 3), L - 4, Y(R.v * Math.cos(Math.PI / z)) + 3);
    ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.textAlign = 'left'; ctx.fillText(`chordal speed variation ${fmt(R.chordal, 2)} % with ${z} teeth · three tooth engagements shown · 17 teeth or more recommended`, L + 6, T + 16); ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('chain speed over three tooth pitches', L + pw / 2, H - 6); return;
  }
  const Tmax = 600, X = tq => L + tq / Tmax * pw, temax = Math.max(R.TeMax * 1.3, R.TeReq * 1.1, 100), Y = te => T + (1 - clamp(te / temax, 0, 1)) * ph, Ys = sl => T + (1 - clamp(sl / 100, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); for (let tq = 0; tq <= Tmax; tq += 100) { ctx.beginPath(); ctx.moveTo(X(tq), T); ctx.lineTo(X(tq), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(tq, X(tq), H - 16); }
  ctx.textAlign = 'right'; for (let f = 0; f <= 1; f += 0.25) ctx.fillText(fmt(temax * f, 0), L - 4, Y(temax * f) + 3); ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); for (const sl of [0, 25, 50, 100]) ctx.fillText(sl + ' %', L + pw + 4, Ys(sl) + 3);
  ctx.beginPath(); ctx.strokeStyle = col('--good'); ctx.lineWidth = 2; ctx.moveTo(L, Y(R.TeMax)); ctx.lineTo(L + pw, Y(R.TeMax)); ctx.stroke(); ctx.fillStyle = col('--good'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`capacity ${fmt(R.TeMax, 0)} N at this tension and wrap`, L + pw - 6, Y(R.TeMax) - 6); ctx.textAlign = 'left';
  const saveT = s.Tload; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 2; for (let tq = 0; tq <= Tmax; tq += 5) { s.Tload = tq; const r = solve(); tq ? ctx.lineTo(X(tq), Y(r.Te)) : ctx.moveTo(X(tq), Y(r.Te)); } ctx.stroke();
  ctx.beginPath(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 1.8; ctx.setLineDash([5, 3]); for (let tq = 0; tq <= Tmax; tq += 5) { s.Tload = tq; const r = solve(); const v = s.type === 'timing' ? (r.jump ? 100 : 0) : r.slip * 100; tq ? ctx.lineTo(X(tq), Ys(v)) : ctx.moveTo(X(tq), Ys(v)); } ctx.stroke(); ctx.setLineDash([]); s.Tload = saveT;
  ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(X(s.Tload), Y(R.Te), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = X(s.Tload) > L + pw * 0.6 ? 'right' : 'left'; ctx.fillText(`${s.Tload} N·m needs ${fmt(R.TeReq, 0)} N · slip ${fmt(R.slip * 100, 1)} %`, X(s.Tload) + (X(s.Tload) > L + pw * 0.6 ? -10 : 10), Y(R.Te) - 10);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('load torque at the driven shaft, N·m · left axis transmitted tension N · right axis slip', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, span = 60, tr = state.trace.filter(p => p.t > state.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const X = t => L + (t - (state.t - span)) / span * pw;
  const nmax = Math.max(1, R.n2nom * 1.1), series = (key, color, lo, hi, w = 1.6) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { const y = T + (1 - clamp((p[key] - lo) / (hi - lo), 0, 1)) * ph; f ? ctx.moveTo(X(p.t), y) : ctx.lineTo(X(p.t), y); f = false; } ctx.stroke(); };
  series('n2', col('--water'), 0, nmax, 2.2); series('slip', col('--alarm'), 0, 100); series('T', col('--brass'), 0, 150); series('e', '#7a5a2a', 0, 5);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`${fmt(nmax, 0)} rpm`, L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('driven rpm', L + pw + 4, T + 10); ctx.fillStyle = col('--alarm'); ctx.fillText('slip %', L + pw + 4, T + 24); ctx.fillStyle = col('--brass'); ctx.fillText('belt °C', L + pw + 4, T + 38); ctx.fillStyle = '#7a5a2a'; ctx.fillText('wear %', L + pw + 4, T + 52);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s · wear clock ×${state.tscale}`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R; $('#pillRun').textContent = s.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', s.run);
  $('#roRatio').textContent = fmt(r.ratio, 2); $('#roRatioA').textContent = r.n2 > 0 ? fmt(s.n1 / r.n2, 2) : '—'; $('#roN2').textContent = fmt(r.n2, 0); $('#roSlip').textContent = fmt(r.slip * 100, 2); $('#roSlipNote').textContent = s.type === 'chain' || s.type === 'timing' ? 'positive drive, no creep' : r.gross ? 'gross slip' : 'creep'; $('#roSlipBox').classList.toggle('alarm', r.gross); $('#roV').textContent = fmt(r.v, 2); $('#roLen').textContent = fmt(r.Lb, 0); $('#roWrap').textContent = fmt(r.theta, 0); $('#roEmu').textContent = Number.isFinite(r.E) ? fmt(r.E, 2) : 'positive';
  $('#roT1').textContent = fmt(r.T1, 0); $('#roT2').textContent = fmt(r.T2, 0); $('#roTc').textContent = fmt(r.Tc, 0); $('#roTe').textContent = fmt(r.Te, 0); $('#roTeMax').textContent = fmt(r.TeMax, 0); $('#roTeBox').classList.toggle('alarm', r.u > 1); $('#roTeBox').classList.toggle('warnv', r.u > 0.8 && r.u <= 1); $('#roP').textContent = fmt(r.P, 2); $('#roEta').textContent = fmt(r.eta, 1);
  $('#roBelts').textContent = s.type === 'chain' ? '—' : r.belts; $('#roZ').textContent = s.z; $('#roBeltsBox').classList.toggle('alarm', s.type !== 'chain' && r.belts > s.z); $('#roBrg').textContent = fmt(r.brg, 0); $('#roBrgBox').classList.toggle('warnv', s.pre >= 160); $('#roTmp').textContent = fmt(s.T, 0); $('#roLife').textContent = fmt(r.life, 2); $('#roTmpBox').classList.toggle('alarm', s.T > T_ALARM);
  const ps = $('#pillState'); ps.textContent = s.glazed ? 'Belt failed' : r.gross ? 'Slipping' : r.jump ? 'Jumped' : s.T > T_ALARM ? 'Running hot' : `${fmt(r.P, 1)} kW · η ${fmt(r.eta, 0)} %`; ps.className = 'pill ' + (s.glazed || r.gross || r.jump ? 'warn' : s.T > T_ALARM ? 'amber' : 'ok'); $('#pillRatio').textContent = `ratio ${fmt(r.ratio, 2)} · wrap ${fmt(r.theta, 0)}°`;
  $('#pillSlip').classList.toggle('hidden', !r.gross); $('#pillJump').classList.toggle('hidden', !r.jump); $('#pillHot').classList.toggle('hidden', s.T <= T_ALARM); $('#pillBrg').classList.toggle('hidden', s.pre < 160 || s.type === 'chain');
  $('#rigTitle').textContent = { v: 'V-belt drive · tight side on top, slack side below', flat: 'Flat belt drive · friction only, no groove', timing: 'Synchronous belt · teeth carry the load, no creep', chain: 'Roller chain · sprockets and chordal action' }[s.type]; $('#chartTitle').textContent = s.type === 'chain' ? 'Chordal action · chain speed pulses once per tooth' : 'Traction · transmitted tension against load, with the capacity of this belt and tension'; $('#btnRun').textContent = s.run ? 'Running' : 'Start';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, type: s.type, sec: r.sec, d1: +r.d1.toFixed(0), d2: +r.d2.toFixed(0), C: s.C, z: s.z, n1: s.n1, Tload: s.Tload, pre: s.pre, wrap: +r.theta.toFixed(0), T1: +r.T1.toFixed(0), T2: +r.T2.toFixed(0), Te: +r.Te.toFixed(0), cap: +r.TeMax.toFixed(0), slip: +(r.slip * 100).toFixed(2), P: +r.P.toFixed(2), eta: +r.eta.toFixed(1), belts: r.belts, brg: +r.brg.toFixed(0), chordal: r.chordal ? +r.chordal.toFixed(2) : '', z1: r.z1 || '' }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.type, r.sec, r.d1, r.d2, r.C, r.z, r.n1, r.Tload, r.pre, r.wrap, r.T1, r.T2, r.Te, r.cap, r.slip, r.P, r.eta, r.belts, r.brg].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,type,section,d1_mm,d2_mm,C_mm,belts,n1_rpm,load_Nm,tension_pct,wrap_deg,T1_N,T2_N,Te_N,capacity_N,slip_pct,kW,eta_pct,belts_required,bearing_N,chordal_pct,z1\n' + state.log.map(r => [r.i, r.tag, r.type, r.sec, r.d1, r.d2, r.C, r.z, r.n1, r.Tload, r.pre, r.wrap, r.T1, r.T2, r.Te, r.cap, r.slip, r.P, r.eta, r.belts, r.brg, r.chordal, r.z1].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'slipload', t: 'Load it until it slips', d: 'Raise the load torque until the V-belt passes its capacity: watch the driven speed fall and the belt heat.', ok: () => state.flags.slipload },
  { id: 'tension', t: 'Tension sets capacity', d: 'Record the same load at two installation tensions and compare slip and capacity.', ok: () => { const L = state.log.filter(r => r.type === 'v' || r.type === 'flat'); return L.some(a => L.some(b => a.pre !== b.pre && a.Tload === b.Tload && a.type === b.type && a.d1 === b.d1)); } },
  { id: 'wrap', t: 'Wrap angle', d: 'Record two drives whose small-pulley wrap differs by 20° or more (change the ratio or the centre distance) and compare capacity.', ok: () => { const L = state.log.filter(r => r.type === 'v'); return L.some(a => L.some(b => Math.abs(a.wrap - b.wrap) >= 20 && a.sec === b.sec && a.pre === b.pre)); } },
  { id: 'vflat', t: 'V against flat', d: 'Record the same pulleys, tension setting and load with a V-belt and with a flat belt. Which carries more before slipping?', ok: () => { const L = state.log; return L.some(a => L.some(b => a.type === 'v' && b.type === 'flat' && a.d1 === b.d1 && a.d2 === b.d2 && a.Tload === b.Tload)); } },
  { id: 'select', t: 'Select the belts', d: 'For at least 10 kW at the load, choose a section and number of belts so that the belts required equals the belts fitted. Record it.', ok: () => state.log.some(r => r.type === 'v' && r.P >= 10 && r.belts === r.z && r.slip < 2) },
  { id: 'tight', t: 'Over-tensioned', d: 'Set the installation tension to 160 % or more and read the shaft load: the bearings pay for tension the drive does not need.', ok: () => state.flags.tight },
  { id: 'timing', t: 'Tooth jump', d: 'Synchronous belt: lower the tension or raise the load until the belt jumps a tooth.', ok: () => state.flags.timing },
  { id: 'chordal', t: 'Chordal action', d: 'Chain: record an 11-tooth driver and one with 19 teeth or more; compare the chordal speed variation.', ok: () => { const L = state.log.filter(r => r.type === 'chain'); return L.some(r => r.z1 <= 12) && L.some(r => r.z1 >= 19); } },
  { id: 'hot', t: 'Belt overheating', d: 'Run a belt in gross slip or with misaligned pulleys until the temperature alarm shows, then read the life factor.', ok: () => state.flags.hot },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawChart(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setType(t) { state.type = t; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === t)); const sel = $('#sec'); sel.innerHTML = t === 'v' ? Object.keys(SEC).map(k => `<option value="${k}">${k}</option>`).join('') : t === 'chain' ? Object.keys(CHAIN).map(k => `<option value="${k}">${k} · pitch ${CHAIN[k].p} mm</option>`).join('') : t === 'timing' ? '<option value="8M">HTD 8M · 30 mm wide</option>' : '<option value="flat">Flat 50 × 3 mm</option>'; state.sec = sel.value = t === 'v' ? 'SPA' : t === 'chain' ? '12B' : sel.options[0].value; $('#d1Name').textContent = t === 'chain' ? 'Driver sprocket (slider ÷ 5 = teeth)' : 'Driver pulley diameter'; $('#d2Name').textContent = t === 'chain' ? 'Driven sprocket (slider ÷ 5 = teeth)' : 'Driven pulley diameter'; $('#z').closest('.ctl').style.display = t === 'v' ? '' : 'none'; if (t !== 'v') state.z = 1; else state.z = +$('#z').value; state.jumped = false; state.glazed = false; state.elong = 0; }
const PRESETS = {
  design: () => { setType('v'); for (const [k, v] of Object.entries({ d1: 125, d2: 250, C: 600, z: 2, misal: 0, n1: 1450, Tload: 60, pre: 100, mu: 0.3, Tamb: 25 })) setCtl(k, v); state.z = 2; state.run = true; },
  overload: () => { PRESETS.design(); setCtl('Tload', 260); },
  loose: () => { PRESETS.design(); setCtl('pre', 45); },
  tight: () => { PRESETS.design(); setCtl('pre', 190); },
  bigratio: () => { PRESETS.design(); setCtl('d1', 90); setCtl('d2', 500); setCtl('C', 450); },
  flat: () => { PRESETS.design(); setType('flat'); },
  timing: () => { PRESETS.design(); setType('timing'); setCtl('pre', 40); setCtl('Tload', 120); },
  chain: () => { PRESETS.design(); setType('chain'); setCtl('d1', 55); setCtl('d2', 190); setCtl('n1', 300); setCtl('Tload', 150); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('d1', v => state.type === 'chain' ? clamp(Math.round(v / 5), 9, 57) + ' teeth' : v + ' mm'); bindR('d2', v => state.type === 'chain' ? clamp(Math.round(v / 5), 9, 114) + ' teeth' : v + ' mm'); bindR('C', v => v + ' mm'); bindR('z', v => String(v)); bindR('misal', v => v.toFixed(1) + '°'); bindR('n1', v => v.toLocaleString().replace(/,/g, ' ') + ' rpm'); bindR('Tload', v => v + ' N·m'); bindR('pre', v => v + ' %'); bindR('mu', v => v.toFixed(2)); bindR('Tamb', v => v + ' °C'); bindR('tscale', v => '×' + v);
  $('#sec').addEventListener('change', e => state.sec = e.target.value); document.querySelectorAll('#types button').forEach(b => b.onclick = () => setType(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnRetension').onclick = () => { state.elong = 0; state.jumped = false; state.glazed = false; state.T = state.Tamb; }; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('bd_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') state.run = !state.run; else if (k === 'R') record(''); else if (/^[1-4]$/.test(k)) setType(['v', 'flat', 'timing', 'chain'][+k - 1]); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawChart(); drawTrace(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.BeltDrive = {
  getState: () => ({ inputs: { type: state.type, d1: state.d1, d2: state.d2, C: state.C, sec: state.sec, z: state.z, misal: state.misal, n1: state.n1, Tload: state.Tload, pre: state.pre, mu: state.mu, run: state.run }, results: { ratio: R.ratio, n2: R.n2, v: R.v, wrap: R.theta, T1: R.T1, T2: R.T2, Te: R.Te, TeMax: R.TeMax, slip: R.slip, P: R.P, eta: R.eta, belts: R.belts, brg: R.brg, T: state.T, gross: R.gross, jump: R.jump, chordal: R.chordal }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.type) setType(inp.type); for (const k of ['d1', 'd2', 'C', 'z', 'misal', 'n1', 'Tload', 'pre', 'mu', 'Tamb', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.sec) { $('#sec').value = inp.sec; state.sec = inp.sec; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.retension) $('#btnRetension').click(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'belt-drive', title: 'Belt and Chain Drive Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.BeltDrive.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'belt-drive', state: window.BeltDrive.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('bd_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setType('v'); step(0); drawRig(); updateReadouts(); drawChart(); drawTrace();
  requestAnimationFrame(tick);
}
build();
