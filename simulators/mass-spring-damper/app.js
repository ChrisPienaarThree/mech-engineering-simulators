/* Mass-Spring-Damper Demonstrator — m x'' + c x' + k x = F(t) integrated with RK4 in real time (adjustable clock),
   with free vibration, harmonic force, base excitation, step and impulse. Measured frequency and log decrement come
   from the trace; a swept sine records settled amplitude ratios over the theoretical frequency response.
   Standalone, offline. Rig data illustrative. */
'use strict';

const XMAX = 0.12, SUB = 2400, REST = 0.5, GACC = 9.81;
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';

/* ------------------------------------------------------------------ state */
const state = {
  m: 2, k: 800, c: 4, tscale: 1, mode: 'free', x0: 0.05, v0: 0, F0: 10, Y0: 0.01, f: 2, sweep: false, imp: 1,
  run: false, x: 0.05, v: 0, t: 0, y: 0, F: 0, trace: [], traceAcc: 0, peaks: [], zeros: [], meas: [], sweepT: 0, sweepR: 0.2, sweepDwell: 0, phOff: 0, hits: 0, hitT: -9, log: [], flags: {}, portrait: [],
};
const R = {};

/* ------------------------------------------------------------------ theory */
function theory() {
  const { m, k, c } = state, wn = Math.sqrt(k / m), cc = 2 * Math.sqrt(k * m), z = c / cc, fn = wn / (2 * Math.PI);
  const wd = z < 1 ? wn * Math.sqrt(1 - z * z) : 0, tau = z > 0 ? 1 / (z * wn) : Infinity, delta = z < 1 ? 2 * Math.PI * z / Math.sqrt(1 - z * z) : Infinity;
  const f = excF(), r = f / fn, D = 1 / Math.sqrt((1 - r * r) ** 2 + (2 * z * r) ** 2), T = Math.sqrt(1 + (2 * z * r) ** 2) * D, phi = Math.atan2(2 * z * r, 1 - r * r) * 180 / Math.PI;
  return { wn, cc, z, fn, wd, fd: wd / (2 * Math.PI), tau, ts: 4 * tau, delta, r, D, T, phi, cls: z === 0 ? 'undamped' : z < 1 ? 'underdamped' : z === 1 || Math.abs(z - 1) < 0.02 ? 'critically damped' : 'overdamped' };
}
function excF() { if (!state.sweep || state.mode === 'free' || state.mode === 'step' || state.mode === 'impulse') return state.f; const fn = Math.sqrt(state.k / state.m) / (2 * Math.PI); return fn * state.sweepR; }
function force(t) { const w = 2 * Math.PI * excF(); if (state.mode === 'harmonic') return state.F0 * Math.sin(w * t + state.phOff); if (state.mode === 'step') return state.F0; return 0; }
function baseY(t) { if (state.mode !== 'base') return [0, 0]; const w = 2 * Math.PI * excF(); return [state.Y0 * Math.sin(w * t + state.phOff), state.Y0 * w * Math.cos(w * t + state.phOff)]; }

/* ------------------------------------------------------------------ integration */
function accel(t, x, v) { const [y, yd] = baseY(t); return (force(t) - state.c * (v - yd) - state.k * (x - y)) / state.m; }
function rk4(dt) {
  const t = state.t, x = state.x, v = state.v;
  const a1 = accel(t, x, v), k1x = v, k1v = a1;
  const k2x = v + k1v * dt / 2, k2v = accel(t + dt / 2, x + k1x * dt / 2, k2x);
  const k3x = v + k2v * dt / 2, k3v = accel(t + dt / 2, x + k2x * dt / 2, k3x);
  const k4x = v + k3v * dt, k4v = accel(t + dt, x + k3x * dt, k4x);
  state.x += dt * (k1x + 2 * k2x + 2 * k3x + k4x) / 6; state.v += dt * (k1v + 2 * k2v + 2 * k3v + k4v) / 6; state.t += dt;
  if (Math.abs(state.x) > XMAX) { state.x = Math.sign(state.x) * XMAX; if (Math.sign(state.v) === Math.sign(state.x)) { state.v = -REST * state.v; state.hits++; state.hitT = state.t; } }
}
function step(dtReal) {
  if (!state.run) return; const dt = dtReal * state.tscale, n = Math.max(1, Math.round(dt * SUB)), h = dt / n;
  const xPrev = state.x;
  for (let i = 0; i < n; i++) { const xb = state.x, vb = state.v; rk4(h); if (vb > 0 && state.v <= 0 && state.x > 0.002) state.peaks.push({ t: state.t, x: state.x }); if (xb <= 0 && state.x > 0) state.zeros.push(state.t); }
  if (state.sweep && (state.mode === 'harmonic' || state.mode === 'base')) {   // stepped sine: dwell at each frequency until settled, log the amplitude ratio, step r
    state.sweepT += dt; state.sweepDwell += dt; const th = theory(), f = excF(), dwell = clamp(4 * th.tau, 1.5, 8) + 3 / f;
    if (state.sweepDwell >= dwell) { measure(); if (Number.isFinite(R.ratioM)) state.meas.push({ r: state.sweepR, v: R.ratioM, base: state.mode === 'base' }); const wOld = 2 * Math.PI * f; state.sweepR += state.sweepR >= 0.78 && state.sweepR < 1.25 ? 0.025 : 0.05; state.sweepDwell = 0; const wNew = 2 * Math.PI * excF(); state.phOff = (wOld * state.t + state.phOff) - wNew * state.t; if (state.sweepR > 3.001) { state.sweep = false; $('#sweep').checked = false; } }
  }
  if (state.peaks.length > 40) state.peaks.shift(); if (state.zeros.length > 40) state.zeros.shift();
  const [y] = baseY(state.t); state.y = y; state.F = force(state.t);
  state.traceAcc += dt; if (state.traceAcc >= 1 / 120) { state.traceAcc = 0; state.trace.push({ t: state.t, x: state.x, y, F: state.F, v: state.v }); if (state.trace.length > 1200 * 2) state.trace.shift(); state.portrait.push([state.x, state.v]); if (state.portrait.length > 900) state.portrait.shift(); }
  measure();
}

/* ------------------------------------------------------------------ measurement from the trace */
function measure() {
  const th = theory(), tr = state.trace; R.th = th;
  const pk = state.peaks.slice(-6); R.fMeas = pk.length >= 3 ? (pk.length - 1) / (pk[pk.length - 1].t - pk[0].t) : NaN;
  if (state.mode === 'free' || state.mode === 'impulse') { const p = state.peaks.slice(-4); R.deltaM = p.length >= 2 && p[p.length - 2].x > 0.003 ? Math.log(p[p.length - 2].x / p[p.length - 1].x) : NaN; R.zM = Number.isFinite(R.deltaM) ? R.deltaM / Math.sqrt(4 * Math.PI * Math.PI + R.deltaM * R.deltaM) : NaN; } else { R.deltaM = NaN; R.zM = NaN; }
  const f = excF(), win = state.mode === 'free' || state.mode === 'impulse' || state.mode === 'step' ? (th.fd > 0 ? 1.1 / th.fd : 2) : 1.1 / f;
  const recent = tr.filter(p => p.t > state.t - win); let mx = -Infinity, mn = Infinity; for (const p of recent) { if (p.x > mx) mx = p.x; if (p.x < mn) mn = p.x; }
  R.amp = recent.length > 5 ? (mx - mn) / 2 : 0; R.mid = recent.length > 5 ? (mx + mn) / 2 : 0;
  R.ratioM = state.mode === 'harmonic' && state.F0 > 0 ? R.amp * state.k / state.F0 : state.mode === 'base' && state.Y0 > 0 ? R.amp / state.Y0 : NaN;
  R.vmax = recent.reduce((s, p) => Math.max(s, Math.abs(p.v)), 0); R.amax = R.amp * (2 * Math.PI * (state.mode === 'harmonic' || state.mode === 'base' ? f : th.fd || th.fn)) ** 2;
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; const SC = 1000, MY = 330;   // px per m, mass rest centre (mass sits on the spring above the base at y = 520)
function buildRig() { svg = el('svg', { viewBox: '0 0 900 560', role: 'img', 'aria-label': 'Mass-spring-damper rig' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig() {
  svg.innerHTML = ''; const th = R.th, x = state.x, y = state.y;
  const baseMode = state.mode === 'base', bottomY = 520 - y * SC;   // base moves in base mode; the mass hangs from the base via spring and damper
  const massY = MY - x * SC;
  // frame / base
  if (baseMode) { el('rect', { x: 150, y: bottomY, width: 300, height: 14, class: 'base' }, svg); el('rect', { x: 270, y: bottomY + 14, width: 60, height: 26, class: 'shaker' }, svg); txt(300, bottomY + 32, 'SHAKER', 'lbl', svg, 'middle'); txt(455, bottomY + 10, `y ${fmt(y * 1000, 1)} mm`, 'eqn', svg); }
  else { el('rect', { x: 150, y: 520, width: 300, height: 14, class: 'base' }, svg); for (let i = 150; i < 450; i += 14) el('line', { x1: i, y1: 534, x2: i - 8, y2: 544, class: 'hatch' }, svg); }
  const anchorY = baseMode ? bottomY : 520;
  // spring (left) from base to mass bottom
  const sx = 245, sTop = massY + 40, sBot = anchorY, n = 10, len = sBot - sTop, hot = Math.abs(x - y) > XMAX * 0.95; let d = `M${sx},${sBot} v-12`; for (let i = 0; i < n; i++) d += ` l${i % 2 ? -18 : 18},${-(len - 24) / n}`; d += ` L${sx},${sTop}`; el('path', { d, class: 'spring' + (hot ? ' hot' : '') }, svg);
  // damper (right)
  const dx = 355, cylTop = anchorY - 120, cylH = 90; el('rect', { x: dx - 14, y: cylTop, width: 28, height: cylH, class: 'damper-fill' }, svg); el('path', { d: `M${dx - 14},${cylTop} V${cylTop + cylH} H${dx + 14} V${cylTop}`, class: 'damper' }, svg); el('line', { x1: dx, y1: anchorY, x2: dx, y2: cylTop + cylH, class: 'damper' }, svg);
  const pistY = clamp(massY + 40 + (cylTop + cylH / 2 - (MY + 40)), cylTop + 4, cylTop + cylH - 4); el('rect', { x: dx - 11, y: pistY - 3, width: 22, height: 6, class: 'damper-piston' }, svg); el('line', { x1: dx, y1: pistY, x2: dx, y2: massY + 40, class: 'damper' }, svg);
  // stops
  el('rect', { x: 205, y: MY - XMAX * SC - 46, width: 190, height: 6, class: 'stop' + (state.t - state.hitT < 0.3 && x > 0 ? ' hit' : '') }, svg); el('rect', { x: 205, y: MY + XMAX * SC + 40, width: 190, height: 6, class: 'stop' + (state.t - state.hitT < 0.3 && x < 0 ? ' hit' : '') }, svg);
  // mass with plates
  const plates = clamp(Math.round(state.m / 1), 1, 12), ph = clamp(60 / plates, 4, 12); el('rect', { x: 215, y: massY - 40, width: 170, height: 80, rx: 4, class: 'mass' }, svg); for (let i = 0; i < plates; i++) el('rect', { x: 222 + (i % 2) * 3, y: massY + 36 - (i + 1) * ph, width: 156 - (i % 2) * 6, height: ph - 1.5, class: 'plate' }, svg);
  txt(300, massY - 24, `m = ${state.m} kg`, 'mass-txt', svg);
  // force actuator (harmonic / step / impulse)
  if (state.mode !== 'free' && state.mode !== 'base') { const F = state.mode === 'impulse' ? 0 : state.F, L = clamp(Math.abs(F) / 100 * 90, 0, 90); if (L > 2) { const up = F > 0, y1 = massY - 44, y2 = y1 - (up ? L : -L); el('line', { x1: 300, y1: y2, x2: 300, y2: y1 - (up ? 10 : -10), class: 'force' }, svg); el('polygon', { points: `300,${y1} 294,${y1 - (up ? 10 : -10)} 306,${y1 - (up ? 10 : -10)}`, class: 'force-head' }, svg); } txt(310, massY - 60, `F ${fmt(F, 1)} N`, 'force-txt', svg); }
  // ruler
  for (let mm = -120; mm <= 120; mm += 20) { const yy = MY - mm / 1000 * SC; el('line', { x1: 430, y1: yy, x2: mm % 40 === 0 ? 446 : 440, y2: yy, class: 'ruler' }, svg); if (mm % 40 === 0) txt(450, yy + 3, mm + '', 'ruler-txt', svg); }
  el('line', { x1: 430, y1: MY - 0.13 * SC, x2: 430, y2: MY + 0.13 * SC, class: 'ruler' }, svg); el('polygon', { points: `428,${massY} 418,${massY - 5} 418,${massY + 5}`, class: 'pointer' }, svg); txt(452, MY - 0.13 * SC - 4, 'x, mm', 'lbl', svg);
  // equation and status panel on the right
  txt(560, 60, 'm ẍ + c ẋ + k x = F(t)', 'eqn', svg); txt(560, 80, `${state.m} kg · ${state.k} N/m · ${state.c} N·s/m`, 'eqn', svg);
  txt(560, 110, `ωn ${fmt(th.wn, 1)} rad/s · fn ${fmt(th.fn, 2)} Hz · ζ ${fmt(th.z, 3)}`, 'eqn', svg); txt(560, 130, th.cls.toUpperCase(), 'lbl', svg);
  const modeTxt = { free: `free vibration from x₀ ${fmt(state.x0 * 1000, 0)} mm, v₀ ${fmt(state.v0, 1)} m/s`, harmonic: `F = ${state.F0} N · sin(2π·${fmt(excF(), 2)} t)`, base: `y = ${state.Y0 * 1000} mm · sin(2π·${fmt(excF(), 2)} t)`, step: `step force ${state.F0} N`, impulse: `impulse ${state.imp} N·s` }; txt(560, 160, modeTxt[state.mode], 'eqn', svg);
  txt(560, 190, `x ${fmt(x * 1000, 1)} mm · ẋ ${fmt(state.v, 2)} m/s`, 'eqn', svg); txt(560, 210, `spring force ${fmt(state.k * (x - y), 1)} N · damper ${fmt(state.c * (state.v - baseY(state.t)[1]), 1)} N`, 'eqn', svg);
  txt(560, 240, `energy ${fmt(0.5 * state.k * (x - y) ** 2 + 0.5 * state.m * state.v ** 2, 3)} J · stops hit ${state.hits}`, 'eqn', svg);
  if (state.sweep && state.run && (state.mode === 'harmonic' || state.mode === 'base')) txt(560, 270, `stepped sweep · r ${fmt(state.sweepR, 3)} · ${fmt(state.sweepT, 0)} s · ${state.meas.length} points`, 'eqn', svg);
  // small energy bars
  const Ek = 0.5 * state.m * state.v ** 2, Ep = 0.5 * state.k * (x - y) ** 2, Emax = Math.max(1e-6, 0.5 * state.k * XMAX ** 2); txt(560, 310, 'ENERGY', 'lbl', svg); el('rect', { x: 560, y: 318, width: 300, height: 10, fill: 'var(--panel2)' }, svg); el('rect', { x: 560, y: 318, width: 300 * clamp(Ep / Emax, 0, 1), height: 10, fill: 'var(--brass)' }, svg); el('rect', { x: 560 + 300 * clamp(Ep / Emax, 0, 1), y: 318, width: 300 * clamp(Ek / Emax, 0, 1 - clamp(Ep / Emax, 0, 1)), height: 10, fill: 'var(--water)' }, svg); txt(560, 342, 'potential (spring) · kinetic (mass)', 'lbl', svg);
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 56, T = 10, B = 24, pw = W - L - Rr, ph = H - T - B, span = 10, t0 = state.t - span, X = t => L + (t - t0) / span * pw, tr = state.trace;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.strokeStyle = col('--grid'); for (let s = 0; s <= span; s++) { ctx.beginPath(); ctx.moveTo(X(t0 + s), T); ctx.lineTo(X(t0 + s), T + ph); ctx.stroke(); }
  const xm = Math.max(0.02, ...tr.map(p => Math.abs(p.x)), Math.abs(state.Y0)) * 1.1, Y = v => T + ph / 2 - v / xm * ph / 2; ctx.strokeStyle = col('--steel'); ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(L + pw, Y(0)); ctx.stroke();
  ctx.setLineDash([3, 3]); ctx.strokeStyle = col('--alarm'); ctx.beginPath(); ctx.moveTo(L, Y(XMAX)); ctx.lineTo(L + pw, Y(XMAX)); ctx.moveTo(L, Y(-XMAX)); ctx.lineTo(L + pw, Y(-XMAX)); ctx.stroke(); ctx.setLineDash([]);
  const series = (key, color, scale, w) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { if (p.t < t0) continue; const yv = Y(p[key] * scale); f ? ctx.moveTo(X(p.t), yv) : ctx.lineTo(X(p.t), yv); f = false; } ctx.stroke(); };
  if (state.mode === 'base') series('y', col('--brass'), 1, 1.5); if (state.mode === 'harmonic' || state.mode === 'step') series('F', col('--brass'), xm / Math.max(1, state.F0), 1.2);
  series('x', col('--water'), 1, 2);
  if (state.mode === 'free' || state.mode === 'impulse') { const th = R.th; if (th.z > 0 && th.z < 1 && state.peaks.length) { const p0 = state.peaks[0]; ctx.beginPath(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 1; ctx.setLineDash([4, 3]); let f = true; for (let t = Math.max(t0, p0.t); t <= state.t; t += 0.05) { const e = p0.x * Math.exp(-th.z * th.wn * (t - p0.t)); f ? ctx.moveTo(X(t), Y(e)) : ctx.lineTo(X(t), Y(e)); f = false; } ctx.stroke(); ctx.setLineDash([]); } for (const p of state.peaks) if (p.t >= t0) { ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(p.t), Y(p.x), 3, 0, 7); ctx.fill(); } }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`${fmt(xm * 1000, 0)} mm`, L - 4, T + 10); ctx.fillText('0', L - 4, Y(0) + 3); ctx.fillText(`−${fmt(xm * 1000, 0)}`, L - 4, T + ph); ctx.textAlign = 'center'; for (let s = 0; s <= span; s += 2) ctx.fillText(fmt(t0 + s, 0) + ' s', X(t0 + s), H - 8);
  ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); if (state.mode === 'harmonic' || state.mode === 'step') ctx.fillText(`F scale ±${fmt(Math.max(1, state.F0), 0)} N`, L + pw + 4, T + 10);
}
function drawFRF() {
  const cv = $('#frf'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 50, Rr = 50, T = 12, B = 30, pw = W - L - Rr, ph = H - T - B, th = R.th, z = th.z;
  const rmax = 3, ymax = Math.min(20, Math.max(3, 1 / (2 * Math.max(z, 0.026)) * 1.15)), X = r => L + r / rmax * pw, Y = v => T + (1 - clamp(v / ymax, 0, 1)) * ph, Yp = p => T + (1 - p / 180) * ph;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.strokeStyle = col('--grid'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'center';
  for (let r = 0; r <= rmax; r += 0.5) { ctx.beginPath(); ctx.moveTo(X(r), T); ctx.lineTo(X(r), T + ph); ctx.stroke(); ctx.fillText(r.toFixed(1), X(r), H - 12); }
  ctx.textAlign = 'right'; for (let v = 0; v <= ymax; v += ymax > 8 ? 4 : 1) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.fillText(v, L - 4, Y(v) + 3); }
  ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); for (const p of [0, 90, 180]) ctx.fillText(p + '°', L + pw + 4, Yp(p) + 3);
  ctx.strokeStyle = col('--muted'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(Math.SQRT2), T); ctx.lineTo(X(Math.SQRT2), T + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--muted'); ctx.fillText('√2', X(Math.SQRT2) + 3, T + 12); ctx.strokeStyle = col('--steel'); ctx.beginPath(); ctx.moveTo(L, Y(1)); ctx.lineTo(L + pw, Y(1)); ctx.stroke();
  const curve = (fn, color, w, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash || []); let f = true; for (let r = 0; r <= rmax; r += 0.01) { const v = fn(r); f ? ctx.moveTo(X(r), v) : ctx.lineTo(X(r), v); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  const D = r => 1 / Math.sqrt((1 - r * r) ** 2 + (2 * z * r) ** 2);
  curve(r => Y(D(r)), col('--water'), 2.2); curve(r => Y(Math.sqrt(1 + (2 * z * r) ** 2) * D(r)), col('--good'), 2); curve(r => Yp(Math.atan2(2 * z * r, 1 - r * r) * 180 / Math.PI), col('--brass'), 1.4, [5, 4]);
  for (const p of state.meas) { ctx.fillStyle = col('--alarm'); ctx.beginPath(); if (p.base) { ctx.rect(X(p.r) - 3, Y(p.v) - 3, 6, 6); } else ctx.arc(X(p.r), Y(p.v), 3.5, 0, 7); ctx.fill(); }
  if (state.mode === 'harmonic' || state.mode === 'base') { const v = state.mode === 'harmonic' ? th.D : th.T; ctx.beginPath(); ctx.fillStyle = col('--alarm'); ctx.arc(X(th.r), Y(v), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); ctx.fillStyle = col('--alarm'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = th.r > 2 ? 'right' : 'left'; ctx.fillText(`r ${fmt(th.r, 2)} · theory ${fmt(v, 2)} · measured ${fmt(R.ratioM, 2)} · φ ${fmt(th.phi, 0)}°`, X(th.r) + (th.r > 2 ? -10 : 10), Math.max(T + 14, Y(v) - 10)); }
  ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.textAlign = 'left'; ctx.fillText(`ζ = ${fmt(z, 3)} · peak 1/2ζ = ${z > 0 ? fmt(1 / (2 * z), 1) : '∞'} · half-power bandwidth ≈ ${fmt(2 * z * th.fn, 2)} Hz`, L + 6, T + 14); ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('r = f / fn', L + pw / 2, H - 1);
}
function drawPhase() {
  const cv = $('#phase'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, T = 10, pw = W - L - 20, ph = H - T - 24;
  const xm = Math.max(0.02, ...state.portrait.map(p => Math.abs(p[0]))) * 1.1, vm = Math.max(0.05, ...state.portrait.map(p => Math.abs(p[1]))) * 1.1, X = x => L + pw / 2 + x / xm * pw / 2, Y = v => T + ph / 2 - v / vm * ph / 2;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.strokeStyle = col('--steel'); ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(L + pw, Y(0)); ctx.moveTo(X(0), T); ctx.lineTo(X(0), T + ph); ctx.stroke();
  ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 1.5; state.portrait.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke();
  ctx.beginPath(); ctx.fillStyle = col('--alarm'); ctx.arc(X(state.x), Y(state.v), 5, 0, 7); ctx.fill();
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`x ±${fmt(xm * 1000, 0)} mm`, L + pw / 2, H - 6); ctx.save(); ctx.translate(14, T + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(`ẋ ±${fmt(vm, 2)} m/s`, 0, 0); ctx.restore();
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const th = R.th;
  $('#roFn').textContent = fmt(th.fn, 2); $('#roWn').textContent = fmt(th.wn, 1); $('#roT').textContent = fmt(1 / th.fn, 3); $('#roZ').textContent = fmt(th.z, 3); $('#roCc').textContent = fmt(th.cc, 1); $('#roClass').textContent = th.cls;
  $('#roFd').textContent = th.z < 1 ? fmt(th.fd, 2) : '—'; $('#roTau').textContent = fmt(th.tau, 2); $('#roTs').textContent = fmt(th.ts, 1); $('#roDelta').textContent = th.z < 1 ? fmt(th.delta, 3) : '—'; $('#roDeltaM').textContent = fmt(R.deltaM, 3); $('#roZm').textContent = fmt(R.zM, 3);
  $('#roX').textContent = fmt(state.x * 1000, 1); $('#roAmp').textContent = fmt(R.amp * 1000, 1); $('#roFm').textContent = fmt(R.fMeas, 2); $('#roR').textContent = state.mode === 'harmonic' || state.mode === 'base' ? fmt(th.r, 2) : '—'; $('#roPhi').textContent = state.mode === 'harmonic' || state.mode === 'base' ? fmt(th.phi, 0) : '—';
  $('#roMagName').textContent = state.mode === 'base' ? 'Transmissibility X/Y' : 'Magnification X·k/F₀'; $('#roMag').textContent = state.mode === 'harmonic' ? fmt(th.D, 2) : state.mode === 'base' ? fmt(th.T, 2) : '—'; $('#roMagM').textContent = fmt(R.ratioM, 2); $('#roMagBox').classList.toggle('warnv', (state.mode === 'harmonic' || state.mode === 'base') && th.r > 0.9 && th.r < 1.1);
  $('#roV').textContent = fmt(R.vmax, 2); $('#roA').textContent = fmt(R.amax, 1); $('#roAg').textContent = fmt(R.amax / GACC, 2);
  const run = $('#pillRun'); run.textContent = state.run ? `Running · t ${fmt(state.t, 1)} s` : 'Stopped'; run.classList.toggle('on', state.run);
  const pd = $('#pillDamp'); pd.textContent = `ζ ${fmt(th.z, 2)} · ${th.cls}`; pd.className = 'pill ' + (th.z < 1 ? 'under' : th.cls === 'critically damped' ? 'crit' : 'over');
  $('#pillFn').innerHTML = `f<sub>n</sub> ${fmt(th.fn, 2)} Hz`; $('#pillR').textContent = state.mode === 'harmonic' || state.mode === 'base' ? `r ${fmt(th.r, 2)} · ${state.mode === 'base' ? 'T' : 'M'} ${fmt(state.mode === 'base' ? th.T : th.D, 2)}` : state.mode;
  $('#pillStop').classList.toggle('hidden', !(state.t - state.hitT < 1)); $('#pillRes').classList.toggle('hidden', !((state.mode === 'harmonic' || state.mode === 'base') && state.run && th.r > 0.9 && th.r < 1.1));
  $('#btnStart').textContent = state.mode === 'free' ? 'Release' : state.mode === 'impulse' ? 'Tap' : 'Start';
}

/* ------------------------------------------------------------------ log and tasks */
function record(tag = '') {
  const th = R.th, r = { i: state.log.length + 1, tag, mode: state.mode, m: state.m, k: state.k, c: state.c, z: +th.z.toFixed(3), fn: +th.fn.toFixed(3), f: state.mode === 'harmonic' || state.mode === 'base' ? +excF().toFixed(3) : '', r: state.mode === 'harmonic' || state.mode === 'base' ? +th.r.toFixed(3) : '', F: state.mode === 'base' ? state.Y0 * 1000 : state.mode === 'free' ? '' : state.F0, amp: +(R.amp * 1000).toFixed(1), fm: Number.isFinite(R.fMeas) ? +R.fMeas.toFixed(3) : '', dm: Number.isFinite(R.deltaM) ? +R.deltaM.toFixed(3) : '', zm: Number.isFinite(R.zM) ? +R.zM.toFixed(3) : '', ratio: state.mode === 'harmonic' ? +th.D.toFixed(3) : state.mode === 'base' ? +th.T.toFixed(3) : '', ratioM: Number.isFinite(R.ratioM) ? +R.ratioM.toFixed(3) : '', phi: state.mode === 'harmonic' || state.mode === 'base' ? +th.phi.toFixed(0) : '' };
  state.log.push(r); renderLog(); checkTasks();
}
function renderLog() {
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.mode, r.m, r.k, r.c, r.z, r.fn, r.f, r.r, r.F, r.amp, r.fm, r.dm, r.zm, r.ratio, r.ratioM, r.phi].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,mode,m_kg,k_Npm,c_Nspm,zeta,fn_Hz,f_Hz,r,F0_N_or_Y_mm,amp_mm,meas_f_Hz,meas_delta,meas_zeta,ratio_theory,ratio_meas,phase_deg\n' + state.log.map(r => [r.i, r.tag, r.mode, r.m, r.k, r.c, r.z, r.fn, r.f, r.r, r.F, r.amp, r.fm, r.dm, r.zm, r.ratio, r.ratioM, r.phi].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
const TASKS = [
  { id: 'fn', t: 'Measure the natural frequency', d: 'Release the mass with light damping, let it swing a few cycles and record: the measured frequency should match f_n = √(k/m)/2π.', ok: () => state.log.some(r => r.mode === 'free' && r.fm && Math.abs(r.fm - r.fn) < 0.05 * r.fn) },
  { id: 'mk', t: 'Four times the mass, half the frequency', d: 'Record two free decays with the same spring and masses in the ratio 4:1.', ok: () => { const L = state.log.filter(r => r.mode === 'free'); return L.some(a => L.some(b => a.k === b.k && Math.abs(b.m - 4 * a.m) < 1e-6)); } },
  { id: 'logdec', t: 'Log decrement gives ζ', d: 'Record two free decays with different damper settings; compare measured ζ from the decrement with c/2√(km).', ok: () => { const L = state.log.filter(r => r.mode === 'free' && r.zm !== '' && Math.abs(r.zm - r.z) < 0.02 + 0.15 * r.z); return new Set(L.map(r => r.c)).size >= 2; } },
  { id: 'crit', t: 'Critical damping', d: 'Set ζ = 1 and release from a displacement: no overshoot and the fastest return.', ok: () => state.flags.crit },
  { id: 'over', t: 'Overdamped creep', d: 'Set ζ = 2 and release: record it and compare the settling with critical.', ok: () => state.log.some(r => r.mode === 'free' && r.z >= 1.8) },
  { id: 'res', t: 'Find resonance', d: 'Harmonic force with ζ ≤ 0.1: tune the frequency to r ≈ 1 and watch the amplitude climb toward 1/2ζ.', ok: () => state.flags.res },
  { id: 'sweep', t: 'Sweep the response', d: 'Run a swept sine and collect at least 12 measured points on the frequency-response chart.', ok: () => state.meas.length >= 12 },
  { id: 'iso', t: 'Isolate the base', d: 'Base excitation with r > √2: measured transmissibility below 1. Then add damping and see it get worse.', ok: () => state.flags.iso },
  { id: 'design', t: 'Design an isolator', d: 'A machine forcing at 10 Hz: choose m, k and c so that transmissibility is below 0.2 at 10 Hz and the amplitude stays under 20 mm.', ok: () => state.flags.design },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() {
  const th = R.th, f = state.flags;
  if (state.run && state.mode === 'free' && Math.abs(th.z - 1) < 0.05 && state.t > 3 * th.tau && state.peaks.length === 0 && Math.abs(state.x0) > 0.02) f.crit = true;
  if (state.run && state.mode === 'harmonic' && th.z <= 0.1 && th.r > 0.95 && th.r < 1.05 && R.ratioM > 4) f.res = true;
  if (state.run && state.mode === 'base' && th.r > Math.SQRT2 && Number.isFinite(R.ratioM) && R.ratioM < 1 && state.t > 5) f.iso = true;
  if (state.run && state.mode === 'harmonic' && Math.abs(excF() - 10) < 0.05 && th.T < 0.2 && R.amp < 0.02 && state.t > 4) f.design = true;
  for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; }
}

/* ------------------------------------------------------------------ control */
function reset() { state.t = 0; state.trace = []; state.peaks = []; state.zeros = []; state.portrait = []; state.hits = 0; state.hitT = -9; state.sweepT = 0; state.sweepR = 0.2; state.sweepDwell = 0; state.phOff = 0; state.y = 0; state.F = 0; }
function start() { reset(); if (state.mode === 'free') { state.x = state.x0; state.v = state.v0; } else if (state.mode === 'impulse') { state.x = 0; state.v = state.imp / state.m; } else { state.x = 0; state.v = 0; } state.run = true; }
function setMode(m) { state.mode = m; document.querySelectorAll('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === m)); const show = (id, on) => $('#' + id).style.display = on ? '' : 'none'; show('ctlX0', m === 'free'); show('ctlV0', m === 'free'); show('ctlF0', m === 'harmonic' || m === 'step'); show('ctlY0', m === 'base'); show('ctlF', m === 'harmonic' || m === 'base'); show('ctlSweep', m === 'harmonic' || m === 'base'); show('ctlImp', m === 'impulse'); state.run = false; reset(); state.x = m === 'free' ? state.x0 : 0; state.v = 0; }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') e.checked = !!v; else e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setZeta(z) { setCtl('c', +(z * 2 * Math.sqrt(state.k * state.m)).toFixed(1)); }
const PRESETS = {
  decay: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.05); setMode('free'); setCtl('x0', 80); setCtl('v0', 0); start(); },
  critical: () => { setCtl('m', 2); setCtl('k', 800); setZeta(1); setMode('free'); setCtl('x0', 80); setCtl('v0', 0); start(); },
  over: () => { setCtl('m', 2); setCtl('k', 800); setZeta(2); setMode('free'); setCtl('x0', 80); setCtl('v0', 0); start(); },
  res: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.05); setMode('harmonic'); setCtl('F0', 5); setCtl('f', +(Math.sqrt(800 / 2) / (2 * Math.PI)).toFixed(2)); setCtl('sweep', false); start(); },
  sweep: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.1); setMode('harmonic'); setCtl('F0', 5); setCtl('sweep', true); state.meas = []; start(); },
  iso: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.05); setMode('base'); setCtl('Y0', 10); setCtl('f', +(3 * Math.sqrt(800 / 2) / (2 * Math.PI)).toFixed(2)); setCtl('sweep', false); start(); },
  car: () => { setCtl('m', 20); setCtl('k', 800); setZeta(0.3); setMode('base'); setCtl('Y0', 30); setCtl('f', 1.5); setCtl('sweep', false); start(); },
  step: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.1); setMode('step'); setCtl('F0', 40); start(); },
  hammer: () => { setCtl('m', 2); setCtl('k', 800); setZeta(0.05); setMode('impulse'); setCtl('imp', 2); start(); },
};
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; if (!R.th) R.th = theory(); step(dt); if (!state.run) measure(); drawRig(); frame++; if (frame % 2 === 0) { drawTrace(); drawPhase(); updateReadouts(); if (frame % 6 === 0) { drawFRF(); checkTasks(); } } requestAnimationFrame(tick); }
function bind() {
  const bindR = (id, f, scale = 1) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value * scale; if (o) o.textContent = f(+e.value); }; e.addEventListener('input', set); set(); };
  bindR('m', v => v.toFixed(1) + ' kg'); bindR('k', v => v + ' N/m'); bindR('c', v => v.toFixed(1) + ' N·s/m'); bindR('tscale', v => v === 1 ? 'real time' : '×' + v.toFixed(1)); bindR('x0', v => v + ' mm', 0.001); bindR('v0', v => v.toFixed(1) + ' m/s'); bindR('F0', v => v + ' N'); bindR('Y0', v => v + ' mm', 0.001); bindR('f', v => v.toFixed(2) + ' Hz'); bindR('imp', v => v.toFixed(1) + ' N·s');
  ['m', 'k', 'c'].forEach(id => $('#' + id).addEventListener('input', () => { R.th = theory(); }));
  $('#x0').addEventListener('input', () => { if (!state.run && state.mode === 'free') state.x = state.x0; });
  $('#sweep').addEventListener('change', e => { state.sweep = e.target.checked; state.sweepT = 0; state.sweepR = 0.2; state.sweepDwell = 0; if (state.sweep) state.meas = []; });
  document.querySelectorAll('.zeta [data-z]').forEach(b => b.onclick = () => setZeta(+b.dataset.z)); document.querySelectorAll('#modes button').forEach(b => b.onclick = () => setMode(b.dataset.mode));
  $('#btnStart').onclick = start; $('#btnStop').onclick = () => state.run = false; $('#btnRecord').onclick = () => record(''); $('#btnClearM').onclick = () => state.meas = [];
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('msd_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const k = e.key.toUpperCase(); if (e.key === ' ') { state.run ? state.run = false : start(); } else if (k === 'R') record(''); else if (/^[1-5]$/.test(k)) setMode(['free', 'harmonic', 'base', 'step', 'impulse'][+k - 1]); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawTrace(); drawFRF(); drawPhase(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.MassSpringDamper = {
  getState: () => ({ inputs: { m: state.m, k: state.k, c: state.c, mode: state.mode, x0: state.x0, v0: state.v0, F0: state.F0, Y0: state.Y0, f: state.f, sweep: state.sweep, imp: state.imp, tscale: state.tscale, run: state.run }, results: { fn: R.th.fn, zeta: R.th.z, fd: R.th.fd, r: R.th.r, x: state.x, amp: R.amp, fMeas: R.fMeas, deltaM: R.deltaM, ratioTheory: state.mode === 'base' ? R.th.T : R.th.D, ratioMeas: R.ratioM, phase: R.th.phi, hits: state.hits, measPoints: state.meas.length }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['m', 'k', 'c', 'tscale', 'F0', 'f', 'imp']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.x0 !== undefined) setCtl('x0', inp.x0 * 1000); if (inp.Y0 !== undefined) setCtl('Y0', inp.Y0 * 1000); if (inp.zeta !== undefined) setZeta(+inp.zeta); if (inp.mode) setMode(inp.mode); if (inp.sweep !== undefined) setCtl('sweep', inp.sweep); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run === true) start(); if (inp.run === false) state.run = false; },
  meta: { id: 'mass-spring-damper', title: 'Mass-Spring-Damper Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.MassSpringDamper.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'mass-spring-damper', state: window.MassSpringDamper.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('msd_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setMode('free'); R.th = theory(); measure(); drawRig(); drawTrace(); drawFRF(); drawPhase(); updateReadouts();
  requestAnimationFrame(tick);
}
build();
