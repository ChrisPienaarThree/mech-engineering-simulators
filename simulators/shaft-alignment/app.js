/* Shaft Alignment Bench — a pump (stationary) and motor (movable) joined by a coupling. As-found offset and angularity in
   both planes from hidden foot errors, rim-and-face and reverse-dial indicator readings with bracket sag, laser display,
   shim and jacking corrections at the feet, thermal growth targets, a soft foot, speed-dependent tolerances, and the
   running consequences: coupling reactions, bearing loads, 1× and 2× vibration and bearing heating. Standalone, offline. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 2) => Number.isFinite(x) ? x.toFixed(d) : '—', sgn = x => (x >= 0 ? '+' : '') + x.toFixed(2);
const TOL = [[1000, 0.09, 0.09], [1500, 0.06, 0.07], [3000, 0.03, 0.04], [6000, 0.02, 0.03]];

/* ------------------------------------------------------------------ state */
const state = { plane: 'v', A: 200, B: 600, D: 150, S: 140, rpm: 1500, sF: 0, sR: 0, mF: 0, mR: 0, sag: 0.04, sagComp: 'off', growth: 0, hotAlign: 'off', softFix: 0, run: false, tscale: 5,
  found: { yA: 0.35, yB: 0.62, zA: -0.2, zB: -0.55 }, softFoot: { foot: 'RR', gap: 0.15, on: false }, Tbrg: 30, t: 0, ang: 0, trace: [], traceAcc: 0, log: [], flags: {}, checked: false };
const R = {};

/* ------------------------------------------------------------------ model */
function tol(rpm) { let lo = TOL[0], hi = TOL[TOL.length - 1]; for (let i = 0; i < TOL.length - 1; i++) if (rpm >= TOL[i][0] && rpm <= TOL[i + 1][0]) { lo = TOL[i]; hi = TOL[i + 1]; } const f = clamp((rpm - lo[0]) / Math.max(1, hi[0] - lo[0]), 0, 1); return { off: lo[1] + f * (hi[1] - lo[1]), ang: lo[2] + f * (hi[2] - lo[2]) }; }
function solve() {
  const s = state, sf = s.softFoot, softV = sf.on ? Math.max(0, sf.gap - s.softFix) : 0; // a soft rear foot pulled down distorts the frame: the rear end drops by part of the gap
  const yA = s.found.yA + s.sF, yB = s.found.yB + s.sR - 0.6 * softV, zA = s.found.zA + s.mF, zB = s.found.zB + s.mR + 0.3 * softV;
  const span = s.B - s.A, Av = (yB - yA) / span * 100, Ov = yA - (yB - yA) * s.A / span, Ah = (zB - zA) / span * 100, Oh = zA - (zB - zA) * s.A / span;
  const OvHot = Ov + s.growth, target = s.hotAlign === 'on' ? -s.growth : 0, tl = tol(s.rpm);
  const rimV = 2 * Ov - 2 * (s.sagComp === 'on' ? 0 : s.sag), faceV = Av * s.D / 100, rimH = 2 * Oh, faceH = Ah * s.D / 100; // indicator TIR readings (bottom minus top, right minus left)
  const OvMeas = rimV / 2, AvMeas = faceV / s.D * 100, OhMeas = rimH / 2, AhMeas = faceH / s.D * 100; // what a technician would infer from the dials
  const revP = 2 * (Ov - Av * s.S / 2 / 100), revM = 2 * (Ov + Av * s.S / 2 / 100); // reverse dial: rim readings at the two hub planes ±S/2 from the coupling centre
  const shimF = -(yA - target) , shimR = -(yB - target), moveF = -zA, moveR = -zB; // corrections still needed (add shims +, remove −)
  const okV = Math.abs(OvHot - 0) <= tl.off && Math.abs(Av) <= tl.ang, okH = Math.abs(Oh) <= tl.off && Math.abs(Ah) <= tl.ang, okVcold = Math.abs(Ov - target) <= tl.off && Math.abs(Av) <= tl.ang;
  const offTot = Math.hypot(OvHot, Oh), angTot = Math.hypot(Av, Ah), F = 2.0 * offTot * 1000 / 1000, M = 0.12 * angTot, v1 = 1.2 + 4 * offTot + 2 * angTot, v2 = 0.4 + 18 * offTot + 12 * angTot, brgLoad = 1 + F / 2, life = 1 / Math.pow(Math.max(1, brgLoad), 3), Tss = 35 + 60 * (v2 / 4.5) ** 1.5;
  return { yA, yB, zA, zB, Av, Ov, Ah, Oh, OvHot, target, tl, rimV, faceV, rimH, faceH, OvMeas, AvMeas, OhMeas, AhMeas, revP, revM, shimF, shimR, moveF, moveR, okV, okH, okVcold, offTot, angTot, F, M, v1, v2, life, Tss, softV, grade: v2 > 7.1 ? 'D · unacceptable' : v2 > 4.5 ? 'C · unsatisfactory' : v2 > 2.8 ? 'B · acceptable' : 'A · good' };
}
function step(dt) {
  const s = state; Object.assign(R, solve()); if (s.run) { s.t += dt * s.tscale; s.ang += s.rpm / 60 * 360 * dt * 0.05; s.Tbrg += (R.Tss - s.Tbrg) * Math.min(1, dt * s.tscale / 300); if (R.v2 > 4.5) s.flags.vib = true; } else s.Tbrg += (30 - s.Tbrg) * Math.min(1, dt * s.tscale / 600);
  if (R.okV && R.okVcold) s.flags.vert = true; if (R.okH) s.flags.horiz = true; if (R.okV && R.okH && R.okVcold) { s.flags.aligned = true; if (s.rpm >= 3000) s.flags.fast = true; if (s.growth >= 0.05 && s.hotAlign === 'on') s.flags.thermal = true; }
  if (s.softFoot.on && R.softV < 0.03 && s.checked) s.flags.softfixed = true;
  s.traceAcc += dt; if (s.traceAcc >= 0.2) { s.traceAcc = 0; s.trace.push({ t: s.t, v1: s.run ? R.v1 : 0, v2: s.run ? R.v2 : 0, T: s.Tbrg }); if (s.trace.length > 900) s.trace.shift(); }
}
function newJob() { const s = state, rnd = (a, b) => a + Math.random() * (b - a); s.found = { yA: rnd(-0.5, 0.5), yB: rnd(-0.8, 0.8), zA: rnd(-0.4, 0.4), zB: rnd(-0.7, 0.7) }; s.softFoot = { foot: ['FL', 'FR', 'RL', 'RR'][Math.floor(Math.random() * 4)], gap: rnd(0.08, 0.25), on: Math.random() < 0.5 }; s.checked = false; for (const k of ['sF', 'sR', 'mF', 'mR', 'softFix']) setCtl(k, 0); s.flags = {}; }

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Pump and motor alignment' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, V = s.plane === 'v', cy = 170, xc = 330, k = 0.42, EX = 40; // EX = exaggeration of misalignment in pixels per mm
  const yOf = x => (V ? (r.yA + (r.yB - r.yA) * (x - s.A) / (s.B - s.A)) : (r.zA + (r.zB - r.zA) * (x - s.A) / (s.B - s.A))) * EX; // motor line height at distance x behind the coupling (mm → px), + is up on screen for vertical
  el('rect', { x: 40, y: cy + 30, width: 880, height: 14, class: 'base' }, svg);
  el('rect', { x: 60, y: cy - 60, width: 220, height: 120, rx: 10, class: 'machine' }, svg); txt(170, cy - 30, 'PUMP · fixed', 'lbl', svg, 'middle'); el('line', { x1: 280, y1: cy, x2: xc - 10, y2: cy, class: 'shaft' }, svg); el('rect', { x: xc - 34, y: cy - 30, width: 24, height: 60, rx: 3, class: 'hub' }, svg);
  const xA = xc + s.A * k, xB = xc + s.B * k, yA = cy - yOf(s.A), yB = cy - yOf(s.B), ang = Math.atan2(yB - yA, xB - xA), mx0 = xc + 40 * k, y0 = cy - yOf(40), Lm = (s.B + 120 - 40) * k;
  const g = el('g', { transform: `translate(${mx0},${y0}) rotate(${ang * 180 / Math.PI})` }, svg); el('rect', { x: 0, y: -60, width: Lm, height: 120, rx: 10, class: 'machine motor' }, g); txt(Lm / 2, -30, 'MOTOR · movable', 'lbl', g, 'middle'); el('line', { x1: -30 * k - 20, y1: 0, x2: 0, y2: 0, class: 'shaft' }, g); el('rect', { x: -30 * k - 34, y: -30, width: 24, height: 60, rx: 3, class: 'hub' }, g);
  for (const [xf, name, shim] of [[s.A, 'front', V ? s.sF : s.mF], [s.B, 'rear', V ? s.sR : s.mR]]) { const gx = (xf - 40) * k; el('rect', { x: gx - 14, y: 60, width: 28, height: 12, class: 'foot' }, g); if (V) { const h = clamp(shim * 40, -20, 24); el('rect', { x: gx - 18, y: 72, width: 36, height: Math.abs(h), class: 'shim' + (h < 0 ? ' neg' : '') }, g); } txt(gx, 96, `${name} foot ${V ? 'shims ' + sgn(shim) + ' mm' : 'move ' + sgn(shim) + ' mm'}`, 'lbl', g, 'middle'); }
  if (s.softFoot.on && s.checked) { const gx = ((s.softFoot.foot[0] === 'F' ? s.A : s.B) - 40) * k; txt(gx, 116, `SOFT FOOT ${s.softFoot.foot} · gap ${fmt(Math.max(0, s.softFoot.gap - s.softFix), 2)} mm`, 'val bad', g, 'middle'); }
  el('line', { x1: 60, y1: cy, x2: 920, y2: cy, class: 'refline' }, svg); txt(900, cy - 6, 'pump shaft line', 'lbl', svg, 'end');
  const dv = V ? r.Ov : r.Oh, da = V ? r.Av : r.Ah; txt(xc - 22, cy - 76, `${V ? 'vertical' : 'horizontal'} offset ${sgn(dv)} mm · angularity ${sgn(da)} mm/100`, 'ftxt', svg, 'middle');
  const dx = 700, dy = 248; for (const [ox, name, top, bot, lft, rgt] of [[dx, 'RIM', 0, V ? r.rimV : r.rimH, 0, V ? r.rimH : r.rimV], [dx + 150, 'FACE', 0, V ? r.faceV : r.faceH, 0, V ? r.faceH : r.faceV]]) { el('circle', { cx: ox, cy: dy, r: 44, class: 'dial' }, svg); txt(ox, dy + 4, name, 'lbl', svg, 'middle'); txt(ox, dy - 30, '0', 'val', svg, 'middle'); txt(ox, dy + 40, sgn(bot), 'val', svg, 'middle'); txt(ox - 36, dy + 4, sgn(lft), 'val', svg, 'end'); txt(ox + 36, dy + 4, sgn(rgt), 'val', svg); }
  txt(dx + 75, dy + 62, `indicator on the pump shaft reading the motor hub · Ø${s.D} face · bracket sag ${s.sag} mm ${s.sagComp === 'on' ? '(compensated)' : '(NOT compensated)'}`, 'lbl', svg, 'middle');
  txt(480, 372, r.okV && r.okH && r.okVcold ? `ALIGNED within tolerance for ${s.rpm} rpm (offset ≤ ${fmt(r.tl.off)} mm, angle ≤ ${fmt(r.tl.ang)} mm/100)${s.growth > 0 ? ' · hot condition' : ''}` : `vertical ${r.okV && r.okVcold ? 'OK' : 'OUT'} · horizontal ${r.okH ? 'OK' : 'OUT'} · tolerance ${fmt(r.tl.off)} mm / ${fmt(r.tl.ang)} mm per 100 at ${s.rpm} rpm`, 'val' + (r.okV && r.okH && r.okVcold ? '' : ' bad'), svg, 'middle');
  txt(480, 392, s.run ? `running · 1× ${fmt(r.v1, 1)} · 2× ${fmt(r.v2, 1)} mm/s (${r.grade}) · coupling ${fmt(r.F, 1)} kN, ${fmt(r.M, 2)} kN·m · bearing ${fmt(s.Tbrg, 0)} °C · life ×${fmt(r.life, 2)}` : s.showSol ? `solution · ${V ? 'shims front ' + sgn(r.shimF) + ' rear ' + sgn(r.shimR) + ' mm' : 'move front ' + sgn(r.moveF) + ' rear ' + sgn(r.moveR) + ' mm'} (add +, remove −) · other plane: ${V ? 'front ' + sgn(r.moveF) + ' rear ' + sgn(r.moveR) : 'front ' + sgn(r.shimF) + ' rear ' + sgn(r.shimR)}` : `${V ? 'vertical' : 'horizontal'} plane · work out the ${V ? 'shims' : 'moves'} at the feet (${s.A} / ${s.B} mm from the coupling) from the readings, or press Show solution`, 'val' + (s.run && r.v2 > 4.5 ? ' bad' : ''), svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawGraph() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const x0 = -300, x1 = s.B + 150, X = x => L + (x - x0) / (x1 - x0) * pw, ymax = Math.max(0.6, Math.abs(r.yA), Math.abs(r.yB), Math.abs(r.zA), Math.abs(r.zB), Math.abs(r.Ov), Math.abs(r.Oh)) * 1.3, Y = y => T + (1 - (y + ymax) / (2 * ymax)) * ph;
  ctx.strokeStyle = col('--grid'); for (let y = -Math.floor(ymax * 2) / 2; y <= ymax; y += 0.25) { ctx.beginPath(); ctx.moveTo(L, Y(y)); ctx.lineTo(L + pw, Y(y)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(sgn(y), L - 4, Y(y) + 3); } for (const x of [-300, -100, 0, s.A, s.B]) { ctx.beginPath(); ctx.moveTo(X(x), T); ctx.lineTo(X(x), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(x === 0 ? 'coupling' : x === s.A ? 'front feet' : x === s.B ? 'rear feet' : x + ' mm', X(x), H - 18); }
  ctx.fillStyle = 'rgba(42,138,92,.12)'; ctx.fillRect(X(-40), Y(r.target + r.tl.off), X(40) - X(-40), Y(r.target - r.tl.off) - Y(r.target + r.tl.off));
  ctx.strokeStyle = col('--steel'); ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(X(x0), Y(0)); ctx.lineTo(X(0), Y(0)); ctx.stroke(); ctx.setLineDash([3, 3]); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(x1), Y(0)); ctx.stroke(); ctx.setLineDash([]);
  const line = (a, b, color, dash) => { ctx.strokeStyle = color; ctx.lineWidth = 2.4; ctx.setLineDash(dash); const f = x => a + (b - a) * (x - s.A) / (s.B - s.A); ctx.beginPath(); ctx.moveTo(X(-60), Y(f(-60))); ctx.lineTo(X(x1), Y(f(x1))); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = color; for (const [x, v] of [[s.A, a], [s.B, b]]) { ctx.beginPath(); ctx.arc(X(x), Y(v), 5, 0, 7); ctx.fill(); } };
  line(r.yA, r.yB, col('--alarm'), []); line(r.zA, r.zB, col('--water'), [6, 3]);
  if (s.growth > 0) { line(r.yA + s.growth, r.yB + s.growth, col('--brass'), [2, 3]); }
  ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); ctx.fillText(`vertical: offset ${sgn(r.Ov)} mm, angle ${sgn(r.Av)} mm/100 → shims front ${sgn(r.shimF)}, rear ${sgn(r.shimR)}`, L + 6, T + 14); ctx.fillStyle = col('--water'); ctx.fillText(`horizontal: offset ${sgn(r.Oh)}, angle ${sgn(r.Ah)} → move front ${sgn(r.moveF)}, rear ${sgn(r.moveR)}`, L + 6, T + 28); if (s.growth > 0) { ctx.fillStyle = col('--brass'); ctx.fillText(`hot: motor grows +${s.growth} mm → cold target ${sgn(r.target)} mm`, L + 6, T + 42); }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('distance from the coupling mm · shaft centreline height mm (motor relative to pump) · green band = offset tolerance at the coupling', L + pw / 2, H - 4);
}
function drawVib() {
  const cv = $('#chart2'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const vmax = Math.max(10, r.v2 * 1.2, r.v1 * 1.2), Y = v => T + (1 - clamp(v / vmax, 0, 1)) * ph; ctx.strokeStyle = col('--grid'); for (let v = 0; v <= vmax; v += 2) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(v, L - 4, Y(v) + 3); }
  for (const [v, name, color] of [[2.8, 'A/B', col('--good')], [4.5, 'B/C', col('--brass')], [7.1, 'C/D', col('--alarm')]]) { ctx.strokeStyle = color; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = color; ctx.textAlign = 'left'; ctx.fillText(`${v} mm/s zone ${name}`, L + pw - 130, Y(v) - 3); }
  const bars = [['1×', s.run ? r.v1 : 0, col('--water')], ['2×', s.run ? r.v2 : 0, col('--alarm')], ['3×', s.run ? 0.3 * r.v2 : 0, col('--steel')]]; bars.forEach(([n, v, c], i) => { const x = L + 80 + i * 140; ctx.fillStyle = c; ctx.fillRect(x, Y(v), 60, Y(0) - Y(v)); ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`${n} · ${fmt(v, 1)}`, x + 30, Y(v) - 6); ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillText(n + ' rpm', x + 30, H - 18); });
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(s.run ? `coupling reaction ${fmt(r.F, 1)} kN · moment ${fmt(r.M, 2)} kN·m · bearing life ×${fmt(r.life, 2)} · ${r.grade}` : 'start the set to see the running vibration', L + 6, T + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('vibration velocity mm/s rms at the motor bearing · offset feeds 1× and 2×, angularity feeds 2× · ISO zones', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, s = state, span = 180, tr = s.trace.filter(p => p.t > s.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const X = t => L + (t - (s.t - span)) / span * pw, Y = v => T + (1 - clamp(v / 12, 0, 1)) * ph, YT = v => T + (1 - clamp((v - 20) / 100, 0, 1)) * ph;
  const ser = (k, color, Yf, w) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Yf(p[k])) : ctx.lineTo(X(p.t), Yf(p[k])); f = false; } ctx.stroke(); }; ser('v1', col('--water'), Y, 1.6); ser('v2', col('--alarm'), Y, 2); ser('T', col('--brass'), YT, 1.6);
  ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.fillText('12 mm/s', L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('1×', L + pw + 4, T + 10); ctx.fillStyle = col('--alarm'); ctx.fillText('2×', L + pw + 4, T + 24); ctx.fillStyle = col('--brass'); ctx.fillText('brg °C', L + pw + 4, T + 38); ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s of machine time · vibration and bearing temperature`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, set = (id, v) => { $('#' + id).textContent = v; };
  set('roOv', sgn(r.Ov)); set('roOvHot', sgn(r.OvHot)); set('roAv', sgn(r.Av)); set('roOh', sgn(r.Oh)); set('roAh', sgn(r.Ah)); set('roTol', `${fmt(r.tl.off)} / ${fmt(r.tl.ang)}`); set('roRim', `${sgn(r.rimV)} / ${sgn(r.rimH)}`); set('roFace', `${sgn(r.faceV)} / ${sgn(r.faceH)}`); set('roRev', `${sgn(r.revP)} / ${sgn(r.revM)}`); set('roMeas', `${sgn(r.OvMeas)} · ${sgn(r.AvMeas)}`); set('roMeasNote', s.sag > 0 && s.sagComp === 'off' ? `from the dials — sag ${s.sag} mm not compensated` : 'from the dials, sag compensated'); set('roLaser', `${sgn(r.Ov)} · ${sgn(r.Av)}`); set('roV2', s.run ? fmt(r.v2, 1) : '—'); set('roV1', s.run ? fmt(r.v1, 1) : '—'); set('roTb', fmt(s.Tbrg, 0)); set('roSoft', s.checked ? (s.softFoot.on ? `${s.softFoot.foot} gap ${fmt(Math.max(0, s.softFoot.gap - s.softFix))} mm` : 'none found') : 'not checked');
  $('#roOvBox').classList.toggle('alarm', !(r.okV && r.okVcold)); $('#roOhBox').classList.toggle('alarm', !r.okH); $('#roV2Box').classList.toggle('alarm', s.run && r.v2 > 4.5); $('#roMeasBox').classList.toggle('warnv', s.sag > 0 && s.sagComp === 'off'); $('#roSoftBox').classList.toggle('alarm', s.checked && s.softFoot.on && R.softV > 0.03);
  const ok = r.okV && r.okH && r.okVcold; const ps = $('#pillState'); ps.textContent = ok ? 'Within tolerance' : 'Misaligned'; ps.className = 'pill ' + (ok ? 'ok' : 'warn');
  $('#pillMode').textContent = `${s.rpm} rpm · V ${sgn(r.OvHot)} / ${sgn(r.Av)} · H ${sgn(r.Oh)} / ${sgn(r.Ah)}`; $('#pillVib').classList.toggle('hidden', !(s.run && r.v2 > 4.5)); $('#pillSoft').classList.toggle('hidden', !(s.checked && s.softFoot.on && R.softV > 0.03)); $('#pillSag').classList.toggle('hidden', !(s.sag > 0 && s.sagComp === 'off'));
  $('#pillRun').textContent = s.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start the set'; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === s.plane));
  $('#rigTitle').textContent = s.plane === 'v' ? 'Side view · vertical plane (offset and angularity exaggerated ×40)' : 'Plan view · horizontal plane (exaggerated ×40)';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, rpm: s.rpm, Ov: +r.Ov.toFixed(3), OvHot: +r.OvHot.toFixed(3), Av: +r.Av.toFixed(3), Oh: +r.Oh.toFixed(3), Ah: +r.Ah.toFixed(3), rimV: +r.rimV.toFixed(2), faceV: +r.faceV.toFixed(2), rimH: +r.rimH.toFixed(2), faceH: +r.faceH.toFixed(2), sag: s.sag, sagComp: s.sagComp, growth: s.growth, sF: s.sF, sR: s.sR, mF: s.mF, mR: s.mR, ok: r.okV && r.okH && r.okVcold ? 'yes' : 'no', v1: s.run ? +r.v1.toFixed(1) : '', v2: s.run ? +r.v2.toFixed(1) : '', tolOff: +r.tl.off.toFixed(3), tolAng: +r.tl.ang.toFixed(3), OvMeas: +r.OvMeas.toFixed(3) }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.rpm, r.Ov, r.OvHot, r.Av, r.Oh, r.Ah, r.rimV, r.faceV, r.rimH, r.faceH, r.sag, r.sagComp, r.growth, r.sF, r.sR, r.mF, r.mR, r.ok, r.v1, r.v2].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,rpm,Ov_mm,Ov_hot_mm,Av_mm_per_100,Oh_mm,Ah_mm_per_100,rim_V,face_V,rim_H,face_H,sag_mm,sag_comp,growth_mm,shim_front,shim_rear,move_front,move_rear,in_tolerance,v1x,v2x,tol_off,tol_ang,Ov_measured\n' + state.log.map(r => [r.i, r.tag, r.rpm, r.Ov, r.OvHot, r.Av, r.Oh, r.Ah, r.rimV, r.faceV, r.rimH, r.faceH, r.sag, r.sagComp, r.growth, r.sF, r.sR, r.mF, r.mR, r.ok, r.v1, r.v2, r.tolOff, r.tolAng, r.OvMeas].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'found', t: 'Read the as-found condition', d: 'Record the as-found readings before touching anything: rim and face in both planes, and the offset and angularity they imply.', ok: () => state.log.some(x => x.sF === 0 && x.sR === 0 && x.mF === 0 && x.mR === 0) },
  { id: 'vert', t: 'Shim the vertical plane', d: 'Work out the shim change at the front and rear feet from the readings and the foot distances, apply it and bring the vertical plane within tolerance.', ok: () => state.flags.vert },
  { id: 'horiz', t: 'Move the horizontal plane', d: 'Jack the motor sideways at each foot to bring the horizontal offset and angularity within tolerance.', ok: () => state.flags.horiz },
  { id: 'aligned', t: 'Aligned in both planes', d: 'Both planes within the tolerance for the running speed. Record it.', ok: () => state.flags.aligned && state.log.some(x => x.ok === 'yes') },
  { id: 'sag', t: 'Bracket sag', d: 'With bracket sag not compensated, record the offset the dials imply; then switch compensation on and record again. The difference is exactly the sag.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.sag > 0 && a.sagComp === 'off' && b.sagComp === 'on' && Math.abs(a.OvMeas - b.OvMeas) > 0.01)); } },
  { id: 'thermal', t: 'Thermal growth', d: 'Set a motor thermal growth of 0.05 mm or more, align for the hot condition (cold target below zero) and get the hot alignment within tolerance.', ok: () => state.flags.thermal },
  { id: 'softfoot', t: 'Soft foot', d: 'Run the soft-foot check, find the foot with a gap, shim it and confirm the gap is closed before aligning.', ok: () => state.flags.softfixed },
  { id: 'vib', t: 'Run it misaligned', d: 'Start the set with the misalignment three times the tolerance or more and watch the 2× vibration climb into zone C and the bearing heat.', ok: () => state.flags.vib },
  { id: 'fast', t: 'Tighter at speed', d: 'Set 3 000 rpm or more and align to the tighter tolerance that applies.', ok: () => state.flags.fast },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawGraph(); drawVib(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
const PRESETS = {
  found: () => { for (const [k, v] of Object.entries({ A: 200, B: 600, D: 150, S: 140, rpm: 1500, sF: 0, sR: 0, mF: 0, mR: 0, sag: 0.04, growth: 0, softFix: 0 })) setCtl(k, v); $('#sagComp').value = 'off'; state.sagComp = 'off'; $('#hotAlign').value = 'off'; state.hotAlign = 'off'; state.found = { yA: 0.35, yB: 0.62, zA: -0.2, zB: -0.55 }; state.softFoot = { foot: 'RR', gap: 0.15, on: false }; state.checked = false; state.run = false; state.showSol = false; state.plane = 'v'; },
  random: () => { PRESETS.found(); newJob(); },
  thermal: () => { PRESETS.found(); setCtl('growth', 0.08); $('#hotAlign').value = 'on'; state.hotAlign = 'on'; },
  soft: () => { PRESETS.found(); state.softFoot = { foot: 'RR', gap: 0.18, on: true }; },
  fast: () => { PRESETS.found(); setCtl('rpm', 3600); },
  run: () => { PRESETS.found(); state.run = true; },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('A', v => v + ' mm'); bindR('B', v => v + ' mm'); bindR('D', v => v + ' mm'); bindR('S', v => v + ' mm'); bindR('rpm', v => v + ' rpm'); bindR('sF', v => sgn(v) + ' mm'); bindR('sR', v => sgn(v) + ' mm'); bindR('mF', v => sgn(v) + ' mm'); bindR('mR', v => sgn(v) + ' mm'); bindR('sag', v => v.toFixed(2) + ' mm'); bindR('growth', v => '+' + v.toFixed(2) + ' mm'); bindR('softFix', v => v.toFixed(2) + ' mm'); bindR('tscale', v => '×' + v);
  for (const id of ['sagComp', 'hotAlign']) $('#' + id).addEventListener('change', e => state[id] = e.target.value); document.querySelectorAll('#types button').forEach(b => b.onclick = () => state.plane = b.dataset.type);
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnCheck').onclick = () => { state.checked = true; }; $('#btnSol').onclick = () => { state.showSol = !state.showSol; }; $('#btnApply').onclick = () => { const r = R; setCtl('sF', +(state.sF + r.shimF).toFixed(2)); setCtl('sR', +(state.sR + r.shimR).toFixed(2)); setCtl('mF', +(state.mF + r.moveF).toFixed(2)); setCtl('mR', +(state.mR + r.moveR).toFixed(2)); }; $('#btnNew').onclick = newJob; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('sa_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'V') state.plane = 'v'; if (k === 'H') state.plane = 'h'; if (k === 'R') record(''); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.ShaftAlignment = {
  getState: () => ({ inputs: { plane: state.plane, A: state.A, B: state.B, D: state.D, S: state.S, rpm: state.rpm, sF: state.sF, sR: state.sR, mF: state.mF, mR: state.mR, sag: state.sag, sagComp: state.sagComp, growth: state.growth, hotAlign: state.hotAlign, softFix: state.softFix, run: state.run }, results: { Ov: R.Ov, OvHot: R.OvHot, Av: R.Av, Oh: R.Oh, Ah: R.Ah, rimV: R.rimV, faceV: R.faceV, rimH: R.rimH, faceH: R.faceH, revP: R.revP, revM: R.revM, tol: R.tl, ok: R.okV && R.okH && R.okVcold, shimF: R.shimF, shimR: R.shimR, moveF: R.moveF, moveR: R.moveR, v1: R.v1, v2: R.v2, Tbrg: state.Tbrg, softFoot: state.checked ? state.softFoot : null }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.plane) state.plane = inp.plane; for (const k of ['A', 'B', 'D', 'S', 'rpm', 'sF', 'sR', 'mF', 'mR', 'sag', 'growth', 'softFix', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); for (const k of ['sagComp', 'hotAlign']) if (inp[k]) { $('#' + k).value = inp[k]; state[k] = inp[k]; } if (inp.found) state.found = inp.found; if (inp.check) state.checked = true; if (inp.applySolution) $('#btnApply').click(); if (inp.newJob) newJob(); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'shaft-alignment', title: 'Shaft Alignment Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.ShaftAlignment.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'shaft-alignment', state: window.ShaftAlignment.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('sa_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); step(0); drawRig(); updateReadouts(); drawGraph(); drawVib(); drawTrace();
  requestAnimationFrame(tick);
}
build();
