/* Control Valve and PID Loop Bench — a control valve with inherent and installed characteristics, actuator lag, stiction and
   positioner, feeding a flow, level or temperature process with dead time, closed by an ISA PID with anti-windup, manual
   and auto modes, step tests, FOPDT identification and tuning suggestions. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const RANGE = 50, H = 0.02;

/* ------------------------------------------------------------------ state */
const state = { proc: 'flow', char: 'eqpct', Kv: 60, Kp: 0.6, taua: 1, stick: 0, positioner: 'off', mode: 'manual', SP: 50, OPm: 60, Kc: 1.0, Ti: 10, Td: 0, aw: 'on', action: 'reverse', theta: 1, taup: 3, noise: 0.2, Ps: 6, Pd: 2, load: 30, tscale: 1, run: true,
  t: 0, OP: 60, stem: 60, target: 60, I: 0, Dprev: 0, PVf: 0, PV: 0, PVraw: 0, level: 50, Tproc: 50, x1: 0, buf: [], trace: [], traceAcc: 0, stepT: null, step0: null, stepSP: null, peaks: [], lastE: 0, satT: 0, log: [], flags: {}, id: null, tune: null, stepLog: [], dash: 0, ePrevSign: 0, oscT: 0 };
const R = {};

/* ------------------------------------------------------------------ model */
const fchar = (c, x) => { x = clamp(x, 0, 1); return c === 'linear' ? x : c === 'eqpct' ? (x <= 0 ? 0 : Math.pow(RANGE, x - 1)) : Math.sqrt(x); };
function flowAt(x) { const s = state, f = fchar(s.char, x), dpa = Math.max(0.05, s.Ps - s.Pd), Kf = s.Kv * f, den = 1 + f * f * s.Kp; return { Q: Kf * Math.sqrt(dpa / den), f, dpv: dpa / den }; }
function valveInfo() { const s = state, full = flowAt(1), dpa = Math.max(0.05, s.Ps - s.Pd), auth = full.dpv / dpa, Qmax = s.Kv * Math.sqrt(4 / (1 + s.Kp)), now = flowAt(s.stem / 100), d = 0.01, gain = (flowAt(clamp(s.stem / 100 + d, 0, 1)).Q - flowAt(clamp(s.stem / 100 - d, 0, 1)).Q) / (2 * d) / full.Q; return { auth, Qmax, Qfull: full.Q, Q: now.Q, dpv: now.dpv, gain, f: now.f }; }
function sub(h) {
  const s = state, v = valveInfo(); s.t += h;
  /* controller */
  const e = (s.action === 'reverse' ? 1 : -1) * (s.SP - s.PV); let OP;
  if (s.mode === 'auto') { const P = s.Kc * e, Dn = s.Td > 0 ? (s.Kc * s.Td * (s.PVf - s.PV) / (s.Td / 10 + h)) : 0; s.PVf += (s.PV - s.PVf) * h / Math.max(s.Td / 10, h); let u = P + s.I + Dn; if (s.Ti > 0) { const dI = s.Kc / s.Ti * e * h; if (s.aw === 'on') { if (!((u >= 100 && dI > 0) || (u <= 0 && dI < 0))) s.I += dI; } else s.I += dI; u = P + s.I + Dn; } OP = clamp(u, 0, 100); if (s.aw === 'off' && s.Ti > 0) s.I = clamp(s.I, -400, 500); }
  else { OP = s.OPm; s.I = OP - s.Kc * e; }
  s.OP = OP; if (OP >= 99.9 || OP <= 0.1) s.satT += h; else s.satT = 0;
  /* actuator with stiction */
  const S = s.positioner === 'on' ? s.stick * 0.15 : s.stick; if (Math.abs(OP - s.target) > S / 2) s.target = S > 0 ? OP + Math.sign(OP - s.target) * S * 0.5 : OP; s.stem += (s.target - s.stem) * Math.min(1, h / s.taua);
  const Q = flowAt(s.stem / 100).Q; let pv;
  if (s.proc === 'flow') { s.x1 += (Q / v.Qmax * 100 - s.x1) * Math.min(1, h / Math.max(0.1, s.taup)); pv = s.x1; }
  else if (s.proc === 'level') { const A = 2.0; s.level += (Q - s.load) / A / 3600 * 100 * h * 5; s.level = clamp(s.level, 0, 100); pv = s.level; }
  else { const Tss = 20 + 100 * Q / v.Qmax * (s.load / 30); s.Tproc += (Tss - s.Tproc) * Math.min(1, h / Math.max(0.5, s.taup)); pv = s.Tproc; }
  s.buf.push(pv); const n = Math.round(s.theta / h); while (s.buf.length > n + 1) s.buf.shift(); const delayed = s.buf[0];
  s.PVraw = delayed; s.PV = clamp(delayed + (Math.random() - 0.5) * s.noise, -5, 105);
  /* oscillation and step metrics */
  const err = s.SP - s.PV, sg = Math.sign(err); if (sg !== 0 && sg !== s.ePrevSign) { if (s.ePrevSign !== 0) { s.peaks.push({ t: s.t, amp: s.lastE }); if (s.peaks.length > 12) s.peaks.shift(); } s.ePrevSign = sg; s.lastE = 0; } s.lastE = Math.max(s.lastE, Math.abs(err));
  if (s.stepT !== null) { const st = s.stepLog, dtp = s.t - s.stepT; if (dtp <= 300) st.push({ dt: dtp, PV: s.PVraw, OP: s.OP }); }
  s.dash += Q / 20 * h * 30; if (s.mode === 'auto' && s.Ti === 0 && Math.abs(err) >= 2 && s.t > 30 && s.satT === 0) s.flags.offset = true;
}
function metrics() {
  const s = state, st = s.stepLog; const m = { osc: false, Pu: NaN, ampl: NaN, ov: NaN, ts: NaN, iae: NaN, offset: Math.abs(s.SP - s.PVraw) };
  const P = s.peaks.slice(-6); if (P.length >= 4) { const amps = P.map(p => p.amp), mean = amps.reduce((a, b) => a + b, 0) / amps.length, half = (P[P.length - 1].t - P[0].t) / (P.length - 1); if (mean > (s.stick > 0 ? 0.4 : 1.5) && Math.min(...amps) > 0.25 * mean && s.t - P[P.length - 1].t < 2.5 * half + 2) { m.osc = true; m.Pu = 2 * half; m.ampl = mean; } }
  if (st.length > 10 && s.step0 !== null && !(s.proc === 'level' && s.mode === 'manual')) { const dS = s.stepSP - s.step0; if (Math.abs(dS) > 0.5) { let mx = -Infinity, iae = 0, tsett = NaN; for (let i = 1; i < st.length; i++) { const p = st[i], y = (p.PV - s.step0) / dS; mx = Math.max(mx, y); iae += Math.abs(p.PV - s.stepSP) * (p.dt - st[i - 1].dt); } m.ov = Math.max(0, (mx - 1) * 100); m.iae = iae; for (let i = st.length - 1; i >= 0; i--) { if (Math.abs((st[i].PV - s.stepSP) / dS) > 0.05) { tsett = i < st.length - 1 ? st[i + 1].dt : NaN; break; } if (i === 0) tsett = 0; } m.ts = Number.isFinite(tsett) ? tsett : NaN; } }
  return m;
}
function identify() {
  const s = state, st = s.stepLog; if (st.length < 20 || s.mode !== 'manual' || s.step0 === null) return null; const dOP = s.stepOP1 - s.stepOP0; if (Math.abs(dOP) < 1) return null;
  const y0 = s.step0, yEnd = st[st.length - 1].PV, dy = yEnd - y0; if (Math.abs(dy) < 0.5) return null;
  if (s.proc === 'level') { const n = st.length, a = st[Math.floor(n / 2)], b = st[n - 1], slope = (b.PV - a.PV) / Math.max(0.1, b.dt - a.dt) / dOP; let th = 0; for (const p of st) { if (Math.abs(p.PV - y0) > 0.3) { th = Math.max(0, p.dt - 0.3); break; } } return { K: slope, tau: Infinity, theta: th, integrating: true }; }
  const K = dy / dOP; let t28 = null, t63 = null; for (const p of st) { const f = (p.PV - y0) / dy; if (t28 === null && f >= 0.283) t28 = p.dt; if (t63 === null && f >= 0.632) { t63 = p.dt; break; } }
  if (t63 === null) return { K, tau: NaN, theta: NaN, partial: true }; const tau = Math.max(0.1, 1.5 * (t63 - t28)), theta = Math.max(0, t63 - tau); return { K, tau, theta, integrating: s.proc === 'level' };
}
function suggest(id, m) { const s = state; if (m && m.osc && Number.isFinite(m.Pu)) return { name: 'Ziegler–Nichols closed loop', Kc: +(0.6 * s.Kc).toFixed(2), Ti: +(m.Pu / 2).toFixed(1), Td: +(m.Pu / 8).toFixed(1), from: `Ku ${fmt(s.Kc, 2)}, Pu ${fmt(m.Pu, 1)} s` }; if (id && id.integrating && Math.abs(id.K) > 1e-4) { const lam = Math.max(4 * id.theta, 10); return { name: 'SIMC for an integrating process', Kc: +(1 / (Math.abs(id.K) * (lam + id.theta))).toFixed(2), Ti: +(4 * (lam + id.theta)).toFixed(0), Td: 0, from: `slope ${fmt(id.K, 3)} %/s per %, θ ${fmt(id.theta, 1)} s` }; }
  if (id && Number.isFinite(id.tau)) { const lam = Math.max(id.tau, 3 * id.theta, 1); return { name: 'Lambda / IMC', Kc: +(id.tau / (Math.abs(id.K) * (lam + id.theta))).toFixed(2), Ti: +id.tau.toFixed(1), Td: +(0.5 * id.theta).toFixed(1), from: `K ${fmt(id.K, 2)}, τ ${fmt(id.tau, 1)} s, θ ${fmt(id.theta, 1)} s, λ = τ` }; } return null; }
function step(dt) {
  const s = state; if (s.run) { const dts = dt * s.tscale, n = Math.max(1, Math.round(dts / H)); for (let i = 0; i < n; i++) sub(dts / n); }
  Object.assign(R, valveInfo(), metrics()); R.id = identify(); R.tune = suggest(R.id, R);
  if (R.osc && s.mode === 'auto') { s.flags.osc = true; if (s.stick >= 1) s.flags.stiction = true; } if (s.mode === 'auto' && s.Ti > 0 && Math.abs(s.SP - s.PVraw) < 0.5 && s.t > 20 && s.satT === 0) { s.oscT += dt; if (s.oscT > 8) s.flags.integral = true; } else s.oscT = 0;
  if (s.satT > 20 && s.aw === 'off') s.flags.satur = true; if (s.aw === 'off' && s.stepSat >= 20 && s.stepT !== null && s.t - s.stepT > 25 && Math.abs(s.SP - s.PVraw) > 2 && s.SP < s.step0) s.flags.windup = true;
  if (s.mode === 'auto' && Number.isFinite(R.ov) && R.ov < 25 && Number.isFinite(R.ts) && s.flags.tuned) s.flags.tunedOK = true;
  s.traceAcc += dt * s.tscale; if (s.traceAcc >= 0.1) { s.traceAcc = 0; s.trace.push({ t: s.t, PV: s.PV, SP: s.SP, OP: s.OP, stem: s.stem, Q: R.Q }); if (s.trace.length > 3000) s.trace.shift(); }
}
function startStep(kind, val) { const s = state; s.stepT = s.t; s.step0 = s.PVraw; s.stepLog = []; s.stepOP0 = s.OP; s.stepSat = s.satT; if (kind === 'sp') { s.SP = clamp(val, 0, 100); s.stepSP = s.SP; const e = $('#SP'); if (e) { e.value = s.SP; const o = $('#SPOut'); if (o) o.textContent = s.SP + ' %'; } } else if (kind === 'op') { s.OPm = clamp(val, 0, 100); s.stepOP1 = s.OPm; s.stepSP = s.SP; } else s.stepSP = s.SP; }

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Control loop' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, y = 250;
  const pipe = (d, cls = '') => { el('path', { d, class: 'pipe ' + cls }, svg); el('path', { d, class: 'flow', style: `stroke-dashoffset:${-s.dash}` }, svg); };
  el('circle', { cx: 90, cy: y, r: 34, class: 'box' }, svg); txt(90, y + 4, 'PUMP', 'lbl', svg, 'middle'); txt(90, y + 52, `${s.Ps.toFixed(1)} bar`, 'val', svg, 'middle');
  pipe(`M124,${y} L300,${y}`); pipe(`M380,${y} L560,${y}`);
  const vx = 340; el('polygon', { points: `${vx - 40},${y - 22} ${vx},${y} ${vx - 40},${y + 22}`, class: 'box' }, svg); el('polygon', { points: `${vx + 40},${y - 22} ${vx},${y} ${vx + 40},${y + 22}`, class: 'box' }, svg);
  el('rect', { x: vx - 6, y: y - 110, width: 12, height: 88, class: 'box' }, svg); el('rect', { x: vx - 50, y: y - 160, width: 100, height: 50, rx: 8, class: 'box' }, svg); txt(vx, y - 138, 'ACTUATOR', 'lbl', svg, 'middle'); txt(vx, y - 120, `stem ${fmt(s.stem, 1)} %`, 'val', svg, 'middle');
  const sy = y - 110 + 88 * (1 - s.stem / 100); el('rect', { x: vx - 16, y: sy - 4, width: 32, height: 8, class: 'stemmark' }, svg); txt(vx + 60, y - 60, `${{ linear: 'linear', eqpct: 'equal %', quick: 'quick opening' }[s.char]} · Kv ${s.Kv}`, 'lbl', svg); txt(vx + 60, y - 44, `Δp valve ${fmt(r.dpv, 2)} bar · authority ${fmt(r.auth, 2)}`, 'val', svg); txt(vx + 60, y - 28, `Q ${fmt(r.Q, 1)} m³/h · f(x) ${fmt(r.f, 3)}`, 'val', svg);
  if (s.stick > 0) txt(vx - 60, y - 60, `stiction ${s.stick} %${s.positioner === 'on' ? ' · positioner' : ''}`, 'lbl', svg, 'end');
  const px = 640; if (s.proc === 'flow') { el('rect', { x: px - 40, y: y - 30, width: 80, height: 60, rx: 8, class: 'box cold' }, svg); txt(px, y - 6, 'FLOW', 'lbl', svg, 'middle'); txt(px, y + 12, 'TRANSMITTER', 'lbl', svg, 'middle'); pipe(`M${px + 40},${y} L880,${y}`); txt(880, y + 20, `to process ${s.Pd.toFixed(1)} bar`, 'lbl', svg, 'end'); }
  else if (s.proc === 'level') { el('rect', { x: px - 60, y: y - 130, width: 120, height: 190, rx: 6, class: 'box' }, svg); const hL = 180 * s.level / 100; el('rect', { x: px - 58, y: y + 58 - hL, width: 116, height: hL, class: 'liquid' }, svg); txt(px, y - 140, 'TANK · LEVEL', 'lbl', svg, 'middle'); pipe(`M560,${y} L${px - 60},${y - 100}`); pipe(`M${px + 60},${y + 40} L880,${y + 40}`); txt(880, y + 60, `outflow ${s.load} m³/h`, 'lbl', svg, 'end'); txt(px, y + 78, `${fmt(s.level, 1)} %`, 'ftxt', svg, 'middle'); }
  else { el('rect', { x: px - 70, y: y - 40, width: 140, height: 80, rx: 8, class: 'box hot' }, svg); txt(px, y - 14, 'HEAT EXCHANGER', 'lbl', svg, 'middle'); txt(px, y + 6, `${fmt(s.Tproc, 1)} °C`, 'ftxt', svg, 'middle'); txt(px, y + 24, `product ${s.load} m³/h`, 'lbl', svg, 'middle'); pipe(`M${px + 70},${y} L880,${y}`); txt(880, y + 20, 'steam → product outlet', 'lbl', svg, 'end'); }
  el('rect', { x: 560, y: 40, width: 260, height: 110, rx: 8, class: 'box' }, svg); txt(690, 60, `CONTROLLER · ${s.mode.toUpperCase()} · ${s.action}`, 'lbl', svg, 'middle'); txt(580, 84, `SP ${fmt(s.SP, 1)}`, 'ftxt sp', svg); txt(690, 84, `PV ${fmt(s.PV, 1)}`, 'ftxt pv', svg, 'middle'); txt(800, 84, `OP ${fmt(s.OP, 1)}`, 'ftxt op', svg, 'end'); txt(690, 106, `Kc ${s.Kc} · Ti ${s.Ti > 0 ? s.Ti + ' s' : 'off'} · Td ${s.Td} s · anti-windup ${s.aw}`, 'val', svg, 'middle'); txt(690, 126, r.osc ? `OSCILLATING · period ${fmt(r.Pu, 1)} s · amplitude ±${fmt(r.ampl, 1)}` : s.satT > 5 ? `OUTPUT SATURATED ${fmt(s.satT, 0)} s${s.aw === 'off' ? ' — integral winding up' : ''}` : `error ${fmt(s.SP - s.PV, 1)} %`, 'val' + (r.osc || s.satT > 5 ? ' bad' : ''), svg, 'middle');
  el('path', { d: `M${px},${s.proc === 'level' ? y - 130 : y - 30} L${px},150`, class: 'signal' }, svg); el('path', { d: `M560,95 L${vx},95 L${vx},${y - 160}`, class: 'signal' }, svg); txt(450, 88, 'OP to actuator', 'lbl', svg, 'middle'); txt(px + 8, 180, 'PV', 'lbl', svg);
  const m = r; txt(480, 372, s.stepT !== null && Number.isFinite(m.ov) ? `last step · overshoot ${fmt(m.ov, 0)} % · settling (5 %) ${Number.isFinite(m.ts) ? fmt(m.ts, 1) + ' s' : 'not yet'} · IAE ${fmt(m.iae, 0)}` : s.mode === 'manual' ? 'manual · step the output and read the process response, then Identify' : 'auto · step the setpoint to read overshoot and settling', 'val', svg, 'middle');
  txt(480, 392, r.id && (Number.isFinite(r.id.tau) || r.id.integrating) ? (r.id.integrating ? `identified integrating: slope ${fmt(r.id.K, 3)} %/s per % · θ ${fmt(r.id.theta, 1)} s` : `identified: K ${fmt(r.id.K, 2)} %/% · τ ${fmt(r.id.tau, 1)} s · θ ${fmt(r.id.theta, 1)} s`) + (r.tune ? ` → ${r.tune.name}: Kc ${r.tune.Kc}, Ti ${r.tune.Ti}, Td ${r.tune.Td}` : '') : r.tune ? `${r.tune.name} from ${r.tune.from}: Kc ${r.tune.Kc}, Ti ${r.tune.Ti} s, Td ${r.tune.Td} s` : `installed valve gain ${fmt(r.gain, 2)} at this opening (1.0 = linear) · authority ${fmt(r.auth, 2)}`, 'val', svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawTrend() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, Hh = cv.height; ctx.clearRect(0, 0, W, Hh); const L = 44, Rr = 44, T = 14, B = 30, pw = W - L - Rr, ph = Hh - T - B, s = state, span = 120, tr = s.trace.filter(p => p.t > s.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.strokeStyle = col('--grid'); for (let v = 0; v <= 100; v += 25) { const y = T + (1 - v / 100) * ph; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(v, L - 4, y + 3); }
  if (tr.length < 2) return; const X = t => L + (t - (s.t - span)) / span * pw, Y = v => T + (1 - clamp(v / 100, -0.05, 1.05)) * ph;
  const ser = (k, color, w, dash = []) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Y(p[k])) : ctx.lineTo(X(p.t), Y(p[k])); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  ser('SP', col('--ink'), 1.4, [5, 4]); ser('OP', col('--brass'), 1.6); ser('stem', col('--steel'), 1, [2, 3]); ser('PV', col('--water'), 2.4);
  if (s.stepT !== null && s.stepT > s.t - span) { ctx.strokeStyle = col('--alarm'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(s.stepT), T); ctx.lineTo(X(s.stepT), T + ph); ctx.stroke(); ctx.setLineDash([]); }
  ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('PV', L + pw + 4, T + 10); ctx.fillStyle = col('--ink'); ctx.fillText('SP', L + pw + 4, T + 24); ctx.fillStyle = col('--brass'); ctx.fillText('OP', L + pw + 4, T + 38); ctx.fillStyle = col('--steel'); ctx.fillText('stem', L + pw + 4, T + 52);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s of process time · PV, SP, controller output and valve stem in %`, L + pw / 2, Hh - 4);
}
function drawChar() {
  const cv = $('#chart2'), ctx = cv.getContext('2d'), W = cv.width, Hh = cv.height; ctx.clearRect(0, 0, W, Hh); const L = 50, Rr = 20, T = 14, B = 30, pw = W - L - Rr, ph = Hh - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.strokeStyle = col('--grid'); const X = x => L + x * pw, Y = q => T + (1 - clamp(q, 0, 1)) * ph; for (let v = 0; v <= 1; v += 0.25) { ctx.beginPath(); ctx.moveTo(X(v), T); ctx.lineTo(X(v), T + ph); ctx.stroke(); ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(fmt(v * 100, 0) + ' %', X(v), Hh - 16); ctx.textAlign = 'right'; ctx.fillText(fmt(v * 100, 0), L - 4, Y(v) + 3); }
  const curve = (fn, color, w, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); for (let i = 0; i <= 50; i++) { const x = i / 50, q = fn(x); i ? ctx.lineTo(X(x), Y(q)) : ctx.moveTo(X(x), Y(q)); } ctx.stroke(); ctx.setLineDash([]); };
  curve(x => fchar('linear', x), col('--steel'), 1, [3, 3]); curve(x => fchar('eqpct', x), col('--steel'), 1, [3, 3]); curve(x => fchar('quick', x), col('--steel'), 1, [3, 3]); curve(x => fchar(s.char, x), col('--muted'), 1.6, []); curve(x => flowAt(x).Q / r.Qfull, col('--water'), 2.6, []);
  ctx.fillStyle = col('--water'); ctx.beginPath(); ctx.arc(X(s.stem / 100), Y(r.Q / r.Qfull), 6, 0, 7); ctx.fill();
  ctx.font = '600 10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--steel'); ctx.textAlign = 'left'; ctx.fillText('linear', X(0.5) + 4, Y(0.5) - 4); ctx.fillText('equal %', X(0.75) + 4, Y(fchar('eqpct', 0.75)) + 12); ctx.fillText('quick opening', X(0.2) + 4, Y(fchar('quick', 0.2)) - 6); ctx.fillStyle = col('--water'); ctx.fillText(`installed (${{ linear: 'linear', eqpct: 'equal %', quick: 'quick opening' }[s.char]}, authority ${fmt(r.auth, 2)})`, L + 6, T + 14); ctx.fillStyle = col('--muted'); ctx.fillText('grey: inherent', L + 6, T + 28);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('stem opening · flow as % of the full-open installed flow · low authority pushes every characteristic towards quick opening', L + pw / 2, Hh - 4);
}
function drawStem() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, Hh = cv.height; ctx.clearRect(0, 0, W, Hh); const L = 44, Rr = 44, T = 8, B = 20, pw = W - L - Rr, ph = Hh - T - B, s = state, span = 40, tr = s.trace.filter(p => p.t > s.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const lo = Math.max(0, Math.min(...tr.map(p => Math.min(p.OP, p.stem))) - 2), hi = Math.min(100, Math.max(...tr.map(p => Math.max(p.OP, p.stem))) + 2), X = t => L + (t - (s.t - span)) / span * pw, Y = v => T + (1 - (v - lo) / Math.max(1, hi - lo)) * ph;
  const ser = (k, color, w) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Y(p[k])) : ctx.lineTo(X(p.t), Y(p[k])); f = false; } ctx.stroke(); }; ser('OP', col('--brass'), 1.6); ser('stem', col('--ink'), 2);
  ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.fillText(fmt(hi, 0), L - 4, T + 10); ctx.fillText(fmt(lo, 0), L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); ctx.fillText('OP', L + pw + 4, T + 10); ctx.fillStyle = col('--ink'); ctx.fillText('stem', L + pw + 4, T + 24); ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s · controller output against valve stem: steps and lag show stiction and actuator lag`, L + pw / 2, Hh - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, set = (id, v) => { $('#' + id).textContent = v; };
  set('roPV', fmt(s.PV, 1)); set('roSP', fmt(s.SP, 1)); set('roE', fmt(s.SP - s.PV, 1)); set('roOP', fmt(s.OP, 1)); set('roStem', fmt(s.stem, 1)); set('roQ', fmt(r.Q, 1)); set('roQmax', fmt(r.Qfull, 1)); set('roDp', fmt(r.dpv, 2)); set('roAuth', fmt(r.auth, 2)); set('roGain', fmt(r.gain, 2));
  set('roOv', Number.isFinite(r.ov) ? fmt(r.ov, 0) : '—'); set('roTs', Number.isFinite(r.ts) ? fmt(r.ts, 1) : '—'); set('roIae', Number.isFinite(r.iae) ? fmt(r.iae, 0) : '—'); set('roPu', r.osc ? fmt(r.Pu, 1) : '—'); set('roAmp', r.osc ? fmt(r.ampl, 1) : '—');
  set('roK', r.id && r.id.integrating ? `slope ${fmt(r.id.K, 3)} %/s per % · θ ${fmt(r.id.theta, 1)} s` : r.id && Number.isFinite(r.id.tau) ? `${fmt(r.id.K, 2)} · τ ${fmt(r.id.tau, 1)} s · θ ${fmt(r.id.theta, 1)} s` : '—'); set('roTune', r.tune ? `${r.tune.Kc} / ${r.tune.Ti} / ${r.tune.Td}` : '—'); set('roTuneName', r.tune ? r.tune.name : 'no data yet');
  $('#roEBox').classList.toggle('alarm', Math.abs(s.SP - s.PV) > 5); $('#roOPBox').classList.toggle('alarm', s.satT > 5); $('#roAuthBox').classList.toggle('warnv', r.auth < 0.3); $('#roOvBox').classList.toggle('alarm', r.ov > 25);
  const ps = $('#pillState'); ps.textContent = r.osc ? 'Oscillating' : s.satT > 5 ? 'Output saturated' : Math.abs(s.SP - s.PVraw) < 1 ? 'On setpoint' : s.mode === 'manual' ? 'Manual' : 'Controlling'; ps.className = 'pill ' + (r.osc || s.satT > 5 ? 'warn' : Math.abs(s.SP - s.PVraw) < 1 ? 'ok' : 'amber');
  $('#pillMode').textContent = `${s.mode.toUpperCase()} · ${{ flow: 'flow', level: 'level', temp: 'temperature' }[s.proc]} loop · Kc ${s.Kc} Ti ${s.Ti > 0 ? s.Ti : 'off'} Td ${s.Td}`; $('#pillOsc').classList.toggle('hidden', !r.osc); $('#pillWind').classList.toggle('hidden', !(s.satT > 10 && s.aw === 'off')); $('#pillStick').classList.toggle('hidden', !(r.osc && s.stick >= 1));
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start'; $('#btnMode').textContent = s.mode === 'auto' ? 'Mode: AUTO' : 'Mode: MANUAL'; $('#btnMode').classList.toggle('on', s.mode === 'auto'); $('#OPm').disabled = s.mode === 'auto';
  $('#rigTitle').textContent = { flow: 'Flow loop · pump, control valve, flow transmitter, controller', level: 'Level loop · valve fills the tank, outflow is the disturbance', temp: 'Temperature loop · steam valve on a heat exchanger with dead time' }[s.proc];
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, proc: s.proc, char: s.char, auth: +r.auth.toFixed(2), stick: s.stick, mode: s.mode, Kc: s.Kc, Ti: s.Ti, Td: s.Td, aw: s.aw, theta: s.theta, SP: s.SP, PV: +s.PVraw.toFixed(1), OP: +s.OP.toFixed(1), stem: +s.stem.toFixed(1), Q: +r.Q.toFixed(1), gain: +r.gain.toFixed(2), ov: Number.isFinite(r.ov) ? +r.ov.toFixed(0) : '', ts: Number.isFinite(r.ts) ? +r.ts.toFixed(1) : '', iae: Number.isFinite(r.iae) ? +r.iae.toFixed(0) : '', Pu: r.osc ? +r.Pu.toFixed(1) : '', K: r.id && (Number.isFinite(r.id.tau) || r.id.integrating) ? +r.id.K.toFixed(3) : '', tau: r.id && Number.isFinite(r.id.tau) ? +r.id.tau.toFixed(1) : r.id && r.id.integrating ? 'int' : '', th: r.id && (Number.isFinite(r.id.tau) || r.id.integrating) ? +r.id.theta.toFixed(1) : '' }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.proc, r.char, r.auth, r.stick, r.mode, r.Kc, r.Ti, r.Td, r.aw, r.theta, r.SP, r.PV, r.OP, r.stem, r.Q, r.gain, r.ov, r.ts, r.iae, r.Pu, r.K, r.tau, r.th].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,process,characteristic,authority,stiction_pct,mode,Kc,Ti_s,Td_s,antiwindup,deadtime_s,SP,PV,OP,stem,Q_m3h,valve_gain,overshoot_pct,settling_s,IAE,Pu_s,K_id,tau_id,theta_id\n' + state.log.map(r => [r.i, r.tag, r.proc, r.char, r.auth, r.stick, r.mode, r.Kc, r.Ti, r.Td, r.aw, r.theta, r.SP, r.PV, r.OP, r.stem, r.Q, r.gain, r.ov, r.ts, r.iae, r.Pu, r.K, r.tau, r.th].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'steptest', t: 'Open-loop step test', d: 'In manual, step the output by 10 % and let the response settle. Record the identified process gain, time constant and dead time.', ok: () => state.log.some(x => x.mode === 'manual' && x.K !== '' && x.tau !== '') },
  { id: 'offset', t: 'Proportional-only offset', d: 'Switch to auto with integral off (Ti = 0). The loop settles short of setpoint: read the offset and note how raising Kc shrinks but never removes it.', ok: () => state.flags.offset },
  { id: 'integral', t: 'Integral removes the offset', d: 'Give the controller integral action (Ti 5–20 s) and hold within 0.5 % of setpoint for 8 s.', ok: () => state.flags.integral },
  { id: 'osc', t: 'Too much gain', d: 'Raise Kc until the loop oscillates steadily. That gain is Ku and the period is Pu — the Ziegler–Nichols closed-loop data.', ok: () => state.flags.osc },
  { id: 'tuned', t: 'Apply a tuning', d: 'Apply the suggested tuning (from the step test or from Ku and Pu) and step the setpoint 10 %: overshoot under 25 % and a measured settling time.', ok: () => state.flags.tunedOK },
  { id: 'windup', t: 'Integral windup', d: 'With anti-windup off and the pump at 5 bar, set a setpoint the valve cannot reach so the output saturates for 20 s or more, then set the setpoint to 80 %: the output stays saturated while the integral unwinds and the PV hangs above setpoint. Switch anti-windup on and repeat.', ok: () => state.flags.windup },
  { id: 'stiction', t: 'Stiction limit cycle', d: 'Give the valve 2 % stiction in auto with integral action and watch the stick-slip limit cycle; then enable the positioner.', ok: () => state.flags.stiction },
  { id: 'char', t: 'Characteristic and authority', d: 'Record the loop with a linear valve and with an equal-percentage valve at an authority below 0.3. Compare the installed gain at the operating point.', ok: () => { const L = state.log.filter(x => x.auth < 0.3); return L.some(a => a.char === 'linear') && L.some(a => a.char === 'eqpct'); } },
  { id: 'deadtime', t: 'Dead time', d: 'Set 5 s or more of dead time and find a tuning that still settles with less than 25 % overshoot. Record it.', ok: () => state.log.some(x => x.theta >= 5 && x.mode === 'auto' && x.ov !== '' && x.ov < 25 && x.ts !== '') },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawStem(); if (frame % 4 === 0) { drawTrend(); if (frame % 12 === 0) { drawChar(); checkTasks(); } } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSel(id, v) { $('#' + id).value = v; state[id] = v; }
function setProc(p) { const s = state; s.proc = p; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === p)); document.querySelectorAll('[data-m]').forEach(e => e.classList.toggle('off', !e.dataset.m.split(' ').includes(p))); s.level = 50; s.Tproc = 50; s.x1 = 50; s.buf = []; s.peaks = []; s.stepT = null; s.step0 = null; s.stepLog = []; s.trace = []; if (p === 'temp') { setCtl('theta', 5); setCtl('taup', 30); } else if (p === 'flow') { setCtl('theta', 1); setCtl('taup', 3); } else { setCtl('theta', 2); setCtl('taup', 3); } }
function setMode(m) { const s = state; if (m === 'auto' && s.mode !== 'auto') { const e = (s.action === 'reverse' ? 1 : -1) * (s.SP - s.PV); s.I = s.OP - s.Kc * e; s.PVf = s.PV; } s.mode = m; if (m === 'manual') { setCtl('OPm', Math.round(s.OP)); } }
const PRESETS = {
  base: () => { setProc('flow'); setSel('char', 'eqpct'); setSel('positioner', 'off'); setSel('aw', 'on'); setSel('action', 'reverse'); for (const [k, v] of Object.entries({ Kv: 60, Kp: 0.6, taua: 1, stick: 0, SP: 50, OPm: 60, Kc: 1, Ti: 10, Td: 0, noise: 0.2, Ps: 6, Pd: 2, load: 30, tscale: 1 })) setCtl(k, v); setMode('manual'); state.run = true; },
  steptest: () => { PRESETS.base(); state.stem = state.target = state.OP = 60; state.x1 = flowAt(0.6).Q / valveInfo().Qfull * 100; startStep('op', 70); },
  ponly: () => { PRESETS.base(); setCtl('Ti', 0); setCtl('Kc', 2); setMode('auto'); startStep('sp', 60); },
  pi: () => { PRESETS.base(); setCtl('Kc', 1.2); setCtl('Ti', 4); setMode('auto'); startStep('sp', 60); },
  highgain: () => { PRESETS.base(); setCtl('Kc', 5); setCtl('Ti', 4); setMode('auto'); startStep('sp', 55); },
  windup: () => { PRESETS.base(); setSel('aw', 'off'); setCtl('Ps', 5); setCtl('Kc', 1.2); setCtl('Ti', 4); setMode('auto'); startStep('sp', 95); },
  stiction: () => { PRESETS.base(); setCtl('stick', 3); setCtl('Kc', 1.2); setCtl('Ti', 4); setMode('auto'); startStep('sp', 55); },
  lowauth: () => { PRESETS.base(); setCtl('Kp', 8); setSel('char', 'linear'); setMode('auto'); setCtl('Kc', 1.2); setCtl('Ti', 4); startStep('sp', 60); },
  temp: () => { PRESETS.base(); setProc('temp'); setCtl('Kc', 1.5); setCtl('Ti', 30); setCtl('Td', 2); setMode('auto'); startStep('sp', 60); },
  level: () => { PRESETS.base(); setProc('level'); setCtl('OPm', 75); setCtl('Kc', 3); setCtl('Ti', 60); setMode('auto'); startStep('sp', 60); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('Kv', v => String(v)); bindR('Kp', v => v.toFixed(1)); bindR('taua', v => v.toFixed(1) + ' s'); bindR('stick', v => v.toFixed(1) + ' %'); bindR('SP', v => v + ' %'); bindR('OPm', v => v + ' %'); bindR('Kc', v => v.toFixed(2)); bindR('Ti', v => v > 0 ? v + ' s' : 'off'); bindR('Td', v => v.toFixed(1) + ' s'); bindR('theta', v => v.toFixed(1) + ' s'); bindR('taup', v => v.toFixed(0) + ' s'); bindR('noise', v => v.toFixed(1) + ' %'); bindR('Ps', v => v.toFixed(1) + ' bar'); bindR('Pd', v => v.toFixed(1) + ' bar'); bindR('load', v => v + ' m³/h'); bindR('tscale', v => '×' + v);
  $('#SP').addEventListener('change', () => startStep('sp', state.SP)); $('#OPm').addEventListener('change', () => { if (state.mode === 'manual') startStep('op', state.OPm); }); $('#Ps').addEventListener('change', () => startStep('dist')); $('#load').addEventListener('change', () => startStep('dist'));
  for (const id of ['char', 'positioner', 'aw', 'action']) $('#' + id).addEventListener('change', e => state[id] = e.target.value); document.querySelectorAll('#types button').forEach(b => b.onclick = () => setProc(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnMode').onclick = () => setMode(state.mode === 'auto' ? 'manual' : 'auto'); $('#btnSPup').onclick = () => startStep('sp', state.SP + 10); $('#btnSPdn').onclick = () => startStep('sp', state.SP - 10); $('#btnApply').onclick = () => { if (R.tune) { setCtl('Kc', R.tune.Kc); setCtl('Ti', R.tune.Ti); setCtl('Td', R.tune.Td); state.flags.tuned = true; } }; $('#btnRecord').onclick = () => record(''); $('#btnClearT').onclick = () => { state.trace = []; state.peaks = []; };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('cv_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'A') $('#btnMode').click(); if (k === 'R') record(''); if (e.key === '+' || e.key === '=') $('#btnSPup').click(); if (e.key === '-') $('#btnSPdn').click(); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.ControlLoop = {
  getState: () => ({ inputs: { proc: state.proc, char: state.char, Kv: state.Kv, Kp: state.Kp, taua: state.taua, stick: state.stick, positioner: state.positioner, mode: state.mode, SP: state.SP, OPm: state.OPm, Kc: state.Kc, Ti: state.Ti, Td: state.Td, aw: state.aw, action: state.action, theta: state.theta, taup: state.taup, Ps: state.Ps, Pd: state.Pd, load: state.load, run: state.run }, results: { PV: state.PVraw, OP: state.OP, stem: state.stem, Q: R.Q, auth: R.auth, gain: R.gain, osc: R.osc, Pu: R.Pu, ov: R.ov, ts: R.ts, iae: R.iae, id: R.id, tune: R.tune, satT: state.satT }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.proc) setProc(inp.proc); for (const k of ['char', 'positioner', 'aw', 'action']) if (inp[k]) setSel(k, inp[k]); for (const k of ['Kv', 'Kp', 'taua', 'stick', 'OPm', 'Kc', 'Ti', 'Td', 'theta', 'taup', 'noise', 'Ps', 'Pd', 'load', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.SP !== undefined) { setCtl('SP', inp.SP); startStep('sp', inp.SP); } if (inp.mode) setMode(inp.mode); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.applyTuning) $('#btnApply').click(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'control-valve-pid', title: 'Control Valve and PID Loop Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.ControlLoop.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'control-valve-pid', state: window.ControlLoop.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('cv_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setProc('flow'); state.x1 = flowAt(0.6).Q / valveInfo().Qfull * 100; state.PV = state.PVraw = state.x1; step(0); drawRig(); updateReadouts(); drawTrend(); drawChar(); drawStem();
  requestAnimationFrame(tick);
}
build();
