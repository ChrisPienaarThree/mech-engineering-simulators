/* Heat Exchanger Trainer — effectiveness-NTU rating of counterflow, parallel, 1-2 shell-and-tube, cross-flow and plate
   arrangements with film coefficients scaled from the clean U, fouling growing on a compressed clock, a hot-side bypass,
   pressure drops and first-order outlet lag. LMTD and F are reported from the solution. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const FLUIDS = { water: { cp: 4180, rho: 998, boil: 100, name: 'water', dpk: 1 }, oil: { cp: 2000, rho: 870, boil: 250, name: 'lube oil', dpk: 3 }, glycol: { cp: 3500, rho: 1060, boil: 105, name: '50 % glycol', dpk: 1.6 } };
const MH0 = 2, MC0 = 3, RF_MAX = 0.001;
const TYPES = { counter: 'Counterflow', parallel: 'Parallel flow', shell12: 'Shell-and-tube 1-2', cross: 'Cross-flow (both unmixed)', plate: 'Plate (counterflow)' };

/* ------------------------------------------------------------------ state */
const state = { type: 'counter', fh: 'oil', mh: 2, Thi: 90, fc: 'water', mc: 3, Tci: 25, A: 12, Uc: 600, foulRate: 1, bypass: 0, tscale: 1, run: false, Rf: 0, days: 0, Tho: 90, Tco: 25, t: 0, trace: [], traceAcc: 0, log: [], flags: {}, cleanedAfter: false };
const R = {};

/* ------------------------------------------------------------------ ε-NTU */
function eps(type, NTU, Cr) {
  if (Cr < 1e-6) return 1 - Math.exp(-NTU);
  if (type === 'counter' || type === 'plate') return Math.abs(Cr - 1) < 1e-6 ? NTU / (1 + NTU) : (1 - Math.exp(-NTU * (1 - Cr))) / (1 - Cr * Math.exp(-NTU * (1 - Cr)));
  if (type === 'parallel') return (1 - Math.exp(-NTU * (1 + Cr))) / (1 + Cr);
  if (type === 'shell12') { const s = Math.sqrt(1 + Cr * Cr), e = Math.exp(-NTU * s); return 2 / (1 + Cr + s * (1 + e) / (1 - e)); }
  return 1 - Math.exp(Math.pow(NTU, 0.22) / Cr * (Math.exp(-Cr * Math.pow(NTU, 0.78)) - 1));
}
function solve() {
  const s = state, H = FLUIDS[s.fh], Cf = FLUIDS[s.fc], mhe = s.mh * (1 - s.bypass / 100), Ch = mhe * H.cp / 1000, Cc = s.mc * Cf.cp / 1000;   // kW/K
  const Cmin = Math.min(Ch, Cc), Cmax = Math.max(Ch, Cc), Cr = Cmin / Cmax;
  const hh = 2 * s.Uc * Math.pow(Math.max(mhe, 0.01) / MH0, 0.8), hc = 2 * s.Uc * Math.pow(s.mc / MC0, 0.8), Uclean = 1 / (1 / hh + 1 / hc), U = 1 / (1 / Uclean + s.Rf);
  const NTU = U * s.A / (Cmin * 1000), e = eps(s.type, NTU, Cr), dTin = s.Thi - s.Tci, Qmax = Cmin * dTin, Q = e * Qmax;
  const Tho_e = s.Thi - Q / Ch, Tco = s.Tci + Q / Cc, Tho = (mhe * Tho_e + s.mh * s.bypass / 100 * s.Thi) / s.mh;
  const d1 = s.type === 'parallel' ? s.Thi - s.Tci : s.Thi - Tco, d2 = s.type === 'parallel' ? Tho_e - Tco : Tho_e - s.Tci, LMTD = Math.abs(d1 - d2) < 1e-6 ? d1 : (d1 - d2) / Math.log(d1 / d2);
  const d1c = s.Thi - Tco, d2c = Tho_e - s.Tci, LMTDc = Math.abs(d1c - d2c) < 1e-6 ? d1c : (d1c - d2c) / Math.log(d1c / d2c), F = s.type === 'counter' || s.type === 'plate' || s.type === 'parallel' ? 1 : Q * 1000 / (U * s.A * LMTDc);
  const approach = Math.min(s.Thi - Tco, Tho_e - s.Tci), dph = 15 * H.dpk * Math.pow(mhe / MH0, 2) * (s.type === 'plate' ? 2 : 1), dpc = 10 * Cf.dpk * Math.pow(s.mc / MC0, 2) * (s.type === 'plate' ? 2 : 1), Pp = dph * 1000 * mhe / H.rho + dpc * 1000 * s.mc / Cf.rho;
  const Uclean0 = 1 / (1 / hh + 1 / hc), Qclean = eps(s.type, Uclean0 * s.A / (Cmin * 1000), Cr) * Qmax, foulPct = (1 - Q / Qclean) * 100;
  const eC = eps('counter', NTU, Cr), crossPossible = s.Tci + eC * Qmax / Cc > s.Thi - eC * Qmax / Ch;
  return { Ch, Cc, Cmin, Cmax, Cr, Uclean, U, NTU, e, Qmax, Q, Tho, Tco, Tho_e, LMTD, F, approach, dph, dpc, Pp, foulPct, cminIs: Ch <= Cc ? 'hot' : 'cold', boil: Tco >= Cf.boil - 2, crossPossible, tubes: Math.round(s.A / (s.type === 'plate' ? 0.25 : 0.19)), mhe };
}

/* ------------------------------------------------------------------ frame */
function step(dt) {
  const s = state; if (s.run) { const days = dt * s.tscale; s.days += days; const k = 3e-5 * s.foulRate * Math.exp((s.Thi - 90) / 40) * (s.Uc / 600) ** 0.3; s.Rf = Math.min(RF_MAX, s.Rf + k * days * (1 - s.Rf / RF_MAX)); }
  Object.assign(R, solve()); const lag = Math.min(1, dt / 4);
  if (s.run) { s.Tho += (R.Tho - s.Tho) * lag; s.Tco += (R.Tco - s.Tco) * lag; } else { s.Tho += (s.Thi - s.Tho) * lag * 0.3; s.Tco += (s.Tci - s.Tco) * lag * 0.3; }
  s.traceAcc += dt; if (s.traceAcc >= 0.25) { s.traceAcc = 0; s.trace.push({ t: s.t, Q: s.run ? R.Q : 0, Tho: s.Tho, Tco: s.Tco, Rf: s.Rf, days: s.days }); if (s.trace.length > 1200) s.trace.shift(); }
  if (s.run && R.foulPct >= 15) s.flags.fouled = true; if (s.run && s.flags.fouled && R.foulPct < 2) s.flags.cleaned = true;
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg, dashH = 0, dashC = 0;
const tcol = T => { const k = clamp((T - 10) / 120, 0, 1); return `hsl(${210 - 210 * k},70%,${45 + 10 * (1 - k)}%)`; };
function buildRig() { svg = el('svg', { viewBox: '0 0 900 300', role: 'img', 'aria-label': 'Heat exchanger schematic' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig(dt) {
  svg.innerHTML = ''; const s = state, x0 = 200, x1 = 700, yT = 150, par = s.type === 'parallel', plate = s.type === 'plate';
  el('rect', { x: x0, y: yT - 60, width: x1 - x0, height: 120, rx: plate ? 4 : 30, class: 'shell' }, svg);
  if (plate) for (let i = 0; i < 12; i++) el('line', { x1: x0 + 20 + i * 38, y1: yT - 50, x2: x0 + 20 + i * 38, y2: yT + 50, class: 'plate' }, svg);
  else for (let i = -1; i <= 1; i++) { const y = yT + i * 26; el('line', { x1: x0 + 10, y1: y, x2: x1 - 10, y2: y, class: 'tube', stroke: 'var(--steel-light)' }, svg); const g = el('linearGradient', { id: 'gh' + i, x1: par ? 0 : 0, x2: 1, y1: 0, y2: 0 }, svg); el('stop', { offset: 0, 'stop-color': tcol(par || !par ? s.Thi : s.Tho) }, g); el('stop', { offset: 1, 'stop-color': tcol(s.Tho) }, g); el('line', { x1: x0 + 10, y1: y, x2: x1 - 10, y2: y, class: 'tube', stroke: `url(#gh${i})`, 'stroke-width': 4 }, svg); }
  const foul = clamp(s.Rf / RF_MAX, 0, 1); el('rect', { x: x0 + 10, y: yT - 52, width: x1 - x0 - 20, height: 104, fill: '#7a5a2a', opacity: 0.45 * foul, rx: 24 }, svg);
  // hot stream pipes (left in, right out), cold stream (parallel: left in; counter: right in)
  el('path', { d: `M60,${yT} H${x0}`, class: 'pipe' }, svg); el('path', { d: `M${x1},${yT} H840`, class: 'pipe' }, svg);
  dashH += R.mhe * 10 * dt; el('path', { d: `M60,${yT} H${x0}`, class: 'flow', stroke: tcol(s.Thi), style: `stroke-dashoffset:${-dashH}` }, svg); el('path', { d: `M${x1},${yT} H840`, class: 'flow', stroke: tcol(s.Tho), style: `stroke-dashoffset:${-dashH}` }, svg);
  const cIn = par ? x0 + 40 : x1 - 40, cOut = par ? x1 - 40 : x0 + 40; el('path', { d: `M${cIn},${yT + 60} V260`, class: 'pipe' }, svg); el('path', { d: `M${cOut},${yT - 60} V40`, class: 'pipe' }, svg);
  dashC += s.mc * 8 * dt; el('path', { d: `M${cIn},260 V${yT + 60}`, class: 'flow', stroke: tcol(s.Tci), style: `stroke-dashoffset:${-dashC}` }, svg); el('path', { d: `M${cOut},${yT - 60} V40`, class: 'flow', stroke: tcol(s.Tco), style: `stroke-dashoffset:${-dashC}` }, svg);
  txt(62, yT - 14, `HOT IN ${fmt(s.Thi, 0)} °C · ${s.mh} kg/s ${FLUIDS[s.fh].name}`, 'ttag hot', svg); txt(838, yT - 14, `HOT OUT ${fmt(s.Tho, 1)} °C`, 'ttag hot', svg, 'end');
  txt(cIn + (par ? 8 : -8), 275, `COLD IN ${fmt(s.Tci, 0)} °C · ${s.mc} kg/s ${FLUIDS[s.fc].name}`, 'ttag cold', svg, par ? 'start' : 'end'); txt(cOut + (par ? -8 : 8), 34, `COLD OUT ${fmt(s.Tco, 1)} °C`, 'ttag cold', svg, par ? 'end' : 'start');
  if (s.bypass > 0) { el('path', { d: `M120,${yT} V70 H780 V${yT}`, class: 'pipe', 'stroke-width': 5 }, svg); el('path', { d: `M120,${yT} V70 H780 V${yT}`, class: 'flow', stroke: tcol(s.Thi), 'stroke-width': 3, style: `stroke-dashoffset:${-dashH * s.bypass / 100}` }, svg); el('polygon', { points: '440,62 456,70 440,78', class: 'valve' }, svg); el('polygon', { points: '472,62 456,70 472,78', class: 'valve' }, svg); txt(456, 92, `bypass ${s.bypass} %`, 'lbl', svg, 'middle'); }
  txt(450, yT + 82, `${TYPES[s.type]} · A ${s.A} m² · U ${fmt(R.U, 0)} W/m²K (clean ${fmt(R.Uclean, 0)}) · fouling ${fmt(s.Rf * 1000, 2)} m²K/kW after ${fmt(s.days, 0)} days`, 'lbl', svg, 'middle');
  txt(450, yT + 98, `duty ${fmt(R.Q, 1)} kW · ε ${fmt(R.e, 3)} · NTU ${fmt(R.NTU, 2)} · Cr ${fmt(R.Cr, 2)}`, 'val', svg, 'middle');
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function profile() {   // temperature of each stream against fractional area, marching with the local rate equation
  const s = state, n = 100, U = R.U, A = s.A, Ch = R.Ch * 1000, Cc = R.Cc * 1000, hot = [], cold = [];
  if (s.type === 'counter' || s.type === 'plate' || s.type === 'parallel') {
    const par = s.type === 'parallel'; let Th = s.Thi, Tc = par ? s.Tci : R.Tco;   // counterflow: cold enters at the far end, so march from the hot inlet with the cold outlet known
    for (let i = 0; i <= n; i++) { hot.push(Th); cold.push(Tc); const dA = A / n, q = U * dA * (Th - Tc); Th -= q / Ch; Tc += (par ? 1 : -1) * q / Cc; }
  } else { for (let i = 0; i <= n; i++) { const x = i / n, k = 1 - Math.exp(-R.NTU * x * 1.2) / (1) ; hot.push(s.Thi + (R.Tho_e - s.Thi) * (1 - Math.exp(-2.2 * x)) / (1 - Math.exp(-2.2))); cold.push(s.type === 'cross' ? s.Tci + (R.Tco - s.Tci) * (1 - Math.exp(-2.2 * x)) / (1 - Math.exp(-2.2)) : R.Tco + (s.Tci - R.Tco) * (1 - Math.exp(-2.2 * x)) / (1 - Math.exp(-2.2))); } }
  return { hot, cold };
}
function drawProfile() {
  const cv = $('#profile'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 20, T = 14, B = 30, pw = W - L - Rr, ph = H - T - B, s = state, { hot, cold } = profile();
  const tmin = Math.min(s.Tci, ...cold) - 5, tmax = Math.max(s.Thi, ...hot) + 5, X = i => L + i / 100 * pw, Y = t => T + (1 - (t - tmin) / (tmax - tmin)) * ph;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.strokeStyle = col('--grid'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const stepT = tmax - tmin > 100 ? 25 : 10; for (let t = Math.ceil(tmin / stepT) * stepT; t <= tmax; t += stepT) { ctx.beginPath(); ctx.moveTo(L, Y(t)); ctx.lineTo(L + pw, Y(t)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(t + ' °C', L - 4, Y(t) + 3); }
  const line = (arr, color) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 2.5; arr.forEach((t, i) => i ? ctx.lineTo(X(i), Y(t)) : ctx.moveTo(X(i), Y(t))); ctx.stroke(); };
  line(hot, col('--alarm')); line(cold, col('--water'));
  let kmin = 0; for (let i = 0; i <= 100; i++) if (hot[i] - cold[i] < hot[kmin] - cold[kmin]) kmin = i;
  ctx.strokeStyle = col('--brass'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(kmin), Y(hot[kmin])); ctx.lineTo(X(kmin), Y(cold[kmin])); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--brass'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = kmin > 60 ? 'right' : 'left'; ctx.fillText(`pinch ${fmt(hot[kmin] - cold[kmin], 1)} K`, X(kmin) + (kmin > 60 ? -6 : 6), (Y(hot[kmin]) + Y(cold[kmin])) / 2 + 4);
  ctx.fillStyle = col('--alarm'); ctx.textAlign = 'left'; ctx.fillText(`hot ${fmt(hot[0], 1)} → ${fmt(hot[100], 1)}`, L + 6, Y(hot[0]) - 6); ctx.fillStyle = col('--water'); ctx.fillText(`cold ${fmt(cold[0], 1)} → ${fmt(cold[100], 1)}`, L + 6, Y(cold[0]) + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`position along the exchanger, hot inlet end → hot outlet end (${TYPES[s.type]}${s.type === 'shell12' || s.type === 'cross' ? ' — profile indicative' : ''})`, L + pw / 2, H - 6);
  if (s.type === 'counter' || s.type === 'plate') { const cross = cold[0] > hot[100]; if (cross) { ctx.fillStyle = col('--good'); ctx.textAlign = 'right'; ctx.fillText('temperature cross: cold out above hot out', L + pw - 6, T + 14); } }
}
function drawENTU() {
  const cv = $('#entu'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 20, T = 14, B = 30, pw = W - L - Rr, ph = H - T - B, X = n => L + n / 6 * pw, Y = e => T + (1 - e) * ph;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.strokeStyle = col('--grid'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  for (let n = 0; n <= 6; n++) { ctx.beginPath(); ctx.moveTo(X(n), T); ctx.lineTo(X(n), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(n, X(n), H - 16); } for (let e = 0; e <= 1; e += 0.2) { ctx.beginPath(); ctx.moveTo(L, Y(e)); ctx.lineTo(L + pw, Y(e)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(e.toFixed(1), L - 4, Y(e) + 3); }
  const colors = { counter: '--good', parallel: '--warn', shell12: '--brass', cross: '--water-deep', plate: '--good' };
  for (const t of ['counter', 'parallel', 'shell12', 'cross']) { ctx.beginPath(); ctx.strokeStyle = col(colors[t]); ctx.lineWidth = t === state.type || (state.type === 'plate' && t === 'counter') ? 3 : 1.3; ctx.setLineDash(t === state.type || (state.type === 'plate' && t === 'counter') ? [] : [4, 3]); for (let n = 0; n <= 6; n += 0.05) { const e = eps(t, Math.max(n, 1e-6), R.Cr); n ? ctx.lineTo(X(n), Y(e)) : ctx.moveTo(X(n), Y(e)); } ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col(colors[t]); ctx.textAlign = 'left'; ctx.font = '600 11px Barlow Condensed, sans-serif'; const lx = { counter: 4.6, parallel: 3.2, shell12: 5.2, cross: 2.2 }[t]; ctx.fillText(TYPES[t], X(lx) + 4, Y(eps(t, lx, R.Cr)) + (t === 'parallel' || t === 'cross' ? 14 : -5)); }
  ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(Math.min(6, R.NTU)), Y(R.e), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`NTU ${fmt(R.NTU, 2)} · ε ${fmt(R.e, 3)} · Cr ${fmt(R.Cr, 2)}`, X(Math.min(6, R.NTU)) + 10, Y(R.e) + 16);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('NTU = UA / Cmin', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, span = 120, tr = state.trace.filter(p => p.t > state.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const X = t => L + (t - (state.t - span)) / span * pw;
  const qmax = Math.max(1, ...tr.map(p => p.Q)) * 1.1, tmax = Math.max(state.Thi, ...tr.map(p => p.Tho)) + 5;
  const series = (key, color, lo, hi, w = 1.6) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { const y = T + (1 - clamp((p[key] - lo) / (hi - lo), 0, 1)) * ph; f ? ctx.moveTo(X(p.t), y) : ctx.lineTo(X(p.t), y); f = false; } ctx.stroke(); };
  series('Q', col('--good'), 0, qmax, 2.2); series('Tho', col('--alarm'), 0, tmax); series('Tco', col('--water'), 0, tmax); series('Rf', '#7a5a2a', 0, RF_MAX);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`${fmt(qmax, 0)} kW`, L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--good'); ctx.fillText('duty', L + pw + 4, T + 10); ctx.fillStyle = col('--alarm'); ctx.fillText('hot out', L + pw + 4, T + 24); ctx.fillStyle = col('--water'); ctx.fillText('cold out', L + pw + 4, T + 38); ctx.fillStyle = '#7a5a2a'; ctx.fillText('fouling', L + pw + 4, T + 52);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s · fouling clock ${state.tscale} day/s · ${fmt(state.days, 0)} days run`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R; $('#pillRun').textContent = s.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', s.run);
  $('#roQ').textContent = fmt(r.Q, 1); $('#roQmax').textContent = fmt(r.Qmax, 1); $('#roEps').textContent = fmt(r.e, 3); $('#roNTU').textContent = fmt(r.NTU, 2); $('#roCr').textContent = fmt(r.Cr, 2); $('#roTho').textContent = fmt(s.Tho, 1); $('#roThi').textContent = s.Thi; $('#roDTh').textContent = fmt(s.Thi - r.Tho, 1); $('#roTco').textContent = fmt(s.Tco, 1); $('#roTci').textContent = s.Tci; $('#roDTc').textContent = fmt(r.Tco - s.Tci, 1); $('#roTcoBox').classList.toggle('alarm', r.boil);
  $('#roLMTD').textContent = fmt(r.LMTD, 1); $('#roF').textContent = fmt(r.F, 3); $('#roApp').textContent = fmt(r.approach, 1); $('#roU').textContent = fmt(r.U, 0); $('#roUc').textContent = fmt(r.Uclean, 0); $('#roUA').textContent = fmt(r.U * s.A / 1000, 2); $('#roUBox').classList.toggle('warnv', r.foulPct > 10); $('#roRf').textContent = fmt(s.Rf * 1000, 3); $('#roFoulPct').textContent = fmt(r.foulPct, 1); $('#roRfBox').classList.toggle('warnv', r.foulPct > 10); $('#roRfBox').classList.toggle('alarm', r.foulPct > 25);
  $('#roCh').textContent = fmt(r.Ch, 2); $('#roCc').textContent = fmt(r.Cc, 2); $('#roCmin').textContent = r.cminIs + ' stream'; $('#roDph').textContent = fmt(r.dph, 1); $('#roDpc').textContent = fmt(r.dpc, 1); $('#roPp').textContent = fmt(r.Pp, 0); $('#roDpBox').classList.toggle('warnv', Math.max(r.dph, r.dpc) > 60); $('#roA').textContent = s.A; $('#roN').textContent = r.tubes;
  const pd = $('#pillDuty'); pd.textContent = `duty ${fmt(r.Q, 1)} kW${s.bypass ? ' · bypass ' + s.bypass + ' %' : ''}`; pd.className = 'pill ' + (r.boil ? 'warn' : 'ok'); $('#pillEff').textContent = `ε ${fmt(r.e, 2)} · NTU ${fmt(r.NTU, 2)}`;
  $('#pillFoul').classList.toggle('hidden', r.foulPct < 10); $('#pillBoil').classList.toggle('hidden', !r.boil); $('#pillCross').classList.toggle('hidden', !(s.type === 'parallel' && r.crossPossible)); $('#pillDp').classList.toggle('hidden', Math.max(r.dph, r.dpc) <= 60);
  $('#rigTitle').textContent = TYPES[s.type] + (s.type === 'plate' ? ' · hot and cold in alternate channels' : ' · hot stream in the tubes, cold in the shell'); $('#btnRun').textContent = s.run ? 'Running' : 'Start';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, type: s.type, fh: s.fh, mh: s.mh, Thi: s.Thi, Tho: +r.Tho.toFixed(1), fc: s.fc, mc: s.mc, Tci: s.Tci, Tco: +r.Tco.toFixed(1), A: s.A, U: +r.U.toFixed(0), Rf: +(s.Rf * 1000).toFixed(3), Q: +r.Q.toFixed(1), e: +r.e.toFixed(3), NTU: +r.NTU.toFixed(2), Cr: +r.Cr.toFixed(2), LMTD: +r.LMTD.toFixed(1), F: +r.F.toFixed(3), app: +r.approach.toFixed(1), byp: s.bypass }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.type, r.fh, r.mh, r.Thi, r.Tho, r.fc, r.mc, r.Tci, r.Tco, r.A, r.U, r.Rf, r.Q, r.e, r.NTU, r.Cr, r.LMTD, r.F].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,type,hot_fluid,mh_kgs,Thi,Tho,cold_fluid,mc_kgs,Tci,Tco,A_m2,U_Wm2K,Rf_m2KkW,Q_kW,eps,NTU,Cr,LMTD,F\n' + state.log.map(r => [r.i, r.tag, r.type, r.fh, r.mh, r.Thi, r.Tho, r.fc, r.mc, r.Tci, r.Tco, r.A, r.U, r.Rf, r.Q, r.e, r.NTU, r.Cr, r.LMTD, r.F].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b) => a.fh === b.fh && a.mh === b.mh && a.Thi === b.Thi && a.fc === b.fc && a.mc === b.mc && a.Tci === b.Tci && a.A === b.A;
const TASKS = [
  { id: 'cvp', t: 'Counterflow against parallel flow', d: 'Record the same exchanger and flows in counterflow and in parallel flow. Compare duty and cold outlet.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.type === 'counter' && b.type === 'parallel' && same(a, b))); } },
  { id: 'cross', t: 'Make a temperature cross', d: 'In counterflow, get the cold outlet above the hot outlet and record it. Then try the same in parallel flow.', ok: () => state.log.some(r => (r.type === 'counter' || r.type === 'plate') && r.Tco > r.Tho) },
  { id: 'balanced', t: 'Balanced flows', d: 'Set the flows so the capacity rates match (Cr within 0.95–1.05) and record: parallel temperature lines, lowest ε for the NTU.', ok: () => state.log.some(r => r.Cr >= 0.95) },
  { id: 'pinch', t: 'Find the pinch', d: 'Starve one stream until the approach temperature falls below 3 K and record it. Which stream limits the duty now?', ok: () => state.log.some(r => r.app < 3) },
  { id: 'flow', t: 'Double the cold flow', d: 'Record, double the cold flow, record again: duty up, cold outlet down, pressure drop four times.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.type === b.type && a.fh === b.fh && a.mh === b.mh && a.Thi === b.Thi && a.fc === b.fc && a.Tci === b.Tci && a.A === b.A && Math.abs(b.mc - 2 * a.mc) < 0.11)); } },
  { id: 'foul', t: 'Foul it, then clean it', d: 'Run with fouling until the duty has fallen 15 %, then clean the exchanger and watch it recover.', ok: () => state.flags.fouled && state.flags.cleaned },
  { id: 'size', t: 'Size the oil cooler', d: 'Oil 2 kg/s at 90 °C, water 3 kg/s at 25 °C, counterflow, clean: choose the smallest area that brings the oil out at 50 °C or below.', ok: () => state.flags.size },
  { id: 'shell', t: 'The F factor', d: 'Record a 1-2 shell-and-tube reading: F below 1 shows the penalty for the partial parallel-flow pass.', ok: () => state.log.some(r => r.type === 'shell12' && r.F < 0.995) },
  { id: 'bypass', t: 'Control with the bypass', d: 'Use the hot-side bypass to hold the hot outlet at 55 ± 2 °C while the flows stay put. Record it.', ok: () => state.log.some(r => r.byp > 0 && Math.abs(r.Tho - 55) <= 2) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { const s = state; if (s.run && s.type === 'counter' && s.fh === 'oil' && s.mh === 2 && s.Thi === 90 && s.fc === 'water' && s.mc === 3 && s.Tci === 25 && s.Rf < 1e-5 && s.bypass === 0 && R.Tho <= 50 && R.Tho > 45) s.flags.size = true; for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt; step(dt); drawRig(dt); frame++; if (frame % 2 === 0) { updateReadouts(); drawProfile(); drawTrace(); if (frame % 6 === 0) { drawENTU(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setType(t) { state.type = t; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === t)); }
const PRESETS = {
  counter: () => { setType('counter'); for (const [k, v] of Object.entries({ mh: 2, Thi: 90, mc: 3, Tci: 25, A: 12, Uc: 600, foulRate: 1, bypass: 0, tscale: 1 })) setCtl(k, v); $('#fh').value = 'oil'; state.fh = 'oil'; $('#fc').value = 'water'; state.fc = 'water'; state.Rf = 0; state.days = 0; state.run = true; },
  parallel: () => { PRESETS.counter(); setType('parallel'); },
  balanced: () => { PRESETS.counter(); setCtl('mc', 1.0); },
  pinch: () => { PRESETS.counter(); setCtl('mc', 0.4); },
  foul: () => { PRESETS.counter(); setCtl('foulRate', 3); setCtl('tscale', 5); },
  plate: () => { PRESETS.counter(); setType('plate'); $('#fh').value = 'water'; state.fh = 'water'; setCtl('A', 4); setCtl('Uc', 3000); },
  shell: () => { PRESETS.counter(); setType('shell12'); },
  boil: () => { PRESETS.counter(); setCtl('Thi', 180); setCtl('mc', 0.3); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('mh', v => v.toFixed(1) + ' kg/s'); bindR('Thi', v => v + ' °C'); bindR('mc', v => v.toFixed(1) + ' kg/s'); bindR('Tci', v => v + ' °C'); bindR('A', v => v + ' m²'); bindR('Uc', v => v + ' W/m²K'); bindR('foulRate', v => ['none', 'light', 'normal', 'heavy', 'severe', 'severe', 'extreme'][Math.round(v * 2)] || 'heavy'); bindR('bypass', v => v + ' %'); bindR('tscale', v => v === 0 ? 'frozen' : v + ' day / s');
  $('#fh').addEventListener('change', e => state.fh = e.target.value); $('#fc').addEventListener('change', e => state.fc = e.target.value);
  document.querySelectorAll('#types button').forEach(b => b.onclick = () => setType(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnClean').onclick = () => { state.Rf = 0; state.days = 0; }; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('hx_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') state.run = !state.run; else if (k === 'C') $('#btnClean').click(); else if (k === 'R') record(''); else if (/^[1-5]$/.test(k)) setType(['counter', 'parallel', 'shell12', 'cross', 'plate'][+k - 1]); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawProfile(); drawENTU(); drawTrace(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.HeatExchanger = {
  getState: () => ({ inputs: { type: state.type, fh: state.fh, mh: state.mh, Thi: state.Thi, fc: state.fc, mc: state.mc, Tci: state.Tci, A: state.A, Uc: state.Uc, foulRate: state.foulRate, bypass: state.bypass, run: state.run }, results: { Q: R.Q, eps: R.e, NTU: R.NTU, Cr: R.Cr, Tho: R.Tho, Tco: R.Tco, LMTD: R.LMTD, F: R.F, U: R.U, Rf: state.Rf, foulPct: R.foulPct, approach: R.approach, dph: R.dph, dpc: R.dpc, days: state.days }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.type) setType(inp.type); for (const k of ['mh', 'Thi', 'mc', 'Tci', 'A', 'Uc', 'foulRate', 'bypass', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); for (const k of ['fh', 'fc']) if (inp[k]) { $('#' + k).value = inp[k]; state[k] = inp[k]; } if (inp.clean) { state.Rf = 0; state.days = 0; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'heat-exchanger', title: 'Heat Exchanger Trainer', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.HeatExchanger.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'heat-exchanger', state: window.HeatExchanger.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('hx_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); step(0); state.Tho = R.Tho; state.Tco = R.Tco; drawRig(0); updateReadouts(); drawProfile(); drawENTU(); drawTrace();
  requestAnimationFrame(tick);
}
build();
