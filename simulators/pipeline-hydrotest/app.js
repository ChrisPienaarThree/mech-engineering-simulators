/* Pipeline Hydrostatic Test Bench — strength and leak test of an onshore gas pipeline after construction, after
   ASME B31.8-2003 841.3 and Appendix N: test sections chosen from the elevation profile, fill behind a pig, 80 % hold,
   pressurise along a pressure–volume plot (water compressibility, pipe expansion, trapped air, yielding joints),
   N5 yield rules (2 × strokes, 0.002 × fill volume), 2 h hold (841.321) with temperature drift against leaks, burst of
   damaged pipe, and the MAOP the test supports (Table 841.322(f)). Standalone, offline. Teaching model. */
'use strict';

const r3 = x => Number.isFinite(x) ? Math.round(x * 1000) / 1000 : x;
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const L = 24000, M = 160, RHOG = 0.0981, PATM = 1.013, E_ST = 207000, NU = 0.3, ALPHA = 1.2e-5, KAPPA = 4.6e-10, ET = 4000, STROKE = 0.005, QFILL = 250 / 3600;
const PROFILE = [[0, 1180], [3000, 1210], [6000, 1320], [8000, 1460], [10000, 1500], [12000, 1390], [15000, 1250], [18000, 1120], [21000, 1150], [24000, 1230]];
const AIR = { pig: 0.0005, vent: 0.004, none: 0.015 };
const RATIO = { '1d1': 1.25, '1d2': 1.1, '2': 1.25, '3': 1.40 }, FDES = { '1d1': 0.80, '1d2': 0.72, '2': 0.60, '3': 0.50 };
function zAt(x) { let i = 1; while (i < PROFILE.length - 1 && PROFILE[i][0] < x) i++; const [x0, z0] = PROFILE[i - 1], [x1, z1] = PROFILE[i]; return z0 + (z1 - z0) * clamp((x - x0) / (x1 - x0), 0, 1) + 6 * Math.sin(x / 700); }
const betaW = T => 1e-6 * (-68 + 17.6 * T - 0.12 * T * T); // water volumetric expansion, 1/K, fitted 4–35 °C
function rng(seed) { let a = seed >>> 0; return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
const gauss = r => Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r());

const state = { OD: 406.4, wt: 5.6, S: 359, MAOP: 70, cls: '1d2', lim: 100, fillm: 'pig', Tfill: 17, Tg: 17, expo: 0, Qp: 150, tscale: 120, target: 0,
  breaks: [], sec: 0, run: true, hidden: { leak: null, gouge: null }, results: {}, log: [], flags: {}, csv: '', sim: null };

/* ------------------------------------------------------------------ sections and derived design values */
const sections = () => { const b = [0, ...state.breaks.slice().sort((a, c) => a - c), L]; return b.slice(0, -1).map((a, i) => [a, b[i + 1]]); };
function design() {
  const s = state, ratio = RATIO[s.cls], hoopPerBar = 0.1 * s.OD / (2 * s.wt); // MPa per bar
  return { ratio, TPmin: ratio * s.MAOP, Pdes: 2 * s.S * s.wt * FDES[s.cls] / s.OD * 10, hoopPerBar, pLim: s.lim / 100 * s.S / hoopPerBar, cEl: (s.OD * (1 - NU * NU) / (E_ST * 1e6 * s.wt) + KAPPA) * 1e5 };
}
function newSim() {
  const s = state, [xa, xb] = sections()[clamp(s.sec, 0, sections().length - 1)], r = rng(97 + Math.round(xa)), d = design();
  const xs = [], zs = [], V = [], sy = [], ep = [], A = Math.PI / 4 * Math.pow((s.OD - 2 * s.wt) / 1000, 2), dx = (xb - xa) / M;
  for (let j = 0; j < M; j++) { const x = xa + (j + 0.5) * dx; xs.push(x); zs.push(zAt(x)); V.push(A * dx); sy.push(s.S * clamp(1.12 + 0.05 * gauss(r), 1.02, 1.30)); ep.push(0); }
  const z0 = zAt(xa), zmax = Math.max(...zs), zmin = Math.min(...zs), Vtot = V.reduce((a, b) => a + b, 0);
  const headTP = d.TPmin + RHOG * (zmax - z0);
  const leak = s.hidden.leak && s.hidden.leak.x > xa && s.hidden.leak.x < xb ? s.hidden.leak.rate : 0;
  let gj = -1, gP = Infinity; if (s.hidden.gouge && s.hidden.gouge.x > xa && s.hidden.gouge.x < xb) { gj = Math.floor((s.hidden.gouge.x - xa) / dx); gP = s.hidden.gouge.frac * (headTP + RHOG * (z0 - zs[gj])); }
  return { xa, xb, xs, zs, V, sy, ep, z0, zmax, zmin, Vtot, headTP, leak, gj, gP, a: 0, fill: 0, phase: 'empty', P: 0, Tw: s.Tg, t: 0, tod0: 7, Vp: 0, Vleak: 0, strokes: 0, pumpOn: false, bleedOn: false,
    pv: [], tl: [], lastPV: -1, lastTL: -1e9, hold80: 0, holdStart: null, holdOK: 0, holdP0: null, holdT0: null, kT: 0, pHighMin: Infinity, lowMax: 0, P2x: null, P02: null, decision: null, burstAt: null, maxHead: 0, dips: 0 };
}
const sim = () => state.sim;
function pAt(sm, j) { return sm.P + RHOG * (sm.z0 - sm.zs[j]); }
function compliance(sm, d, rising) { // m³ per bar
  let C = sm.Vtot * d.cEl;
  for (let j = 0; j < M; j++) { const pa = Math.max(pAt(sm, j), 0) + PATM; C += sm.a * sm.V[j] * PATM / (pa * pa); if (rising) { const sig = pAt(sm, j) * d.hoopPerBar; if (sig >= sm.sy[j] + ET * sm.ep[j] - 0.01) C += sm.V[j] * 2 * d.hoopPerBar / ET; } }
  return C;
}

/* ------------------------------------------------------------------ physics step */
function envT(sm) { const h = (sm.tod0 + sm.t / 3600) % 24; return state.Tg + state.expo / 100 * 14 * Math.sin(2 * Math.PI * (h - 9) / 24); }
function step(dtSim) {
  const sm = sim(), s = state; if (!sm) return; const d = design();
  const n = clamp(Math.ceil(dtSim / 2), 1, 600), dt = dtSim / n;
  for (let k = 0; k < n; k++) {
    if (sm.phase === 'burst') { sm.t += dt; continue; }
    sm.t += dt;
    const Tenv = envT(sm), tau = (s.expo > 0 ? 6 : 10) * 3600, dT = (Tenv - sm.Tw) * dt / tau; sm.Tw += dT;
    if (sm.phase === 'filling') { sm.fill = Math.min(1, sm.fill + QFILL * dt / sm.Vtot); if (sm.fill >= 1) { sm.phase = 'filled'; sm.P = RHOG * (sm.zmax - sm.z0); sm.Vp = 0; sm.strokes = 0; s.flags.filled = true; } continue; }
    if (sm.phase === 'empty') continue;
    let dV = sm.Vtot * (betaW(sm.Tw) - 2 * ALPHA) * dT;
    if (sm.pumpOn) { const q = s.Qp / 60000 * dt; dV += q; sm.Vp += q; sm.strokes += q / STROKE; }
    if (sm.leak > 0) { const q = sm.leak / 3600 * Math.sqrt(Math.max(sm.P, 0) / 77) * dt; dV -= q; sm.Vleak += q; }
    if (sm.bleedOn) dV -= Math.max(s.Qp, 300) / 60000 * 2 * dt;
    const C = compliance(sm, d, dV > 0); let dP = dV / C;
    sm.P += dP;
    const pStatic = RHOG * (sm.zmax - sm.z0);
    if (sm.bleedOn && sm.P <= pStatic) { sm.P = pStatic; sm.bleedOn = false; sm.phase = 'filled'; }
    if (sm.P < pStatic - 0.01 && !sm.pumpOn) sm.P = Math.max(sm.P, pStatic * 0.2); // line drains into a partial vacuum at the high point: crude floor
    for (let j = 0; j < M; j++) {
      const sig = pAt(sm, j) * d.hoopPerBar;
      if (sig > sm.sy[j] + ET * sm.ep[j]) sm.ep[j] = (sig - sm.sy[j]) / ET;
      if (sig >= 1.25 * sm.sy[j] || (j === sm.gj && pAt(sm, j) >= sm.gP)) { burst(sm, j); break; }
    }
    if (sm.phase === 'burst') continue;
    sm.maxHead = Math.max(sm.maxHead, sm.P); sm.lowMax = Math.max(sm.lowMax, (sm.P + RHOG * (sm.z0 - sm.zmin)) * d.hoopPerBar / s.S * 100);
    if (sm.pumpOn && sm.P >= s.target) { sm.pumpOn = false; if (s.target >= sm.headTP - 0.01) { if (sm.holdStart === null) { sm.holdStart = sm.t; sm.holdP0 = sm.P; sm.holdT0 = sm.Tw; sm.kT = sm.Vtot * (betaW(sm.Tw) - 2 * ALPHA) / compliance(sm, d, false); } sm.phase = 'hold'; } else sm.phase = 'hold80'; }
    if (sm.pumpOn && sm.P - sm.lastPV >= 0.5) { sm.pv.push([sm.Vp, sm.P]); sm.lastPV = sm.P; detectYield(sm); }
    const pHigh = sm.P - RHOG * (sm.zmax - sm.z0);
    if (sm.phase === 'hold80' && sm.P >= 0.7 * sm.headTP) { sm.hold80 += dt; if (sm.hold80 >= 1800) s.flags.hold80 = true; }
    if (sm.holdStart !== null && sm.phase === 'hold') { if (pHigh >= d.TPmin - 1e-6) { sm.holdOK += dt; sm.pHighMin = Math.min(sm.pHighMin, pHigh); } else if (sm.holdOK > 0) { sm.holdOK = 0; sm.dips++; } }
    if (sm.holdOK >= 7200) s.flags.hold2h = true;
    if (sm.t - sm.lastTL >= 120 && sm.phase !== 'filling') { sm.tl.push([sm.t, sm.P, sm.Tw]); sm.lastTL = sm.t; if (sm.tl.length > 4000) sm.tl.splice(0, 1000); }
  }
}
function burst(sm, j) { sm.phase = 'burst'; sm.burstAt = sm.xs[j]; sm.P = 0; sm.pumpOn = false; sm.bleedOn = false; state.flags.burst = true; state.flags.burstGouge = j === sm.gj; }
function detectYield(sm) {
  const pts = sm.pv.filter(([, p]) => p >= 0.35 * sm.headTP && p <= 0.75 * sm.headTP); if (pts.length < 4) return;
  const n = pts.length, mp = pts.reduce((a, q) => a + q[1], 0) / n, mv = pts.reduce((a, q) => a + q[0], 0) / n;
  const k0 = pts.reduce((a, q) => a + (q[1] - mp) * (q[0] - mv), 0) / pts.reduce((a, q) => a + (q[1] - mp) ** 2, 0); sm.fit = { k0, mp, mv };
  const L2 = sm.pv.length; if (L2 - (sm.pvSeg || 0) < 3) return; const [v1, p1] = sm.pv[L2 - 1], [v0, p0] = sm.pv[L2 - 3];
  if (p1 > 0.75 * sm.headTP && sm.P2x === null && (v1 - v0) / (p1 - p0) >= 2 * k0) { sm.P2x = p1; state.flags.yield = true; }
  if (p1 > 0.75 * sm.headTP && sm.P02 === null && v1 - (mv + k0 * (p1 - mp)) >= 0.002 * sm.Vtot) { sm.P02 = p1; state.flags.yield = true; }
}

/* ------------------------------------------------------------------ actions */
function fill() { const sm = sim(); if (sm.phase !== 'empty') return; sm.phase = 'filling'; sm.a = AIR[state.fillm]; sm.Tw = state.Tfill; sm.fill = 0; state.run = true; }
function pumpTo(target) { const sm = sim(); if (!['filled', 'hold80', 'hold'].includes(sm.phase)) return; setCtl('target', +target.toFixed(1)); sm.bleedOn = false; sm.pumpOn = sm.P < state.target; if (sm.pumpOn) { sm.pvSeg = sm.pv.length; sm.lastPV = sm.P - 0.5; } /* each pumping run starts a new P–V segment */ if (sm.pumpOn && sm.phase === 'filled') sm.phase = 'pumping'; state.run = true; }
function stopPump() { const sm = sim(); sm.pumpOn = false; if (sm.phase === 'pumping') sm.phase = 'hold80'; }
function bleed() { const sm = sim(); if (['hold', 'hold80', 'pumping'].includes(sm.phase)) { sm.pumpOn = false; sm.bleedOn = true; sm.holdStart = null; sm.holdOK = 0; sm.pHighMin = Infinity; } }
function decide(reject) {
  const sm = sim(), d = design(); if (sm.holdStart === null) return;
  const leak = sm.leak > 0, ok = sm.holdOK >= 7200 && sm.pHighMin >= d.TPmin - 1e-6 && (sm.lowMax <= state.lim || sm.P2x === null);
  const correct = reject ? leak : (!leak && ok);
  sm.decision = reject ? 'reject' : 'accept';
  const maop = !reject && ok ? Math.min(d.Pdes, sm.pHighMin / d.ratio) : null;
  state.results[`${sm.xa}-${sm.xb}`] = { decision: sm.decision, correct, maop, leak, ok };
  if (reject && leak) state.flags.leakFound = true; if (!reject && !leak && ok && Math.abs(sm.Tw - sm.holdT0) >= 1) state.flags.tempAccepted = true;
  if (!reject && leak) state.flags.acceptedLeak = true; if (reject && !leak) state.flags.rejectedGood = true;
  if (!reject && ok && state.flags.burstGouge && state.flags.repaired) state.flags.retested = true;
}
function resetSec() { if (sim() && sim().phase === 'burst' && state.hidden.gouge) { state.hidden.gouge = null; state.flags.repaired = true; } state.sim = newSim(); }

/* ------------------------------------------------------------------ profile (SVG) */
const VW = 1000, VH = 330, X0 = 60, XW = 920;
let ZLO = 1060, ZHI = 1540;
const xs = x => X0 + x / L * XW, ys = z => 30 + (ZHI - z) / (ZHI - ZLO) * 230;
const hoopCol = p => p < 72 ? 'var(--water)' : p < 90 ? 'var(--good)' : p <= 100 ? 'var(--brass)' : 'var(--alarm)';
function drawRig() {
  const s = state, sm = sim(), d = design(), P = [], secs = sections();
  P.push(`<svg viewBox="0 0 ${VW} ${VH}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Elevation profile with test sections">`);
  const pts = []; for (let x = 0; x <= L; x += 100) pts.push(`${xs(x).toFixed(1)},${ys(zAt(x) - 4).toFixed(1)}`);
  P.push(`<polygon points="${xs(0)},262 ${pts.join(' ')} ${xs(L)},262" style="fill:var(--panel2);stroke:none"/>`);
  for (let z = 1100; z <= 1500; z += 100) P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="${ys(z)}" y2="${ys(z)}" style="stroke:var(--grid)"/><text x="${X0 - 6}" y="${ys(z) + 4}" text-anchor="end" style="fill:var(--muted);font:10px var(--mono)">${z} m</text>`);
  P.push(`<polyline points="${pts.join(' ')}" style="fill:none;stroke:var(--steel);stroke-width:3;opacity:.55"/>`);
  if (sm) {
    const active = sm.phase !== 'empty';
    for (let j = 0; j < M; j++) {
      const x0 = sm.xa + j * (sm.xb - sm.xa) / M, x1 = x0 + (sm.xb - sm.xa) / M;
      let col = 'var(--ink)'; if (sm.phase === 'filling') col = j / M < sm.fill ? 'var(--water)' : 'var(--ink)'; else if (active && sm.phase !== 'burst') col = hoopCol(pAt(sm, j) * d.hoopPerBar / s.S * 100);
      P.push(`<line x1="${xs(x0)}" x2="${xs(x1)}" y1="${ys(zAt(x0) - 4)}" y2="${ys(zAt(x1) - 4)}" style="stroke:${col};stroke-width:6;stroke-linecap:round"/>`);
    }
    const pk = sm.P * d.hoopPerBar / s.S * 100 + RHOG * (sm.z0 - sm.zmin) * d.hoopPerBar / s.S * 100;
    if (active && pk > 50 && sm.phase !== 'burst') P.push(`<rect x="${xs(sm.xa)}" y="26" width="${xs(sm.xb) - xs(sm.xa)}" height="238" style="fill:var(--alarm);opacity:.07"/><text x="${(xs(sm.xa) + xs(sm.xb)) / 2}" y="256" text-anchor="middle" style="fill:var(--alarm);font:600 10px var(--mono)">EXCLUSION ZONE · above 50 % SMYS</text>`);
    const jh = sm.zs.indexOf(sm.zmax), jl = sm.zs.indexOf(sm.zmin);
    P.push(`<circle cx="${xs(sm.xs[jh])}" cy="${ys(sm.zmax) - 10}" r="4" style="fill:var(--good)"/><text x="${xs(sm.xs[jh])}" y="${ys(sm.zmax) - 18}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">high ${fmt(sm.zmax, 0)} m${active ? ' · ' + fmt(sm.P - RHOG * (sm.zmax - sm.z0), 1) + ' bar' : ''}</text>`);
    P.push(`<circle cx="${xs(sm.xs[jl])}" cy="${ys(sm.zmin) + 4}" r="4" style="fill:var(--alarm)"/><text x="${xs(sm.xs[jl])}" y="${ys(sm.zmin) + 20}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">low ${fmt(sm.zmin, 0)} m${active ? ' · ' + fmt((sm.P + RHOG * (sm.z0 - sm.zmin)) * d.hoopPerBar / s.S * 100, 0) + ' % SMYS' : ''}</text>`);
    const hx = xs(sm.xa) + 2, hy = ys(sm.z0 - 4);
    P.push(`<rect x="${hx}" y="${hy - 30}" width="40" height="18" rx="3" style="fill:var(--panel);stroke:var(--ink)"/><text x="${hx + 20}" y="${hy - 17}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">${fmt(sm.P, 1)}</text><line x1="${hx + 6}" x2="${hx + 6}" y1="${hy - 12}" y2="${hy}" style="stroke:var(--ink)"/>`);
    if (sm.burstAt !== null) P.push(`<text x="${xs(sm.burstAt)}" y="${ys(zAt(sm.burstAt)) - 6}" text-anchor="middle" style="fill:var(--alarm);font:700 20px var(--mono)">✸</text><text x="${xs(sm.burstAt)}" y="${ys(zAt(sm.burstAt)) - 26}" text-anchor="middle" style="fill:var(--alarm);font:700 11px var(--mono)">RUPTURE</text>`);
  }
  secs.forEach(([a, b], i) => { const zz = []; for (let x = a; x <= b; x += 50) zz.push(zAt(x)); const dz = Math.max(...zz) - Math.min(...zz); P.push(`<text x="${(xs(a) + xs(b)) / 2}" y="285" text-anchor="middle" style="fill:${s.sec === i ? 'var(--ink)' : 'var(--muted)'};font:${s.sec === i ? 700 : 400} 10px var(--mono)">S${i + 1} · Δz ${fmt(dz, 0)} m${state.results[`${a}-${b}`] ? (state.results[`${a}-${b}`].decision === 'accept' ? ' ✓' : ' ✗') : ''}</text>`); });
  for (const b of s.breaks) P.push(`<line x1="${xs(b)}" x2="${xs(b)}" y1="26" y2="264" style="stroke:var(--ink);stroke-dasharray:4 4"/><text x="${xs(b)}" y="272" text-anchor="middle" style="fill:var(--ink);font:11px var(--mono)">✂</text>`);
  for (let k = 0; k <= 24; k += 2) P.push(`<text x="${xs(k * 1000)}" y="302" text-anchor="middle" style="fill:var(--muted);font:10px var(--mono)">${k}</text>`);
  P.push(`<text x="${X0 + XW}" y="318" text-anchor="end" style="fill:var(--muted);font:10px var(--mono)">route km · test head (gauge, bar) at the start of the section</text></svg>`);
  $('#rig').innerHTML = P.join('');
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function axes(ctx, W, H, pad, x0, x1, y0, y1, xl, yl, nx = 5, ny = 4, dx = 1, dy = 0) {
  ctx.fillStyle = css('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.strokeStyle = css('--grid');
  for (let k = 0; k <= nx; k++) { const x = pad.l + k / nx * (W - pad.l - pad.r); ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, H - pad.b); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText((x0 + k / nx * (x1 - x0)).toFixed(dx), x, H - pad.b + 14); }
  for (let k = 0; k <= ny; k++) { const y = H - pad.b - k / ny * (H - pad.t - pad.b); ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText((y0 + k / ny * (y1 - y0)).toFixed(dy), pad.l - 6, y + 4); }
  ctx.textAlign = 'center'; ctx.fillText(xl, (pad.l + W - pad.r) / 2, H - 4); ctx.save(); ctx.translate(12, (pad.t + H - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yl, 0, 0); ctx.restore();
}
function drawPV() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, sm = sim();
  const vmax = Math.max(1, ...sm.pv.map(q => q[0])) * 1.1, pmax = Math.max(sm.headTP * 1.15, ...sm.pv.map(q => q[1]));
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, 0, vmax, 0, pmax, 'water pumped since full, m³', 'gauge at head, bar', 5, 4, vmax < 10 ? 1 : 0);
  const X = v => pad.l + v / vmax * (W - pad.l - pad.r), Y = p => H - pad.b - p / pmax * (H - pad.t - pad.b);
  ctx.strokeStyle = css('--alarm'); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(pad.l, Y(sm.headTP)); ctx.lineTo(W - pad.r, Y(sm.headTP)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--alarm'); ctx.textAlign = 'right'; ctx.fillText(`test pressure at head ${fmt(sm.headTP, 1)} bar`, W - pad.r - 4, Y(sm.headTP) - 4);
  if (sm.fit) { const { k0, mp, mv } = sm.fit; ctx.strokeStyle = css('--steel'); ctx.lineWidth = 1.5; ctx.beginPath(); const p0 = 0, p1 = pmax; ctx.moveTo(X(mv + k0 * (p0 - mp)), Y(p0)); ctx.lineTo(X(mv + k0 * (p1 - mp)), Y(p1)); ctx.stroke(); ctx.fillStyle = css('--steel'); ctx.textAlign = 'left'; ctx.fillText(`straight line ${fmt(k0 * 1000, 1)} L/bar · ${fmt(k0 / STROKE, 1)} strokes/bar`, pad.l + 8, pad.t + 14); }
  ctx.fillStyle = css('--water'); for (const [v, p] of sm.pv) { ctx.beginPath(); ctx.arc(X(v), Y(p), 2.5, 0, 7); ctx.fill(); }
  ctx.fillStyle = css('--alarm'); ctx.textAlign = 'left';
  if (sm.P2x !== null) { ctx.fillText(`2 × strokes at ${fmt(sm.P2x, 1)} bar`, pad.l + 8, pad.t + 30); }
  if (sm.P02 !== null) { ctx.fillText(`0.2 % fill volume at ${fmt(sm.P02, 1)} bar`, pad.l + 8, pad.t + 46); }
}
function drawLog() {
  const cv = $('#schart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 50, t: 14, b: 36 }, sm = sim(), d = design();
  frame(ctx, W, H, pad); const tl = sm.tl; const t1 = Math.max(3600, sm.t) / 3600;
  const ps = tl.map(q => q[1]); const pmin = Math.max(0, Math.min(sm.headTP - 6, ...ps)), pmax = Math.max(sm.headTP + 6, ...ps);
  axes(ctx, W, H, pad, 0, t1, pmin, pmax, 'time since start, h', 'gauge at head, bar', 6, 4, 1, 0);
  const X = t => pad.l + t / 3600 / t1 * (W - pad.l - pad.r), Y = p => H - pad.b - (p - pmin) / (pmax - pmin) * (H - pad.t - pad.b);
  const Tmin = Math.min(...tl.map(q => q[2]), state.Tg) - 1, Tmax = Math.max(...tl.map(q => q[2]), state.Tg) + 1, YT = T => H - pad.b - (T - Tmin) / (Tmax - Tmin) * (H - pad.t - pad.b);
  const need = d.TPmin + RHOG * (sm.zmax - sm.z0); ctx.strokeStyle = css('--alarm'); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(pad.l, Y(need)); ctx.lineTo(W - pad.r, Y(need)); ctx.stroke(); ctx.setLineDash([]);
  const line = (f, c, w = 2) => { ctx.strokeStyle = css(c); ctx.lineWidth = w; ctx.beginPath(); tl.forEach((q, i) => { const y = f(q); i ? ctx.lineTo(X(q[0]), y) : ctx.moveTo(X(q[0]), y); }); ctx.stroke(); };
  if (tl.length > 1) { line(q => YT(q[2]), '--brass', 1.5); line(q => Y(q[1]), '--water'); if (sm.holdStart !== null) { ctx.strokeStyle = css('--good'); ctx.lineWidth = 2; ctx.setLineDash([6, 3]); ctx.beginPath(); let f = true; for (const q of tl) { if (q[0] < sm.holdStart) continue; const pc = q[1] - sm.kT * (q[2] - sm.holdT0); f ? ctx.moveTo(X(q[0]), Y(pc)) : ctx.lineTo(X(q[0]), Y(pc)); f = false; } ctx.stroke(); ctx.setLineDash([]); } }
  ctx.fillStyle = css('--brass'); ctx.textAlign = 'left'; ctx.fillText(`${fmt(Tmax, 0)} °C`, W - pad.r + 4, pad.t + 8); ctx.fillText(`${fmt(Tmin, 0)} °C`, W - pad.r + 4, H - pad.b);
}
function drawAlong() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, sm = sim(), d = design();
  const ptest = j => sm.headTP + RHOG * (sm.z0 - sm.zs[j]), pmax = Math.max(d.pLim * 1.08, ptest(sm.zs.indexOf(sm.zmin)) * 1.05);
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, sm.xa / 1000, sm.xb / 1000, 0, pmax, 'section km', 'pressure, bar', 5, 4, 1, 0);
  const X = j => pad.l + (j + 0.5) / M * (W - pad.l - pad.r), Y = p => H - pad.b - p / pmax * (H - pad.t - pad.b);
  const hl = (p, c, lb) => { ctx.strokeStyle = css(c); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(pad.l, Y(p)); ctx.lineTo(W - pad.r, Y(p)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css(c); ctx.textAlign = 'right'; ctx.fillText(lb, W - pad.r - 4, Y(p) - 4); };
  hl(d.TPmin, '--good', `minimum test pressure ${fmt(d.TPmin, 1)} bar`); hl(d.pLim, '--alarm', `${state.lim} % SMYS ${fmt(d.pLim, 1)} bar`);
  const zr = Math.max(1, sm.zmax - sm.zmin); ctx.strokeStyle = css('--brass'); ctx.lineWidth = 1; ctx.beginPath(); for (let j = 0; j < M; j++) { const y = H - pad.b - 10 - (sm.zs[j] - sm.zmin) / zr * 40; j ? ctx.lineTo(X(j), y) : ctx.moveTo(X(j), y); } ctx.stroke();
  ctx.strokeStyle = css('--steel'); ctx.setLineDash([2, 3]); ctx.beginPath(); for (let j = 0; j < M; j++) { j ? ctx.lineTo(X(j), Y(ptest(j))) : ctx.moveTo(X(j), Y(ptest(j))); } ctx.stroke(); ctx.setLineDash([]);
  if (!['empty', 'filling'].includes(sm.phase)) { ctx.strokeStyle = css('--water'); ctx.lineWidth = 2.5; ctx.beginPath(); for (let j = 0; j < M; j++) { j ? ctx.lineTo(X(j), Y(pAt(sm, j))) : ctx.moveTo(X(j), Y(pAt(sm, j))); } ctx.stroke(); }
  ctx.fillStyle = css('--steel'); ctx.textAlign = 'left'; ctx.fillText('dotted: pressure everywhere when the high point is at the minimum', pad.l + 6, pad.t + 12);
}

/* ------------------------------------------------------------------ readouts */
const PH = { empty: 'Empty', filling: 'Filling', filled: 'Full, at static head', pumping: 'Pressurising', hold80: 'Holding below test', hold: 'Strength hold', burst: 'Ruptured' };
function updateReadouts() {
  const s = state, sm = sim(), d = design(), pHigh = sm.P - RHOG * (sm.zmax - sm.z0), lowPct = (sm.P + RHOG * (sm.z0 - sm.zmin)) * d.hoopPerBar / s.S * 100, live = !['empty', 'filling', 'burst'].includes(sm.phase);
  $('#roGauge').textContent = live ? fmt(sm.P, 1) + ' bar' : sm.phase === 'filling' ? fmt(sm.fill * 100, 0) + ' % full' : '—';
  $('#roGaugeNote').textContent = `test head at ${fmt(sm.xa / 1000, 1)} km, ${fmt(sm.z0, 0)} m · test needs ${fmt(sm.headTP, 1)} bar here${sm.pumpOn ? ' · pump running' : ''}`;
  $('#roHigh').textContent = live ? fmt(pHigh, 1) + ' bar' : '—'; $('#roHighNote').textContent = `minimum ${fmt(d.TPmin, 1)} bar = ${d.ratio} × MAOP ${s.MAOP} bar · ${fmt(sm.zmax, 0)} m`;
  $('#roHighBox').classList.toggle('alarm', live && sm.phase === 'hold' && pHigh < d.TPmin);
  $('#roLow').textContent = live ? fmt(lowPct, 1) + ' %' : '—'; $('#roLowNote').textContent = `SMYS at ${fmt(sm.zmin, 0)} m · limit ${s.lim} % · highest so far ${fmt(sm.lowMax, 1)} %`;
  $('#roLowBox').classList.toggle('alarm', live && lowPct > s.lim);
  $('#roVol').textContent = fmt(sm.Vp, 2) + ' m³'; $('#roVolNote').textContent = `${fmt(sm.strokes, 0)} strokes of ${STROKE * 1000} L · fill volume ${fmt(sm.Vtot, 0)} m³`;
  $('#roTemp').textContent = fmt(sm.Tw, 1) + ' °C'; $('#roTempNote').textContent = `ground ${s.Tg} °C · ${fmt(betaW(sm.Tw) * 1e6, 0)} ×10⁻⁶/K expansion · ${String(Math.floor((sm.tod0 + sm.t / 3600) % 24)).padStart(2, '0')}:${String(Math.floor(((sm.tod0 * 60 + sm.t / 60) % 60))).padStart(2, '0')} day ${1 + Math.floor((sm.tod0 + sm.t / 3600) / 24)}`;
  $('#roHold').textContent = sm.holdStart === null ? '—' : fmt(sm.holdOK / 3600, 2) + ' h'; $('#roHoldNote').textContent = sm.holdStart === null ? 'starts when the head reaches the test pressure' : `continuous at or above the minimum · needs 2.00 h${sm.dips ? ' · fell below ' + sm.dips + '×, clock restarted' : ''}`;
  $('#roHoldBox').classList.toggle('alarm', sm.dips > 0 && sm.holdOK < 7200);
  if (sm.holdStart !== null) { const dP = sm.P - sm.holdP0, dT = sm.Tw - sm.holdT0, dPT = sm.kT * dT; $('#roDrift').textContent = `${dP >= 0 ? '+' : ''}${fmt(dP, 2)} bar · temperature ${dPT >= 0 ? '+' : ''}${fmt(dPT, 2)} · unexplained ${fmt(dP - dPT, 2)}`; $('#roDriftNote').textContent = `ΔT ${fmt(dT, 2)} °C × ${fmt(sm.kT, 2)} bar/°C since the hold started${sm.pumpOn ? ' · top-up in progress' : ''}`; }
  else { $('#roDrift').textContent = '—'; $('#roDriftNote').textContent = 'measured from the start of the strength hold'; }
  const res = s.results[`${sm.xa}-${sm.xb}`];
  $('#roVerdict').textContent = sm.phase === 'burst' ? 'Ruptured during test' : res ? (res.decision === 'accept' ? 'Accepted' : 'Rejected: leak') + (res.correct ? ' · correct' : ' · WRONG') : 'Not yet judged';
  $('#roVerdictNote').textContent = res && res.maop ? `supports MAOP ${fmt(res.maop, 1)} bar (lesser of design ${fmt(d.Pdes, 1)} and high-point test ÷ ${d.ratio})` : res && !res.correct ? (res.leak ? 'the section had a leak' : res.ok ? 'there was no leak: the change was temperature' : 'the hold did not meet the minimum for 2 h') : `design pressure ${fmt(d.Pdes, 1)} bar · MAOP ${s.MAOP} bar${s.MAOP > d.Pdes ? ' is ABOVE design' : ''}`;
  $('#roVerdictBox').classList.toggle('alarm', !!res && !res.correct);
  $('#pillPhase').textContent = PH[sm.phase]; $('#pillSec').textContent = `Section ${s.sec + 1} of ${sections().length} · ${fmt(sm.xa / 1000, 1)}–${fmt(sm.xb / 1000, 1)} km`;
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('run', s.run);
  $('#pillPeople').classList.toggle('hidden', !(live && lowPct > 50)); $('#pillYield').classList.toggle('hidden', !(sm.P2x !== null || sm.P02 !== null)); $('#pillBurst').classList.toggle('hidden', sm.phase !== 'burst'); $('#pillHold').classList.toggle('hidden', sm.holdOK < 7200);
}

/* ------------------------------------------------------------------ log and tasks */
function record(tag = '') {
  const s = state, sm = sim(), d = design(), res = s.results[`${sm.xa}-${sm.xb}`];
  const dz = sm.zmax - sm.zmin, dP = sm.holdStart !== null ? sm.P - sm.holdP0 : null, dT = sm.holdStart !== null ? sm.Tw - sm.holdT0 : null;
  const row = { i: s.log.length + 1, tag, sec: `${r3(sm.xa / 1000)}–${r3(sm.xb / 1000)}`, dz: r3(dz), tp: r3(d.TPmin), hmax: r3(sm.maxHead), hmin: Number.isFinite(sm.pHighMin) ? r3(sm.pHighMin) : '', low: r3(sm.lowMax), hold: r3(sm.holdOK / 3600), dP: dP === null ? '' : r3(dP), dT: dT === null ? '' : r3(dT), dPT: dT === null ? '' : r3(sm.kT * dT), air: r3(sm.a * 100), p2x: sm.P2x === null ? '' : r3(sm.P2x), p02: sm.P02 === null ? '' : r3(sm.P02), dec: res ? res.decision : '', ok: res ? (res.correct ? 'yes' : 'no') : '', maop: res && res.maop ? r3(res.maop) : '', phase: sm.phase, fillm: s.fillm, nsec: sections().length };
  s.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  const K = ['i', 'tag', 'sec', 'dz', 'tp', 'hmax', 'hmin', 'low', 'hold', 'dP', 'dT', 'dPT', 'air', 'p2x', 'p02', 'dec', 'ok', 'maop', 'phase'];
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${K.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,section_km,elevation_range_m,TP_min_bar,head_max_bar,high_point_min_in_hold_bar,low_point_max_pct_SMYS,hold_h,dP_hold_bar,dT_hold_C,dP_temperature_bar,air_pct,double_stroke_bar,offset_0_2pct_bar,decision,correct,MAOP_supported_bar,phase\n' + state.log.map(r => K.map(k => r[k]).join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
function sectionsOK() { const d = design(); return sections().every(([a, b]) => { let zmax = -1e9, zmin = 1e9; for (let x = a; x <= b; x += 25) { const z = zAt(x); zmax = Math.max(zmax, z); zmin = Math.min(zmin, z); } return d.TPmin + RHOG * (zmax - zmin) <= d.pLim; }); }
const TASKS = [
  { id: 'fill', t: 'Fill a section', d: 'Fill the selected section behind a pig. Read the gauge at the test head when it is full: that is the static head between the head and the vented high point.', ok: () => state.flags.filled },
  { id: 'sections', t: 'Plan the test sections', d: 'With one section the low point would pass the hoop limit before the high point reaches the minimum. Click the profile to add breaks until every section passes (the lower chart helps). Record it.', ok: () => state.log.some(r => r.nsec > 1) && state.flags.sectionsOK },
  { id: 'hold80', t: 'The 80 % hold (N4)', d: 'Pressurise to 80 % of the test pressure and hold for at least 30 minutes, looking for big leaks.', ok: () => state.flags.hold80 },
  { id: 'pv', t: 'The straight line', d: 'Pressurise to test pressure and read the straight-line slope of the P–V plot in litres and strokes per bar. Record it.', ok: () => state.log.some(r => r.phase === 'hold') },
  { id: 'hold2h', t: 'Two hours (841.321)', d: 'Hold for 2 hours with the high point at or above the minimum test pressure. Top up if it falls.', ok: () => state.flags.hold2h },
  { id: 'air', t: 'Air in the line', d: 'Fill a section without a pig (no venting) and compare the strokes per bar with a pigged fill. Record both.', ok: () => state.log.some(r => r.air >= 1) && state.log.some(r => r.air < 0.1 && r.phase !== 'empty') },
  { id: 'yield', t: 'Find yield (N5)', d: 'Test the whole line as one section with the hoop limit at 110 %. Watch the P–V plot leave the straight line and stop when the 2 × strokes or 0.2 % volume rule is reached.', ok: () => state.flags.yield },
  { id: 'leak', t: 'Find the leak', d: 'Judge scenarios A and B. One loses water; the other only cools. Correct the pressure change for temperature before you decide.', ok: () => state.flags.leakFound },
  { id: 'temp', t: 'Not a leak', d: 'Accept a good section whose pressure changed by more than a bar because the water temperature changed.', ok: () => state.flags.tempAccepted },
  { id: 'sun', t: 'Sun on the pipe', d: 'Hold a section with exposed pipe through an afternoon and a night. Watch the pressure follow the sun.', ok: () => state.log.some(r => r.hold >= 6) },
  { id: 'gouge', t: 'The test did its job', d: 'Load "Construction damage", pressurise to test and find where it fails. Drain, repair, retest and accept the section.', ok: () => state.flags.retested },
  { id: 'maop', t: 'Every section passed', d: 'Accept every section of the line correctly and record the MAOP each test supports.', ok: () => sections().length > 1 && sections().every(([a, b]) => { const r = state.results[`${a}-${b}`]; return r && r.decision === 'accept' && r.correct; }) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { if (sectionsOK() && sections().length > 1) state.flags.sectionsOK = true; for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ wiring */
let last = performance.now(), frameN = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; if (state.run) step(dt * state.tscale); frameN++; if (frameN % 2 === 0) { drawRig(); updateReadouts(); if (frameN % 4 === 0) { drawPV(); drawLog(); drawAlong(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function fillSecSelect() { const sel = $('#sec'); sel.innerHTML = sections().map(([a, b], i) => `<option value="${i}">S${i + 1} · ${fmt(a / 1000, 1)}–${fmt(b / 1000, 1)} km</option>`).join(''); state.sec = clamp(state.sec, 0, sections().length - 1); sel.value = state.sec; }
function setBreaks(b) { state.breaks = b.slice().sort((a, c) => a - c); fillSecSelect(); state.sim = newSim(); }
function setSec(i) { state.sec = clamp(i, 0, sections().length - 1); $('#sec').value = state.sec; state.sim = newSim(); }
function reset(over = {}) {
  state.hidden = { leak: null, gouge: null }; state.results = {};
  const v = Object.assign({ OD: 406.4, wt: 5.6, grade: 359, MAOP: 70, cls: '1d2', lim: 100, fillm: 'pig', Tfill: 17, Tg: 17, expo: 0, Qp: 150, tscale: 120 }, over);
  for (const [k, x] of Object.entries(v)) setCtl(k, x);
  state.run = true; setBreaks([]);
}
const SPLIT = [6000, 10000, 13500];
const PRESETS = {
  base: () => reset(),
  split: () => { reset(); setBreaks(SPLIT); },
  air: () => { reset({ fillm: 'none' }); setBreaks(SPLIT); setSec(1); },
  cold: () => { reset({ Tfill: 26, Tg: 12 }); setBreaks(SPLIT); setSec(1); },
  sun: () => { reset({ expo: 25 }); setBreaks(SPLIT); setSec(1); },
  leakA: () => { reset(); state.hidden.leak = { x: 8200, rate: 0.08 }; setBreaks(SPLIT); setSec(1); },
  leakB: () => { reset({ Tfill: 23, Tg: 17 }); setBreaks(SPLIT); setSec(1); },
  yield: () => { reset({ lim: 110 }); },
  gouge: () => { reset(); state.hidden.gouge = { x: 11700, frac: 0.94 }; setBreaks(SPLIT); setSec(2); },
  class3: () => { reset({ wt: 8.7, cls: '3' }); setBreaks(SPLIT); setSec(0); },
};
function bind() {
  const bindR = (id, f, key = id, onChange) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[key] = +e.value; if (o) o.textContent = f(state[key]); if (onChange) onChange(); }; e.addEventListener('input', set); set(); };
  const renew = () => { if (state.sim && state.sim.phase === 'empty') state.sim = newSim(); };
  bindR('MAOP', v => v + ' bar', 'MAOP', renew); bindR('lim', v => v + ' % SMYS'); bindR('Tfill', v => v + ' °C'); bindR('Tg', v => v + ' °C'); bindR('expo', v => v + ' %'); bindR('Qp', v => v + ' L/min'); bindR('tscale', v => '×' + v); bindR('target', v => fmt(v, 1) + ' bar');
  for (const [id, key, num] of [['OD', 'OD', 1], ['wt', 'wt', 1], ['grade', 'S', 1], ['cls', 'cls', 0], ['fillm', 'fillm', 0]]) { const e = $('#' + id); const set = () => { state[key] = num ? +e.value : e.value; if (state.sim && (state.sim.phase === 'empty' || key !== 'fillm')) state.sim = newSim(); }; e.addEventListener('change', set); state[key] = num ? +e.value : e.value; }
  $('#sec').addEventListener('change', e => setSec(+e.target.value));
  $('#rig').addEventListener('click', e => { const r = $('#rig svg').getBoundingClientRect(), sx = (e.clientX - r.left) / r.width * VW, sy = (e.clientY - r.top) / r.height * VH; if (sx < X0 || sx > X0 + XW || sy < 20 || sy > 285) return; const x = Math.round((sx - X0) / XW * L / 100) * 100; const near = state.breaks.findIndex(b => Math.abs(b - x) < 400); if (near >= 0) setBreaks(state.breaks.filter((_, i) => i !== near)); else if (x > 1000 && x < L - 1000 && state.breaks.every(b => Math.abs(b - x) >= 1000)) setBreaks([...state.breaks, x]); });
  $('#btnFill').onclick = fill; $('#btn80').onclick = () => pumpTo(0.8 * sim().headTP); $('#btnTest').onclick = () => pumpTo(sim().headTP + 0.5); $('#btnPump').onclick = () => pumpTo(Math.max(state.target, sim().P + 0.5));
  $('#btnStop').onclick = stopPump; $('#btnBleed').onclick = bleed; $('#btnAccept').onclick = () => decide(false); $('#btnReject').onclick = () => decide(true); $('#btnReset').onclick = resetSec;
  $('#btnPause').onclick = () => state.run = !state.run; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { const done = () => { $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); }; try { navigator.clipboard.writeText(state.csv || '').then(done, () => { $('#btnCopy').textContent = 'Use Download CSV'; }); } catch (e) { $('#btnCopy').textContent = 'Use Download CSV'; } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tk_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'F') fill(); if (k === 'T') $('#btnTest').click(); if (k === 'S') stopPump(); if (k === 'R') record(''); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.PipelineHydrotest = {
  getState: () => { const s = state, sm = sim(), d = design(); return {
    inputs: { OD: s.OD, wt: s.wt, grade: s.S, MAOP: s.MAOP, cls: s.cls, lim: s.lim, fillm: s.fillm, Tfill: s.Tfill, Tg: s.Tg, expo: s.expo, Qp: s.Qp, tscale: s.tscale, target: s.target, breaks: s.breaks.slice(), sec: s.sec, run: s.run },
    results: { phase: sm.phase, P: r3(sm.P), pHigh: r3(sm.P - RHOG * (sm.zmax - sm.z0)), lowPctSMYS: r3((sm.P + RHOG * (sm.z0 - sm.zmin)) * d.hoopPerBar / s.S * 100), TPmin: r3(d.TPmin), headTP: r3(sm.headTP), Tw: r3(sm.Tw), holdH: r3(sm.holdOK / 3600), Vp: r3(sm.Vp), strokes: r3(sm.strokes), Vtot: r3(sm.Vtot), slopeLperBar: sm.fit ? r3(sm.fit.k0 * 1000) : null, P2x: sm.P2x === null ? null : r3(sm.P2x), P02: sm.P02 === null ? null : r3(sm.P02), kT: r3(sm.kT), dPhold: sm.holdStart === null ? null : r3(sm.P - sm.holdP0), dThold: sm.holdStart === null ? null : r3(sm.Tw - sm.holdT0), burstAtKm: sm.burstAt === null ? null : r3(sm.burstAt / 1000), sectionsOK: sectionsOK(), results: JSON.parse(JSON.stringify(s.results)), Pdes: r3(d.Pdes) },
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }; },
  setState: inp => {
    if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset]();
    for (const k of ['OD', 'wt', 'grade', 'MAOP', 'cls', 'lim', 'fillm', 'Tfill', 'Tg', 'expo', 'Qp', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]);
    if (Array.isArray(inp.breaks)) setBreaks(inp.breaks.map(Number).filter(x => x > 1000 && x < L - 1000));
    if (inp.sec !== undefined) setSec(+inp.sec);
    if (inp.fill) fill(); if (inp.to80) pumpTo(0.8 * sim().headTP); if (inp.toTest) pumpTo(sim().headTP + 0.5); if (inp.target !== undefined && inp.pump) pumpTo(+inp.target);
    if (inp.stop) stopPump(); if (inp.bleed) bleed(); if (inp.accept) decide(false); if (inp.reject) decide(true); if (inp.resetSection) resetSec(); if (inp.record) record(inp.record === true ? '' : String(inp.record));
    if (inp.advance) { let left = Math.min(+inp.advance, 7 * 86400); while (left > 0) { const h = Math.min(60, left); step(h); left -= h; } }
    if (inp.run !== undefined) state.run = !!inp.run;
  },
  meta: { id: 'pipeline-hydrotest', title: 'Pipeline Hydrostatic Test Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PipelineHydrotest.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'pipeline-hydrotest', state: window.PipelineHydrotest.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function boot() {
  try { const th = localStorage.getItem('tk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildTasks(); bind(); setBreaks([]); renderLog(); drawRig(); updateReadouts(); drawPV(); drawLog(); drawAlong(); requestAnimationFrame(tick);
}
boot();
