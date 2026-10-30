/* Pipeline Corrosion and Integrity Assessor — a 20 km gas line at year 15: cathodic protection solved as a leaky
   resistor ladder (rectifier, coating conductance, soil, insulating flanges, DC-railway interference), on and
   instant-off potentials against the Appendix K criteria (NACE RP0169 §6), corrosion rate at coating holidays, in-line
   inspection features assessed by ASME B31.8-2003 862.213 / Appendix L (original B31G) with modified B31G as a
   comparison, repairs per 851.44, MAOP reduction, growth over time, failures, remaining life and re-inspection.
   Standalone, offline. Teaching model. */
'use strict';

const r3 = x => Number.isFinite(x) ? Math.round(x * 1000) / 1000 : x;
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const L = 20000, DX = 100, N = L / DX + 1, RHO_ST = 1.8e-7, X_TR = 10000, X_RAIL = 15000;
const COAT = { fbe: 3e-5, ct: 4e-4, tape: 2e-3 }, COATN = { fbe: 'FBE', ct: 'coal tar', tape: 'tape' };
const soilAt = x => x < 4000 ? 60 : x < 6000 ? 8 : x < 11000 ? 45 : x < 14000 ? 450 : x < 16000 ? 30 : 70; // Ω·m
const FEAT0 = [ // id, x m, depth fraction of wall at year 15, length mm [illustrative]
  ['F01', 2300, 0.12, 60], ['F02', 4300, 0.62, 300], ['F03', 4650, 0.48, 90], ['F04', 5200, 0.83, 30], ['F05', 5600, 0.08, 120], ['F06', 7400, 0.35, 650],
  ['F07', 8800, 0.22, 150], ['F08', 9900, 0.15, 40], ['F09', 11500, 0.40, 200], ['F10', 12600, 0.55, 120], ['F11', 13300, 0.30, 90], ['F12', 15050, 0.58, 160],
  ['F13', 15300, 0.25, 70], ['F14', 17200, 0.18, 300], ['F15', 18400, 0.45, 45], ['F16', 19500, 0.11, 500], ['F17', 1200, 0.06, 80], ['F18', 3300, 0.14, 35],
  ['F19', 6400, 0.19, 220], ['F20', 10800, 0.09, 60], ['F21', 14500, 0.16, 110], ['F22', 16600, 0.13, 75]];
const ACTS = { '': '—', accept: 'Accept, monitor', sleeve: 'Sleeve', cutout: 'Cut out', patch: 'Plate patch', weld: 'Weld deposit' };

const state = { OD: 406.4, wt: 7.1, S: 359, F: 0.72, MAOP: 70, method: 'L', tol: false, I: 1.0, coat: 'ct', bond: false, recoat: false,
  year: 15, feats: [], sel: -1, post: -1, failures: [], derated: false, log: [], flags: {}, csv: '' };
const R = {};

/* ------------------------------------------------------------------ cathodic protection (resistor ladder) */
function cp(s) {
  const r = RHO_ST / (Math.PI * (s.OD - s.wt) / 1000 * s.wt / 1000), a = 1 / (r * DX), G = new Float64Array(N), J = new Float64Array(N);
  for (let i = 0; i < N; i++) { const x = i * DX; let g = COAT[s.coat]; if (x >= 4000 && x <= 6000) g *= s.recoat ? 0.3 : 6; if (x >= 13000 && x <= 14000) g *= 3; G[i] = g * Math.PI * s.OD / 1000 * DX; }
  J[X_TR / DX] += s.I;
  const Is = 0.6; if (!s.bond) { J[X_RAIL / DX] -= Is; for (let i = 170; i < N; i++) J[i] += Is / (N - 170); } else for (let i = 170; i < N; i++) J[i] += 0.3 / (N - 170);
  // tridiagonal: a V[i-1] − (2a + G) V[i] + a V[i+1] = −J ; insulating flanges at both ends
  const lo = new Float64Array(N), di = new Float64Array(N), up = new Float64Array(N), rhs = new Float64Array(N);
  for (let i = 0; i < N; i++) { lo[i] = i > 0 ? a : 0; up[i] = i < N - 1 ? a : 0; di[i] = -(lo[i] + up[i]) - G[i]; rhs[i] = -J[i]; }
  for (let i = 1; i < N; i++) { const m = lo[i] / di[i - 1]; di[i] -= m * up[i - 1]; rhs[i] -= m * rhs[i - 1]; }
  const V = new Float64Array(N); V[N - 1] = rhs[N - 1] / di[N - 1]; for (let i = N - 2; i >= 0; i--) V[i] = (rhs[i] - up[i] * V[i + 1]) / di[i];
  if (!s.bond) for (let i = 0; i < N; i++) { const d = Math.abs(i * DX - X_RAIL); if (d <= 400) V[i] -= 0.35 * (0.5 + 0.5 * Math.cos(Math.PI * d / 400)); } // stray current leaves through holidays near the crossing [illustrative]
  const on = [], off = [], nat = [], pol = [], rate = [], free = [], ok = [];
  for (let i = 0; i < N; i++) {
    const x = i * DX, rho = soilAt(x), f = rho / (rho + 120), En = rho < 20 ? -0.62 : -0.55, Ep = V[i] * (1 - f);
    nat.push(En); on.push(En - V[i]); off.push(En - Ep); pol.push(Ep);
    const r0 = 0.6 / (1 + rho / 15); free.push(r0); rate.push(clamp(r0 * Math.exp(-Ep / 0.035), 0.002, 20 * r0));
    ok.push(off[i] <= -0.85 || Ep >= 0.100);
  }
  return { V, on, off, nat, pol, rate, free, ok, cover: ok.filter(Boolean).length / N * 100, worstOff: Math.max(...off), mostNeg: Math.min(...off), stray: !s.bond };
}

/* ------------------------------------------------------------------ remaining strength (Appendix L, modified B31G) */
const Pdes = s => 2 * s.S * s.wt * s.F / s.OD * 10; // bar, without E
const depthOf = (s, f) => clamp(f.d + (s.tol ? 0.10 : 0), 0, 1);
function Aof(s, Lmm) { return 0.893 * Lmm / Math.sqrt(s.OD * s.wt); }
function Pprime(s, dt, Lmm, method = s.method) {
  const P = Math.max(Pdes(s), s.MAOP);
  if (method === 'mod') { const z = Lmm * Lmm / (s.OD * s.wt), M = z <= 50 ? Math.sqrt(1 + 0.6275 * z - 0.003375 * z * z) : 0.032 * z + 3.3; const Sf = (s.S + 69) * (1 - 0.85 * dt) / (1 - 0.85 * dt / M); return s.F * 2 * Sf * s.wt / s.OD * 10; }
  if (dt < 0.10) return P;
  const A = Aof(s, Lmm); const Pp = A <= 4 ? 1.1 * P * (1 - 2 / 3 * dt) / (1 - 2 / 3 * dt / Math.sqrt(A * A + 1)) : 1.1 * P * (1 - dt);
  return Math.min(Pp, P);
}
function PfailBest(s, dt, Lmm) { const z = Lmm * Lmm / (s.OD * s.wt), M = z <= 50 ? Math.sqrt(1 + 0.6275 * z - 0.003375 * z * z) : 0.032 * z + 3.3; return 2 * (s.S + 69) * (1 - 0.85 * dt) / (1 - 0.85 * dt / M) * s.wt / s.OD * 10; }
function burnThrough(Pbar) { const p = (Pbar + 1.013) * 14.5038, inch = p <= 500 ? 0.320 + (0.270 - 0.320) * (p - 15) / 485 : p <= 900 ? 0.270 + (0.190 - 0.270) * (p - 500) / 400 : 0.190; return inch * 25.4; } // 851.44(f) table at 10 ft/s
function assess(s, f, c) {
  const dt = depthOf(s, f), Pp = Pprime(s, dt, f.L), A = Aof(s, f.L), rate = c.rate[Math.round(f.x / DX)];
  let verdict = dt < 0.10 ? 'no reduction' : dt > 0.80 ? 'over 80 %: remove' : Pp >= s.MAOP ? 'acceptable' : 'P′ below MAOP';
  const mustAct = dt > 0.80 || Pp < s.MAOP;
  const valid = { '': !mustAct, accept: !mustAct, sleeve: true, cutout: true, patch: s.S <= 276 && f.L <= Math.PI * s.OD / 2, weld: f.L <= 100 && s.wt * (1 - dt) >= burnThrough(s.MAOP) && dt <= 0.80 };
  let life = 0; { let d = f.d; for (let y = 0; y <= 50; y += 0.1) { const dd = clamp(d + (s.tol ? 0.1 : 0), 0, 1); if (dd > 0.80 || Pprime(s, dd, f.L) < s.MAOP) break; life = y; d += rate * 0.1 / s.wt; } }
  return { dt, Pp, A, rate, verdict, mustAct, valid, life: f.act === 'sleeve' || f.act === 'cutout' || f.act === 'patch' || f.act === 'weld' ? 50 : life };
}
const repaired = f => ['sleeve', 'cutout', 'patch', 'weld'].includes(f.act);

function compute() {
  const s = state, c = cp(s); R.c = c; R.Pdes = Pdes(s);
  R.as = s.feats.map(f => assess(s, f, c));
  R.mustAct = R.as.filter((a, i) => a.mustAct && !repaired(s.feats[i])).length;
  R.open = s.feats.filter((f, i) => R.as[i].mustAct && !repaired(f)).length;
  R.wrong = s.feats.filter((f, i) => f.act && !R.as[i].valid[f.act]).length;
  R.repairs = s.feats.filter(repaired).length;
  const live = s.feats.map((f, i) => [f, R.as[i]]).filter(([f]) => !repaired(f));
  R.lowP = live.length ? Math.min(...live.map(([, a]) => a.Pp)) : null;
  R.life = Math.min(50, ...live.map(([, a]) => a.life)); R.reinspect = Math.min(10, 0.5 * R.life);
}

/* ------------------------------------------------------------------ time */
function years(n) {
  const s = state; compute();
  for (let k = 0; k < n * 10; k++) {
    const c = R.c;
    for (const f of s.feats) {
      if (repaired(f) || f.failed) continue;
      f.d += c.rate[Math.round(f.x / DX)] * 0.1 / s.wt;
      const Pf = PfailBest(s, Math.min(f.d, 0.999), f.L);
      if (f.d >= 0.95 || Pf <= s.MAOP) { f.failed = true; f.act = 'cutout'; const type = f.d >= 0.95 || f.L < 60 ? 'leak' : 'rupture'; s.failures.push({ id: f.id, year: +(s.year + (k + 1) / 10).toFixed(1), type, x: f.x }); s.flags.failure = true; }
    }
  }
  s.year += n; compute();
}

/* ------------------------------------------------------------------ route plot (SVG) */
const VW = 1000, VH = 380, X0 = 60, XW = 920, ET = -0.3, EB = -1.5;
const xs = x => X0 + x / L * XW, ye = E => 80 + (E - ET) / (EB - ET) * 190;
const verdictCol = (a, f) => repaired(f) ? 'var(--steel)' : a.mustAct ? 'var(--alarm)' : 'var(--good)';
function drawRig() {
  const s = state, c = R.c, P = [];
  P.push(`<svg viewBox="0 0 ${VW} ${VH}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="CP potentials and corrosion features along the route">`);
  P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="24" y2="24" style="stroke:var(--steel);stroke-width:5"/>`);
  for (const x of [0, L]) P.push(`<line x1="${xs(x) + (x ? -3 : 3)}" x2="${xs(x) + (x ? -3 : 3)}" y1="15" y2="33" style="stroke:var(--ink);stroke-width:3"/>`);
  for (let k = 0; k <= 20; k++) P.push(`<line x1="${xs(k * 1000)}" x2="${xs(k * 1000)}" y1="27" y2="33" style="stroke:var(--ink)"/>`);
  P.push(`<rect x="${xs(X_TR) - 22}" y="2" width="44" height="14" rx="2" style="fill:var(--panel);stroke:var(--ink)"/><text x="${xs(X_TR)}" y="12" text-anchor="middle" style="fill:var(--ink);font:600 9px var(--mono)">TR ${fmt(s.I, 1)} A</text>`);
  P.push(`<line x1="${xs(X_RAIL)}" x2="${xs(X_RAIL)}" y1="6" y2="36" style="stroke:var(--alarm);stroke-width:2;stroke-dasharray:3 2"/><text x="${xs(X_RAIL) + 4}" y="12" style="fill:var(--alarm);font:9px var(--mono)">DC rail${s.bond ? ' · bonded' : ''}</text>`);
  for (let i = 0; i < N - 1; i++) { const x = i * DX; let g = COAT[s.coat]; if (x >= 4000 && x <= 6000) g *= s.recoat ? 0.3 : 6; if (x >= 13000 && x <= 14000) g *= 3; const o = clamp((Math.log10(g) + 5) / 3.2, 0.05, 1), rho = soilAt(x), so = clamp(1 - Math.log10(rho) / 2.8, 0.05, 1);
    P.push(`<rect x="${xs(x)}" y="42" width="${xs(DX) - X0 + 0.5}" height="9" style="fill:var(--brass);opacity:${o.toFixed(2)}"/><rect x="${xs(x)}" y="55" width="${xs(DX) - X0 + 0.5}" height="9" style="fill:var(--water);opacity:${so.toFixed(2)}"/>`); }
  P.push(`<text x="${X0 - 6}" y="50" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">coating</text><text x="${X0 - 6}" y="63" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">wet soil</text>`);
  for (let E = -0.4; E >= -1.5; E -= 0.2) P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="${ye(E)}" y2="${ye(E)}" style="stroke:var(--grid)"/><text x="${X0 - 6}" y="${ye(E) + 3}" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">${E.toFixed(1)}</text>`);
  const hl = (E, col, dash, lb) => P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="${ye(E)}" y2="${ye(E)}" style="stroke:${col};stroke-dasharray:${dash}"/><text x="${X0 + XW - 4}" y="${ye(E) - 3}" text-anchor="end" style="fill:${col};font:9px var(--mono)">${lb}</text>`);
  hl(-0.85, 'var(--alarm)', '6 3', '−0.85 V'); hl(-1.2, 'var(--brass)', '2 3', '−1.20 V');
  const poly = arr => arr.map((E, i) => `${xs(i * DX).toFixed(1)},${ye(clamp(E, EB, ET)).toFixed(1)}`).join(' ');
  P.push(`<polyline points="${poly(c.nat)}" style="fill:none;stroke:var(--muted);stroke-dasharray:2 3"/>`);
  P.push(`<polyline points="${poly(c.nat.map(E => E - 0.1))}" style="fill:none;stroke:var(--good);stroke-dasharray:1 3;opacity:.8"/>`);
  P.push(`<polyline points="${poly(c.on)}" style="fill:none;stroke:var(--water);stroke-width:2"/>`);
  P.push(`<polyline points="${poly(c.off)}" style="fill:none;stroke:var(--good);stroke-width:2.5"/>`);
  for (let i = 0; i < N - 1; i++) if (!c.ok[i]) P.push(`<rect x="${xs(i * DX)}" y="272" width="${xs(DX) - X0 + 0.5}" height="4" style="fill:var(--alarm)"/>`);
  P.push(`<text x="${X0 + 4}" y="${ye(-0.3) + 12}" style="fill:var(--muted);font:9px var(--mono)">V to Cu/CuSO₄ · blue on · green instant-off · dotted native and native − 100 mV · red strip: no criterion met</text>`);
  if (s.post >= 0) { const i = s.post * 10; P.push(`<line x1="${xs(i * DX)}" x2="${xs(i * DX)}" y1="78" y2="272" style="stroke:var(--ink);stroke-dasharray:3 3"/><circle cx="${xs(i * DX)}" cy="${ye(c.on[i])}" r="4" style="fill:var(--water)"/><circle cx="${xs(i * DX)}" cy="${ye(c.off[i])}" r="4" style="fill:var(--good)"/>`); }
  P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="350" y2="350" style="stroke:var(--line)"/><text x="${X0 - 6}" y="322" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">ILI</text>`);
  s.feats.forEach((f, i) => { const a = R.as[i], h = Math.max(2, depthOf(s, f) * 60), x = xs(f.x); P.push(`<rect x="${x - 3}" y="${350 - h}" width="6" height="${h}" style="fill:${f.failed ? 'var(--ink)' : verdictCol(a, f)};${i === s.sel ? 'stroke:var(--ink);stroke-width:2' : ''}"/>`); if (i === s.sel || a.mustAct && !repaired(f)) P.push(`<text x="${x}" y="${350 - h - 4}" text-anchor="middle" style="fill:var(--ink);font:${i === s.sel ? 700 : 400} 9px var(--mono)">${f.id}</text>`); if (f.failed) P.push(`<text x="${x}" y="${350 - h - 14}" text-anchor="middle" style="fill:var(--alarm);font:700 14px var(--mono)">✸</text>`); });
  for (let k = 0; k <= 20; k += 2) P.push(`<text x="${xs(k * 1000)}" y="366" text-anchor="middle" style="fill:var(--muted);font:10px var(--mono)">${k}</text>`);
  P.push(`<text x="${X0 + XW}" y="378" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">route km · feature bar height = depth (green acceptable, red act, grey repaired)</text></svg>`);
  $('#rig').innerHTML = P.join('');
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function axes(ctx, W, H, pad, x0, x1, y0, y1, xl, yl, nx = 5, ny = 4, dx = 0, dy = 0) {
  ctx.fillStyle = css('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.strokeStyle = css('--grid');
  for (let k = 0; k <= nx; k++) { const x = pad.l + k / nx * (W - pad.l - pad.r); ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, H - pad.b); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText((x0 + k / nx * (x1 - x0)).toFixed(dx), x, H - pad.b + 14); }
  for (let k = 0; k <= ny; k++) { const y = H - pad.b - k / ny * (H - pad.t - pad.b); ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText((y0 + k / ny * (y1 - y0)).toFixed(dy), pad.l - 6, y + 4); }
  ctx.textAlign = 'center'; ctx.fillText(xl, (pad.l + W - pad.r) / 2, H - 4); ctx.save(); ctx.translate(12, (pad.t + H - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yl, 0, 0); ctx.restore();
}
function curveDt(s, Lmm) { let lo = 0, hi = 1; if (Pprime(s, 0.999, Lmm) >= s.MAOP) return 1; for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (Pprime(s, m, Lmm) >= s.MAOP) lo = m; else hi = m; } return lo; }
function drawAccept() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, s = state, Lm = 800;
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, 0, Lm, 0, 100, 'feature length L, mm (axial)', 'depth, % of wall', 8, 5);
  const X = l => pad.l + l / Lm * (W - pad.l - pad.r), Y = p => H - pad.b - p / 100 * (H - pad.t - pad.b);
  ctx.fillStyle = css('--alarm'); ctx.globalAlpha = 0.08; ctx.beginPath(); ctx.moveTo(X(0), Y(100)); for (let l = 1; l <= Lm; l += 5) ctx.lineTo(X(l), Y(Math.max(10, curveDt(s, l) * 100))); ctx.lineTo(X(Lm), Y(100)); ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
  ctx.strokeStyle = css('--water'); ctx.lineWidth = 2.5; ctx.beginPath(); for (let l = 1; l <= Lm; l += 5) { const y = Y(Math.max(10, curveDt(s, l) * 100)); l === 1 ? ctx.moveTo(X(l), y) : ctx.lineTo(X(l), y); } ctx.stroke();
  for (const [p, lb] of [[10, '10 %: no reduction below'], [80, '80 %: consider removing']]) { ctx.strokeStyle = css('--steel'); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(pad.l, Y(p)); ctx.lineTo(W - pad.r, Y(p)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--steel'); ctx.textAlign = 'right'; ctx.fillText(lb, W - pad.r - 4, Y(p) - 4); }
  const A4 = 4 * Math.sqrt(s.OD * s.wt) / 0.893; if (A4 < Lm) { ctx.strokeStyle = css('--muted'); ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(A4), pad.t); ctx.lineTo(X(A4), H - pad.b); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--muted'); ctx.textAlign = 'left'; ctx.fillText('A = 4', X(A4) + 4, pad.t + 12); }
  ctx.fillStyle = css('--water'); ctx.textAlign = 'left'; ctx.fillText(`curve: P′ = MAOP ${s.MAOP} bar · ${s.method === 'mod' ? 'modified B31G' : 'Appendix L'}${s.tol ? ' · depths +10 %' : ''}`, pad.l + 8, pad.t + 14);
  s.feats.forEach((f, i) => { const a = R.as[i], x = X(Math.min(f.L, Lm)), y = Y(a.dt * 100); ctx.beginPath(); ctx.arc(x, y, i === s.sel ? 7 : 4.5, 0, 7); if (repaired(f)) { ctx.strokeStyle = css('--steel'); ctx.lineWidth = 1.5; ctx.stroke(); } else { ctx.fillStyle = css(a.mustAct ? '--alarm' : '--good'); ctx.fill(); } if (i === s.sel) { ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2; ctx.stroke(); ctx.fillStyle = css('--ink'); ctx.fillText(f.id, x + 9, y + 4); } });
}
function drawProj() {
  const cv = $('#schart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 56, t: 14, b: 36 }, s = state;
  frame(ctx, W, H, pad); const f = s.feats[s.sel];
  axes(ctx, W, H, pad, s.year, s.year + 30, 0, 100, 'year in service', 'depth, % of wall', 6, 5);
  const X = y => pad.l + (y - s.year) / 30 * (W - pad.l - pad.r), Y = p => H - pad.b - clamp(p, 0, 100) / 100 * (H - pad.t - pad.b), Pm = Math.max(R.Pdes, s.MAOP) * 1.15, YP = p => H - pad.b - clamp(p / Pm, 0, 1) * (H - pad.t - pad.b);
  ctx.fillStyle = css('--water'); ctx.textAlign = 'left'; for (let k = 0; k <= 4; k++) ctx.fillText((k / 4 * Pm).toFixed(0), W - pad.r + 6, H - pad.b - k / 4 * (H - pad.t - pad.b) + 4); ctx.fillText('bar', W - pad.r + 6, H - pad.b + 14);
  ctx.strokeStyle = css('--alarm'); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(pad.l, YP(s.MAOP)); ctx.lineTo(W - pad.r, YP(s.MAOP)); ctx.moveTo(pad.l, Y(80)); ctx.lineTo(W - pad.r, Y(80)); ctx.stroke(); ctx.setLineDash([]);
  if (!f) { ctx.fillStyle = css('--muted'); ctx.textAlign = 'center'; ctx.fillText('select a feature', W / 2, H / 2); return; }
  const a = R.as[s.sel]; let d = f.d; const dp = [], pp = [];
  for (let y = 0; y <= 30; y += 0.5) { const dd = clamp(d + (s.tol ? 0.1 : 0), 0, 1); dp.push([s.year + y, dd * 100]); pp.push([s.year + y, Pprime(s, dd, f.L)]); if (!repaired(f)) d += a.rate * 0.5 / s.wt; }
  ctx.lineWidth = 2.5; ctx.strokeStyle = css('--steel'); ctx.beginPath(); dp.forEach(([y, v], k) => k ? ctx.lineTo(X(y), Y(v)) : ctx.moveTo(X(y), Y(v))); ctx.stroke();
  ctx.strokeStyle = css('--water'); ctx.beginPath(); pp.forEach(([y, v], k) => k ? ctx.lineTo(X(y), YP(v)) : ctx.moveTo(X(y), YP(v))); ctx.stroke();
  ctx.fillStyle = css('--ink'); ctx.textAlign = 'left'; ctx.fillText(`${f.id} at ${fmt(f.x / 1000, 1)} km · ${fmt(a.rate, 3)} mm/y · ${repaired(f) ? 'repaired' : 'life ' + (a.life >= 50 ? '50+ ' : fmt(a.life, 1) + ' ') + 'y'}`, pad.l + 8, pad.t + 14);
  if (!repaired(f) && a.life < 30) { ctx.strokeStyle = css('--alarm'); ctx.beginPath(); ctx.moveTo(X(s.year + a.life), pad.t); ctx.lineTo(X(s.year + a.life), H - pad.b); ctx.stroke(); }
}
function drawRate() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, c = R.c, ym = Math.max(0.45, ...c.rate) * 1.1;
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, 0, 20, 0, ym, 'route km', 'mm/y', 10, 4, 0, 2);
  const X = i => pad.l + i / (N - 1) * (W - pad.l - pad.r), Y = v => H - pad.b - clamp(v / ym, 0, 1) * (H - pad.t - pad.b);
  const line = (arr, col, w, dash = []) => { ctx.strokeStyle = css(col); ctx.lineWidth = w; ctx.setLineDash(dash); ctx.beginPath(); arr.forEach((v, i) => i ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v))); ctx.stroke(); ctx.setLineDash([]); };
  line(c.free, '--steel', 1.5, [4, 3]); line(c.rate, '--alarm', 2.5);
}

/* ------------------------------------------------------------------ readouts, anomaly table */
function updateReadouts() {
  const s = state, c = R.c;
  if (s.post >= 0) { const i = s.post * 10; $('#roPost').textContent = `km ${s.post}: on ${fmt(c.on[i], 3)} · off ${fmt(c.off[i], 3)} V`; $('#roPostNote').textContent = `native ${fmt(c.nat[i], 2)} · polarisation ${fmt(c.pol[i] * 1000, 0)} mV · IR drop ${fmt((c.off[i] - c.on[i]) * 1000, 0)} mV · soil ${soilAt(i * DX)} Ω·m · ${c.ok[i] ? 'protected' : 'NOT protected'}`; $('#roPostBox').classList.toggle('alarm', !c.ok[i]); }
  $('#roCover').textContent = fmt(c.cover, 1) + ' %'; $('#roCoverNote').textContent = `−0.85 V off or 100 mV polarisation · worst off ${fmt(c.worstOff, 3)} V · most negative ${fmt(c.mostNeg, 3)} V`;
  $('#roCoverBox').classList.toggle('alarm', c.cover < 99);
  $('#roTR').textContent = fmt(s.I, 1) + ' A'; $('#roTRNote').textContent = `drain point at 10 km · ${COATN[s.coat]} coating${s.recoat ? ' · wetland recoated' : ''}${s.bond ? ' · railway bonded' : ''}`;
  const f = s.feats[s.sel];
  if (f) { const a = R.as[s.sel]; $('#roFeat').textContent = `${f.id}: d/t ${fmt(a.dt * 100, 0)} %, L ${f.L} mm, A ${fmt(a.A, 2)} → P′ ${fmt(a.Pp, 1)} bar`; $('#roFeatNote').textContent = `${a.verdict}${repaired(f) ? ' · repaired: ' + ACTS[f.act] : f.act ? ' · ' + ACTS[f.act] : ''}${f.act && !a.valid[f.act] ? ' · NOT a valid choice' : ''} · ${a.A > 4 ? 'long: P′ = 1.1 P (1 − d/t)' : 'short: parabolic area'}`; $('#roFeatBox').classList.toggle('alarm', (a.mustAct && !repaired(f) && f.act !== 'accept') || (f.act && !a.valid[f.act])); }
  $('#roSum').textContent = `${s.feats.length} · act on ${s.feats.filter((x, i) => R.as[i].mustAct).length} · open ${R.open}`; $('#roSumNote').textContent = `${R.repairs} repaired · ${R.wrong} invalid choices · lowest unrepaired P′ ${fmt(R.lowP, 1)} bar`;
  $('#roSumBox').classList.toggle('alarm', R.open > 0 || R.wrong > 0);
  $('#roLife').textContent = (R.life >= 50 ? '50+' : fmt(R.life, 1)) + ' y'; $('#roLifeNote').textContent = `shortest remaining life · next inspection within ${fmt(R.reinspect, 1)} y (half-life rule, ≤ 10 y)`;
  $('#roMaop').textContent = fmt(s.MAOP, 1) + ' bar'; $('#roMaopNote').textContent = `design pressure ${fmt(R.Pdes, 1)} bar (F ${s.F}) · hoop at MAOP ${fmt(s.MAOP / 10 * s.OD / (2 * s.wt) / s.S * 100, 0)} % SMYS${s.MAOP / 10 * s.OD / (2 * s.wt) / s.S < 0.4 ? ' · below 40 %: Appendix L not required' : ''}`;
  $('#roFail').textContent = s.failures.length ? s.failures.map(x => `${x.id} ${x.type} yr ${x.year}`).join(' · ') : 'No failures'; $('#roFailNote').textContent = s.failures.length ? 'failed features were cut out after the event' : `year ${s.year} · +5 years grows unrepaired features`;
  $('#roFailBox').classList.toggle('alarm', s.failures.length > 0);
  $('#pillYear').textContent = `Year ${s.year} in service`; $('#pillCP').textContent = `CP ${fmt(c.cover, 0)} % of route`; $('#pillCP').className = 'pill ' + (c.cover >= 99 ? 'ok' : 'amber');
  $('#pillOver').classList.toggle('hidden', c.mostNeg >= -1.2); $('#pillStray').classList.toggle('hidden', !c.stray); $('#pillFail').classList.toggle('hidden', !s.failures.length); $('#pillOpen').classList.toggle('hidden', R.open === 0);
}
function renderTable() {
  const s = state;
  $('#featTable tbody').innerHTML = s.feats.map((f, i) => { const a = R.as[i], v = f.act ? a.valid[f.act] : !a.mustAct; return `<tr data-i="${i}" class="${i === s.sel ? 'sel' : ''}"><td>${f.id}${f.failed ? ' ✸' : ''}</td><td>${fmt(f.x / 1000, 2)}</td><td>${fmt(a.dt * 100, 0)}</td><td>${f.L}</td><td>${fmt(a.A, 2)}</td><td>${fmt(a.Pp, 1)}</td><td class="${a.mustAct ? 'bad' : 'okc'}">${a.verdict}</td><td>${fmt(a.rate, 3)}</td><td>${repaired(f) ? '—' : a.life >= 50 ? '50+' : fmt(a.life, 1)}</td><td><select data-i="${i}" aria-label="Action for ${f.id}">${Object.entries(ACTS).map(([k, t]) => `<option value="${k}"${(f.act || '') === k ? ' selected' : ''}>${t}</option>`).join('')}</select></td><td class="${v ? 'okc' : 'bad'}">${f.act || a.mustAct ? (v ? 'yes' : 'no') : ''}</td></tr>`; }).join('');
}

/* ------------------------------------------------------------------ log and tasks */
function record(tag = '') {
  const s = state, c = R.c, gname = { 245: 'Gr B', 290: 'X42', 359: 'X52', 414: 'X60' }[s.S];
  const row = { i: s.log.length + 1, tag, year: s.year, grade: gname, MAOP: r3(s.MAOP), method: s.method, tol: s.tol ? 'yes' : 'no', I: r3(s.I), coat: COATN[s.coat], bond: s.bond ? 'yes' : 'no', cover: r3(c.cover), worst: r3(c.worstOff), neg: r3(c.mostNeg), must: s.feats.filter((x, i) => R.as[i].mustAct).length, open: R.open, lowP: r3(R.lowP), life: r3(R.life), reins: r3(R.reinspect), rep: R.repairs, wrong: R.wrong, fail: s.failures.length };
  s.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  const K = ['i', 'tag', 'year', 'grade', 'MAOP', 'method', 'tol', 'I', 'coat', 'bond', 'cover', 'worst', 'neg', 'must', 'open', 'lowP', 'life', 'reins', 'rep', 'wrong', 'fail'];
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${K.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,year,grade,MAOP_bar,method,tool_tolerance,rectifier_A,coating,rail_bond,CP_pct_route,worst_off_V,most_negative_off_V,features_to_act_on,open,lowest_Pprime_bar,remaining_life_y,reinspect_y,repairs,invalid_choices,failures\n' + state.log.map(r => K.map(k => r[k]).join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
const TASKS = [
  { id: 'post', t: 'Read a test post', d: 'Click the potential plot to read a test post: on, instant-off, native, polarisation and IR drop.', ok: () => state.flags.post },
  { id: 'irtrap', t: 'The IR-drop trap', d: 'Find a test post where the on potential is more negative than −0.85 V but the pipe is not protected (instant-off above −0.85 V and less than 100 mV polarisation). Look in the high-resistivity ground.', ok: () => state.flags.irtrap },
  { id: 'cover', t: 'Protect the whole line', d: 'Raise the rectifier current until at least 99 % of the route meets a criterion, without any off potential below −1.20 V. Record it.', ok: () => state.log.some(r => r.cover >= 99 && r.neg >= -1.2) },
  { id: 'over', t: 'Too much', d: 'Push the current until the off potential near the drain point is below −1.20 V. Read why that is a problem.', ok: () => state.flags.over },
  { id: 'stray', t: 'Stray current', d: 'Find where the DC railway current leaves the pipe (potentials swing positive at 15 km). Install the drainage bond and record the difference.', ok: () => state.log.some(r => r.bond === 'no' && r.cover < 99) && state.log.some(r => r.bond === 'yes') },
  { id: 'shortlong', t: 'Short and long', d: 'Select a short deep feature (A ≤ 4) and a long shallow one (A > 4). Compare which formula Appendix L uses and why the long one fails at a shallower depth.', ok: () => state.flags.shortSel && state.flags.longSel },
  { id: 'tol', t: 'Tool tolerance', d: 'Record the assessment with and without +10 % depth tolerance. Count how many more features need action.', ok: () => state.log.some(a => a.tol === 'yes' && state.log.some(b => b.tol === 'no' && b.year === a.year && a.must > b.must)) },
  { id: 'method', t: 'Original against modified', d: 'Record the same year with Appendix L and with modified B31G. Compare the lowest P′ and the number of features to act on.', ok: () => state.log.some(a => a.method === 'mod' && state.log.some(b => b.method === 'L' && b.year === a.year && b.tol === a.tol)) },
  { id: 'act', t: 'Deal with every feature', d: 'Choose a valid action for every feature that needs one (none open, no invalid choices) and record. Over 80 % deep: take it out.', ok: () => state.log.some(r => r.open === 0 && r.wrong === 0 && r.rep > 0) },
  { id: 'derate', t: 'Or reduce the pressure', d: 'Instead of repairing, reduce the MAOP to the lowest P′. Which feature could you still not leave in service?', ok: () => state.flags.derate },
  { id: 'neglect', t: 'Ten years of doing nothing', d: 'From the year-15 baseline, leave CP at 1 A and repair nothing. Run +5 years twice. Record the failures and where they happened.', ok: () => state.log.some(r => r.fail > 0) },
  { id: 'patch', t: 'When a patch is allowed', d: 'On the old Grade B line (35 ksi), repair a failing feature with a plate patch. Try the same on X52: why is it not valid?', ok: () => state.flags.patchValid },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ wiring */
let dirty = true;
function refresh() { compute(); if (R.c.mostNeg < -1.2) state.flags.over = true; drawRig(); updateReadouts(); renderTable(); drawAccept(); drawProj(); drawRate(); checkTasks(); dirty = false; }
function tick() { if (dirty) refresh(); requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') e.checked = !!v; else e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function selectFeat(i) { const s = state; s.sel = clamp(i, -1, s.feats.length - 1); if (s.sel >= 0) { compute(); const a = R.as[s.sel]; if (a.dt >= 0.1) { if (a.A > 4) s.flags.longSel = true; else s.flags.shortSel = true; } } dirty = true; }
function selectPost(k) { const s = state; s.post = clamp(k, 0, 20); s.flags.post = true; compute(); const i = s.post * 10, c = R.c; if (c.on[i] <= -0.85 && !c.ok[i]) s.flags.irtrap = true; dirty = true; }
function act(kind, i = state.sel) { const s = state, f = s.feats[i]; if (!f || f.failed) return; compute(); const a = R.as[i]; f.act = kind; if (kind === 'patch' && a.valid.patch && a.mustAct) s.flags.patchValid = true; dirty = true; }
function derate() { const s = state; compute(); if (R.lowP === null) return; setCtl('MAOP', Math.max(20, Math.floor(R.lowP * 2) / 2)); s.derated = true; s.flags.derate = true; dirty = true; }
function reset(over = {}) {
  const s = state; s.feats = FEAT0.map(([id, x, d, Lm]) => ({ id, x, d, L: Lm, act: '', failed: false })); s.year = 15; s.failures = []; s.sel = 1; s.post = -1; s.derated = false;
  const v = Object.assign({ OD: 406.4, wt: 7.1, grade: 359, cls: '0.72', MAOP: 70, method: 'L', tol: false, I: 4.0, coat: 'ct', bond: false, recoat: false }, over);
  for (const [k, x] of Object.entries(v)) setCtl(k, x); dirty = true;
}
const PRESETS = {
  base: () => reset(),
  lowcp: () => { reset({ I: 1.0 }); selectPost(5); },
  rocky: () => { reset({ I: 4.0 }); selectPost(12); },
  over: () => { reset({ I: 9 }); selectPost(10); },
  stray: () => { reset({ I: 6.0 }); selectPost(15); selectFeat(11); },
  neglect: () => { reset({ I: 1.0 }); years(10); },
  grb: () => { reset({ grade: 245, wt: 7.9, MAOP: 55 }); },
  fbe: () => { reset({ coat: 'fbe', I: 0.3 }); state.feats.forEach(f => f.d = Math.min(f.d, 0.12)); dirty = true; },
};
function bind() {
  const bindR = (id, f, key = id) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[key] = +e.value; if (o) o.textContent = f(state[key]); dirty = true; }; e.addEventListener('input', set); set(); };
  bindR('MAOP', v => fmt(v, 1) + ' bar'); bindR('I', v => fmt(v, 1) + ' A');
  for (const [id, key, num] of [['OD', 'OD', 1], ['wt', 'wt', 1], ['grade', 'S', 1], ['cls', 'F', 1], ['method', 'method', 0], ['coat', 'coat', 0]]) { const e = $('#' + id); const set = () => { state[key] = num ? +e.value : e.value; dirty = true; }; e.addEventListener('change', set); set(); }
  for (const id of ['tol', 'bond', 'recoat']) { const e = $('#' + id); const set = () => { state[id] = e.checked; dirty = true; }; e.addEventListener('change', set); set(); }
  $('#rig').addEventListener('click', e => { const r = $('#rig svg').getBoundingClientRect(), sx = (e.clientX - r.left) / r.width * VW, sy = (e.clientY - r.top) / r.height * VH; if (sx < X0 - 10 || sx > X0 + XW + 10) return; const x = clamp((sx - X0) / XW * L, 0, L);
    if (sy >= 280 && sy <= 356) { let bi = -1, bd = 1e9; state.feats.forEach((f, i) => { const d = Math.abs(xs(f.x) - sx); if (d < bd) { bd = d; bi = i; } }); if (bd < 14) selectFeat(bi); } else if (sy >= 70 && sy < 280) selectPost(Math.round(x / 1000)); });
  $('#featTable').addEventListener('click', e => { if (e.target.tagName === 'SELECT' || e.target.tagName === 'OPTION') return; const tr = e.target.closest('tr[data-i]'); if (tr) selectFeat(+tr.dataset.i); });
  $('#featTable').addEventListener('change', e => { if (e.target.tagName === 'SELECT') { selectFeat(+e.target.dataset.i); act(e.target.value, +e.target.dataset.i); } });
  document.querySelectorAll('[data-act]').forEach(b => b.onclick = () => act(b.dataset.act));
  $('#btnDerate').onclick = derate; $('#btnYears').onclick = () => { years(5); dirty = true; }; $('#btnRecord').onclick = () => { compute(); record(''); };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { const done = () => { $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); }; try { navigator.clipboard.writeText(state.csv || '').then(done, () => { $('#btnCopy').textContent = 'Use Download CSV'; }); } catch (e) { $('#btnCopy').textContent = 'Use Download CSV'; } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tk_theme', next); } catch (e) {} dirty = true; };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (k === 'Y') $('#btnYears').click(); if (k === 'R') $('#btnRecord').click(); if (e.key === ']') selectFeat(state.sel + 1); if (e.key === '[') selectFeat(Math.max(0, state.sel - 1)); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.PipelineIntegrity = {
  getState: () => { compute(); const s = state, c = R.c; return {
    inputs: { OD: s.OD, wt: s.wt, grade: s.S, cls: s.F, MAOP: s.MAOP, method: s.method, tol: s.tol, I: s.I, coat: s.coat, bond: s.bond, recoat: s.recoat, year: s.year, actions: Object.fromEntries(s.feats.filter(f => f.act).map(f => [f.id, f.act])) },
    results: { cover: r3(c.cover), worstOff: r3(c.worstOff), mostNeg: r3(c.mostNeg), stray: c.stray, Pdes: r3(R.Pdes), features: s.feats.map((f, i) => ({ id: f.id, km: r3(f.x / 1000), dt: r3(R.as[i].dt), L: f.L, A: r3(R.as[i].A), Pprime: r3(R.as[i].Pp), verdict: R.as[i].verdict, rate: r3(R.as[i].rate), life: r3(R.as[i].life), act: f.act, valid: f.act ? R.as[i].valid[f.act] : null, failed: f.failed })), mustAct: s.feats.filter((x, i) => R.as[i].mustAct).length, open: R.open, wrong: R.wrong, repairs: R.repairs, lowestPprime: r3(R.lowP), life: r3(R.life), reinspect: r3(R.reinspect), failures: s.failures.slice() },
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }; },
  setState: inp => {
    if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset]();
    for (const k of ['OD', 'wt', 'grade', 'cls', 'MAOP', 'method', 'tol', 'I', 'coat', 'bond', 'recoat']) if (inp[k] !== undefined) setCtl(k, inp[k]);
    if (inp.post !== undefined) selectPost(+inp.post); if (inp.select !== undefined) selectFeat(state.feats.findIndex(f => f.id === inp.select));
    if (inp.actions) for (const [id, a] of Object.entries(inp.actions)) { const i = state.feats.findIndex(f => f.id === id); if (i >= 0 && a in ACTS) act(a, i); }
    if (inp.derate) derate(); if (inp.years) years(Math.max(0, Math.floor(+inp.years))); if (inp.record) { compute(); record(inp.record === true ? '' : String(inp.record)); }
    dirty = true;
  },
  meta: { id: 'pipeline-integrity', title: 'Pipeline Corrosion and Integrity Assessor', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PipelineIntegrity.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'pipeline-integrity', state: window.PipelineIntegrity.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function boot() {
  try { const th = localStorage.getItem('tk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildTasks(); bind(); reset(); renderLog(); refresh(); requestAnimationFrame(tick);
}
boot();
