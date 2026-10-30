/* Pressure Reduction Station Simulator — a let-down station feeding a downstream system from a 70 bar line: water-bath
   heater, slam-shut, monitor and worker regulators in series, relief valve with isolation valve, downstream line pack and
   a daily demand profile. Inject failures and see which device catches them, against ASME B31.8-2003 845.1, 845.212,
   845.222, 845.34–845.36, 845.411 (MAOP + 4 % / + 10 % / 75 % SMYS), 845.421 and 841.126. Standalone, offline.
   Teaching model — regulator sizes, gains, demand and line pack are illustrative. */
'use strict';

const r3 = x => Number.isFinite(x) ? Math.round(x * 1000) / 1000 : x;
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const PA = 1.013, K = 1000, LEAK = 0.0015, KP = 6, KI = 0.4, RATE = 0.5, DT = 0.2, JT = 0.5; // Sm³/h per bar abs fully open; seat leakage; PI gains per bar and per bar·s; actuator stroke /s; s; °C per bar
const SYS = {
  dist: { name: 'Distribution lateral', MAOP: 19, D: 219.1, t: 4.8, S: 290, V: 689, Pmin: 7, sets: { Sw: 16, Sm: 17, Srv: 18, Sssv: 19 } },
  spur: { name: 'Transmission spur', MAOP: 50, D: 406.4, t: 6.4, S: 359, V: 1825, Pmin: 30, sets: { Sw: 44, Sm: 46.5, Srv: 48.5, Sssv: 50 } },
  old: { name: 'Old spur, 80 % SMYS', MAOP: 55, D: 406.4, t: 4.8, S: 290, V: 1237, Pmin: 35, sets: { Sw: 50, Sm: 53, Srv: 54, Sssv: 57 } },
};
const FAILN = { wOpen: 'Worker fails open', wShut: 'Worker fails shut', mOpen: 'Monitor stuck open', sense: 'Sensing line severed', trip: 'Demand to zero', surge: 'Inlet surge +12 bar', ssvStick: 'Slam-shut fails to trip' };

const state = { sys: 'dist', P1: 70, Qpk: 25000, Tin: 15, heat: true, dnOut: 154.1, Sw: 16, Sm: 17, Sssv: 19, Srv: 18, Crel: 600, hasMon: true, hasSSV: true, rvIso: 'locked', sense: 'sep', tscale: 60, fail: 'wOpen',
  run: true, t: 6 * 3600, P2: 16, yw: 0.3, ym: 1, ywi: 0.3, ymi: 1, Pm: 70, ssvTripped: false, faults: {}, frozen: false, freezeT: 0, maxP: 16, vented: 0, ventedSinceInject: 0, inject: null, hist: [], lastH: -1e9, Q: 0, Qd: 0, Qrel: 0, log: [], flags: {}, csv: '' };

/* ------------------------------------------------------------------ physics */
const sys = () => SYS[state.sys];
const hoopPct = s => s.MAOP / 10 * s.D / (2 * s.t) / s.S * 100;
function limitOf(s) { const p75 = 0.75 * s.S * 2 * s.t / s.D * 10; return hoopPct(s) > 72 ? { L: s.MAOP * 1.04, rule: 'MAOP + 4 % (hoop over 72 % SMYS)' } : (s.MAOP * 1.10 <= p75 ? { L: s.MAOP * 1.10, rule: 'MAOP + 10 %' } : { L: p75, rule: '75 % SMYS' }); }
function qv(y, p1a, p2a) { if (p1a <= p2a) return 0; const r = p2a / p1a; return K * y * p1a * (r < 0.5283 ? 0.849 : Math.sqrt(1 - r * r)); }
function seriesQ(s, p1a, p2a) {
  const lk = s.faults.trip ? 0.004 : LEAK, yw = Math.max(s.yw, lk); if (!s.hasMon) { s.Pm = p1a - PA; return qv(yw, p1a, p2a); }
  const ym = Math.max(s.ym, lk); let lo = p2a, hi = p1a;
  for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; if (qv(ym, p1a, m) > qv(yw, m, p2a)) lo = m; else hi = m; }
  s.Pm = (lo + hi) / 2 - PA; return qv(yw, (lo + hi) / 2, p2a);
}
function demandFrac(t) { const h = (t / 3600) % 24, g1 = Math.exp(-(((h - 7.5) / 2.2) ** 2)), g2 = Math.exp(-(((h - 18.5) / 2.6) ** 2)); return 0.25 + 0.75 * Math.max(g1, g2); }
function Tout(s) { const raw = s.Tin - JT * (P1eff(s) - s.P2); return s.heat ? Math.max(raw, 5) : raw; }
const P1eff = s => s.P1 + (s.faults.surge ? 12 : 0);
function stepPhys(dt) {
  const s = state, Y = sys(), f = s.faults;
  s.t += dt;
  const p1a = P1eff(s) + PA, p2a = s.P2 + PA;
  // sensing
  const wSees = f.sense ? 0 : s.P2, mSees = f.sense && s.sense === 'shared' ? 0 : s.P2;
  // freezing of the worker pilot (841.126(c))
  const T = Tout(s); if (T < 0) { s.freezeT += dt; if (s.freezeT > 120) s.frozen = true; } else { s.freezeT = 0; if (T > 2) s.frozen = false; }
  // controllers: integral action on the sensed pressure, rate-limited
  const pi = (sp, pv, ik, yk) => { const e = sp - pv; s[ik] = clamp(s[ik] + KI * e * dt, 0, 1); const cmd = clamp(s[ik] + KP * e, 0, 1); s[yk] += clamp(cmd - s[yk], -RATE * dt, RATE * dt); }; // PI with anti-windup and a stroke-rate limit
  if (f.wOpen) s.yw = 1; else if (f.wShut) s.yw = 0; else if (!s.frozen) pi(s.Sw, wSees, 'ywi', 'yw');
  if (!s.hasMon || f.mOpen) { s.ym = 1; s.ymi = 1; } else pi(s.Sm, mSees, 'ymi', 'ym');
  // slam-shut
  if (s.hasSSV && !f.ssvStick && !s.ssvTripped && s.P2 >= s.Sssv) { s.ssvTripped = true; s.flags.ssvTrip = true; }
  const Q = s.ssvTripped ? 0 : seriesQ(s, p1a, p2a);
  const Qd = s.Qpk * demandFrac(s.t) * (f.trip ? 0 : 1) * clamp(s.P2 / Y.Pmin, 0, 1);
  const Qrel = s.rvIso === 'locked' ? s.Crel * clamp((s.P2 - s.Srv) / (0.1 * Math.max(s.Srv, 1)), 0, 1) * p2a / (s.Srv * 1.1 + PA) : 0;
  s.P2 = Math.max(0, s.P2 + (Q - Qd - Qrel) / 3600 * PA / Y.V * dt);
  s.Q = Q; s.Qd = Qd; s.Qrel = Qrel; s.vented += Qrel / 3600 * dt; if (s.inject) s.ventedSinceInject += Qrel / 3600 * dt;
  s.maxP = Math.max(s.maxP, s.P2); if (s.inject) s.inject.max = Math.max(s.inject.max, s.P2);
  if (Qrel > 1) s.flags.vent = true;
  if (s.t - s.lastH >= 5) { s.hist.push([s.t, s.P2, Qd, Q, Qrel]); s.lastH = s.t; if (s.hist.length > 6000) s.hist.splice(0, 1000); }
}
function step(dtSim) { const n = Math.ceil(dtSim / DT); for (let k = 0; k < n; k++) stepPhys(dtSim / n); }

/* ------------------------------------------------------------------ checks */
function checks() {
  const s = state, Y = sys(), lim = limitOf(Y), Tn = Tout(s), vel = velPeak();
  const p1a = P1eff(s) + PA, need = qv(1, p1a, lim.L + PA), relAtLim = s.Crel * clamp((lim.L - s.Srv) / (0.1 * s.Srv), 0, 1) * (lim.L + PA) / (s.Srv * 1.1 + PA);
  const prot = s.hasMon || (s.hasSSV) || (s.rvIso === 'locked' && relAtLim >= need);
  return [
    ['C1', 'Worker below monitor (≥ 0.5 bar)', !s.hasMon || s.Sm - s.Sw >= 0.5, '841.126'],
    ['C2', 'Monitor at or below MAOP', !s.hasMon || s.Sm <= Y.MAOP, '845.421'],
    ['C3', 'Relief full lift within the limit', s.Srv * 1.1 <= lim.L + 1e-9 && s.Srv > (s.hasMon ? s.Sm : s.Sw), '845.411'],
    ['C4', 'Slam-shut above relief, within limit', !s.hasSSV || (s.Sssv > s.Srv && s.Sssv <= lim.L), '845.222(e)'],
    ['C5', 'A device can stop a wide-open failure', prot, '845.1, 845.411'],
    ['C6', 'Relief isolation locked open', s.rvIso === 'locked', '845.34'],
    ['C7', 'Separate sensing lines', s.sense === 'sep', '845.36(b)'],
    ['C8', 'Outlet gas above 0 °C', Tn >= 0, '841.126(c)'],
    ['C9', 'Outlet velocity ≤ 30 m/s at peak', vel <= 30, '841.126(e)'],
  ].map(([id, t, ok, ref]) => ({ id, t, ok, ref }));
}
function velPeak() { const s = state, A = Math.PI / 4 * (s.dnOut / 1000) ** 2, q = s.Qpk * PA / (s.Sw + PA) / 3600; return q / A; }
function outcome() {
  const s = state, Y = sys(), lim = limitOf(Y); if (!s.inject) return '—';
  const m = s.inject.max, parts = [];
  if (m > lim.L + 0.01) parts.push(`OVERPRESSURE ${fmt(m, 2)} bar > limit ${fmt(lim.L, 2)}`); else if (m > Y.MAOP + 0.01) parts.push(`peak ${fmt(m, 2)} bar: above MAOP, within limit`); else parts.push(`peak ${fmt(m, 2)} bar: at or below MAOP`);
  if (s.ssvTripped) parts.push('slam-shut tripped'); if (s.ventedSinceInject > 1) parts.push(`relief vented ${fmt(s.ventedSinceInject, 0)} Sm³`);
  if (s.hasMon && !s.faults.mOpen && s.ym < 0.98 && !s.ssvTripped) parts.push('monitor in control'); if (s.P2 < Y.Pmin) parts.push('customers short of pressure');
  return parts.join(' · ');
}

/* ------------------------------------------------------------------ schematic (SVG) */
const VW = 1000, VH = 360;
function valve(x, y, open, failed, label, sub) {
  const col = failed ? 'var(--alarm)' : open > 0.02 ? 'var(--good)' : 'var(--alarm)';
  return `<polygon points="${x - 16},${y - 11} ${x},${y} ${x - 16},${y + 11}" style="fill:${col};opacity:.85"/><polygon points="${x + 16},${y - 11} ${x},${y} ${x + 16},${y + 11}" style="fill:${col};opacity:.85"/>` +
    `<line x1="${x}" x2="${x}" y1="${y}" y2="${y - 22}" style="stroke:var(--ink);stroke-width:2"/><rect x="${x - 14}" y="${y - 34}" width="28" height="12" rx="2" style="fill:var(--panel);stroke:var(--ink)"/>` +
    `<text x="${x}" y="${y + 28}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">${label}</text><text x="${x}" y="${y + 40}" text-anchor="middle" style="fill:var(--muted);font:9px var(--mono)">${sub}</text>${failed ? `<text x="${x}" y="${y - 38}" text-anchor="middle" style="fill:var(--alarm);font:700 12px var(--mono)">✕</text>` : ''}`;
}
function gauge(x, y, v, lab, bad) { return `<circle cx="${x}" cy="${y}" r="17" style="fill:var(--panel);stroke:${bad ? 'var(--alarm)' : 'var(--ink)'};stroke-width:${bad ? 3 : 1.5}"/><text x="${x}" y="${y + 4}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">${v}</text><text x="${x}" y="${y - 21}" text-anchor="middle" style="fill:var(--muted);font:9px var(--mono)">${lab}</text>`; }
function drawRig() {
  const s = state, Y = sys(), f = s.faults, lim = limitOf(Y), yL = 170, P = [], flow = s.Q > 50;
  P.push(`<svg viewBox="0 0 ${VW} ${VH}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Station schematic">`);
  const pipe = (x1, x2, live) => P.push(`<line x1="${x1}" x2="${x2}" y1="${yL}" y2="${yL}" style="stroke:${live ? 'var(--water)' : 'var(--steel)'};stroke-width:${live ? 9 : 7};stroke-linecap:round"/>`);
  pipe(20, 600, flow); pipe(600, 960, s.P2 > 0.5);
  P.push(`<text x="24" y="${yL - 44}" style="fill:var(--ink);font:600 11px var(--mono)">inlet ${fmt(P1eff(s), 1)} bar</text>`);
  P.push(`<rect x="95" y="${yL - 26}" width="60" height="52" rx="6" style="fill:${s.heat ? 'var(--brass)' : 'var(--panel2)'};opacity:.85;stroke:var(--ink)"/><text x="125" y="${yL + 4}" text-anchor="middle" style="fill:var(--ink);font:700 10px var(--mono)">HEATER</text><text x="125" y="${yL + 44}" text-anchor="middle" style="fill:var(--muted);font:9px var(--mono)">${s.heat ? 'on' : 'off'}</text>`);
  if (s.hasSSV) P.push(valve(230, yL, s.ssvTripped ? 0 : 1, f.ssvStick, 'SLAM-SHUT', s.ssvTripped ? 'TRIPPED' : `trip ${fmt(s.Sssv, 1)}`));
  if (s.hasMon) P.push(valve(340, yL, s.ym, f.mOpen, 'MONITOR', `set ${fmt(s.Sm, 1)} · ${fmt(s.ym * 100, 0)} %`));
  P.push(valve(460, yL, s.yw, f.wOpen || f.wShut || s.frozen, 'WORKER', `set ${fmt(s.Sw, 1)} · ${fmt(s.yw * 100, 0)} %${s.frozen ? ' FROZEN' : ''}`));
  if (s.hasMon) P.push(gauge(400, yL - 70, fmt(s.Pm, 1), 'between', false));
  P.push(gauge(560, yL - 70, fmt(s.P2, 2), 'outlet', s.P2 > Y.MAOP));
  P.push(`<line x1="560" x2="560" y1="${yL - 53}" y2="${yL - 4}" style="stroke:var(--ink)"/>`);
  // sensing lines
  const sevX = 520;
  if (s.sense === 'shared') { P.push(`<polyline points="600,${yL + 6} 600,${yL + 70} 340,${yL + 70}" style="fill:none;stroke:var(--brass);stroke-width:2;stroke-dasharray:5 3"/><line x1="460" x2="460" y1="${yL + 70}" y2="${yL + 44}" style="stroke:var(--brass);stroke-width:2;stroke-dasharray:5 3"/>${s.hasMon ? `<line x1="340" x2="340" y1="${yL + 70}" y2="${yL + 44}" style="stroke:var(--brass);stroke-width:2;stroke-dasharray:5 3"/>` : ''}`); }
  else { P.push(`<polyline points="610,${yL + 6} 610,${yL + 62} 460,${yL + 62} 460,${yL + 44}" style="fill:none;stroke:var(--brass);stroke-width:2;stroke-dasharray:5 3"/>`); if (s.hasMon) P.push(`<polyline points="620,${yL + 6} 620,${yL + 78} 340,${yL + 78} 340,${yL + 44}" style="fill:none;stroke:var(--brass);stroke-width:2;stroke-dasharray:5 3"/>`); }
  if (f.sense) P.push(`<text x="${sevX}" y="${yL + (s.sense === 'shared' ? 74 : 66)}" text-anchor="middle" style="fill:var(--alarm);font:700 14px var(--mono)">✂</text>`);
  // relief
  P.push(`<line x1="680" x2="680" y1="${yL}" y2="${yL - 60}" style="stroke:var(--steel);stroke-width:5"/><rect x="672" y="${yL - 38}" width="16" height="10" style="fill:${s.rvIso === 'locked' ? 'var(--good)' : 'var(--alarm)'}"/><text x="694" y="${yL - 30}" style="fill:var(--muted);font:9px var(--mono)">${s.rvIso === 'locked' ? 'locked open' : 'CLOSED'}</text>`);
  P.push(`<polygon points="668,${yL - 60} 692,${yL - 60} 680,${yL - 78}" style="fill:${s.Qrel > 1 ? 'var(--alarm)' : 'var(--panel2)'};stroke:var(--ink)"/><line x1="680" x2="680" y1="${yL - 78}" y2="${yL - 112}" style="stroke:var(--steel);stroke-width:4"/><text x="694" y="${yL - 66}" style="fill:var(--ink);font:600 10px var(--mono)">RELIEF ${fmt(s.Srv, 1)}</text><text x="694" y="${yL - 54}" style="fill:var(--muted);font:9px var(--mono)">${fmt(s.Crel, 0)} Sm³/h</text>`);
  if (s.Qrel > 1) P.push(`<text x="680" y="${yL - 118}" text-anchor="middle" style="fill:var(--alarm);font:700 11px var(--mono)">VENTING ${fmt(s.Qrel, 0)} Sm³/h</text>`);
  // downstream pack and customers
  P.push(`<rect x="740" y="${yL - 22}" width="190" height="44" rx="20" style="fill:none;stroke:var(--steel);stroke-width:2"/><text x="835" y="${yL - 30}" text-anchor="middle" style="fill:var(--ink);font:600 10px var(--mono)">${Y.name}</text><text x="835" y="${yL + 38}" text-anchor="middle" style="fill:var(--muted);font:9px var(--mono)">MAOP ${Y.MAOP} · limit ${fmt(lim.L, 1)} bar · ${fmt(hoopPct(Y), 0)} % SMYS</text>`);
  P.push(`<text x="960" y="${yL - 14}" text-anchor="end" style="fill:var(--ink);font:600 10px var(--mono)">customers</text><text x="960" y="${yL + 22}" text-anchor="end" style="fill:var(--muted);font:9px var(--mono)">${fmt(s.Qd, 0)} Sm³/h</text>`);
  // set-point ladder
  const lx = 40, ly0 = 330, ly1 = 260, pmax = lim.L * 1.12, pmin = Math.min(s.Sw, Y.MAOP) * 0.85, yy = p => ly0 - (p - pmin) / (pmax - pmin) * (ly0 - ly1);
  P.push(`<text x="${lx}" y="${ly1 - 8}" style="fill:var(--muted);font:9px var(--mono)">set-point ladder (bar)</text>`);
  const rungs = [['worker', s.Sw, 'var(--good)'], ...(s.hasMon ? [['monitor', s.Sm, 'var(--good)']] : []), ['relief', s.Srv, 'var(--brass)'], ['relief full lift', s.Srv * 1.1, 'var(--brass)'], ...(s.hasSSV ? [['slam-shut', s.Sssv, 'var(--brass)']] : []), ['MAOP', Y.MAOP, 'var(--alarm)'], ['limit', lim.L, 'var(--alarm)']];
  rungs.sort((a, b) => a[1] - b[1]).forEach(([n, p, c], i) => { const y = clamp(yy(p), ly1 - 4, ly0 + 4), x = lx + i * 112; P.push(`<line x1="${x}" x2="${x + 100}" y1="${y}" y2="${y}" style="stroke:${c};stroke-width:3"/><text x="${x}" y="${y - 4}" style="fill:${c};font:9px var(--mono)">${n} ${fmt(p, 1)}</text>`); });
  P.push('</svg>'); $('#rig').innerHTML = P.join('');
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function axes(ctx, W, H, pad, x0, x1, y0, y1, xl, yl, nx = 6, ny = 4, dx = 1, dy = 0) {
  ctx.fillStyle = css('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.strokeStyle = css('--grid');
  for (let k = 0; k <= nx; k++) { const x = pad.l + k / nx * (W - pad.l - pad.r); ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, H - pad.b); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText((x0 + k / nx * (x1 - x0)).toFixed(dx), x, H - pad.b + 14); }
  for (let k = 0; k <= ny; k++) { const y = H - pad.b - k / ny * (H - pad.t - pad.b); ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText((y0 + k / ny * (y1 - y0)).toFixed(dy), pad.l - 6, y + 4); }
  ctx.textAlign = 'center'; ctx.fillText(xl, (pad.l + W - pad.r) / 2, H - 4); ctx.save(); ctx.translate(12, (pad.t + H - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yl, 0, 0); ctx.restore();
}
const WIN = 3 * 3600;
function drawP() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 110, t: 14, b: 36 }, s = state, Y = sys(), lim = limitOf(Y);
  const t1 = s.t / 3600, t0 = t1 - WIN / 3600, h = s.hist.filter(q => q[0] >= s.t - WIN);
  const ymax = Math.max(lim.L * 1.08, ...h.map(q => q[1])), ymin = Math.max(0, Math.min(s.Sw * 0.85, ...h.map(q => q[1])));
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, t0, t1, ymin, ymax, 'time of day, h', 'outlet, bar', 6, 4, 1, 0);
  const X = t => pad.l + (t / 3600 - t0) / (t1 - t0) * (W - pad.l - pad.r), Yp = p => H - pad.b - (p - ymin) / (ymax - ymin) * (H - pad.t - pad.b);
  const used = []; const hl = (p, c, lb, dash) => { if (p < ymin || p > ymax) return; ctx.strokeStyle = css(c); ctx.setLineDash(dash); ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(pad.l, Yp(p)); ctx.lineTo(W - pad.r, Yp(p)); ctx.stroke(); ctx.setLineDash([]); let ty = Yp(p) + 4; while (used.some(u => Math.abs(u - ty) < 11)) ty -= 11; used.push(ty); ctx.fillStyle = css(c); ctx.textAlign = 'left'; ctx.fillText(`${lb} ${fmt(p, 1)}`, W - pad.r + 4, ty); }; // labels with equal values stack instead of overlapping
  hl(s.Sw, '--good', 'worker', [3, 3]); if (s.hasMon) hl(s.Sm, '--good', 'monitor', [6, 3]); hl(s.Srv, '--brass', 'relief', [3, 3]); if (s.hasSSV) hl(s.Sssv, '--brass', 'slam-shut', [6, 3]); hl(Y.MAOP, '--alarm', 'MAOP', [6, 3]); hl(lim.L, '--alarm', 'limit', []);
  ctx.strokeStyle = css('--water'); ctx.lineWidth = 2.5; ctx.beginPath(); h.forEach((q, i) => i ? ctx.lineTo(X(q[0]), Yp(q[1])) : ctx.moveTo(X(q[0]), Yp(q[1]))); ctx.stroke();
  if (s.inject && s.inject.t >= s.t - WIN) { ctx.strokeStyle = css('--alarm'); ctx.beginPath(); ctx.moveTo(X(s.inject.t), pad.t); ctx.lineTo(X(s.inject.t), H - pad.b); ctx.stroke(); ctx.fillStyle = css('--alarm'); ctx.textAlign = 'left'; ctx.fillText(FAILN[s.inject.kind], X(s.inject.t) + 4, pad.t + 12); }
}
function drawQ() {
  const cv = $('#schart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 64, r: 18, t: 14, b: 36 }, s = state;
  const t1 = s.t / 3600, t0 = t1 - WIN / 3600, h = s.hist.filter(q => q[0] >= s.t - WIN), ym = Math.max(s.Qpk * 1.2, ...h.map(q => Math.max(q[3], q[4]))) / 1000;
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, t0, t1, 0, ym, 'time of day, h', 'flow, 1000 Sm³/h', 6, 4, 1, 0);
  const X = t => pad.l + (t / 3600 - t0) / (t1 - t0) * (W - pad.l - pad.r), Yq = q => H - pad.b - q / 1000 / ym * (H - pad.t - pad.b);
  const line = (k, c, w, dash = []) => { ctx.strokeStyle = css(c); ctx.lineWidth = w; ctx.setLineDash(dash); ctx.beginPath(); h.forEach((q, i) => i ? ctx.lineTo(X(q[0]), Yq(q[k])) : ctx.moveTo(X(q[0]), Yq(q[k]))); ctx.stroke(); ctx.setLineDash([]); };
  line(2, '--steel', 1.5, [4, 3]); line(3, '--water', 2.5); line(4, '--alarm', 2);
}
function drawSizing() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 64, r: 18, t: 14, b: 36 }, s = state, Y = sys(), lim = limitOf(Y);
  const p0 = Math.max(0, s.Srv * 0.9), p1 = lim.L * 1.2, p1a = P1eff(s) + PA;
  const fq = p => qv(1, p1a, p + PA), rq = p => s.rvIso === 'locked' ? s.Crel * clamp((p - s.Srv) / (0.1 * s.Srv), 0, 1) * (p + PA) / (s.Srv * 1.1 + PA) : 0;
  const ym = Math.max(fq(p0), rq(p1), 1000) * 1.1 / 1000;
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, p0, p1, 0, ym, 'outlet pressure, bar', 'flow, 1000 Sm³/h', 6, 4, 1, 0);
  const X = p => pad.l + (p - p0) / (p1 - p0) * (W - pad.l - pad.r), Yq = q => H - pad.b - clamp(q / 1000 / ym, 0, 1.02) * (H - pad.t - pad.b);
  const curve = (fn, c, w) => { ctx.strokeStyle = css(c); ctx.lineWidth = w; ctx.beginPath(); for (let k = 0; k <= 100; k++) { const p = p0 + k / 100 * (p1 - p0); k ? ctx.lineTo(X(p), Yq(fn(p))) : ctx.moveTo(X(p), Yq(fn(p))); } ctx.stroke(); };
  curve(fq, '--alarm', 2.5); curve(rq, '--water', 2.5);
  ctx.strokeStyle = css('--brass'); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(X(lim.L), pad.t); ctx.lineTo(X(lim.L), H - pad.b); ctx.stroke(); ctx.setLineDash([]);
  let pe = null; for (let k = 0; k <= 400; k++) { const p = p0 + k / 400 * (p1 - p0); if (rq(p) >= fq(p)) { pe = p; break; } }
  ctx.fillStyle = css('--ink'); ctx.textAlign = 'left'; ctx.fillText(`one regulator wide open at ${fmt(P1eff(s), 1)} bar inlet, no demand · relief alone ${pe === null ? 'never catches it in this range' : 'holds it at ' + fmt(pe, 1) + ' bar'} · limit ${fmt(lim.L, 1)} (${lim.rule})`, pad.l + 6, pad.t + 12);
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const s = state, Y = sys(), lim = limitOf(Y), T = Tout(s), v = velPeak(), h = (s.t / 3600) % 24;
  $('#roP2').textContent = fmt(s.P2, 2) + ' bar'; $('#roP2Note').textContent = `worker ${fmt(s.Sw, 1)} · MAOP ${Y.MAOP} · limit ${fmt(lim.L, 2)} (${lim.rule})`; $('#roP2Box').classList.toggle('alarm', s.P2 > Y.MAOP);
  $('#roMax').textContent = fmt(s.maxP, 2) + ' bar'; $('#roMaxNote').textContent = s.maxP > lim.L ? 'EXCEEDED the allowed overpressure' : s.maxP > Y.MAOP ? 'above MAOP, within the limit' : 'at or below MAOP'; $('#roMaxBox').classList.toggle('alarm', s.maxP > lim.L);
  $('#roQ').textContent = fmt(s.Q, 0) + ' Sm³/h'; $('#roQNote').textContent = `demand ${fmt(s.Qd, 0)} · relief ${fmt(s.Qrel, 0)} · worker ${fmt(s.yw * 100, 0)} % open${s.hasMon ? ' · monitor ' + fmt(s.ym * 100, 0) + ' %' : ''}`;
  $('#roT').textContent = fmt(T, 1) + ' °C'; $('#roTNote').textContent = `inlet ${s.Tin} °C − ${JT} °C/bar × ${fmt(P1eff(s) - s.P2, 1)} bar let-down${s.heat ? ' · heater holds ≥ 5 °C' : ''}${s.frozen ? ' · WORKER PILOT FROZEN' : ''}`; $('#roTBox').classList.toggle('alarm', T < 0);
  $('#roVel').textContent = fmt(v, 1) + ' m/s'; $('#roVelNote').textContent = `DN ${{ 102.3: 100, 154.1: 150, 202.7: 200, 254.5: 250 }[s.dnOut]} at ${fmt(s.Qpk, 0)} Sm³/h and ${fmt(s.Sw, 1)} bar · limit 30 m/s (100 ft/s)`; $('#roVelBox').classList.toggle('alarm', v > 30);
  const p1a = P1eff(s) + PA, need = qv(1, p1a, lim.L + PA), have = s.Crel * clamp((lim.L - s.Srv) / (0.1 * s.Srv), 0, 1) * (lim.L + PA) / (s.Srv * 1.1 + PA);
  $('#roRel').textContent = `${fmt(have, 0)} of ${fmt(need, 0)} Sm³/h`; $('#roRelNote').textContent = `relief at the limit against one regulator wide open with no demand · ${have >= need ? 'full-capacity relief' : 'token relief: needs a monitor or slam-shut'}`; $('#roRelBox').classList.toggle('alarm', have < need && !s.hasMon && !s.hasSSV);
  $('#roVent').textContent = fmt(s.vented, 0) + ' Sm³'; $('#roVentNote').textContent = 'natural gas released to atmosphere since the scenario began';
  $('#roEvt').textContent = s.inject ? FAILN[s.inject.kind] + ' at ' + fmtClock(s.inject.t) : '—'; $('#roEvtNote').textContent = outcome(); $('#roEvtBox').classList.toggle('alarm', !!s.inject && s.inject.max > lim.L);
  $('#checks').innerHTML = checks().map(c => `<div><b class="${c.ok ? 'y' : 'n'}">${c.ok ? '✓' : '✗'} ${c.id}</b><span>${c.t} <span style="color:var(--muted)">(${c.ref})</span></span></div>`).join('');
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('run', s.run); $('#pillClock').textContent = fmtClock(s.t) + ` · demand ${fmt(demandFrac(s.t) * 100, 0)} %`;
  $('#pillCtl').textContent = s.ssvTripped ? 'No control: slam-shut shut' : s.hasMon && s.ym < 0.98 ? 'Monitor in control' : s.yw >= 0.999 && s.Qrel > 1 ? 'Relief limiting' : 'Worker in control';
  $('#pillOver').classList.toggle('hidden', s.P2 <= lim.L); $('#pillMaop').classList.toggle('hidden', !(s.P2 > Y.MAOP && s.P2 <= lim.L)); $('#pillSSV').classList.toggle('hidden', !s.ssvTripped); $('#pillLow').classList.toggle('hidden', s.P2 >= Y.Pmin); $('#pillVent').classList.toggle('hidden', s.Qrel <= 1);
}
const fmtClock = t => { const h = Math.floor(t / 3600) % 24, m = Math.floor(t / 60) % 60; return `day ${1 + Math.floor(t / 86400)} ${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`; };

/* ------------------------------------------------------------------ log and tasks */
function record(tag = '') {
  const s = state, Y = sys(), lim = limitOf(Y), ck = checks();
  const row = { i: s.log.length + 1, tag, sys: s.sys, maop: Y.MAOP, lim: r3(lim.L), sw: s.Sw, sm: s.hasMon ? s.Sm : '', srv: s.Srv, sssv: s.hasSSV ? s.Sssv : '', crel: s.Crel, iso: s.rvIso, sense: s.sense, max: r3(s.maxP), fail: s.inject ? s.inject.kind : '', out: outcome().replace(/,/g, ';'), vent: r3(s.vented), T: r3(Tout(s)), v: r3(velPeak()), ck: ck.filter(c => c.ok).length + '/9', allOk: ck.every(c => c.ok), over: s.maxP > lim.L, ssv: s.ssvTripped, hasMon: s.hasMon, hasSSV: s.hasSSV };
  s.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  const K = ['i', 'tag', 'sys', 'maop', 'lim', 'sw', 'sm', 'srv', 'sssv', 'crel', 'iso', 'sense', 'max', 'fail', 'out', 'vent', 'T', 'v', 'ck'];
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${K.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,system,MAOP_bar,limit_bar,worker_bar,monitor_bar,relief_bar,slamshut_bar,relief_capacity_Sm3h,relief_isolation,sensing,highest_bar,failure,outcome,vented_Sm3,T_out_C,velocity_ms,checks\n' + state.log.map(r => K.map(k => r[k]).join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
const TASKS = [
  { id: 'day', t: 'A normal day', d: 'Run through a morning and evening peak. Watch the worker open and close while the outlet holds its set point and the monitor waits wide open. Record a reading.', ok: () => state.log.some(r => !r.fail && r.max <= r.maop) },
  { id: 'order', t: 'Set the ladder', d: 'Set worker, monitor, relief and slam-shut so that checks C1 to C4 all pass. Record it.', ok: () => state.log.some(r => r.allOk) },
  { id: 'monitor', t: 'The monitor takes over', d: 'Inject "Worker fails open" with the monitor in service. The outlet settles at the monitor set point, at or below MAOP (845.421).', ok: () => state.log.some(r => r.fail === 'wOpen' && r.hasMon && !r.over && !r.ssv && r.max <= r.maop + 0.05) },
  { id: 'hidden', t: 'The hidden failure', d: 'Inject "Monitor stuck open" (nothing happens), then "Worker fails open". What stops it? Why must monitors be checked (845.421)?', ok: () => state.flags.hiddenSeen && state.flags.ssvTrip },
  { id: 'reliefonly', t: 'Relief valve only', d: 'Remove the monitor and slam-shut. Size the relief so a wide-open worker stays within the limit (use the sizing chart). Inject the failure and record the gas vented.', ok: () => state.log.some(r => r.fail === 'wOpen' && !r.hasMon && !r.hasSSV && !r.over) },
  { id: 'lockup', t: 'Night lock-up', d: 'Inject "Demand to zero". Seat leakage creeps the pressure up; a small token relief deals with it before the slam-shut trips.', ok: () => state.log.some(r => r.fail === 'trip' && !r.ssv && !r.over && r.vent > 0) },
  { id: 'isolated', t: 'Relief left isolated', d: 'Close the relief isolation valve and repeat a failure without a monitor or slam-shut. Record what 845.34 is there to prevent.', ok: () => state.log.some(r => r.iso === 'closed' && r.over) },
  { id: 'shared', t: 'One line, two regulators', d: 'With one shared sensing line, sever it: both regulators open. Repeat with separate lines (845.36(b)).', ok: () => state.log.some(r => r.fail === 'sense' && r.sense === 'shared') && state.log.some(r => r.fail === 'sense' && r.sense === 'sep' && !r.ssv && !r.over) },
  { id: 'freeze', t: 'Cold gas', d: 'Let the gas go cold with the heater off until the worker pilot freezes. Then switch the heater on (841.126(c)).', ok: () => state.flags.frozen },
  { id: 'velocity', t: 'Size the outlet', d: 'Raise the peak demand to 50 000 Sm³/h and choose the smallest outlet pipe that keeps the velocity at or below 30 m/s. Record it.', ok: () => state.log.some(r => r.v <= 30 && state.Qpk >= 50000) },
  { id: 'old', t: 'Four per cent', d: 'On the old spur at 80 % SMYS the limit is MAOP + 4 %. Move the relief and slam-shut so all nine checks pass. Record it.', ok: () => state.log.some(r => r.sys === 'old' && r.allOk) },
  { id: 'reset', t: 'Manual reset', d: 'After a slam-shut trip, repair the fault and reset it by hand (845.222(e)). Record the reading.', ok: () => state.flags.reset },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { if (state.frozen) state.flags.frozen = true; for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ actions and wiring */
function inject(kind = state.fail) { const s = state; if (kind === 'wOpen' && s.faults.mOpen) s.flags.hiddenSeen = true; s.faults[kind] = true; s.inject = { kind, t: s.t, max: s.P2 }; s.ventedSinceInject = 0; s.maxP = s.P2; }
function repair() { const s = state; const was = s.ssvTripped; s.faults = {}; s.frozen = false; if (s.P2 < s.Sssv * 0.98) { s.ssvTripped = false; if (was) s.flags.reset = true; } }
function settle() { const s = state; s.P2 = s.Sw; s.yw = 0.3; s.ym = 1; s.ywi = 0.3; s.ymi = 1; s.ssvTripped = false; s.faults = {}; s.frozen = false; s.freezeT = 0; s.hist = []; s.lastH = -1e9; s.inject = null; s.vented = 0; step(600); s.maxP = s.P2; s.vented = 0; s.hist = s.hist.slice(-5); }
let last = performance.now(), frameN = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; if (state.run) step(dt * state.tscale); frameN++; if (frameN % 2 === 0) { drawRig(); updateReadouts(); if (frameN % 4 === 0) { drawP(); drawQ(); drawSizing(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') e.checked = !!v; else e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function load(sysId, over = {}) {
  $('#sys').value = sysId; state.sys = sysId; const Y = SYS[sysId]; // set directly: the select's change handler calls load()
  const v = Object.assign({ P1: 70, Qpk: sysId === 'dist' ? 25000 : 40000, Tin: 15, heat: true, dnOut: sysId === 'dist' ? 154.1 : 254.5, hasMon: true, hasSSV: true, Crel: 600, rvIso: 'locked', sense: 'sep', tscale: 60 }, Y.sets, over);
  for (const [k, x] of Object.entries(v)) setCtl(k, x);
  state.t = 6 * 3600; settle();
}
const PRESETS = {
  base: () => load('dist'),
  wopen: () => { load('dist'); state.t = 9 * 3600; settle(); inject('wOpen'); },
  hidden: () => { load('dist'); inject('mOpen'); },
  reliefonly: () => { load('dist', { hasMon: false, hasSSV: false, Crel: 600 }); setCtl('fail', 'wOpen'); },
  lockup: () => { load('dist', { hasMon: false }); state.t = 2 * 3600; settle(); inject('trip'); },
  isolated: () => { load('dist', { hasMon: false, hasSSV: false, Crel: 70000, rvIso: 'closed' }); setCtl('fail', 'wOpen'); },
  shared: () => { load('dist', { sense: 'shared' }); setCtl('fail', 'sense'); },
  cold: () => { load('dist', { Tin: 4, heat: false }); },
  old: () => load('old'),
  badsets: () => load('dist', { Sw: 17.5, Sm: 17, Srv: 20, Sssv: 22 }),
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('P1', v => fmt(v, 1) + ' bar'); bindR('Qpk', v => v.toLocaleString('en-ZA') + ' Sm³/h'); bindR('Tin', v => v + ' °C'); bindR('Crel', v => v.toLocaleString('en-ZA') + ' Sm³/h'); bindR('tscale', v => '×' + v);
  for (const id of ['Sw', 'Sm', 'Sssv', 'Srv']) bindR(id, v => fmt(v, 1));
  for (const [id, num] of [['sys', 0], ['dnOut', 1], ['rvIso', 0], ['sense', 0], ['fail', 0]]) { const e = $('#' + id); const set = () => { state[id] = num ? +e.value : e.value; }; e.addEventListener('change', set); set(); }
  $('#sys').addEventListener('change', () => load(state.sys));
  for (const id of ['heat', 'hasMon', 'hasSSV']) { const e = $('#' + id); const set = () => { state[id] = e.checked; }; e.addEventListener('change', set); set(); }
  $('#btnInject').onclick = () => inject(); $('#btnRepair').onclick = repair; $('#btnClearMax').onclick = () => { state.maxP = state.P2; if (state.inject) state.inject.max = state.P2; };
  $('#btnPause').onclick = () => state.run = !state.run; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { const done = () => { $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); }; try { navigator.clipboard.writeText(state.csv || '').then(done, () => { $('#btnCopy').textContent = 'Use Download CSV'; }); } catch (e) { $('#btnCopy').textContent = 'Use Download CSV'; } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tk_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'I') inject(); if (k === 'X') repair(); if (k === 'R') record(''); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.PressureStation = {
  getState: () => { const s = state, Y = sys(), lim = limitOf(Y); return {
    inputs: { sys: s.sys, P1: s.P1, Qpk: s.Qpk, Tin: s.Tin, heat: s.heat, dnOut: s.dnOut, Sw: s.Sw, Sm: s.Sm, Sssv: s.Sssv, Srv: s.Srv, Crel: s.Crel, hasMon: s.hasMon, hasSSV: s.hasSSV, rvIso: s.rvIso, sense: s.sense, tscale: s.tscale, run: s.run, faults: Object.keys(s.faults) },
    results: { P2: r3(s.P2), maxP: r3(s.maxP), MAOP: Y.MAOP, limit: r3(lim.L), limitRule: lim.rule, hoopPct: r3(hoopPct(Y)), Q: r3(s.Q), Qd: r3(s.Qd), Qrel: r3(s.Qrel), yw: r3(s.yw), ym: r3(s.ym), Pm: r3(s.Pm), ssvTripped: s.ssvTripped, frozen: s.frozen, Tout: r3(Tout(s)), velocity: r3(velPeak()), vented: r3(s.vented), outcome: outcome(), checks: checks().map(c => ({ id: c.id, ok: c.ok })) },
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }; },
  setState: inp => {
    if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset]();
    if (inp.sys && SYS[inp.sys]) load(inp.sys);
    for (const k of ['P1', 'Qpk', 'Tin', 'heat', 'dnOut', 'Sw', 'Sm', 'Sssv', 'Srv', 'Crel', 'hasMon', 'hasSSV', 'rvIso', 'sense', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]);
    if (inp.settle) settle(); if (inp.inject && FAILN[inp.inject]) inject(inp.inject); if (inp.repair) repair();
    if (inp.advance) step(Math.min(+inp.advance, 2 * 86400));
    if (inp.record) record(inp.record === true ? '' : String(inp.record)); if (inp.run !== undefined) state.run = !!inp.run;
  },
  meta: { id: 'pipeline-overpressure', title: 'Pressure Reduction Station Simulator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PressureStation.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'pipeline-overpressure', state: window.PressureStation.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function boot() {
  try { const th = localStorage.getItem('tk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildTasks(); bind(); load('dist'); renderLog(); drawRig(); updateReadouts(); drawP(); drawQ(); drawSizing(); requestAnimationFrame(tick);
}
boot();
