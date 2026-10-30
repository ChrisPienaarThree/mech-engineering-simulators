/* Brake and Clutch Demonstrator — disc brake, drum brake and plate clutch. Actuating pressure to clamp force, effective
   friction radius (uniform wear or uniform pressure), lining pressure against the allowable, torque capacity, the stop or
   engagement transient integrated on the inertia, energy absorbed and temperature rise per event on a thermal clock, fade
   as the lining heats, drum self-energising, clutch slip time and lock-up. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const MAT = { organic: { name: 'Organic (resin bonded)', mu: 0.40, Tfade: 300, pmax: 1.0 }, semi: { name: 'Semi-metallic', mu: 0.38, Tfade: 450, pmax: 1.6 }, sintered: { name: 'Sintered metallic', mu: 0.32, Tfade: 600, pmax: 3.0 }, ceramic: { name: 'Ceramic composite', mu: 0.36, Tfade: 550, pmax: 2.0 } };
const C_ROT = 480, KLEV = 1.6, QSE = 1.25, SHOE_ARC = 100 * Math.PI / 180, T_ALARM = 400, T_FAIL = 750, EV_MAX = 60;

/* ------------------------------------------------------------------ state */
const state = { mode: 'disc', p: 40, dp: 44, Fs: 4, plates: 1, ro: 140, ri: 100, pad: 100, dd: 250, w: 50, mat: 'organic', dist: 'wear', J: 20, n0: 1500, ne: 2000, Teng: 150, Je: 0.4, Jl: 1, TL: 60, ta: 0.6, mrot: 6, cool: 1, Tamb: 25, repeat: 20, tscale: 1, auto: false, run: true,
  T: 25, phase: 'idle', w1: 0, w2: 0, tph: 0, ev: null, lastEv: null, events: 0, t: 0, trace: [], traceAcc: 0, log: [], flags: {}, ang1: 0, ang2: 0, Ppk: 0, failed: false };
const R = {};

/* ------------------------------------------------------------------ model */
function muOf(T, mat = state.mat) { const m = MAT[mat]; return m.mu * (0.35 + 0.65 / (1 + Math.exp((T - m.Tfade) / 45))); }
function solve(muUse) {
  const s = state, m = MAT[s.mat], mu0 = m.mu, mu = muUse !== undefined ? muUse : muOf(s.T);
  const clutch = s.mode === 'clutch', F = clutch ? s.Fs * 1000 : s.p * 1e5 * Math.PI * (s.dp / 2000) ** 2, faces = clutch ? 2 * s.plates : 2;
  let out = { mu, mu0, fade: mu / mu0, F, faces, pallow: m.pmax, selflock: false };
  if (s.mode === 'drum') {
    const r = s.dd / 2, A = s.w * r * SHOE_ARC, den = 1 - mu * QSE, den0 = 1 - mu0 * QSE, dn = Math.max(den, 0.05), NL = F * KLEV / dn, NT = F * KLEV / (1 + mu * QSE);
    const Tq = mu * r / 1000 * (NL + NT), Tq0 = mu0 * r / 1000 * (F * KLEV / Math.max(den0, 0.05) + F * KLEV / (1 + mu0 * QSE));
    Object.assign(out, { Re: r, A: 2 * A, pmax: NL / A, pmean: (NL + NT) / 2 / A, Tq, Tq0, BF: Tq / (F * r / 1000), NL, NT, selflock: den <= 0.05, TqL: mu * r / 1000 * NL, TqT: mu * r / 1000 * NT });
  } else {
    const ro = s.ro, ri = Math.min(s.ri, s.ro - 10), Re = s.dist === 'wear' ? (ro + ri) / 2 : 2 / 3 * (ro ** 3 - ri ** 3) / (ro ** 2 - ri ** 2), th = clutch ? 2 * Math.PI : s.pad * Math.PI / 180, A = th / 2 * (ro * ro - ri * ri);
    const pmean = F / A, pmax = s.dist === 'wear' ? F / (th * ri * (ro - ri)) : pmean, Tq = mu * F * Re / 1000 * faces, Tq0 = mu0 * F * Re / 1000 * faces;
    Object.assign(out, { Re, ri, A, pmean, pmax, Tq, Tq0, BF: Tq / (F * Re / 1000) });
  }
  const w0 = s.n0 * 2 * Math.PI / 60;
  if (!clutch) { const E = 0.5 * s.J * w0 * w0; Object.assign(out, { w0, E, dT: E / (s.mrot * C_ROT), tsEst: s.J * w0 / Math.max(out.Tq, 1e-6) + 0.15, decel: out.Tq / s.J, Pavg: E / (s.J * w0 / Math.max(out.Tq, 1e-6) + 0.15) }); }
  else { const we = s.ne * 2 * Math.PI / 60, den = (out.Tq - s.Teng) / s.Je + (out.Tq - s.TL) / s.Jl, est = clutchEstimate(out.Tq, we); Object.assign(out, { w0: we, E: est.E, dT: est.E / (s.mrot * C_ROT), tsEst: est.ts, decel: (out.Tq - s.TL) / s.Jl, Pavg: est.E / est.ts, holds: out.Tq > s.TL, canLock: den > 0 && est.locked, stalls: est.stalled }); }
  out.overp = out.pmax > m.pmax; return out;
}
function clutchEstimate(Tcap, we) { const s = state, h = 0.002; let w1 = we, w2 = 0, t = 0, E = 0; while (t < 10) { const Tq = Tcap * Math.min(1, t / Math.max(s.ta, 0.05)), slip = w1 - w2; if (slip <= 0) return { ts: t, E, locked: true, stalled: false }; if (w1 < 30) return { ts: t, E, locked: false, stalled: true }; E += Tq * slip * h; w1 = Math.min(1.3 * we, w1 + (s.Teng - Tq) / s.Je * h); w2 = Math.max(0, w2 + (Tq - s.TL) / s.Jl * h); t += h; } return { ts: Infinity, E: Infinity, locked: false, stalled: false }; }
function startEvent() { const s = state; s.phase = s.mode === 'clutch' ? 'engaging' : 'braking'; s.tph = 0; s.Ppk = 0; s.ev = { T0: s.T, E: 0, pts: [], acc: 0, ts: null, locked: false, note: '', mode: s.mode }; }
function endEvent(note) { const s = state, ev = s.ev; if (!ev) return; if (ev.ts === null) ev.ts = s.tph; ev.dT = s.T - ev.T0; ev.note = note; ev.Ppk = s.Ppk; ev.tEnd = s.tph; s.lastEv = ev; s.ev = null; s.events++; s.phase = s.mode === 'clutch' ? 'released' : 'stopped'; s.tph = 0; if (s.mode !== 'clutch') s.flags.firststop = true; if (note === 'locked') s.flags.clutch = true; if (note === 'slipping continuously') s.flags.burn = true; if (note === 'stalled') s.flags.stall = true; record(note || (s.mode === 'clutch' ? 'engaged' : 'stop')); }
function sub(h) {
  const s = state, r = solve(), w0 = r.w0; let Pheat = 0, Tq = 0; s.tph += h; s.t += h;
  if (s.mode !== 'clutch') {
    if (s.phase === 'idle') { s.w1 = w0; if (s.auto && s.run && s.tph > 0.5) startEvent(); }
    else if (s.phase === 'braking') { Tq = r.Tq * Math.min(1, s.tph / 0.3); const dw = Tq / s.J * h; Pheat = Tq * Math.max(0, s.w1 - dw / 2); if (s.w1 - dw <= 0) { s.w1 = 0; endEvent(''); } else { s.w1 -= dw; if (s.tph > EV_MAX) endEvent('did not stop in 60 s'); } if (r.fade < 0.8) s.flags.fade = true; }
    else if (s.phase === 'stopped') { s.w1 = 0; if (s.tph >= (s.auto ? Math.max(2, s.repeat - 3) : 2)) { s.phase = 'spinup'; s.tph = 0; } }
    else if (s.phase === 'spinup') { s.w1 = Math.min(w0, s.w1 + w0 / 3 * h); if (s.w1 >= w0) { s.phase = 'idle'; s.tph = 0; } }
    s.w2 = 0;
  } else {
    const we = s.ne * 2 * Math.PI / 60;
    if (s.phase === 'idle' || s.phase === 'released') { s.w1 += (we - s.w1) * Math.min(1, 3 * h); s.w2 = Math.max(0, s.w2 - s.TL / s.Jl * h); if (s.phase === 'released' && s.tph > 2) { s.phase = 'idle'; s.tph = 0; } if (s.phase === 'idle' && s.auto && s.run && s.tph > s.repeat) startEvent(); }
    else if (s.phase === 'engaging') {
      const ev = s.ev, cap = r.Tq * Math.min(1, s.tph / Math.max(s.ta, 0.05));
      if (!ev.locked) { const slip = s.w1 - s.w2; Tq = cap; const w1n = Math.min(1.3 * we, s.w1 + (s.Teng - Tq) / s.Je * h), w2n = s.w2 + (Tq - s.TL) / s.Jl * h; Pheat = Tq * Math.max(0, slip); if (w1n <= w2n) { const w = (s.Je * s.w1 + s.Jl * s.w2) / (s.Je + s.Jl); s.w1 = s.w2 = w; ev.locked = true; ev.ts = s.tph; } else { s.w1 = w1n; s.w2 = Math.max(0, w2n); } if (s.tph > 10) endEvent('slipping continuously'); }
      if (ev && ev.locked) { const a = (s.Teng - s.TL) / (s.Je + s.Jl), Tneed = s.Jl * a + s.TL; if (Tneed > cap) { ev.locked = false; s.w1 += 0.5; } else { Tq = Tneed; const wn = Math.min(1.3 * we, s.w1 + a * h); s.w1 = s.w2 = wn; if (s.tph - ev.ts > 3) endEvent('locked'); } }
      if (s.w1 < 30) endEvent('stalled');
      if (r.fade < 0.8) s.flags.fade = true;
    }
  }
  const kc = { disc: 4, drum: 1.6, clutch: 2 }[s.mode] * s.cool, nrpm = s.w1 * 60 / (2 * Math.PI), Pc = kc * (1 + 0.01 * nrpm) * (s.T - s.Tamb);
  s.T += (Pheat - Pc) / (s.mrot * C_ROT) * h; if (Pheat > s.Ppk) s.Ppk = Pheat;
  if (s.ev) { s.ev.E += Pheat * h; s.ev.acc += h; const iv = s.tph > 12 ? 0.1 : 0.02; if (s.ev.acc >= iv) { s.ev.acc = 0; s.ev.pts.push({ t: s.tph, w1: s.w1, w2: s.w2, Tq, T: s.T }); } }
  if (s.T > T_FAIL && !s.failed) { s.failed = true; s.flags.failed = true; record('lining destroyed'); }
  s.ang1 += s.w1 * h * 0.08 * 180 / Math.PI; s.ang2 += s.w2 * h * 0.08 * 180 / Math.PI;
}
function step(dtReal) {
  const s = state; if (s.run) { const dts = dtReal * s.tscale, n = Math.max(1, Math.ceil(dts / 0.01)), h = dts / n; for (let i = 0; i < n; i++) sub(h); }
  Object.assign(R, solve()); if (R.overp) s.flags.overp = true; if (R.selflock) s.flags.selflock = true;
  s.traceAcc += dtReal * s.tscale; if (s.traceAcc >= 0.25) { s.traceAcc = 0; s.trace.push({ t: s.t, T: s.T, fade: R.fade * 100, n1: s.w1 * 60 / (2 * Math.PI), ev: s.phase === 'braking' || s.phase === 'engaging' }); if (s.trace.length > 1600) s.trace.shift(); }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg;
function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Brake or clutch rig' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
const heat = T => { const f = clamp((T - 60) / 600, 0, 1); return `rgba(${Math.round(150 + 90 * f)},${Math.round(160 - 120 * f)},${Math.round(175 - 150 * f)},1)`; };
const arrow = (x1, y1, x2, y2, cls, p) => { el('line', { x1, y1, x2, y2, class: cls }, p); const a = Math.atan2(y2 - y1, x2 - x1); el('polygon', { points: `${x2},${y2} ${x2 - 10 * Math.cos(a - 0.4)},${y2 - 10 * Math.sin(a - 0.4)} ${x2 - 10 * Math.cos(a + 0.4)},${y2 - 10 * Math.sin(a + 0.4)}`, class: cls.replace('arrow', 'head') }, p); };
const sector = (cx, cy, r1, r2, a1, a2) => `M${cx + r1 * Math.cos(a1)},${cy + r1 * Math.sin(a1)} L${cx + r2 * Math.cos(a1)},${cy + r2 * Math.sin(a1)} A${r2},${r2} 0 ${a2 - a1 > Math.PI ? 1 : 0} 1 ${cx + r2 * Math.cos(a2)},${cy + r2 * Math.sin(a2)} L${cx + r1 * Math.cos(a2)},${cy + r1 * Math.sin(a2)} A${r1},${r1} 0 ${a2 - a1 > Math.PI ? 1 : 0} 0 ${cx + r1 * Math.cos(a1)},${cy + r1 * Math.sin(a1)} Z`;
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, hot = s.T > T_ALARM, n1 = s.w1 * 60 / (2 * Math.PI), n2 = s.w2 * 60 / (2 * Math.PI), active = s.phase === 'braking' || s.phase === 'engaging';
  if (s.mode === 'disc' || s.mode === 'drum') {
    const cx = 300, cy = 190, Ro = s.mode === 'disc' ? s.ro : s.dd / 2, k = 140 / Ro, ro = Ro * k, ri = s.mode === 'disc' ? Math.min(s.ri, s.ro - 10) * k : ro - 16;
    el('circle', { cx, cy, r: ro, class: 'rotor', fill: heat(s.T) }, svg); el('circle', { cx, cy, r: s.mode === 'disc' ? ri * 0.55 : ro * 0.35, class: 'hub' }, svg);
    const a = s.ang1 * Math.PI / 180; for (let i = 0; i < 3; i++) { const aa = a + i * 2 * Math.PI / 3; el('line', { x1: cx + ro * 0.3 * Math.cos(aa), y1: cy + ro * 0.3 * Math.sin(aa), x2: cx + ro * 0.9 * Math.cos(aa), y2: cy + ro * 0.9 * Math.sin(aa), class: 'mark', opacity: 0.5 }, svg); }
    if (s.mode === 'disc') {
      const th = s.pad * Math.PI / 180, a1 = -Math.PI / 2 - th / 2, a2 = -Math.PI / 2 + th / 2; el('path', { d: sector(cx, cy, ri, ro, a1, a2), class: 'pad' + (hot ? ' hot' : '') }, svg);
      el('rect', { x: cx - 46, y: cy - ro - 40 - (active ? 0 : 8), width: 92, height: 34, rx: 6, class: 'caliper' }, svg); el('rect', { x: cx - 16, y: cy - ro - 12 - (active ? 0 : 8), width: 32, height: 10, class: 'piston' }, svg);
      txt(cx, cy - ro - 48 - (active ? 0 : 8), `caliper · p ${s.p} bar · Ø${s.dp} piston`, 'lbl', svg, 'middle');
      if (active) { arrow(cx + 70, cy - ro - 44, cx + 70, cy - ro - 4, 'farrow', svg); txt(cx + 78, cy - ro - 24, `F ${fmt(r.F, 0)} N`, 'ftxt', svg); }
      el('line', { x1: cx, y1: cy, x2: cx + ri * Math.cos(-0.6), y2: cy + ri * Math.sin(-0.6), class: 'mark', 'stroke-dasharray': '3 3' }, svg); txt(cx + 22, cy - 6, `Re ${fmt(r.Re, 0)} mm`, 'val', svg);
      txt(cx, cy + ro + 20, `disc Ø${2 * s.ro} · pad ${s.ri}–${s.ro} mm · ${s.pad}° arc · ${MAT[s.mat].name}`, 'lbl', svg, 'middle');
    } else {
      const rs = ro - 8, arcs = [[-Math.PI / 2 - 0.35 - SHOE_ARC, -Math.PI / 2 - 0.35, 'leading'], [-Math.PI / 2 + 0.35, -Math.PI / 2 + 0.35 + SHOE_ARC, 'trailing']];
      for (const [a1, a2, nm] of arcs) { const off = active ? 0 : 4, rr = rs - off; el('path', { d: `M${cx + rr * Math.cos(a1)},${cy + rr * Math.sin(a1)} A${rr},${rr} 0 0 1 ${cx + rr * Math.cos(a2)},${cy + rr * Math.sin(a2)}`, class: 'shoe' + (hot ? ' hot' : '') }, svg); const am = (a1 + a2) / 2; txt(cx + (rr - 30) * Math.cos(am), cy + (rr - 30) * Math.sin(am) + 4, nm, 'lbl', svg, 'middle'); }
      el('circle', { cx: cx - 30, cy: cy + rs - 6, r: 5, class: 'pivot' }, svg); el('circle', { cx: cx + 30, cy: cy + rs - 6, r: 5, class: 'pivot' }, svg); txt(cx, cy + rs - 2, 'pivots', 'lbl', svg, 'middle');
      el('rect', { x: cx - 34, y: cy - rs + 6, width: 68, height: 20, rx: 4, class: 'cyl' }, svg); txt(cx, cy - rs + 40, `wheel cylinder · p ${s.p} bar · Ø${s.dp}`, 'lbl', svg, 'middle');
      if (active) { arrow(cx - 40, cy - rs + 16, cx - 80, cy - rs + 16, 'farrow', svg); arrow(cx + 40, cy - rs + 16, cx + 80, cy - rs + 16, 'farrow', svg); txt(cx, cy - rs - 12, `Fa ${fmt(r.F, 0)} N on each shoe`, 'ftxt', svg, 'middle'); txt(cx - 100, cy - 20, `N ${fmt(r.NL / 1000, 1)} kN`, 'val', svg, 'middle'); txt(cx + 100, cy - 20, `N ${fmt(r.NT / 1000, 1)} kN`, 'val', svg, 'middle'); }
      txt(cx, cy + ro + 20, `drum Ø${s.dd} · shoes ${s.w} mm wide × 100° · leading-trailing · ${MAT[s.mat].name}`, 'lbl', svg, 'middle');
      const ad = s.ang1 * Math.PI / 180 + 0.9; arrow(cx + (ro + 18) * Math.cos(ad - 0.5), cy + (ro + 18) * Math.sin(ad - 0.5), cx + (ro + 18) * Math.cos(ad), cy + (ro + 18) * Math.sin(ad), 'tarrow', svg);
    }
    el('line', { x1: cx + ro + 16, y1: cy, x2: 640, y2: cy, class: 'shaft' }, svg); el('rect', { x: 640, y: cy - 60, width: 130, height: 120, rx: 8, class: 'motor' }, svg); txt(705, cy - 30, 'flywheel / vehicle', 'lbl', svg, 'middle'); txt(705, cy - 8, `J ${s.J} kg·m²`, 'val', svg, 'middle'); txt(705, cy + 14, `${fmt(n1, 0)} rpm`, 'ftxt', svg, 'middle'); txt(705, cy + 36, `E ${fmt(r.E / 1000, 0)} kJ at ${s.n0} rpm`, 'val', svg, 'middle');
    txt(705, cy + 50, s.phase === 'braking' ? 'BRAKING' : s.phase === 'stopped' ? 'stopped' : s.phase === 'spinup' ? 'spinning up' : 'running', 'lbl', svg, 'middle');
  } else {
    const cy = 190, gap = active || s.phase === 'released' && s.tph < 0.3 ? 0 : 26, kh = 110;
    el('line', { x1: 60, y1: cy, x2: 250, y2: cy, class: 'shaft' }, svg); el('rect', { x: 60, y: cy - 50, width: 110, height: 100, rx: 8, class: 'motor' }, svg); txt(115, cy - 26, 'engine', 'lbl', svg, 'middle'); txt(115, cy - 4, `${fmt(n1, 0)} rpm`, 'ftxt', svg, 'middle'); txt(115, cy + 16, `Je ${s.Je} kg·m²`, 'val', svg, 'middle'); txt(115, cy + 34, `${s.Teng} N·m`, 'val', svg, 'middle');
    el('rect', { x: 250, y: cy - kh, width: 34, height: 2 * kh, rx: 3, class: 'plate', fill: heat(s.T) }, svg); txt(267, cy + kh + 16, 'flywheel', 'lbl', svg, 'middle');
    for (let i = 0; i < s.plates; i++) { const x = 290 + gap + i * (16 + 30); el('rect', { x, y: cy - kh * 0.9, width: 12, height: 1.8 * kh, class: 'fdisc' + (hot ? ' hot' : '') }, svg); if (i < s.plates - 1) { el('rect', { x: x + 12, y: cy - kh, width: 30, height: 2 * kh, rx: 3, class: 'plate', fill: heat(s.T) }, svg); } }
    const xp = 290 + gap + s.plates * 46 - 30 + 12; el('rect', { x: xp, y: cy - kh, width: 30, height: 2 * kh, rx: 3, class: 'plate', fill: heat(s.T) }, svg); txt(xp + 15, cy + kh + 16, 'pressure plate', 'lbl', svg, 'middle');
    for (const dy of [-70, 0, 70]) { let d = `M${xp + 30},${cy + dy}`; for (let i = 0; i < 6; i++) d += ` l6,${i % 2 ? 6 : -6}`; d += ` l6,0`; el('path', { d, class: 'spring' }, svg); } el('rect', { x: xp + 72, y: cy - kh, width: 12, height: 2 * kh, class: 'plate' }, svg);
    if (active) { arrow(xp + 130, cy - 90, xp + 90, cy - 90, 'farrow', svg); txt(xp + 134, cy - 86, `F ${fmt(r.F / 1000, 1)} kN`, 'ftxt', svg); }
    el('line', { x1: 302 + gap, y1: cy, x2: 640, y2: cy, class: 'shaft' }, svg); el('rect', { x: 640, y: cy - 50, width: 130, height: 100, rx: 8, class: 'load' }, svg); txt(705, cy - 26, 'gearbox / load', 'lbl', svg, 'middle'); txt(705, cy - 4, `${fmt(n2, 0)} rpm`, 'ftxt', svg, 'middle'); txt(705, cy + 16, `Jl ${s.Jl} kg·m²`, 'val', svg, 'middle'); txt(705, cy + 34, `load ${s.TL} N·m`, 'val', svg, 'middle');
    txt(480, cy + kh + 40, `${s.plates} friction plate${s.plates > 1 ? 's' : ''} (${r.faces} faces) · ${s.ri}–${s.ro} mm · Re ${fmt(r.Re, 0)} mm · ${MAT[s.mat].name}`, 'lbl', svg, 'middle');
    txt(480, cy - kh - 20, s.phase === 'engaging' ? (s.ev && s.ev.locked ? 'LOCKED — no slip' : `SLIPPING · ${fmt(n1 - n2, 0)} rpm slip`) : s.phase === 'released' ? 'released' : 'disengaged', 'ftxt', svg, 'middle');
  }
  const tb = 830; el('rect', { x: tb, y: 40, width: 26, height: 300, rx: 4, class: 'tbar' }, svg); const hT = clamp((s.T - 0) / 800, 0, 1) * 300; el('rect', { x: tb, y: 340 - hT, width: 26, height: hT, class: 'tfill' + (hot ? ' bad' : '') }, svg);
  const yF = 340 - clamp(MAT[s.mat].Tfade / 800, 0, 1) * 300; el('line', { x1: tb - 6, y1: yF, x2: tb + 32, y2: yF, class: 'farrow', 'stroke-dasharray': '4 3' }, svg); txt(tb + 36, yF + 4, 'fade', 'lbl', svg); txt(tb + 13, 30, 'lining °C', 'lbl', svg, 'middle'); txt(tb + 13, 358, `${fmt(s.T, 0)} °C`, 'ftxt', svg, 'middle'); txt(tb + 36, 340 - 300 * 0.5 + 4, '400', 'lbl', svg); txt(tb + 36, 44, '800', 'lbl', svg);
  const mb = 900; el('rect', { x: mb, y: 40, width: 26, height: 300, rx: 4, class: 'tbar' }, svg); const hM = clamp(r.fade, 0, 1) * 300; el('rect', { x: mb, y: 340 - hM, width: 26, height: hM, fill: r.fade < 0.8 ? 'var(--alarm)' : 'var(--good)' }, svg); txt(mb + 13, 30, 'μ / μ cold', 'lbl', svg, 'middle'); txt(mb + 13, 358, `${fmt(r.fade * 100, 0)} %`, 'ftxt', svg, 'middle');
  const ev = s.lastEv; txt(400, 372, ev ? (ev.mode === 'clutch' ? `last engagement · slip time ${fmt(ev.ts, 2)} s · energy ${fmt(ev.E / 1000, 1)} kJ · ΔT ${fmt(ev.dT, 0)} K · peak ${fmt(ev.Ppk / 1000, 0)} kW${ev.note && ev.note !== 'locked' ? ' · ' + ev.note.toUpperCase() : ''}` : `last stop · ${fmt(ev.ts, 2)} s · energy ${fmt(ev.E / 1000, 1)} kJ · ΔT ${fmt(ev.dT, 0)} K · peak ${fmt(ev.Ppk / 1000, 0)} kW${ev.note ? ' · ' + ev.note.toUpperCase() : ''}`) : 'no event yet — press Apply brake / Engage clutch', 'val' + (ev && ev.note && ev.note !== 'locked' ? ' bad' : ''), svg, 'middle');
  txt(400, 392, s.failed ? 'LINING DESTROYED — temperature above 750 °C · Reset the rig' : r.selflock ? 'DRUM SELF-LOCKING — leading shoe friction moment exceeds the actuation moment' : r.overp ? `LINING PRESSURE ${fmt(r.pmax, 2)} MPa ABOVE ALLOWABLE ${MAT[s.mat].pmax} MPa — larger pads or lower pressure` : r.fade < 0.8 ? `FADE — friction down to ${fmt(r.fade * 100, 0)} % of cold, torque capacity ${fmt(r.Tq, 0)} N·m against ${fmt(r.Tq0, 0)} cold` : s.mode === 'clutch' && !r.holds ? `CAPACITY ${fmt(r.Tq, 0)} N·m BELOW LOAD ${s.TL} N·m — the clutch cannot lock` : `torque capacity ${fmt(r.Tq, 0)} N·m · ${s.mode === 'clutch' ? 'estimated slip time ' + fmt(r.tsEst, 2) + ' s' : 'estimated stop ' + fmt(r.tsEst, 2) + ' s · ' + fmt(r.decel, 0) + ' rad/s²'} · ΔT per event ${fmt(r.dT, 0)} K`, 'val' + (s.failed || r.selflock || r.overp || r.fade < 0.8 || (s.mode === 'clutch' && !r.holds) ? ' bad' : ''), svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawChart() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 64, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, ev = s.ev || s.lastEv;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  if (!ev || ev.pts.length < 2) { ctx.textAlign = 'center'; ctx.fillStyle = col('--muted'); ctx.font = '600 13px Barlow Condensed, sans-serif'; ctx.fillText('The transient of the last stop or engagement appears here', L + pw / 2, T + ph / 2); return; }
  const pts = ev.pts, tmax = Math.max(1, pts[pts.length - 1].t), X = t => L + t / tmax * pw, nmax = Math.max(100, ...pts.map(p => p.w1 * 60 / (2 * Math.PI))) * 1.1, tqmax = Math.max(10, ...pts.map(p => p.Tq)) * 1.15, Tmax = Math.max(100, ...pts.map(p => p.T)) * 1.1;
  const Yn = n => T + (1 - clamp(n / nmax, 0, 1)) * ph, Yq = q => T + (1 - clamp(q / tqmax, 0, 1)) * ph, YT = t => T + (1 - clamp(t / Tmax, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); const tstep = tmax > 20 ? 5 : tmax > 8 ? 2 : tmax > 3 ? 1 : 0.5; for (let t = 0; t <= tmax + 1e-9; t += tstep) { ctx.beginPath(); ctx.moveTo(X(t), T); ctx.lineTo(X(t), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(fmt(t, tstep < 1 ? 1 : 0) + ' s', X(t), H - 18); }
  ctx.textAlign = 'right'; ctx.fillStyle = col('--water'); for (let f = 0; f <= 1; f += 0.25) ctx.fillText(fmt(nmax * f, 0), L - 4, Yn(nmax * f) + 3); ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); for (let f = 0; f <= 1; f += 0.5) ctx.fillText(fmt(tqmax * f, 0) + ' N·m', L + pw + 4, Yq(tqmax * f) + 3); ctx.fillStyle = col('--brass'); ctx.fillText(fmt(Tmax, 0) + ' °C', L + pw + 4, YT(Tmax) + 14);
  const line = (key, color, Y, w = 2, dash = []) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); let f = true; for (const p of pts) { f ? ctx.moveTo(X(p.t), Y(p[key])) : ctx.lineTo(X(p.t), Y(p[key])); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  line('w1', col('--water'), v => Yn(v * 60 / (2 * Math.PI)), 2.4); if (ev.mode === 'clutch') line('w2', col('--water-deep'), v => Yn(v * 60 / (2 * Math.PI)), 2.4, [6, 3]); line('Tq', col('--alarm'), Yq, 1.8); line('T', col('--brass'), YT, 1.8, [2, 3]);
  if (ev.ts !== null && ev.ts !== undefined) { ctx.strokeStyle = col('--good'); ctx.lineWidth = 1.5; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(X(ev.ts), T); ctx.lineTo(X(ev.ts), T + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--good'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = X(ev.ts) > L + pw * 0.7 ? 'right' : 'left'; ctx.fillText((ev.mode === 'clutch' ? 'lock-up · slip time ' : 'stopped · ') + fmt(ev.ts, 2) + ' s', X(ev.ts) + (X(ev.ts) > L + pw * 0.7 ? -6 : 6), T + 14); }
  ctx.fillStyle = col('--ink'); ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.textAlign = 'left'; ctx.fillText(`energy ${fmt(ev.E / 1000, 1)} kJ · ΔT ${fmt((s.ev ? s.T - ev.T0 : ev.dT), 0)} K · peak ${fmt((s.ev ? s.Ppk : ev.Ppk) / 1000, 0)} kW`, L + 6, T + ph - 8);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`time from ${ev.mode === 'clutch' ? 'pedal release' : 'brake application'} · left axis rpm · right axis torque N·m and lining °C`, L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, span = 240, tr = state.trace.filter(p => p.t > state.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const X = t => L + (t - (state.t - span)) / span * pw, Tmax = Math.max(400, ...tr.map(p => p.T)) * 1.05;
  ctx.fillStyle = 'rgba(200,60,40,.10)'; for (const p of tr) if (p.ev) ctx.fillRect(X(p.t) - 1, T, pw / span * 0.25 + 1, ph);
  const series = (key, color, lo, hi, w = 1.6) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { const y = T + (1 - clamp((p[key] - lo) / (hi - lo), 0, 1)) * ph; f ? ctx.moveTo(X(p.t), y) : ctx.lineTo(X(p.t), y); f = false; } ctx.stroke(); };
  series('T', col('--brass'), 0, Tmax, 2.2); series('fade', col('--good'), 0, 100); series('n1', col('--water'), 0, Math.max(state.n0, state.ne) * 1.1, 1.2);
  const yF = T + (1 - MAT[state.mat].Tfade / Tmax) * ph; ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L, yF); ctx.lineTo(L + pw, yF); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`${fmt(Tmax, 0)} °C`, L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); ctx.fillText('lining °C', L + pw + 4, T + 10); ctx.fillStyle = col('--good'); ctx.fillText('μ % cold', L + pw + 4, T + 24); ctx.fillStyle = col('--water'); ctx.fillText('rpm', L + pw + 4, T + 38); ctx.fillStyle = col('--alarm'); ctx.fillText('fade °C', L + pw + 4, yF + 3);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s of simulated time · shaded = brake or clutch applied`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, m = MAT[s.mat], clutch = s.mode === 'clutch';
  $('#roF').textContent = fmt(r.F / 1000, 2); $('#roFNote').textContent = clutch ? 'spring clamp force' : `${s.p} bar on Ø${s.dp} mm${s.mode === 'drum' ? ' per shoe' : ''}`;
  $('#roRe').textContent = fmt(r.Re, 1); $('#roReNote').textContent = s.mode === 'drum' ? 'drum radius' : s.dist === 'wear' ? 'uniform wear (r₁+r₂)/2' : 'uniform pressure ⅔(r₂³−r₁³)/(r₂²−r₁²)';
  $('#roP').textContent = fmt(r.pmax, 2); $('#roPallow').textContent = fmt(m.pmax, 1); $('#roPBox').classList.toggle('alarm', r.overp); $('#roPmean').textContent = fmt(r.pmean, 2);
  $('#roTq').textContent = fmt(r.Tq, 0); $('#roTq0').textContent = fmt(r.Tq0, 0); $('#roTqBox').classList.toggle('warnv', r.fade < 0.8); $('#roBF').textContent = fmt(r.BF, 2);
  $('#roMu').textContent = fmt(r.mu, 3); $('#roMu0').textContent = fmt(r.mu0, 2); $('#roFade').textContent = fmt(r.fade * 100, 0); $('#roMuBox').classList.toggle('alarm', r.fade < 0.8);
  $('#roE').textContent = Number.isFinite(r.E) ? fmt(r.E / 1000, 1) : '∞'; $('#roENote').textContent = clutch ? (r.canLock ? 'predicted for one engagement' : r.stalls ? 'predicted: engine stalls' : 'cannot lock — slips for ever') : `½Jω² from ${s.n0} rpm`;
  $('#roDT').textContent = Number.isFinite(r.dT) ? fmt(r.dT, 0) : '∞'; $('#roMrot').textContent = s.mrot;
  $('#roT').textContent = fmt(s.T, 0); $('#roTfade').textContent = m.Tfade; $('#roTBox').classList.toggle('alarm', s.T > T_ALARM); $('#roTBox').classList.toggle('warnv', s.T > m.Tfade - 100 && s.T <= T_ALARM);
  const ev = s.lastEv; $('#roTs').textContent = ev ? fmt(ev.ts, 2) : '—'; $('#roTsEst').textContent = Number.isFinite(r.tsEst) ? fmt(r.tsEst, 2) : '∞'; $('#roTsNote').textContent = clutch ? 'slip time' : 'stop time';
  $('#roEv').textContent = s.events; $('#roPpk').textContent = ev ? fmt(ev.Ppk / 1000, 0) : '—';
  const ps = $('#pillState'); const txtS = s.failed ? 'Lining destroyed' : s.phase === 'braking' ? 'Braking' : s.phase === 'engaging' ? (s.ev && s.ev.locked ? 'Locked up' : 'Slipping') : s.phase === 'stopped' ? 'Stopped' : s.phase === 'spinup' ? 'Spinning up' : s.phase === 'released' ? 'Released' : clutch ? 'Disengaged' : 'Running, brake off'; ps.textContent = txtS; ps.className = 'pill ' + (s.failed ? 'warn' : s.phase === 'braking' || s.phase === 'engaging' ? 'amber' : 'ok');
  $('#pillMode').textContent = { disc: 'Disc brake', drum: 'Drum brake', clutch: 'Plate clutch' }[s.mode] + ` · ${fmt(r.Tq, 0)} N·m`; $('#pillFade').classList.toggle('hidden', r.fade >= 0.8); $('#pillOverP').classList.toggle('hidden', !r.overp); $('#pillHot').classList.toggle('hidden', s.T <= T_ALARM); $('#pillLock').classList.toggle('hidden', !r.selflock);
  $('#rigTitle').textContent = { disc: 'Disc brake · floating caliper, two pads on a ventilated disc', drum: 'Drum brake · leading and trailing shoes, one wheel cylinder', clutch: 'Plate clutch · flywheel, friction plate(s), pressure plate and springs' }[s.mode];
  $('#btnRun').textContent = s.run ? 'Running' : 'Start'; $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnEvent').textContent = clutch ? 'Engage clutch' : 'Apply brake'; $('#btnAuto').textContent = s.auto ? 'Auto-repeat: on' : 'Auto-repeat: off'; $('#btnAuto').classList.toggle('on', s.auto);
}
function record(tag = '') { const s = state, r = R, ev = s.lastEv, row = { i: s.log.length + 1, tag, mode: s.mode, mat: s.mat, dist: s.mode === 'drum' ? 'drum' : s.dist, act: s.mode === 'clutch' ? s.Fs + ' kN' : s.p + ' bar', p: s.mode === 'clutch' ? s.Fs : s.p, dp: s.dp, F: +r.F.toFixed(0), Re: +r.Re.toFixed(1), pmax: +r.pmax.toFixed(2), mu: +r.mu.toFixed(3), Tq: +r.Tq.toFixed(0), BF: +r.BF.toFixed(2), T: +s.T.toFixed(0), E: ev ? +(ev.E / 1000).toFixed(1) : '', dT: ev ? +ev.dT.toFixed(0) : '', ts: ev ? +ev.ts.toFixed(2) : '', note: ev ? ev.note : '', ro: s.ro, ri: s.ri, J: s.J, n0: s.n0, TL: s.TL, fade: +(r.fade * 100).toFixed(0) }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.mode, r.mat, r.dist, r.act, r.F, r.Re, r.pmax, r.mu, r.Tq, r.BF, r.T, r.E, r.dT, r.ts].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,mode,material,distribution,actuation,F_N,Re_mm,p_max_MPa,mu,torque_Nm,brake_factor,T_C,E_kJ,dT_K,time_s,note,ro,ri,J,n0,TL,fade_pct\n' + state.log.map(r => [r.i, r.tag, r.mode, r.mat, r.dist, r.act, r.F, r.Re, r.pmax, r.mu, r.Tq, r.BF, r.T, r.E, r.dT, r.ts, r.note, r.ro, r.ri, r.J, r.n0, r.TL, r.fade].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'firststop', t: 'Make a stop', d: 'Apply the disc brake from 1 500 rpm. Read the stop time, the energy absorbed and the temperature rise of the disc.', ok: () => state.flags.firststop },
  { id: 'dist', t: 'Uniform wear against uniform pressure', d: 'Record the same disc with both pressure distributions and compare the effective radius and torque. Which is the safe design assumption?', ok: () => { const L = state.log.filter(r => r.mode === 'disc'); return L.some(a => L.some(b => a.dist === 'wear' && b.dist === 'pressure' && a.ro === b.ro && a.ri === b.ri && a.p === b.p)); } },
  { id: 'pressure', t: 'Pressure to torque', d: 'Record a brake at two hydraulic pressures with the same geometry: torque capacity scales with clamp force.', ok: () => { const L = state.log.filter(r => r.mode !== 'clutch'); return L.some(a => L.some(b => a.mode === b.mode && a.p !== b.p && a.ro === b.ro && a.dp === b.dp && a.mat === b.mat)); } },
  { id: 'overp', t: 'Lining pressure limit', d: 'Shrink the pad or raise the pressure until the lining pressure passes the allowable for the material.', ok: () => state.flags.overp },
  { id: 'fade', t: 'Fade', d: 'Switch auto-repeat on and let the brake make repeated stops until friction falls below 80 % of cold. Watch the stop time grow.', ok: () => state.flags.fade },
  { id: 'drum', t: 'Drum self-energising', d: 'Record a drum brake and a disc brake at the same pressure and piston bore (set the disc caliper to the drum bore). Compare the brake factor: the leading shoe multiplies the actuation.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.mode === 'drum' && b.mode === 'disc' && a.p === b.p && a.dp === b.dp)); } },
  { id: 'drumfade', t: 'Drum sensitivity', d: 'Record a drum brake cold and again when faded: because the leading shoe multiplies μ, its torque falls further than the disc brake does.', ok: () => { const L = state.log.filter(r => r.mode === 'drum'); return L.some(a => L.some(b => a.p === b.p && a.fade >= 95 && b.fade <= 80)); } },
  { id: 'clutch', t: 'Engage the clutch', d: 'Release the clutch pedal: the engine droops, the load spins up, the plates lock. Read the slip time and the energy absorbed.', ok: () => state.flags.clutch },
  { id: 'burn', t: 'Clutch that cannot lock', d: 'Raise the load torque above the clutch capacity (or drop the spring force) and engage: the clutch slips continuously and burns.', ok: () => state.flags.burn },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawChart(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setMode(m) { const s = state; s.mode = m; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === m)); document.querySelectorAll('[data-m]').forEach(e => e.classList.toggle('off', !e.dataset.m.split(' ').includes(m))); s.phase = 'idle'; s.tph = 0; s.ev = null; s.lastEv = null; s.w1 = m === 'clutch' ? s.ne * 2 * Math.PI / 60 : s.n0 * 2 * Math.PI / 60; s.w2 = 0; s.auto = false; }
function resetRig() { const s = state; s.T = s.Tamb; s.failed = false; s.phase = 'idle'; s.tph = 0; s.ev = null; s.w1 = s.mode === 'clutch' ? s.ne * 2 * Math.PI / 60 : s.n0 * 2 * Math.PI / 60; s.w2 = 0; }
const PRESETS = {
  single: () => { setMode('disc'); for (const [k, v] of Object.entries({ p: 40, dp: 44, ro: 140, ri: 100, pad: 100, J: 20, n0: 1500, mrot: 6, cool: 1, Tamb: 25, repeat: 20, tscale: 1, ta: 0.6 })) setCtl(k, v); $('#mat').value = 'organic'; state.mat = 'organic'; $('#dist').value = 'wear'; state.dist = 'wear'; resetRig(); state.run = true; startEvent(); },
  repeat: () => { PRESETS.single(); state.ev = null; state.phase = 'idle'; setCtl('repeat', 12); setCtl('tscale', 4); state.auto = true; },
  overp: () => { PRESETS.single(); state.ev = null; state.phase = 'idle'; setCtl('p', 100); setCtl('pad', 30); setCtl('ri', 125); },
  drum: () => { setMode('drum'); for (const [k, v] of Object.entries({ p: 40, dp: 25, dd: 250, w: 50, J: 20, n0: 1500, mrot: 8, cool: 1, Tamb: 25, repeat: 20, tscale: 1 })) setCtl(k, v); $('#mat').value = 'organic'; state.mat = 'organic'; resetRig(); state.run = true; startEvent(); },
  drumfade: () => { PRESETS.drum(); state.ev = null; state.phase = 'idle'; setCtl('repeat', 12); setCtl('tscale', 4); state.auto = true; },
  clutch: () => { setMode('clutch'); for (const [k, v] of Object.entries({ Fs: 4, plates: 1, ro: 120, ri: 80, ne: 2000, Teng: 150, Je: 0.4, Jl: 1, TL: 60, ta: 0.6, mrot: 5, cool: 1, Tamb: 25, repeat: 10, tscale: 1 })) setCtl(k, v); $('#mat').value = 'organic'; state.mat = 'organic'; $('#dist').value = 'wear'; state.dist = 'wear'; resetRig(); state.run = true; startEvent(); },
  clutchover: () => { PRESETS.clutch(); state.ev = null; state.phase = 'idle'; setCtl('TL', 200); setCtl('Fs', 1.5); state.w1 = state.ne * 2 * Math.PI / 60; startEvent(); },
  clutchheavy: () => { PRESETS.clutch(); state.ev = null; state.phase = 'idle'; setCtl('Jl', 4); setCtl('ta', 1.5); state.w1 = state.ne * 2 * Math.PI / 60; startEvent(); },
  stall: () => { PRESETS.clutch(); state.ev = null; state.phase = 'idle'; setCtl('Jl', 6); setCtl('ta', 0.1); state.w1 = state.ne * 2 * Math.PI / 60; startEvent(); },
  clutchrepeat: () => { PRESETS.clutch(); state.ev = null; state.phase = 'idle'; setCtl('Jl', 6); setCtl('ta', 2); setCtl('repeat', 2); setCtl('cool', 0.3); setCtl('tscale', 4); state.auto = true; },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('p', v => v + ' bar'); bindR('dp', v => v + ' mm'); bindR('Fs', v => v.toFixed(1) + ' kN'); bindR('plates', v => String(v)); bindR('ro', v => v + ' mm'); bindR('ri', v => v + ' mm'); bindR('pad', v => v + '°'); bindR('dd', v => v + ' mm'); bindR('w', v => v + ' mm');
  bindR('J', v => v + ' kg·m²'); bindR('n0', v => v + ' rpm'); bindR('ne', v => v + ' rpm'); bindR('Teng', v => v + ' N·m'); bindR('Je', v => v.toFixed(2) + ' kg·m²'); bindR('Jl', v => v.toFixed(1) + ' kg·m²'); bindR('TL', v => v + ' N·m'); bindR('ta', v => v.toFixed(1) + ' s');
  bindR('mrot', v => v + ' kg'); bindR('cool', v => '×' + v.toFixed(1)); bindR('Tamb', v => v + ' °C'); bindR('repeat', v => v + ' s'); bindR('tscale', v => '×' + v);
  $('#mat').addEventListener('change', e => state.mat = e.target.value); $('#dist').addEventListener('change', e => state.dist = e.target.value); document.querySelectorAll('#types button').forEach(b => b.onclick = () => setMode(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnEvent').onclick = () => { if (state.phase === 'idle' || state.phase === 'released' || state.phase === 'spinup') { state.w1 = state.mode === 'clutch' ? Math.max(state.w1, 0) : state.n0 * 2 * Math.PI / 60; startEvent(); state.run = true; } };
  $('#btnAuto').onclick = () => { state.auto = !state.auto; state.run = true; }; $('#btnReset').onclick = resetRig; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('bc_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'B') $('#btnEvent').click(); if (k === 'R') record(''); if (k === 'A') $('#btnAuto').click(); if (k === '1') setMode('disc'); if (k === '2') setMode('drum'); if (k === '3') setMode('clutch'); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.BrakeClutch = {
  getState: () => ({ inputs: { mode: state.mode, p: state.p, dp: state.dp, Fs: state.Fs, plates: state.plates, ro: state.ro, ri: state.ri, pad: state.pad, dd: state.dd, w: state.w, mat: state.mat, dist: state.dist, J: state.J, n0: state.n0, ne: state.ne, Teng: state.Teng, Je: state.Je, Jl: state.Jl, TL: state.TL, ta: state.ta, mrot: state.mrot, cool: state.cool, repeat: state.repeat, auto: state.auto, run: state.run }, results: { F: R.F, Re: R.Re, pmax: R.pmax, mu: R.mu, fade: R.fade, Tq: R.Tq, Tq0: R.Tq0, BF: R.BF, E: R.E, dT: R.dT, tsEst: R.tsEst, T: state.T, phase: state.phase, events: state.events, last: state.lastEv ? { ts: state.lastEv.ts, E: state.lastEv.E, dT: state.lastEv.dT, note: state.lastEv.note } : null }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.mode) setMode(inp.mode); for (const k of ['p', 'dp', 'Fs', 'plates', 'ro', 'ri', 'pad', 'dd', 'w', 'J', 'n0', 'ne', 'Teng', 'Je', 'Jl', 'TL', 'ta', 'mrot', 'cool', 'Tamb', 'repeat', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.mat) { $('#mat').value = inp.mat; state.mat = inp.mat; } if (inp.dist) { $('#dist').value = inp.dist; state.dist = inp.dist; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.reset) resetRig(); if (inp.auto !== undefined) state.auto = !!inp.auto; if (inp.event) $('#btnEvent').click(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'brake-clutch', title: 'Brake and Clutch Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.BrakeClutch.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'brake-clutch', state: window.BrakeClutch.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('bc_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setMode('disc'); state.T = state.Tamb; step(0); drawRig(); updateReadouts(); drawChart(); drawTrace();
  requestAnimationFrame(tick);
}
build();
