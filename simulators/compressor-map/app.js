/* Compressor Map Simulator — single-stage centrifugal process compressor with a Greitzer-type lumped
   surge model: compressor duct with inertia, discharge plenum, process valve into a fixed header, and an
   anti-surge recycle valve driven by a PI controller with quick-open and a surge counter. The operating
   point, surge cycles, recovery and choke all come out of the same equations. Standalone, offline.
   Machine data illustrative. */
'use strict';

/* ------------------------------------------------------------------ machine constants */
const ND = 10000, QD = 8000, PHI_D = 0.75;                 // design speed rpm, design inlet flow m³/h, φ at design
const W = 0.3, PSI0 = 0.3, HH = 0.35, HREF = 108.6;        // Moore–Greitzer cubic: surge at φ = 2W, ψ_surge = 1 → HREF kJ/kg at N = ND
const RG = 297, T1 = 300, KG = 1.4, SIG = 0.366;           // gas: R J/kg·K, inlet K, k, (n−1)/n for the head→PR conversion
const CPT = RG * T1 / SIG / 1000;                          // 243.3 kJ/kg
const VPL = 6, AL = 0.001, T2D = 400, A2 = KG * RG * T2D;  // plenum m³, duct A/L m, plenum gas K, a² m²/s²
const KPV = 0.0176, KRV = 0.0075;                          // process valve and recycle valve coefficients, kg/s per √(Pa·kg/m³)
const VIB_ALARM = 45, VIB_TRIP = 70, PMAX = 1800;          // µm p-p, kW
const PHI_S = 2 * W, PHI_CH = 2.9 * W;
const DT = 0.0005;                                         // physics substep s
const PHASES = { stopped: 'Stopped', tripped: 'Tripped', startup: 'Running up', normal: 'Normal', holding: 'On control line — recycling', approach: 'Approaching surge', surge: 'SURGE', recovery: 'Recovering', choke: 'Choke (stonewall)' };
const $ = s => document.querySelector(s);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);
const fmtQ = q => Math.round(q).toLocaleString('en-ZA').replace(/,/g, ' ');

/* ------------------------------------------------------------------ compressor characteristic */
function psiC(phi) { if (phi >= 0) { const x = phi / W - 1; return PSI0 + HH * (1 + 1.5 * x - 0.5 * x * x * x); } return PSI0 + HH * (phi / W) * (phi / W); }
const prOf = hp => Math.pow(Math.max(0.02, 1 + hp / CPT), 1 / SIG);
const hpOf = pr => CPT * (Math.pow(Math.max(pr, 0.02), SIG) - 1);
const etaOf = phi => clamp(0.80 - 4 * (phi - 0.72) ** 2, 0.2, 0.8);
function compressor(m, n, ps) {   // m kg/s, n = N/ND, ps Pa → point on the characteristic
  const nn = Math.max(n, 0.05), rho1 = ps / (RG * T1), Q = m / rho1 * 3600, phi = Q / QD * PHI_D / nn, psi = psiC(phi);
  const hp = psi * HREF * nn * nn, pr = prOf(hp);
  return { Q, phi, psi, hp, pr, pc: ps * pr, eta: etaOf(phi), rho1 };
}

/* ------------------------------------------------------------------ state */
const state = {
  ps: 5, phdr: 11, nsp: 100, demand: 70, stroke: 2, margin: 10, asc: 'auto', asvMan: 0,
  run: false, tripped: null, n: 0, m: 0, pp: 5e5, asv: 100, asvLag: 100, asvDem: 100, I: 0, asvStep: 0,
  t: 0, surges: 0, surgeTimes: [], lastSurge: -1e9, reversed: false, recovering: false, phase: 'stopped', prevPhase: 'stopped',
  vib: 10, vibImp: 0, vibHiT: 0, qf: 0, qPrev: 0, trail: [], log: [], events: [], ramps: [], trace: [], traceAcc: 0, ctlAcc: 0, flags: {}, ep: null, cand: null, candT: 0, pending: null, impAng: 0, dash: { main: 0, proc: 0, rec: 0 },
};
const R = { c: compressor(0, 0, 5e5), mp: 0, mr: 0, qs: 100, qscl: 110, sm: 0, qch: 1, smc: 100, power: 0, t2: T1 };

/* ------------------------------------------------------------------ physics */
function stepPhysics(dt) {
  const ps = state.ps * 1e5, c = compressor(state.m, state.n / 100, ps); R.c = c;
  state.m += AL * (c.pc - state.pp) * dt;
  const rho2 = state.pp / (RG * T2D);
  const mp = state.demand / 100 * KPV * Math.sqrt(Math.max(0, state.pp - state.phdr * 1e5) * rho2);
  const mr = state.asv / 100 * KRV * Math.sqrt(Math.max(0, state.pp - ps) * rho2);
  state.pp += A2 / VPL * (state.m - mp - mr) * dt;
  if (state.pp < ps * 0.95) state.pp = ps * 0.95;
  R.mp = mp; R.mr = mr;
}

/* ------------------------------------------------------------------ anti-surge controller and valve */
function control(dtc) {
  const c = R.c; state.qf += (c.Q - state.qf) * Math.min(1, dtc / 0.1);
  const pr = state.pp / (state.ps * 1e5), neq = Math.sqrt(Math.max(0, hpOf(Math.max(1, pr))) / HREF);
  R.pr = pr; R.qs = Math.max(150, PHI_S / PHI_D * QD * neq); R.qscl = R.qs * (1 + state.margin / 100);
  R.sm = (state.qf - R.qs) / R.qs * 100; R.qch = PHI_CH / PHI_D * QD * Math.max(0.05, state.n / 100); R.smc = (R.qch - state.qf) / R.qch * 100;
  R.stable = stablePoint();
  const dq = (state.qf - state.qPrev) / dtc; state.qPrev = state.qf;
  const rev = state.m < 0; if (rev && !state.reversed && state.n > 20) surgeEvent(); state.reversed = rev;
  let dem;
  if (!state.run || state.tripped) { dem = 100; state.I = 100; }
  else if (state.asc === 'manual') dem = state.asvMan;
  else {
    const err = (R.qscl - state.qf) / R.qscl;                 // positive when left of the control line
    state.I = clamp(state.I + 80 * err * dtc, 0, 100);
    const pi = 500 * err + state.I - clamp(dq / QD, -1, 0) * 60;   // P + I + anticipation on fast flow loss
    if (dq < -QD * 2.5 || rev) state.asvStep = Math.min(100, Math.max(state.asvStep, state.asv + 30));   // quick-open step
    state.asvStep = Math.max(0, state.asvStep - 4 * dtc);
    dem = clamp(Math.max(pi, state.asvStep), 0, 100);
    if (state.asv > dem + 1) state.I = clamp(state.I + (state.asv - dem) * dtc * 2, 0, 100);   // integral tracks the slowly closing valve (anti-windup)
  }
  state.asvDem = dem;
  state.asvLag += (dem - state.asvLag) * Math.min(1, dtc / Math.max(0.05, state.stroke / 6));
  const d = state.asvLag - state.asv, lim = (d > 0 ? 100 / state.stroke : 100 / Math.max(25, state.stroke * 3)) * dtc;   // opens at stroke speed, closes slowly (≥ 25 s)
  state.asv = clamp(state.asv + clamp(d, -lim, lim), 0, 100);
}
function stablePoint() {   // quasi-steady intersection of the falling branch with the current system line; null if none (the machine will cycle)
  const ps = state.ps * 1e5, n = Math.max(0.05, state.n / 100), rho1 = ps / (RG * T1); let prev = null;
  for (let phi = PHI_S; phi <= 3.15 * W; phi += 0.02) {
    const Q = phi / PHI_D * QD * n, pp = ps * prOf(psiC(phi) * HREF * n * n), rho2 = pp / (RG * T2D);
    const msys = state.demand / 100 * KPV * Math.sqrt(Math.max(0, pp - state.phdr * 1e5) * rho2) + state.asv / 100 * KRV * Math.sqrt(Math.max(0, pp - ps) * rho2);
    const diff = Q * rho1 / 3600 - msys;
    if (diff >= 0) { if (prev === null) return null; const Qs = PHI_S / PHI_D * QD * n; return { Q, sm: (Q - Qs) / Qs * 100 }; }   // crossing left of the surge line = unstable
    prev = diff;
  }
  return null;
}
function surgeEvent() {
  state.surges++; state.surgeTimes.push(state.t); state.lastSurge = state.t; state.recovering = true; state.vibImp = 50;
  addEvent(`Surge #${state.surges} — flow reversed at PR ${fmt(R.pr, 2)}, ASV ${fmt(state.asv, 0)} %, ${state.asc === 'auto' ? 'stroke ' + state.stroke + ' s' : 'manual'}`, 'surge');
  record('surge #' + state.surges);
  if (state.asc === 'auto' && state.stroke >= 10) state.flags.slow = true;
  if (state.ps <= 3.8) state.flags.ps = true;
  if (state.surgeTimes.filter(t => state.t - t < 60).length >= 5 && !state.tripped) trip('surge counter: 5 surges in 60 s');
}
function trip(reason) { state.tripped = reason; state.run = false; state.flags.trip = true; addEvent('TRIP — ' + reason, 'trip'); record('trip'); $('#btnReset').classList.remove('hidden'); }
function addEvent(txt, cls) { state.events.unshift({ t: state.t, txt, cls }); if (state.events.length > 200) state.events.pop(); renderEvents(); }

/* ------------------------------------------------------------------ phase, vibration, derived */
function phaseOf() {
  if (state.n < 20) return state.tripped ? 'tripped' : state.run ? 'startup' : 'stopped';
  if (state.t - state.lastSurge < 2.5 || state.reversed) return 'surge';
  if (state.recovering) { if (!R.stable) return 'surge'; if (R.sm >= state.margin && state.t - state.lastSurge > 6) state.recovering = false; else return 'recovery'; }
  const prev = state.phase, mg = state.margin;
  if (R.sm < mg * 0.5) return 'approach';                                   // deep in the band: the valve is losing
  if (state.asv > 3 && R.sm < mg + (prev === 'holding' ? 3 : 1.5)) return 'holding';   // ASC holding the point on the control line
  if (R.sm < mg - (prev === 'normal' ? 0 : -1.5)) return 'approach';
  if (R.smc < 4) return 'choke';
  return 'normal';
}
function sample(dts) {
  const n = state.n / 100; let v = 8 + 7 * n * n;
  const pre = state.m > 0 && R.sm < 20 ? 30 * n * n * clamp((20 - R.sm) / 20, 0, 1) ** 2 : 0;
  const burst = state.vibImp + (state.reversed ? 20 + 12 * Math.random() : 0); state.vibImp *= Math.exp(-dts / 0.5);
  v += Math.max(pre, burst);
  if (R.smc < 8 && n > 0.3) v += 20 * clamp((8 - R.smc) / 8, 0, 2) ** 2;
  v += (Math.random() - 0.5) * 3;
  state.vib += (v - state.vib) * 0.4;
  if (state.vib > VIB_TRIP && state.n > 20) { state.vibHiT += dts; if (state.vibHiT > 2 && !state.tripped) trip(`vibration ${fmt(state.vib, 0)} µm p-p above ${VIB_TRIP} µm for 2 s`); } else state.vibHiT = 0;
  const c = R.c; R.t2 = T1 * Math.pow(Math.max(1, R.pr), (KG - 1) / KG / c.eta);
  R.power = Math.max(0, state.m) * Math.max(0, c.hp) / c.eta + 45 * n ** 3;
  if (state.run && state.n >= 95 && state.ps <= 3.8 && ['approach', 'holding', 'surge'].includes(state.phase)) state.flags.ps = true;
  if (state.phase === 'holding' && state.ep && state.t - state.ep.t > 10 && state.surges === state.ep.surges && state.run && state.n >= 95) state.flags.fast = true;
  const cand = phaseOf(), fast = ['surge', 'recovery', 'tripped', 'stopped', 'startup'];
  if (cand !== state.phase) { if (cand === state.cand) state.candT += dts; else { state.cand = cand; state.candT = 0; } if (fast.includes(cand) || fast.includes(state.phase) || state.candT >= 0.8) { state.prevPhase = state.phase; state.phase = cand; onPhase(state.prevPhase, cand); } } else state.cand = null;
  state.trace.push({ t: state.t, pd: state.pp / 1e5, ps: state.ps, phdr: state.phdr, q: c.Q, qscl: R.qscl, asv: state.asv, vib: state.vib, ph: state.phase });
  if (state.trace.length > 1200) state.trace.shift();
  if (state.n > 20) { state.trail.push([c.Q, R.pr]); if (state.trail.length > 120) state.trail.shift(); } else state.trail.length = 0;
}
function onPhase(from, to) {
  const f = state.flags;
  const loaded = state.run && state.n >= 95 && !state.tripped;
  if (to === 'approach') { addEvent(`Approaching surge — SM ${fmt(R.sm, 1)} %`, 'warn'); if (loaded) { f.approach = true; if (state.ps <= 3.8) f.ps = true; } }
  if (to === 'holding') { addEvent(`On the control line — recycle ${fmt(state.asv, 0)} % holding SM ${fmt(R.sm, 1)} %`, 'ok'); if (loaded && (from === 'normal' || from === 'approach') && state.asc === 'auto' && state.stroke <= 2) state.ep = { surges: state.surges, t: state.t }; }
  if (to === 'choke') { if (loaded) f.choke = true; addEvent('Choke — flow limited by the stonewall', 'warn'); }
  if (to === 'recovery') addEvent('Recovering — recycle holding the point right of the surge line', 'ok');
  if (to === 'normal') { if (loaded && from === 'recovery' && state.asc === 'manual' && state.asv > 5) f.recover = true; if (loaded && (from === 'holding' || from === 'approach') && state.ep && state.ep.surges === state.surges) f.fast = true; state.ep = null; addEvent('Normal operation', 'ok'); }
  if (to === 'surge') state.ep = null;
  if (to === 'stopped') addEvent('Stopped', 'ok');
  if (to === 'startup') addEvent('Running up', 'ok');
}

/* ------------------------------------------------------------------ rig (P&ID) */
function buildRig() {
  $('#rig').innerHTML = `<svg viewBox="0 0 900 300" role="img" aria-label="Compressor, aftercooler, check valve, process valve and anti-surge recycle valve">
    <circle class="glow" id="glow" cx="310" cy="160" r="56"/>
    <path class="pipe" stroke-width="12" d="M20,160 H272"/><path class="pipe" stroke-width="12" d="M310,122 V80 H640"/><path class="pipe" stroke-width="12" d="M640,80 H880"/><path class="pipe" stroke-width="10" d="M640,80 V270 H200 V160"/>
    <path class="pipe-fluid" id="fSuc" stroke-width="6" d="M20,160 H272"/><path class="pipe-fluid" id="fDis" stroke-width="6" d="M310,122 V80 H640"/><path class="pipe-fluid" id="fProc" stroke-width="6" d="M640,80 H880"/><path class="pipe-fluid rec" id="fRec" stroke-width="5" d="M640,80 V270 H200 V160"/>
    <line class="orifice" x1="118" y1="150" x2="118" y2="170"/><line class="orifice" x1="126" y1="150" x2="126" y2="170"/>
    <rect class="nozzle" x="262" y="150" width="14" height="20" rx="2"/><rect class="nozzle" x="300" y="116" width="20" height="14" rx="2"/>
    <circle class="casing" cx="310" cy="160" r="38"/>
    <g id="imp" transform="translate(310,160)"><circle r="7" class="impeller"/>${[0, 60, 120, 180, 240, 300].map(a => `<path class="impeller" d="M0,-8 Q10,-20 4,-30" transform="rotate(${a})"/>`).join('')}</g>
    <line class="stem" x1="310" y1="198" x2="310" y2="212"/><circle class="motor" cx="310" cy="232" r="20"/><text class="tag" x="310" y="237" text-anchor="middle">M</text>
    <rect class="cooler" x="395" y="62" width="50" height="36" rx="3"/><path class="coil" d="M399,80 l6,-10 l8,20 l8,-20 l8,20 l8,-20 l6,10"/>
    <polygon class="check" points="690,72 690,88 710,80"/><line class="stem" x1="710" y1="70" x2="710" y2="90"/>
    <g id="pv"><polygon class="valve-body" points="776,70 792,80 776,90"/><polygon class="valve-body" points="808,70 792,80 808,90"/><line class="stem" x1="792" y1="80" x2="792" y2="58"/><rect class="actuator" x="780" y="46" width="24" height="12" rx="2"/><rect id="pvOpen" class="valve-open" x="782" y="48" width="20" height="8"/></g>
    <g id="asv"><polygon class="valve-body" points="404,260 420,270 404,280"/><polygon class="valve-body" points="436,260 420,270 436,280"/><line class="stem" x1="420" y1="270" x2="420" y2="248"/><rect class="actuator" x="406" y="234" width="28" height="14" rx="3"/><rect id="asvOpen" class="valve-open" x="408" y="236" width="24" height="10"/></g>
    ${inst('PT', 60, 112, 'iPs', 'lead', 60, 128, 60, 154)}${inst('FT', 168, 112, 'iQ', 'lead', 168, 128, 126, 150)}${inst('PT', 520, 40, 'iPd', 'lead', 520, 56, 520, 74)}${inst('TT', 585, 40, 'iT2', 'lead', 585, 56, 585, 74)}${inst('VT', 385, 210, 'iVib', 'lead', 369, 210, 341, 178)}${inst('ST', 250, 232, 'iN', 'lead', 266, 232, 288, 232)}
    <text class="hdr" x="22" y="185">Suction header</text><text class="hdr" x="420" y="112" text-anchor="middle">Aftercooler</text><text class="hdr" x="700" y="106" text-anchor="middle">Check</text><text class="hdr" x="792" y="112" text-anchor="middle">Process valve</text><text class="inst-val" id="iPv" x="792" y="126">70 %</text>
    <text class="hdr" x="878" y="66" text-anchor="end">To process header</text><text class="inst-val" id="iHdr" x="878" y="106" text-anchor="end">11.0 bar a</text>
    <text class="hdr" x="565" y="296" text-anchor="middle">Anti-surge recycle valve (ASV)</text><text class="inst-val" id="iAsv" x="420" y="296" text-anchor="middle">100 % open</text>
    <text class="revtxt" id="revTxt" x="146" y="230" opacity="0">FLOW REVERSED</text>
  </svg>`;
}
function inst(tag, x, y, id, lc, x1, y1, x2, y2) { return `<line class="${lc}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/><circle class="inst" cx="${x}" cy="${y}" r="15"/><text class="inst-tag" x="${x}" y="${y + 4}">${tag}</text><text class="inst-val" id="${id}" x="${x}" y="${y - 20}">—</text>`; }
function updateRig(dt) {
  const n = state.n / 100, c = R.c;
  state.impAng = (state.impAng + n * 540 * dt) % 360; $('#imp').setAttribute('transform', `translate(310,160) rotate(${state.impAng})`);
  state.dash.main -= state.m * 5 * dt; state.dash.proc -= R.mp * 5 * dt; state.dash.rec -= R.mr * 5 * dt;
  for (const id of ['fSuc', 'fDis']) { const e = $('#' + id); e.style.strokeDashoffset = state.dash.main; e.classList.toggle('rev', state.m < -0.2); }
  $('#fProc').style.strokeDashoffset = state.dash.proc; $('#fRec').style.strokeDashoffset = state.dash.rec; $('#fRec').style.opacity = R.mr > 0.1 ? 0.9 : 0.25; $('#fProc').style.opacity = R.mp > 0.1 ? 0.9 : 0.25;
  $('#glow').style.opacity = state.reversed ? 0.35 : (state.phase === 'approach' ? 0.12 : 0); $('#revTxt').setAttribute('opacity', state.reversed ? 1 : 0);
  $('#asvOpen').setAttribute('width', 24 * state.asv / 100); $('#pvOpen').setAttribute('width', 20 * state.demand / 100);
  $('#iPs').textContent = fmt(state.ps, 1) + ' bar a'; $('#iQ').textContent = fmtQ(c.Q) + ' m³/h'; $('#iPd').textContent = fmt(state.pp / 1e5, 2) + ' bar a';
  const t2 = $('#iT2'); t2.textContent = fmt(R.t2 - 273.15, 0) + ' °C'; t2.classList.toggle('hot', R.t2 - 273.15 > 170);
  const vt = $('#iVib'); vt.textContent = fmt(state.vib, 0) + ' µm'; vt.classList.toggle('hot', state.vib > VIB_ALARM);
  $('#iN').textContent = Math.round(n * ND).toLocaleString().replace(/,/g, ' ') + ' rpm'; $('#iPv').textContent = Math.round(state.demand) + ' %'; $('#iHdr').textContent = fmt(state.phdr, 1) + ' bar a'; $('#iAsv').textContent = Math.round(state.asv) + ' % open';
}

/* ------------------------------------------------------------------ map */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
const QMAX = 12000, PRMIN = 1, PRMAX = 3.3;
function drawMap() {
  const cv = $('#map'), ctx = cv.getContext('2d'), Wd = cv.width, Hd = cv.height; ctx.clearRect(0, 0, Wd, Hd);
  const L = 58, Rr = 14, T = 14, B = 40, X = q => L + (q / QMAX) * (Wd - L - Rr), Y = pr => T + (1 - (pr - PRMIN) / (PRMAX - PRMIN)) * (Hd - T - B);
  ctx.font = '11px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillStyle = col('--muted'); ctx.strokeStyle = col('--grid'); ctx.lineWidth = 1;
  for (let q = 0; q <= QMAX; q += 2000) { ctx.beginPath(); ctx.moveTo(X(q), T); ctx.lineTo(X(q), Hd - B); ctx.stroke(); ctx.fillText(fmtQ(q), X(q), Hd - B + 16); }
  ctx.textAlign = 'right'; for (let pr = 1; pr <= PRMAX + 1e-9; pr += 0.5) { ctx.beginPath(); ctx.moveTo(L, Y(pr)); ctx.lineTo(Wd - Rr, Y(pr)); ctx.stroke(); ctx.fillText(pr.toFixed(1), L - 6, Y(pr) + 4); }
  ctx.textAlign = 'center'; ctx.fillText('inlet flow, m³/h actual', (L + Wd - Rr) / 2, Hd - 6); ctx.save(); ctx.translate(12, (T + Hd - B) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('pressure ratio Pd / Ps', 0, 0); ctx.restore();
  ctx.save(); ctx.beginPath(); ctx.rect(L, T, Wd - L - Rr, Hd - T - B); ctx.clip();
  const line = (pts, color, width, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); pts.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke(); ctx.setLineDash([]); };
  const speedPt = (phi, n) => [phi / PHI_D * QD * n, prOf(psiC(phi) * HREF * n * n)];
  for (const n of [0.7, 0.8, 0.9, 1.0, 1.05]) {
    const pts = []; for (let phi = PHI_S; phi <= 3.12 * W; phi += 0.01) pts.push(speedPt(phi, n)); line(pts, col('--steel'), n === 1 ? 2 : 1.3);
    const s = speedPt(PHI_S, n); ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.font = '11px IBM Plex Mono, monospace'; ctx.fillText(Math.round(n * 100) + ' %', X(s[0]) - 6, Y(s[1]) - 2);
  }
  const unst = []; for (let phi = 0; phi <= PHI_S; phi += 0.02) unst.push(speedPt(phi, 1)); line(unst, col('--steel-light'), 1, [2, 5]);
  const surge = [], scl = [], choke = []; for (let n = 0.55; n <= 1.12; n += 0.01) { const s = speedPt(PHI_S, n); surge.push(s); scl.push([s[0] * (1 + state.margin / 100), s[1]]); choke.push(speedPt(PHI_CH, n)); }
  line(surge, col('--alarm'), 2.2); line(scl, col('--alarm'), 1.4, [6, 5]); line(choke, col('--warn'), 2);
  ctx.font = '600 12px Barlow Condensed, sans-serif'; ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); const sTop = surge[surge.length - 1]; ctx.fillText('surge line', X(sTop[0]) - 62, Y(sTop[1]) + 12); ctx.fillText('control line', X(scl[scl.length - 1][0]) + 4, Y(sTop[1]) + 12);
  ctx.fillStyle = col('--warn'); const cTop = choke[choke.length - 1]; ctx.fillText('choke / stonewall', X(cTop[0]) + 6, Y(cTop[1]) + 4);
  ctx.fillStyle = col('--muted'); ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillText('unstable branch', X(unst[8][0]) - 20, Y(unst[8][1]) - 8);
  if (state.n > 20) {   // system line for the current process and recycle settings
    const ps = state.ps * 1e5, rho1 = ps / (RG * T1), sys = [];
    for (let pp = ps; pp <= ps * PRMAX; pp += ps * 0.02) { const rho2 = pp / (RG * T2D); const m = state.demand / 100 * KPV * Math.sqrt(Math.max(0, pp - state.phdr * 1e5) * rho2) + state.asv / 100 * KRV * Math.sqrt(Math.max(0, pp - ps) * rho2); sys.push([m / rho1 * 3600, pp / ps]); }
    line(sys, col('--brass'), 1.6, [8, 4]);
  }
  if (state.trail.length > 1) { for (let i = 1; i < state.trail.length; i++) { const a = state.trail[i - 1], b = state.trail[i]; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.globalAlpha = 0.15 + 0.7 * i / state.trail.length; ctx.lineWidth = 2; ctx.moveTo(X(a[0]), Y(a[1])); ctx.lineTo(X(b[0]), Y(b[1])); ctx.stroke(); } ctx.globalAlpha = 1; }
  if (state.n > 20) { const p = [R.c.Q, R.pr]; ctx.beginPath(); ctx.fillStyle = state.phase === 'surge' ? col('--alarm') : state.phase === 'approach' ? col('--warn') : col('--water'); ctx.arc(X(p[0]), Y(p[1]), 7, 0, Math.PI * 2); ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = col('--panel'); ctx.stroke(); }
  ctx.restore();
}

/* ------------------------------------------------------------------ traces */
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), Wd = cv.width, Hd = cv.height; ctx.clearRect(0, 0, Wd, Hd);
  const L = 58, Rr = 46, T = 14, B = 22, span = 60, t1 = state.t, t0 = t1 - span, X = t => L + (t - t0) / span * (Wd - L - Rr);
  const tr = state.trace; const strips = [[T, T + 96], [T + 106, T + 202], [T + 212, Hd - B]];
  const PH = { normal: '--good', holding: '--water-deep', approach: '--warn', surge: '--alarm', recovery: '--water', choke: '--brass' };
  for (let i = 1; i < tr.length; i++) { const c = PH[tr[i].ph]; if (!c) continue; ctx.fillStyle = col(c); ctx.fillRect(X(tr[i - 1].t), 2, Math.max(1, X(tr[i].t) - X(tr[i - 1].t) + 0.5), 7); }
  ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'center';
  for (let s = 0; s <= span; s += 10) { const x = X(t0 + s); ctx.strokeStyle = col('--grid'); ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, Hd - B); ctx.stroke(); ctx.fillText('−' + (span - s) + ' s', x, Hd - 6); }
  const strip = (i, lo, hi, label, ylabels) => { const [a, b] = strips[i]; ctx.strokeStyle = col('--line'); ctx.strokeRect(L, a, Wd - L - Rr, b - a); ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.font = '10px IBM Plex Mono, monospace'; for (const v of ylabels) ctx.fillText(v, L - 5, a + (1 - (v - lo) / (hi - lo)) * (b - a) + 3); ctx.textAlign = 'left'; ctx.font = '600 11px Barlow Condensed, sans-serif'; ctx.fillStyle = col('--ink'); ctx.fillText(label, L + 5, a + 12); return v => a + (1 - clamp((v - lo) / (hi - lo), -0.02, 1.02)) * (b - a); };
  const series = (Yf, key, color, width, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); let first = true; for (const p of tr) { const v = typeof key === 'function' ? key(p) : p[key]; if (!Number.isFinite(v)) continue; const x = X(p.t), y = Yf(v); first ? ctx.moveTo(x, y) : ctx.lineTo(x, y); first = false; } ctx.stroke(); ctx.setLineDash([]); };
  ctx.save(); ctx.beginPath(); ctx.rect(L, T, Wd - L - Rr, Hd - T - B); ctx.clip();
  const pmax = Math.max(14, state.phdr + 3, ...tr.map(p => p.pd)) + 0.5; const Y1 = strip(0, 0, pmax, 'Pressure, bar a', [0, Math.round(pmax / 2), Math.round(pmax)]);
  series(Y1, 'phdr', col('--steel'), 1, [4, 4]); series(Y1, 'ps', col('--water-soft'), 1.5); series(Y1, 'pd', col('--water'), 2);
  const Y2 = strip(1, -5000, QMAX, 'Inlet flow, m³/h  ·  recycle valve %', [0, 6000, 12000]); const Y2b = v => strips[1][0] + (1 - v / 100) * (strips[1][1] - strips[1][0]);
  series(Y2, () => 0, col('--line'), 1); series(Y2, 'qscl', col('--alarm'), 1, [6, 4]); series(Y2b, 'asv', col('--good'), 1.5); series(Y2, 'q', col('--brass'), 2);
  const Y3 = strip(2, 0, 120, 'Shaft vibration, µm p-p', [0, 45, 70, 120]);
  series(Y3, () => VIB_ALARM, col('--warn'), 1, [4, 4]); series(Y3, () => VIB_TRIP, col('--alarm'), 1, [4, 4]); series(Y3, 'vib', col('--alarm'), 1.6);
  ctx.restore();
  ctx.textAlign = 'left'; ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--good'); ctx.fillText('ASV 100', Wd - Rr + 4, strips[1][0] + 4); ctx.fillText('0', Wd - Rr + 4, strips[1][1] + 3);
  ctx.fillStyle = col('--water'); ctx.fillText('Pd', Wd - Rr + 4, Y1(tr.length ? tr[tr.length - 1].pd : 0) + 3); ctx.fillStyle = col('--steel'); ctx.fillText('hdr', Wd - Rr + 26, Y1(state.phdr) + 3);
}

/* ------------------------------------------------------------------ readouts and status */
function updateReadouts() {
  const c = R.c;
  $('#roQ').textContent = fmtQ(c.Q); $('#roM').textContent = fmt(state.m, 1); $('#roPR').textContent = fmt(R.pr, 2);
  $('#roSM').textContent = state.n > 20 ? (R.sm > 0 ? '+' : '') + fmt(R.sm, 1) + ' %' : '—'; $('#roSMbox').classList.toggle('alarm', R.sm < 0 && state.n > 20); $('#roSMbox').classList.toggle('warnv', R.sm >= 0 && R.sm < state.margin && state.n > 20);
  $('#roASV').textContent = fmt(state.asv, 0); $('#roVib').textContent = fmt(state.vib, 0); $('#roVibBox').classList.toggle('alarm', state.vib > VIB_TRIP); $('#roVibBox').classList.toggle('warnv', state.vib > VIB_ALARM && state.vib <= VIB_TRIP);
  $('#roN').textContent = Math.round(state.n / 100 * ND).toLocaleString().replace(/,/g, ' '); $('#roP').textContent = fmt(R.power, 0); $('#roT2').textContent = fmt(R.t2 - 273.15, 0); $('#roPBox').classList.toggle('alarm', R.power > PMAX);
  const run = $('#pillRun'); run.textContent = state.tripped ? 'Tripped — ' + state.tripped : state.run ? (state.n >= 99 ? 'Running' : 'Running up ' + Math.round(state.n) + ' %') : state.n > 1 ? 'Coasting down' : 'Stopped'; run.classList.toggle('on', state.run); run.classList.toggle('trip', !!state.tripped);
  const ph = $('#pillPhase'); ph.textContent = PHASES[state.phase]; ph.className = 'pill phase ' + state.phase;
  $('#pillAsc').textContent = state.asc === 'auto' ? `ASC auto · ${state.stroke} s stroke` : 'ASC manual'; $('#pillSurges').textContent = state.surges + (state.surges === 1 ? ' surge' : ' surges');
  $('#pillVib').classList.toggle('hidden', state.vib <= VIB_ALARM); $('#pillPower').classList.toggle('hidden', R.power <= PMAX);
  $('#btnStart').disabled = state.run || !!state.tripped; $('#btnStop').disabled = !state.run;
}

/* ------------------------------------------------------------------ log, events */
function record(tag = '') {
  const r = { i: state.log.length + 1, t: +state.t.toFixed(1), tag, ps: +state.ps.toFixed(1), pd: +(state.pp / 1e5).toFixed(2), pr: +R.pr.toFixed(2), Q: Math.round(R.c.Q), m: +state.m.toFixed(2), rpm: Math.round(state.n / 100 * ND), sm: +R.sm.toFixed(1), asv: Math.round(state.asv), vib: Math.round(state.vib), kw: Math.round(R.power), phase: state.phase };
  state.log.push(r); renderLog();
}
function renderLog() {
  const tb = $('#logTable tbody'); tb.innerHTML = state.log.map(r => `<tr><td>${r.i}</td><td>${r.t}</td><td>${r.tag}</td><td>${r.ps}</td><td>${r.pd}</td><td>${r.pr}</td><td>${fmtQ(r.Q)}</td><td>${r.m}</td><td>${r.rpm}</td><td>${r.sm}</td><td>${r.asv}</td><td>${r.vib}</td><td>${r.kw}</td><td>${r.phase}</td></tr>`).join('');
  $('#logCount').textContent = state.log.length;
  const head = 'n,t_s,tag,Ps_bar,Pd_bar,PR,Q_m3h,m_kgs,rpm,SM_pct,ASV_pct,vib_um,kW,phase';
  state.csv = head + '\n' + state.log.map(r => [r.i, r.t, r.tag, r.ps, r.pd, r.pr, r.Q, r.m, r.rpm, r.sm, r.asv, r.vib, r.kw, r.phase].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
function renderEvents() { $('#events').innerHTML = state.events.map(e => `<li class="${e.cls || ''}">${e.t.toFixed(1).padStart(6)} s  ${e.txt}</li>`).join(''); $('#evCount').textContent = state.events.length; }

/* ------------------------------------------------------------------ learning tasks */
const TASKS = [
  { id: 'start', t: 'Start and load the compressor', d: 'Start, let it run up with the recycle open, and watch the ASC close the valve as the process takes the flow. Where does the point settle relative to the control line?', ok: () => state.flags.start, st: () => state.run && state.n >= 99 ? `SM ${fmt(R.sm, 1)} %` : '' },
  { id: 'approach', t: 'Enter the control band', d: 'Move the point into the band between the surge control line and the surge line with any of the four sliders. Watch the vibration begin to rise before anything else does.', ok: () => state.flags.approach },
  { id: 'surge', t: 'Make it surge', d: 'Cross the surge line (ASC in manual with the valve shut, or a slow valve) and watch the map, the pressure trace and the vibration trace during a surge cycle.', ok: () => state.surges > 0, st: () => state.surges ? `${state.surges} surge(s)` : '' },
  { id: 'recover', t: 'Recover it by hand', d: 'With the ASC in manual, open the recycle valve until the oscillation stops and the point sits right of the control line. Note how much recycle was needed.', ok: () => state.flags.recover },
  { id: 'slow', t: 'Slow valve loses the race', d: 'ASC auto, valve stroke 10 s or more, then close the process slowly. Count the surge cycles before the recycle catches up.', ok: () => state.flags.slow },
  { id: 'fast', t: 'Fast valve catches it', d: 'Same disturbance with the stroke at 2 s or less. The point enters the band, the valve opens and the point never crosses the line.', ok: () => state.flags.fast },
  { id: 'ps', t: 'Suction pressure alone', d: 'Change nothing downstream; drop the suction pressure to 3.8 bar a or less and explain why the point still moves toward surge.', ok: () => state.flags.ps },
  { id: 'choke', t: 'Find the stonewall', d: 'Open the process valve fully and lower the header until the point runs out to the choke line. What happens to flow when you ask for more?', ok: () => state.flags.choke },
  { id: 'trip', t: 'Let the protection trip it', d: 'Leave a surge unattended (manual, valve shut) or set a very slow valve: the surge counter or the vibration trip stops the machine.', ok: () => state.flags.trip },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { if (state.run && state.n >= 99 && R.mp > 0.5 && state.phase === 'normal') state.flags.start = true; for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ scenarios */
function setCtl(id, v) { state[id] = v; const e = $('#' + id); if (e) { e.value = v; e.dispatchEvent(new Event('input')); } }
function ramp(key, to, secs) { state.ramps = state.ramps.filter(r => r.key !== key); state.ramps.push({ key, to, v: state[key], rate: (to - state[key]) / secs }); }
function setAsc(mode) { state.asc = mode; document.querySelectorAll('#ascSeg button').forEach(b => b.classList.toggle('on', b.dataset.asc === mode)); $('#manualBox').classList.toggle('off', mode !== 'manual'); if (mode === 'manual') { state.asvMan = Math.round(state.asv); setCtl('asvMan', state.asvMan); } else { state.I = clamp(state.asv, 0, 100); state.asvLag = state.asv; } }
function start() { if (state.tripped) return; state.run = true; state.I = 100; state.asvLag = 100; state.asvStep = 0; state.vibHiT = 0; addEvent('Start — running up with the recycle valve open', 'ok'); }
function stop() { state.run = false; addEvent('Stop — coasting down, recycle opening', 'ok'); }
function resetTrip() { state.tripped = null; state.surgeTimes = []; $('#btnReset').classList.add('hidden'); addEvent('Trip reset', 'ok'); }
const loaded = () => state.run && !state.tripped && state.n >= 99.5 && state.asv <= 1 && R.mp > 0.5;
function whenLoaded(fn) {   // scenarios act on a loaded machine: start it first if needed and wait for the point to settle
  if (loaded()) { state.pending = null; fn(); return; }
  if (state.tripped) { addEvent('Reset the trip first', 'warn'); return; }
  if (!state.run) start(); state.pending = fn; addEvent('Scenario armed — it starts once the machine is at speed and loaded to the process', 'warn');
}
const PRESETS = {
  design: () => { for (const [k, v] of Object.entries({ ps: 5, phdr: 11, nsp: 100, demand: 70, stroke: 2, margin: 10 })) setCtl(k, v); setAsc('auto'); state.ramps = []; state.pending = null; if (!state.run && !state.tripped) start(); },
  sequence: () => { PRESETS.design(); whenLoaded(() => { setCtl('stroke', 15); ramp('demand', 22, 6); addEvent('Scenario: process upset — process valve closing to 22 % in 6 s, recycle valve stroke 15 s', 'warn'); }); },
  closeProc: () => whenLoaded(() => { ramp('demand', 22, 6); addEvent(`Scenario: process upset — process valve closing to 22 % in 6 s (recycle valve stroke ${state.stroke} s)`, 'warn'); }),
  header: () => whenLoaded(() => { ramp('phdr', 13.5, 40); addEvent('Scenario: header pressure rising to 13.5 bar a over 40 s', 'warn'); }),
  speed: () => whenLoaded(() => { setCtl('nsp', 85); addEvent('Scenario: speed setpoint cut to 85 %', 'warn'); }),
  lowPs: () => whenLoaded(() => { ramp('ps', 3.4, 30); addEvent('Scenario: suction pressure falling to 3.4 bar a over 30 s', 'warn'); }),
  manual: () => { PRESETS.design(); whenLoaded(() => { setAsc('manual'); setCtl('asvMan', 0); ramp('demand', 22, 12); addEvent('Scenario: ASC in manual, valve shut, process closing — recover it by hand', 'warn'); }); },
};

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = clamp((now - last) / 1000, 0, 0.1); last = now; state.t += dt;
  if (state.pending && loaded()) { const fn = state.pending; state.pending = null; fn(); }
  for (const r of state.ramps) { r.v += r.rate * dt; const done = r.rate >= 0 ? r.v >= r.to : r.v <= r.to; setCtl(r.key, done ? r.to : r.v); if (done) r.done = true; } state.ramps = state.ramps.filter(r => !r.done);
  const target = state.run && !state.tripped ? state.nsp : 0, rate = (state.run ? (target > state.n && state.n < 90 ? 4 : 2) : 4) * dt;   // run-up 4 %/s, load changes 2 %/s, coast 4 %/s
  state.n += clamp(target - state.n, -rate, rate);
  const nsub = Math.max(1, Math.round(dt / DT)); for (let i = 0; i < nsub; i++) { stepPhysics(DT); state.ctlAcc += DT; if (state.ctlAcc >= 0.01) { control(state.ctlAcc); state.ctlAcc = 0; } }
  state.traceAcc += dt; if (state.traceAcc >= 0.05) { sample(state.traceAcc); state.traceAcc = 0; }
  updateRig(dt);
  frame++; if (frame % 2 === 0) { drawMap(); drawTrace(); updateReadouts(); if (frame % 12 === 0) checkTasks(); }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function bindRange(id, fmtFn) { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = fmtFn(state[id]); }; e.addEventListener('input', set); set(); }
function bind() {
  bindRange('ps', v => v.toFixed(1) + ' bar a'); bindRange('phdr', v => v.toFixed(1) + ' bar a'); bindRange('nsp', v => Math.round(v) + ' %'); bindRange('demand', v => Math.round(v) + ' %'); bindRange('stroke', v => v.toFixed(1) + ' s'); bindRange('margin', v => Math.round(v) + ' %'); bindRange('asvMan', v => Math.round(v) + ' %');
  for (const id of ['ps', 'phdr', 'nsp', 'demand']) $('#' + id).addEventListener('input', e => { if (e.isTrusted) state.ramps = state.ramps.filter(r => r.key !== id); });
  document.querySelectorAll('#ascSeg button').forEach(b => b.onclick = () => setAsc(b.dataset.asc));
  $('#btnStart').onclick = start; $('#btnStop').onclick = stop; $('#btnReset').onclick = resetTrip; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('cm_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase();
    if (e.key === 'ArrowUp') setCtl('nsp', clamp(state.nsp + 1, 60, 105)); else if (e.key === 'ArrowDown') setCtl('nsp', clamp(state.nsp - 1, 60, 105));
    else if (e.key === 'ArrowRight') setCtl('demand', clamp(state.demand + 2, 0, 100)); else if (e.key === 'ArrowLeft') setCtl('demand', clamp(state.demand - 2, 0, 100));
    else if (k === 'S') (state.run ? stop : start)(); else if (k === 'M') setAsc(state.asc === 'auto' ? 'manual' : 'auto'); else if (k === 'R') record(''); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawMap(); drawTrace(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.CompressorMap = {
  getState: () => ({ inputs: { ps: state.ps, phdr: state.phdr, nsp: state.nsp, demand: state.demand, stroke: state.stroke, margin: state.margin, asc: state.asc, asvMan: state.asvMan, run: state.run }, results: { Q: R.c.Q, m: state.m, pd: state.pp / 1e5, PR: R.pr, rpm: state.n / 100 * ND, sm: R.sm, asv: state.asv, vib: state.vib, power: R.power, t2: R.t2 - 273.15, phase: state.phase, surges: state.surges, tripped: state.tripped }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['ps', 'phdr', 'nsp', 'demand', 'stroke', 'margin', 'asvMan']) if (inp[k] !== undefined) { state.ramps = state.ramps.filter(r => r.key !== k); setCtl(k, +inp[k]); } if (inp.asc) setAsc(inp.asc); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.reset) resetTrip(); if (inp.run === true && !state.run) start(); if (inp.run === false && state.run) stop(); },
  meta: { id: 'compressor-map', title: 'Compressor Map Simulator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.CompressorMap.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'compressor-map', state: window.CompressorMap.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('cm_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); renderEvents(); setAsc('auto');
  control(0.01); sample(0.05); updateRig(0); drawMap(); drawTrace(); updateReadouts(); checkTasks();
  requestAnimationFrame(tick);
}
build();
