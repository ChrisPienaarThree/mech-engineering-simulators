/* PD Pump Skid — diaphragm or gear pump with transparent lines, pulsation dampener, gauge,
   relief valve, back-pressure valve, calibration column, and the centrifugal-vs-PD closure
   demonstration. Standalone, offline. Skid data illustrative (training only). */
'use strict';

/* ------------------------------------------------------------------ physics (bar, L/min) */
const PUMPS = { diaphragm: { name: 'Diaphragm', Vd: 0.050, Nmax: 120, unit: 'spm', slip0: 0.02, slipP: 0.010, rippleF: 0.55 }, gear: { name: 'Gear', Vd: 0.008, Nmax: 1500, unit: 'rpm', slip0: 0.03, slipP: 0.020, rippleF: 0.06 } };
const SKID = { Pstat: 0.4, hose: 10, stallP: 14, reliefSpan: 0.6, reliefCap: 12, colVol: 500, cent: { P0: 3.4, k: 0.004 } };
function thrK(x) { x = Math.max(0, Math.min(1, x)); return x <= 0.005 ? Infinity : 0.002 * Math.pow(1 / x, 2.2); }   // bar per (L/min)²
function sysP(Q, x) { const K = thrK(x); return Number.isFinite(K) ? SKID.Pstat + K * Q * Q : Infinity; }
function centQ(P) { return P >= SKID.cent.P0 ? 0 : Math.sqrt((SKID.cent.P0 - P) / SKID.cent.k); }          // centrifugal curve P = P0 - kQ²
function centPoint(x) { let lo = 0, hi = SKID.cent.P0; for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (sysP(centQ(m), x) < m) hi = m; else lo = m; } const P = (lo + hi) / 2; return { P, Q: centQ(P) }; }

function solve(s) {
  const pd = PUMPS[s.type], n = s.speedActual / 100, N = n * pd.Nmax, strokeF = s.type === 'diaphragm' ? s.stroke / 100 : 1;
  const Qth = pd.Vd * N * strokeF;                         // theoretical L/min
  const x = s.dthr / 100, reliefOn = !s.reliefIso, Pset = s.relief;
  // suction: starvation when the suction valve is nearly shut
  const sucK = s.sthr <= 8 ? 1 : Math.max(0, 1 - Math.pow(Math.max(0, 25 - s.sthr) / 17, 2)) ; // fraction of chamber fill
  const Psuc = -0.05 - (1 - Math.min(1, s.sthr / 100)) * 0.7;
  let Q = 0, P = 0, Qrel = 0, stalled = false, burst = s.burst;
  if (n > 0.02 && Qth > 0) {
    const flowAt = Pp => Qth * sucK * Math.max(0, 1 - pd.slip0 - pd.slipP * Math.max(0, Pp));
    // pressure the system would take at delivered flow; iterate pressure
    let Pp = SKID.Pstat;
    for (let i = 0; i < 40; i++) {
      const Qp = flowAt(Pp);
      let Pnew;
      if (burst) Pnew = 0.2;
      else if (!Number.isFinite(sysP(Qp, x))) Pnew = 99;                              // shut discharge
      else Pnew = Math.max(sysP(Qp, x), s.bpv);
      // relief valve: opens at Pset, full capacity at Pset + span
      if (reliefOn && Pnew > Pset) {
        // find P in [Pset, Pset+span] where relief flow + system flow = pump flow
        let lo = Pset, hi = Pset + SKID.reliefSpan;
        for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; const qr = SKID.reliefCap * (m - Pset) / SKID.reliefSpan; const K = thrK(x); const qs = Number.isFinite(K) ? Math.sqrt(Math.max(0, (m - SKID.Pstat) / K)) : 0; if (qr + qs < flowAt(m)) lo = m; else hi = m; }
        Pnew = (lo + hi) / 2; if (Pnew > Pset + SKID.reliefSpan) Pnew = Pset + SKID.reliefSpan + 0.5 * (flowAt(Pnew) - SKID.reliefCap);
      }
      Pnew = Math.min(Pnew, 30);
      if (Math.abs(Pnew - Pp) < 0.005) { Pp = Pnew; break; } Pp = 0.5 * Pp + 0.5 * Pnew;
    }
    P = Pp; const Qp = flowAt(P);
    if (reliefOn && P > Pset) Qrel = Math.min(Qp, SKID.reliefCap * Math.min(1, (P - Pset) / SKID.reliefSpan));
    Q = Qp - Qrel;
    if (P >= SKID.stallP) { stalled = true; Q = 0; Qrel = 0; }
  }
  const torque = Math.min(1.6, P / SKID.stallP * 1.15) * (n > 0.02 ? 1 : 0);
  const I = n > 0.02 ? 0.6 + 4.2 * torque : 0;
  const Pshaft = P * 1e5 * (Q + Qrel) / 60000 / 0.55;                                    // W, mechanical efficiency 55 %
  // pulsation: dampener effectiveness depends on charge relative to line pressure
  let damp = 0; if (!s.dampIso && P > 0.3) { const r = s.charge / Math.max(P, 0.3); damp = r < 0.1 ? 0.1 : r > 1.05 ? 0.05 : 0.92 * (1 - Math.pow((r - 0.8) / 0.75, 2)); damp = Math.max(0.05, Math.min(0.92, damp)); }
  const ripple = (n > 0.02 && !stalled) ? pd.rippleF * (1 - damp) : 0;                  // fraction of mean, peak-to-peak
  const Ppeak = P * (1 + ripple / 2);
  return { pd, n, N, Qth, Q, Qrel, P, Ppeak, ripple, damp, torque, I, Pshaft, Psuc, stalled, burst, reliefLift: reliefOn && P > Pset ? Math.min(1, (P - Pset) / SKID.reliefSpan) : 0, over: P > SKID.hose * 0.85, x, cent: centPoint(x), starved: n > 0.02 && sucK < 0.6, sucK };
}

/* ------------------------------------------------------------------ state */
const state = { type: 'diaphragm', speed: 0, speedActual: 0, stroke: 100, dthr: 100, sthr: 100, relief: 6, reliefIso: false, bpv: 2, charge: 1.6, dampIso: false, gaugeDamp: true, burst: false, phase: 0, gaugeP: 0, hist: [], log: [], cal: { on: false, vol: 500, t: 0, startVol: 500, rows: [] }, demo: null, demoTrace: [], flags: {} };
let R = solve(state);
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);

/* ------------------------------------------------------------------ rig svg */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent, anchor) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; if (anchor) t.style.textAnchor = anchor; return t; }
const rig = {}; const PY = 400;
function pipe(svg, d, w, key) { el('path', { d, class: 'pipe-edge', 'stroke-width': w + 4 }, svg); el('path', { d, class: 'pipe', 'stroke-width': w }, svg); const f = el('path', { d, class: 'pipe-fluid', 'stroke-width': w - 4 }, svg); if (key) rig[key] = f; return f; }
function valveSym(svg, x, y, vert, key) { const g = el('g', { transform: `translate(${x} ${y}) rotate(${vert ? 90 : 0})` }, svg); el('path', { d: 'M-12 -9 L0 0 L-12 9 Z', class: 'valve-body' }, g); el('path', { d: 'M12 -9 L0 0 L12 9 Z', class: 'valve-body' }, g); const gate = el('rect', { x: -2, y: -8, width: 4, height: 16, fill: 'var(--panel)' }, g); if (key) rig[key] = gate; return g; }
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 560', role: 'img' }, $('#rig')); rig.svg = svg;
  // tank and calibration column
  el('rect', { x: 40, y: 300, width: 130, height: 200, class: 'reservoir', rx: 4 }, svg); el('rect', { x: 42, y: 360, width: 126, height: 138, class: 'fluid' }, svg); txt(160, 292, 'TANK', 'label', svg, 'end');
  el('rect', { x: 200, y: 220, width: 40, height: 280, class: 'glass', rx: 4 }, svg); rig.col = el('rect', { x: 202, y: 222, width: 36, height: 276, class: 'column' }, svg);
  for (let i = 0; i <= 10; i++) { const y = 498 - i * 27.6; el('line', { x1: 232, y1: y, x2: 240, y2: y, class: 'gauge-tick' }, svg); if (i % 2 === 0) txt(246, y + 3, (i * 50) + '', 'scale-txt', svg); }
  txt(220, 210, 'CALIBRATION COLUMN · 500 mL', 'label', svg, 'middle'); rig.colTxt = txt(220, 196, '', 'gauge-val', svg, 'middle');
  // suction: tank and column both feed a suction manifold → suction valve → pump inlet
  pipe(svg, `M170 470 L300 470 L300 ${PY} L340 ${PY}`, 12, 'sucTank'); rig.sucCol = pipe(svg, `M240 470 L300 470`, 12); valveSym(svg, 300, 436, true, 'sthrGate'); txt(284, 432, 'SUCTION', 'label', svg, 'end'); txt(284, 444, 'VALVE', 'label', svg, 'end');
  rig.calSel = el('rect', { x: 266, y: 462, width: 14, height: 16, fill: 'var(--alarm)', rx: 2, opacity: .0 }, svg); txt(273, 452, 'column select', 'scale-txt', svg, 'middle');
  // pump body
  rig.pumpG = el('g', {}, svg); drawPump();
  // discharge: pump → dampener tee → gauge tee → relief tee → BPV → discharge valve → flowmeter → return to tank top
  pipe(svg, `M460 ${PY} L520 ${PY} L520 140 L860 140 L860 300 L860 470 L840 470`, 12, 'dis'); // main discharge
  // hose section (transparent) marked between 560 and 700 on the top run
  el('rect', { x: 560, y: 128, width: 140, height: 24, fill: 'none', stroke: 'var(--alarm)', 'stroke-dasharray': '4 3', rx: 6 }, svg); txt(630, 118, 'FLEXIBLE HOSE · RATED 10 bar', 'label', svg, 'middle');
  rig.spray = el('g', {}, svg); for (let i = 0; i < 24; i++) el('circle', { r: 2 + (i % 3), class: 'spray' }, rig.spray);
  // dampener on a tee at (520, 300)
  pipe(svg, `M520 300 L470 300`, 10); el('ellipse', { cx: 440, cy: 300, rx: 30, ry: 44, class: 'glass' }, svg); rig.gas = el('path', { d: '', class: 'gas' }, svg); rig.dampLiq = el('path', { d: '', class: 'column' }, svg); txt(440, 222, 'PULSATION DAMPENER', 'label', svg, 'middle'); rig.dampTxt = txt(440, 236, '', 'gauge-val', svg, 'middle'); rig.dampIsoX = cross(svg, 490, 300);
  // gauge on a tee at (520,220)
  pipe(svg, `M520 220 L570 220`, 8); rig.gauge = gauge(svg, 600, 220, 'DISCHARGE', 0, 16, 'bar');
  // relief valve on a tee at (600, 140)? use (700,140) → spring valve → return line down to tank
  pipe(svg, `M700 140 L700 190`, 10); rig.relG = el('g', { transform: 'translate(700 214)' }, svg); el('rect', { x: -14, y: -22, width: 28, height: 44, class: 'check', rx: 3 }, rig.relG); rig.relDisc = el('rect', { x: -10, y: 14, width: 20, height: 5, class: 'valve-body' }, rig.relG); el('path', { d: 'M-8 -18 L8 -14 L-8 -10 L8 -6 L-8 -2 L8 2 L-8 6 L8 10', class: 'spring' }, rig.relG);
  rig.relOut = pipe(svg, `M700 238 L700 300 L760 300 L760 470 L760 480`, 8); txt(730, 250, 'RELIEF VALVE', 'label', svg); rig.relTxt = txt(730, 264, '', 'gauge-val', svg, 'start'); rig.relIsoX = cross(svg, 700, 168);
  el('rect', { x: 740, y: 480, width: 40, height: 30, fill: 'none', stroke: 'var(--steel)', 'stroke-width': 1.5 }, svg); txt(760, 522, 'to tank', 'scale-txt', svg, 'middle');
  // BPV at (860, 220), discharge valve (860, 330), flowmeter at (860, 420)
  valveSym(svg, 860, 220, true, 'bpvGate'); txt(842, 212, 'BACK-PRESSURE', 'label', svg, 'end'); txt(842, 224, 'VALVE', 'label', svg, 'end'); rig.bpvTxt = txt(842, 238, '', 'gauge-val', svg, 'end');
  valveSym(svg, 860, 330, true, 'dthrGate'); txt(842, 322, 'DISCHARGE', 'label', svg, 'end'); txt(842, 334, 'VALVE', 'label', svg, 'end'); rig.dthrTxt = txt(842, 348, '', 'gauge-val', svg, 'end');
  el('rect', { x: 836, y: 404, width: 48, height: 32, class: 'flowmeter', rx: 4 }, svg); rig.rotor = el('g', { transform: 'translate(860 420)' }, svg); for (let i = 0; i < 4; i++) el('line', { x1: 0, y1: 0, x2: 10, y2: 0, transform: `rotate(${i * 90})`, class: 'rotor' }, rig.rotor); rig.qTxt = txt(860, 452, '', 'gauge-val', svg, 'middle'); txt(860, 464, 'FLOWMETER', 'label', svg, 'middle');
  pipe(svg, `M840 470 L790 470 L790 532 L28 532 L28 282 L105 282 L105 302`, 8, 'ret'); // return around to the tank top
  // pulses in the discharge line
  rig.pulses = []; const pg = el('g', {}, svg); rig.disPath = [[460, PY], [520, PY], [520, 140], [860, 140], [860, 300]]; for (let i = 0; i < 12; i++) rig.pulses.push({ s: i / 12, e: el('circle', { r: 5, class: 'pulse' }, pg) });
  rig.parts = []; for (let i = 0; i < 10; i++) rig.parts.push({ p: [[170, 470], [300, 470], [300, PY], [340, PY]], s: i / 10, e: el('circle', { r: 2.2, class: 'particle' }, pg) });
  txt(400, 494, 'PUMP', 'label', svg, 'middle'); rig.spdTxt = txt(400, 508, '', 'gauge-val', svg, 'middle');
}
function cross(svg, x, y) { const g = el('g', { opacity: 0 }, svg); el('line', { x1: x - 8, y1: y - 8, x2: x + 8, y2: y + 8, class: 'tap' }, g); el('line', { x1: x - 8, y1: y + 8, x2: x + 8, y2: y - 8, class: 'tap' }, g); return g; }
function gauge(svg, cx, cy, name, min, max, unit) { const g = el('g', { transform: `translate(${cx} ${cy})` }, svg); el('circle', { r: 26, class: 'gauge-face' }, g); for (let i = 0; i <= 8; i++) { const a = -135 + i * 33.75, r = Math.PI * a / 180; el('line', { x1: 18 * Math.sin(r), y1: -18 * Math.cos(r), x2: 22 * Math.sin(r), y2: -22 * Math.cos(r), class: 'gauge-tick' }, g); } const red = el('path', { d: '', fill: 'none', stroke: 'var(--alarm)', 'stroke-width': 4 }, g); const needle = el('line', { x1: 0, y1: 4, x2: 0, y2: -20, class: 'gauge-needle' }, g); el('circle', { r: 2.5, fill: 'var(--alarm)' }, g); txt(0, 38, name, 'gauge-name', g); const val = txt(0, 50, '', 'gauge-val', g); return { set(v, pset) { const t = Math.max(0, Math.min(1, (v - min) / (max - min))); needle.setAttribute('transform', `rotate(${-135 + 270 * t})`); val.textContent = fmt(v, 2) + ' ' + unit; const a0 = (-135 + 270 * Math.min(1, (pset - min) / (max - min))) * Math.PI / 180, a1 = 135 * Math.PI / 180; red.setAttribute('d', `M${24 * Math.sin(a0)} ${-24 * Math.cos(a0)} A24 24 0 0 1 ${24 * Math.sin(a1)} ${-24 * Math.cos(a1)}`); } }; }
function drawPump() {
  const g = rig.pumpG; g.innerHTML = '';
  if (state.type === 'diaphragm') {
    el('rect', { x: 340, y: PY - 44, width: 120, height: 88, class: 'glass', rx: 8 }, g);
    rig.dia = el('path', { d: '', class: 'diaphragm' }, g);
    el('rect', { x: 340, y: PY - 44, width: 120, height: 88, fill: 'none', stroke: 'var(--steel)', 'stroke-width': 2, rx: 8 }, g);
    rig.cvIn = el('path', { d: `M348 ${PY + 6} L356 ${PY - 4} L364 ${PY + 6} Z`, class: 'check' }, g); rig.cvOut = el('path', { d: `M436 ${PY + 6} L444 ${PY - 4} L452 ${PY + 6} Z`, class: 'check' }, g);
    txt(356, PY + 20, 'inlet check', 'scale-txt', g, 'middle'); txt(444, PY + 20, 'outlet check', 'scale-txt', g, 'middle');
    el('rect', { x: 380, y: PY + 44, width: 40, height: 28, class: 'motor', rx: 4 }, g); txt(400, PY + 63, 'M', 'tag', g, 'middle');
  } else {
    el('rect', { x: 340, y: PY - 44, width: 120, height: 88, class: 'glass', rx: 8 }, g);
    rig.gearA = el('g', { transform: `translate(378 ${PY})` }, g); rig.gearB = el('g', { transform: `translate(422 ${PY})` }, g);
    for (const [gg, n] of [[rig.gearA, 8], [rig.gearB, 8]]) { let d = ''; for (let i = 0; i < n * 2; i++) { const r = i % 2 ? 22 : 16, a = i / (n * 2) * Math.PI * 2; d += (i ? 'L' : 'M') + (r * Math.cos(a)).toFixed(1) + ' ' + (r * Math.sin(a)).toFixed(1) + ' '; } el('path', { d: d + 'Z', class: 'gear' }, gg); el('circle', { r: 4, class: 'check' }, gg); }
    el('rect', { x: 380, y: PY + 44, width: 40, height: 28, class: 'motor', rx: 4 }, g); txt(400, PY + 63, 'M', 'tag', g, 'middle');
  }
}
function pathLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); return L; }
function pointAt(pts, d) { for (let i = 1; i < pts.length; i++) { const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); if (d <= l) { const t = d / l; return [pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * t, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * t]; } d -= l; } return pts[pts.length - 1]; }
function updateRig(dt) {
  const s = state;
  // column
  const h = s.cal.vol / SKID.colVol * 276; rig.col.setAttribute('y', 498 - h); rig.col.setAttribute('height', h); rig.colTxt.textContent = Math.round(s.cal.vol) + ' mL'; rig.calSel.setAttribute('opacity', s.cal.on ? 1 : 0); rig.sucCol.style.opacity = s.cal.on && R.Q > 0 ? .75 : .15; rig.sucTank.style.opacity = !s.cal.on && R.n > .02 ? .75 : .25;
  rig.sthrGate.setAttribute('height', 16 * s.sthr / 100); rig.sthrGate.setAttribute('y', -8 * s.sthr / 100); rig.dthrGate.setAttribute('height', 16 * s.dthr / 100); rig.dthrGate.setAttribute('y', -8 * s.dthr / 100); rig.bpvGate.setAttribute('height', 16 * Math.max(.15, 1 - s.bpv / 6)); rig.bpvGate.setAttribute('y', -8 * Math.max(.15, 1 - s.bpv / 6));
  rig.dthrTxt.textContent = s.dthr + ' %'; rig.bpvTxt.textContent = s.bpv.toFixed(1) + ' bar'; rig.spdTxt.textContent = Math.round(R.N) + ' ' + R.pd.unit;
  // pump animation
  if (s.type === 'diaphragm') { const ph = Math.sin(s.phase); const dx = 400 + 14 * ph; rig.dia.setAttribute('d', `M400 ${PY - 44} Q${dx} ${PY} 400 ${PY + 44} L360 ${PY + 44} L360 ${PY - 44} Z`); rig.dia.setAttribute('d', `M400 ${PY - 40} C${dx + 6} ${PY - 20} ${dx + 6} ${PY + 20} 400 ${PY + 40}`); rig.dia.style.fill = 'none'; rig.dia.style.strokeWidth = 4; const disch = Math.cos(s.phase) < 0; rig.cvOut.style.fill = disch && R.n > .02 ? 'var(--water)' : 'var(--panel)'; rig.cvIn.style.fill = !disch && R.n > .02 ? 'var(--water)' : 'var(--panel)'; }
  else { rig.ga = (rig.ga || 0) + R.N / 60 * 360 * dt * 0.15; rig.gearA.setAttribute('transform', `translate(378 ${PY}) rotate(${rig.ga % 360})`); rig.gearB.setAttribute('transform', `translate(422 ${PY}) rotate(${(-rig.ga + 22.5) % 360})`); }
  // dampener: gas volume vs liquid; charge relative to line pressure
  const liq = s.dampIso ? 0.1 : Math.max(0.08, Math.min(0.92, 1 - s.charge / Math.max(R.P, 0.3) * 0.5)); const yLiq = 344 - liq * 88; rig.dampLiq.setAttribute('d', `M410 300 A30 44 0 0 0 470 300 L470 ${Math.min(344, yLiq)} L410 ${Math.min(344, yLiq)} Z`); rig.gas.setAttribute('d', `M410 300 A30 44 0 0 1 470 300 Z`); rig.dampTxt.textContent = s.dampIso ? 'isolated' : s.charge.toFixed(1) + ' bar · ' + Math.round(R.damp * 100) + ' % effective'; rig.dampIsoX.setAttribute('opacity', s.dampIso ? 1 : 0);
  // gauge: undamped follows instantaneous; glycerine-filled follows mean with slow response
  const inst = instP(); const tau = s.gaugeDamp ? 0.6 : 0.03; s.gaugeP += (inst - s.gaugeP) * Math.min(1, dt / tau); rig.gauge.set(s.gaugeP, s.relief);
  // relief valve
  rig.relDisc.setAttribute('y', 14 - 10 * R.reliefLift); rig.relOut.style.opacity = R.Qrel > 0.05 ? .75 : .15; rig.relTxt.textContent = s.reliefIso ? 'ISOLATED' : (R.reliefLift > 0 ? `lifting ${Math.round(R.reliefLift * 100)} % · ${fmt(R.Qrel, 1)} L/min` : `set ${s.relief.toFixed(1)} bar · closed`); rig.relIsoX.setAttribute('opacity', s.reliefIso ? 1 : 0);
  // flow
  rig.ro = (rig.ro || 0) + R.Q * 40 * dt; rig.rotor.setAttribute('transform', `translate(860 420) rotate(${rig.ro % 360})`); rig.qTxt.textContent = fmt(R.Q, 2) + ' L/min'; rig.dis.style.opacity = R.n > .02 && !R.stalled ? .75 : .25; rig.ret.style.opacity = R.Q > .05 ? .75 : .2;
  const len = pathLen(rig.disPath); for (const p of rig.pulses) { p.s = (p.s + (R.Q + R.Qrel) * 0.9 * dt / (len / 100)) % 1; const [px, py] = pointAt(rig.disPath, p.s * len); p.e.setAttribute('cx', px); p.e.setAttribute('cy', py); const amp = R.ripple * 6; const ph = s.type === 'diaphragm' ? Math.max(0, Math.sin(s.phase - p.s * 6)) : 0.6 + 0.4 * Math.sin(s.phase * 8 - p.s * 20); p.e.setAttribute('r', R.n > .02 && !R.stalled && !R.burst ? 2 + amp * ph : 0); }
  for (const p of rig.parts) { const l2 = pathLen(p.p); p.s = (p.s + (R.Q + R.Qrel) * 0.8 * dt / (l2 / 100)) % 1; const [px, py] = pointAt(p.p, p.s * l2); p.e.setAttribute('cx', px); p.e.setAttribute('cy', py); p.e.style.opacity = R.n > .02 && !s.cal.on ? .85 : 0; }
  rig.spray.querySelectorAll('circle').forEach((c, i) => { if (R.burst) { const u = ((performance.now() / 500) + i * .21) % 1; c.setAttribute('cx', 630 + (i % 5 - 2) * 12 + u * (i % 2 ? 40 : -40)); c.setAttribute('cy', 128 - u * 90 + u * u * 60); c.style.opacity = .8 * (1 - u); } else c.style.opacity = 0; });
}
function instP() { const s = state; if (R.n < .02 || R.stalled) return R.burst ? 0.2 : (R.n < .02 ? 0 : R.P); const w = s.type === 'diaphragm' ? Math.max(0, Math.sin(s.phase)) * 2 - 1 : Math.sin(s.phase * 8); return R.P * (1 + R.ripple / 2 * w); }

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function setup(id, ratio) { const c = $(id), dpr = window.devicePixelRatio || 1, W = c.clientWidth || 900, H = W * ratio; if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); } const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H); return { ctx, W, H }; }
function drawChart() {
  const { ctx, W, H } = setup('#chart', 380 / 900); const m = { l: 50, r: 54, t: 16, b: 38 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const Pmax = 16, Qmax = 30;
  const X = x => m.l + (100 - x) / 100 * pw, YP = p => m.t + ph - Math.min(Pmax, p) / Pmax * ph, YQ = q => m.t + ph - Math.min(Qmax, q) / Qmax * ph;
  ctx.font = '11px ' + col('--mono'); ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.lineWidth = 1; ctx.textAlign = 'center';
  for (let x = 100; x >= 0; x -= 20) { ctx.beginPath(); ctx.moveTo(X(x), m.t); ctx.lineTo(X(x), m.t + ph); ctx.stroke(); ctx.fillText(x + ' %', X(x), H - m.b + 16); }
  ctx.textAlign = 'right'; for (let p = 0; p <= Pmax; p += 4) { ctx.beginPath(); ctx.moveTo(m.l, YP(p)); ctx.lineTo(m.l + pw, YP(p)); ctx.stroke(); ctx.fillText(p, m.l - 8, YP(p) + 4); }
  ctx.textAlign = 'left'; for (let q = 0; q <= Qmax; q += 10) ctx.fillText(q, m.l + pw + 8, YQ(q) + 4);
  ctx.textAlign = 'center'; ctx.font = '600 12px ' + col('--body'); ctx.fillText('Discharge valve opening  (open → shut)', m.l + pw / 2, H - 4); ctx.save(); ctx.translate(14, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('Pressure  (bar)  — solid', 0, 0); ctx.restore(); ctx.save(); ctx.translate(W - 8, m.t + ph / 2); ctx.rotate(Math.PI / 2); ctx.fillText('Flow  (L/min)  — dashed', 0, 0); ctx.restore();
  // relief set line
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(m.l, YP(state.relief)); ctx.lineTo(m.l + pw, YP(state.relief)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.textAlign = 'left'; ctx.font = '11px ' + col('--mono'); ctx.fillText('relief set ' + state.relief.toFixed(1) + ' bar' + (state.reliefIso ? ' — ISOLATED' : ''), m.l + 6, YP(state.relief) - 4);
  ctx.strokeStyle = col('--warn'); ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(m.l, YP(SKID.hose)); ctx.lineTo(m.l + pw, YP(SKID.hose)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--warn'); ctx.fillText('hose rating 10 bar', m.l + 6, YP(SKID.hose) - 4);
  // curves for the current pump settings, computed over the opening range (PD with current relief state; centrifugal)
  const cur = { ...state }; const plot = (fn, style, dash, width = 2.5) => { ctx.beginPath(); ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash || []); let f = true; for (let x = 100; x >= 0; x -= 1) { const y = fn(x); if (!Number.isFinite(y)) { f = true; continue; } f ? ctx.moveTo(X(x), y) : ctx.lineTo(X(x), y); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  if (R.n > .02) {
    const pdAt = x => { const r = solve({ ...cur, dthr: x, burst: false }); return r; };
    const cache = {}; const pd = x => cache[x] || (cache[x] = pdAt(x));
    plot(x => YP(pd(x).P), col('--brass')); plot(x => YQ(pd(x).Q), col('--brass'), [6, 4]);
    plot(x => YP(centPoint(x / 100).P), col('--water')); plot(x => YQ(centPoint(x / 100).Q), col('--water'), [6, 4]);
  }
  for (const r of state.log) { ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(X(r.dthr), YP(r.P), 4, 0, 7); ctx.fill(); }
  if (R.n > .02) { const pulse = 6 + 2 * Math.sin(performance.now() / 300); ctx.fillStyle = R.over ? col('--alarm') : col('--brass'); ctx.beginPath(); ctx.arc(X(state.dthr), YP(R.P), pulse, 0, 7); ctx.fill(); ctx.fillStyle = col('--panel'); ctx.beginPath(); ctx.arc(X(state.dthr), YP(R.P), 2.5, 0, 7); ctx.fill(); ctx.fillStyle = col('--water'); ctx.beginPath(); ctx.arc(X(state.dthr), YP(R.cent.P), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.textAlign = 'left'; ctx.font = '600 12px ' + col('--mono'); ctx.fillText(`PD ${fmt(R.P, 2)} bar · ${fmt(R.Q, 1)} L/min    centrifugal ${fmt(R.cent.P, 2)} bar · ${fmt(R.cent.Q, 1)} L/min`, m.l + 6, m.t + 12); }
  else { ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.font = '600 14px ' + col('--body'); ctx.fillText('Start the pump to trace the curves', m.l + pw / 2, m.t + ph / 2); }
}
function drawStrip() {
  const { ctx, W, H } = setup('#strip', 170 / 900); const m = { l: 40, r: 60, t: 8, b: 18 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const T = 4; const Pmax = Math.max(4, Math.ceil((R.Ppeak * 1.2 + 0.5) / 2) * 2);
  ctx.strokeStyle = col('--grid'); for (let i = 0; i <= 4; i++) { const y = m.t + ph * i / 4; ctx.beginPath(); ctx.moveTo(m.l, y); ctx.lineTo(m.l + pw, y); ctx.stroke(); } ctx.fillStyle = col('--muted'); ctx.font = '10px ' + col('--mono'); ctx.textAlign = 'right'; for (let i = 0; i <= 4; i++) ctx.fillText((Pmax * (1 - i / 4)).toFixed(0) + ' bar', m.l - 4, m.t + ph * i / 4 + 4);
  const now = performance.now() / 1000; const Y = p => m.t + ph - Math.max(0, Math.min(Pmax, p)) / Pmax * ph;
  for (const [k, c] of [['p', col('--brass')], ['g', col('--water-soft')]]) { ctx.beginPath(); ctx.strokeStyle = c; ctx.lineWidth = k === 'p' ? 1.5 : 2.5; let f = true; for (const h of state.hist) { const x = m.l + pw * (1 - (now - h.t) / T); if (x < m.l) continue; f ? ctx.moveTo(x, Y(h[k])) : ctx.lineTo(x, Y(h[k])); f = false; } ctx.stroke(); }
  ctx.fillStyle = col('--ink'); ctx.textAlign = 'left'; ctx.font = '600 11px ' + col('--mono'); ctx.fillText(`mean ${fmt(R.P, 2)} · peak ${fmt(R.Ppeak, 2)} · ripple ${Math.round(R.ripple * 100)} % p-p`, m.l + pw + 6, m.t + 12);
}

/* ------------------------------------------------------------------ readouts, status, log, tasks */
const READ = [['P', 'Discharge pressure (mean)', () => fmt(R.P, 2), 'bar', () => R.over], ['Ppk', 'Peak pressure', () => fmt(R.Ppeak, 2), 'bar', () => R.Ppeak > SKID.hose], ['Q', 'Delivered flow', () => fmt(R.Q, 2), 'L/min'], ['Qth', 'Theoretical displacement flow', () => fmt(R.Qth, 2), 'L/min'], ['slip', 'Volumetric efficiency', () => fmt(R.Qth > 0 ? (R.Q + R.Qrel) / R.Qth * 100 : null, 1), '%'], ['Qrel', 'Relief flow to tank', () => fmt(R.Qrel, 2), 'L/min', () => R.Qrel > .05], ['rip', 'Pulsation (peak-to-peak)', () => Math.round(R.ripple * 100), '% of mean', () => R.ripple > .3], ['tq', 'Motor torque', () => Math.round(R.torque * 100), '% of rated', () => R.torque > 1], ['I', 'Motor current', () => fmt(R.I, 2), 'A', () => R.torque > 1], ['N', 'Pump speed', () => Math.round(R.N), () => R.pd.unit], ['Psuc', 'Suction pressure', () => fmt(R.Psuc, 2), 'bar g', () => R.starved], ['bpv', 'Back-pressure valve', () => state.bpv.toFixed(1), 'bar']];
const TOP = ['P', 'Ppk', 'Q', 'Qth', 'slip', 'Qrel', 'rip', 'tq'];
function buildReadouts() { const mk = (box, keys) => { box.innerHTML = ''; for (const r of READ) { if (keys && !keys.includes(r[0])) continue; const d = document.createElement('div'); d.className = 'ro'; d.dataset.k = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u"></span>`; box.appendChild(d); } }; mk($('#readouts'), TOP); mk($('#dataGrid'), null); }
function updateReadouts() { document.querySelectorAll('.ro[data-k]').forEach(d => { const r = READ.find(x => x[0] === d.dataset.k); if (!r) return; d.querySelector('.v').textContent = r[2](); d.querySelector('.u').textContent = typeof r[3] === 'function' ? r[3]() : r[3]; d.classList.toggle('alarm', !!(r[4] && r[4]())); }); }
function updateStatus() {
  const on = R.n > .02; $('#pillRun').textContent = on ? `${R.pd.name} pump · ${Math.round(R.N)} ${R.pd.unit}` : 'Pump stopped'; $('#pillRun').classList.toggle('on', on);
  $('#pillP').textContent = on ? `${fmt(R.P, 2)} bar · ${fmt(R.Q, 2)} L/min` : '—';
  $('#pillRelief').classList.toggle('hidden', R.reliefLift <= 0); $('#pillRelief').textContent = `Relief valve lifting ${Math.round(R.reliefLift * 100)} % · ${fmt(R.Qrel, 1)} L/min back to tank`;
  $('#pillPuls').classList.toggle('hidden', !(on && R.ripple > .3)); $('#pillOver').classList.toggle('hidden', !(on && R.over && !R.burst && !R.stalled)); $('#pillOver').textContent = `Overpressure · ${fmt(R.P, 1)} bar against a 10 bar hose`;
  $('#pillBurst').classList.toggle('hidden', !R.burst); $('#pillStall').classList.toggle('hidden', !R.stalled); $('#pillSuc').classList.toggle('hidden', !R.starved);
  Object.assign(state.flags, { relief: state.flags.relief || R.reliefLift > 0, burst: state.flags.burst || R.burst, stall: state.flags.stall || R.stalled, puls: state.flags.puls || (on && R.ripple > .3) });
}
function record(tag) { const r = { i: state.log.length + 1, type: R.pd.name, N: Math.round(R.N) + ' ' + R.pd.unit, stroke: state.type === 'diaphragm' ? state.stroke : 100, dthr: state.dthr, relief: state.reliefIso ? 'isolated' : state.relief.toFixed(1) + ' bar', bpv: state.bpv, damp: state.dampIso ? 'isolated' : state.charge.toFixed(1) + ' bar', Q: +R.Q.toFixed(2), P: +R.P.toFixed(2), Ppk: +R.Ppeak.toFixed(2), rip: Math.round(R.ripple * 100), Qrel: +R.Qrel.toFixed(2), tq: Math.round(R.torque * 100), tag }; state.log.push(r); renderLog(); checkTasks(); }
function renderLog() { const tb = $('#logTable tbody'); tb.innerHTML = ''; for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.type, r.N, r.stroke, r.dthr, r.relief, r.bpv, r.damp, r.Q, r.P, r.Ppk, r.rip, r.Qrel, r.tq].map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); } $('#logCount').textContent = state.log.length; const csv = ['n,pump,speed,stroke_pct,discharge_valve_pct,relief,bpv_bar,dampener,Q_Lmin,P_mean_bar,P_peak_bar,ripple_pct,relief_flow_Lmin,torque_pct'].concat(state.log.map(r => [r.i, r.type, r.N, r.stroke, r.dthr, r.relief, r.bpv, r.damp, r.Q, r.P, r.Ppk, r.rip, r.Qrel, r.tq].map(v => `"${v}"`).join(','))).join('\n'); state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv); }
const CALREAD = [['vol', 'Column volume', () => Math.round(state.cal.vol), 'mL'], ['drawn', 'Volume drawn', () => Math.round(state.cal.startVol - state.cal.vol), 'mL'], ['t', 'Elapsed', () => fmt(state.cal.t, 1), 's'], ['qm', 'Measured flow', () => state.cal.t > 0.5 ? fmt((state.cal.startVol - state.cal.vol) / state.cal.t * 60 / 1000, 3) : '—', 'L/min'], ['qt', 'Theoretical', () => fmt(R.Qth, 3), 'L/min'], ['ev', 'Volumetric efficiency', () => state.cal.t > 0.5 && R.Qth > 0 ? fmt((state.cal.startVol - state.cal.vol) / state.cal.t * 60 / 1000 / R.Qth * 100, 1) : '—', '%']];
function buildCal() { const box = $('#calGrid'); box.innerHTML = ''; for (const r of CALREAD) { const d = document.createElement('div'); d.className = 'ro'; d.dataset.c = r[0]; d.innerHTML = `<span class="k">${r[1]}</span><span class="v">—</span><span class="u">${r[3]}</span>`; box.appendChild(d); } }
function updateCal() { document.querySelectorAll('#calGrid .ro').forEach(d => { const r = CALREAD.find(x => x[0] === d.dataset.c); d.querySelector('.v').textContent = r[2](); }); $('#btnCal').textContent = state.cal.on ? 'Stop calibration test' : 'Calibration test: draw from column'; $('#btnCal2').textContent = state.cal.on ? 'Stop test' : 'Start test'; }
function toggleCal() { const c = state.cal; if (c.on) { c.on = false; if (c.t > 0.5) { const qm = (c.startVol - c.vol) / c.t * 60 / 1000; c.rows.push({ i: c.rows.length + 1, type: R.pd.name, N: Math.round(R.N) + ' ' + R.pd.unit, stroke: state.type === 'diaphragm' ? state.stroke : 100, P: +R.P.toFixed(2), vol: Math.round(c.startVol - c.vol), t: +c.t.toFixed(1), qm: +qm.toFixed(3), qt: +R.Qth.toFixed(3), ev: R.Qth > 0 ? +(qm / R.Qth * 100).toFixed(1) : 0 }); const tb = $('#calTable tbody'); tb.innerHTML = c.rows.map(r => `<tr>${[r.i, r.type, r.N, r.stroke, r.P, r.vol, r.t, r.qm, r.qt, r.ev].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); checkTasks(); } } else { if (R.n < .02) { alert('Start the pump first.'); return; } if (c.vol < 60) { alert('Column nearly empty — refill it first.'); return; } c.on = true; c.t = 0; c.startVol = c.vol; document.querySelector('[data-tab="cal"]').click(); } updateCal(); }
const TASKS = [
  { id: 'cal', t: 'Calibrate the pump with the column', d: 'Run at 100 % speed with the discharge open; draw from the column for at least 20 s; compute volumetric efficiency. Repeat at 20 % opening and note the effect of pressure on slip.', ok: () => state.cal.rows.length >= 2 && Math.max(...state.cal.rows.map(r => r.P)) - Math.min(...state.cal.rows.map(r => r.P)) > 1, st: () => state.cal.rows.length ? `${state.cal.rows.length} test(s); last η vol ${state.cal.rows[state.cal.rows.length - 1].ev} %` : '' },
  { id: 'close', t: 'Close the discharge with the relief in service', d: 'Record points at 100, 60, 40, 20 and 0 % opening. Flow stays almost constant until the relief lifts; the centrifugal trace flattens at its shut-off head instead.', ok: () => [100, 60, 40, 20, 0].every(v => state.log.some(r => Math.abs(r.dthr - v) <= 5 && r.relief !== 'isolated')) },
  { id: 'relief', t: 'See the relief valve do its job', d: 'With the discharge shut, all the pump flow returns to the tank through the relief valve at set pressure. Compare the relief flow reading with the theoretical displacement.', ok: () => !!state.flags.relief && state.dthr <= 5 && R.reliefLift > 0.5 },
  { id: 'iso', t: 'Isolate the relief valve and close the discharge', d: 'The pressure climbs past the hose rating and the hose ruptures, or the motor stalls first if you set the relief high. This is why PD relief protection is mandatory. Reset to repair.', ok: () => !!state.flags.burst || !!state.flags.stall },
  { id: 'damp', t: 'Measure what the dampener does', d: 'Record ripple with the dampener in service at about 80 % of line pressure, then isolated, then charged far above line pressure.', ok: () => state.log.some(r => r.damp === 'isolated' && r.rip > 25) && state.log.some(r => r.damp !== 'isolated' && r.rip < 12) },
  { id: 'bpv', t: 'Back-pressure valve and metering accuracy', d: 'With the discharge fully open and the BPV at 0 bar, then at 2 bar, compare delivered flow — the pump meters against a positive head either way, but the BPV keeps the check valves seating cleanly on a low-resistance line.', ok: () => state.log.some(r => r.dthr >= 95 && r.bpv === 0) && state.log.some(r => r.dthr >= 95 && r.bpv >= 2) },
  { id: 'gear', t: 'Diaphragm against gear', d: 'Repeat the closure with the gear pump: same relief logic, far lower pulsation, higher slip at pressure.', ok: () => state.log.some(r => r.type === 'Gear') && state.log.some(r => r.type === 'Diaphragm') },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), frame = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  const ramp = 60 * dt; state.speedActual += Math.max(-ramp, Math.min(ramp, state.speed - state.speedActual)); if (Math.abs(state.speed - state.speedActual) < .05) state.speedActual = state.speed;
  if (state.demo) { const d = state.demo; d.t += dt; state.dthr = Math.max(0, Math.round(100 - d.t / 20 * 100)); $('#dthr').value = state.dthr; if (state.dthr <= d.next) { record('demo'); d.next -= 20; } if (state.dthr <= 0 && d.t > 22) { state.demo = null; $('#btnDemo').textContent = 'Run the closure demonstration'; } }
  R = solve(state);
  // hose rupture and motor stall are latching events
  if (!state.burst && R.n > .02 && R.Ppeak > SKID.hose * 1.05 && !R.stalled) { state.burst = true; R = solve(state); }
  if (R.stalled && state.speedActual > 0) { state.speedActual = Math.max(0, state.speedActual - 200 * dt); }
  state.phase += (R.N / 60) * 2 * Math.PI * dt * (state.type === 'diaphragm' ? 1 : 0.15);
  state.hist.push({ t: now / 1000, p: instP(), g: state.gaugeP }); while (state.hist.length && now / 1000 - state.hist[0].t > 4.2) state.hist.shift();
  if (state.cal.on) { state.cal.t += dt; state.cal.vol = Math.max(0, state.cal.vol - (R.Q + R.Qrel) * 1000 / 60 * dt); if (state.cal.vol <= 0) toggleCal(); }
  updateRig(dt); drawChart(); drawStrip();
  if (frame++ % 6 === 0) { updateReadouts(); updateStatus(); updateCal(); checkTasks(); $('#speedOut').textContent = state.speed + ' %'; $('#speedUnit').textContent = Math.round(R.N) + ' ' + R.pd.unit; $('#strokeOut').textContent = state.stroke + ' %'; $('#dthrOut').textContent = state.dthr + ' %'; $('#sthrOut').textContent = state.sthr + ' %'; $('#reliefOut').textContent = state.relief.toFixed(1) + ' bar'; $('#bpvOut').textContent = state.bpv.toFixed(1) + ' bar'; $('#chargeOut').textContent = state.charge.toFixed(1) + ' bar'; }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function setType(t) { state.type = t; document.querySelectorAll('#typeSeg button').forEach(b => b.classList.toggle('on', b.dataset.type === t)); $('#stroke').disabled = t !== 'diaphragm'; drawPump(); }
function bind() {
  const on = (id, fn) => $('#' + id).addEventListener('input', e => fn(e.target.value));
  on('speed', v => state.speed = +v); on('stroke', v => state.stroke = +v); on('dthr', v => { state.dthr = +v; state.demo = null; }); on('sthr', v => state.sthr = +v); on('relief', v => state.relief = +v); on('bpv', v => state.bpv = +v); on('charge', v => state.charge = +v);
  $('#reliefIso').addEventListener('change', e => state.reliefIso = e.target.checked); $('#dampIso').addEventListener('change', e => state.dampIso = e.target.checked); $('#gaugeDamp').addEventListener('change', e => state.gaugeDamp = e.target.checked);
  document.querySelectorAll('[data-speed]').forEach(b => b.onclick = () => { state.speed = +b.dataset.speed; $('#speed').value = state.speed; }); document.querySelectorAll('[data-dthr]').forEach(b => b.onclick = () => { state.dthr = +b.dataset.dthr; $('#dthr').value = state.dthr; state.demo = null; });
  document.querySelectorAll('#typeSeg button').forEach(b => b.onclick = () => setType(b.dataset.type));
  $('#btnRecord').onclick = () => record('manual'); $('#btnCal').onclick = toggleCal; $('#btnCal2').onclick = toggleCal; $('#btnRefill').onclick = () => { state.cal.vol = SKID.colVol; };
  $('#btnDemo').onclick = () => { if (state.demo) { state.demo = null; $('#btnDemo').textContent = 'Run the closure demonstration'; return; } if (state.speed < 5) { state.speed = 100; $('#speed').value = 100; } state.dthr = 100; $('#dthr').value = 100; state.demo = { t: 0, next: 100 }; $('#btnDemo').textContent = 'Stop demonstration'; };
  $('#btnReset').onclick = () => { Object.assign(state, { speed: 0, speedActual: 0, stroke: 100, dthr: 100, sthr: 100, relief: 6, reliefIso: false, bpv: 2, charge: 1.6, dampIso: false, burst: false, demo: null, flags: {} }); state.cal.on = false; state.cal.vol = SKID.colVol; ['speed', 'stroke', 'dthr', 'sthr', 'relief', 'bpv', 'charge'].forEach(id => $('#' + id).value = state[id]); $('#reliefIso').checked = false; $('#dampIso').checked = false; document.querySelectorAll('.tasks li').forEach(l => l.classList.remove('done')); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('pd_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const st = e.shiftKey ? 10 : 2;
    if (e.key === 'ArrowLeft') { state.dthr = Math.max(0, state.dthr - st); $('#dthr').value = state.dthr; state.demo = null; } else if (e.key === 'ArrowRight') { state.dthr = Math.min(100, state.dthr + st); $('#dthr').value = state.dthr; state.demo = null; }
    else if (e.key === 'ArrowUp') { state.speed = Math.min(100, state.speed + st); $('#speed').value = state.speed; } else if (e.key === 'ArrowDown') { state.speed = Math.max(0, state.speed - st); $('#speed').value = state.speed; }
    else if (e.key === 'd' || e.key === 'D') $('#btnDemo').click(); else if (e.key === 'r' || e.key === 'R') record('manual'); else if (e.key === 'c' || e.key === 'C') toggleCal(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawChart(); drawStrip(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.PDPumpSkid = {
  getState: () => ({ inputs: { type: state.type, speed: state.speed, stroke: state.stroke, dthr: state.dthr, sthr: state.sthr, relief: state.relief, reliefIso: state.reliefIso, bpv: state.bpv, charge: state.charge, dampIso: state.dampIso }, results: { P: R.P, Ppeak: R.Ppeak, Q: R.Q, Qrel: R.Qrel, ripple: R.ripple, torque: R.torque, burst: state.burst, stalled: R.stalled, centP: R.cent.P, centQ: R.cent.Q }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.type && PUMPS[inp.type]) setType(inp.type); for (const k of ['speed', 'stroke', 'dthr', 'sthr', 'relief', 'bpv', 'charge']) if (inp[k] !== undefined) { state[k] = +inp[k]; $('#' + k).value = state[k]; } for (const k of ['reliefIso', 'dampIso']) if (inp[k] !== undefined) { state[k] = !!inp[k]; $('#' + k).checked = state[k]; } if (inp.repair) state.burst = false; },
  meta: { id: 'pd-pump-skid', title: 'Positive-Displacement Pump Skid', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PDPumpSkid.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'pd-pump-skid', state: window.PDPumpSkid.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('pd_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildReadouts(); buildCal(); buildTasks(); bind(); renderLog();
  state.speed = 100; state.speedActual = 100; $('#speed').value = 100; R = solve(state);
  requestAnimationFrame(tick);
}
build();
