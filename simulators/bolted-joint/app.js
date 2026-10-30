/* Bolted Joint and Torque Bench — one bolt clamping two plates with an optional gasket. Bolt and member stiffness
   (frustum model), joint diagram, preload from torque with the torque-coefficient scatter of the lubrication state,
   turn-of-nut and stretch methods, embedding loss on a clock, external static or cyclic load, separation, proof-load
   yield and the bolt fatigue factor. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const BOLT = { M6: { d: 6, p: 1, At: 20.1 }, M8: { d: 8, p: 1.25, At: 36.6 }, M10: { d: 10, p: 1.5, At: 58 }, M12: { d: 12, p: 1.75, At: 84.3 }, M16: { d: 16, p: 2, At: 157 }, M20: { d: 20, p: 2.5, At: 245 }, M24: { d: 24, p: 3, At: 353 } };
const GRADE = { '8.8': { Sp: 600, Sy: 660, Su: 830, Se: 129 }, '10.9': { Sp: 830, Sy: 940, Su: 1040, Se: 162 }, '12.9': { Sp: 970, Sy: 1100, Su: 1220, Se: 190 } };
const MEMB = { steel: { name: 'Steel', E: 207000 }, iron: { name: 'Cast iron', E: 100000 }, alu: { name: 'Aluminium', E: 71000 } };
const GASKET = { none: { name: 'No gasket (metal to metal)', E: 0, t: 0 }, fibre: { name: 'Compressed fibre sheet 1.5 mm', E: 1500, t: 1.5 }, spiral: { name: 'Spiral-wound 4.5 mm', E: 800, t: 4.5 }, ptfe: { name: 'PTFE sheet 2 mm', E: 500, t: 2 }, rubber: { name: 'Rubber sheet 3 mm', E: 60, t: 3 } };
const LUBE = { dry: { name: 'Dry, black oxide', K: 0.30, s: 0.35 }, zinc: { name: 'Zinc plated', K: 0.20, s: 0.25 }, oil: { name: 'Oiled', K: 0.18, s: 0.20 }, moly: { name: 'MoS₂ anti-seize', K: 0.12, s: 0.15 } };
const SURF = { machined: { name: 'Machined, clean', fz: 4 }, rough: { name: 'Rough / hot rolled', fz: 9 }, painted: { name: 'Painted surfaces', fz: 16 } };
const METHOD = { torque: { name: 'Torque wrench', s: null }, angle: { name: 'Turn of nut (angle)', s: 0.10 }, stretch: { name: 'Bolt stretch (ultrasonic)', s: 0.03 } };
const E_BOLT = 207000;

/* ------------------------------------------------------------------ state */
const state = { size: 'M12', grade: '8.8', grip: 40, memb: 'steel', gasket: 'none', surf: 'machined', method: 'torque', T: 90, lube: 'zinc', target: 75, Pmax: 20, Pmin: 5, cyc: false, fload: 1, nload: 0.5, tscale: 1, run: true,
  Fi: 0, Fi0: 0, Ksample: null, tight: 0, tphase: 'off', tt: 0, embed: 0, embedMax: 0, P: 0, yielded: false, separated: false, samples: [], t: 0, trace: [], traceAcc: 0, log: [], flags: {} };
const R = {};

/* ------------------------------------------------------------------ model */
function stiffness() {
  const s = state, b = BOLT[s.size], g = GASKET[s.gasket], d = b.d, Ad = Math.PI * d * d / 4, grip = s.grip + g.t, LT = 2 * d + 6, L = grip + 0.8 * d + 2, ld = clamp(L - LT, 0, grip), lt = grip - ld;
  const kb = Ad * b.At * E_BOLT / (Ad * lt + b.At * ld) / 1000; // kN/mm
  const E = MEMB[s.memb].E, t = s.grip / 2, kh = 0.5774 * Math.PI * E * d / Math.log((1.155 * t + 0.5 * d) * 2.5 * d / ((1.155 * t + 2.5 * d) * 0.5 * d)) / 1000;
  let km = kh / 2, kg = Infinity; if (g.t > 0) { const Ag = Math.PI * ((3 * d) ** 2 - (1.1 * d) ** 2) / 4; kg = g.E * Ag / g.t / 1000; km = 1 / (2 / kh + 1 / kg); }
  return { kb, km, kg, kh, C: kb / (kb + km), ld, lt, Ad };
}
function solve() {
  const s = state, b = BOLT[s.size], gr = GRADE[s.grade], st = stiffness(), Fp = gr.Sp * b.At / 1000, lu = LUBE[s.lube], me = METHOD[s.method];
  const FiT = s.T / (lu.K * b.d / 1000) / 1000, sc = me.s === null ? lu.s : me.s, FiTarget = s.method === 'torque' ? FiT : s.target / 100 * Fp, FiMin = FiTarget * (s.method === 'torque' ? 1 / (1 + sc) : 1 - sc), FiMax = FiTarget * (s.method === 'torque' ? 1 / (1 - sc) : 1 + sc);
  const Trec = lu.K * b.d / 1000 * 0.75 * Fp * 1000, nC = s.nload * st.C, Fi = s.Fi, P = s.P;
  const P0 = Fi / (1 - nC), sep = P >= P0 && Fi > 0, Fb = sep ? P : Fi + nC * P, Fm = sep ? 0 : Fi - (1 - nC) * P;
  const sb = Fb * 1000 / b.At, sbMax = (Fi + nC * Math.max(s.Pmax, s.P)) * 1000 / b.At, fz = SURF[s.surf].fz + (s.gasket === 'none' ? 0 : 6), dFembed = fz / 1000 * (st.kb * st.km / (st.kb + st.km));
  const Pa = s.cyc ? (s.Pmax - s.Pmin) / 2 : 0, sa = nC * Pa * 1000 / b.At, si = Fi * 1000 / b.At, sm = si + sa, nf = Pa > 0 && Fi > 0 ? gr.Se * (gr.Su - si) / (sa * (gr.Su + gr.Se)) : Infinity, ny = Fb > 0 ? Fp / (Fi + nC * Math.max(s.Pmax, s.P)) : Infinity, nsep = P0 / Math.max(s.Pmax, s.P, 1e-6);
  return Object.assign({ Fp, FiT, FiTarget, FiMin, FiMax, sc, Trec, nC, P0, sep, Fb, Fm, sb, sbMax, fz, dFembed, Pa, sa, si, sm, nf, ny, nsep, sepMin: FiMin / (1 - nC), yieldMax: FiMax + nC * Math.max(s.Pmax, s.P) > Fp, Kn: lu.K, d: b.d, At: b.At, gr }, st);
}
function sampleK() { const s = state, lu = LUBE[s.lube], me = METHOD[s.method], u = (Math.random() + Math.random() + Math.random() - 1.5) / 1.5; if (s.method === 'torque') return { K: lu.K * (1 + lu.s * u), sc: lu.s }; return { K: null, err: me.s * u, sc: me.s }; }
function preloadFor(sm) { const s = state, b = BOLT[s.size], Fp = GRADE[s.grade].Sp * b.At / 1000; return s.method === 'torque' ? s.T / (sm.K * b.d / 1000) / 1000 : s.target / 100 * Fp * (1 + sm.err); }
function tighten() { const s = state; s.Ksample = sampleK(); s.tphase = 'ramp'; s.tt = 0; s.yielded = false; s.embed = 0; s.tight++; s.run = true; }
function tightenMany(n) { const s = state; for (let i = 0; i < n; i++) { const sm = sampleK(), Fi = preloadFor(sm); s.samples.push({ T: s.T, Fi, K: sm.K, method: s.method, lube: s.lube, size: s.size }); } if (s.samples.length > 400) s.samples.splice(0, s.samples.length - 400); tighten(); s.flags.scatter = s.samples.filter(x => x.method === 'torque' && x.lube === 'dry').length >= 20 || s.flags.scatter; if (s.samples.filter(x => x.method !== 'torque').length >= 20) s.flags.method = true; }
function step(dt) {
  const s = state, dts = dt * s.tscale; if (s.run) s.t += dts;
  if (s.run && s.tphase === 'ramp') { s.tt += dt; const f = Math.min(1, s.tt / 1.5); const Fi = preloadFor(s.Ksample) * f; s.Fi0 = Fi; s.Fi = Fi; if (f >= 1) { s.tphase = 'set'; s.tt = 0; s.embedMax = solve().dFembed; s.samples.push({ T: s.T, Fi: s.Fi0, K: s.Ksample.K, method: s.method, lube: s.lube, size: s.size }); record('tightened'); s.flags.tighten = true; } }
  else if (s.run && s.tphase === 'set') { s.tt += dts; const e = s.embedMax * (1 - Math.exp(-s.tt / 8)); s.embed = Math.min(s.embedMax, e); s.Fi = Math.max(0, s.Fi0 - s.embed); if (s.embed >= 0.05 * s.Fi0 && s.Fi0 > 0) s.flags.embed = true; }
  if (s.run) { s.P = s.cyc ? s.Pmin + (s.Pmax - s.Pmin) * (0.5 - 0.5 * Math.cos(2 * Math.PI * s.fload * s.t)) : s.Pmax; } 
  Object.assign(R, solve());
  if (R.sep && s.Fi > 0) { s.separated = true; s.flags.separate = true; }
  if (s.Fi > 0 && R.Fb > R.Fp && !s.yielded) { s.yielded = true; s.flags.yield = true; record('bolt yielded'); }
  if (s.yielded && R.Fb > R.Fp) { const over = R.Fb - R.Fp; s.Fi0 -= over; s.Fi = Math.max(0, s.Fi - over); }
  if (s.cyc && Number.isFinite(R.nf) && R.nf < 1) s.flags.fatigue = true;
  s.traceAcc += dt; if (s.traceAcc >= 0.1) { s.traceAcc = 0; s.trace.push({ t: s.t, Fb: R.Fb, Fm: R.Fm, P: s.P, Fi: s.Fi }); if (s.trace.length > 1200) s.trace.shift(); }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg;
function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Bolted joint cross-section' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
const arrow = (x1, y1, x2, y2, cls, p) => { el('line', { x1, y1, x2, y2, class: cls }, p); const a = Math.atan2(y2 - y1, x2 - x1); el('polygon', { points: `${x2},${y2} ${x2 - 10 * Math.cos(a - 0.4)},${y2 - 10 * Math.sin(a - 0.4)} ${x2 - 10 * Math.cos(a + 0.4)},${y2 - 10 * Math.sin(a + 0.4)}`, class: cls.replace('arrow', 'head') }, p); };
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, b = BOLT[s.size], g = GASKET[s.gasket], k = Math.min(3.2, 200 / (s.grip + g.t + 2 * b.d)), d = b.d * k, cx = 330, gripPx = s.grip * k, gPx = g.t * k, top = 90, plateW = 300;
  const gap = r.sep ? Math.min(14, 4 + (s.P - r.P0) * 0.4) : 0, t1 = gripPx / 2, t2 = gripPx / 2, y1 = top - gap, y2 = top + t1 + gPx; // top plate lifts when separated
  el('rect', { x: cx - plateW / 2, y: y1, width: plateW, height: t1, class: 'plate' }, svg); el('rect', { x: cx - plateW / 2, y: y2, width: plateW, height: t2, class: 'plate' }, svg);
  if (gPx > 0) el('rect', { x: cx - plateW / 2, y: y1 + t1, width: plateW, height: gPx + gap, class: 'gasket' }, svg);
  el('rect', { x: cx - d * 0.55, y: y1, width: d * 1.1, height: gripPx + gPx + gap, class: 'holecut' }, svg);
  const util = clamp(r.sb / r.gr.Sp, 0, 1.2), bc = `rgba(${Math.round(60 + 170 * Math.min(1, util))},${Math.round(120 - 60 * Math.min(1, util))},${Math.round(200 - 170 * Math.min(1, util))},1)`;
  const bTop = y1 - 0.7 * d, bLen = gripPx + gPx + gap + 0.8 * d + 8 + 0.7 * d; el('rect', { x: cx - d / 2, y: bTop, width: d, height: bLen, fill: bc, class: 'bolt' }, svg);
  const thrTop = bTop + 0.7 * d + r.ld * k; for (let y = thrTop; y < bTop + bLen - 2; y += Math.max(3, b.p * k * 1.2)) el('line', { x1: cx - d / 2, y1: y, x2: cx + d / 2, y2: y + 2, class: 'thread' }, svg);
  el('rect', { x: cx - 0.8 * d, y: bTop, width: 1.6 * d, height: 0.7 * d, class: 'head' }, svg); el('rect', { x: cx - 0.8 * d, y: y2 + t2, width: 1.6 * d, height: 0.8 * d, class: 'head' }, svg); el('rect', { x: cx - 1.1 * d, y: y1 - 3, width: 2.2 * d, height: 3, class: 'washer' }, svg);
  txt(cx + d + 8, bTop + 0.5 * d, `${s.size} ${s.grade} · At ${b.At} mm² · proof ${fmt(r.Fp, 1)} kN`, 'lbl', svg); txt(cx + d + 8, y2 + t2 + 0.6 * d + 4, `nut · thread pitch ${b.p} mm`, 'lbl', svg);
  txt(cx - plateW / 2 - 8, y1 + t1 / 2 + 4, `${MEMB[s.memb].name} ${fmt(s.grip / 2, 0)} mm`, 'lbl', svg, 'end'); txt(cx - plateW / 2 - 8, y2 + t2 / 2 + 4, `${MEMB[s.memb].name} ${fmt(s.grip / 2, 0)} mm`, 'lbl', svg, 'end'); if (gPx > 0) txt(cx - plateW / 2 - 8, y1 + t1 + gPx / 2 + 4, g.name.split(' ')[0] + ' gasket', 'lbl', svg, 'end');
  const Pl = 30 + clamp(s.P / 60, 0, 1) * 60; if (s.P > 0.05) { arrow(cx + plateW / 2 - 40, y1 + 4, cx + plateW / 2 - 40, y1 - Pl, 'parrow', svg); arrow(cx + plateW / 2 - 40, y2 + t2 - 4, cx + plateW / 2 - 40, y2 + t2 + Pl, 'parrow', svg); txt(cx + plateW / 2 - 30, y1 - Pl + 4, `P ${fmt(s.P, 1)} kN`, 'ftxt p', svg); }
  if (s.tphase !== 'off') { const Fl = 30 + clamp(r.Fm / r.Fp, 0, 1) * 50; arrow(cx - plateW / 2 + 40, y1 - Fl, cx - plateW / 2 + 40, y1 - 2, 'farrow', svg); arrow(cx - plateW / 2 + 40, y2 + t2 + Fl, cx - plateW / 2 + 40, y2 + t2 + 2, 'farrow', svg); txt(cx - plateW / 2 + 50, y1 - Fl + 4, `clamp Fm ${fmt(r.Fm, 1)} kN`, 'ftxt f', svg); }
  if (r.sep) txt(cx, y1 + t1 + gPx / 2 + gap / 2 + 4, 'JOINT SEPARATED — gap open', 'val bad', svg, 'middle');
  const wx = 560, wy = 60; if (s.tphase === 'ramp') { const ang = -0.6 + 1.1 * Math.min(1, s.tt / 1.5); el('line', { x1: cx, y1: bTop + 0.35 * d, x2: cx + 220 * Math.cos(ang), y2: bTop + 0.35 * d + 220 * Math.sin(ang), class: 'wrench' }, svg); txt(cx + 230 * Math.cos(ang), bTop + 0.35 * d + 230 * Math.sin(ang), `${s.method === 'torque' ? s.T + ' N·m' : METHOD[s.method].name} · K ${s.Ksample && s.Ksample.K ? fmt(s.Ksample.K, 3) : '—'}`, 'ftxt', svg); }
  const bx = 640; txt(bx, 40, 'Preload from this torque · scatter band', 'lbl', svg); const bw = 200, X = F => bx + clamp(F / (r.Fp * 1.3), 0, 1) * bw;
  el('rect', { x: bx, y: 50, width: bw, height: 16, class: 'tbar' }, svg); el('rect', { x: X(r.FiMin), y: 50, width: Math.max(1, X(r.FiMax) - X(r.FiMin)), height: 16, class: 'band' + (r.FiMax > r.Fp ? ' bad' : '') }, svg); el('line', { x1: X(r.Fp), y1: 44, x2: X(r.Fp), y2: 72, class: 'plim' }, svg); txt(X(r.Fp), 84, `proof ${fmt(r.Fp, 0)} kN`, 'lbl', svg, 'middle'); el('line', { x1: X(r.FiTarget), y1: 48, x2: X(r.FiTarget), y2: 68, class: 'ptarget' }, svg);
  if (s.Fi > 0) { el('polygon', { points: `${X(s.Fi)},68 ${X(s.Fi) - 6},78 ${X(s.Fi) + 6},78`, class: 'pnow' }, svg); } txt(bx, 100, `nominal ${fmt(r.FiTarget, 1)} kN · range ${fmt(r.FiMin, 1)}–${fmt(r.FiMax, 1)} (±${fmt(r.sc * 100, 0)} %)`, 'val', svg);
  txt(bx, 132, 'Bolt stress · share of proof', 'lbl', svg); el('rect', { x: bx, y: 142, width: bw, height: 16, class: 'tbar' }, svg); el('rect', { x: bx, y: 142, width: clamp(r.sb / r.gr.Sp, 0, 1.3) * bw / 1.3, height: 16, class: 'bfill' + (r.sb > r.gr.Sp ? ' bad' : '') }, svg); el('line', { x1: bx + bw / 1.3, y1: 136, x2: bx + bw / 1.3, y2: 164, class: 'plim' }, svg); txt(bx, 178, `σb ${fmt(r.sb, 0)} MPa · ${fmt(r.sb / r.gr.Sp * 100, 0)} % of proof ${r.gr.Sp}`, 'val', svg);
  txt(bx, 210, 'Load partition · this external load', 'lbl', svg); el('rect', { x: bx, y: 220, width: bw, height: 16, class: 'tbar' }, svg); const pP = clamp(s.P / Math.max(r.P0, 1e-6), 0, 1); el('rect', { x: bx, y: 220, width: pP * bw, height: 16, class: 'sfill' + (r.sep ? ' bad' : '') }, svg); txt(bx, 256, `P ${fmt(s.P, 1)} of separation ${fmt(r.P0, 1)} kN`, 'val', svg); txt(bx, 272, `bolt takes ${fmt(r.nC * 100, 0)} % of the load, members ${fmt(100 - r.nC * 100, 0)} %`, 'val', svg);
  txt(bx, 300, `kb ${fmt(r.kb, 0)} · km ${fmt(r.km, 0)} kN/mm · C ${fmt(r.C, 3)}`, 'val', svg); txt(bx, 316, `embedding ${fmt(r.fz, 0)} µm → loses ${fmt(r.dFembed, 1)} kN`, 'val', svg);
  txt(480, 372, s.tphase === 'off' ? 'not tightened — press Tighten' : s.yielded ? `BOLT YIELDED — load reached ${fmt(r.Fb, 1)} kN above proof ${fmt(r.Fp, 1)} kN · preload lost · replace the bolt` : r.sep ? `JOINT SEPARATED at P ${fmt(s.P, 1)} kN — clamp gone, the bolt carries the whole load and every load cycle` : `preload ${fmt(s.Fi, 1)} kN after ${fmt(s.embed, 1)} kN embedding · bolt load ${fmt(r.Fb, 1)} · clamp ${fmt(r.Fm, 1)} kN · separation at ${fmt(r.P0, 1)} kN (×${fmt(r.nsep, 2)})`, 'val' + (s.yielded || r.sep ? ' bad' : ''), svg, 'middle');
  txt(480, 392, s.cyc ? `cyclic ${s.Pmin}–${s.Pmax} kN · bolt alternating stress ${fmt(r.sa, 0)} MPa · fatigue factor ${Number.isFinite(r.nf) ? fmt(r.nf, 2) : '—'}${Number.isFinite(r.nf) && r.nf < 1 ? ' — BOLT WILL FAIL IN FATIGUE' : ''}` : `static load ${s.Pmax} kN · recommended torque for 75 % proof with ${LUBE[s.lube].name.toLowerCase()} threads: ${fmt(r.Trec, 0)} N·m`, 'val' + (s.cyc && Number.isFinite(r.nf) && r.nf < 1 ? ' bad' : ''), svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawJoint() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const Fi = s.Fi > 0 ? s.Fi : r.FiTarget, db = Fi / r.kb, dm = Fi / r.km, Fmax = Math.max(r.Fp * 1.15, Fi + r.nC * s.Pmax + 5, s.Pmax * 1.05), xmax = (db + dm) * 1.25 + (s.Pmax > 0 ? s.Pmax * r.nC / r.kb * 1.5 : 0), X = x => L + x / xmax * pw, Y = F => T + (1 - clamp(F / Fmax, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); const gs = Fmax > 200 ? 50 : Fmax > 80 ? 20 : 10; for (let F = 0; F <= Fmax; F += gs) { ctx.beginPath(); ctx.moveTo(L, Y(F)); ctx.lineTo(L + pw, Y(F)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(F, L - 4, Y(F) + 3); }
  const xs = xmax > 0.4 ? 0.1 : xmax > 0.1 ? 0.02 : 0.01; for (let x = 0; x <= xmax; x += xs) { ctx.beginPath(); ctx.moveTo(X(x), T); ctx.lineTo(X(x), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(fmt(x * 1000, 0), X(x), H - 18); }
  ctx.fillStyle = 'rgba(200,60,40,.08)'; ctx.fillRect(L, Y(r.FiMax), pw, Math.max(1, Y(r.FiMin) - Y(r.FiMax)));
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([6, 3]); ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(L, Y(r.Fp)); ctx.lineTo(L + pw, Y(r.Fp)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.font = '600 10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`proof load ${fmt(r.Fp, 1)} kN`, L + pw - 6, Y(r.Fp) - 4);
  ctx.lineWidth = 2.4; ctx.strokeStyle = col('--water'); ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(db), Y(Fi)); ctx.stroke(); ctx.strokeStyle = col('--good'); ctx.beginPath(); ctx.moveTo(X(db + dm), Y(0)); ctx.lineTo(X(db), Y(Fi)); ctx.stroke();
  const P = s.P, sep = r.sep, dx = sep ? P / r.kb - db : r.nC * P / r.kb, Fb = r.Fb, Fm = r.Fm;
  ctx.strokeStyle = col('--water'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(db), Y(Fi)); ctx.lineTo(X(db + dx), Y(Fb)); ctx.stroke(); ctx.strokeStyle = col('--good'); ctx.beginPath(); ctx.moveTo(X(db), Y(Fi)); ctx.lineTo(X(db + dx), Y(Fm)); ctx.stroke(); ctx.setLineDash([]);
  if (P > 0.05) { ctx.strokeStyle = col('--brass'); ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(X(db + dx), Y(Fm)); ctx.lineTo(X(db + dx), Y(Fb)); ctx.stroke(); ctx.fillStyle = col('--brass'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`P ${fmt(P, 1)} kN`, X(db + dx) + 8, Y((Fb + Fm) / 2) + 4); }
  ctx.fillStyle = col('--water'); ctx.beginPath(); ctx.arc(X(db + dx), Y(Fb), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--good'); ctx.beginPath(); ctx.arc(X(db + dx), Y(Fm), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.beginPath(); ctx.arc(X(db), Y(Fi), 5, 0, 7); ctx.fill();
  ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.textAlign = 'right'; ctx.fillStyle = col('--ink'); ctx.fillText(`preload Fi ${fmt(Fi, 1)} kN`, X(db) - 8, Y(Fi) - 10); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText(`bolt Fb ${fmt(Fb, 1)}`, X(db + dx) + 8, Y(Fb) - 8); ctx.fillStyle = col('--good'); ctx.fillText(`members Fm ${fmt(Fm, 1)}`, X(db + dx) + 8, Y(Fm) + 16);
  ctx.fillStyle = col('--water'); ctx.fillText(`bolt stretches · kb ${fmt(r.kb, 0)} kN/mm`, X(db * 0.15), Y(Fi * 0.15) - 8); ctx.fillStyle = col('--good'); ctx.textAlign = 'right'; ctx.fillText(`members compress · km ${fmt(r.km, 0)} kN/mm`, X(db + dm) - 4, Y(0) - 8);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`deflection µm · force kN · shaded band = preload scatter of this tightening method · separation when the member line reaches zero (P₀ ${fmt(r.P0, 1)} kN)`, L + pw / 2, H - 4);
}
function drawScatter() {
  const cv = $('#schart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R, b = BOLT[s.size], lu = LUBE[s.lube];
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const Tmax = Math.max(s.T * 1.6, 20), Fmax = r.Fp * 1.4, X = t => L + t / Tmax * pw, Y = F => T + (1 - clamp(F / Fmax, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); const ts = Tmax > 400 ? 100 : Tmax > 150 ? 50 : Tmax > 60 ? 20 : 10; for (let t = 0; t <= Tmax; t += ts) { ctx.beginPath(); ctx.moveTo(X(t), T); ctx.lineTo(X(t), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(t, X(t), H - 18); }
  const fs = Fmax > 200 ? 50 : Fmax > 80 ? 20 : 10; for (let F = 0; F <= Fmax; F += fs) { ctx.beginPath(); ctx.moveTo(L, Y(F)); ctx.lineTo(L + pw, Y(F)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(F, L - 4, Y(F) + 3); }
  const line = (K, color, w, dash) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); ctx.moveTo(X(0), Y(0)); const F = Tmax / (K * b.d / 1000) / 1000; ctx.lineTo(X(Tmax), Y(F)); ctx.stroke(); ctx.setLineDash([]); };
  line(lu.K, col('--water'), 2.2, []); line(lu.K * (1 + lu.s), col('--water'), 1.2, [4, 3]); line(lu.K * (1 - lu.s), col('--water'), 1.2, [4, 3]);
  ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 1.4; ctx.setLineDash([6, 3]); ctx.beginPath(); ctx.moveTo(L, Y(r.Fp)); ctx.lineTo(L + pw, Y(r.Fp)); ctx.stroke(); ctx.strokeStyle = col('--brass'); ctx.beginPath(); ctx.moveTo(L, Y(0.75 * r.Fp)); ctx.lineTo(L + pw, Y(0.75 * r.Fp)); ctx.stroke(); ctx.strokeStyle = col('--good'); ctx.beginPath(); ctx.moveTo(L, Y(Math.max(s.Pmax, s.P) * (1 - r.nC))); ctx.lineTo(L + pw, Y(Math.max(s.Pmax, s.P) * (1 - r.nC))); ctx.stroke(); ctx.setLineDash([]);
  ctx.font = '600 10px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); ctx.fillText(`proof ${fmt(r.Fp, 1)} kN — bolt yields above`, L + 6, Y(r.Fp) - 4); ctx.fillStyle = col('--brass'); ctx.fillText('75 % proof target', L + 6, Y(0.75 * r.Fp) - 4); ctx.fillStyle = col('--good'); ctx.fillText(`preload needed to stop separation at ${fmt(Math.max(s.Pmax, s.P), 0)} kN`, L + 6, Y(Math.max(s.Pmax, s.P) * (1 - r.nC)) + 12);
  ctx.fillStyle = col('--water'); ctx.textAlign = 'right'; ctx.fillText(`K ${fmt(lu.K, 2)} ±${fmt(lu.s * 100, 0)} % (${lu.name})`, X(Tmax) - 6, Y(Tmax / (lu.K * b.d / 1000) / 1000) + 14);
  for (const p of s.samples) { if (p.size !== s.size) continue; ctx.fillStyle = p.method === 'torque' ? (p.lube === s.lube ? col('--alarm') : col('--muted')) : col('--good'); ctx.globalAlpha = 0.75; ctx.beginPath(); ctx.arc(X(p.method === 'torque' ? p.T : s.T), Y(p.Fi), 3.5, 0, 7); ctx.fill(); ctx.globalAlpha = 1; }
  const here = s.samples.filter(p => p.size === s.size && p.T === s.T && p.method === s.method && p.lube === s.lube); if (here.length >= 2) { const F = here.map(p => p.Fi), lo = Math.min(...F), hi = Math.max(...F), mean = F.reduce((a, b) => a + b, 0) / F.length; ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`${here.length} bolts at this setting: ${fmt(lo, 1)}–${fmt(hi, 1)} kN, mean ${fmt(mean, 1)} (spread ${fmt((hi - lo) / mean * 100, 0)} %)`, L + 6, T + 14); }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('wrench torque N·m · preload kN · red dots = torque-wrench tightenings at this lubrication, green = angle or stretch method', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 70, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, span = 30, s = state, r = R, tr = s.trace.filter(p => p.t > s.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const X = t => L + (t - (s.t - span)) / span * pw, Fmax = Math.max(r.Fp * 1.1, s.Pmax * 1.1, ...tr.map(p => p.Fb)) * 1.05, Y = F => T + (1 - clamp(F / Fmax, 0, 1)) * ph;
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L, Y(r.Fp)); ctx.lineTo(L + pw, Y(r.Fp)); ctx.stroke(); ctx.setLineDash([]);
  const series = (key, color, w = 1.6, dash = []) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Y(p[key])) : ctx.lineTo(X(p.t), Y(p[key])); f = false; } ctx.stroke(); ctx.setLineDash([]); };
  series('P', col('--brass'), 1.4); series('Fi', col('--ink'), 1.2, [3, 3]); series('Fm', col('--good'), 2); series('Fb', col('--water'), 2.2);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillText(`${fmt(Fmax, 0)} kN`, L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--water'); ctx.fillText('bolt Fb', L + pw + 4, T + 10); ctx.fillStyle = col('--good'); ctx.fillText('clamp Fm', L + pw + 4, T + 24); ctx.fillStyle = col('--brass'); ctx.fillText('load P', L + pw + 4, T + 38); ctx.fillStyle = col('--ink'); ctx.fillText('preload', L + pw + 4, T + 52); ctx.fillStyle = col('--alarm'); ctx.fillText('proof', L + pw + 4, Y(r.Fp) + 3);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span} s · bolt load, clamp load, external load and preload (embedding shows as the preload sagging)`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, b = BOLT[s.size];
  $('#roBolt').textContent = `${s.size} ${s.grade}`; $('#roAt').textContent = b.At; $('#roFp').textContent = fmt(r.Fp, 1);
  $('#roKb').textContent = fmt(r.kb, 0); $('#roKm').textContent = fmt(r.km, 0); $('#roC').textContent = fmt(r.C, 3); $('#roCBox').classList.toggle('warnv', r.C > 0.4);
  $('#roFiT').textContent = fmt(r.FiTarget, 1); $('#roFiMin').textContent = fmt(r.FiMin, 1); $('#roFiMax').textContent = fmt(r.FiMax, 1); $('#roFiTBox').classList.toggle('alarm', r.FiMax > r.Fp); $('#roFiTBox').classList.toggle('warnv', r.FiMax <= r.Fp && r.FiMin < Math.max(s.Pmax, s.P) * (1 - r.nC));
  $('#roFi').textContent = s.tphase === 'off' ? '—' : fmt(s.Fi, 1); $('#roEmb').textContent = fmt(s.embed, 1); $('#roFiPct').textContent = s.tphase === 'off' ? '—' : fmt(s.Fi / r.Fp * 100, 0);
  $('#roFb').textContent = s.tphase === 'off' ? '—' : fmt(r.Fb, 1); $('#roSb').textContent = fmt(r.sb, 0); $('#roSbPct').textContent = fmt(r.sb / r.gr.Sp * 100, 0); $('#roFbBox').classList.toggle('alarm', r.Fb > r.Fp); $('#roFbBox').classList.toggle('warnv', r.Fb <= r.Fp && r.Fb > 0.9 * r.Fp);
  $('#roFm').textContent = s.tphase === 'off' ? '—' : fmt(r.Fm, 1); $('#roFmBox').classList.toggle('alarm', r.sep);
  $('#roP0').textContent = s.tphase === 'off' ? '—' : fmt(r.P0, 1); $('#roNsep').textContent = s.tphase === 'off' ? '—' : fmt(r.nsep, 2); $('#roP0Box').classList.toggle('alarm', s.tphase !== 'off' && r.nsep < 1); $('#roP0Box').classList.toggle('warnv', s.tphase !== 'off' && r.nsep >= 1 && r.nsep < 1.3);
  $('#roNy').textContent = s.tphase === 'off' ? '—' : fmt(r.ny, 2); $('#roNyBox').classList.toggle('alarm', s.tphase !== 'off' && r.ny < 1);
  $('#roNf').textContent = s.cyc && s.tphase !== 'off' ? (Number.isFinite(r.nf) ? fmt(r.nf, 2) : '∞') : '—'; $('#roSa').textContent = fmt(r.sa, 0); $('#roNfBox').classList.toggle('alarm', s.cyc && Number.isFinite(r.nf) && r.nf < 1); $('#roNfBox').classList.toggle('warnv', s.cyc && Number.isFinite(r.nf) && r.nf >= 1 && r.nf < 1.5);
  $('#roTrec').textContent = fmt(r.Trec, 0); $('#roTight').textContent = s.tight;
  const ps = $('#pillState'); ps.textContent = s.tphase === 'off' ? 'Not tightened' : s.tphase === 'ramp' ? 'Tightening' : s.yielded ? 'Bolt yielded' : r.sep ? 'Separated' : s.cyc && Number.isFinite(r.nf) && r.nf < 1 ? 'Fatigue failure' : 'Clamped'; ps.className = 'pill ' + (s.yielded || r.sep || (s.cyc && Number.isFinite(r.nf) && r.nf < 1) ? 'warn' : s.tphase === 'ramp' ? 'amber' : s.tphase === 'off' ? '' : 'ok');
  $('#pillJoint').textContent = `${s.size} ${s.grade} · C ${fmt(r.C, 2)} · Fi ${s.tphase === 'off' ? '—' : fmt(s.Fi, 1) + ' kN'}`; $('#pillSep').classList.toggle('hidden', !r.sep); $('#pillYield').classList.toggle('hidden', !s.yielded); $('#pillFat').classList.toggle('hidden', !(s.cyc && Number.isFinite(r.nf) && r.nf < 1 && s.tphase !== 'off')); $('#pillScatter').classList.toggle('hidden', !(r.FiMax > r.Fp));
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start'; $('#btnCyc').textContent = s.cyc ? 'Cyclic load: on' : 'Cyclic load: off'; $('#btnCyc').classList.toggle('on', s.cyc);
  $('#ctlT').classList.toggle('off', s.method !== 'torque'); $('#ctlTarget').classList.toggle('off', s.method === 'torque');
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, bolt: s.size + ' ' + s.grade, grip: s.grip, memb: s.memb, gasket: s.gasket, method: s.method, lube: s.lube, T: s.method === 'torque' ? s.T : '', K: s.Ksample && s.Ksample.K ? +s.Ksample.K.toFixed(3) : '', kb: +r.kb.toFixed(0), km: +r.km.toFixed(0), C: +r.C.toFixed(3), FiT: +r.FiTarget.toFixed(1), FiMin: +r.FiMin.toFixed(1), FiMax: +r.FiMax.toFixed(1), Fi: +s.Fi.toFixed(1), emb: +s.embed.toFixed(1), Pmax: s.Pmax, Pmin: s.cyc ? s.Pmin : s.Pmax, Fb: +r.Fb.toFixed(1), Fm: +r.Fm.toFixed(1), P0: +r.P0.toFixed(1), ny: +r.ny.toFixed(2), nf: s.cyc ? (Number.isFinite(r.nf) ? +r.nf.toFixed(2) : 'inf') : '', Fp: +r.Fp.toFixed(1), surf: s.surf }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.bolt, r.grip, r.gasket, r.method, r.lube, r.T, r.K, r.kb, r.km, r.C, r.FiT, r.Fi, r.emb, r.Pmax, r.Fb, r.Fm, r.P0, r.ny, r.nf].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,bolt,grip_mm,member,gasket,method,lubrication,torque_Nm,K_sample,kb_kN_mm,km_kN_mm,C,Fi_nominal_kN,Fi_min_kN,Fi_max_kN,Fi_actual_kN,embedding_loss_kN,Pmax_kN,Pmin_kN,Fb_kN,Fm_kN,P_separation_kN,n_yield,n_fatigue,Fp_kN,surface\n' + state.log.map(r => [r.i, r.tag, r.bolt, r.grip, r.memb, r.gasket, r.method, r.lube, r.T, r.K, r.kb, r.km, r.C, r.FiT, r.FiMin, r.FiMax, r.Fi, r.emb, r.Pmax, r.Pmin, r.Fb, r.Fm, r.P0, r.ny, r.nf, r.Fp, r.surf].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'tighten', t: 'Tighten a bolt', d: 'Tighten the M12 with the torque wrench and read the preload, the bolt stress as a share of proof and the load partition on the joint diagram.', ok: () => state.flags.tighten },
  { id: 'scatter', t: 'Torque is not tension', d: 'Set the threads to dry and tighten 20 bolts at the same torque. Read the spread of preload: this is why torque control alone is ±35 %.', ok: () => state.flags.scatter },
  { id: 'lube', t: 'Lubrication changes everything', d: 'Record a tightening dry and one with MoS₂ at the same torque. The lubricated bolt sees far more preload — enough to yield if the torque was set for dry threads.', ok: () => { const L = state.log.filter(x => x.method === 'torque'); return L.some(a => L.some(b => same(a, b, ['bolt', 'T']) && a.lube === 'dry' && b.lube === 'moly')); } },
  { id: 'gasket', t: 'Gasket stiffness', d: 'Record the joint metal-to-metal and again with a rubber gasket. Compare C: the soft gasket hands most of the external load to the bolt.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['bolt', 'grip']) && a.gasket === 'none' && b.gasket === 'rubber' && b.C - a.C >= 0.3)); } },
  { id: 'embed', t: 'Embedding loss', d: 'Tighten a joint with painted or rough surfaces and watch the preload sag by 5 % or more in the first minute as the surfaces bed in.', ok: () => state.flags.embed },
  { id: 'separate', t: 'Separate the joint', d: 'Raise the external load past the separation load: the clamp goes to zero and the bolt carries the whole load, cycle after cycle.', ok: () => state.flags.separate },
  { id: 'yield', t: 'Yield the bolt', d: 'Over-torque, or lubricate a bolt torqued for dry threads, until the bolt load passes the proof load. Note the preload that is lost.', ok: () => state.flags.yield },
  { id: 'fatigue', t: 'Fatigue of the bolt', d: 'With a cyclic load, find a joint whose bolt fatigue factor is below 1 and record it; then fix it (stiffer joint, more preload, bigger bolt) to a factor of 1.5 or more and record that.', ok: () => state.log.some(x => x.nf !== '' && x.nf !== 'inf' && x.nf < 1) && state.log.some(x => x.nf !== '' && (x.nf === 'inf' || x.nf >= 1.5)) },
  { id: 'method', t: 'A better method', d: 'Switch to turn-of-nut or bolt stretch and tighten 20 bolts. Compare the spread with the torque wrench.', ok: () => state.flags.method },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawJoint(); drawScatter(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSel(id, v) { const e = $('#' + id); e.value = v; state[id] = v; }
function loosen() { const s = state; s.Fi = 0; s.Fi0 = 0; s.embed = 0; s.tphase = 'off'; s.yielded = false; s.separated = false; s.Ksample = null; s.t = 0; }
const PRESETS = {
  base: () => { setSel('size', 'M12'); setSel('grade', '8.8'); setSel('memb', 'steel'); setSel('gasket', 'none'); setSel('surf', 'machined'); setSel('method', 'torque'); setSel('lube', 'zinc'); for (const [k, v] of Object.entries({ grip: 40, T: 90, target: 75, Pmax: 20, Pmin: 5, fload: 1, nload: 0.5, tscale: 1 })) setCtl(k, v); state.cyc = false; loosen(); state.run = true; tighten(); },
  scatter: () => { PRESETS.base(); setSel('lube', 'dry'); setCtl('T', 130); loosen(); tightenMany(20); },
  lube: () => { PRESETS.base(); setSel('lube', 'moly'); setCtl('T', 130); loosen(); tighten(); },
  gasket: () => { PRESETS.base(); setSel('gasket', 'rubber'); loosen(); tighten(); },
  embed: () => { PRESETS.base(); setSel('surf', 'painted'); setSel('gasket', 'fibre'); loosen(); tighten(); },
  separate: () => { PRESETS.base(); setCtl('Pmax', 50); loosen(); tighten(); },
  fatigue: () => { PRESETS.base(); setSel('gasket', 'rubber'); setCtl('T', 80); setCtl('Pmax', 30); setCtl('Pmin', 0); state.cyc = true; loosen(); tighten(); },
  stretch: () => { PRESETS.base(); setSel('method', 'stretch'); setCtl('target', 75); loosen(); tightenMany(20); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('grip', v => v + ' mm'); bindR('T', v => v + ' N·m'); bindR('target', v => v + ' % of proof'); bindR('Pmax', v => v + ' kN'); bindR('Pmin', v => v + ' kN'); bindR('fload', v => v.toFixed(1) + ' Hz'); bindR('nload', v => v.toFixed(2)); bindR('tscale', v => '×' + v);
  for (const id of ['size', 'grade', 'memb', 'gasket', 'surf', 'method', 'lube']) $('#' + id).addEventListener('change', e => { state[id] = e.target.value; if (['size', 'grade', 'memb', 'gasket', 'grip'].includes(id)) loosen(); });
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnTighten').onclick = () => tighten(); $('#btnMany').onclick = () => tightenMany(20); $('#btnLoosen').onclick = loosen; $('#btnCyc').onclick = () => { state.cyc = !state.cyc; }; $('#btnRecord').onclick = () => record(''); $('#btnClearS').onclick = () => { state.samples = []; };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('bj_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'T') tighten(); if (k === 'M') tightenMany(20); if (k === 'L') loosen(); if (k === 'C') $('#btnCyc').click(); if (k === 'R') record(''); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.BoltedJoint = {
  getState: () => ({ inputs: { size: state.size, grade: state.grade, grip: state.grip, memb: state.memb, gasket: state.gasket, surf: state.surf, method: state.method, T: state.T, lube: state.lube, target: state.target, Pmax: state.Pmax, Pmin: state.Pmin, cyc: state.cyc, fload: state.fload, nload: state.nload, run: state.run }, results: { kb: R.kb, km: R.km, C: R.C, Fp: R.Fp, FiTarget: R.FiTarget, FiMin: R.FiMin, FiMax: R.FiMax, Fi: state.Fi, embed: state.embed, Fb: R.Fb, Fm: R.Fm, P0: R.P0, sep: R.sep, ny: R.ny, nf: R.nf, yielded: state.yielded, samples: state.samples.length }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['size', 'grade', 'memb', 'gasket', 'surf', 'method', 'lube']) if (inp[k]) setSel(k, inp[k]); for (const k of ['grip', 'T', 'target', 'Pmax', 'Pmin', 'fload', 'nload', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.cyc !== undefined) state.cyc = !!inp.cyc; if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.loosen) loosen(); if (inp.tighten) tighten(); if (inp.tightenMany) tightenMany(+inp.tightenMany); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'bolted-joint', title: 'Bolted Joint and Torque Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.BoltedJoint.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'bolted-joint', state: window.BoltedJoint.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('bj_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); step(0); drawRig(); updateReadouts(); drawJoint(); drawScatter(); drawTrace();
  requestAnimationFrame(tick);
}
build();
