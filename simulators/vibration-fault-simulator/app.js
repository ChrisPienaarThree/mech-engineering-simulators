/* Rotating Machinery Fault Simulator — the acceleration signal is synthesised from the machine's
   state, then analysed (FFT, integration to velocity, envelope demodulation, 1× phase against the
   tacho). Peaks appear because the machine produces them. Standalone, offline. Rig data illustrative. */
'use strict';

/* ------------------------------------------------------------------ constants */
const FS = 10240, N = 16384, DT = 1 / FS;
const BRG = { bpfo: 3.585, bpfi: 5.415, bsf: 2.356, ftf: 0.398 };        // 6205 orders of shaft speed
const GEAR = { z1: 23, z2: 57 }, BELT = { ratio: 0.5, order: 0.2513 };
const POINTS = ['MDE', 'B1', 'B2', 'GB', 'FAN'], PNAME = { MDE: 'Motor DE', B1: 'Bearing 1', B2: 'Bearing 2', GB: 'Gearbox', FAN: 'Fan bearing' };
const ZONES = [[1.4, 'A'], [2.8, 'B'], [4.5, 'C'], [1e9, 'D']];
const $ = s => document.querySelector(s);
const fmt = (x, d = 1) => (x === null || x === undefined || !Number.isFinite(x)) ? '—' : x.toFixed(d);

/* ------------------------------------------------------------------ state */
const state = {
  rpm: 1480, rpmActual: 1480, fitBelt: false, fitGear: false, fitDisc2: false,
  ub: { mass: 0, ang: 0, tmass: 0, tang: 0 }, ma: { ang: 0, off: 0, cpl: 'flex' }, loose: { bolt: 100, fit: 0 }, bent: 0,
  brg: { pos: 'B1', type: 'none', sev: 1, lube: 100 }, gear: { type: 'none', sev: 0 }, belt: { type: 'none', sev: 0 }, res: { fn: 1800, zeta: 0.05 },
  point: 'B1', dir: 'H', sweep: null, bump: 0, log: [], flags: {}, blind: null, blindScore: 0, blindTotal: 0, seed: 1,
};
function eff() { return state.blind ? state.blind.params : state; }   // parameters in force (blind scenario overrides the panels)

/* ------------------------------------------------------------------ signal model
   Returns for a (point, dir): sinusoidal velocity components [{f, V mm/s pk, ph rad}], impact trains
   [{rate Hz, amp g, fres Hz, zeta, mod:{f, depth}}], and broadband noise level (g rms). */
function model(point, dir, p) {
  const fr = state.rpmActual / 60; if (fr <= 0) return { comps: [], impacts: [], noise: 0.002, fr };
  const comps = [], impacts = []; let noise = 0.003;
  const add = (f, V, ph) => { if (V > 1e-4) comps.push({ f, V, ph }); };
  const rad = dir !== 'A', H = dir === 'H', Vd = dir === 'V', A = dir === 'A';
  const wPoint = { MDE: 0.35, B1: 1.0, B2: 0.65, GB: 0.25, FAN: 0.15 }[point];
  // unbalance: resultant of fault mass and correction mass (g at 80 mm), plus residual 3 g at 200°
  const ux = p.ub.mass * Math.cos(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.cos(p.ub.tang * Math.PI / 180) + 3 * Math.cos(200 * Math.PI / 180);
  const uy = p.ub.mass * Math.sin(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.sin(p.ub.tang * Math.PI / 180) + 3 * Math.sin(200 * Math.PI / 180);
  const um = Math.hypot(ux, uy), uang = Math.atan2(uy, ux);
  const Vu = 0.2 * um * Math.pow(fr / 24.67, 2) * wPoint * (state.fitDisc2 ? 1.25 : 1);
  if (rad) add(fr, Vu * (H ? 1 : 0.7), uang + 0.5 + (Vd ? -Math.PI / 2 : 0)); else add(fr, Vu * 0.08, uang);
  // misalignment (coupling between MDE and B1)
  const cf = p.ma.cpl === 'rigid' ? 1.5 : p.ma.cpl === 'gear' ? 1.2 : 1; const near = point === 'MDE' || point === 'B1'; const side = point === 'MDE' ? 0 : Math.PI;
  if (near && p.ma.ang > 0) { const a = p.ma.ang * cf; if (A) { add(fr, 0.9 * a, 1.0 + side); add(2 * fr, 0.45 * a, 0.3 + side); } else { add(fr, 0.3 * a, 2.0 + side); add(2 * fr, 0.5 * a, 1.1 + side); } }
  if (near && p.ma.off > 0) { const o = p.ma.off * cf; if (rad) { add(2 * fr, (H ? 6 : 4) * o, 0.7 + side); add(fr, 2 * o, 1.8 + side); add(3 * fr, 1.0 * o, 2.4 + side); } else add(2 * fr, 1.0 * o, 0.7); }
  if (p.ma.cpl === 'gear' && near && (p.ma.ang + p.ma.off) > 0) for (let k = 1; k <= 3; k++) add(k * 44 * fr / 44 * 8, 0.15 * cf * (p.ma.ang + p.ma.off * 4) / k, k);
  // looseness at B2 (bolt) and fit clearance
  const Lb = (100 - p.loose.bolt) / 100, Lf = p.loose.fit / 100;
  if (point === 'B2' && Lb > 0) { const g = Vd ? 1 : H ? 0.4 : 0.15; for (let k = 1; k <= 8; k++) add(k * fr, 1.6 * Lb * g / Math.pow(k, 0.6), k * 0.9); for (const h of [0.5, 1.5, 2.5, 3.5]) add(h * fr, 0.7 * Lb * g, h * 2); impacts.push({ rate: fr, amp: 3 * Lb * g, fres: 900, zeta: 0.08, mod: null }); }
  if (point === 'B2' && Lf > 0) { const g = rad ? 1 : 0.2; for (let k = 1; k <= 10; k++) add(k * fr, 0.9 * Lf * g / Math.pow(k, 0.5), k * 1.7 + 0.4); add(0.5 * fr, 0.5 * Lf * g, 0.2); noise += 0.02 * Lf; }
  // bent shaft: axial 1× at the two ends 180° apart
  if (p.bent > 0) { const b = p.bent; if (A && (point === 'B1' || point === 'B2')) { add(fr, 8 * b, 0.4 + (point === 'B2' ? Math.PI : 0)); add(2 * fr, 2 * b, 1.1); } if (rad && (point === 'B1' || point === 'B2')) add(fr, 3 * b, 2.6); }
  // bearing defect
  if (p.brg.type !== 'none' && point === p.brg.pos && rad) { const sev = p.brg.sev, amp = [0, 0.6, 1.5, 3.5, 7][sev]; const fres = point === 'B1' ? 3500 : 3200; const o = BRG[p.brg.type]; const rate = p.brg.type === 'bsf' ? 2 * o * fr : o * fr;
    const mod = p.brg.type === 'bpfi' ? { f: fr, depth: 0.7 } : p.brg.type === 'bsf' ? { f: BRG.ftf * fr, depth: 0.6 } : null; impacts.push({ rate, amp, fres, zeta: 0.04, mod });
    if (sev >= 3) { for (let k = 1; k <= 3; k++) add(k * rate, 0.15 * sev / k, k); noise += 0.01 * sev; } if (sev === 4) { add(fr, 0.8, 0.3); noise += 0.05; } }
  if (p.brg.lube < 50 && (point === 'B1' || point === 'B2')) noise += 0.04 * (50 - p.brg.lube) / 50;
  // gearbox
  if (state.fitGear && point === 'GB') { const gmf = GEAR.z1 * fr, fo = fr * GEAR.z1 / GEAR.z2, s = p.gear.sev / 100, g = rad ? 1 : 0.5;
    add(gmf, 0.8 * g, 0.2); add(2 * gmf, 0.3 * g, 1.0); add(fo, 0.4 * g, 0.9); add(fr, 0.3 * g, 0.2);
    if (p.gear.type === 'wear') { add(gmf, 2.4 * s * g, 0.2); add(2 * gmf, 1.6 * s * g, 1.0); add(3 * gmf, 1.2 * s * g, 2.1); for (const k of [-1, 1]) add(gmf + k * fr, 0.3 * s * g, 0.5); }
    if (p.gear.type === 'crack') { impacts.push({ rate: fr, amp: 1 + 6 * s * g, fres: 1800, zeta: 0.05, mod: null }); for (let k = -3; k <= 3; k++) if (k) add(gmf + k * fr, (0.3 + 1.2 * s) * g / Math.abs(k), 0.5 * k); }
    if (p.gear.type === 'eccentric') { for (let k = -2; k <= 2; k++) if (k) add(gmf + k * fo, (0.5 + 1.5 * s) * g / Math.abs(k), 0.4 * k); add(fo, 1.5 * s * g, 0.9); } }
  // belt drive to the fan
  if (state.fitBelt && (point === 'FAN' || point === 'B2')) { const fb = BELT.order * fr, ff = fr * BELT.ratio, s = p.belt.sev / 100, g = point === 'FAN' ? 1 : 0.5;
    if (point === 'FAN' && rad) { add(ff, 0.6, 0.3); add(2 * ff, 0.15, 1.2); }
    if (p.belt.type === 'defect' && rad) [0.8, 1.6, 0.6, 0.4].forEach((a, i) => add((i + 1) * fb, a * s * g, i)); if (p.belt.type === 'loose' && rad) { add(ff, 2.5 * s * g, 0.3); add(fb, 0.8 * s * g, 1); noise += 0.01 * s; } if (p.belt.type === 'pulley' && A) { add(ff, 2 * s * g, 0.4); add(fr, 1.5 * s * g, 0.4 + (point === 'B2' ? Math.PI : 0)); } }
  // pedestal resonance: vertical at B1 and B2 amplifies and phase-shifts every velocity component
  if (Vd && (point === 'B1' || point === 'B2')) { const fn = p.res.fn / 60, z = p.res.zeta; for (const c of comps) { const r = c.f / fn; const Hm = 1 / Math.sqrt(Math.pow(1 - r * r, 2) + Math.pow(2 * z * r, 2)); c.V *= Math.min(Hm, 1 / (2 * z)); c.ph -= Math.atan2(2 * z * r, 1 - r * r); } }
  return { comps, impacts, noise, fr };
}

/* ------------------------------------------------------------------ synthesis + analysis */
let acc = new Float64Array(N), vel = new Float64Array(N), re = new Float64Array(N), im = new Float64Array(N);
let rng = 12345; const rnd = () => { rng = (rng * 1664525 + 1013904223) >>> 0; return rng / 4294967296; };
function synth(m) {
  acc.fill(0); const G9 = 9.81;
  for (const c of m.comps) { const w = 2 * Math.PI * c.f, a = w * c.V / 1000; for (let i = 0; i < N; i++) acc[i] += a * Math.cos(w * i * DT + c.ph); }
  for (const im_ of m.impacts) { if (im_.rate <= 0) continue; const T = 1 / im_.rate, wr = 2 * Math.PI * im_.fres, tau = 1 / (wr * im_.zeta); const L = Math.min(N, Math.round(6 * tau * FS)); let t0 = rnd() * T;
    while (t0 < N * DT) { const modf = im_.mod ? (1 + im_.mod.depth * Math.cos(2 * Math.PI * im_.mod.f * t0)) / (1 + im_.mod.depth) : 1; const a0 = im_.amp * G9 * modf * (0.85 + 0.3 * rnd()); const i0 = Math.round(t0 * FS); for (let k = 0; k < L && i0 + k < N; k++) { const t = k * DT; acc[i0 + k] += a0 * Math.exp(-t / tau) * Math.sin(wr * t); } t0 += T * (1 + 0.01 * (rnd() - 0.5)); } }
  const ns = m.noise * G9; for (let i = 0; i < N; i++) acc[i] += ns * (rnd() + rnd() + rnd() - 1.5) * 1.6;
  if (state.bump > 0) { const fn = eff().res.fn / 60, wr = 2 * Math.PI * fn, tau = 1 / (wr * eff().res.zeta); for (let i = 0; i < N; i++) { const t = i * DT - 0.2; if (t > 0) acc[i] += 25 * state.bump * Math.exp(-t / tau) * Math.sin(wr * t); } }
}
function fft(re, im, inv) { const n = re.length; for (let i = 1, j = 0; i < n; i++) { let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; } }
  for (let len = 2; len <= n; len <<= 1) { const ang = 2 * Math.PI / len * (inv ? 1 : -1), wr = Math.cos(ang), wi = Math.sin(ang); for (let i = 0; i < n; i += len) { let cr = 1, ci = 0; for (let j = 0; j < len / 2; j++) { const a = i + j, b = a + len / 2; const tr = re[b] * cr - im[b] * ci, ti = re[b] * ci + im[b] * cr; re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti; const nr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nr; } } }
  if (inv) for (let i = 0; i < n; i++) { re[i] /= n; im[i] /= n; } }
const hann = new Float64Array(N); for (let i = 0; i < N; i++) hann[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / N);
const analysis = { spec: new Float32Array(N / 2), env: new Float32Array(N / 2), df: FS / N, overall: 0, peakG: 0, crest: 0, envMax: 0 };
function analyse() {
  // velocity by frequency-domain integration
  for (let i = 0; i < N; i++) { re[i] = acc[i]; im[i] = 0; } fft(re, im, false);
  for (let k = 0; k < N; k++) { const f = (k < N / 2 ? k : k - N) * FS / N; if (Math.abs(f) < 3) { re[k] = 0; im[k] = 0; continue; } const w = 2 * Math.PI * f; const r = re[k], i_ = im[k]; re[k] = i_ / w; im[k] = -r / w; }
  const vr = re.slice(), vi = im.slice(); fft(vr, vi, true); for (let i = 0; i < N; i++) vel[i] = vr[i] * 1000; // mm/s
  // velocity spectrum (Hann, peak amplitude)
  for (let i = 0; i < N; i++) { re[i] = vel[i] * hann[i]; im[i] = 0; } fft(re, im, false);
  let ss = 0; for (let k = 0; k < N / 2; k++) { const a = 2 * Math.hypot(re[k], im[k]) / (N * 0.5); analysis.spec[k] = a; const f = k * FS / N; if (f >= 10 && f <= 1000) ss += a * a / 2; }
  analysis.overall = Math.sqrt(ss) / 1.22; // Hann incoherent power gain correction ≈ 1.5 → amplitude √1.5
  // envelope: band-pass 2–6 kHz on acceleration, analytic signal, magnitude, spectrum
  for (let i = 0; i < N; i++) { re[i] = acc[i]; im[i] = 0; } fft(re, im, false);
  for (let k = 0; k < N; k++) { const f = k * FS / N; if (k >= N / 2 || f < 2000 || f > 6000) { re[k] = 0; im[k] = 0; } else { re[k] *= 2; im[k] *= 2; } } fft(re, im, true);
  let mean = 0; for (let i = 0; i < N; i++) { re[i] = Math.hypot(re[i], im[i]); mean += re[i]; } mean /= N; for (let i = 0; i < N; i++) { re[i] = (re[i] - mean) * hann[i] / 9.81; im[i] = 0; } fft(re, im, false);
  analysis.envMax = 0; for (let k = 0; k < N / 2; k++) { const a = 2 * Math.hypot(re[k], im[k]) / (N * 0.5); analysis.env[k] = a; if (k > 2 && k * FS / N < 1000 && a > analysis.envMax) analysis.envMax = a; }
  let pk = 0, sq = 0; for (let i = 0; i < N; i++) { const g = Math.abs(acc[i]) / 9.81; if (g > pk) pk = g; sq += g * g; } analysis.peakG = pk; analysis.crest = pk / Math.sqrt(sq / N || 1e-9);
}
function orderPhasor(comps, fr, order) { let x = 0, y = 0; for (const c of comps) if (Math.abs(c.f / fr - order) < 0.02) { x += c.V * Math.cos(c.ph); y += c.V * Math.sin(c.ph); } return { amp: Math.hypot(x, y), ph: ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360 }; }
function topPeaks(spec, fr, n = 3, fmax = 1000) { const out = []; const kmax = Math.min(N / 2 - 2, Math.floor(fmax / analysis.df)); for (let k = 3; k < kmax; k++) if (spec[k] > spec[k - 1] && spec[k] >= spec[k + 1] && spec[k] > 0.05) out.push([k * analysis.df, spec[k]]); return out.sort((a, b) => b[1] - a[1]).slice(0, n).map(([f, a]) => `${(f / fr).toFixed(2)}× ${a.toFixed(2)}`); }

/* ------------------------------------------------------------------ machine drawing */
const NS = 'http://www.w3.org/2000/svg';
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function txt(x, y, s, cls, parent, anchor) { const t = el('text', { x, y, class: cls }, parent); t.textContent = s; if (anchor) t.style.textAnchor = anchor; return t; }
const rig = {}; const SY = 170;
function buildRig() {
  const svg = el('svg', { viewBox: '0 0 900 300', role: 'img' }, $('#rig')); rig.svg = svg;
  el('rect', { x: 20, y: 236, width: 860, height: 10, class: 'pedestal' }, svg); // baseplate
  // motor
  el('rect', { x: 40, y: SY - 40, width: 150, height: 80, class: 'motor-body', rx: 8 }, svg); el('rect', { x: 70, y: SY - 58, width: 60, height: 18, class: 'motor-body', rx: 3 }, svg); txt(115, SY + 6, 'MOTOR', 'tag', svg, 'middle'); rig.rpmTxt = txt(115, SY + 22, '', 'gauge-val', svg, 'middle');
  el('rect', { x: 60, y: SY + 40, width: 20, height: 26, class: 'pedestal' }, svg); el('rect', { x: 150, y: SY + 40, width: 20, height: 26, class: 'pedestal' }, svg);
  // shaft group (animated)
  rig.shaftG = el('g', {}, svg);
  rig.shaft = el('path', { d: '', class: 'shaft', 'stroke-width': 1.5 }, rig.shaftG);
  el('rect', { x: 190, y: SY - 6, width: 22, height: 12, class: 'shaft' }, svg); // motor stub
  // coupling
  el('rect', { x: 212, y: SY - 18, width: 16, height: 36, class: 'disc', rx: 3 }, svg); el('rect', { x: 232, y: SY - 18, width: 16, height: 36, class: 'disc', rx: 3 }, svg); txt(230, SY + 34, 'COUPLING', 'scale-txt', svg, 'middle');
  // tacho
  el('rect', { x: 250, y: SY - 52, width: 14, height: 18, class: 'accel', rx: 2 }, svg); el('line', { x1: 257, y1: SY - 34, x2: 257, y2: SY - 10, class: 'tap' }, svg); txt(257, SY - 58, 'TACHO', 'scale-txt', svg, 'middle');
  rig.tape = el('rect', { x: 254, y: SY - 8, width: 6, height: 4, class: 'tacho' }, rig.shaftG);
  // bearings B1, B2 with pedestals
  for (const [key, x] of [['B1', 300], ['B2', 520]]) { el('rect', { x: x - 16, y: SY + 22, width: 32, height: 44, class: 'pedestal', rx: 3 }, svg); rig[key + 'ped'] = el('rect', { x: x - 20, y: SY + 60, width: 40, height: 8, class: 'pedestal' }, svg); rig[key + 'ring'] = el('circle', { cx: x, cy: SY, r: 22, class: 'brg' }, rig.shaftG); el('circle', { cx: x, cy: SY, r: 12, class: 'brg' }, rig.shaftG); txt(x, SY + 82, key, 'tag', svg, 'middle'); }
  rig.bolt = el('circle', { cx: 536, cy: SY + 64, r: 4, class: 'loosebolt', opacity: 0 }, svg);
  // discs
  rig.disc1 = el('g', { transform: `translate(400 ${SY})` }, rig.shaftG); el('circle', { r: 46, class: 'disc' }, rig.disc1); el('circle', { r: 6, class: 'shaft' }, rig.disc1); el('rect', { x: -2, y: -46, width: 4, height: 8, fill: 'var(--steel)' }, rig.disc1); rig.mass = el('circle', { r: 0, class: 'mass' }, rig.disc1); rig.tmass = el('circle', { r: 0, class: 'mass trial' }, rig.disc1);
  rig.disc2 = el('g', { transform: `translate(460 ${SY})`, opacity: 0 }, rig.shaftG); el('circle', { r: 40, class: 'disc' }, rig.disc2); el('circle', { r: 6, class: 'shaft' }, rig.disc2);
  txt(400, SY + 62, 'DISC 1', 'scale-txt', svg, 'middle'); rig.disc2Txt = txt(460, SY + 56, 'DISC 2', 'scale-txt', svg, 'middle');
  // gearbox (optional) at the shaft end
  rig.gearG = el('g', { opacity: 0 }, svg); el('rect', { x: 600, y: SY - 50, width: 110, height: 100, class: 'gearbox', rx: 6 }, rig.gearG); el('circle', { cx: 635, cy: SY + 10, r: 22, class: 'disc' }, rig.gearG); el('circle', { cx: 680, cy: SY - 14, r: 34, class: 'disc' }, rig.gearG); txt(655, SY + 68, 'GEARBOX 23 : 57', 'scale-txt', rig.gearG, 'middle'); el('rect', { x: 620, y: SY + 50, width: 20, height: 16, class: 'pedestal' }, rig.gearG); el('rect', { x: 680, y: SY + 50, width: 20, height: 16, class: 'pedestal' }, rig.gearG);
  // belt drive (optional): pulley on shaft end, belt up to fan pulley
  rig.beltG = el('g', { opacity: 0 }, svg); el('circle', { cx: 570, cy: SY, r: 18, class: 'pulley' }, rig.beltG); el('circle', { cx: 760, cy: 70, r: 36, class: 'pulley' }, rig.beltG); el('path', { d: `M570 ${SY - 18} L760 34 M570 ${SY + 18} L760 106`, class: 'belt' }, rig.beltG); el('rect', { x: 740, y: 106, width: 40, height: 60, class: 'pedestal' }, rig.beltG); txt(760, 190, 'FAN', 'tag', rig.beltG, 'middle'); rig.fanImp = el('g', { transform: 'translate(760 70)' }, rig.beltG); for (let i = 0; i < 4; i++) el('line', { x1: 0, y1: 0, x2: 28, y2: 0, transform: `rotate(${i * 90})`, class: 'impeller' }, rig.fanImp);
  // accelerometers
  rig.acc = {}; for (const [key, x, y] of [['MDE', 178, SY - 52], ['B1', 300, SY - 34], ['B2', 520, SY - 34], ['GB', 655, SY - 62], ['FAN', 760, 22]]) { const g = el('g', {}, svg); el('rect', { x: x - 7, y: y - 7, width: 14, height: 14, class: 'accel', rx: 2 }, g); el('line', { x1: x, y1: y - 7, x2: x, y2: y - 16, class: 'tap' }, g); rig.acc[key] = g; }
  txt(20, 24, 'ACCELEROMETERS (H · V · A) AT EACH BEARING · TACHO ON THE COUPLING', 'label', svg);
  rig.bentTxt = txt(400, SY - 60, '', 'scale-txt', svg, 'middle');
}
function updateRig(dt) {
  const p = eff(), fr = state.rpmActual / 60;
  rig.ang = (rig.ang || 0) + fr * 360 * dt * 0.06; const a = rig.ang % 360;
  // exaggerated shaft motion at the 1× amplitude of B1 (slow strobe)
  const ph1 = orderPhasor(model('B1', 'H', p).comps, fr || 1, 1), pv1 = orderPhasor(model('B1', 'V', p).comps, fr || 1, 1);
  const t = performance.now() / 1000; const dx = Math.min(6, ph1.amp * 0.8) * Math.sin(2 * Math.PI * 1.2 * t), dy = Math.min(6, pv1.amp * 0.8) * Math.sin(2 * Math.PI * 1.2 * t - Math.PI / 2);
  rig.shaftG.setAttribute('transform', `translate(${fr > 0 ? dx : 0} ${fr > 0 ? dy : 0})`);
  const bow = p.bent * 40 * Math.sin(2 * Math.PI * 0.6 * t); rig.shaft.setAttribute('d', `M248 ${SY - 6} Q 400 ${SY - 6 - bow} 590 ${SY - 6} L590 ${SY + 6} Q 400 ${SY + 6 - bow} 248 ${SY + 6} Z`);
  rig.tape.setAttribute('transform', `rotate(${a} 257 ${SY})`); rig.tape.style.opacity = (a % 360) < 40 ? 1 : 0.25;
  rig.disc1.setAttribute('transform', `translate(400 ${SY}) rotate(${a})`); rig.disc2.setAttribute('transform', `translate(460 ${SY}) rotate(${a})`); rig.disc2.style.opacity = state.fitDisc2 ? 1 : 0; rig.disc2Txt.style.opacity = state.fitDisc2 ? 1 : 0;
  const mr = state.blind ? 0 : Math.sqrt(state.ub.mass) * 1.4, tr = state.blind ? 0 : Math.sqrt(state.ub.tmass) * 1.4; rig.mass.setAttribute('r', mr); rig.mass.setAttribute('cx', 36 * Math.cos(-state.ub.ang * Math.PI / 180)); rig.mass.setAttribute('cy', 36 * Math.sin(-state.ub.ang * Math.PI / 180)); rig.tmass.setAttribute('r', tr); rig.tmass.setAttribute('cx', 36 * Math.cos(-state.ub.tang * Math.PI / 180)); rig.tmass.setAttribute('cy', 36 * Math.sin(-state.ub.tang * Math.PI / 180));
  rig.gearG.style.opacity = state.fitGear ? 1 : 0; rig.beltG.style.opacity = state.fitBelt ? 1 : 0; rig.fanImp.setAttribute('transform', `translate(760 70) rotate(${a * BELT.ratio})`);
  rig.bolt.setAttribute('opacity', !state.blind && p.loose.bolt < 100 ? 1 : 0); rig.B1ring.classList.toggle('bad', !state.blind && p.brg.type !== 'none' && p.brg.pos === 'B1'); rig.B2ring.classList.toggle('bad', !state.blind && p.brg.type !== 'none' && p.brg.pos === 'B2');
  rig.bentTxt.textContent = !state.blind && p.bent > 0 ? `bent shaft ${p.bent.toFixed(2)} mm run-out` : '';
  for (const k in rig.acc) { rig.acc[k].style.opacity = (k === 'GB' && !state.fitGear) || (k === 'FAN' && !state.fitBelt) ? 0 : 1; rig.acc[k].querySelector('rect').classList.toggle('sel', k === state.point); }
  rig.rpmTxt.textContent = Math.round(state.rpmActual) + ' rpm';
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function setup(id, ratio) { const c = $(id), dpr = window.devicePixelRatio || 1, W = c.clientWidth || 900, H = W * ratio; if (c.width !== Math.round(W * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); } const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, W, H); return { ctx, W, H }; }
function faultCursors(fr) { const p = eff(); const c = [['1×', fr, '--muted'], ['2×', 2 * fr, '--muted'], ['BPFO', BRG.bpfo * fr, '--alarm'], ['BPFI', BRG.bpfi * fr, '--alarm'], ['2×BSF', 2 * BRG.bsf * fr, '--alarm'], ['FTF', BRG.ftf * fr, '--alarm']]; if (state.fitGear) c.push(['GMF', GEAR.z1 * fr, '--good'], ['f out', fr * GEAR.z1 / GEAR.z2, '--good']); if (state.fitBelt) c.push(['belt', BELT.order * fr, '--brass'], ['fan 1×', fr * BELT.ratio, '--brass']); c.push(['fn', p.res.fn / 60, '--warn']); return c; }
function drawSpec() {
  const { ctx, W, H } = setup('#spec', 260 / 900); const m = { l: 46, r: 12, t: 12, b: 30 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const fr = Math.max(state.rpmActual / 60, 0.1); const fmax = +$('#fmax').value, orders = $('#orders').checked, logy = $('#logy').checked;
  const kmax = Math.min(N / 2 - 1, Math.floor(fmax / analysis.df)); let amax = 0.5; for (let k = 2; k <= kmax; k++) amax = Math.max(amax, analysis.spec[k]); amax *= 1.15;
  const Y = a => logy ? m.t + ph - Math.max(0, Math.log10(Math.max(a, 0.01) / 0.01) / Math.log10(Math.max(amax, 0.02) / 0.01)) * ph : m.t + ph - Math.min(1, a / amax) * ph; const X = f => m.l + f / fmax * pw;
  ctx.font = '10px ' + col('--mono'); ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.lineWidth = 1; ctx.textAlign = 'center';
  const step = orders ? (fmax / fr > 60 ? 10 : fmax / fr > 25 ? 5 : 1) : fmax / 10; for (let x = 0; ; x += step) { const f = orders ? x * fr : x; if (f > fmax) break; ctx.beginPath(); ctx.moveTo(X(f), m.t); ctx.lineTo(X(f), m.t + ph); ctx.stroke(); ctx.fillText(orders ? x + '×' : Math.round(f) + ' Hz', X(f), H - m.b + 14); }
  ctx.textAlign = 'right'; for (let i = 0; i <= 4; i++) { const a = logy ? 0.01 * Math.pow(amax / 0.01, i / 4) : amax * i / 4; ctx.fillText(a.toFixed(a < 1 ? 2 : 1), m.l - 4, Y(a) + 3); }
  ctx.textAlign = 'center'; ctx.font = '600 11px ' + col('--body'); ctx.fillText(orders ? 'Orders of running speed (' + Math.round(state.rpmActual) + ' rpm = 1×)' : 'Frequency (Hz)', m.l + pw / 2, H - 2); ctx.save(); ctx.translate(11, m.t + ph / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('mm/s peak', 0, 0); ctx.restore();
  if ($('#cursors').checked) for (const [name, f, c] of faultCursors(fr)) { if (f > fmax || f < 1) continue; ctx.strokeStyle = col(c); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(f), m.t); ctx.lineTo(X(f), m.t + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col(c); ctx.font = '9px ' + col('--mono'); ctx.fillText(name, X(f), m.t + 8); }
  ctx.strokeStyle = col('--brass'); ctx.lineWidth = 1.2; ctx.beginPath(); for (let k = 0; k <= kmax; k++) { const x = X(k * analysis.df), y = Y(analysis.spec[k]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
  const peaks = []; for (let k = 3; k < kmax; k++) if (analysis.spec[k] > analysis.spec[k - 1] && analysis.spec[k] >= analysis.spec[k + 1] && analysis.spec[k] > amax * 0.12) peaks.push(k); peaks.sort((a, b) => analysis.spec[b] - analysis.spec[a]); ctx.fillStyle = col('--ink'); ctx.font = '600 10px ' + col('--mono'); for (const k of peaks.slice(0, 5)) { const f = k * analysis.df; ctx.fillText(orders ? (f / fr).toFixed(2) + '×' : Math.round(f) + ' Hz', X(f), Y(analysis.spec[k]) - 4); }
}
function drawWave() {
  const { ctx, W, H } = setup('#wave', 200 / 450); const m = { l: 36, r: 8, t: 8, b: 20 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const Tw = 0.25, n = Math.floor(Tw * FS); let pk = 0.05; for (let i = 0; i < n; i++) pk = Math.max(pk, Math.abs(acc[i]) / 9.81); pk *= 1.1;
  ctx.strokeStyle = col('--grid'); ctx.beginPath(); ctx.moveTo(m.l, m.t + ph / 2); ctx.lineTo(m.l + pw, m.t + ph / 2); ctx.stroke(); ctx.fillStyle = col('--muted'); ctx.font = '9px ' + col('--mono'); ctx.textAlign = 'right'; ctx.fillText('+' + pk.toFixed(1) + ' g', m.l - 3, m.t + 8); ctx.fillText('−' + pk.toFixed(1) + ' g', m.l - 3, m.t + ph); ctx.textAlign = 'center'; for (let s = 0; s <= 0.25; s += 0.05) ctx.fillText(s.toFixed(2) + ' s', m.l + pw * s / Tw, H - 6);
  const fr = state.rpmActual / 60; if (fr > 0) { ctx.strokeStyle = col('--alarm'); for (let t = 0; t < Tw; t += 1 / fr) { ctx.beginPath(); ctx.moveTo(m.l + pw * t / Tw, m.t); ctx.lineTo(m.l + pw * t / Tw, m.t + 8); ctx.stroke(); } }
  ctx.strokeStyle = col('--brass'); ctx.lineWidth = 1; ctx.beginPath(); for (let i = 0; i < n; i++) { const x = m.l + pw * i / n, y = m.t + ph / 2 - acc[i] / 9.81 / pk * ph / 2; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
}
function drawEnv() {
  const { ctx, W, H } = setup('#env', 200 / 450); const m = { l: 36, r: 8, t: 10, b: 20 }, pw = W - m.l - m.r, ph = H - m.t - m.b; const fr = Math.max(state.rpmActual / 60, 0.1); const fmax = Math.min(1000, 12 * fr + 50); const kmax = Math.floor(fmax / analysis.df); let amax = 0.02; for (let k = 3; k <= kmax; k++) amax = Math.max(amax, analysis.env[k]); amax *= 1.2;
  ctx.strokeStyle = col('--grid'); ctx.fillStyle = col('--muted'); ctx.font = '9px ' + col('--mono'); ctx.textAlign = 'center'; for (let o = 0; o * fr <= fmax; o += 1) { const x = m.l + o * fr / fmax * pw; ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke(); if (o % 2 === 0) ctx.fillText(o + '×', x, H - 6); } ctx.textAlign = 'right'; ctx.fillText(amax.toFixed(2) + ' g', m.l - 3, m.t + 8);
  if ($('#cursors').checked) for (const [name, f, c] of faultCursors(fr)) { if (f > fmax || !c.includes('alarm')) continue; const x = m.l + f / fmax * pw; ctx.strokeStyle = col(c); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(x, m.t); ctx.lineTo(x, m.t + ph); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col(c); ctx.fillText(name, x + 24, m.t + 8); }
  ctx.strokeStyle = col('--good'); ctx.lineWidth = 1.2; ctx.beginPath(); for (let k = 2; k <= kmax; k++) { const x = m.l + k * analysis.df / fmax * pw, y = m.t + ph - Math.min(1, analysis.env[k] / amax) * ph; k === 2 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); } ctx.stroke();
  const pk = []; for (let k = 3; k < kmax; k++) if (analysis.env[k] > analysis.env[k - 1] && analysis.env[k] >= analysis.env[k + 1] && analysis.env[k] > amax * 0.25) pk.push(k); pk.sort((a, b) => analysis.env[b] - analysis.env[a]); ctx.fillStyle = col('--ink'); ctx.textAlign = 'center'; ctx.font = '600 10px ' + col('--mono'); for (const k of pk.slice(0, 3)) ctx.fillText((k * analysis.df / fr).toFixed(2) + '×', m.l + k * analysis.df / fmax * pw, m.t + ph - Math.min(1, analysis.env[k] / amax) * ph - 4);
}
function updatePhaseTable() { const p = eff(), fr = state.rpmActual / 60; const tb = $('#phaseTable tbody'); tb.innerHTML = ''; for (const pt of POINTS) { if ((pt === 'GB' && !state.fitGear) || (pt === 'FAN' && !state.fitBelt)) continue; const cells = [PNAME[pt]]; let r2 = 0; for (const d of ['H', 'V', 'A']) { const c = model(pt, d, p).comps; const o1 = orderPhasor(c, fr || 1, 1), o2 = orderPhasor(c, fr || 1, 2); cells.push(fr > 0 ? o1.amp.toFixed(2) : '—', fr > 0 && o1.amp > 0.05 ? Math.round(o1.ph) + '°' : '—'); if (d === 'H') r2 = o1.amp > 0.05 ? o2.amp / o1.amp : 0; } cells.push(fr > 0 ? r2.toFixed(2) : '—'); const tr = document.createElement('tr'); tr.innerHTML = cells.map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); } }

/* ------------------------------------------------------------------ readouts, log, blind, tasks */
function zoneOf(v) { for (const [lim, z] of ZONES) if (v <= lim) return z; return 'D'; }
function updateReadouts() { const fr = state.rpmActual / 60; const z = zoneOf(analysis.overall); $('#ovVal').textContent = fmt(analysis.overall, 2) + ' mm/s'; const zel = $('#ovZone'); zel.textContent = fr > 0 ? 'ISO zone ' + z : '—'; zel.className = 'zone ' + z; $('#pkVal').textContent = fmt(analysis.peakG, 2) + ' g'; $('#cfVal').textContent = 'crest ' + fmt(analysis.crest, 1);
  $('#pillRun').textContent = fr > 0 ? `${Math.round(state.rpmActual)} rpm · ${(fr).toFixed(2)} Hz` : 'Stopped'; $('#pillRun').classList.toggle('on', fr > 0); $('#pillOverall').textContent = `${PNAME[state.point]} ${state.dir} · ${fmt(analysis.overall, 2)} mm/s · zone ${z}`;
  const p = eff(); const faults = []; if (state.blind) faults.push('hidden fault'); else { if (p.ub.mass > 0) faults.push('unbalance'); if (p.ma.ang > 0 || p.ma.off > 0) faults.push('misalignment'); if (p.loose.bolt < 100 || p.loose.fit > 0) faults.push('looseness'); if (p.bent > 0) faults.push('bent shaft'); if (p.brg.type !== 'none') faults.push('bearing ' + p.brg.type.toUpperCase()); if (p.gear.type !== 'none' && state.fitGear) faults.push('gear ' + p.gear.type); if (p.belt.type !== 'none' && state.fitBelt) faults.push('belt ' + p.belt.type); }
  $('#pillFaults').textContent = faults.length ? faults.join(' · ') : 'no faults introduced'; const r = fr / (p.res.fn / 60); $('#pillRes').classList.toggle('hidden', !(fr > 0 && Math.abs(r - 1) < 0.12)); if (fr > 0 && Math.abs(r - 1) < 0.06) state.flags.res = true; }
function record() { const fr = state.rpmActual / 60 || 1; const p = eff(); const c = model(state.point, state.dir, p).comps; const o1 = orderPhasor(c, fr, 1), o2 = orderPhasor(c, fr, 2); let envTop = '—'; { let bk = 0; for (let k = 3; k < Math.floor(600 / analysis.df); k++) if (analysis.env[k] > analysis.env[bk]) bk = k; if (bk) envTop = `${(bk * analysis.df / fr).toFixed(2)}× · ${analysis.env[bk].toFixed(2)}`; }
  const r = { i: state.log.length + 1, pt: state.point, dir: state.dir, rpm: Math.round(state.rpmActual), ov: +analysis.overall.toFixed(2), z: zoneOf(analysis.overall), a1: +o1.amp.toFixed(2), p1: Math.round(o1.ph), a2: +o2.amp.toFixed(2), pk: +analysis.peakG.toFixed(2), cf: +analysis.crest.toFixed(1), tops: topPeaks(analysis.spec, fr).join(' · '), env: envTop, envOrder: envTop === '—' ? 0 : parseFloat(envTop) }; state.log.push(r); renderLog(); checkTasks(); }
function renderLog() { const tb = $('#logTable tbody'); tb.innerHTML = ''; for (const r of state.log) { const tr = document.createElement('tr'); tr.innerHTML = [r.i, r.pt, r.dir, r.rpm, r.ov, r.z, r.a1, r.p1 + '°', r.a2, r.pk, r.cf, r.tops, r.env].map(v => `<td>${v}</td>`).join(''); tb.appendChild(tr); } $('#logCount').textContent = state.log.length; const csv = ['n,point,dir,rpm,overall_mms,zone,1x_mms,1x_phase_deg,2x_mms,peak_g,crest,top_peaks,envelope_top'].concat(state.log.map(r => [r.i, r.pt, r.dir, r.rpm, r.ov, r.z, r.a1, r.p1, r.a2, r.pk, r.cf, r.tops, r.env].map(v => `"${v}"`).join(','))).join('\n'); state.csv = csv; $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv); }
const BLIND = [
  { key: 'unbalance', name: 'Unbalance', params: { ub: { mass: 30, ang: 120, tmass: 0, tang: 0 } }, tell: 'Dominant 1× radial at both bearings, H and V about 90° apart, phase steady, little axial, amplitude rising with speed squared.' },
  { key: 'angular', name: 'Angular misalignment', params: { ma: { ang: 6, off: 0, cpl: 'flex' } }, tell: 'High axial 1× (with 2×) at motor DE and bearing 1, axial phase 180° apart across the coupling.' },
  { key: 'offset', name: 'Parallel misalignment', params: { ma: { ang: 0, off: 0.6, cpl: 'flex' } }, tell: 'Radial 2× larger than 1× at the coupling-side points, radial phase 180° across the coupling.' },
  { key: 'loose', name: 'Mechanical looseness', params: { loose: { bolt: 20, fit: 0 } }, tell: 'A comb of harmonics with half-order lines, strongest vertical at bearing 2, impacts visible in the waveform.' },
  { key: 'bent', name: 'Bent shaft', params: { bent: 0.35 }, tell: 'Axial 1× at both bearings with the two ends 180° apart; radial 1× also present.' },
  { key: 'bpfo', name: 'Bearing outer-race defect', params: { brg: { pos: 'B1', type: 'bpfo', sev: 2, lube: 100 } }, tell: 'Little in the velocity spectrum, a haystack near 3.5 kHz, and clean lines at 3.58× and harmonics in the envelope spectrum with no sidebands.' },
  { key: 'bpfi', name: 'Bearing inner-race defect', params: { brg: { pos: 'B1', type: 'bpfi', sev: 2, lube: 100 } }, tell: 'Envelope lines at 5.42× with 1× sidebands, because the defect passes in and out of the load zone each revolution.' },
  { key: 'crack', name: 'Cracked pinion tooth', params: { gear: { type: 'crack', sev: 70 } }, fit: 'gear', tell: 'Gear-mesh frequency with 1× sidebands at the gearbox, and a once-per-revolution impact in the waveform.' },
  { key: 'belt', name: 'Damaged belt', params: { belt: { type: 'defect', sev: 70 } }, fit: 'belt', tell: 'Sub-synchronous lines at belt frequency and its harmonics at the fan bearing, 2× belt often largest.' },
  { key: 'resonance', name: 'Structural resonance', params: { res: { fn: 1500, zeta: 0.04 }, ub: { mass: 8, ang: 40, tmass: 0, tang: 0 } }, tell: 'Vertical 1× at the pedestal far larger than horizontal, changing sharply with small speed changes and swinging in phase.' },
];
function baseParams() { return JSON.parse(JSON.stringify({ ub: { mass: 0, ang: 0, tmass: 0, tang: 0 }, ma: { ang: 0, off: 0, cpl: 'flex' }, loose: { bolt: 100, fit: 0 }, bent: 0, brg: { pos: 'B1', type: 'none', sev: 1, lube: 100 }, gear: { type: 'none', sev: 0 }, belt: { type: 'none', sev: 0 }, res: { fn: 2400, zeta: 0.05 } })); }
function startBlind() { const sc = BLIND[Math.floor(Math.random() * BLIND.length)]; const params = Object.assign(baseParams(), JSON.parse(JSON.stringify(sc.params))); state.blind = { sc, params, answered: false }; if (sc.fit === 'gear') { state.fitGear = true; $('#fitGear').checked = true; } if (sc.fit === 'belt') { state.fitBelt = true; $('#fitBelt').checked = true; } if (sc.key === 'resonance') { state.rpm = 1500; $('#rpm').value = 1500; }
  document.querySelectorAll('.fpanel').forEach(f => f.classList.add('locked')); $('#btnBlind').textContent = 'Exit blind machine'; $('#btnBlind').classList.add('on'); $('#blindText').textContent = 'A fault is hidden in this machine. Take readings at every point and direction, look at the spectrum, phase table and envelope, then name it.'; $('#blindFb').textContent = ''; $('#blindFb').className = 'feedback';
  const ch = $('#blindChoices'); ch.innerHTML = ''; for (const b of BLIND) { const btn = document.createElement('button'); btn.textContent = b.name; btn.onclick = () => answerBlind(b.key, btn); ch.appendChild(btn); } document.querySelector('[data-tab="blind"]').click(); }
function answerBlind(key, btn) { const bl = state.blind; if (!bl || bl.answered) return; bl.answered = true; const ok = key === bl.sc.key; state.blindTotal++; if (ok) state.blindScore++; document.querySelectorAll('#blindChoices button').forEach(b => { if (b.textContent === bl.sc.name) b.classList.add('right'); if (b === btn && !ok) b.classList.add('wrong'); }); $('#blindFb').className = 'feedback ' + (ok ? 'ok' : 'no'); $('#blindFb').textContent = (ok ? 'Correct. ' : `Not this one — it was ${bl.sc.name.toLowerCase()}. `) + bl.sc.tell; $('#blindScore').textContent = `${state.blindScore} of ${state.blindTotal} diagnosed`; checkTasks(); }
function exitBlind() { state.blind = null; document.querySelectorAll('.fpanel').forEach(f => f.classList.remove('locked')); $('#btnBlind').textContent = 'Blind machine'; $('#btnBlind').classList.remove('on'); $('#blindText').textContent = 'Press "Blind machine" to load a machine with a hidden fault.'; $('#blindChoices').innerHTML = ''; }
const TASKS = [
  { id: 'make1x', t: 'Manufacture the 1× peak', d: 'Add at least 15 g to disc 1 at 1 480 rpm and watch 1× rise at bearing 1 H. Note the phase.', ok: () => { const p = eff(); return !state.blind && p.ub.mass >= 15 && orderPhasor(model('B1', 'H', p).comps, state.rpmActual / 60 || 1, 1).amp > 2.5; } },
  { id: 'balance', t: 'Balance it', d: 'Use the tacho phase to place a correction mass. Done when the resultant unbalance is below 20 % of the mass you added.', ok: () => { const p = eff(); if (state.blind || p.ub.mass < 15) return false; const x = p.ub.mass * Math.cos(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.cos(p.ub.tang * Math.PI / 180), y = p.ub.mass * Math.sin(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.sin(p.ub.tang * Math.PI / 180); return Math.hypot(x, y) < 0.2 * p.ub.mass; }, st: () => { const p = eff(); if (state.blind || p.ub.mass < 1) return ''; const x = p.ub.mass * Math.cos(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.cos(p.ub.tang * Math.PI / 180), y = p.ub.mass * Math.sin(p.ub.ang * Math.PI / 180) + p.ub.tmass * Math.sin(p.ub.tang * Math.PI / 180); return `residual ${Math.hypot(x, y).toFixed(1)} g of ${p.ub.mass} g`; } },
  { id: 'misalign', t: 'Angular misalignment across the coupling', d: 'Set at least 3 mm/100 mm angular, then record axial readings at motor DE and bearing 1. The phases differ by about 180°.', ok: () => !state.blind && eff().ma.ang >= 3 && state.log.some(r => r.pt === 'MDE' && r.dir === 'A' && r.a1 > 1) && state.log.some(r => r.pt === 'B1' && r.dir === 'A' && r.a1 > 1) },
  { id: 'loose', t: 'Looseness harmonics', d: 'Back the bearing 2 bolts off to 40 % or less and record bearing 2 vertical. Count the harmonics and look for half-order lines.', ok: () => !state.blind && eff().loose.bolt <= 40 && state.log.some(r => r.pt === 'B2' && r.dir === 'V' && r.ov > 2) },
  { id: 'bearing', t: 'Find the bearing defect in the envelope', d: 'Fit a stage-2 or worse defect and record at that bearing; the envelope top line should sit at the defect order (3.58× outer, 5.42× inner, 4.71× ball, 0.40× cage).', ok: () => { const p = eff(); if (state.blind || p.brg.type === 'none' || p.brg.sev < 2) return false; const o = p.brg.type === 'bsf' ? 2 * BRG.bsf : BRG[p.brg.type]; return state.log.some(r => r.pt === p.brg.pos && r.dir !== 'A' && Math.abs(r.envOrder - o) / o < 0.05); } },
  { id: 'gear', t: 'Gear-mesh sidebands', d: 'Fit the gearbox, crack a pinion tooth at 40 % or more, and record at the gearbox with F max 5 000 Hz.', ok: () => !state.blind && state.fitGear && eff().gear.type === 'crack' && eff().gear.sev >= 40 && state.log.some(r => r.pt === 'GB') },
  { id: 'res', t: 'Sweep through resonance', d: 'With some unbalance on the disc, sweep the speed through the pedestal natural frequency and record bearing 1 vertical near it.', ok: () => !!state.flags.res && state.log.some(r => r.pt === 'B1' && r.dir === 'V' && Math.abs(r.rpm - eff().res.fn) / eff().res.fn < 0.1) },
  { id: 'blind', t: 'Diagnose three blind machines', d: 'Load a blind machine, take a full route, and name the fault. Three correct diagnoses complete the task.', ok: () => state.blindScore >= 3, st: () => state.blindTotal ? `${state.blindScore} of ${state.blindTotal}` : '' },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); const extra = t.st ? t.st() : ''; li.querySelector('.st').textContent = (li.classList.contains('done') ? 'done ✓ ' : '') + (extra || (li.classList.contains('done') ? '' : 'not yet')); } }

/* ------------------------------------------------------------------ loop */
let last = performance.now(), acc_t = 0, frame = 0;
function tick(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  if (state.sweep) { state.sweep.t += dt; state.rpm = Math.min(3000, Math.round(300 + state.sweep.t / 30 * 2700)); $('#rpm').value = state.rpm; if (state.rpm >= 3000) state.sweep = null; }
  const ramp = 900 * dt; state.rpmActual += Math.max(-ramp, Math.min(ramp, state.rpm - state.rpmActual)); if (Math.abs(state.rpm - state.rpmActual) < 1) state.rpmActual = state.rpm;
  if (state.bump > 0) state.bump = Math.max(0, state.bump - dt / 2.5);
  updateRig(dt);
  acc_t += dt; if (acc_t > 0.3) { acc_t = 0; synth(model(state.point, state.dir, eff())); analyse(); drawSpec(); drawWave(); drawEnv(); updateReadouts(); if (frame++ % 3 === 0) { updatePhaseTable(); checkTasks(); } }
  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ wiring */
function bindRange(id, obj, key, fmtFn, scale = 1) { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { const v = +e.value * scale; if (obj) obj[key] = v; else state[key] = v; if (o) o.textContent = fmtFn(v); }; e.addEventListener('input', set); set(); }
function bind() {
  bindRange('rpm', null, 'rpm', v => v.toLocaleString() + ' rpm'); document.querySelectorAll('[data-rpm]').forEach(b => b.onclick = () => { state.rpm = +b.dataset.rpm; $('#rpm').value = state.rpm; $('#rpmOut').textContent = state.rpm + ' rpm'; state.sweep = null; });
  $('#btnSweep').onclick = () => { state.sweep = { t: 0 }; state.rpm = 300; };
  bindRange('ubMass', state.ub, 'mass', v => v + ' g'); bindRange('ubAng', state.ub, 'ang', v => v + '°'); bindRange('trMass', state.ub, 'tmass', v => v + ' g'); bindRange('trAng', state.ub, 'tang', v => v + '°');
  bindRange('maAng', state.ma, 'ang', v => v.toFixed(1) + ' mm/100 mm'); bindRange('maOff', state.ma, 'off', v => v.toFixed(2) + ' mm'); $('#cplType').addEventListener('change', e => state.ma.cpl = e.target.value);
  bindRange('looseBolt', state.loose, 'bolt', v => v + ' %'); bindRange('looseFit', state.loose, 'fit', v => v + ' %'); bindRange('bent', null, 'bent', v => v.toFixed(2) + ' mm');
  $('#brgPos').addEventListener('change', e => state.brg.pos = e.target.value); $('#brgType').addEventListener('change', e => state.brg.type = e.target.value); bindRange('brgSev', state.brg, 'sev', v => String(v)); bindRange('brgLube', state.brg, 'lube', v => v >= 70 ? 'good' : v >= 40 ? 'marginal' : 'poor');
  $('#gearType').addEventListener('change', e => state.gear.type = e.target.value); bindRange('gearSev', state.gear, 'sev', v => v + ' %'); $('#beltType').addEventListener('change', e => state.belt.type = e.target.value); bindRange('beltSev', state.belt, 'sev', v => v + ' %');
  bindRange('fn', state.res, 'fn', v => v.toLocaleString() + ' cpm'); bindRange('zeta', state.res, 'zeta', v => v.toFixed(2)); $('#btnBump').onclick = () => { if (state.rpmActual > 0) { alert('Stop the machine first — a bump test needs it standing still.'); return; } state.bump = 1; };
  for (const id of ['fitBelt', 'fitGear', 'fitDisc2']) $('#' + id).addEventListener('change', e => { state[id] = e.target.checked; if (!state.fitGear && state.point === 'GB') { state.point = 'B1'; $('#point').value = 'B1'; } if (!state.fitBelt && state.point === 'FAN') { state.point = 'B1'; $('#point').value = 'B1'; } });
  document.querySelectorAll('#ftabs button').forEach(b => b.onclick = () => { document.querySelectorAll('#ftabs button').forEach(x => x.classList.remove('on')); document.querySelectorAll('.fpanel').forEach(x => x.classList.remove('on')); b.classList.add('on'); $('#f-' + b.dataset.f).classList.add('on'); });
  $('#point').addEventListener('change', e => { const v = e.target.value; if ((v === 'GB' && !state.fitGear) || (v === 'FAN' && !state.fitBelt)) { alert('Fit that component first (checkboxes above the machine).'); e.target.value = state.point; return; } state.point = v; });
  document.querySelectorAll('#dirSeg button').forEach(b => b.onclick = () => { state.dir = b.dataset.dir; document.querySelectorAll('#dirSeg button').forEach(x => x.classList.toggle('on', x === b)); });
  ['fmax', 'orders', 'cursors', 'logy'].forEach(id => $('#' + id).addEventListener('change', () => { drawSpec(); drawEnv(); }));
  $('#btnRecord').onclick = record; $('#btnBlind').onclick = () => state.blind ? exitBlind() : startBlind();
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('vf_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const k = e.key.toUpperCase();
    if (e.key === 'ArrowUp') { state.rpm = Math.min(3000, state.rpm + (e.shiftKey ? 100 : 10)); $('#rpm').value = state.rpm; $('#rpmOut').textContent = state.rpm + ' rpm'; } else if (e.key === 'ArrowDown') { state.rpm = Math.max(0, state.rpm - (e.shiftKey ? 100 : 10)); $('#rpm').value = state.rpm; $('#rpmOut').textContent = state.rpm + ' rpm'; }
    else if (['H', 'V', 'A'].includes(k)) document.querySelector(`#dirSeg [data-dir="${k}"]`).click(); else if (/^[1-5]$/.test(k)) { const pt = POINTS[+k - 1]; if (!((pt === 'GB' && !state.fitGear) || (pt === 'FAN' && !state.fitBelt))) { state.point = pt; $('#point').value = pt; } } else if (k === 'R') record(); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawSpec(); drawWave(); drawEnv(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.VibrationFaultSim = {
  getState: () => ({ inputs: { rpm: state.rpm, point: state.point, dir: state.dir, fitBelt: state.fitBelt, fitGear: state.fitGear, faults: state.blind ? 'hidden' : { ub: state.ub, ma: state.ma, loose: state.loose, bent: state.bent, brg: state.brg, gear: state.gear, belt: state.belt, res: state.res } }, results: { overall: analysis.overall, zone: zoneOf(analysis.overall), peakG: analysis.peakG, crest: analysis.crest, rpm: state.rpmActual }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.rpm !== undefined) { state.rpm = +inp.rpm; $('#rpm').value = state.rpm; } if (inp.point && POINTS.includes(inp.point)) { state.point = inp.point; $('#point').value = inp.point; } if (inp.dir) document.querySelector(`#dirSeg [data-dir="${inp.dir}"]`)?.click(); for (const k of ['fitBelt', 'fitGear', 'fitDisc2']) if (inp[k] !== undefined) { state[k] = !!inp[k]; $('#' + k).checked = state[k]; } if (inp.faults && !state.blind) { for (const g of ['ub', 'ma', 'loose', 'brg', 'gear', 'belt', 'res']) if (inp.faults[g]) Object.assign(state[g], inp.faults[g]); if (inp.faults.bent !== undefined) state.bent = +inp.faults.bent; syncControls(); } if (inp.blind) startBlind(); },
  meta: { id: 'vibration-fault-simulator', title: 'Rotating Machinery Fault Simulator', version: '1.0' },
};
function syncControls() { const m = { ubMass: state.ub.mass, ubAng: state.ub.ang, trMass: state.ub.tmass, trAng: state.ub.tang, maAng: state.ma.ang, maOff: state.ma.off, looseBolt: state.loose.bolt, looseFit: state.loose.fit, bent: state.bent, brgSev: state.brg.sev, brgLube: state.brg.lube, gearSev: state.gear.sev, beltSev: state.belt.sev, fn: state.res.fn, zeta: state.res.zeta }; for (const k in m) { const e = $('#' + k); if (e) { e.value = m[k]; e.dispatchEvent(new Event('input')); } } $('#cplType').value = state.ma.cpl; $('#brgPos').value = state.brg.pos; $('#brgType').value = state.brg.type; $('#gearType').value = state.gear.type; $('#beltType').value = state.belt.type; }
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.VibrationFaultSim.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'vibration-fault-simulator', state: window.VibrationFaultSim.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('vf_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog();
  synth(model(state.point, state.dir, eff())); analyse(); drawSpec(); drawWave(); drawEnv(); updateReadouts(); updatePhaseTable();
  requestAnimationFrame(tick);
}
build();
