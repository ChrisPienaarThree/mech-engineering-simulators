/* Gearbox Tooth Loads and Efficiency Bench — a spur or helical gear pair: geometry from module, teeth, face width and
   helix; tangential, radial and axial tooth forces and bearing loads; Lewis bending and Hertz contact stresses against
   the material allowables with a dynamic factor; mesh friction, churning, bearing and seal losses giving efficiency
   against load and speed; oil temperature on a clock with viscosity feedback, scuffing and pitting. Standalone, offline. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—', DEG = Math.PI / 180;
const MAT = { th: { name: 'Steel through-hardened 300 HB', sb: 300, sh: 950 }, ch: { name: 'Steel case-hardened 60 HRC', sb: 450, sh: 1500 }, ci: { name: 'Grey cast iron', sb: 110, sh: 520 }, br: { name: 'Bronze (worm-type)', sb: 90, sh: 360 } };
const LEWIS = [[12, 0.245], [14, 0.277], [16, 0.296], [18, 0.309], [20, 0.322], [24, 0.337], [28, 0.353], [32, 0.365], [40, 0.389], [50, 0.408], [60, 0.421], [75, 0.435], [100, 0.447], [150, 0.460], [300, 0.472]];
const VG = { 68: 68, 150: 150, 220: 220, 320: 320, 460: 460 }, CP = 191, ALPHA = 20 * DEG;
const lewis = z => { if (z <= 12) return 0.245; for (let i = 0; i < LEWIS.length - 1; i++) if (z <= LEWIS[i + 1][0]) { const t = (z - LEWIS[i][0]) / (LEWIS[i + 1][0] - LEWIS[i][0]); return LEWIS[i][1] + t * (LEWIS[i + 1][1] - LEWIS[i][1]); } return 0.472; };
const visc = (vg, T) => vg * Math.exp(-0.032 * (T - 40)) ; // cSt, rough Walther-like fall with temperature

/* ------------------------------------------------------------------ state */
const state = { m: 4, z1: 20, z2: 60, b: 50, beta: 0, Qv: 8, mat: 'th', n1: 1450, P: 30, Ko: 1.25, vg: 220, imm: 1, Tamb: 25, tscale: 20, run: true, T: 40, t: 0, ang: 0, pit: 0, log: [], flags: {}, trace: [], traceAcc: 0, scuffed: false, pitted: false };
const R = {};

/* ------------------------------------------------------------------ model */
function solve(o = {}) {
  const s = Object.assign({}, state, o), be = s.beta * DEG, cb = Math.cos(be), mt = s.m / cb, d1 = mt * s.z1, d2 = mt * s.z2, u = s.z2 / s.z1, a = (d1 + d2) / 2, at = Math.atan(Math.tan(ALPHA) / cb), n2 = s.n1 / u;
  const w1 = s.n1 * 2 * Math.PI / 60, T1 = s.P * 1000 / w1, v = w1 * d1 / 2000, Wt = 2 * T1 / (d1 / 1000), Wr = Wt * Math.tan(at), Wa = Wt * Math.tan(be), Wn = Wt / (Math.cos(ALPHA) * cb);
  const ra1 = d1 / 2 + s.m, ra2 = d2 / 2 + s.m, rb1 = d1 / 2 * Math.cos(at), rb2 = d2 / 2 * Math.cos(at), ea = (Math.sqrt(Math.max(0, ra1 * ra1 - rb1 * rb1)) + Math.sqrt(Math.max(0, ra2 * ra2 - rb2 * rb2)) - a * Math.sin(at)) / (Math.PI * mt * Math.cos(at)), eb = s.b * Math.sin(be) / (Math.PI * s.m);
  const B = 0.25 * Math.pow(12 - s.Qv, 2 / 3), A = 50 + 56 * (1 - B), Kv = Math.pow((A + Math.sqrt(200 * v)) / A, B), Km = 1.15 + 0.002 * s.b, zv1 = s.z1 / (cb * cb * cb), zv2 = s.z2 / (cb * cb * cb), Y1 = lewis(zv1), Y2 = lewis(zv2), Yh = be > 0 ? 1 / (1 + 0.25 * Math.min(1.5, eb)) : 1;
  const sb1 = Wt * s.Ko * Kv * Km / (s.b * s.m * Y1) * Yh, sb2 = Wt * s.Ko * Kv * Km / (s.b * s.m * Y2) * Yh, mN = be > 0 ? clamp(0.95 / (0.65 + 0.35 * Math.min(2, ea + eb)), 0.6, 1) : 1, I = Math.cos(at) * Math.sin(at) / (2 * mN) * u / (u + 1), sH = CP * Math.sqrt(Wt * s.Ko * Kv * Km / (s.b * d1 * I));
  const mat = MAT[s.mat], SFb = mat.sb / Math.max(sb1, sb2), SFH = mat.sh / sH;
  const nu = visc(s.vg, s.T), mu = clamp(0.045 * Math.pow(Wt / (s.b * Math.max(v, 0.2)), 0.2) * Math.pow(nu, -0.05) * 1.0, 0.02, 0.15), Hv = Math.PI * (u + 1) / (s.z1 * u * cb) * (1 - ea + ea * ea / 2), Pmesh = s.P * mu * Hv;
  const Pchurn = 0.35 * s.imm * Math.pow(nu / 100, 0.3) * Math.pow(v / 10, 1.6) * (s.b / 40) * (d1 / 80) * (1 + u / 3), Fb = Math.hypot(Wt, Wr), Pbrg = 0.0015 * s.P + 0.0006 * Fb * v / 1000 * 2, Pseal = 0.05 + 0.02 * s.n1 / 1000, Ploss = Pmesh + Pchurn + Pbrg + Pseal, eta = 1 - Ploss / s.P;
  const brg1 = Fb / 2, brg2 = Fb / 2, hmin = 0.6 * Math.pow(nu * v * 1e-3, 0.7) * Math.pow(Wt / s.b, -0.13) * 20, lam = hmin / 1.0, Tflash = s.T + 30 * mu * Math.pow(Wt / s.b, 0.75) * Math.sqrt(v) / 3, scuff = (Tflash > 180 && lam < 1.3) || lam < 0.8, Lh = 800 * Math.pow(Math.max(SFH, 0.2), 6.6);
  return { mt, d1, d2, u, a, at, n2, T1, T2: T1 * u, v, Wt, Wr, Wa, Wn, ea, eb, Kv, Km, Y1, Y2, sb1, sb2, sH, SFb, SFH, mat, nu, mu, Hv, Pmesh, Pchurn, Pbrg, Pseal, Ploss, eta, brg1, brg2, lam, Tflash, scuff, Lh, Fb, zv1, undercut: zv1 < 17, thin: nu < 20 };
}
function step(dt) {
  const s = state; Object.assign(R, solve());
  if (s.run) { const dts = dt * s.tscale; s.t += dts; s.ang += s.n1 / 60 * 360 * dt * 0.02; const hA = 0.012 + 0.0008 * R.v; s.T += (R.Ploss - hA * (s.T - s.Tamb)) / 20 * dts; if (R.SFH < 1) s.pit += dts / 3600 / Math.max(1, R.Lh) * 100; if (s.pit >= 100 && !s.pitted) { s.pitted = true; s.flags.pitting = true; record('pitting'); } if (R.scuff && !s.scuffed && s.t > 5) { s.scuffed = true; s.flags.scuff = true; record('scuffing'); } }
  else s.T += (s.Tamb - s.T) * Math.min(1, dt * s.tscale / 600);
  if (R.Wa > 0 && s.beta >= 15) s.flags.axial = true; if (s.T > 95) s.flags.hot = true;
  s.traceAcc += dt; if (s.traceAcc >= 0.2) { s.traceAcc = 0; s.trace.push({ t: s.t, T: s.T, eta: R.eta * 100, loss: R.Ploss }); if (s.trace.length > 900) s.trace.shift(); }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Gear pair' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
const arrow = (x1, y1, x2, y2, cls, p) => { el('line', { x1, y1, x2, y2, class: cls }, p); const a = Math.atan2(y2 - y1, x2 - x1); el('polygon', { points: `${x2},${y2} ${x2 - 10 * Math.cos(a - 0.4)},${y2 - 10 * Math.sin(a - 0.4)} ${x2 - 10 * Math.cos(a + 0.4)},${y2 - 10 * Math.sin(a + 0.4)}`, class: cls.replace('arrow', 'head') }, p); };
function gearPath(cx, cy, rp, z, m, phase) { const ra = rp + m, rf = rp - 1.25 * m, pts = []; for (let i = 0; i < z; i++) { const a0 = phase + i * 2 * Math.PI / z, w = Math.PI / z; const seq = [[rf, -w / 2], [rf, -w * 0.3], [ra, -w * 0.18], [ra, w * 0.18], [rf, w * 0.3], [rf, w / 2]]; for (const [r, da] of seq) pts.push(`${cx + r * Math.cos(a0 + da)},${cy + r * Math.sin(a0 + da)}`); } return 'M' + pts.join(' L') + ' Z'; }
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, k = Math.min(1.1, 260 / (r.d1 + r.d2)), rp1 = r.d1 / 2 * k, rp2 = r.d2 / 2 * k, cx1 = 250, cy = 190, cx2 = cx1 + (rp1 + rp2), mk = Math.max(2.5, s.m * k);
  const a1 = s.ang * DEG, a2 = -a1 / r.u + Math.PI / s.z2 * (s.z1 % 2 === 0 ? 1 : 0);
  el('path', { d: gearPath(cx1, cy, rp1, s.z1, mk, a1), class: 'gear' + (s.pitted ? ' pitted' : '') }, svg); el('path', { d: gearPath(cx2, cy, rp2, s.z2, mk, a2 + Math.PI / s.z2), class: 'gear driven' + (s.pitted ? ' pitted' : '') }, svg);
  el('circle', { cx: cx1, cy, r: rp1, class: 'pitch' }, svg); el('circle', { cx: cx2, cy, r: rp2, class: 'pitch' }, svg); el('circle', { cx: cx1, cy, r: 10, class: 'hub' }, svg); el('circle', { cx: cx2, cy, r: 12, class: 'hub' }, svg);
  const px = cx1 + rp1, fs = 90 / Math.max(r.Wt, 1); arrow(px, cy, px, cy - Math.max(24, r.Wt * fs), 'farrow', svg); txt(px + 8, cy - Math.max(24, r.Wt * fs) - 4, `Wt ${fmt(r.Wt / 1000, 2)} kN`, 'ftxt', svg); arrow(px, cy, px + Math.max(16, r.Wr * fs), cy, 'tarrow', svg); txt(px + Math.max(16, r.Wr * fs) + 6, cy + 4, `Wr ${fmt(r.Wr / 1000, 2)} kN`, 'ftxt', svg);
  if (r.Wa > 0) { txt(px + 8, cy + 40, `Wa ${fmt(r.Wa / 1000, 2)} kN axial (into the page)`, 'ftxt', svg); }
  txt(cx1, cy + rp1 + 22, `pinion z ${s.z1} · d ${fmt(r.d1, 1)} mm · ${s.n1} rpm · ${fmt(r.T1, 0)} N·m`, 'lbl', svg, 'middle'); txt(cx2, cy + rp2 + 22, `gear z ${s.z2} · d ${fmt(r.d2, 1)} mm · ${fmt(r.n2, 0)} rpm · ${fmt(r.T2, 0)} N·m`, 'lbl', svg, 'middle');
  txt(cx1, cy - rp1 - 14, `bearing load ${fmt(r.brg1 / 1000, 2)} kN each`, 'lbl', svg, 'middle'); txt(cx2, cy - rp2 - 14, `bearing load ${fmt(r.brg2 / 1000, 2)} kN each`, 'lbl', svg, 'middle');
  const bx = 700; const bar = (y, name, v, vmax, unit, bad, d = 0) => { txt(bx, y - 6, name, 'lbl', svg); el('rect', { x: bx, y, width: 200, height: 14, class: 'tbar' }, svg); el('rect', { x: bx, y, width: clamp(v / vmax, 0, 1) * 200, height: 14, class: 'tfill' + (bad ? ' bad' : '') }, svg); txt(bx + 206, y + 11, `${fmt(v, d)} ${unit}`, 'val', svg); };
  bar(40, `bending stress · allowable ${r.mat.sb} MPa`, Math.max(r.sb1, r.sb2), r.mat.sb * 1.5, 'MPa', r.SFb < 1, 0); el('line', { x1: bx + 200 / 1.5, y1: 34, x2: bx + 200 / 1.5, y2: 60, class: 'plim' }, svg);
  bar(90, `contact stress · allowable ${r.mat.sh} MPa`, r.sH, r.mat.sh * 1.5, 'MPa', r.SFH < 1, 0); el('line', { x1: bx + 200 / 1.5, y1: 84, x2: bx + 200 / 1.5, y2: 110, class: 'plim' }, svg);
  bar(140, 'oil temperature', s.T, 150, '°C', s.T > 95, 0); bar(190, 'efficiency', r.eta * 100, 100, '%', r.eta < 0.95, 1); bar(240, 'pitting damage', s.pit, 100, '%', s.pit > 50, 0);
  txt(bx, 290, `losses ${fmt(r.Ploss, 2)} kW: mesh ${fmt(r.Pmesh, 2)} · churn ${fmt(r.Pchurn, 2)} · brg ${fmt(r.Pbrg, 2)} · seals ${fmt(r.Pseal, 2)}`, 'val', svg); txt(bx, 308, `oil VG ${s.vg} → ${fmt(r.nu, 0)} cSt at ${fmt(s.T, 0)} °C · μ ${fmt(r.mu, 3)} · λ ${fmt(r.lam, 2)}`, 'val', svg);
  const warn = s.scuffed ? `SCUFFED — flash temperature ${fmt(r.Tflash, 0)} °C / film ratio ${fmt(r.lam, 2)}: oil too thin or too hot for this load and speed` : s.pitted ? 'PITTED — contact stress above the allowable long enough to use up the surface life' : r.SFH < 1 ? `CONTACT STRESS ${fmt(r.sH, 0)} MPa ABOVE ALLOWABLE — pitting life ${fmt(r.Lh, 0)} h, damage ${fmt(s.pit, 0)} %` : r.SFb < 1 ? `BENDING STRESS ${fmt(Math.max(r.sb1, r.sb2), 0)} MPa ABOVE ALLOWABLE — tooth root fracture risk` : r.undercut ? `UNDERCUT — virtual pinion teeth ${fmt(r.zv1, 1)} below 17 at 20°: weak root, use more teeth or profile shift` : `safety factors · bending ${fmt(r.SFb, 2)} · contact ${fmt(r.SFH, 2)} · pitting life ${r.SFH >= 1 ? '> 800' : fmt(r.Lh, 0)} h`;
  txt(480, 372, warn, 'val' + (/SCUFF|PITTED|ABOVE|UNDERCUT/.test(warn) ? ' bad' : ''), svg, 'middle');
  txt(480, 392, `${s.beta > 0 ? 'helical β ' + s.beta + '°' : 'spur'} · module ${s.m} · ratio ${fmt(r.u, 2)} · pitch line ${fmt(r.v, 1)} m/s · contact ratio εα ${fmt(r.ea, 2)}${s.beta > 0 ? ' + εβ ' + fmt(r.eb, 2) : ''} · Kv ${fmt(r.Kv, 2)} (Qv ${s.Qv}) · η ${fmt(r.eta * 100, 1)} %`, 'val', svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawStress() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const Pmax = Math.max(s.P * 2, 20), X = P => L + P / Pmax * pw, smax = Math.max(r.mat.sh * 1.2, r.sH * 1.3), Y = v => T + (1 - clamp(v / smax, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); const ps = Pmax > 200 ? 50 : Pmax > 80 ? 20 : 10; for (let P = 0; P <= Pmax; P += ps) { ctx.beginPath(); ctx.moveTo(X(P), T); ctx.lineTo(X(P), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(P, X(P), H - 18); } for (let v = 0; v <= smax; v += 250) { ctx.beginPath(); ctx.moveTo(L, Y(v)); ctx.lineTo(L + pw, Y(v)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(v, L - 4, Y(v) + 3); }
  const line = (f, color, w, dash = []) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; ctx.setLineDash(dash); for (let i = 1; i <= 60; i++) { const P = Pmax * i / 60, q = solve({ P }), v = f(q); i === 1 ? ctx.moveTo(X(P), Y(v)) : ctx.lineTo(X(P), Y(v)); } ctx.stroke(); ctx.setLineDash([]); };
  line(q => Math.max(q.sb1, q.sb2), col('--water'), 2.4); line(q => q.sH, col('--alarm'), 2.4);
  ctx.setLineDash([6, 3]); ctx.lineWidth = 1.4; ctx.strokeStyle = col('--water'); ctx.beginPath(); ctx.moveTo(L, Y(r.mat.sb)); ctx.lineTo(L + pw, Y(r.mat.sb)); ctx.stroke(); ctx.strokeStyle = col('--alarm'); ctx.beginPath(); ctx.moveTo(L, Y(r.mat.sh)); ctx.lineTo(L + pw, Y(r.mat.sh)); ctx.stroke(); ctx.setLineDash([]);
  ctx.font = '600 10px IBM Plex Mono, monospace'; ctx.textAlign = 'right'; ctx.fillStyle = col('--water'); ctx.fillText(`bending allowable ${r.mat.sb} MPa`, L + pw - 6, Y(r.mat.sb) - 4); ctx.fillStyle = col('--alarm'); ctx.fillText(`contact allowable ${r.mat.sh} MPa`, L + pw - 6, Y(r.mat.sh) - 4);
  for (const [v, c] of [[Math.max(r.sb1, r.sb2), col('--water')], [r.sH, col('--alarm')]]) { ctx.fillStyle = c; ctx.beginPath(); ctx.arc(X(s.P), Y(v), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); }
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`bending ∝ load · contact ∝ √load · at ${s.P} kW: σb ${fmt(Math.max(r.sb1, r.sb2), 0)} (SF ${fmt(r.SFb, 2)}) · σH ${fmt(r.sH, 0)} (SF ${fmt(r.SFH, 2)})`, L + 6, T + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('transmitted power kW at this speed · tooth stresses MPa · blue bending (Lewis), red contact (Hertz)', L + pw / 2, H - 4);
}
function drawEta() {
  const cv = $('#chart2'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 60, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const X = f => L + f / 1.2 * pw, e0 = Math.min(90, r.eta * 100 - 2, 80), Y = e => T + (1 - clamp((e - e0) / (100 - e0), 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); for (let f = 0; f <= 1.2; f += 0.2) { ctx.beginPath(); ctx.moveTo(X(f), T); ctx.lineTo(X(f), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(fmt(f * 100, 0) + ' %', X(f), H - 18); } for (let e = Math.ceil(e0); e <= 100; e += 2) { ctx.beginPath(); ctx.moveTo(L, Y(e)); ctx.lineTo(L + pw, Y(e)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(e, L - 4, Y(e) + 3); }
  for (const [n1, color, w] of [[s.n1, col('--water'), 2.4], [s.n1 * 2, col('--brass'), 1.4]]) { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; for (let i = 2; i <= 60; i++) { const f = 1.2 * i / 60, q = solve({ P: s.P * f, n1 }); i === 2 ? ctx.moveTo(X(f), Y(q.eta * 100)) : ctx.lineTo(X(f), Y(q.eta * 100)); } ctx.stroke(); }
  ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(1), Y(r.eta * 100), 6, 0, 7); ctx.fill();
  const bx = L + pw - 190, parts = [['mesh', r.Pmesh, col('--alarm')], ['churn', r.Pchurn, col('--water')], ['bearings', r.Pbrg, col('--steel')], ['seals', r.Pseal, col('--brass')]], tot = Math.max(r.Ploss, 1e-6); let y = T + 30; ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; for (const [n, v, c] of parts) { ctx.fillStyle = c; ctx.fillRect(bx, y, 120 * v / tot, 10); ctx.fillStyle = col('--ink'); ctx.fillText(`${n} ${fmt(v, 2)} kW`, bx + 126, y + 9); y += 16; }
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`efficiency at ${s.n1} rpm (blue) and ${s.n1 * 2} rpm (brass) · losses ${fmt(r.Ploss, 2)} kW of ${s.P} kW`, L + 6, T + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('load as a share of the set power · efficiency % · fixed losses (churning, seals) dominate at part load', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, s = state, span = 120, tr = s.trace.filter(p => p.t > s.t - span * s.tscale);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); if (tr.length < 2) return; const t0 = s.t - span * s.tscale, X = t => L + (t - t0) / (span * s.tscale) * pw, YT = v => T + (1 - clamp(v / 150, 0, 1)) * ph, YE = v => T + (1 - clamp((v - 80) / 20, 0, 1)) * ph;
  ctx.strokeStyle = col('--alarm'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L, YT(95)); ctx.lineTo(L + pw, YT(95)); ctx.stroke(); ctx.setLineDash([]);
  const ser = (k, color, Yf, w) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Yf(p[k])) : ctx.lineTo(X(p.t), Yf(p[k])); f = false; } ctx.stroke(); }; ser('T', col('--brass'), YT, 2.2); ser('eta', col('--water'), YE, 1.6);
  ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.fillText('150 °C', L - 4, T + 10); ctx.fillText('0', L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); ctx.fillText('oil °C', L + pw + 4, T + 10); ctx.fillStyle = col('--water'); ctx.fillText('η (80–100)', L + pw + 4, T + 24); ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span * s.tscale} s of gearbox time · oil temperature (alarm 95 °C) and efficiency`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, set = (id, v) => { $('#' + id).textContent = v; };
  set('roU', fmt(r.u, 2)); set('roN2', fmt(r.n2, 0)); set('roT2', fmt(r.T2, 0)); set('roV', fmt(r.v, 1)); set('roD1', fmt(r.d1, 1)); set('roD2', fmt(r.d2, 1)); set('roWt', fmt(r.Wt / 1000, 2)); set('roWr', fmt(r.Wr / 1000, 2)); set('roWa', fmt(r.Wa / 1000, 2)); set('roSb', fmt(Math.max(r.sb1, r.sb2), 0)); set('roSFb', fmt(r.SFb, 2)); set('roY', fmt(r.Y1, 3)); set('roSH', fmt(r.sH, 0)); set('roSFH', fmt(r.SFH, 2)); set('roLh', r.SFH >= 1 ? '> 800' : fmt(r.Lh, 0)); set('roEa', fmt(r.ea, 2)); set('roEb', fmt(r.eb, 2)); set('roKv', fmt(r.Kv, 2)); set('roEta', fmt(r.eta * 100, 2)); set('roLoss', fmt(r.Ploss, 2)); set('roMesh', fmt(r.Pmesh, 2)); set('roChurn', fmt(r.Pchurn, 2)); set('roToil', fmt(s.T, 0)); set('roNu', fmt(r.nu, 0)); set('roLam', fmt(r.lam, 2)); set('roBrg', fmt(r.brg1 / 1000, 2)); set('roBrg2', fmt(r.brg2 / 1000, 2)); set('roPit', fmt(s.pit, 0));
  $('#roSbBox').classList.toggle('alarm', r.SFb < 1); $('#roSbBox').classList.toggle('warnv', r.SFb >= 1 && r.SFb < 1.3); $('#roSHBox').classList.toggle('alarm', r.SFH < 1); $('#roSHBox').classList.toggle('warnv', r.SFH >= 1 && r.SFH < 1.2); $('#roToilBox').classList.toggle('alarm', s.T > 95 || r.scuff); $('#roEtaBox').classList.toggle('warnv', r.eta < 0.95);
  const ps = $('#pillState'); ps.textContent = s.scuffed ? 'Scuffed' : s.pitted ? 'Pitted' : r.SFH < 1 || r.SFb < 1 ? 'Overstressed' : s.T > 95 ? 'Oil hot' : `η ${fmt(r.eta * 100, 1)} %`; ps.className = 'pill ' + (s.scuffed || s.pitted || r.SFH < 1 || r.SFb < 1 ? 'warn' : s.T > 95 ? 'amber' : 'ok');
  $('#pillMode').textContent = `${s.beta > 0 ? 'helical ' + s.beta + '°' : 'spur'} m${s.m} ${s.z1}:${s.z2} · ${s.P} kW · ${s.n1} rpm`; $('#pillScuff').classList.toggle('hidden', !s.scuffed); $('#pillPit').classList.toggle('hidden', !(r.SFH < 1 || s.pitted)); $('#pillUnder').classList.toggle('hidden', !r.undercut);
  $('#pillRun').textContent = s.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start';
  $('#rigTitle').textContent = s.beta > 0 ? `Helical gear pair · β ${s.beta}° · tooth forces at the pitch point` : 'Spur gear pair · tooth forces at the pitch point';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, m: s.m, z1: s.z1, z2: s.z2, b: s.b, beta: s.beta, mat: s.mat, Qv: s.Qv, n1: s.n1, P: s.P, Ko: s.Ko, vg: s.vg, u: +r.u.toFixed(2), v: +r.v.toFixed(2), Wt: +r.Wt.toFixed(0), Wr: +r.Wr.toFixed(0), Wa: +r.Wa.toFixed(0), sb: +Math.max(r.sb1, r.sb2).toFixed(0), SFb: +r.SFb.toFixed(2), sH: +r.sH.toFixed(0), SFH: +r.SFH.toFixed(2), ea: +r.ea.toFixed(2), eta: +(r.eta * 100).toFixed(2), Ploss: +r.Ploss.toFixed(2), churn: +r.Pchurn.toFixed(2), mesh: +r.Pmesh.toFixed(2), T: +s.T.toFixed(0), lam: +r.lam.toFixed(2), brg: +r.brg1.toFixed(0) }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.m, r.z1, r.z2, r.b, r.beta, r.mat, r.n1, r.P, r.u, r.v, r.Wt, r.Wr, r.Wa, r.sb, r.SFb, r.sH, r.SFH, r.ea, r.eta, r.Ploss, r.T].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,module,z1,z2,b_mm,helix_deg,material,Qv,n1_rpm,P_kW,Ko,VG,ratio,v_ms,Wt_N,Wr_N,Wa_N,sigma_b_MPa,SF_b,sigma_H_MPa,SF_H,eps_alpha,eta_pct,loss_kW,churn_kW,mesh_kW,T_oil_C,lambda,bearing_N\n' + state.log.map(r => [r.i, r.tag, r.m, r.z1, r.z2, r.b, r.beta, r.mat, r.Qv, r.n1, r.P, r.Ko, r.vg, r.u, r.v, r.Wt, r.Wr, r.Wa, r.sb, r.SFb, r.sH, r.SFH, r.ea, r.eta, r.Ploss, r.churn, r.mesh, r.T, r.lam, r.brg].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, k) => k.every(x => a[x] === b[x]);
const TASKS = [
  { id: 'forces', t: 'Tooth forces', d: 'Record the baseline pair (m 4, 20:60, 50 mm face, 30 kW at 1 450 rpm): tangential, radial and axial force and the bearing loads.', ok: () => state.log.length > 0 },
  { id: 'module', t: 'Module against bending', d: 'Record the same power and teeth with a module 2 mm larger: the tangential force falls with the larger pitch diameter and the tooth is thicker — bending stress drops sharply.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['z1', 'z2', 'P', 'n1', 'b', 'beta']) && b.m - a.m >= 2 && b.sb < 0.6 * a.sb)); } },
  { id: 'face', t: 'Face width', d: 'Record face widths 20 mm apart at the same load: both stresses fall, contact stress with the square root.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['z1', 'z2', 'P', 'n1', 'm', 'beta']) && b.b - a.b >= 20 && b.sH < a.sH)); } },
  { id: 'helix', t: 'Helix angle and thrust', d: 'Set a helix angle of 15° or more: the mesh is smoother (overlap ratio) and the stresses fall, but an axial thrust appears that the bearings must carry.', ok: () => state.flags.axial && state.log.some(x => x.beta >= 15) },
  { id: 'governs', t: 'Which stress governs?', d: 'Find a load where the contact safety factor is below 1.2 while bending is still above 1.5 — pitting governs most steel gears. Record it.', ok: () => state.log.some(x => x.SFH < 1.2 && x.SFb > 1.5) },
  { id: 'partload', t: 'Efficiency at part load', d: 'Record the pair at full power and at 20 % power: the fixed churning and seal losses make part-load efficiency worse.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['z1', 'z2', 'n1', 'm', 'b', 'beta', 'vg']) && b.P <= 0.25 * a.P && b.eta < a.eta)); } },
  { id: 'churn', t: 'Churning at speed', d: 'Run at 2 500 rpm or more and record: churning grows faster than the transmitted power. Compare a thinner oil.', ok: () => state.log.some(x => x.n1 >= 2500 && x.churn >= 0.3 * x.Ploss) },
  { id: 'pitting', t: 'Overload to pitting', d: 'Overload the pair until the contact stress passes the allowable, run the clock and watch pitting damage accumulate to failure.', ok: () => state.flags.pitting },
  { id: 'scuff', t: 'Scuffing', d: 'Combine high load, high speed and thin hot oil (VG 68, ambient 45 °C) until the film ratio collapses and the teeth scuff.', ok: () => state.flags.scuff },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawStress(); drawEta(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSel(id, v) { $('#' + id).value = v; state[id] = typeof state[id] === 'number' ? +v : v; }
function newGears() { state.pit = 0; state.pitted = false; state.scuffed = false; state.T = state.Tamb; }
const PRESETS = {
  base: () => { for (const [k, v] of Object.entries({ m: 4, z1: 20, z2: 60, b: 50, beta: 0, Qv: 8, n1: 1450, P: 30, Ko: 1.25, Tamb: 25, tscale: 20 })) setCtl(k, v); setSel('mat', 'th'); setSel('vg', 220); setSel('imm', 1); newGears(); state.run = true; },
  helical: () => { PRESETS.base(); setCtl('beta', 20); },
  coarse: () => { PRESETS.base(); setCtl('m', 6); },
  overload: () => { PRESETS.base(); setCtl('P', 75); setCtl('tscale', 1000); },
  fast: () => { PRESETS.base(); setCtl('n1', 3000); },
  thin: () => { PRESETS.base(); setCtl('P', 60); setCtl('n1', 2500); setSel('vg', 68); setCtl('Tamb', 45); setCtl('tscale', 100); },
  iron: () => { PRESETS.base(); setSel('mat', 'ci'); setCtl('tscale', 1000); },
  part: () => { PRESETS.base(); setCtl('P', 6); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('m', v => v + ' mm'); bindR('z1', v => String(v)); bindR('z2', v => String(v)); bindR('b', v => v + ' mm'); bindR('beta', v => v + '°'); bindR('Qv', v => 'Qv ' + v); bindR('n1', v => v + ' rpm'); bindR('P', v => v + ' kW'); bindR('Ko', v => v.toFixed(2)); bindR('Tamb', v => v + ' °C'); bindR('tscale', v => '×' + v);
  $('#mat').addEventListener('change', e => { state.mat = e.target.value; newGears(); }); $('#vg').addEventListener('change', e => state.vg = +e.target.value); $('#imm').addEventListener('change', e => state.imm = +e.target.value);
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnNew').onclick = newGears; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('gb_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'R') record(''); if (k === 'N') newGears(); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.GearboxBench = {
  getState: () => ({ inputs: { m: state.m, z1: state.z1, z2: state.z2, b: state.b, beta: state.beta, Qv: state.Qv, mat: state.mat, n1: state.n1, P: state.P, Ko: state.Ko, vg: state.vg, imm: state.imm, Tamb: state.Tamb, tscale: state.tscale, run: state.run }, results: { u: R.u, v: R.v, Wt: R.Wt, Wr: R.Wr, Wa: R.Wa, sb: Math.max(R.sb1, R.sb2), SFb: R.SFb, sH: R.sH, SFH: R.SFH, ea: R.ea, eb: R.eb, Kv: R.Kv, eta: R.eta, Ploss: R.Ploss, Pmesh: R.Pmesh, Pchurn: R.Pchurn, T: state.T, nu: R.nu, lam: R.lam, pit: state.pit, scuffed: state.scuffed, pitted: state.pitted }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['m', 'z1', 'z2', 'b', 'beta', 'Qv', 'n1', 'P', 'Ko', 'Tamb', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.mat) setSel('mat', inp.mat); if (inp.vg) setSel('vg', inp.vg); if (inp.imm !== undefined) setSel('imm', inp.imm); if (inp.newGears) newGears(); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'gearbox-teeth', title: 'Gearbox Tooth Loads and Efficiency Bench', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.GearboxBench.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'gearbox-teeth', state: window.GearboxBench.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('gb_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); state.T = state.Tamb; step(0); drawRig(); updateReadouts(); drawStress(); drawEta(); drawTrace();
  requestAnimationFrame(tick);
}
build();
