/* Steam and Gas Turbine Cycle Board — Rankine cycle (boiler pressure, superheat, condenser pressure, reheat, turbine and
   pump efficiencies) on a saturation table with an effective superheat cp, Brayton cycle (pressure ratio, turbine inlet
   temperature, component efficiencies, regenerator) on the air standard, and a combined cycle joining them through a heat
   recovery steam generator. T-s diagram, parameter sweeps, flow board. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
/* saturation table: T °C, p kPa, hf, hg, sf, sg (kJ/kg, kJ/kg·K) — textbook values, interpolated */
const SAT = [[10, 1.228, 42.0, 2519.2, 0.1511, 8.8998], [20, 2.339, 83.9, 2537.4, 0.2965, 8.6661], [30, 4.247, 125.7, 2555.5, 0.4368, 8.4520], [40, 7.385, 167.5, 2573.5, 0.5724, 8.2555], [50, 12.35, 209.3, 2591.3, 0.7038, 8.0748], [60, 19.95, 251.2, 2608.8, 0.8313, 7.9081], [70, 31.20, 293.0, 2626.1, 0.9551, 7.7540], [80, 47.42, 335.0, 2643.0, 1.0756, 7.6111], [90, 70.18, 377.0, 2659.5, 1.1929, 7.4781], [100, 101.42, 419.2, 2675.6, 1.3072, 7.3541], [120, 198.7, 503.8, 2705.9, 1.5279, 7.1291], [140, 361.5, 589.2, 2733.4, 1.7392, 6.9294], [160, 618.2, 675.5, 2757.4, 1.9426, 6.7475], [180, 1002.8, 763.1, 2777.2, 2.1392, 6.5841], [200, 1554.9, 852.3, 2792.0, 2.3305, 6.4302], [220, 2319.6, 943.6, 2801.1, 2.5177, 6.2840], [240, 3346.9, 1037.6, 2803.0, 2.7020, 6.1425], [260, 4692.3, 1134.8, 2796.6, 2.8849, 6.0017], [280, 6416.6, 1236.1, 2779.9, 3.0685, 5.8579], [300, 8587.9, 1344.8, 2749.6, 3.2552, 5.7059], [320, 11284, 1462.2, 2700.6, 3.4494, 5.5372], [340, 14601, 1594.5, 2622.1, 3.6601, 5.3356], [360, 18666, 1761.5, 2481.0, 3.9165, 5.0536]];
const CPV = [[10, 2.0], [100, 2.03], [500, 2.1], [1000, 2.2], [2000, 2.35], [4000, 2.6], [6000, 2.9], [8000, 3.15], [10000, 3.45], [12000, 3.8], [14000, 4.2], [16000, 4.8], [18000, 5.6]];
const interp = (tab, x, ix, iy, logx = false) => { const f = logx ? Math.log : v => v, X = f(x); if (X <= f(tab[0][ix])) return tab[0][iy]; for (let i = 0; i < tab.length - 1; i++) { const a = f(tab[i][ix]), b = f(tab[i + 1][ix]); if (X <= b) { const t = (X - a) / (b - a); return tab[i][iy] + t * (tab[i + 1][iy] - tab[i][iy]); } } return tab[tab.length - 1][iy]; };
const satP = p => { const T = interp(SAT, p, 1, 0, true); const s = { T, p, hf: interp(SAT, p, 1, 2, true), hg: interp(SAT, p, 1, 3, true), sf: interp(SAT, p, 1, 4, true), sg: interp(SAT, p, 1, 5, true) }; s.hfg = s.hg - s.hf; s.sfg = s.sg - s.sf; return s; };
const cpv = p => interp(CPV, p, 0, 1, true);
function steamState(p, T) { const s = satP(p), cp = cpv(p); if (T <= s.T + 0.01) return { p, T: s.T, h: s.hg, s: s.sg, x: 1, sat: s }; return { p, T, h: s.hg + cp * (T - s.T), s: s.sg + cp * Math.log((T + 273.15) / (s.T + 273.15)), x: NaN, sat: s }; }
function expand(st, p2, eta) { const s2 = satP(p2), cp = cpv(p2); let h2s, T2s; if (st.s <= s2.sg) { const x = (st.s - s2.sf) / s2.sfg; h2s = s2.hf + x * s2.hfg; T2s = s2.T; } else { T2s = (s2.T + 273.15) * Math.exp((st.s - s2.sg) / cp) - 273.15; h2s = s2.hg + cp * (T2s - s2.T); } const h2 = st.h - eta * (st.h - h2s); let out; if (h2 <= s2.hg) { const x = (h2 - s2.hf) / s2.hfg; out = { p: p2, T: s2.T, h: h2, s: s2.sf + x * s2.sfg, x, sat: s2 }; } else { const T2 = s2.T + (h2 - s2.hg) / cp; out = { p: p2, T: T2, h: h2, s: s2.sg + cp * Math.log((T2 + 273.15) / (s2.T + 273.15)), x: NaN, sat: s2 }; } out.h2s = h2s; return out; }
const K = 1.4, CPA = 1.005, RA = 0.287;

/* ------------------------------------------------------------------ state */
const state = { mode: 'rankine', p1: 8, T1: 500, p2: 10, etat: 0.88, etap: 0.8, reheat: 'off', prh: 0.25, rp: 10, Tamb: 15, T3: 1200, etac: 0.85, etagt: 0.88, eps: 0, Pmw: 100, run: true, t: 0, dash: 0, log: [], flags: {} };
const R = {};

/* ------------------------------------------------------------------ model */
function rankine(o = {}) {
  const s = Object.assign({}, state, o), p1 = Math.max(1, s.p1) * 1000, p2 = Math.max(1, s.p2), sat1 = satP(p1), T1 = Math.max(s.T1, sat1.T), st1 = steamState(p1, T1), stC = satP(p2);
  const wp = 0.001 * (p1 - p2) / s.etap, hfw = stC.hf + wp, Tfw = stC.T + wp / 4.18;
  let st2 = expand(st1, p2, s.etat), qin = st1.h - hfw, wt = st1.h - st2.h, st3 = null, st4 = null, prh = 0;
  if (s.reheat === 'on') { prh = s.prh * p1; const st2a = expand(st1, prh, s.etat); st3 = steamState(prh, T1); st4 = expand(st3, p2, s.etat); wt = (st1.h - st2a.h) + (st3.h - st4.h); qin = (st1.h - hfw) + (st3.h - st2a.h); st2 = st2a; }
  const exit = st4 || st2, wnet = wt - wp, eta = wnet / qin, qout = exit.h - stC.hf, carnot = 1 - (stC.T + 273.15) / (T1 + 273.15), m = s.Pmw * 1000 / wnet, hr = 3600 / eta;
  return { st1, st2, st3, st4, exit, stC, sat1, T1, wp, wt, wnet, qin, qout, eta, carnot, m, hr, bwr: wp / wt, x: exit.x, hfw, Tfw, prh, wet: Number.isFinite(exit.x) && exit.x < 0.88 };
}
function brayton(o = {}) {
  const s = Object.assign({}, state, o), T1 = s.Tamb + 273.15, T3 = s.T3 + 273.15, e = (K - 1) / K, T2s = T1 * Math.pow(s.rp, e), T2 = T1 + (T2s - T1) / s.etac, T4s = T3 / Math.pow(s.rp, e), T4 = T3 - s.etagt * (T3 - T4s);
  const T5 = s.eps > 0 && T4 > T2 ? T2 + s.eps * (T4 - T2) : T2, T6 = T4 - (T5 - T2), wc = CPA * (T2 - T1), wt = CPA * (T3 - T4), wnet = wt - wc, qin = CPA * (T3 - T5), eta = wnet / qin, rpopt = Math.pow(T3 / T1 * s.etac * s.etagt, 1 / (2 * e));
  const sOf = (T, p) => CPA * Math.log(T / T1) - RA * Math.log(p); const pts = { s1: 0, s2: sOf(T2, s.rp), s3: sOf(T3, s.rp), s4: sOf(T4, 1), s5: sOf(T5, s.rp), s6: sOf(T6, 1) };
  return { T1, T2, T2s, T3, T4, T4s, T5, T6, wc, wt, wnet, qin, eta, bwr: wc / wt, rpopt, m: s.Pmw * 1000 / wnet, hr: 3600 / eta, carnot: 1 - T1 / T3, regenOK: T4 > T2, pts, hot: s.T3 > 1350 };
}
function combined(o = {}) {
  const g = brayton(Object.assign({ eps: 0 }, o)), Tstack = 110 + 273.15, T1cap = Math.min(state.T1, g.T4 - 273.15 - 25), r = rankine(Object.assign({ T1: Math.max(T1cap, satP(state.p1 * 1000).T + 1) }, o)), qexh = CPA * Math.max(0, g.T4 - Tstack) * 0.92, ms = qexh / r.qin, wst = ms * r.wnet, eta = (g.wnet + wst) / g.qin;
  return { g, r, qexh, ms, wst, eta, T1cap, capped: T1cap < state.T1, wnet: g.wnet + wst, m: state.Pmw * 1000 / (g.wnet + wst), hr: 3600 / eta, split: wst / (g.wnet + wst) };
}
function step(dt) { const s = state; if (s.run) { s.t += dt; s.dash += 40 * dt; } Object.assign(R, s.mode === 'rankine' ? rankine() : s.mode === 'brayton' ? brayton() : combined()); }

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Cycle flow board' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
const pipe = (d, cls) => { el('path', { d, class: 'pipe ' + cls }, svg); el('path', { d, class: 'flow', style: `stroke-dashoffset:${-state.dash}` }, svg); };
const box = (x, y, w, h, cls, name, sub) => { el('rect', { x, y, width: w, height: h, rx: 8, class: 'box ' + cls }, svg); txt(x + w / 2, y + h / 2 - 2, name, 'lbl', svg, 'middle'); if (sub) txt(x + w / 2, y + h / 2 + 14, sub, 'val', svg, 'middle'); };
const turb = (x, y, w, h, name) => { el('polygon', { points: `${x},${y + h * 0.2} ${x + w},${y} ${x + w},${y + h} ${x},${y + h * 0.8}`, class: 'box' }, svg); txt(x + w / 2, y + h / 2 + 4, name, 'lbl', svg, 'middle'); };
const comp = (x, y, w, h, name) => { el('polygon', { points: `${x},${y} ${x + w},${y + h * 0.2} ${x + w},${y + h * 0.8} ${x},${y + h}`, class: 'box' }, svg); txt(x + w / 2, y + h / 2 + 4, name, 'lbl', svg, 'middle'); };
const stateTag = (x, y, n, lines) => { el('circle', { cx: x, cy: y, r: 11, class: 'state' }, svg); txt(x, y + 4, n, 'statet', svg, 'middle'); lines.forEach((l, i) => txt(x + 16, y - 4 + i * 13, l, 'val', svg)); };
function drawRankine(r, y0 = 0, compact = false) {
  const s = state, sc = compact ? 0.62 : 1; const g = el('g', { transform: `translate(0,${y0}) scale(${sc})` }, svg); const old = svg; svg = g;
  box(90, 120, 150, 120, 'hot', s.mode === 'combined' ? 'HRSG' : 'BOILER', `${fmt(r.qin, 0)} kJ/kg in`); turb(360, 110, 170, 140, s.reheat === 'on' && !compact ? 'HP + LP TURBINE' : 'TURBINE'); box(640, 250, 170, 90, 'cold', 'CONDENSER', `${fmt(r.qout, 0)} kJ/kg out · ${fmt(r.stC.T, 0)} °C`); el('circle', { cx: 300, cy: 320, r: 30, class: 'box' }, svg); txt(300, 324, 'PUMP', 'lbl', svg, 'middle');
  pipe('M240,150 L360,150', 'hot'); pipe('M530,180 L700,180 L700,250', ''); pipe('M640,300 L330,300', 'cold'); pipe('M270,320 L165,320 L165,240', 'cold');
  if (s.reheat === 'on' && !compact) { pipe('M440,110 L440,60 L165,60 L165,120', 'hot'); txt(300, 52, `reheat at ${fmt(r.prh / 1000, 1)} MPa back to ${fmt(r.T1, 0)} °C`, 'lbl', svg, 'middle'); }
  stateTag(280, 130, '1', [`${fmt(r.st1.p / 1000, 1)} MPa · ${fmt(r.T1, 0)} °C`, `h ${fmt(r.st1.h, 0)} kJ/kg`]); stateTag(620, 200, '2', [`${fmt(r.exit.p, 0)} kPa · ${fmt(r.exit.T, 0)} °C`, Number.isFinite(r.x) ? `x ${fmt(r.x, 3)} · h ${fmt(r.exit.h, 0)}` : `superheated · h ${fmt(r.exit.h, 0)}`]); stateTag(480, 330, '3', [`saturated liquid ${fmt(r.stC.T, 0)} °C`, `h ${fmt(r.stC.hf, 0)} kJ/kg`]); stateTag(165, 280, '4', [`${fmt(r.st1.p / 1000, 1)} MPa · ${fmt(r.Tfw, 0)} °C`, `h ${fmt(r.hfw, 0)} · w pump ${fmt(r.wp, 1)}`]);
  txt(445, 90, `w turbine ${fmt(r.wt, 0)} kJ/kg`, 'ftxt', svg, 'middle'); el('line', { x1: 530, y1: 180, x2: 600, y2: 180 }, svg);
  const gen = el('circle', { cx: 580, cy: 60, r: 26, class: 'box' }, svg); txt(580, 64, 'G', 'statet', svg, 'middle'); el('line', { x1: 530, y1: 130, x2: 560, y2: 78, class: 'pipe' }, svg); txt(620, 64, `${fmt(s.Pmw, 0)} MW → ${fmt(r.m, 0)} kg/s steam`, 'val', svg);
  svg = old;
}
function drawBrayton(g, y0 = 0, compact = false) {
  const s = state, sc = compact ? 0.62 : 1; const gg = el('g', { transform: `translate(0,${y0}) scale(${sc})` }, svg); const old = svg; svg = gg;
  comp(120, 110, 160, 140, 'COMPRESSOR'); box(390, 60, 180, 80, 'hot', 'COMBUSTOR', `${fmt(g.qin, 0)} kJ/kg fuel heat`); turb(640, 110, 170, 140, 'TURBINE'); el('line', { x1: 280, y1: 180, x2: 640, y2: 180, class: 'pipe' }, svg); txt(460, 196, 'shaft', 'lbl', svg, 'middle');
  pipe('M40,180 L120,180', 'cold'); pipe('M280,140 L330,140 L330,100 L390,100', ''); pipe('M570,100 L640,100 L640,140', 'hot'); pipe('M810,180 L880,180 L880,320', 'hot');
  if (s.eps > 0 && g.regenOK) { box(300, 260, 200, 60, '', 'REGENERATOR', `ε ${fmt(s.eps, 2)} · air to ${fmt(g.T5 - 273.15, 0)} °C`); pipe('M880,290 L500,290', 'hot'); pipe('M300,290 L250,290 L250,60 L390,60', ''); txt(700, 340, `exhaust ${fmt(g.T6 - 273.15, 0)} °C`, 'val', svg); } else txt(880, 340, `exhaust ${fmt(g.T4 - 273.15, 0)} °C`, 'val', svg, 'middle');
  stateTag(60, 150, '1', [`${fmt(g.T1 - 273.15, 0)} °C · 1 bar`]); stateTag(330, 220, '2', [`${fmt(g.T2 - 273.15, 0)} °C · ${s.rp} bar`, `w comp ${fmt(g.wc, 0)} kJ/kg`]); stateTag(600, 160, '3', [`${fmt(g.T3 - 273.15, 0)} °C · ${s.rp} bar`]); stateTag(840, 240, '4', [`${fmt(g.T4 - 273.15, 0)} °C`, `w turb ${fmt(g.wt, 0)} kJ/kg`]);
  txt(725, 90, `net ${fmt(g.wnet, 0)} kJ/kg · bwr ${fmt(g.bwr, 2)}`, 'ftxt', svg, 'middle');
  svg = old;
}
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R;
  if (s.mode === 'rankine') drawRankine(r); else if (s.mode === 'brayton') drawBrayton(r); else { drawBrayton(r.g, -10, true); drawRankine(r.r, 150, true); txt(700, 190, `GT exhaust ${fmt(r.g.T4 - 273.15, 0)} °C → HRSG · ${fmt(r.ms, 3)} kg steam per kg air`, 'ftxt', svg, 'middle'); txt(700, 208, `steam side ${fmt(r.split * 100, 0)} % of the output`, 'val', svg, 'middle'); }
  const line1 = s.mode === 'rankine' ? `Rankine · η ${fmt(r.eta * 100, 1)} % (Carnot ${fmt(r.carnot * 100, 0)} %) · net ${fmt(r.wnet, 0)} kJ/kg · heat rate ${fmt(r.hr, 0)} kJ/kWh` : s.mode === 'brayton' ? `Brayton · η ${fmt(r.eta * 100, 1)} % (Carnot ${fmt(r.carnot * 100, 0)} %) · net ${fmt(r.wnet, 0)} kJ/kg air · optimum rp for work ${fmt(r.rpopt, 1)}` : `Combined · η ${fmt(r.eta * 100, 1)} % · GT ${fmt(r.g.eta * 100, 1)} % + steam ${fmt(r.r.eta * 100, 1)} % on the exhaust · net ${fmt(r.wnet, 0)} kJ/kg air`;
  txt(480, 372, line1, 'val', svg, 'middle');
  const warn = s.mode === 'rankine' ? (r.wet ? `TURBINE EXHAUST TOO WET — x ${fmt(r.x, 3)} below 0.88: blade erosion · raise superheat, add reheat or lower the pressure` : Number.isFinite(r.x) ? `exhaust quality ${fmt(r.x, 3)} · ok` : 'exhaust superheated — heat thrown away in the condenser') : s.mode === 'brayton' ? (r.hot ? `TURBINE INLET ${s.T3} °C ABOVE 1 350 °C — blade cooling and materials limit` : !r.regenOK && s.eps > 0 ? 'REGENERATOR USELESS — compressor delivery hotter than turbine exhaust at this pressure ratio' : `compressor delivery ${fmt(r.T2 - 273.15, 0)} °C · exhaust ${fmt(r.T4 - 273.15, 0)} °C`) : (r.capped ? `steam temperature capped at ${fmt(r.T1cap, 0)} °C by the exhaust (25 K approach)` : r.r.wet ? `steam exhaust wet: x ${fmt(r.r.x, 3)}` : 'combined cycle running');
  txt(480, 392, warn, 'val' + (/TOO WET|ABOVE|USELESS/.test(warn) ? ' bad' : ''), svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawTs() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  if (s.mode !== 'brayton') {
    const rr = s.mode === 'rankine' ? r : r.r, X = sv => L + sv / 9.5 * pw, Y = Tc => T + (1 - Tc / 650) * ph;
    ctx.strokeStyle = col('--grid'); for (let sv = 0; sv <= 9; sv++) { ctx.beginPath(); ctx.moveTo(X(sv), T); ctx.lineTo(X(sv), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(sv, X(sv), H - 18); } for (let Tc = 0; Tc <= 600; Tc += 100) { ctx.beginPath(); ctx.moveTo(L, Y(Tc)); ctx.lineTo(L + pw, Y(Tc)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(Tc, L - 4, Y(Tc) + 3); }
    ctx.beginPath(); ctx.strokeStyle = col('--steel'); ctx.lineWidth = 2; SAT.forEach((p, i) => i ? ctx.lineTo(X(p[4]), Y(p[0])) : ctx.moveTo(X(p[4]), Y(p[0]))); ctx.lineTo(X(4.41), Y(374)); for (let i = SAT.length - 1; i >= 0; i--) ctx.lineTo(X(SAT[i][5]), Y(SAT[i][0])); ctx.stroke();
    const p1 = rr.st1.p, sat1 = rr.sat1, cp = cpv(p1), path = [[rr.stC.sf, rr.stC.T], [rr.stC.sf + 0.002, rr.Tfw]]; for (let Tc = rr.Tfw; Tc < sat1.T; Tc += 10) path.push([interp(SAT, Tc, 0, 4), Tc]); path.push([sat1.sf, sat1.T], [sat1.sg, sat1.T]); for (let Tc = sat1.T; Tc <= rr.T1; Tc += 10) path.push([sat1.sg + cp * Math.log((Tc + 273.15) / (sat1.T + 273.15)), Tc]); path.push([rr.st1.s, rr.T1]);
    if (rr.st3) { path.push([rr.st2.s, rr.st2.T]); const s3 = rr.st3.sat, cp3 = cpv(rr.st3.p); for (let Tc = rr.st2.T; Tc <= rr.T1; Tc += 10) path.push([s3.sg + cp3 * Math.log((Tc + 273.15) / (s3.T + 273.15)), Tc]); path.push([rr.st3.s, rr.T1], [rr.st4.s, rr.st4.T]); } else path.push([rr.st2.s, rr.st2.T]); path.push([rr.stC.sf, rr.stC.T]);
    ctx.beginPath(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 2.4; path.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke();
    const dot = (sv, Tc, lbl) => { ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(X(sv), Y(Tc), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(lbl, X(sv) + 8, Y(Tc) - 4); };
    dot(rr.st1.s, rr.T1, '1'); dot(rr.exit.s, rr.exit.T, rr.st3 ? '4' : '2'); if (rr.st3) { dot(rr.st2.s, rr.st2.T, '2'); dot(rr.st3.s, rr.T1, '3'); } dot(rr.stC.sf, rr.stC.T, rr.st3 ? '5' : '3');
    ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('entropy s kJ/kg·K · temperature °C · steam dome from the saturation table · cycle in red', L + pw / 2, H - 4);
  } else {
    const g = r, smin = -0.3, smax = 1.6, X = sv => L + (sv - smin) / (smax - smin) * pw, Y = Tk => T + (1 - (Tk - 250) / 1700) * ph;
    ctx.strokeStyle = col('--grid'); for (let Tk = 300; Tk <= 1900; Tk += 200) { ctx.beginPath(); ctx.moveTo(L, Y(Tk)); ctx.lineTo(L + pw, Y(Tk)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(Tk - 273, L - 4, Y(Tk) + 3); } for (let sv = 0; sv <= 1.5; sv += 0.5) { ctx.beginPath(); ctx.moveTo(X(sv), T); ctx.lineTo(X(sv), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(sv.toFixed(1), X(sv), H - 18); }
    const isobar = (p, color) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.setLineDash([3, 3]); for (let Tk = 250; Tk <= 1950; Tk += 25) { const sv = CPA * Math.log(Tk / g.T1) - RA * Math.log(p); Tk === 250 ? ctx.moveTo(X(sv), Y(Tk)) : ctx.lineTo(X(sv), Y(Tk)); } ctx.stroke(); ctx.setLineDash([]); }; isobar(1, col('--steel')); isobar(s.rp, col('--steel'));
    const P = g.pts, path = [[P.s1, g.T1], [P.s2, g.T2]]; for (let Tk = g.T5; Tk <= g.T3; Tk += 20) path.push([CPA * Math.log(Tk / g.T1) - RA * Math.log(s.rp), Tk]); path.push([P.s3, g.T3], [P.s4, g.T4]); for (let Tk = g.T6; Tk >= g.T1; Tk -= 20) path.push([CPA * Math.log(Tk / g.T1), Tk]); path.push([P.s1, g.T1]);
    if (s.eps > 0 && g.regenOK) { ctx.beginPath(); ctx.strokeStyle = col('--good'); ctx.lineWidth = 3; for (let Tk = g.T2; Tk <= g.T5; Tk += 10) { const sv = CPA * Math.log(Tk / g.T1) - RA * Math.log(s.rp); Tk === g.T2 ? ctx.moveTo(X(sv), Y(Tk)) : ctx.lineTo(X(sv), Y(Tk)); } ctx.stroke(); }
    ctx.beginPath(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 2.4; path.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke();
    const dot = (sv, Tk, lbl) => { ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(X(sv), Y(Tk), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(lbl, X(sv) + 8, Y(Tk) - 4); }; dot(P.s1, g.T1, '1'); dot(P.s2, g.T2, '2'); dot(P.s3, g.T3, '3'); dot(P.s4, g.T4, '4'); if (s.eps > 0 && g.regenOK) dot(P.s5, g.T5, '5 (after regenerator)');
    ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`entropy relative to state 1, kJ/kg·K · temperature °C · isobars 1 bar and ${s.rp} bar · green = heat recovered by the regenerator`, L + pw / 2, H - 4);
  }
}
function drawSweep() {
  const cv = $('#chart2'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  let xs, f, xl, x0, x1, xlab, ylab2, y2max, cur; if (s.mode === 'rankine') { x0 = 1; x1 = 18; f = p => { const q = rankine({ p1: p }); return [q.eta * 100, Number.isFinite(q.x) ? q.x : 1]; }; xlab = 'boiler pressure MPa'; ylab2 = 'exhaust quality'; y2max = 1; cur = s.p1; } else if (s.mode === 'brayton') { x0 = 2; x1 = 40; f = rp => { const q = brayton({ rp }); return [q.eta * 100, q.wnet]; }; xlab = 'pressure ratio'; ylab2 = 'net work kJ/kg'; y2max = 800; cur = s.rp; } else { x0 = 4; x1 = 40; f = rp => { const q = combined({ rp }); return [q.eta * 100, q.g.eta * 100]; }; xlab = 'gas turbine pressure ratio'; ylab2 = 'GT alone %'; y2max = 70; cur = s.rp; }
  const X = x => L + (x - x0) / (x1 - x0) * pw, Y = e => T + (1 - clamp(e / 70, 0, 1)) * ph, Y2 = v => T + (1 - clamp(v / y2max, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); for (let e = 0; e <= 70; e += 10) { ctx.beginPath(); ctx.moveTo(L, Y(e)); ctx.lineTo(L + pw, Y(e)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(e + ' %', L - 4, Y(e) + 3); } const xstep = (x1 - x0) > 20 ? 5 : 2; for (let x = x0; x <= x1; x += xstep) { ctx.textAlign = 'center'; ctx.fillText(x, X(x), H - 18); }
  ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); for (let k = 0; k <= 1; k += 0.25) ctx.fillText(fmt(y2max * k, y2max <= 1 ? 2 : 0), L + pw + 4, Y2(y2max * k) + 3);
  const N = 60; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = 2.4; for (let i = 0; i <= N; i++) { const x = x0 + (x1 - x0) * i / N, v = f(x); i ? ctx.lineTo(X(x), Y(v[0])) : ctx.moveTo(X(x), Y(v[0])); } ctx.stroke();
  ctx.beginPath(); ctx.strokeStyle = col('--brass'); ctx.lineWidth = 1.8; ctx.setLineDash([5, 3]); for (let i = 0; i <= N; i++) { const x = x0 + (x1 - x0) * i / N, v = f(x); i ? ctx.lineTo(X(x), Y2(v[1])) : ctx.moveTo(X(x), Y2(v[1])); } ctx.stroke(); ctx.setLineDash([]);
  if (s.mode === 'rankine') { ctx.strokeStyle = col('--alarm'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(L, Y2(0.88)); ctx.lineTo(L + pw, Y2(0.88)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = col('--alarm'); ctx.textAlign = 'right'; ctx.fillText('x 0.88 erosion limit', L + pw - 4, Y2(0.88) - 4); }
  const v = f(cur); ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(cur), Y(v[0]), 6, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`${xlab} ${cur} → η ${fmt(v[0], 1)} % · ${ylab2} ${fmt(v[1], y2max <= 1 ? 3 : 0)}`, L + 6, T + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`${xlab} · blue efficiency (left) · dashed ${ylab2} (right) · other settings as set`, L + pw / 2, H - 4);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, m = s.mode, set = (id, v) => { $('#' + id).textContent = v; };
  if (m === 'rankine') { set('roEta', fmt(r.eta * 100, 1)); set('roCarnot', fmt(r.carnot * 100, 0)); set('roWnet', fmt(r.wnet, 0)); set('roQin', fmt(r.qin, 0)); set('roWt', fmt(r.wt, 0)); set('roWc', fmt(r.wp, 1)); set('roWcName', 'pump'); set('roBwr', fmt(r.bwr * 100, 1)); set('roX', Number.isFinite(r.x) ? fmt(r.x, 3) : 'superheated'); set('roXName', 'Turbine exhaust quality'); set('roXNote', `at ${fmt(r.stC.T, 1)} °C · limit 0.88`); set('roT2', fmt(r.sat1.T, 0)); set('roT2Name', 'Boiler saturation temperature'); set('roT2Note', `°C at ${s.p1} MPa · superheat ${fmt(r.T1 - r.sat1.T, 0)} K`); set('roM', fmt(r.m, 0)); set('roMNote', `kg/s steam for ${s.Pmw} MW`); set('roHr', fmt(r.hr, 0)); set('roQout', fmt(r.qout, 0)); $('#roXBox').classList.toggle('alarm', r.wet); }
  else if (m === 'brayton') { set('roEta', fmt(r.eta * 100, 1)); set('roCarnot', fmt(r.carnot * 100, 0)); set('roWnet', fmt(r.wnet, 0)); set('roQin', fmt(r.qin, 0)); set('roWt', fmt(r.wt, 0)); set('roWc', fmt(r.wc, 0)); set('roWcName', 'compressor'); set('roBwr', fmt(r.bwr * 100, 0)); set('roX', fmt(r.T4 - 273.15, 0)); set('roXName', 'Turbine exhaust temperature'); set('roXNote', `°C · after regenerator ${fmt(r.T6 - 273.15, 0)} °C`); set('roT2', fmt(r.T2 - 273.15, 0)); set('roT2Name', 'Compressor delivery'); set('roT2Note', `°C · isentropic ${fmt(r.T2s - 273.15, 0)} °C`); set('roM', fmt(r.m, 0)); set('roMNote', `kg/s air for ${s.Pmw} MW`); set('roHr', fmt(r.hr, 0)); set('roQout', fmt(r.qin - r.wnet, 0)); $('#roXBox').classList.toggle('alarm', r.hot); }
  else { set('roEta', fmt(r.eta * 100, 1)); set('roCarnot', fmt(r.g.carnot * 100, 0)); set('roWnet', fmt(r.wnet, 0)); set('roQin', fmt(r.g.qin, 0)); set('roWt', fmt(r.g.wt, 0)); set('roWc', fmt(r.g.wc, 0)); set('roWcName', 'compressor'); set('roBwr', fmt(r.g.bwr * 100, 0)); set('roX', fmt(r.split * 100, 0)); set('roXName', 'Steam share of output'); set('roXNote', `% · ${fmt(r.ms, 3)} kg steam per kg air · x ${Number.isFinite(r.r.x) ? fmt(r.r.x, 3) : '—'}`); set('roT2', fmt(r.g.T4 - 273.15, 0)); set('roT2Name', 'GT exhaust into the HRSG'); set('roT2Note', `°C · steam at ${fmt(r.r.T1, 0)} °C · stack 110 °C`); set('roM', fmt(r.m, 0)); set('roMNote', `kg/s air for ${s.Pmw} MW`); set('roHr', fmt(r.hr, 0)); set('roQout', fmt(r.g.qin - r.wnet, 0)); $('#roXBox').classList.toggle('alarm', r.r.wet); }
  $('#pillMode').textContent = { rankine: 'Rankine steam', brayton: 'Brayton gas turbine', combined: 'Combined cycle' }[m] + ` · η ${fmt(r.eta * 100, 1)} %`; const bad = m === 'rankine' ? r.wet : m === 'brayton' ? r.hot : r.r.wet; $('#pillWarn').classList.toggle('hidden', !bad); $('#pillWarn').textContent = m === 'rankine' ? 'Wet exhaust — blade erosion' : m === 'brayton' ? 'Turbine inlet above 1 350 °C' : 'Steam exhaust wet';
  $('#pillState').textContent = `net ${fmt(r.wnet, 0)} kJ/kg`; $('#pillState').className = 'pill ok'; $('#pillRun').textContent = s.run ? 'Flowing' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Flowing' : 'Start';
  $('#rigTitle').textContent = { rankine: 'Rankine cycle · boiler, turbine, condenser, feed pump', brayton: 'Brayton cycle · compressor, combustor, turbine' + (s.eps > 0 ? ', regenerator' : ''), combined: 'Combined cycle · gas turbine exhaust raises steam in the HRSG' }[m];
  $('#chartTitle').textContent = m === 'brayton' ? 'T-s diagram · air-standard Brayton cycle' : 'T-s diagram · steam dome and the Rankine cycle'; $('#chart2Title').textContent = m === 'rankine' ? 'Sweep · efficiency and exhaust quality against boiler pressure' : m === 'brayton' ? 'Sweep · efficiency and net work against pressure ratio' : 'Sweep · combined and gas-turbine efficiency against pressure ratio';
}
function record(tag = '') { const s = state, r = R, m = s.mode, row = { i: s.log.length + 1, tag, mode: m, p1: s.p1, T1: s.T1, p2: s.p2, etat: s.etat, reheat: s.reheat, rp: s.rp, T3: s.T3, eps: s.eps, eta: +((r.eta) * 100).toFixed(2), wnet: +r.wnet.toFixed(0), qin: +(m === 'combined' ? r.g.qin : r.qin).toFixed(0), x: m === 'rankine' ? (Number.isFinite(r.x) ? +r.x.toFixed(3) : 'sh') : m === 'combined' ? (Number.isFinite(r.r.x) ? +r.r.x.toFixed(3) : 'sh') : '', bwr: +((m === 'combined' ? r.g.bwr : r.bwr) * 100).toFixed(1), Texh: m === 'rankine' ? '' : +((m === 'combined' ? r.g.T4 : r.T4) - 273.15).toFixed(0), rpopt: m === 'brayton' ? +r.rpopt.toFixed(1) : '', m: +r.m.toFixed(0), hr: +r.hr.toFixed(0) }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.mode, r.p1, r.T1, r.p2, r.etat, r.reheat, r.rp, r.T3, r.eps, r.eta, r.wnet, r.qin, r.x, r.bwr, r.Texh, r.m, r.hr].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,mode,p1_MPa,T1_C,p2_kPa,eta_turbine,reheat,rp,T3_C,regen_eps,efficiency_pct,wnet_kJkg,qin_kJkg,exhaust_quality,bwr_pct,Texh_C,rp_opt,mass_flow_kgs,heat_rate_kJkWh\n' + state.log.map(r => [r.i, r.tag, r.mode, r.p1, r.T1, r.p2, r.etat, r.reheat, r.rp, r.T3, r.eps, r.eta, r.wnet, r.qin, r.x, r.bwr, r.Texh, r.rpopt, r.m, r.hr].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, k) => k.every(x => a[x] === b[x]);
const TASKS = [
  { id: 'base', t: 'Baseline Rankine cycle', d: 'Record the 8 MPa / 500 °C / 10 kPa cycle. Compare its efficiency with the Carnot limit between the same temperatures and note where the loss is.', ok: () => state.log.some(x => x.mode === 'rankine') },
  { id: 'pressure', t: 'Raise the boiler pressure', d: 'Record two Rankine cycles whose boiler pressures differ by 4 MPa or more with the same superheat temperature: efficiency rises, the exhaust gets wetter.', ok: () => { const L = state.log.filter(x => x.mode === 'rankine'); return L.some(a => L.some(b => same(a, b, ['T1', 'p2', 'reheat']) && b.p1 - a.p1 >= 4 && b.eta > a.eta && b.x !== 'sh' && a.x !== 'sh' && b.x < a.x)); } },
  { id: 'superheat', t: 'Superheat dries the exhaust', d: 'Record the same pressure with turbine inlet temperatures 100 K apart: the exhaust quality improves and so does efficiency.', ok: () => { const L = state.log.filter(x => x.mode === 'rankine'); return L.some(a => L.some(b => same(a, b, ['p1', 'p2', 'reheat']) && b.T1 - a.T1 >= 100 && b.x !== 'sh' && a.x !== 'sh' && b.x > a.x)); } },
  { id: 'vacuum', t: 'Condenser vacuum', d: 'Record condenser pressures 40 kPa apart: a better vacuum lowers the heat-rejection temperature and lifts efficiency, at the price of a wetter exhaust and a bigger condenser.', ok: () => { const L = state.log.filter(x => x.mode === 'rankine'); return L.some(a => L.some(b => same(a, b, ['p1', 'T1', 'reheat']) && a.p2 - b.p2 >= 40 && b.eta > a.eta)); } },
  { id: 'reheat', t: 'Reheat cures wetness', d: 'At 12 MPa or more, switch reheat on and record a cycle whose exhaust quality is at least 0.90.', ok: () => state.log.some(x => x.mode === 'rankine' && x.reheat === 'on' && x.p1 >= 12 && x.x !== 'sh' && x.x >= 0.9) },
  { id: 'isen', t: 'Turbine efficiency', d: 'Record the same cycle with turbine isentropic efficiencies 0.10 apart. Irreversibility dries the exhaust but costs work.', ok: () => { const L = state.log.filter(x => x.mode === 'rankine'); return L.some(a => L.some(b => same(a, b, ['p1', 'T1', 'p2', 'reheat']) && Math.abs(a.etat - b.etat) >= 0.1)); } },
  { id: 'rpopt', t: 'Pressure ratio optimum', d: 'Brayton: find the pressure ratio for maximum net work (within 20 % of the optimum) and record it; then note that efficiency keeps rising beyond it.', ok: () => state.log.some(x => x.mode === 'brayton' && Math.abs(x.rp - x.rpopt) <= 0.2 * x.rpopt) },
  { id: 'regen', t: 'Regenerator', d: 'Brayton at a low pressure ratio (6): record without and with a regenerator of effectiveness 0.6 or more. Where does the recovered heat show on the T-s diagram?', ok: () => { const L = state.log.filter(x => x.mode === 'brayton'); return L.some(a => L.some(b => same(a, b, ['rp', 'T3']) && a.eps === 0 && b.eps >= 0.6 && b.eta > a.eta)); } },
  { id: 'combined', t: 'Combined cycle above 50 %', d: 'Combine the two: choose the pressure ratio, turbine inlet temperature and steam conditions that take the combined efficiency to 50 % or more, and record it.', ok: () => state.log.some(x => x.mode === 'combined' && x.eta >= 50) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); if (frame % 6 === 0) { drawTs(); drawSweep(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setMode(m) { state.mode = m; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === m)); document.querySelectorAll('[data-m]').forEach(e => e.classList.toggle('off', !e.dataset.m.split(' ').includes(m))); }
const PRESETS = {
  base: () => { setMode('rankine'); for (const [k, v] of Object.entries({ p1: 8, T1: 500, p2: 10, etat: 0.88, etap: 0.8, prh: 0.25 })) setCtl(k, v); $('#reheat').value = 'off'; state.reheat = 'off'; },
  hp: () => { PRESETS.base(); setCtl('p1', 16); },
  vac: () => { PRESETS.base(); setCtl('p2', 5); },
  reheat: () => { PRESETS.base(); setCtl('p1', 16); $('#reheat').value = 'on'; state.reheat = 'on'; },
  poor: () => { PRESETS.base(); setCtl('etat', 0.75); },
  gt: () => { setMode('brayton'); for (const [k, v] of Object.entries({ rp: 10, Tamb: 15, T3: 1200, etac: 0.85, etagt: 0.88, eps: 0 })) setCtl(k, v); },
  regen: () => { PRESETS.gt(); setCtl('rp', 6); setCtl('eps', 0.8); },
  highrp: () => { PRESETS.gt(); setCtl('rp', 30); },
  cc: () => { PRESETS.base(); PRESETS.gt(); setMode('combined'); setCtl('rp', 16); setCtl('T3', 1400); setCtl('p1', 10); setCtl('T1', 540); setCtl('p2', 5); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('p1', v => v + ' MPa'); bindR('T1', v => v + ' °C'); bindR('p2', v => v + ' kPa'); bindR('etat', v => v.toFixed(2)); bindR('etap', v => v.toFixed(2)); bindR('prh', v => (v * 100).toFixed(0) + ' % of boiler pressure'); bindR('rp', v => String(v)); bindR('Tamb', v => v + ' °C'); bindR('T3', v => v + ' °C'); bindR('etac', v => v.toFixed(2)); bindR('etagt', v => v.toFixed(2)); bindR('eps', v => v.toFixed(2)); bindR('Pmw', v => v + ' MW');
  $('#reheat').addEventListener('change', e => state.reheat = e.target.value); document.querySelectorAll('#types button').forEach(b => b.onclick = () => setMode(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnRecord').onclick = () => record('');
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tc_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'R') record(''); if (k === '1') setMode('rankine'); if (k === '2') setMode('brayton'); if (k === '3') setMode('combined'); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.TurbineCycles = {
  getState: () => ({ inputs: { mode: state.mode, p1: state.p1, T1: state.T1, p2: state.p2, etat: state.etat, etap: state.etap, reheat: state.reheat, prh: state.prh, rp: state.rp, Tamb: state.Tamb, T3: state.T3, etac: state.etac, etagt: state.etagt, eps: state.eps, Pmw: state.Pmw }, results: state.mode === 'combined' ? { eta: R.eta, wnet: R.wnet, etaGT: R.g.eta, etaST: R.r.eta, ms: R.ms, Texh: R.g.T4 - 273.15 } : state.mode === 'rankine' ? { eta: R.eta, wnet: R.wnet, wt: R.wt, wp: R.wp, qin: R.qin, x: R.x, m: R.m, hr: R.hr } : { eta: R.eta, wnet: R.wnet, wc: R.wc, wt: R.wt, qin: R.qin, T2: R.T2 - 273.15, T4: R.T4 - 273.15, rpopt: R.rpopt }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.mode) setMode(inp.mode); for (const k of ['p1', 'T1', 'p2', 'etat', 'etap', 'prh', 'rp', 'Tamb', 'T3', 'etac', 'etagt', 'eps', 'Pmw']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.reheat) { $('#reheat').value = inp.reheat; state.reheat = inp.reheat; } if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'turbine-cycles', title: 'Steam and Gas Turbine Cycle Board', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.TurbineCycles.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'turbine-cycles', state: window.TurbineCycles.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('tc_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setMode('rankine'); step(0); drawRig(); updateReadouts(); drawTs(); drawSweep();
  requestAnimationFrame(tick);
}
build();
