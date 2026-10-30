/* Gas Pipeline Route and Wall Thickness Designer — an onshore gas transmission line designed to ASME B31.8: location
   class from a sliding 1-mile, 1/4-mile-wide unit (840.21, 840.22), design factors with crossing, station and
   concentration-of-people exceptions (Tables 841.114A/B), joint and temperature factors (841.115A, 841.116A), the
   design formula (841.11), standard walls, handling D/t, test pressure (Table 841.322(f)), fracture control (841.11(c)),
   potential impact radius and HCAs (B31.8S), then build the line and let the town grow: MAOP against Table 854.1(c),
   855.1, derate or replace. Standalone, offline. Teaching model — clause numbers follow ASME B31.8-2003 and must be
   checked against the edition in force. */
'use strict';

const r3 = x => Number.isFinite(x) ? Math.round(x * 1000) / 1000 : x; // outputs to three decimals
const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const MI = 1609.344, HALF = MI / 8, CLUSTER = 660 * 0.3048, STEP = 10, WSTEP = 20; // 1 mile unit, 1/8 mile each side, 660 ft cluster end
const WALLS = [4.8, 5.2, 5.6, 6.4, 7.1, 7.9, 8.7, 9.5, 10.3, 11.1, 11.9, 12.7, 14.3, 15.9, 17.5, 19.1, 20.6, 22.2, 23.8, 25.4, 28.6, 31.8];
const GRADES = [[245, 'Gr B'], [290, 'X42'], [317, 'X46'], [359, 'X52'], [386, 'X56'], [414, 'X60'], [448, 'X65'], [483, 'X70'], [552, 'X80']];
// Table 841.114B rows, columns: Class 1 Div 1, Class 1 Div 2, Class 2, Class 3, Class 4
const FTAB = { base: [0.80, 0.72, 0.60, 0.50, 0.40], road: [0.60, 0.60, 0.50, 0.50, 0.40], station: [0.60, 0.60, 0.60, 0.50, 0.40], people: [0.50, 0.50, 0.50, 0.50, 0.40] };
const TTAB = [[121, 1.000], [149, 0.967], [177, 0.933], [204, 0.900], [232, 0.867]]; // Table 841.116A, °F 250..450 in °C
const UNITS = { house: 1, school: 1, multi: 20 }; // dwelling units counted per item; a multistorey block = 20 flats [illustrative]
const MULTI_PREVALENT = 3; // multistorey blocks in a unit taken as "prevalent" → Class 4 [model assumption]
const col = (cls, div) => cls === 1 ? (div === 1 ? 0 : 1) : cls;
const testRatio = (cls, div) => cls === 1 ? (div === 1 ? 1.25 : 1.1) : cls === 2 ? 1.25 : 1.40;
const pirM = (D, Pbar) => 0.69 * (D / 25.4) * Math.sqrt(Pbar * 14.5038) * 0.3048; // B31.8S r = 0.69 d √p (ft, in, psig)
function Tfac(T) { if (T <= TTAB[0][0]) return 1; for (let i = 1; i < TTAB.length; i++) if (T <= TTAB[i][0]) { const [a, fa] = TTAB[i - 1], [b, fb] = TTAB[i]; return fa + (fb - fa) * (T - a) / (b - a); } return TTAB[TTAB.length - 1][1]; }
function rng(seed) { let a = seed >>> 0; return () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }

const state = { L: 12000, OD: 406.4, S: 359, MAOP: 70, Tdes: 40, E: 1, div: 2, CA: 0, dt: 100, cluster: 1, tool: 'inspect', feats: [], hoverX: null, built: null, years: 0, replaced: 0, log: [], flags: {}, csv: '' };
const R = {};

/* ------------------------------------------------------------------ location class (840.2, 840.3) */
function classify(s) {
  const N = Math.floor(s.L / STEP) + 1, cls = new Int8Array(N).fill(1), count = new Int16Array(N), multi = new Uint8Array(N), people = new Uint8Array(N), road = new Uint8Array(N), station = new Uint8Array(N);
  const bld = s.feats.filter(f => UNITS[f.t] && Math.abs(f.y) <= HALF).sort((a, b) => a.x - b.x);
  const idx = x => clamp(Math.round(x / STEP), 0, N - 1);
  const last = Math.max(0, s.L - MI);
  for (let a = 0; a <= last + 1e-6; a += WSTEP) {
    const b = a + Math.min(MI, s.L), inW = bld.filter(f => f.x >= a && f.x <= b);
    if (!inW.length) continue;
    const n = inW.reduce((k, f) => k + UNITS[f.t], 0), nm = inW.filter(f => f.t === 'multi').length;
    const c = nm >= MULTI_PREVALENT ? 4 : n >= 46 ? 3 : n >= 11 ? 2 : 1;
    let lo = a, hi = b;
    if (s.cluster && (c === 2 || c === 3)) { lo = Math.max(a, inW[0].x - CLUSTER); hi = Math.min(b, inW[inW.length - 1].x + CLUSTER); }
    for (let i = idx(lo); i <= idx(hi); i++) { if (c > cls[i]) cls[i] = c; if (n > count[i]) count[i] = n; if (c === 4) multi[i] = 1; }
  }
  for (const f of s.feats) {
    if (f.t === 'school' && Math.abs(f.y) <= HALF) for (let i = idx(f.x - HALF); i <= idx(f.x + HALF); i++) people[i] = 1; // "near" taken as within the zone [model]
    if (f.t === 'road') for (let i = idx(f.x - 15); i <= idx(f.x + 15); i++) road[i] = 1;
    if (f.t === 'station') for (let i = idx(f.x - 100); i <= idx(f.x + 100); i++) station[i] = 1;
  }
  return { N, cls, count, multi, people, road, station };
}

/* ------------------------------------------------------------------ design (841.11, 841.114–116) */
function factorAt(c, i, div) { const k = col(c.cls[i], div); let F = FTAB.base[k]; if (c.road[i]) F = Math.min(F, FTAB.road[k]); if (c.station[i]) F = Math.min(F, FTAB.station[k]); if (c.people[i] && c.cls[i] <= 2) F = Math.min(F, FTAB.people[k]); return F; }
function design(s, c, S = s.S) {
  const P = s.MAOP / 10, D = s.OD, T = Tfac(s.Tdes), E = s.E, tH = D / s.dt, N = c.N;
  const F = new Float64Array(N), treq = new Float64Array(N), t = new Float64Array(N), gov = new Array(N);
  let mass = 0, handling = 0;
  for (let i = 0; i < N; i++) {
    F[i] = factorAt(c, i, s.div);
    const td = P * D / (2 * S * F[i] * E * T) + s.CA;
    treq[i] = Math.max(td, tH);
    t[i] = WALLS.find(w => w >= treq[i] - 1e-9) ?? WALLS[WALLS.length - 1];
    gov[i] = td >= tH ? (td <= WALLS[0] ? 'min' : 'P') : 'D/t';
    if (gov[i] === 'D/t') handling += STEP;
    mass += 0.02466 * (D - t[i]) * t[i] * (i < N - 1 ? STEP : 0);
  }
  return { F, treq, t, gov, mass, handling, T, P, D, S, E };
}

/* ------------------------------------------------------------------ class change of an existing line (854, Table 854.1(c), 855.1) */
function limitsBuilt(s, c) {
  const b = s.built, N = c.N, lim = new Float64Array(N), why = new Array(N);
  const Pat = (i, x) => 2 * b.S * b.t[i] * x / b.D * 10; // bar at a hoop stress of x·SMYS
  for (let i = 0; i < N; i++) {
    const c0 = b.cls0[i], n = c.count[i], TP = b.TP[i];
    const lim0 = Math.min(Pat(i, b.F0[i] * b.E * b.T), TP / testRatio(c0, b.div));
    let L = lim0, w = 'design';
    if (c.multi[i] && c0 < 4) { L = Math.min(lim0, 0.555 * TP, Pat(i, 0.50)); w = '854: multistorey'; }
    else if (c0 === 1 && n > 10) {
      if (n <= 25) { L = Math.min(lim0, b.MAOP0, Pat(i, b.div === 1 ? 0.80 : 0.72)); w = '854: 11–25'; }
      else if (n <= 45) { L = Math.min(lim0, 0.800 * TP, Pat(i, 0.72)); w = '854: 26–45'; }
      else { L = Math.min(lim0, 0.667 * TP, Pat(i, 0.60)); w = '854: 46+'; }
    } else if (c0 === 2 && n > 45) {
      if (n <= 65) { L = Math.min(lim0, b.MAOP0, Pat(i, 0.60)); w = '854: 46–65'; }
      else { L = Math.min(lim0, 0.667 * TP, Pat(i, 0.60)); w = '854: 66+'; }
    }
    if (c.people[i] && !b.people0[i] && c0 <= 2) { L = Math.min(L, Pat(i, 0.50)); w = '855.1: people'; }
    lim[i] = L; why[i] = w;
  }
  return { lim, why };
}

/* ------------------------------------------------------------------ high consequence areas (B31.8S, PIR) */
function hca(s, c, r) {
  const N = c.N, h = new Uint8Array(N), bl = s.feats.filter(f => UNITS[f.t]);
  for (let i = 0; i < N; i++) {
    if (c.cls[i] >= 3) { h[i] = 1; continue; }
    const x = i * STEP; let n = 0, site = false;
    for (const f of bl) { if (Math.abs(f.x - x) > r) continue; if (Math.hypot(f.x - x, f.y) <= r) { n += UNITS[f.t]; if (f.t === 'school') site = true; } }
    if (site || n >= 20) h[i] = 1; // identified site or 20+ buildings in the circle [check against B31.8S]
  }
  return h;
}

function compute() {
  const s = state, c = classify(s), d = design(s, c), r = pirM(s.OD, s.MAOP), h = hca(s, c, r), N = c.N;
  const len = [0, 0, 0, 0, 0]; for (let i = 0; i < N - 1; i++) len[c.cls[i]] += STEP;
  const tUse = s.built ? s.built.t : d.t;
  const hoop = new Float64Array(N), limPct = new Float64Array(N); let maxPct = 0, over = 0, hcaLen = 0;
  const lb = s.built ? limitsBuilt(s, c) : null;
  for (let i = 0; i < N; i++) {
    hoop[i] = d.P * s.OD / (2 * tUse[i]) / s.S * 100; maxPct = Math.max(maxPct, hoop[i]);
    limPct[i] = lb ? lb.lim[i] / 10 * s.OD / (2 * tUse[i]) / s.S * 100 : d.F[i] * s.E * d.T * 100;
    if (i < N - 1) { if (lb && s.MAOP > lb.lim[i] + 1e-6) over += STEP; if (h[i]) hcaLen += STEP; }
  }
  const walls = [...new Set(Array.from(d.t))].sort((a, b) => a - b);
  const units = s.feats.reduce((k, f) => k + (UNITS[f.t] || 0), 0);
  const tp = {}; for (let i = 0; i < N; i++) { const k = c.cls[i] === 1 ? 'C1' : 'C' + c.cls[i]; tp[k] = testRatio(c.cls[i], s.div) * s.MAOP; }
  const fracture = s.OD >= 406.4 ? maxPct > 40 : maxPct > 72;
  const sweep = GRADES.map(([S, n]) => { const g = design(s, c, S); return { S, n, mass: g.mass, handling: g.handling > 0 }; });
  Object.assign(R, { c, d, r, h, len, hoop, limPct, maxPct, over, hcaLen, walls, units, tp, fracture, lb, sweep, tUse, minLim: lb ? Math.min(...lb.lim) : null });
}

/* ------------------------------------------------------------------ route scenarios */
function baseFeats(seed = 7) {
  const r = rng(seed), f = [], u = (a, b) => a + (b - a) * r(), side = () => (r() < 0.5 ? -1 : 1);
  for (let k = 0; k < 8; k++) f.push({ t: 'house', x: u(300, 4500), y: side() * u(80, 650) });
  for (let k = 0; k < 18; k++) f.push({ t: 'house', x: u(5200, 6300), y: u(-180, 180) });
  for (let k = 0; k < 56; k++) f.push({ t: 'house', x: u(8600, 10300), y: u(-190, 190) });
  for (let k = 0; k < 20; k++) f.push({ t: 'house', x: u(8400, 10600), y: side() * u(260, 650) });
  f.push({ t: 'school', x: 9500, y: 60 }, { t: 'road', x: 2400, y: 0 }, { t: 'road', x: 7300, y: 0 }, { t: 'station', x: 11700, y: 0 });
  return f;
}
function grow() { // five years of development around the existing buildings
  const s = state; s.years += 5; const r = rng(1000 + s.years), anchors = s.feats.filter(f => f.t === 'house');
  for (let k = 0; k < 14 && anchors.length; k++) { const a = anchors[Math.floor(r() * anchors.length)]; s.feats.push({ t: 'house', x: clamp(a.x + (r() - 0.5) * 300, 50, s.L - 50), y: clamp(a.y + (r() - 0.5) * 300, -680, 680) }); }
  if (s.years === 10) { const c = classify(s); let bi = 0; for (let i = 0; i < c.N; i++) if (c.count[i] > c.count[bi] && c.cls[i] <= 2) bi = i; s.feats.push({ t: 'school', x: bi * STEP, y: 110 }); }
  dirty = true;
}
function build() {
  const s = state; compute(); const c = R.c, d = R.d;
  s.built = { t: Float64Array.from(d.t), F0: Float64Array.from(d.F), cls0: Int8Array.from(c.cls), people0: Uint8Array.from(c.people), TP: Float64Array.from(c.cls, k => testRatio(k, s.div) * s.MAOP), MAOP0: s.MAOP, div: s.div, D: s.OD, S: s.S, E: s.E, T: d.T };
  s.years = 0; s.replaced = 0; s.flags.built = true; lockPipe(true); dirty = true;
}
function unbuild() { state.built = null; state.replaced = 0; state.years = 0; lockPipe(false); dirty = true; }
function derate() { if (!state.built) return; compute(); const p = Math.floor(R.minLim); setCtl('MAOP', clamp(p, 10, 150)); state.flags.derate = true; dirty = true; }
function replace() { // 854.2(d): replace with pipe designed to Table 841.114A for the current class, tested afresh
  const s = state, b = s.built; if (!b) return; compute(); const c = R.c, d = R.d, lb = R.lb; let n = 0;
  for (let i = 0; i < c.N; i++) if (s.MAOP > lb.lim[i] + 1e-6) { b.t[i] = d.t[i]; b.F0[i] = d.F[i]; b.cls0[i] = c.cls[i]; b.people0[i] = c.people[i]; b.TP[i] = testRatio(c.cls[i], b.div) * s.MAOP; n++; }
  s.replaced += n * STEP; s.flags.replace = n > 0; dirty = true;
}
function lockPipe(on) { for (const id of ['OD', 'grade', 'seam', 'div', 'Tdes', 'CA', 'dt']) { const e = $('#' + id); if (e) e.disabled = on; } }

/* ------------------------------------------------------------------ route plan (SVG) */
const VW = 1000, VH = 470, X0 = 60, XW = 920, YC = 135, KY = 0.15;
const xs = x => X0 + x / state.L * XW, ys = y => YC - y * KY;
function segs(N, key) { const out = []; let a = 0; for (let i = 1; i <= N; i++) if (i === N || key(i) !== key(a)) { out.push({ a, b: i - 1, v: key(a) }); a = i; } return out; }
const clsCol = k => k === 1 ? 'var(--good)' : k === 2 ? 'var(--brass)' : 'var(--alarm)';
function unitAt(x) {
  const a = x - MI / 2, b = x + MI / 2, inW = state.feats.filter(f => UNITS[f.t] && Math.abs(f.y) <= HALF && f.x >= a && f.x <= b);
  const n = inW.reduce((k, f) => k + UNITS[f.t], 0), nm = inW.filter(f => f.t === 'multi').length, sc = inW.some(f => f.t === 'school');
  return { n, nm, sc, cls: nm >= MULTI_PREVALENT ? 4 : n >= 46 ? 3 : n >= 11 ? 2 : 1 };
}
function drawRig() {
  const s = state, c = R.c, d = R.d, N = c.N, P = [];
  P.push(`<svg viewBox="0 0 ${VW} ${VH}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Pipeline route plan with buildings, location classes and wall thickness">`);
  P.push(`<rect x="${X0}" y="30" width="${XW}" height="210" style="fill:var(--panel2);stroke:var(--line)"/>`);
  for (const g of segs(N, i => c.cls[i])) P.push(`<rect x="${xs(g.a * STEP)}" y="30" width="${Math.max(xs((g.b + 1) * STEP) - xs(g.a * STEP), 0.5)}" height="210" style="fill:${clsCol(g.v)};opacity:${g.v === 1 ? 0.07 : g.v === 4 ? 0.32 : 0.18}"/>`);
  for (const sg of [1, -1]) P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="${ys(sg * HALF)}" y2="${ys(sg * HALF)}" style="stroke:var(--muted);stroke-dasharray:2 4;opacity:.6"/>`);
  for (const sg of [1, -1]) if (R.r < 700) P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="${ys(sg * R.r)}" y2="${ys(sg * R.r)}" style="stroke:var(--alarm);stroke-dasharray:6 4;opacity:.55"/>`);
  P.push(`<text x="${X0 + 4}" y="${ys(HALF) - 3}" style="fill:var(--muted);font:10px var(--mono)">¼-mile zone edge</text>`);
  if (R.r < 700) P.push(`<text x="${X0 + XW - 4}" y="${ys(R.r) - 3}" text-anchor="end" style="fill:var(--alarm);font:10px var(--mono)">PIR ${fmt(R.r, 0)} m</text>`);
  for (const f of s.feats) {
    const x = xs(f.x), y = ys(f.y);
    if (f.t === 'road') P.push(`<line x1="${x}" x2="${x}" y1="30" y2="240" style="stroke:var(--steel);stroke-width:5;opacity:.6"/><text x="${x + 4}" y="42" style="fill:var(--muted);font:10px var(--mono)">road</text>`);
    else if (f.t === 'house') P.push(`<rect x="${x - 2.5}" y="${y - 2.5}" width="5" height="5" style="fill:var(--brass)"/>`);
    else if (f.t === 'multi') P.push(`<rect x="${x - 4}" y="${y - 9}" width="8" height="14" style="fill:var(--steel);stroke:var(--ink)"/>`);
    else if (f.t === 'school') P.push(`<rect x="${x - 7}" y="${y - 6}" width="14" height="12" style="fill:var(--alarm)"/><text x="${x}" y="${y + 4}" text-anchor="middle" style="fill:#fff;font:700 9px var(--mono)">S</text>`);
    else if (f.t === 'station') P.push(`<rect x="${x - 9}" y="${YC - 9}" width="18" height="18" style="fill:var(--panel);stroke:var(--ink);stroke-width:1.5"/><text x="${x}" y="${YC - 13}" text-anchor="middle" style="fill:var(--ink);font:10px var(--mono)">M/R</text>`);
  }
  const bad = s.built ? (i => s.MAOP > R.lb.lim[i] + 1e-6 ? 1 : 0) : (() => 0);
  for (const g of segs(N, i => R.tUse[i] * 10 + bad(i) * 1000)) {
    const t = R.tUse[g.a], isBad = bad(g.a);
    P.push(`<line x1="${xs(g.a * STEP)}" x2="${xs(Math.min(g.b + 1, N - 1) * STEP)}" y1="${YC}" y2="${YC}" style="stroke:${isBad ? 'var(--alarm)' : 'var(--water)'};stroke-width:${2 + t / 2.5};stroke-linecap:butt"/>`);
  }
  if (s.hoverX !== null) {
    const u = unitAt(s.hoverX), a = xs(clamp(s.hoverX - MI / 2, 0, s.L)), b = xs(clamp(s.hoverX + MI / 2, 0, s.L));
    P.push(`<rect x="${a}" y="${ys(HALF)}" width="${b - a}" height="${ys(-HALF) - ys(HALF)}" style="fill:none;stroke:var(--ink);stroke-width:1.5;stroke-dasharray:6 4"/>`);
    P.push(`<text x="${(a + b) / 2}" y="${ys(HALF) - 6}" text-anchor="middle" style="fill:var(--ink);font:600 11px var(--mono)">${u.n} buildings${u.nm ? ' · ' + u.nm + ' multistorey' : ''}${u.sc ? ' · school' : ''} → Class ${u.cls}</text>`);
  }
  // strips
  const lab = (y, t) => P.push(`<text x="${X0 - 6}" y="${y}" text-anchor="end" style="fill:var(--muted);font:10px var(--mono)">${t}</text>`);
  lab(271, 'Class'); for (const g of segs(N, i => c.cls[i])) { const a = xs(g.a * STEP), w = xs((g.b + 1) * STEP) - a; P.push(`<rect x="${a}" y="258" width="${Math.max(w, 0.5)}" height="18" style="fill:${clsCol(g.v)};opacity:${g.v === 4 ? 1 : 0.75}"/>`); if (w > 14) P.push(`<text x="${a + w / 2}" y="271" text-anchor="middle" style="fill:#fff;font:700 10px var(--mono)">${g.v}</text>`); }
  lab(295, 'F'); for (const g of segs(N, i => d.F[i])) { const a = xs(g.a * STEP), w = xs((g.b + 1) * STEP) - a; P.push(`<line x1="${a}" x2="${a}" y1="282" y2="300" style="stroke:var(--line)"/>`); if (w > 26) P.push(`<text x="${a + w / 2}" y="295" text-anchor="middle" style="fill:var(--ink);font:10px var(--mono)">${g.v.toFixed(2)}</text>`); }
  const tmax = Math.max(...R.tUse, ...d.treq) * 1.15, ty = t => 380 - t / tmax * 70;
  lab(345, 'Wall mm'); P.push(`<line x1="${X0}" x2="${X0 + XW}" y1="380" y2="380" style="stroke:var(--line)"/>`);
  const poly = arr => Array.from(arr, (t, i) => `${xs(i * STEP).toFixed(1)},${ty(t).toFixed(1)}`).join(' ');
  P.push(`<polyline points="${poly(d.treq)}" style="fill:none;stroke:var(--muted);stroke-dasharray:3 3"/>`);
  P.push(`<polyline points="${poly(d.t)}" style="fill:none;stroke:var(--brass);stroke-width:2"/>`);
  if (s.built) P.push(`<polyline points="${poly(s.built.t)}" style="fill:none;stroke:var(--water);stroke-width:2;stroke-dasharray:8 3"/>`);
  for (const g of segs(N, i => d.t[i])) { const a = xs(g.a * STEP), w = xs((g.b + 1) * STEP) - a; if (w > 30) P.push(`<text x="${a + w / 2}" y="${ty(g.v) - 4}" text-anchor="middle" style="fill:var(--ink);font:10px var(--mono)">${g.v}</text>`); }
  lab(396, 'HCA'); for (const g of segs(N, i => R.h[i])) if (g.v) P.push(`<rect x="${xs(g.a * STEP)}" y="388" width="${Math.max(xs((g.b + 1) * STEP) - xs(g.a * STEP), 1)}" height="10" style="fill:var(--alarm);opacity:.75"/>`);
  if (s.built) { lab(414, 'MAOP'); for (const g of segs(N, bad)) P.push(`<rect x="${xs(g.a * STEP)}" y="406" width="${Math.max(xs((g.b + 1) * STEP) - xs(g.a * STEP), 1)}" height="10" style="fill:${g.v ? 'var(--alarm)' : 'var(--good)'};opacity:${g.v ? 0.9 : 0.35}"/>`); }
  for (let k = 0; k <= s.L / 1000; k++) { const x = xs(k * 1000); P.push(`<line x1="${x}" x2="${x}" y1="426" y2="432" style="stroke:var(--muted)"/><text x="${x}" y="446" text-anchor="middle" style="fill:var(--muted);font:10px var(--mono)">${k}</text>`); }
  P.push(`<text x="${X0 + XW}" y="462" text-anchor="end" style="fill:var(--muted);font:10px var(--mono)">route km · wall: brass ordered, dashed required${s.built ? ', blue dashed as built' : ''}</text>`);
  P.push('</svg>');
  $('#rig').innerHTML = P.join('');
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function axes(ctx, W, H, pad, xmax, ymax, xl, yl, nx = 5, ny = 4) {
  ctx.fillStyle = css('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.strokeStyle = css('--grid');
  for (let k = 0; k <= nx; k++) { const x = pad.l + k / nx * (W - pad.l - pad.r); ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, H - pad.b); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(+(k / nx * xmax).toFixed(1), x, H - pad.b + 14); }
  for (let k = 0; k <= ny; k++) { const y = H - pad.b - k / ny * (H - pad.t - pad.b); ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(+(k / ny * ymax).toFixed(0), pad.l - 6, y + 4); }
  ctx.textAlign = 'center'; ctx.fillText(xl, (pad.l + W - pad.r) / 2, H - 4); ctx.save(); ctx.translate(12, (pad.t + H - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText(yl, 0, 0); ctx.restore();
}
function drawPT() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, s = state, d = R.d;
  const tx = Math.max(12, Math.min(32, Math.max(...R.walls) * 1.8)), Pm = Math.max(s.MAOP * 1.8, 40);
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, tx, Pm, 'nominal wall t, mm', 'design pressure, bar');
  const X = t => pad.l + t / tx * (W - pad.l - pad.r), Y = p => H - pad.b - clamp(p / Pm, 0, 1.05) * (H - pad.t - pad.b);
  const lines = [[0.80, '--good', 'C1 Div 1'], [0.72, '--good', 'C1 Div 2'], [0.60, '--brass', 'C2'], [0.50, '--alarm', 'C3'], [0.40, '--alarm', 'C4']];
  lines.forEach(([F, cvar, lb], k) => { const k1 = 2 * s.S * F * s.E * d.T / s.OD * 10, te = Math.min(tx, Pm / k1); ctx.strokeStyle = css(cvar); ctx.lineWidth = F === 0.80 || F === 0.40 ? 1.2 : 2; ctx.setLineDash(F === 0.80 || F === 0.40 ? [5, 4] : []); ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(te), Y(k1 * te)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css(cvar); ctx.textAlign = 'left'; ctx.fillText(`F ${F.toFixed(2)}  ${lb}`, pad.l + 8, pad.t + 16 + k * 14); });
  ctx.strokeStyle = css('--water'); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(0), Y(s.MAOP)); ctx.lineTo(X(tx), Y(s.MAOP)); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = css('--water'); ctx.textAlign = 'right'; ctx.fillText(`MAOP ${s.MAOP} bar`, W - pad.r - 4, Y(s.MAOP) - 5);
  for (const t of R.walls) { ctx.beginPath(); ctx.arc(X(t), Y(s.MAOP), 4, 0, 7); ctx.fill(); ctx.textAlign = 'center'; ctx.fillText(t, X(t), Y(s.MAOP) + 16); }
  const tH = s.OD / s.dt; ctx.strokeStyle = css('--steel'); ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(tH), pad.t); ctx.lineTo(X(tH), H - pad.b); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--steel'); ctx.textAlign = 'left'; ctx.fillText(`handling D/t ${s.dt}`, X(tH) + 4, H - pad.b - 6);
}
function drawHoop() {
  const cv = $('#schart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, s = state, N = R.c.N;
  frame(ctx, W, H, pad); axes(ctx, W, H, pad, s.L / 1000, 100, 'route km', '% SMYS', 6, 5);
  const X = i => pad.l + i / (N - 1) * (W - pad.l - pad.r), Y = p => H - pad.b - clamp(p / 100, 0, 1) * (H - pad.t - pad.b);
  const plot = (arr, cv_, w) => { ctx.strokeStyle = css(cv_); ctx.lineWidth = w; ctx.beginPath(); for (let i = 0; i < N; i++) { i ? ctx.lineTo(X(i), Y(arr[i])) : ctx.moveTo(X(i), Y(arr[i])); } ctx.stroke(); };
  plot(R.limPct, '--alarm', 2); plot(R.hoop, '--water', 2.5);
  ctx.fillStyle = css('--muted'); ctx.textAlign = 'left'; ctx.fillText(R.fracture ? 'above the fracture-control threshold (841.11(c))' : '', pad.l + 6, pad.t + 14);
  ctx.strokeStyle = css('--steel'); ctx.setLineDash([2, 4]); const th = s.OD >= 406.4 ? 40 : 72; ctx.beginPath(); ctx.moveTo(pad.l, Y(th)); ctx.lineTo(W - pad.r, Y(th)); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--steel'); ctx.textAlign = 'right'; ctx.fillText(`${th} % fracture control`, W - pad.r - 4, Y(th) - 4);
}
function drawSweep() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 56, r: 18, t: 14, b: 36 }, s = state, sw = R.sweep;
  const ym = Math.max(...sw.map(g => g.mass)) / 1000 * 1.15; frame(ctx, W, H, pad);
  ctx.fillStyle = css('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.textAlign = 'right';
  for (let k = 0; k <= 4; k++) { const y = H - pad.b - k / 4 * (H - pad.t - pad.b); ctx.fillText((k / 4 * ym).toFixed(0), pad.l - 6, y + 4); }
  ctx.save(); ctx.translate(12, (pad.t + H - pad.b) / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('steel, t', 0, 0); ctx.restore();
  const bw = (W - pad.l - pad.r) / sw.length;
  sw.forEach((g, k) => { const h = g.mass / 1000 / ym * (H - pad.t - pad.b), x = pad.l + k * bw + bw * 0.18; ctx.fillStyle = css(g.handling ? '--steel' : '--water'); ctx.fillRect(x, H - pad.b - h, bw * 0.64, h); if (g.S === s.S) { ctx.strokeStyle = css('--brass'); ctx.lineWidth = 3; ctx.strokeRect(x - 2, H - pad.b - h - 2, bw * 0.64 + 4, h + 2); } ctx.fillStyle = css('--muted'); ctx.textAlign = 'center'; ctx.fillText(g.n, x + bw * 0.32, H - pad.b + 14); ctx.fillText((g.mass / 1000).toFixed(0), x + bw * 0.32, H - pad.b - h - 6); });
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const s = state, d = R.d, km = m => (m / 1000).toFixed(1);
  if (s.hoverX !== null) { const u = unitAt(s.hoverX); $('#roUnit').textContent = `${u.n} buildings → Class ${u.cls}`; $('#roUnitNote').textContent = `centred at ${km(s.hoverX)} km${u.nm ? ' · ' + u.nm + ' multistorey blocks' : ''}${u.sc ? ' · school in the zone: Class 3 rules near it (840.3)' : ''}`; if (u.cls === 2) s.flags.sawC2 = true; if (u.cls >= 3) s.flags.sawC3 = true; }
  $('#roClass').textContent = [1, 2, 3, 4].map(k => `C${k} ${km(R.len[k])}`).join(' · ') + ' km';
  $('#roClassNote').textContent = `${R.units} dwelling units on the map · cluster rule ${s.cluster ? 'on (660 ft)' : 'off'}`;
  $('#roWall').textContent = R.walls.join(' / ') + ' mm';
  $('#roWallNote').textContent = d.handling > 0 ? `handling D/t ≤ ${s.dt} governs ${km(d.handling)} km (t ≥ ${fmt(s.OD / s.dt, 2)} mm)` : 'pressure governs everywhere';
  $('#roSteel').textContent = fmt(d.mass / 1000, 0); $('#roSteelNote').textContent = `t of line pipe for ${km(s.L)} km · ${fmt(d.mass / s.L, 0)} kg/m average`;
  $('#roHoop').textContent = fmt(R.maxPct, 1) + ' %'; $('#roHoopNote').textContent = `of SMYS at ${s.MAOP} bar · T ${d.T.toFixed(3)} · E ${s.E.toFixed(2)}${R.fracture ? ' · fracture control required (841.11(c))' : ''}`;
  $('#roTest').textContent = Object.entries(R.tp).map(([k, v]) => `${k} ${fmt(v, 0)}`).join(' · ') + ' bar'; $('#roTestNote').textContent = s.built ? `recorded at build for MAOP ${s.built.MAOP0} bar` : 'water test, minimum, per location class';
  $('#roPIR').textContent = fmt(R.r, 0) + ' m'; $('#roPIRNote').textContent = `HCA ${km(R.hcaLen)} km · r = 0.69 d √p (B31.8S)`;
  if (s.built) { $('#roBuilt').textContent = R.over > 0 ? `${km(R.over)} km not commensurate` : 'Compliant'; $('#roBuiltNote').textContent = `built at ${s.built.MAOP0} bar · ${s.years} years on · lowest allowed MAOP ${fmt(R.minLim, 1)} bar${s.replaced ? ' · replaced ' + km(s.replaced) + ' km' : ''}`; }
  else { $('#roBuilt').textContent = 'Design stage'; $('#roBuiltNote').textContent = 'Press Build pipeline to freeze the walls and test pressure'; }
  $('#roBuiltBox').classList.toggle('alarm', !!s.built && R.over > 0);
  const maxC = Math.max(...R.c.cls); $('#pillClass').textContent = `Highest class ${maxC}`;
  $('#pillState').textContent = s.built ? `As built · year ${s.years}` : 'Design stage';
  $('#pillOver').classList.toggle('hidden', !(s.built && R.over > 0));
  $('#pillHCA').classList.toggle('hidden', R.hcaLen <= 0); $('#pillFrac').classList.toggle('hidden', !R.fracture); $('#pillTemp').classList.toggle('hidden', s.Tdes <= 232);
}

/* ------------------------------------------------------------------ log and tasks */
function record(tag = '') {
  const s = state, gname = (GRADES.find(g => g[0] === s.S) || [0, '?'])[1];
  const row = { i: s.log.length + 1, tag, OD: s.OD, grade: gname, MAOP: s.MAOP, T: s.Tdes, E: s.E, div: s.div, c1: r3(R.len[1] / 1000), c2: r3(R.len[2] / 1000), c3: r3(R.len[3] / 1000), c4: r3(R.len[4] / 1000), walls: R.walls.join('/'), steel: r3(R.d.mass / 1000), pct: r3(R.maxPct), gov: R.d.handling > 0 ? 'D/t' : 'P', pir: r3(R.r), hca: r3(R.hcaLen / 1000), built: s.built ? 'yes' : 'no', over: r3(R.over / 1000), repl: r3(s.replaced / 1000), units: R.units, cluster: s.cluster, S: s.S };
  s.log.push(row); renderLog(); checkTasks();
}
function renderLog() {
  const K = ['i', 'tag', 'OD', 'grade', 'MAOP', 'T', 'E', 'div', 'c1', 'c2', 'c3', 'c4', 'walls', 'steel', 'pct', 'gov', 'pir', 'hca', 'built', 'over', 'repl', 'units'];
  $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${K.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length;
  state.csv = 'n,tag,OD_mm,grade,MAOP_bar,T_C,E,class1_division,class1_km,class2_km,class3_km,class4_km,walls_mm,steel_t,max_pct_SMYS,governs,PIR_m,HCA_km,built,non_compliant_km,replaced_km,dwelling_units\n' + state.log.map(r => K.map(k => r[k]).join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'baseline', t: 'Read the baseline', d: 'Record the 12 km route as it opens. Read the length in each location class, the walls used and the steel.', ok: () => state.log.length > 0 },
  { id: 'unit', t: 'Slide the unit', d: 'Move the pointer along the route. Find a position where the 1-mile unit holds 11–45 buildings (Class 2) and one where it holds 46 or more (Class 3).', ok: () => state.flags.sawC2 && state.flags.sawC3 },
  { id: 'cluster', t: 'Where does the class end?', d: 'Record the route with the cluster rule on and again with it off (Class boundaries). The cluster rule ends Class 2 and 3 at 660 ft past the last building, so less heavy pipe is needed.', ok: () => state.log.some(a => state.log.some(b => same(a, b, ['OD', 'S', 'MAOP', 'units']) && a.cluster !== b.cluster && Math.abs((a.c2 + a.c3) - (b.c2 + b.c3)) >= 0.2)) },
  { id: 'houses', t: 'Eleven houses', d: 'With the House or Estate tool, add buildings in a Class 1 stretch until it turns into Class 2. Watch F drop from 0.72 to 0.60 and the wall step up.', ok: () => state.flags.made2 },
  { id: 'school', t: 'Concentration of people', d: 'Place a school close to the line in a Class 1 stretch. Table 841.114B treats it as Class 3: F 0.50 near it.', ok: () => state.flags.schoolC1 },
  { id: 'road', t: 'Road crossing', d: 'Add a road crossing in a Class 1 stretch. An uncased crossing of a hard-surface road needs F 0.60 for a short length: find the local wall step.', ok: () => state.flags.roadC1 },
  { id: 'grade', t: 'Buy a better steel', d: 'Record the same route in two grades. Use the grade chart: the steel falls with SMYS until the handling D/t limit takes over.', ok: () => state.log.some(a => state.log.some(b => same(a, b, ['OD', 'MAOP', 'units', 'div']) && a.S !== b.S && Math.abs(a.steel - b.steel) / Math.max(a.steel, b.steel) >= 0.15)) },
  { id: 'handling', t: 'Too thin to handle', d: 'Find a design where the handling D/t limit, not pressure, sets the wall (small pipe, low pressure or high grade). Record it.', ok: () => state.log.some(x => x.gov === 'D/t') },
  { id: 'div1', t: 'Division 1', d: 'Record the rural route with Class 1 Division 2 and again with Division 1. Division 1 saves steel but needs a 1.25 × MAOP water test.', ok: () => state.log.some(a => a.div === 1 && state.log.some(b => b.div === 2 && same(a, b, ['OD', 'S', 'MAOP', 'units']) && a.steel < b.steel)) },
  { id: 'hot', t: 'Hot gas', d: 'Raise the design temperature above 121 °C (compressor discharge). Record it and read the temperature derating factor T.', ok: () => state.log.some(x => x.T > 121) },
  { id: 'grow', t: 'The town grows', d: 'Build the pipeline, then press +5 years until part of the line is no longer commensurate with its location class (Table 854.1(c)). Record it.', ok: () => state.log.some(x => x.built === 'yes' && x.over > 0) },
  { id: 'derate', t: 'Comply by derating', d: 'Reduce the MAOP to comply and record. Note the capacity you gave up.', ok: () => state.flags.derate && state.log.some(x => x.built === 'yes' && x.over === 0 && x.units > 0) },
  { id: 'replace', t: 'Comply by replacing', d: 'Instead keep the MAOP: replace the non-compliant pipe with pipe designed for the new class (854.2(d)). Record the length replaced.', ok: () => state.log.some(x => x.repl > 0 && x.over === 0) },
  { id: 'pir', t: 'Potential impact radius', d: 'Record two designs whose PIR differs by 30 % or more (change diameter or MAOP). Watch the HCA length follow.', ok: () => state.log.some(a => state.log.some(b => Math.abs(a.pir - b.pir) / Math.max(a.pir, b.pir) >= 0.3)) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ placing items */
function place(x, y) {
  const s = state, t = s.tool; if (t === 'inspect') return;
  if (t === 'erase') { let bi = -1, bd = 1e9; s.feats.forEach((f, i) => { const dd = Math.hypot(xs(f.x) - xs(x), (f.t === 'road' || f.t === 'station' ? 0 : ys(f.y) - ys(y))); if (dd < bd) { bd = dd; bi = i; } }); if (bi >= 0 && bd < 25) s.feats.splice(bi, 1); dirty = true; return; }
  const before = classify(s), i0 = clamp(Math.round(x / STEP), 0, before.N - 1);
  if (t === 'estate') { const r = rng(Math.round(x * 7 + y)); for (let k = 0; k < 10; k++) s.feats.push({ t: 'house', x: clamp(x + (r() - 0.5) * 160, 0, s.L), y: clamp(y + (r() - 0.5) * 120, -690, 690) }); }
  else if (t === 'road' || t === 'station') s.feats.push({ t, x, y: 0 });
  else s.feats.push({ t, x, y });
  const after = classify(s);
  for (let i = 0; i < after.N; i++) if (before.cls[i] === 1 && after.cls[i] === 2) { s.flags.made2 = true; break; }
  if (t === 'school' && after.people[i0] && after.cls[i0] <= 2) s.flags.schoolC1 = true;
  if (t === 'road' && after.cls[i0] === 1) s.flags.roadC1 = true;
  dirty = true;
}
function mapXY(e) { const r = $('#rig svg').getBoundingClientRect(), sx = (e.clientX - r.left) / r.width * VW, sy = (e.clientY - r.top) / r.height * VH; if (sx < X0 || sx > X0 + XW || sy < 30 || sy > 240) return null; return { x: (sx - X0) / XW * state.L, y: (YC - sy) / KY }; }

/* ------------------------------------------------------------------ loop and wiring */
let dirty = true, redraw = true;
function refresh() { compute(); dirty = false; redraw = true; }
function tick() { if (dirty) refresh(); if (redraw) { drawRig(); updateReadouts(); drawPT(); drawHoop(); drawSweep(); checkTasks(); redraw = false; } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setTool(t) { state.tool = t; document.querySelectorAll('[data-tool]').forEach(b => b.classList.toggle('on', b.dataset.tool === t)); }
function reset(over = {}) {
  unbuild(); Object.assign(state, { feats: baseFeats(), years: 0, replaced: 0, hoverX: null });
  const v = Object.assign({ OD: 406.4, grade: 359, MAOP: 70, Tdes: 40, seam: 1, div: 2, CA: 0, dt: 100, cluster: 1 }, over);
  for (const [k, x] of Object.entries(v)) setCtl(k, x);
  dirty = true;
}
const PRESETS = {
  base: () => reset(),
  rural: () => { reset(); state.feats = state.feats.filter(f => f.x < 5000 || f.t === 'station'); dirty = true; },
  div1: () => { PRESETS.rural(); setCtl('div', 1); },
  city: () => { reset(); [8800, 9100, 9400, 9700, 10000, 10200].forEach((x, k) => state.feats.push({ t: 'multi', x, y: k % 2 ? 110 : -120 })); dirty = true; },
  encroach: () => { reset(); refresh(); build(); grow(); grow(); grow(); },
  school: () => { PRESETS.rural(); refresh(); build(); state.feats.push({ t: 'school', x: 3500, y: 90 }); state.flags.schoolC1 = true; dirty = true; },
  big: () => reset({ OD: 914, grade: 483, MAOP: 100 }),
  small: () => reset({ OD: 168.3, grade: 290, MAOP: 20 }),
  hot: () => reset({ Tdes: 180 }),
};
function bind() {
  const bindR = (id, f, key = id) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[key] = +e.value; if (o) o.textContent = f(state[key]); dirty = true; }; e.addEventListener('input', set); set(); };
  bindR('MAOP', v => v + ' bar'); bindR('Tdes', v => v + ' °C'); bindR('CA', v => v.toFixed(1) + ' mm'); bindR('dt', v => '≤ ' + v);
  for (const [id, key] of [['OD', 'OD'], ['grade', 'S'], ['seam', 'E'], ['div', 'div'], ['cluster', 'cluster']]) { const e = $('#' + id); const set = () => { state[key] = +e.value; dirty = true; }; e.addEventListener('change', set); set(); }
  document.querySelectorAll('[data-tool]').forEach(b => b.onclick = () => setTool(b.dataset.tool));
  const rig = $('#rig');
  rig.addEventListener('pointermove', e => { const p = mapXY(e); state.hoverX = p ? clamp(p.x, 0, state.L) : null; redraw = true; });
  rig.addEventListener('pointerleave', () => { state.hoverX = null; redraw = true; });
  rig.addEventListener('click', e => { const p = mapXY(e); if (p) place(clamp(p.x, 0, state.L), clamp(p.y, -690, 690)); });
  $('#btnBuild').onclick = () => { refresh(); build(); }; $('#btnYears').onclick = () => { if (!state.built) { refresh(); build(); } grow(); };
  $('#btnDerate').onclick = derate; $('#btnReplace').onclick = replace; $('#btnUnbuild').onclick = unbuild; $('#btnRecord').onclick = () => { if (dirty) refresh(); record(''); };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { const done = () => { $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); }; try { navigator.clipboard.writeText(state.csv || '').then(done, () => { $('#btnCopy').textContent = 'Use Download CSV'; }); } catch (e) { $('#btnCopy').textContent = 'Use Download CSV'; } };
  $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('tk_theme', next); } catch (e) {} redraw = true; };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  const TOOLS = ['inspect', 'house', 'estate', 'multi', 'school', 'road', 'station', 'erase'];
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (k === 'B') $('#btnBuild').click(); if (k === 'Y') $('#btnYears').click(); if (k === 'R') $('#btnRecord').click(); if (/^[1-8]$/.test(e.key)) setTool(TOOLS[+e.key - 1]); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
  window.addEventListener('resize', () => redraw = true);
}

/* ------------------------------------------------------------------ dashboard API */
window.PipelineRoute = {
  getState: () => { if (dirty) refresh(); const s = state; return {
    inputs: { OD: s.OD, grade: s.S, MAOP: s.MAOP, Tdes: s.Tdes, seam: s.E, div: s.div, CA: s.CA, dt: s.dt, cluster: s.cluster, built: !!s.built, years: s.years, feats: s.feats.map(f => ({ ...f })) },
    results: { classKm: [1, 2, 3, 4].map(k => r3(R.len[k] / 1000)), walls: R.walls, steel_t: r3(R.d.mass / 1000), maxPctSMYS: r3(R.maxPct), Tfactor: r3(R.d.T), handlingKm: r3(R.d.handling / 1000), testPressure: Object.fromEntries(Object.entries(R.tp).map(([k, v]) => [k, r3(v)])), PIR: r3(R.r), HCAkm: r3(R.hcaLen / 1000), fracture: R.fracture, nonCompliantKm: r3(R.over / 1000), minAllowedMAOP: r3(R.minLim), replacedKm: r3(s.replaced / 1000), units: R.units },
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }; },
  setState: inp => {
    if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset]();
    const map = { OD: 'OD', grade: 'grade', MAOP: 'MAOP', Tdes: 'Tdes', seam: 'seam', div: 'div', CA: 'CA', dt: 'dt', cluster: 'cluster' };
    for (const [k, id] of Object.entries(map)) if (inp[k] !== undefined) setCtl(id, inp[k]);
    if (Array.isArray(inp.feats)) { state.feats = inp.feats.filter(f => ['house', 'multi', 'school', 'road', 'station'].includes(f.t) && Number.isFinite(+f.x) && Number.isFinite(+f.y)).map(f => ({ t: f.t, x: clamp(+f.x, 0, state.L), y: clamp(+f.y, -700, 700) })); dirty = true; }
    if (inp.build) { refresh(); build(); } if (inp.years) for (let k = 0; k < Math.floor(inp.years / 5); k++) grow(); if (inp.derate) derate(); if (inp.replace) replace(); if (inp.unbuild) unbuild();
    refresh();
  },
  meta: { id: 'gas-pipeline-route', title: 'Gas Pipeline Route and Wall Thickness Designer', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PipelineRoute.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'gas-pipeline-route', state: window.PipelineRoute.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function boot() {
  try { const th = localStorage.getItem('tk_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildTasks(); bind(); state.feats = baseFeats(); renderLog(); refresh(); requestAnimationFrame(tick);
}
boot();
