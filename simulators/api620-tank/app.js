/* API 620 Tank Design Demonstrator — a large welded low-pressure storage tank (gas pressure up to 103 kPa gauge above the
   liquid) designed by membrane theory: meridional and latitudinal unit forces T1 and T2 at every level of a dome,
   torispherical or cone roof and a cylindrical sidewall under gas pressure, liquid head, vacuum, live load and the
   weight of the metal; tension thickness t = T/(S E) + CA, compression thickness from the buckling allowable, the knuckle
   in hoop compression, the compression-ring region at a dome- or cone-to-sidewall junction, uplift and anchor bolts,
   scope limits (15 psig, 121 °C, Annex R and Q temperature ranges), materials with minimum design metal temperatures,
   steel mass and a pressure sweep. Fill, pressurise, hydro-pneumatic test and a blocked-vent pump-out on a clock with a
   gas-law model of the vapour space and live utilisation of every element. Standalone, offline. Teaching model — data
   and clause references are illustrative and must be verified against the edition of API 620 in force. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const PLATES = [5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 32, 36, 40, 45, 50];
const MAT = { // Sts = lesser of 0.30 Fu and 0.60 Fy (main body rule, MPa); Tmin = lowest design metal temperature the plate may be used at [illustrative]; annex = where it is listed
  a283c: { name: 'A283M Gr C', Fy: 205, Fu: 380, Sts: 114, Tmin: -7, annex: 'body' },
  a36: { name: 'A36M', Fy: 250, Fu: 400, Sts: 120, Tmin: -7, annex: 'body' },
  a516_485: { name: 'A516M Gr 485, impact tested', Fy: 260, Fu: 485, Sts: 145, Tmin: -29, annex: 'body' },
  a516_485n: { name: 'A516M Gr 485 normalised (Annex R)', Fy: 260, Fu: 485, Sts: 145, Tmin: -45, annex: 'R' },
  a537_1: { name: 'A537M Cl 1 (Annex R)', Fy: 345, Fu: 485, Sts: 145, Tmin: -51, annex: 'R' },
  a553: { name: 'A553M 9 % nickel (Annex Q)', Fy: 585, Fu: 690, Sts: 207, Tmin: -196, annex: 'Q' },
  ss304: { name: 'A240M 304 stainless (Annex Q)', Fy: 205, Fu: 515, Sts: 123, Tmin: -196, annex: 'Q' },
  al5083: { name: '5083-O aluminium (Annex Q)', Fy: 125, Fu: 275, Sts: 75, Tmin: -196, annex: 'Q', rho: 2660, Emod: 70000 },
};
const JOINT = { full: { name: 'Fully radiographed', E: 1.0 }, spot: { name: 'Spot radiographed', E: 0.85 }, none: { name: 'Not radiographed', E: 0.70 } };
const PRODUCT = { // sets gravity, design metal temperature and a typical design gas pressure [illustrative]
  naphtha: { name: 'Light naphtha, ambient', G: 0.70, T: 40, Pg: 7 },
  blanket: { name: 'Nitrogen-blanketed process liquid', G: 1.0, T: 40, Pg: 35 },
  pentane: { name: 'Pentane blend, warm, gas pad', G: 0.63, T: 60, Pg: 50 },
  butane: { name: 'Butane, refrigerated −5 °C', G: 0.60, T: -5, Pg: 7 },
  ammonia: { name: 'Ammonia, refrigerated −33 °C', G: 0.68, T: -33, Pg: 10 },
  propane: { name: 'Propane, refrigerated −42 °C', G: 0.58, T: -42, Pg: 10 },
  ethylene: { name: 'Ethylene, refrigerated −104 °C', G: 0.57, T: -104, Pg: 15 },
  lng: { name: 'LNG, refrigerated −162 °C', G: 0.45, T: -162, Pg: 20 },
};
const ROOF = { dome: 'Dome with compression ring', tori: 'Torispherical (knuckle)', cone: 'Cone with compression ring' };
const BOLT = { M24: 353, M30: 561, M36: 817, M42: 1121, M48: 1473, M56: 2030 }; // tensile stress area mm²
const RHO = 7850, SC_RING = 103.4, SB = 105, SB_TEST = 140, PATM = 101.3, PMAX = 103.4, TMAX = 121;

/* ------------------------------------------------------------------ state */
const state = { D: 30, n: 6, hc: 2.4, H: 13.5, product: 'ammonia', G: 0.68, Tdes: -33, Pg: 10, Pe: 1.0, CA: 1.0, mat: 'a516_485n', joint: 'full', roof: 'dome', rc: 1.0, rk: 0.10, slope: 15, Lr: 1.2, S: 0, ringA: 12500, bolt: 'M42', Nb: 48, fillRate: 500, tscale: 60, run: true,
  level: 0, Pgas: 0, vent: 'open', mode: 'idle', target: 0, ptarget: 0, fluid: 'product', overrides: {}, sel: null, t: 0, failed: '', relief: 0, log: [], flags: {}, csv: '' };
const R = {};

/* ------------------------------------------------------------------ model */
const roundUp = t => PLATES.find(p => p >= t - 1e-9) ?? PLATES[PLATES.length - 1];
const sca = x => x <= 0.00667 ? 12410 * x : x <= 0.0175 ? 82.7 + 1911 * (x - 0.00667) : 103.4; // MPa, x = (t − CA)/R  [5.10.3.4 form]
function tComp(Tc, Rmm, CA) { if (!(Tc > 0)) return CA; let lo = CA + 0.01, hi = 400; if (Tc / (hi - CA) > sca((hi - CA) / Rmm)) return Infinity; for (let k = 0; k < 50; k++) { const mid = (lo + hi) / 2; (Tc / (mid - CA) > sca((mid - CA) / Rmm)) ? lo = mid : hi = mid; } return hi; }
const util = (Tt, Tc, t, CA, S, E, Rmm) => { const tc = t - CA; if (tc <= 0) return Infinity; return Math.max(Tt > 0 ? Tt / (tc * S * E) : 0, Tc > 0 ? Tc / (tc * sca(tc / Rmm)) : 0); };
function roofGeom(p, D) {
  const Rr = D / 2, roof = ROOF[p.roof] ? p.roof : 'dome', o = { roof, R: Rr };
  if (roof === 'cone') { o.theta = clamp(p.slope, 3, 45) * Math.PI / 180; o.hr = Rr * Math.tan(o.theta); o.area = Math.PI * Rr * Rr / Math.cos(o.theta); o.V = Math.PI * Rr * Rr * o.hr / 3; return o; }
  o.Rc = clamp(p.rc, 0.55, 2) * D; o.phiE = Math.asin(Rr / o.Rc);
  if (roof === 'dome') { o.hr = o.Rc * (1 - Math.cos(o.phiE)); o.area = 2 * Math.PI * o.Rc * o.hr; o.V = Math.PI * o.hr * o.hr * (3 * o.Rc - o.hr) / 3; return o; }
  o.rk = clamp(p.rk, 0.02, 0.45) * D; o.phiT = Math.asin(clamp((Rr - o.rk) / (o.Rc - o.rk), 0, 1)); o.hr = o.Rc - (o.Rc - o.rk) * Math.cos(o.phiT);
  o.areaCrown = 2 * Math.PI * o.Rc * o.Rc * (1 - Math.cos(o.phiT)); o.areaK = 2 * Math.PI * o.rk * ((Rr - o.rk) * (Math.PI / 2 - o.phiT) + o.rk * Math.cos(o.phiT)); o.area = o.areaCrown + o.areaK; o.V = Math.PI * o.hr * (3 * Rr * Rr + o.hr * o.hr) / 6; return o;
}
function design(p) {
  const D = clamp(p.D, 2, 120), Rr = D / 2, n = clamp(Math.round(p.n), 1, 24), hc = clamp(p.hc, 0.5, 4), Hs = n * hc, H = clamp(p.H, 0, Hs), G = clamp(p.G, 0.1, 3), CA = clamp(p.CA, 0, 10), m = MAT[p.mat] || MAT.a516_485n, E = (JOINT[p.joint] || JOINT.full).E, Pg = clamp(p.Pg, 0, 200), Pe = clamp(p.Pe, 0, 20), Tdes = p.Tdes, Lr = Math.max(p.Lr, 0), S = Math.max(p.S, 0), ov = p.overrides || {}, rho = m.rho || RHO;
  const Sts = m.Sts, At = Math.PI * Rr * Rr, tmin = 5, wkg = t => t / 1000 * rho, wkPa = t => t / 1000 * rho * 9.81 / 1000; // plate weight per m² as kPa
  const g = roofGeom(p, D), roof = g.roof, Rmm = Rr * 1000;
  // scope and temperature
  const scope = Pg > PMAX ? 'asme' : Tdes > TMAX ? 'hot' : 'ok', annex = Tdes < -51 ? 'Q' : Tdes < 4 ? 'R' : 'body';
  const matTempOk = Tdes >= m.Tmin, matAnnexOk = annex === 'Q' ? m.annex === 'Q' : annex === 'R' ? m.annex !== 'body' || m.Tmin <= Tdes : true;
  // roof plate (two passes for self-weight)
  let tr = 5, rf = {};
  for (let k = 0; k < 2; k++) { const w = wkPa(tr), Lv = Math.max(Lr, S);
    if (roof === 'cone') { const sn = Math.sin(g.theta), cs = Math.cos(g.theta), R2 = Rr / sn; rf = { T1p: Pg * Rr / (2 * sn), T2p: Pg * Rr / sn, T1v: -(Pe + Lv + w * cs) * Rr / (2 * sn), T2v: -(Pe + Lv + w * cs) * Rr / sn, R2 }; }
    else { const R2 = g.Rc; rf = { T1p: (Pg - w) * R2 / 2, T2p: (Pg - w) * R2 / 2, T1v: -(Pe + Lv + w) * R2 / 2, T2v: -(Pe + Lv + w) * R2 / 2, R2 }; }
    rf.Tt = Math.max(rf.T1p, rf.T2p, 0); rf.Tc = Math.max(-rf.T1v, -rf.T2v, 0); rf.tTen = rf.Tt / (Sts * E) + CA; rf.tCmp = tComp(rf.Tc, rf.R2 * 1000, CA); rf.treq = Math.max(rf.tTen, rf.tCmp, tmin); rf.tauto = roundUp(rf.treq); tr = Number.isFinite(ov.roof) ? ov.roof : rf.tauto; }
  rf.t = tr; rf.over = Number.isFinite(ov.roof); rf.gov = rf.treq === tmin ? 'minimum' : rf.tTen >= rf.tCmp ? 'tension' : 'compression'; rf.util = util(rf.Tt, rf.Tc, tr, CA, Sts, E, rf.R2 * 1000); rf.ok = rf.util <= 1 + 1e-9 && rf.treq <= 50 + 1e-9;
  const Wroof = g.area * wkg(tr) * 9.81 / 1000; // kN
  // knuckle (torispherical)
  let kn = null, tk = 0, Wk = 0;
  if (roof === 'tori') { const T1j = (Rr / 2) * (Pg - Wroof / At), T2j = Rr * (Pg - T1j / g.rk), T1v = (Rr / 2) * (-Pe - (Wroof + Math.max(Lr, S) * At) / At), T2v = Rr * (-Pe - T1v / g.rk);
    kn = { T1j, T2j, T1v, T2v, Tt: Math.max(Pg * g.Rc / 2, T1j, T2j, T2v, 0), Tc: Math.max(-T2j, -T1v, -T2v, 0), rkOk: g.rk >= 0.06 * g.Rc - 1e-9 };
    kn.tTen = kn.Tt / (Sts * E) + CA; kn.tCmp = tComp(kn.Tc, Rmm, CA); kn.treq = Math.max(kn.tTen, kn.tCmp, tmin); kn.tauto = roundUp(kn.treq); tk = Number.isFinite(ov.knuckle) ? ov.knuckle : kn.tauto; kn.t = tk; kn.over = Number.isFinite(ov.knuckle); kn.gov = kn.treq === tmin ? 'minimum' : kn.tTen >= kn.tCmp ? 'tension' : 'compression'; kn.util = util(kn.Tt, kn.Tc, tk, CA, Sts, E, Rmm); kn.ok = kn.util <= 1 + 1e-9 && kn.rkOk && kn.treq <= 50 + 1e-9; Wk = g.areaK * wkg(tk) * 9.81 / 1000; }
  // compression ring region (dome or cone)
  let ring = null, Wring = 0, ringMass = 0;
  // sidewall courses — two passes so the weight above each level uses ordered plate
  let courses = [], Wshell = 0;
  for (let pass = 0; pass < 2; pass++) { const prev = courses; courses = []; let Wab = Wroof + Wk + Wring; const W_i = i => prev[i] ? Math.PI * D * hc * wkg(prev[i].t) * 9.81 / 1000 : 0;
    for (let i = n - 1; i >= 0; i--) { const z = i * hc, head = 9.81 * G * Math.max(H - z, 0), T2 = (Pg + head) * Rr, T1 = (Rr / 2) * (Pg - (Wab) / At), T2v = -Pe * Rr, T1v = (Rr / 2) * (-Pe - (Wab + Math.max(Lr, S) * At) / At);
      const Tt = Math.max(T2, T1, 0), Tc = Math.max(-T1, -T2v, -T1v, 0), tTen = Tt / (Sts * E) + CA, tCmp = tComp(Tc, Rmm, CA), treq = Math.max(tTen, tCmp, tmin);
      courses[i] = { i, z, head, T1, T2, T1v, T2v, Tt, Tc, tTen, tCmp, treq, tauto: roundUp(treq), gov: treq === tmin ? 'minimum' : tTen >= tCmp ? 'tension' : 'compression', t: Number.isFinite(ov[i]) ? ov[i] : roundUp(treq), over: Number.isFinite(ov[i]) };
      Wab += W_i(i); }
    for (let i = n - 2; i >= 0; i--) if (!courses[i].over && !courses[i + 1].over) courses[i].tauto = courses[i].t = Math.max(courses[i].t, courses[i + 1].t);
  }
  let allOk = true; Wshell = 0;
  for (const c of courses) { c.util = util(c.Tt, c.Tc, c.t, CA, Sts, E, Rmm); c.okOrder = c.i === n - 1 || c.t >= courses[c.i + 1].t; c.ok = c.util <= 1 + 1e-9 && c.treq <= 50 + 1e-9 && c.okOrder; if (!c.ok) allOk = false; c.mass = Math.PI * D * hc * wkg(c.t); Wshell += c.mass * 9.81 / 1000; c.sTen = c.Tt / (c.t - CA); c.sCmp = c.Tc / (c.t - CA); }
  const tsTop = courses[n - 1].t;
  if (roof !== 'tori') { const F = Pg * At - Wroof, ang = roof === 'cone' ? g.theta : g.phiE, T1r = F / (2 * Math.PI * Rr * Math.sin(ang)), q = T1r * Math.cos(ang), Q = q * Rr, Ac = Math.max(Q, 0) * 1000 / SC_RING, R2 = roof === 'cone' ? Rr / Math.sin(g.theta) : g.Rc;
    const wc = 0.6 * Math.sqrt(R2 * 1000 * Math.max(tr - CA, 0.01)), wh = 0.6 * Math.sqrt(Rmm * Math.max(tsTop - CA, 0.01)), Apart = wc * (tr - CA) + wh * (tsTop - CA), Aneed = Math.max(0, Ac - Apart), provided = Math.max(p.ringA, 0);
    ring = { T1r, q, Q, Ac, wc, wh, Apart, Aneed, provided, ok: provided >= Aneed - 1e-6, util: Aneed > 0 ? Q * 1000 / ((Apart + provided) * SC_RING) : Q * 1000 / (Apart * SC_RING) }; ringMass = provided / 1e6 * Math.PI * D * rho; Wring = ringMass * 9.81 / 1000; }
  // bottom, masses
  const tb = 6 + CA, massBottom = Math.PI * (Rr + 0.05) ** 2 * wkg(tb), massShell = courses.reduce((a, c) => a + c.mass, 0), massRoof = g.area * wkg(tr) + (kn ? g.areaK * wkg(tk) : 0), massTotal = massShell + massRoof + massBottom + ringMass;
  const Wtot = Wshell + Wroof + Wk + Wring; // metal above the bottom joint, kN
  // uplift and anchor bolts (empty tank at design pressure; test at 1.25 Pg)
  const U = Pg * At - Wtot, Utest = 1.25 * Pg * At - Wtot, As = BOLT[p.bolt] || BOLT.M36, Fb = SB * As / 1000, Fbt = SB_TEST * As / 1000, Nb = clamp(Math.round(p.Nb), 0, 400), spacing = Nb ? Math.PI * D / Nb : Infinity;
  const anchorsNeeded = U > 0 || Utest > 0, Nreq = anchorsNeeded ? Math.max(4, Math.ceil(Math.max(U / Fb, Utest / Fbt, Math.PI * D / 3))) : 0, perBolt = Nb ? Math.max(U, 0) / Nb : Infinity, perBoltTest = Nb ? Math.max(Utest, 0) / Nb : Infinity;
  const anchorOk = !anchorsNeeded || (Nb > 0 && perBolt <= Fb + 1e-9 && perBoltTest <= Fbt + 1e-9 && spacing <= 3 + 1e-9);
  const Vwork = At * H, Vgas0 = At * (Hs - H) + g.V;
  return { D, R: Rr, n, hc, Hs, H, G, CA, m, E, Sts, Pg, Pe, Tdes, Lr, S, tmin, g, roof, rf, tr, kn, tk, ring, courses, allOk, scope, annex, matTempOk, matAnnexOk, tb, massShell, massRoof, massBottom, ringMass, massTotal, Wshell, Wroof, Wtot, U, Utest, As, Fb, Fbt, Nb, spacing, anchorsNeeded, Nreq, perBolt, perBoltTest, anchorOk, Vwork, Vgas0, At, kgm3: massTotal / Math.max(Vwork, 1e-6),
    designOk: allOk && rf.ok && (!kn || kn.ok) && (!ring || ring.ok) && anchorOk && scope === 'ok' && matTempOk && matAnnexOk };
}
function sweep(s) { const pts = []; for (let Pg = 0; Pg <= 110; Pg += 2) { const r = design(Object.assign({}, s, { Pg, overrides: {} })); pts.push({ Pg, mass: r.massTotal, shell: r.massShell, Nreq: r.Nreq, U: r.U }); } return pts; }
/* live utilisation of every element at the current level and gas pressure (gas pressure may be negative = vacuum) */
function live(r) {
  const s = state, Gf = s.fluid === 'water' ? 1 : r.G, kf = s.fluid === 'water' ? 1.25 : 1, P = s.Pgas, Rr = r.R, At = r.At, CA = r.CA, out = { courses: [], max: 0, where: '', kf };
  const wgt = r.Wroof + (r.kn ? r.g.areaK * (r.tk / 1000 * (r.m.rho || RHO)) * 9.81 / 1000 : 0) + (r.ring ? r.ringMass * 9.81 / 1000 : 0);
  let Wab = wgt; const uOf = (T1, T2, t, R2mm) => util(Math.max(T1, T2, 0), Math.max(-T1, -T2, 0), t, CA, r.Sts, r.E, R2mm) / kf;
  const R2 = r.roof === 'cone' ? Rr / Math.sin(r.g.theta) : r.g.Rc, w = r.tr / 1000 * (r.m.rho || RHO) * 9.81 / 1000;
  const rT1 = r.roof === 'cone' ? (P - w * Math.cos(r.g.theta)) * Rr / (2 * Math.sin(r.g.theta)) : (P - w) * R2 / 2, rT2 = r.roof === 'cone' ? (P - w * Math.cos(r.g.theta)) * Rr / Math.sin(r.g.theta) : rT1; out.roof = uOf(rT1, rT2, r.tr, R2 * 1000); out.roofT = [rT1, rT2];
  if (r.kn) { const T1j = (Rr / 2) * (P - r.Wroof / At), T2j = Rr * (P - T1j / r.g.rk); out.knuckle = uOf(Math.max(T1j, P * r.g.Rc / 2), T2j, r.tk, Rr * 1000); out.knT = [T1j, T2j]; }
  if (r.ring) { const F = P * At - r.Wroof, ang = r.roof === 'cone' ? r.g.theta : r.g.phiE, Q = F / (2 * Math.PI * Rr * Math.sin(ang)) * Math.cos(ang) * Rr; out.ring = Q > 0 ? Q * 1000 / ((r.ring.Apart + r.ring.provided) * SC_RING) / kf : 0; out.ringQ = Q; }
  for (let i = r.n - 1; i >= 0; i--) { const c = r.courses[i], head = 9.81 * Gf * Math.max(s.level - c.z, 0), T2 = (P + head) * Rr, T1 = (Rr / 2) * (P - Wab / At); out.courses[i] = { u: uOf(T1, T2, c.t, Rr * 1000), T1, T2 }; Wab += c.mass * 9.81 / 1000; }
  const cands = [['roof', out.roof], ['knuckle', out.knuckle ?? 0], ['ring', out.ring ?? 0], ...out.courses.map((c, i) => ['course ' + (i + 1), c.u])];
  for (const [k, v] of cands) if (v > out.max) { out.max = v; out.where = k; }
  out.uplift = P * At - r.Wtot - (s.level > 0 ? 0 : 0); out.upliftU = r.Nb ? Math.max(out.uplift, 0) / (r.Nb * r.Fbt) : (out.uplift > 0 ? Infinity : 0);
  return out;
}

/* ------------------------------------------------------------------ dynamics: vapour space, vent, fill, drain, pressurise */
function step(dt) {
  const s = state, r = R; s.t += dt * s.tscale; if (!s.run || s.failed) return;
  const ds = dt * s.tscale, A = r.At, Vg = () => A * (r.Hs - s.level) + r.g.V;
  let V0 = Vg(), dz = s.fillRate * ds / 3600 / A;
  if (s.mode === 'fill') { s.level += dz; if (s.target !== undefined && s.level >= s.target) { s.level = s.target; if (s.ptarget > s.Pgas) { s.mode = 'pressurise'; if (s.testPending) { s.vent = 'gagged'; s.testPending = false; } } else s.mode = 'idle'; } if (s.level >= r.Hs) { s.level = r.Hs; s.mode = 'idle'; } }
  else if (s.mode === 'drain') { s.level -= dz; if (s.level <= 0) { s.level = 0; s.mode = 'idle'; } }
  // isothermal vapour space: absolute pressure × volume constant while the level moves
  const V1 = Vg(); if (V1 > 0.01 && Math.abs(V1 - V0) > 1e-9) s.Pgas = (s.Pgas + PATM) * V0 / V1 - PATM;
  if (s.mode === 'pressurise') { s.Pgas += 10 / 60 * ds; if (s.Pgas >= s.ptarget) { s.Pgas = s.ptarget; s.mode = 'idle'; if (s.fluid === 'water' && s.ptarget >= 1.25 * r.Pg - 1e-6 && s.level >= 0.98 * Math.min(r.H, 1.25 * r.G * r.H)) s.flags.test = true; } }
  s.relief = 0; if (s.vent === 'open') { if (s.Pgas > r.Pg) { s.Pgas = r.Pg; s.relief = 1; } if (s.Pgas < -r.Pe) { s.Pgas = -r.Pe; s.relief = -1; } }
  const L = live(r); if (L.max > 1.5) { s.failed = (L.where === 'ring' ? 'compression ring buckled' : s.Pgas < 0 ? `${L.where} buckled under vacuum` : `${L.where} yielded under pressure`); s.mode = 'idle'; s.flags.failed = true; }
  if (L.upliftU > 1.5 && r.Nb > 0) { s.failed = 'anchor bolts failed — shell lifted'; s.mode = 'idle'; s.flags.failed = true; } else if (r.Nb === 0 && L.uplift > 0 && L.uplift > 0.3 * r.Wtot) { s.failed = 'unanchored shell lifted off the bottom'; s.mode = 'idle'; s.flags.failed = true; }
  if (s.Pgas > 1.05 * r.Pg && s.vent === 'blocked') s.flags.overpressure = true; if (s.Pgas < -1.05 * r.Pe && s.vent === 'blocked') s.flags.vacuum = true;
}

/* ------------------------------------------------------------------ rig drawing */
const NS = 'http://www.w3.org/2000/svg', E_ = {};
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); if (parent) parent.appendChild(e); return e; }
function buildRig() {
  const host = $('#rig'); host.innerHTML = ''; const svg = el('svg', { viewBox: '0 0 960 540', role: 'img', 'aria-label': 'Tank elevation' }, host); E_.svg = svg;
  for (const k of ['found', 'gas', 'fluid', 'shell', 'roof', 'extras', 'gauge', 'dims', 'txt']) E_[k] = el('g', {}, svg);
  svg.addEventListener('click', e => { const c = e.target && e.target.closest ? e.target.closest('[data-sel]') : null; if (c) { const v = c.dataset.sel; state.sel = /^\d+$/.test(v) ? +v : v; drawRig(); updateReadouts(); } });
}
function roofPath(r, sc, cx, yTop) { // right half outline from apex to the junction, mirrored
  const g = r.g, pts = [];
  if (r.roof === 'cone') pts.push([0, g.hr], [r.R, 0]);
  else if (r.roof === 'dome') { for (let k = 0; k <= 24; k++) { const ph = g.phiE * k / 24; pts.push([g.Rc * Math.sin(ph), g.hr - g.Rc * (1 - Math.cos(ph))]); } }
  else { for (let k = 0; k <= 16; k++) { const ph = g.phiT * k / 16; pts.push([g.Rc * Math.sin(ph), g.hr - g.Rc * (1 - Math.cos(ph))]); } for (let k = 1; k <= 12; k++) { const ph = g.phiT + (Math.PI / 2 - g.phiT) * k / 12; pts.push([(r.R - g.rk) + g.rk * Math.sin(ph), g.rk * Math.cos(ph)]); } }
  const P = pts.map(([x, y]) => [cx + x * sc, yTop - y * sc]), L = pts.map(([x, y]) => [cx - x * sc, yTop - y * sc]).reverse();
  return { d: 'M' + L.concat(P).map(q => q[0].toFixed(1) + ' ' + q[1].toFixed(1)).join(' L'), tangentY: r.roof === 'tori' ? yTop - g.rk * Math.cos(g.phiT) * sc : null, tangentX: r.roof === 'tori' ? (r.R - g.rk) + g.rk * Math.sin(g.phiT) : null };
}
function drawRig() {
  const s = state, r = R; if (!E_.svg || !r.courses) return;
  for (const k of ['found', 'gas', 'fluid', 'shell', 'roof', 'extras', 'gauge', 'dims', 'txt']) E_[k].innerHTML = '';
  const totalH = r.Hs + r.g.hr, sc = Math.min(500 / r.D, 330 / Math.max(totalH, 1)), cx = 470, yb = 450, xL = cx - r.R * sc, xR = cx + r.R * sc, yTop = yb - r.Hs * sc, wall = t => Math.min(4 + t * 0.5, 26), L = live(r);
  el('rect', { x: xL - 30, y: yb, width: xR - xL + 60, height: 26, class: 'pad' }, E_.found); el('rect', { x: xL - 8, y: yb - 4, width: xR - xL + 16, height: 4, class: 'bottomp' }, E_.found);
  // gas space tint by pressure
  const rp = roofPath(r, sc, cx, yTop), pAbs = clamp(s.Pgas / Math.max(r.Pg, 1), -1, 1.6), gasCls = s.Pgas < -0.02 ? 'gas vac' : 'gas';
  el('path', { d: rp.d + ` L${xR} ${yb - 4} L${xL} ${yb - 4} Z`, class: gasCls, style: `opacity:${(0.08 + 0.35 * Math.abs(pAbs)).toFixed(2)}` }, E_.gas);
  if (s.level > 0.005) { const yl = yb - 4 - s.level * sc; el('rect', { x: xL + 2, y: yl, width: xR - xL - 4, height: yb - 4 - yl, class: 'liquid ' + (s.fluid === 'water' ? 'water' : 'product') }, E_.fluid); el('text', { x: xL + 14, y: Math.min(yl + 15, yb - 10), class: 'val' }, E_.fluid).textContent = `${s.fluid === 'water' ? 'water' : 'product G ' + fmt(r.G, 2)} · ${fmt(s.level, 2)} m`; }
  // sidewall courses
  r.courses.forEach((c, i) => { const y1 = yb - 4 - (c.z + r.hc) * sc, h = r.hc * sc, w = wall(c.t), sel = s.sel === i, lu = L.courses[i].u, cls = 'course' + (c.ok ? '' : ' bad') + (sel ? ' sel' : '') + (c.over ? ' over' : '') + (lu > 1 ? ' hot' : '');
    el('rect', { x: xL - w / 2, y: y1, width: w, height: h, class: cls, 'data-sel': i }, E_.shell); el('rect', { x: xR - w / 2, y: y1, width: w, height: h, class: cls, 'data-sel': i }, E_.shell); el('rect', { x: xL + w / 2, y: y1, width: xR - xL - w, height: h, class: 'coursehit', 'data-sel': i }, E_.shell);
    el('line', { x1: xL - 12, y1: y1 + h, x2: xR + 12, y2: y1 + h, class: 'weld' }, E_.shell);
    el('text', { x: xL - w / 2 - 8, y: y1 + h / 2 + 4, class: 'ctag' + (c.ok ? '' : ' bad') + (sel ? ' sel' : ''), 'text-anchor': 'end' }, E_.txt).textContent = `C${i + 1} · ${c.t} mm${c.over ? ' *' : ''}`;
    if (h > 14) el('text', { x: xR + w / 2 + 8, y: y1 + h / 2 + 4, class: 'cval' + (lu > 1 ? ' bad' : '') }, E_.txt).textContent = s.mode !== 'idle' || s.level > 0.01 || Math.abs(s.Pgas) > 0.05 ? `${fmt(lu * 100, 0)} %` : `${c.gov} · req ${fmt(c.treq, 1)}`; });
  // roof
  const rw = wall(r.tr), rsel = s.sel === 'roof', ksel = s.sel === 'knuckle';
  el('path', { d: rp.d, class: 'roofp' + (r.rf.ok ? '' : ' bad') + (rsel ? ' sel' : '') + (L.roof > 1 ? ' hot' : ''), 'stroke-width': rw, 'data-sel': 'roof' }, E_.roof);
  if (r.kn) { const kx = rp.tangentX * sc, ky = rp.tangentY; const dk = (side) => { const pts = []; for (let k = 0; k <= 12; k++) { const ph = r.g.phiT + (Math.PI / 2 - r.g.phiT) * k / 12; pts.push([cx + side * ((r.R - r.g.rk) + r.g.rk * Math.sin(ph)) * sc, yTop - r.g.rk * Math.cos(ph) * sc]); } return 'M' + pts.map(q => q[0].toFixed(1) + ' ' + q[1].toFixed(1)).join(' L'); };
    for (const side of [-1, 1]) el('path', { d: dk(side), class: 'knuckle' + (r.kn.ok ? '' : ' bad') + (ksel ? ' sel' : '') + (L.knuckle > 1 ? ' hot' : ''), 'stroke-width': wall(r.tk) + 2, 'data-sel': 'knuckle' }, E_.roof);
    void kx; el('text', { x: cx, y: ky + 16, class: 'lbl' + (r.kn.ok ? '' : ' bad'), 'text-anchor': 'middle' }, E_.txt).textContent = `knuckle rk ${fmt(r.g.rk, 2)} m · ${r.tk} mm · T2 ${fmt(r.kn.T2j, 0)} kN/m`; }
  if (r.ring) { const rsz = 6 + Math.min(14, Math.sqrt(r.ring.provided) / 8), cls = 'ring' + (r.ring.ok ? '' : ' bad') + (s.sel === 'ring' ? ' sel' : ''); el('rect', { x: xL - rsz, y: yTop - rsz / 2, width: rsz * 2, height: rsz, class: cls, 'data-sel': 'ring' }, E_.roof); el('rect', { x: xR - rsz, y: yTop - rsz / 2, width: rsz * 2, height: rsz, class: cls, 'data-sel': 'ring' }, E_.roof); el('text', { x: xL + 14, y: yTop + 32, class: 'lbl' + (r.ring.ok ? '' : ' bad') }, E_.txt).textContent = `compression ring · Q ${fmt(r.ring.Q, 0)} kN · need ${fmt(r.ring.Aneed, 0)} mm²`; }
  const apexY = yTop - r.g.hr * sc; el('text', { x: cx, y: apexY - 12, class: 'lbl' + (r.rf.ok ? '' : ' bad'), 'text-anchor': 'middle' }, E_.txt).textContent = `${ROOF[r.roof]} · plate ${r.tr} mm${r.roof === 'cone' ? ' · slope ' + fmt(s.slope, 0) + '°' : ' · Rc ' + fmt(r.g.Rc, 1) + ' m'}${r.rf.ok ? '' : ' ✗'}`;
  // PV valve on the roof
  const vx = cx - r.R * sc * 0.7, vy = yTop - (r.roof === 'cone' ? (r.R - r.R * 0.7) * Math.tan(r.g.theta) : (r.g.hr - r.g.Rc * (1 - Math.cos(Math.asin(Math.min(1, r.R * 0.7 / r.g.Rc)))))) * sc;
  el('rect', { x: vx - 7, y: vy - 22, width: 14, height: 22, class: 'pv' + (s.vent === 'blocked' ? ' blocked' : s.vent === 'gagged' ? ' gagged' : '') }, E_.extras); el('rect', { x: vx - 12, y: vy - 30 - (s.relief > 0 ? 6 : 0), width: 24, height: 8, class: 'pvcap' + (s.relief ? ' lift' : '') }, E_.extras); el('text', { x: vx + 12, y: vy + 16, class: 'lbl' }, E_.txt).textContent = s.vent === 'blocked' ? 'VENT BLOCKED' : s.vent === 'gagged' ? 'vent gagged for test' : s.relief > 0 ? `PV valve relieving at ${fmt(r.Pg, 1)} kPa` : s.relief < 0 ? `vacuum pallet open at −${fmt(r.Pe, 1)} kPa` : `PV valve · +${fmt(r.Pg, 1)} / −${fmt(r.Pe, 1)} kPa`;
  // anchors
  if (r.Nb > 0) { const show = Math.min(r.Nb, 40); for (let k = 0; k < show; k++) { const ang = Math.PI * (k + 0.5) / show, x = cx - r.R * sc * Math.cos(ang); el('line', { x1: x, y1: yb - 10, x2: x, y2: yb + 22, class: 'bolt' + (r.anchorOk ? '' : ' bad') }, E_.extras); } for (const x of [xL, xR]) el('path', { d: `M${x - 11} ${yb} l11 -20 l11 20 z`, class: 'chair' + (r.anchorOk ? '' : ' bad') }, E_.extras); }
  el('text', { x: cx, y: yb + 42, class: 'lbl' + (r.anchorsNeeded && !r.anchorOk ? ' bad' : ''), 'text-anchor': 'middle' }, E_.txt).textContent = r.anchorsNeeded ? `${r.Nb} × ${s.bolt} anchors at ${fmt(r.spacing, 2)} m · uplift ${fmt(r.U, 0)} kN empty at Pg, ${fmt(r.Utest, 0)} kN at test${r.anchorOk ? '' : ' — INADEQUATE, need ' + r.Nreq}` : `no uplift at Pg — anchors not required (${r.Nb} fitted)`;
  // design level, dimensions
  const yH = yb - 4 - r.H * sc; el('line', { x1: xL, y1: yH, x2: xR, y2: yH, class: 'hline' }, E_.dims); el('text', { x: xR - 12, y: yH + 14, class: 'lbl', 'text-anchor': 'end' }, E_.txt).textContent = `design liquid level H ${fmt(r.H, 2)} m`;
  const yd = Math.max(apexY - 34, 16); el('line', { x1: xL, y1: yd, x2: xR, y2: yd, class: 'dim' }, E_.dims); el('line', { x1: xL, y1: yd - 6, x2: xL, y2: yd + 6, class: 'dim' }, E_.dims); el('line', { x1: xR, y1: yd - 6, x2: xR, y2: yd + 6, class: 'dim' }, E_.dims); el('text', { x: cx, y: yd - 5, class: 'dimt', 'text-anchor': 'middle' }, E_.txt).textContent = `D ${fmt(r.D, 1)} m`;
  const xd = xL - 92; el('line', { x1: xd, y1: yTop, x2: xd, y2: yb - 4, class: 'dim' }, E_.dims); el('line', { x1: xd - 6, y1: yTop, x2: xd + 6, y2: yTop, class: 'dim' }, E_.dims); el('line', { x1: xd - 6, y1: yb - 4, x2: xd + 6, y2: yb - 4, class: 'dim' }, E_.dims); el('text', { x: xd - 8, y: (yTop + yb) / 2, class: 'dimt', 'text-anchor': 'middle', transform: `rotate(-90 ${xd - 8} ${(yTop + yb) / 2})` }, E_.txt).textContent = `${fmt(r.Hs, 1)} m · ${r.n} × ${fmt(r.hc, 2)} m`;
  // pressure gauge at top left
  const gx = 90, gy = 90, gr = 52, pmin = -10, pmax = 120, ang = p => Math.PI * (1 + (clamp(p, pmin, pmax) - pmin) / (pmax - pmin)), arc = (p0, p1, cls) => { const a0 = ang(p0), a1 = ang(p1); el('path', { d: `M${gx + gr * 0.86 * Math.cos(a0)} ${gy + gr * 0.86 * Math.sin(a0)} A${gr * 0.86} ${gr * 0.86} 0 ${a1 - a0 > Math.PI ? 1 : 0} 1 ${gx + gr * 0.86 * Math.cos(a1)} ${gy + gr * 0.86 * Math.sin(a1)}`, class: cls }, E_.gauge); };
  el('circle', { cx: gx, cy: gy, r: gr, class: 'gauge-face' }, E_.gauge); arc(pmin, 0, 'garc vac'); arc(0, r.Pg, 'garc ok'); arc(r.Pg, Math.min(PMAX, pmax), 'garc warn'); arc(PMAX, pmax, 'garc bad');
  const na = ang(s.Pgas); el('line', { x1: gx, y1: gy, x2: gx + gr * 0.8 * Math.cos(na), y2: gy + gr * 0.8 * Math.sin(na), class: 'gauge-needle' }, E_.gauge); el('circle', { cx: gx, cy: gy, r: 4, class: 'pnow' }, E_.gauge);
  el('text', { x: gx, y: gy + 22, class: 'gauge-val' }, E_.txt).textContent = `${s.Pgas >= 0 ? '+' : ''}${fmt(s.Pgas, 1)} kPa`; el('text', { x: gx, y: gy + 36, class: 'gauge-name' }, E_.txt).textContent = 'vapour space'; el('text', { x: gx, y: gy + gr + 16, class: 'lbl', 'text-anchor': 'middle' }, E_.txt).textContent = `set +${fmt(r.Pg, 1)} · limit 103.4`;
  if (s.failed) el('text', { x: cx, y: yb + 58, class: 'lbl bad', 'text-anchor': 'middle' }, E_.txt).textContent = 'FAILED: ' + s.failed;
}

/* ------------------------------------------------------------------ charts */
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim() || '#888';
function frame(ctx, W, H, pad) { ctx.clearRect(0, 0, W, H); ctx.fillStyle = css('--panel'); ctx.fillRect(0, 0, W, H); ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.strokeRect(pad.l, pad.t, W - pad.l - pad.r, H - pad.t - pad.b); }
function meridian(r) { // samples of T1, T2 (design pressure case and vacuum case) along the meridian from the apex to the bottom
  const g = r.g, Rr = r.R, At = r.At, pts = [], w = r.tr / 1000 * (r.m.rho || RHO) * 9.81 / 1000, Lv = Math.max(r.Lr, r.S); let sArc = 0, marks = [];
  const push = (label, T1, T2, T1v, T2v, ds) => { sArc += ds; pts.push({ s: sArc, T1, T2, T1v, T2v, label }); };
  if (r.roof === 'cone') { const sn = Math.sin(g.theta), cs = Math.cos(g.theta), Lc = Rr / cs; for (let k = 0; k <= 20; k++) { const rr = Rr * k / 20, Wc = Math.PI * rr * rr / cs * w; const T1 = rr > 0 ? (r.Pg * Math.PI * rr * rr - Wc) / (2 * Math.PI * rr * sn) : r.Pg * 1e-6, T2 = (r.Pg - w * cs) * rr / sn, T1v = rr > 0 ? -((r.Pe + Lv) * Math.PI * rr * rr + Wc) / (2 * Math.PI * rr * sn) : 0, T2v = -(r.Pe + Lv + w * cs) * rr / sn; push('cone', T1, T2, T1v, T2v, k ? Lc / 20 : 0); } }
  else { const phiEnd = r.roof === 'dome' ? g.phiE : g.phiT; for (let k = 0; k <= 20; k++) { const ph = phiEnd * k / 20, rr = g.Rc * Math.sin(ph), Wc = 2 * Math.PI * g.Rc * g.Rc * (1 - Math.cos(ph)) * w; const T1 = rr > 0 ? (r.Pg * Math.PI * rr * rr - Wc) / (2 * Math.PI * rr * Math.sin(ph)) : (r.Pg - w) * g.Rc / 2, T2 = g.Rc * (r.Pg - w * Math.cos(ph)) - T1, T1v = rr > 0 ? -((r.Pe + Lv) * Math.PI * rr * rr + Wc) / (2 * Math.PI * rr * Math.sin(ph)) : -(r.Pe + Lv + w) * g.Rc / 2, T2v = g.Rc * (-r.Pe - Lv - w * Math.cos(ph)) - T1v; push('crown', T1, T2, T1v, T2v, k ? g.Rc * phiEnd / 20 : 0); }
    if (r.roof === 'tori') { marks.push({ s: sArc, name: 'tangent line' }); const wk = r.tk / 1000 * (r.m.rho || RHO) * 9.81 / 1000; for (let k = 1; k <= 12; k++) { const ph = g.phiT + (Math.PI / 2 - g.phiT) * k / 12, rr = (Rr - g.rk) + g.rk * Math.sin(ph), R2 = rr / Math.sin(ph), Wc = g.areaCrown * w + 2 * Math.PI * g.rk * ((Rr - g.rk) * (ph - g.phiT) + g.rk * (Math.cos(g.phiT) - Math.cos(ph))) * wk; const T1 = (r.Pg * Math.PI * rr * rr - Wc) / (2 * Math.PI * rr * Math.sin(ph)), T2 = R2 * (r.Pg - T1 / g.rk), T1v = -((r.Pe + Lv) * Math.PI * rr * rr + Wc) / (2 * Math.PI * rr * Math.sin(ph)), T2v = R2 * (-r.Pe - T1v / g.rk); push('knuckle', T1, T2, T1v, T2v, g.rk * (Math.PI / 2 - g.phiT) / 12); } } }
  marks.push({ s: sArc, name: r.roof === 'tori' ? 'junction' : 'compression ring' });
  let Wab = r.Wroof + (r.kn ? g.areaK * (r.tk / 1000 * (r.m.rho || RHO)) * 9.81 / 1000 : 0) + (r.ring ? r.ringMass * 9.81 / 1000 : 0);
  for (let i = r.n - 1; i >= 0; i--) { const c = r.courses[i]; for (const f of [1, 0]) { const z = c.z + f * r.hc, head = 9.81 * r.G * Math.max(r.H - z, 0), Wz = Wab + c.mass * 9.81 / 1000 * (1 - f); push('course ' + (i + 1), (Rr / 2) * (r.Pg - Wz / At), (r.Pg + head) * Rr, (Rr / 2) * (-r.Pe - (Wz + Lv * At) / At), -r.Pe * Rr, f === 1 ? 0 : r.hc); } Wab += c.mass * 9.81 / 1000; }
  return { pts, marks, total: sArc };
}
function drawForces() {
  const cv = $('#chart'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 64, r: 16, t: 14, b: 32 }, r = R, M = meridian(r); frame(ctx, W, H, pad);
  const vals = M.pts.flatMap(p => [p.T1, p.T2, p.T1v, p.T2v]), tmax = Math.max(10, ...vals.map(Math.abs)) * 1.1, x = s => pad.l + (W - pad.l - pad.r) * s / M.total, y = T => pad.t + (H - pad.t - pad.b) * (1 - (T + tmax) / (2 * tmax));
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillStyle = css('--alarm'); ctx.globalAlpha = 0.07; ctx.fillRect(pad.l, y(0), W - pad.l - pad.r, H - pad.b - y(0)); ctx.globalAlpha = 1;
  const stepT = tmax > 2000 ? 1000 : tmax > 800 ? 500 : tmax > 300 ? 200 : tmax > 120 ? 50 : tmax > 40 ? 20 : 10; ctx.fillStyle = css('--muted'); ctx.textAlign = 'right';
  for (let T = -Math.floor(tmax / stepT) * stepT; T <= tmax; T += stepT) { ctx.strokeStyle = T === 0 ? css('--ink') : css('--grid'); ctx.beginPath(); ctx.moveTo(pad.l, y(T)); ctx.lineTo(W - pad.r, y(T)); ctx.stroke(); ctx.fillText(T + '', pad.l - 6, y(T) + 4); }
  ctx.textAlign = 'center'; ctx.fillText('unit force kN/m along the meridian · apex → roof edge → down the sidewall to the bottom · shaded = compression', (pad.l + W - pad.r) / 2, H - 4);
  for (const m of M.marks) { ctx.strokeStyle = css('--brass'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(x(m.s), pad.t); ctx.lineTo(x(m.s), H - pad.b); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = css('--brass'); ctx.fillText(m.name, x(m.s), pad.t + 10); }
  const line = (key, col, wd, dash) => { ctx.strokeStyle = col; ctx.lineWidth = wd; ctx.setLineDash(dash || []); ctx.beginPath(); M.pts.forEach((p, i) => { const X = x(p.s), Y = y(clamp(p[key], -tmax, tmax)); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); }); ctx.stroke(); ctx.setLineDash([]); };
  line('T1v', css('--water'), 1.2, [5, 4]); line('T2v', css('--good'), 1.2, [5, 4]); line('T1', css('--water'), 2.5); line('T2', css('--good'), 2.5);
  ctx.fillStyle = css('--ink'); ctx.textAlign = 'left'; ctx.fillText('T1 meridional (solid: design pressure and liquid · dashed: vacuum, live load and weight)', pad.l + 6, H - pad.b - 18); ctx.fillText('T2 latitudinal (hoop)', pad.l + 6, H - pad.b - 6);
}
function drawThick() {
  const cv = $('#schart'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 70, r: 20, t: 14, b: 32 }, r = R; frame(ctx, W, H, pad);
  const rows = [{ name: 'roof', t: r.tr, req: r.rf.treq, ten: r.rf.tTen, cmp: r.rf.tCmp, ok: r.rf.ok, gov: r.rf.gov, over: r.rf.over }]; if (r.kn) rows.push({ name: 'knuckle', t: r.tk, req: r.kn.treq, ten: r.kn.tTen, cmp: r.kn.tCmp, ok: r.kn.ok, gov: r.kn.gov, over: r.kn.over });
  r.courses.slice().reverse().forEach(c => rows.push({ name: 'C' + (c.i + 1), t: c.t, req: c.treq, ten: c.tTen, cmp: c.tCmp, ok: c.ok, gov: c.gov, over: c.over }));
  const tmax = Math.max(24, ...rows.map(q => Math.max(q.t, Number.isFinite(q.req) ? q.req : 0))) * 1.05, x = t => pad.l + (W - pad.l - pad.r) * clamp(t, 0, tmax) / tmax, rowH = (H - pad.t - pad.b) / rows.length;
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.fillStyle = css('--muted'); ctx.textAlign = 'center'; const st = tmax > 60 ? 10 : 5;
  for (let t = 0; t <= tmax; t += st) { ctx.strokeStyle = css('--grid'); ctx.beginPath(); ctx.moveTo(x(t), pad.t); ctx.lineTo(x(t), H - pad.b); ctx.stroke(); ctx.fillText(t + '', x(t), H - pad.b + 14); }
  ctx.fillText('plate thickness, mm · bar: ordered · brass: tension need · water: compression (buckling) need · black: required', (pad.l + W - pad.r) / 2, H - 4);
  rows.forEach((q, k) => { const y0 = pad.t + k * rowH + 2, hgt = Math.max(rowH - 4, 4); ctx.fillStyle = q.ok ? css('--water-soft') : css('--alarm'); ctx.fillRect(pad.l, y0, x(q.t) - pad.l, hgt);
    ctx.fillStyle = css('--brass'); ctx.fillRect(pad.l, y0 + hgt * 0.28, x(q.ten) - pad.l, hgt * 0.2); ctx.fillStyle = css('--water'); ctx.fillRect(pad.l, y0 + hgt * 0.54, x(Number.isFinite(q.cmp) ? q.cmp : tmax) - pad.l, hgt * 0.2);
    ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x(q.req), y0); ctx.lineTo(x(q.req), y0 + hgt); ctx.stroke();
    ctx.fillStyle = css('--ink'); ctx.textAlign = 'right'; ctx.fillText(q.name, pad.l - 6, y0 + hgt / 2 + 4); ctx.textAlign = 'left'; ctx.fillText(`${q.t} mm · ${q.gov}${q.over ? ' · manual' : ''}${Number.isFinite(q.req) ? '' : ' · no plate thick enough'}`, Math.min(x(q.t) + 5, W - pad.r - 150), y0 + hgt / 2 + 4); });
}
function drawSweep() {
  const cv = $('#trace'); if (!cv || !cv.getContext) return; const ctx = cv.getContext('2d'), W = cv.width, H = cv.height, pad = { l: 64, r: 56, t: 14, b: 32 }, r = R, pts = R.sweep || []; frame(ctx, W, H, pad); if (!pts.length) return;
  const mmax = Math.max(...pts.map(p => p.mass)) / 1000 * 1.1, nmax = Math.max(8, ...pts.map(p => p.Nreq)) * 1.15, x = P => pad.l + (W - pad.l - pad.r) * P / 110, y = m => H - pad.b - (H - pad.t - pad.b) * m / mmax, y2 = nn => H - pad.b - (H - pad.t - pad.b) * nn / nmax;
  ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.globalAlpha = 0.08; ctx.fillStyle = css('--good'); ctx.fillRect(x(0), pad.t, x(18) - x(0), H - pad.t - pad.b); ctx.fillStyle = css('--water'); ctx.fillRect(x(18), pad.t, x(PMAX) - x(18), H - pad.t - pad.b); ctx.fillStyle = css('--alarm'); ctx.fillRect(x(PMAX), pad.t, x(110) - x(PMAX), H - pad.t - pad.b); ctx.globalAlpha = 1;
  ctx.fillStyle = css('--muted'); ctx.textAlign = 'center'; for (let P = 0; P <= 110; P += 10) { ctx.strokeStyle = css('--grid'); ctx.beginPath(); ctx.moveTo(x(P), pad.t); ctx.lineTo(x(P), H - pad.b); ctx.stroke(); ctx.fillText(P + '', x(P), H - pad.b + 14); }
  ctx.fillText('design gas pressure, kPa gauge · green: API 650 Annex F range · blue: API 620 · red: above 15 psig, pressure-vessel code', (pad.l + W - pad.r) / 2, H - 4);
  const stepM = mmax > 400 ? 100 : mmax > 150 ? 50 : mmax > 60 ? 20 : 10; ctx.textAlign = 'right'; for (let m = 0; m <= mmax; m += stepM) { ctx.fillText(m + ' t', pad.l - 6, y(m) + 4); } ctx.textAlign = 'left'; ctx.fillStyle = css('--alarm'); for (let nn = 0; nn <= nmax; nn += Math.ceil(nmax / 4)) ctx.fillText(nn + ' bolts', W - pad.r + 4, y2(nn) + 4);
  const line = (fn, col, wd) => { ctx.strokeStyle = col; ctx.lineWidth = wd; ctx.beginPath(); pts.forEach((p, i) => { i ? ctx.lineTo(x(p.Pg), fn(p)) : ctx.moveTo(x(p.Pg), fn(p)); }); ctx.stroke(); };
  line(p => y(p.mass / 1000), css('--water'), 2.5); line(p => y(p.shell / 1000), css('--steel'), 1.5); line(p => y2(p.Nreq), css('--alarm'), 1.5);
  ctx.fillStyle = css('--brass'); ctx.beginPath(); ctx.arc(x(r.Pg), y(r.massTotal / 1000), 6, 0, 7); ctx.fill(); ctx.fillStyle = css('--ink'); ctx.fillText(`this tank ${fmt(r.massTotal / 1000, 1)} t · ${r.Nreq} × ${state.bolt} needed`, Math.min(x(r.Pg) + 8, W - pad.r - 190), y(r.massTotal / 1000) - 8);
}

/* ------------------------------------------------------------------ readouts */
function updateReadouts() {
  const s = state, r = R, L = live(r), c1 = r.courses[0];
  $('#roCap').textContent = fmt(r.Vwork, 0); $('#roVg').textContent = fmt(r.Vgas0, 0); $('#roKg').textContent = fmt(r.kgm3, 1);
  $('#roT1').textContent = c1.t; $('#roT1req').textContent = fmt(c1.treq, 2); $('#roT1gov').textContent = c1.gov; $('#roT1Box').classList.toggle('alarm', !c1.ok); $('#roT1f').textContent = `${fmt(c1.T2, 0)} / ${fmt(c1.T1, 0)}`;
  $('#roSched').textContent = r.courses.map(c => c.t).join(' / '); $('#roSchedBox').classList.toggle('alarm', !r.allOk); $('#roSchedNote').textContent = r.allOk ? `${r.n} courses bottom to top · minimum ${r.tmin} mm · S ${r.Sts} MPa × E ${r.E}` : 'a course fails a check — see the chart';
  $('#roRoof').textContent = r.tr + ' mm'; $('#roRoofNote').textContent = `${r.rf.gov} · T ${fmt(r.rf.Tt, 0)} kN/m tension, ${fmt(r.rf.Tc, 0)} compression · rise ${fmt(r.g.hr, 2)} m`; $('#roRoofBox').classList.toggle('alarm', !r.rf.ok);
  const kb = $('#roKnBox'); if (r.kn) { kb.classList.remove('off'); $('#roKn').textContent = r.tk + ' mm'; $('#roKnNote').textContent = `T2 at the junction ${fmt(r.kn.T2j, 0)} kN/m (${r.kn.T2j < 0 ? 'compression' : 'tension'}) · ${r.kn.gov}${r.kn.rkOk ? '' : ' · rk below 6 % of Rc'}`; kb.classList.toggle('alarm', !r.kn.ok); } else kb.classList.add('off');
  const rb = $('#roRingBox'); if (r.ring) { rb.classList.remove('off'); $('#roRing').textContent = fmt(r.ring.Aneed, 0) + ' mm²'; $('#roRingNote').textContent = `Q ${fmt(r.ring.Q, 0)} kN → Ac ${fmt(r.ring.Ac, 0)} mm² at 103 MPa · roof and sidewall give ${fmt(r.ring.Apart, 0)} (wc ${fmt(r.ring.wc, 0)}, wh ${fmt(r.ring.wh, 0)} mm) · fitted ${fmt(r.ring.provided, 0)}`; rb.classList.toggle('alarm', !r.ring.ok); } else rb.classList.add('off');
  $('#roMassT').textContent = fmt(r.massTotal / 1000, 1); $('#roMassS').textContent = fmt(r.massShell / 1000, 1); $('#roMassR').textContent = fmt((r.massRoof + r.ringMass) / 1000, 1); $('#roMassB').textContent = fmt(r.massBottom / 1000, 1);
  $('#roUp').textContent = r.anchorsNeeded ? fmt(r.U, 0) : 'none'; $('#roUpNote').textContent = r.anchorsNeeded ? `kN empty at Pg (${fmt(r.Utest, 0)} at test) · ${r.Nb} × ${s.bolt}: ${fmt(r.perBolt, 0)} kN/bolt of ${fmt(r.Fb, 0)} allowed · spacing ${fmt(r.spacing, 2)} m · need ${r.Nreq}` : `metal ${fmt(r.Wtot, 0)} kN outweighs Pg × area ${fmt(r.Pg * r.At, 0)} kN`; $('#roUpBox').classList.toggle('alarm', !r.anchorOk);
  const scopeTxt = r.scope === 'asme' ? 'above 15 psig — pressure-vessel code' : r.scope === 'hot' ? 'above 121 °C — outside API 620' : r.annex === 'Q' ? 'Annex Q (below −51 °C)' : r.annex === 'R' ? 'Annex R (−51 °C to +4 °C)' : 'main body (ambient)'; $('#roScope').textContent = scopeTxt; $('#roScopeNote').textContent = `${r.m.name} · usable to ${r.m.Tmin} °C · design ${r.Tdes} °C${r.matTempOk ? '' : ' — TOO COLD for this plate'}${r.matAnnexOk ? '' : ' — not an Annex Q material'}`; $('#roScopeBox').classList.toggle('alarm', r.scope !== 'ok' || !r.matTempOk || !r.matAnnexOk);
  $('#roLive').textContent = fmt(L.max * 100, 0); $('#roLiveNote').textContent = `${L.where} · level ${fmt(s.level, 2)} m · ${s.Pgas >= 0 ? '+' : ''}${fmt(s.Pgas, 1)} kPa · uplift ${fmt(Math.max(L.uplift, 0), 0)} kN${L.kf > 1 ? ' · against 1.25 × allowable (test)' : ''}`; $('#roLiveBox').classList.toggle('alarm', L.max > 1); $('#roLiveBox').classList.toggle('warnv', L.max <= 1 && L.max > 0.9);
  const sel = typeof s.sel === 'number' ? r.courses[s.sel] : null; let selT = 'click an element', selN = 'course, roof, knuckle or ring · then Thicker / Thinner / Auto';
  if (sel) { selT = `C${sel.i + 1} · ${sel.t} mm`; selN = `T2 ${fmt(sel.T2, 0)}, T1 ${fmt(sel.T1, 0)} kN/m · vacuum T2 ${fmt(sel.T2v, 0)}, T1 ${fmt(sel.T1v, 0)} · need ${fmt(sel.tTen, 2)} tension / ${fmt(sel.tCmp, 2)} compression · ${sel.over ? 'manual' : 'auto'}${sel.ok ? '' : ' · FAILS' + (sel.okOrder ? '' : ' (thinner than course above)')}`; }
  else if (s.sel === 'roof') { selT = `roof · ${r.tr} mm`; selN = `T1 ${fmt(r.rf.T1p, 0)}, T2 ${fmt(r.rf.T2p, 0)} kN/m at Pg · vacuum ${fmt(r.rf.T1v, 0)}, ${fmt(r.rf.T2v, 0)} · need ${fmt(r.rf.tTen, 2)} / ${fmt(r.rf.tCmp, 2)} · ${r.rf.over ? 'manual' : 'auto'}`; }
  else if (s.sel === 'knuckle' && r.kn) { selT = `knuckle · ${r.tk} mm`; selN = `junction T1 ${fmt(r.kn.T1j, 0)}, T2 ${fmt(r.kn.T2j, 0)} kN/m · tension need ${fmt(r.kn.tTen, 2)} · compression need ${fmt(r.kn.tCmp, 2)} · ${r.kn.over ? 'manual' : 'auto'}`; }
  else if (s.sel === 'ring' && r.ring) { selT = `ring · ${fmt(r.ring.provided, 0)} mm²`; selN = `edge T1 ${fmt(r.ring.T1r, 0)} kN/m pulls in at ${fmt(r.ring.q, 0)} kN/m → hoop compression Q ${fmt(r.ring.Q, 0)} kN · use the ring-area slider`; }
  $('#roSel').textContent = selT; $('#roSelNote').textContent = selN; $('#roSelBox').classList.toggle('alarm', (sel && !sel.ok) || (s.sel === 'roof' && !r.rf.ok) || (s.sel === 'knuckle' && r.kn && !r.kn.ok) || (s.sel === 'ring' && r.ring && !r.ring.ok));
  const ps = $('#pillState'); ps.textContent = s.failed ? 'FAILED — ' + s.failed : r.scope !== 'ok' ? (r.scope === 'asme' ? 'Above 15 psig: outside API 620' : 'Above 121 °C: outside API 620') : !r.matTempOk || !r.matAnnexOk ? 'Material not permitted at this temperature' : !r.allOk ? 'Sidewall course fails' : !r.rf.ok ? 'Roof plate fails' : r.kn && !r.kn.ok ? 'Knuckle fails' : r.ring && !r.ring.ok ? 'Compression ring short' : !r.anchorOk ? 'Anchorage inadequate' : 'Design ok'; ps.className = 'pill ' + (s.failed || !r.designOk ? 'warn' : 'ok');
  $('#pillTank').textContent = `D ${fmt(r.D, 1)} m · Pg ${fmt(r.Pg, 1)} kPa · ${r.m.name.split(' (')[0]} · ${fmt(r.massTotal / 1000, 0)} t`;
  $('#pillFill').textContent = s.mode === 'fill' ? `Filling ${s.fluid} ${fmt(s.level, 1)} m` : s.mode === 'drain' ? `Pumping out ${fmt(s.level, 1)} m` : s.mode === 'pressurise' ? `Pressurising ${fmt(s.Pgas, 1)} kPa` : `${s.level > 0.01 ? s.fluid + ' at ' + fmt(s.level, 1) + ' m' : 'empty'} · ${s.Pgas >= 0 ? '+' : ''}${fmt(s.Pgas, 1)} kPa`; $('#pillFill').className = 'pill ' + (s.mode !== 'idle' ? 'on run' : '');
  $('#pillVent').textContent = s.vent === 'blocked' ? 'VENT BLOCKED' : s.vent === 'gagged' ? 'Vent gagged (test)' : s.relief > 0 ? 'PV valve relieving' : s.relief < 0 ? 'Vacuum pallet open' : 'Vent normal'; $('#pillVent').className = 'pill ' + (s.vent === 'blocked' ? 'warn' : s.relief ? 'amber' : '');
  $('#pillOver').classList.toggle('hidden', !(L.max > 1) || !!s.failed); $('#pillAnch').classList.toggle('hidden', r.anchorOk); $('#pillAnnex').textContent = r.annex === 'Q' ? 'Annex Q' : r.annex === 'R' ? 'Annex R' : 'Main body'; $('#pillAnnex').classList.toggle('amber', r.annex !== 'body');
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start'; $('#btnVent').textContent = s.vent === 'blocked' ? 'Vent: BLOCKED' : 'Vent: normal'; $('#btnVent').classList.toggle('on', s.vent === 'blocked');
  $('#ctlRc').classList.toggle('off', s.roof === 'cone'); $('#ctlRk').classList.toggle('off', s.roof !== 'tori'); $('#ctlSlope').classList.toggle('off', s.roof !== 'cone'); $('#ctlRing').classList.toggle('off', s.roof === 'tori');
  $('#HOut').textContent = fmt(r.H, 1) + ' m'; $('#H').max = r.Hs;
  if (!r.designOk) s.flags.bad = true; if (s.flags.bad && r.designOk && (Object.keys(s.overrides).length || s.flags.everBadCourse)) s.flags.fixed = true; if (!r.allOk) s.flags.everBadCourse = true; if (r.scope === 'asme') s.flags.asme = true; if (s.sel !== null) s.flags.select = true; if (r.kn && r.kn.T2j < 0) s.flags.knuckle = true;
}
function record(tag = '') { const s = state, r = R, L = live(r), row = { i: s.log.length + 1, tag, D: +r.D.toFixed(1), Hs: +r.Hs.toFixed(2), H: +r.H.toFixed(2), G: +r.G.toFixed(2), T: r.Tdes, Pg: +r.Pg.toFixed(1), Pe: +r.Pe.toFixed(1), mat: s.mat, E: r.E, CA: r.CA, roof: r.roof, sched: r.courses.map(c => c.t).join('/'), govs: r.courses.map(c => c.gov[0]).join(''), gov1: r.courses[0].gov, T2_1: +r.courses[0].T2.toFixed(0), T1_1: +r.courses[0].T1.toFixed(0), tr: r.tr, tk: r.kn ? r.tk : '', T2k: r.kn ? +r.kn.T2j.toFixed(0) : '', Q: r.ring ? +r.ring.Q.toFixed(0) : '', Aneed: r.ring ? +r.ring.Aneed.toFixed(0) : '', ringOk: r.ring ? (r.ring.ok ? 'ok' : 'short') : '', ok: r.designOk ? 'ok' : 'FAIL', scope: r.scope, annex: r.annex, shell: +(r.massShell / 1000).toFixed(1), total: +(r.massTotal / 1000).toFixed(1), kgm3: +r.kgm3.toFixed(1), cap: +r.Vwork.toFixed(0), U: +r.U.toFixed(0), Nb: r.Nb, Nreq: r.Nreq, bolt: s.bolt, level: +s.level.toFixed(2), Pgas: +s.Pgas.toFixed(1), fluid: s.fluid, util: +(L.max * 100).toFixed(0), failed: s.failed }; s.log.push(row); renderLog(); checkTasks(); }
const LOGK = ['i', 'tag', 'D', 'Hs', 'H', 'G', 'T', 'Pg', 'Pe', 'mat', 'E', 'CA', 'roof', 'sched', 'govs', 'gov1', 'T2_1', 'T1_1', 'tr', 'tk', 'T2k', 'Q', 'Aneed', 'ringOk', 'ok', 'scope', 'annex', 'shell', 'total', 'kgm3', 'cap', 'U', 'Nb', 'Nreq', 'bolt', 'level', 'Pgas', 'fluid', 'util', 'failed'];
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${LOGK.map(k => `<td>${r[k]}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,D_m,sidewall_m,design_level_m,G,T_design_C,Pg_kPa,Pe_kPa,material,E,CA_mm,roof,schedule_mm,governs_per_course,course1_governs,course1_T2_kN_m,course1_T1_kN_m,roof_mm,knuckle_mm,knuckle_T2_kN_m,ring_Q_kN,ring_area_needed_mm2,ring,design,scope,annex,shell_t,total_t,kg_per_m3,capacity_m3,uplift_kN,bolts_fitted,bolts_needed,bolt,level_m,Pgas_kPa,fluid,live_util_pct,failed\n' + state.log.map(r => LOGK.map(k => r[k]).join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, keys) => keys.every(k => a[k] === b[k]);
const TASKS = [
  { id: 'baseline', t: 'Read the baseline', d: 'Record the 30 m refrigerated ammonia tank as it opens: plate schedule, roof plate, compression ring, uplift and anchors, total steel.', ok: () => state.log.length > 0 },
  { id: 'forces', t: 'T1 and T2 along the meridian', d: 'Click the bottom course, the roof and the ring. On the unit-force chart follow T1 and T2 from the apex, across the junction and down the sidewall; note where the hoop force is set by gas pressure and where the liquid head takes over.', ok: () => state.flags.select },
  { id: 'pressure', t: 'Pressure against head', d: 'Record the tank at 10 kPa and again at 50 kPa. Compare the bottom-course hoop force T2: at 10 kPa the liquid head dominates, at 50 kPa the gas pressure does, and every course above the liquid grows.', ok: () => state.log.some(a => state.log.some(b => same(a, b, ['D', 'Hs', 'H', 'mat']) && a.Pg <= 15 && b.Pg >= 45)) },
  { id: 'knuckle', t: 'The knuckle in compression', d: 'Switch to the torispherical roof. Read T2 at the junction: negative — the knuckle is in hoop compression under internal pressure, and its plate is sized by buckling. Shrink the knuckle radius and watch it grow.', ok: () => state.flags.knuckle && state.log.some(x => x.roof === 'tori') },
  { id: 'ring', t: 'Size the compression ring', d: 'On the dome roof, set the ring-area slider below the area needed, see the region fail, then bring it up to pass. Record the passing design.', ok: () => state.log.some(x => x.roof === 'dome' && x.ringOk === 'short') && state.log.some(x => x.roof === 'dome' && x.ringOk === 'ok') },
  { id: 'cone', t: 'Why not a cone?', d: 'Put a 15° cone on the same tank at the same pressure and record it. Its hoop force is R/sin θ times the pressure: the roof plate and the ring both grow, and the vacuum case is worse still.', ok: () => state.log.some(a => a.roof === 'cone' && state.log.some(b => b.roof !== 'cone' && same(a, b, ['D', 'Pg']))) },
  { id: 'uplift', t: 'Hold it down', d: 'Raise the pressure until the empty tank lifts: uplift Pg × area exceeds the metal weight. Fit enough anchor bolts of a size that pass at design and test pressure, with spacing under 3 m, and record it.', ok: () => state.log.some(x => x.U > 0 && x.Nb >= x.Nreq && x.ok === 'ok') },
  { id: 'vacuum', t: 'Vacuum design', d: 'Raise the design vacuum to 5 kPa or more and record it. The top courses and the roof are now sized by compression, not tension: the buckling allowable depends on t/R.', ok: () => state.log.some(x => x.Pe >= 5 && x.govs.includes('c')) },
  { id: 'cold', t: 'Cold service', d: 'Choose LNG (−162 °C) with a carbon steel and read the material warning; then pick an Annex Q material and record a passing design.', ok: () => state.log.some(x => x.T <= -51 && x.ok === 'ok' && MAT[x.mat].annex === 'Q') },
  { id: 'scope', t: 'Fifteen psig', d: 'Push the gas pressure past 103.4 kPa and read the scope warning: above 15 psig the tank is a pressure vessel, not an API 620 tank. Then bring it back inside.', ok: () => state.flags.asme },
  { id: 'test', t: 'Hydro-pneumatic test', d: 'Run the test: water to the test level (1.25 × G × H, so the head matches 1.25 × the product load), vent gagged, then air to 1.25 × Pg. Watch the utilisation of every element and record the reading at test pressure.', ok: () => state.flags.test && state.log.some(x => x.fluid === 'water' && x.Pgas >= 1.2 * x.Pg) },
  { id: 'blocked', t: 'Blocked vent', d: 'Block the vent and pump out: the vapour space goes into vacuum within seconds and the sidewall buckles. Then block it and fill: pressure climbs past the setpoint until something yields. Record one failure.', ok: () => state.log.some(x => x.failed) || (state.flags.failed && state.log.some(x => x.util >= 100)) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame_ = 0, dirty = true;
function recompute() { Object.assign(R, design(state)); R.sweep = sweep(state); if (state.level > R.Hs) state.level = R.Hs; dirty = false; }
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; if (dirty) recompute(); step(dt); drawRig(); frame_++; if (frame_ % 2 === 0) { updateReadouts(); if (frame_ % 6 === 0) { drawForces(); drawThick(); drawSweep(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setSel(id, v) { const e = $('#' + id); e.value = v; state[id] = v; dirty = true; }
function resetProcess() { const s = state; s.level = 0; s.Pgas = 0; s.mode = 'idle'; s.failed = ''; s.relief = 0; s.vent = 'open'; s.testPending = false; s.fluid = 'product'; setSel('fluid', 'product'); }
function fillTo(target, fluid) { const s = state; if (s.failed) return; s.fluid = fluid; setSel('fluid', fluid); s.target = target; s.ptarget = s.Pgas; s.mode = target > s.level ? 'fill' : 'drain'; s.run = true; }
function pressuriseTo(p) { const s = state; if (s.failed) return; s.ptarget = p; if (s.vent === 'open' && p > R.Pg) s.vent = 'gagged'; s.mode = 'pressurise'; s.run = true; }
function runTest() { const s = state; if (s.failed) return; s.vent = 'open'; s.testPending = true; s.fluid = 'water'; setSel('fluid', 'water'); s.target = Math.min(R.H, 1.25 * R.G * R.H); s.ptarget = 1.25 * R.Pg; s.mode = 'fill'; s.run = true; }
const selKey = () => typeof state.sel === 'number' ? state.sel : state.sel === 'roof' ? 'roof' : state.sel === 'knuckle' ? 'knuckle' : null;
function adjust(dir) { const k = selKey(); if (k === null) return; const cur = typeof k === 'number' ? R.courses[k].t : k === 'roof' ? R.tr : R.tk; const idx = PLATES.indexOf(cur); state.overrides[k] = PLATES[clamp((idx < 0 ? 0 : idx) + dir, 0, PLATES.length - 1)]; dirty = true; recompute(); }
function autoSel() { const k = selKey(); if (k !== null) delete state.overrides[k]; dirty = true; recompute(); }
function resetOv() { state.overrides = {}; dirty = true; recompute(); }
function applyProduct(key) { const pr = PRODUCT[key]; if (!pr) return; setCtl('G', pr.G); setCtl('Tdes', pr.T); setCtl('Pg', pr.Pg); }
const PRESETS = {
  base: () => { setSel('product', 'ammonia'); setSel('mat', 'a516_485n'); setSel('joint', 'full'); setSel('roof', 'dome'); setSel('bolt', 'M42'); for (const [k, v] of Object.entries({ D: 30, n: 6, hc: 2.4, G: 0.68, Tdes: -33, Pg: 10, Pe: 1.0, CA: 1.0, rc: 1.0, rk: 0.10, slope: 15, Lr: 1.2, S: 0, ringA: 12500, Nb: 48, fillRate: 500, tscale: 60 })) setCtl(k, v); recompute(); setCtl('H', 13.5); state.overrides = {}; state.sel = null; resetProcess(); state.run = true; dirty = true; recompute(); },
  pressure: () => { PRESETS.base(); setSel('product', 'pentane'); applyProduct('pentane'); recompute(); },
  tori: () => { PRESETS.base(); setSel('roof', 'tori'); setCtl('rk', 0.08); recompute(); },
  cone: () => { PRESETS.base(); setSel('roof', 'cone'); setCtl('slope', 15); setCtl('ringA', 20000); recompute(); },
  uplift: () => { PRESETS.base(); setSel('product', 'blanket'); applyProduct('blanket'); setCtl('Nb', 12); recompute(); },
  vacuum: () => { PRESETS.base(); setCtl('Pe', 6); recompute(); },
  lng: () => { PRESETS.base(); setSel('product', 'lng'); applyProduct('lng'); setCtl('D', 40); setCtl('n', 10); setCtl('hc', 2.4); recompute(); setCtl('H', 22); setSel('bolt', 'M56'); setCtl('Nb', 100); setCtl('ringA', 54000); recompute(); },
  asme: () => { PRESETS.base(); setSel('product', 'blanket'); applyProduct('blanket'); setCtl('Pg', 110); recompute(); },
  test: () => { PRESETS.base(); setCtl('tscale', 600); recompute(); runTest(); },
  blockdrain: () => { PRESETS.base(); setCtl('tscale', 60); setCtl('fillRate', 1000); recompute(); state.level = R.H; state.vent = 'blocked'; state.fluid = 'product'; state.target = 0; state.ptarget = 0; state.mode = 'drain'; },
  blockfill: () => { PRESETS.base(); setCtl('tscale', 200); setCtl('fillRate', 1000); recompute(); state.level = 0.5 * R.H; state.vent = 'blocked'; state.target = R.Hs; state.ptarget = 0; state.mode = 'fill'; },
  small: () => { PRESETS.base(); setSel('product', 'naphtha'); applyProduct('naphtha'); setSel('mat', 'a36'); setCtl('D', 12); setCtl('n', 4); setCtl('hc', 2.0); recompute(); setCtl('H', 7.5); setCtl('Nb', 16); setCtl('ringA', 2000); recompute(); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); dirty = true; }; e.addEventListener('input', set); set(); };
  bindR('D', v => v.toFixed(1) + ' m'); bindR('n', v => v + ' courses'); bindR('hc', v => v.toFixed(2) + ' m'); bindR('H', v => v.toFixed(1) + ' m'); bindR('G', v => v.toFixed(2)); bindR('Tdes', v => v + ' °C'); bindR('Pg', v => v.toFixed(1) + ' kPa (' + (v / 6.895).toFixed(2) + ' psig)'); bindR('Pe', v => v.toFixed(1) + ' kPa'); bindR('CA', v => v.toFixed(1) + ' mm'); bindR('rc', v => v.toFixed(2) + ' × D'); bindR('rk', v => v.toFixed(2) + ' × D'); bindR('slope', v => v.toFixed(0) + '°'); bindR('Lr', v => v.toFixed(2) + ' kPa'); bindR('S', v => v.toFixed(2) + ' kPa'); bindR('ringA', v => v + ' mm²'); bindR('Nb', v => v + ' bolts'); bindR('fillRate', v => v + ' m³/h'); bindR('tscale', v => '×' + v);
  for (const id of ['mat', 'joint', 'roof', 'bolt', 'fluid']) $('#' + id).addEventListener('change', e => { state[id] = e.target.value; dirty = true; });
  $('#product').addEventListener('change', e => { state.product = e.target.value; applyProduct(state.product); dirty = true; });
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnFill').onclick = () => fillTo(R.H, state.fluid === 'water' ? 'water' : 'product'); $('#btnDrain').onclick = () => fillTo(0, state.fluid); $('#btnPress').onclick = () => pressuriseTo(R.Pg); $('#btnTest').onclick = runTest; $('#btnVent').onclick = () => { state.vent = state.vent === 'blocked' ? 'open' : 'blocked'; }; $('#btnResetP').onclick = resetProcess; $('#btnRecord').onclick = () => record('');
  $('#btnThick').onclick = () => adjust(1); $('#btnThin').onclick = () => adjust(-1); $('#btnAuto').onclick = autoSel; $('#btnReset').onclick = resetOv;
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('t620_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'F') $('#btnFill').click(); if (k === 'E') $('#btnDrain').click(); if (k === 'P') $('#btnPress').click(); if (k === 'W') runTest(); if (k === 'V') $('#btnVent').click(); if (k === 'R') record(''); if (e.key === 'ArrowUp' && state.sel !== null) { e.preventDefault(); adjust(1); } if (e.key === 'ArrowDown' && state.sel !== null) { e.preventDefault(); adjust(-1); } if (k === 'A') autoSel(); if (e.key === 'Escape') { $('#help').classList.add('hidden'); state.sel = null; } });
}

/* ------------------------------------------------------------------ dashboard API */
window.TankDesign620 = {
  getState: () => ({ inputs: { D: state.D, n: state.n, hc: state.hc, H: state.H, product: state.product, G: state.G, Tdes: state.Tdes, Pg: state.Pg, Pe: state.Pe, CA: state.CA, mat: state.mat, joint: state.joint, roof: state.roof, rc: state.rc, rk: state.rk, slope: state.slope, Lr: state.Lr, S: state.S, ringA: state.ringA, bolt: state.bolt, Nb: state.Nb, fluid: state.fluid, fillRate: state.fillRate, tscale: state.tscale, vent: state.vent, run: state.run, overrides: Object.assign({}, state.overrides) },
    results: R.courses ? { Hs: R.Hs, schedule: R.courses.map(c => c.t), required: R.courses.map(c => +c.treq.toFixed(2)), governs: R.courses.map(c => c.gov), T1: R.courses.map(c => +c.T1.toFixed(1)), T2: R.courses.map(c => +c.T2.toFixed(1)), tr: R.tr, tk: R.tk, knuckleT2: R.kn ? R.kn.T2j : null, ringQ: R.ring ? R.ring.Q : null, ringAneed: R.ring ? R.ring.Aneed : null, ringOk: R.ring ? R.ring.ok : null, allOk: R.allOk, designOk: R.designOk, scope: R.scope, annex: R.annex, matTempOk: R.matTempOk, massShell: R.massShell, massTotal: R.massTotal, kgm3: R.kgm3, Vwork: R.Vwork, U: R.U, Utest: R.Utest, Nreq: R.Nreq, anchorOk: R.anchorOk, level: state.level, Pgas: state.Pgas, failed: state.failed, liveUtil: live(R).max } : {},
    tasksDone: TASKS.filter(t => { try { return t.ok(); } catch (e) { return false; } }).map(t => t.id) }),
  setState: inp => { for (const k of ['mat', 'joint', 'roof', 'bolt', 'fluid']) if (inp[k]) setSel(k, inp[k]); if (inp.product) { setSel('product', inp.product); applyProduct(inp.product); } for (const k of ['D', 'n', 'hc', 'G', 'Tdes', 'Pg', 'Pe', 'CA', 'rc', 'rk', 'slope', 'Lr', 'S', 'ringA', 'Nb', 'fillRate', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.H !== undefined) { recompute(); setCtl('H', inp.H); } if (inp.overrides) state.overrides = Object.assign({}, inp.overrides); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.vent) state.vent = inp.vent; if (inp.fill) fillTo(R.H, state.fluid); if (inp.drain) fillTo(0, state.fluid); if (inp.pressurise) pressuriseTo(R.Pg); if (inp.test) runTest(); if (inp.resetProcess) resetProcess(); if (inp.level !== undefined) state.level = clamp(+inp.level, 0, R.Hs || 0); if (inp.Pgas !== undefined) state.Pgas = +inp.Pgas; if (inp.run !== undefined) state.run = !!inp.run; dirty = true; recompute(); },
  meta: { id: 'api620-tank', title: 'API 620 Tank Design Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.TankDesign620.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'api620-tank', state: window.TankDesign620.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('t620_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); recompute(); step(0); drawRig(); updateReadouts(); drawForces(); drawThick(); drawSweep();
  requestAnimationFrame(tick);
}
build();
