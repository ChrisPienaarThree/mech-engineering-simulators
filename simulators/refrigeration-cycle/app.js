/* Refrigeration Cycle Trainer — vapour-compression cycle on R134a: evaporating and condensing temperatures, superheat,
   subcooling, compressor isentropic and volumetric efficiency, capacity, power and COP on a p-h diagram; and a cold room
   whose temperature, evaporating and condensing temperatures follow the load, the ambient and a thermostat on a clock.
   Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
/* R134a saturation: T °C, p kPa, hf, hg, sf, sg (IIR reference: h 200 kJ/kg, s 1.0 kJ/kg·K at 0 °C liquid) */
const SAT = [[-40, 51.2, 148.1, 374.0, 0.7956, 1.7643], [-30, 84.4, 161.1, 380.3, 0.8500, 1.7515], [-20, 132.7, 173.6, 386.5, 0.9002, 1.7413], [-10, 200.6, 186.7, 392.7, 0.9506, 1.7334], [0, 292.8, 200.0, 398.6, 1.0000, 1.7271], [10, 414.6, 213.6, 404.3, 1.0485, 1.7218], [20, 571.7, 227.5, 409.8, 1.0962, 1.7180], [30, 770.2, 241.7, 414.8, 1.1432, 1.7145], [40, 1016.6, 256.4, 419.4, 1.1903, 1.7111], [50, 1317.9, 271.6, 423.4, 1.2373, 1.7072], [60, 1681.8, 287.5, 426.6, 1.2848, 1.7024], [70, 2116.8, 304.3, 428.8, 1.3332, 1.6961], [80, 2632.9, 322.4, 429.3, 1.3833, 1.6868]];
const CPV = [[50, 0.78], [130, 0.82], [290, 0.90], [570, 0.98], [1000, 1.08], [1300, 1.15], [1700, 1.25], [2100, 1.40], [2650, 1.6]], CPL = 1.4, RG = 0.0815, ZG = 0.92;
const interp = (tab, x, ix, iy, logx = false) => { const f = logx ? Math.log : v => v, X = f(x); if (X <= f(tab[0][ix])) return tab[0][iy]; for (let i = 0; i < tab.length - 1; i++) { const a = f(tab[i][ix]), b = f(tab[i + 1][ix]); if (X <= b) { const t = (X - a) / (b - a); return tab[i][iy] + t * (tab[i + 1][iy] - tab[i][iy]); } } return tab[tab.length - 1][iy]; };
const satT = T => { const p = Math.exp(interp(SAT.map(r => [r[0], Math.log(r[1])]), T, 0, 1)); return { T, p, hf: interp(SAT, T, 0, 2), hg: interp(SAT, T, 0, 3), sf: interp(SAT, T, 0, 4), sg: interp(SAT, T, 0, 5) }; };
const cpv = p => interp(CPV, p, 0, 1, true);

/* ------------------------------------------------------------------ state */
const state = { mode: 'explorer', Te: -10, Tc: 40, SH: 5, SC: 3, eta: 0.7, Vd: 20, clr: 4, charge: 100, Tamb: 30, SPr: -18, load: 3, UA: 0.25, Cth: 800, foul: 0, dTe: 8, dTc: 12, tscale: 30, run: true,
  Troom: 20, comp: true, cycles: 0, t: 0, dash: 0, trace: [], traceAcc: 0, log: [], flags: {}, TeDyn: -10, TcDyn: 40, pulled: false, lastOn: 0 };
const R = {};

/* ------------------------------------------------------------------ model */
function cycle(Te, Tc, o = {}) {
  const s = Object.assign({}, state, o), e = satT(Te), c = satT(Tc), SH = s.SH + (100 - s.charge) * 0.3, SC = Math.max(0, s.SC - (100 - s.charge) * 0.1), cp1 = cpv(e.p), cp2 = cpv(c.p);
  const T1 = Te + SH, h1 = e.hg + cp1 * SH, s1 = e.sg + cp1 * Math.log((T1 + 273.15) / (Te + 273.15)), wet = SH <= 0.05;
  const T2s = (Tc + 273.15) * Math.exp((s1 - c.sg) / cp2) - 273.15, h2s = c.hg + cp2 * Math.max(0, T2s - Tc), h2 = h1 + (h2s - h1) / s.eta, T2 = Tc + (h2 - c.hg) / cp2;
  const h3 = c.hf - CPL * SC, T3 = Tc - SC, h4 = h3, x4 = (h4 - e.hf) / (e.hg - e.hf);
  const rp = c.p / e.p, etav = clamp(1 - s.clr / 100 * (Math.pow(rp, 1 / 1.1) - 1) - 0.03, 0.05, 1), v1 = ZG * RG * (T1 + 273.15) / e.p, m = etav * s.Vd / 3600 / v1 * (s.charge < 70 ? (s.charge - 40) / 30 : 1);
  const qe = h1 - h4, w = h2 - h1, qc = h2 - h3, Qe = m * qe, W = m * w, Qc = m * qc, COP = qe / w, COPc = (Te + 273.15) / (Tc - Te), hp = COP + 1;
  return { e, c, T1, h1, s1, T2s, h2s, h2, T2, h3, T3, h4, x4, SH, SC, rp, etav, v1, m, qe, w, qc, Qe, W, Qc, COP, COPc, hp, wet, hot: T2 > 120, flash: x4 > 0.4, Te, Tc, hg2: c.hg };
}
function step(dt) {
  const s = state;
  if (s.mode === 'explorer') { Object.assign(R, cycle(s.Te, s.Tc)); R.room = null; if (s.run) { s.t += dt; s.dash += 30 * dt; } }
  else {
    const dts = dt * s.tscale; if (s.run) { s.t += dts; }
    const Tc = s.Tamb + s.dTc + s.foul, Te = Math.max(-45, s.Troom - s.dTe); s.TeDyn = Te; s.TcDyn = Tc; const cy = cycle(Te, Tc);
    if (s.run) { if (s.comp && s.Troom < s.SPr - 1) { s.comp = false; } else if (!s.comp && s.Troom > s.SPr + 1) { s.comp = true; s.cycles++; } const Qin = s.UA * (s.Tamb - s.Troom) + s.load, Qout = s.comp ? cy.Qe : 0; s.Troom += (Qin - Qout) / s.Cth * dts; if (s.Troom <= s.SPr + 0.5) s.pulled = true; s.dash += (s.comp ? 30 : 0) * dt; }
    Object.assign(R, cy); R.room = { Qin: s.UA * (s.Tamb - s.Troom) + s.load, on: s.comp, Te, Tc }; if (!s.comp) { R.Qe = 0; R.W = 0; R.m = 0; }
    if (s.pulled) s.flags.pulldown = true; if (s.cycles >= 3) s.flags.cycling = true;
  }
  if (R.wet && (s.mode === 'explorer' || s.comp)) s.flags.slug = true; if (R.hot && (s.mode === 'explorer' || s.comp)) s.flags.hot = true;
  s.traceAcc += dt; if (s.traceAcc >= 0.2) { s.traceAcc = 0; s.trace.push({ t: s.t, Troom: s.Troom, Qe: R.Qe, W: R.W, on: s.comp ? 1 : 0, Te: R.Te }); if (s.trace.length > 1500) s.trace.shift(); }
}

/* ------------------------------------------------------------------ drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p, anchor) => { const t = el('text', { x, y, class: cls }, p); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
let svg; function buildRig() { svg = el('svg', { viewBox: '0 0 960 400', role: 'img', 'aria-label': 'Refrigeration circuit' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); }
function drawRig() {
  svg.innerHTML = ''; const s = state, r = R, on = s.mode === 'explorer' || s.comp;
  const pipe = (d, cls) => { el('path', { d, class: 'pipe ' + cls }, svg); if (on) el('path', { d, class: 'flow', style: `stroke-dashoffset:${-s.dash}` }, svg); };
  const tag = (x, y, n, lines) => { el('circle', { cx: x, cy: y, r: 11, class: 'state' }, svg); txt(x, y + 4, n, 'statet', svg, 'middle'); lines.forEach((l, i) => txt(x + 16, y - 4 + i * 13, l, 'val', svg)); };
  if (s.mode === 'coldroom') { el('rect', { x: 100, y: 234, width: 300, height: 112, rx: 8, class: 'room' }, svg); txt(250, 250, `COLD ROOM ${fmt(s.Troom, 1)} °C · set ${s.SPr} °C · ${s.comp ? 'ON' : 'OFF'} · ${s.cycles} cycles`, 'ftxt', svg, 'middle'); txt(250, 342, `heat in ${fmt(r.room.Qin, 2)} kW (leak + product)`, 'lbl', svg, 'middle'); }
  el('rect', { x: 120, y: 260, width: 260, height: 70, rx: 8, class: 'box cold' }, svg); txt(250, 288, 'EVAPORATOR', 'lbl', svg, 'middle'); txt(250, 306, `${fmt(r.Te, 1)} °C · ${fmt(r.e.p, 0)} kPa · ${fmt(r.Qe, 2)} kW absorbed`, 'val', svg, 'middle');
  el('rect', { x: 120, y: 50, width: 260, height: 70, rx: 8, class: 'box hot' }, svg); txt(250, 78, 'CONDENSER', 'lbl', svg, 'middle'); txt(250, 96, `${fmt(r.Tc, 1)} °C · ${fmt(r.c.p, 0)} kPa · ${fmt(r.Qc, 2)} kW rejected`, 'val', svg, 'middle'); if (s.mode === 'coldroom') txt(250, 38, `ambient ${s.Tamb} °C${s.foul > 0 ? ' · fouled +' + s.foul + ' K' : ''}`, 'lbl', svg, 'middle');
  el('circle', { cx: 520, cy: 190, r: 46, class: 'box' }, svg); txt(520, 186, 'COMPRESSOR', 'lbl', svg, 'middle'); txt(520, 204, on ? `${fmt(r.W, 2)} kW` : 'off', 'ftxt', svg, 'middle');
  el('polygon', { points: '60,180 80,168 80,192', class: 'box' }, svg); el('polygon', { points: '60,180 40,168 40,192', class: 'box' }, svg); txt(60, 156, 'TXV', 'lbl', svg, 'middle');
  pipe('M380,295 L520,295 L520,236', 'cold'); pipe('M520,144 L520,85 L380,85', 'hot'); pipe('M120,85 L60,85 L60,168', 'hot'); pipe('M60,192 L60,295 L120,295', 'cold');
  tag(470, 250, '1', [`${fmt(r.T1, 1)} °C · superheat ${fmt(r.SH, 1)} K`, `h ${fmt(r.h1, 1)} kJ/kg`]); tag(470, 120, '2', [`${fmt(r.T2, 0)} °C discharge`, `h ${fmt(r.h2, 1)} · rp ${fmt(r.rp, 2)}`]); tag(60, 130, '3', [`${fmt(r.T3, 1)} °C · subcooled ${fmt(r.SC, 1)} K`, `h ${fmt(r.h3, 1)} kJ/kg`]); tag(60, 212, '4', [`${fmt(r.Te, 1)} °C · x ${fmt(r.x4, 2)}`, `flash gas ${fmt(r.x4 * 100, 0)} %`]);
  const bx = 720; const bar = (y, name, v, vmax, unit, bad) => { txt(bx, y - 6, name, 'lbl', svg); el('rect', { x: bx, y, width: 200, height: 14, class: 'tbar' }, svg); el('rect', { x: bx, y, width: clamp(v / vmax, 0, 1) * 200, height: 14, class: 'tfill' + (bad ? ' bad' : '') }, svg); txt(bx + 206, y + 11, `${fmt(v, unit === '°C' ? 0 : 2)} ${unit}`, 'val', svg); };
  bar(50, 'COP (Carnot ' + fmt(r.COPc, 1) + ')', r.COP, 8, '', r.COP < 2); bar(100, 'Discharge temperature (limit 120 °C)', r.T2, 160, '°C', r.hot); bar(150, 'Flash gas after the valve', r.x4, 1, '', r.flash); bar(200, 'Volumetric efficiency', r.etav, 1, '', r.etav < 0.5); bar(250, 'Capacity kW', r.Qe, Math.max(10, r.Qe * 1.2), 'kW', false);
  txt(bx, 300, `mass flow ${fmt(r.m * 1000, 1)} g/s · v₁ ${fmt(r.v1, 4)} m³/kg · ${s.Vd} m³/h swept`, 'val', svg); txt(bx, 318, `q evap ${fmt(r.qe, 1)} · w comp ${fmt(r.w, 1)} kJ/kg`, 'val', svg);
  const w1 = r.wet ? 'NO SUPERHEAT — liquid reaches the compressor: slugging and valve damage' : r.hot ? `DISCHARGE ${fmt(r.T2, 0)} °C — oil breakdown above 120 °C: pressure ratio ${fmt(r.rp, 1)} too high` : r.flash ? `FLASH GAS ${fmt(r.x4 * 100, 0)} % — most of the liquid boils in the valve: subcool more or lower the condensing temperature` : s.mode === 'coldroom' && !s.comp ? 'compressor off on thermostat — room warming on heat leak' : `COP ${fmt(r.COP, 2)} · ${fmt(r.Qe, 2)} kW cooling for ${fmt(r.W, 2)} kW electrical`;
  txt(480, 372, w1, 'val' + (r.wet || r.hot || r.flash ? ' bad' : ''), svg, 'middle');
  txt(480, 392, s.mode === 'explorer' ? `lift ${fmt(r.Tc - r.Te, 0)} K · pressure ratio ${fmt(r.rp, 2)} · isentropic efficiency ${s.eta} · charge ${s.charge} %` : `cold room ${fmt(s.Troom, 1)} °C → evaporating ${fmt(r.Te, 1)} °C (approach ${s.dTe} K) · ambient ${s.Tamb} → condensing ${fmt(r.Tc, 0)} °C · clock ×${s.tscale}`, 'val', svg, 'middle');
}
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawPh() {
  const cv = $('#chart'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 20, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const h0 = 120, h1 = 480, lp0 = Math.log(40), lp1 = Math.log(3000), X = h => L + (h - h0) / (h1 - h0) * pw, Y = p => T + (1 - (Math.log(p) - lp0) / (lp1 - lp0)) * ph;
  ctx.strokeStyle = col('--grid'); for (let h = 150; h <= 450; h += 50) { ctx.beginPath(); ctx.moveTo(X(h), T); ctx.lineTo(X(h), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(h, X(h), H - 18); } for (const p of [50, 100, 200, 500, 1000, 2000]) { ctx.beginPath(); ctx.moveTo(L, Y(p)); ctx.lineTo(L + pw, Y(p)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(p, L - 4, Y(p) + 3); }
  ctx.beginPath(); ctx.strokeStyle = col('--steel'); ctx.lineWidth = 2; SAT.forEach((q, i) => i ? ctx.lineTo(X(q[2]), Y(q[1])) : ctx.moveTo(X(q[2]), Y(q[1]))); ctx.lineTo(X(389), Y(4059)); for (let i = SAT.length - 1; i >= 0; i--) ctx.lineTo(X(SAT[i][3]), Y(SAT[i][1])); ctx.stroke();
  ctx.setLineDash([2, 3]); ctx.lineWidth = 1; for (const q of SAT) if (q[0] % 20 === 0) { ctx.beginPath(); ctx.moveTo(X(q[2]), Y(q[1])); ctx.lineTo(X(q[3]), Y(q[1])); ctx.stroke(); ctx.fillStyle = col('--muted'); ctx.textAlign = 'left'; ctx.fillText(q[0] + ' °C', X(q[3]) + 4, Y(q[1]) + 3); } ctx.setLineDash([]);
  const pe = r.e.p, pc = r.c.p; ctx.beginPath(); ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 2.4; ctx.moveTo(X(r.h1), Y(pe)); ctx.lineTo(X(r.h2), Y(pc)); ctx.lineTo(X(r.h3), Y(pc)); ctx.lineTo(X(r.h4), Y(pe)); ctx.closePath(); ctx.stroke();
  ctx.setLineDash([4, 3]); ctx.strokeStyle = col('--water-deep'); ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(X(r.h1), Y(pe)); ctx.lineTo(X(r.h2s), Y(pc)); ctx.stroke(); ctx.setLineDash([]);
  const dot = (h, p, lbl, dx = 8, dy = -6) => { ctx.fillStyle = col('--brass'); ctx.beginPath(); ctx.arc(X(h), Y(p), 5, 0, 7); ctx.fill(); ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(lbl, X(h) + dx, Y(p) + dy); };
  dot(r.h1, pe, '1 suction', 8, 16); dot(r.h2, pc, `2 discharge ${fmt(r.T2, 0)} °C`); dot(r.h3, pc, '3 liquid', -60, -8); dot(r.h4, pe, `4 · x ${fmt(r.x4, 2)}`, -70, 16);
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`q evap ${fmt(r.qe, 0)}`, X((r.h1 + r.h4) / 2), Y(pe) + 28); ctx.fillText(`w ${fmt(r.w, 0)}`, X((r.h1 + r.h2) / 2) + 30, Y(Math.sqrt(pe * pc))); ctx.fillText(`q cond ${fmt(r.qc, 0)}`, X((r.h2 + r.h3) / 2), Y(pc) - 22);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('enthalpy kJ/kg · pressure kPa (log) · R134a dome with isotherms · dashed = isentropic compression', L + pw / 2, H - 4);
}
function drawSweep() {
  const cv = $('#chart2'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 14, B = 34, pw = W - L - Rr, ph = H - T - B, s = state, r = R;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; ctx.fillStyle = col('--muted');
  const Te0 = -35, Te1 = 15, X = Te => L + (Te - Te0) / (Te1 - Te0) * pw, Qref = cycle(10, r.Tc).Qe * 1.1, Y = q => T + (1 - clamp(q / Qref, 0, 1)) * ph, Y2 = c => T + (1 - clamp(c / 8, 0, 1)) * ph;
  ctx.strokeStyle = col('--grid'); for (let Te = Te0; Te <= Te1; Te += 5) { ctx.beginPath(); ctx.moveTo(X(Te), T); ctx.lineTo(X(Te), T + ph); ctx.stroke(); ctx.textAlign = 'center'; ctx.fillText(Te, X(Te), H - 18); } for (let f = 0; f <= 1; f += 0.25) { ctx.textAlign = 'right'; ctx.fillStyle = col('--water'); ctx.fillText(fmt(Qref * f, 1), L - 4, Y(Qref * f) + 3); ctx.textAlign = 'left'; ctx.fillStyle = col('--brass'); ctx.fillText(fmt(8 * f, 0), L + pw + 4, Y2(8 * f) + 3); }
  const curves = [[r.Tc, 2.4, 1], [r.Tc + 15, 1.2, 0.5]]; for (const [Tc, w, a] of curves) { ctx.globalAlpha = a; ctx.beginPath(); ctx.strokeStyle = col('--water'); ctx.lineWidth = w; for (let i = 0; i <= 50; i++) { const Te = Te0 + (Te1 - Te0) * i / 50, c = cycle(Te, Tc); i ? ctx.lineTo(X(Te), Y(c.Qe)) : ctx.moveTo(X(Te), Y(c.Qe)); } ctx.stroke(); ctx.beginPath(); ctx.strokeStyle = col('--brass'); ctx.setLineDash([5, 3]); for (let i = 0; i <= 50; i++) { const Te = Te0 + (Te1 - Te0) * i / 50, c = cycle(Te, Tc); i ? ctx.lineTo(X(Te), Y2(c.COP)) : ctx.moveTo(X(Te), Y2(c.COP)); } ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1; }
  ctx.fillStyle = col('--alarm'); ctx.beginPath(); ctx.arc(X(r.Te), Y(cycle(r.Te, r.Tc).Qe), 6, 0, 7); ctx.fill(); ctx.beginPath(); ctx.arc(X(r.Te), Y2(r.COP), 6, 0, 7); ctx.fill();
  ctx.fillStyle = col('--ink'); ctx.font = '600 11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`condensing ${fmt(r.Tc, 0)} °C (bold) and ${fmt(r.Tc + 15, 0)} °C (faint) · capacity solid, COP dashed`, L + 6, T + 14);
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('evaporating temperature °C · left axis capacity kW (blue) · right axis COP (brass) · same compressor', L + pw / 2, H - 4);
}
function drawTrace() {
  const cv = $('#trace'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const L = 56, Rr = 60, T = 8, B = 20, pw = W - L - Rr, ph = H - T - B, s = state, span = s.mode === 'coldroom' ? 3600 : 120, tr = s.trace.filter(p => p.t > s.t - span);
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L, T, pw, ph); ctx.font = '10px IBM Plex Mono, monospace'; if (tr.length < 2 || s.mode !== 'coldroom') { ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText('cold-room mode: room temperature, capacity and power against time', L + pw / 2, T + ph / 2); return; }
  const X = t => L + (t - (s.t - span)) / span * pw, Tlo = Math.min(-30, s.SPr - 5), Thi = Math.max(35, s.Tamb + 5), YT = v => T + (1 - (v - Tlo) / (Thi - Tlo)) * ph, Qm = Math.max(1, ...tr.map(p => p.Qe)) * 1.1, YQ = v => T + (1 - v / Qm) * ph;
  ctx.fillStyle = 'rgba(60,120,200,.10)'; for (const p of tr) if (p.on) ctx.fillRect(X(p.t) - 1, T, pw / span * 0.2 * s.tscale + 1, ph);
  ctx.strokeStyle = col('--ink'); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(L, YT(s.SPr)); ctx.lineTo(L + pw, YT(s.SPr)); ctx.stroke(); ctx.setLineDash([]);
  const ser = (k, color, Y, w) => { ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = w; let f = true; for (const p of tr) { f ? ctx.moveTo(X(p.t), Y(p[k])) : ctx.lineTo(X(p.t), Y(p[k])); f = false; } ctx.stroke(); };
  ser('Qe', col('--water'), YQ, 1.4); ser('W', col('--brass'), YQ, 1.4); ser('Troom', col('--alarm'), YT, 2.4); ser('Te', col('--water-deep'), YT, 1);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'right'; ctx.fillText(`${Thi} °C`, L - 4, T + 10); ctx.fillText(`${Tlo}`, L - 4, T + ph); ctx.textAlign = 'left'; ctx.fillStyle = col('--alarm'); ctx.fillText('room', L + pw + 4, T + 10); ctx.fillStyle = col('--water-deep'); ctx.fillText('evap', L + pw + 4, T + 24); ctx.fillStyle = col('--water'); ctx.fillText('kW cool', L + pw + 4, T + 38); ctx.fillStyle = col('--brass'); ctx.fillText('kW elec', L + pw + 4, T + 52);
  ctx.fillStyle = col('--muted'); ctx.textAlign = 'center'; ctx.fillText(`last ${span / 60} min of room time · shaded = compressor running · dashed = thermostat setting`, L + pw / 2, H - 6);
}

/* ------------------------------------------------------------------ readouts, log, tasks */
function updateReadouts() {
  const s = state, r = R, set = (id, v) => { $('#' + id).textContent = v; };
  set('roQe', fmt(r.Qe, 2)); set('roW', fmt(r.W, 2)); set('roCOP', fmt(r.COP, 2)); set('roCOPc', fmt(r.COPc, 1)); set('roM', fmt(r.m * 1000, 1)); set('roEtav', fmt(r.etav, 2)); set('roTe', fmt(r.Te, 1)); set('roPe', fmt(r.e.p, 0)); set('roTc', fmt(r.Tc, 1)); set('roPc', fmt(r.c.p, 0)); set('roRp', fmt(r.rp, 2)); set('roT2', fmt(r.T2, 0)); set('roSH', fmt(r.SH, 1)); set('roSC', fmt(r.SC, 1)); set('roX4', fmt(r.x4 * 100, 0)); set('roQc', fmt(r.Qc, 2)); set('roRoom', s.mode === 'coldroom' ? fmt(s.Troom, 1) : '—'); set('roRoomNote', s.mode === 'coldroom' ? `°C · set ${s.SPr} · ${s.comp ? 'running' : 'off'} · ${s.cycles} cycles` : 'explorer mode');
  $('#roT2Box').classList.toggle('alarm', r.hot); $('#roSHBox').classList.toggle('alarm', r.wet); $('#roX4Box').classList.toggle('warnv', r.flash); $('#roCOPBox').classList.toggle('warnv', r.COP < 2);
  const ps = $('#pillState'); ps.textContent = r.wet ? 'Liquid slugging' : r.hot ? 'Discharge too hot' : s.mode === 'coldroom' && !s.comp ? 'Compressor off' : `COP ${fmt(r.COP, 2)}`; ps.className = 'pill ' + (r.wet || r.hot ? 'warn' : s.mode === 'coldroom' && !s.comp ? 'amber' : 'ok');
  $('#pillMode').textContent = `${s.mode === 'explorer' ? 'cycle explorer' : 'cold room'} · ${fmt(r.Te, 0)} / ${fmt(r.Tc, 0)} °C · rp ${fmt(r.rp, 1)}`; $('#pillSlug').classList.toggle('hidden', !r.wet); $('#pillHot').classList.toggle('hidden', !r.hot); $('#pillFlash').classList.toggle('hidden', !r.flash);
  $('#pillRun').textContent = s.run ? 'Running' : 'Paused'; $('#pillRun').classList.toggle('on', s.run); $('#btnRun').textContent = s.run ? 'Running' : 'Start';
  $('#rigTitle').textContent = s.mode === 'explorer' ? 'Vapour-compression circuit · R134a · evaporator, compressor, condenser, expansion valve' : 'Cold room · the evaporator sits in the room, the condenser in the ambient, a thermostat runs the compressor';
}
function record(tag = '') { const s = state, r = R, row = { i: s.log.length + 1, tag, mode: s.mode, Te: +r.Te.toFixed(1), Tc: +r.Tc.toFixed(1), SH: +r.SH.toFixed(1), SC: +r.SC.toFixed(1), eta: s.eta, Vd: s.Vd, charge: s.charge, foul: s.foul, rp: +r.rp.toFixed(2), etav: +r.etav.toFixed(2), m: +(r.m * 1000).toFixed(1), Qe: +r.Qe.toFixed(2), W: +r.W.toFixed(2), COP: +r.COP.toFixed(2), COPc: +r.COPc.toFixed(1), T2: +r.T2.toFixed(0), x4: +r.x4.toFixed(2), qe: +r.qe.toFixed(1), w: +r.w.toFixed(1), Troom: s.mode === 'coldroom' ? +s.Troom.toFixed(1) : '' }; s.log.push(row); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.mode, r.Te, r.Tc, r.SH, r.SC, r.eta, r.charge, r.foul, r.rp, r.etav, r.m, r.Qe, r.W, r.COP, r.COPc, r.T2, r.x4, r.Troom].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,mode,Te_C,Tc_C,superheat_K,subcool_K,eta_is,Vd_m3h,charge_pct,fouling_K,rp,eta_vol,mflow_gs,Qe_kW,W_kW,COP,COP_carnot,T2_C,x4,qe_kJkg,w_kJkg,Troom_C\n' + state.log.map(r => [r.i, r.tag, r.mode, r.Te, r.Tc, r.SH, r.SC, r.eta, r.Vd, r.charge, r.foul, r.rp, r.etav, r.m, r.Qe, r.W, r.COP, r.COPc, r.T2, r.x4, r.qe, r.w, r.Troom].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const same = (a, b, k) => k.every(x => a[x] === b[x]);
const TASKS = [
  { id: 'base', t: 'Read the cycle', d: 'Record the −10 / 40 °C cycle: capacity, power, COP, and how far it sits below the Carnot COP.', ok: () => state.log.some(x => x.mode === 'explorer') },
  { id: 'condens', t: 'Condensing temperature', d: 'Record the same evaporating temperature with condensing temperatures 15 K apart: a hot condenser costs COP and capacity.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.Te === b.Te && b.Tc - a.Tc >= 15 && b.COP < a.COP)); } },
  { id: 'evap', t: 'Evaporating temperature', d: 'Record evaporating temperatures 20 K apart at the same condensing temperature: capacity collapses at low temperature because the suction gas is thin.', ok: () => { const L = state.log; return L.some(a => L.some(b => a.Tc === b.Tc && a.Te - b.Te >= 20 && b.Qe < 0.7 * a.Qe)); } },
  { id: 'slug', t: 'No superheat', d: 'Set the superheat to zero (or overcharge) and see the suction state land inside the dome: liquid slugging.', ok: () => state.flags.slug },
  { id: 'hot', t: 'Discharge too hot', d: 'Push the pressure ratio up (deep freeze evaporating, hot condenser) until the discharge passes 120 °C.', ok: () => state.flags.hot },
  { id: 'subcool', t: 'Subcooling adds capacity', d: 'Record 0 K and 10 K of subcooling at the same temperatures: the extra liquid enthalpy goes straight into capacity at no compressor cost.', ok: () => { const L = state.log; return L.some(a => L.some(b => same(a, b, ['Te', 'Tc', 'SH']) && a.SC <= 0.5 && b.SC >= 9 && b.Qe > a.Qe)); } },
  { id: 'pulldown', t: 'Pull the room down', d: 'Cold-room mode: start at ambient and pull the room to its set point. Read how the capacity falls as the evaporating temperature drops.', ok: () => state.flags.pulldown },
  { id: 'cycling', t: 'Thermostat cycling', d: 'Let the cold room cycle on the thermostat at least three times and read the run fraction from the shaded trace.', ok: () => state.flags.cycling },
  { id: 'foul', t: 'Fouled condenser', d: 'In the cold room add 8 K or more of condenser fouling and record: condensing temperature, power and discharge temperature rise, capacity falls.', ok: () => state.log.some(x => x.mode === 'coldroom' && x.foul >= 8) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; step(dt); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawTrace(); if (frame % 6 === 0) { drawPh(); drawSweep(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setMode(m) { state.mode = m; document.querySelectorAll('#types button').forEach(b => b.classList.toggle('on', b.dataset.type === m)); document.querySelectorAll('[data-m]').forEach(e => e.classList.toggle('off', !e.dataset.m.split(' ').includes(m))); if (m === 'coldroom') { state.Troom = state.Tamb; state.comp = true; state.cycles = 0; state.pulled = false; state.trace = []; } }
const PRESETS = {
  base: () => { setMode('explorer'); for (const [k, v] of Object.entries({ Te: -10, Tc: 40, SH: 5, SC: 3, eta: 0.7, Vd: 20, clr: 4, charge: 100 })) setCtl(k, v); state.run = true; },
  hot: () => { PRESETS.base(); setCtl('Tc', 55); },
  deep: () => { PRESETS.base(); setCtl('Te', -30); },
  slug: () => { PRESETS.base(); setCtl('SH', 0); },
  overheat: () => { PRESETS.base(); setCtl('Te', -35); setCtl('Tc', 60); setCtl('eta', 0.55); },
  lowcharge: () => { PRESETS.base(); setCtl('charge', 60); },
  room: () => { PRESETS.base(); for (const [k, v] of Object.entries({ Tamb: 30, SPr: -18, load: 3, UA: 0.25, Cth: 800, foul: 0, dTe: 8, dTc: 12, tscale: 30 })) setCtl(k, v); setMode('coldroom'); },
  roomhot: () => { PRESETS.room(); setCtl('Tamb', 42); setCtl('foul', 10); },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('Te', v => v + ' °C'); bindR('Tc', v => v + ' °C'); bindR('SH', v => v + ' K'); bindR('SC', v => v + ' K'); bindR('eta', v => v.toFixed(2)); bindR('Vd', v => v + ' m³/h'); bindR('clr', v => v + ' %'); bindR('charge', v => v + ' %'); bindR('Tamb', v => v + ' °C'); bindR('SPr', v => v + ' °C'); bindR('load', v => v.toFixed(1) + ' kW'); bindR('UA', v => v.toFixed(2) + ' kW/K'); bindR('Cth', v => v + ' kJ/K'); bindR('foul', v => '+' + v + ' K'); bindR('dTe', v => v + ' K'); bindR('dTc', v => v + ' K'); bindR('tscale', v => '×' + v);
  document.querySelectorAll('#types button').forEach(b => b.onclick = () => setMode(b.dataset.type));
  $('#btnRun').onclick = () => state.run = true; $('#btnStop').onclick = () => state.run = false; $('#btnRecord').onclick = () => record(''); $('#btnWarm').onclick = () => { state.Troom = state.Tamb; state.cycles = 0; state.pulled = false; state.trace = []; };
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('rf_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase(); if (e.key === ' ') { e.preventDefault(); state.run = !state.run; } if (k === 'R') record(''); if (k === '1') setMode('explorer'); if (k === '2') setMode('coldroom'); if (e.key === 'Escape') $('#help').classList.add('hidden'); });
}

/* ------------------------------------------------------------------ dashboard API */
window.RefrigerationCycle = {
  getState: () => ({ inputs: { mode: state.mode, Te: state.Te, Tc: state.Tc, SH: state.SH, SC: state.SC, eta: state.eta, Vd: state.Vd, clr: state.clr, charge: state.charge, Tamb: state.Tamb, SPr: state.SPr, load: state.load, UA: state.UA, Cth: state.Cth, foul: state.foul, dTe: state.dTe, dTc: state.dTc, tscale: state.tscale, run: state.run }, results: { Te: R.Te, Tc: R.Tc, rp: R.rp, etav: R.etav, m: R.m, Qe: R.Qe, W: R.W, COP: R.COP, COPc: R.COPc, T2: R.T2, x4: R.x4, wet: R.wet, hot: R.hot, Troom: state.Troom, comp: state.comp, cycles: state.cycles }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { if (inp.mode) setMode(inp.mode); for (const k of ['Te', 'Tc', 'SH', 'SC', 'eta', 'Vd', 'clr', 'charge', 'Tamb', 'SPr', 'load', 'UA', 'Cth', 'foul', 'dTe', 'dTc', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.warm) $('#btnWarm').click(); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'refrigeration-cycle', title: 'Refrigeration Cycle Trainer', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.RefrigerationCycle.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'refrigeration-cycle', state: window.RefrigerationCycle.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('rf_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); setMode('explorer'); step(0); drawRig(); updateReadouts(); drawPh(); drawSweep(); drawTrace();
  requestAnimationFrame(tick);
}
build();
