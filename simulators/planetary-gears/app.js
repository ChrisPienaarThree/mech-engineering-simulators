/* Planetary Gear Demonstrator — epicyclic train kinematics from the Willis equation, torque balance from tooth
   counts, planets drawn with teeth kept in phase with the sun mesh so the animation meshes. Sun, carrier and ring
   can each be held or driven; two inputs make a differential. Standalone, offline. Data illustrative. */
'use strict';

const $ = s => document.querySelector(s), clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const TAU = Math.PI * 2, RPM = 60 / TAU;

/* ------------------------------------------------------------------ state */
const state = { Ns: 24, Nr: 72, np: 3, hold: 'ring', drive: 'sun', rpm: 60, rpm2: 0, T: 10, eta: 98, tscale: 0.2, trace: true, run: false, th: { s: 0, c: 0, r: 0, p: 0 }, path: [], log: [], flags: {}, hist: [], t: 0 };
const R = {};

/* ------------------------------------------------------------------ kinematics and torques */
function solve() {
  const { Ns, Nr, hold } = state, np = Math.max(1, state.np | 0), Np = (Nr - Ns) / 2, sum = Ns + Nr; let ws, wc, wr, drive = state.drive;
  const win = state.rpm / RPM, w2 = state.rpm2 / RPM;
  if (hold === 'lock') { ws = wc = wr = win; drive = 'locked train'; }
  else if (hold === 'none') { const d1 = drive, d2 = drive === 'sun' ? 'ring' : drive === 'ring' ? 'sun' : 'ring'; const v = { [d1]: win, [d2]: w2 }; if (v.sun !== undefined && v.ring !== undefined) { ws = v.sun; wr = v.ring; wc = (ws * Ns + wr * Nr) / sum; } else { wc = v.carrier; if (v.sun !== undefined) { ws = v.sun; wr = (sum * wc - ws * Ns) / Nr; } else { wr = v.ring; ws = (sum * wc - wr * Nr) / Ns; } } R.second = d2; }
  else { if (hold === drive) drive = hold === 'ring' ? 'sun' : hold === 'sun' ? 'ring' : 'sun'; const v = { sun: 0, carrier: 0, ring: 0 }; v[drive] = win; v[hold] = 0; const free = ['sun', 'carrier', 'ring'].find(m => m !== hold && m !== drive); if (free === 'carrier') v.carrier = (v.sun * Ns + v.ring * Nr) / sum; else if (free === 'sun') v.sun = (sum * v.carrier - v.ring * Nr) / Ns; else v.ring = (sum * v.carrier - v.sun * Ns) / Nr; ws = v.sun; wc = v.carrier; wr = v.ring; R.out = free; }
  const wp = wc - (ws - wc) * Ns / Np;   // planet spin about its own axis (absolute)
  const wIn = drive === 'locked train' ? win : (drive === 'sun' ? ws : drive === 'carrier' ? wc : wr), wOut = hold === 'lock' ? win : hold === 'none' ? wc : ({ sun: ws, carrier: wc, ring: wr })[R.out];
  const ratio = Math.abs(wOut) > 1e-9 ? wIn / wOut : Infinity;
  // torques: Ts : Tr : Tc = Ns : Nr : −(Ns+Nr), scaled so the driven member carries the input torque
  const unit = { sun: Ns, ring: Nr, carrier: -sum }, Tin = state.T, tIn = hold === 'lock' ? Tin : Tin, scale = hold === 'lock' ? 0 : Tin / Math.abs(unit[drive === 'locked train' ? 'sun' : drive]);
  const T = hold === 'lock' ? { sun: 0, ring: 0, carrier: 0 } : { sun: unit.sun * scale, ring: unit.ring * scale, carrier: unit.carrier * scale };
  const sgn = Math.sign(wIn) || 1; for (const k in T) T[k] *= sgn * (drive === 'locked train' ? 1 : Math.sign(unit[drive]));   // input torque in the direction of the input speed
  const meshes = hold === 'lock' ? 0 : 2, eta = Math.pow(state.eta / 100, meshes), Pin = Math.abs(T[drive === 'locked train' ? 'sun' : drive] * wIn), Pout = Pin * eta;
  const asm = Number.isInteger(sum / np) && Np >= 6, tipR = Np / 2 + 1, gap = np > 1 ? 2 * ((Ns + Np) / 2) * Math.sin(Math.PI / np) - 2 * tipR : 99;
  return { Ns, Nr, Np, ws, wc, wr, wp, drive, ratio, T, Pin, Pout, eta, asm, clash: gap < 0.5, days: Math.abs(wc) > 1e-9 ? (wp - wc) / wc : Infinity, mesh: Ns * Math.abs(ws - wc) / TAU, wIn, wOut };
}

/* ------------------------------------------------------------------ gear drawing */
const NS = 'http://www.w3.org/2000/svg', el = (t, a = {}, p) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const txt = (x, y, s, cls, p) => { const t = el('text', { x, y, class: cls }, p); t.textContent = s; return t; };
let svg, gGears, gTrace, gLabels;
function gearPath(N, m, internal) {   // trapezoid teeth; pitch radius m·N/2; tooth 0 centred on angle 0
  const rp = m * N / 2, ra = internal ? rp - m : rp + m, rf = internal ? rp + 1.25 * m : rp - 1.25 * m, p = TAU / N, pts = [];
  for (let i = 0; i < N; i++) { const a = i * p; const tw = p * 0.25, bw = p * 0.32; pts.push([rf, a - bw - p * 0.5 + p * 0.5 - bw * 0], [rf, a - bw], [ra, a - tw], [ra, a + tw], [rf, a + bw]); }
  let d = ''; pts.forEach(([r, a], i) => { d += (i ? 'L' : 'M') + (r * Math.cos(a)).toFixed(2) + ',' + (r * Math.sin(a)).toFixed(2) + ' '; }); return d + 'Z';
}
function buildRig() { svg = el('svg', { viewBox: '-320 -320 640 640', role: 'img', 'aria-label': 'Planetary gearset' }); $('#rig').innerHTML = ''; $('#rig').appendChild(svg); gTrace = el('g', {}, svg); gGears = el('g', {}, svg); gLabels = el('g', {}, svg); }
function drawRig() {
  gGears.innerHTML = ''; gLabels.innerHTML = ''; const k = R, m = 480 / (k.Nr + 8), rs = m * k.Ns / 2, rp = m * k.Np / 2, rr = m * k.Nr / 2, rc = rs + rp, th = state.th;
  const deg = a => a * 180 / Math.PI, held = x => state.hold === x || state.hold === 'lock' && (x === 'sun' || x === 'ring');
  // planet phases: each planet's gap faces the nearest sun tooth; the ring's tooth pattern is then derived from planet 0 (it rotates at exactly ωr by the Willis equation)
  const ps = TAU / k.Ns, pp = TAU / k.Np, pr = TAU / k.Nr, fold = (a, p) => { a = ((a % p) + p) % p; return a > p / 2 ? a - p : a; };
  const nP = Math.max(1, state.np | 0), planets = []; for (let i = 0; i < nP; i++) { const phi = th.c + i * TAU / nP, e = fold(phi - th.s, ps), thp = phi + Math.PI - pp / 2 - e * k.Ns / k.Np; planets.push({ phi, thp, cx: rc * Math.cos(phi), cy: rc * Math.sin(phi) }); }
  const ep0 = fold(planets[0].phi - planets[0].thp, pp), ringA = planets[0].phi + ep0 * k.Np / k.Nr - pr / 2; R.thp0 = planets[0].thp; R.ringA = ringA;
  // ring: outer body with internal teeth cut out
  el('path', { d: `M${rr + 4 * m},0 A${rr + 4 * m},${rr + 4 * m} 0 1 0 ${-(rr + 4 * m)},0 A${rr + 4 * m},${rr + 4 * m} 0 1 0 ${rr + 4 * m},0 Z ` + gearPath(k.Nr, m, true), class: 'ring' + (held('ring') ? ' held' : ''), transform: `rotate(${deg(ringA)})` }, gGears);
  for (let i = 0; i < 4; i++) el('line', { x1: (rr + 1.4 * m) * Math.cos(i * Math.PI / 2 + ringA), y1: (rr + 1.4 * m) * Math.sin(i * Math.PI / 2 + ringA), x2: (rr + 3.6 * m) * Math.cos(i * Math.PI / 2 + ringA), y2: (rr + 3.6 * m) * Math.sin(i * Math.PI / 2 + ringA), class: 'spoke ring' }, gGears);
  // carrier
  const gc = el('g', { transform: `rotate(${deg(th.c)})`, class: held('carrier') ? 'held' : '' }, gGears); for (let i = 0; i < nP; i++) { const a = i * TAU / nP; el('line', { x1: 0, y1: 0, x2: rc * Math.cos(a), y2: rc * Math.sin(a), class: 'carrier' + (held('carrier') ? ' held' : '') }, gc); } el('circle', { r: Math.max(6, rs * 0.35), class: 'carrier-hub' }, gc);
  // sun
  el('path', { d: gearPath(k.Ns, m, false), class: 'sun' + (held('sun') ? ' held' : ''), transform: `rotate(${deg(th.s)})` }, gGears); el('line', { x1: 0, y1: 0, x2: rs * 0.8 * Math.cos(th.s), y2: rs * 0.8 * Math.sin(th.s), class: 'spoke sun' }, gGears);
  // planets
  for (let i = 0; i < nP; i++) {
    const { thp, cx, cy } = planets[i];
    el('path', { d: gearPath(k.Np, m, false), class: 'planet', transform: `translate(${cx.toFixed(2)},${cy.toFixed(2)}) rotate(${deg(thp)})` }, gGears);
    el('circle', { cx, cy, r: Math.max(4, rp * 0.18), class: 'pin' }, gGears); el('line', { x1: cx, y1: cy, x2: cx + rp * 0.8 * Math.cos(thp), y2: cy + rp * 0.8 * Math.sin(thp), class: 'spoke planet' }, gGears);
    if (i === 0) { const mx = cx + rp * Math.cos(thp), my = cy + rp * Math.sin(thp); el('circle', { cx: mx, cy: my, r: 5, class: 'marker' }, gGears); if (state.trace && state.run) { state.path.push([mx, my]); if (state.path.length > 2500) state.path.shift(); } }
  }
  if (state.path.length > 1) el('polyline', { points: state.path.map(p => p[0].toFixed(1) + ',' + p[1].toFixed(1)).join(' '), class: 'trace' }, gTrace); else gTrace.innerHTML = '';
  if (state.hold === 'lock') { el('rect', { x: rs + 2, y: -4, width: rr - rs - 2, height: 8, class: 'lock', transform: `rotate(${deg(th.s)})` }, gGears); }
  // labels and direction arrows
  const arrow = (r, w, cls, label) => { if (Math.abs(w) < 1e-6) { txt(0, -r - 6, label + ' held', 'glabel sub', gLabels); return; } const a0 = -Math.PI / 2 - 0.35, a1 = -Math.PI / 2 + 0.35, dir = w > 0 ? 1 : 0; const sa = dir ? a0 : a1, ea = dir ? a1 : a0; el('path', { d: `M${r * Math.cos(sa)},${r * Math.sin(sa)} A${r},${r} 0 0 ${dir} ${r * Math.cos(ea)},${r * Math.sin(ea)}`, class: 'arrow ' + cls }, gLabels); const hx = r * Math.cos(ea), hy = r * Math.sin(ea), t = ea + (dir ? Math.PI / 2 : -Math.PI / 2); el('polygon', { points: `${hx},${hy} ${hx - 9 * Math.cos(t - 0.5)},${hy - 9 * Math.sin(t - 0.5)} ${hx - 9 * Math.cos(t + 0.5)},${hy - 9 * Math.sin(t + 0.5)}`, class: 'ahead', fill: 'currentColor', style: `color:var(--${cls === 'sun' ? 'brass' : cls === 'carrier' ? 'good' : 'steel'})` }, gLabels); txt(0, -r - 6, `${label} ${fmt(w * RPM, 0)} rpm`, 'glabel sub', gLabels); };
  arrow(rs * 0.55, k.ws, 'sun', 'sun'); arrow(rc + rp * 0.55 + 4, k.wc, 'carrier', 'carrier'); arrow(rr + 2.6 * m, k.wr, 'ring', 'ring');
  txt(0, rr + 4 * m + 16, `Ns ${k.Ns} · Np ${k.Np} · Nr ${k.Nr} · ${state.np} planet${state.np > 1 ? 's' : ''} · module scaled to fit`, 'glabel sub', gLabels);
  if (!k.asm) txt(0, rr + 4 * m + 30, `(Ns + Nr) / planets = ${((k.Ns + k.Nr) / state.np).toFixed(2)} — not a whole number: the planets cannot all mesh`, 'warn-txt', gLabels);
  else if (k.clash) txt(0, rr + 4 * m + 30, 'adjacent planets overlap — fewer planets or a larger ring', 'warn-txt', gLabels);
}

/* ------------------------------------------------------------------ charts */
const cs = () => getComputedStyle(document.documentElement), col = n => cs().getPropertyValue(n).trim();
function drawNomo() {
  const cv = $('#nomo'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const k = R, L = 90, Rr = 90, T = 24, B = 40, pw = W - L - Rr, ph = H - T - B;
  const xs = L, xr = L + pw, xc = L + pw * k.Ns / (k.Ns + k.Nr), vmax = Math.max(60, Math.abs(k.ws), Math.abs(k.wc), Math.abs(k.wr)) * RPM * 1.15, Y = w => T + ph / 2 - (w * RPM) / vmax * ph / 2;
  ctx.fillStyle = col('--panel2'); ctx.fillRect(L - 30, T, pw + 60, ph); ctx.strokeStyle = col('--steel'); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(L - 30, Y(0)); ctx.lineTo(L + pw + 30, Y(0)); ctx.stroke();
  for (const [x, name, c] of [[xs, 'SUN', '--brass'], [xc, 'CARRIER', '--good'], [xr, 'RING', '--steel']]) { ctx.strokeStyle = col(c); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke(); ctx.fillStyle = col(c); ctx.font = '600 13px Barlow Condensed, sans-serif'; ctx.textAlign = 'center'; ctx.fillText(name, x, H - 18); }
  ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText(`Ns ${k.Ns}`, (xs + xc) / 2, H - 6); ctx.fillText(`Nr ${k.Nr}`, (xc + xr) / 2, H - 6); ctx.textAlign = 'right'; ctx.fillText(`+${fmt(vmax, 0)} rpm`, L - 34, T + 10); ctx.fillText('0', L - 34, Y(0) + 3); ctx.fillText(`−${fmt(vmax, 0)}`, L - 34, T + ph);
  ctx.strokeStyle = col('--alarm'); ctx.lineWidth = 2.5; ctx.beginPath(); const y0 = Y(k.ws), y1 = Y(k.wr); const ext = 0.12; ctx.moveTo(xs - ext * pw, y0 - (y1 - y0) * ext); ctx.lineTo(xr + ext * pw, y1 + (y1 - y0) * ext); ctx.stroke();
  for (const [x, w, c] of [[xs, k.ws, '--brass'], [xc, k.wc, '--good'], [xr, k.wr, '--steel']]) { ctx.fillStyle = col(c); ctx.beginPath(); ctx.arc(x, Y(w), 7, 0, 7); ctx.fill(); ctx.strokeStyle = col('--panel'); ctx.lineWidth = 2; ctx.stroke(); ctx.fillStyle = col('--ink'); ctx.font = '600 12px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`${fmt(w * RPM, 0)} rpm`, x + 12, Y(w) + 4); }
  ctx.fillStyle = col('--muted'); ctx.font = '11px IBM Plex Sans, sans-serif'; ctx.textAlign = 'left'; ctx.fillText(state.hold === 'none' ? 'no member held: the line pivots freely — a differential' : state.hold === 'lock' ? 'sun locked to ring: the line is horizontal, everything at one speed' : `${state.hold} held at zero: the line pivots about it`, L - 30, T + 14);
}
function drawSpeeds() {
  const cv = $('#speeds'), ctx = cv.getContext('2d'), W = cv.width, H = cv.height; ctx.clearRect(0, 0, W, H); const k = R, L = 150, Rr = 20, T = 12, pw = W - L - Rr, rows = [['Sun', k.ws * RPM, k.T.sun, '--brass'], ['Carrier', k.wc * RPM, k.T.carrier, '--good'], ['Ring', k.wr * RPM, k.T.ring, '--steel'], ['Planet spin (abs.)', k.wp * RPM, NaN, '--water'], ['Planet rel. carrier', (k.wp - k.wc) * RPM, NaN, '--water-deep']];
  const vmax = Math.max(1, ...rows.map(r => Math.abs(r[1]))), tmax = Math.max(1, ...rows.map(r => Math.abs(r[2]) || 0)), rowH = (H - T - 30) / rows.length, half = pw / 2;
  ctx.font = '12px IBM Plex Sans, sans-serif';
  rows.forEach(([name, w, t, c], i) => { const y = T + i * rowH; ctx.fillStyle = col('--ink'); ctx.textAlign = 'right'; ctx.fillText(name, L - 10, y + rowH * 0.62); ctx.fillStyle = col('--panel2'); ctx.fillRect(L, y + 4, pw, rowH - 8); ctx.fillStyle = col(c); const wpx = w / vmax * (half - 4); ctx.fillRect(L + half + Math.min(0, wpx), y + 6, Math.abs(wpx), rowH * 0.45 - 6); if (Number.isFinite(t)) { ctx.globalAlpha = 0.45; const tpx = t / tmax * (half - 4); ctx.fillRect(L + half + Math.min(0, tpx), y + rowH * 0.5, Math.abs(tpx), rowH * 0.45 - 6); ctx.globalAlpha = 1; } ctx.fillStyle = col('--muted'); ctx.font = '11px IBM Plex Mono, monospace'; ctx.textAlign = 'left'; ctx.fillText(`${fmt(w, 1)} rpm${Number.isFinite(t) ? ` · ${fmt(t, 1)} Nm` : ''}`, L + half + (w >= 0 ? 8 + Math.abs(wpx) : 8), y + rowH * 0.42); ctx.font = '12px IBM Plex Sans, sans-serif'; });
  ctx.strokeStyle = col('--steel'); ctx.beginPath(); ctx.moveTo(L + half, T); ctx.lineTo(L + half, H - 26); ctx.stroke(); ctx.fillStyle = col('--muted'); ctx.font = '10px IBM Plex Mono, monospace'; ctx.textAlign = 'center'; ctx.fillText('← clockwise negative · anticlockwise positive → · solid = speed, faint = torque', L + half, H - 8);
}
function renderTabular() {
  const k = R, tb = $('#tabular tbody'); const x = k.wIn * RPM;
  if (state.hold === 'none' || state.hold === 'lock') { tb.innerHTML = `<tr><td>Willis</td><td>${fmt(k.ws * RPM, 1)}</td><td>${fmt(k.wc * RPM, 1)}</td><td>${fmt(k.wr * RPM, 1)}</td><td>${fmt(k.wp * RPM, 1)}</td></tr><tr class="sum"><td>${state.hold === 'lock' ? 'all locked together' : 'carrier = (Ns·ωs + Nr·ωr)/(Ns+Nr)'}</td><td colspan="4"></td></tr>`; return; }
  const rows = []; const y = k.wc * RPM;   // step 1: all locked, rotate y (carrier speed); step 2: carrier held, sun turns x2
  const x2 = k.ws * RPM - y; rows.push(['1 · lock the train, turn it y', y, y, y, y]); rows.push(['2 · hold the carrier, turn the sun x', x2, 0, -x2 * k.Ns / k.Nr, -x2 * k.Ns / k.Np]); rows.push(['Sum (held member = 0)', y + x2, y, y - x2 * k.Ns / k.Nr, y - x2 * k.Ns / k.Np]);
  tb.innerHTML = rows.map((r, i) => `<tr${i === 2 ? ' class="sum"' : ''}><td>${r[0]}</td>${r.slice(1).map(v => `<td>${fmt(v, 1)}</td>`).join('')}</tr>`).join('') + `<tr><td>ratio in/out</td><td colspan="4">${Number.isFinite(k.ratio) ? fmt(k.ratio, 3) : '∞'} · ${k.drive} in, ${R.out} out${k.ratio < 0 ? ' · reversed' : ''}</td></tr>`;
}

/* ------------------------------------------------------------------ readouts, log, tasks */
const dir = w => Math.abs(w) < 1e-9 ? 'held' : w > 0 ? 'anticlockwise' : 'clockwise';
function updateReadouts() {
  const k = R; $('#roS').textContent = fmt(k.ws * RPM, 1); $('#roSd').textContent = dir(k.ws); $('#roTs').textContent = fmt(k.T.sun, 1); $('#roC').textContent = fmt(k.wc * RPM, 1); $('#roCd').textContent = dir(k.wc); $('#roTc').textContent = fmt(k.T.carrier, 1); $('#roR').textContent = fmt(k.wr * RPM, 1); $('#roRd').textContent = dir(k.wr); $('#roTr').textContent = fmt(k.T.ring, 1);
  $('#roP').textContent = fmt(k.wp * RPM, 1); $('#roPrel').textContent = fmt((k.wp - k.wc) * RPM, 1); $('#roRatio').textContent = Number.isFinite(k.ratio) ? fmt(k.ratio, 3) : '∞'; $('#roRev').textContent = k.ratio < 0 ? 'output reversed' : 'same direction'; $('#roDays').textContent = Number.isFinite(k.days) ? fmt(k.days, 2) : '—';
  $('#roPin').textContent = fmt(k.Pin, 1); $('#roPout').textContent = fmt(k.Pout, 1); $('#roEta').textContent = fmt(k.eta * 100, 1); $('#roMesh').textContent = fmt(k.mesh, 1); $('#roNp').textContent = k.Np;
  $('#pillRun').textContent = state.run ? 'Running' : 'Stopped'; $('#pillRun').classList.toggle('on', state.run);
  const pm = $('#pillMode'); pm.textContent = state.hold === 'lock' ? 'Locked · 1 : 1' : state.hold === 'none' ? `Differential · ${k.drive} + ${R.second} in, carrier out` : `${state.hold} held · ${k.drive} in · ${R.out} out`; pm.className = 'pill ' + (k.asm && !k.clash ? 'ok' : 'warn');
  $('#pillRatio').textContent = `ratio ${Number.isFinite(k.ratio) ? fmt(k.ratio, 2) : '∞'}${k.ratio < 0 ? ' (reversed)' : ''}`; $('#pillAsm').classList.toggle('hidden', k.asm); $('#pillClash').classList.toggle('hidden', !k.clash || !k.asm);
  $('#btnRun').textContent = state.run ? 'Stop' : 'Run';
}
function record(tag = '') { const k = R, r = { i: state.log.length + 1, tag, Ns: k.Ns, Np: k.Np, Nr: k.Nr, np: state.np, hold: state.hold, drive: k.drive, s: +(k.ws * RPM).toFixed(1), c: +(k.wc * RPM).toFixed(1), r: +(k.wr * RPM).toFixed(1), p: +(k.wp * RPM).toFixed(1), ratio: Number.isFinite(k.ratio) ? +k.ratio.toFixed(3) : 'inf', Ts: +k.T.sun.toFixed(1), Tc: +k.T.carrier.toFixed(1), Tr: +k.T.ring.toFixed(1), asm: k.asm && !k.clash }; state.log.push(r); renderLog(); checkTasks(); }
function renderLog() { $('#logTable tbody').innerHTML = state.log.map(r => `<tr>${[r.i, r.tag, r.Ns, r.Np, r.Nr, r.np, r.hold, r.drive, r.s, r.c, r.r, r.p, r.ratio, r.Ts, r.Tc, r.Tr, r.asm ? 'yes' : 'no'].map(v => `<td>${v}</td>`).join('')}</tr>`).join(''); $('#logCount').textContent = state.log.length; state.csv = 'n,tag,Ns,Np,Nr,planets,held,driven,sun_rpm,carrier_rpm,ring_rpm,planet_rpm,ratio,Ts_Nm,Tc_Nm,Tr_Nm,assembles\n' + state.log.map(r => [r.i, r.tag, r.Ns, r.Np, r.Nr, r.np, r.hold, r.drive, r.s, r.c, r.r, r.p, r.ratio, r.Ts, r.Tc, r.Tr, r.asm].join(',')).join('\n'); $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv); }
const TASKS = [
  { id: 'reducer', t: 'Ring held: 1 + Nr/Ns', d: 'Hold the ring, drive the sun. Record it and check the ratio against 1 + Nr/Ns. Which way does the carrier turn?', ok: () => state.log.some(r => r.hold === 'ring' && r.drive === 'sun' && r.asm) },
  { id: 'star', t: 'Carrier held: reversal', d: 'Hold the carrier: the ring turns the other way at −Nr/Ns. Record it.', ok: () => state.log.some(r => r.hold === 'carrier' && r.drive === 'sun') },
  { id: 'over', t: 'Sun held: 1 + Ns/Nr', d: 'Hold the sun and drive the ring: the smallest ratio of the three. Record it.', ok: () => state.log.some(r => r.hold === 'sun' && r.drive === 'ring') },
  { id: 'three', t: 'Three ratios, one gearset', d: 'Record all three held-member cases with the same tooth counts.', ok: () => { const L = state.log; return ['ring', 'carrier', 'sun'].every(h => L.some(r => r.hold === h && r.Ns === state.Ns && r.Nr === state.Nr)); } },
  { id: 'stall', t: 'Stop the carrier with two inputs', d: 'Hold nothing; drive sun and ring so that the carrier stands still: ωs·Ns = −ωr·Nr.', ok: () => state.flags.stall },
  { id: 'days', t: 'Days per year', d: 'With the ring held, note how many times the planet spins per orbit; change Ns and see it change.', ok: () => { const L = state.log.filter(r => r.hold === 'ring' && r.drive === 'sun'); return new Set(L.map(r => r.Ns)).size >= 2; } },
  { id: 'torque', t: 'Torque balance', d: 'Record a ring-held case and check Ts + Tc + Tr = 0 and Tc/Ts = −(1 + Nr/Ns).', ok: () => state.log.some(r => r.hold === 'ring' && Math.abs(r.Ts + r.Tc + r.Tr) < 0.2 && r.Ts !== 0) },
  { id: 'asm', t: 'Make it assemble', d: 'Set four planets with tooth counts that fail the assembly check, then change the ring so they fit.', ok: () => state.flags.noasm && state.flags.asm4 },
  { id: 'big', t: 'Ten to one in one stage', d: 'Find tooth counts that give a ratio of 10 or more with the ring held and at least three planets that assemble.', ok: () => state.flags.big },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() {
  const k = R, f = state.flags; if (state.hold === 'none' && Math.abs(k.wc) < 0.05 && Math.abs(k.ws) > 1 && Math.abs(k.wr) > 1) f.stall = true; if (state.np === 4 && !k.asm) f.noasm = true; if (state.np === 4 && k.asm && !k.clash && f.noasm) f.asm4 = true; if (state.hold === 'ring' && k.drive === 'sun' && k.ratio >= 10 && state.np >= 3 && k.asm && !k.clash) f.big = true;
  for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; }
}

/* ------------------------------------------------------------------ loop and wiring */
let last = performance.now(), frame = 0;
function advance(dt) { const k = R, th = state.th; th.s += k.ws * dt; th.c += k.wc * dt; th.r += k.wr * dt; th.p += k.wp * dt; }
function tick(now) { const dt = clamp((now - last) / 1000, 0, 0.1); last = now; Object.assign(R, solve()); if (state.run) advance(dt * state.tscale); drawRig(); frame++; if (frame % 2 === 0) { updateReadouts(); drawSpeeds(); if (frame % 6 === 0) { drawNomo(); renderTabular(); checkTasks(); } } requestAnimationFrame(tick); }
function setCtl(id, v) { const e = $('#' + id); if (!e) return; if (e.type === 'checkbox') e.checked = !!v; else e.value = v; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); }
function setHold(h) { state.hold = h; if (h === state.drive && h !== 'none' && h !== 'lock') state.drive = h === 'ring' ? 'sun' : h === 'sun' ? 'ring' : 'sun'; syncDrive(); }
function setDrive(d) { if (d === state.hold && state.hold !== 'none' && state.hold !== 'lock') return; state.drive = d; syncDrive(); }
function syncDrive() { document.querySelectorAll('[data-hold]').forEach(b => b.classList.toggle('on', b.dataset.hold === state.hold)); document.querySelectorAll('[data-in]').forEach(b => { b.classList.toggle('on', b.dataset.in === state.drive); b.disabled = b.dataset.in === state.hold && state.hold !== 'none'; }); $('#ctlRpm2').style.display = state.hold === 'none' ? '' : 'none'; $('#rpmName').textContent = (state.hold === 'lock' ? 'Train' : state.drive) + ' speed'; const second = state.drive === 'sun' ? 'ring' : state.drive === 'ring' ? 'sun' : 'ring'; $('#rpm2Name').textContent = `Second input (${second})`; state.path = []; }
const PRESETS = {
  reducer: () => { setCtl('Ns', 24); setCtl('Nr', 72); setCtl('np', 3); setHold('ring'); setDrive('sun'); setCtl('rpm', 60); state.run = true; },
  star: () => { setCtl('Ns', 24); setCtl('Nr', 72); setCtl('np', 3); setHold('carrier'); setDrive('sun'); setCtl('rpm', 60); state.run = true; },
  over: () => { setCtl('Ns', 24); setCtl('Nr', 72); setCtl('np', 3); setHold('sun'); setDrive('ring'); setCtl('rpm', 60); state.run = true; },
  diff: () => { setCtl('Ns', 24); setCtl('Nr', 72); setCtl('np', 3); setHold('none'); setDrive('sun'); setCtl('rpm', 60); setCtl('rpm2', 30); state.run = true; },
  stall: () => { setCtl('Ns', 24); setCtl('Nr', 72); setCtl('np', 3); setHold('none'); setDrive('sun'); setCtl('rpm', 90); setCtl('rpm2', -30); state.run = true; },
  lock: () => { setHold('lock'); setCtl('rpm', 60); state.run = true; },
  big: () => { setCtl('Ns', 12); setCtl('Nr', 108); setCtl('np', 3); setHold('ring'); setDrive('sun'); setCtl('rpm', 200); state.run = true; },
  noasm: () => { setCtl('Ns', 22); setCtl('Nr', 72); setCtl('np', 4); setHold('ring'); setDrive('sun'); setCtl('rpm', 60); state.run = true; },
};
function bind() {
  const bindR = (id, f) => { const e = $('#' + id), o = $('#' + id + 'Out'); const set = () => { state[id] = +e.value; if (o) o.textContent = f(state[id]); }; e.addEventListener('input', set); set(); };
  bindR('Ns', v => String(v)); bindR('Nr', v => String(v)); bindR('np', v => String(v)); bindR('rpm', v => v + ' rpm'); bindR('rpm2', v => v + ' rpm'); bindR('T', v => v + ' Nm'); bindR('eta', v => v.toFixed(1) + ' %'); bindR('tscale', v => '×' + v.toFixed(2));
  $('#Ns').addEventListener('input', () => { if ((state.Nr - state.Ns) % 2) setCtl('Nr', state.Nr + 1); if (state.Nr - state.Ns < 12) setCtl('Nr', state.Ns + 12); state.path = []; }); $('#Nr').addEventListener('input', () => { if (state.Nr - state.Ns < 12) setCtl('Ns', state.Nr - 12); state.path = []; }); $('#np').addEventListener('input', () => state.path = []);
  $('#trace').addEventListener('change', e => { state.trace = e.target.checked; state.path = []; });
  document.querySelectorAll('[data-hold]').forEach(b => b.onclick = () => setHold(b.dataset.hold)); document.querySelectorAll('[data-in]').forEach(b => b.onclick = () => setDrive(b.dataset.in));
  $('#btnRun').onclick = () => state.run = !state.run; $('#btnStep').onclick = () => { Object.assign(R, solve()); const ws = Math.abs(R.wIn) > 1e-9 ? R.wIn : 1; advance((TAU / state.Ns) / Math.abs(R.ws || ws) * Math.sign(1)); }; $('#btnRecord').onclick = () => record(''); $('#btnClearTrace').onclick = () => state.path = [];
  document.querySelectorAll('[data-preset]').forEach(b => b.onclick = () => PRESETS[b.dataset.preset]());
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } }; $('#btnClear').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('pg_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName) && e.target.type !== 'checkbox') return; const k = e.key.toUpperCase(); if (e.key === ' ') state.run = !state.run; else if (e.key === '.') $('#btnStep').click(); else if (k === 'R') record(''); else if (/^[1-3]$/.test(k)) setHold(['ring', 'carrier', 'sun'][+k - 1]); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
  window.addEventListener('resize', () => { drawNomo(); drawSpeeds(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.PlanetaryGears = {
  getState: () => ({ inputs: { Ns: state.Ns, Nr: state.Nr, planets: state.np, hold: state.hold, drive: state.drive, rpm: state.rpm, rpm2: state.rpm2, T: state.T, eta: state.eta, run: state.run }, results: { Np: R.Np, sun_rpm: R.ws * RPM, carrier_rpm: R.wc * RPM, ring_rpm: R.wr * RPM, planet_rpm: R.wp * RPM, ratio: R.ratio, Ts: R.T.sun, Tc: R.T.carrier, Tr: R.T.ring, Pin: R.Pin, Pout: R.Pout, assembles: R.asm && !R.clash, days: R.days }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }),
  setState: inp => { for (const k of ['Ns', 'Nr', 'np', 'rpm', 'rpm2', 'T', 'eta', 'tscale']) if (inp[k] !== undefined) setCtl(k, inp[k]); if (inp.planets !== undefined) setCtl('np', inp.planets); if (inp.hold) setHold(inp.hold); if (inp.drive) setDrive(inp.drive); if (inp.preset && PRESETS[inp.preset]) PRESETS[inp.preset](); if (inp.run !== undefined) state.run = !!inp.run; },
  meta: { id: 'planetary-gears', title: 'Planetary Gear Demonstrator', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.PlanetaryGears.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'planetary-gears', state: window.PlanetaryGears.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('pg_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildRig(); buildTasks(); bind(); renderLog(); syncDrive(); Object.assign(R, solve()); drawRig(); updateReadouts(); drawNomo(); drawSpeeds(); renderTabular(); state.run = true;
  requestAnimationFrame(tick);
}
build();
