/* Generates the physical Operating-Point Board from ../model.js:
   board-A1.svg (the chart), pieces-A1-*.svg (magnetic curve pieces at the same scale), pieces-small-A3.svg (bars, dots, flags),
   scenario-cards.html (A5 team cards, 2 per A4 landscape), facilitator-key.html (answers + figures), team-record-A4.html.
   Run: node make-print.js  → then print-pdf.sh converts everything to PDF with headless Chrome at 100 % scale. */
'use strict';
const fs = require('fs'), path = require('path');
require('../model.js'); const M = globalThis.OPB;
const { PUMP, CHART, POR, AOR } = M, { sx, sy, ox, oy, Qmax, Hmax } = CHART;
const OUT = __dirname;
const C = { ink: '#172231', muted: '#5B6B7C', grid: '#D5DEE7', gridMajor: '#AEB9C6', water: '#1F8AD9', waterDeep: '#0F5FA0', waterSoft: '#7FBDEB', brass: '#C7891E', brassDeep: '#8F5F0E', brassSoft: '#E4B96A', good: '#2A8A5C', warn: '#D98A12', alarm: '#C6302C', panel: '#F4F7FA', white: '#FFFFFF' };
const FONT = "font-family:'IBM Plex Sans','Segoe UI',Arial,sans-serif", MONO = "font-family:'IBM Plex Mono',Menlo,monospace", DISP = "font-family:'Barlow Condensed','Arial Narrow',Arial,sans-serif";
const X = Q => ox + sx * Q, Y = H => oy - sy * H;
const f1 = v => v.toFixed(1), f0 = v => v.toFixed(0);
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
const svgDoc = (w, h, body, extra = '') => `<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="${w}mm" height="${h}mm" viewBox="0 0 ${w} ${h}" ${extra}>\n<rect width="${w}" height="${h}" fill="${C.white}"/>\n${body}\n</svg>\n`;
const pts = (fn, a, b, step = 0.25) => { const out = []; for (let q = a; q <= b + 1e-9; q += step) out.push([q, fn(q)]); return out; };
const pathD = (P, map) => P.map((p, i) => (i ? 'L' : 'M') + map(p).map(v => v.toFixed(2)).join(',')).join(' ');
const local = ([q, h]) => [sx * q, -sy * h];   // piece-local mm, origin at (Q = 0, H = 0)
const text = (x, y, s, o = {}) => `<text x="${x}" y="${y}" font-size="${o.size || 4}" fill="${o.fill || C.ink}" style="${o.font || FONT};${o.weight ? 'font-weight:' + o.weight : ''}" text-anchor="${o.anchor || 'start'}"${o.rot ? ` transform="rotate(${o.rot} ${x} ${y})"` : ''}>${esc(s)}</text>`;

/* ---------------------------------------------------------------- chart (board) */
function chart(o = {}) {   // grid + axes in board coordinates; o.compact for key figures
  let s = '';
  s += `<rect x="${X(0)}" y="${Y(Hmax)}" width="${sx * Qmax}" height="${sy * Hmax}" fill="${C.white}" stroke="${C.ink}" stroke-width="0.6"/>`;
  for (let q = 0; q <= Qmax; q += 1) { const maj = q % 5 === 0; s += `<line x1="${X(q)}" y1="${Y(0)}" x2="${X(q)}" y2="${Y(Hmax)}" stroke="${maj ? C.gridMajor : C.grid}" stroke-width="${maj ? 0.45 : 0.2}"/>`; if (maj) s += text(X(q), Y(0) + (o.compact ? 7 : 9), q, { anchor: 'middle', size: o.compact ? 5 : 7, font: MONO }); }
  for (let h = 0; h <= Hmax; h += 1) { const maj = h % 5 === 0; s += `<line x1="${X(0)}" y1="${Y(h)}" x2="${X(Qmax)}" y2="${Y(h)}" stroke="${maj ? C.gridMajor : C.grid}" stroke-width="${maj ? 0.45 : 0.2}"/>`; if (maj) s += text(X(0) - 4, Y(h) + 2.2, h, { anchor: 'end', size: o.compact ? 5 : 7, font: MONO }); }
  s += text(X(Qmax / 2), Y(0) + (o.compact ? 15 : 22), 'Flow  Q,  m³/h', { anchor: 'middle', size: o.compact ? 6 : 10, weight: 600 });
  s += text(X(0) - (o.compact ? 12 : 22), Y(Hmax / 2), 'Head  H,  m', { anchor: 'middle', size: o.compact ? 6 : 10, weight: 600, rot: -90 });
  return s;
}
const bandStyle = kind => kind === 'speed' ? [C.water, C.waterDeep] : kind === 'trim' ? [C.brassSoft, C.brassDeep] : kind === 'parallel' ? [C.waterDeep, '#083F6E'] : kind === 'system' ? [C.brass, C.brassDeep] : [C.muted, C.ink];

/* ---------------------------------------------------------------- pieces (local coordinates) */
let uid = 0;
function strip(P, o) {   // band 10 mm wide with cut outline, centreline, registration ticks, label along the curve
  const id = 'p' + (uid++), [fill, dark] = bandStyle(o.kind), d = pathD(P, local);
  let s = `<g class="piece" data-id="${o.id}">`;
  s += `<path d="${d}" fill="none" stroke="#000" stroke-width="10.8" stroke-linecap="round" stroke-linejoin="round"/>`;   // cut line
  s += `<path d="${d}" fill="none" stroke="${fill}" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>`;
  s += `<path id="${id}" d="${d}" fill="none" stroke="${dark}" stroke-width="0.6"/>`;   // the curve itself
  for (const [q, h] of P) if (Math.abs(q - Math.round(q)) < 1e-6 && q % 10 === 0 && q > 0) { const [x, y] = local([q, h]); s += `<line x1="${x}" y1="${y - 5}" x2="${x}" y2="${y + 5}" stroke="#fff" stroke-width="0.8"/>` + text(x + 1.2, y - 2.2, 'Q ' + q, { size: 2.6, fill: '#fff', font: MONO }); }
  const [x0, y0] = local(P[0]); s += `<circle cx="${x0}" cy="${y0}" r="1.2" fill="#fff"/>` + text(x0 + 2.5, y0 + (o.kind === 'system' ? -3 : 4.2), o.kind === 'system' ? 'place here on the static head' : `Q = 0 · H = ${f1(P[0][1])} m`, { size: 2.6, fill: '#fff', font: MONO });
  s += `<text font-size="4.6" fill="#fff" style="${DISP};font-weight:600" dy="1.5"><textPath xlink:href="#${id}" startOffset="${o.offset || '30%'}">${o.id} · ${esc(o.label)}</textPath></text>`;
  return s + '</g>';
}
const pumpPts = p => pts(q => M.head(q / (p.count || 1), p.n, p.d), 0, Math.min(Qmax, M.runout(p.n, p.d) * (p.count || 1)), 0.5);
const sysPts = S => pts(q => S.K * q * q, 0, Math.min(Qmax, Math.sqrt(CHART.HsysRel / S.K)), 0.5);
function bar(w) {   // POR / AOR bar for a pump piece: amber AOR ends, green POR centre, BEP tick; placed on the Q axis
  const lo = AOR[0] * w.Qbep, hi = Math.min(Qmax, AOR[1] * w.Qbep), plo = POR[0] * w.Qbep, phi = Math.min(Qmax, POR[1] * w.Qbep);
  let s = `<g class="piece" data-id="${w.id}"><rect x="${sx * lo}" y="-6" width="${sx * (hi - lo)}" height="12" fill="${C.warn}" stroke="#000" stroke-width="0.35"/>`;
  s += `<rect x="${sx * plo}" y="-6" width="${sx * (phi - plo)}" height="12" fill="${C.good}"/><line x1="${sx * w.Qbep}" y1="-6" x2="${sx * w.Qbep}" y2="6" stroke="#fff" stroke-width="1"/>`;
  s += text(sx * w.Qbep, -7.5, `BEP ${f1(w.Qbep)}`, { anchor: 'middle', size: 3, fill: C.ink, font: MONO });
  s += text(sx * (plo + phi) / 2, 1.6, `${w.id} · POR 70–120 %  ·  ${w.label}`, { anchor: 'middle', size: 3.2, fill: '#fff', weight: 600 });
  s += text(sx * lo + 2, 1.6, 'AOR', { size: 3, fill: '#fff' }) + text(sx * hi - 2, 1.6, 'AOR', { size: 3, fill: '#fff', anchor: 'end' });
  return s + '</g>';
}
const dot = (r, fill, stroke, label, sub) => `<g><circle r="${r}" fill="${fill}" stroke="${stroke}" stroke-width="0.8"/><circle r="${r + 0.5}" fill="none" stroke="#000" stroke-width="0.3"/>${text(0, 1.5, label, { anchor: 'middle', size: 4, fill: fill === 'none' ? stroke : '#fff', weight: 700 })}${sub ? text(0, r + 4.5, sub, { anchor: 'middle', size: 2.6 }) : ''}</g>`;

/* ---------------------------------------------------------------- sheet layout: greedy vertical stacking of diagonal bands */
function layoutSheets(items, W, H, margin = 22) {   // items: {svg, P (local points), id}
  const sheets = []; let cur = { body: '', placed: [] };
  const clearance = 16;
  for (const it of items) {
    const ys = it.P.map(p => local(p)[1]), top = Math.min(...ys), bottom = Math.max(...ys), xsMax = Math.max(...it.P.map(p => local(p)[0]));
    const x0 = margin, tryPlace = sheet => { for (let ty = margin - top; ty + bottom < H - margin; ty += 2) { let ok = true; for (const pl of sheet.placed) for (const p of it.P) { const [lx, ly] = local(p), x = x0 + lx, y = ty + ly; const yo = pl.at(x); if (yo !== null && Math.abs(yo - y) < clearance) { ok = false; break; } } if (ok) return ty; } return null; };
    let ty = tryPlace(cur); if (ty === null) { sheets.push(cur); cur = { body: '', placed: [] }; ty = tryPlace(cur); }
    cur.body += `<g transform="translate(${x0},${ty})">${it.svg}</g>`;
    const map = new Map(it.P.map(p => { const [lx, ly] = local(p); return [Math.round(x0 + lx), ty + ly]; }));
    cur.placed.push({ at: x => { const v = map.get(Math.round(x)); return v === undefined ? null : v; } });
    if (xsMax + x0 > W - 5) console.warn('piece wider than sheet', it.id);
  }
  sheets.push(cur); return sheets.map(s => s.body);
}

/* ---------------------------------------------------------------- 1. board */
function boardA1() {
  let b = chart();
  b += text(ox, 30, 'Magnetic Operating-Point Board', { size: 16, weight: 600, font: DISP }) + text(ox, 40, 'Centrifugal pump H–Q chart · place the pump curve, the system curve and the POR/AOR bar, then mark the operating point', { size: 6, fill: C.muted });
  b += text(X(Qmax), 30, 'Mechanical engineering training · Istec Training', { size: 6, anchor: 'end', fill: C.muted }) + text(X(Qmax), 40, '[Illustrative pump and system data, for training only]', { size: 5, anchor: 'end', fill: C.muted });
  // nameplate + legend in the empty top-right of the plot
  const nx = X(30) + 4, ny = Y(Hmax) + 6;
  b += `<rect x="${nx}" y="${ny}" width="${X(Qmax) - nx - 4}" height="118" rx="2" fill="${C.panel}" stroke="${C.gridMajor}" stroke-width="0.4"/>`;
  b += text(nx + 4, ny + 8, 'PUMP NAMEPLATE', { size: 5, weight: 700, font: DISP });
  const rows = [[`${PUMP.name}`], [`Rated speed ${PUMP.N0} rpm · full impeller Ø${PUMP.D0} mm`], [`Shut-off head ${PUMP.H0} m · BEP ${PUMP.Qbep} m³/h at ${f1(M.head(PUMP.Qbep))} m · η ${f0(PUMP.etaMax * 100)} %`], [`Motor ${PUMP.kW} kW · runout ${f1(M.runout())} m³/h`], [''], ['AFFINITY LAWS', true], ['Q ∝ N · D      H ∝ (N · D)²      P ∝ (N · D)³'], [''], ['SYSTEM CURVE', true], ['H = H_static + K · Q²   (K in m per (m³/h)²)'], [`Baseline: H_static ${M.baseline.Hs} m, K = ${M.baseline.K.toFixed(3)} (pipe 0.015 + strainer 0.005)`], [''], ['PIECES', true], ['P1–P4 speed curves · P5–P7 trimmed impellers · P8 two pumps in parallel'], ['S1–S5 system parabolas (slide up/down to the static head)'], ['W1–W8 POR/AOR bars (lay on the Q axis) · BEP dots · OP markers'], [''], ['REGISTRATION', true], ['Align the Q = 0 end and the white "Q 10 / 20 / 30" ticks to the grid.']];
  rows.forEach((r, i) => { if (r[0]) b += text(nx + 4, ny + 15 + i * 5.4, r[0], { size: r[1] ? 4.2 : 4, weight: r[1] ? 700 : 400, font: r[1] ? DISP : (r[0].includes('∝') || r[0].startsWith('H =') ? MONO : FONT) }); });
  // registration guide under the chart
  b += text(ox, Y(0) + 34, 'Static head: the bottom-left end of a system piece sits on the H = static head line at Q = 0.   POR = 70–120 % of Q_BEP (ANSI/HI 9.6.3) · AOR = 50–130 % (illustrative) · both scale with speed and diameter.', { size: 4.6, fill: C.muted });
  b += text(ox, Y(0) + 42, 'Print at 100 % on A1 (841 × 594 mm). Mount on a steel sheet or a magnetic whiteboard. Pieces are on the matching A1/A3 sheets: laminate, cut on the black outline, magnetic tape on the back.', { size: 4.6, fill: C.muted });
  fs.writeFileSync(path.join(OUT, 'board-A1.svg'), svgDoc(841, 594, b));
}

/* ---------------------------------------------------------------- 2. pieces */
function pieceSheets() {
  const items = [];
  for (const p of M.PUMPS) { const P = pumpPts(p); items.push({ id: p.id, P, svg: strip(P, { id: p.id, label: p.label, kind: p.kind, offset: p.kind === 'parallel' ? '45%' : P.length < 80 ? '42%' : '28%' }) }); }
  const sheets = layoutSheets(items, 841, 594);
  sheets.forEach((body, i) => fs.writeFileSync(path.join(OUT, `pieces-A1-${i + 1}-pump-curves.svg`), svgDoc(841, 594, body + sheetFoot(`Pump-curve pieces, sheet ${i + 1} of ${sheets.length}`))));
  const sItems = M.SYSTEMS.slice().sort((a, b) => b.K - a.K).map(S => { const P = sysPts(S); return { id: S.id, P, svg: strip(P, { id: S.id, label: S.label, kind: 'system', offset: P.length < 50 ? '52%' : '35%' }) }; });
  const sSheets = layoutSheets(sItems, 841, 594);
  sSheets.forEach((body, i) => fs.writeFileSync(path.join(OUT, `pieces-A1-${sheets.length + i + 1}-system-curves.svg`), svgDoc(841, 594, body + sheetFoot(`System-curve pieces, sheet ${i + 1} of ${sSheets.length} · each parabola is H − H_static = K · Q²`))));
  // small pieces on A3 landscape
  let s = text(15, 14, 'Small pieces — POR/AOR bars (lay on the Q axis, BEP tick on Q_BEP), BEP dots, operating-point markers, static-head lines', { size: 6, weight: 600 });
  let y = 30; for (const w of M.WINDOWS) { s += `<g transform="translate(15,${y})">${bar(w)}</g>`; y += 20; }
  y += 6; const dots = [];
  for (let i = 0; i < 6; i++) dots.push(dot(6, C.good, C.good, 'BEP'));
  for (let i = 0; i < 6; i++) dots.push(dot(6, 'none', C.alarm, '?', 'predicted OP'));
  for (let i = 0; i < 6; i++) dots.push(dot(6, C.water, C.waterDeep, '✓', 'verified OP'));
  dots.forEach((d, i) => { s += `<g transform="translate(${25 + (i % 18) * 22},${y + Math.floor(i / 18) * 24})">${d}</g>`; });
  y += 30; for (let i = 0; i < 3; i++) { s += `<g transform="translate(15,${y + i * 12})"><rect x="0" y="-4" width="140" height="8" fill="${C.brassSoft}" stroke="#000" stroke-width="0.3"/><line x1="0" y1="0" x2="140" y2="0" stroke="${C.brassDeep}" stroke-width="0.6" stroke-dasharray="3 2"/>${text(70, 1.4, 'static head  H_s  (lay along the grid line)', { anchor: 'middle', size: 3.2 })}</g>`; }
  const flags = ['Q_BEP moves with speed and diameter', 'Throttling: extra loss in the valve', 'Static head: friction cannot buy it back', 'Two pumps ≠ double flow', 'Trim limit ~10–15 % D'];
  flags.forEach((t, i) => { s += `<g transform="translate(${180 + (i % 3) * 78},${y + Math.floor(i / 3) * 14})"><rect x="0" y="-5" width="72" height="10" rx="1.5" fill="${C.panel}" stroke="#000" stroke-width="0.3"/>${text(36, 1.4, t, { anchor: 'middle', size: 3.2 })}</g>`; });
  fs.writeFileSync(path.join(OUT, `pieces-A1-${sheets.length + sSheets.length + 1}-small-pieces.svg`), svgDoc(841, 594, s + sheetFoot('Small pieces · A1 landscape')));
  return sheets.length + sSheets.length + 1;
}
const sheetFoot = (t, W = 841, H = 594) => text(15, H - 8, `${t} · print at 100 % · same scale as the board (${sx} mm per m³/h, ${sy} mm per m) · laminate, cut on the black outline, magnetic tape on the back`, { size: 4, fill: C.muted });

/* ---------------------------------------------------------------- 3. key figure (compact board with the answer placed) */
function keyFigure(sc) {
  const p = sc.pump, S = sc.sys, a = sc.ans;
  let b = chart({ compact: true });
  const curve = (P, col, w, dash) => `<path d="${pathD(P, ([q, h]) => [X(q), Y(h)])}" fill="none" stroke="${col}" stroke-width="${w}"${dash ? ` stroke-dasharray="${dash}"` : ''}/>`;
  b += curve(pumpPts(M.PUMPS[0]), C.waterSoft, 0.8, '3 2');
  b += curve(pumpPts(p), bandStyle(p.kind)[0], 2.2);
  b += curve(pts(q => M.sysHead(q, S), 0, Math.min(Qmax, Math.sqrt((Hmax - S.Hs) / S.K)), 0.5), C.brass, 2.2);
  b += `<line x1="${X(0)}" y1="${Y(S.Hs)}" x2="${X(Qmax)}" y2="${Y(S.Hs)}" stroke="${C.brassDeep}" stroke-width="0.5" stroke-dasharray="2 2"/>` + text(X(Qmax) - 2, Y(S.Hs) - 1.5, `H_s ${S.Hs} m`, { anchor: 'end', size: 4, fill: C.brassDeep, font: MONO });
  const w = M.WINDOWS.find(w => w.pump === p.id); b += `<g transform="translate(${X(0)},${Y(0) - 3})">${bar(w)}</g>`;
  const qb = w.Qbep; b += `<circle cx="${X(qb)}" cy="${Y(M.head(qb / (p.count || 1), p.n, p.d))}" r="3" fill="${C.good}"/>` + text(X(qb) + 4, Y(M.head(qb / (p.count || 1), p.n, p.d)) - 3, 'BEP', { size: 4, fill: C.good, weight: 700 });
  b += `<circle cx="${X(a.Q)}" cy="${Y(a.H)}" r="3.2" fill="${C.alarm}"/>` + text(X(a.Q) + 5, Y(a.H) + 1.5, `OP  ${f1(a.Q)} m³/h · ${f1(a.H)} m`, { size: 4.2, weight: 700, fill: C.alarm, font: MONO });
  if (sc.id === 'parallel') b += `<circle cx="${X(a.Qp)}" cy="${Y(a.H)}" r="2.2" fill="none" stroke="${C.alarm}" stroke-width="0.8"/>` + text(X(a.Qp) - 4, Y(a.H) - 4, `each pump ${f1(a.Qp)}`, { anchor: 'end', size: 3.6, fill: C.alarm, font: MONO });
  if (sc.id === 'throttle') { const hv = (S.K - M.baseline.K) * a.Q * a.Q; b += `<line x1="${X(a.Q)}" y1="${Y(a.H)}" x2="${X(a.Q)}" y2="${Y(a.H - hv)}" stroke="${C.warn}" stroke-width="1.2"/>` + text(X(a.Q) + 3, Y(a.H - hv / 2), `valve loss ${f1(hv)} m`, { size: 3.6, fill: C.warn, font: MONO }); b += curve(pts(q => M.sysHead(q, M.baseline), 0, 36, 0.5), C.brassSoft, 0.8, '3 2'); }
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="${ox - 28} ${Y(Hmax) - 6} ${sx * Qmax + 40} ${sy * Hmax + 36}" style="width:100%;height:auto;display:block">${b}</svg>`;
}

/* ---------------------------------------------------------------- 4. HTML documents */
const CSS = `body{font-family:'IBM Plex Sans','Segoe UI',Arial,sans-serif;color:#172231;margin:0;font-size:10.5pt;line-height:1.35}h1,h2,h3{font-family:'Barlow Condensed','Arial Narrow',Arial,sans-serif;font-weight:600;margin:0 0 4mm}h1{font-size:22pt}h2{font-size:16pt;color:#0F5FA0}h3{font-size:12.5pt;margin-bottom:2mm}table{border-collapse:collapse;width:100%;font-size:9.5pt}th,td{border:0.3pt solid #AEB9C6;padding:1.4mm 2mm;text-align:left;vertical-align:top}th{background:#EAF0F6}.mono{font-family:'IBM Plex Mono',Menlo,monospace}.muted{color:#5B6B7C}.blank{display:inline-block;min-width:22mm;border-bottom:0.4pt solid #172231;height:5mm}.eyebrow{font-size:8pt;text-transform:uppercase;letter-spacing:.12em;color:#5B6B7C;font-weight:600}.pill{display:inline-block;background:#EAF0F6;border-radius:3pt;padding:0.5mm 2mm;font-family:'IBM Plex Mono',monospace;font-size:8.5pt;margin-right:1.5mm}ol,ul{margin:1.5mm 0 1.5mm 5mm;padding:0}li{margin:0.8mm 0}`;
function cardsHtml() {
  const cards = M.SCENARIOS.map(sc => {
    const pieces = sc.pieces.map(id => `<span class="pill">${id}${id === 'OP' ? ' ? marker' : id === 'BEP' ? ' dot' : ''}</span>`).join('');
    return `<section class="card"><div class="head"><span class="letter">${sc.card}</span><div><span class="eyebrow">Operating-point board · scenario card</span><h2>${esc(sc.title)}</h2></div></div>
<h3>The situation</h3><p>${esc(sc.brief)}</p>
<h3>On the board</h3><ol><li>Keep the baseline pieces in place unless the card changes them.</li><li>Pieces for this scenario: ${pieces}</li><li>Register each piece to the grid (Q = 0 end and the white Q 10 / 20 / 30 ticks). Slide the system piece to the static head.</li><li>Put the BEP dot on the curve and lay the POR/AOR bar on the Q axis.</li><li>Place the red <b>?</b> marker where the curves cross.</li></ol>
<h3>Predict, then write it down</h3><p>${esc(sc.ask)}</p>
<table><tr><th>Flow Q (m³/h)</th><th>Head H (m)</th><th>% of Q_BEP</th><th>Inside POR?</th><th>η (%)</th><th>Shaft power (kW)</th></tr><tr><td><span class="blank"></span></td><td><span class="blank"></span></td><td><span class="blank"></span></td><td>yes / no</td><td><span class="blank"></span></td><td><span class="blank"></span></td></tr></table>
<p class="muted">Then verify on the digital board or the hydraulic loop bench and mark the true point with the blue ✓. Power: P = ρ g Q H / η with Q in m³/s.</p>
<div class="foot mono">Pump: H = 32 − 0.2 Q − 0.01 Q² at 1 480 rpm · Q_BEP 24 m³/h · System: H = H_s + K Q² · baseline H_s 10 m, K 0.020 · POR 70–120 % · AOR 50–130 %</div></section>`;
  }).join('');
  const html = `<!doctype html><html><head><meta charset="utf-8"><title>Operating-point board — scenario cards</title><style>${CSS}@page{size:A4 landscape;margin:8mm}body{display:grid;grid-template-columns:1fr 1fr;gap:6mm}.card{border:0.5pt solid #172231;border-radius:3mm;padding:6mm 7mm;height:181mm;box-sizing:border-box;break-inside:avoid;position:relative;display:flex;flex-direction:column}.card:nth-child(2n){page-break-after:always}.head{display:flex;gap:4mm;align-items:center;margin-bottom:2mm}.letter{font-family:'Barlow Condensed',Arial,sans-serif;font-size:30pt;font-weight:700;color:#fff;background:#1F8AD9;border-radius:3mm;width:16mm;height:16mm;display:grid;place-items:center;flex:0 0 auto}.card h2{margin:0}.card h3{margin:2mm 0 1mm;font-size:11.5pt}.card p{margin:0 0 1mm}.foot{margin-top:auto;font-size:7.5pt;color:#5B6B7C;border-top:0.3pt solid #AEB9C6;padding-top:1.5mm}</style></head><body>${cards}</body></html>`;
  fs.writeFileSync(path.join(OUT, 'scenario-cards.html'), html);
}
function keyHtml(nSheets) {
  const rows = M.SCENARIOS.map(sc => `<tr><td><b>${sc.card}</b> ${esc(sc.title)}</td><td class="mono">${sc.pump.short}${sc.pump.count ? ' ×2' : ''}</td><td class="mono">H_s ${sc.sys.Hs} m · K ${sc.sys.K.toFixed(3)}</td><td class="mono">${f1(sc.ans.Q)}${sc.ans.Qp && sc.pump.count ? ` (${f1(sc.ans.Qp)} each)` : ''}</td><td class="mono">${f1(sc.ans.H)}</td><td class="mono">${f0(sc.ans.pct * 100)} %</td><td>${sc.ans.pct >= POR[0] && sc.ans.pct <= POR[1] ? 'yes' : sc.ans.pct >= AOR[0] && sc.ans.pct <= AOR[1] ? 'no — AOR' : 'no — outside AOR'}</td><td class="mono">${f0(sc.ans.eta * 100)}</td><td class="mono">${(sc.ans.P / 1000).toFixed(2)}</td></tr>`).join('');
  const pieceTable = M.PUMPS.map(p => `<tr><td class="mono">${p.id}</td><td>${esc(p.label)}</td><td class="mono">H = ${(p.n * p.d) ** 2 === 1 ? '' : ((p.n * p.d) ** 2).toFixed(3) + ' × '}(32 − 0.2 q − 0.01 q²), q = Q${p.count ? '/2' : ''}${p.n * p.d === 1 ? '' : ' / ' + (p.n * p.d).toFixed(3)}</td><td class="mono">${f1(M.head(0, p.n, p.d))}</td><td class="mono">${f1(PUMP.Qbep * p.n * p.d * (p.count || 1))}</td></tr>`).join('') + M.SYSTEMS.map(S => `<tr><td class="mono">${S.id}</td><td>${esc(S.label)}</td><td class="mono">H = H_s + ${S.K.toFixed(3)} Q²</td><td>—</td><td>—</td></tr>`).join('');
  const scen = M.SCENARIOS.map(sc => `<section class="sc"><h2>${sc.card} · ${esc(sc.title)}</h2><div class="two"><div>${keyFigure(sc)}</div><div><p><b>Set-up:</b> ${esc(sc.brief)}</p><p><b>Answer:</b> ${esc(sc.key)}</p><table><tr><th>Q m³/h</th><th>H m</th><th>% Q_BEP</th><th>η %</th><th>kW</th></tr><tr class="mono"><td>${f1(sc.ans.Q)}${sc.pump.count ? ` (${f1(sc.ans.Qp)} per pump)` : ''}</td><td>${f1(sc.ans.H)}</td><td>${f0(sc.ans.pct * 100)}</td><td>${f0(sc.ans.eta * 100)}</td><td>${(sc.ans.P / 1000).toFixed(2)}</td></tr></table><p class="muted"><b>Watch for:</b> ${esc(WATCH[sc.id])}</p></div></div></section>`).join('');
  const html = `<!doctype html><html><head><meta charset="utf-8"><title>Operating-point board — facilitator key</title><style>${CSS}@page{size:A4;margin:14mm 14mm 16mm}.two{display:grid;grid-template-columns:1.15fr 1fr;gap:6mm;align-items:start}.sc{break-inside:avoid;margin-bottom:8mm;padding-bottom:4mm;border-bottom:0.3pt solid #AEB9C6}h2{margin-top:2mm}</style></head><body>
<span class="eyebrow">Mechanical engineering training · Operating-point board</span><h1>Facilitator key and set-up guide</h1>
<h2>Building the board</h2><ol><li>Print <b>board-A1.pdf</b> at 100 % (A1, 841 × 594 mm). Mount it on a steel sheet, or laminate it and fix it to a magnetic whiteboard with magnets. Scale: ${sx} mm per m³/h, ${sy} mm per m.</li><li>Print the <b>${nSheets} piece sheets (A1)</b> (the last one carries the POR/AOR bars, dots and markers) at 100 % on the same printer, laminate, cut along the black outline, and stick magnetic tape along the back of each strip. Alternative: print the pieces on A4/A3 printable magnetic paper, tiled by the print driver at 100 % with cut-and-join marks.</li><li>Colour code: blue bands are speed curves, sand bands are trimmed impellers, dark blue is two pumps in parallel, brass parabolas are system curves. The thin dark line inside each band is the curve; the band is only there to hold a magnet.</li><li>Registration: each pump piece carries its Q = 0 end (shut-off head) and white ticks at Q = 10, 20, 30 m³/h. Each system piece is a parabola <i>relative to the static head</i>: its bottom-left end sits on the H = H_s grid line at Q = 0. Sliding it up and down is how a team sets the static head — that is the whole point of the piece.</li><li>POR/AOR bars lie on the Q axis. The green segment is 70–120 % of Q_BEP, amber extends to 50–130 %. There is one bar per pump piece because Q_BEP moves with speed, diameter and the number of pumps.</li></ol>
<h2>Running the exercise</h2><ol><li>Teams of 3–4, one board per team (or one board with teams taking turns while the others predict on paper).</li><li>Card 0 commissions the board. Then hand out A–D one at a time; E and F are for fast teams.</li><li>Each team registers the pieces, marks BEP, marks its predicted operating point with the red ?, and writes Q, H, % BEP, POR yes/no, η and kW on the card before anyone checks.</li><li>Verify with the digital board (same pieces, same scenarios, reveals the true intersection) and, for A–D, on the hydraulic loop bench. Score: within 1.5 m³/h and 1.5 m is a hit.</li><li>Debrief with the answer table below; the discussion points are the learning.</li></ol>
<h2>Answer table</h2><table><tr><th>Scenario</th><th>Pump piece</th><th>System</th><th>Q m³/h</th><th>H m</th><th>% Q_BEP</th><th>In POR</th><th>η %</th><th>Shaft kW</th></tr>${rows}</table>
<p class="muted">Tolerance for a "hit": ±1.5 m³/h and ±1.5 m. Pieces are 10 mm wide bands; reading the thin centreline gives about ±0.5 m³/h on this scale.</p>
<h2>Pieces and their equations</h2><table><tr><th>Piece</th><th>Description</th><th>Equation (Q in m³/h, H in m)</th><th>Shut-off m</th><th>Q_BEP m³/h</th></tr>${pieceTable}</table>
<p class="muted">Strainer fouled 40 %: loss through a screen scales with 1/(open area)², so the strainer's 0.005 becomes 0.005/0.6² = 0.0139 and K = 0.015 + 0.0139 = 0.029. Throttling to 18 m³/h: K = (H_pump(18) − H_s)/18² = ${M.kForFlow(18, M.baseline, M.PUMPS[0]).toFixed(4)}. Trim for 18 m³/h: diameter ratio ${M.dForFlow(18, M.baseline).toFixed(3)} → Ø${f0(M.dForFlow(18, M.baseline) * PUMP.D0)} mm.</p>
${scen}
<h2>Verify on the hydraulic loop bench</h2><p>The loop bench simulator uses the same pump. Card A ≈ strainer fouling slider; card B ≈ reservoir level; card C ≈ discharge valve; card D ≈ speed 81 %. Its pipe friction differs from the board's K, so compare the <i>direction and size</i> of each move, not the exact numbers.</p>
<p class="muted">[Illustrative pump and system data, for training only.] Generated from model.js — regenerate with node make-print.js after any change.</p></body></html>`;
  fs.writeFileSync(path.join(OUT, 'facilitator-key.html'), html);
}
const WATCH = {
  base: 'Teams that place the system piece with its end at H = 0 instead of the static head. Teams that mark BEP at the top of the curve.',
  strainer: 'Teams that move the static head instead of changing K. Ask where the loss is: suction side, so NPSH margin falls.',
  level: 'Teams that slide the piece down (level falls, head rises — the pump lifts further). Teams that change the shape of the parabola.',
  throttle: 'Teams that pick S3 because "it is steeper": ask them to check where it crosses. Make them draw the valve loss as a vertical distance at 18 m³/h.',
  vfd: 'Teams that leave the POR bar and BEP where they were. Teams that expect the cube law (P ∝ N³) to hold: the static head breaks it.',
  parallel: 'Teams that draw the parallel curve by adding heads. Teams that expect 48 m³/h. Ask what each pump sees: 61 % of BEP, in the low-flow recirculation zone.',
  trim: 'Teams that scale only head or only flow. Point out the trim limit and that the trimmed pump cannot be "un-trimmed" when the duty comes back.',
};
function recordHtml() {
  const rows = M.SCENARIOS.map(sc => `<tr><td><b>${sc.card}</b> ${esc(sc.title)}</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr>`).join('');
  fs.writeFileSync(path.join(OUT, 'team-record-A4.html'), `<!doctype html><html><head><meta charset="utf-8"><title>Team record sheet</title><style>${CSS}@page{size:A4 landscape;margin:12mm}td{height:12mm}</style></head><body><span class="eyebrow">Operating-point board</span><h1>Team record sheet &nbsp; <span class="muted" style="font-size:12pt;font-weight:400">Team ________________ &nbsp; Date __________</span></h1><table><tr><th>Scenario</th><th>Predicted Q</th><th>Predicted H</th><th>Predicted kW</th><th>Verified Q</th><th>Verified H</th><th>Hit? (±1.5)</th><th>What we learned</th></tr>${rows}</table><p class="muted">Hit = within 1.5 m³/h and 1.5 m of the verified point. Verified on: digital board / hydraulic loop bench.</p></body></html>`);
}

/* ---------------------------------------------------------------- run */
boardA1(); const n = pieceSheets(); cardsHtml(); keyHtml(n); recordHtml();
console.log('generated board-A1.svg,', n, 'piece sheets (last = small pieces), scenario-cards.html, facilitator-key.html, team-record-A4.html');
