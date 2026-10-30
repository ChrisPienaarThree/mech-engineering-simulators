/* Operating-Point Board — digital twin of the magnetic H–Q board. Same model.js as the printed pieces, same mm
   coordinates (841 × 594 board, 14 mm per m³/h, 12.5 mm per m). Pieces are dragged onto the chart and snap when
   registered; the team marks a predicted operating point and Check compares it with the true intersection.
   Standalone, offline. Data illustrative. */
'use strict';
const M = window.OPB, { PUMP, CHART, POR, AOR } = M, { sx, sy, ox, oy, Qmax, Hmax } = CHART;
const $ = s => document.querySelector(s), NS = 'http://www.w3.org/2000/svg';
const clamp = (x, a, b) => Math.max(a, Math.min(b, x)), fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';
const X = Q => ox + sx * Q, Y = H => oy - sy * H, Qof = x => (x - ox) / sx, Hof = y => (oy - y) / sy;
const el = (tag, attrs = {}, parent) => { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; };
const txt = (x, y, s, cls, parent, anchor) => { const t = el('text', { x, y, class: cls }, parent); if (anchor) t.setAttribute('text-anchor', anchor); t.textContent = s; return t; };
const TOL = { hit: 1.5, near: 3 };

/* ------------------------------------------------------------------ state */
const state = { sc: 0, pieces: [], log: [], flags: {}, revealed: false, projector: false, team: '' };
let svg, gPieces, gTruth, gRead;

/* ------------------------------------------------------------------ geometry of pieces (local mm, origin at Q = 0, H = 0) */
const pumpPts = p => { const out = []; const qmax = Math.min(Qmax, M.runout(p.n, p.d) * (p.count || 1)); for (let q = 0; q <= qmax + 1e-9; q += 0.5) out.push([sx * q, -sy * M.head(q / (p.count || 1), p.n, p.d)]); return out; };
const sysPts = S => { const out = []; const qmax = Math.min(Qmax, Math.sqrt(CHART.HsysRel / S.K)); for (let q = 0; q <= qmax + 1e-9; q += 0.5) out.push([sx * q, -sy * S.K * q * q]); return out; };
const d = P => P.map((p, i) => (i ? 'L' : 'M') + p[0].toFixed(2) + ',' + p[1].toFixed(2)).join(' ');
const PIECE_DEFS = {};
for (const p of M.PUMPS) PIECE_DEFS[p.id] = { type: 'pump', kind: p.kind, pump: p, pts: pumpPts(p), home: [ox, oy], label: p.id + ' · ' + p.label };
for (const S of M.SYSTEMS) PIECE_DEFS[S.id] = { type: 'system', kind: 'system', sys: S, pts: sysPts(S), home: [ox, null], label: S.id + ' · ' + S.label };
for (const w of M.WINDOWS) PIECE_DEFS[w.id] = { type: 'bar', win: w, home: [ox, oy - 3], label: w.id + ' · ' + w.label };
PIECE_DEFS.BEP = { type: 'bep', label: 'BEP dot' }; PIECE_DEFS.OP = { type: 'op', label: '? predicted operating point' }; PIECE_DEFS.HS = { type: 'hs', label: 'static-head line' };

/* ------------------------------------------------------------------ board */
function buildBoard() {
  svg = $('#board'); svg.innerHTML = '';
  el('rect', { x: X(0), y: Y(Hmax), width: sx * Qmax, height: sy * Hmax, class: 'frame' }, svg);
  for (let q = 0; q <= Qmax; q++) { el('line', { x1: X(q), y1: Y(0), x2: X(q), y2: Y(Hmax), class: q % 5 ? 'grid-minor' : 'grid-major' }, svg); if (q % 5 === 0) txt(X(q), Y(0) + 9, q, 'axis-txt', svg, 'middle'); }
  for (let h = 0; h <= Hmax; h++) { el('line', { x1: X(0), y1: Y(h), x2: X(Qmax), y2: Y(h), class: h % 5 ? 'grid-minor' : 'grid-major' }, svg); if (h % 5 === 0) txt(X(0) - 4, Y(h) + 2.2, h, 'axis-txt', svg, 'end'); }
  txt(X(Qmax / 2), Y(0) + 22, 'Flow  Q,  m³/h', 'axis-name', svg, 'middle');
  const yl = txt(X(0) - 22, Y(Hmax / 2), 'Head  H,  m', 'axis-name', svg, 'middle'); yl.setAttribute('transform', `rotate(-90 ${X(0) - 22} ${Y(Hmax / 2)})`);
  txt(X(Qmax) - 2, Y(Hmax) + 8, `${PUMP.name} · ${PUMP.N0} rpm · Ø${PUMP.D0} · H = 32 − 0.2Q − 0.01Q² · BEP ${PUMP.Qbep} m³/h`, 'axis-txt', svg, 'end');
  gTruth = el('g', { id: 'truth' }, svg); gPieces = el('g', { id: 'pieces' }, svg); gRead = el('g', { id: 'readouts' }, svg);
}

/* ------------------------------------------------------------------ pieces */
function addPiece(id, at) {
  const def = PIECE_DEFS[id]; if (!def) return;
  if (['pump', 'system', 'bar'].includes(def.type) && state.pieces.some(p => p.id === id)) { selectPiece(state.pieces.find(p => p.id === id)); return; }
  const pc = { id, def, uid: Math.random().toString(36).slice(2), x: 0, y: 0, reg: false };
  const g = el('g', { class: 'piece ' + (def.kind || def.type), 'data-uid': pc.uid }, gPieces); pc.g = g;
  if (def.type === 'pump' || def.type === 'system') {
    const dd = d(def.pts); el('path', { d: dd, class: 'cut' }, g); el('path', { d: dd, class: 'band' }, g); const c = el('path', { d: dd, class: 'centre', id: 'c' + pc.uid }, g);
    for (const [x, y] of def.pts) { const q = x / sx; if (q > 0 && Math.abs(q % 10) < 1e-6) { el('line', { x1: x, y1: y - 5, x2: x, y2: y + 5, class: 'tick' }, g); txt(x + 1.2, y - 2.2, 'Q ' + q, 'ticktxt', g); } }
    el('circle', { cx: 0, cy: def.pts[0][1], r: 1.2, fill: '#fff' }, g); txt(2.5, def.pts[0][1] + (def.type === 'system' ? -3 : 4.2), def.type === 'system' ? 'place on the static head' : `Q = 0 · H = ${fmt(-def.pts[0][1] / sy)} m`, 'ticktxt', g);
    const t = el('text', { class: 'lbl', dy: 1.5 }, g); const tp = el('textPath', { href: '#c' + pc.uid, startOffset: def.type === 'system' ? '35%' : def.kind === 'parallel' ? '45%' : '28%' }, t); tp.textContent = def.label;
    pc.x = at ? at[0] : X(Qmax) - def.pts[def.pts.length - 1][0] - 10; pc.y = at ? at[1] : (def.type === 'system' ? Y(2) : Y(Hmax) - def.pts[0][1] + 6);
  } else if (def.type === 'bar') {
    const w = def.win, lo = AOR[0] * w.Qbep, hi = Math.min(Qmax, AOR[1] * w.Qbep), plo = POR[0] * w.Qbep, phi = Math.min(Qmax, POR[1] * w.Qbep);
    el('rect', { x: sx * lo, y: -6, width: sx * (hi - lo), height: 12, class: 'bar-aor' }, g); el('rect', { x: sx * plo, y: -6, width: sx * (phi - plo), height: 12, class: 'bar-por' }, g);
    el('line', { x1: sx * w.Qbep, y1: -6, x2: sx * w.Qbep, y2: 6, stroke: '#fff', 'stroke-width': 1 }, g); txt(sx * w.Qbep, -7.5, `BEP ${fmt(w.Qbep)}`, 'bar-bep', g, 'middle');
    txt(sx * (plo + phi) / 2, 1.6, `${w.id} · POR 70–120 % · ${w.label}`, 'bar-txt', g, 'middle'); txt(sx * lo + 2, 1.6, 'AOR', 'bar-txt', g); txt(sx * hi - 2, 1.6, 'AOR', 'bar-txt', g, 'end');
    pc.x = at ? at[0] : X(0) + 40; pc.y = at ? at[1] : Y(Hmax) + 30;
  } else if (def.type === 'bep') { el('circle', { r: 6, class: 'dot-bep' }, g); txt(0, 1.5, 'BEP', 'dot-txt', g, 'middle'); pc.x = at ? at[0] : X(40); pc.y = at ? at[1] : Y(30); }
  else if (def.type === 'op') { el('circle', { r: 6, class: 'dot-op' }, g); txt(0, 1.6, '?', 'dot-txt dot-op-txt', g, 'middle'); pc.x = at ? at[0] : X(44); pc.y = at ? at[1] : Y(30); }
  else if (def.type === 'hs') { el('rect', { x: 0, y: -4, width: 140, height: 8, fill: 'var(--brass-soft)', stroke: 'var(--ink)', 'stroke-width': 0.3 }, g); el('line', { x1: 0, y1: 0, x2: 140, y2: 0, class: 'hs-line' }, g); txt(70, 1.4, 'static head  H_s', 'bar-bep', g, 'middle'); pc.x = at ? at[0] : X(0); pc.y = at ? at[1] : Y(Hmax) + 50; }
  state.pieces.push(pc); place(pc); makeDraggable(pc); refresh();
}
function place(pc) { pc.g.setAttribute('transform', `translate(${pc.x.toFixed(2)},${pc.y.toFixed(2)})`); }
function removePiece(pc) { pc.g.remove(); state.pieces = state.pieces.filter(p => p !== pc); refresh(); }
function selectPiece(pc) { state.pieces.forEach(p => p.g.classList.toggle('sel', p === pc)); pc.g.parentNode.appendChild(pc.g); }
function snap(pc) {   // registration: pump pieces and bars snap to their true place; system pieces snap to Q = 0 and 0.5 m; markers free
  const t = pc.def; pc.reg = false;
  if (t.type === 'pump') { if (Math.abs(pc.x - ox) < 7 && Math.abs(pc.y - oy) < 7) { pc.x = ox; pc.y = oy; pc.reg = true; } }
  else if (t.type === 'system') { if (Math.abs(pc.x - ox) < 7) { pc.x = ox; pc.y = oy - sy * Math.round(Hof(pc.y) * 2) / 2; pc.reg = Hof(pc.y) >= 0 && Hof(pc.y) <= Hmax; } }
  else if (t.type === 'bar') { if (Math.abs(pc.x - ox) < 7 && Math.abs(pc.y - (oy - 3)) < 12) { pc.x = ox; pc.y = oy - 3; pc.reg = true; } }
  else if (t.type === 'hs') { if (Math.abs(pc.x - ox) < 7) { pc.x = ox; pc.y = oy - sy * Math.round(Hof(pc.y) * 2) / 2; pc.reg = true; } }
  pc.g.classList.toggle('registered', pc.reg); place(pc);
}
function makeDraggable(pc) {
  let start = null;
  pc.g.addEventListener('pointerdown', e => { if (state.projector) return; e.preventDefault(); pc.g.setPointerCapture(e.pointerId); const p = toSvg(e); start = { px: p.x, py: p.y, x: pc.x, y: pc.y, moved: false }; pc.g.classList.add('dragging'); selectPiece(pc); });
  pc.g.addEventListener('pointermove', e => { if (!start) return; const p = toSvg(e); pc.x = start.x + p.x - start.px; pc.y = start.y + p.y - start.py; if (Math.abs(p.x - start.px) + Math.abs(p.y - start.py) > 1) start.moved = true; place(pc); if (pc.def.type === 'op') refresh(); });
  const up = e => { if (!start) return; pc.g.classList.remove('dragging'); const moved = start.moved; start = null; snap(pc); if (!moved && e.shiftKey) removePiece(pc); refresh(); };
  pc.g.addEventListener('pointerup', up); pc.g.addEventListener('pointercancel', up);
  pc.g.addEventListener('dblclick', () => removePiece(pc));
}
function toSvg(e) { const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY; return pt.matrixTransform(svg.getScreenCTM().inverse()); }
function clearBoard() { state.pieces.forEach(p => p.g.remove()); state.pieces = []; hideTruth(); refresh(); }
function placeBaseline() { clearBoard(); addPiece('P1', [ox, oy]); addPiece('S2', [ox, oy - sy * M.baseline.Hs]); addPiece('W1', [ox, oy - 3]); state.pieces.forEach(snap); addPiece('BEP'); addPiece('OP'); refresh(); }

/* ------------------------------------------------------------------ reading the board */
function boardState() {
  const pumps = state.pieces.filter(p => p.def.type === 'pump'), regPumps = pumps.filter(p => p.reg);
  const sysP = state.pieces.filter(p => p.def.type === 'system' && p.reg), bars = state.pieces.filter(p => p.def.type === 'bar' && p.reg);
  const op = state.pieces.find(p => p.def.type === 'op'), bep = state.pieces.find(p => p.def.type === 'bep');
  return { pump: regPumps[regPumps.length - 1] || null, sys: sysP[sysP.length - 1] || null, bar: bars[bars.length - 1] || null, op, bep, nReg: state.pieces.filter(p => p.reg).length,
    Hs: sysP.length ? Hof(sysP[sysP.length - 1].y) : null, predQ: op ? Qof(op.x) : null, predH: op ? Hof(op.y) : null };
}
function refresh() {
  const b = boardState(), sc = M.SCENARIOS[state.sc];
  $('#pQ').textContent = b.op ? fmt(b.predQ) : '—'; $('#pH').textContent = b.op ? fmt(b.predH) : '—';
  $('#pSys').textContent = b.sys ? `${b.sys.def.sys.id} K ${b.sys.def.sys.K.toFixed(3)} · ${fmt(b.Hs)} m` : '—';
  $('#pPump').textContent = b.pump ? b.pump.def.pump.id + ' ' + b.pump.def.pump.short : '—';
  $('#pillPieces').textContent = `${b.nReg} piece${b.nReg === 1 ? '' : 's'} registered`;
  gRead.innerHTML = ''; if (b.op && !state.projector) txt(b.op.x + 8, b.op.y - 6, `? ${fmt(b.predQ)} m³/h · ${fmt(b.predH)} m`, 'readout', gRead);
  document.querySelectorAll('#tray [data-piece]').forEach(btn => { const id = btn.dataset.piece; btn.classList.toggle('on', state.pieces.some(p => p.id === id)); btn.classList.toggle('need', sc.pieces.includes(id)); });
  checkTasks();
}

/* ------------------------------------------------------------------ truth overlay */
function showTruth(sc) {
  hideTruth(); state.revealed = true; const p = sc.pump, S = sc.sys, a = sc.ans;
  const line = (pts, cls) => el('path', { d: pts.map((q, i) => (i ? 'L' : 'M') + X(q[0]).toFixed(2) + ',' + Y(q[1]).toFixed(2)).join(' '), class: 'truth ' + cls }, gTruth);
  const pp = []; for (let q = 0; q <= Math.min(Qmax, M.runout(p.n, p.d) * (p.count || 1)); q += 0.5) pp.push([q, M.head(q / (p.count || 1), p.n, p.d)]); line(pp, 'pump');
  const sp = []; for (let q = 0; q <= Math.min(Qmax, Math.sqrt((Hmax - S.Hs) / S.K)); q += 0.5) sp.push([q, M.sysHead(q, S)]); line(sp, 'sys');
  el('line', { x1: X(0), y1: Y(S.Hs), x2: X(Qmax), y2: Y(S.Hs), class: 'hs-line' }, gTruth);
  el('circle', { cx: X(a.Q), cy: Y(a.H), r: 4, class: 'truth-op' }, gTruth); txt(X(a.Q) + 7, Y(a.H) + 1.5, `✓ ${fmt(a.Q)} m³/h · ${fmt(a.H)} m · η ${fmt(a.eta * 100, 0)} % · ${fmt(a.P / 1000, 2)} kW · ${fmt(a.pct * 100, 0)} % of BEP`, 'truth-txt', gTruth);
  if (p.count) { el('circle', { cx: X(a.Qp), cy: Y(a.H), r: 3, class: 'truth-op' }, gTruth); txt(X(a.Qp) - 5, Y(a.H) - 5, `each pump ${fmt(a.Qp)}`, 'truth-txt', gTruth, 'end'); }
  const qb = a.Qbep * (p.count || 1); el('circle', { cx: X(qb), cy: Y(M.head(qb / (p.count || 1), p.n, p.d)), r: 3, class: 'dot-bep' }, gTruth); txt(X(qb) + 4, Y(M.head(qb / (p.count || 1), p.n, p.d)) - 4, 'BEP', 'truth-txt', gTruth);
  $('#btnHide').classList.remove('hidden');
}
function hideTruth() { gTruth.innerHTML = ''; state.revealed = false; }

/* ------------------------------------------------------------------ check */
function check() {
  const sc = M.SCENARIOS[state.sc], b = boardState(), a = sc.ans, v = $('#verdict');
  if (!b.op) { v.className = 'verdict miss'; v.textContent = 'Add the ? marker from the tray and drag it to your predicted operating point first.'; return; }
  const dQ = b.predQ - a.Q, dH = b.predH - a.H, err = Math.max(Math.abs(dQ) / TOL.hit, Math.abs(dH) / TOL.hit);
  const verdict = err <= 1 ? 'hit' : err <= TOL.near / TOL.hit ? 'near' : 'miss';
  const setupNotes = [];
  if (!b.pump) setupNotes.push('no pump piece is registered to the grid'); else if (b.pump.def.pump.id !== sc.pump.id) setupNotes.push(`the card needs ${sc.pump.id} (${sc.pump.short}), the board has ${b.pump.def.pump.id}`);
  if (!b.sys) setupNotes.push('no system piece is registered at Q = 0'); else { if (Math.abs(b.sys.def.sys.K - sc.sys.K) > 0.003) setupNotes.push(`system K on the board is ${b.sys.def.sys.K.toFixed(3)}, the card implies ${sc.sys.K.toFixed(3)}`); if (Math.abs(b.Hs - sc.sys.Hs) > 0.6) setupNotes.push(`static head on the board is ${fmt(b.Hs)} m, the card says ${sc.sys.Hs} m`); }
  if (b.bep) { const qb = a.Qbep * (sc.pump.count || 1), hb = M.head(a.Qbep, sc.pump.n, sc.pump.d); if (Math.abs(Qof(b.bep.x) - qb) > 1.5 || Math.abs(Hof(b.bep.y) - hb) > 1.5) setupNotes.push(`BEP dot is at ${fmt(Qof(b.bep.x))} m³/h; for this pump piece it belongs at ${fmt(qb)} m³/h on the curve`); }
  if (b.bar && b.bar.def.win.pump !== sc.pump.id) setupNotes.push(`the POR/AOR bar on the board is ${b.bar.def.win.id}; this pump piece needs ${M.WINDOWS.find(w => w.pump === sc.pump.id).id}`);
  v.className = 'verdict ' + verdict;
  v.innerHTML = `<b>${verdict === 'hit' ? 'Hit.' : verdict === 'near' ? 'Close.' : 'Off.'}</b> Predicted ${fmt(b.predQ)} m³/h at ${fmt(b.predH)} m; true point ${fmt(a.Q)} m³/h at ${fmt(a.H)} m (Δ ${dQ >= 0 ? '+' : ''}${fmt(dQ)} m³/h, ${dH >= 0 ? '+' : ''}${fmt(dH)} m). ${sc.pump.count ? `Each pump ${fmt(a.Qp)} m³/h. ` : ''}η ${fmt(a.eta * 100, 0)} %, shaft power ${fmt(a.P / 1000, 2)} kW, ${fmt(a.pct * 100, 0)} % of Q<sub>BEP</sub> — ${a.pct >= POR[0] && a.pct <= POR[1] ? 'inside the POR' : a.pct >= AOR[0] && a.pct <= AOR[1] ? 'outside the POR but inside the AOR' : 'outside the AOR'}.${setupNotes.length ? '<br><span class="note">Board set-up: ' + setupNotes.join('; ') + '.</span>' : ''}<br><span class="note">${sc.key}</span>`;
  showTruth(sc);
  const row = { i: state.log.length + 1, team: state.team || '—', card: sc.card, pump: b.pump ? b.pump.def.pump.id : '—', sys: b.sys ? `${b.sys.def.sys.id}@${fmt(b.Hs)}` : '—', pq: +b.predQ.toFixed(1), ph: +b.predH.toFixed(1), tq: +a.Q.toFixed(1), th: +a.H.toFixed(1), dq: +dQ.toFixed(1), dh: +dH.toFixed(1), verdict, eta: Math.round(a.eta * 100), kw: +(a.P / 1000).toFixed(2), pct: Math.round(a.pct * 100) };
  state.log.push(row); renderLog();
  if (verdict === 'hit' && setupNotes.length === 0) state.flags[sc.id] = true;
  if (verdict !== 'miss') state.flags['any_' + sc.id] = true;
  $('#pillScore').textContent = state.log.filter(r => r.verdict === 'hit').length + ' hits';
  document.querySelector(`#scsel [data-sc="${state.sc}"]`).classList.toggle('done', !!state.flags[sc.id]);
  checkTasks();
}
function renderLog() {
  const tb = $('#logTable tbody'); tb.innerHTML = state.log.map(r => `<tr><td>${r.i}</td><td>${r.team}</td><td>${r.card}</td><td>${r.pump}</td><td>${r.sys}</td><td>${r.pq}</td><td>${r.ph}</td><td>${r.tq}</td><td>${r.th}</td><td>${r.dq}</td><td>${r.dh}</td><td>${r.verdict}</td><td>${r.eta}</td><td>${r.kw}</td><td>${r.pct}</td></tr>`).join('');
  $('#logCount').textContent = state.log.length;
  state.csv = 'n,team,card,pump_piece,system_piece@Hs,pred_Q,pred_H,true_Q,true_H,dQ,dH,verdict,eta_pct,kW,pct_BEP\n' + state.log.map(r => [r.i, r.team, r.card, r.pump, r.sys, r.pq, r.ph, r.tq, r.th, r.dq, r.dh, r.verdict, r.eta, r.kw, r.pct].join(',')).join('\n');
  $('#btnDownload').href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(state.csv);
}

/* ------------------------------------------------------------------ scenarios */
function setScenario(i) {
  state.sc = i; const sc = M.SCENARIOS[i]; hideTruth();
  document.querySelectorAll('#scsel button').forEach(b => b.classList.toggle('on', +b.dataset.sc === i));
  $('#scLetter').textContent = sc.card; $('#scTitle').textContent = sc.title; $('#scBrief').textContent = sc.brief; $('#scAsk').textContent = sc.ask;
  $('#scPieces').textContent = 'Pieces: ' + sc.pieces.map(id => id + (PIECE_DEFS[id] && PIECE_DEFS[id].label ? ' (' + PIECE_DEFS[id].label.replace(/^\w+ · /, '') + ')' : '')).join(', ');
  $('#pillScenario').textContent = 'Card ' + sc.card; $('#verdict').className = 'verdict'; $('#verdict').textContent = 'Set up the board for this card, drag the ? marker to the crossing, then press Check.';
  refresh();
}

/* ------------------------------------------------------------------ tasks */
const TASKS = [
  { id: 'base', t: 'Commission the board', d: 'Card 0: register P1, S2 at 10 m and W1, put the BEP dot on the curve, and hit the operating point.', ok: () => state.flags.base },
  { id: 'strainer', t: 'Strainer fouls 40 %', d: 'Card A: swap S2 for S3, keep the static head, hit the new point.', ok: () => state.flags.strainer },
  { id: 'level', t: 'Tank level falls 5 m', d: 'Card B: slide S2 up to 15 m, hit the new point.', ok: () => state.flags.level },
  { id: 'throttle', t: 'Throttle to 18 m³/h', d: 'Card C: choose the system piece that crosses P1 at 18 m³/h, hit the point.', ok: () => state.flags.throttle },
  { id: 'vfd', t: 'VFD to 1 200 rpm', d: 'Card D: place P3 with its own bar W3 and BEP, hit the point.', ok: () => state.flags.vfd },
  { id: 'parallel', t: 'Second pump in parallel', d: 'Card E: place P8 and W8, hit the total flow point.', ok: () => state.flags.parallel },
  { id: 'trim', t: 'Trim for 18 m³/h', d: 'Card F: place P7 and W7, hit the point.', ok: () => state.flags.trim },
  { id: 'sweep', t: 'All four routes compared', d: 'Cards A to D each checked at least once (hit or close): which change cost the most flow, and which the least power?', ok: () => ['strainer', 'level', 'throttle', 'vfd'].every(k => state.flags['any_' + k]) },
];
function buildTasks() { const ol = $('#tasks'); ol.innerHTML = ''; for (const t of TASKS) { const li = document.createElement('li'); li.dataset.id = t.id; li.innerHTML = `<b>${t.t}</b><span>${t.d}</span><br><span class="st">not yet</span>`; ol.appendChild(li); } }
function checkTasks() { for (const t of TASKS) { const li = document.querySelector(`.tasks li[data-id="${t.id}"]`); if (!li) continue; let ok = false; try { ok = t.ok(); } catch (e) {} if (ok) li.classList.add('done'); li.querySelector('.st').textContent = li.classList.contains('done') ? 'done ✓' : 'not yet'; } }

/* ------------------------------------------------------------------ wiring */
function buildTray() {
  const mk = (parent, id, label) => { const b = document.createElement('button'); b.dataset.piece = id; b.textContent = label; b.title = PIECE_DEFS[id].label; b.onclick = () => addPiece(id); $(parent).appendChild(b); };
  for (const p of M.PUMPS) mk('#trayPumps', p.id, `${p.id} ${p.short}`);
  for (const S of M.SYSTEMS) mk('#traySystems', S.id, `${S.id} ${S.K.toFixed(3)}`);
  for (const w of M.WINDOWS) mk('#trayBars', w.id, `${w.id} ${M.PUMPS.find(p => p.id === w.pump).short}`);
  document.querySelectorAll('#tray .grp:nth-child(4) [data-piece]').forEach(b => b.onclick = () => addPiece(b.dataset.piece));
  M.SCENARIOS.forEach((sc, i) => { const b = document.createElement('button'); b.dataset.sc = i; b.textContent = sc.card; b.title = sc.title; b.onclick = () => setScenario(i); $('#scsel').appendChild(b); });
}
function bind() {
  $('#btnClear').onclick = clearBoard; $('#btnBaseline').onclick = placeBaseline; $('#btnCheck').onclick = check;
  $('#btnReveal').onclick = () => showTruth(M.SCENARIOS[state.sc]); $('#btnHide').onclick = hideTruth;
  $('#team').addEventListener('input', e => state.team = e.target.value.trim());
  $('#btnMode').onclick = () => { state.projector = !state.projector; document.documentElement.classList.toggle('projector', state.projector); gPieces.style.display = state.projector ? 'none' : ''; $('#pillMode').classList.toggle('hidden', !state.projector); $('#btnMode').classList.toggle('on', state.projector); refresh(); };
  $('#btnCopy').onclick = () => { try { navigator.clipboard.writeText(state.csv || ''); $('#btnCopy').textContent = 'Copied'; setTimeout(() => $('#btnCopy').textContent = 'Copy as CSV', 1500); } catch (e) { alert('Clipboard not available here — use Download CSV.'); } };
  $('#btnClearLog').onclick = () => { state.log = []; renderLog(); };
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => { document.querySelectorAll('.tab').forEach(x => x.classList.remove('on')); document.querySelectorAll('.tabpanel').forEach(x => x.classList.remove('on')); t.classList.add('on'); $('#tab-' + t.dataset.tab).classList.add('on'); });
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('opb_theme', next); } catch (e) {} };
  $('#btnHelp').onclick = () => $('#help').classList.remove('hidden'); $('#btnHelpClose').onclick = () => $('#help').classList.add('hidden'); $('#help').addEventListener('click', e => { if (e.target === $('#help')) $('#help').classList.add('hidden'); });
  document.addEventListener('keydown', e => { if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return; const k = e.key.toUpperCase();
    if (k === '0') setScenario(0); else if (/^[A-F]$/.test(k) && !e.metaKey && !e.ctrlKey) { const i = M.SCENARIOS.findIndex(s => s.card === k); if (i >= 0) setScenario(i); else return; } else if (e.key === 'Enter') check(); else if (k === 'R') showTruth(M.SCENARIOS[state.sc]); else if (e.key === 'Escape') $('#help').classList.add('hidden'); else return; e.preventDefault(); });
}

/* ------------------------------------------------------------------ dashboard API */
window.OperatingPointBoard = {
  getState: () => { const b = boardState(); return { inputs: { scenario: M.SCENARIOS[state.sc].card, team: state.team, pieces: state.pieces.map(p => ({ id: p.id, Q: Qof(p.x), H: Hof(p.y), registered: p.reg })), projector: state.projector }, results: { predQ: b.predQ, predH: b.predH, trueQ: M.SCENARIOS[state.sc].ans.Q, trueH: M.SCENARIOS[state.sc].ans.H, hits: state.log.filter(r => r.verdict === 'hit').length, attempts: state.log.length }, tasksDone: TASKS.filter(t => document.querySelector(`.tasks li[data-id="${t.id}"]`)?.classList.contains('done')).map(t => t.id) }; },
  setState: inp => { if (inp.scenario !== undefined) { const i = M.SCENARIOS.findIndex(s => s.card === String(inp.scenario)); if (i >= 0) setScenario(i); } if (inp.team !== undefined) { state.team = String(inp.team); $('#team').value = state.team; } if (inp.baseline) placeBaseline(); if (inp.clear) clearBoard(); if (inp.reveal) showTruth(M.SCENARIOS[state.sc]); if (inp.projector !== undefined && !!inp.projector !== state.projector) $('#btnMode').click(); },
  meta: { id: 'operating-point-board', title: 'Operating-Point Board', version: '1.0' },
};
window.addEventListener('message', e => { const d = e.data || {}; if (d.type === 'sim:set') window.OperatingPointBoard.setState(d.inputs || {}); if (d.type === 'sim:get') (e.source || window).postMessage({ type: 'sim:state', id: 'operating-point-board', state: window.OperatingPointBoard.getState() }, '*'); });

/* ------------------------------------------------------------------ boot */
function build() {
  try { const th = localStorage.getItem('opb_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildBoard(); buildTray(); buildTasks(); bind(); renderLog(); setScenario(0); $('#btnHide').classList.add('hidden');
}
build();
