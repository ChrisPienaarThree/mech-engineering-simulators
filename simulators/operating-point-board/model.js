/* Operating-Point Board — shared model for the physical (printed) board and the digital board.
   Same fictional 4 kW pump as the other simulators. Loaded as a classic script (window.OPB) and by node (vm).
   [Illustrative data, for training only.] */
'use strict';
(function (root) {
  const PUMP = { N0: 1480, D0: 174, H0: 32, a1: 0.2, a2: 0.01, Qbep: 24, etaMax: 0.72, kW: 4, name: 'Loop-bench pump, 4 kW, 4-pole' };
  const SYSTEM = { Hs0: 10, Kpipe: 0.015, Kstr: 0.005 };            // static head m; K in m per (m³/h)²
  const CHART = { Qmax: 50, Hmax: 36, HsysRel: 26, sx: 14, sy: 12.5, ox: 95, oy: 530 };   // system pieces span 26 m above their base so they fit the chart from a 10 m static head   // A1 board: mm per m³/h, mm per m, plot origin (mm from top-left)
  const POR = [0.7, 1.2], AOR = [0.5, 1.3];                         // fractions of Q_BEP (POR per ANSI/HI 9.6.3; AOR illustrative)
  const G = 9.81, RHO = 1000;
  const head = (Q, n = 1, d = 1) => { const s = n * d; if (s <= 0.02) return 0; const q = Q / s; return s * s * Math.max(0, PUMP.H0 - PUMP.a1 * q - PUMP.a2 * q * q); };
  const eta = (Q, n = 1, d = 1) => { const q = Q / (n * d); return Math.max(0.03, PUMP.etaMax * (1 - Math.pow((q - PUMP.Qbep) / PUMP.Qbep, 2))); };
  const runout = (n = 1, d = 1) => n * d * (-PUMP.a1 + Math.sqrt(PUMP.a1 * PUMP.a1 + 4 * PUMP.a2 * PUMP.H0)) / (2 * PUMP.a2);
  const qAtH = (H, n = 1, d = 1) => { if (H >= head(0, n, d)) return 0; let lo = 0, hi = runout(n, d); for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (head(m, n, d) > H) lo = m; else hi = m; } return (lo + hi) / 2; };
  const sysHead = (Q, sys) => sys.Hs + sys.K * Q * Q;
  const power = (Q, H, e) => RHO * G * Q / 3600 * H / e;            // W
  // operating point of `count` identical pumps (n, d) in parallel against system {Hs, K}
  function opPoint(sys, pump) {
    const { n = 1, d = 1, count = 1 } = pump; const H0 = head(0, n, d); if (H0 <= sys.Hs) return { Q: 0, H: sys.Hs, dead: true };
    let lo = sys.Hs, hi = H0; for (let i = 0; i < 60; i++) { const H = (lo + hi) / 2, Qtot = count * qAtH(H, n, d); if (sysHead(Qtot, sys) < H) hi = H; else lo = H; }
    const H = (lo + hi) / 2, Qtot = count * qAtH(H, n, d), Qp = Qtot / count, e = eta(Qp, n, d);
    return { Q: Qtot, H, Qp, eta: e, P: count * power(Qp, H, e), Qbep: PUMP.Qbep * n * d, pct: Qp / (PUMP.Qbep * n * d) };
  }
  const K_STRAINER_FOULED = SYSTEM.Kpipe + SYSTEM.Kstr / (0.6 * 0.6);   // 40 % of the strainer area blocked: loss ∝ 1/area²
  const kForFlow = (Qt, sys, pump) => (head(Qt / (pump.count || 1), pump.n, pump.d) - sys.Hs) / (Qt * Qt);
  const dForFlow = (Qt, sys) => { let lo = 0.5, hi = 1; for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (opPoint(sys, { d: m }).Q < Qt) lo = m; else hi = m; } return (lo + hi) / 2; };

  /* pieces: every printed / draggable piece, in model units */
  const PUMPS = [
    { id: 'P1', n: 1, d: 1, label: '1 480 rpm · Ø174 full impeller', short: '1480 rpm', kind: 'speed' },
    { id: 'P2', n: 1350 / 1480, d: 1, label: '1 350 rpm', short: '1350 rpm', kind: 'speed' },
    { id: 'P3', n: 1200 / 1480, d: 1, label: '1 200 rpm', short: '1200 rpm', kind: 'speed' },
    { id: 'P4', n: 1000 / 1480, d: 1, label: '1 000 rpm', short: '1000 rpm', kind: 'speed' },
    { id: 'P5', n: 1, d: 165 / 174, label: 'Ø165 mm trimmed · 1 480 rpm', short: 'Ø165', kind: 'trim' },
    { id: 'P6', n: 1, d: 155 / 174, label: 'Ø155 mm trimmed · 1 480 rpm', short: 'Ø155', kind: 'trim' },
    { id: 'P7', n: 1, d: 147 / 174, label: 'Ø147 mm trimmed · 1 480 rpm', short: 'Ø147', kind: 'trim' },
    { id: 'P8', n: 1, d: 1, count: 2, label: 'Two pumps in parallel · 1 480 rpm · Ø174', short: '2 × parallel', kind: 'parallel' },
  ];
  const SYSTEMS = [
    { id: 'S1', K: 0.014, label: 'K = 0.014 (larger bore)' },
    { id: 'S2', K: 0.020, label: 'K = 0.020 baseline · clean strainer · valve open' },
    { id: 'S3', K: +K_STRAINER_FOULED.toFixed(4), label: 'K = 0.029 strainer 40 % blocked' },
    { id: 'S4', K: 0.047, label: 'K = 0.047 discharge throttled' },
    { id: 'S5', K: 0.070, label: 'K = 0.070 heavily throttled' },
  ];
  const baseline = { Hs: SYSTEM.Hs0, K: SYSTEM.Kpipe + SYSTEM.Kstr };
  const P1 = PUMPS[0], P3 = PUMPS[2], P8 = PUMPS[7];
  const base = opPoint(baseline, P1);
  const kThr = kForFlow(18, baseline, P1), dTrim = dForFlow(18, baseline);
  const SCENARIOS = [
    { id: 'base', card: '0', title: 'Commission the board', brief: 'Pump at 1 480 rpm with the full Ø174 impeller. Supply tank level gives a static head of 10 m; clean strainer, discharge valve fully open (K = 0.020). Place the pump curve, the system curve and the POR/AOR bar, mark the BEP, and mark the operating point.',
      pieces: ['P1', 'S2', 'W1', 'BEP', 'OP'], sys: baseline, pump: P1, ans: base,
      ask: 'Where does the pump run relative to the BEP? Is it inside the POR?', key: `The curves cross at ${base.Q.toFixed(1)} m³/h and ${base.H.toFixed(1)} m — right on the BEP (${(base.pct * 100).toFixed(0)} % of Q_BEP), efficiency ${(base.eta * 100).toFixed(0)} %, shaft power ${(base.P / 1000).toFixed(2)} kW. The board is now the reference for every scenario.` },
    { id: 'strainer', card: 'A', title: 'Strainer fouls by 40 %', brief: 'Debris blocks 40 % of the suction strainer area. Nothing else changes. Loss through a screen goes with 1/(open area)², so the strainer coefficient rises by (1/0.6)² = 2.8×: K goes from 0.020 to 0.029.',
      pieces: ['P1', 'S3', 'W1', 'BEP', 'OP'], sys: { Hs: baseline.Hs, K: K_STRAINER_FOULED }, pump: P1,
      ask: 'Swap the system piece. Predict the new Q and H. Does the pump stay inside the POR? What happens to the suction pressure?',
      key: 'Same static head, steeper parabola: the operating point slides up and left along the pump curve. Flow falls, head rises, the pump moves left of BEP but stays inside the POR. The loss is on the suction side, so NPSH margin falls — the cavitation trainer shows what comes next.' },
    { id: 'level', card: 'B', title: 'Supply tank level falls 5 m', brief: 'The supply tank is being drawn down and its level drops 5 m. The pump lifts from the tank to the same delivery point, so the static head rises from 10 m to 15 m. Friction unchanged (K = 0.020).',
      pieces: ['P1', 'S2', 'W1', 'BEP', 'OP'], sys: { Hs: baseline.Hs + 5, K: baseline.K }, pump: P1,
      ask: 'Slide the same system piece up 5 m. Predict Q and H. Compare the flow loss with scenario A: which change hurt more?',
      key: 'The parabola keeps its shape and moves up 5 m. The operating point moves up and left along the pump curve; the whole curve shift costs more flow than the strainer did. Static head is the one thing a bigger valve opening cannot buy back.' },
    { id: 'throttle', card: 'C', title: 'Operator throttles the discharge to 18 m³/h', brief: 'Production asks for 18 m³/h instead of 24. The operator closes the discharge valve until the flowmeter reads 18 m³/h. Static head 10 m; the extra loss is all in the valve.',
      pieces: ['P1', 'S4', 'W1', 'BEP', 'OP'], sys: { Hs: baseline.Hs, K: kThr }, pump: P1, targetQ: 18,
      ask: 'Which system piece crosses the pump curve at 18 m³/h? Predict the head. How much head is being burned in the valve, and what is the shaft power?',
      key: `The pump curve at 18 m³/h reads ${head(18).toFixed(1)} m, so the system parabola must pass through that point: K ≈ ${kThr.toFixed(3)} (piece S4). The valve is destroying ${((kThr - baseline.K) * 18 * 18).toFixed(1)} m of head — ${(((kThr - baseline.K) * 18 * 18) / head(18) * 100).toFixed(0)} % of what the pump produces. Shaft power ${(power(18, head(18), eta(18)) / 1000).toFixed(2)} kW.` },
    { id: 'vfd', card: 'D', title: 'VFD reduces speed from 1 480 to 1 200 rpm', brief: 'Instead of throttling, the same flow reduction is attempted with the VFD. Speed goes to 1 200 rpm; the valve stays fully open (K = 0.020, static head 10 m).',
      pieces: ['P3', 'S2', 'W2', 'BEP', 'OP'], sys: baseline, pump: P3,
      ask: 'Place the 1 200 rpm curve and move the POR/AOR bar and the BEP with it. Predict Q and H. Compare the shaft power with scenario C at the same flow.',
      key: `Flow scales with speed but head with speed², so the curve shrinks toward the origin and the operating point lands at about ${opPoint(baseline, P3).Q.toFixed(1)} m³/h and ${opPoint(baseline, P3).H.toFixed(1)} m. The BEP moves to ${(PUMP.Qbep * P3.n).toFixed(1)} m³/h, so the pump still runs at ${(opPoint(baseline, P3).pct * 100).toFixed(0)} % of BEP flow — inside the POR. Shaft power ${(opPoint(baseline, P3).P / 1000).toFixed(2)} kW against ${(power(opPoint(baseline, P3).Q, head(opPoint(baseline, P3).Q), eta(opPoint(baseline, P3).Q)) / 1000).toFixed(2)} kW if the same flow were throttled: the static head keeps the saving below the cube law.` },
    { id: 'parallel', card: 'E', title: 'Bonus — second pump started in parallel', brief: 'A second identical pump is started in parallel to "double the flow". Baseline system (static head 10 m, K = 0.020), both at 1 480 rpm.',
      pieces: ['P8', 'S2', 'W7', 'BEP', 'OP'], sys: baseline, pump: P8,
      ask: 'Place the parallel curve. Predict the total flow and the flow per pump. Where does each pump sit in its POR?',
      key: `Total flow rises only to ${opPoint(baseline, P8).Q.toFixed(1)} m³/h, ${((opPoint(baseline, P8).Q / base.Q - 1) * 100).toFixed(0)} % more, not double, because the system curve is steep. Each pump delivers ${opPoint(baseline, P8).Qp.toFixed(1)} m³/h — ${(opPoint(baseline, P8).pct * 100).toFixed(0)} % of BEP, left of the POR. Two pumps in parallel only pay off on flat (friction-light) systems.` },
    { id: 'trim', card: 'F', title: 'Bonus — trim the impeller for 18 m³/h', brief: 'The 18 m³/h duty is permanent. Instead of throttling (C) or a VFD (D), the impeller is trimmed. Baseline system, 1 480 rpm.',
      pieces: ['P7', 'S2', 'W6', 'BEP', 'OP'], sys: baseline, pump: PUMPS[6],
      ask: 'Which trimmed curve passes closest to 18 m³/h on the baseline system? Predict Q and H and the shaft power. What is the trim limit?',
      key: `A diameter ratio of ${dTrim.toFixed(3)} (Ø${(dTrim * PUMP.D0).toFixed(0)} mm) gives 18 m³/h; the Ø147 piece is the nearest. Operating point about ${opPoint(baseline, PUMPS[6]).Q.toFixed(1)} m³/h and ${opPoint(baseline, PUMPS[6]).H.toFixed(1)} m at ${(opPoint(baseline, PUMPS[6]).P / 1000).toFixed(2)} kW. A ${((1 - dTrim) * 100).toFixed(0)} % trim is beyond the usual 10–15 % limit where efficiency starts to suffer; the affinity laws for diameter are only reliable for small trims.` },
  ];
  for (const s of SCENARIOS) if (!s.ans) s.ans = opPoint(s.sys, s.pump);
  const WINDOWS = PUMPS.map((p, i) => ({ id: 'W' + (i + 1), pump: p.id, Qbep: PUMP.Qbep * p.n * p.d * (p.count || 1), label: `POR / AOR for ${p.short}` }));
  root.OPB = { PUMP, SYSTEM, CHART, POR, AOR, head, eta, runout, qAtH, sysHead, power, opPoint, kForFlow, dForFlow, PUMPS, SYSTEMS, WINDOWS, SCENARIOS, baseline, K_STRAINER_FOULED };
})(typeof window !== 'undefined' ? window : globalThis);
