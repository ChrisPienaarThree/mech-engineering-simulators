"use strict";
const FAC = false;
const ENC = true;
const dec = x => ENC ? atob(x).split("").reverse().join("") : x;
const $ = id => document.getElementById(id);

// ---------- model data ----------
const SYS = { // flows, targets, pressures and pump sizes are teaching values
  eff: {name:"Wastewater: ferric chloride", chem:"Ferric chloride 40 % w/w (ρ 1.42)", S:568, vol:false, flow:250, unit:"mg/L", target:30,
        pinj:0.0, siphonBar:0.56, qmax:20, s0:5, spm:120, prv:8, bpv:2, col:1000, tank:1000},
  hcl: {name:"Desalination: HCl", chem:"Hydrochloric acid 33 % w/w (ρ 1.16)", S:383, vol:false, flow:150, unit:"mg/L", target:50,
        pinj:6.0, siphonBar:0.0, qmax:30, s0:5, spm:100, prv:10, bpv:0, col:1000, tank:1000},
  caustic:{name:"Wastewater pH: caustic soda", chem:"Caustic soda 50 % w/w (ρ 1.52)", S:760, vol:false, flow:300, unit:"mg/L", target:20,
        pinj:0.5, siphonBar:0.0, qmax:12, s0:5, spm:100, prv:8, bpv:2, col:500, tank:1000},
  cw:  {name:"Extra: cooling water", chem:"Sodium hypochlorite 12.5 % w/v", S:125, vol:false, flow:2000, unit:"mg/L", target:1.0,
        pinj:0.3, siphonBar:0.0, qmax:25, s0:5, spm:120, prv:10, bpv:3, col:1000, tank:1000, gassy:true},
  bfw: {name:"Extra: boiler feed water", chem:"Sodium sulfite 20 % w/w (ρ 1.18)", S:236, vol:false, flow:60, unit:"mg/L", target:3.0,
        pinj:1.0, siphonBar:0.0, qmax:2, s0:6, spm:100, prv:10, bpv:3, col:100, tank:200},
  proc:{name:"Extra: process inhibitor", chem:"Corrosion inhibitor (dose in ppm v/v)", S:null, vol:true, flow:400, unit:"ppm v/v", target:15,
        pinj:25, siphonBar:0.0, qmax:10, s0:4, spm:100, prv:35, bpv:0, col:500, tank:500},
};
const FAULTS = {
  none:{t:"No fault"}, gas:{t:"Gas lock"}, dcv:{t:"Discharge check valve leaking"}, bpvf:{t:"BPV failed (open)"},
  damper:{t:"Pulsation damper flat"}, quill:{t:"Injection quill blocked"}, weak:{t:"Product weaker than label"},
  strainer:{t:"Strainer blocked"}, diaphragm:{t:"Diaphragm ruptured"}, gear:{t:"Gearbox oil low"},
};
const ACTIONS = [
  ["degas","Degas / re-prime","Open the bleed valve and run at 100 % stroke until bubble-free","gas"],
  ["valves","Service check valves","Isolate and clean or replace the check valves","dcv"],
  ["bpvfix","Repair BPV","Replace or reset the back-pressure valve","bpvf"],
  ["recharge","Recharge damper","Restore the damper pre-charge with nitrogen","damper"],
  ["quillfix","Clear quill","Isolate, withdraw and clear the injection quill","quill"],
  ["product","Test / replace product","Test product strength; replace the batch if weak","weak"],
  ["strainerfix","Clean strainer","Isolate and clean the Y-strainer","strainer"],
  ["diaphragmfix","Change diaphragm","Isolate, decontaminate and fit a new diaphragm","diaphragm"],
  ["oilfix","Check / top up gearbox oil","Stop and isolate, check oil level and condition, fill with the OEM grade","gear"],
  ["refill","Refill day tank","Supervised delivery into the day tank",null],
];
// Scenario cards (codes match the facilitator manual appendix). Faults hidden from the learner.
const SCEN = {
  "D2-1":{sys:"eff",fault:"ZW5vbg==",text:"Target 30 mg/L ferric chloride. Calculate the pump rate, set stroke and speed, prove it by drawdown within ±3 %."},
  "D2-2":{sys:"hcl",fault:"ZW5vbg==",text:"Target 50 mg/L as HCl. Same task."},
  "D2-3":{sys:"caustic",fault:"ZW5vbg==",text:"Target 20 mg/L as NaOH. Same task."},
  "D2-4":{sys:"cw",fault:"ZW5vbg==",text:"Extra: target 1.0 mg/L available chlorine. Same task."},
  "D3-1":{sys:"cw",fault:"c2Fn",set:true},"D3-2":{sys:"hcl",fault:"dmNk",set:true},"D3-3":{sys:"eff",fault:"ZnZwYg==",set:true},
  "D3-4":{sys:"hcl",fault:"cmVwbWFk",set:true},"D3-5":{sys:"caustic",fault:"bGxpdXE=",set:true},"D3-6":{sys:"cw",fault:"a2Fldw==",set:true},
  "D3-7":{sys:"eff",fault:"cmVuaWFydHM=",set:true},"D3-8":{sys:"hcl",fault:"bWdhcmhwYWlk",set:true},"D3-9":{sys:"caustic",fault:"cmFlZw==",set:true},
  "A-1":{sys:"hcl",fault:"ZW5vbg==",text:"Observed practical A-1: set the target dose and prove it by drawdown within ±3 %."},
  "A-2":{sys:"eff",fault:"ZW5vbg==",off:-0.12,text:"Observed practical A-2: the pump is running off target. Calibrate it and correct the setting to within ±3 %."},
  "A-3":{sys:"caustic",fault:"dmNk",set:true},
};
// Learner edition carries codes in lightly obfuscated form so the fault is not visible in the card text.
const CODES = {"D2-1":"KT41","D2-2":"MW82","D2-3":"PR53","D2-4":"BZ27","D3-1":"QF19","D3-2":"HX64","D3-3":"VN38","D3-4":"JC75",
  "D3-5":"LD96","D3-6":"RG42","D3-7":"YS13","D3-8":"WE58","D3-9":"GM63","A-1":"TA21","A-2":"TB34","A-3":"TC47"};

// ---------- state ----------
const st = {
  sys:"cw", running:false, mode:"manual", stroke:50, speed:100, flowF:100, bpv:3, fault:"none",
  tankL:0, colmL:0, tankValve:true, colValve:false, timer:0, timing:false, marks:[], t:0,
  hist:[], lag:[], analyser:0, prvLift:false, p:0, amp:0, q:0, qs:0, toProc:0, actions:0, scenario:null, off:0, gbT:25, trip:false
};
function sp(){ return SYS[st.sys]; }

function log(msg, cls){
  const d = document.createElement("div");
  d.innerHTML = `<span style="color:var(--mute)">${fmtT(st.t)}</span> ${msg}`;
  if (cls) d.className = cls;
  $("log").prepend(d);
}
function fmtT(t){ const m = Math.floor(t/60), s = Math.floor(t%60); return `${m}:${String(s).padStart(2,"0")}`; }

function loadSystem(id, keepFault){
  st.sys = id; const s = sp();
  st.tankL = s.tank * 0.7; st.colmL = 0; st.tankValve = true; st.colValve = false; st.bpv = s.bpv;
  st.hist = []; st.lag = []; st.analyser = 0; st.timer = 0; st.timing = false; st.marks = []; st.gbT = 25; st.trip = false;
  if (!keepFault) st.fault = "none";
  $("sys").value = id; $("bpv").value = s.bpv; $("prvRo").textContent = s.prv;
  $("chem").textContent = s.chem; $("doseUnit").textContent = s.unit; $("colCap").textContent = `mL of ${s.col}`;
  $("cDose").value = s.target; $("cFlow").value = s.flow; $("cS").value = s.S ?? ""; calc();
}

// ---------- physics, 10 Hz ----------
function deliveryFrac(stroke, s0){ return Math.max(0, Math.min(1, (stroke - s0) / (100 - s0))); }
function step(dt){
  const s = sp(); st.t += dt;
  const flow = s.flow * st.flowF / 100;
  if (st.mode === "paced") { st.speed = Math.min(100, Math.round(100 * flow / (s.flow * 1.5))); }
  const f = st.fault;
  // gearbox: low oil overheats it over a few minutes, then the motor trips on overload
  const gbTarget = st.running ? 25 + 20 * st.speed / 100 + (f === "gear" ? 65 : 0) : 25;
  st.gbT += (gbTarget - st.gbT) * dt / 180;
  if (st.running && f === "gear" && st.gbT >= 95) { st.running = false; st.trip = true; log("Motor tripped on overload", "bad"); }
  const supplyOk = st.tankValve ? st.tankL > 2 : (st.colValve && st.colmL > 1);
  const bpvEff = f === "bpvf" ? 0 : Math.max(0, st.bpv);
  let pMean = Math.max(s.pinj + 0.2, bpvEff);
  let q = 0;
  if (st.running) {
    const base = s.qmax * st.speed / 100 * deliveryFrac(st.stroke, s.s0) * (1 + st.off);
    let eta = 1 - 0.0025 * pMean;
    if (f === "gas") eta *= 0.3 + 0.2 * Math.random();
    if (f === "dcv") eta *= Math.max(0.3, 0.85 - 0.012 * pMean);
    if (f === "strainer") eta *= 0.45 + 0.1 * Math.random();
    if (f === "diaphragm") eta *= 0.55;
    if (!supplyOk) eta = 0;
    q = base * eta * (1 + 0.01 * (Math.random() - 0.5) * 2);
  }
  // blocked quill: pressure climbs to PRV, everything relieves back to tank
  st.prvLift = false;
  if (st.running && f === "quill" && q > 0) { pMean = s.prv; st.prvLift = true; }
  st.amp = st.running && q > 0 ? (f === "damper" ? 0.55 * pMean + 1.5 : 0.06 * pMean + 0.1) : 0;
  if (st.running && !st.prvLift && pMean + st.amp > s.prv) { st.prvLift = true; q *= 0.75; }
  // siphon: liquid head above a low-pressure injection point with no working BPV
  st.qs = 0;
  if (s.siphonBar > s.pinj && bpvEff < s.siphonBar && supplyOk && f !== "quill") st.qs = 9 * Math.sqrt(s.siphonBar - Math.max(s.pinj, bpvEff));
  const draw = q + st.qs;                       // L/h leaving the suction
  st.toProc = f === "quill" ? 0 : draw;
  const dL = draw / 3600 * dt;
  if (!st.tankValve && st.colValve) st.colmL = Math.max(0, st.colmL - dL * 1000);
  else st.tankL = Math.max(0, st.tankL - dL);
  if (st.tankValve && st.colValve) { const fill = Math.min(s.col - st.colmL, s.col / 15 * dt); st.colmL += fill; st.tankL -= fill / 1000; }
  if (st.prvLift && f === "quill") st.tankL += dL; // relieved liquid returns to the tank
  st.q = q; st.p = st.running ? pMean : s.pinj;
  // dose at the dosing point, then dead time + first-order lag to the analyser
  const Sact = (s.S || 0) * (f === "weak" ? 0.6 : 1);
  const dose = s.vol ? st.toProc * 1000 / flow * (f === "weak" ? 0.6 : 1) : st.toProc * Sact / flow;
  st.lag.push(dose); const dead = 150; // 15 s dead time at 10 Hz
  const delayed = st.lag.length > dead ? st.lag.shift() : 0;
  st.analyser += (delayed - st.analyser) * dt / 20;
  if (st.timing) st.timer += dt;
  if (Math.round(st.t * 10) % 10 === 0) { st.hist.push([st.analyser, st.p]); if (st.hist.length > 180) st.hist.shift(); }
}

// ---------- drawing ----------
function drawSkid(){
  const s = sp(), f = st.fault, run = st.running && st.q > 0;
  const bub = run && (f === "gas" || f === "strainer");
  const chem = "var(--chem)", ink = "var(--ink)", mute = "var(--mute)";
  const lvl = Math.max(0, Math.min(1, st.tankL / s.tank)), col = Math.max(0, Math.min(1, st.colmL / s.col));
  const pulse = run ? (Math.sin(st.t * Math.PI * 2 * (s.spm * st.speed / 100) / 60) + 1) / 2 : 0;
  const shake = f === "damper" && run ? 3 * Math.sin(st.t * 40) : 0;
  const y = 210;
  let g = `<rect x="20" y="250" width="170" height="36" fill="none" stroke="${mute}" stroke-dasharray="6 4"/>`;
  g += `<rect x="40" y="70" width="110" height="170" fill="none" stroke="${ink}" stroke-width="2"/>`;
  g += `<rect x="41" y="${70 + 170 * (1 - lvl)}" width="108" height="${170 * lvl}" fill="${chem}" opacity=".35"/>`;
  g += `<text x="95" y="60" text-anchor="middle" fill="${ink}" font-size="14" font-weight="bold">Day tank</text>`;
  g += `<line x1="150" y1="${y}" x2="400" y2="${y}" stroke="${chem}" stroke-width="5"/>`;
  g += valveSym(180, y, st.tankValve, "Tank valve");
  g += `<line x1="235" y1="${y}" x2="235" y2="160" stroke="${chem}" stroke-width="3"/>`;
  g += valveSym(235, 175, st.colValve, "", true);
  g += `<rect x="223" y="40" width="24" height="110" fill="none" stroke="${ink}" stroke-width="2"/>`;
  g += `<rect x="224" y="${40 + 110 * (1 - col)}" width="22" height="${110 * col}" fill="${chem}" opacity=".5"/>`;
  for (let i = 0; i <= 10; i++) g += `<line x1="223" y1="${40 + i * 11}" x2="${i % 5 ? 230 : 235}" y2="${40 + i * 11}" stroke="${ink}"/>`;
  g += `<text x="255" y="52" fill="${ink}" font-size="12">Calibration</text><text x="255" y="66" fill="${ink}" font-size="12">column</text>`;
  g += `<path d="M300,${y} l22,0 l-11,24 z" fill="${f === "strainer" ? "var(--warn)" : "none"}" stroke="${ink}" stroke-width="2"/><text x="311" y="${y + 42}" text-anchor="middle" fill="${ink}" font-size="12">Strainer</text>`;
  if (bub) for (let i = 0; i < 5; i++) g += `<circle cx="${340 + ((st.t * 60 + i * 13) % 55)}" cy="${y - 1}" r="3" fill="#fff" stroke="${chem}"/>`;
  g += `<line x1="440" y1="${y - 38}" x2="440" y2="${y - 48}" stroke="${ink}" stroke-width="2"/><rect x="408" y="${y - 82}" width="64" height="34" rx="3" fill="${st.gbT > 75 ? "var(--bad)" : st.gbT > 60 ? "var(--warn)" : "var(--card)"}" stroke="${ink}" stroke-width="2"/>`;
  g += `<text x="440" y="${y - 69}" text-anchor="middle" fill="${st.gbT > 60 ? "#fff" : ink}" font-size="10">Gearbox</text><text x="440" y="${y - 55}" text-anchor="middle" fill="${st.gbT > 60 ? "#fff" : ink}" font-size="11" font-weight="bold">${st.gbT.toFixed(0)} °C</text>`;
  g += `<circle cx="440" cy="${y}" r="38" fill="none" stroke="${f === "diaphragm" ? "var(--bad)" : ink}" stroke-width="3"/>`;
  g += `<circle cx="440" cy="${y}" r="${10 + 14 * pulse}" fill="${chem}" opacity=".3"/>`;
  g += `<text x="440" y="${y + 5}" text-anchor="middle" fill="${ink}" font-size="12" font-weight="bold">PUMP</text>`;
  g += `<text x="440" y="${y + 58}" text-anchor="middle" fill="${ink}" font-size="12">${st.running ? "Running" : "Stopped"} · ${st.stroke}% / ${st.speed}%</text>`;
  g += `<line x1="478" y1="${y + shake}" x2="900" y2="${y + shake}" stroke="${chem}" stroke-width="5"/>`;
  // gauge
  const ang = -120 + 240 * Math.min(1, (st.p + st.amp * (pulse - 0.5) * 2) / (s.prv * 1.2));
  g += `<line x1="510" y1="${y}" x2="510" y2="${y - 40}" stroke="${ink}" stroke-width="2"/><circle cx="510" cy="${y - 58}" r="18" fill="var(--card)" stroke="${ink}" stroke-width="2"/>`;
  g += `<line x1="510" y1="${y - 58}" x2="${510 + 14 * Math.sin(ang * Math.PI / 180)}" y2="${y - 58 - 14 * Math.cos(ang * Math.PI / 180)}" stroke="var(--bad)" stroke-width="2"/>`;
  // PRV
  g += `<line x1="570" y1="${y}" x2="570" y2="${y - 50}" stroke="${ink}" stroke-width="2"/><rect x="558" y="${y - 82}" width="24" height="32" fill="${st.prvLift ? "var(--bad)" : "var(--card)"}" stroke="var(--bad)" stroke-width="2"/>`;
  g += `<text x="590" y="${y - 70}" fill="var(--bad)" font-size="12">PRV ${st.prvLift ? "LIFTING" : ""}</text>`;
  // damper
  g += `<line x1="650" y1="${y}" x2="650" y2="${y - 30}" stroke="${ink}" stroke-width="2"/><ellipse cx="650" cy="${y - 58}" rx="20" ry="28" fill="none" stroke="${ink}" stroke-width="2"/>`;
  g += `<ellipse cx="650" cy="${y - 58}" rx="${f === "damper" ? 4 : 12 + 4 * pulse}" ry="${f === "damper" ? 6 : 18}" fill="${mute}" opacity=".5"/>`;
  g += `<text x="650" y="${y + 40}" text-anchor="middle" fill="${ink}" font-size="12">Damper</text>`;
  // BPV
  g += `<rect x="730" y="${y - 14}" width="34" height="28" fill="${f === "bpvf" ? "var(--warn)" : "var(--card)"}" stroke="${ink}" stroke-width="2"/>`;
  g += `<text x="747" y="${y + 40}" text-anchor="middle" fill="${ink}" font-size="12">BPV ${st.bpv} bar</text>`;
  // process + quill
  g += `<rect x="900" y="60" width="60" height="230" fill="none" stroke="${ink}" stroke-width="2"/>`;
  g += `<rect x="901" y="61" width="58" height="228" fill="${chem}" opacity="${Math.min(.5, st.analyser / (s.target * 4))}"/>`;
  g += `<line x1="900" y1="${y}" x2="932" y2="${y}" stroke="${f === "quill" ? "var(--bad)" : chem}" stroke-width="5"/>`;
  g += `<text x="960" y="50" text-anchor="end" fill="${ink}" font-size="13" font-weight="bold">${s.name}</text>`;
  g += `<text x="930" y="282" text-anchor="middle" fill="${ink}" font-size="11">${s.pinj} bar(g)</text>`;
  if (st.qs > 0) g += `<text x="700" y="${y + 70}" text-anchor="middle" fill="var(--bad)" font-size="13">flow through pump: ${st.qs.toFixed(1)} L/h</text>`;
  $("skid").innerHTML = g;
}
function valveSym(x, y, open, text, vert){
  const c = open ? "var(--ok)" : "var(--bad)";
  const d = vert ? `M${x - 10},${y - 12} L${x + 10},${y + 12} L${x - 10},${y + 12} L${x + 10},${y - 12} z` : `M${x - 12},${y - 10} L${x + 12},${y + 10} L${x + 12},${y - 10} L${x - 12},${y + 10} z`;
  return `<path d="${d}" fill="${c}" stroke="var(--ink)" stroke-width="1.5"/>` + (text ? `<text x="${x}" y="${y + 30}" text-anchor="middle" fill="var(--ink)" font-size="11">${text}</text>` : "");
}
function drawTrend(){
  const c = $("trend"), x = c.getContext("2d"), W = c.width, H = c.height, s = sp();
  x.clearRect(0, 0, W, H);
  const cs = getComputedStyle(document.documentElement);
  const dmax = s.target * 2.5, pmax = s.prv * 1.2;
  x.strokeStyle = cs.getPropertyValue("--line").trim(); x.setLineDash([]); x.strokeRect(0.5, 0.5, W - 1, H - 1);
  x.strokeStyle = cs.getPropertyValue("--mute").trim(); x.setLineDash([6, 4]);
  x.beginPath(); x.moveTo(0, H - H * s.target / dmax); x.lineTo(W, H - H * s.target / dmax); x.stroke(); x.setLineDash([]);
  [[0, "--chem", dmax], [1, "--warn", pmax]].forEach(([k, v, mx]) => {
    x.strokeStyle = cs.getPropertyValue(v).trim(); x.lineWidth = 2; x.beginPath();
    st.hist.forEach((h, i) => { const px = W - (st.hist.length - 1 - i) * W / 180, py = H - H * Math.min(1, h[k] / mx); i ? x.lineTo(px, py) : x.moveTo(px, py); });
    x.stroke();
  });
}
function readouts(){
  const s = sp(), f = st.fault;
  $("qRo").textContent = st.q.toFixed(s.qmax < 5 ? 2 : 1);
  $("pRo").textContent = st.p.toFixed(1);
  $("pBand").textContent = st.amp > 0 ? `bar(g), swing ±${st.amp.toFixed(1)}` : "bar(g)";
  $("doseRo").textContent = st.analyser.toFixed(s.target < 5 ? 2 : 1);
  $("gbRo").textContent = st.gbT.toFixed(0);
  $("tankRo").textContent = st.tankL.toFixed(0);
  $("colRo").textContent = st.colmL.toFixed(0);
  $("flowRo").textContent = (s.flow * st.flowF / 100).toFixed(0);
  $("timerRo").textContent = st.timer.toFixed(1);
  $("strokeV").textContent = st.stroke; $("speedV").textContent = st.speed; $("flowV").textContent = st.flowF;
  $("speed").value = st.speed; $("speed").disabled = st.mode === "paced";
  $("valveState").textContent = `Tank valve ${st.tankValve ? "open" : "closed"}, column valve ${st.colValve ? "open" : "closed"}`;
  $("startBtn").textContent = st.running ? "Stop" : "Start";
  const o = [];
  if (st.running && (f === "gas" || f === "strainer") && st.q > 0) o.push("Bubbles in clear suction tubing");
  if (st.running && f === "strainer") o.push("Knocking noise from the pump");
  if (st.running && f === "damper") o.push("Discharge line vibrating");
  if (st.prvLift) o.push("PRV outlet flowing back to tank");
  if (st.qs > 0 && !st.running) o.push("Liquid moving in the suction line with pump stopped");
  if (f === "quill" && st.sys === "caustic") o.push("White crystals at the quill fitting");
  if (f === "gear") { o.push("Gearbox sight glass: oil below the mark"); if (st.running && st.gbT > 55) o.push("Gearbox noisy and hot"); }
  if (!o.length) o.push(st.running ? "Nothing unusual" : "Pump stopped");
  $("obs").innerHTML = o.join("<br>");
  const a = [];
  if (st.tankL < s.tank * 0.1) a.push("Day tank LOW");
  if (st.running && st.q < 0.05 * s.qmax && st.stroke > s.s0 + 5 && st.speed > 10) a.push("Pump running, no flow");
  if (st.trip) a.push("MOTOR TRIPPED ON OVERLOAD");
  if (f === "diaphragm" && st.running) a.push("DIAPHRAGM LEAK DETECTED");
  if (st.analyser > s.target * 1.3 && st.t > 30) a.push("Analyser HIGH");
  if (st.analyser < s.target * 0.7 && st.t > 60 && st.running) a.push("Analyser LOW");
  $("alarms").innerHTML = a.map(t => `<div class="alarm" role="status">${t}</div>`).join("");
}

// ---------- UI ----------
function calc(){
  const d = +$("cDose").value, fl = +$("cFlow").value, S = +$("cS").value, s = sp();
  let out = NaN;
  if (s.vol) { out = fl * d * 1e-6 * 1000; $("cNote").textContent = "Volume dosing: rate (L/h) = flow (m³/h) × ppm v/v × 10⁻⁶ × 1000. Strength not used."; }
  else { out = d * fl / S; $("cNote").textContent = "Rate (L/h) = dose (mg/L) × flow (m³/h) / S (g/L)."; }
  $("cOut").textContent = isFinite(out) ? out.toFixed(2) : "–";
}
function act(id){
  const a = ACTIONS.find(x => x[0] === id);
  st.actions++;
  if (id === "refill") { st.tankL = sp().tank * 0.9; log("Day tank refilled"); return; }
  if (id === "oilfix" && st.running) { log("Stop and isolate the pump before opening the gearbox", "warn"); return; }
  if (a[3] === st.fault) { log(`${a[1]}: fault cleared`, "ok"); st.fault = "none"; if (id === "oilfix") st.trip = false; }
  else log(`${a[1]}: no change found`, "warn");
}
function loadCode(){
  const raw = $("code").value.trim().toUpperCase();
  const key = Object.keys(CODES).find(k => CODES[k] === raw || (FAC && k === raw));
  if (!key) { $("scenText").textContent = "Code not recognised."; return; }
  const sc = SCEN[key]; st.scenario = key; st.off = sc.off || 0; st.actions = 0;
  loadSystem(sc.sys); st.fault = dec(sc.fault); $("log").innerHTML = "";
  if (sc.set || sc.off) {
    const s = sp(); const need = s.vol ? s.target * s.flow / 1000 : s.target * s.flow / s.S;
    const frac = need / (s.qmax * (1 - 0.0025 * Math.max(s.pinj + 0.2, s.bpv)));
    st.speed = 100; st.stroke = Math.round(s.s0 + frac * (100 - s.s0)); st.running = true;
    $("stroke").value = st.stroke;
    for (let i = 0; i < 1200; i++) step(0.1); // settle two minutes so trends show the fault
  }
  $("scenText").innerHTML = `<b>${key}</b> · ${SYS[sc.sys].name}. ` + (sc.text || "Fault drill: the plant reports a dosing problem on this system. Find the fault with the readings, drawdown and field checks, act, then submit your diagnosis.");
  log(`Scenario ${key} loaded`);
}
function buildFac(){
  if (!FAC) { $("facTab").remove(); $("facPanel").remove(); $("edition").textContent = "Learner edition"; return; }
  $("edition").textContent = "Facilitator edition";
  let h = `<label for="fsel">Inject fault (hidden from learners' view)</label><select id="fsel">` +
    Object.entries(FAULTS).map(([k, v]) => `<option value="${k}">${v.t}</option>`).join("") + `</select>`;
  h += `<p style="font-size:12px">Current: <b id="fcur"></b></p><table><tr><th>Card</th><th>Code</th><th>System</th><th>Fault</th></tr>` +
    Object.keys(SCEN).map(k => `<tr><td>${k}</td><td>${CODES[k]}</td><td>${SYS[SCEN[k].sys].name}</td><td>${FAULTS[SCEN[k].fault].t}${SCEN[k].off ? " (12 % low)" : ""}</td></tr>`).join("") + `</table>`;
  $("facPanel").innerHTML = h;
  $("fsel").onchange = e => { st.fault = e.target.value; log(`Facilitator set fault: ${FAULTS[st.fault].t}`); };
}
function init(){
  $("sys").innerHTML = Object.entries(SYS).map(([k, v]) => `<option value="${k}">${v.name}</option>`).join("");
  $("diag").innerHTML = Object.entries(FAULTS).map(([k, v]) => `<option value="${k}">${v.t}</option>`).join("");
  $("actions").innerHTML = ACTIONS.map(a => `<button data-act="${a[0]}" data-tip="${a[2]}">${a[1]}</button>`).join("");
  $("actions").onclick = e => { const b = e.target.closest("button"); if (b) act(b.dataset.act); };
  $("sys").onchange = e => { st.scenario = null; st.off = 0; loadSystem(e.target.value); log(`System: ${sp().name}`); };
  $("startBtn").onclick = () => {
    if (!st.running && st.trip) { log("Motor tripped: find and fix the cause before restarting", "warn"); return; }
    st.running = !st.running; log(st.running ? "Pump started" : "Pump stopped"); };
  $("mode").onchange = e => { st.mode = e.target.value; log(`Mode: ${st.mode}`); };
  $("stroke").oninput = e => st.stroke = +e.target.value;
  $("speed").oninput = e => st.speed = +e.target.value;
  $("flowF").oninput = e => st.flowF = +e.target.value;
  $("bpv").onchange = e => { st.bpv = Math.max(0, +e.target.value); log(`BPV set to ${st.bpv} bar`); };
  $("fillBtn").onclick = () => { st.tankValve = true; st.colValve = true; log("Column filling"); };
  $("drawBtn").onclick = () => { if (!st.colValve) { log("Open the column valve first (step 1)", "warn"); return; } st.tankValve = false; log("Tank valve closed: drawing from column"); };
  $("restoreBtn").onclick = () => { st.tankValve = true; st.colValve = false; log("Tank valve open, column valve closed"); };
  $("timerBtn").onclick = () => {
    if (!st.timing) { st.timer = 0; st.timing = true; st.marks = [st.colmL]; $("timerMarks").textContent = `start ${st.colmL.toFixed(0)} mL`; }
    else { st.timing = false; st.marks.push(st.colmL); $("timerMarks").textContent = `start ${st.marks[0].toFixed(0)} mL, stop ${st.colmL.toFixed(0)} mL`; log(`Drawdown: ${(st.marks[0] - st.colmL).toFixed(0)} mL in ${st.timer.toFixed(1)} s`); }
  };
  $("diagBtn").onclick = () => {
    const d = $("diag").value, real = st.scenario ? dec(SCEN[st.scenario].fault) : st.fault;
    const okd = d === real; log(`Diagnosis: ${FAULTS[d].t}: ${okd ? "correct" : "not correct"} (${st.actions} field actions used)`, okd ? "ok" : "bad");
  };
  ["cDose", "cFlow", "cS"].forEach(i => $(i).oninput = calc);
  $("code").onkeydown = e => { if (e.key === "Enter") loadCode(); };
  $("codeBtn").onclick = loadCode;
  document.querySelectorAll("[role=tab]").forEach(b => b.onclick = () => {
    document.querySelectorAll("[role=tab]").forEach(x => x.setAttribute("aria-selected", x === b));
    document.querySelectorAll("[data-panel]").forEach(p => p.classList.toggle("hide", p.dataset.panel !== b.dataset.tab));
  });
  $("themeBtn").onclick = () => { const r = document.documentElement; const cur = r.dataset.theme || (matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light"); r.dataset.theme = cur === "dark" ? "light" : "dark"; };
  buildFac(); loadSystem("eff"); $("stroke").value = st.stroke;
  setInterval(() => { step(0.1); readouts(); drawSkid(); if (FAC) $("fcur").textContent = FAULTS[st.fault].t; }, 100);
  setInterval(drawTrend, 1000);
  readouts(); drawSkid(); drawTrend();
}
// test hook
window.DosingSim = { st, SYS, step, loadSystem, act, loadCode, calc };
init();

// ---------- Mechanical engineering suite adapter (appended by build_suite.py) ----------
const suiteFlags = { prove: false, siphon: false, paced: false, diag: false, fault: false };
const TASKS = [
  { id: 'prove', t: 'Prove a dose', d: 'Choose a system and use the dose calculator for the pump rate. Set stroke and speed, then run a drawdown of at least 60 s. The measured rate must be within 3 % of the calculated rate.', ok: () => suiteFlags.prove },
  { id: 'siphon', t: 'See a siphon', d: 'On Wastewater: ferric chloride, stop the pump and set the BPV to 0 bar. Watch liquid move through the stopped pump, then raise the BPV until the flow stops.', ok: () => suiteFlags.siphon },
  { id: 'paced', t: 'Flow-paced dosing', d: 'Run the pump in Flow-paced mode and change the stream flow. Watch the speed follow the flow and the analyser hold closer to target than in Manual.', ok: () => suiteFlags.paced },
  { id: 'fault', t: 'Find and clear a fault', d: 'Load a Mystery fault from the Launch Centre, or a scenario card code. Confirm the symptom, walk the skid in flow order, clear the fault and submit the correct diagnosis.', ok: () => suiteFlags.diag && st.fault === 'none' },
];
const suiteDone = () => TASKS.filter(t => t.ok()).map(t => t.id);
function suiteTasks() {
  const done = suiteDone();
  $("suiteTasks").innerHTML = TASKS.map(t => `<li><b>${done.includes(t.id) ? "✓ " : ""}${t.t}</b><br><span>${t.d}</span></li>`).join("");
}
function suiteTab(b) {
  document.querySelectorAll("[role=tab]").forEach(x => x.setAttribute("aria-selected", x === b));
  document.querySelectorAll("[data-panel]").forEach(p => p.classList.toggle("hide", p.dataset.panel !== b.dataset.tab));
}
(function suiteInit() {
  const tab = document.createElement("button");
  tab.setAttribute("role", "tab"); tab.setAttribute("aria-selected", "false"); tab.dataset.tab = "tasks"; tab.textContent = "Learning tasks";
  tab.onclick = () => suiteTab(tab);
  document.querySelector(".tabs").appendChild(tab);
  const panel = document.createElement("div");
  panel.dataset.panel = "tasks"; panel.className = "hide";
  panel.innerHTML = `<ol id="suiteTasks" style="margin:0;padding-left:18px;font-size:13px;display:grid;gap:6px"></ol>`;
  [...document.querySelectorAll("[data-panel]")].pop().after(panel);
  $("timerBtn").addEventListener("click", () => {
    if (st.timing || st.marks.length !== 2 || st.timer < 60 || st.tankValve) return;
    const s = sp(), flow = s.flow * st.flowF / 100, need = s.vol ? s.target * flow / 1000 : s.target * flow / s.S;
    const rate = (st.marks[0] - st.marks[1]) / st.timer * 3.6;
    if (Math.abs(rate / need - 1) <= 0.03) suiteFlags.prove = true;
  });
  $("diagBtn").addEventListener("click", () => {
    if (!st.scenario || !st.scenario.startsWith("D3-")) return;
    if ($("diag").value === dec(SCEN[st.scenario].fault)) suiteFlags.diag = true;
  });
  $("btnHelp").onclick = () => $("help").classList.remove("hidden");
  $("btnHelpClose").onclick = () => $("help").classList.add("hidden");
  setInterval(() => {
    if (st.qs > 0 && !st.running) suiteFlags.siphon = true;
    if (st.mode === "paced" && st.running && st.flowF !== 100) suiteFlags.paced = true;
    suiteTasks();
  }, 250);
  suiteTasks();
})();

function suiteSystem(id) {
  st.scenario = null; st.off = 0; st.running = false; loadSystem(id);
  log(`System: ${sp().name}`);
}
function suiteMystery() {
  const keys = Object.keys(CODES).filter(k => k.startsWith("D3-"));
  $("code").value = CODES[keys[Math.floor(Math.random() * keys.length)]];
  loadCode(); $("code").value = "";
}
window.DosingPump = {
  getState: () => ({
    inputs: { sys: st.sys, mode: st.mode, stroke: st.stroke, speed: st.speed, flowF: st.flowF, bpv: st.bpv, running: st.running },
    results: { status: st.trip ? "Motor tripped" : st.running ? "Running" : "Stopped", pumpLh: st.q, dischBar: st.p, doseOut: st.analyser, tankL: st.tankL, colmL: st.colmL, gearboxC: st.gbT, prvLift: st.prvLift ? "Lifting" : "Closed" },
    tasksDone: suiteDone(),
  }),
  setState: inp => {
    if (inp.preset === "mystery") suiteMystery();
    else if (inp.preset && SYS[inp.preset]) suiteSystem(inp.preset);
    if (inp.sys && SYS[inp.sys]) suiteSystem(inp.sys);
    if (inp.mode === "manual" || inp.mode === "paced") { st.mode = inp.mode; $("mode").value = inp.mode; }
    if (Number.isFinite(inp.stroke)) { st.stroke = Math.max(0, Math.min(100, inp.stroke)); $("stroke").value = st.stroke; }
    if (Number.isFinite(inp.speed)) st.speed = Math.max(0, Math.min(100, inp.speed));
    if (Number.isFinite(inp.flowF)) { st.flowF = Math.max(50, Math.min(150, inp.flowF)); $("flowF").value = st.flowF; }
    if (Number.isFinite(inp.bpv)) { st.bpv = Math.max(0, inp.bpv); $("bpv").value = st.bpv; }
    if (typeof inp.running === "boolean" && !(inp.running && st.trip)) st.running = inp.running;
    if (Number.isFinite(inp.advance)) for (let i = 0; i < Math.min(inp.advance, 3600) * 10; i++) step(0.1);
    return window.DosingPump.getState();
  },
};
window.addEventListener("message", e => {
  const d = e.data || {};
  if (d.type === "sim:set") window.DosingPump.setState(d.inputs || {});
  if (d.type === "sim:get") (e.source || window).postMessage({ type: "sim:state", id: "dosing-pump", state: window.DosingPump.getState() }, "*");
});
