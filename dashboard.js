/* Simulator Launch Centre — lists simulators from simulators/manifest.js, embeds them,
   reads live state through postMessage (sim:get → sim:state), sends presets (sim:set),
   and tracks learning-task progress per device. Offline, no server. */
'use strict';
const $ = s => document.querySelector(s);
const SIMS = (window.SIM_MANIFEST && window.SIM_MANIFEST.simulators) || [];
const LABEL = { Q: ['Flow', 'm³/h', 1], H: ['Head', 'm', 1], eta: ['Efficiency', '', 2], Pelec: ['Electrical power', 'W', 0], P1: ['Suction P', 'Pa', 0], P2: ['Discharge P', 'Pa', 0], margin: ['NPSH margin', 'm', 2], rpm: ['Speed', 'rpm', 0], dpTaps: ['ΔP element', 'Pa', 0], dpPerm: ['Total loss', 'Pa', 0], K: ['K', '', 2], Qmeter: ['Meter Q', 'm³/h', 2], err: ['Meter error', '%', 1], npsha: ['NPSHa', 'm', 2], npshr: ['NPSHr', 'm', 2], drop: ['Head drop', '%', 1], vib: ['Vibration', 'mm/s', 1], Tb: ['Bearing temp', '°C', 1], Tf: ['Fluid temp', '°C', 1], dB: ['Sound', 'dB', 0], overall: ['Overall', 'mm/s', 2], zone: ['ISO zone', '', null], peakG: ['Peak', 'g', 2], crest: ['Crest factor', '', 1], P: ['Pressure', 'bar', 2], Ppeak: ['Peak', 'bar', 2], Qrel: ['Relief flow', 'L/min', 2], ripple: ['Ripple', '', 2], torque: ['Torque', '', 2], centP: ['Centrifugal P', 'bar', 2], centQ: ['Centrifugal Q', 'L/min', 1], stage: ['Stage', '', null] };
const PRESETS = {
  'hydraulic-loop': [
    ['Normal duty', 'Speed 100 %, valve 50 %, DN40', { speed: 100, valve: 50, pipe: 40, length: 8, temp: 20, strainer: 0, level: 0.8 }],
    ['Throttled', 'Same speed, valve 20 % — operating point climbs the curve', { speed: 100, valve: 20 }],
    ['Speed-controlled', 'Valve open, speed 55 % — same flow for less power', { speed: 55, valve: 100 }],
    ['Small pipe', 'DN25 fitted, everything open', { speed: 100, valve: 100, pipe: 25 }],
    ['Cavitation', 'Hot fluid, fouled strainer', { speed: 100, valve: 80, temp: 80, strainer: 85, level: 0.3 }],
    ['Dead-head', 'Valve shut with the pump running', { speed: 100, valve: 0 }],
  ],
  'restriction-board': [
    ['Venturi — Bernoulli', 'β 0.6 at 70 % speed', { element: 'venturi', options: { beta: 0.6 }, speed: 70, throttle: 100 }],
    ['Orifice — calibrate', 'β 0.6, check meter error', { element: 'orifice', options: { beta: 0.6 }, speed: 70, throttle: 100 }],
    ['Sudden expansion', 'DN40 → DN65, pressure recovery', { element: 'expansion', options: { d2: 65 }, speed: 70, throttle: 100 }],
    ['Four mitre elbows', 'Minor losses add up', { element: 'elbows', options: { count: 4, type: 'mitre' }, speed: 70 }],
    ['Rusty pipe', 'Straight pipe, rough bore', { element: 'pipe', options: { surface: 'rusty' }, speed: 100 }],
    ['Throat cavitation', 'Venturi β 0.5, 85 °C, full speed', { element: 'venturi', options: { beta: 0.5 }, speed: 100, throttle: 100, temp: 85 }],
  ],
  'pump-trainer': [
    ['Single, test-ready', 'P1 100 %, throttle 60 %', { arr: 'single', n1: 100, n2: 0, dthr: 60, sthr: 100, level: 1.0, strainer: 0 }],
    ['Parallel, both 100 %', 'Flow adds at equal head', { arr: 'parallel', n1: 100, n2: 100, dthr: 60 }],
    ['Series, both 100 %', 'Head adds at equal flow', { arr: 'series', n1: 100, n2: 100, dthr: 60 }],
    ['Mismatched parallel', 'P2 at 70 % against P1 at 100 %, throttle 30 %', { arr: 'parallel', n1: 100, n2: 70, dthr: 30 }],
    ['Suction starved', 'Strainer 85 %, tank 1.5 m below the eye', { arr: 'single', n1: 100, dthr: 80, strainer: 85, level: -1.5 }],
  ],
  'pd-pump-skid': [
    ['Diaphragm, open discharge', 'Relief 6 bar, BPV 2 bar, dampener charged', { type: 'diaphragm', speed: 100, dthr: 100, relief: 6, reliefIso: false, bpv: 2, charge: 1.6, dampIso: false, repair: true }],
    ['Discharge shut, relief in service', 'All flow returns through the relief valve', { type: 'diaphragm', speed: 100, dthr: 0, reliefIso: false, repair: true }],
    ['Relief isolated', 'Close the discharge and watch the hose', { type: 'diaphragm', speed: 100, dthr: 100, reliefIso: true }],
    ['Dampener isolated', 'Full pulsation on the gauge', { type: 'diaphragm', speed: 100, dthr: 60, dampIso: true, repair: true }],
    ['Gear pump', 'Low pulsation, more slip at pressure', { type: 'gear', speed: 100, dthr: 60, dampIso: false, repair: true }],
  ],
  'cavitation-trainer': [
    ['Healthy', 'Open suction, cool fluid, level above the eye', { throttle: 100, temp: 20, lift: 0.5, speed: 100, dvalve: 70 }],
    ['Incipient', 'Throttle 35 % — audible before the head moves', { throttle: 35, temp: 20, lift: 0.5 }],
    ['Developed', 'Throttle 20 % — head down about 3 %', { throttle: 20, temp: 20, lift: 0.5 }],
    ['Hot fluid only', 'Throttle open, 85 °C', { throttle: 100, temp: 85, lift: 0.5 }],
    ['Suction lift', 'Tank 3 m below the eye', { throttle: 100, temp: 20, lift: -3 }],
  ],
  'vibration-fault-simulator': [
    ['Healthy machine', '1 480 rpm, no faults', { rpm: 1480, point: 'B1', dir: 'H', faults: { ub: { mass: 0, tmass: 0 }, ma: { ang: 0, off: 0 }, loose: { bolt: 100, fit: 0 }, bent: 0, brg: { type: 'none' }, gear: { type: 'none', sev: 0 }, belt: { type: 'none', sev: 0 }, res: { fn: 2400, zeta: 0.05 } } }],
    ['Unbalance 30 g', 'Watch 1× appear at bearing 1', { rpm: 1480, point: 'B1', dir: 'H', faults: { ub: { mass: 30, ang: 90, tmass: 0 } } }],
    ['Angular misalignment', 'Axial 1× and 2×, 180° across the coupling', { rpm: 1480, point: 'B1', dir: 'A', faults: { ma: { ang: 6, off: 0 } } }],
    ['Looseness at bearing 2', 'Harmonic comb, vertical', { rpm: 1480, point: 'B2', dir: 'V', faults: { loose: { bolt: 20 } } }],
    ['Outer-race defect, stage 2', 'Look at the envelope spectrum', { rpm: 1480, point: 'B1', dir: 'H', faults: { brg: { pos: 'B1', type: 'bpfo', sev: 2 } } }],
    ['Cracked pinion tooth', 'Gearbox fitted, GMF with sidebands', { rpm: 1480, fitGear: true, point: 'GB', dir: 'H', faults: { gear: { type: 'crack', sev: 70 } } }],
    ['Blind machine', 'Hidden fault to diagnose', { blind: true }],
  ],
  'compressor-map': [
    ['Design point', 'Start and load to the process; ASC auto, 2 s valve', { preset: 'design' }],
    ['Surge sequence (slow valve)', 'Process upset with a 15 s recycle valve: normal → approaching → surge → recovery', { preset: 'sequence' }],
    ['Same upset, fast valve', 'Stroke 2 s: the ASC holds it on the control line', { stroke: 2, asc: 'auto', preset: 'closeProc' }],
    ['Header pressure creeps up', 'Discharge header to 13.5 bar a over 40 s', { preset: 'header' }],
    ['Speed cut to 85 %', 'Cannot meet the header; ends in total recycle', { preset: 'speed' }],
    ['Suction pressure falls', 'Nothing downstream changes, PR still rises', { preset: 'lowPs' }],
    ['ASC off — recover by hand', 'Manual, valve shut, process closing; open the recycle yourself', { preset: 'manual' }],
  ],
  'operating-point-board': [
    ['Card 0 — commission', 'Baseline pieces placed; mark the BEP and the operating point', { scenario: '0', baseline: true }],
    ['Card A — strainer fouls 40 %', 'Swap S2 for S3', { scenario: 'A' }],
    ['Card B — tank level falls 5 m', 'Slide S2 up to 15 m', { scenario: 'B' }],
    ['Card C — throttle to 18 m³/h', 'Find the system piece that crosses at 18', { scenario: 'C' }],
    ['Card D — VFD to 1 200 rpm', 'P3 with its own bar and BEP', { scenario: 'D' }],
    ['Card E — second pump in parallel', 'P8 and W8', { scenario: 'E' }],
    ['Card F — trim for 18 m³/h', 'P7 and W7', { scenario: 'F' }],
    ['Projector mode', 'Hide the digital pieces; reveal answers for the magnetic board', { projector: true }],
  ],
  'hydraulic-power-demonstrator': [
    ['Small bore, fast', 'Ø25/12, 200 kg, 150 bar', { preset: 'small' }],
    ['Big bore, strong', 'Ø80/45, 3 000 kg, 150 bar', { preset: 'big' }],
    ['Stall test', 'Brake at 100 kN: force = relief × area', { preset: 'stall' }],
    ['Lift 2 000 kg', 'Find the smallest bore at ≤ 150 bar', { preset: 'lift' }],
    ['Meter-out closed', 'Rod-side intensification on Ø50/28', { preset: 'intens' }],
    ['Wasteful meter-in', '30 % opening at 200 bar: watch the tank', { preset: 'waste' }],
    ['Overrunning load', '2 500 kg lowered with the meter-out open', { preset: 'overrun' }],
  ],
  'derrick-crane': [
    ['Standard rig', '24 m mast, 30 m boom, 18 m sills, 45° spread, 10 t', { preset: 'std' }],
    ['Reach out to the limit', '12 t, boom luffing out until something governs', { preset: 'reach' }],
    ['Tall mast, same lift', '36 m mast: topping lift eases', { preset: 'tall' }],
    ['Narrow legs, slewed', '30° spread, slewed 120°: one leg loaded', { preset: 'narrow' }],
    ['Light anchors', '80 kN hold-down, 15 t: watch the uplift', { preset: 'light' }],
    ['Single-part hoist', 'Rope and winch limits with 8 t', { preset: 'single' }],
    ['Long boom', '44 m boom at 40°: strut capacity falls with length²', { preset: 'long' }],
  ],
  'beam-loads': [
    ['Central point load', '20 kN at mid-span of 6 m: PL/4', { preset: 'pl' }],
    ['Uniform load', '10 kN/m over 6 m: wL²/8', { preset: 'udl' }],
    ['Four-point bending', 'Constant moment between the loads', { preset: 'four' }],
    ['Cantilever tip load', 'M = PL, δ = PL³/3EI', { preset: 'cant' }],
    ['Overhangs', 'Supports at 1.2 and 4.8 m under a uniform load', { preset: 'over' }],
    ['Propped cantilever', 'Contraflexure and the prop reaction', { preset: 'prop' }],
    ['Fixed both ends', 'wL²/12 at the supports', { preset: 'ff' }],
    ['Load on the overhang', 'Support A lifts off', { preset: 'uplift' }],
  ],
  'mass-spring-damper': [
    ['Free decay', '2 kg, 800 N/m, ζ 0.05, released from 80 mm', { preset: 'decay' }],
    ['Critical damping', 'ζ = 1: no overshoot', { preset: 'critical' }],
    ['Overdamped', 'ζ = 2: creeps back', { preset: 'over' }],
    ['Resonance', 'Harmonic force at f = fn with ζ 0.05', { preset: 'res' }],
    ['Sweep the FRF', 'Swept sine 0.2 → 3 fn, measured points over theory', { preset: 'sweep' }],
    ['Isolation r = 3', 'Base excitation above √2', { preset: 'iso' }],
    ['Car on a wavy road', '20 kg on 800 N/m, ζ 0.3, road at 1.5 Hz', { preset: 'car' }],
    ['Hammer tap', 'Impulse response', { preset: 'hammer' }],
  ],
  'planetary-gears': [
    ['Reducer', 'Ring held, sun in: 1 + Nr/Ns', { preset: 'reducer' }],
    ['Star', 'Carrier held: reversal at −Nr/Ns', { preset: 'star' }],
    ['Overdrive', 'Sun held, ring in: 1 + Ns/Nr', { preset: 'over' }],
    ['Differential', 'Sun and ring driven, carrier averages', { preset: 'diff' }],
    ['Stop the carrier', 'Opposing inputs: ωs·Ns = −ωr·Nr', { preset: 'stall' }],
    ['Locked 1 : 1', 'Sun locked to ring', { preset: 'lock' }],
    ['High ratio 10 : 1', '12 / 48 / 108 with four planets', { preset: 'big' }],
    ['Won\'t assemble', '22 / 72 with four planets', { preset: 'noasm' }],
  ],
  'bearing-lube-rig': [
    ['Journal design point', 'Ø60, 5 kN, 1 500 rpm, VG 46', { preset: 'design' }],
    ['Slow and heavy', '30 rpm, 40 kN, VG 22: boundary lubrication', { preset: 'slow' }],
    ['Thin oil, hot supply', 'VG 22 at 70 °C', { preset: 'thin' }],
    ['Oil supply lost', 'Watch the film temperature run away', { preset: 'starve' }],
    ['Tight clearance at 4 000 rpm', 'Heat without load capacity', { preset: 'tight' }],
    ['Rolling bearing 6208', '6.5 kN, 1 500 rpm', { preset: 'roll' }],
    ['Axial load on a ball bearing', 'Fr 3 kN, Fa 3 kN', { preset: 'axial' }],
    ['Contaminated grease', 'Heavy contamination, modified life collapses', { preset: 'dirty' }],
  ],
  'heat-exchanger': [
    ['Oil cooler, counterflow', 'Oil 2 kg/s at 90 °C against water 3 kg/s at 25 °C', { preset: 'counter' }],
    ['Same cooler, parallel flow', 'Less duty, no temperature cross', { preset: 'parallel' }],
    ['Balanced flows', 'Cr = 1: parallel temperature lines', { preset: 'balanced' }],
    ['Cold flow starved', 'Pinch at the cold outlet', { preset: 'pinch' }],
    ['Fouling over 60 days', 'Duty falls; clean to recover', { preset: 'foul' }],
    ['Plate exchanger', 'Water-water, U 3 000, 4 m²', { preset: 'plate' }],
    ['Shell-and-tube 1-2', 'F below 1', { preset: 'shell' }],
    ['Cold side to boiling', 'Oil at 180 °C, 0.3 kg/s water', { preset: 'boil' }],
  ],
  'belt-drive': [
    ['Design point', 'SPA × 2, 125/250, 1 450 rpm, 60 N·m', { preset: 'design' }],
    ['Load until it slips', '260 N·m on the same drive', { preset: 'overload' }],
    ['Belt too loose', '45 % installation tension', { preset: 'loose' }],
    ['Belt too tight', '190 %: shaft load doubles', { preset: 'tight' }],
    ['Big ratio, small wrap', '90/500 on a 450 mm centre distance', { preset: 'bigratio' }],
    ['Flat belt', 'Same pulleys and tension, less capacity', { preset: 'flat' }],
    ['Timing belt jump', '40 % tension under 120 N·m', { preset: 'timing' }],
    ['Chain, 11-tooth sprocket', 'Chordal action 4 %', { preset: 'chain' }],
  ],
  'brake-clutch': [
    ['Single stop', 'Disc brake from 1 500 rpm, 20 kg·m²', { preset: 'single' }],
    ['Repeated stops → fade', 'Auto-repeat every 12 s at ×4', { preset: 'repeat' }],
    ['Small pad, high pressure', 'Lining pressure above allowable', { preset: 'overp' }],
    ['Drum brake', 'Same pressure, 25 mm bore, brake factor 1.7', { preset: 'drum' }],
    ['Drum brake fade', 'Self-energising makes fade worse', { preset: 'drumfade' }],
    ['Clutch engagement', 'M12-sized clutch, 2 000 rpm, 60 N·m load', { preset: 'clutch' }],
    ['Heavy load, slow release', '4 kg·m² on a 1.5 s release', { preset: 'clutchheavy' }],
    ['Overloaded clutch', 'Slips for ever and burns', { preset: 'clutchover' }],
    ['Dump the clutch', 'Engine stalls', { preset: 'stall' }],
  ],
  'fatigue-notch': [
    ['Sharp fillet', '1045 shaft, r 0.5 mm, 220 N·m: finite life', { preset: 'base' }],
    ['Fillet radius 4 mm', 'Same load: infinite life', { preset: 'bigr' }],
    ['Ground finish', 'ka rises, life doubles', { preset: 'polish' }],
    ['Add mean stress', '150 N·m mean: Goodman point moves right', { preset: 'mean' }],
    ['Same shaft in 4340', 'Higher Kf, higher Se', { preset: 'strong' }],
    ['Infinite-life design', 'r 3 mm, ground, 85 % of Se', { preset: 'infinite' }],
    ['Aluminium', 'No endurance limit, 3×10⁷ cycles', { preset: 'alu' }],
    ['Plate with a hole', 'Kt 2.5 on the net section', { preset: 'hole' }],
  ],
  'bolted-joint': [
    ['M12 8.8, 90 N·m', 'Zinc plated, C 0.185, 20 kN load', { preset: 'base' }],
    ['20 bolts, dry threads', 'Preload scatter ±35 %', { preset: 'scatter' }],
    ['Same torque, MoS₂', 'Preload past proof: bolt yields', { preset: 'lube' }],
    ['Rubber gasket', 'C 0.97: the bolt takes the load', { preset: 'gasket' }],
    ['Painted surfaces', 'Embedding loses 15 % of preload', { preset: 'embed' }],
    ['Overload → separation', '50 kN against a 39 kN separation load', { preset: 'separate' }],
    ['Cyclic load, soft joint', 'Bolt fatigue factor below 1', { preset: 'fatigue' }],
    ['20 bolts by stretch', 'Scatter ±3 %', { preset: 'stretch' }],
  ],
  'turbine-cycles': [
    ['Rankine baseline', '8 MPa / 500 °C / 10 kPa', { preset: 'base' }],
    ['Boiler to 16 MPa', 'Efficiency up, exhaust wetter', { preset: 'hp' }],
    ['Condenser to 5 kPa', 'Better vacuum', { preset: 'vac' }],
    ['16 MPa with reheat', 'Dries the exhaust', { preset: 'reheat' }],
    ['Gas turbine rp 10', '1 200 °C turbine inlet', { preset: 'gt' }],
    ['rp 6 with regenerator', 'Exhaust heat recovered', { preset: 'regen' }],
    ['Combined cycle', 'rp 16, 1 400 °C, 10 MPa steam', { preset: 'cc' }],
  ],
  'control-valve-pid': [
    ['Manual step test', 'OP 60 → 70 %, identify the process', { preset: 'steptest' }],
    ['P-only offset', 'Kc 2, no integral', { preset: 'ponly' }],
    ['PI, well tuned', 'Kc 1.2, Ti 4 s', { preset: 'pi' }],
    ['Gain too high', 'Kc 5: sustained oscillation', { preset: 'highgain' }],
    ['Windup', 'Unreachable setpoint, anti-windup off', { preset: 'windup' }],
    ['Stiction 3 %', 'Limit cycle', { preset: 'stiction' }],
    ['Linear valve, low authority', 'Installed characteristic bends', { preset: 'lowauth' }],
    ['Temperature loop', '5 s dead time', { preset: 'temp' }],
    ['Level loop', 'Integrating process', { preset: 'level' }],
  ],
  'refrigeration-cycle': [
    ['−10 / 40 °C cycle', '5 K superheat, 3 K subcooling', { preset: 'base' }],
    ['Condensing 55 °C', 'COP falls', { preset: 'hot' }],
    ['Evaporating −30 °C', 'Capacity collapses', { preset: 'deep' }],
    ['No superheat', 'Liquid slugging', { preset: 'slug' }],
    ['Pressure ratio too high', 'Discharge above 120 °C', { preset: 'overheat' }],
    ['Low charge 60 %', 'High superheat, low capacity', { preset: 'lowcharge' }],
    ['Cold room pull-down', 'Room from 30 to −18 °C on a clock', { preset: 'room' }],
    ['Hot day, fouled condenser', '42 °C ambient, +10 K', { preset: 'roomhot' }],
  ],
  'rotor-balancing': [
    ['Run-up through the critical', '40 kg disc, 1 988 rpm critical', { preset: 'sweep' }],
    ['Fast ramp, low damping', 'Resonance build-up limits the peak', { preset: 'fast' }],
    ['Damping 0.2', 'Peak falls', { preset: 'damped' }],
    ['Soft supports', 'Critical drops', { preset: 'soft' }],
    ['Balancing job at 1 200 rpm', 'Hidden unbalance, trial mass, correction', { preset: 'balance' }],
    ['Run above the critical', '4 500 rpm, self-centring', { preset: 'above' }],
  ],
  'shaft-alignment': [
    ['As-found job', '1 500 rpm, both planes out', { preset: 'found' }],
    ['Random job', 'New as-found errors', { preset: 'random' }],
    ['Hot motor', 'Align for 0.08 mm growth', { preset: 'thermal' }],
    ['Soft foot', 'Find and shim it first', { preset: 'soft' }],
    ['3 600 rpm', 'Tighter tolerance', { preset: 'fast' }],
    ['Run it as found', '2× vibration in zone C', { preset: 'run' }],
  ],
  'gearbox-teeth': [
    ['Spur m4 20:60, 30 kW', 'Baseline pair', { preset: 'base' }],
    ['Helical 20°', 'Smoother, axial thrust', { preset: 'helical' }],
    ['Module 6', 'Bending stress falls', { preset: 'coarse' }],
    ['Part load 6 kW', 'Efficiency falls', { preset: 'part' }],
    ['3 000 rpm', 'Churning dominates', { preset: 'fast' }],
    ['Overload 75 kW', 'Pitting on the clock', { preset: 'overload' }],
    ['Thin hot oil', 'Scuffing', { preset: 'thin' }],
    ['Cast iron gears', 'Low allowables', { preset: 'iron' }],
  ],
  'api650-tank': [
    ['30 m × 14.4 m, A283 C', 'Baseline tank, cone roof', { preset: 'base' }],
    ['A516 Gr 485', 'Stronger plate, annular ring', { preset: 'material' }],
    ['3 mm corrosion allowance', 'Product case only', { preset: 'ca' }],
    ['Heavy product G 1.25', 'Product governs', { preset: 'heavy' }],
    ['Hydrotest', 'Water to H on a fast clock', { preset: 'hydro' }],
    ['Tall open top 12 × 24 m', 'Girders and anchors', { preset: 'tall' }],
    ['45 m open top, 230 km/h', 'Intermediate girders', { preset: 'wind' }],
    ['6 m tank empty in wind', 'Overturns, anchors', { preset: 'anchor' }],
    ['Self-supporting cone, 30 m', 'Plate over 13 mm', { preset: 'selfroof' }],
    ['30 m dome', 'Feasible self-supporting roof', { preset: 'dome' }],
    ['70 m in A537', 'Beyond the one-foot method', { preset: 'big' }],
    ['10 m water tank', 'Minimum thickness governs', { preset: 'small' }],
  ],
  'pipeline-overpressure': [
    ['Normal day', 'Worker holds the set point', { preset: 'base' }],
    ['Worker fails open', 'Monitor takes over', { preset: 'wopen' }],
    ['Hidden monitor failure', 'Slam-shut trips', { preset: 'hidden' }],
    ['Relief valve only', 'Size it, count the vent', { preset: 'reliefonly' }],
    ['Shared sensing line', 'One cut, both open', { preset: 'shared' }],
    ['Old spur at 80 % SMYS', 'MAOP + 4 %', { preset: 'old' }],
  ],
  'pipeline-integrity': [
    ['Year 15: first ILI', 'Assess 22 features', { preset: 'base' }],
    ['High-resistivity ground', 'The IR-drop trap', { preset: 'rocky' }],
    ['Too much current', 'Overprotection', { preset: 'over' }],
    ['DC railway interference', 'Stray current discharge', { preset: 'stray' }],
    ['Ten years of doing nothing', 'Ruptures and leaks', { preset: 'neglect' }],
    ['Old Grade B line', 'When a patch is allowed', { preset: 'grb' }],
  ],
  'pipeline-hydrotest': [
    ['24 km, one section', 'Low point past the limit', { preset: 'base' }],
    ['Four test sections', 'Every section passes', { preset: 'split' }],
    ['Filled without a pig', 'Air in the P–V plot', { preset: 'air' }],
    ['Result to judge (A)', 'Leak or temperature?', { preset: 'leakA' }],
    ['Result to judge (B)', 'Leak or temperature?', { preset: 'leakB' }],
    ['Construction damage', 'Rupture under test', { preset: 'gouge' }],
  ],
  'gas-pipeline-route': [
    ['12 km: farms, village, town', 'Classes 1 to 3 on one route', { preset: 'base' }],
    ['All rural, Class 1 Division 1', 'F 0.80 and a 1.25 test', { preset: 'div1' }],
    ['City edge, multistorey', 'Class 4, F 0.40', { preset: 'city' }],
    ['Built, then 15 years of growth', 'Table 854.1(c)', { preset: 'encroach' }],
    ['New school beside the line', '855.1: 50 % SMYS', { preset: 'school' }],
    ['DN 150 at 20 bar', 'Handling D/t governs', { preset: 'small' }],
  ],
  'api620-tank': [
    ['30 m ammonia, 10 kPa', 'Baseline dome-roof tank', { preset: 'base' }],
    ['Pentane at 50 kPa', 'Gas pressure dominates', { preset: 'pressure' }],
    ['Torispherical roof', 'Knuckle in compression', { preset: 'tori' }],
    ['15° cone roof', 'Large hoop force', { preset: 'cone' }],
    ['35 kPa, too few bolts', 'Uplift and anchorage', { preset: 'uplift' }],
    ['6 kPa design vacuum', 'Buckling governs', { preset: 'vacuum' }],
    ['40 m LNG, 9 % Ni', 'Annex Q material', { preset: 'lng' }],
    ['110 kPa', 'Past 15 psig', { preset: 'asme' }],
    ['Hydro-pneumatic test', 'Water then 1.25 × Pg', { preset: 'test' }],
    ['Blocked vent, pump out', 'Vacuum collapse', { preset: 'blockdrain' }],
    ['Blocked vent, fill', 'Overpressure', { preset: 'blockfill' }],
    ['12 m naphtha tank', 'Minimum plate', { preset: 'small' }],
  ],
};
const TASK_NAMES = {
  'hydraulic-loop': { start: 'Start the loop', bep: 'Find the BEP', throttle: 'Same flow, two ways', pipe: 'Swap the test section', cav: 'Make it cavitate', dead: 'Dead-head the pump', runout: 'Run off the end of the curve' },
  'restriction-board': { bern: 'Bernoulli at the throat', recov: 'Pressure recovery', fric: 'Friction ∝ Q²', minor: 'Minor losses add up', cal: 'Calibrate the orifice', sys: 'Trace an element curve', cav: 'Cavitate the throat' },
  'pump-trainer': { hq: 'H–Q curve', eta: 'Efficiency curve and BEP', sys: 'System curve', aff: 'Affinity-law curves', par: 'Parallel curve', ser: 'Series curve', op: 'Predict the operating point', mismatch: 'Mismatched pumps in parallel' },
  'pd-pump-skid': { cal: 'Calibrate with the column', close: 'Close the discharge (relief in service)', relief: 'Relief valve doing its job', iso: 'Relief isolated — rupture or stall', damp: 'Measure the dampener', bpv: 'Back-pressure valve', gear: 'Diaphragm against gear' },
  'cavitation-trainer': { hear: 'Hear it first', npsh3: 'Find NPSH3', temp: 'Temperature alone', lift: 'Suction lift', recover: 'Recover it', diag: 'Diagnose all four impellers' },
  'compressor-map': { start: 'Start and load the compressor', approach: 'Enter the control band', surge: 'Make it surge', recover: 'Recover it by hand', slow: 'Slow valve loses the race', fast: 'Fast valve catches it', ps: 'Suction pressure alone', choke: 'Find the stonewall', trip: 'Let the protection trip it' },
  'operating-point-board': { base: 'Commission the board', strainer: 'Strainer fouls 40 %', level: 'Tank level falls 5 m', throttle: 'Throttle to 18 m³/h', vfd: 'VFD to 1 200 rpm', parallel: 'Second pump in parallel', trim: 'Trim for 18 m³/h', sweep: 'All four routes compared' },
  'hydraulic-power-demonstrator': { fpa: 'Force = pressure × area', speed: 'Speed = flow ÷ area', rod: 'Rod side', stall: 'Stall it', intens: 'Intensify the rod side', waste: 'Where the power goes', overload: 'Overload the motor', size: 'Size the cylinder', overrun: 'Overrunning load' },
  'derrick-crane': { reach: 'Find the radius limit', reeve: 'Reeving halves the rope pull', topping: 'Topping lift against boom angle', mast: 'A taller mast', legs: 'Slew toward a leg', anchor: 'Lift an anchor', boomlen: 'Longer boom, same radius', design: 'Design a 10 t lift at 20 m', twoblock: 'Two-block it' },
  'beam-loads': { udl: 'wL²/8 and wL/2', pl4: 'PL/4', cant: 'Cantilever tip load', over: 'Best overhang', depth: 'Double the depth', fixed: 'Fixed ends', uplift: 'Lift a support', contra: 'Point of contraflexure', design: 'Size the beam' },
  'mass-spring-damper': { fn: 'Measure the natural frequency', mk: 'Four times the mass, half the frequency', logdec: 'Log decrement gives ζ', crit: 'Critical damping', over: 'Overdamped creep', res: 'Find resonance', sweep: 'Sweep the response', iso: 'Isolate the base', design: 'Design an isolator' },
  'planetary-gears': { reducer: 'Ring held: 1 + Nr/Ns', star: 'Carrier held: reversal', over: 'Sun held: 1 + Ns/Nr', three: 'Three ratios, one gearset', stall: 'Stop the carrier', days: 'Days per year', torque: 'Torque balance', asm: 'Make it assemble', big: 'Ten to one in one stage' },
  'bearing-lube-rig': { regimes: 'Cross the Stribeck curve', oil: 'Heavier oil, thicker film, hotter bearing', load: 'Double the load', clear: 'Clearance', wipe: 'Lose the oil supply', cube: 'The cube law', axial: 'Axial load on a ball bearing', contam: 'Clean versus dirty', speed: 'Speed and viscosity' },
  'heat-exchanger': { cvp: 'Counterflow against parallel flow', cross: 'Make a temperature cross', balanced: 'Balanced flows', pinch: 'Find the pinch', flow: 'Double the cold flow', foul: 'Foul it, then clean it', size: 'Size the oil cooler', shell: 'The F factor', bypass: 'Control with the bypass' },
  'belt-drive': { slipload: 'Load it until it slips', tension: 'Tension sets capacity', wrap: 'Wrap angle', vflat: 'V against flat', select: 'Select the belts', tight: 'Over-tensioned', timing: 'Tooth jump', chordal: 'Chordal action', hot: 'Belt overheating' },
  'brake-clutch': { firststop: 'Make a stop', dist: 'Uniform wear against uniform pressure', pressure: 'Pressure to torque', overp: 'Lining pressure limit', fade: 'Fade', drum: 'Drum self-energising', drumfade: 'Drum sensitivity', clutch: 'Engage the clutch', burn: 'Clutch that cannot lock' },
  'fatigue-notch': { fracture: 'Run one to fracture', radius: 'Fillet radius buys life', finish: 'Surface finish', mean: 'Mean stress hurts', infinite: 'Design for infinite life', sens: 'Notch sensitivity', crack: 'Watch the crack', alu: 'No endurance limit', hole: 'Hole in a plate' },
  'bolted-joint': { tighten: 'Tighten a bolt', scatter: 'Torque is not tension', lube: 'Lubrication changes everything', gasket: 'Gasket stiffness', embed: 'Embedding loss', separate: 'Separate the joint', yield: 'Yield the bolt', fatigue: 'Fatigue of the bolt', method: 'A better method' },
  'turbine-cycles': { base: 'Baseline Rankine cycle', pressure: 'Raise the boiler pressure', superheat: 'Superheat dries the exhaust', vacuum: 'Condenser vacuum', reheat: 'Reheat cures wetness', isen: 'Turbine efficiency', rpopt: 'Pressure ratio optimum', regen: 'Regenerator', combined: 'Combined cycle above 50 %' },
  'control-valve-pid': { steptest: 'Open-loop step test', offset: 'Proportional-only offset', integral: 'Integral removes the offset', osc: 'Too much gain', tuned: 'Apply a tuning', windup: 'Integral windup', stiction: 'Stiction limit cycle', char: 'Characteristic and authority', deadtime: 'Dead time' },
  'refrigeration-cycle': { base: 'Read the cycle', condens: 'Condensing temperature', evap: 'Evaporating temperature', slug: 'No superheat', hot: 'Discharge too hot', subcool: 'Subcooling adds capacity', pulldown: 'Pull the room down', cycling: 'Thermostat cycling', foul: 'Fouled condenser' },
  'rotor-balancing': { critical: 'Find the critical speed', phase: 'Phase across the critical', damping: 'Damping and the peak', trip: 'Vibration trip', orig: 'Original reading', trial: 'Trial mass', balance: 'Balance it', grade: 'Balance grade', support: 'Soft supports' },
  'shaft-alignment': { found: 'Read the as-found condition', vert: 'Shim the vertical plane', horiz: 'Move the horizontal plane', aligned: 'Aligned in both planes', sag: 'Bracket sag', thermal: 'Thermal growth', softfoot: 'Soft foot', vib: 'Run it misaligned', fast: 'Tighter at speed' },
  'pipeline-overpressure': { day: 'A normal day', order: 'Set the ladder', monitor: 'The monitor takes over', hidden: 'The hidden failure', reliefonly: 'Relief valve only', lockup: 'Night lock-up', isolated: 'Relief left isolated', shared: 'One line, two regulators', freeze: 'Cold gas', velocity: 'Size the outlet', old: 'Four per cent', reset: 'Manual reset' },
  'pipeline-integrity': { post: 'Read a test post', irtrap: 'The IR-drop trap', cover: 'Protect the whole line', over: 'Too much', stray: 'Stray current', shortlong: 'Short and long', tol: 'Tool tolerance', method: 'Original against modified', act: 'Deal with every feature', derate: 'Or reduce the pressure', neglect: 'Ten years of doing nothing', patch: 'When a patch is allowed' },
  'pipeline-hydrotest': { fill: 'Fill a section', sections: 'Plan the test sections', hold80: 'The 80 % hold (N4)', pv: 'The straight line', hold2h: 'Two hours (841.321)', air: 'Air in the line', yield: 'Find yield (N5)', leak: 'Find the leak', temp: 'Not a leak', sun: 'Sun on the pipe', gouge: 'The test did its job', maop: 'Every section passed' },
  'gas-pipeline-route': { baseline: 'Read the baseline', unit: 'Slide the unit', cluster: 'Where does the class end?', houses: 'Eleven houses', school: 'Concentration of people', road: 'Road crossing', grade: 'Buy a better steel', handling: 'Too thin to handle', div1: 'Division 1', hot: 'Hot gas', grow: 'The town grows', derate: 'Comply by derating', replace: 'Comply by replacing', pir: 'Potential impact radius' },
  'api620-tank': { baseline: 'Read the baseline', forces: 'T1 and T2 along the meridian', pressure: 'Pressure against head', knuckle: 'The knuckle in compression', ring: 'Size the compression ring', cone: 'Why not a cone?', uplift: 'Hold it down', vacuum: 'Vacuum design', cold: 'Cold service', scope: 'Fifteen psig', test: 'Hydro-pneumatic test', blocked: 'Blocked vent' },
  'api650-tank': { baseline: 'Read the baseline', select: 'The one-foot rule', govern: 'Who governs?', material: 'Buy a better plate', ca: 'Corrosion allowance', optimise: 'Squat or tall?', girder: 'Stiffen the shell', anchor: 'Blow it over', override: 'Break a course and fix it', hydro: 'Hydrotest', roof: 'Roof choice', vdp: 'Beyond the one-foot method' },
  'gearbox-teeth': { forces: 'Tooth forces', module: 'Module against bending', face: 'Face width', helix: 'Helix angle and thrust', governs: 'Which stress governs?', partload: 'Efficiency at part load', churn: 'Churning at speed', pitting: 'Overload to pitting', scuff: 'Scuffing' },
  'vibration-fault-simulator': { make1x: 'Manufacture the 1× peak', balance: 'Balance it', misalign: 'Misalignment across the coupling', loose: 'Looseness harmonics', bearing: 'Bearing defect in the envelope', gear: 'Gear-mesh sidebands', res: 'Sweep through resonance', blind: 'Diagnose three blind machines' },
};
const PLAN = [
  ['20 min', 'Hydraulic Loop Bench', 'Start-up, BEP, throttling versus speed control. Deliverable: the "same flow, two ways" power comparison.'],
  ['30 min', 'Instrumented Pump Trainer', 'Construct the H–Q and efficiency curves from the instrument board, then the system curve, affinity curves and parallel/series curves. Deliverable: the full curve set built from recorded points before the model is revealed.'],
  ['25 min', 'Restriction Board', 'Bernoulli at the venturi, pressure recovery in the expansion, friction ∝ Q², minor losses. Deliverable: element ΔP–Q curves from the auto-sweep.'],
  ['15 min', 'Restriction Board', 'Flow measurement: calibrate the orifice Cd, compare permanent loss of orifice and venturi.'],
  ['20 min', 'Positive-Displacement Pump Skid', 'Calibrate from the column, then the closure demonstration: PD against centrifugal, relief valve lifting, relief isolated. Deliverable: pressure-versus-opening points for both pumps and the ripple with and without the dampener.'],
  ['25 min', 'Cavitation Trainer', 'Reduce NPSHa three ways; hear, see and measure; run the NPSH test to NPSH3. Deliverable: the head-versus-NPSHa curve.'],
  ['15 min', 'Cavitation Trainer', 'Damaged-impeller diagnosis: four samples, four mechanisms, evidence-based reasoning.'],
  ['25 min', 'Mass-Spring-Damper Demonstrator', 'Free decay: measure fn and the log decrement, then critical and overdamped. Harmonic force: find resonance, sweep the FRF. Base excitation: isolation above √2 and why damping hurts there. Deliverable: measured FRF points against theory and an isolator design for 10 Hz.'],
  ['20 min', 'Planetary Gear Demonstrator', 'Watch the planet spin and orbit, then the three held-member ratios on one gearset, the differential, torque balance and the assembly rule. Deliverable: the three ratios recorded and checked against the Willis equation, and a 10 : 1 single stage that assembles.'],
  ['25 min', 'Bearing and Lubrication Rig', 'Journal bearing across the Stribeck curve by sweep, oil grade and clearance, starve it; rolling bearing: cube law, axial load, contamination, and the defect frequencies that feed the vibration simulator. Deliverable: two readings per regime and a life comparison.'],
  ['25 min', 'Heat Exchanger Trainer', 'Counterflow against parallel, balanced flows, pinch, fouling and cleaning, sizing the oil cooler, F factor on the 1-2 shell-and-tube. Deliverable: the sized cooler and the ε-NTU point for each arrangement.'],
  ['20 min', 'Belt and Chain Drive Bench', 'Load a V-belt drive to slip, tension it loose and tight, wrap angle, V against flat, belt selection for 10 kW, timing belt jump, chain chordal action. Deliverable: the belt selection with its slip and shaft load.'],
  ['20 min', 'Brake and Clutch Demonstrator', 'One stop and its energy, uniform wear against uniform pressure, lining pressure limit, repeated stops to fade, drum against disc brake factor, clutch engagement and an overloaded clutch. Deliverable: the stop time and temperature rise per stop, and the number of stops to fade.'],
  ['25 min', 'Fatigue and Stress Concentration Bench', 'Kt to Kf, Marin factors, a specimen run to fracture, fillet radius and finish against life, mean stress on the Goodman diagram, infinite-life design, aluminium without an endurance limit. Deliverable: the fillet radius and finish that take the baseline shaft to infinite life.'],
  ['20 min', 'Bolted Joint and Torque Bench', 'Tighten, read the joint diagram, twenty bolts of torque scatter, dry against lubricated, gasket stiffness, embedding, separation, proof-load yield and bolt fatigue. Deliverable: torque, lubrication and gasket choice that keep the joint closed with a bolt fatigue factor above 1.5.'],
  ['25 min', 'Steam and Gas Turbine Cycle Board', 'Rankine baseline against Carnot, boiler pressure, superheat, condenser vacuum, reheat, turbine efficiency; Brayton pressure ratio optimum and regenerator; combined cycle above 50 %. Deliverable: the steam and gas settings that reach 50 % combined.'],
  ['30 min', 'Control Valve and PID Loop Bench', 'Manual step test and identification, P-only offset, PI, ultimate gain, apply a tuning, windup, stiction, valve characteristic and authority, dead time. Deliverable: a tuning with less than 25 % overshoot on a 5 s dead-time loop.'],
  ['20 min', 'Refrigeration Cycle Trainer', 'Read the cycle, condensing and evaporating temperature effects, no superheat, hot discharge, subcooling, cold-room pull-down, thermostat cycling, fouled condenser. Deliverable: the COP and capacity change from a 10 K fouled condenser.'],
  ['25 min', 'Rotor Balancing and Critical Speed Rig', 'Run-up through the critical, phase across it, damping, trip, then a balancing job: original and trial readings, correction, balance grade; soft supports. Deliverable: the correction mass and angle and the residual grade.'],
  ['25 min', 'Shaft Alignment Bench', 'As-found readings, shims and moves from the foot distances, sag, thermal growth, soft foot, run it misaligned, tighter tolerance at speed. Deliverable: the shim and move at each foot for the as-found job.'],
  ['20 min', 'Gearbox Tooth Loads and Efficiency Bench', 'Tooth forces, module, face width, helix and thrust, which stress governs, part-load efficiency, churning at speed, pitting and scuffing. Deliverable: the geometry change that takes the contact safety factor above 1.2.'],
  ['30 min', 'API 650 Tank Design Demonstrator', 'Baseline plate schedule and the one-foot rule, who governs (product against hydrotest), material and corrosion allowance, squat against tall at constant capacity, wind girders, overturning and anchors, break and fix a course, hydrotest, roof choice, past 61 m. Deliverable: the plate schedule and steel mass for a 10 000 m³ tank at the least-steel diameter with a feasible roof and no anchors.'],
  ['30 min', 'API 620 Tank Design Demonstrator', 'Unit forces along the meridian, gas pressure against liquid head, the knuckle in compression, sizing the compression ring, why not a cone, uplift and anchor bolts, vacuum design, cold service materials, the 15 psig limit, the hydro-pneumatic test and a blocked vent. Deliverable: a passing 30 m ammonia tank at 10 kPa with the ring area, bolt count and plate schedule recorded, and the blocked-vent failure named.'],
  ['30 min', 'Gas Pipeline Route and Wall Thickness Designer', 'Slide the class-location unit, design factors and their exceptions, cluster rule, grade against steel, handling D/t, Division 1, then build the line and let the town grow: comply by derating or replacing. Deliverable: the wall schedule for the route and the cost of each compliance option after 15 years.'],
  ['35 min', 'Pipeline Hydrostatic Test Bench', 'Plan the test sections from the profile, fill behind a pig, 80 % hold, pressurise along the P–V line, 2 h hold, then judge two results (leak or temperature?), find yield on the one-section test and the damaged pipe that ruptures. Deliverable: the section plan with the MAOP each test supports.'],
  ['40 min', 'Pipeline Corrosion and Integrity Assessor', 'CP survey: on against instant-off, the IR-drop trap, enough current without overprotection, the railway bond; then the in-line inspection: Appendix L for short and long features, tool tolerance, modified B31G, a valid repair or MAOP reduction for every feature, and ten years of doing nothing. Deliverable: the CP setting and the repair list with the re-inspection interval.'],
  ['35 min', 'Pressure Reduction Station Simulator', 'A normal day, then the set-point ladder; fail the worker with and without the monitor, the hidden monitor failure, relief valve alone (size it, count the gas), lock-up, isolated relief, shared sensing line, cold gas, outlet velocity and the 4 % rule on the old spur. Deliverable: a set-point ladder and protection scheme that passes all nine checks with the failure tests logged.'],
  ['40 min', 'Rotating Machinery Fault Simulator', 'Manufacture each fault and measure it: unbalance and balancing, misalignment phase, looseness harmonics, bearing envelope, gear sidebands, resonance sweep. Finish with three blind machines. Deliverable: a route log per fault with the tell-tale signature named.'],
  ['25 min', 'Hydraulic Power Demonstrator', 'Cylinder rack: stall force and speed at three bores, rod-side ratio, then the failure modes — intensification, meter-in heating, motor overload, overrunning load. Deliverable: force-against-area and speed-against-area plots from the data log, and the cylinder sized for 2 000 kg at 150 bar.'],
  ['25 min', 'Derrick Crane Demonstrator', 'Standard rig first, then each dimension in turn: mast height, boom length, sill reach, leg spread, reeving, anchors. Slew toward a leg, lift an anchor, two-block it. Deliverable: the load chart explained limit by limit and a 10 t at 20 m design with all members under 90 %.'],
  ['25 min', 'Loads on Beams Demonstrator', 'Standard cases first (PL/4, wL²/8, cantilever), then break them: overhangs, fixed ends, propped cantilever, uplift. Section and material: double the depth, I-section against rectangle, steel against aluminium. Deliverable: a sized beam for 20 kN/m over 6 m with the check calculation.'],
  ['35 min', 'Operating-Point Board', 'Teams on the magnetic board: card 0, then A–D (strainer, tank level, throttle, VFD), E–F for fast teams. Predict on the board, verify on the digital board in projector mode or on the loop bench. Deliverable: the team record sheet with hits and the two comparisons (which change cost the most flow; throttle against VFD power).'],
  ['30 min', 'Compressor Map Simulator', 'Start-up on recycle, then the four routes into surge: header pressure, process demand, suction pressure, speed. Surge sequence with a 15 s valve against the same upset with a 2 s valve; manual recovery; choke. Deliverable: the data log rows at each surge with the valve position and the phase named.'],
  ['10 min', 'Side by side', 'Loop bench next to the cavitation trainer: how a system change upstream becomes a suction problem at the pump.'],
];

/* ---------------- progress (per device) ---------------- */
function loadProg() { try { return JSON.parse(localStorage.getItem('sim_progress') || '{}'); } catch (e) { return {}; } }
function saveProg(p) { try { localStorage.setItem('sim_progress', JSON.stringify(p)); } catch (e) {} }
let PROG = loadProg();
function mergeTasks(id, done) { PROG[id] = Array.from(new Set([...(PROG[id] || []), ...(done || [])])); saveProg(PROG); renderProgress(); }
function renderProgress() {
  let total = 0, done = 0; const box = $('#progress'); box.innerHTML = '';
  for (const s of SIMS) { const names = TASK_NAMES[s.id] || {}; const n = Object.keys(names).length, d = (PROG[s.id] || []).filter(k => names[k]).length; total += n; done += d; const row = document.createElement('div'); row.className = 'progrow'; row.innerHTML = `<span>${s.title}</span><div class="prog"><i style="width:${n ? d / n * 100 : 0}%"></i></div><span class="muted">${d} / ${n}</span>`; box.appendChild(row); const card = document.querySelector(`.card[data-id="${s.id}"] .prog i`); if (card) card.style.width = (n ? d / n * 100 : 0) + '%'; const cp = document.querySelector(`.card[data-id="${s.id}"] .pv`); if (cp) cp.textContent = `${d} / ${n} tasks`; }
  $('#pillProgress').textContent = `${done} / ${total} tasks done`;
}

/* ---------------- launch cards ---------------- */
function buildCards() {
  const box = $('#cards'); box.innerHTML = '';
  SIMS.forEach((s, i) => { const c = document.createElement('div'); c.className = 'card'; c.dataset.id = s.id; const num = s.user_numbering ? s.user_numbering.replace(/\D+/, '').padStart(2, '0') : String(i + 1).padStart(2, '0');
    c.innerHTML = `<div class="head"><span class="num">${num}</span><div><h2>${s.title}</h2><div class="topic">${s.topic}</div></div></div><p>${s.summary}</p><div class="chips">${(s.failure_modes || []).map(f => `<span class="chip fail">${f}</span>`).join('')}</div><div class="prog"><i></i></div><div class="muted small pv"></div><div class="actions"><button class="act" data-run="${s.id}">Run here</button><a class="act ghost" href="simulators/${s.path}" target="_blank" rel="noopener">Open in new tab</a></div>`;
    box.appendChild(c); });
  document.querySelectorAll('[data-run]').forEach(b => b.onclick = () => { setView('run'); $('#runSel').value = b.dataset.run; loadRun(); });
  $('#count').textContent = SIMS.length + ' simulators';
}

/* ---------------- run view ---------------- */
let runId = null, pollTimer = null;
function fillSelects() { for (const id of ['runSel', 'splitA', 'splitB']) { const sel = $('#' + id); sel.innerHTML = SIMS.map(s => `<option value="${s.id}">${s.title}</option>`).join(''); } if (SIMS[1]) $('#splitB').value = SIMS[1].id; }
function simById(id) { return SIMS.find(s => s.id === id); }
function loadRun() {
  runId = $('#runSel').value; const s = simById(runId); if (!s) return; $('#runFrame').src = 'simulators/' + s.path; $('#runStatus').textContent = 'Loading ' + s.title + '…'; $('#runLive').innerHTML = '<span class="muted">Waiting for the simulator…</span>';
  const pb = $('#runPresets'); pb.innerHTML = ''; (PRESETS[runId] || []).forEach(([t, d, inputs]) => { const b = document.createElement('button'); b.className = 'preset'; b.innerHTML = `<b>${t}</b><span>${d}</span>`; b.onclick = () => post($('#runFrame'), { type: 'sim:set', inputs }); pb.appendChild(b); });
  renderRunTasks();
}
function renderRunTasks() { const ul = $('#runTasks'); ul.innerHTML = ''; const names = TASK_NAMES[runId] || {}; for (const [k, n] of Object.entries(names)) { const li = document.createElement('li'); li.textContent = n; if ((PROG[runId] || []).includes(k)) li.classList.add('done'); ul.appendChild(li); } }
function post(frame, msg) { try { frame.contentWindow.postMessage(msg, '*'); } catch (e) {} }
function poll() { const v = $('#view-run').classList.contains('on'); if (v && runId) post($('#runFrame'), { type: 'sim:get' }); if ($('#view-split').classList.contains('on')) { post($('#frameA'), { type: 'sim:get' }); post($('#frameB'), { type: 'sim:get' }); } }
window.addEventListener('message', e => {
  const d = e.data || {}; if (d.type !== 'sim:state' || !d.state) return;
  mergeTasks(d.id, d.state.tasksDone);
  if (d.id === runId && e.source === $('#runFrame').contentWindow) {
    const r = d.state.results || {}; const box = $('#runLive'); box.innerHTML = '';
    for (const [k, v] of Object.entries(r)) { const L = LABEL[k] || [k, '', null]; if (v === null || v === undefined || typeof v === 'object') continue; const val = typeof v === 'number' ? (L[1] === 'Pa' ? (v / 1000).toFixed(1) + ' kPa' : (L[1] === 'W' ? (v / 1000).toFixed(2) + ' kW' : (L[2] === null ? (Number.isInteger(v) ? String(v) : +v.toPrecision(4) + '') : v.toFixed(L[2])) + (L[1] ? ' ' + L[1] : ''))) : String(v); const d2 = document.createElement('div'); d2.className = 'lv'; d2.innerHTML = `<div class="k">${L[0]}</div><div class="v">${val}</div>`; box.appendChild(d2); }
    const s = simById(runId); $('#runStatus').textContent = `${s.title} · live · ${(d.state.tasksDone || []).length} task(s) done this session`; renderRunTasks();
  }
});

/* ---------------- views, split, session ---------------- */
function setView(v) { document.querySelectorAll('.vbtn').forEach(b => b.classList.toggle('on', b.dataset.view === v)); document.querySelectorAll('.view').forEach(x => x.classList.toggle('on', x.id === 'view-' + v)); if (v === 'run' && !$('#runFrame').src) loadRun(); if (v === 'split') loadSplit(); }
function loadSplit() { const a = simById($('#splitA').value), b = simById($('#splitB').value); if (a && !$('#frameA').src.endsWith(a.path)) $('#frameA').src = 'simulators/' + a.path; if (b && !$('#frameB').src.endsWith(b.path)) $('#frameB').src = 'simulators/' + b.path; }
function buildPlan() { const ol = $('#plan'); ol.innerHTML = ''; for (const [t, sim, what] of PLAN) { const li = document.createElement('li'); li.innerHTML = `<b>${sim} <span class="t">${t}</span></b><span>${what}</span>`; ol.appendChild(li); } }

function build() {
  try { const th = localStorage.getItem('sim_theme'); if (th) document.documentElement.setAttribute('data-theme', th); } catch (e) {}
  buildCards(); fillSelects(); buildPlan(); renderProgress();
  document.querySelectorAll('.vbtn').forEach(b => b.onclick = () => setView(b.dataset.view));
  $('#runSel').onchange = loadRun; $('#runReload').onclick = loadRun; $('#runOpenTab').onclick = () => { const s = simById(runId); if (s) window.open('simulators/' + s.path, '_blank'); };
  $('#splitA').onchange = loadSplit; $('#splitB').onchange = loadSplit;
  $('#btnClearProgress').onclick = () => { PROG = {}; saveProg(PROG); renderProgress(); renderRunTasks(); };
  $('#btnTheme').onclick = () => { const r = document.documentElement; const next = r.getAttribute('data-theme') === 'dark' ? 'light' : 'dark'; r.setAttribute('data-theme', next); try { localStorage.setItem('sim_theme', next); } catch (e) {} };
  $('#btnFull').onclick = () => { const d = document.documentElement; if (!document.fullscreenElement) (d.requestFullscreen || d.webkitRequestFullscreen).call(d); else (document.exitFullscreen || document.webkitExitFullscreen).call(document); };
  pollTimer = setInterval(poll, 1000);
}
build();
