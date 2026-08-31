// Closed-loop evaluation (version 3, 3D task), headless, same protocol as the
// page's trials: 6 s window, settle = tip error under 5 mm (0.05 length
// units) held for 0.8 s, steady state = mean error over the final second.
// Display convention: 1 length unit = 100 mm.
//
// Two target sets: "interior" (tips of random configurations at up to 85% of
// the curvature limit) and "edge" (tips at 95 to 100% of the limit), both
// restricted to above the table the upright robot stands on. Each
// controller runs as the direct law and under the plan-then-track layer.
// Run: node train/eval.js   Prints markdown tables for the README and page.
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');

const W = 460, H = 345;
const DT = 1 / 60;
const TRIALS = 40;
const TRIAL_S = 6;
const SETTLE_U = 0.05, SETTLE_HOLD = 0.8;
const MM = 100;

const weights = JSON.parse(fs.readFileSync(path.join(__dirname, 'weights.json'), 'utf8'));
const trainIK = CR.planner.createPlanner({ limitScale: 0.9 * CR.pcc.FLEX_RANGE[1], seed: 13 }); // population the ensemble was trained on
const targetTest = (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable;
const camSide = CR.camera.sideCamera(W, H), camTop = CR.camera.topCamera(W, H);
const planner = CR.planner.createPlanner();

const CONDITIONS = [
  { name: 'nominal', payload: 0, drift: false },
  { name: 'payload', payload: 1, drift: false },
  { name: 'drift', payload: 0, drift: true },
  { name: 'payload + drift', payload: 1, drift: true },
];

function randomQ(rng, lo, hi) {
  const q = [];
  for (let i = 0; i < 2; i++) {
    const a = rng() * 2 * Math.PI;
    const k = (lo + (hi - lo) * Math.sqrt(rng())) * CR.pcc.KMAX[i];
    q.push(k * Math.cos(a), k * Math.sin(a));
  }
  return q;
}

function makeCtrl(kind, mode) {
  const base = kind === 'classical' ? CR.ibvs.createClassical() : CR.learned.createLearned(weights, { targetTest });
  return mode === 'planned' ? CR.planner.createTracked(base, planner, kind) : CR.planner.createDirect(base, kind);
}

function runTrial(kind, mode, cond, targetSet, seed) {
  const rng = CR.makeRng(seed);
  const sim = CR.truth.createTruth(seed);
  sim.payload = cond.payload;
  sim.driftOn = cond.drift;
  const q0 = randomQ(rng, 0, 0.7);
  sim.reset(q0);
  for (let i = 0; i < 120; i++) sim.step(DT);

  const ctrl = makeCtrl(kind, mode);
  ctrl.reset(q0);
  // targets above the table the robot stands on (z >= 0.05); the curled-back
  // region below the base is physically blocked and not evaluated
  let target = null;
  for (let tries = 0; tries < 200 && !target; tries++) {
    const p = CR.pcc.tip3(targetSet === 'edge' ? randomQ(rng, 0.95, 1.0) : randomQ(rng, 0, 0.85));
    if (p[2] >= 0.05) target = p;
  }
  if (!target) return null;

  let bandEnter = null, settle = null, oodSteps = 0;
  const tail = [];
  const steps = Math.round(TRIAL_S / DT);
  for (let s = 0; s < steps; s++) {
    const t = s * DT;
    const markers = CR.camera.senseMarkers(sim.markers3(), camSide, camTop);
    if (!markers) return null;
    const err = CR.v3.norm(CR.v3.sub(markers[3], target));
    if (err < SETTLE_U) {
      if (bandEnter == null) bandEnter = t;
      if (settle == null && t - bandEnter >= SETTLE_HOLD) settle = bandEnter;
    } else bandEnter = null;
    if (t > TRIAL_S - 1) tail.push(err);
    const out = ctrl.step(markers, target, DT);
    sim.setCommand(out.qCmd);
    if (out.ood) oodSteps++;
    sim.step(DT);
  }
  return { settle, ss: tail.reduce((a, b) => a + b, 0) / tail.length, oodFrac: oodSteps / steps };
}

function median(a) {
  const s = a.slice().sort((x, y) => x - y);
  return s.length ? s[Math.floor(s.length / 2)] : NaN;
}

const rowsOut = [];
for (const targetSet of ['interior', 'edge']) {
  console.log('\n### ' + targetSet + ' targets\n');
  console.log('| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state | OOD flagged (planned) |');
  console.log('|---|---|---|---|---|---|---|---|---|');
  for (const cond of CONDITIONS) {
    for (const kind of ['classical', 'learned']) {
      const cells = [];
      let oodCell = '–';
      for (const mode of ['direct', 'planned']) {
        const res = [];
        for (let n = 0; n < TRIALS; n++) {
          const r = runTrial(kind, mode, cond, targetSet, 31000 + n * 7);
          if (r) res.push(r);
        }
        const settled = res.filter((r) => r.settle != null);
        cells.push(settled.length + '/' + res.length);
        cells.push(settled.length ? median(settled.map((r) => r.settle)).toFixed(2) + ' s' : 'n/a');
        cells.push((res.reduce((a, r) => a + r.ss, 0) / res.length * MM).toFixed(1) + ' mm');
        if (mode === 'planned' && kind === 'learned') {
          oodCell = (100 * res.reduce((a, r) => a + r.oodFrac, 0) / res.length).toFixed(0) + '% of steps';
        }
      }
      console.log('| ' + cond.name + ' | ' + kind + ' | ' + cells.join(' | ') + ' | ' + oodCell + ' |');
      rowsOut.push({ targets: targetSet, condition: cond.name, controller: kind, cells, ood: oodCell });
    }
  }
}
fs.writeFileSync(path.join(__dirname, 'eval.json'), JSON.stringify({ date: '2026-08-31', trials: TRIALS, rows: rowsOut }, null, 1));
console.log('\nwrote train/eval.json');
