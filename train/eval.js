// Closed-loop evaluation of both controllers on the truth sim, headless,
// same protocol as the page's trials (6 s window, settle = error under 6 px
// held for 0.8 s, steady state = mean error over the final second).
// Run: node train/eval.js   Prints a markdown table for the README.
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');

const W = 460, H = 345;
const DT = 1 / 60;
const TRIALS = 40;
const TRIAL_S = 6;
const SETTLE_PX = 6, SETTLE_HOLD = 0.8;

const weights = JSON.parse(fs.readFileSync(path.join(__dirname, 'weights.json'), 'utf8'));
const cam = CR.camera.defaultCamera(W, H);

const CONDITIONS = [
  { name: 'nominal', payload: 0, drift: false },
  { name: 'payload', payload: 1, drift: false },
  { name: 'drift', payload: 0, drift: true },
  { name: 'payload + drift', payload: 1, drift: true },
];

function randomQ(rng, scale) {
  const q = [];
  for (let i = 0; i < 2; i++) {
    const a = rng() * 2 * Math.PI;
    const k = Math.sqrt(rng()) * scale * CR.pcc.KMAX[i];
    q.push(k * Math.cos(a), k * Math.sin(a));
  }
  return q;
}

function runTrial(kind, cond, seed) {
  const rng = CR.makeRng(seed);
  const sim = CR.truth.createTruth(seed);
  sim.payload = cond.payload;
  sim.driftOn = cond.drift;
  const q0 = randomQ(rng, 0.7);
  sim.reset(q0);
  for (let i = 0; i < 120; i++) sim.step(DT); // settle + accumulate drift

  const ctrl = kind === 'classical'
    ? CR.ibvs.createClassical(cam)
    : CR.learned.createLearned(weights);
  ctrl.reset(q0);
  const target = cam.project(CR.pcc.tip3(randomQ(rng, 0.85)));
  if (!target) return null;

  let bandEnter = null, settle = null, oodSteps = 0;
  const tail = [];
  const steps = Math.round(TRIAL_S / DT);
  for (let s = 0; s < steps; s++) {
    const t = s * DT;
    const markersPx = sim.markers3().map((p) => cam.project(p));
    const tip = markersPx[3];
    const err = Math.hypot(tip[0] - target[0], tip[1] - target[1]);
    if (err < SETTLE_PX) {
      if (bandEnter == null) bandEnter = t;
      if (settle == null && t - bandEnter >= SETTLE_HOLD) settle = bandEnter;
    } else bandEnter = null;
    if (t > TRIAL_S - 1) tail.push(err);

    if (kind === 'classical') {
      sim.setCommand(ctrl.step([tip[0], tip[1]], target, DT));
    } else {
      const out = ctrl.step(markersPx, target, DT, W, H);
      sim.setCommand(out.qCmd);
      if (out.ood) oodSteps++;
    }
    sim.step(DT);
  }
  return {
    settle,
    ss: tail.reduce((a, b) => a + b, 0) / tail.length,
    oodFrac: oodSteps / steps,
  };
}

function median(a) {
  const s = a.slice().sort((x, y) => x - y);
  return s.length ? s[Math.floor(s.length / 2)] : NaN;
}

console.log('| condition | controller | settled | median settle | mean steady-state | OOD flagged |');
console.log('|---|---|---|---|---|---|');
for (const cond of CONDITIONS) {
  for (const kind of ['classical', 'learned']) {
    const res = [];
    for (let n = 0; n < TRIALS; n++) {
      const r = runTrial(kind, cond, 31000 + n * 7);
      if (r) res.push(r);
    }
    const settled = res.filter((r) => r.settle != null);
    const ood = res.reduce((a, r) => a + r.oodFrac, 0) / res.length;
    console.log(
      '| ' + cond.name + ' | ' + kind + ' | ' +
      settled.length + '/' + res.length + ' | ' +
      (settled.length ? median(settled.map((r) => r.settle)).toFixed(2) + ' s' : 'n/a') + ' | ' +
      (res.reduce((a, r) => a + r.ss, 0) / res.length).toFixed(1) + ' px | ' +
      (kind === 'learned' ? (100 * ood).toFixed(0) + '% of steps |' : '– |'));
  }
}
