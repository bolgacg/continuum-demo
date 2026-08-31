// Closed-loop evaluation of both controllers on the truth sim, headless,
// same protocol as the page's trials (6 s window, settle = error under 6 px
// held for 0.8 s, steady state = mean error over the final second).
//
// Two target sets: "interior" (tips of random configurations at up to 85% of
// the curvature limit, the version 1 protocol) and "eccentric" (tips at 95 to
// 100% of the limit, near the workspace edge, where version 1 was never
// tested). Each controller runs as version 1 (direct image loop) and version
// 2 (plan-then-track, src/core/planner.js).
// Run: node train/eval.js   Prints markdown tables for the README.
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
const planner = CR.planner.createPlanner(cam);

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

function makeCtrl(kind, version) {
  const base = kind === 'classical'
    ? CR.ibvs.createClassical(cam)
    : CR.learned.createLearned(weights);
  return version === 2 ? CR.planner.createTracked(base, planner, kind) : base;
}

function runTrial(kind, version, cond, targetSet, seed) {
  const rng = CR.makeRng(seed);
  const sim = CR.truth.createTruth(seed);
  sim.payload = cond.payload;
  sim.driftOn = cond.drift;
  const q0 = randomQ(rng, 0, 0.7);
  sim.reset(q0);
  for (let i = 0; i < 120; i++) sim.step(DT); // settle + accumulate drift

  const ctrl = makeCtrl(kind, version);
  ctrl.reset(q0);
  let target = null;
  for (let tries = 0; tries < 50 && !target; tries++) {
    const q = targetSet === 'eccentric' ? randomQ(rng, 0.95, 1.0) : randomQ(rng, 0, 0.85);
    const p = cam.project(CR.pcc.tip3(q));
    if (p && p[0] > 8 && p[0] < W - 8 && p[1] > 8 && p[1] < H - 8) target = [p[0], p[1]];
  }
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

    if (version === 1 && kind === 'classical') {
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

for (const targetSet of ['interior', 'eccentric']) {
  console.log('\n### ' + targetSet + ' targets\n');
  console.log('| condition | controller | v1 settled | v1 median settle | v1 steady-state | v2 settled | v2 median settle | v2 steady-state |');
  console.log('|---|---|---|---|---|---|---|---|');
  for (const cond of CONDITIONS) {
    for (const kind of ['classical', 'learned']) {
      const cells = [];
      for (const version of [1, 2]) {
        const res = [];
        for (let n = 0; n < TRIALS; n++) {
          const r = runTrial(kind, version, cond, targetSet, 31000 + n * 7);
          if (r) res.push(r);
        }
        const settled = res.filter((r) => r.settle != null);
        cells.push(settled.length + '/' + res.length);
        cells.push(settled.length ? median(settled.map((r) => r.settle)).toFixed(2) + ' s' : 'n/a');
        cells.push((res.reduce((a, r) => a + r.ss, 0) / res.length).toFixed(1) + ' px');
      }
      console.log('| ' + cond.name + ' | ' + kind + ' | ' + cells.join(' | ') + ' |');
    }
  }
}
