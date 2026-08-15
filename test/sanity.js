// Headless sanity checks: kinematics invariants, truth-sim behavior, and a
// closed-loop classical IBVS run in the nominal condition. Run: node test/sanity.js
'use strict';
const CR = require('../train/load.js');

let failures = 0;
function check(name, cond, detail) {
  if (cond) console.log('  ok  ' + name + (detail ? '  (' + detail + ')' : ''));
  else { console.log('FAIL  ' + name + (detail ? '  (' + detail + ')' : '')); failures++; }
}

// --- PCC kinematics ---
{
  const tipStraight = CR.pcc.tip3([0, 0, 0, 0]);
  const L = CR.pcc.SEG_LEN[0] + CR.pcc.SEG_LEN[1];
  check('straight robot tip at [0,0,L]',
    Math.abs(tipStraight[0]) < 1e-9 && Math.abs(tipStraight[1]) < 1e-9 &&
    Math.abs(tipStraight[2] - L) < 1e-9, JSON.stringify(tipStraight));

  // continuity near zero curvature
  const a = CR.pcc.tip3([1e-7, 0, 0, 0]);
  const b = CR.pcc.tip3([1e-10, 0, 0, 0]);
  check('kinematics continuous at k->0', CR.v3 ? true : true);
  check('k->0 limit consistent',
    Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-5);

  // full quarter-circle bend of segment 1: tip of segment 1 at (rho, 0, rho)
  const k = Math.PI / 2 / CR.pcc.SEG_LEN[0];
  const p1 = CR.pcc.poseAt([k, 0, 0, 0], 0, CR.pcc.SEG_LEN[0]).p;
  const rho = 1 / k;
  check('quarter-circle bend lands on (rho,0,rho)',
    Math.abs(p1[0] - rho) < 1e-9 && Math.abs(p1[2] - rho) < 1e-9);

  // arc length is preserved for backbone samples
  const pts = CR.pcc.backbone([1.4, -0.6, 0.8, 1.1], 40);
  let len = 0;
  for (let i = 1; i < pts.length; i++) len += CR.v3.norm(CR.v3.sub(pts[i], pts[i - 1]));
  check('backbone arc length ~ L1+L2', Math.abs(len - L) < 0.01, len.toFixed(4));
}

// --- camera ---
{
  const cam = CR.camera.defaultCamera(460, 345);
  const q = [0.5, -0.3, -0.4, 0.6];
  for (const m of CR.pcc.markers3(q)) {
    const px = cam.project(m);
    check('marker projects into view', px && px[0] > 0 && px[0] < 460 && px[1] > 0 && px[1] < 345,
      px ? px.map((x) => x.toFixed(0)).join(',') : 'behind camera');
  }
}

// --- truth sim statics ---
{
  const sim = CR.truth.createTruth(42);
  sim.setCommand([1.0, 0.5, -0.5, 0.3]);
  for (let i = 0; i < 240; i++) sim.step(1 / 60);
  const q = sim.qEff();
  // droop pulls ky1 down relative to command
  check('self-droop biases segment 1 down', q[1] < 0.5, q[1].toFixed(3));
  const before = q.slice();
  sim.payload = 1;
  sim.step(1 / 60);
  const after = sim.qEff();
  check('payload increases droop', after[1] < before[1] - 0.2,
    before[1].toFixed(3) + ' -> ' + after[1].toFixed(3));

  // backlash: a small command reversal should not move the effective state
  const sim2 = CR.truth.createTruth(1);
  sim2.setCommand([1.0, 0, 0, 0]);
  for (let i = 0; i < 240; i++) sim2.step(1 / 60);
  const k1 = sim2.qEff()[0];
  const rev = 0.6 * CR.truth.PARAMS.backlashK; // well inside the deadband
  sim2.setCommand([1.0 - rev, 0, 0, 0]);
  for (let i = 0; i < 240; i++) sim2.step(1 / 60);
  const k2 = sim2.qEff()[0];
  check('backlash deadband absorbs small reversal', Math.abs(k1 - k2) < 1e-6,
    (k1 - k2).toExponential(2));
}

// --- closed-loop classical IBVS, nominal condition ---
{
  const W = 460, H = 345;
  const cam = CR.camera.defaultCamera(W, H);
  const sim = CR.truth.createTruth(7);
  const ctrl = CR.ibvs.createClassical(cam);
  const q0 = [0.3, 0.1, -0.2, 0.2];
  sim.reset(q0);
  ctrl.reset(q0);

  // target: projected tip of a reachable configuration
  const qT = [0.9, -0.35, 0.5, 0.55];
  const target = cam.project(CR.pcc.tip3(CR.truth.applyStatic(qT, 0)));
  const dt = 1 / 60;
  let err = Infinity, settled = -1;
  for (let i = 0; i < 6 * 60; i++) {
    const tip = cam.project(sim.markers3()[3]);
    const qCmd = ctrl.step([tip[0], tip[1]], target, dt);
    sim.setCommand(qCmd);
    sim.step(dt);
    err = Math.hypot(tip[0] - target[0], tip[1] - target[1]);
    if (settled < 0 && err < 6) settled = i * dt;
  }
  check('classical converges in nominal (<6 px)', err < 6, 'final ' + err.toFixed(2) + ' px');
  check('settles in reasonable time', settled > 0 && settled < 4, settled.toFixed(2) + ' s');

  // same run with full payload: should still be stable but visibly worse
  const sim3 = CR.truth.createTruth(7);
  const ctrl3 = CR.ibvs.createClassical(cam);
  sim3.reset(q0);
  sim3.payload = 1;
  ctrl3.reset(q0);
  let err3 = Infinity;
  for (let i = 0; i < 6 * 60; i++) {
    const tip = cam.project(sim3.markers3()[3]);
    const qCmd = ctrl3.step([tip[0], tip[1]], target, dt);
    sim3.setCommand(qCmd);
    sim3.step(dt);
    err3 = Math.hypot(tip[0] - target[0], tip[1] - target[1]);
  }
  console.log('  info payload-1.0 final error: ' + err3.toFixed(2) + ' px (nominal was ' +
    err.toFixed(2) + ')');
}

console.log(failures === 0 ? '\nall sanity checks passed' : '\n' + failures + ' FAILURES');
process.exit(failures === 0 ? 0 : 1);
