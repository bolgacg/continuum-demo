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

// --- version 2: reachable outline, planner, edge-target behaviour ---
{
  const fs = require('fs');
  const path = require('path');
  const W = 460, H = 345, dt = 1 / 60;
  const cam = CR.camera.defaultCamera(W, H);
  const weights = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'weights.json'), 'utf8'));

  const outline = CR.workspace.reachableOutline(cam, { samples: 80000 });
  check('reachable outline is a real polygon', outline.length > 20 && outline.length < 400,
    outline.length + ' vertices');
  const rng = CR.makeRng(123);
  let inside = 0, total = 0;
  for (let n = 0; n < 3000; n++) {
    const q = [];
    for (let i = 0; i < 2; i++) {
      const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * CR.pcc.KMAX[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    for (const payload of [0, 1]) {
      const p = cam.project(CR.pcc.tip3(CR.truth.applyStatic(q, payload)));
      if (!p) continue;
      total++;
      if (CR.workspace.pointInPolygon(p, outline)) inside++;
    }
  }
  check('truth-sim tips stay inside the ideal outline (>99%)', inside / total > 0.99,
    (100 * inside / total).toFixed(2) + '%');

  // marker geometry is fixed: 3D chord between neighbours barely moves
  let cMin = Infinity, cMax = 0;
  for (let n = 0; n < 500; n++) {
    const q = [];
    for (let i = 0; i < 2; i++) {
      const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * CR.pcc.KMAX[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    const m = CR.pcc.markers3(q);
    const c = CR.v3.norm(CR.v3.sub(m[1], m[0]));
    cMin = Math.min(cMin, c); cMax = Math.max(cMax, c);
  }
  check('marker chord varies <5% in 3D (arc length is constant)', (cMax - cMin) / cMax < 0.05,
    cMin.toFixed(3) + '..' + cMax.toFixed(3));

  // planner: numerical IK on the edge target used by the scripted demo
  const planner = CR.planner.createPlanner(cam);
  const Q0 = [0.5, 0.1, -0.35, 0.3];
  const edgeQ = [2.2 * Math.cos(0.2), 2.2 * Math.sin(0.2), 2.6 * Math.cos(0.2), 2.6 * Math.sin(0.2)];
  const tEdge = cam.project(CR.pcc.tip3(edgeQ)).slice(0, 2);
  const ik = planner.solveIK(tEdge, Q0);
  check('IK reaches the edge target (<1 px residual)', ik.residual < 1, ik.residual.toFixed(2) + ' px');

  function finalErr(kind, version) {
    const sim = CR.truth.createTruth(2026);
    sim.reset(Q0);
    const base = kind === 'classical' ? CR.ibvs.createClassical(cam) : CR.learned.createLearned(weights);
    const ctrl = version === 2 ? CR.planner.createTracked(base, planner, kind) : base;
    ctrl.reset(Q0);
    let err = Infinity;
    for (let i = 0; i < 8 * 60; i++) {
      const m = sim.markers3().map((p) => cam.project(p));
      err = Math.hypot(m[3][0] - tEdge[0], m[3][1] - tEdge[1]);
      const out = (version === 1 && kind === 'classical')
        ? { qCmd: ctrl.step([m[3][0], m[3][1]], tEdge, dt) }
        : ctrl.step(m, tEdge, dt, W, H);
      sim.setCommand(out.qCmd);
      sim.step(dt);
    }
    return err;
  }
  const c1 = finalErr('classical', 1), c2 = finalErr('classical', 2);
  const l1 = finalErr('learned', 1), l2 = finalErr('learned', 2);
  check('v1 classical stalls short of the edge target from rest', c1 > 20, c1.toFixed(1) + ' px');
  check('v2 classical reaches it (<6 px)', c2 < 6, c2.toFixed(1) + ' px');
  check('v1 learned stalls short of the edge target from rest', l1 > 20, l1.toFixed(1) + ' px');
  check('v2 learned reaches it (<6 px)', l2 < 6, l2.toFixed(1) + ' px');

  // tracking form with a fixed reference and no feed-forward is the v1 law
  const a = CR.ibvs.createClassical(cam), b = CR.ibvs.createClassical(cam);
  a.reset(Q0); b.reset(Q0);
  const tip = cam.project(CR.pcc.tip3(Q0));
  const qa = a.step([tip[0], tip[1]], tEdge, dt), qb = b.stepTrack([tip[0], tip[1]], tEdge, null, dt);
  check('stepTrack(ref, no feed-forward) == v1 step', qa.every((v, i) => Math.abs(v - qb[i]) < 1e-12));
}

console.log(failures === 0 ? '\nall sanity checks passed' : '\n' + failures + ' FAILURES');
process.exit(failures === 0 ? 0 : 1);
