// Headless sanity checks (version 3): kinematics invariants, cameras and
// triangulation, truth-sim behaviour, closed-loop control on the 3D task, the
// planner, and the reachable envelope. Run: node test/sanity.js
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('../train/load.js');
const { pcc, truth, ibvs, camera, planner, workspace, v3 } = CR;

let failures = 0;
function check(name, cond, detail) {
  if (cond) console.log('  ok  ' + name + (detail ? '  (' + detail + ')' : ''));
  else { console.log('FAIL  ' + name + (detail ? '  (' + detail + ')' : '')); failures++; }
}
function randomQ(rng, scale) {
  const q = [];
  for (let i = 0; i < 2; i++) {
    const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * scale * pcc.KMAX[i];
    q.push(k * Math.cos(a), k * Math.sin(a));
  }
  return q;
}

// --- PCC kinematics ---
{
  const tipStraight = pcc.tip3([0, 0, 0, 0]);
  const L = pcc.SEG_LEN[0] + pcc.SEG_LEN[1];
  check('straight robot tip at [0,0,L]',
    Math.abs(tipStraight[0]) < 1e-9 && Math.abs(tipStraight[1]) < 1e-9 && Math.abs(tipStraight[2] - L) < 1e-9);
  const a = pcc.tip3([1e-7, 0, 0, 0]), b = pcc.tip3([1e-10, 0, 0, 0]);
  check('k->0 limit consistent', v3.norm(v3.sub(a, b)) < 1e-5);
  const k = Math.PI / 2 / pcc.SEG_LEN[0];
  const p1 = pcc.poseAt([k, 0, 0, 0], 0, pcc.SEG_LEN[0]).p;
  check('quarter-circle bend lands on (rho,0,rho)', Math.abs(p1[0] - 1 / k) < 1e-9 && Math.abs(p1[2] - 1 / k) < 1e-9);
  const pts = pcc.backbone([1.4, -0.6, 0.8, 1.1], 40);
  let len = 0;
  for (let i = 1; i < pts.length; i++) len += v3.norm(v3.sub(pts[i], pts[i - 1]));
  check('backbone arc length ~ L1+L2 (fixed geometry)', Math.abs(len - L) < 0.01, len.toFixed(4));
  // marker geometry is fixed: 3D chord between neighbours barely moves
  const rng = CR.makeRng(123);
  let cMin = Infinity, cMax = 0;
  for (let n = 0; n < 500; n++) {
    const m = pcc.markers3(randomQ(rng, 1));
    const c = v3.norm(v3.sub(m[1], m[0]));
    cMin = Math.min(cMin, c); cMax = Math.max(cMax, c);
  }
  check('marker chord varies <5% in 3D (arc length is constant)', (cMax - cMin) / cMax < 0.05, cMin.toFixed(3) + '..' + cMax.toFixed(3));
}

// --- cameras + triangulation ---
const W = 460, H = 345;
const camSide = camera.sideCamera(W, H), camTop = camera.topCamera(W, H);
{
  const c = camera.CENTER;
  const p0 = camSide.project(c), pz = camSide.project(v3.add(c, [0, 0, 0.3])), py = camSide.project(v3.add(c, [0, 0.3, 0]));
  check('side sensor: axis (+z) is image-up, +y is image-right', pz[1] < p0[1] && py[0] > p0[0]);
  const t0 = camTop.project(c), ty = camTop.project(v3.add(c, [0, 0.3, 0])), tx = camTop.project(v3.add(c, [0.3, 0, 0]));
  check('top sensor: +y is image-right, +x is image-down', ty[0] > t0[0] && tx[1] > t0[1]);
  const rng = CR.makeRng(9);
  let worst = 0;
  for (let n = 0; n < 500; n++) {
    const m = pcc.markers3(randomQ(rng, 1));
    const s = camera.senseMarkers(m, camSide, camTop);
    if (!s) { worst = Infinity; break; }
    for (let i = 0; i < 4; i++) worst = Math.max(worst, v3.norm(v3.sub(s[i], m[i])));
  }
  check('two-view triangulation exact (<1e-9)', worst < 1e-9, worst.toExponential(2));
  const cam = camera.orbitCamera(0.95, 0.52, W, H);
  const p = [0.3, -0.2, 1.1], px = cam.project(p);
  const hit = camera.rayPlaneZ(cam.pos, cam.rayDir(px[0], px[1]), p[2]);
  check('click ray x height plane recovers the point', hit && v3.norm(v3.sub(hit, p)) < 1e-9);
}

// --- truth sim statics ---
{
  // upright robot: no sag while straight, more sag the further it leans
  const straight = truth.applyStatic([0, 0, 0, 0], 1);
  check('upright straight robot does not sag under payload', straight.every((v) => Math.abs(v) < 1e-9));
  // static map alone (no backlash): a leaning segment sags further, payload more so
  const lean = [1.0, 0.5, -0.5, 0.3];
  const k1cmd = Math.hypot(lean[0], lean[1]);
  const s0 = truth.applyStatic(lean, 0), s1 = truth.applyStatic(lean, 1);
  const k1s0 = Math.hypot(s0[0], s0[1]), k1s1 = Math.hypot(s1[0], s1[1]);
  check('leaning segment sags further under its own weight', k1s0 > k1cmd, k1cmd.toFixed(3) + ' -> ' + k1s0.toFixed(3));
  check('payload increases the sag of a leaning segment', k1s1 > k1s0 + 0.1, k1s1.toFixed(3));
  // through the dynamics, backlash costs a little curvature; still stable
  const sim = truth.createTruth(42);
  sim.setCommand(lean);
  for (let i = 0; i < 240; i++) sim.step(1 / 60);
  check('dynamic steady state is finite and near the static map', sim.qEff().every((v) => Number.isFinite(v)) && Math.abs(Math.hypot(sim.qEff()[0], sim.qEff()[1]) - k1s0) < 0.1);
  const sim2 = truth.createTruth(1);
  sim2.setCommand([1.0, 0, 0, 0]);
  for (let i = 0; i < 240; i++) sim2.step(1 / 60);
  const k1 = sim2.qEff()[0];
  sim2.setCommand([1.0 - 0.6 * truth.PARAMS.backlashK, 0, 0, 0]);
  for (let i = 0; i < 240; i++) sim2.step(1 / 60);
  check('backlash deadband absorbs small reversal', Math.abs(k1 - sim2.qEff()[0]) < 1e-6);
}

// --- closed loop, planner, envelope ---
{
  const weights = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'weights.json'), 'utf8'));
  const wsAll = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'workspace.json'), 'utf8'));
  const trainIK = planner.createPlanner({ limitScale: 0.9 * pcc.FLEX_RANGE[1], seed: 13 });
  const targetTest = (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable;
  const pl = planner.createPlanner();
  const Q0 = [0.5, 0.1, -0.35, 0.3];
  const dt = 1 / 60;
  function run(kind, mode, target, secs, payload) {
    const sim = truth.createTruth(2026);
    sim.payload = payload || 0;
    sim.reset(Q0);
    const base = kind === 'classical' ? ibvs.createClassical() : CR.learned.createLearned(weights, { targetTest });
    const ctrl = mode === 'planned' ? planner.createTracked(base, pl, kind) : planner.createDirect(base, kind);
    ctrl.reset(Q0);
    let err = Infinity, settled = -1;
    for (let i = 0; i < secs * 60; i++) {
      const m = camera.senseMarkers(sim.markers3(), camSide, camTop);
      err = v3.norm(v3.sub(m[3], target));
      if (settled < 0 && err < 0.05) settled = i * dt;
      const out = ctrl.step(m, target, dt);
      sim.setCommand(out.qCmd);
      sim.step(dt);
    }
    return { err, settled };
  }
  const tIn = pcc.tip3([0.9, -0.35, 0.5, 0.55]);
  let r = run('classical', 'direct', tIn, 6);
  check('classical direct converges nominal (<5 mm)', r.err < 0.05, (r.err * 100).toFixed(2) + ' mm, first in band ' + r.settled.toFixed(2) + ' s');
  r = run('classical', 'direct', tIn, 6, 1);
  check('classical direct under payload (<5 mm)', r.err < 0.05, (r.err * 100).toFixed(2) + ' mm');
  r = run('learned', 'direct', tIn, 6);
  check('learned direct converges nominal (<5 mm)', r.err < 0.05, (r.err * 100).toFixed(2) + ' mm');
  r = run('learned', 'direct', tIn, 6, 1);
  check('learned direct under payload (<5 mm)', r.err < 0.05, (r.err * 100).toFixed(2) + ' mm');

  const edgeQ = [2.2 * Math.cos(0.2), 2.2 * Math.sin(0.2), 2.6 * Math.cos(0.2), 2.6 * Math.sin(0.2)];
  const tEdge = pcc.tip3(edgeQ);
  const ik = pl.solveIK(tEdge, Q0);
  check('IK reaches the edge target (<1 mm residual)', ik.residual < 0.01, (ik.residual * 100).toFixed(3) + ' mm');
  const c1 = run('classical', 'direct', tEdge, 8).err, c2 = run('classical', 'planned', tEdge, 8).err;
  const l1 = run('learned', 'direct', tEdge, 8).err, l2 = run('learned', 'planned', tEdge, 8).err;
  check('demo edge target (curled back below the base): direct classical stalls (>20 mm)', c1 > 0.2, (c1 * 100).toFixed(1) + ' mm');
  check('planned classical reaches it (<5 mm)', c2 < 0.05, (c2 * 100).toFixed(1) + ' mm');
  console.log('  info direct learned on the same target from rest: ' + (l1 * 100).toFixed(1) + ' mm');
  check('planned learned reaches it (<5 mm)', l2 < 0.05, (l2 * 100).toFixed(1) + ' mm');

  // tracking form with a fixed reference and no feed-forward is the direct law
  const a = ibvs.createClassical(), b = ibvs.createClassical();
  a.reset(Q0); b.reset(Q0);
  const tip = pcc.tip3(Q0);
  const qa = a.step(tip, tEdge, dt), qb = b.stepTrack(tip, tEdge, null, dt);
  check('stepTrack(ref, no feed-forward) == direct step', qa.every((v, i) => Math.abs(v - qb[i]) < 1e-12));

  // envelope: truth tips (payload 0 and 1) stay inside
  const vol = wsAll.reach;
  const rng = CR.makeRng(5);
  let inside = 0, total = 0;
  for (let n = 0; n < 3000; n++) {
    const q = randomQ(rng, 1);
    for (const payload of [0, 1]) {
      total++;
      if (workspace.insideEnvelope(vol, pcc.tip3(truth.applyStatic(q, payload)))) inside++;
    }
  }
  check('truth-sim tips inside the reachable envelope (>99%)', inside / total > 0.99, (100 * inside / total).toFixed(2) + '%');
  const mesh = workspace.envelopeMesh(vol);
  check('envelope mesh shape', mesh.length === vol.nt + 2 && mesh[1].length === vol.np);
  check('a point far above the envelope is outside', !workspace.insideEnvelope(vol, [0.1, 1.9, 0.6]));
  check('the demo\'s beyond-reach target is outside the envelope', !workspace.insideEnvelope(vol, [0.1, 0.0, 2.3]));
  // occupancy grid agrees with the planner's reachability on random points
  const grid = workspace.gridFromJSON(wsAll.grid);
  let agree = 0, nPts = 0, gridYes = 0, ikYes = 0;
  const rg = CR.makeRng(77);
  for (let n = 0; n < 300; n++) {
    const p = [-1.6 + 3.2 * rg(), -1.6 + 3.2 * rg(), -1.1 + 3.2 * rg()];
    const g = workspace.gridContains(grid, p), k = pl.solveIK(p, [0, 0, 0, 0]).reachable;
    nPts++; if (g === k) agree++; if (g) gridYes++; if (k) ikYes++;
  }
  check('reach grid agrees with IK reachability on >=93% of random points', agree / nPts >= 0.93, (100 * agree / nPts).toFixed(1) + '% (grid ' + gridYes + ', IK ' + ikYes + ' of ' + nPts + ')');
  check('a slice of the grid has contour segments', workspace.gridSectionSegments(grid, 1.2).length > 50);
  // flexibility: scaling the curvature limits enlarges the reachable set
  const cellsAt = (f) => { pcc.setFlex(f); const g = workspace.reachableGrid({ samples: 60000 }); pcc.setFlex(1); let n = 0; for (const b of g.bits) for (let k = 0; k < 8; k++) n += (b >> k) & 1; return n; };
  const n10 = cellsAt(1.0), n16 = cellsAt(1.6);
  check('flexibility x1.6 reaches more cells than x1.0', n16 > n10 * 1.15, n10 + ' -> ' + n16);
  pcc.setFlex(1.6); const plFlex = planner.createPlanner(); pcc.setFlex(1.0);
  check('planner built at x1.6 carries x1.6 limits', Math.abs(plFlex.limits()[0] - 1.6 * pcc.KMAX_BASE[0]) < 1e-9);
  check('planner built at x1.0 keeps x1.0 limits', Math.abs(pl.limits()[0] - pcc.KMAX_BASE[0]) < 1e-9);
  // the ensemble flags targets outside the population it was trained on
  const L = CR.learned.createLearned(weights, { targetTest });
  const simF = truth.createTruth(1); simF.reset(Q0);
  const mF = camera.senseMarkers(simF.markers3(), camSide, camTop);
  L.reset(Q0);
  check('learned flags a target beyond any reach', L.step(mF, [0.1, 0.0, 2.3], dt).ood);
  L.reset(Q0);
  check('learned does not flag the demo edge target (inside the trained range)', !L.step(mF, tEdge, dt).ood);
  check('training-population IK reaches the demo edge target', trainIK.solveIK(tEdge, [0, 0, 0, 0]).reachable);
  check('training-population IK reaches an interior target', trainIK.solveIK(tIn, [0, 0, 0, 0]).reachable);
  L.reset(Q0);
  check('learned does not flag an interior target', !L.step(mF, tIn, dt).ood);
}

console.log(failures === 0 ? '\nall sanity checks passed' : '\n' + failures + ' FAILURES');
process.exit(failures === 0 ? 0 : 1);
