// Plan-then-track layer (version 2).
//
// Version 1 closed the loop directly on the clicked pixel. That is a local
// law: with one camera the image target is a ray, the robot has four degrees
// of freedom for a two-dimensional error, and the pseudo-inverse picks the
// bending plane greedily. From some starting poses it commits to the wrong
// plane, hits the curvature limit and stalls with the tip short of the target.
// No local feedback law fixes a basin problem, so version 2 adds the standard
// remedy: a global plan on the model, tracked by the image loop.
//
//   1. Numerical inverse kinematics on the ideal PCC model: coarse search over
//      a sampled forward-kinematics table for configurations whose projected
//      tip lands near the target, pick the one nearest the current
//      configuration, refine with damped Gauss-Newton on the pixel residual
//      while staying inside the curvature limits.
//   2. A straight configuration-space path from the current configuration to
//      the solution, at a fixed fraction of the actuator rate limit.
//   3. The controller tracks the projected reference along that path with
//      feed-forward (the path velocity) plus its own feedback law on the
//      residual between the observed tip and the moving reference. When the
//      path ends the reference is the clicked target and the loop is the
//      plain version-1 law again.
//
// The plan is only as good as the ideal model. Under payload the reference
// path is not where the real tip goes, and the feedback term has to carry the
// difference; that cost is measured, not hidden (see train/eval.js).
(function (CR) {
  'use strict';
  const { pcc, ibvs } = CR;

  const TABLE_N = 8000;      // sampled configurations for the coarse search
  const CANDIDATE_PX = 14;   // coarse residual below which a sample is a candidate
  const GN_ITERS = 12;       // Gauss-Newton refinement steps
  const PATH_RATE = 0.6 * ibvs.RATE_MAX; // configuration speed along the plan

  function sampleQ(rng) {
    const q = [];
    for (let i = 0; i < pcc.NSEG; i++) {
      const a = rng() * 2 * Math.PI;
      const k = Math.sqrt(rng()) * pcc.KMAX[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    return q;
  }

  function buildTable(cam, seed) {
    const rng = CR.makeRng(seed || 11);
    const table = [];
    while (table.length < TABLE_N) {
      const q = sampleQ(rng);
      const p = cam.project(pcc.tip3(q));
      if (p) table.push({ q, px: [p[0], p[1]] });
    }
    return table;
  }

  function qDist(a, b) {
    let s = 0;
    for (let i = 0; i < 4; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
    return Math.sqrt(s);
  }

  // Damped Gauss-Newton on r(q) = project(tip(q)) - target, minimum-change
  // from the coarse candidate, curvature limits enforced by projection.
  function refine(q0, target, cam) {
    let q = q0.slice();
    for (let it = 0; it < GN_ITERS; it++) {
      const p = cam.project(pcc.tip3(q));
      if (!p) break;
      const r = [p[0] - target[0], p[1] - target[1]];
      if (Math.hypot(r[0], r[1]) < 0.05) break;
      const J = ibvs.idealJacobianPx(q, cam);
      const P = ibvs.dampedPinv(J);
      const next = new Array(4);
      for (let i = 0; i < 4; i++) next[i] = q[i] - (P[2 * i] * r[0] + P[2 * i + 1] * r[1]);
      q = pcc.clampQ(next);
    }
    return q;
  }

  function createPlanner(cam, opts) {
    const table = buildTable(cam, opts && opts.seed);

    // Global IK: nearest-in-configuration among the samples that reach the
    // target, refined; if nothing reaches it, the sample with the smallest
    // residual (closest reachable point, in the image).
    function solveIK(target, qNow) {
      let best = null, bestD = Infinity;
      let fallback = null, fallbackR = Infinity;
      for (const e of table) {
        const r = Math.hypot(e.px[0] - target[0], e.px[1] - target[1]);
        if (r < fallbackR) { fallbackR = r; fallback = e; }
        if (r < CANDIDATE_PX) {
          const d = qDist(e.q, qNow);
          if (d < bestD) { bestD = d; best = e; }
        }
      }
      const seed = best || fallback;
      const q = refine(seed.q, target, cam);
      const p = cam.project(pcc.tip3(q));
      const residual = p ? Math.hypot(p[0] - target[0], p[1] - target[1]) : Infinity;
      return { q, residual, reachable: !!best };
    }

    // A plan: configuration path q(t) from qStart to qGoal at PATH_RATE, and
    // the projected reference s_ref(t) with its feed-forward velocity.
    function plan(target, qStart) {
      const ik = solveIK(target, qStart);
      const dq = ik.q.map((v, i) => v - qStart[i]);
      const dist = Math.max(...dq.map(Math.abs));
      const T = dist / PATH_RATE;
      return {
        target: target.slice(),
        qStart: qStart.slice(),
        qGoal: ik.q,
        ikResidual: ik.residual,
        reachable: ik.reachable,
        T,
        // reference at time t since the plan started
        at(t) {
          if (t >= T || T <= 0) {
            return { qRef: ik.q, qDot: [0, 0, 0, 0], sRef: target.slice(), done: true };
          }
          const a = t / T;
          const qRef = qStart.map((v, i) => v + dq[i] * a);
          const qDot = dq.map((v) => v / T);
          const p = cam.project(pcc.tip3(qRef));
          return { qRef, qDot, sRef: p ? [p[0], p[1]] : target.slice(), done: false };
        },
      };
    }

    return { plan, solveIK, table };
  }

  // Wraps a version-1 controller (classical or learned) in the plan-then-
  // track loop. newTarget() makes the plan from the controller's current
  // configuration belief; step() advances along it. The wrapped controller's
  // own law is unchanged; it just tracks a moving reference with feed-forward.
  function createTracked(inner, plannerObj, kind) {
    let plan = null, tPlan = 0;
    const qNow = () => (kind === 'classical' ? inner.qBelief() : inner.qCmd());
    return {
      name: inner.name + '+plan',
      inner,
      sigmaWarn: inner.sigmaWarn,
      targetHull: inner.targetHull,
      reset(q0) { inner.reset(q0); plan = null; tPlan = 0; },
      qBelief: () => qNow(),
      qCmd: () => qNow(),
      plan: () => plan,
      newTarget(targetPx) {
        plan = plannerObj.plan(targetPx, qNow());
        tPlan = 0;
      },
      step(markersPx, targetPx, dt, w, h) {
        if (!plan || plan.target[0] !== targetPx[0] || plan.target[1] !== targetPx[1]) {
          this.newTarget(targetPx);
        }
        tPlan += dt;
        const ref = plan.at(tPlan);
        const tip = markersPx[3];
        if (kind === 'classical') {
          const qCmd = inner.stepTrack([tip[0], tip[1]], ref.sRef, ref.qDot, dt);
          return { qCmd, sRef: ref.sRef, tracking: !ref.done, plan };
        }
        const out = inner.stepTrack(markersPx, ref.sRef, ref.qDot, dt, w, h, targetPx);
        out.sRef = ref.sRef;
        out.tracking = !ref.done;
        out.plan = plan;
        return out;
      },
    };
  }

  CR.planner = { createPlanner, createTracked, PATH_RATE, CANDIDATE_PX };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
