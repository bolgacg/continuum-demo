// Plan-then-track layer.
//
// The feedback laws are local: they descend the tip error from wherever the
// robot is. With four degrees of freedom for a three-dimensional target and
// curvature limits, a local law can still commit to a configuration from
// which the target is not reachable within the limits, and stall. Whether
// that happens often enough to matter with the 3D task is measured in
// train/eval.js; the page reports the measurement. The remedy, when needed,
// is the standard one: a global plan on the model, tracked by the loop.
//
//   1. Numerical inverse kinematics on the ideal PCC model: coarse search over
//      a sampled forward-kinematics table for configurations whose tip lands
//      near the target, pick the one nearest the current configuration,
//      refine with damped Gauss-Newton inside the curvature limits.
//   2. A straight configuration-space path from the current configuration to
//      the solution, at a fixed fraction of the actuator rate limit.
//   3. The controller tracks the reference point along that path with
//      feed-forward (the path velocity) plus its own feedback law on the
//      residual between the sensed tip and the moving reference. When the
//      path ends the reference is the target and the loop is the direct law.
//
// The plan is only as good as the ideal model. Under payload the reference
// path is not where the real tip goes, and the feedback term carries the
// difference; that cost is measured, not hidden.
(function (CR) {
  'use strict';
  const { pcc, ibvs, v3 } = CR;

  const TABLE_N = 12000;     // sampled configurations for the coarse search
  const CANDIDATE_R = 0.12;  // coarse residual (length units) below which a sample is a candidate
  const REACH_TOL = 0.02;    // refined residual (2 mm) below which the target counts as reachable; well inside the 5 mm settle band
  const GN_ITERS = 30;       // Gauss-Newton refinement steps
  const PATH_RATE = 0.6 * ibvs.RATE_MAX; // configuration speed along the plan

  function sampleQ(rng, limits) {
    const q = [];
    for (let i = 0; i < pcc.NSEG; i++) {
      const a = rng() * 2 * Math.PI;
      const k = Math.sqrt(rng()) * limits[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    return q;
  }

  function buildTable(seed, limits) {
    const rng = CR.makeRng(seed || 11);
    const table = [];
    for (let n = 0; n < TABLE_N; n++) {
      const q = sampleQ(rng, limits);
      table.push({ q, p: pcc.tip3(q) });
    }
    return table;
  }

  // clamp each segment's curvature magnitude to absolute limits
  function clampTo(q, limits) {
    const out = q.slice();
    for (let i = 0; i < pcc.NSEG; i++) {
      const k = Math.hypot(out[2 * i], out[2 * i + 1]);
      if (k > limits[i]) { const f = limits[i] / k; out[2 * i] *= f; out[2 * i + 1] *= f; }
    }
    return out;
  }

  function qDist(a, b) {
    let s = 0;
    for (let i = 0; i < 4; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
    return Math.sqrt(s);
  }

  // Damped Gauss-Newton on r(q) = tip(q) - target, minimum-change from the
  // coarse candidate, curvature limits enforced by projection.
  function refine(q0, target, limits) {
    let q = q0.slice();
    for (let it = 0; it < GN_ITERS; it++) {
      const r = v3.sub(pcc.tip3(q), target);
      if (v3.norm(r) < 1e-5) break;
      const J = ibvs.idealJacobian3(q);
      const P = ibvs.dampedPinv(J);
      const next = new Array(4);
      for (let i = 0; i < 4; i++) {
        next[i] = q[i] - (P[3 * i] * r[0] + P[3 * i + 1] * r[1] + P[3 * i + 2] * r[2]);
      }
      q = clampTo(next, limits);
    }
    return q;
  }

  // The planner captures ABSOLUTE curvature limits when built (or rebuilt):
  // opts.limitScale times the mechanism's current limits. The app rebuilds its
  // planner when the flexibility changes; the ensemble's training-population
  // test is a second planner built once, at the limits the training covered,
  // and never rebuilt.
  function createPlanner(opts) {
    const limitScale = (opts && opts.limitScale) || 1;
    const seed = opts && opts.seed;
    let limits = pcc.KMAX.map((k) => k * limitScale);
    let table = buildTable(seed, limits);
    function rebuild() {
      limits = pcc.KMAX.map((k) => k * limitScale);
      table = buildTable(seed, limits);
    }

    // Global IK: nearest-in-configuration among the samples that reach the
    // target, refined; if nothing reaches it, the sample with the smallest
    // residual (closest reachable point).
    function solveIK(target, qNow) {
      let best = null, bestD = Infinity;
      let fallback = null, fallbackR = Infinity;
      for (const e of table) {
        const r = v3.norm(v3.sub(e.p, target));
        if (r < fallbackR) { fallbackR = r; fallback = e; }
        if (r < CANDIDATE_R) {
          const d = qDist(e.q, qNow);
          if (d < bestD) { bestD = d; best = e; }
        }
      }
      const cand = best || fallback;
      const q = refine(cand.q, target, limits);
      const residual = v3.norm(v3.sub(pcc.tip3(q), target));
      return { q, residual, reachable: residual < REACH_TOL };
    }

    // A plan: configuration path q(t) from qStart to qGoal at PATH_RATE, and
    // the reference point s_ref(t) with its feed-forward velocity.
    function plan(target, qStart) {
      const ik = solveIK(target, qStart);
      const dq = ik.q.map((v, i) => v - qStart[i]);
      const dist = Math.max(...dq.map(Math.abs));
      const T = dist / PATH_RATE;
      return {
        target: target.slice(),
        qStart: qStart.slice(),
        qGoal: ik.q,
        goalPoint: pcc.tip3(ik.q),
        ikResidual: ik.residual,
        reachable: ik.reachable,
        T,
        at(t) {
          if (t >= T || T <= 0) {
            return { qRef: ik.q, qDot: [0, 0, 0, 0], sRef: target.slice(), done: true };
          }
          const a = t / T;
          const qRef = qStart.map((v, i) => v + dq[i] * a);
          const qDot = dq.map((v) => v / T);
          return { qRef, qDot, sRef: pcc.tip3(qRef), done: false };
        },
      };
    }

    return { plan, solveIK, rebuild, limitScale, limits: () => limits.slice(), table: () => table };
  }

  // Wraps a controller (classical or learned) in the plan-then-track loop.
  // newTarget() makes the plan from the controller's current configuration
  // belief; step() advances along it. The wrapped controller's own law is
  // unchanged; it tracks a moving reference with feed-forward.
  function createTracked(inner, plannerObj, kind) {
    let plan = null, tPlan = 0;
    const qNow = () => (kind === 'classical' ? inner.qBelief() : inner.qCmd());
    return {
      name: inner.name + '+plan',
      inner,
      sigmaWarn: inner.sigmaWarn,
      reset(q0) { inner.reset(q0); plan = null; tPlan = 0; },
      qBelief: () => qNow(),
      qCmd: () => qNow(),
      plan: () => plan,
      newTarget(target3) {
        plan = plannerObj.plan(target3, qNow());
        tPlan = 0;
      },
      step(markers3, target3, dt) {
        if (!plan || plan.target[0] !== target3[0] || plan.target[1] !== target3[1] || plan.target[2] !== target3[2]) {
          this.newTarget(target3);
        }
        tPlan += dt;
        const ref = plan.at(tPlan);
        if (kind === 'classical') {
          const qCmd = inner.stepTrack(markers3[3], ref.sRef, ref.qDot, dt);
          return { qCmd, sRef: ref.sRef, tracking: !ref.done, plan };
        }
        const out = inner.stepTrack(markers3, ref.sRef, ref.qDot, dt, target3);
        out.sRef = ref.sRef;
        out.tracking = !ref.done;
        out.plan = plan;
        return out;
      },
    };
  }

  // Direct wrapper with the same interface (no plan), so the app and the
  // evaluation can switch between the two without special cases.
  function createDirect(inner, kind) {
    return {
      name: inner.name,
      inner,
      sigmaWarn: inner.sigmaWarn,
      reset(q0) { inner.reset(q0); },
      qBelief: () => (kind === 'classical' ? inner.qBelief() : inner.qCmd()),
      qCmd: () => (kind === 'classical' ? inner.qBelief() : inner.qCmd()),
      plan: () => null,
      newTarget() {},
      step(markers3, target3, dt) {
        if (kind === 'classical') {
          return { qCmd: inner.step(markers3[3], target3, dt), sRef: target3, tracking: false, plan: null };
        }
        const out = inner.step(markers3, target3, dt);
        out.sRef = target3; out.tracking = false; out.plan = null;
        return out;
      },
    };
  }

  CR.planner = { createPlanner, createTracked, createDirect, PATH_RATE, CANDIDATE_R, REACH_TOL };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
