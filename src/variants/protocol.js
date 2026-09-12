// The page's trial protocol as a reusable module, so that numbers computed in
// the browser are the same numbers train/eval.js writes into eval.json.
//
// Protocol (train/eval.js): 6 s window at 60 Hz; settle = tip error under 5 mm
// (0.05 length units) held for 0.8 s, reported as the band-entry time; steady
// state = mean error over the final second; a 2 s settle of the plant before
// the trial; seeds 31000 + 7 n; interior targets from configurations at up to
// 85 percent of the curvature limits, edge targets at 95 to 100 percent.
// Display convention: 1 length unit = 100 mm.
(function (CR) {
  'use strict';
  const { pcc, truth, ibvs, learned, planner, camera, v3, makeRng } = CR;

  const CONST = {
    W: 460, H: 345, DT: 1 / 60, TRIAL_S: 6, SETTLE_U: 0.05, SETTLE_HOLD: 0.8, MM: 100,
    TRIALS: 40, SEED_BASE: 31000, SEED_STEP: 7, PRE_STEPS: 120,
  };
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
      const k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i];
      q.push(k * Math.cos(a), k * Math.sin(a));
    }
    return q;
  }

  // Everything a trial needs that is expensive to build: cameras, the planner
  // and the training-population test. Build once per page or process.
  function makeEnv(weights, opts) {
    const o = opts || {};
    const W = o.W || CONST.W, H = o.H || CONST.H;
    const trainIK = o.trainIK || planner.createPlanner({ limitScale: 0.9 * pcc.FLEX_RANGE[1], seed: 13 });
    return {
      weights: weights || null,
      camSide: camera.sideCamera(W, H),
      camTop: camera.topCamera(W, H),
      planner: o.planner || planner.createPlanner(),
      trainIK,
      targetTest: (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable,
    };
  }

  function makeCtrl(kind, mode, env) {
    const base = kind === 'classical'
      ? ibvs.createClassical()
      : learned.createLearned(env.weights, { targetTest: env.targetTest });
    if (!base) return null;
    return mode === 'planned' ? planner.createTracked(base, env.planner, kind) : planner.createDirect(base, kind);
  }

  // One trial. job = { kind, mode, cond, targetSet, seed }. Returns
  // { settle, ss, oodFrac } or null when a marker left both cameras.
  function runTrial(job, env) {
    const { DT, TRIAL_S, SETTLE_U, SETTLE_HOLD, PRE_STEPS } = CONST;
    const rng = makeRng(job.seed);
    const sim = truth.createTruth(job.seed);
    sim.payload = job.cond.payload;
    sim.driftOn = job.cond.drift;
    const q0 = randomQ(rng, 0, 0.7);
    sim.reset(q0);
    for (let i = 0; i < PRE_STEPS; i++) sim.step(DT);
    const ctrl = makeCtrl(job.kind, job.mode, env);
    if (!ctrl) return null;
    ctrl.reset(q0);
    const target = pcc.tip3(job.targetSet === 'edge' ? randomQ(rng, 0.95, 1.0) : randomQ(rng, 0, 0.85));
    let bandEnter = null, settle = null, oodSteps = 0;
    const tail = [];
    const steps = Math.round(TRIAL_S / DT);
    for (let s = 0; s < steps; s++) {
      const t = s * DT;
      const markers = camera.senseMarkers(sim.markers3(), env.camSide, env.camTop);
      if (!markers) return null;
      const err = v3.norm(v3.sub(markers[3], target));
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
    return { settle, ss: tail.reduce((a, b) => a + b, 0) / tail.length, oodFrac: oodSteps / steps, target };
  }

  // The job list for a spec. spec = { trials, targetSets, conditions, controllers, modes }.
  function jobs(spec) {
    const s = spec || {};
    const trials = s.trials || CONST.TRIALS;
    const out = [];
    for (const targetSet of s.targetSets || ['interior', 'edge']) {
      for (const cond of s.conditions || CONDITIONS) {
        for (const kind of s.controllers || ['classical', 'learned']) {
          for (const mode of s.modes || ['direct', 'planned']) {
            for (let n = 0; n < trials; n++) {
              out.push({ kind, mode, cond, targetSet, seed: CONST.SEED_BASE + n * CONST.SEED_STEP, n });
            }
          }
        }
      }
    }
    return out;
  }

  // eval.js's median: the upper middle element.
  function median(a) {
    const s = a.slice().sort((x, y) => x - y);
    return s.length ? s[Math.floor(s.length / 2)] : NaN;
  }

  // Cells exactly as eval.js prints them for one (targetSet, cond, kind):
  // per mode: settled 'x/y', median settle 'n.nn s' or 'n/a', steady 'n.n mm';
  // plus the planned-learned OOD cell.
  function cellsFor(resultsByMode) {
    const cells = [];
    let ood = '–';
    for (const mode of ['direct', 'planned']) {
      const res = (resultsByMode[mode] || []).filter(Boolean);
      const settled = res.filter((r) => r.settle != null);
      cells.push(settled.length + '/' + res.length);
      cells.push(settled.length ? median(settled.map((r) => r.settle)).toFixed(2) + ' s' : 'n/a');
      cells.push(res.length ? (res.reduce((a, r) => a + r.ss, 0) / res.length * CONST.MM).toFixed(1) + ' mm' : '–');
      if (mode === 'planned' && resultsByMode.kind === 'learned') {
        ood = res.length ? (100 * res.reduce((a, r) => a + r.oodFrac, 0) / res.length).toFixed(0) + '% of steps' : '–';
      }
    }
    return { cells, ood };
  }

  // Group finished jobs into eval.json-shaped rows.
  function reduce(jobsDone) {
    const groups = new Map();
    for (const j of jobsDone) {
      const key = j.targetSet + '|' + j.cond.name + '|' + j.kind;
      if (!groups.has(key)) groups.set(key, { targets: j.targetSet, condition: j.cond.name, controller: j.kind, direct: [], planned: [], kind: j.kind });
      groups.get(key)[j.mode].push(j.result);
    }
    const rows = [];
    for (const g of groups.values()) {
      const c = cellsFor(g);
      rows.push({ targets: g.targets, condition: g.condition, controller: g.controller, cells: c.cells, ood: c.ood,
        n: { direct: g.direct.length, planned: g.planned.length } });
    }
    return rows;
  }

  CR.protocol = { CONST, CONDITIONS, randomQ, makeEnv, makeCtrl, runTrial, jobs, cellsFor, reduce, median };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
