// Hysteresis: identification and compensation of the tendon play (backlash).
//
// The truth sim transmits tendon displacements through a play operator: the
// first `2w` of any reversed motion goes nowhere. This module contains the
// controller-side answer, in two parts, both restricted to what a controller
// may legitimately know:
//
//   1. An identifier that measures the play width from the outside: it
//      commands a slow triangular sweep on one curvature channel and watches
//      the triangulated tip (the same two-camera sensing the controllers
//      use). After each command reversal the tip keeps moving the old way for
//      a while; the commanded distance covered before the tip turns around is
//      the dead distance. The dead distance also contains a lag contribution
//      that grows with sweep speed, so the sweep runs at two speeds and the
//      dead distance is extrapolated to zero speed; half the intercept is the
//      estimated half-width.
//
//   2. An inverse-play compensator (Kuhnen-style feed-forward): the commanded
//      tendon displacement is led by the estimated half-width in the current
//      direction of motion, so the plant's play band is crossed at command
//      time instead of at the tip's expense. It filters the commanded
//      tendons only; it reads nothing from the simulator.
//
// Everything here is deterministic under a fixed seed. The evaluation
// harness runs the same trial protocol as the page's trial table.
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, v3, makeRng } = CR;
  const plannerMod = CR.planner;

  const DT = 1 / 60;
  // The sweep bends in the y direction (curvature channel k1y): that is the
  // side camera's image-right, so the measurement is visible in the view the
  // page shows, and the tip's y coordinate is the natural scalar to watch.
  const SWEEP = {
    qBase: [0, 0.55, 0, 0.30],
    amp: 0.35,                 // sweep amplitude on k1y, curvature units
    speeds: [0.25, 0.75],      // curvature units / s; both far below the rate limits
    cyclesPerSpeed: 3,
    settleS: 1.6,              // let lag and play engage before measuring
    reactTol: 2e-4,            // tip movement (length units) that counts as "turned around"
  };

  // ---- inverse play compensator (controller side) ----
  // filter(idx, v): v led by w in the direction v is moving. Engagement state
  // per tendon; w is the half-width in curvature units, converted per segment
  // to tendon units with the same linear map the plant uses.
  function createCompensator(wK) {
    const prev = new Array(6).fill(null);
    const engage = new Array(6).fill(0);
    const comp = {
      wK,
      reset() { prev.fill(null); engage.fill(0); },
      filter(idx, v) {
        if (prev[idx] != null) {
          const d = v - prev[idx];
          if (d > 1e-12) engage[idx] = 1;
          else if (d < -1e-12) engage[idx] = -1;
        }
        prev[idx] = v;
        const seg = idx < 3 ? 0 : 1;
        const w = comp.wK * truth.PARAMS.tendonRadius * pcc.SEG_LEN[seg];
        return v + w * engage[idx];
      },
    };
    return comp;
  }

  // ---- lag identification from a small step, cameras only ----
  // Take up the play by moving the bend one way and holding, then step the
  // command a little further the same way and watch the triangulated tip.
  // A first-order lag makes the remaining distance shrink by the same ratio r
  // every frame; a straight-line fit of its logarithm gives r, and the time
  // constant follows from the discrete lag, tau = r dt / (1 - r).
  function identifyLag(seed) {
    const camSide = camera.sideCamera(460, 345), camTop = camera.topCamera(460, 345);
    const sim = truth.createTruth(seed || 4141);
    const q = SWEEP.qBase.slice();
    sim.reset(q);
    const tipY = () => { const m = camera.senseMarkers(sim.markers3(), camSide, camTop); return m ? m[3][1] : NaN; };
    // take up the play in the + direction, then hold
    for (let i = 0; i < 60; i++) { q[1] = SWEEP.qBase[1] + 0.1 * (i + 1) / 60; sim.setCommand(q); sim.step(DT); }
    for (let i = 0; i < 90; i++) { sim.setCommand(q); sim.step(DT); }
    q[1] += 0.05;
    const ys = [];
    for (let i = 0; i < 90; i++) { sim.setCommand(q); sim.step(DT); ys.push(tipY()); }
    const yEnd = ys.slice(-18).reduce((a, b) => a + b, 0) / 18;
    const e0 = Math.abs(yEnd - ys[0]);
    const pts = [];
    for (let n = 0; n < ys.length; n++) { const e = Math.abs(yEnd - ys[n]); if (e < 0.05 * e0) break; pts.push([n, Math.log(e)]); }
    const mx = pts.reduce((a, p) => a + p[0], 0) / pts.length, my = pts.reduce((a, p) => a + p[1], 0) / pts.length;
    const slope = pts.reduce((a, p) => a + (p[0] - mx) * (p[1] - my), 0) / pts.reduce((a, p) => a + (p[0] - mx) * (p[0] - mx), 0);
    const r = Math.exp(slope);
    return { tau: (r * DT) / (1 - r), r, n: pts.length };
  }

  // ---- inverse play followed by an inverse first-order lag (a lead) ----
  // u = p + tau * dp/dt on each tendon, p the inverse-play output. It undoes a
  // first-order lag of time constant tau exactly, up to the rate limit.
  function createLeadCompensator(wK, tau) {
    const play = createCompensator(wK);
    const prev = new Array(6).fill(null);
    return {
      filter(idx, v) {
        const p = play.filter(idx, v);
        const d = prev[idx] == null ? 0 : (p - prev[idx]) / DT;
        prev[idx] = p;
        return p + tau * d;
      },
    };
  }

  // ---- identification ----
  // Incremental so the page can animate it; runIdentification() drives it
  // headless for tests. trueWidthK is written into the private sim only; the
  // estimator itself never reads it.
  function createIdentifier(opts) {
    const o = opts || {};
    const W = o.W || 460, H = o.H || 345;
    const camSide = camera.sideCamera(W, H);
    const camTop = camera.topCamera(W, H);
    const sim = truth.createTruth(o.seed || 4141);
    if (o.trueWidthK != null) sim.backlashK = o.trueWidthK;
    sim.reset(SWEEP.qBase);

    // phases: per speed, a settle ramp then full triangles
    const phases = SWEEP.speeds.map((v) => ({
      v,
      T: SWEEP.settleS + SWEEP.cyclesPerSpeed * (4 * SWEEP.amp) / v,
      deads: [],
    }));
    let phaseIdx = 0, tPhase = 0;
    let kPrev = SWEEP.qBase[1];
    let mPrev = null, gSign = 0, gAccum = 0;
    let pending = null; // { k0, dirOld, mExtreme }
    const loop = [];    // [kCmd, tipDisplayComponent] ring for the loop chart
    const LOOP_MAX = 2600;

    // triangle around qBase[0]: rises first
    function kAt(t, v) {
      const tt = Math.max(0, t - SWEEP.settleS);
      const P4 = (4 * SWEEP.amp) / v;
      const ph = tt % P4;
      const a = ph * v;
      const tri = a < 2 * SWEEP.amp ? a - SWEEP.amp : 3 * SWEEP.amp - a;
      // settle ramp: move from base to the triangle's start (-amp) slowly
      if (t < SWEEP.settleS) {
        const f = t / SWEEP.settleS;
        return SWEEP.qBase[1] - SWEEP.amp * f;
      }
      return SWEEP.qBase[1] + tri;
    }

    const ident = {
      sim, done: false,
      result: null,
      loop,
      kNow: SWEEP.qBase[1],
      phase: () => phaseIdx,
      progress: 0,
      step(dt) {
        if (ident.done) return;
        const ph = phases[phaseIdx];
        tPhase += dt;
        const k = kAt(tPhase, ph.v);
        ident.kNow = k;
        const q = [SWEEP.qBase[0], k, SWEEP.qBase[2], SWEEP.qBase[3]];
        sim.setCommand(q);
        sim.step(dt);
        const sensed = camera.senseMarkers(sim.markers3(), camSide, camTop);
        if (sensed) {
          const m = sensed[3][1]; // tip y from the triangulated markers
          if (mPrev != null) {
            const dk = k - kPrev, dm = m - mPrev;
            if (Math.abs(dk) > 1e-9) gAccum += Math.sign(dk) * dm;
            gSign = Math.sign(gAccum) || 1;
            const g = gSign;
            const mm = g * m; // moves with the command once corrected for sign
            // reversal bookkeeping
            const dir = Math.sign(dk);
            if (dir !== 0 && ident.dirCmd != null && dir !== ident.dirCmd && tPhase > SWEEP.settleS + 0.2) {
              pending = { k0: k, dirOld: ident.dirCmd, mExtreme: mm, first: ph.deads.length === 0 && phaseIdx >= 0 && !ident.sawFirst };
              ident.sawFirst = true;
            }
            if (dir !== 0) ident.dirCmd = dir;
            if (pending) {
              // while the tip still follows the OLD direction, extend the extreme
              if (pending.dirOld > 0) pending.mExtreme = Math.max(pending.mExtreme, mm);
              else pending.mExtreme = Math.min(pending.mExtreme, mm);
              const turned = pending.dirOld > 0
                ? mm < pending.mExtreme - SWEEP.reactTol
                : mm > pending.mExtreme + SWEEP.reactTol;
              if (turned) {
                const dead = Math.abs(k - pending.k0);
                if (!pending.first) ph.deads.push(dead); // first reversal per run is the transient
                pending = null;
              }
            }
            loop.push([k, m]);
            if (loop.length > LOOP_MAX) loop.shift();
          }
          mPrev = m;
        }
        kPrev = k;
        const total = phases.reduce((a, p) => a + p.T, 0);
        const before = phases.slice(0, phaseIdx).reduce((a, p) => a + p.T, 0);
        ident.progress = Math.min(1, (before + tPhase) / total);
        if (tPhase >= ph.T) {
          phaseIdx++;
          tPhase = 0;
          pending = null;
          ident.sawFirst = false;
          loop.push(null); // pen-up between the two speeds' loops
          if (phaseIdx >= phases.length) finish();
        }
      },
    };
    ident.dirCmd = null;

    function finish() {
      const means = phases.map((p) =>
        p.deads.length ? p.deads.reduce((a, b) => a + b, 0) / p.deads.length : NaN);
      // dead(v) = 2 w_dir + c v  -> intercept from the two speeds
      const [v1, v2] = SWEEP.speeds;
      const [d1, d2] = means;
      const c = (d2 - d1) / (v2 - v1);
      const intercept = d1 - c * v1;
      // The dead band along a bending direction is set by the tendon that
      // carries that direction hardest: for a y sweep the two tendons at
      // +-120 degrees, with coefficient sin(120) = 0.866. The routing angles
      // are design constants (the same idealized-model knowledge the
      // controllers already use), so the directional width converts to the
      // per-tendon half-width by that geometric gain.
      const dirGain = Math.max(...truth.BETA.map((b) => Math.abs(Math.sin(b))));
      ident.result = {
        wDir: Math.max(0, intercept / 2),
        wEstK: Math.max(0, intercept / 2) * dirGain,
        dirGain,
        deadMeans: means,
        speeds: SWEEP.speeds.slice(),
        counts: phases.map((p) => p.deads.length),
        slope: c,
      };
      ident.done = true;
    }

    return ident;
  }

  function runIdentification(trueWidthK, seed) {
    const ident = createIdentifier({ trueWidthK, seed });
    let guard = 0;
    while (!ident.done && guard++ < 60 * 240) ident.step(DT);
    return ident.result;
  }

  // ---- paired evaluation ----
  // The page's trial protocol (6 s, settle = error under 5 mm held 0.8 s,
  // steady state = mean error over the final second), run headless on N
  // seeded targets under each compensation condition. Same targets, same
  // seeds, same disturbances across conditions; only the filter changes.
  const EVAL = { N: 20, TRIAL_S: 6, SETTLE_U: 0.05, SETTLE_HOLD: 0.8 };

  function sampleTargets(n, seed) {
    const rng = makeRng(seed || 77);
    const targets = [];
    while (targets.length < n) {
      const q = [];
      for (let i = 0; i < pcc.NSEG; i++) {
        const a = rng() * 2 * Math.PI;
        const k = Math.sqrt(rng()) * 0.85 * pcc.KMAX[i];
        q.push(k * Math.cos(a), k * Math.sin(a));
      }
      targets.push(pcc.tip3(q));
    }
    return targets;
  }

  // One trial; cond.wK = null (off) or half-width for the compensator.
  function runTrial(target, cond, trueWidthK, seed, planner, W, H) {
    const camSide = camera.sideCamera(W || 460, H || 345);
    const camTop = camera.topCamera(W || 460, H || 345);
    const sim = truth.createTruth(seed);
    if (trueWidthK != null) sim.backlashK = trueWidthK;
    const Q0 = [0.5, 0.1, -0.35, 0.3];
    sim.reset(Q0);
    const ctrl = plannerMod.createTracked(ibvs.createClassical(), planner, 'classical');
    ctrl.reset(Q0);
    if (cond.wK != null) {
      const comp = createCompensator(cond.wK);
      sim.tendonFilter = comp.filter;
    }
    ctrl.newTarget(target);
    let bandEnter = null, settle = null;
    const tail = [];
    const steps = Math.round(EVAL.TRIAL_S / DT);
    for (let s = 0; s < steps; s++) {
      const t = (s + 1) * DT;
      const markers = camera.senseMarkers(sim.markers3(), camSide, camTop);
      if (!markers) continue;
      const err = v3.norm(v3.sub(markers[3], target));
      const out = ctrl.step(markers, target, DT);
      sim.setCommand(out.qCmd);
      sim.step(DT);
      if (err < EVAL.SETTLE_U) {
        if (bandEnter == null) bandEnter = t;
        if (settle == null && t - bandEnter >= EVAL.SETTLE_HOLD) settle = bandEnter;
      } else bandEnter = null;
      if (t > EVAL.TRIAL_S - 1) tail.push(err);
    }
    const steady = tail.length ? tail.reduce((a, b) => a + b, 0) / tail.length : NaN;
    return { settle, steady };
  }

  // conds: [{label, wK|null}]. Returns one row per condition with the
  // page's three numbers. onProgress(iDone, iTotal) is optional.
  function runPairedEval(conds, trueWidthK, opts) {
    const o = opts || {};
    const planner = o.planner || plannerMod.createPlanner();
    const targets = sampleTargets(o.N || EVAL.N, o.targetSeed);
    const total = conds.length * targets.length;
    let done = 0;
    const rows = conds.map((c) => ({ label: c.label, wK: c.wK, settled: 0, settles: [], steadies: [] }));
    for (let ti = 0; ti < targets.length; ti++) {
      for (let ci = 0; ci < conds.length; ci++) {
        const r = runTrial(targets[ti], conds[ci], trueWidthK, 3000 + ti, planner, o.W, o.H);
        const row = rows[ci];
        if (r.settle != null) { row.settled++; row.settles.push(r.settle); }
        if (!Number.isNaN(r.steady)) row.steadies.push(r.steady);
        done++;
        if (o.onProgress) o.onProgress(done, total);
      }
    }
    for (const row of rows) {
      row.n = targets.length;
      row.settleMedian = median(row.settles);
      row.steadyMean = row.steadies.length ? row.steadies.reduce((a, b) => a + b, 0) / row.steadies.length : NaN;
    }
    return rows;
  }

  // ---- the loop, open loop ----
  // One run of the identifier's triangle sweep at one of its two speeds, with
  // no feedback at all: the command goes straight to the simulated tendons.
  // Returns the loop the cameras see (commanded bend of segment one against
  // the triangulated tip's sideways position, pen-up between cycles is not
  // needed because the sweep is continuous), the ideal model's curve for the
  // same commands, and the loop's mean width: the gap between the rising and
  // the falling pass at the same command, averaged over command bins. With
  // compW a number, the inverse-play filter at that half-width runs on the
  // commanded tendons; with compW a function, it is used as the filter.
  function sweepLoop(trueWidthK, compW, speedIdx, seed) {
    const ident = createIdentifier({ trueWidthK, seed: seed || 4141 });
    const camSide = camera.sideCamera(460, 345), camTop = camera.topCamera(460, 345);
    if (typeof compW === 'function') ident.sim.tendonFilter = compW;
    else if (compW != null) ident.sim.tendonFilter = createCompensator(compW).filter;
    const points = [], ideal = [];
    const up = new Map(), dn = new Map();
    let kPrev = null, guard = 0, t = 0;
    const tSkip = SWEEP.settleS + 1.0; // the first ramp and half triangle are transient
    while (!ident.done && guard++ < 60 * 240) {
      const ph = ident.phase();
      ident.step(DT);
      if (ph !== speedIdx) { kPrev = null; t = 0; continue; }
      t += DT;
      const k = ident.kNow;
      const sensed = camera.senseMarkers(ident.sim.markers3(), camSide, camTop);
      if (!sensed) continue;
      const y = sensed[3][1];
      if (t > tSkip) {
        points.push([k, y]);
        ideal.push([k, pcc.tip3([SWEEP.qBase[0], k, SWEEP.qBase[2], SWEEP.qBase[3]])[1]]);
        if (kPrev != null) {
          const d = k - kPrev;
          const bin = Math.round(k * 50);
          const m = d > 0 ? up : d < 0 ? dn : null;
          if (m) { if (!m.has(bin)) m.set(bin, []); m.get(bin).push(y); }
        }
      }
      kPrev = k;
    }
    const mean = (a) => a.reduce((x, z) => x + z, 0) / a.length;
    const gaps = [];
    for (const [bin, ys] of up) if (dn.has(bin)) gaps.push(Math.abs(mean(ys) - mean(dn.get(bin))));
    return { points, ideal, gapMean: gaps.length ? mean(gaps) : NaN, bins: gaps.length, speed: SWEEP.speeds[speedIdx] };
  }

  function median(a) {
    if (!a.length) return NaN;
    const s = a.slice().sort((x, y) => x - y);
    const m = s.length >> 1;
    return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
  }

  CR.hyst = { SWEEP, EVAL, createCompensator, createIdentifier, runIdentification,
    sampleTargets, runTrial, runPairedEval, sweepLoop, createLeadCompensator, identifyLag };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
