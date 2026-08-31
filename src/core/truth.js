// "Truth" simulator: PCC kinematics plus the effects the idealized model
// ignores. This is a phenomenological model (not Cosserat rod mechanics);
// its job is to be wrong relative to the ideal PCC model in ways that are
// qualitatively realistic for tendon-driven continuum robots:
//
//   - actuator lag + rate limit on tendon displacements
//   - tendon backlash (play operator) -> deadband hysteresis
//   - load-dependent droop: curvature biased toward gravity's lateral
//     component at each segment's midpoint, scaled by payload (upright robot:
//     no sag while straight, more the further it leans)
//   - inter-segment coupling: proximal tendons disturb the distal segment
//   - optional slow tendon drift (creep + random walk)
//
// Controllers never read this state; they only see projected marker pixels.
(function (CR) {
  'use strict';
  const { pcc, m3, makeRng } = CR;

  const P = {
    tendonRadius: 0.05,      // tendon routing radius (normalized units)
    lagTau: 0.09,            // actuator first-order lag, seconds
    rateMaxK: 4.0,           // actuator rate limit, curvature units / s
    backlashK: 0.035,        // backlash half-width, curvature-equivalent
    coupling: 0.12,          // fraction of seg1 curvature leaking into seg2
    droopSelf: [0.05, 0.03], // always-on gravity sag per segment
    droopLoad: [0.52, 0.32], // extra sag per unit payload
    driftWalkK: 0.035,       // drift random walk, curvature-equiv / sqrt(s)
    driftCreepK: 0.025,      // drift creep, curvature-equiv / s
    driftMaxK: 0.30,         // drift bias cap, curvature-equivalent
  };

  const NSEG = pcc.NSEG;
  const BETA = [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];
  const G_WORLD = [0, 0, -1]; // upright robot: gravity along the axis, toward the base

  // Curvature vector -> 3 tendon displacements for segment i (linear map).
  function tendonsFromK(kx, ky, i) {
    const rL = P.tendonRadius * pcc.SEG_LEN[i];
    return BETA.map((b) => -rL * (kx * Math.cos(b) + ky * Math.sin(b)));
  }

  // Pseudo-inverse of the map above.
  function kFromTendons(t, i) {
    const rL = P.tendonRadius * pcc.SEG_LEN[i];
    let cx = 0, cy = 0;
    for (let j = 0; j < 3; j++) {
      cx += t[j] * Math.cos(BETA[j]);
      cy += t[j] * Math.sin(BETA[j]);
    }
    return [(-2 / (3 * rL)) * cx, (-2 / (3 * rL)) * cy];
  }

  // Static part of the truth model: base curvature per segment -> effective
  // curvature after coupling and gravity droop. Gravity is expressed in the
  // frame at each segment's midpoint; its component perpendicular to the
  // backbone there biases the curvature toward gravity. An upright straight
  // robot therefore does not sag; the further a segment leans, the more it
  // sags, and the payload scales that.
  function applyStatic(qBase, payload) {
    const q = [
      qBase[0], qBase[1],
      qBase[2] + P.coupling * qBase[0],
      qBase[3] + P.coupling * qBase[1],
    ];
    for (let i = 0; i < NSEG; i++) {
      const R = pcc.poseAt(q, i, 0.5 * pcc.SEG_LEN[i]).R; // local -> world at the midpoint
      // R^T G: gravity in the midpoint frame; keep the x-y (bending-plane) part
      const gx = R[0] * G_WORLD[0] + R[3] * G_WORLD[1] + R[6] * G_WORLD[2];
      const gy = R[1] * G_WORLD[0] + R[4] * G_WORLD[1] + R[7] * G_WORLD[2];
      const d = P.droopSelf[i] + payload * P.droopLoad[i];
      q[2 * i] += d * gx;
      q[2 * i + 1] += d * gy;
    }
    return pcc.clampQ(q);
  }

  function createTruth(seed) {
    const rng = makeRng(seed);
    const sim = {
      payload: 0,        // 0..1, ramped by the UI
      driftOn: false,
      qCmd: [0, 0, 0, 0],
      target: new Array(6).fill(0), // tendon targets (2 segments x 3 tendons)
      actual: new Array(6).fill(0), // after lag + rate limit
      play: new Array(6).fill(0),   // after backlash
      drift: new Array(6).fill(0),  // slow bias
      qEffCache: [0, 0, 0, 0],
      time: 0,
    };

    function updateTargets() {
      for (let i = 0; i < NSEG; i++) {
        const t = tendonsFromK(sim.qCmd[2 * i], sim.qCmd[2 * i + 1], i);
        for (let j = 0; j < 3; j++) sim.target[3 * i + j] = t[j];
      }
    }

    sim.setCommand = function (q) {
      sim.qCmd = pcc.clampQ(q);
      updateTargets();
    };

    sim.reset = function (q0) {
      sim.qCmd = pcc.clampQ(q0 || [0, 0, 0, 0]);
      updateTargets();
      for (let j = 0; j < 6; j++) {
        sim.actual[j] = sim.target[j];
        sim.play[j] = sim.target[j];
        sim.drift[j] = 0;
      }
      sim.time = 0;
      sim.step(0);
    };

    sim.step = function (dt) {
      sim.time += dt;
      const alpha = dt > 0 ? dt / (P.lagTau + dt) : 0;
      for (let i = 0; i < NSEG; i++) {
        const rL = P.tendonRadius * pcc.SEG_LEN[i];
        const rateMax = P.rateMaxK * rL * dt;
        const bl = P.backlashK * rL;
        for (let j = 0; j < 3; j++) {
          const idx = 3 * i + j;
          // first-order lag with rate limit
          let da = alpha * (sim.target[idx] - sim.actual[idx]);
          if (da > rateMax) da = rateMax;
          if (da < -rateMax) da = -rateMax;
          sim.actual[idx] += da;
          // backlash: play operator with half-width bl
          const gap = sim.actual[idx] - sim.play[idx];
          if (gap > bl) sim.play[idx] = sim.actual[idx] - bl;
          else if (gap < -bl) sim.play[idx] = sim.actual[idx] + bl;
          // drift: creep in a fixed per-tendon direction + random walk
          if (sim.driftOn && dt > 0) {
            const dir = (i + j) % 2 === 0 ? 1 : -1;
            let d = sim.drift[idx];
            d += dir * P.driftCreepK * rL * dt;
            d += P.driftWalkK * rL * Math.sqrt(dt) * rng.gauss();
            const cap = P.driftMaxK * rL;
            if (d > cap) d = cap;
            if (d < -cap) d = -cap;
            sim.drift[idx] = d;
          }
        }
      }
      // effective curvature from transmitted tendon displacements
      const qBase = new Array(4);
      for (let i = 0; i < NSEG; i++) {
        const t = [0, 1, 2].map((j) => sim.play[3 * i + j] + sim.drift[3 * i + j]);
        const k = kFromTendons(t, i);
        qBase[2 * i] = k[0];
        qBase[2 * i + 1] = k[1];
      }
      sim.qEffCache = applyStatic(qBase, sim.payload);
      return sim.qEffCache;
    };

    sim.qEff = () => sim.qEffCache;
    // transmitted tendon displacement (after lag, backlash, drift) and the
    // commanded one, per tendon index 3*i+j; negative = shortened (pulled)
    sim.tendonState = (idx) => ({ commanded: sim.target[idx], transmitted: sim.play[idx] + sim.drift[idx] });
    sim.markers3 = () => pcc.markers3(sim.qEffCache);
    sim.backbone = (n) => pcc.backbone(sim.qEffCache, n);

    sim.reset([0, 0, 0, 0]);
    return sim;
  }

  // Privileged pixel Jacobian of tip wrt command, evaluated through the
  // static truth map (coupling + droop) at the sim's current state. Used by
  // the training-time expert and by nothing at demo runtime.
  function truthJacobianPx(sim, cam) {
    const h = 1e-3;
    const J = [[0, 0, 0, 0], [0, 0, 0, 0]];
    // Operating point: the transmitted curvature (after backlash + drift),
    // not the raw command, so the linearization is taken where the plant
    // actually is.
    const qOp = new Array(4);
    for (let i = 0; i < NSEG; i++) {
      const t = [0, 1, 2].map((j) => sim.play[3 * i + j] + sim.drift[3 * i + j]);
      const k = kFromTendons(t, i);
      qOp[2 * i] = k[0];
      qOp[2 * i + 1] = k[1];
    }
    for (let c = 0; c < 4; c++) {
      const qp = qOp.slice(); qp[c] += h;
      const qm = qOp.slice(); qm[c] -= h;
      const pp = cam.project(pcc.tip3(applyStatic(qp, sim.payload)));
      const pm = cam.project(pcc.tip3(applyStatic(qm, sim.payload)));
      if (!pp || !pm) continue;
      J[0][c] = (pp[0] - pm[0]) / (2 * h);
      J[1][c] = (pp[1] - pm[1]) / (2 * h);
    }
    return J;
  }

  // Privileged 3x4 Jacobian of the tip position wrt command, through the
  // static truth map at the transmitted curvature. Training-time expert only.
  function truthJacobian3(sim) {
    const h = 1e-3;
    const J = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    const qOp = new Array(4);
    for (let i = 0; i < NSEG; i++) {
      const t = [0, 1, 2].map((j) => sim.play[3 * i + j] + sim.drift[3 * i + j]);
      const k = kFromTendons(t, i);
      qOp[2 * i] = k[0];
      qOp[2 * i + 1] = k[1];
    }
    for (let c = 0; c < 4; c++) {
      const qp = qOp.slice(); qp[c] += h;
      const qm = qOp.slice(); qm[c] -= h;
      const pp = pcc.tip3(applyStatic(qp, sim.payload));
      const pm = pcc.tip3(applyStatic(qm, sim.payload));
      for (let r = 0; r < 3; r++) J[r][c] = (pp[r] - pm[r]) / (2 * h);
    }
    return J;
  }

  CR.truth = { PARAMS: P, BETA, createTruth, truthJacobianPx, truthJacobian3, applyStatic };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
