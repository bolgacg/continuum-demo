// ---- src/core/math3.js ----
// Minimal 3D math: vectors as [x,y,z], 3x3 matrices as row-major arrays of 9.
(function (CR) {
  'use strict';

  const v3 = {
    add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
    sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
    scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
    dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
    cross: (a, b) => [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ],
    norm: (a) => Math.hypot(a[0], a[1], a[2]),
    normalize(a) {
      const n = Math.hypot(a[0], a[1], a[2]) || 1;
      return [a[0] / n, a[1] / n, a[2] / n];
    },
  };

  const m3 = {
    ident: () => [1, 0, 0, 0, 1, 0, 0, 0, 1],
    mul(a, b) {
      const r = new Array(9);
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          r[3 * i + j] =
            a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
        }
      }
      return r;
    },
    mulVec: (m, v) => [
      m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
      m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
      m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    ],
    rotY(t) {
      const c = Math.cos(t), s = Math.sin(t);
      return [c, 0, s, 0, 1, 0, -s, 0, c];
    },
    rotZ(t) {
      const c = Math.cos(t), s = Math.sin(t);
      return [c, -s, 0, s, c, 0, 0, 0, 1];
    },
  };

  // Deterministic RNG (mulberry32) so runs are reproducible and the two
  // side-by-side sims can share one disturbance realization.
  function makeRng(seed) {
    let a = seed >>> 0;
    const next = function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    next.gauss = function () {
      // Box-Muller
      let u = 0, v = 0;
      while (u === 0) u = next();
      while (v === 0) v = next();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    };
    return next;
  }

  CR.v3 = v3;
  CR.m3 = m3;
  CR.makeRng = makeRng;
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/pcc.js ----
// Piecewise constant curvature (PCC) kinematics for a 2-segment continuum robot.
//
// Configuration q = [kx1, ky1, kx2, ky2]: each segment bends with a curvature
// vector (kx, ky); magnitude is the curvature, direction is the bending plane.
// The robot's base frame has +z along the undeformed backbone. The robot
// stands upright: +z is world up and gravity is -z, along the axis. Units are
// normalized (segment lengths ~1); the demo does not claim a physical scale.
(function (CR) {
  'use strict';
  const { m3 } = CR;

  const SEG_LEN = [1.0, 0.8];
  const NSEG = 2;
  const KMAX_BASE = [2.2, 2.6]; // max curvature magnitude per segment at flexibility 1
  const KMAX = KMAX_BASE.slice(); // live limits; scaled by setFlex (same array object everywhere)
  // Flexibility scales the curvature limits of the mechanism. It changes what
  // the robot can reach, not what any controller knows; the planner, envelope
  // and grid are rebuilt by the app when it changes.
  function setFlex(f) {
    for (let i = 0; i < NSEG; i++) KMAX[i] = KMAX_BASE[i] * f;
    return f;
  }
  const flex = () => KMAX[0] / KMAX_BASE[0];
  const FLEX_RANGE = [0.7, 1.8]; // slider range; the ensemble is trained across it

  // Pose (position + rotation) at arc length s along one segment with
  // curvature vector (kx, ky), in the segment's base frame.
  function segPose(kx, ky, s) {
    const k = Math.hypot(kx, ky);
    if (k < 1e-9) {
      return { p: [0, 0, s], R: m3.ident() };
    }
    const phi = Math.atan2(ky, kx);
    const th = k * s;
    const a = (1 - Math.cos(th)) / k;
    const b = Math.sin(th) / k;
    const p = [Math.cos(phi) * a, Math.sin(phi) * a, b];
    const R = m3.mul(m3.rotZ(phi), m3.mul(m3.rotY(th), m3.rotZ(-phi)));
    return { p, R };
  }

  function composePose(base, local) {
    return {
      p: CR.v3.add(base.p, m3.mulVec(base.R, local.p)),
      R: m3.mul(base.R, local.R),
    };
  }

  // World-frame pose at arc length s (0..SEG_LEN[i]) along segment i.
  function poseAt(q, seg, s) {
    let pose = { p: [0, 0, 0], R: m3.ident() };
    for (let i = 0; i < seg; i++) {
      pose = composePose(pose, segPose(q[2 * i], q[2 * i + 1], SEG_LEN[i]));
    }
    return composePose(pose, segPose(q[2 * seg], q[2 * seg + 1], s));
  }

  function tip3(q) {
    return poseAt(q, NSEG - 1, SEG_LEN[NSEG - 1]).p;
  }

  // Backbone sample points for rendering, base to tip.
  function backbone(q, nPerSeg) {
    const pts = [];
    for (let i = 0; i < NSEG; i++) {
      for (let j = 0; j <= nPerSeg; j++) {
        pts.push(poseAt(q, i, (j / nPerSeg) * SEG_LEN[i]).p);
      }
    }
    return pts;
  }

  // Marker positions: mid segment 1, end segment 1, mid segment 2, tip.
  // These are the only "sensor readings" either controller gets (as pixels).
  function markers3(q) {
    return [
      poseAt(q, 0, 0.5 * SEG_LEN[0]).p,
      poseAt(q, 0, SEG_LEN[0]).p,
      poseAt(q, 1, 0.5 * SEG_LEN[1]).p,
      poseAt(q, 1, SEG_LEN[1]).p,
    ];
  }

  function clampQ(q, scale) {
    const out = q.slice();
    const sc = scale || 1;
    for (let i = 0; i < NSEG; i++) {
      const kx = out[2 * i], ky = out[2 * i + 1];
      const k = Math.hypot(kx, ky);
      if (k > KMAX[i] * sc) {
        const f = (KMAX[i] * sc) / k;
        out[2 * i] = kx * f;
        out[2 * i + 1] = ky * f;
      }
    }
    return out;
  }

  CR.pcc = { SEG_LEN, NSEG, KMAX, KMAX_BASE, FLEX_RANGE, setFlex, flex, NQ: 2 * NSEG, segPose, poseAt, tip3, backbone, markers3, clampQ };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/camera.js ----
// Pinhole cameras: fixed poses, perspective projection world -> pixels, the
// inverse ray for a pixel, and two-view triangulation.
//
// Version 3 sensing: two fixed, calibrated cameras (side and top) observe the
// four markers; each marker is triangulated to a 3D point and that estimate
// is what the controllers get. A third, orbiting camera is the human's
// inspector view; it is not a sensor.
(function (CR) {
  'use strict';
  const { v3, m3 } = CR;

  // Workspace center every camera looks at (on the robot's axis), orbit radius.
  // World frame: the robot stands on the origin with its axis along +z (up).
  const CENTER = [0, 0, 0.85];
  const DIST = 3.2;
  const EL_SIDE = Math.asin(0.2 / DIST); // side sensor sits 0.2 above the center

  function lookAtRotation(pos, target, up) {
    // Camera frame: +z forward (into the scene), +x right, +y down (image v).
    const fwd = v3.normalize(v3.sub(target, pos));
    const right = v3.normalize(v3.cross(fwd, up));
    const down = v3.cross(fwd, right);
    // Rows of R map world vectors into camera coords.
    return [
      right[0], right[1], right[2],
      down[0], down[1], down[2],
      fwd[0], fwd[1], fwd[2],
    ];
  }

  function makeCamera(opts) {
    const { pos, target, up, f, w, h } = opts;
    const R = lookAtRotation(pos, target, up);
    return {
      pos, target, up, f, w, h, R,
      // Returns [u, v, zCam] or null if the point is behind the camera.
      project(p) {
        const d = v3.sub(p, pos);
        const c = m3.mulVec(R, d);
        if (c[2] < 1e-4) return null;
        return [w / 2 + (f * c[0]) / c[2], h / 2 + (f * c[1]) / c[2], c[2]];
      },
      // Camera-space depth of a world point (for painter's sorting).
      depth(p) {
        const d = v3.sub(p, pos);
        return R[6] * d[0] + R[7] * d[1] + R[8] * d[2];
      },
      // Unit world direction of the viewing ray through pixel (u, v). A pixel
      // is this ray, not a point: one camera does not observe depth.
      rayDir(u, v) {
        const c = [(u - w / 2) / f, (v - h / 2) / f, 1];
        return v3.normalize([
          R[0] * c[0] + R[3] * c[1] + R[6] * c[2],
          R[1] * c[0] + R[4] * c[1] + R[7] * c[2],
          R[2] * c[0] + R[5] * c[1] + R[8] * c[2],
        ]);
      },
    };
  }

  // Orbit camera on a sphere around CENTER: azimuth about the axis (+z),
  // elevation from the horizontal plane. The up vector is the sphere's tangent
  // toward higher elevation, so it stays perpendicular to the view direction
  // everywhere, including straight down (no gimbal degeneracy).
  function orbitCamera(az, el, w, h, dist) {
    const d = dist || DIST;
    const ce = Math.cos(el), se = Math.sin(el);
    const dir = [ce * Math.cos(az), ce * Math.sin(az), se];
    const up = [-se * Math.cos(az), -se * Math.sin(az), ce];
    return makeCamera({ pos: v3.add(CENTER, v3.scale(dir, d)), target: CENTER, up, f: 0.62 * w, w, h });
  }

  // The two sensors. Side: from +x, nearly level with the workspace centre
  // (image-up is the axis, image-right is +y). Top: straight down the axis
  // from above (image-right +y, image-down +x, the side sensor's side).
  const PRESETS = {
    side: { az: 0, el: EL_SIDE },
    top: { az: 0, el: Math.PI / 2 - 1e-3 },
    iso: { az: 0.6, el: 0.5 },
  };
  function sideCamera(w, h) { return orbitCamera(PRESETS.side.az, PRESETS.side.el, w, h); }
  function topCamera(w, h) { return orbitCamera(PRESETS.top.az, PRESETS.top.el, w, h); }

  // Two-view triangulation: midpoint of the closest points of the two viewing
  // rays. Exact when the pixels are exact, which in this simulation they are;
  // the cameras add no noise, and the page says so.
  function triangulate(camA, pxA, camB, pxB) {
    const a = camA.pos, b = camB.pos;
    const dA = camA.rayDir(pxA[0], pxA[1]), dB = camB.rayDir(pxB[0], pxB[1]);
    const w0 = v3.sub(a, b);
    const B = v3.dot(dA, dB), D = v3.dot(dA, w0), E = v3.dot(dB, w0);
    const den = 1 - B * B;
    if (den < 1e-9) return null; // parallel rays
    const s = (B * E - D) / den;
    const t = (E - B * D) / den;
    const pA = v3.add(a, v3.scale(dA, s));
    const pB = v3.add(b, v3.scale(dB, t));
    return v3.scale(v3.add(pA, pB), 0.5);
  }

  // Sensing layer: project the markers into both sensors and triangulate.
  // Returns null if any marker is behind a camera.
  function senseMarkers(markers3, camSide, camTop) {
    const out = [];
    for (const m of markers3) {
      const pa = camSide.project(m), pb = camTop.project(m);
      if (!pa || !pb) return null;
      const p = triangulate(camSide, pa, camTop, pb);
      if (!p) return null;
      out.push(p);
    }
    return out;
  }

  // Ray-plane intersection with the height plane z = h (horizontal, since the
  // axis is up). Returns the point, or null when the ray is within ~3 degrees
  // of parallel to the plane.
  function rayPlaneZ(origin, dir, h) {
    if (Math.abs(dir[2]) < 0.05) return null;
    const t = (h - origin[2]) / dir[2];
    if (t <= 0) return null;
    return v3.add(origin, v3.scale(dir, t));
  }

  CR.camera = { CENTER, DIST, PRESETS, makeCamera, orbitCamera, sideCamera, topCamera,
    triangulate, senseMarkers, rayPlaneZ };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/truth.js ----
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
      backlashK: null,   // per-sim backlash half-width override (curvature units); null = P.backlashK
      tendonFilter: null, // optional controller-side filter(idx, tendonCmd) -> tendonCmd, e.g. inverse play
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
        for (let j = 0; j < 3; j++) {
          const idx = 3 * i + j;
          sim.target[idx] = sim.tendonFilter ? sim.tendonFilter(idx, t[j]) : t[j];
        }
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
        const bl = (sim.backlashK != null ? sim.backlashK : P.backlashK) * rL;
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

// ---- src/core/ibvs.js ----
// Classical resolved-rate control on the triangulated tip (version 3).
//
// The controller keeps an internal belief of the configuration (the commands
// it has integrated), differentiates the *idealized* PCC model there for the
// 3x4 Jacobian of tip position with respect to curvature, and does damped
// least squares descent on the 3D tip error. It never reads the truth sim's
// state: only the triangulated marker positions the sensing layer gives it.
// The gap between its ideal model and the truth sim is what the demo is about.
//
// Version 1 did the same thing on a 2x4 pixel Jacobian with a single camera,
// which made the task a line, not a point.
(function (CR) {
  'use strict';
  const { pcc } = CR;

  const GAIN = 2.2;      // error feedback gain, 1/s
  const RATE_MAX = 3.0;  // command rate clamp, curvature units / s
  const DAMP_FRAC = 0.02;

  // Jacobian of the ideal PCC tip position wrt q, central differences (3x4).
  function idealJacobian3(q) {
    const h = 1e-3;
    const J = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (let c = 0; c < 4; c++) {
      const qp = q.slice(); qp[c] += h;
      const qm = q.slice(); qm[c] -= h;
      const pp = pcc.tip3(qp), pm = pcc.tip3(qm);
      for (let r = 0; r < 3; r++) J[r][c] = (pp[r] - pm[r]) / (2 * h);
    }
    return J;
  }

  function inv3(m) {
    const [a, b, c, d, e, f, g, h, i] = m;
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const det = a * A + b * B + c * C;
    const s = 1 / det;
    return [
      A * s, -(b * i - c * h) * s, (b * f - c * e) * s,
      B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
      C * s, -(a * h - b * g) * s, (a * e - b * d) * s,
    ];
  }

  // Damped pseudo-inverse of an m x 4 Jacobian (m = 2 or 3):
  // P = J^T (J J^T + mu I)^-1, returned row-major as 4 rows of m.
  function dampedPinv(J) {
    const m = J.length;
    const M = new Array(m * m).fill(0);
    let tr = 0;
    for (let r = 0; r < m; r++) {
      for (let s = 0; s < m; s++) {
        let v = 0;
        for (let i = 0; i < 4; i++) v += J[r][i] * J[s][i];
        M[r * m + s] = v;
      }
      tr += M[r * m + r];
    }
    const mu = DAMP_FRAC * tr / m + 1e-6;
    for (let r = 0; r < m; r++) M[r * m + r] += mu;
    let Mi;
    if (m === 3) Mi = inv3(M);
    else {
      const det = M[0] * M[3] - M[1] * M[2];
      Mi = [M[3] / det, -M[1] / det, -M[2] / det, M[0] / det];
    }
    const P = new Array(4 * m);
    for (let i = 0; i < 4; i++) {
      for (let s = 0; s < m; s++) {
        let v = 0;
        for (let r = 0; r < m; r++) v += J[r][i] * Mi[r * m + s];
        P[i * m + s] = v;
      }
    }
    return P;
  }

  // v = -gain * P e   (damped least squares descent on the error)
  function dlsVelocity(J, e, gain) {
    const m = J.length;
    const P = dampedPinv(J);
    const v = new Array(4);
    for (let i = 0; i < 4; i++) {
      let s = 0;
      for (let k = 0; k < m; k++) s += P[i * m + k] * e[k];
      v[i] = -gain * s;
    }
    return v;
  }

  function clampRate(v, rateMax) {
    const out = v.slice();
    for (let i = 0; i < out.length; i++) {
      if (out[i] > rateMax) out[i] = rateMax;
      if (out[i] < -rateMax) out[i] = -rateMax;
    }
    return out;
  }

  function createClassical() {
    let qBelief = [0, 0, 0, 0];
    return {
      name: 'classical',
      reset(q0) { qBelief = (q0 || [0, 0, 0, 0]).slice(); },
      qBelief: () => qBelief.slice(),
      // Direct law: feedback on the target, no feed-forward.
      step(tip3, target3, dt) {
        return this.stepTrack(tip3, target3, null, dt);
      },
      // Tracking form: feedback on a (possibly moving) reference point plus
      // the reference's configuration velocity as feed-forward. With a fixed
      // reference and no feed-forward this is exactly the direct law.
      stepTrack(tip3, ref3, qDotRef, dt) {
        const e = [tip3[0] - ref3[0], tip3[1] - ref3[1], tip3[2] - ref3[2]];
        const J = idealJacobian3(qBelief);
        const fb = dlsVelocity(J, e, GAIN);
        const v = clampRate(qDotRef ? fb.map((x, i) => x + qDotRef[i]) : fb, RATE_MAX);
        for (let i = 0; i < 4; i++) qBelief[i] += v[i] * dt;
        qBelief = pcc.clampQ(qBelief);
        return qBelief.slice();
      },
    };
  }

  CR.ibvs = { GAIN, RATE_MAX, idealJacobian3, dampedPinv, dlsVelocity, clampRate, createClassical };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/features.js ----
// Feature vector shared by the learned controller's training and runtime.
// Inputs are the sensing layer's output only: the four triangulated marker
// positions (12 numbers) and the tip-to-target error (3 numbers). Units are
// the simulator's normalized length units; standardization happens in
// training and is stored with the weights.
(function (CR) {
  'use strict';

  const DIM = 15;

  function build(markers3, target3) {
    const x = new Array(DIM);
    for (let i = 0; i < 4; i++) {
      x[3 * i] = markers3[i][0];
      x[3 * i + 1] = markers3[i][1];
      x[3 * i + 2] = markers3[i][2];
    }
    const tip = markers3[3];
    x[12] = tip[0] - target3[0];
    x[13] = tip[1] - target3[1];
    x[14] = tip[2] - target3[2];
    return x;
  }

  CR.features = { DIM, build };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/mlp.js ----
// Tiny fully-connected network: tanh hidden layers, linear output.
// Plain arrays so weights serialize straight to JSON. Includes just enough
// training machinery (backprop + Adam) for the offline training script.
(function (CR) {
  'use strict';

  function create(sizes, rng) {
    const layers = [];
    for (let l = 0; l < sizes.length - 1; l++) {
      const nIn = sizes[l], nOut = sizes[l + 1];
      const scale = Math.sqrt(1 / nIn);
      const W = new Array(nOut * nIn);
      for (let i = 0; i < W.length; i++) W[i] = rng.gauss() * scale;
      layers.push({ W, b: new Array(nOut).fill(0), nIn, nOut });
    }
    return { sizes: sizes.slice(), layers };
  }

  function forward(net, x) {
    let a = x;
    for (let l = 0; l < net.layers.length; l++) {
      const { W, b, nIn, nOut } = net.layers[l];
      const z = new Array(nOut);
      for (let i = 0; i < nOut; i++) {
        let s = b[i];
        const row = i * nIn;
        for (let j = 0; j < nIn; j++) s += W[row + j] * a[j];
        z[i] = l < net.layers.length - 1 ? Math.tanh(s) : s;
      }
      a = z;
    }
    return a;
  }

  // Forward pass keeping activations, then backprop of 0.5*||y - t||^2.
  // Accumulates into grads (same shape as net layers). Returns the loss.
  function backprop(net, x, target, grads) {
    const acts = [x];
    let a = x;
    for (let l = 0; l < net.layers.length; l++) {
      const { W, b, nIn, nOut } = net.layers[l];
      const z = new Array(nOut);
      for (let i = 0; i < nOut; i++) {
        let s = b[i];
        const row = i * nIn;
        for (let j = 0; j < nIn; j++) s += W[row + j] * a[j];
        z[i] = l < net.layers.length - 1 ? Math.tanh(s) : s;
      }
      acts.push(z);
      a = z;
    }
    const out = acts[acts.length - 1];
    let loss = 0;
    let delta = new Array(out.length);
    for (let i = 0; i < out.length; i++) {
      const d = out[i] - target[i];
      loss += 0.5 * d * d;
      delta[i] = d;
    }
    for (let l = net.layers.length - 1; l >= 0; l--) {
      const { W, nIn, nOut } = net.layers[l];
      const aPrev = acts[l];
      const g = grads[l];
      for (let i = 0; i < nOut; i++) {
        const row = i * nIn;
        g.b[i] += delta[i];
        for (let j = 0; j < nIn; j++) g.W[row + j] += delta[i] * aPrev[j];
      }
      if (l > 0) {
        const newDelta = new Array(nIn).fill(0);
        for (let j = 0; j < nIn; j++) {
          let s = 0;
          for (let i = 0; i < nOut; i++) s += W[i * nIn + j] * delta[i];
          const h = acts[l][j]; // tanh activation of layer l-1's output
          newDelta[j] = s * (1 - h * h);
        }
        delta = newDelta;
      }
    }
    return loss;
  }

  function zeroGrads(net) {
    return net.layers.map((L) => ({
      W: new Array(L.W.length).fill(0),
      b: new Array(L.b.length).fill(0),
    }));
  }

  function makeAdam(net, lr) {
    const m = zeroGrads(net), v = zeroGrads(net);
    let t = 0;
    const b1 = 0.9, b2 = 0.999, eps = 1e-8;
    return function step(grads, batchSize) {
      t++;
      const c1 = 1 - Math.pow(b1, t), c2 = 1 - Math.pow(b2, t);
      for (let l = 0; l < net.layers.length; l++) {
        const L = net.layers[l], g = grads[l], ml = m[l], vl = v[l];
        for (let k = 0; k < L.W.length; k++) {
          const gk = g.W[k] / batchSize;
          ml.W[k] = b1 * ml.W[k] + (1 - b1) * gk;
          vl.W[k] = b2 * vl.W[k] + (1 - b2) * gk * gk;
          L.W[k] -= (lr * (ml.W[k] / c1)) / (Math.sqrt(vl.W[k] / c2) + eps);
        }
        for (let k = 0; k < L.b.length; k++) {
          const gk = g.b[k] / batchSize;
          ml.b[k] = b1 * ml.b[k] + (1 - b1) * gk;
          vl.b[k] = b2 * vl.b[k] + (1 - b2) * gk * gk;
          L.b[k] -= (lr * (ml.b[k] / c1)) / (Math.sqrt(vl.b[k] / c2) + eps);
        }
      }
    };
  }

  CR.mlp = { create, forward, backprop, zeroGrads, makeAdam };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/learned.js ----
// Learned controller runtime (version 3): an ensemble of small MLPs maps the
// sensing layer's output (triangulated markers + 3D target error) to a 4x3
// feedback gain matrix G ~ -gain * pinv(J_truth), the truth sim's
// damped-least-squares law. The command is v = G e, so it vanishes exactly at
// zero error and network noise scales down with the error instead of
// dithering around the target.
//
// The ensemble gives two things one network would not: a smoother mean gain
// and an honest disagreement signal. When the target leaves the region the
// ensemble was trained on, the features leave the training set, or the
// members disagree, the controller flags it and scales its authority down
// rather than extrapolating with confidence.
(function (CR) {
  'use strict';
  const { pcc, mlp, features, ibvs } = CR;

  const OOD_GAIN = 0.3; // authority multiplier while flagged
  const M = 3;          // error dimension

  // opts.targetTest(p) -> true if the target lies in the population the
  // training targets were drawn from (tips at up to 90% of the curvature
  // limits). The app supplies the planner's inverse kinematics with the limits
  // scaled to 90% as that test: exact up to IK convergence. A target outside
  // it is something the ensemble was never shown, whatever the feature-space
  // distance says: the features cannot separate absolute target position
  // sharply, so this test is explicit, as it was in version 1. Cached per
  // target, since it costs an IK solve.
  function createLearned(blob, opts) {
    if (!blob || !blob.members || !blob.members.length) return null;
    const { inputMean, inputStd, envelope, knnK, knnWarn, labelMean, labelStd, sigmaWarn } = blob;
    const targetTest = opts && opts.targetTest;
    let lastTarget = null, lastInside = true;
    function targetInPopulation(p) {
      if (!targetTest) return true;
      if (lastTarget && p[0] === lastTarget[0] && p[1] === lastTarget[1] && p[2] === lastTarget[2]) return lastInside;
      lastTarget = p.slice();
      lastInside = !!targetTest(p);
      return lastInside;
    }
    let qCmd = [0, 0, 0, 0];

    // distance to the k-th nearest stored training feature vector,
    // standardized space; the training script set knnWarn from holdout data.
    // The features carry absolute marker positions and the error vector, so
    // this test covers both "where the robot is" and "what it is asked".
    const D = features.DIM;
    const envFlat = new Float64Array(envelope.length * D);
    for (let n = 0; n < envelope.length; n++) for (let i = 0; i < D; i++) envFlat[n * D + i] = envelope[n][i];
    function knnDist(xn) {
      const best = new Array(knnK).fill(Infinity); // ascending
      for (let n = 0, off = 0; n < envelope.length; n++, off += D) {
        let d2 = 0;
        for (let i = 0; i < D; i++) { const d = xn[i] - envFlat[off + i]; d2 += d * d; }
        if (d2 < best[knnK - 1]) {
          let j = knnK - 1;
          while (j > 0 && best[j - 1] > d2) { best[j] = best[j - 1]; j--; }
          best[j] = d2;
        }
      }
      return Math.sqrt(best[knnK - 1]);
    }
    const standardize = (x) => x.map((v, i) => (v - inputMean[i]) / inputStd[i]);

    // per-member velocity v = G e, with G de-standardized from the net output
    function memberVelocities(xn, e) {
      return blob.members.map((net) => {
        const g = mlp.forward(net, xn);
        const v = new Array(4);
        for (let i = 0; i < 4; i++) {
          let s = 0;
          for (let k = 0; k < M; k++) {
            const idx = i * M + k;
            s += (g[idx] * labelStd[idx] + labelMean[idx]) * e[k];
          }
          v[i] = s;
        }
        return v;
      });
    }

    return {
      name: 'learned',
      sigmaWarn,
      knnWarn,
      reset(q0) { qCmd = (q0 || [0, 0, 0, 0]).slice(); },
      qCmd: () => qCmd.slice(),
      // Direct law: feedback on the target, no feed-forward.
      step(markers3, target3, dt) {
        return this.stepTrack(markers3, target3, null, dt, target3);
      },
      // Tracking form: the networks' feedback acts on the reference point; the
      // plan's configuration velocity is added as feed-forward and is not
      // scaled by the authority drop, since it does not come from the
      // networks. The envelope test is run for the reference AND for the final
      // target, so a request the ensemble was never shown is flagged even
      // while the reference is still nearby.
      stepTrack(markers3, ref3, qDotRef, dt, finalTarget3) {
        const tip = markers3[3];
        const e = [tip[0] - ref3[0], tip[1] - ref3[1], tip[2] - ref3[2]];
        const xn = standardize(features.build(markers3, ref3));
        const dEnv = knnDist(xn);
        let dEnvFinal = dEnv;
        if (finalTarget3 && finalTarget3 !== ref3) {
          dEnvFinal = knnDist(standardize(features.build(markers3, finalTarget3)));
        }
        const outsideTargets = !targetInPopulation(finalTarget3 || ref3);
        let ood = outsideTargets || dEnv > knnWarn || dEnvFinal > knnWarn;

        const vs = memberVelocities(xn, e);
        const mean = [0, 0, 0, 0];
        for (const v of vs) for (let i = 0; i < 4; i++) mean[i] += v[i] / vs.length;
        let varSum = 0;
        for (const v of vs) for (let i = 0; i < 4; i++) {
          const d = v[i] - mean[i];
          varSum += d * d;
        }
        const sigma = Math.sqrt(varSum / (vs.length * 4));
        if (sigma > sigmaWarn) ood = true;

        const gainScale = ood ? OOD_GAIN : 1;
        const v = ibvs.clampRate(
          mean.map((vi, i) => vi * gainScale + (qDotRef ? qDotRef[i] : 0)), ibvs.RATE_MAX);
        for (let i = 0; i < 4; i++) qCmd[i] += v[i] * dt;
        qCmd = pcc.clampQ(qCmd);
        return { qCmd: qCmd.slice(), sigma, dEnv: Math.max(dEnv, dEnvFinal), outsideTargets, ood, gainScale, membersV: vs, meanV: mean };
      },
    };
  }

  CR.learned = { createLearned, OOD_GAIN };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/planner.js ----
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

// ---- src/core/workspace.js ----
// Reachable workspace of the ideal model, in 3D.
//
// Sample the configuration space densely up to the curvature limits, keep the
// tip positions, and describe their outer boundary as a radial envelope
// around the cloud's centroid: for each direction (theta, phi) the largest
// tip radius seen in that angular bin, hole-filled, smoothed, and then scaled
// up by the smallest factor that keeps 99.95% of the samples inside, plus
// half a percent. The
// result is a closed, smooth surface, the "dome" drawn in the inspector. It
// is an outer envelope: the reachable set is a solid inside it, and where the
// true set is concave the envelope is generous. Reachability of a specific
// target is decided by the planner's inverse kinematics, not by this surface.
(function (CR) {
  'use strict';
  const { pcc, v3 } = CR;

  function reachableVolume(opts) {
    const o = Object.assign({ samples: 200000, nt: 36, np: 72, seed: 3, keep: 0.9995, margin: 1.005, smooth: 2, scale: 1.0 }, opts || {});
    const rng = CR.makeRng(o.seed);
    const pts = new Float64Array(o.samples * 3);
    const c = [0, 0, 0];
    for (let n = 0; n < o.samples; n++) {
      const q = [];
      for (let i = 0; i < pcc.NSEG; i++) {
        const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * o.scale * pcc.KMAX[i];
        q.push(k * Math.cos(a), k * Math.sin(a));
      }
      const p = pcc.tip3(q);
      pts[3 * n] = p[0]; pts[3 * n + 1] = p[1]; pts[3 * n + 2] = p[2];
      c[0] += p[0] / o.samples; c[1] += p[1] / o.samples; c[2] += p[2] / o.samples;
    }
    const { nt, np } = o;
    const rmax = new Float64Array(nt * np);
    const binOf = (p) => {
      const d = v3.sub(p, c), r = v3.norm(d);
      const th = Math.acos(Math.max(-1, Math.min(1, d[2] / (r || 1e-9)))); // polar axis = robot axis (+z)
      const ph = Math.atan2(d[1], d[0]);
      const i = Math.min(nt - 1, Math.floor((th / Math.PI) * nt));
      const j = ((Math.floor(((ph + Math.PI) / (2 * Math.PI)) * np) % np) + np) % np;
      return { i, j, r };
    };
    for (let n = 0; n < o.samples; n++) {
      const b = binOf([pts[3 * n], pts[3 * n + 1], pts[3 * n + 2]]);
      if (b.r > rmax[b.i * np + b.j]) rmax[b.i * np + b.j] = b.r;
    }
    // hole filling: empty bins take the mean of their filled neighbours
    let grid = Float64Array.from(rmax);
    for (let pass = 0; pass < 8; pass++) {
      const next = Float64Array.from(grid);
      let holes = 0;
      for (let i = 0; i < nt; i++) for (let j = 0; j < np; j++) {
        if (grid[i * np + j] > 0) continue;
        let s = 0, k = 0;
        for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
          const ii = i + di, jj = ((j + dj) % np + np) % np;
          if (ii < 0 || ii >= nt) continue;
          const v = grid[ii * np + jj];
          if (v > 0) { s += v; k++; }
        }
        if (k) next[i * np + j] = s / k; else holes++;
      }
      grid = next;
      if (!holes) break;
    }
    // smoothing: box filter, wrap in phi, clamp in theta
    for (let pass = 0; pass < o.smooth; pass++) {
      const next = new Float64Array(nt * np);
      for (let i = 0; i < nt; i++) for (let j = 0; j < np; j++) {
        let s = 0, k = 0;
        for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
          const ii = i + di, jj = ((j + dj) % np + np) % np;
          if (ii < 0 || ii >= nt) continue;
          s += grid[ii * np + jj]; k++;
        }
        next[i * np + j] = s / k;
      }
      grid = next;
    }
    // containment margin: scale so that `keep` of the samples are inside
    const ratios = new Float64Array(o.samples);
    for (let n = 0; n < o.samples; n++) {
      const b = binOf([pts[3 * n], pts[3 * n + 1], pts[3 * n + 2]]);
      ratios[n] = b.r / grid[b.i * np + b.j];
    }
    ratios.sort();
    const scale = Math.max(1, ratios[Math.floor(o.keep * (o.samples - 1))]) * o.margin;
    for (let k = 0; k < grid.length; k++) grid[k] *= scale;
    let inside = 0;
    for (let n = 0; n < o.samples; n++) if (ratios[n] <= scale) inside++;
    return {
      center: c.map((v) => Math.round(v * 1e4) / 1e4),
      nt, np,
      r: Array.from(grid, (v) => Math.round(v * 1e4) / 1e4),
      meta: { samples: o.samples, curvatureFraction: o.scale, marginScale: Math.round(scale * 1e4) / 1e4, insideFrac: inside / o.samples },
    };
  }

  // Direction for bin centres (theta from +z, phi about +z measured from +x
  // toward +y, matching binOf above).
  function dir(theta, phi) {
    const st = Math.sin(theta);
    return [st * Math.cos(phi), st * Math.sin(phi), Math.cos(theta)];
  }

  // Vertex grid of the envelope surface: rows theta (with pole rows added),
  // columns phi; each entry a world point.
  function envelopeMesh(vol) {
    const { nt, np, r, center } = vol;
    const rows = [];
    const poleR = (i) => { let s = 0; for (let j = 0; j < np; j++) s += r[i * np + j]; return s / np; };
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, 0, 1], poleR(0)))));
    for (let i = 0; i < nt; i++) {
      const th = ((i + 0.5) / nt) * Math.PI;
      const row = [];
      for (let j = 0; j < np; j++) {
        const ph = ((j + 0.5) / np) * 2 * Math.PI - Math.PI;
        row.push(v3.add(center, v3.scale(dir(th, ph), r[i * np + j])));
      }
      rows.push(row);
    }
    rows.push(new Array(np).fill(0).map(() => v3.add(center, v3.scale([0, 0, -1], poleR(nt - 1)))));
    return rows;
  }

  // Occupancy grid of the reachable tip set (the envelope is an outer surface
  // and hides the unreachable interior near the base; this does not). Cells of
  // side `res` over a fixed box, filled from sampled tips, then a 3D closing
  // (dilate, erode, 6-neighbour) to remove sampling holes. Packed bits.
  const GRID_BOX = { min: [-1.7, -1.7, -1.2], max: [1.7, 1.7, 2.2] };
  function reachableGrid(opts) {
    const o = Object.assign({ samples: 300000, res: 0.06, seed: 5, scale: 1.0 }, opts || {});
    const n = [0, 1, 2].map((k) => Math.ceil((GRID_BOX.max[k] - GRID_BOX.min[k]) / o.res));
    const total = n[0] * n[1] * n[2];
    let g = new Uint8Array(total);
    const idx = (i, j, k) => (i * n[1] + j) * n[2] + k;
    const rng = CR.makeRng(o.seed);
    for (let s = 0; s < o.samples; s++) {
      const q = [];
      for (let m = 0; m < pcc.NSEG; m++) {
        const a = rng() * 2 * Math.PI, kk = Math.sqrt(rng()) * o.scale * pcc.KMAX[m];
        q.push(kk * Math.cos(a), kk * Math.sin(a));
      }
      const p = pcc.tip3(q);
      const i = Math.floor((p[0] - GRID_BOX.min[0]) / o.res), j = Math.floor((p[1] - GRID_BOX.min[1]) / o.res), k = Math.floor((p[2] - GRID_BOX.min[2]) / o.res);
      if (i >= 0 && j >= 0 && k >= 0 && i < n[0] && j < n[1] && k < n[2]) g[idx(i, j, k)] = 1;
    }
    const morph = (src, fill) => {
      const out = new Uint8Array(total);
      for (let i = 0; i < n[0]; i++) for (let j = 0; j < n[1]; j++) for (let k = 0; k < n[2]; k++) {
        const c = src[idx(i, j, k)];
        const nb = [
          i > 0 ? src[idx(i - 1, j, k)] : 0, i < n[0] - 1 ? src[idx(i + 1, j, k)] : 0,
          j > 0 ? src[idx(i, j - 1, k)] : 0, j < n[1] - 1 ? src[idx(i, j + 1, k)] : 0,
          k > 0 ? src[idx(i, j, k - 1)] : 0, k < n[2] - 1 ? src[idx(i, j, k + 1)] : 0,
        ];
        out[idx(i, j, k)] = fill ? (c || nb.some((v) => v) ? 1 : 0) : (c && nb.every((v) => v) ? 1 : 0);
      }
      return out;
    };
    // closing removes sampling holes, then an opening removes isolated cells
    g = morph(morph(g, true), false);
    g = morph(morph(g, false), true);
    return { min: GRID_BOX.min.slice(), res: o.res, n, bits: packBits(g) };
  }
  function packBits(u8) {
    const out = new Uint8Array(Math.ceil(u8.length / 8));
    for (let i = 0; i < u8.length; i++) if (u8[i]) out[i >> 3] |= 1 << (i & 7);
    return out;
  }
  function gridGet(grid, i, j, k) {
    const { n } = grid;
    if (i < 0 || j < 0 || k < 0 || i >= n[0] || j >= n[1] || k >= n[2]) return 0;
    const b = (i * n[1] + j) * n[2] + k;
    return (grid.bits[b >> 3] >> (b & 7)) & 1;
  }
  function gridContains(grid, p) {
    const i = Math.floor((p[0] - grid.min[0]) / grid.res), j = Math.floor((p[1] - grid.min[1]) / grid.res), k = Math.floor((p[2] - grid.min[2]) / grid.res);
    return !!gridGet(grid, i, j, k);
  }
  // base64 round trip for JSON embedding (Node and browser)
  function gridToJSON(grid) {
    const bytes = grid.bits;
    let bin = '';
    for (let i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
    const b64 = typeof btoa === 'function' ? btoa(bin) : Buffer.from(bin, 'binary').toString('base64');
    return { min: grid.min, res: grid.res, n: grid.n, bits64: b64 };
  }
  function gridFromJSON(j) {
    const bin = typeof atob === 'function' ? atob(j.bits64) : Buffer.from(j.bits64, 'base64').toString('binary');
    const bits = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bits[i] = bin.charCodeAt(i);
    return { min: j.min, res: j.res, n: j.n, bits };
  }

  // Horizontal slice of the grid at height h (z), traced with marching squares:
  // returns line segments (pairs of world points) covering every contour,
  // holes included. Corner values are cell occupancies; segments pass through
  // edge midpoints, so the outline sits half a cell outside occupied centres.
  function gridSectionSegments(grid, h) {
    const { n, res, min } = grid;
    const k = Math.floor((h - min[2]) / res);
    if (k < 0 || k >= n[2]) return [];
    const cx = (i) => min[0] + (i + 0.5) * res, cy = (j) => min[1] + (j + 0.5) * res;
    const segs = [];
    // corners of the marching cell (i,j): a=(i,j) b=(i+1,j) c=(i+1,j+1) d=(i,j+1)
    for (let i = -1; i < n[0]; i++) for (let j = -1; j < n[1]; j++) {
      const a = gridGet(grid, i, j, k), b = gridGet(grid, i + 1, j, k), c = gridGet(grid, i + 1, j + 1, k), d = gridGet(grid, i, j + 1, k);
      const code = (a << 3) | (b << 2) | (c << 1) | d;
      if (code === 0 || code === 15) continue;
      const top = [cx(i) + res / 2, cy(j), h], right = [cx(i + 1), cy(j) + res / 2, h];
      const bottom = [cx(i) + res / 2, cy(j + 1), h], left = [cx(i), cy(j) + res / 2, h];
      const add = (p, q) => segs.push([p, q]);
      switch (code) {
        case 1: case 14: add(left, bottom); break;
        case 2: case 13: add(bottom, right); break;
        case 3: case 12: add(left, right); break;
        case 4: case 11: add(top, right); break;
        case 5: add(top, left); add(bottom, right); break;
        case 6: case 9: add(top, bottom); break;
        case 7: case 8: add(top, left); break;
        case 10: add(top, right); add(left, bottom); break;
        default: break;
      }
    }
    return segs;
  }
  // occupied cell centres on the slice (for tests and hints)
  function gridSliceCells(grid, h) {
    const { n, res, min } = grid;
    const k = Math.floor((h - min[2]) / res);
    const out = [];
    if (k < 0 || k >= n[2]) return out;
    for (let i = 0; i < n[0]; i++) for (let j = 0; j < n[1]; j++) {
      if (gridGet(grid, i, j, k)) out.push([min[0] + (i + 0.5) * res, min[1] + (j + 0.5) * res, h]);
    }
    return out;
  }

  // Is a world point inside the envelope (nearest-bin radial test)?
  function insideEnvelope(vol, p) {
    const { nt, np, r, center } = vol;
    const d = v3.sub(p, center), rr = v3.norm(d);
    if (rr < 1e-9) return true;
    const th = Math.acos(Math.max(-1, Math.min(1, d[2] / rr)));
    const ph = Math.atan2(d[1], d[0]);
    const i = Math.min(nt - 1, Math.floor((th / Math.PI) * nt));
    const j = ((Math.floor(((ph + Math.PI) / (2 * Math.PI)) * np) % np) + np) % np;
    return rr <= r[i * np + j];
  }

  CR.workspace = { reachableVolume, envelopeMesh, insideEnvelope,
    reachableGrid, gridContains, gridToJSON, gridFromJSON, gridSectionSegments, gridSliceCells };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));

// ---- src/core/hyst.js ----
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
