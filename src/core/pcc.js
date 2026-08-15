// Piecewise constant curvature (PCC) kinematics for a 2-segment continuum robot.
//
// Configuration q = [kx1, ky1, kx2, ky2]: each segment bends with a curvature
// vector (kx, ky); magnitude is the curvature, direction is the bending plane.
// The robot's base frame has +z along the undeformed backbone (horizontal in
// the world; gravity is -y). Units are normalized (segment lengths ~1); the
// demo does not claim a physical scale.
(function (CR) {
  'use strict';
  const { m3 } = CR;

  const SEG_LEN = [1.0, 0.8];
  const NSEG = 2;
  const KMAX = [2.2, 2.6]; // max curvature magnitude per segment

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

  function clampQ(q) {
    const out = q.slice();
    for (let i = 0; i < NSEG; i++) {
      const kx = out[2 * i], ky = out[2 * i + 1];
      const k = Math.hypot(kx, ky);
      if (k > KMAX[i]) {
        const f = KMAX[i] / k;
        out[2 * i] = kx * f;
        out[2 * i + 1] = ky * f;
      }
    }
    return out;
  }

  CR.pcc = { SEG_LEN, NSEG, KMAX, NQ: 2 * NSEG, segPose, poseAt, tip3, backbone, markers3, clampQ };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
