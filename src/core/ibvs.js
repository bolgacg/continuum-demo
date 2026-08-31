// Classical image-based visual servoing with a model-based Jacobian.
//
// The controller keeps an internal belief of the configuration (the commands
// it has integrated), computes the pixel Jacobian of the *idealized* PCC
// model at that belief, and does damped-least-squares descent on the image
// error. It never sees the truth sim's state: only the tip pixel from the
// camera. The gap between its ideal model and the truth sim is exactly what
// the demo is about.
(function (CR) {
  'use strict';
  const { pcc } = CR;

  const GAIN = 2.2;      // error feedback gain, 1/s
  const RATE_MAX = 3.0;  // command rate clamp, curvature units / s
  const DAMP_FRAC = 0.02;

  // Pixel Jacobian of the ideal PCC tip wrt q, central differences.
  function idealJacobianPx(q, cam) {
    const h = 1e-3;
    const J = [[0, 0, 0, 0], [0, 0, 0, 0]];
    for (let c = 0; c < 4; c++) {
      const qp = q.slice(); qp[c] += h;
      const qm = q.slice(); qm[c] -= h;
      const pp = cam.project(pcc.tip3(qp));
      const pm = cam.project(pcc.tip3(qm));
      if (!pp || !pm) continue;
      J[0][c] = (pp[0] - pm[0]) / (2 * h);
      J[1][c] = (pp[1] - pm[1]) / (2 * h);
    }
    return J;
  }

  // Damped pseudo-inverse of a 2x4 Jacobian: P = J^T (J J^T + mu I)^-1,
  // returned as a 4x2 row-major array.
  function dampedPinv(J) {
    const a = J[0], b = J[1];
    let aa = 0, ab = 0, bb = 0;
    for (let i = 0; i < 4; i++) {
      aa += a[i] * a[i];
      ab += a[i] * b[i];
      bb += b[i] * b[i];
    }
    const mu = DAMP_FRAC * (aa + bb) / 2 + 1e-6;
    const m00 = aa + mu, m01 = ab, m11 = bb + mu;
    const det = m00 * m11 - m01 * m01;
    // inv(JJ^T + mu I) = [i00 i01; i01 i11]
    const i00 = m11 / det, i01 = -m01 / det, i11 = m00 / det;
    const P = new Array(8);
    for (let i = 0; i < 4; i++) {
      P[2 * i] = a[i] * i00 + b[i] * i01;
      P[2 * i + 1] = a[i] * i01 + b[i] * i11;
    }
    return P;
  }

  // v = -gain * P e   (damped least squares descent on the image error)
  function dlsVelocity(J, e, gain) {
    const P = dampedPinv(J);
    const v = new Array(4);
    for (let i = 0; i < 4; i++) v[i] = -gain * (P[2 * i] * e[0] + P[2 * i + 1] * e[1]);
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

  function createClassical(cam) {
    let qBelief = [0, 0, 0, 0];
    return {
      name: 'classical',
      reset(q0) { qBelief = (q0 || [0, 0, 0, 0]).slice(); },
      qBelief: () => qBelief.slice(),
      // tipPx/targetPx: [u, v]. Returns the new absolute command.
      // Version 1 law: feedback on the clicked target, no feed-forward.
      step(tipPx, targetPx, dt) {
        return this.stepTrack(tipPx, targetPx, null, dt);
      },
      // Tracking form: feedback on a (possibly moving) reference pixel plus
      // the reference's configuration velocity as feed-forward. With a fixed
      // reference and no feed-forward this is exactly the version 1 law.
      stepTrack(tipPx, refPx, qDotRef, dt) {
        const e = [tipPx[0] - refPx[0], tipPx[1] - refPx[1]];
        const J = idealJacobianPx(qBelief, cam);
        const fb = dlsVelocity(J, e, GAIN);
        const v = clampRate(qDotRef ? fb.map((x, i) => x + qDotRef[i]) : fb, RATE_MAX);
        for (let i = 0; i < 4; i++) qBelief[i] += v[i] * dt;
        qBelief = pcc.clampQ(qBelief);
        return qBelief.slice();
      },
    };
  }

  CR.ibvs = { GAIN, RATE_MAX, idealJacobianPx, dampedPinv, dlsVelocity, clampRate, createClassical };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
