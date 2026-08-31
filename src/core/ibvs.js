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
