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
