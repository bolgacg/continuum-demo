// Image-feature vector shared by the learned controller's training and
// runtime. Inputs are pixels only: 4 marker positions + tip-to-target error,
// normalized by image size. 10 numbers total.
(function (CR) {
  'use strict';

  const DIM = 10;

  function build(markersPx, targetPx, w, h) {
    const x = new Array(DIM);
    const sx = w / 2, sy = h / 2;
    for (let i = 0; i < 4; i++) {
      x[2 * i] = (markersPx[i][0] - sx) / sx;
      x[2 * i + 1] = (markersPx[i][1] - sy) / sy;
    }
    const tip = markersPx[3];
    x[8] = (tip[0] - targetPx[0]) / sx;
    x[9] = (tip[1] - targetPx[1]) / sy;
    return x;
  }

  CR.features = { DIM, build };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
