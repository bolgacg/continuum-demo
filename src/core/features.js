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
