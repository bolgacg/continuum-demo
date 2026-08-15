// Learned controller runtime: an ensemble of small MLPs maps image features
// (marker pixels + target error) to a curvature-rate command. The ensemble
// gives two things the single network would not: a smoother mean command and
// an honest disagreement signal. When the input leaves the training envelope
// or the members disagree too much, the controller flags OOD and scales its
// gain down rather than extrapolating at full authority.
(function (CR) {
  'use strict';
  const { pcc, mlp, features, ibvs } = CR;

  const OOD_GAIN = 0.3;      // authority multiplier while flagged OOD
  const ENVELOPE_MARGIN = 0.10; // fraction of the per-dim training range

  function createLearned(blob) {
    if (!blob || !blob.members || !blob.members.length) return null;
    const { inputMean, inputStd, inputMin, inputMax, sigmaWarn } = blob;
    let qCmd = [0, 0, 0, 0];

    function memberVelocities(x) {
      const xn = x.map((v, i) => (v - inputMean[i]) / inputStd[i]);
      return blob.members.map((net) => mlp.forward(net, xn));
    }

    return {
      name: 'learned',
      sigmaWarn,
      reset(q0) { qCmd = (q0 || [0, 0, 0, 0]).slice(); },
      qCmd: () => qCmd.slice(),
      step(markersPx, targetPx, dt, w, h) {
        const x = features.build(markersPx, targetPx, w, h);

        // envelope check against the training data's per-dim range
        let ood = false;
        for (let i = 0; i < x.length; i++) {
          const range = inputMax[i] - inputMin[i];
          const m = ENVELOPE_MARGIN * range;
          if (x[i] < inputMin[i] - m || x[i] > inputMax[i] + m) { ood = true; break; }
        }

        const vs = memberVelocities(x);
        const nOut = vs[0].length;
        const mean = new Array(nOut).fill(0);
        for (const v of vs) for (let i = 0; i < nOut; i++) mean[i] += v[i] / vs.length;
        let varSum = 0;
        for (const v of vs) for (let i = 0; i < nOut; i++) {
          const d = v[i] - mean[i];
          varSum += d * d;
        }
        const sigma = Math.sqrt(varSum / (vs.length * nOut));
        if (sigma > sigmaWarn) ood = true;

        const gainScale = ood ? OOD_GAIN : 1;
        const v = ibvs.clampRate(mean.map((vi) => vi * gainScale), ibvs.RATE_MAX);
        for (let i = 0; i < 4; i++) qCmd[i] += v[i] * dt;
        qCmd = pcc.clampQ(qCmd);
        return { qCmd: qCmd.slice(), sigma, ood, gainScale, membersV: vs, meanV: mean };
      },
    };
  }

  CR.learned = { createLearned, OOD_GAIN };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
