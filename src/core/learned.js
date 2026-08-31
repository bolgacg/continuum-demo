// Learned controller runtime: an ensemble of small MLPs maps image features
// (marker pixels + target error) to a 4x2 feedback gain matrix
// G ~ -gain * pinv(J_truth), the truth sim's damped-least-squares law. The
// command is v = G e, so it vanishes exactly at zero pixel error and network
// noise scales down with the error instead of dithering around the target.
//
// The ensemble gives two things one network would not: a smoother mean gain
// and an honest disagreement signal. When the input leaves the training
// envelope or the members disagree, the controller flags OOD and scales its
// authority down rather than extrapolating with confidence.
(function (CR) {
  'use strict';
  const { pcc, mlp, features, ibvs } = CR;

  const OOD_GAIN = 0.3; // authority multiplier while flagged OOD

  function createLearned(blob) {
    if (!blob || !blob.members || !blob.members.length) return null;
    const { inputMean, inputStd, envelope, knnK, knnWarn,
            targetHull, labelMean, labelStd, sigmaWarn } = blob;
    let qCmd = [0, 0, 0, 0];

    // is the requested target inside the convex hull of training targets?
    // (ray casting; the relative-error features cannot see absolute position)
    function targetInEnvelope(p) {
      let inside = false;
      for (let i = 0, j = targetHull.length - 1; i < targetHull.length; j = i++) {
        const a = targetHull[i], b = targetHull[j];
        if ((a[1] > p[1]) !== (b[1] > p[1]) &&
            p[0] < ((b[0] - a[0]) * (p[1] - a[1])) / (b[1] - a[1]) + a[0]) {
          inside = !inside;
        }
      }
      return inside;
    }

    // distance to the k-th nearest stored training feature vector,
    // standardized space; the training script set knnWarn from holdout data
    function knnDist(xn) {
      const D = xn.length;
      const best = new Array(knnK).fill(Infinity); // ascending
      for (const p of envelope) {
        let d2 = 0;
        for (let i = 0; i < D; i++) { const d = xn[i] - p[i]; d2 += d * d; }
        if (d2 < best[knnK - 1]) {
          let j = knnK - 1;
          while (j > 0 && best[j - 1] > d2) { best[j] = best[j - 1]; j--; }
          best[j] = d2;
        }
      }
      return Math.sqrt(best[knnK - 1]);
    }

    // per-member velocity v = G e, with G de-standardized from the net output
    function memberVelocities(xn, e) {
      return blob.members.map((net) => {
        const g = mlp.forward(net, xn);
        const v = new Array(4);
        for (let i = 0; i < 4; i++) {
          const g0 = g[2 * i] * labelStd[2 * i] + labelMean[2 * i];
          const g1 = g[2 * i + 1] * labelStd[2 * i + 1] + labelMean[2 * i + 1];
          v[i] = g0 * e[0] + g1 * e[1];
        }
        return v;
      });
    }

    return {
      name: 'learned',
      sigmaWarn,
      targetHull,
      reset(q0) { qCmd = (q0 || [0, 0, 0, 0]).slice(); },
      qCmd: () => qCmd.slice(),
      // Version 1 law: feedback on the clicked target, no feed-forward.
      step(markersPx, targetPx, dt, w, h) {
        return this.stepTrack(markersPx, targetPx, null, dt, w, h, targetPx);
      },
      // Tracking form: the network's feedback acts on the reference pixel;
      // the plan's configuration velocity is added as feed-forward and is not
      // scaled by the OOD authority drop, since it does not come from the
      // networks. The envelope test is on the final target, the thing the
      // ensemble was never shown, not on the moving reference.
      stepTrack(markersPx, refPx, qDotRef, dt, w, h, finalTargetPx) {
        const x = features.build(markersPx, refPx, w, h);
        const tip = markersPx[3];
        const e = [tip[0] - refPx[0], tip[1] - refPx[1]];

        // envelope checks: target inside the training-target hull, then
        // feature-space nearest-neighbor distance to the training set
        const xn = x.map((v, i) => (v - inputMean[i]) / inputStd[i]);
        const dEnv = knnDist(xn);
        let ood = !targetInEnvelope(finalTargetPx || refPx) || dEnv > knnWarn;

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
        return { qCmd: qCmd.slice(), sigma, dEnv, ood, gainScale, membersV: vs, meanV: mean };
      },
    };
  }

  CR.learned = { createLearned, OOD_GAIN };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
