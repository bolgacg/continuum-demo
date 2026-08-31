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
