// Trains the ensemble (version 3, 3D task). Run: node train/train.js
//
// Data: episodes on the truth sim with randomized flexibility (curvature
// limits), payload, drift, initial pose and 3D targets. The label at each visited state is the privileged
// expert's feedback gain matrix G = -gain * dampedPinv(J_truth), 4x3: the
// truth sim's Jacobian only exists at training time. The applied action is
// the expert's command plus exploration noise, so visited states spread
// beyond the expert's own trajectory. The networks see the sensing layer's
// output only (triangulated markers + target error), so whatever the expert
// knew about the disturbances has to be inferred from marker geometry.
//
// Triangulation is exact in this noiseless simulation (test/sanity.js
// asserts < 1e-9), so training reads the simulator's marker positions
// directly; they are identical to what the two cameras would return.
//
// Output: train/weights.json (ensemble + input stats + OOD envelope).
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');

const DT = 1 / 60;
const N_EPISODES = 600;          // doubled with the flexibility range: one network covers all mechanisms
const TARGETS_PER_EP = 3;
const STEPS_PER_TARGET = 156;   // 2.6 s
const RECORD_EVERY = 3;
const N_MEMBERS = 5;
const ARCH = [CR.features.DIM, 64, 48, 12]; // output: 4x3 gain matrix, row-major
const EPOCHS = 40;
const BATCH = 256;
const LR0 = 2e-3;
const HOLDOUT_FRAC = 0.1;

const rng = CR.makeRng(20260831);

function randomQ(scale) {
  const q = [];
  for (let i = 0; i < 2; i++) {
    const a = rng() * 2 * Math.PI;
    const k = Math.sqrt(rng()) * scale * CR.pcc.KMAX[i];
    q.push(k * Math.cos(a), k * Math.sin(a));
  }
  return q;
}

// ---------- dataset ----------
console.log('generating data...');
const X = [], Y = [];
const [FLEX_MIN, FLEX_MAX] = CR.pcc.FLEX_RANGE;
for (let ep = 0; ep < N_EPISODES; ep++) {
  // the mechanism's flexibility is a slider on the page; train across its
  // whole range so the ensemble has seen the poses every setting can reach
  CR.pcc.setFlex(FLEX_MIN + (FLEX_MAX - FLEX_MIN) * rng());
  const sim = CR.truth.createTruth(1000 + ep);
  sim.payload = rng() < 0.4 ? 0 : rng();
  sim.driftOn = rng() < 0.5;
  const q0 = randomQ(0.9);
  sim.reset(q0);
  let qCmd = q0.slice();
  for (let i = 0; i < 180; i++) sim.step(DT); // settle, let drift accumulate

  for (let tg = 0; tg < TARGETS_PER_EP; tg++) {
    const target = CR.pcc.tip3(randomQ(0.9));
    for (let s = 0; s < STEPS_PER_TARGET; s++) {
      const markers = sim.markers3();
      const tip = markers[3];
      const e = [tip[0] - target[0], tip[1] - target[1], tip[2] - target[2]];
      const J = CR.truth.truthJacobian3(sim);
      const P = CR.ibvs.dampedPinv(J); // 4 rows of 3
      const vExpert = CR.ibvs.clampRate(
        [0, 1, 2, 3].map((i) => -CR.ibvs.GAIN * (P[3 * i] * e[0] + P[3 * i + 1] * e[1] + P[3 * i + 2] * e[2])),
        CR.ibvs.RATE_MAX);

      if (s % RECORD_EVERY === 0) {
        X.push(CR.features.build(markers, target));
        Y.push(P.map((p) => -CR.ibvs.GAIN * p)); // label: G = -gain * pinv(J_truth)
      }
      const v = vExpert.map((vi) => vi + 0.35 * rng.gauss());
      for (let i = 0; i < 4; i++) qCmd[i] += v[i] * DT;
      qCmd = CR.pcc.clampQ(qCmd);
      sim.setCommand(qCmd);
      sim.step(DT);
    }
  }
}
CR.pcc.setFlex(1.0);
console.log('samples: ' + X.length);

// ---------- input stats + envelope ----------
const D = CR.features.DIM;
const mean = new Array(D).fill(0), std = new Array(D).fill(0);
for (const x of X) for (let i = 0; i < D; i++) mean[i] += x[i] / X.length;
for (const x of X) for (let i = 0; i < D; i++) std[i] += (x[i] - mean[i]) ** 2 / X.length;
for (let i = 0; i < D; i++) std[i] = Math.sqrt(std[i]) || 1;
const Xn = X.map((x) => x.map((v, i) => (v - mean[i]) / std[i]));

// OOD envelope: distance to the k-th nearest neighbour among a stored
// subsample of training features, in standardized space. The feature support
// is not Gaussian, so a covariance ellipsoid or per-dim box would swallow real
// outliers; nearest-neighbour distance tracks the actual data manifold.
const KNN_K = 8, ENVELOPE_N = 2048;

const DY = Y[0].length;
const labelMean = new Array(DY).fill(0), labelStd = new Array(DY).fill(0);
for (const y of Y) for (let i = 0; i < DY; i++) labelMean[i] += y[i] / Y.length;
for (const y of Y) for (let i = 0; i < DY; i++) labelStd[i] += (y[i] - labelMean[i]) ** 2 / Y.length;
for (let i = 0; i < DY; i++) labelStd[i] = Math.sqrt(labelStd[i]) || 1;
const Yn = Y.map((y) => y.map((v, i) => (v - labelMean[i]) / labelStd[i]));

// ---------- holdout split ----------
const idx = Xn.map((_, i) => i);
for (let i = idx.length - 1; i > 0; i--) {
  const j = Math.floor(rng() * (i + 1));
  [idx[i], idx[j]] = [idx[j], idx[i]];
}
const nHold = Math.floor(idx.length * HOLDOUT_FRAC);
const holdIdx = idx.slice(0, nHold);
const trainIdx = idx.slice(nHold);

const envelope = [];
for (let i = 0; i < ENVELOPE_N; i++) {
  envelope.push(Xn[trainIdx[Math.floor(rng() * trainIdx.length)]]);
}
function knnDist(xn) {
  const best = new Array(KNN_K).fill(Infinity);
  for (const p of envelope) {
    let d2 = 0;
    for (let i = 0; i < D; i++) { const d = xn[i] - p[i]; d2 += d * d; }
    if (d2 < best[KNN_K - 1]) {
      let j = KNN_K - 1;
      while (j > 0 && best[j - 1] > d2) { best[j] = best[j - 1]; j--; }
      best[j] = d2;
    }
  }
  return Math.sqrt(best[KNN_K - 1]);
}
const holdDists = holdIdx.map((i) => knnDist(Xn[i])).sort((a, b) => a - b);
const dP995 = holdDists[Math.floor(holdDists.length * 0.995)];
const knnWarn = Math.round(dP995 * 1.25 * 1000) / 1000;
console.log('holdout kNN dist p99.5: ' + dP995.toFixed(3) + '  -> knnWarn ' + knnWarn);

// ---------- train ensemble ----------
const members = [];
for (let m = 0; m < N_MEMBERS; m++) {
  const mrng = CR.makeRng(77 + m * 13);
  const net = CR.mlp.create(ARCH, mrng);
  const boot = new Array(trainIdx.length);
  for (let i = 0; i < boot.length; i++) boot[i] = trainIdx[Math.floor(mrng() * trainIdx.length)];
  const adam = CR.mlp.makeAdam(net, LR0);
  for (let epoch = 0; epoch < EPOCHS; epoch++) {
    for (let i = boot.length - 1; i > 0; i--) {
      const j = Math.floor(mrng() * (i + 1));
      [boot[i], boot[j]] = [boot[j], boot[i]];
    }
    let loss = 0;
    for (let b = 0; b < boot.length; b += BATCH) {
      const grads = CR.mlp.zeroGrads(net);
      const end = Math.min(b + BATCH, boot.length);
      for (let i = b; i < end; i++) loss += CR.mlp.backprop(net, Xn[boot[i]], Yn[boot[i]], grads);
      adam(grads, end - b);
    }
    if ((epoch + 1) % 10 === 0) {
      console.log('member ' + m + ' epoch ' + (epoch + 1) + ' train mse ' + (2 * loss / boot.length / DY).toFixed(5));
    }
  }
  members.push(net);
}

// ---------- holdout metrics + sigma threshold ----------
// The sigma threshold lives in the runtime's units: the spread of the member
// velocities v = G e, with e read straight from the feature vector.
let holdMse = 0;
const sigmas = [];
for (const i of holdIdx) {
  const outs = members.map((net) => CR.mlp.forward(net, Xn[i]));
  const meanOut = new Array(DY).fill(0);
  for (const o of outs) for (let k = 0; k < DY; k++) meanOut[k] += o[k] / outs.length;
  let err = 0;
  for (let k = 0; k < DY; k++) err += (meanOut[k] - Yn[i][k]) ** 2;
  holdMse += err / DY / holdIdx.length;

  const e = [X[i][12], X[i][13], X[i][14]];
  const vels = outs.map((o) => {
    const v = new Array(4);
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += (o[3 * r + k] * labelStd[3 * r + k] + labelMean[3 * r + k]) * e[k];
      v[r] = s;
    }
    return v;
  });
  const meanV = [0, 0, 0, 0];
  for (const v of vels) for (let k = 0; k < 4; k++) meanV[k] += v[k] / vels.length;
  let varSum = 0;
  for (const v of vels) for (let k = 0; k < 4; k++) varSum += (v[k] - meanV[k]) ** 2;
  sigmas.push(Math.sqrt(varSum / (vels.length * 4)));
}
sigmas.sort((a, b) => a - b);
const p995 = sigmas[Math.floor(sigmas.length * 0.995)];
const sigmaWarn = Math.round(p995 * 1.3 * 1000) / 1000;
console.log('holdout mse (ensemble mean): ' + holdMse.toFixed(5));
console.log('holdout sigma p99.5: ' + p995.toFixed(4) + '  -> sigmaWarn ' + sigmaWarn);

// ---------- serialize ----------
const round = (v) => Math.round(v * 1e5) / 1e5;
const blob = {
  meta: {
    date: '2026-08-31',
    version: 3,
    samples: X.length,
    episodes: N_EPISODES,
    arch: ARCH,
    members: N_MEMBERS,
    epochs: EPOCHS,
    holdoutMse: Math.round(holdMse * 1e6) / 1e6,
    note: 'ensemble regresses G = -gain*pinv(J_truth) (4x3); command is v = G e; inputs are triangulated markers + 3D target error; upright robot, gravity along the axis; episodes span flexibility ' + FLEX_MIN + '..' + FLEX_MAX,
    flexRange: [FLEX_MIN, FLEX_MAX],
  },
  inputMean: mean.map(round),
  inputStd: std.map(round),
  envelope: envelope.map((p) => p.map((v) => Math.round(v * 1e3) / 1e3)),
  knnK: KNN_K,
  knnWarn,
  labelMean: labelMean.map((v) => Math.round(v * 1e7) / 1e7),
  labelStd: labelStd.map((v) => Math.round(v * 1e7) / 1e7),
  sigmaWarn,
  members: members.map((net) => ({
    sizes: net.sizes,
    layers: net.layers.map((L) => ({ W: L.W.map(round), b: L.b.map(round), nIn: L.nIn, nOut: L.nOut })),
  })),
};
const out = path.join(__dirname, 'weights.json');
fs.writeFileSync(out, JSON.stringify(blob));
console.log('wrote ' + out + ' (' + (fs.statSync(out).size / 1024).toFixed(0) + ' kB)');
