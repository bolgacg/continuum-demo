// Tiny fully-connected network: tanh hidden layers, linear output.
// Plain arrays so weights serialize straight to JSON. Includes just enough
// training machinery (backprop + Adam) for the offline training script.
(function (CR) {
  'use strict';

  function create(sizes, rng) {
    const layers = [];
    for (let l = 0; l < sizes.length - 1; l++) {
      const nIn = sizes[l], nOut = sizes[l + 1];
      const scale = Math.sqrt(1 / nIn);
      const W = new Array(nOut * nIn);
      for (let i = 0; i < W.length; i++) W[i] = rng.gauss() * scale;
      layers.push({ W, b: new Array(nOut).fill(0), nIn, nOut });
    }
    return { sizes: sizes.slice(), layers };
  }

  function forward(net, x) {
    let a = x;
    for (let l = 0; l < net.layers.length; l++) {
      const { W, b, nIn, nOut } = net.layers[l];
      const z = new Array(nOut);
      for (let i = 0; i < nOut; i++) {
        let s = b[i];
        const row = i * nIn;
        for (let j = 0; j < nIn; j++) s += W[row + j] * a[j];
        z[i] = l < net.layers.length - 1 ? Math.tanh(s) : s;
      }
      a = z;
    }
    return a;
  }

  // Forward pass keeping activations, then backprop of 0.5*||y - t||^2.
  // Accumulates into grads (same shape as net layers). Returns the loss.
  function backprop(net, x, target, grads) {
    const acts = [x];
    let a = x;
    for (let l = 0; l < net.layers.length; l++) {
      const { W, b, nIn, nOut } = net.layers[l];
      const z = new Array(nOut);
      for (let i = 0; i < nOut; i++) {
        let s = b[i];
        const row = i * nIn;
        for (let j = 0; j < nIn; j++) s += W[row + j] * a[j];
        z[i] = l < net.layers.length - 1 ? Math.tanh(s) : s;
      }
      acts.push(z);
      a = z;
    }
    const out = acts[acts.length - 1];
    let loss = 0;
    let delta = new Array(out.length);
    for (let i = 0; i < out.length; i++) {
      const d = out[i] - target[i];
      loss += 0.5 * d * d;
      delta[i] = d;
    }
    for (let l = net.layers.length - 1; l >= 0; l--) {
      const { W, nIn, nOut } = net.layers[l];
      const aPrev = acts[l];
      const g = grads[l];
      for (let i = 0; i < nOut; i++) {
        const row = i * nIn;
        g.b[i] += delta[i];
        for (let j = 0; j < nIn; j++) g.W[row + j] += delta[i] * aPrev[j];
      }
      if (l > 0) {
        const newDelta = new Array(nIn).fill(0);
        for (let j = 0; j < nIn; j++) {
          let s = 0;
          for (let i = 0; i < nOut; i++) s += W[i * nIn + j] * delta[i];
          const h = acts[l][j]; // tanh activation of layer l-1's output
          newDelta[j] = s * (1 - h * h);
        }
        delta = newDelta;
      }
    }
    return loss;
  }

  function zeroGrads(net) {
    return net.layers.map((L) => ({
      W: new Array(L.W.length).fill(0),
      b: new Array(L.b.length).fill(0),
    }));
  }

  function makeAdam(net, lr) {
    const m = zeroGrads(net), v = zeroGrads(net);
    let t = 0;
    const b1 = 0.9, b2 = 0.999, eps = 1e-8;
    return function step(grads, batchSize) {
      t++;
      const c1 = 1 - Math.pow(b1, t), c2 = 1 - Math.pow(b2, t);
      for (let l = 0; l < net.layers.length; l++) {
        const L = net.layers[l], g = grads[l], ml = m[l], vl = v[l];
        for (let k = 0; k < L.W.length; k++) {
          const gk = g.W[k] / batchSize;
          ml.W[k] = b1 * ml.W[k] + (1 - b1) * gk;
          vl.W[k] = b2 * vl.W[k] + (1 - b2) * gk * gk;
          L.W[k] -= (lr * (ml.W[k] / c1)) / (Math.sqrt(vl.W[k] / c2) + eps);
        }
        for (let k = 0; k < L.b.length; k++) {
          const gk = g.b[k] / batchSize;
          ml.b[k] = b1 * ml.b[k] + (1 - b1) * gk;
          vl.b[k] = b2 * vl.b[k] + (1 - b2) * gk * gk;
          L.b[k] -= (lr * (ml.b[k] / c1)) / (Math.sqrt(vl.b[k] / c2) + eps);
        }
      }
    };
  }

  CR.mlp = { create, forward, backprop, zeroGrads, makeAdam };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
