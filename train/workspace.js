// Computes the ideal model's reachable envelope (radial envelope of the tip
// set at full curvature limits) and writes train/workspace.json for build.js
// to embed. Run: node train/workspace.js
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');

const t0 = Date.now();
const reach = CR.workspace.reachableVolume({ scale: 1.0 });
const rs = reach.r;
console.log('reach: ' + reach.nt + 'x' + reach.np + ' bins, centre ' + JSON.stringify(reach.center) +
  ', radius ' + Math.min(...rs).toFixed(3) + '..' + Math.max(...rs).toFixed(3) +
  ', margin scale ' + reach.meta.marginScale + ', inside ' + (100 * reach.meta.insideFrac).toFixed(2) + '%, ' + (Date.now() - t0) + ' ms');
// the ensemble's training population: tips at up to 90% of the limits at the
// highest flexibility it was trained on (a second, outer cage on the page)
const trainScale = 0.9 * CR.pcc.FLEX_RANGE[1];
const train = CR.workspace.reachableVolume({ scale: trainScale, seed: 4 });
console.log('train envelope (limits x' + trainScale.toFixed(2) + '): radius ' + Math.min(...train.r).toFixed(3) + '..' + Math.max(...train.r).toFixed(3));
const t1 = Date.now();
const grid = CR.workspace.reachableGrid();
let filled = 0;
for (const b of grid.bits) for (let k = 0; k < 8; k++) filled += (b >> k) & 1;
console.log('grid: ' + grid.n.join('x') + ' cells at ' + grid.res + ', ' + filled + ' reachable cells, ' + (Date.now() - t1) + ' ms');
const out = path.join(__dirname, 'workspace.json');
fs.writeFileSync(out, JSON.stringify({
  meta: { date: '2026-08-31', note: 'train: same construction for the ensemble training population (limits x1.62). reach: outer radial envelope of the reachable tip set of the ideal PCC model at full curvature limits (bins theta x phi around the centroid, hole-filled, smoothed, scaled so 99.9% of 200k samples are inside). grid: occupancy of reachable tips on a 0.06 grid from 300k samples with a 3D closing; the envelope hides the unreachable interior, the grid does not.' },
  reach: { center: reach.center, nt: reach.nt, np: reach.np, r: reach.r, meta: reach.meta },
  train: { center: train.center, nt: train.nt, np: train.np, r: train.r, meta: train.meta },
  grid: CR.workspace.gridToJSON(grid),
}));
console.log('wrote ' + out + ' (' + (fs.statSync(out).size / 1024).toFixed(0) + ' kB)');
