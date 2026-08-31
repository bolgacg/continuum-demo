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
const out = path.join(__dirname, 'workspace.json');
fs.writeFileSync(out, JSON.stringify({
  meta: { date: '2026-08-31', note: 'outer radial envelope of the reachable tip set of the ideal PCC model at full curvature limits; bins theta x phi around the centroid, hole-filled, smoothed, scaled so 99.9% of 200k samples are inside' },
  reach: { center: reach.center, nt: reach.nt, np: reach.np, r: reach.r, meta: reach.meta },
}));
console.log('wrote ' + out + ' (' + (fs.statSync(out).size / 1024).toFixed(0) + ' kB)');
