// Computes the ideal model's reachable outline in the camera image and writes
// train/workspace.json for build.js to embed. Run: node train/workspace.js
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');

const W = 460, H = 345;
const cam = CR.camera.defaultCamera(W, H);
const t0 = Date.now();
const outline = CR.workspace.reachableOutline(cam);
const ms = Date.now() - t0;
const xs = outline.map((p) => p[0]), ys = outline.map((p) => p[1]);
console.log('outline: ' + outline.length + ' vertices, x ' + Math.min(...xs).toFixed(0) + '..' +
  Math.max(...xs).toFixed(0) + ', y ' + Math.min(...ys).toFixed(0) + '..' + Math.max(...ys).toFixed(0) +
  ' (canvas ' + W + 'x' + H + '), ' + ms + ' ms');
const out = path.join(__dirname, 'workspace.json');
fs.writeFileSync(out, JSON.stringify({
  meta: { date: '2026-08-31', samples: 200000, cell: 3, note: 'reachable tip set of the ideal PCC model at full curvature limits, projected through the demo camera; boundary traced from a 3 px occupancy grid' },
  outline: outline.map((p) => [Math.round(p[0] * 10) / 10, Math.round(p[1] * 10) / 10]),
}));
console.log('wrote ' + out);
