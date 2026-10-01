// Closed-loop check of the backlash compensation, headless and seeded.
// Identifies the play width from the two cameras, then runs the page's trial
// protocol (6 s, settle = under 5 mm held 0.8 s, steady state = mean error
// over the final second) on 20 seeded interior targets under three filters:
// none, the inverse play at the camera estimate, and at twice the estimate.
// Writes train/hyst.json, which build-variants.js reads.
// Run: node train/hyst-eval.js
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('./load.js');
const { hyst, truth } = CR;

const trueW = truth.PARAMS.backlashK;
const ident = hyst.runIdentification(trueW, 4141);
const N = 20;
const rows = hyst.runPairedEval([
  { label: 'none', wK: null },
  { label: 'estimate', wK: ident.wEstK },
  { label: 'twice', wK: 2 * ident.wEstK },
], trueW, { N });
const out = {
  date: new Date().toISOString().slice(0, 10),
  trueW, estW: ident.wEstK, N,
  rows: rows.map((r) => ({ label: r.label, settled: r.settled, n: r.n,
    settleMedianS: r.settleMedian, steadyMeanMm: r.steadyMean * 100 })),
};
fs.writeFileSync(path.join(__dirname, 'hyst.json'), JSON.stringify(out, null, 2) + '\n');
console.log(JSON.stringify(out, null, 2));
