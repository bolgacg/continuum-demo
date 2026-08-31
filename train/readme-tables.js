// Rewrites the evaluation tables in README.md from train/eval.json so the
// numbers on the page and in the README always come from the same run.
// Run: node train/readme-tables.js  (after node train/eval.js)
'use strict';
const fs = require('fs');
const path = require('path');
const ev = JSON.parse(fs.readFileSync(path.join(__dirname, 'eval.json'), 'utf8'));
const head = '| condition | controller | direct settled | direct median settle | direct steady-state | planned settled | planned median settle | planned steady-state |\n|---|---|---|---|---|---|---|---|';
const block = (set, title) => {
  const rows = ev.rows.filter((r) => r.targets === set).map((r) => '| ' + r.condition + ' | ' + r.controller + ' | ' + r.cells.join(' | ') + ' |');
  return '### ' + title + '\n\n' + head + '\n' + rows.join('\n') + '\n';
};
const out = '<!--EVAL_TABLES-->\n' + block('interior', 'Interior targets') + '\n' + block('edge', 'Edge targets') + '<!--/EVAL_TABLES-->';
const p = path.join(__dirname, '..', 'README.md');
const s = fs.readFileSync(p, 'utf8');
const a = s.indexOf('<!--EVAL_TABLES-->'), b = s.indexOf('<!--/EVAL_TABLES-->') + '<!--/EVAL_TABLES-->'.length;
if (a < 0 || b < a) throw new Error('README markers not found');
fs.writeFileSync(p, s.slice(0, a) + out + s.slice(b));
console.log('README tables rewritten from eval.json (' + ev.rows.length + ' rows, ' + ev.date + ')');
