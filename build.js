// Assembles demo.html: inlines the core and UI modules plus the pretrained
// weights into template.html. Run: node build.js
'use strict';
const fs = require('fs');
const path = require('path');

const ROOT = __dirname;
const CORE = ['math3', 'pcc', 'camera', 'truth', 'ibvs', 'features', 'mlp', 'learned']
  .map((f) => path.join(ROOT, 'src', 'core', f + '.js'));
const UI = ['render', 'chart', 'main']
  .map((f) => path.join(ROOT, 'src', 'ui', f + '.js'));

function bundle(files) {
  return files
    .map((f) => '// ---- ' + path.relative(ROOT, f) + ' ----\n' + fs.readFileSync(f, 'utf8'))
    .join('\n');
}

const weightsPath = path.join(ROOT, 'train', 'weights.json');
const weights = fs.existsSync(weightsPath)
  ? fs.readFileSync(weightsPath, 'utf8').trim()
  : 'null';

let html = fs.readFileSync(path.join(ROOT, 'template.html'), 'utf8');
html = html.replace('/*__WEIGHTS__*/', 'const CR_WEIGHTS = ' + weights + ';');
html = html.replace('/*__CORE__*/', bundle(CORE));
html = html.replace('/*__UI__*/', bundle(UI));

const out = path.join(ROOT, 'demo.html');
fs.writeFileSync(out, html);
// index.html is the same page, so GitHub Pages serves it at the bare URL.
fs.writeFileSync(path.join(ROOT, 'index.html'), html);
const kb = (fs.statSync(out).size / 1024).toFixed(0);
console.log('demo.html + index.html written (' + kb + ' kB, weights ' +
  (weights === 'null' ? 'MISSING' : 'embedded') + ')');
