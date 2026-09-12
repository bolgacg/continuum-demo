// Builds the variant pages under variants/ without touching the main page.
// Run: node build-variants.js
//
// Outputs: variants/assets.js (weights + workspace, once), variants/core.js
// (the same CORE bundle as build.js), variants/kit.js (renderer, charts,
// protocol, kit, spotlight, extra charts), variants/shared.css (the main
// page's style block plus variants.css), variants/<n>.html per target,
// variants/index.html (the chooser), and .nojekyll at the repo root.
'use strict';
const fs = require('fs');
const path = require('path');

const ROOT = __dirname;
const CORE = ['math3', 'pcc', 'camera', 'truth', 'ibvs', 'features', 'mlp', 'learned', 'planner', 'workspace']
  .map((f) => path.join(ROOT, 'src', 'core', f + '.js'));
const KIT = [
  path.join(ROOT, 'src', 'ui', 'scene.js'),
  path.join(ROOT, 'src', 'ui', 'chart.js'),
  path.join(ROOT, 'src', 'variants', 'protocol.js'),
  path.join(ROOT, 'src', 'variants', 'kit.js'),
  path.join(ROOT, 'src', 'variants', 'spotlight.js'),
  path.join(ROOT, 'src', 'variants', 'charts2.js'),
];
const OUT = path.join(ROOT, 'variants');

const TARGETS = [
  { n: 1, slug: 'chapters', name: 'Chapters: one act per question, one scene per act',
    blurb: 'The three questions become three chapters, each with its own scene, one control and one chart: the reachable set and its cross-section, fixed arc lengths under a moving viewpoint, and inverse kinematics above the feedback law.' },
  { n: 2, slug: 'one-camera', name: 'The three consequences of one camera, replayed then corrected',
    blurb: 'Each act reproduces a version 1 fault inside the current simulator behind one switch, "as in version 1 | as now", and measures both states: a point seen by one camera is a line, perspective and an unmodelled taper, the bending plane a local law picks.' },
  { n: 3, slug: 'layers', name: 'Sensing, model, control: the three layers',
    blurb: 'The page is the three layers of the build, each stating what it is allowed to know: what the cameras sense, what the geometric model assumes with its four invented effects, and what the control law does with both.' },
  { n: 4, slug: 'audits', name: 'Three experiments the reader can rerun',
    blurb: 'Each answer is a claim and each act is its test with a hypothesis box and a Run button: the boundary against inverse kinematics, chord and arc length over random poses, and the planner over ten seeded targets.' },
  { n: 5, slug: 'one-scene', name: 'One scene, a three-stop walkthrough',
    blurb: 'One sticky scene, three stops; each stop pins one control and answers one question, with the walkthrough replacing the scripted demo.' },
];

function read(p) { return fs.readFileSync(p, 'utf8'); }
function bundle(files) {
  return files.map((f) => '// ---- ' + path.relative(ROOT, f) + ' ----\n' + read(f)).join('\n');
}
function jekyllCheck(text, file) {
  if (text.includes('{{') || text.includes('{%')) throw new Error(file + ' contains Liquid braces; Pages would mangle it');
}
function write(rel, text) {
  jekyllCheck(text, rel);
  const p = path.join(ROOT, rel);
  fs.mkdirSync(path.dirname(p), { recursive: true });
  fs.writeFileSync(p, text);
  return (fs.statSync(p).size / 1024).toFixed(0) + ' kB';
}

// shared bundles
const weights = read(path.join(ROOT, 'train', 'weights.json')).trim();
const workspace = read(path.join(ROOT, 'train', 'workspace.json')).trim();
const evalJson = JSON.parse(read(path.join(ROOT, 'train', 'eval.json')));
const template = read(path.join(ROOT, 'template.html'));
const style = (template.match(/<style>([\s\S]*?)<\/style>/) || [, ''])[1];
const sizes = {};
sizes['assets.js'] = write('variants/assets.js', 'const CR_WEIGHTS = ' + weights + ';\nconst CR_WORKSPACE = ' + workspace + ';\n');
sizes['core.js'] = write('variants/core.js', bundle(CORE));
sizes['kit.js'] = write('variants/kit.js', bundle(KIT));
sizes['shared.css'] = write('variants/shared.css', style + '\n' + read(path.join(ROOT, 'src', 'variants', 'variants.css')));

const HEAD = '<link rel="stylesheet" href="shared.css">\n' +
  '<script src="assets.js"></script>\n<script src="core.js"></script>\n<script src="kit.js"></script>';

// eval.json helpers for the templates
function evalRow(targets, condition, controller) {
  const r = evalJson.rows.find((x) => x.targets === targets && x.condition === condition && x.controller === controller);
  if (!r) throw new Error('no eval row ' + [targets, condition, controller].join('/'));
  return r;
}
function fill(html, name) {
  html = html.replace('<!--CR_HEAD-->', HEAD);
  // <!--EVAL:edge,nominal,classical,0--> -> one cell string
  html = html.replace(/<!--EVAL:([^,]+),([^,]+),([^,]+),(\d)-->/g, (m, t, c, k, i) => evalRow(t, c, k).cells[Number(i)]);
  // <!--EVAL_ROWS:targets=edge,condition=nominal--> -> table rows
  html = html.replace(/<!--EVAL_ROWS(?::([^>]*))?-->/g, (m, filt) => {
    const f = {};
    (filt || '').split(',').filter(Boolean).forEach((kv) => { const [k, v] = kv.split('='); f[k.trim()] = v.trim(); });
    return evalJson.rows.filter((r) => Object.keys(f).every((k) => r[k] === f[k])).map((r) => {
      const c = r.cells;
      return '<tr><td>' + r.targets + '</td><td>' + r.condition + '</td><td>' + r.controller + '</td>' +
        '<td>' + c[0] + '</td><td>' + c[1] + '</td><td>' + c[2] + '</td>' +
        '<td class="hi">' + c[3] + '</td><td class="hi">' + c[4] + '</td><td class="hi">' + c[5] + '</td></tr>';
    }).join('\n');
  });
  html = html.replace(/<!--BUILD_STAMP-->/g, new Date().toISOString().slice(0, 10) + ', generated by build-variants.js');
  html = html.replace(/<!--EVAL_TRIALS-->/g, String(evalJson.trials));
  html = html.replace(/<!--EVAL_DATE-->/g, evalJson.date);
  return html;
}

// per-variant pages
const built = [];
for (const t of TARGETS) {
  const tpl = path.join(OUT, 'src', t.n + '.template.html');
  const ui = path.join(ROOT, 'src', 'variants', 'v' + t.n + '.js');
  if (!fs.existsSync(tpl) || !fs.existsSync(ui)) { console.log('variant ' + t.n + ': template or ui missing, skipped'); continue; }
  let html = fill(read(tpl), t.n + '.html');
  html = html.replace('/*__UI__*/', read(ui));
  sizes[t.n + '.html'] = write('variants/' + t.n + '.html', html);
  built.push(t);
}

// chooser
const idxTpl = path.join(OUT, 'src', 'index.template.html');
if (fs.existsSync(idxTpl)) {
  let html = read(idxTpl).replace('<!--CR_HEAD-->', '<link rel="stylesheet" href="shared.css">');
  const sections = TARGETS.map((t) => {
    const ready = built.includes(t);
    return '<section class="variant" id="v' + t.n + '" data-n="' + t.n + '">' +
      '<div class="vhead"><span class="vn">Variant ' + t.n + '</span><h2>' + t.name + '</h2>' +
      (ready ? '<a href="' + t.n + '.html">open on its own</a><button class="cr-tour-btn" data-tour="' + t.n + '">Walkthrough</button>' : '<span class="vn">not built yet</span>') + '</div>' +
      '<p class="blurb">' + t.blurb + '</p>' +
      (ready ? '<iframe src="' + t.n + '.html" title="Variant ' + t.n + ': ' + t.name + '" loading="eager"></iframe><div class="vstatus" data-status="' + t.n + '">loading</div>' : '') +
      '</section>';
  }).join('\n');
  html = html.replace('<!--VARIANT_SECTIONS-->', sections);
  html = html.replace('<!--VARIANT_NAV-->', TARGETS.map((t) => '<a href="#v' + t.n + '">' + t.n + ' · ' + t.name.split(':')[0] + '</a>').join(''));
  html = html.replace('/*__UI__*/', read(path.join(ROOT, 'src', 'variants', 'chooser.js')));
  html = html.replace(/<!--BUILD_STAMP-->/g, new Date().toISOString().slice(0, 10));
  sizes['index.html'] = write('variants/index.html', html);
}

const nojekyll = path.join(ROOT, '.nojekyll');
if (!fs.existsSync(nojekyll)) fs.writeFileSync(nojekyll, '');

console.log('variants built: ' + Object.entries(sizes).map(([k, v]) => k + ' ' + v).join(', '));
console.log('main page untouched: build-variants.js never writes index.html, demo.html or template.html');
