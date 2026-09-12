// Checks for the variants kit. Part 1: the in-browser protocol reproduces
// train/eval.json, so every number an act computes is the published number.
// Run: node test/variants.js          (about a minute)
'use strict';
const fs = require('fs');
const path = require('path');
const CR = require('../train/load.js');
require('../src/variants/protocol.js');
const { protocol } = CR;

let failures = 0;
function check(name, cond, detail) {
  if (cond) console.log('  ok  ' + name + (detail ? '  (' + detail + ')' : ''));
  else { console.log('FAIL  ' + name + (detail ? '  (' + detail + ')' : '')); failures++; }
}

const evalJson = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'eval.json'), 'utf8'));
const weights = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'weights.json'), 'utf8'));
const env = protocol.makeEnv(weights);

// --- 1. protocol reproduces two published rows cell for cell ---
for (const want of [
  { targets: 'interior', condition: 'nominal', controller: 'classical' },
  { targets: 'edge', condition: 'nominal', controller: 'learned' },
]) {
  const ref = evalJson.rows.find((r) => r.targets === want.targets && r.condition === want.condition && r.controller === want.controller);
  const cond = protocol.CONDITIONS.find((c) => c.name === want.condition);
  const js = protocol.jobs({ targetSets: [want.targets], conditions: [cond], controllers: [want.controller] });
  const t0 = Date.now();
  for (const j of js) j.result = protocol.runTrial(j, env);
  const row = protocol.reduce(js)[0];
  const same = row.cells.every((c, i) => c === ref.cells[i]);
  check('protocol reproduces eval.json row ' + want.targets + '/' + want.condition + '/' + want.controller,
    same, 'got [' + row.cells.join(', ') + '] want [' + ref.cells.join(', ') + '] in ' + ((Date.now() - t0) / 1000).toFixed(0) + ' s');
  if (want.controller === 'learned') check('ood cell matches', row.ood === ref.ood, row.ood + ' vs ' + ref.ood);
}

// --- 2. the kit's DOM-free model: planned settles the hook target, direct classical stalls ---
{
  require('../src/ui/scene.js');
  require('../src/ui/chart.js');
  require('../src/variants/kit.js');
  globalThis.CR_WEIGHTS = weights;
  globalThis.CR_WORKSPACE = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'train', 'workspace.json'), 'utf8'));
  const kit = CR.kit;
  const hook = kit.T3(kit.TQ.hook);
  const run = (plan) => {
    const sim = kit.createSim({ robots: ['classical', 'learned'] });
    sim.setPlan(plan);
    sim.startTrial(hook);
    let row = null;
    sim.on('trial', (r) => { row = r; });
    for (let i = 0; i < 6 * 60 + 2 && !row; i++) sim.step();
    return row;
  };
  const planned = run(true), direct = run(false);
  check('createSim: planned, both settle the hook target', planned && planned.classical.settle != null && planned.learned.settle != null,
    planned ? 'classical ' + planned.classical.settle + ' s, learned ' + planned.learned.settle + ' s' : 'no row');
  check('createSim: direct, the classical law stalls on the hook target', direct && direct.classical.settle == null,
    direct ? 'classical steady ' + (direct.classical.steady || 0).toFixed(1) + ' mm' : 'no row');
  const consts = fs.readFileSync(path.join(__dirname, '..', 'src', 'ui', 'main.js'), 'utf8');
  const grab = (name) => { const m = consts.match(new RegExp('\\b' + name + ' = (-?[0-9./ ]+?)[,;]')); return m ? m[1].trim() : null; };
  const same = ['SEED', 'MM', 'TRIAL_S', 'SETTLE_HOLD', 'SETTLE_U', 'FAN_HORIZON', 'PLANE_MIN', 'PLANE_MAX']
    .every((k) => grab(k) != null && Math.abs(eval(grab(k)) - kit.CONST[k]) < 1e-12);
  check('kit constants equal main.js constants', same, ['SEED', 'TRIAL_S', 'PLANE_MIN'].map((k) => k + ' ' + grab(k)).join(' '));
  const m = kit.markup({ charts: true, table: true });
  const keys = [...m.matchAll(/data-cr="([^"]+)"/g)].map((x) => x[1]);
  check('markup data-cr keys are unique', new Set(keys).size === keys.length, keys.length + ' keys');
  check('markup has no Liquid braces', !m.includes('{{') && !m.includes('{%'));
}

// --- 3. the CORE list of build-variants.js equals build.js's (once it exists) ---
{
  const bv = path.join(__dirname, '..', 'build-variants.js');
  if (fs.existsSync(bv)) {
    const a = fs.readFileSync(path.join(__dirname, '..', 'build.js'), 'utf8').match(/const CORE = \[([^\]]+)\]/)[1];
    const b = fs.readFileSync(bv, 'utf8').match(/const CORE = \[([^\]]+)\]/);
    check('build-variants CORE list equals build.js', b && b[1].replace(/\s/g, '') === a.replace(/\s/g, ''));
  }
}

console.log(failures ? failures + ' FAILURE(S)' : 'all variants checks passed');
process.exit(failures ? 1 : 0);
