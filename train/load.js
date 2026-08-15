// Loads the core modules (browser-style namespace files) into Node.
'use strict';
const path = require('path');
const FILES = ['math3', 'pcc', 'camera', 'truth', 'ibvs', 'features', 'mlp', 'learned'];
for (const f of FILES) {
  const p = path.join(__dirname, '..', 'src', 'core', f + '.js');
  try {
    require(p);
  } catch (e) {
    if (e.code === 'MODULE_NOT_FOUND' && e.message.includes(f)) continue; // learned.js may not exist yet
    throw e;
  }
}
module.exports = globalThis.CR;
