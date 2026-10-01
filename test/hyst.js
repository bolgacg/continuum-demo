// Headless checks for the hysteresis act: the inverse-play filter really
// inverts the play operator, the two-speed identifier lands near the true
// width across a range of widths, and the paired evaluation orders the
// conditions the way the page claims. Run: node test/hyst.js
'use strict';
const CR = require('../train/load.js');
const { pcc, truth, hyst } = CR;

let failures = 0;
function check(name, cond, detail) {
  if (cond) console.log('  ok  ' + name + (detail ? '  (' + detail + ')' : ''));
  else { console.log('FAIL  ' + name + (detail ? '  (' + detail + ')' : '')); failures++; }
}

// --- 1. inverse play against a bare play operator (no lag, no plant) ---
{
  const wK = 0.035;
  const seg = 0;
  const rL = truth.PARAMS.tendonRadius * pcc.SEG_LEN[seg];
  const w = wK * rL; // half-width in tendon units
  const comp = hyst.createCompensator(wK);
  let play = 0; // play operator state
  let maxDev = 0, maxDevRaw = 0, playRaw = 0, engaged = false;
  for (let s = 0; s <= 2000; s++) {
    const t = s / 200;
    const v = 0.03 * Math.sin(2 * Math.PI * 0.4 * t); // desired tendon displacement
    const u = comp.filter(seg * 3 + 0, v);
    // play operator, the same formula the sim uses
    if (u - play > w) play = u - w;
    else if (u - play < -w) play = u + w;
    if (playRaw < v - w) playRaw = v - w;
    else if (playRaw > v + w) playRaw = v + w;
    if (t > 0.5) { engaged = true; }
    if (engaged) {
      maxDev = Math.max(maxDev, Math.abs(play - v));
      maxDevRaw = Math.max(maxDevRaw, Math.abs(playRaw - v));
    }
  }
  check('inverse play cancels the play operator after engagement',
    maxDev < 1e-9, 'max dev ' + maxDev.toExponential(2) + ' vs uncompensated ' + maxDevRaw.toFixed(4));
  check('uncompensated play deviates by about the half-width', Math.abs(maxDevRaw - w) < 1e-6);
}

// --- 2. identification across true widths ---
{
  for (const trueW of [0.02, 0.035, 0.055]) {
    const res = hyst.runIdentification(trueW, 4141);
    const err = Math.abs(res.wEstK - trueW);
    check('identified w for true ' + trueW.toFixed(3),
      res && err < 0.006,
      res ? 'est ' + res.wEstK.toFixed(4) + ' err ' + err.toFixed(4) +
        ' deads ' + res.deadMeans.map((d) => d.toFixed(4)).join('/') +
        ' n ' + res.counts.join('/') : 'no result');
  }
}

// --- 3. paired evaluation ordering ---
{
  const trueW = 0.05; // a rough mechanism, where compensation must show
  const wEst = hyst.runIdentification(trueW, 4141).wEstK;
  const rows = hyst.runPairedEval([
    { label: 'off', wK: null },
    { label: 'on', wK: wEst },
    { label: 'x2', wK: 2 * wEst },
  ], trueW, { N: 12 });
  const [off, on, x2] = rows;
  console.log('  eval  off: settled ' + off.settled + '/' + off.n + ' steady ' + (off.steadyMean * 100).toFixed(2) + ' mm-ish');
  console.log('  eval  on : settled ' + on.settled + '/' + on.n + ' steady ' + (on.steadyMean * 100).toFixed(2));
  console.log('  eval  x2 : settled ' + x2.settled + '/' + x2.n + ' steady ' + (x2.steadyMean * 100).toFixed(2));
  check('compensation lowers mean steady-state error', on.steadyMean < off.steadyMean,
    (on.steadyMean * 100).toFixed(2) + ' vs ' + (off.steadyMean * 100).toFixed(2));
  check('compensation settles no fewer targets', on.settled >= off.settled);
  check('a doubled estimate is worse than the estimate', x2.steadyMean > on.steadyMean,
    (x2.steadyMean * 100).toFixed(2) + ' vs ' + (on.steadyMean * 100).toFixed(2));
}

// --- 4. the hook defaults leave the main demo untouched ---
{
  const a = truth.createTruth(2026); a.reset([0.5, 0.1, -0.35, 0.3]);
  const b = truth.createTruth(2026); b.reset([0.5, 0.1, -0.35, 0.3]);
  b.backlashK = null; b.tendonFilter = null;
  for (let s = 0; s < 300; s++) {
    const q = [0.5 + 0.3 * Math.sin(s / 40), 0.1, -0.35, 0.3];
    a.setCommand(q); b.setCommand(q);
    a.step(1 / 60); b.step(1 / 60);
  }
  const qa = a.qEff(), qb = b.qEff();
  check('null hooks change nothing', qa.every((v, i) => v === qb[i]));
}

// --- 5. the open-loop sweep the page draws ---
{
  const w = truth.PARAMS.backlashK;
  const est = hyst.runIdentification(w, 4141).wEstK;
  const off = hyst.sweepLoop(w, null, 0), on = hyst.sweepLoop(w, est, 0), onFast = hyst.sweepLoop(w, est, 1);
  console.log('  loop  slow off ' + (off.gapMean * 100).toFixed(2) + ' mm, slow on ' + (on.gapMean * 100).toFixed(2) +
    ' mm, fast on ' + (onFast.gapMean * 100).toFixed(2) + ' mm, bins ' + off.bins + '/' + on.bins + '/' + onFast.bins);
  check('the loop has enough paired bins to measure', off.bins >= 20 && on.bins >= 20 && onFast.bins >= 20);
  check('compensation narrows the open-loop loop', on.gapMean < 0.6 * off.gapMean);
  check('what compensation leaves grows with speed (it is lag)', onFast.gapMean > on.gapMean);
  // with the lag removed, the inverse play leaves almost nothing: the remainder really is lag
  const P = truth.PARAMS, lag = P.lagTau;
  P.lagTau = 1e-6;
  const offNL = hyst.sweepLoop(w, null, 0), onNL = hyst.sweepLoop(w, est, 0);
  P.lagTau = lag;
  console.log('  loop  no lag: off ' + (offNL.gapMean * 100).toFixed(2) + ' mm, on ' + (onNL.gapMean * 100).toFixed(2) + ' mm');
  check('without lag, compensation removes the loop to under 0.5 mm', onNL.gapMean * 100 < 0.5);
}

// --- 6. the lag, identified from a step and inverted ---
{
  const P = truth.PARAMS, lag = P.lagTau;
  for (const tau of [0.05, 0.09, 0.15]) {
    P.lagTau = tau;
    const r = hyst.identifyLag(4141);
    check('identified lag for true ' + tau, Math.abs(r.tau - tau) < 0.005, 'est ' + r.tau.toFixed(4) + ' from ' + r.n + ' frames');
  }
  P.lagTau = lag;
  const w = P.backlashK, est = hyst.runIdentification(w, 4141).wEstK, tau = hyst.identifyLag(4141).tau;
  const slow = hyst.sweepLoop(w, hyst.createLeadCompensator(est, tau).filter, 0);
  const fast = hyst.sweepLoop(w, hyst.createLeadCompensator(est, tau).filter, 1);
  console.log('  loop  play and lag inverted: slow ' + (slow.gapMean * 100).toFixed(2) + ' mm, fast ' + (fast.gapMean * 100).toFixed(2) + ' mm');
  check('inverting play and lag closes the slow loop to under 0.3 mm', slow.gapMean * 100 < 0.3);
  check('and the fast loop to under 1 mm', fast.gapMean * 100 < 1);
}

console.log(failures ? failures + ' FAILURE(S)' : 'all hysteresis checks passed');
process.exit(failures ? 1 : 0);
