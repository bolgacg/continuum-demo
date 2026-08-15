// App wiring: two truth-sim instances (same seed, same disturbances), one per
// controller, a shared clicked target, trial metrics, charts and the
// scripted demo sequence.
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, learned, render, chart } = CR;

  const W = 460, H = 345;
  const DT = 1 / 60;
  const SEED = 2026;
  const Q0 = [0.5, 0.1, -0.35, 0.3];
  const SETTLE_PX = 6, SETTLE_HOLD = 0.8, TRIAL_S = 6;
  const FAN_HORIZON = 0.35; // seconds of predicted motion shown by the fan

  const $ = (id) => document.getElementById(id);
  const cam = camera.defaultCamera(W, H);

  // ---- reachable-set outline (ideal model), for the dashed HUD polygon ----
  function computeHull() {
    const rng = CR.makeRng(7);
    const pts = [];
    for (let n = 0; n < 3000; n++) {
      const q = [];
      for (let i = 0; i < 2; i++) {
        const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * pcc.KMAX[i];
        q.push(k * Math.cos(a), k * Math.sin(a));
      }
      const p = cam.project(pcc.tip3(q));
      if (p) pts.push([p[0], p[1]]);
    }
    pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lower = [], upper = [];
    for (const p of pts) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
      lower.push(p);
    }
    for (let i = pts.length - 1; i >= 0; i--) {
      const p = pts[i];
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
      upper.push(p);
    }
    return lower.slice(0, -1).concat(upper.slice(0, -1));
  }
  const hull = computeHull();
  const hullCentroid = hull.reduce((a, p) => [a[0] + p[0] / hull.length, a[1] + p[1] / hull.length], [0, 0]);

  // ---- views ----
  const weights = typeof CR_WEIGHTS !== 'undefined' ? CR_WEIGHTS : null;
  const learnedCtrl = learned.createLearned(weights);

  const views = {
    classical: {
      canvas: $('feed-classical'),
      sim: truth.createTruth(SEED),
      ctrl: ibvs.createClassical(cam),
      accent: '#d95926',
      label: 'CAM 01 · CLASSICAL',
      lastOut: null,
    },
    learned: {
      canvas: $('feed-learned'),
      sim: truth.createTruth(SEED),
      ctrl: learnedCtrl,
      accent: '#3987e5',
      label: 'CAM 01 · LEARNED',
      lastOut: null,
    },
  };
  for (const v of Object.values(views)) {
    const dpr = window.devicePixelRatio || 1;
    v.canvas.width = W * dpr; v.canvas.height = H * dpr;
    v.ctx = v.canvas.getContext('2d');
    v.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  // ---- charts ----
  const charts = chart.createCharts($('chart-err'), $('chart-sigma'), $('tooltip'));
  if (learnedCtrl && learnedCtrl.sigmaWarn) charts.setSigmaThreshold(learnedCtrl.sigmaWarn);
  let lastSample = null;
  const origPush = charts.push;
  charts.push = (s) => { lastSample = s; origPush(s); };

  // ---- state ----
  let t = 0;
  let target = null;
  let payloadTarget = 0, driftOn = false;
  let trial = null;
  let trialCount = 0;
  let demo = null; // {steps, idx, tStart}
  let hadClick = false;

  function condString() {
    const c = [];
    if (payloadTarget > 0) c.push('payload');
    if (driftOn) c.push('drift');
    return c.length ? c.join('+') : 'nominal';
  }

  function startTrial(px) {
    target = px;
    trial = {
      id: ++trialCount,
      t0: t,
      cond: condString(),
      per: {
        classical: { bandEnter: null, settle: null, tail: [] },
        learned: { bandEnter: null, settle: null, tail: [] },
      },
    };
    for (const key of Object.keys(views)) setState(key, 'servoing');
  }

  function abandonTrial() { trial = null; }

  function finishTrial() {
    const row = document.createElement('tr');
    const fmt = (p) => {
      const settle = p.settle != null ? p.settle.toFixed(2) + ' s' : 'dns';
      const ss = p.tail.length
        ? (p.tail.reduce((a, b) => a + b, 0) / p.tail.length).toFixed(1) + ' px' : '–';
      return [settle, ss];
    };
    const [cs, css] = fmt(trial.per.classical);
    const [ls, lss] = fmt(trial.per.learned);
    row.innerHTML =
      '<td>' + trial.id + '</td><td class="cond">' + trial.cond + '</td>' +
      '<td>' + cs + '</td><td>' + css + '</td>' +
      '<td>' + (views.learned.ctrl ? ls : '–') + '</td><td>' + (views.learned.ctrl ? lss : '–') + '</td>';
    const body = $('trial-rows');
    const empty = $('empty-row');
    if (empty) empty.remove();
    body.insertBefore(row, body.firstChild);
    while (body.children.length > 10) body.removeChild(body.lastChild);
    for (const key of Object.keys(views)) {
      setState(key, trial.per[key].settle != null ? 'settled' : 'did not settle');
      $(key === 'classical' ? 'ro-c-settle' : 'ro-l-settle').textContent =
        trial.per[key].settle != null ? trial.per[key].settle.toFixed(2) + ' s' : 'dns';
    }
    trial = null;
  }

  function setState(key, s) {
    $(key === 'classical' ? 'ro-c-state' : 'ro-l-state').textContent = s;
  }

  // ---- per-step update ----
  function stepOnce() {
    t += DT;
    const sample = { t, errC: null, errL: null, sigma: null, ood: false };

    for (const [key, v] of Object.entries(views)) {
      // payload ramp + drift flag
      const pt = payloadTarget;
      if (v.sim.payload < pt) v.sim.payload = Math.min(pt, v.sim.payload + 1.2 * DT);
      if (v.sim.payload > pt) v.sim.payload = Math.max(pt, v.sim.payload - 1.2 * DT);
      v.sim.driftOn = driftOn;

      const markersPx = v.sim.markers3().map((p) => cam.project(p));
      const tipPx = markersPx[3];
      let err = null;
      if (target && tipPx && (key === 'classical' || v.ctrl)) {
        err = Math.hypot(tipPx[0] - target[0], tipPx[1] - target[1]);
        if (key === 'classical') {
          const qCmd = v.ctrl.step([tipPx[0], tipPx[1]], target, DT);
          v.sim.setCommand(qCmd);
        } else if (v.ctrl) {
          const out = v.ctrl.step(markersPx, target, DT, W, H);
          v.sim.setCommand(out.qCmd);
          v.lastOut = out;
          sample.sigma = out.sigma;
          sample.ood = out.ood;
        }
      }
      v.sim.step(DT);
      if (err != null) sample[key === 'classical' ? 'errC' : 'errL'] = err;

      // trial bookkeeping
      if (trial && err != null) {
        const p = trial.per[key];
        const tt = t - trial.t0;
        if (err < SETTLE_PX) {
          if (p.bandEnter == null) p.bandEnter = tt;
          if (p.settle == null && tt - p.bandEnter >= SETTLE_HOLD) p.settle = p.bandEnter;
        } else p.bandEnter = null;
        if (tt > TRIAL_S - 1) p.tail.push(err);
      }
    }

    if (trial && t - trial.t0 >= TRIAL_S) finishTrial();
    if (target) charts.push(sample);
  }

  // ---- drawing ----
  let frames = 0, fpsT = performance.now(), domTick = 0;
  function drawAll() {
    for (const [key, v] of Object.entries(views)) {
      let fan = null;
      if (key === 'learned' && v.ctrl && v.lastOut && target) {
        const J = ibvs.idealJacobianPx(v.ctrl.qCmd(), cam);
        const toPx = (vel) => {
          const a = [
            (J[0][0] * vel[0] + J[0][1] * vel[1] + J[0][2] * vel[2] + J[0][3] * vel[3]) * FAN_HORIZON,
            (J[1][0] * vel[0] + J[1][1] * vel[1] + J[1][2] * vel[2] + J[1][3] * vel[3]) * FAN_HORIZON,
          ];
          const n = Math.hypot(a[0], a[1]);
          return n > 46 ? [a[0] * 46 / n, a[1] * 46 / n] : a;
        };
        fan = { members: v.lastOut.membersV.map(toPx), mean: toPx(v.lastOut.meanV) };
      }
      render.drawFeed(v.ctx, {
        W, H, cam, sim: v.sim, accent: v.accent, target, hull,
        fan,
        ood: key === 'learned' && v.ctrl && v.lastOut ? v.lastOut.ood : false,
        oodGain: learned.OOD_GAIN,
        label: v.label + (key === 'learned' && !v.ctrl ? ' · NO WEIGHTS EMBEDDED' : ''),
        clickHint: !hadClick && !demo,
        t,
      });
    }
    charts.draw();

    if (++domTick % 6 === 0) {
      const last = target ? lastSample : null;
      if (last) {
        $('ro-c-err').textContent = last.errC != null ? last.errC.toFixed(1) + ' px' : '–';
        $('ro-l-err').textContent = last.errL != null ? last.errL.toFixed(1) + ' px' : '–';
        $('ro-l-sigma').textContent = last.sigma != null ? last.sigma.toFixed(3) : '–';
      }
    }

    frames++;
    const now = performance.now();
    if (now - fpsT > 1000) {
      $('fps').textContent = frames + ' fps';
      frames = 0; fpsT = now;
    }
  }

  // ---- main loop ----
  let prev = performance.now(), acc = 0;
  function loop(now) {
    let dtReal = (now - prev) / 1000;
    prev = now;
    if (dtReal > 0.1) dtReal = 0.1;
    acc += dtReal;
    let n = 0;
    while (acc >= DT && n < 4) { stepOnce(); runDemo(); acc -= DT; n++; }
    drawAll();
    requestAnimationFrame(loop);
  }

  // ---- interaction ----
  function canvasClick(ev, v) {
    const rect = v.canvas.getBoundingClientRect();
    const px = [
      (ev.clientX - rect.left) * (W / rect.width),
      (ev.clientY - rect.top) * (H / rect.height),
    ];
    hadClick = true;
    if (demo) stopDemo('demo stopped, you have the controls');
    startTrial(px);
  }
  for (const v of Object.values(views)) {
    v.canvas.addEventListener('click', (ev) => canvasClick(ev, v));
  }

  function toggleChanged() {
    const p = $('tgl-payload').checked, d = $('tgl-drift').checked;
    if (p !== (payloadTarget > 0)) {
      payloadTarget = p ? 1 : 0;
      charts.addEvent(t, p ? 'payload on' : 'payload off');
    }
    if (d !== driftOn) {
      driftOn = d;
      charts.addEvent(t, d ? 'drift on' : 'drift off');
    }
    if (trial && t - trial.t0 < TRIAL_S) abandonTrial();
  }
  $('tgl-payload').addEventListener('change', toggleChanged);
  $('tgl-drift').addEventListener('change', toggleChanged);

  function resetAll() {
    for (const v of Object.values(views)) {
      v.sim = truth.createTruth(SEED);
      v.sim.reset(Q0);
      if (v.ctrl) v.ctrl.reset(Q0);
      v.lastOut = null;
    }
    target = null;
    trial = null;
    charts.reset();
    $('ro-c-err').textContent = '–'; $('ro-l-err').textContent = '–';
    $('ro-c-settle').textContent = '–'; $('ro-l-settle').textContent = '–';
    $('ro-l-sigma').textContent = '–';
    setState('classical', 'idle'); setState('learned', 'idle');
  }
  $('btn-reset').addEventListener('click', () => {
    $('tgl-payload').checked = false; $('tgl-drift').checked = false;
    payloadTarget = 0; driftOn = false;
    if (demo) stopDemo('');
    resetAll();
  });

  // ---- scripted demo ----
  function targetFromQ(q) {
    const p = cam.project(pcc.tip3(q));
    return [p[0], p[1]];
  }
  const TQ = {
    a: [1.15, -0.25, 0.55, 0.4],
    b: [0.45, 0.5, -0.7, 0.35],
    c: [1.6, 0.15, 0.2, -0.5],
    d: [0.8, -0.5, 0.9, 0.6],
  };
  function oodTarget() {
    // a pixel well outside the reachable outline: push a hull vertex out from
    // the centroid by 40%
    let best = hull[0], bd = 0;
    for (const p of hull) {
      const d = Math.hypot(p[0] - hullCentroid[0], p[1] - hullCentroid[1]);
      if (d > bd) { bd = d; best = p; }
    }
    return [
      hullCentroid[0] + (best[0] - hullCentroid[0]) * 1.4,
      hullCentroid[1] + (best[1] - hullCentroid[1]) * 1.4,
    ];
  }

  function demoSteps() {
    const s = [];
    let at = 0;
    const add = (dt, fn, note) => { at += dt; s.push({ at, fn, note }); };
    add(0, () => {
      $('tgl-payload').checked = false; $('tgl-drift').checked = false;
      payloadTarget = 0; driftOn = false;
      resetAll();
    }, 'Nominal physics. Same targets for both controllers.');
    add(0.8, () => startTrial(targetFromQ(TQ.a)));
    add(6.4, () => startTrial(targetFromQ(TQ.b)));
    add(6.4, () => startTrial(targetFromQ(TQ.c)));
    add(6.6, () => {
      $('tgl-payload').checked = true; toggleChanged();
    }, 'Payload on. The ideal model does not know about sag.');
    add(1.2, () => startTrial(targetFromQ(TQ.a)));
    add(6.4, () => startTrial(targetFromQ(TQ.b)));
    add(6.4, () => startTrial(targetFromQ(TQ.d)));
    add(6.6, () => {
      $('tgl-drift').checked = true; toggleChanged();
    }, 'Tendon drift on. A slow bias neither controller was told about.');
    add(1.2, () => startTrial(targetFromQ(TQ.c)));
    add(6.4, () => startTrial(targetFromQ(TQ.a)));
    add(6.6, () => startTrial(oodTarget()),
      'A target outside the training envelope. Warn, do not bluff.');
    add(7.5, () => {
      $('tgl-payload').checked = false; $('tgl-drift').checked = false;
      toggleChanged();
    }, 'Done. The table has the numbers; click anywhere to keep playing.');
    add(0.1, () => stopDemo()); // keep the final note on screen
    return s;
  }

  function runDemo() {
    if (!demo) return;
    while (demo.idx < demo.steps.length && t - demo.tStart >= demo.steps[demo.idx].at) {
      const st = demo.steps[demo.idx++];
      st.fn();
      if (st.note) $('demo-note').textContent = '▸ ' + st.note;
    }
  }
  function stopDemo(msg) {
    demo = null;
    $('btn-demo').textContent = 'Run scripted demo';
    if (typeof msg === 'string') $('demo-note').textContent = msg;
  }
  $('btn-demo').addEventListener('click', () => {
    if (demo) { stopDemo(''); return; }
    hadClick = true;
    demo = { steps: demoSteps(), idx: 0, tStart: t };
    $('btn-demo').textContent = 'Stop demo';
  });

  // ---- go ----
  resetAll();
  // #demo starts the scripted demo immediately (handy for screen recording);
  // #demo,ff=12 additionally fast-forwards 12 s of sim time (dev hook)
  const hash = location.hash.slice(1).split(',');
  if (hash.includes('demo')) $('btn-demo').click();
  const ff = hash.find((s) => s.startsWith('ff='));
  if (ff) {
    const secs = Math.min(120, parseFloat(ff.slice(3)) || 0);
    for (let i = 0; i < secs * 60; i++) { stepOnce(); runDemo(); }
  }
  requestAnimationFrame((now) => { prev = now; requestAnimationFrame(loop); });
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
