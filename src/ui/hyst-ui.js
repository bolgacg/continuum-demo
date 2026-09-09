// Hysteresis act wiring: a private simulator tracking two alternating
// targets under the classical planned law, the camera-only identifier, the
// inverse-play compensator, and the in-browser paired evaluation. Fully
// separate from the main app's robots; shares only the core modules.
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, hyst, v3 } = CR;
  const plannerMod = CR.planner;

  const $ = (id) => document.getElementById(id);
  if (!$('hy-view')) return; // page without the act (v1 frame etc.)

  const W = 460, H = 345, DT = 1 / 60, MM = 100;
  const ORANGE = '#d95926';
  const SEED = 4242;
  const Q0 = [0.5, 0.1, -0.35, 0.3];
  // Two fixed targets on opposite sides of the side camera's image (the
  // robot bends in +y then -y), both inside the reach.
  const TARGETS = [pcc.tip3([0.05, 0.8, 0, 0.55]), pcc.tip3([0.05, -0.6, 0, -0.75])];
  const SWITCH_S = 4.0;

  const camSide = camera.sideCamera(W, H);
  const camTop = camera.topCamera(W, H);
  const CAMS = { side: { cam: camSide, name: 'cam 01 side' }, top: { cam: camTop, name: 'cam 02 top' } };
  let viewCamKey = 'side';
  const planner = plannerMod.createPlanner();

  // ---- state ----
  let sim = truth.createTruth(SEED);
  let ctrl = plannerMod.createTracked(ibvs.createClassical(), planner, 'classical');
  let trueW = 0.035;
  let comp = null;          // active compensator when the switch is on
  let wEst = null;          // last identification result (curvature units)
  let mode = 'track';       // 'track' | 'identify'
  let ident = null;
  let lastLoop = null;      // frozen loop points from the last identification
  let tgtIdx = 0, tSwitch = 0, tAct = 0;
  let errBuf = [];          // {t, err}
  let events = [];          // {t, label}
  let evalJob = null;

  function resetSims() {
    sim = truth.createTruth(SEED);
    sim.backlashK = trueW;
    sim.reset(Q0);
    ctrl = plannerMod.createTracked(ibvs.createClassical(), planner, 'classical');
    ctrl.reset(Q0);
    if (comp) { comp.reset(); sim.tendonFilter = comp.filter; }
    tgtIdx = 0; tSwitch = 0;
    ctrl.newTarget(TARGETS[0]);
  }
  resetSims();

  // ---- canvases ----
  const cvView = $('hy-view'), cvLoop = $('hy-loop'), cvErr = $('hy-err');
  const dpr = window.devicePixelRatio || 1;
  for (const c of [cvView, cvLoop]) {
    c.width = W * dpr; c.height = H * dpr;
    c.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  const EW = 944, EH = 170;
  cvErr.width = EW * dpr; cvErr.height = EH * dpr;
  cvErr.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
  const vctx = cvView.getContext('2d'), lctx = cvLoop.getContext('2d'), ectx = cvErr.getContext('2d');

  function label(ctx, text) {
    ctx.font = '600 10px ui-monospace, Menlo, monospace';
    ctx.fillStyle = '#8a8f88';
    ctx.fillText(text.toUpperCase(), 10, 18);
  }

  function drawView() {
    const vc = CAMS[viewCamKey].cam;
    vctx.fillStyle = '#151614';
    vctx.fillRect(0, 0, W, H);
    const s = mode === 'identify' && ident ? ident.sim : sim;
    // targets (track mode)
    if (mode === 'track') {
      for (let i = 0; i < TARGETS.length; i++) {
        const p = vc.project(TARGETS[i]);
        if (!p) continue;
        const active = i === tgtIdx;
        vctx.strokeStyle = active ? '#e8eae6' : 'rgba(138,143,136,0.45)';
        vctx.lineWidth = active ? 1.6 : 1;
        const r = 6;
        vctx.beginPath();
        vctx.moveTo(p[0] - r, p[1]); vctx.lineTo(p[0] + r, p[1]);
        vctx.moveTo(p[0], p[1] - r); vctx.lineTo(p[0], p[1] + r);
        vctx.stroke();
      }
    }
    // backbone
    const bb = s.backbone(14).map((p) => vc.project(p)).filter(Boolean);
    vctx.strokeStyle = ORANGE;
    vctx.lineWidth = 5;
    vctx.lineJoin = 'round';
    vctx.lineCap = 'round';
    vctx.beginPath();
    bb.forEach((p, i) => (i ? vctx.lineTo(p[0], p[1]) : vctx.moveTo(p[0], p[1])));
    vctx.stroke();
    // markers: hollow at midpoints, filled at segment ends
    const mk = s.markers3().map((p) => vc.project(p));
    mk.forEach((p, i) => {
      if (!p) return;
      const hollow = i === 0 || i === 2;
      vctx.beginPath();
      vctx.arc(p[0], p[1], 4, 0, 2 * Math.PI);
      if (hollow) { vctx.strokeStyle = '#e8eae6'; vctx.lineWidth = 1.6; vctx.stroke(); }
      else { vctx.fillStyle = '#e8eae6'; vctx.fill(); }
    });
    const modeTxt = mode === 'identify'
      ? 'identifying · sweep ' + (ident ? (ident.phase() + 1) : 1) + ' of 2'
      : 'tracking · compensation ' + (sim.tendonFilter ? 'on' : 'off');
    label(vctx, CAMS[viewCamKey].name + ' · ' + modeTxt);
  }

  function drawLoop() {
    lctx.fillStyle = '#151614';
    lctx.fillRect(0, 0, W, H);
    const pts = mode === 'identify' && ident ? ident.loop : lastLoop;
    label(lctx, mode === 'identify' ? 'command vs tip · live' : 'command vs tip · last sweep');
    if (!pts || pts.length < 2) {
      lctx.font = '12px system-ui, sans-serif';
      lctx.fillStyle = '#8a8f88';
      lctx.fillText('Click "Identify from the cameras" to draw the loop.', 10, H / 2);
      return;
    }
    let kMin = Infinity, kMax = -Infinity, mMin = Infinity, mMax = -Infinity;
    for (const p of pts) {
      if (!p) continue;
      const [k, m] = p;
      if (k < kMin) kMin = k; if (k > kMax) kMax = k;
      if (m < mMin) mMin = m; if (m > mMax) mMax = m;
    }
    const padX = 46, padY = 26;
    const sx = (k) => padX + ((k - kMin) / (kMax - kMin || 1)) * (W - padX - 14);
    const sy = (m) => H - padY - ((m - mMin) / (mMax - mMin || 1)) * (H - padY - 30);
    lctx.strokeStyle = 'rgba(138,143,136,0.35)';
    lctx.lineWidth = 1;
    lctx.strokeRect(padX, 30, W - padX - 14, H - padY - 30);
    lctx.strokeStyle = ORANGE;
    lctx.lineWidth = 1.4;
    lctx.beginPath();
    let pen = false;
    for (const p of pts) {
      if (!p) { pen = false; continue; }
      const x = sx(p[0]), y = sy(p[1]);
      if (pen) lctx.lineTo(x, y); else { lctx.moveTo(x, y); pen = true; }
    }
    lctx.stroke();
    lctx.font = '10px ui-monospace, Menlo, monospace';
    lctx.fillStyle = '#8a8f88';
    lctx.fillText('commanded curvature k1y', W / 2 - 70, H - 8);
    lctx.save();
    lctx.translate(12, H / 2 + 34);
    lctx.rotate(-Math.PI / 2);
    lctx.fillText('tip y, mm', 0, 0);
    lctx.restore();
    lctx.fillText(kMin.toFixed(2), padX - 4, H - padY + 12);
    lctx.fillText(kMax.toFixed(2), W - 40, H - padY + 12);
    lctx.fillText((mMin * MM).toFixed(0), padX - 34, H - padY);
    lctx.fillText((mMax * MM).toFixed(0), padX - 34, 38);
  }

  const ERR_WINDOW = 24;
  function drawErr() {
    ectx.fillStyle = '#fcfcfb';
    ectx.fillRect(0, 0, EW, EH);
    const t1 = tAct, t0 = Math.max(0, t1 - ERR_WINDOW);
    while (errBuf.length && errBuf[0].t < t0) errBuf.shift();
    while (events.length && events[0].t < t0) events.shift();
    // log axis: the switch transients are hundreds of mm, the compensated
    // floor is hundredths; a linear axis can only show one of them.
    const E_LO = 0.02, E_HI = 400;
    const padL = 44, padB = 20, padT = 8;
    const sx = (t) => padL + ((t - t0) / ERR_WINDOW) * (EW - padL - 8);
    const sy = (e) => {
      const c = Math.min(E_HI, Math.max(E_LO, e));
      const f = (Math.log10(c) - Math.log10(E_LO)) / (Math.log10(E_HI) - Math.log10(E_LO));
      return EH - padB - f * (EH - padB - padT);
    };
    ectx.font = '10px ui-monospace, Menlo, monospace';
    for (const g of [0.1, 1, 10, 100]) {
      ectx.strokeStyle = '#eceae4';
      ectx.lineWidth = 1;
      ectx.beginPath(); ectx.moveTo(padL, sy(g)); ectx.lineTo(EW - 8, sy(g)); ectx.stroke();
      ectx.fillStyle = '#898781';
      ectx.fillText(g < 1 ? g.toFixed(1) : String(g), padL - (g >= 100 ? 26 : g >= 10 ? 20 : 14), sy(g) + 3);
    }
    ectx.strokeStyle = '#c9c8c0';
    ectx.beginPath(); ectx.moveTo(padL, sy(5)); ectx.lineTo(EW - 8, sy(5)); ectx.stroke();
    ectx.fillText('5 mm', EW - 42, sy(5) - 4);
    // events: staggered rows, labels skipped when they would collide
    let lastLabelX = -1e9, rowFlip = false;
    for (const ev of events) {
      const x = sx(ev.t);
      ectx.strokeStyle = '#c9c8c0';
      ectx.beginPath(); ectx.moveTo(x, padT); ectx.lineTo(x, EH - padB); ectx.stroke();
      if (x - lastLabelX > 56) {
        ectx.fillStyle = '#898781';
        ectx.fillText(ev.label, x + 3, padT + (rowFlip ? 20 : 9));
        lastLabelX = x; rowFlip = !rowFlip;
      }
    }
    // series
    ectx.strokeStyle = ORANGE;
    ectx.lineWidth = 1.4;
    ectx.beginPath();
    let started = false;
    for (const s of errBuf) {
      const x = sx(s.t), y = sy(s.err);
      if (!started) { ectx.moveTo(x, y); started = true; } else ectx.lineTo(x, y);
    }
    ectx.stroke();
  }

  // ---- stepping ----
  function stepTrack(dt) {
    tAct += dt;
    tSwitch += dt;
    if (tSwitch >= SWITCH_S) {
      tSwitch = 0;
      tgtIdx = 1 - tgtIdx;
      ctrl.newTarget(TARGETS[tgtIdx]);
    }
    const markers = camera.senseMarkers(sim.markers3(), camSide, camTop);
    if (!markers) return;
    const target = TARGETS[tgtIdx];
    const err = v3.norm(v3.sub(markers[3], target)) * MM;
    const out = ctrl.step(markers, target, dt);
    sim.setCommand(out.qCmd);
    sim.step(dt);
    errBuf.push({ t: tAct, err });
  }

  let actVisible = true;
  if (typeof IntersectionObserver === 'function') {
    const io = new IntersectionObserver((entries) => {
      actVisible = entries.some((e) => e.isIntersecting);
    }, { rootMargin: '200px' });
    io.observe(cvView); io.observe(cvErr);
  }

  let prev = performance.now(), acc = 0;
  function loop(now) {
    if (!actVisible) { prev = now; acc = 0; requestAnimationFrame(loop); return; }
    let dtReal = (now - prev) / 1000;
    prev = now;
    if (dtReal > 0.1) dtReal = 0.1;
    acc += dtReal;
    let n = 0;
    while (acc >= DT && n < 4) {
      if (mode === 'track') stepTrack(DT);
      else if (ident) {
        for (let k = 0; k < 6 && !ident.done; k++) ident.step(DT); // 6x fast-forward
        if (ident.done) identDone();
      }
      acc -= DT; n++;
    }
    drawView();
    drawLoop();
    drawErr();
    requestAnimationFrame(loop);
  }

  // ---- identification flow ----
  function identDone() {
    const r = ident.result;
    wEst = r.wEstK;
    lastLoop = ident.loop.slice();
    mode = 'track';
    $('hy-est').textContent = 'est ' + wEst.toFixed(3) + ' · sim ' + trueW.toFixed(3);
    $('hy-loop-cap').textContent =
      'dead ' + r.deadMeans.map((d) => d.toFixed(3)).join(' / ') +
      ' at ' + r.speeds.join(' / ') + ' 1/s; per-tendon w ' + wEst.toFixed(3);
    $('hy-comp').disabled = false;
    $('hy-west').disabled = false;
    $('hy-west').value = wEst.toFixed(3);
    $('hy-west-val').textContent = wEst.toFixed(3) + ' (the estimate)';
    $('hy-eval').disabled = false;
    $('hy-note').textContent = 'Identified. Turn Compensation on and watch the error chart, or run the paired evaluation.';
    $('hy-view-cap').textContent = 'classical law, planner on, alternating between two fixed targets';
    resetSims();
    ident = null;
  }

  $('hy-identify').addEventListener('click', () => {
    if (mode === 'identify') return;
    mode = 'identify';
    ident = hyst.createIdentifier({ trueWidthK: trueW, seed: 4141, W, H });
    $('hy-note').textContent = 'Sweeping at two speeds; the estimator sees only the commanded values and the triangulated tip.';
    $('hy-view-cap').textContent = 'identification sweep on curvature channel k1x, drift off';
    events.push({ t: tAct, label: 'identify' });
  });

  // ---- controls ----
  $('hy-true').addEventListener('input', () => {
    trueW = parseFloat($('hy-true').value);
    $('hy-true-val').textContent = trueW.toFixed(3);
    sim.backlashK = trueW;
    events.push({ t: tAct, label: 'sim w ' + trueW.toFixed(3) });
    if (wEst != null) $('hy-est').textContent = 'est ' + wEst.toFixed(3) + ' · sim ' + trueW.toFixed(3);
  });
  $('hy-true-val').textContent = trueW.toFixed(3);

  function applyComp() {
    const on = $('hy-comp').checked;
    const wUsed = parseFloat($('hy-west').value);
    if (on) {
      comp = hyst.createCompensator(wUsed);
      sim.tendonFilter = comp.filter;
    } else {
      comp = null;
      sim.tendonFilter = null;
    }
    events.push({ t: tAct, label: on ? 'comp on ' + wUsed.toFixed(3) : 'comp off' });
    $('hy-view-cap').textContent = 'classical law, planner on, alternating between two fixed targets';
  }
  $('hy-comp').addEventListener('change', applyComp);
  $('hy-west').addEventListener('input', () => {
    const wUsed = parseFloat($('hy-west').value);
    const near = wEst != null && Math.abs(wUsed - wEst) < 0.0015;
    $('hy-west-val').textContent = wUsed.toFixed(3) + (near ? ' (the estimate)' : (wEst != null ? ' · estimate was ' + wEst.toFixed(3) : ''));
  });
  $('hy-west').addEventListener('change', () => { if ($('hy-comp').checked) applyComp(); });

  $('hy-reset').addEventListener('click', () => {
    if (mode === 'identify') { ident = null; mode = 'track'; }
    $('hy-comp').checked = false;
    comp = null;
    errBuf = []; events = []; tAct = 0;
    $('hy-note').textContent = '';
    resetSims();
  });

  // ---- paired evaluation, chunked so the page stays alive ----
  $('hy-eval').addEventListener('click', () => {
    if (evalJob || wEst == null) return;
    const wUsed = parseFloat($('hy-west').value);
    const conds = [
      { label: 'off', wK: null, name: 'off' },
      { label: 'on', wK: wUsed, name: 'on, width used' },
      { label: 'x2', wK: 2 * wUsed, name: 'on, twice the width' },
    ];
    const targets = hyst.sampleTargets(hyst.EVAL.N, 77);
    const rows = conds.map((c) => ({ c, settled: 0, settles: [], steadies: [] }));
    const queue = [];
    for (let ti = 0; ti < targets.length; ti++)
      for (let ci = 0; ci < conds.length; ci++) queue.push([ti, ci]);
    let i = 0;
    const trueAtRun = trueW;
    evalJob = { cancelled: false };
    $('hy-eval').disabled = true;
    function runChunk() {
      const t0 = performance.now();
      while (i < queue.length && performance.now() - t0 < 30) {
        const [ti, ci] = queue[i++];
        const r = hyst.runTrial(targets[ti], conds[ci], trueAtRun, 3000 + ti, planner, W, H);
        const row = rows[ci];
        if (r.settle != null) { row.settled++; row.settles.push(r.settle); }
        if (!Number.isNaN(r.steady)) row.steadies.push(r.steady);
      }
      $('hy-note').textContent = 'Evaluating ' + i + ' of ' + queue.length + ' trials...';
      if (i < queue.length) { setTimeout(runChunk, 0); return; }
      // render
      const fmt = (row) => {
        const med = row.settles.length ? median(row.settles).toFixed(2) + ' s' : 'dns';
        const st = row.steadies.length
          ? (row.steadies.reduce((a, b) => a + b, 0) / row.steadies.length * MM).toFixed(2) + ' mm' : '–';
        return { med, st };
      };
      $('hy-eval-rows').innerHTML = rows.map((row) => {
        const f = fmt(row);
        return '<tr><td>' + row.c.name + '</td><td>' + (row.c.wK == null ? '–' : row.c.wK.toFixed(3)) + '</td>' +
          '<td class="hi">' + row.settled + '/' + targets.length + '</td><td>' + f.med + '</td><td class="hi">' + f.st + '</td></tr>';
      }).join('');
      $('hy-eval-sub').textContent =
        targets.length + ' seeded targets, 6 s each, sim half-width ' + trueAtRun.toFixed(3) +
        ', computed in this browser just now. Settle: error under 5 mm held 0.8 s. Steady state: mean error over the final second.';
      $('hy-note').textContent = 'Done. Same targets, same seeds; only the tendon filter differed between the rows.';
      $('hy-eval').disabled = false;
      evalJob = null;
    }
    function median(a) { const s = a.slice().sort((x, y) => x - y); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; }
    runChunk();
  });

  for (const b of document.querySelectorAll('.chip[data-hycam]')) {
    b.addEventListener('click', () => {
      viewCamKey = b.dataset.hycam;
      for (const x of document.querySelectorAll('.chip[data-hycam]')) x.classList.toggle('active', x === b);
    });
  }

  sim.backlashK = trueW;
  requestAnimationFrame((now) => { prev = now; requestAnimationFrame(loop); });
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
