// App wiring (version 3): two truth-sim instances (same seed, same
// disturbances), one per controller, drawn together in two views: an orbiting
// inspector and the side sensor feed. Sensing is two fixed cameras and
// triangulation; the task is a 3D point placed by ray (click) x plane
// (height set in the side view). Both controllers sit under the same
// plan-then-track layer, which can be switched off to see the direct laws.
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, learned, scene, chart, workspace, v3 } = CR;
  const plannerMod = CR.planner;

  const W = 460, H = 345;
  const DT = 1 / 60;
  const SEED = 2026;
  const Q0 = [0.5, 0.1, -0.35, 0.3];
  const MM = 100;                 // display convention: 1 length unit = 100 mm
  const SETTLE_U = 0.05;          // 5 mm settle band
  const SETTLE_HOLD = 0.8, TRIAL_S = 6;
  const FAN_HORIZON = 0.35;       // seconds of predicted motion shown by the fan
  const PLANE_MIN = -1.5, PLANE_MAX = 1.5;

  const $ = (id) => document.getElementById(id);
  const camSide = camera.sideCamera(W, H);
  const camTop = camera.topCamera(W, H);

  // ---- controllers, planner, envelope ----
  const weights = typeof CR_WEIGHTS !== 'undefined' ? CR_WEIGHTS : null;
  const ws = (typeof CR_WORKSPACE !== 'undefined' && CR_WORKSPACE && CR_WORKSPACE.reach) ? CR_WORKSPACE : null;
  const volume = ws ? ws.reach : workspace.reachableVolume({ samples: 60000 });
  const planner = plannerMod.createPlanner();
  // membership test for the ensemble's training population: the same IK with
  // the curvature limits scaled to 90%
  const trainIK = plannerMod.createPlanner({ limitScale: 0.9, seed: 13 });
  const classicalInner = ibvs.createClassical();
  const learnedInner = learned.createLearned(weights, { targetTest: (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable });
  const mesh = workspace.envelopeMesh(volume);
  const grid = ws && ws.grid ? workspace.gridFromJSON(ws.grid) : workspace.reachableGrid({ samples: 120000 });
  let section = workspace.gridSectionSegments(grid, camera.CENTER[1]);

  const robots = {
    classical: {
      key: 'classical', sim: truth.createTruth(SEED), inner: classicalInner,
      tracked: plannerMod.createTracked(classicalInner, planner, 'classical'),
      direct: plannerMod.createDirect(classicalInner, 'classical'),
      accent: '#d95926', lastOut: null,
    },
    learned: learnedInner ? {
      key: 'learned', sim: truth.createTruth(SEED), inner: learnedInner,
      tracked: plannerMod.createTracked(learnedInner, planner, 'learned'),
      direct: plannerMod.createDirect(learnedInner, 'learned'),
      accent: '#3987e5', lastOut: null,
    } : null,
  };
  const robotList = Object.values(robots).filter(Boolean);
  let usePlan = true;
  const ctrlOf = (r) => (usePlan ? r.tracked : r.direct);

  // ---- views ----
  const views = {
    inspector: { canvas: $('view-inspector') },
    side: { canvas: $('view-side') },
  };
  for (const v of Object.values(views)) {
    const dpr = window.devicePixelRatio || 1;
    v.canvas.width = W * dpr; v.canvas.height = H * dpr;
    v.ctx = v.canvas.getContext('2d');
    v.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  const orbit = Object.assign({}, camera.PRESETS.iso);
  const orbitCam = () => camera.orbitCamera(orbit.az, orbit.el, W, H);
  function presetName() {
    const near = (p) => Math.abs(orbit.az - p.az) < 1e-6 && Math.abs(orbit.el - p.el) < 1e-6;
    if (near(camera.PRESETS.side)) return 'CAM 01 SIDE';
    if (near(camera.PRESETS.top)) return 'CAM 02 TOP';
    if (near(camera.PRESETS.iso)) return 'ISO';
    return 'az ' + ((orbit.az * 180) / Math.PI).toFixed(0) + '° el ' + ((orbit.el * 180) / Math.PI).toFixed(0) + '°';
  }

  // ---- charts ----
  const charts = chart.createCharts($('chart-err'), $('chart-sigma'), $('tooltip'));
  if (learnedInner && learnedInner.sigmaWarn) charts.setSigmaThreshold(learnedInner.sigmaWarn);
  let lastSample = null;
  const origPush = charts.push;
  charts.push = (s) => { lastSample = s; origPush(s); };

  // ---- state ----
  let t = 0;
  let target = null;          // 3D point
  let lastRay = null;         // {origin, dir} of the click that placed the target
  let rayNote = '';
  let planeY = camera.CENTER[1];
  let payloadTarget = 0, driftOn = false;
  let trial = null;
  let trialCount = 0;
  let demo = null;
  let hadClick = false;
  let planeDrag = false;
  let showTendons = false;

  function condString(reachable) {
    const c = [];
    if (payloadTarget > 0) c.push('payload');
    if (driftOn) c.push('drift');
    if (!usePlan) c.push('no plan');
    if (!reachable) c.push('beyond reach');
    return c.length ? c.join(' + ') : 'nominal';
  }

  function startTrial(p3) {
    target = p3.slice();
    for (const r of robotList) ctrlOf(r).newTarget(target);
    const ik = planner.solveIK(target, robots.classical.inner.qBelief());
    trial = {
      id: ++trialCount,
      t0: t,
      cond: condString(ik.reachable),
      per: {
        classical: { bandEnter: null, settle: null, tail: [] },
        learned: { bandEnter: null, settle: null, tail: [] },
      },
    };
    for (const r of robotList) setState(r.key, 'servoing');
  }

  function abandonTrial() { trial = null; }

  function finishTrial() {
    const row = document.createElement('tr');
    const fmt = (p) => {
      const settle = p.settle != null ? p.settle.toFixed(2) + ' s' : 'dns';
      const ss = p.tail.length
        ? (p.tail.reduce((a, b) => a + b, 0) / p.tail.length * MM).toFixed(1) + ' mm' : '–';
      return [settle, ss];
    };
    const [cs, css] = fmt(trial.per.classical);
    const [ls, lss] = fmt(trial.per.learned);
    row.innerHTML =
      '<td>' + trial.id + '</td><td class="cond">' + trial.cond + '</td>' +
      '<td>' + cs + '</td><td>' + css + '</td>' +
      '<td>' + (robots.learned ? ls : '–') + '</td><td>' + (robots.learned ? lss : '–') + '</td>';
    const body = $('trial-rows');
    const empty = $('empty-row');
    if (empty) empty.remove();
    body.insertBefore(row, body.firstChild);
    while (body.children.length > 10) body.removeChild(body.lastChild);
    for (const r of robotList) {
      const p = trial.per[r.key];
      setState(r.key, p.settle != null ? 'settled' : 'did not settle');
      $(r.key === 'classical' ? 'ro-c-settle' : 'ro-l-settle').textContent =
        p.settle != null ? p.settle.toFixed(2) + ' s' : 'dns';
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
    for (const r of robotList) {
      const pt = payloadTarget;
      if (r.sim.payload < pt) r.sim.payload = Math.min(pt, r.sim.payload + 1.2 * DT);
      if (r.sim.payload > pt) r.sim.payload = Math.max(pt, r.sim.payload - 1.2 * DT);
      r.sim.driftOn = driftOn;

      const markers = camera.senseMarkers(r.sim.markers3(), camSide, camTop);
      let err = null;
      if (target && markers) {
        err = v3.norm(v3.sub(markers[3], target));
        const out = ctrlOf(r).step(markers, target, DT);
        r.sim.setCommand(out.qCmd);
        r.lastOut = out;
        if (r.key === 'learned') { sample.sigma = out.sigma; sample.ood = out.ood; }
      }
      r.sim.step(DT);
      if (err != null) sample[r.key === 'classical' ? 'errC' : 'errL'] = err * MM;

      if (trial && err != null) {
        const p = trial.per[r.key];
        const tt = t - trial.t0;
        if (err < SETTLE_U) {
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
  function fanFor(r) {
    if (r.key !== 'learned' || !r.lastOut || !target || !r.lastOut.membersV) return null;
    const J = ibvs.idealJacobian3(ctrlOf(r).qCmd());
    const tip = r.sim.markers3()[3];
    const toPoint = (vel) => {
      const d = [0, 1, 2].map((k) => (J[k][0] * vel[0] + J[k][1] * vel[1] + J[k][2] * vel[2] + J[k][3] * vel[3]) * FAN_HORIZON);
      const n = v3.norm(d);
      return v3.add(tip, n > 0.4 ? v3.scale(d, 0.4 / n) : d);
    };
    return { members: r.lastOut.membersV.map(toPoint), mean: toPoint(r.lastOut.meanV) };
  }
  function drawAll() {
    const learnedOut = robots.learned && robots.learned.lastOut;
    const live = target && robots.classical.lastOut;
    const phase = live ? (robots.classical.lastOut.tracking ? ' · PLAN' : ' · LOOP') : '';
    const robotsDraw = robotList.map((r) => ({
      sim: r.sim, accent: r.accent, fan: fanFor(r),
      sRef: r.lastOut ? r.lastOut.sRef : null,
      tracking: r.lastOut ? r.lastOut.tracking : false,
    }));
    const common = {
      W, H, robots: robotsDraw, target, naked: showTendons, section,
      volume: { mesh, show: true },
      ood: !!(target && learnedOut && learnedOut.ood), oodGain: learned.OOD_GAIN, t,
    };
    scene.draw(views.inspector.ctx, Object.assign({}, common, {
      layerKey: 'inspector',
      cam: orbitCam(),
      plane: { y: planeY, show: false },
      sensors: [{ cam: camSide, label: 'CAM 01' }, { cam: camTop, label: 'CAM 02' }],
      label: 'INSPECTOR · ' + presetName() + phase + (showTendons ? ' · TENDONS ×' + scene.TENDON_DRAW_SCALE : ''),
      clickHint: !hadClick && !demo ? 'drag to orbit · click inside the outline to place a target' : null,
      hint: rayNote,
    }));
    scene.draw(views.side.ctx, Object.assign({}, common, {
      layerKey: 'side',
      cam: camSide,
      plane: { y: planeY, show: true, active: planeDrag },
      sensors: null,
      label: 'CAM 01 SIDE SENSOR' + phase + (showTendons ? ' · TENDONS ×' + scene.TENDON_DRAW_SCALE : ''),
      clickHint: !hadClick && !demo ? 'drag the plane to set the target height' : null,
      hint: '',
    }));
    charts.draw();

    if (++domTick % 6 === 0) {
      const last = target ? lastSample : null;
      if (last) {
        $('ro-c-err').textContent = last.errC != null ? last.errC.toFixed(1) + ' mm' : '–';
        $('ro-l-err').textContent = last.errL != null ? last.errL.toFixed(1) + ' mm' : '–';
        $('ro-l-sigma').textContent = last.sigma != null ? last.sigma.toFixed(3) : '–';
      }
      $('plane-y-val').textContent = (planeY * MM).toFixed(0) + ' mm';
    }
    frames++;
    const now = performance.now();
    if (now - fpsT > 1000) {
      $('fps').textContent = frames + ' fps';
      frames = 0; fpsT = now;
    }
  }

  // ---- main loop ----
  // dev hook: window.CR_PROF accumulates ms spent stepping and drawing
  const prof = (window.CR_PROF = { stepMs: 0, drawMs: 0, steps: 0, frames: 0 });
  let prev = performance.now(), acc = 0;
  function loop(now) {
    let dtReal = (now - prev) / 1000;
    prev = now;
    if (dtReal > 0.1) dtReal = 0.1;
    acc += dtReal;
    let n = 0;
    const t0 = performance.now();
    while (acc >= DT && n < 4) { stepOnce(); runDemo(); acc -= DT; n++; prof.steps++; }
    const t1 = performance.now();
    drawAll();
    prof.stepMs += t1 - t0; prof.drawMs += performance.now() - t1; prof.frames++;
    requestAnimationFrame(loop);
  }

  // ---- target placement: click ray x height plane ----
  function targetFromRay() {
    const hit = camera.rayPlaneY(lastRay.origin, lastRay.dir, planeY);
    if (hit) { rayNote = ''; return hit; }
    const w = v3.sub(camera.CENTER, lastRay.origin);
    const tt = Math.max(0.1, v3.dot(w, lastRay.dir));
    rayNote = 'ray nearly parallel to the plane: target placed at the nearest point to the workspace centre';
    return v3.add(lastRay.origin, v3.scale(lastRay.dir, tt));
  }
  function canvasPx(ev, cv) {
    const rect = cv.getBoundingClientRect();
    return [(ev.clientX - rect.left) * (W / rect.width), (ev.clientY - rect.top) * (H / rect.height)];
  }

  // inspector: drag orbits, click places
  {
    const cv = views.inspector.canvas;
    let down = null;
    cv.addEventListener('pointerdown', (ev) => {
      down = { x: ev.clientX, y: ev.clientY, az: orbit.az, el: orbit.el, moved: false };
      cv.setPointerCapture(ev.pointerId);
    });
    cv.addEventListener('pointermove', (ev) => {
      if (!down) return;
      const dx = ev.clientX - down.x, dy = ev.clientY - down.y;
      if (!down.moved && Math.hypot(dx, dy) < 4) return;
      down.moved = true;
      orbit.az = down.az - dx * 0.008;
      orbit.el = Math.max(-1.55, Math.min(1.55, down.el + dy * 0.008));
    });
    const up = (ev) => {
      if (!down) return;
      const wasClick = !down.moved;
      down = null;
      if (!wasClick) return;
      hadClick = true;
      if (demo) stopDemo('demo stopped, you have the controls');
      const [u, v] = canvasPx(ev, cv);
      const cam = orbitCam();
      lastRay = { origin: cam.pos, dir: cam.rayDir(u, v) };
      startTrial(targetFromRay());
    };
    cv.addEventListener('pointerup', up);
    cv.addEventListener('pointercancel', () => { down = null; });
  }

  // side feed: drag the plane
  function setPlaneY(y, live) {
    planeY = Math.max(PLANE_MIN, Math.min(PLANE_MAX, y));
    $('plane-y').value = planeY.toFixed(2);
    section = workspace.gridSectionSegments(grid, planeY);
    if (lastRay && live) {
      target = targetFromRay();
      if (trial) abandonTrial();
    }
  }
  {
    const cv = views.side.canvas;
    let moved = false;
    cv.addEventListener('pointerdown', (ev) => {
      const [, v] = canvasPx(ev, cv);
      const py = scene.planeScreenY(camSide, planeY);
      if (py != null && Math.abs(v - py) < 16) {
        planeDrag = true; moved = false;
        cv.setPointerCapture(ev.pointerId);
        hadClick = true;
        if (demo) stopDemo('demo stopped, you have the controls');
      }
    });
    cv.addEventListener('pointermove', (ev) => {
      if (!planeDrag) return;
      const [u, v] = canvasPx(ev, cv);
      const y = scene.planeYFromPixel(camSide, u, v);
      if (y != null) { setPlaneY(y, true); moved = true; }
    });
    const up = () => {
      if (!planeDrag) return;
      planeDrag = false;
      if (moved && target) startTrial(target);
    };
    cv.addEventListener('pointerup', up);
    cv.addEventListener('pointercancel', up);
  }
  $('plane-y').addEventListener('input', (ev) => setPlaneY(parseFloat(ev.target.value), true));
  $('plane-y').addEventListener('change', () => { if (target) startTrial(target); });

  $('tgl-tendons').addEventListener('change', (ev) => { showTendons = ev.target.checked; });

  // presets
  for (const b of document.querySelectorAll('button[data-preset]')) {
    b.addEventListener('click', () => {
      const p = camera.PRESETS[b.dataset.preset];
      if (p) { orbit.az = p.az; orbit.el = p.el; }
    });
  }

  // toggles
  function toggleChanged() {
    const p = $('tgl-payload').checked, d = $('tgl-drift').checked, pl = $('tgl-plan').checked;
    if (p !== (payloadTarget > 0)) {
      payloadTarget = p ? 1 : 0;
      charts.addEvent(t, p ? 'payload on' : 'payload off');
    }
    if (d !== driftOn) {
      driftOn = d;
      charts.addEvent(t, d ? 'drift on' : 'drift off');
    }
    if (pl !== usePlan) {
      usePlan = pl;
      charts.addEvent(t, pl ? 'plan on' : 'plan off');
      if (target) for (const r of robotList) ctrlOf(r).newTarget(target);
    }
    if (trial && t - trial.t0 < TRIAL_S) abandonTrial();
  }
  for (const id of ['tgl-payload', 'tgl-drift', 'tgl-plan']) $(id).addEventListener('change', toggleChanged);

  function resetAll() {
    for (const r of robotList) {
      r.sim = truth.createTruth(SEED);
      r.sim.reset(Q0);
      r.tracked.reset(Q0);
      r.direct.reset(Q0);
      r.lastOut = null;
    }
    target = null; lastRay = null; rayNote = '';
    trial = null;
    charts.reset();
    $('ro-c-err').textContent = '–'; $('ro-l-err').textContent = '–';
    $('ro-c-settle').textContent = '–'; $('ro-l-settle').textContent = '–';
    $('ro-l-sigma').textContent = '–';
    setState('classical', 'idle'); setState('learned', 'idle');
  }
  $('btn-reset').addEventListener('click', () => {
    $('tgl-payload').checked = false; $('tgl-drift').checked = false; $('tgl-plan').checked = true;
    payloadTarget = 0; driftOn = false; usePlan = true;
    if (demo) stopDemo('');
    resetAll();
  });

  // ---- scripted demo ----
  const TQ = {
    a: [1.15, -0.25, 0.55, 0.4],
    b: [0.45, 0.5, -0.7, 0.35],
    c: [1.6, 0.15, 0.2, -0.5],
    d: [0.8, -0.5, 0.9, 0.6],
    // both segments curled the same way at the curvature limit: a workspace
    // edge target the direct laws cannot reach from the rest pose
    edge: [2.2 * Math.cos(0.2), 2.2 * Math.sin(0.2), 2.6 * Math.cos(0.2), 2.6 * Math.sin(0.2)],
  };
  const T3 = (q) => pcc.tip3(q);
  // just past the straight arm's reach (tip at z = 1.8), visible in both views
  const BEYOND = [0.1, -0.1, 2.35];
  function setPreset(name) { const p = camera.PRESETS[name]; orbit.az = p.az; orbit.el = p.el; }
  function setPlan(on) { $('tgl-plan').checked = on; toggleChanged(); }

  function demoSteps() {
    const s = [];
    let at = 0;
    const add = (dt, fn, note) => { at += dt; s.push({ at, fn, note }); };
    add(0, () => {
      $('tgl-payload').checked = false; $('tgl-drift').checked = false; $('tgl-plan').checked = false;
      payloadTarget = 0; driftOn = false; usePlan = false;
      setPreset('iso');
      resetAll();
    }, 'Nominal physics, planner OFF: the direct feedback laws, from the rest pose.');
    add(0.8, () => startTrial(T3(TQ.edge)),
      'A workspace-edge target with the direct laws. The classical law commits to the wrong bending plane and stalls short of it.');
    add(6.6, () => { setPlan(true); }, 'Planner ON. Same target, same starting pose.');
    add(0.4, () => startTrial(T3(TQ.edge)));
    add(6.4, () => startTrial(T3(TQ.a)), 'Interior targets. The plan costs a little time here; the table keeps score.');
    add(6.4, () => startTrial(T3(TQ.b)));
    add(6.6, () => { $('tgl-payload').checked = true; toggleChanged(); setPreset('side'); },
      'Payload on, seen from the side sensor. The ideal model does not know about sag.');
    add(1.2, () => startTrial(T3(TQ.a)));
    add(6.4, () => startTrial(T3(TQ.d)));
    add(6.6, () => { $('tgl-drift').checked = true; toggleChanged(); },
      'Tendon drift on. A slow bias neither controller was told about.');
    add(1.2, () => startTrial(T3(TQ.c)));
    add(6.4, () => startTrial(T3(TQ.a)));
    add(6.6, () => { setPreset('iso'); startTrial(BEYOND); },
      'A target outside the reachable envelope. The plan stops at the closest reachable point; the ensemble flags what it was never shown.');
    add(7.5, () => {
      $('tgl-payload').checked = false; $('tgl-drift').checked = false;
      toggleChanged();
    }, 'Done. The table has the numbers; orbit, move the plane, click anywhere to keep playing.');
    add(0.1, () => stopDemo());
    return s;
  }

  function runDemo() {
    if (!demo) return;
    // a step may call stopDemo(), which nulls demo; re-check each iteration
    while (demo && demo.idx < demo.steps.length && t - demo.tStart >= demo.steps[demo.idx].at) {
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

  // dev hook: projected reach section in the inspector, for tests
  window.CR_DEBUG = { reachCellsPx: () => { const cam = orbitCam(); return workspace.gridSliceCells(grid, planeY).map((p) => cam.project(p)).filter(Boolean).map((p) => [p[0], p[1]]); } };

  // ---- go ----
  $('plane-y').min = PLANE_MIN; $('plane-y').max = PLANE_MAX; $('plane-y').step = 0.01;
  $('plane-y').value = planeY.toFixed(2);
  resetAll();
  const hash = location.hash.slice(1).split(',');
  if (hash.includes('demo')) $('btn-demo').click();
  const ff = hash.find((s) => s.startsWith('ff='));
  if (ff) {
    const secs = Math.min(120, parseFloat(ff.slice(3)) || 0);
    for (let i = 0; i < secs * 60; i++) { stepOnce(); runDemo(); }
  }
  requestAnimationFrame((now) => { prev = now; requestAnimationFrame(loop); });
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
