// Variants kit: the version 3 scene as a factory. Ports the wiring of
// src/ui/main.js (which stays untouched) so that a page can hold several
// independent scenes, each with its own simulators, views, controls and
// charts, sharing one set of assets (weights, planner, envelope, grid).
//
//   CR.kit.assets()                 memoised shared assets for this page
//   CR.kit.createSim(opts)          DOM-free model: robots, trials, planner, plane
//   CR.kit.createScene(root, opts)  views + controls + charts around a sim
//   CR.kit.createTicker()           one requestAnimationFrame for all scenes
//   CR.kit.runTrials(spec, cb)      the eval protocol, chunked, in the browser
//   CR.kit.tooltipEl()              one shared chart tooltip per document
//   CR.kit.modelCard(meta)          the ensemble's model card as HTML
//   CR.kit.embed()                  child side of the chooser's iframe protocol
(function (CR) {
  'use strict';
  const { pcc, camera, truth, ibvs, learned, scene, chart, workspace, v3, protocol } = CR;
  const plannerMod = CR.planner;

  const CONST = {
    W: 460, H: 345, DT: 1 / 60, SEED: 2026, Q0: [0.5, 0.1, -0.35, 0.3], MM: 100,
    SETTLE_U: 0.05, SETTLE_HOLD: 0.8, TRIAL_S: 6, FAN_HORIZON: 0.35,
    PLANE_MIN: -0.95, PLANE_MAX: 2.1, THUMB: 14,
  };
  // demo targets from main.js: verified to settle under the beat each runs
  const TQ = {
    hook: [2.2 * Math.cos(0.2), 2.2 * Math.sin(0.2), 2.6 * Math.cos(0.2), 2.6 * Math.sin(0.2)],
    sweep: [1.3 * Math.cos(0.2 + Math.PI), 1.3 * Math.sin(0.2 + Math.PI), 0.9 * Math.cos(0.2 + Math.PI), 0.9 * Math.sin(0.2 + Math.PI)],
    lateral: [1.8 * Math.cos(1.77), 1.8 * Math.sin(1.77), 1.4 * Math.cos(1.77), 1.4 * Math.sin(1.77)],
    top: [0.5 * Math.cos(2.5), 0.5 * Math.sin(2.5), 0.5 * Math.cos(2.5 - Math.PI), 0.5 * Math.sin(2.5 - Math.PI)],
  };
  const BEYOND = [0.1, 0.0, 2.3];
  const T3 = (q) => pcc.tip3(q);
  const ACCENT = { classical: '#d95926', learned: '#3987e5' };

  // ---------------- shared assets ----------------
  let _assets = null;
  function assets() {
    if (_assets) return _assets;
    const weights = typeof CR_WEIGHTS !== 'undefined' ? CR_WEIGHTS : null;
    const ws = (typeof CR_WORKSPACE !== 'undefined' && CR_WORKSPACE && CR_WORKSPACE.reach) ? CR_WORKSPACE : null;
    const planner = plannerMod.createPlanner();
    const trainIK = plannerMod.createPlanner({ limitScale: 0.9 * pcc.FLEX_RANGE[1], seed: 13 });
    const targetTest = (p) => trainIK.solveIK(p, [0, 0, 0, 0]).reachable;
    const trainMesh = ws && ws.train ? workspace.envelopeMesh(ws.train) : null;
    const geoCache = new Map();
    // reach geometry at a flexibility: embedded at x1.0, computed otherwise
    function geometry(f) {
      const key = f.toFixed(2);
      if (geoCache.has(key)) return geoCache.get(key);
      let volume, grid;
      if (Math.abs(f - 1) < 1e-9 && ws) {
        volume = ws.reach; grid = workspace.gridFromJSON(ws.grid);
      } else {
        volume = workspace.reachableVolume({ samples: 100000 });
        grid = workspace.reachableGrid({ samples: 150000 });
      }
      const g = { volume, mesh: workspace.envelopeMesh(volume), grid };
      geoCache.set(key, g);
      return g;
    }
    _assets = { weights, ws, planner, trainIK, targetTest, trainMesh, geometry };
    return _assets;
  }

  function makeEmitter() {
    const map = new Map();
    return {
      on(ev, fn) { if (!map.has(ev)) map.set(ev, new Set()); map.get(ev).add(fn); return () => map.get(ev).delete(fn); },
      off(ev, fn) { if (map.has(ev)) map.get(ev).delete(fn); },
      emit(ev, arg) { if (map.has(ev)) for (const fn of map.get(ev)) fn(arg); },
    };
  }

  // ---------------- the DOM-free simulation model ----------------
  function createSim(opts) {
    const o = opts || {};
    const A = assets();
    const { W, H, DT, MM, SETTLE_U, SETTLE_HOLD, TRIAL_S, PLANE_MIN, PLANE_MAX } = CONST;
    const seed = o.seed != null ? o.seed : CONST.SEED;
    const q0 = (o.q0 || CONST.Q0).slice();
    const kinds = o.robots || ['classical', 'learned'];
    const camSide = camera.sideCamera(W, H), camTop = camera.topCamera(W, H);
    const em = makeEmitter();

    const robots = {};
    function buildRobots() {
      for (const k of Object.keys(robots)) delete robots[k];
      if (kinds.includes('classical')) {
        const inner = ibvs.createClassical();
        robots.classical = { key: 'classical', sim: truth.createTruth(seed), inner,
          tracked: plannerMod.createTracked(inner, A.planner, 'classical'),
          direct: plannerMod.createDirect(inner, 'classical'), accent: ACCENT.classical, lastOut: null };
      }
      if (kinds.includes('learned') && A.weights) {
        const inner = learned.createLearned(A.weights, { targetTest: A.targetTest });
        if (inner) robots.learned = { key: 'learned', sim: truth.createTruth(seed), inner,
          tracked: plannerMod.createTracked(inner, A.planner, 'learned'),
          direct: plannerMod.createDirect(inner, 'learned'), accent: ACCENT.learned, lastOut: null };
      }
    }
    buildRobots();
    const list = () => Object.values(robots);

    const st = {
      t: 0, target: null, lastRay: null, rayNote: '', payloadTarget: 0, driftOn: false, usePlan: true,
      trial: null, trialCount: 0, planeY: o.planeY != null ? o.planeY : 1.2, lastSample: null,
    };
    let geo = A.geometry(pcc.flex());
    let section = workspace.gridSectionSegments(geo.grid, st.planeY);

    const ctrlOf = (r) => (st.usePlan ? r.tracked : r.direct);

    function condString(reachable) {
      const c = [];
      if (st.payloadTarget > 0) c.push('payload');
      if (st.driftOn) c.push('drift');
      if (!st.usePlan) c.push('no plan');
      if (!reachable) c.push('beyond reach');
      return c.length ? c.join(' + ') : 'nominal';
    }

    function startTrial(p3) {
      st.target = p3.slice();
      for (const r of list()) ctrlOf(r).newTarget(st.target);
      const ref = robots.classical ? robots.classical.inner.qBelief() : (robots.learned ? robots.learned.inner.qCmd() : [0, 0, 0, 0]);
      const ik = A.planner.solveIK(st.target, ref);
      st.trial = { id: ++st.trialCount, t0: st.t, cond: condString(ik.reachable), reachable: ik.reachable,
        per: { classical: { bandEnter: null, settle: null, tail: [] }, learned: { bandEnter: null, settle: null, tail: [] } } };
      em.emit('target', { target: st.target.slice(), reachable: ik.reachable, trial: st.trial });
      for (const r of list()) em.emit('state', { key: r.key, state: 'servoing' });
      return st.trial;
    }
    function abandonTrial() { st.trial = null; }
    function finishTrial() {
      const tr = st.trial;
      const row = { id: tr.id, cond: tr.cond, reachable: tr.reachable };
      for (const key of ['classical', 'learned']) {
        if (!robots[key]) { row[key] = null; continue; }
        const p = tr.per[key];
        row[key] = { settle: p.settle, steady: p.tail.length ? p.tail.reduce((a, b) => a + b, 0) / p.tail.length * MM : null };
        em.emit('state', { key, state: p.settle != null ? 'settled' : 'did not settle' });
      }
      st.trial = null;
      em.emit('trial', row);
    }

    function step() {
      st.t += DT;
      const sample = { t: st.t, errC: null, errL: null, sigma: null, ood: false };
      for (const r of list()) {
        const pt = st.payloadTarget;
        if (r.sim.payload < pt) r.sim.payload = Math.min(pt, r.sim.payload + 1.2 * DT);
        if (r.sim.payload > pt) r.sim.payload = Math.max(pt, r.sim.payload - 1.2 * DT);
        r.sim.driftOn = st.driftOn;
        const markers = camera.senseMarkers(r.sim.markers3(), camSide, camTop);
        let err = null;
        if (st.target && markers) {
          err = v3.norm(v3.sub(markers[3], st.target));
          const out = ctrlOf(r).step(markers, st.target, DT);
          r.sim.setCommand(out.qCmd);
          r.lastOut = out;
          if (r.key === 'learned') { sample.sigma = out.sigma; sample.ood = out.ood; }
        }
        r.sim.step(DT);
        if (err != null) sample[r.key === 'classical' ? 'errC' : 'errL'] = err * MM;
        if (st.trial && err != null) {
          const p = st.trial.per[r.key];
          const tt = st.t - st.trial.t0;
          if (err < SETTLE_U) {
            if (p.bandEnter == null) p.bandEnter = tt;
            if (p.settle == null && tt - p.bandEnter >= SETTLE_HOLD) p.settle = p.bandEnter;
          } else p.bandEnter = null;
          if (tt > TRIAL_S - 1) p.tail.push(err);
        }
      }
      if (st.trial && st.t - st.trial.t0 >= TRIAL_S) finishTrial();
      if (st.target) { st.lastSample = sample; em.emit('sample', sample); }
      return sample;
    }

    function setPlaneY(z, live) {
      st.planeY = Math.max(PLANE_MIN, Math.min(PLANE_MAX, z));
      section = workspace.gridSectionSegments(geo.grid, st.planeY);
      if (live && st.target) {
        st.target = [st.target[0], st.target[1], st.planeY];
        if (st.trial) abandonTrial();
      }
      em.emit('plane', st.planeY);
    }
    function targetFromRay(origin, dir) {
      const hit = camera.rayPlaneZ(origin, dir, st.planeY);
      st.lastRay = { origin, dir };
      if (hit) { st.rayNote = ''; return hit; }
      const w = v3.sub(camera.CENTER, origin);
      const tt = Math.max(0.1, v3.dot(w, dir));
      st.rayNote = 'ray nearly parallel to the plane: target placed at the nearest point to the workspace centre';
      return v3.add(origin, v3.scale(dir, tt));
    }
    function event(label) { em.emit('event', { t: st.t, label }); }
    function setPlan(on) {
      if (on === st.usePlan) return;
      st.usePlan = on; event(on ? 'plan on' : 'plan off');
      if (st.target) for (const r of list()) ctrlOf(r).newTarget(st.target);
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function setPayload(v) {
      const p = v ? 1 : 0;
      if (p === st.payloadTarget) return;
      st.payloadTarget = p; event(p ? 'payload on' : 'payload off');
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function setDrift(on) {
      if (on === st.driftOn) return;
      st.driftOn = on; event(on ? 'drift on' : 'drift off');
      if (st.trial && st.t - st.trial.t0 < TRIAL_S) abandonTrial();
    }
    function refreshGeometry() {
      geo = A.geometry(pcc.flex());
      section = workspace.gridSectionSegments(geo.grid, st.planeY);
      if (st.trial) abandonTrial();
      if (st.target) for (const r of list()) ctrlOf(r).newTarget(st.target);
    }
    function reset() {
      buildRobots();
      for (const r of list()) { r.sim.reset(q0); r.tracked.reset(q0); r.direct.reset(q0); r.lastOut = null; }
      st.target = null; st.lastRay = null; st.rayNote = ''; st.trial = null; st.lastSample = null;
      em.emit('reset');
    }
    reset();

    return {
      robots, list, ctrlOf, camSide, camTop, st,
      step, startTrial, abandonTrial, setPlaneY, targetFromRay, setPlan, setPayload, setDrift,
      refreshGeometry, reset, event,
      geometry: () => geo, section: () => section,
      state: () => ({ t: st.t, target: st.target ? st.target.slice() : null, planeY: st.planeY, usePlan: st.usePlan,
        payload: st.payloadTarget, drift: st.driftOn, trial: st.trial, lastSample: st.lastSample, rayNote: st.rayNote }),
      on: em.on, off: em.off,
    };
  }

  // ---------------- ticker ----------------
  function createTicker() {
    const scenes = new Set();
    let prev = null, frames = 0, fpsT = 0, fps = 0, running = true, raf = null;
    function loop(now) {
      if (prev == null) prev = now;
      let dt = (now - prev) / 1000;
      prev = now;
      if (dt > 0.1) dt = 0.1;
      if (running) for (const s of scenes) if (s.visible && !s.paused) s.advance(dt);
      frames++;
      if (now - fpsT > 1000) { fps = frames; frames = 0; fpsT = now; }
      raf = requestAnimationFrame(loop);
    }
    raf = requestAnimationFrame(loop);
    return {
      add(s) { scenes.add(s); }, remove(s) { scenes.delete(s); },
      pause() { running = false; }, resume() { running = true; prev = null; },
      get running() { return running; }, fps: () => fps, scenes,
    };
  }
  let _ticker = null;
  const ticker = () => (_ticker || (_ticker = createTicker()));

  // ---------------- shared DOM bits ----------------
  function tooltipEl() {
    let el = document.getElementById('tooltip');
    if (!el) { el = document.createElement('div'); el.id = 'tooltip'; document.body.appendChild(el); }
    return el;
  }

  // Markup for a scene block. Every element is found by data-cr, never id.
  function markup(o) {
    const views = o.views || ['inspector', 'side'];
    const parts = [];
    if (views.length) {
      parts.push('<div class="feeds' + (views.length === 1 ? ' one' : '') + '">');
      if (views.includes('inspector')) parts.push(
        '<div class="feed-card"><div class="feed-head"><span class="feed-title">Inspector</span><span class="feed-sub">orbit view</span></div>' +
        '<div class="feed-frame"><canvas data-cr="inspector" width="460" height="345"></canvas>' +
        (o.presets === false ? '' : '<div class="hud-chips"><button class="chip" data-cr-preset="iso">Iso</button><button class="chip" data-cr-preset="side">Side</button><button class="chip" data-cr-preset="top">Top</button></div>') +
        '</div><div class="feed-cap"><span class="grow" data-cr="cap-inspector">drag to orbit · click inside the outline to aim</span></div></div>');
      if (views.includes('side')) parts.push(
        '<div class="feed-card"><div class="feed-head"><span class="feed-title">CAM 01 · side sensor</span><span class="feed-sub">what the controllers see</span></div>' +
        '<div class="feed-frame"><canvas data-cr="side" width="460" height="345"></canvas>' +
        (o.planeSlider === false ? '' : '<input type="range" data-cr="plane" class="vslider" min="0" max="345" step="0.5" value="172" aria-label="target plane height">') +
        '</div><div class="feed-cap"><span class="grow" data-cr="cap-side">drag the plane or the slider</span><span class="cr-plane-val" data-cr="plane-val">target plane z = 120 mm</span></div></div>');
      parts.push('</div>');
    }
    if (o.readouts !== false) {
      const robots = o.robots || ['classical', 'learned'];
      parts.push('<div class="readouts stats">');
      if (robots.includes('classical')) parts.push(
        '<div class="grp stat" data-cr-robot="classical" title="click to hide or show this robot"><div class="toprow"><span class="dot" style="background:var(--classical)"></span><span class="name">Classical</span><span class="state" data-cr="ro-c-state">idle</span></div>' +
        '<div class="metrics"><span><span class="lbl">error</span> <span class="val" data-cr="ro-c-err">–</span></span><span><span class="lbl">settle</span> <span class="val" data-cr="ro-c-settle">–</span></span></div></div>');
      if (robots.includes('learned')) parts.push(
        '<div class="grp stat" data-cr-robot="learned" title="click to hide or show this robot"><div class="toprow"><span class="dot" style="background:var(--learned)"></span><span class="name">Learned ensemble</span><span class="state" data-cr="ro-l-state">idle</span></div>' +
        '<div class="metrics"><span><span class="lbl">error</span> <span class="val" data-cr="ro-l-err">–</span></span><span><span class="lbl">settle</span> <span class="val" data-cr="ro-l-settle">–</span></span><span><span class="lbl">σ</span> <span class="val" data-cr="ro-l-sigma">–</span></span></div></div>');
      parts.push('</div>');
    }
    const tb = o.toolbar === false ? [] : (o.toolbar || ['flex', 'tendons', 'plan', 'payload', 'drift', 'reset']);
    if (tb.length) {
      parts.push('<div class="toolbar">');
      if (tb.includes('flex')) parts.push('<span class="tgroup"><label>Flexibility</label><input type="range" data-cr="flex" min="0.7" max="1.8" step="0.1" value="1"><span class="tval" data-cr="flex-val">×1.0</span></span>');
      const sw = [];
      if (tb.includes('tendons')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-tendons"><span class="knob"></span>Tendons</label>');
      if (tb.includes('plan')) sw.push('<label class="switch" title="On: each new target is first solved by inverse kinematics on the ideal model and the controllers track that path. Off: the feedback laws chase the target directly."><input type="checkbox" data-cr="tgl-plan" checked><span class="knob"></span>Planner</label>');
      if (tb.includes('payload')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-payload"><span class="knob"></span>Payload</label>');
      if (tb.includes('drift')) sw.push('<label class="switch"><input type="checkbox" data-cr="tgl-drift"><span class="knob"></span>Tendon drift</label>');
      if (sw.length) parts.push('<span class="tgroup">' + sw.join('') + '</span>');
      if (tb.includes('reset')) parts.push('<span class="tgroup last"><button data-cr="btn-reset">Reset</button></span>');
      parts.push('</div>');
    }
    parts.push('<div class="demo-note" data-cr="note"></div>');
    if (o.charts) {
      parts.push('<div class="panel"><div class="panel-title">Tip error over time</div><div class="panel-sub">Distance from the triangulated tip to the target. Hairline: 5 mm settle band. Lengths are shown as if the robot were 180 mm long.</div>' +
        '<div class="legend"><span><i style="background:var(--classical)"></i>classical</span><span><i style="background:var(--learned)"></i>learned</span></div><canvas data-cr="chart-err" width="944" height="200"></canvas></div>');
      parts.push('<div class="panel' + (o.charts === 'err' ? ' cr-hidden' : '') + '"><div class="panel-title">Ensemble disagreement</div><div class="panel-sub">Spread of the five networks\' commands. Shaded spans: outside the training envelope, gain reduced.</div><canvas data-cr="chart-sigma" width="944" height="80"></canvas></div>');
    }
    if (o.table) {
      parts.push('<div class="panel"><div class="panel-title">Trials</div><div class="panel-sub">One row per target, 6 s window. Settle: first time the error stays under 5 mm for 0.8 s. Steady state: mean error over the final second.</div>' +
        '<div class="table-scroll"><table><thead><tr><th>#</th><th>Condition</th><th class="col-c">classical settle</th><th class="col-c">steady state</th><th class="col-l">learned settle</th><th class="col-l">steady state</th></tr></thead>' +
        '<tbody data-cr="rows"><tr data-cr="empty"><td colspan="6">No trials yet. Click in the inspector to place a target.</td></tr></tbody></table></div></div>');
    }
    return parts.join('\n');
  }

  // ---------------- the scene ----------------
  const usedIds = new Set();
  function createScene(root, opts) {
    const o = opts || {};
    if (!o.id) throw new Error('createScene needs a unique id');
    if (usedIds.has(o.id)) throw new Error('scene id already used: ' + o.id);
    usedIds.add(o.id);
    const { W, H, DT, MM, FAN_HORIZON, THUMB, TRIAL_S } = CONST;
    const A = assets();
    if (!root.querySelector('[data-cr]')) root.innerHTML = markup(o);
    const $ = (k) => root.querySelector('[data-cr="' + k + '"]');
    const sim = o.sim || createSim(o);
    const views = {};
    for (const key of ['inspector', 'side']) {
      const cv = $(key);
      if (!cv) continue;
      const dpr = window.devicePixelRatio || 1;
      cv.width = W * dpr; cv.height = H * dpr;
      const ctx = cv.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      views[key] = { canvas: cv, ctx };
    }
    const orbit = Object.assign({}, camera.PRESETS[o.preset || 'iso']);
    let camOverride = null;
    const orbitCam = () => camOverride || camera.orbitCamera(orbit.az, orbit.el, W, H, o.dist);
    const visible = { classical: true, learned: true };
    let showTendons = !!o.tendons, showCages = o.cages !== false, showPlane = o.plane !== false;
    let hadClick = false, planeDrag = false, spinning = false, play = null;
    let charts = null;
    if (o.charts && $('chart-err') && $('chart-sigma')) {
      charts = chart.createCharts($('chart-err'), $('chart-sigma'), tooltipEl());
      if (sim.robots.learned && sim.robots.learned.inner.sigmaWarn) charts.setSigmaThreshold(sim.robots.learned.inner.sigmaWarn);
    }
    const em = makeEmitter();

    function presetName() {
      const near = (p) => Math.abs(orbit.az - p.az) < 1e-6 && Math.abs(orbit.el - p.el) < 1e-6;
      if (near(camera.PRESETS.side)) return 'CAM 01 SIDE';
      if (near(camera.PRESETS.top)) return 'CAM 02 TOP';
      if (camOverride) return camOverride.name || 'FIXED CAMERA';
      if (near(camera.PRESETS.iso)) return 'ISO';
      return 'az ' + ((orbit.az * 180) / Math.PI).toFixed(0) + '° el ' + ((orbit.el * 180) / Math.PI).toFixed(0) + '°';
    }
    function fanFor(r) {
      const st = sim.st;
      if (r.key !== 'learned' || !r.lastOut || !st.target || !r.lastOut.membersV) return null;
      const J = ibvs.idealJacobian3(sim.ctrlOf(r).qCmd());
      const tip = r.sim.markers3()[3];
      const toPoint = (vel) => {
        const d = [0, 1, 2].map((k) => (J[k][0] * vel[0] + J[k][1] * vel[1] + J[k][2] * vel[2] + J[k][3] * vel[3]) * FAN_HORIZON);
        const n = v3.norm(d);
        return v3.add(tip, n > 0.4 ? v3.scale(d, 0.4 / n) : d);
      };
      return { members: r.lastOut.membersV.map(toPoint), mean: toPoint(r.lastOut.meanV) };
    }
    function drawAll() {
      const st = sim.st;
      const geo = sim.geometry();
      const learnedOut = sim.robots.learned && sim.robots.learned.lastOut;
      const first = sim.list()[0];
      const live = st.target && first && first.lastOut;
      const phase = live ? (first.lastOut.tracking ? ' · PLAN' : ' · LOOP') : '';
      const robotsDraw = sim.list().filter((r) => visible[r.key]).map((r) => ({
        sim: r.sim, accent: r.accent, fan: fanFor(r),
        sRef: r.lastOut ? r.lastOut.sRef : null, tracking: r.lastOut ? r.lastOut.tracking : false,
      }));
      const common = {
        W, H, robots: robotsDraw, target: st.target, naked: showTendons, section: showPlane ? sim.section() : null,
        volume: { mesh: geo.mesh, show: showCages }, trainVolume: { mesh: showCages ? A.trainMesh : null },
        ood: !!(st.target && learnedOut && learnedOut.ood), oodGain: learned.OOD_GAIN, t: st.t,
      };
      const tendonTag = showTendons ? ' · TENDONS ×' + scene.TENDON_DRAW_SCALE : '';
      if (views.inspector) scene.draw(views.inspector.ctx, Object.assign({}, common, {
        layerKey: o.id + '/inspector', cam: orbitCam(), plane: { y: st.planeY, show: false },
        sensors: o.sensors ? [{ cam: sim.camSide, label: 'CAM 01' }, { cam: sim.camTop, label: 'CAM 02' }] : null,
        label: 'INSPECTOR · ' + presetName() + phase + tendonTag,
        clickHint: !hadClick && !play && o.clickToPlace !== false ? (o.clickHint || 'drag to orbit · click inside the outline to place a target') : null,
        hint: st.rayNote,
      }));
      if (views.side) scene.draw(views.side.ctx, Object.assign({}, common, {
        layerKey: o.id + '/side', cam: sim.camSide, plane: { y: st.planeY, show: showPlane, active: planeDrag }, sensors: null,
        label: 'CAM 01 SIDE SENSOR' + phase + tendonTag,
        clickHint: !hadClick && !play && showPlane && o.planeSlider !== false ? 'drag the plane to set the target height' : null, hint: '',
      }));
      if (charts) charts.draw();
      const last = st.target ? st.lastSample : null;
      const set = (k, v) => { const el = $(k); if (el) el.textContent = v; };
      if (last) {
        set('ro-c-err', last.errC != null ? last.errC.toFixed(1) + ' mm' : '–');
        set('ro-l-err', last.errL != null ? last.errL.toFixed(1) + ' mm' : '–');
        set('ro-l-sigma', last.sigma != null ? last.sigma.toFixed(3) : '–');
      }
      set('plane-val', 'target plane z = ' + (st.planeY * MM).toFixed(0) + ' mm');
      for (const b of root.querySelectorAll('.chip[data-cr-preset]')) {
        const p = camera.PRESETS[b.dataset.crPreset];
        b.classList.toggle('active', Math.abs(orbit.az - p.az) < 1e-6 && Math.abs(orbit.el - p.el) < 1e-6);
      }
      em.emit('draw');
    }

    // sim events into DOM
    sim.on('sample', (s) => { if (charts) charts.push(s); });
    sim.on('event', (e) => { if (charts) charts.addEvent(e.t, e.label); });
    sim.on('state', ({ key, state }) => { const el = $(key === 'classical' ? 'ro-c-state' : 'ro-l-state'); if (el) el.textContent = state; });
    sim.on('trial', (row) => {
      const fmt = (p) => p ? [p.settle != null ? p.settle.toFixed(2) + ' s' : 'dns', p.steady != null ? p.steady.toFixed(1) + ' mm' : '–'] : ['–', '–'];
      const [cs, css] = fmt(row.classical), [ls, lss] = fmt(row.learned);
      const body = $('rows');
      if (body) {
        const empty = $('empty'); if (empty) empty.remove();
        const tr = document.createElement('tr');
        tr.innerHTML = '<td>' + row.id + '</td><td class="cond">' + row.cond + '</td><td>' + cs + '</td><td>' + css + '</td><td>' + ls + '</td><td>' + lss + '</td>';
        body.insertBefore(tr, body.firstChild);
        const max = typeof o.table === 'number' ? o.table : 10;
        while (body.children.length > max) body.removeChild(body.lastChild);
      }
      const set = (k, v) => { const el = $(k); if (el) el.textContent = v; };
      if (row.classical) set('ro-c-settle', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'dns');
      if (row.learned) set('ro-l-settle', row.learned.settle != null ? row.learned.settle.toFixed(2) + ' s' : 'dns');
      em.emit('trial', row);
    });
    sim.on('reset', () => {
      if (charts) charts.reset();
      for (const k of ['ro-c-err', 'ro-l-err', 'ro-c-settle', 'ro-l-settle', 'ro-l-sigma']) { const el = $(k); if (el) el.textContent = '–'; }
      for (const k of ['ro-c-state', 'ro-l-state']) { const el = $(k); if (el) el.textContent = 'idle'; }
    });

    // ---- plane slider (1:1 with the plane line in the side view) ----
    function sliderHcss() { const r = views.side ? views.side.canvas.getBoundingClientRect() : null; return (r && r.height) || H; }
    function syncPlaneSlider() {
      const sl = $('plane'); if (!sl || !views.side) return;
      const row = scene.planeScreenY(sim.camSide, sim.st.planeY);
      if (row == null) return;
      const Hc = sliderHcss();
      const v = ((row * Hc / H) - THUMB / 2) / (Hc - THUMB) * H;
      sl.value = Math.max(0, Math.min(H, v)).toFixed(1);
    }
    function setPlaneY(z, live) { sim.setPlaneY(z, live); syncPlaneSlider(); }
    sim.on('plane', () => syncPlaneSlider());
    function canvasPx(ev, cv) { const rect = cv.getBoundingClientRect(); return [(ev.clientX - rect.left) * (W / rect.width), (ev.clientY - rect.top) * (H / rect.height)]; }

    function startTrial(p3) { hadClick = true; return sim.startTrial(p3); }
    function demoTarget(p3) { setPlaneY(p3[2], false); return startTrial(p3); }

    if (views.inspector) {
      const cv = views.inspector.canvas;
      let down = null;
      cv.addEventListener('pointerdown', (ev) => { if (o.orbitDrag === false && o.clickToPlace === false) return; down = { x: ev.clientX, y: ev.clientY, az: orbit.az, el: orbit.el, moved: false }; cv.setPointerCapture(ev.pointerId); });
      cv.addEventListener('pointermove', (ev) => {
        if (!down || o.orbitDrag === false) return;
        const dx = ev.clientX - down.x, dy = ev.clientY - down.y;
        if (!down.moved && Math.hypot(dx, dy) < 4) return;
        down.moved = true;
        orbit.az = down.az - dx * 0.008;
        orbit.el = Math.max(-1.55, Math.min(1.55, down.el + dy * 0.008));
        em.emit('orbit', orbit);
      });
      const up = (ev) => {
        if (!down) return;
        const wasClick = !down.moved; down = null;
        if (!wasClick || o.clickToPlace === false) return;
        if (play) stop('demo stopped, you have the controls');
        const [u, v] = canvasPx(ev, cv);
        const cam = orbitCam();
        startTrial(sim.targetFromRay(cam.pos, cam.rayDir(u, v)));
      };
      cv.addEventListener('pointerup', up);
      cv.addEventListener('pointercancel', () => { down = null; });
    }
    if (views.side && o.planeSlider !== false) {
      const cv = views.side.canvas;
      let moved = false;
      cv.addEventListener('pointerdown', (ev) => {
        const [, v] = canvasPx(ev, cv);
        const py = scene.planeScreenY(sim.camSide, sim.st.planeY);
        if (py != null && Math.abs(v - py) < 16) { planeDrag = true; moved = false; cv.setPointerCapture(ev.pointerId); hadClick = true; if (play) stop('demo stopped, you have the controls'); }
      });
      cv.addEventListener('pointermove', (ev) => {
        if (!planeDrag) return;
        const [u, v] = canvasPx(ev, cv);
        const y = scene.planeYFromPixel(sim.camSide, u, v);
        if (y != null) { setPlaneY(y, true); moved = true; }
      });
      const up = () => { if (!planeDrag) return; planeDrag = false; if (moved && sim.st.target) startTrial(sim.st.target); };
      cv.addEventListener('pointerup', up);
      cv.addEventListener('pointercancel', up);
      const sl = $('plane');
      if (sl) {
        sl.min = 0; sl.max = H; sl.step = 0.5;
        sl.addEventListener('input', (ev) => {
          const Hc = sliderHcss();
          const rowCss = THUMB / 2 + (parseFloat(ev.target.value) / H) * (Hc - THUMB);
          const z = scene.planeYFromPixel(sim.camSide, W / 2, rowCss * H / Hc);
          if (z != null) setPlaneY(z, true);
        });
        sl.addEventListener('change', () => { if (sim.st.target) startTrial(sim.st.target); });
      }
    }
    for (const b of root.querySelectorAll('.chip[data-cr-preset]')) b.addEventListener('click', () => setPreset(b.dataset.crPreset));
    for (const g of root.querySelectorAll('[data-cr-robot]')) g.addEventListener('click', () => { const k = g.dataset.crRobot; visible[k] = !visible[k]; g.classList.toggle('off', !visible[k]); });
    const tglT = $('tgl-tendons'); if (tglT) tglT.addEventListener('change', (ev) => { showTendons = ev.target.checked; });
    const tglP = $('tgl-plan'); if (tglP) tglP.addEventListener('change', (ev) => sim.setPlan(ev.target.checked));
    const tglL = $('tgl-payload'); if (tglL) tglL.addEventListener('change', (ev) => sim.setPayload(ev.target.checked));
    const tglD = $('tgl-drift'); if (tglD) tglD.addEventListener('change', (ev) => sim.setDrift(ev.target.checked));
    const flexEl = $('flex');
    const flexLabel = (f) => '×' + f.toFixed(1) + ' · ' + Math.round((pcc.KMAX_BASE[0] * f * pcc.SEG_LEN[0] * 180) / Math.PI) + '°/' + Math.round((pcc.KMAX_BASE[1] * f * pcc.SEG_LEN[1] * 180) / Math.PI) + '°';
    if (flexEl) {
      flexEl.addEventListener('input', (ev) => { const el = $('flex-val'); if (el) el.textContent = flexLabel(parseFloat(ev.target.value)); });
      flexEl.addEventListener('change', (ev) => setFlex(parseFloat(ev.target.value)));
      const el = $('flex-val'); if (el) el.textContent = flexLabel(pcc.flex());
    }
    const btnReset = $('btn-reset'); if (btnReset) btnReset.addEventListener('click', () => { if (play) stop(''); reset(); });

    function setPreset(name) { camOverride = null; const p = camera.PRESETS[name]; if (p) { orbit.az = p.az; orbit.el = p.el; em.emit('orbit', orbit); } }
    function setInspectorCamera(cam) { camOverride = cam || null; }
    function setOrbit(az, el) { orbit.az = az; orbit.el = el; }
    function note(text) { const el = $('note'); if (el) el.textContent = text || ''; }
    function setToggle(k, on) { const el = $(k); if (el) el.checked = !!on; }
    function setPlan(on) { setToggle('tgl-plan', on); sim.setPlan(!!on); }
    function setPayload(on) { setToggle('tgl-payload', on); sim.setPayload(on ? 1 : 0); }
    function setDrift(on) { setToggle('tgl-drift', on); sim.setDrift(!!on); }
    function setTendons(on) { setToggle('tgl-tendons', on); showTendons = !!on; }
    function setVisible(k, on) { visible[k] = !!on; const g = root.querySelector('[data-cr-robot="' + k + '"]'); if (g) g.classList.toggle('off', !on); }
    function reset() {
      setToggle('tgl-payload', false); setToggle('tgl-drift', false); setToggle('tgl-plan', true);
      sim.st.payloadTarget = 0; sim.st.driftOn = false; sim.st.usePlan = true;
      sim.reset(); note('');
    }

    // page-wide flexibility: every scene in this context follows
    function setFlex(f) {
      note('recomputing reach for flexibility ' + flexLabel(f) + ' ...');
      return new Promise((resolve) => setTimeout(() => {
        pcc.setFlex(f);
        A.planner.rebuild();
        for (const s of ticker().scenes) {
          s.sim.refreshGeometry();
          s.syncFlexUI(f);
          if (s.charts) s.charts.addEvent(s.sim.st.t, 'flex ×' + f.toFixed(1));
        }
        scene.invalidateLayers();
        note('');
        resolve(f);
      }, 30));
    }
    function syncFlexUI(f) { if (flexEl) flexEl.value = String(f); const el = $('flex-val'); if (el) el.textContent = flexLabel(f); }

    // ---- scripted acts: [{at, fn, note}] ----
    function playSteps(steps) { play = { steps, idx: 0, tStart: sim.st.t }; hadClick = true; em.emit('play', true); }
    function runPlay() {
      if (!play) return;
      if (spinning) orbit.az -= 0.035 * DT;
      while (play && play.idx < play.steps.length && sim.st.t - play.tStart >= play.steps[play.idx].at) {
        const st = play.steps[play.idx++];
        st.fn();
        if (st.note) note('▸ ' + st.note);
      }
    }
    function stop(msg) { play = null; spinning = false; if (typeof msg === 'string') note(msg); em.emit('play', false); }

    // ---- stepping and visibility ----
    let acc = 0;
    const self = {
      id: o.id, root, sim, charts, views, orbit, visible: true, paused: false,
      robots: sim.robots,
      advance(dtReal) {
        acc += dtReal;
        let n = 0;
        while (acc >= DT && n < 4) { sim.step(); runPlay(); acc -= DT; n++; }
        drawAll();
      },
      step: () => { sim.step(); runPlay(); }, draw: drawAll,
      startTrial, demoTarget, setPlaneY, setPlan, setPayload, setDrift, setTendons, setVisible, setPreset, setOrbit,
      setCages(on) { showCages = !!on; }, setPlaneShown(on) { showPlane = !!on; }, setInspectorCamera, orbitCam,
      spin(on) { spinning = !!on; }, setFlex, syncFlexUI, reset, play: playSteps, stop, note, isPlaying: () => !!play,
      pause() { self.paused = true; }, resume() { self.paused = false; },
      on: em.on, off: em.off, $,
      destroy() { ticker().remove(self); usedIds.delete(o.id); },
    };
    if (o.autoVisibility !== false && typeof IntersectionObserver === 'function') {
      const io = new IntersectionObserver((entries) => { self.visible = entries.some((e) => e.isIntersecting); }, { rootMargin: '120px' });
      io.observe(root);
    }
    syncPlaneSlider();
    drawAll();
    ticker().add(self);
    return self;
  }

  // ---------------- in-browser evaluation ----------------
  function runTrials(spec, onProgress) {
    const A = assets();
    const env = protocol.makeEnv(A.weights, { planner: A.planner, trainIK: A.trainIK });
    const js = protocol.jobs(spec);
    let i = 0, cancelled = false;
    const budget = (spec && spec.budgetMs) || 30;
    const promise = new Promise((resolve) => {
      function chunk() {
        if (cancelled) { resolve(null); return; }
        const t0 = performance.now();
        while (i < js.length && performance.now() - t0 < budget) { js[i].result = protocol.runTrial(js[i], env); i++; }
        if (onProgress) onProgress({ done: i, total: js.length, rows: protocol.reduce(js.slice(0, i)), jobs: js });
        if (i < js.length) setTimeout(chunk, 0); else resolve({ rows: protocol.reduce(js), jobs: js });
      }
      setTimeout(chunk, 0);
    });
    return { promise, cancel: () => { cancelled = true; } };
  }

  // ---------------- the setup, drawn once with labels ----------------
  function schematic(box) {
    if (!box) return;
    const cv = box.querySelector('canvas'), svg = box.querySelector('svg');
    const W = 920, H = 690, dpr = window.devicePixelRatio || 1;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cam = camera.orbitCamera(0.75, 0.42, W, H);
    const sim = truth.createTruth(1); sim.reset([0.9, 0.25, 0.6, -0.4]);
    const target = [0.55, 0.65, 1.1];
    const g = assets().geometry(pcc.flex());
    scene.draw(ctx, { W, H, cam, robots: [{ sim, accent: ACCENT.classical }], target, plane: { y: 1.1, show: true },
      section: workspace.gridSectionSegments(g.grid, 1.1), volume: { mesh: g.mesh, show: true }, trainVolume: { mesh: null },
      sensors: [{ cam: camera.sideCamera(460, 345), label: '' }, { cam: camera.topCamera(460, 345), label: '' }],
      label: 'THE SETUP', t: 0, layerKey: 'kit/schematic' });
    const m = sim.markers3();
    const NS = 'http://www.w3.org/2000/svg';
    const lab = (p, text, dx, dy) => { const s = cam.project(p); if (!s) return; const el = document.createElementNS(NS, 'text'); el.setAttribute('x', s[0] + (dx || 8)); el.setAttribute('y', s[1] + (dy || -8)); el.textContent = text; svg.appendChild(el); };
    lab([0, 0, 0], 'base', 10, 18);
    lab(pcc.poseAt(sim.qEff(), 0, 0.3).p, 'segment 1, length 1.0', -150, 0);
    lab(pcc.poseAt(sim.qEff(), 1, 0.3).p, 'segment 2, length 0.8', 12, 0);
    ['m1 (mid)', 'm2 (end)', 'm3 (mid)', 'm4 (tip)'].forEach((s, i) => lab(m[i], s, 10, -8));
    lab(camera.sideCamera(460, 345).pos, 'CAM 01, side', -40, -12);
    lab(camera.topCamera(460, 345).pos, 'CAM 02, top', 10, -8);
    lab(target, 'target on the plane', 12, 20);
    lab([1.5, -1.5, 1.1], 'target plane, z = 110 mm', -40, -8);
    const dir = v3.normalize(v3.sub(target, cam.pos));
    const a = cam.project(v3.sub(target, v3.scale(dir, 1.4))), b = cam.project(target);
    if (a && b) { const l = document.createElementNS(NS, 'line'); l.setAttribute('x1', a[0]); l.setAttribute('y1', a[1]); l.setAttribute('x2', b[0]); l.setAttribute('y2', b[1]); l.setAttribute('stroke-dasharray', '4 4'); svg.appendChild(l); lab(v3.sub(target, v3.scale(dir, 0.9)), 'click ray', 10, -6); }
  }

  // ---------------- model card ----------------
  function modelCard(meta) {
    const m = meta || (typeof CR_WEIGHTS !== 'undefined' && CR_WEIGHTS && CR_WEIGHTS.meta) || {};
    const arch = (m.arch || []).join(', ');
    return '<dl class="cr-card">' +
      '<dt>What it is</dt><dd>' + (m.members || 5) + ' multilayer perceptrons, layer sizes ' + arch + ', an ensemble.</dd>' +
      '<dt>What it predicts</dt><dd>A 4 by 3 gain matrix G from the triangulated markers and the tip error; the command is v = G e, so it is zero at zero error.</dd>' +
      '<dt>Fitted on</dt><dd>' + (m.samples || 0).toLocaleString('en-US') + ' samples from ' + (m.episodes || 0) + ' simulated episodes across flexibility ×' + (m.flexRange ? m.flexRange[0] : '') + ' to ×' + (m.flexRange ? m.flexRange[1] : '') + ', labels from a training-time expert that reads the simulator\'s true Jacobian; ' + (m.epochs || 0) + ' epochs.</dd>' +
      '<dt>Held out</dt><dd>10 percent of the samples, holdout MSE ' + (m.holdoutMse != null ? m.holdoutMse.toFixed(3) : '–') + ' in normalised units; the two extrapolation thresholds sit at the held-out 99.5th percentile.</dd>' +
      '</dl>';
  }

  // ---------------- iframe protocol (child side) ----------------
  function embed(opts) {
    if (typeof window === 'undefined' || window === window.top) return null;
    const id = (opts && opts.id) || document.body.dataset.variant || '';
    const post = (msg) => window.parent.postMessage(Object.assign({ cr: 1, id }, msg), '*');
    const height = () => post({ type: 'height', h: document.documentElement.scrollHeight });
    window.addEventListener('message', (ev) => {
      const d = ev.data;
      if (!d || d.cr !== 1) return;
      if (d.type === 'pause') ticker().pause();
      if (d.type === 'resume') ticker().resume();
      if (d.type === 'tour' && window.CR_TOUR) { if (d.action === 'open') window.CR_TOUR.open(0); else window.CR_TOUR.close(); }
    });
    if (typeof ResizeObserver === 'function') new ResizeObserver(height).observe(document.body);
    window.addEventListener('load', height);
    setInterval(() => post({ type: 'fps', fps: ticker().fps() }), 1000);
    post({ type: 'ready' });
    height();
    return { post, scrollTo: (y) => post({ type: 'scrollTo', y }) };
  }

  CR.kit = { CONST, TQ, BEYOND, T3, ACCENT, assets, createSim, createScene, createTicker, ticker, markup, tooltipEl, runTrials, modelCard, schematic, embed };
})(typeof globalThis.CR === 'object' ? globalThis.CR : (globalThis.CR = {}));
