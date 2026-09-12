// Variant 2: the three consequences of one camera, replayed then corrected.
// Every act has one control, a two-position switch "as in version 1 | as now".
(function () {
  'use strict';
  if (!document.getElementById('variant-2')) return;
  const { kit, charts2, pcc, camera, ibvs, v3, spotlight } = CR;
  const A = kit.assets();
  const { W, H, MM } = kit.CONST;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  window.CR_EMBED = kit.embed({ id: '2' });
  kit.schematic(q('[data-schematic]'));

  // a two-position switch; returns current state, calls onChange(state)
  function makeSwitch(key, onChange) {
    const box = q('[data-switch="' + key + '"]');
    let state = 'now';
    for (const b of box.querySelectorAll('button')) b.addEventListener('click', () => {
      if (b.dataset.state === state) return;
      state = b.dataset.state;
      for (const x of box.querySelectorAll('button')) x.classList.toggle('on', x === b);
      onChange(state);
    });
    return { get: () => state };
  }
  // rolling 16 s chart over a canvas
  function rolling(canvas, defs, opts) {
    const buf = [], events = [];
    const WIN = 16;
    return {
      push(t, vals) { buf.push([t, vals]); while (buf.length && buf[0][0] < t - WIN - 1) buf.shift(); while (events.length && events[0].x < t - WIN - 1) events.shift(); },
      event(t, label) { events.push({ x: t, label }); },
      clear() { buf.length = 0; events.length = 0; },
      draw() {
        if (!buf.length) return;
        const t1 = buf[buf.length - 1][0], t0 = Math.max(0, t1 - WIN);
        charts2.line(canvas, Object.assign({
          series: defs.map((d, i) => ({ name: d.name, color: d.color, dashed: d.dashed, points: buf.map(([t, v]) => [t, v[i]]) })),
          x: { min: t0, max: t0 + WIN, unit: 's', label: 'time, s' }, events,
        }, opts || {}));
      },
      finalSecond(i) { const t1 = buf.length ? buf[buf.length - 1][0] : 0; const tail = buf.filter(([t]) => t > t1 - 1).map(([, v]) => v[i]); return tail.length ? tail.reduce((a, b) => a + b, 0) / tail.length : null; },
    };
  }

  // ================= act one: a point seen by one camera is a line =================
  const a1 = kit.createScene(q('[data-scene="a1"]'), { id: 'v2-a1', views: ['inspector', 'side'], robots: ['classical'], toolbar: false, charts: false, table: 4, cages: false, clickToPlace: false });
  a1.setPlan(false);
  const T1 = kit.T3([0.95, 0.3, 0.55, 0.15]);
  const camSide = a1.sim.camSide;
  // version 1's law: resolved-rate on the 2 by 4 pixel Jacobian of one camera
  function pixelLaw(cam) {
    let qB = [0, 0, 0, 0];
    function jac(qq) {
      const h = 1e-3, J = [[0, 0, 0, 0], [0, 0, 0, 0]];
      for (let c = 0; c < 4; c++) {
        const qp = qq.slice(); qp[c] += h; const qm = qq.slice(); qm[c] -= h;
        const pp = cam.project(pcc.tip3(qp)), pm = cam.project(pcc.tip3(qm));
        if (!pp || !pm) continue;
        J[0][c] = (pp[0] - pm[0]) / (2 * h); J[1][c] = (pp[1] - pm[1]) / (2 * h);
      }
      return J;
    }
    const inner = {
      name: 'pixel', reset(q0) { qB = (q0 || [0, 0, 0, 0]).slice(); }, qBelief: () => qB.slice(),
      step(tip3, target3, dt) {
        const pt = cam.project(tip3), pg = cam.project(target3);
        if (!pt || !pg) return qB.slice();
        const e = [pt[0] - pg[0], pt[1] - pg[1]];
        // the pixel gain is the 3D gain scaled by the focal length, so the two laws move at comparable rates
        const v = ibvs.clampRate(ibvs.dlsVelocity(jac(qB), e, ibvs.GAIN), ibvs.RATE_MAX);
        for (let i = 0; i < 4; i++) qB[i] += v[i] * dt;
        qB = pcc.clampQ(qB);
        return qB.slice();
      },
    };
    return { name: 'pixel', inner, reset(q0) { inner.reset(q0); }, qBelief: () => inner.qBelief(), qCmd: () => inner.qBelief(), plan: () => null, newTarget() {},
      step(markers3, target3, dt) { return { qCmd: inner.step(markers3[3], target3, dt), sRef: target3, tracking: false, plan: null }; } };
  }
  const directNow = a1.sim.robots.classical.direct;
  const directV1 = pixelLaw(camSide);
  const rayChart = rolling(q('[data-chart="ray"]'), [{ name: 'to the point', color: '#eb6834' }, { name: 'to the ray', color: '#eb6834', dashed: true }],
    { y: { label: 'distance, mm', min: 0 }, hairline: { y: 5, label: '5 mm' } });
  const rayDir = v3.normalize(v3.sub(T1, camSide.pos));
  const distToRay = (p) => { const w = v3.sub(p, camSide.pos); const along = v3.dot(w, rayDir); return v3.norm(v3.sub(w, v3.scale(rayDir, along))); };
  let a1State = 'now';
  function rerunA1() {
    a1.sim.reset();
    a1.sim.robots.classical.direct = a1State === 'v1' ? directV1 : directNow;
    a1.sim.robots.classical.direct.reset(kit.CONST.Q0);
    rayChart.event(a1.sim.st.t, a1State === 'v1' ? 'as in version 1' : 'as now');
    a1.demoTarget(T1);
    a1.note('re-running the target from rest, ' + (a1State === 'v1' ? 'pixel law on one camera' : 'the same law on the triangulated point'));
  }
  const sw1 = makeSwitch('a1', (s) => { a1State = s; rerunA1(); });
  let a1Tick = 0;
  a1.on('draw', () => {
    const st = a1.sim.st;
    if (st.target) {
      const tip = a1.sim.robots.classical.sim.markers3()[3];
      rayChart.push(st.t, [v3.norm(v3.sub(tip, st.target)) * MM, distToRay(tip) * MM]);
      // the side camera's ray through the target, drawn on the inspector
      const ctx = a1.views.inspector.ctx, cam = a1.orbitCam();
      const a = cam.project(v3.add(camSide.pos, v3.scale(rayDir, 1.6))), b = cam.project(v3.add(camSide.pos, v3.scale(rayDir, 4.2)));
      if (a && b) { ctx.save(); ctx.strokeStyle = 'rgba(232,234,230,0.55)'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.restore(); ctx.fillStyle = '#8a8f88'; ctx.font = '10px ui-monospace, Menlo, monospace'; const txt = 'CAM 01 ray through the target'; const tw = ctx.measureText(txt).width; ctx.fillText(txt, Math.max(14, Math.min(b[0] + 6, W - 14 - tw)), Math.max(30, Math.min(b[1] + 4, H - 30))); }
    }
    if (++a1Tick % 3 === 0) rayChart.draw();
  });
  a1.sim.on('trial', () => {
    const p = rayChart.finalSecond(0), r = rayChart.finalSecond(1);
    if (p == null) return;
    if (a1State === 'v1') { setV('rayPoint', p.toFixed(1) + ' mm'); setV('rayRay', r.toFixed(1) + ' mm'); const h = q('[data-h="ray"]'); if (h) h.textContent = p.toFixed(0) + ' mm from the point, ' + r.toFixed(1) + ' mm from the ray'; }
    else setV('nowPoint', p.toFixed(1) + ' mm');
  });
  setTimeout(rerunA1, 300);

  // ================= act two: perspective =================
  const a2 = kit.createScene(q('[data-scene="a2"]'), { id: 'v2-a2', views: ['inspector'], robots: ['classical'], readouts: false, toolbar: false, charts: false, table: false, cages: false, plane: false, clickToPlace: false, preset: 'side', dist: 2.4 });
  // version 1's camera position and focal length, re-expressed in the upright frame
  const camV1 = camera.makeCamera({ pos: [2.9, 1.1, 0.5], target: camera.CENTER, up: [0, 0, 1], f: 0.78 * W, w: W, h: H });
  camV1.name = 'VERSION 1 CAMERA';
  const sweep = [kit.T3([0.9, 0.3, 0.7, -0.2]), kit.T3([-0.6, -0.5, -0.9, 0.4])];
  let sIdx = 0;
  a2.sim.on('trial', () => { sIdx = 1 - sIdx; a2.startTrial(sweep[sIdx]); });
  a2.startTrial(sweep[0]);
  const chordChart = rolling(q('[data-chart="chord"]'), [{ name: 'spacing on screen', color: '#eb6834' }, { name: 'chord in 3D', color: '#52514e' }], { y: { label: 'ratio, log', log: true, floor: 0.02 } });
  const bases = { chord: null, v1: null, now: null };
  let a2State = 'now';
  const stat = { chordMin: Infinity, chordMax: -Infinity, v1: { min: Infinity, max: -Infinity }, now: { min: Infinity, max: -Infinity } };
  const sw2 = makeSwitch('a2', (s) => {
    a2State = s;
    if (s === 'v1') a2.setInspectorCamera(camV1); else a2.setPreset('side');
    chordChart.event(a2.sim.st.t, s === 'v1' ? 'version 1 camera' : 'side sensor');
  });
  let a2Tick = 0;
  a2.on('draw', () => {
    const m = a2.sim.robots.classical.sim.markers3();
    const chord = v3.norm(v3.sub(m[0], m[1])) * MM;
    const cam = a2State === 'v1' ? camV1 : a2.sim.camSide;
    const p0 = cam.project(m[0]), p1 = cam.project(m[1]);
    const px = p0 && p1 ? Math.hypot(p0[0] - p1[0], p0[1] - p1[1]) : null;
    if (bases.chord == null) bases.chord = chord;
    if (px != null && bases[a2State] == null) bases[a2State] = Math.max(px, 0.5);
    chordChart.push(a2.sim.st.t, [px != null ? Math.max(px, 0.5) / bases[a2State] : null, chord / bases.chord]);
    stat.chordMin = Math.min(stat.chordMin, chord); stat.chordMax = Math.max(stat.chordMax, chord);
    if (px != null) { const s = stat[a2State]; s.min = Math.min(s.min, px); s.max = Math.max(s.max, px); }
    if (++a2Tick % 3 === 0) {
      chordChart.draw();
      const pct = ((stat.chordMax - stat.chordMin) / (0.5 * (stat.chordMax + stat.chordMin))) * 100;
      setV('chordPct', pct.toFixed(1));
      setV('pixOld', isFinite(stat.v1.max) ? (stat.v1.max / Math.max(stat.v1.min, 0.5)).toFixed(1) : 'not measured yet, flip the switch');
      setV('pixNew', isFinite(stat.now.max) ? (stat.now.max / Math.max(stat.now.min, 0.5)).toFixed(1) : 'not yet run');
      const h = q('[data-h="chord"]'); if (h) h.textContent = pct.toFixed(1) + ' percent against ×' + (isFinite(stat.v1.max) ? (stat.v1.max / Math.max(stat.v1.min, 0.5)).toFixed(1) : (stat.now.max / Math.max(stat.now.min, 0.5)).toFixed(1));
    }
  });

  // ================= act three: the bending plane =================
  const a3 = kit.createScene(q('[data-scene="a3"]'), { id: 'v2-a3', views: ['inspector', 'side'], toolbar: false, charts: false, table: 4, clickToPlace: false });
  const hook = kit.T3(kit.TQ.hook);
  const planeChart = rolling(q('[data-chart="plane"]'), [{ name: 'classical', color: '#eb6834' }, { name: 'learned', color: '#2a78d6' }], { y: { label: 'degrees off the solution plane', min: 0, max: 180 } });
  let a3State = 'now';
  let solPlane = null;
  function angleOff(qc) { if (solPlane == null) return null; let d = Math.atan2(qc[1], qc[0]) - solPlane; d = Math.abs(((d + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI); return (d * 180) / Math.PI; }
  function rerunA3() {
    a3.sim.reset();
    a3.setPlan(a3State === 'now');
    const ik = A.planner.solveIK(hook, kit.CONST.Q0);
    solPlane = Math.atan2(ik.q[1], ik.q[0]);
    planeChart.event(a3.sim.st.t, a3State === 'v1' ? 'planner off' : 'planner on');
    a3.demoTarget(hook);
    a3.note('re-running the target under the base, planner ' + (a3State === 'v1' ? 'off, as in version 1' : 'on'));
  }
  const sw3 = makeSwitch('a3', (s) => { a3State = s; rerunA3(); });
  let a3Tick = 0;
  a3.on('draw', () => {
    if (!a3.sim.st.target) return;
    const vals = ['classical', 'learned'].map((k) => { const r = a3.sim.robots[k]; return r ? angleOff(a3.sim.ctrlOf(r).qCmd()) : null; });
    planeChart.push(a3.sim.st.t, vals);
    if (++a3Tick % 3 === 0) planeChart.draw();
  });
  a3.sim.on('trial', (row) => {
    const ang = planeChart.finalSecond(0);
    if (a3State === 'v1' && row.classical) { setV('angC', ang != null ? ang.toFixed(0) + ' degrees' : '…'); setV('stallC', row.classical.settle != null ? 'settled' : row.classical.steady.toFixed(1) + ' mm'); }
    if (a3State === 'now' && row.classical) setV('settleNow', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'did not settle');
  });
  setTimeout(rerunA3, 500);

  // ================= model card, free play, tour =================
  q('[data-modelcard]').innerHTML = kit.modelCard();
  kit.createScene(q('[data-scene="a4"]'), { id: 'v2-free', charts: true, table: 10 });
  setInterval(() => { const el = q('[data-fps]'); if (el) el.textContent = kit.ticker().fps() + ' fps'; }, 1000);
  window.CR_TOUR = spotlight.createTour([
    { target: 'header h1', title: 'One camera, three consequences', body: 'Version 1 showed a 3D robot through one 2D camera and gave its controllers a pixel to chase. Each act below replays one consequence inside the current simulator, then switches to the correction, and measures both.' },
    { target: '[data-schematic]', title: 'The click ray', body: 'A click on a camera image picks a line in space, not a point. Version 3 turns it into a point with a second camera and a target plane; version 1 never did.', place: 'below' },
    { target: '.questions', title: 'Their questions', body: 'Three questions, one cause. The wording is quoted as asked.' },
    { target: '[data-switch="a1"]', title: 'Act one, the switch', body: 'Flip to "as in version 1" and watch the dashed line reach zero first: the controller is satisfied anywhere on the ray. Flip back and both distances reach the band.' },
    { target: '[data-chart="ray"]', title: 'Act one, the chart', body: 'The gap between the solid and dashed lines under version 1\'s rule is the depth the camera never observed.', place: 'below' },
    { target: '[data-switch="a2"]', title: 'Act two, the switch', body: 'The same sweep seen through version 1\'s oblique camera and through the side sensor. The grey 3D chord does not care which; the pixel spacing swings under both.' },
    { target: '[data-chart="chord"]', title: 'Act two, the chart', body: 'Read the ratio in the verdict: how far the on-screen spacing swings under each camera against how little the chord moves in space.', place: 'below' },
    { target: '[data-switch="a3"]', title: 'Act three, the switch', body: '"As in version 1" is the planner off. The orange line parks off the solution plane and the tip stops short; with the plan the line reaches zero before the loop starts.' },
    { target: '[data-chart="plane"]', title: 'Act three, the chart', body: 'The stall is a wrong bending plane held at the curvature limit, a hidden degree of freedom, not a slow controller.', place: 'below' },
    { target: '[data-act="3"] .verdict', title: 'The published numbers', body: 'The evaluation run behind the live traces: direct against planned over 40 edge targets per controller.', place: 'below' },
    { target: '[data-modelcard]', title: 'The model card', body: 'The blue controller is the one trained thing on this page: what it is, what it was fitted on, how it scores against the classical law.', place: 'below' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'The replays are reconstructions on the version 3 stack, the ensemble has no version 1 counterpart in act one, the truth model is invented, the cameras are perfect.', place: 'below' },
    { target: '[data-scene="a4"] .toolbar', title: 'Free play', body: 'Everything unpinned. The walkthrough can be restarted from the byline at the top.', place: 'below' },
  ], { autoOpenOnce: true, storageKey: 'cr_tour_v2', button: '[data-tour-open]' });
  window.CR_VARIANT_READY = '2';
  void sw1; void sw2; void sw3;
})();
