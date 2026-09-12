// Variant 5: one scene, three stops. Each stop sets the scene, pins one control, and writes
// its verdict from what the reader does with that control.
(function () {
  'use strict';
  if (!document.getElementById('variant-5')) return;
  const { kit, charts2, pcc, workspace, v3, spotlight } = CR;
  const A = kit.assets();
  const { MM } = kit.CONST;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  const setH = (k, v) => { const el = q('[data-h="' + k + '"]'); if (el) el.textContent = v; };
  window.CR_EMBED = kit.embed({ id: '5' });
  kit.schematic(q('[data-schematic]'));

  // the one scene
  const S = kit.createScene(q('[data-scene="main"]'), { id: 'v5-main', views: ['inspector', 'side'], toolbar: ['flex', 'tendons', 'plan', 'payload', 'drift', 'reset'], charts: false, table: 10 });
  const root = S.root;
  const planLabel = root.querySelector('[data-cr="tgl-plan"]') && root.querySelector('[data-cr="tgl-plan"]').closest('label');
  const hook = kit.T3(kit.TQ.hook);
  const still = kit.T3([0.9, 0.3, 0.7, -0.2]);
  let stop = null;

  function pin(which) {
    root.classList.add('pinned');
    for (const el of root.querySelectorAll('.toolbar .pin')) el.classList.remove('pin');
    if (which === 'plan' && planLabel) { planLabel.classList.add('pin'); planLabel.querySelector('input').classList.add('pin'); }
  }
  function unpin() { root.classList.remove('pinned'); for (const el of root.querySelectorAll('.toolbar .pin')) el.classList.remove('pin'); }
  function pinnote(t) { const el = q('[data-pinnote]'); if (el) el.textContent = t; }
  function markActive(n) { for (const s of document.querySelectorAll('[data-stop]')) s.classList.toggle('active', s.dataset.stop === String(n)); }

  // ---------- strip charts ----------
  function strip(canvas, defs, opts) {
    const buf = [], events = [];
    return {
      push(t, vals) { buf.push([t, vals]); },
      event(t, label) { events.push({ x: t, label }); },
      clear() { buf.length = 0; events.length = 0; },
      length: () => buf.length,
      draw(extra) {
        if (!buf.length) return;
        const t0 = buf[0][0], t1 = Math.max(buf[buf.length - 1][0], t0 + 1);
        charts2.line(canvas, Object.assign({
          series: defs.map((d, i) => ({ name: d.name, color: d.color, dashed: d.dashed, points: buf.map(([t, v]) => [t - t0, v[i]]).filter((p) => p[1] != null) })),
          x: { min: 0, max: t1 - t0, unit: 's', label: 'time since the stop began, s' }, events: events.map((e) => ({ x: e.x - t0, label: e.label })),
        }, opts || {}, extra || {}));
      },
      finalSecond(i) { const t1 = buf.length ? buf[buf.length - 1][0] : 0; const tail = buf.filter(([t]) => t > t1 - 1).map(([, v]) => v[i]).filter((x) => x != null); return tail.length ? tail.reduce((a, b) => a + b, 0) / tail.length : null; },
    };
  }
  const c1 = strip(q('[data-chart="s1"]'), [{ name: 'classical', color: '#eb6834' }, { name: 'learned', color: '#2a78d6' }], { y: { label: 'tip error, mm', min: 0 }, hairline: { y: 5, label: '5 mm' } });
  const c2 = strip(q('[data-chart="s2"]'), [{ name: 'chord in 3D', color: '#52514e' }, { name: 'spacing on screen', color: '#eb6834' }], { y: { label: 'ratio, log', log: true, floor: 0.01 } });
  const c3 = strip(q('[data-chart="s3"]'), [{ name: 'classical', color: '#eb6834' }], { y: { label: 'tip error, mm', min: 0 }, hairline: { y: 5, label: '5 mm' } });
  let tick = 0;

  // ---------- stop one: the boundary ----------
  function agreement() {
    const g = A.geometry(pcc.flex()), h = S.sim.st.planeY, rng = CR.makeRng(21);
    let agree = 0;
    const N = 300;
    for (let i = 0; i < N; i++) {
      const p = [-1.7 + 3.4 * rng(), -1.7 + 3.4 * rng(), h];
      if (workspace.gridContains(g.grid, p) === A.planner.solveIK(p, [0, 0, 0, 0]).reachable) agree++;
    }
    setV('agreeN', String(N)); setV('agreeP', (100 * agree / N).toFixed(1));
  }
  let agreeTimer = null;
  S.sim.on('plane', () => { if (stop === 1) { clearTimeout(agreeTimer); agreeTimer = setTimeout(agreement, 250); } });
  S.sim.on('target', ({ target, reachable }) => {
    if (stop === 1) {
      const g = A.geometry(pcc.flex());
      const inG = workspace.gridContains(g.grid, target);
      c1.clear(); c1.event(S.sim.st.t, 'click');
      setV('lastZ', (target[2] * MM).toFixed(0) + ' mm'); setV('lastIK', reachable ? 'reachable' : 'beyond reach'); setV('lastGrid', inG ? 'reachable' : 'beyond reach');
      setV('v1open', (reachable ? 'Reachable, ' : 'Beyond reach, ') + (inG === reachable ? 'and the outline agreed.' : 'and the outline disagreed; the point sits on the boundary, within a 6 mm cell of it.'));
      setH('reach', (reachable ? 'reachable' : 'beyond reach') + (inG === reachable ? ', outline agreed' : ', outline disagreed'));
    }
    if (stop === 3) { c3.event(S.sim.st.t, S.sim.st.usePlan ? 'planner on' : 'planner off'); }
  });

  // ---------- stop two: the geometry ----------
  let orbitStart = null; // { chord, px, az, el }
  const chordOf = (m) => v3.norm(v3.sub(m[0], m[1]));
  const pxOf = (m) => { const cam = S.orbitCam(); const a = cam.project(m[0]), b = cam.project(m[1]); return a && b ? Math.hypot(a[0] - b[0], a[1] - b[1]) : null; };
  let orbited = false;
  S.on('orbit', () => { if (stop === 2 && !orbited) { orbited = true; setV('v2open', 'Orbiting: the chord line stays at one, the spacing line does not.'); } });
  function chordSpread() {
    const rng = CR.makeRng(9); let mn = Infinity, mx = -Infinity, sum = 0;
    for (let i = 0; i < 300; i++) {
      const cfg = []; for (let s = 0; s < 2; s++) { const a = rng() * 2 * Math.PI, k = Math.sqrt(rng()) * pcc.KMAX[s]; cfg.push(k * Math.cos(a), k * Math.sin(a)); }
      const d = chordOf(pcc.markers3(cfg)); mn = Math.min(mn, d); mx = Math.max(mx, d); sum += d;
    }
    setV('chordP', (100 * (mx - mn) / (sum / 300)).toFixed(1));
    setV('arc', (pcc.SEG_LEN[0] + pcc.SEG_LEN[1]).toFixed(4) + ' units');
  }

  // ---------- stop three: the planner ----------
  let a3Phase = null; // 'direct' | 'planned'
  function runHook() { S.sim.reset(); S.demoTarget(hook); S.note('running the target under the base, planner ' + (S.sim.st.usePlan ? 'on' : 'off')); }
  S.sim.on('event', (e) => { if (stop === 3 && /^plan/.test(e.label)) setTimeout(runHook, 50); });
  S.sim.on('trial', (row) => {
    if (stop === 1 && row.classical) {
      const p = S.sim.st.lastSample;
      setV('v1open', q('[data-v="v1open"]').textContent.replace(/ The controllers.*$/, '') + ' The controllers ended ' + row.classical.steady.toFixed(1) + ' and ' + (row.learned ? row.learned.steady.toFixed(1) : '…') + ' mm from it.');
    }
    if (stop === 3 && row.classical) {
      if (!S.sim.st.usePlan) { setV('stall', row.classical.settle != null ? 'settled in ' + row.classical.settle.toFixed(2) + ' s' : 'stalled ' + row.classical.steady.toFixed(1) + ' mm short'); setV('v3open', row.classical.settle == null ? 'Direct, the classical law stalled ' + row.classical.steady.toFixed(1) + ' mm short. Flip the planner.' : 'Direct, the classical law settled this time; flip the planner and compare.'); setH('plan', row.classical.steady.toFixed(0) + ' mm short direct'); }
      else { setV('plannedSettle', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'did not settle'); setV('v3open', row.classical.settle != null ? 'Planned, the same law settled the same target in ' + row.classical.settle.toFixed(2) + ' s.' : 'Planned, the law still did not settle; read the table.'); const h = q('[data-h="plan"]'); if (h && !/planned/.test(h.textContent)) h.textContent += ', settled planned'; }
    }
  });

  // ---------- per-frame ----------
  S.on('draw', () => {
    tick++;
    const st = S.sim.st;
    if (stop === 1 && st.target && st.lastSample) { c1.push(st.t, [st.lastSample.errC, st.lastSample.errL]); if (tick % 3 === 0) c1.draw(); }
    if (stop === 2) {
      const m = S.sim.robots.classical.sim.markers3();
      const ch = chordOf(m), px = pxOf(m);
      if (!orbitStart) { if (st.target == null && px) orbitStart = { chord: ch, px: Math.max(px, 0.5), t: st.t }; }
      else if (px != null) {
        c2.push(st.t, [ch / orbitStart.chord, Math.max(px, 0.5) / orbitStart.px]);
        if (tick % 3 === 0) {
          c2.draw();
          const move = 100 * Math.abs(ch - orbitStart.chord) / orbitStart.chord;
          const f = Math.max(px, 0.5) / orbitStart.px;
          setV('chordMove', move.toFixed(2)); setV('pxFactor', (f >= 1 ? f : 1 / f).toFixed(1));
          if (orbited) setH('chord', move.toFixed(2) + ' percent against ×' + (f >= 1 ? f : 1 / f).toFixed(1));
        }
      }
    }
    if (stop === 3 && st.target && st.lastSample) { c3.push(st.t, [st.lastSample.errC]); if (tick % 3 === 0) c3.draw(); }
  });

  // ---------- entering a stop ----------
  function enter(n) {
    if (stop === n) return;
    stop = n; markActive(n);
    if (S.isPlaying()) S.stop('');
    if (n === 1) {
      S.setCages(true); S.setPlaneShown(true); S.setTendons(false); S.spin(false); S.setPreset('iso'); S.setPlan(true); S.sim.reset();
      pin('plane'); pinnote('Stop one is pinned: the plane height. Drag the plane in the side feed, then click on it to place a target.');
      c1.clear(); c1.draw(); agreement(); S.note('');
    } else if (n === 2) {
      S.setCages(false); S.setPlaneShown(false); S.setTendons(true); S.setPlan(true); S.sim.reset(); S.setPreset('iso');
      pin('orbit'); pinnote('Stop two is pinned: the orbit. Drag in the inspector; the robot is held still.');
      orbitStart = null; orbited = false; c2.clear();
      // settle the robot at a bent pose, then hold: the trial ends and the target clears
      S.demoTarget(still);
      const off = S.sim.on('trial', () => { off(); S.sim.st.target = null; S.spin(true); S.note('drag to orbit; the robot is held still'); });
      chordSpread();
      setV('v2open', 'Drag in the inspector to orbit.');
    } else if (n === 3) {
      S.setCages(true); S.setPlaneShown(true); S.setTendons(false); S.spin(false); S.setPreset('iso'); S.setPlan(false);
      pin('plan'); pinnote('Stop three is pinned: the Planner switch. Flipping it reruns the target under the base.');
      c3.clear(); setV('v3open', 'Running the target with the planner off.');
      setTimeout(runHook, 50);
    } else {
      unpin(); S.spin(false); S.setCages(true); S.setPlaneShown(true); S.note('everything unpinned, you have the controls'); pinnote('Free play: everything unpinned.');
    }
  }
  for (const b of document.querySelectorAll('[data-go]')) b.addEventListener('click', () => { const v = b.dataset.go; enter(v === 'free' ? 'free' : parseInt(v, 10)); const el = b.closest('.stop'); if (el && window.innerWidth <= 900) q('[data-scene="main"]').scrollIntoView({ behavior: 'smooth', block: 'start' }); });
  // scroll-driven: the stop whose section crosses the middle band of the viewport becomes active
  if (typeof IntersectionObserver === 'function') {
    const io = new IntersectionObserver((entries) => {
      for (const e of entries) if (e.isIntersecting) { const v = e.target.dataset.stop; enter(v === 'free' ? 'free' : parseInt(v, 10)); }
    }, { rootMargin: '-40% 0px -40% 0px', threshold: 0 });
    for (const s of document.querySelectorAll('[data-stop]')) io.observe(s);
  }
  enter(1);

  // ---------- model card, fps, tour ----------
  q('[data-modelcard]').innerHTML = kit.modelCard();
  setInterval(() => { const el = q('[data-fps]'); if (el) el.textContent = kit.ticker().fps() + ' fps'; }, 1000);
  window.CR_TOUR = spotlight.createTour([
    { target: 'header h1', title: 'One scene, three stops', body: 'The scene on the left stays put while the stops on the right set it up for one question each, pin one control, and write a verdict from what you do.' },
    { target: '[data-schematic]', title: 'What the cameras see', body: 'Two fixed sensors, four markers, a target plane, and the ray a click makes.', place: 'below' },
    { target: '.questions', title: 'Their questions', body: 'Three questions about version 1, quoted as asked; one stop each.' },
    { target: '[data-stop="1"] .lede', title: 'Stop one', body: 'The outline on the plane is a slice of the reachable set. The plane height is the pinned control.', onEnter: () => enter(1) },
    { target: '[data-scene="main"] [data-cr="plane"]', title: 'Stop one, the plane', body: 'Drag the plane below the base and the outline becomes a ring. Click on the plane to place a target; the verdict says whether the outline and the solver agreed.' },
    { target: '[data-chart="s1"]', title: 'Stop one, the chart', body: 'Both controllers\' tip error after your click; a target outside the outline flattens above the band.', place: 'below' },
    { target: '[data-stop="2"] .lede', title: 'Stop two', body: 'The robot is held still; only the viewpoint moves. The orbit is the pinned control.', onEnter: () => enter(2) },
    { target: '[data-scene="main"] [data-cr="inspector"]', title: 'Stop two, the orbit', body: 'Drag here. The first pair\'s 3D chord and its on-screen spacing are measured every frame.' },
    { target: '[data-chart="s2"]', title: 'Stop two, the chart', body: 'The chord line stays at one; the spacing line swings by a factor of ten or more. Nothing moves along the robot; the screen does.', place: 'below' },
    { target: '[data-stop="3"] .lede', title: 'Stop three', body: 'The target under the base, at the edge of reach, with the planner off. The Planner switch is the pinned control.', onEnter: () => enter(3) },
    { target: '[data-scene="main"] [data-cr="tgl-plan"]', title: 'Stop three, the switch', body: 'Flip it: the same target reruns from rest with inverse kinematics choosing the bend first.' },
    { target: '[data-chart="s3"]', title: 'Stop three, the chart', body: 'Planner off flattens tens of millimetres above the band; planner on crosses it and stays.', place: 'below' },
    { target: '[data-modelcard]', title: 'The model card', body: 'The blue controller is the one trained thing on this page.', place: 'below' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'One scene serves three stops; the pose is held still in stop two; the cameras are perfect; a corrected number from an earlier version.', place: 'below' },
  ], { autoOpenOnce: true, storageKey: 'cr_tour_v5', button: '[data-tour-open]' });
  window.CR_VARIANT_READY = '5';
})();
