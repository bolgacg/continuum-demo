// Variant 1: chapters, one act per question, one scene per act.
(function () {
  'use strict';
  if (!document.getElementById('variant-1')) return;
  const { kit, charts2, pcc, workspace, camera, scene, v3, spotlight } = CR;
  const A = kit.assets();
  const MM = kit.CONST.MM;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  window.CR_EMBED = kit.embed({ id: '1' });

  // ---------- computations shared by the headline lines and the chapters ----------
  function randomQ(rng, lo, hi) {
    const out = [];
    for (let i = 0; i < 2; i++) { const a = rng() * 2 * Math.PI, k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i]; out.push(k * Math.cos(a), k * Math.sin(a)); }
    return out;
  }
  const camSide = camera.sideCamera(kit.CONST.W, kit.CONST.H);
  // chord / pixel spread at the current flexibility over n random poses
  function geometrySpread(n, seed) {
    const rng = CR.makeRng(seed || 4);
    const pairs = [[0, 1], [1, 2], [2, 3]];
    const ch = pairs.map(() => ({ min: Infinity, max: -Infinity, sum: 0 }));
    const px = pairs.map(() => ({ min: Infinity, max: -Infinity, sum: 0 }));
    let arcMin = Infinity, arcMax = -Infinity;
    for (let i = 0; i < n; i++) {
      const cfg = randomQ(rng, 0, 1);
      const m = pcc.markers3(cfg);
      // arc length is a constant of the model: the segment lengths, whatever the bend
      const arc = pcc.SEG_LEN[0] + pcc.SEG_LEN[1];
      arcMin = Math.min(arcMin, arc); arcMax = Math.max(arcMax, arc);
      const pr = m.map((p) => camSide.project(p));
      pairs.forEach(([a, b], j) => {
        const d = v3.norm(v3.sub(m[a], m[b]));
        ch[j].min = Math.min(ch[j].min, d); ch[j].max = Math.max(ch[j].max, d); ch[j].sum += d;
        if (pr[a] && pr[b]) { const s = Math.hypot(pr[a][0] - pr[b][0], pr[a][1] - pr[b][1]); px[j].min = Math.min(px[j].min, s); px[j].max = Math.max(px[j].max, s); px[j].sum += s; }
      });
    }
    const pct = (o) => ((o.max - o.min) / (o.sum / n)) * 100;
    return { chordPct: ch.map(pct), pixPct: px.map(pct), pixRatio: px.map((o) => o.max / Math.max(o.min, 1e-6)), arcMin, arcMax, arcPct: ((arcMax - arcMin) / (0.5 * (arcMax + arcMin))) * 100 };
  }
  // outline (grid) and cage against inverse kinematics on the plane at height h
  function agreement(h, n, seed) {
    const g = A.geometry(pcc.flex());
    const rng = CR.makeRng(seed || 9);
    let agreeGrid = 0, agreeCage = 0;
    for (let i = 0; i < n; i++) {
      const p = [-1.7 + 3.4 * rng(), -1.7 + 3.4 * rng(), h];
      const ik = A.planner.solveIK(p, [0, 0, 0, 0]).reachable;
      if (workspace.gridContains(g.grid, p) === ik) agreeGrid++;
      if (workspace.insideEnvelope(g.volume, p) === ik) agreeCage++;
    }
    return { grid: 100 * agreeGrid / n, cage: 100 * agreeCage / n, n };
  }

  // ---------- flexibility sweep for chapter two (runs before the scenes exist) ----------
  const flexSweep = [];
  {
    const f0 = pcc.flex();
    for (let f = 0.7; f <= 1.8 + 1e-9; f += 0.1) {
      pcc.setFlex(+f.toFixed(1));
      flexSweep.push(Object.assign({ f: +f.toFixed(1) }, geometrySpread(300, 4)));
    }
    pcc.setFlex(f0);
  }
  const flexRow = (f) => flexSweep.reduce((a, b) => (Math.abs(b.f - f) < Math.abs(a.f - f) ? b : a));

  kit.schematic(q('[data-schematic]'));

  // ---------- chapter one ----------
  const a1 = kit.createScene(q('[data-scene="a1"]'), { id: 'v1-a1', views: ['inspector', 'side'], toolbar: false, charts: false, table: 6,
    clickHint: 'click inside the outline to place a target' });
  const areaCanvas = q('[data-chart="area"]');
  let areaSeries = null;
  function computeAreas() {
    const g = A.geometry(pcc.flex());
    const res = g.grid.res, gridPts = [], cagePts = [];
    for (let h = kit.CONST.PLANE_MIN; h <= kit.CONST.PLANE_MAX + 1e-9; h += 0.05) {
      const cells = workspace.gridSliceCells(g.grid, h);
      gridPts.push([h * MM, cells.length * res * res * MM * MM]);
      let inside = 0;
      for (let x = -1.7; x <= 1.7; x += res) for (let y = -1.7; y <= 1.7; y += res) if (workspace.insideEnvelope(g.volume, [x, y, h])) inside++;
      cagePts.push([h * MM, inside * res * res * MM * MM]);
    }
    areaSeries = { gridPts, cagePts };
  }
  function drawArea() {
    if (!areaSeries) computeAreas();
    charts2.line(areaCanvas, {
      series: [{ name: 'cage slice', color: '#9a9d97', points: areaSeries.cagePts, dashed: true }, { name: 'reachable', color: '#52514e', points: areaSeries.gridPts }],
      x: { label: 'plane height, mm', min: kit.CONST.PLANE_MIN * MM, max: kit.CONST.PLANE_MAX * MM }, y: { label: 'area, mm²', min: 0 },
      marker: { x: a1.sim.st.planeY * MM, label: 'plane' },
    });
  }
  let agreeTimer = null;
  function refreshAgreement() {
    const r = agreement(a1.sim.st.planeY, 300, 9);
    setV('agreeP', r.grid.toFixed(1)); setV('agreeN', String(r.n)); setV('cageP', r.cage.toFixed(1));
    const h = q('[data-h="agree"]'); if (h) h.textContent = r.grid.toFixed(0) + ' percent of ' + r.n + ' points';
  }
  a1.sim.on('plane', () => { drawArea(); clearTimeout(agreeTimer); agreeTimer = setTimeout(refreshAgreement, 250); });
  drawArea(); refreshAgreement();

  // ---------- chapter two ----------
  const a2 = kit.createScene(q('[data-scene="a2"]'), { id: 'v1-a2', views: ['inspector'], robots: ['classical'], readouts: false, toolbar: ['flex'],
    charts: false, table: false, tendons: true, cages: false, plane: false, clickToPlace: false, preset: 'iso', dist: 2.3 });
  const sweepTargets = [kit.T3([0.9, 0.3, 0.7, -0.2]), kit.T3([-0.6, -0.5, -0.9, 0.4])];
  let sweepIdx = 0;
  a2.sim.on('trial', () => { sweepIdx = 1 - sweepIdx; a2.startTrial(sweepTargets[sweepIdx]); });
  a2.startTrial(sweepTargets[0]);
  const flexCanvas = q('[data-chart="flex"]');
  function drawFlex() {
    const floor = 0.01;
    const pts = (fn) => flexSweep.map((r) => [r.f, Math.max(floor, fn(r))]);
    charts2.line(flexCanvas, {
      series: [
        { name: 'pixels, pair 1', color: '#2a78d6', points: pts((r) => r.pixPct[0]) },
        { name: 'chord, pair 3', color: '#c08a5a', points: pts((r) => r.chordPct[2]) },
        { name: 'chord, pair 2', color: '#d95926', points: pts((r) => r.chordPct[1]) },
        { name: 'chord, pair 1', color: '#eb6834', points: pts((r) => r.chordPct[0]) },
        { name: 'arc length', color: '#52514e', points: pts((r) => r.arcPct), width: 2.5 },
      ],
      x: { label: 'flexibility, ×', min: 0.7, max: 1.8 }, y: { label: 'variation over poses, % of mean', log: true, floor, min: floor, max: 2000 },
      marker: { x: pcc.flex(), label: '×' + pcc.flex().toFixed(1) },
    });
    const r = flexRow(pcc.flex());
    setV('chordP', r.chordPct[0].toFixed(1)); setV('chordP2', r.chordPct[1].toFixed(1)); setV('chordP3', r.chordPct[2].toFixed(1));
    setV('pixR', r.pixRatio[0] >= 100 ? r.pixRatio[0].toFixed(0) : r.pixRatio[0].toFixed(1)); setV('arc', r.arcMin.toFixed(4));
    const h = q('[data-h="chord"]'); if (h) h.textContent = r.chordPct[0].toFixed(1) + ' percent against ×' + (r.pixRatio[0] >= 100 ? r.pixRatio[0].toFixed(0) : r.pixRatio[0].toFixed(1));
  }
  drawFlex();
  // flexibility is page-wide: every scene follows, and the chapter-one areas are recomputed
  a2.on('draw', (() => { let last = pcc.flex(); return () => { if (Math.abs(pcc.flex() - last) > 1e-9) { last = pcc.flex(); areaSeries = null; drawArea(); refreshAgreement(); drawFlex(); } }; })());

  // ---------- chapter three ----------
  const a3 = kit.createScene(q('[data-scene="a3"]'), { id: 'v1-a3', views: ['inspector', 'side'], toolbar: ['plan'], charts: 'err', table: 6, clickToPlace: false });
  const hook = kit.T3(kit.TQ.hook);
  function runHook() { a3.sim.reset(); a3.demoTarget(hook); a3.note('running the target under the base, planner ' + (a3.sim.st.usePlan ? 'on' : 'off')); }
  a3.sim.on('event', (e) => { if (/^plan/.test(e.label)) setTimeout(runHook, 50); });
  a3.sim.on('trial', (row) => {
    if (row.classical) {
      if (!a3.sim.st.usePlan) setV('stall', row.classical.settle != null ? 'settled (' + row.classical.settle.toFixed(2) + ' s)' : row.classical.steady.toFixed(1) + ' mm');
      else setV('plannedSettle', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'did not settle');
    }
  });
  q('[data-run="hook"]').addEventListener('click', runHook);
  a3.setPlan(false);
  setTimeout(runHook, 400);

  // ---------- model card, free play ----------
  q('[data-modelcard]').innerHTML = kit.modelCard();
  kit.createScene(q('[data-scene="a4"]'), { id: 'v1-free', charts: true, table: 10 });

  // ---------- fps, walkthrough ----------
  setInterval(() => { const el = q('[data-fps]'); if (el) el.textContent = kit.ticker().fps() + ' fps'; }, 1000);
  const tour = spotlight.createTour([
    { target: 'header h1', title: 'Three questions, three chapters', body: 'One robotics researcher asked three questions about the first version of this page. Each chapter below answers one of them with a scene you can operate, a chart with its three lines, and a verdict computed here, not typed.' },
    { target: '[data-schematic]', title: 'What the cameras see', body: 'Two fixed cameras see four markers on the robot; each marker is triangulated to a point in space, and that is all either controller gets. The click ray meets the target plane, and the outline on the plane is where a target can be reached.', place: 'below' },
    { target: '.questions', title: 'Their questions', body: 'The wording this page answers, quoted as asked. The references under it are the three pieces of literature every mechanism here comes from.' },
    { target: '[data-scene="a1"] [data-cr="plane"]', title: 'Chapter one, the slider', body: 'Drag the plane below the base and watch the outline become a ring: a robot of fixed length cannot bring its tip next to its own base. Then click inside the outline and both controllers go for the point.' },
    { target: '[data-chart="area"]', title: 'Chapter one, the chart', body: 'The gap between the grey cage line and the solid reachable line is the cage over-promising. The outline on the plane comes from the solid line.', place: 'below' },
    { target: '[data-scene="a2"] [data-cr="flex"]', title: 'Chapter two, the slider', body: 'Push flexibility to ×1.8. The reach changes, the arc-length line on the chart does not move at all, and the chord lines rise by a few percent because an arc\'s chord shortens as it bends.' },
    { target: '[data-scene="a2"] .hud-chips', title: 'Chapter two, the views', body: 'Press Side, then Top. The three tendons per segment keep their radius from every angle; what changes is only where they land on a screen.' },
    { target: '[data-scene="a3"] [data-cr="tgl-plan"]', title: 'Chapter three, the switch', body: 'The target sits curled under the base. With the planner off the orange line parks above the settle band; flip it on and the same feedback law tracks a solved path and settles.' },
    { target: '[data-act="3"] .verdict', title: 'Chapter three, the numbers', body: () => 'Direct against planned over the evaluation run: the verdict quotes the published cells, and the live trial above adds this run\'s stall distance.', place: 'below' },
    { target: '[data-modelcard]', title: 'The model card', body: 'The blue controller is the one trained thing on this page: what it is, what it predicts, what it was fitted on, and how it scores against the classical law in millimetres.', place: 'below' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'The limits, stated before you find them: an invented truth model, perfect cameras, a plan that costs a third of a second, and a corrected number from an earlier version.', place: 'below' },
    { target: '[data-scene="a4"] .toolbar', title: 'Free play', body: 'Everything unpinned: both robots, every toggle, the charts and the trial table. The walkthrough can be restarted from the byline at the top.', place: 'below' },
  ], { autoOpenOnce: true, storageKey: 'cr_tour_v1', button: '[data-tour-open]' });
  window.CR_TOUR = tour;
  window.CR_VARIANT_READY = '1';
})();
