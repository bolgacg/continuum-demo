// Variant 3: sensing, model, control, the three layers in the reader's vocabulary.
(function () {
  'use strict';
  if (!document.getElementById('variant-3')) return;
  const { kit, charts2, pcc, truth, workspace, camera, v3, spotlight } = CR;
  const A = kit.assets();
  const { MM } = kit.CONST;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  window.CR_EMBED = kit.embed({ id: '3' });
  kit.schematic(q('[data-schematic]'));
  function randomQ(rng, lo, hi) { const out = []; for (let i = 0; i < 2; i++) { const a = rng() * 2 * Math.PI, k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i]; out.push(k * Math.cos(a), k * Math.sin(a)); } return out; }

  // ================= layer one: sensing =================
  const a1 = kit.createScene(q('[data-scene="a1"]'), { id: 'v3-a1', views: ['inspector', 'side'], toolbar: false, charts: false, table: 6, sensors: true,
    clickHint: 'click inside the outline to place a target' });
  const rayCanvas = q('[data-chart="ray"]');
  let lastRay = null; // {origin, dir}
  function drawRay() {
    const { ctx, W, H } = charts2.prep(rayCanvas);
    const g = A.geometry(pcc.flex());
    const PAD = { l: 44, r: 20 };
    const smax = 5.2, s0 = 0.4;
    const px = (s) => PAD.l + ((s - s0) / (smax - s0)) * (W - PAD.l - PAD.r);
    const y = H / 2;
    if (!lastRay) { const cam = a1.orbitCam(); lastRay = { origin: cam.pos, dir: v3.normalize(v3.sub([0.3, 0.2, a1.sim.st.planeY], cam.pos)), synthetic: true }; }
    ctx.strokeStyle = '#c3c2b7'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(px(s0), y); ctx.lineTo(px(smax), y); ctx.stroke();
    const step = g.grid.res / 2;
    let inside = false, start = 0;
    for (let s = s0; s <= smax; s += step) {
      const p = v3.add(lastRay.origin, v3.scale(lastRay.dir, s));
      const inS = workspace.gridContains(g.grid, p);
      if (inS && !inside) { inside = true; start = s; }
      if ((!inS || s + step > smax) && inside) { inside = false; ctx.fillStyle = 'rgba(138,143,136,0.45)'; ctx.fillRect(px(start), y - 14, Math.max(2, px(s) - px(start)), 28); }
    }
    // plane cut
    const hit = camera.rayPlaneZ(lastRay.origin, lastRay.dir, a1.sim.st.planeY);
    if (hit) { const s = v3.norm(v3.sub(hit, lastRay.origin)); ctx.strokeStyle = '#52514e'; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(px(s), 8); ctx.lineTo(px(s), H - 22); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = '#52514e'; ctx.fillText('plane cuts the ray here', px(s) + 5, 16); }
    ctx.fillStyle = '#898781';
    for (const s of [1, 2, 3, 4, 5]) ctx.fillText((s * MM).toFixed(0) + ' mm', px(s) - 14, H - 6);
    ctx.fillText(lastRay.synthetic ? 'a default ray until you click' : 'the last click ray', PAD.l, H - 6 - 18 - 14);
    ctx.fillText('camera', px(s0) - 36, y + 4);
  }
  function agreement(h, n) {
    const g = A.geometry(pcc.flex()); const rng = CR.makeRng(9); let agree = 0;
    for (let i = 0; i < n; i++) { const p = [-1.7 + 3.4 * rng(), -1.7 + 3.4 * rng(), h]; if (workspace.gridContains(g.grid, p) === A.planner.solveIK(p, [0, 0, 0, 0]).reachable) agree++; }
    return { p: 100 * agree / n, n };
  }
  let agreeTimer = null;
  function refreshAgreement() { const r = agreement(a1.sim.st.planeY, 300); setV('agreeP', r.p.toFixed(1)); setV('agreeN', String(r.n)); const h = q('[data-h="agree"]'); if (h) h.textContent = r.p.toFixed(0) + ' percent of ' + r.n + ' points'; }
  a1.sim.on('plane', () => { drawRay(); clearTimeout(agreeTimer); agreeTimer = setTimeout(refreshAgreement, 250); });
  a1.sim.on('target', ({ target, reachable }) => {
    if (a1.sim.st.lastRay) lastRay = { origin: a1.sim.st.lastRay.origin, dir: a1.sim.st.lastRay.dir };
    drawRay();
    setV('lastVerdict', (reachable ? 'inside the reachable set' : 'beyond reach') + ' at z = ' + (target[2] * MM).toFixed(0) + ' mm');
  });
  drawRay(); refreshAgreement();

  // ================= layer two: model =================
  const a2 = kit.createScene(q('[data-scene="a2"]'), { id: 'v3-a2', views: ['inspector', 'side'], toolbar: ['payload'], charts: false, table: false, tendons: true, cages: false, plane: false, clickToPlace: false, preset: 'side', dist: 2.6 });
  const lean = kit.T3([1.5, 0.35, 0.9, 0.2]);
  a2.demoTarget(lean);
  const sagCanvas = q('[data-chart="sag"]');
  const P = truth.PARAMS;
  function sagCurve(payload) {
    const pts = [];
    for (let k = 0; k <= pcc.KMAX[0]; k += pcc.KMAX[0] / 40) {
      const qq = [k, 0, k * 0.6, 0];
      const ideal = pcc.tip3(qq), sagged = pcc.tip3(truth.applyStatic(qq, payload));
      pts.push([(k * pcc.SEG_LEN[0] * 180) / Math.PI, v3.norm(v3.sub(sagged, ideal)) * MM]);
    }
    return pts;
  }
  const sag0 = sagCurve(0), sag1 = sagCurve(1);
  function drawSag() {
    const r = a2.sim.robots.classical;
    const qc = r ? r.sim.qCmd : [0, 0, 0, 0];
    const leanDeg = (Math.hypot(qc[0], qc[1]) * pcc.SEG_LEN[0] * 180) / Math.PI;
    charts2.line(sagCanvas, {
      series: [{ name: 'no payload', color: '#9a9d97', points: sag0 }, { name: 'full payload', color: '#52514e', points: sag1 }],
      x: { label: 'lean of segment one, degrees', min: 0, max: sag1[sag1.length - 1][0] }, y: { label: 'tip droop, mm', min: 0 },
      marker: { x: leanDeg, label: 'now, payload ' + (r ? r.sim.payload.toFixed(1) : '0') },
    });
  }
  let sagTick = 0;
  a2.on('draw', () => { if (++sagTick % 6 === 0) drawSag(); });
  drawSag();
  { const full = sag1[sag1.length - 1][1]; const h = q('[data-h="sag"]'); if (h) h.textContent = full.toFixed(0) + ' mm at ' + sag1[sag1.length - 1][0].toFixed(0) + ' degrees'; }
  // chord spread and arc length
  {
    const rng = CR.makeRng(4); let mn = Infinity, mx = -Infinity, sum = 0; const n = 300;
    for (let i = 0; i < n; i++) { const m = pcc.markers3(randomQ(rng, 0, 1)); const d = v3.norm(v3.sub(m[0], m[1])); mn = Math.min(mn, d); mx = Math.max(mx, d); sum += d; }
    setV('chordP', (((mx - mn) / (sum / n)) * 100).toFixed(1));
    setV('arc', (pcc.SEG_LEN[0] + pcc.SEG_LEN[1]).toFixed(4));
  }
  // the coefficient table, read from the truth model itself
  {
    const rows = [
      ['lag', 'first-order lag on tendon displacement, with a rate limit', 'lagTau, rateMaxK', P.lagTau + ' s, ' + P.rateMaxK + ' /s'],
      ['backlash', 'play operator per tendon (the Prandtl-Ishlinskii element)', 'backlashK', P.backlashK + ' curvature'],
      ['sag', 'curvature biased toward gravity at each segment midpoint, scaled by payload', 'droopSelf, droopLoad', P.droopSelf.join(', ') + '; ' + P.droopLoad.join(', ')],
      ['coupling', 'a share of segment-one curvature leaking into segment two', 'coupling', String(P.coupling)],
      ['drift', 'slow creep plus a random walk on each tendon, capped', 'driftCreepK, driftWalkK, driftMaxK', P.driftCreepK + ', ' + P.driftWalkK + ', ' + P.driftMaxK],
    ];
    q('[data-coef] tbody').innerHTML = rows.map((r) => '<tr><td>' + r[0] + '</td><td>' + r[1] + '</td><td>' + r[2] + '</td><td>' + r[3] + ' <span class="pill">typed by the author</span></td></tr>').join('');
  }

  // ================= layer three: control =================
  const a3 = kit.createScene(q('[data-scene="a3"]'), { id: 'v3-a3', views: ['inspector', 'side'], toolbar: ['plan'], charts: false, table: 6, clickToPlace: false });
  const hook = kit.T3(kit.TQ.hook);
  function runHook() { a3.sim.reset(); a3.demoTarget(hook); a3.note('running the target under the base, planner ' + (a3.sim.st.usePlan ? 'on' : 'off')); }
  a3.sim.on('event', (e) => { if (/^plan/.test(e.label)) setTimeout(runHook, 50); });
  a3.sim.on('trial', (row) => {
    if (!row.classical) return;
    if (!a3.sim.st.usePlan) setV('stall', row.classical.settle != null ? 'settled (' + row.classical.settle.toFixed(2) + ' s)' : row.classical.steady.toFixed(1) + ' mm');
    else setV('plannedSettle', row.classical.settle != null ? row.classical.settle.toFixed(2) + ' s' : 'did not settle');
  });
  q('[data-run="hook"]').addEventListener('click', runHook);
  a3.setPlan(false);
  setTimeout(runHook, 400);
  {
    const cv = q('[data-chart="bars"]');
    const n = (s) => parseInt(String(s).split('/')[0], 10);
    const groups = [['nominal', 'nominal'], ['payload', 'payload'], ['drift', 'drift'], ['both', 'payload + drift']].map(([k, label]) => {
      const c = cv.dataset['c' + k[0].toUpperCase() + k.slice(1)].split('|'), l = cv.dataset['l' + k[0].toUpperCase() + k.slice(1)].split('|');
      return { label, bars: [
        { label: 'classical direct', value: n(c[0]), color: '#eb6834', light: true }, { label: 'classical planned', value: n(c[1]), color: '#eb6834' },
        { label: 'learned direct', value: n(l[0]), color: '#2a78d6', light: true }, { label: 'learned planned', value: n(l[1]), color: '#2a78d6' },
      ] };
    });
    charts2.bars(cv, { groups, y: { max: 40 }, valueLabel: (v) => String(v) });
  }

  // ================= model card, free play, tour =================
  q('[data-modelcard]').innerHTML = kit.modelCard();
  kit.createScene(q('[data-scene="a4"]'), { id: 'v3-free', charts: true, table: 10 });
  setInterval(() => { const el = q('[data-fps]'); if (el) el.textContent = kit.ticker().fps() + ' fps'; }, 1000);
  window.CR_TOUR = spotlight.createTour([
    { target: 'header h1', title: 'A geometric model', body: 'The controller on this page is built on a geometric model. The page is its three layers: what the cameras sense, what the model assumes, what the control law does with both. Each layer says what it is allowed to know.' },
    { target: '[data-schematic]', title: 'What the cameras see', body: 'Two fixed sensors, four markers, a target plane, and the click ray that a click makes.', place: 'below' },
    { target: '.questions', title: 'Their questions', body: 'Three questions, plus the later one: geometric model, or some other type. Layer two answers it directly.' },
    { target: '[data-act="1"] .knows', title: 'Layer one, what it knows', body: 'Pixels in two calibrated cameras, and nothing about the robot\'s shape. Everything the controllers get comes through here.' },
    { target: '[data-scene="a1"] [data-cr="plane"]', title: 'Layer one, the slider', body: 'Move the plane through the base and watch the ray\'s shaded stretches split: near the base the reachable set is a ring.' },
    { target: '[data-scene="a2"] [data-cr="tgl-tendons"], [data-scene="a2"] .feeds', title: 'Layer two, the tendons', body: 'Three tendons per segment at a fixed radius, drawn as they are. The geometry is fixed; the truth model adds four effects on top of it.' },
    { target: '[data-scene="a2"] [data-cr="tgl-payload"]', title: 'Layer two, the payload', body: 'Flip it and the marker climbs the sag curve: a rule the ideal model does not contain, with a coefficient the author typed.' },
    { target: '[data-coef]', title: 'Layer two, the coefficients', body: 'Every one of the four effects, its form, and its typed value. This is the honest answer to "geometric or some other type": geometric, with invented additions.', place: 'below' },
    { target: '[data-scene="a3"] [data-cr="tgl-plan"]', title: 'Layer three, the switch', body: 'Planner off: the feedback law alone chases a target curled under the base and parks in the wrong bending plane. Planner on: the inverse kinematics is solved on the model first.' },
    { target: '[data-chart="bars"]', title: 'Layer three, the bars', body: 'Light bars are the law alone, solid bars the same law tracking a plan; the gap is the plan. Forty edge targets per bar.', place: 'below' },
    { target: '[data-modelcard]', title: 'The model card', body: 'The blue controller is the one trained thing on this page.', place: 'below' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'Invented coefficients, perfect cameras, a plan that costs a third of a second, and a corrected number from an earlier version.', place: 'below' },
  ], { autoOpenOnce: true, storageKey: 'cr_tour_v3', button: '[data-tour-open]' });
  window.CR_VARIANT_READY = '3';
})();
