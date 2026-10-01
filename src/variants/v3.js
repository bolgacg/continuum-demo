// Variant 3: sensing, model, control, the three layers in the reader's vocabulary.
(function () {
  'use strict';
  if (!document.getElementById('variant-3')) return;
  const { kit, charts2, pcc, truth, workspace, camera, v3, spotlight, scene, hyst } = CR;
  const A = kit.assets();
  const { MM, DT } = kit.CONST;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  window.CR_EMBED = kit.embed({ id: '3' });
  kit.schematic(q('[data-schematic]'));
  function randomQ(rng, lo, hi) { const out = []; for (let i = 0; i < 2; i++) { const a = rng() * 2 * Math.PI, k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i]; out.push(k * Math.cos(a), k * Math.sin(a)); } return out; }

  // ================= layer one: sensing =================
  const a1 = kit.createScene(q('[data-scene="a1"]'), { id: 'v3-a1', views: ['inspector', 'side'], toolbar: false, charts: false, table: 6,
    rejectBeyond: true, clickHint: 'click in the band between the two outlines' });
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
    const hit = camera.rayPlaneZ(lastRay.origin, lastRay.dir, a1.sim.st.planeY);
    if (hit) { const s = v3.norm(v3.sub(hit, lastRay.origin)); ctx.strokeStyle = '#52514e'; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(px(s), 8); ctx.lineTo(px(s), H - 22); ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = '#52514e'; ctx.fillText('plane cuts the ray here', px(s) + 5, 16); }
    ctx.fillStyle = '#898781';
    for (const s of [1, 2, 3, 4, 5]) ctx.fillText((s * MM).toFixed(0) + ' mm', px(s) - 14, H - 6);
    ctx.fillText(lastRay.synthetic ? 'a default ray until you click' : 'the last click ray', PAD.l, H - 6 - 18 - 14);
    ctx.fillText('camera', px(s0) - 36, y + 4);
  }
  // agreement between the drawn outline (grid) and the solver, and how far each disagreeing point is from the outline
  function agreement(h, n) {
    const g = A.geometry(pcc.flex()); const rng = CR.makeRng(9); let agree = 0; let maxEdge = 0; let dis = 0;
    for (let i = 0; i < n; i++) {
      const p = [-1.7 + 3.4 * rng(), -1.7 + 3.4 * rng(), h];
      const inG = workspace.gridContains(g.grid, p);
      if (inG === A.planner.solveIK(p, [0, 0, 0, 0]).reachable) { agree++; continue; }
      dis++;
      let edge = Infinity;
      for (let r = 0.01; r <= 0.3 && edge === Infinity; r += 0.01) for (let a = 0; a < 48; a++) {
        const qq = [p[0] + r * Math.cos(a * Math.PI / 24), p[1] + r * Math.sin(a * Math.PI / 24), h];
        if (workspace.gridContains(g.grid, qq) !== inG) { edge = r; break; }
      }
      maxEdge = Math.max(maxEdge, edge);
    }
    return { p: 100 * agree / n, n, dis, maxEdge };
  }
  let agreeTimer = null, headlineSet = false;
  function refreshAgreement() {
    const r = agreement(a1.sim.st.planeY, 300);
    setV('agreeP', r.p.toFixed(1)); setV('agreeN', String(r.n));
    setV('disN', String(r.dis)); setV('disMax', r.dis ? (r.maxEdge * MM).toFixed(0) + ' mm' : '0 mm');
    const h = q('[data-h="agree"]'); if (h && !headlineSet) { h.textContent = r.p.toFixed(0) + ' percent of ' + r.n + ' points'; headlineSet = true; }
  }
  a1.sim.on('plane', () => { drawRay(); clearTimeout(agreeTimer); agreeTimer = setTimeout(refreshAgreement, 250); });
  a1.on('rejected', (p) => setV('lastVerdict', p ? 'beyond reach at z = ' + (p[2] * MM).toFixed(0) + ' mm, so it was not run' : 'off the plane, so it was not run'));
  // moving the plane abandons any trial: the cards go back to idle
  a1.sim.on('plane', () => { for (const k of ['ro-c-state', 'ro-l-state']) { const el = a1.$(k); if (el) el.textContent = 'idle'; } for (const k of ['ro-c-settle', 'ro-l-settle']) { const el = a1.$(k); if (el) el.textContent = '–'; } });
  a1.sim.on('target', ({ target, reachable }) => {
    if (a1.sim.st.lastRay) lastRay = { origin: a1.sim.st.lastRay.origin, dir: a1.sim.st.lastRay.dir };
    drawRay();
    setV('lastVerdict', (reachable ? 'inside the reachable set' : 'beyond reach') + ' at z = ' + (target[2] * MM).toFixed(0) + ' mm');
  });
  drawRay(); refreshAgreement();

  // ================= layer two: model, and the backlash loop =================
  // chord spread and arc length
  {
    // the first pair (segment one's midpoint and end) spans half of segment one's arc;
    // its chord can only be shorter than that arc, so the figure is the largest shortening
    const rng = CR.makeRng(4); let mn = Infinity; const n = 300, arcPair = 0.5 * pcc.SEG_LEN[0];
    for (let i = 0; i < n; i++) { const m = pcc.markers3(randomQ(rng, 0, 1)); mn = Math.min(mn, v3.norm(v3.sub(m[0], m[1]))); }
    setV('chordP', (((arcPair - mn) / arcPair) * 100).toFixed(2));
    setV('arc', (pcc.SEG_LEN[0] + pcc.SEG_LEN[1]).toFixed(4));
  }
  const P = truth.PARAMS;
  // the coefficient table, read from the truth model itself
  {
    const rows = [
      ['lag', 'first-order lag on tendon displacement, with a rate limit', 'lagTau, rateMaxK', P.lagTau + ' s, ' + P.rateMaxK + ' /s'],
      ['backlash', 'play operator per tendon (the Prandtl-Ishlinskii element)', 'backlashK', 'half-width ' + P.backlashK + ' curvature, ' + ((P.backlashK * pcc.SEG_LEN[0] * 180) / Math.PI).toFixed(2) + ' degrees of bend in segment one'],
      ['sag', 'curvature biased toward gravity at each segment midpoint, scaled by payload', 'droopSelf, droopLoad', P.droopSelf.join(', ') + '; ' + P.droopLoad.join(', ')],
      ['coupling', 'a share of segment-one curvature leaking into segment two', 'coupling', String(P.coupling)],
      ['drift', 'slow creep plus a random walk on each tendon, capped', 'driftCreepK, driftWalkK, driftMaxK', P.driftCreepK + ', ' + P.driftWalkK + ', ' + P.driftMaxK],
    ];
    q('[data-coef] tbody').innerHTML = rows.map((r) => '<tr><td>' + r[0] + '</td><td>' + r[1] + '</td><td>' + r[2] + '</td><td>' + r[3] + '</td></tr>').join('');
  }

  // The loops are computed once, on load, by the same sweep the animation runs.
  const deg = (k) => (k * pcc.SEG_LEN[0] * 180) / Math.PI;
  const loopPts = (pts) => pts.map(([k, y]) => [deg(k), y * MM]);
  const MODES = ['no compensation', 'inverse play', 'play and lag'];
  const hy = { est: null, tau: null, loops: null, gaps: null };
  const filterFor = (mode) => mode === 1 ? hyst.createCompensator(hy.est).filter : mode === 2 ? hyst.createLeadCompensator(hy.est, hy.tau).filter : null;
  function computeLoops() {
    const w = P.backlashK;
    hy.est = hyst.runIdentification(w, 4141).wEstK;
    hy.tau = hyst.identifyLag(4141).tau;
    const run = (mode, sp) => hyst.sweepLoop(w, mode === 0 ? null : filterFor(mode), sp);
    const L = [0, 1].map((sp) => [0, 1, 2].map((m) => run(m, sp)));
    const idealCurve = (pts) => { const m = new Map(); for (const [x, y] of loopPts(pts)) m.set(x.toFixed(1), [x, y]); return [...m.values()].sort((a, b) => a[0] - b[0]); };
    hy.loops = L.map((row) => ({ modes: row.map((r) => loopPts(r.points)), ideal: idealCurve(row[0].ideal) }));
    hy.gaps = L.map((row) => row.map((r) => r.gapMean * MM));
    { const r = L[0][2]; let sum = 0; for (let i = 0; i < r.points.length; i++) sum += r.points[i][1] - r.ideal[i][1]; setV('leadOffset', (Math.abs(sum / r.points.length) * MM).toFixed(1) + ' mm'); }
    const mm = (x) => x.toFixed(x < 1 ? 2 : 1) + ' mm';
    const bendDeg = (k) => deg(k).toFixed(2) + ' degrees';
    setV('estW', bendDeg(hy.est)); setV('trueW', bendDeg(w));
    setV('estTau', hy.tau.toFixed(4) + ' s'); setV('trueTau', P.lagTau.toFixed(2) + ' s');
    setV('gapOff', mm(hy.gaps[0][0])); setV('gapOn', mm(hy.gaps[0][1])); setV('gapLead', mm(hy.gaps[0][2]));
    setV('gapOffFast', mm(hy.gaps[1][0])); setV('gapOnFast', mm(hy.gaps[1][1])); setV('gapLeadFast', mm(hy.gaps[1][2]));
    const h = q('[data-h="loop"]'); if (h) h.textContent = mm(hy.gaps[0][0]) + ' uncompensated, ' + mm(hy.gaps[0][2]) + ' compensated';
    showNumbers();
  }
  function showNumbers() {
    if (!hy.gaps) return;
    const g = hy.gaps[live.speedIdx], mm = (x) => x.toFixed(x < 1 ? 2 : 1) + ' mm';
    q('[data-hy="numbers"]').textContent = 'Mean gap, ' + (live.speedIdx ? 'fast' : 'slow') + ' sweep: ' +
      mm(g[0]) + ' with no compensation, ' + mm(g[1]) + ' with the inverse play, ' + mm(g[2]) + ' with play and lag inverted.';
  }

  // the live sweep: one private simulator, commanded open loop
  const view = q('[data-hy="view"]'), loopCv = q('[data-chart="loop"]');
  const VW = 460, VH = 345;
  const dpr = window.devicePixelRatio || 1;
  view.width = VW * dpr; view.height = VH * dpr;
  const vctx = view.getContext('2d'); vctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const viewCam = camera.orbitCamera(camera.PRESETS.side.az, 0.12, VW, VH, 2.1);
  const S = hyst.SWEEP;
  const live = { speedIdx: 0, mode: 1, t: 0, sim: null, visible: true };
  function resetLive() {
    live.sim = truth.createTruth(4141);
    live.sim.reset([S.qBase[0], S.qBase[1] - S.amp, S.qBase[2], S.qBase[3]]);
    live.t = 0;
    live.sim.tendonFilter = hy.est != null ? filterFor(live.mode) : null;
  }
  function kAt(t) {
    const v = S.speeds[live.speedIdx], P4 = (4 * S.amp) / v, a = (t % P4) * v;
    return S.qBase[1] + (a < 2 * S.amp ? a - S.amp : 3 * S.amp - a);
  }
  function drawLive() {
    const s = live.sim;
    scene.draw(vctx, { W: VW, H: VH, cam: viewCam, robots: [{ sim: s, accent: kit.ACCENT.classical }], target: null,
      plane: { y: 0, show: false }, section: null, volume: { mesh: null, show: false }, trainVolume: { mesh: null }, sensors: null,
      label: 'SIDE VIEW, ZOOMED · ' + (live.speedIdx ? 'FAST' : 'SLOW') + ' SWEEP · ' + MODES[live.mode].toUpperCase(), t: live.t, layerKey: 'v3/hy-view' });
    const bb = pcc.backbone(s.qCmd, 14).map((p) => viewCam.project(p)).filter(Boolean);
    vctx.save(); vctx.strokeStyle = 'rgba(232,234,230,0.7)'; vctx.lineWidth = 1.5; vctx.setLineDash([5, 4]);
    vctx.beginPath(); bb.forEach((p, i) => (i ? vctx.lineTo(p[0], p[1]) : vctx.moveTo(p[0], p[1]))); vctx.stroke(); vctx.restore();
  }
  const MODE_COLOR = ['#8a8f88', '#d95926', '#1d7a6a'];
  function drawLoop() {
    if (!hy.loops) return;
    const L = hy.loops[live.speedIdx];
    const all = L.modes[0].concat(L.ideal);
    const ys = all.map((p) => p[1]);
    const series = [{ name: '', color: live.mode === 0 ? MODE_COLOR[0] : '#c9ccc6', points: L.modes[0], width: live.mode === 0 ? 1.8 : 1.4 }];
    if (live.mode > 0) series.push({ name: '', color: MODE_COLOR[live.mode], points: L.modes[live.mode], width: 2 });
    series.push({ name: '', color: '#1f1e1c', points: L.ideal, dashed: true, width: 1.4 });
    charts2.line(loopCv, {
      series, x: { label: 'commanded bend of segment one, degrees' },
      y: { label: 'tip sideways, mm', min: Math.floor(Math.min(...ys) / 10) * 10, max: Math.ceil(Math.max(...ys) / 10) * 10 },
      marker: { x: deg(live.sim.qCmd[1]) },
    });
  }
  let loopTick = 0, prevT = null;
  function frame(now) {
    if (prevT == null) prevT = now;
    const dt = Math.min(0.1, (now - prevT) / 1000); prevT = now;
    if (live.visible && live.sim) {
      let acc = dt;
      while (acc >= DT) { live.t += DT; live.sim.setCommand([S.qBase[0], kAt(live.t), S.qBase[2], S.qBase[3]]); live.sim.step(DT); acc -= DT; }
      drawLive();
      if (++loopTick % 4 === 0) drawLoop();
    }
    requestAnimationFrame(frame);
  }
  if (typeof IntersectionObserver === 'function') new IntersectionObserver((e) => { live.visible = e.some((x) => x.isIntersecting); }, { rootMargin: '120px' }).observe(view);
  const segButtons = (key, attr, onPick) => { for (const b of document.querySelectorAll('[data-hy="' + key + '"] button')) b.addEventListener('click', () => {
    for (const x of document.querySelectorAll('[data-hy="' + key + '"] button')) x.classList.toggle('on', x === b);
    onPick(Number(b.dataset[attr]));
  }); };
  segButtons('mode', 'mode', (m) => { live.mode = m; live.sim.tendonFilter = hy.est != null ? filterFor(m) : null; drawLoop(); });
  segButtons('speed', 'speed', (sp) => { live.speedIdx = sp; resetLive(); drawLoop(); showNumbers(); });
  resetLive(); drawLive();
  requestAnimationFrame(frame);
  setTimeout(() => { computeLoops(); resetLive(); drawLoop(); }, 60);

  // ================= layer three: control =================
  const a3 = kit.createScene(q('[data-scene="a3"]'), { id: 'v3-a3', views: ['inspector', 'side'], toolbar: ['plan'], charts: false, table: 6, clickToPlace: false });
  const hook = kit.T3(kit.TQ.hook);
  function runHook() { a3.sim.reset(); a3.demoTarget(hook); a3.note('running the target under the base, planner ' + (a3.sim.st.usePlan ? 'on' : 'off')); }
  a3.sim.on('event', (e) => { if (/^plan/.test(e.label)) setTimeout(runHook, 50); });
  // the verdict's two numbers come from the same target run headless, planner off then on
  function headlessHook(usePlan) {
    const sim = kit.createSim({ robots: ['classical'] });
    sim.st.usePlan = usePlan;
    let row = null;
    sim.on('trial', (r) => { row = r; });
    sim.setPlaneY(hook[2], false);
    sim.startTrial(hook);
    for (let i = 0; i < Math.round(kit.CONST.TRIAL_S / DT) + 2 && !row; i++) sim.step();
    return row && row.classical;
  }
  setTimeout(() => {
    const off = headlessHook(false), on = headlessHook(true);
    if (off) setV('stall', off.settle != null ? 'settled (' + off.settle.toFixed(2) + ' s)' : off.steady.toFixed(1) + ' mm');
    if (on) setV('plannedSettle', on.settle != null ? on.settle.toFixed(2) + ' s' : 'did not settle');
  }, 200);
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
    charts2.bars(cv, { groups, y: { max: 40 }, valueLabel: (v) => String(v), legend: [
      { label: 'classical, direct', color: '#eb6834', light: true }, { label: 'classical, planned', color: '#eb6834' },
      { label: 'learned, direct', color: '#2a78d6', light: true }, { label: 'learned, planned', color: '#2a78d6' },
    ] });
  }

  // ================= model card, free play, tour =================
  q('[data-modelcard]').innerHTML = kit.modelCard().replace('</dl>', q('#card-extra').innerHTML + '</dl>');
  const fp = q('details.freeplay');
  let freeScene = null;
  fp.addEventListener('toggle', () => { if (fp.open && !freeScene) freeScene = kit.createScene(q('[data-scene="a4"]'), { id: 'v3-free', charts: true, table: 10 }); });
  window.CR_TOUR = spotlight.createTour([
    { target: 'header .intro', title: 'The answer first', body: 'A geometric model: two constant-curvature arcs of fixed length, with lag, backlash, sag and drift added on top.' },
    { target: '[data-chart="loop"]', title: 'The backlash loop', body: 'One bend swept with no feedback. Grey is the loop without compensation; orange adds the inverse play, green inverts the lag as well. Both were identified from the cameras.', place: 'below' },
    { target: '[data-hy="speed"]', title: 'Faster', body: 'Three times faster, the inverse play leaves more, because what it leaves is the lag, which depends on speed; inverting the lag too closes it.' },
    { target: '[data-scene="a3"] [data-cr="tgl-plan"]', title: 'Inverse kinematics', body: 'Planner off: the feedback law alone stops short of a target under the base. Planner on: the inverse kinematics is solved on the model first and the law tracks it.' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'Invented coefficients, one idealised play operator, a lag that inverts cleanly only because the simulator is exactly first order, perfect cameras.', place: 'below' },
  ], { autoOpenOnce: false, storageKey: 'cr_tour_v3', button: '[data-tour-open]' });
  window.CR_VARIANT_READY = '3';
})();
