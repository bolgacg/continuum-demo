// Variant 4: three experiments the reader can rerun. Each act has a Run button.
(function () {
  'use strict';
  if (!document.getElementById('variant-4')) return;
  const { kit, charts2, pcc, workspace, camera, v3, spotlight, protocol } = CR;
  const A = kit.assets();
  const { MM } = kit.CONST;
  const q = (s) => document.querySelector(s);
  const setV = (k, v) => { for (const el of document.querySelectorAll('[data-v="' + k + '"]')) el.textContent = v; };
  const setH = (k, v) => { const el = q('[data-h="' + k + '"]'); if (el) el.textContent = v; };
  const progress = (n, text) => { const el = q('[data-progress="' + n + '"]'); if (el) el.textContent = text || ''; };
  window.CR_EMBED = kit.embed({ id: '4' });
  kit.schematic(q('[data-schematic]'));
  function randomQ(rng, lo, hi) { const out = []; for (let i = 0; i < 2; i++) { const a = rng() * 2 * Math.PI, k = (lo + (hi - lo) * Math.sqrt(rng())) * pcc.KMAX[i]; out.push(k * Math.cos(a), k * Math.sin(a)); } return out; }

  // ================= audit one: the boundary =================
  const a1 = kit.createScene(q('[data-scene="a1"]'), { id: 'v4-a1', views: ['inspector', 'side'], toolbar: false, charts: false, table: false, readouts: false, preset: 'top', planeSlider: false, clickToPlace: false });
  const scatterCv = q('[data-chart="scatter"]');
  function runAudit1() {
    const g = A.geometry(pcc.flex());
    const h = a1.sim.st.planeY;
    const rng = CR.makeRng(21);
    const pts = [];
    let agreeGrid = 0, agreeCage = 0;
    for (let i = 0; i < 400; i++) {
      const p = [-1.7 + 3.4 * rng(), -1.7 + 3.4 * rng(), h];
      const ik = A.planner.solveIK(p, [0, 0, 0, 0]).reachable;
      const inG = workspace.gridContains(g.grid, p), inC = workspace.insideEnvelope(g.volume, p);
      if (inG === ik) agreeGrid++;
      if (inC === ik) agreeCage++;
      pts.push({ x: p[0] * MM, y: p[1] * MM, cls: inG === ik ? 'agree' : 'disagree', ring: ik });
    }
    const outline = workspace.gridSectionSegments(g.grid, h).map(([p, r]) => [[p[0] * MM, p[1] * MM], [r[0] * MM, r[1] * MM]]);
    charts2.scatter(scatterCv, { points: pts, classes: { agree: { color: '#9a9d97', label: 'outline and solver agree' }, disagree: { color: '#eb6834', label: 'disagree' } }, x: { min: -170, max: 170 }, y: { min: -170, max: 170 }, outline });
    const P = 100 * agreeGrid / 400, Q = 100 * agreeCage / 400;
    setV('agreeP', P.toFixed(1)); setV('agreeN', '400'); setV('agreeZ', (h * MM).toFixed(0) + ' mm'); setV('cageP', Q.toFixed(1));
    setV('v1open', P >= 95 ? 'The prediction holds: the outline is the solver\'s answer to within its resolution.' : 'The prediction fails: the outline and the solver disagree on more than 5 percent of points.');
    setH('agree', P.toFixed(0) + ' percent agree, cage ' + Q.toFixed(0));
    progress(1, 'done: 400 points classified three ways.');
  }
  q('[data-run="1"]').addEventListener('click', () => { progress(1, 'running...'); setTimeout(runAudit1, 30); });
  charts2.scatter(scatterCv, { points: [], classes: { agree: { color: '#9a9d97', label: 'outline and solver agree' }, disagree: { color: '#eb6834', label: 'disagree' } }, x: { min: -170, max: 170 }, y: { min: -170, max: 170 },
    outline: workspace.gridSectionSegments(A.geometry(pcc.flex()).grid, a1.sim.st.planeY).map(([p, r]) => [[p[0] * MM, p[1] * MM], [r[0] * MM, r[1] * MM]]) });

  // ================= audit two: the geometry =================
  const a2 = kit.createScene(q('[data-scene="a2"]'), { id: 'v4-a2', views: ['inspector'], robots: ['classical'], readouts: false, toolbar: false, charts: false, table: false, tendons: true, cages: false, plane: false, clickToPlace: false, preset: 'iso', dist: 2.4 });
  const rangeCv = q('[data-chart="range"]');
  let poseCycle = null;
  a2.on('draw', () => {
    if (!poseCycle) return;
    if (++poseCycle.tick % 8 === 0) { poseCycle.i = (poseCycle.i + 1) % poseCycle.qs.length; a2.sim.robots.classical.sim.setCommand(poseCycle.qs[poseCycle.i]); }
  });
  function runAudit2() {
    const rng = CR.makeRng(4);
    const camSide = a2.sim.camSide, camTop = a2.sim.camTop;
    const pairs = [[0, 1], [1, 2], [2, 3]];
    const ch = pairs.map(() => ({ min: Infinity, max: -Infinity, sum: 0 }));
    const px1 = { min: Infinity, max: -Infinity }, px2 = { min: Infinity, max: -Infinity };
    const qs = [];
    for (let i = 0; i < 500; i++) {
      const cfg = randomQ(rng, 0, 1);
      if (i % 10 === 0) qs.push(cfg);
      const m = pcc.markers3(cfg);
      pairs.forEach(([a, b], j) => { const d = v3.norm(v3.sub(m[a], m[b])); ch[j].min = Math.min(ch[j].min, d); ch[j].max = Math.max(ch[j].max, d); ch[j].sum += d; });
      const s1 = [camSide.project(m[0]), camSide.project(m[1])], s2 = [camTop.project(m[0]), camTop.project(m[1])];
      if (s1[0] && s1[1]) { const d = Math.hypot(s1[0][0] - s1[1][0], s1[0][1] - s1[1][1]); px1.min = Math.min(px1.min, d); px1.max = Math.max(px1.max, d); }
      if (s2[0] && s2[1]) { const d = Math.hypot(s2[0][0] - s2[1][0], s2[0][1] - s2[1][1]); px2.min = Math.min(px2.min, d); px2.max = Math.max(px2.max, d); }
    }
    const arc = pcc.SEG_LEN[0] + pcc.SEG_LEN[1];
    // each pair spans half a segment; its chord shortens by at most 1 - sin(f/2)/(f/2) at that half's bending limit
    const segOf = [0, 1, 1];
    const bound = segOf.map((s) => { const f = pcc.KMAX[s] * pcc.SEG_LEN[s] / 2; return 100 * (1 - Math.sin(f / 2) / (f / 2)); });
    const short = pairs.map((_, j) => { const arcPair = pcc.SEG_LEN[segOf[j]] / 2; return 100 * (arcPair - ch[j].min) / arcPair; });
    const rows = [
      { label: 'arc length', min: 1, max: 1, color: '#52514e', text: (arc * MM).toFixed(1) + ' to ' + (arc * MM).toFixed(1) + ' mm' },
      ...pairs.map(([a, b], j) => ({ label: 'chord m' + (a + 1) + ' to m' + (b + 1), min: 1, max: ch[j].max / ch[j].min, color: '#eb6834', text: (ch[j].min * MM).toFixed(1) + ' to ' + (ch[j].max * MM).toFixed(1) + ' mm' })),
      { label: 'CAM 01 spacing m1 to m2', min: 1, max: px1.max / Math.max(px1.min, 0.05), color: '#2a78d6', text: px1.min.toFixed(1) + ' to ' + px1.max.toFixed(1) + ' px' },
      { label: 'CAM 02 spacing m1 to m2', min: 1, max: px2.max / Math.max(px2.min, 0.05), color: '#2a78d6', text: px2.min.toFixed(1) + ' to ' + px2.max.toFixed(1) + ' px' },
    ];
    charts2.rangeLog(rangeCv, { rows, x: { label: 'largest over smallest, log scale', floor: 1 } });
    const f1 = px1.max / Math.max(px1.min, 0.05), f2 = px2.max / Math.max(px2.min, 0.05);
    setV('arcRange', (arc * MM).toFixed(1) + ' to ' + (arc * MM).toFixed(1) + ' mm');
    setV('chord1', short[0].toFixed(1)); setV('chord2', short[1].toFixed(1)); setV('chord3', short[2].toFixed(1));
    setV('bound1', bound[0].toFixed(1)); setV('bound2', bound[1].toFixed(1)); setV('bound3', bound[2].toFixed(1));
    setV('px1', px1.min.toFixed(1) + ' to ' + px1.max.toFixed(1) + ' px, a factor of ' + f1.toFixed(1)); setV('px2', px2.min.toFixed(1) + ' to ' + px2.max.toFixed(1) + ' px, a factor of ' + f2.toFixed(1));
    const chordsHold = short.every((s, j) => s <= bound[j] + 0.05), screenHolds = f1 > 2 && f2 > 2;
    setV('v2open', chordsHold && screenHolds ? 'The prediction holds: every chord stayed within its geometric bound and both screens varied by far more.' : chordsHold ? 'The chords held their bounds; a screen varied by less than a factor of two, read the ranges.' : 'A chord shortened beyond its geometric bound, which would mean the model is not what this page says it is; read the numbers.');
    setH('chord', short[0].toFixed(1) + ' percent at most against ×' + f1.toFixed(0) + ' on screen');
    progress(2, 'done: 500 configurations measured; the robot above steps through 50 of them.');
    poseCycle = { qs, i: 0, tick: 0 };
    a2.sim.st.target = null;
  }
  q('[data-run="2"]').addEventListener('click', () => { progress(2, 'running...'); setTimeout(runAudit2, 30); });
  charts2.rangeLog(rangeCv, { rows: [{ label: 'press Run', min: 1, max: 1, color: '#c3c2b7', text: '' }], x: { label: 'largest over smallest, log scale', floor: 1 } });

  // ================= audit three: the planner =================
  const a3 = kit.createScene(q('[data-scene="a3"]'), { id: 'v4-a3', views: ['inspector', 'side'], toolbar: false, charts: false, table: 10, clickToPlace: false });
  const dotsCv = q('[data-chart="dots"]');
  let job = null;
  function drawDots(rows, jobs) {
    const groups = [['classical', 'direct', '#eb6834', 'classical, direct'], ['classical', 'planned', '#eb6834', 'classical, planned'], ['learned', 'direct', '#2a78d6', 'learned, direct'], ['learned', 'planned', '#2a78d6', 'learned, planned']]
      .map(([k, m, color, label]) => ({ label, color, values: jobs.filter((j) => j.kind === k && j.mode === m && j.result).map((j) => j.result.ss * MM) }));
    charts2.dots(dotsCv, { groups, y: { max: 60 }, hairline: { y: 5, label: '5 mm' } });
  }
  function runAudit3() {
    if (job) return;
    const nominal = protocol.CONDITIONS[0];
    const seen = new Set();
    job = kit.runTrials({ trials: 10, targetSets: ['edge'], conditions: [nominal], controllers: ['classical', 'learned'], modes: ['direct', 'planned'] }, ({ done, total, jobs }) => {
      progress(3, 'running ' + done + ' of ' + total + ' trials...');
      drawDots(null, jobs);
      // replay each new target in the scene, illustratively
      for (const j of jobs) if (j.result && j.result.target && !seen.has(j.n)) { seen.add(j.n); if (seen.size === 1) { a3.sim.reset(); a3.demoTarget(j.result.target); } }
    });
    job.promise.then(({ rows, jobs }) => {
      drawDots(rows, jobs);
      const cnt = (k, m) => jobs.filter((j) => j.kind === k && j.mode === m && j.result && j.result.settle != null).length;
      const cd = cnt('classical', 'direct'), ld = cnt('learned', 'direct'), cp = cnt('classical', 'planned'), lp = cnt('learned', 'planned');
      setV('cd', String(cd)); setV('ld', String(ld)); setV('cp', String(cp)); setV('lp', String(lp));
      setV('v3open', cp === 10 && lp === 10 && cd < 10 ? 'The prediction holds: planned, both laws settle every target; direct, the classical law loses some.' : 'Read the counts: the prediction did not hold in full on these ten seeds.');
      setH('plan', cd + ' of 10 against ' + cp + ' of 10');
      progress(3, 'done: 40 trials, the first ten seeds of the protocol, all four conditions.');
      // replay the ten targets one after another in the scene
      const targets = jobs.filter((j) => j.kind === 'classical' && j.mode === 'planned' && j.result).map((j) => j.result.target);
      let i = 0;
      const next = () => { if (i >= targets.length) return; a3.demoTarget(targets[i++]); };
      a3.sim.on('trial', next);
      a3.sim.reset(); next();
      job = null;
    });
  }
  q('[data-run="3"]').addEventListener('click', runAudit3);
  drawDots(null, []);

  // ================= model card, free play, tour =================
  q('[data-modelcard]').innerHTML = kit.modelCard();
  kit.createScene(q('[data-scene="a4"]'), { id: 'v4-free', charts: true, table: 10 });
  setInterval(() => { const el = q('[data-fps]'); if (el) el.textContent = kit.ticker().fps() + ' fps'; }, 1000);
  window.CR_TOUR = spotlight.createTour([
    { target: 'header h1', title: 'Three claims, three tests', body: 'Each answer given about version 1 is a claim; each act below is its test, run in this browser on the code that runs the robots. Press Run and the verdict is written from the result.' },
    { target: '[data-schematic]', title: 'What the cameras see', body: 'Two fixed sensors, four markers, a target plane, and the click ray a click makes.', place: 'below' },
    { target: '.questions', title: 'Their questions', body: 'Three questions, three answers, quoted as asked; the audits test the answers.' },
    { target: '[data-act="1"] .hyp', title: 'Audit one, the hypothesis', body: 'Prediction, baseline and caveat are written before the run, so the result cannot be read into the hypothesis afterwards.' },
    { target: '[data-run="1"]', title: 'Audit one, Run', body: 'Press it: 400 random points on the plane, each classified by the outline, by the cage and by the solver.' },
    { target: '[data-chart="scatter"]', title: 'Audit one, the scatter', body: 'Disagreements sit on the boundary itself, where a 6 mm cell straddles the edge.', place: 'below' },
    { target: '[data-run="2"]', title: 'Audit two, Run', body: 'Five hundred random configurations; the robot steps through fifty of them while the numbers are measured.' },
    { target: '[data-chart="range"]', title: 'Audit two, the range chart', body: 'The arc-length bar is a dot; the chord bars are short; the on-screen bars are long. That is the whole answer to the second question.', place: 'below' },
    { target: '[data-run="3"]', title: 'Audit three, Run', body: 'The evaluation protocol on its first ten seeds, both controllers, with and without the plan, about ten seconds.' },
    { target: '[data-chart="dots"]', title: 'Audit three, the dots', body: 'One dot per trial. Dots above the band are stalls; the planned columns sit on the band.', place: 'below' },
    { target: '[data-modelcard]', title: 'The model card', body: 'The blue controller is the one trained thing on this page.', place: 'below' },
    { target: '.failpane', title: 'What this page gets wrong', body: 'Ten trials check the protocol, they do not replace 40; the outline is tested against the solver, not the truth; the cameras are perfect.', place: 'below' },
  ], { autoOpenOnce: true, storageKey: 'cr_tour_v4', button: '[data-tour-open]' });
  window.CR_VARIANT_READY = '4';
})();
