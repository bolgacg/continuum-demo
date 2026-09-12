// Interaction checks for the variants: presses every Run button (v4), walks the stops (v5),
// flips the switches (v2), and prints the verdicts the page computed. Screenshots go to
// $SCRATCH/variants-shots. Usage: node test/variants-interact.js [2] [4] [5]
const path = require('path');
const http = require('http');
const fs = require('fs');
const { chromium } = require(process.env.PW || '/home/bolgac/projects/minimalist-workout-app/node_modules/playwright');
const ROOT = path.join(__dirname, '..');
const OUT = path.join(process.env.SCRATCH || '/tmp', 'variants-shots');
fs.mkdirSync(OUT, { recursive: true });
const which = process.argv.slice(2).map(Number).filter(Boolean);
const want = (n) => !which.length || which.includes(n);
let failures = 0;
const check = (name, ok, detail) => { console.log((ok ? '  ok  ' : 'FAIL  ') + name + (detail ? '  (' + detail + ')' : '')); if (!ok) failures++; };
const server = http.createServer((req, res) => {
  const f = path.join(ROOT, decodeURIComponent(req.url.split('?')[0]));
  fs.readFile(f, (err, data) => { if (err) { res.writeHead(404); res.end(); return; } res.writeHead(200, { 'Content-Type': f.endsWith('.js') ? 'text/javascript' : f.endsWith('.css') ? 'text/css' : 'text/html' }); res.end(data); });
});
(async () => {
  await new Promise((r) => server.listen(0, r));
  const base = 'http://127.0.0.1:' + server.address().port + '/variants/';
  const browser = await chromium.launch();
  const open = async (n) => {
    const page = await browser.newPage({ viewport: { width: 1536, height: 900 } });
    await page.addInitScript((k) => { try { localStorage.setItem(k, 'done'); } catch (e) { /* no storage */ } }, 'cr_tour_v' + n);
    const errors = [];
    page.on('pageerror', (e) => errors.push(String(e)));
    page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
    await page.goto(base + n + '.html');
    await page.waitForFunction(() => window.CR_VARIANT_READY, null, { timeout: 30000 });
    await page.evaluate(() => window.CR_TOUR && window.CR_TOUR.close());
    return { page, errors };
  };
  const text = (page, sel) => page.$eval(sel, (el) => el.textContent.replace(/\s+/g, ' ').trim());
  const shot = async (page, sel, name) => { const el = await page.$(sel); if (el) { await el.scrollIntoViewIfNeeded(); await el.screenshot({ path: path.join(OUT, name) }); } };

  if (want(2)) {
    const { page, errors } = await open(2);
    // scenes only run while in view: bring act one on screen before waiting for its first run
    await (await page.$('[data-scene="a1"]')).scrollIntoViewIfNeeded();
    await page.waitForTimeout(9000);
    await page.click('[data-switch="a1"] button[data-state="v1"]');
    await page.waitForTimeout(9000);
    const v1 = await text(page, '[data-act="1"] .verdict');
    check('v2 act 1 verdict has both states', !/…/.test(v1), v1.slice(0, 160));
    await shot(page, '[data-act="1"] .panel:has(canvas.cr-chart)', 'v2-act1-run.png');
    await page.click('[data-switch="a2"] button[data-state="v1"]');
    await page.waitForTimeout(9000);
    const v2 = await text(page, '[data-act="2"] .verdict');
    check('v2 act 2 verdict measured under both cameras', !/not measured|…/.test(v2), v2.slice(0, 220));
    await shot(page, '[data-act="2"] .panel:has(canvas.cr-chart)', 'v2-act2-run.png');
    await page.click('[data-switch="a3"] button[data-state="v1"]');
    await page.waitForTimeout(7000);
    await page.click('[data-switch="a3"] button[data-state="now"]');
    await page.waitForTimeout(7000);
    const v3 = await text(page, '[data-act="3"] .verdict');
    check('v2 act 3 verdict has both runs', !/…/.test(v3), v3.slice(0, 220));
    await shot(page, '[data-act="3"] .panel:has(canvas.cr-chart)', 'v2-act3-run.png');
    check('v2 no page errors during interaction', errors.length === 0, errors.slice(0, 3).join(' | '));
    await page.close();
  }
  if (want(4)) {
    const { page, errors } = await open(4);
    await page.click('[data-run="1"]');
    await page.waitForFunction(() => /done/.test(document.querySelector('[data-progress="1"]').textContent), null, { timeout: 60000 });
    const v1 = await text(page, '[data-act="1"] .verdict');
    const p1 = parseFloat((await text(page, '[data-v="agreeP"]')));
    check('v4 audit 1 agreement above 95 percent', p1 >= 95, v1.slice(0, 200));
    await shot(page, '[data-act="1"] .panel:has(canvas.cr-chart)', 'v4-act1-run.png');
    await page.click('[data-run="2"]');
    await page.waitForFunction(() => /done/.test(document.querySelector('[data-progress="2"]').textContent), null, { timeout: 60000 });
    const v2 = await text(page, '[data-act="2"] .verdict');
    check('v4 audit 2 verdict computed', !/…/.test(v2), v2.slice(0, 260));
    await page.waitForTimeout(1500);
    await shot(page, '[data-act="2"] .panel:has(canvas.cr-chart)', 'v4-act2-run.png');
    await shot(page, '[data-act="2"] .scene', 'v4-act2-scene-run.png');
    await page.click('[data-run="3"]');
    await page.waitForFunction(() => /done/.test(document.querySelector('[data-progress="3"]').textContent), null, { timeout: 180000 });
    const v3 = await text(page, '[data-act="3"] .verdict');
    const cd = await text(page, '[data-v="cd"]'), cp = await text(page, '[data-v="cp"]'), lp = await text(page, '[data-v="lp"]');
    check('v4 audit 3 planned settles all ten, direct classical loses some', cp === '10' && lp === '10' && cd !== '10', v3.slice(0, 220));
    await shot(page, '[data-act="3"] .panel:has(canvas.cr-chart)', 'v4-act3-run.png');
    const head = await text(page, '.cr-headlines');
    check('v4 headlines filled', !/press Run/.test(head), head.slice(0, 200));
    check('v4 no page errors during interaction', errors.length === 0, errors.slice(0, 3).join(' | '));
    await page.close();
  }
  if (want(5)) {
    const { page, errors } = await open(5);
    // stop one: click on the plane in the inspector
    const insp = await page.$('[data-scene="main"] [data-cr="inspector"]');
    await insp.scrollIntoViewIfNeeded();
    await page.waitForTimeout(400);
    const box = await insp.boundingBox();
    await page.mouse.click(box.x + box.width * 0.55, box.y + box.height * 0.42);
    await page.waitForTimeout(6800);
    const s1 = await text(page, '[data-stop="1"] .verdict');
    check('v5 stop 1 verdict classifies the click', /reachable|beyond reach/i.test(s1) && !/…/.test(s1), s1.slice(0, 220));
    await shot(page, '[data-stop="1"] .panel', 'v5-stop1-run.png');
    // stop two: set the scene, wait for the hold, drag to orbit
    await page.click('[data-go="2"]');
    await page.waitForTimeout(7500);
    const b2 = await insp.boundingBox();
    await page.mouse.move(b2.x + b2.width * 0.5, b2.y + b2.height * 0.5);
    await page.mouse.down();
    for (let i = 1; i <= 30; i++) { await page.mouse.move(b2.x + b2.width * 0.5 + i * 12, b2.y + b2.height * 0.5 + i * 3); await page.waitForTimeout(40); }
    await page.mouse.up();
    await page.waitForTimeout(800);
    const s2 = await text(page, '[data-stop="2"] .verdict');
    const move = parseFloat(await text(page, '[data-v="chordMove"]')), factor = parseFloat(await text(page, '[data-v="pxFactor"]'));
    check('v5 stop 2 chord still, spacing moved', move < 0.5 && factor > 1.05, s2.slice(0, 200));
    const pinned = await page.$eval('[data-scene="main"]', (el) => el.classList.contains('pinned'));
    check('v5 stop 2 controls pinned', pinned);
    await shot(page, '[data-stop="2"] .panel', 'v5-stop2-run.png');
    await shot(page, '[data-scene="main"]', 'v5-stop2-scene.png');
    // stop three: direct run, then flip the planner
    await page.click('[data-go="3"]');
    await page.waitForTimeout(7000);
    const stall = await text(page, '[data-v="stall"]');
    check('v5 stop 3 direct run stalled', /stalled/.test(stall), stall);
    const plan = await page.$('[data-scene="main"] [data-cr="tgl-plan"]');
    const enabled = await plan.evaluate((el) => getComputedStyle(el.closest('label')).pointerEvents !== 'none' && getComputedStyle(el.closest('label')).opacity === '1');
    check('v5 stop 3 planner switch is the active control', enabled);
    await plan.evaluate((el) => el.closest('label').click());
    await page.waitForTimeout(7000);
    const s3 = await text(page, '[data-stop="3"] .verdict');
    check('v5 stop 3 planned run settled', /settled in/.test(await text(page, '[data-v="plannedSettle"]')) || /\d s$/.test(await text(page, '[data-v="plannedSettle"]')), s3.slice(0, 220));
    await shot(page, '[data-stop="3"] .panel', 'v5-stop3-run.png');
    await page.click('[data-go="free"]');
    const unp = await page.$eval('[data-scene="main"]', (el) => !el.classList.contains('pinned'));
    check('v5 free play unpins', unp);
    check('v5 no page errors during interaction', errors.length === 0, errors.slice(0, 3).join(' | '));
    await page.close();
  }
  await browser.close();
  server.close();
  console.log(failures ? failures + ' FAILURE(S)' : 'all interaction checks passed');
  process.exit(failures ? 1 : 0);
})().catch((e) => { console.error(e); server.close(); process.exit(1); });
