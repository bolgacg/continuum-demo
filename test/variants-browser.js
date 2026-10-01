// Browser checks for the variant pages (Playwright, headless Chromium):
// no page errors, the variant reports ready, no horizontal overflow at 1536
// and 390 px, screenshots per act, the walkthrough walks, a click places a
// target that produces a trial row, and the frame rate is reported.
// Run: node test/variants-browser.js [n ...]   (default: all built variants,
// the main page index.html as variant 3, and the chooser; "main" = the main page only)
'use strict';
const fs = require('fs');
const path = require('path');
const http = require('http');
const { chromium } = require('/home/bolgac/projects/minimalist-workout-app/node_modules/playwright');

const ROOT = path.join(__dirname, '..');
const OUT = process.env.SHOTS || path.join(process.env.SCRATCH || '/tmp', 'variants-shots');
fs.mkdirSync(OUT, { recursive: true });
const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json', '.png': 'image/png' };

function serve() {
  return new Promise((resolve) => {
    const srv = http.createServer((req, res) => {
      const p = path.join(ROOT, decodeURIComponent(req.url.split('?')[0].replace(/\/$/, '/index.html')));
      if (!p.startsWith(ROOT) || !fs.existsSync(p) || fs.statSync(p).isDirectory()) { res.writeHead(404); res.end(); return; }
      res.writeHead(200, { 'Content-Type': MIME[path.extname(p)] || 'application/octet-stream' });
      fs.createReadStream(p).pipe(res);
    });
    srv.listen(0, '127.0.0.1', () => resolve(srv));
  });
}

let failures = 0;
function check(name, cond, detail) {
  if (cond) console.log('  ok  ' + name + (detail ? '  (' + detail + ')' : ''));
  else { console.log('FAIL  ' + name + (detail ? '  (' + detail + ')' : '')); failures++; }
}

async function testVariant(browser, base, arg) {
  const main = arg === 'main', n = main ? '3' : arg, url = main ? '/index.html' : '/variants/' + n + '.html';
  const label = main ? 'main(v3)' : 'v' + n;
  for (const width of [1536, 390]) {
    const page = await browser.newPage({ viewport: { width, height: width > 800 ? 960 : 844 } });
    const errors = [];
    page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));
    page.on('console', (m) => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
    await page.goto(base + url);
    await page.waitForFunction(() => window.CR_VARIANT_READY, null, { timeout: 60000 }).catch(() => {});
    const ready = await page.evaluate(() => window.CR_VARIANT_READY);
    check(label + ' @' + width + ' ready', ready === String(n), String(ready));
    await page.waitForTimeout(2500);
    await page.evaluate(() => { if (window.CR_TOUR) window.CR_TOUR.close(); });
    await page.waitForTimeout(300);
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth - document.documentElement.clientWidth);
    check(label + ' @' + width + ' no horizontal overflow', overflow <= 0, overflow + ' px');
    const acts = await page.$$('[data-act]');
    let i = 0;
    for (const act of acts) { i++; await act.scrollIntoViewIfNeeded(); await page.waitForTimeout(600); await act.screenshot({ path: path.join(OUT, label + '-' + width + '-act' + i + '.png') }).catch(() => {}); }
    await page.screenshot({ path: path.join(OUT, label + '-' + width + '-full.png'), fullPage: true });
    if (width > 800) {
      // walkthrough
      const steps = await page.evaluate(() => (window.CR_TOUR ? window.CR_TOUR.steps.length : 0));
      const minSteps = n === '3' ? 5 : 10; // variant 3 has a deliberately short 5-step tour
      check(label + ' tour has steps', steps >= minSteps, steps + ' steps');
      let bad = 0;
      for (let s = 0; s < steps; s++) {
        await page.evaluate((k) => window.CR_TOUR.goto(k), s);
        await page.waitForTimeout(700);
        const geo = await page.evaluate(() => {
          const hl = document.querySelector('.cr-tour-hl').getBoundingClientRect();
          const card = document.querySelector('.cr-tour-card').getBoundingClientRect();
          return { hlTop: hl.top, hlBottom: hl.bottom, cardTop: card.top, cardBottom: card.bottom, vh: innerHeight, hlH: hl.height };
        });
        const ok = geo.hlTop >= -4 && geo.hlBottom <= geo.vh + 4 && (geo.cardTop >= geo.hlBottom - 2 || geo.cardTop >= geo.hlTop) && geo.cardBottom <= geo.vh + 300;
        if (!ok) { bad++; console.log('      step ' + (s + 1) + ' geometry ' + JSON.stringify(geo)); }
        if (s === 0 || s === 3 || s === steps - 1) await page.screenshot({ path: path.join(OUT, label + '-tour' + (s + 1) + '.png') });
      }
      check(label + ' tour steps sit in the viewport above their card', bad === 0, bad + ' bad of ' + steps);
      await page.evaluate(() => window.CR_TOUR.close());
      // click to place in the free-play scene (falls back to the first scene with an inspector);
      // a collapsed free-play section builds its scene when opened
      await page.evaluate(() => { const d = document.querySelector('details.freeplay'); if (d && !d.open) d.open = true; });
      await page.waitForTimeout(1500);
      const insp = (await page.$('[data-scene="a4"] [data-cr="inspector"]')) || (await page.$('[data-scene="free"] [data-cr="inspector"]')) || (await page.$('[data-scene] [data-cr="inspector"]'));
      if (insp) {
        await insp.scrollIntoViewIfNeeded();
        const box = await insp.boundingBox();
        await page.mouse.click(box.x + box.width * 0.52, box.y + box.height * 0.45);
        await page.waitForTimeout(6800);
        const rows = await page.evaluate(() => { const b = document.querySelector('[data-scene="a4"] [data-cr="rows"]') || document.querySelector('[data-scene="free"] [data-cr="rows"]') || document.querySelector('[data-scene] [data-cr="rows"]'); return b ? b.querySelectorAll('tr:not([data-cr="empty"])').length : -1; });
        check(label + ' click places a target and a trial row appears', rows > 0, rows + ' rows');
      }
      const fps = await page.evaluate(() => CR.kit.ticker().fps());
      console.log('      ' + label + ' fps ' + fps);
    }
    check(label + ' @' + width + ' no page errors', errors.length === 0, errors.slice(0, 3).join(' | '));
    await page.close();
  }
}

async function testChooser(browser, base) {
  const page = await browser.newPage({ viewport: { width: 1536, height: 960 } });
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  await page.goto(base + '/variants/index.html');
  await page.waitForTimeout(5000);
  const heights = await page.evaluate(() => [...document.querySelectorAll('iframe')].map((f) => f.getBoundingClientRect().height));
  check('chooser iframes sized from the child', heights.every((h) => h > 1000), heights.map((h) => Math.round(h)).join(','));
  const statuses = await page.evaluate(() => [...document.querySelectorAll('[data-status]')].map((s) => s.textContent));
  check('chooser frames report', statuses.every((s) => /running|paused/.test(s)), statuses.join(' | '));
  await page.screenshot({ path: path.join(OUT, 'chooser-top.png') });
  // stress: all in view
  await page.goto(base + '/variants/index.html#all');
  await page.waitForTimeout(5000);
  const fpsAll = await page.evaluate(() => document.querySelector('[data-fps]').textContent);
  console.log('      chooser fps with #all: ' + fpsAll);
  check('chooser no page errors', errors.length === 0, errors.slice(0, 3).join(' | '));
  await page.close();
}

(async () => {
  const srv = await serve();
  const base = 'http://127.0.0.1:' + srv.address().port;
  const browser = await chromium.launch();
  const want = process.argv.slice(2).length ? process.argv.slice(2) : ['1', '2', '3', '4', '5'].filter((n) => fs.existsSync(path.join(ROOT, 'variants', n + '.html'))).concat(['main']);
  for (const n of want) await testVariant(browser, base, n);
  if (fs.existsSync(path.join(ROOT, 'variants', 'index.html'))) await testChooser(browser, base);
  await browser.close();
  srv.close();
  console.log(failures ? failures + ' FAILURE(S); screenshots in ' + OUT : 'all browser checks passed; screenshots in ' + OUT);
  process.exit(failures ? 1 : 0);
})();
