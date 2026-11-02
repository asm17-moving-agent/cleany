const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const {chromium} = require('../../ros2_ws/src/cleany_dev_monitor/web/node_modules/@playwright/test');

const root = path.resolve(process.argv[2] || 'ros2_ws/src/cleany_display/web');
const output = path.resolve('output/cleany-display-jetson/transitions');
let state = {mode: 'idle', connected: true};
const server = http.createServer((req, res) => {
  if (req.url === '/api/state') {
    res.setHeader('Content-Type', 'application/json');
    return res.end(JSON.stringify(state));
  }
  const name = req.url === '/' ? 'index.html' : req.url.slice(1);
  if (!['index.html', 'face.js', 'face.css'].includes(name)) {res.writeHead(404); return res.end();}
  res.setHeader('Content-Type', name.endsWith('.js') ? 'text/javascript' : name.endsWith('.css') ? 'text/css' : 'text/html');
  res.end(fs.readFileSync(path.join(root, name)));
});

async function main() {
  fs.mkdirSync(output, {recursive: true});
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({headless: true, channel: process.env.CLEANY_TEST_BROWSER || 'chrome'});
    const page = await browser.newPage({viewport: {width: 1280, height: 720}});
    const errors = [], results = {};
    page.on('pageerror', error => errors.push(error.message));
    // Drive real API updates between deterministic frames; inspect the rendered pixels.
    await page.addInitScript(() => {
      Math.random = () => .5;
      let pending = [], frame = 0, firstTranslation = true;
      window.requestAnimationFrame = callback => {pending.push(callback); return pending.length;};
      const originalTransform = CanvasRenderingContext2D.prototype.setTransform;
      const originalTranslate = CanvasRenderingContext2D.prototype.translate;
      CanvasRenderingContext2D.prototype.setTransform = function(...args) {
        if (this.canvas === document.querySelector('canvas')) firstTranslation = true;
        return originalTransform.apply(this, args);
      };
      CanvasRenderingContext2D.prototype.translate = function(...args) {
        originalTranslate.apply(this, args);
        if (this.canvas === document.querySelector('canvas') && firstTranslation) {
          firstTranslation = false;
          window.faceOrigin = this.getTransform().f;
        }
      };
      window.advance = count => {
        for (let i = 0; i < count; i++) {
          const callbacks = pending; pending = [];
          callbacks.forEach(callback => callback(1000 + frame * 1000 / 60));
          frame++;
        }
      };
      window.pixels = (region, blue = false) => {
        const [x, y, w, h] = region;
        const data = document.querySelector('canvas').getContext('2d').getImageData(x, y, w, h).data;
        const bottoms = Array(w).fill(-1);
        let minY = h, maxY = -1, count = 0;
        for (let row = 0; row < h; row++) for (let col = 0; col < w; col++) {
          const i = (row * w + col) * 4, r = data[i], g = data[i + 1], b = data[i + 2];
          if (g > 90 && (blue ? b > g * 1.04 : g > r * 1.3 && b > r)) {
            minY = Math.min(minY, row); maxY = Math.max(maxY, row); count++;
            bottoms[col] = row;
          }
        }
        const columns = bottoms.flatMap((v, i) => v >= 0 ? [i] : []);
        const center = columns.length ? Math.round((columns[0] + columns.at(-1)) / 2) : 0;
        return {count, height: maxY < 0 ? 0 : maxY - minY + 1,
          centerNotch: maxY < 0 ? 0 : maxY - bottoms[center]};
      };
    });
    const url = `http://127.0.0.1:${server.address().port}/`;
    const advance = count => page.evaluate(n => advance(n), count);
    async function select(mode) {
      state = {mode, connected: true};
      await page.waitForFunction(m => document.querySelector('#cleany-display').dataset.mode === m,
        mode, {polling: 10});
    }
    async function fresh(mode, frames) {
      state = {mode: 'idle', connected: true};
      await page.goto(url);
      await page.waitForFunction(() => document.querySelector('#cleany-display').dataset.connected === 'true', null, {polling: 10});
      await page.evaluate(() => document.fonts.ready);
      await page.waitForTimeout(30);
      await advance(2);
      if (mode !== 'idle') await select(mode);
      await advance(frames);
    }
    for (const mode of ['moving', 'checking', 'collecting', 'complete', 'charging', 'help']) {
      await fresh('idle', 147);
      const before = await page.evaluate(() => pixels([250, 100, 330, 420]));
      await select(mode); await advance(1);
      const after = await page.evaluate(() => pixels([250, 100, 330, 420]));
      assert.ok(before.height < 20 && after.height < 25, `Interrupted blink entering ${mode}: ${before.height} -> ${after.height}`);
      results['blink-' + mode] = {before, after};
    }
    await fresh('complete', 167);
    const beforeY = await page.evaluate(() => faceOrigin);
    await select('idle'); await advance(1);
    const afterY = await page.evaluate(() => faceOrigin);
    assert.ok(Math.abs(afterY - beforeY) < 5, `Face jumped ${afterY - beforeY}px`);
    await advance(5);
    const eye = await page.evaluate(() => pixels([250, 100, 330, 300]));
    assert.ok(eye.centerNotch < 5, `Happy eye morph has a ${eye.centerNotch}px notch`);
    await page.screenshot({path: path.join(output, 'complete-idle-100ms.png')});
    results.completeIdle = {beforeY, afterY, eye};

    await fresh('checking', 110);
    const mouthBefore = await page.evaluate(() => pixels([540, 415, 240, 140]));
    await advance(1);
    const mouthAfter = await page.evaluate(() => pixels([540, 415, 240, 140]));
    assert.ok(mouthAfter.count > mouthBefore.count * .75, 'Opening mouth abruptly lost its thickness');
    results.mouth = {mouthBefore, mouthAfter};

    await fresh('collecting', 96);
    const sweatBefore = await page.evaluate(() => pixels([100, 100, 1080, 430], true));
    await select('checking'); await advance(1);
    const sweatAfter = await page.evaluate(() => pixels([100, 100, 1080, 430], true));
    assert.ok(sweatBefore.count > 100 && sweatAfter.count > sweatBefore.count * .5, 'Sweat disappeared in one frame');
    await advance(45);
    assert.equal((await page.evaluate(() => pixels([100, 100, 1080, 430], true))).count, 0);
    results.sweat = {sweatBefore, sweatAfter};

    await fresh('charging', 120);
    await page.waitForTimeout(300);
    await select('idle'); await advance(1);
    const fadingDreams = await page.locator('.cd-dreams').evaluate(el => ({hidden: el.hidden, opacity: Number(getComputedStyle(el).opacity)}));
    assert.equal(fadingDreams.hidden, false);
    assert.ok(fadingDreams.opacity > 0);
    await page.waitForFunction(() => document.querySelector('.cd-dreams').hidden, null, {polling: 10});
    results.dreams = fadingDreams;

    // Interrupt a text swap before it finishes; an older callback must never restore it.
    await fresh('moving', 60);
    await page.waitForTimeout(400);
    await select('checking'); await advance(1);
    await page.locator('.cd-caption').evaluate(el => el.getAnimations().forEach(animation => animation.pause()));
    assert.equal(await page.locator('.cd-message').textContent(), '지나갈게요');
    state = {mode: 'idle', connected: true};
    await page.waitForFunction(() => document.querySelector('#cleany-display').dataset.mode === 'idle', null, {polling: 10});
    await page.waitForTimeout(500);
    assert.equal(await page.locator('.cd-caption').evaluate(el => el.hidden), true);
    await select('help'); await advance(60);
    await page.waitForTimeout(300);
    assert.equal(await page.locator('.cd-help').evaluate(el => getComputedStyle(el).opacity), '1');
    const helpText = await page.locator('.cd-help-request').textContent();
    await select('checking'); await advance(1);
    assert.equal(await page.locator('.cd-help-request').textContent(), helpText);
    assert.equal(await page.locator('.cd-caption').evaluate(el => el.hidden), true);
    await page.waitForTimeout(400);
    assert.equal(await page.locator('.cd-help').evaluate(el => el.hidden), true);
    assert.equal(await page.locator('.cd-caption').evaluate(el => getComputedStyle(el).opacity), '1');
    await fresh('help', 60);
    await page.waitForTimeout(300);
    await page.emulateMedia({reducedMotion: 'reduce'});
    await select('charging'); await advance(2);
    await page.waitForTimeout(50);
    assert.equal(await page.locator('.cd-help').evaluate(el => el.hidden), true);
    assert.deepEqual(errors, []);
    fs.writeFileSync(path.join(output, 'verification.json'), JSON.stringify({results, errors}, null, 2));
    console.log('PASS: blink continuity across 6 modes, eye/mouth shape, bob position, effect fade, interrupted text and reduced motion');
  } finally {
    if (browser) await browser.close();
    server.close();
  }
}
main().catch(error => {console.error(error); process.exitCode = 1;});
