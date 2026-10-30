const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const {chromium} = require('../../ros2_ws/src/cleany_dev_monitor/web/node_modules/@playwright/test');

const root = path.resolve(__dirname, '../../ros2_ws/src/cleany_display/web');
const output = path.resolve(__dirname, '../../output/cleany-display-jetson');
const modes = ['idle', 'moving', 'checking', 'collecting', 'complete', 'charging', 'help'];
const viewports = [
  {width: 1920, height: 1080}, {width: 1920, height: 1200},
  {width: 640, height: 480}, {width: 3840, height: 2160},
];
let state = {mode: 'idle', connected: true, direction: 'forward'};
let online = true;

const server = http.createServer((req, res) => {
  if (req.url === '/api/state') {
    if (!online) { res.writeHead(503); return res.end(); }
    res.setHeader('Content-Type', 'application/json');
    return res.end(JSON.stringify(state));
  }
  const name = req.url === '/' ? 'index.html' : req.url.slice(1);
  if (!['index.html', 'face.js', 'face.css'].includes(name)) {
    res.writeHead(404);
    return res.end();
  }
  res.setHeader('Content-Type', name.endsWith('.js') ? 'text/javascript'
    : name.endsWith('.css') ? 'text/css' : 'text/html');
  res.end(fs.readFileSync(path.join(root, name)));
});

async function measurePaints(page, duration = 1200) {
  return page.evaluate(duration => new Promise(resolve => {
    const context = document.querySelector('canvas').getContext('2d');
    const original = context.setTransform;
    const dreams = document.querySelector('.cd-dreams').getAnimations({subtree: true});
    const dreamStart = dreams[0]?.currentTime;
    let count = 0;
    const start = performance.now();
    context.setTransform = function(...args) {
      count++;
      return original.apply(this, args);
    };
    setTimeout(() => {
      context.setTransform = original;
      resolve({fps: count * 1000 / (performance.now() - start),
        dreamElapsed: dreamStart == null ? null : dreams[0].currentTime - dreamStart,
        dreamStates: dreams.map(animation => animation.playState)});
    }, duration);
  }), duration);
}

async function main() {
  fs.mkdirSync(output, {recursive: true});
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({headless: true, channel: process.env.CLEANY_TEST_BROWSER || 'chrome'});
    const page = await browser.newPage({viewport: viewports[0]});
    const errors = [], external = [], results = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', req => {
      if (!req.url().startsWith('http://127.0.0.1:')) external.push(req.url());
    });
    const url = `http://127.0.0.1:${server.address().port}/`;
    await page.goto(url);
    for (const mode of modes) {
      state = {mode, connected: true, direction: 'forward', battery_percent: 72,
        message: mode === 'help' ? '상태를 확인해 주세요' : ''};
      await page.waitForFunction(mode => document.querySelector('#cleany-display').dataset.mode === mode, mode);
      const timing = await measurePaints(page);
      assert.ok(timing.fps >= 50 && timing.fps <= 62, `${mode}: ${timing.fps} canvas fps`);
      if (mode === 'charging') {
        assert.ok(timing.dreamElapsed >= 1000 && timing.dreamElapsed <= 1400);
        assert.deepEqual(timing.dreamStates, ['running', 'running', 'running']);
      }
      const geometry = await page.locator('.cd-scene').boundingBox();
      assert.equal(geometry.width, 1920);
      assert.equal(geometry.height, 1080);
      assert.equal(await page.locator('button,select,input').count(), 0);
      await page.screenshot({path: path.join(output, `${mode}.png`)});
      results.push({mode, geometry, timing});
    }
    for (const viewport of viewports) {
      await page.setViewportSize(viewport);
      const box = await page.locator('.cd-scene').boundingBox();
      assert.ok(Math.abs(box.width / box.height - 16 / 9) < .001);
      assert.ok(box.x >= 0 && box.y >= 0 && box.x + box.width <= viewport.width + 1
        && box.y + box.height <= viewport.height + 1);
      const canvas = await page.locator('canvas').evaluate(c => ({width: c.width, height: c.height,
        alpha: c.getContext('2d').getContextAttributes().alpha}));
      assert.ok(canvas.width <= 1280);
      assert.equal(canvas.alpha, false);
      results.push({viewport, box, canvas});
    }
    const retina = await browser.newContext({viewport: viewports[0], deviceScaleFactor: 2});
    const retinaPage = await retina.newPage();
    await retinaPage.goto(url);
    await retinaPage.waitForFunction(() => document.querySelector('canvas').width === 1280);
    state = {mode: 'charging', connected: true, rendering: {max_canvas_width: 960, max_pixel_ratio: 2, max_fps: 15}};
    await retinaPage.waitForFunction(() => document.querySelector('canvas').width === 960);
    const configuredTiming = await measurePaints(retinaPage);
    assert.ok(configuredTiming.fps >= 12 && configuredTiming.fps <= 17);
    assert.deepEqual(configuredTiming.dreamStates, ['paused', 'paused', 'paused']);
    assert.ok(configuredTiming.dreamElapsed >= 1000 && configuredTiming.dreamElapsed <= 1400);
    results.push({configuredTiming});
    assert.equal(await retinaPage.locator('meta[name="google"]').getAttribute('content'), 'notranslate');
    await retina.close();
    state = {mode: 'idle', connected: true, rendering: {max_canvas_width: 1280, max_pixel_ratio: 2, max_fps: 60}};
    await page.waitForFunction(() => document.querySelector('canvas').width === 1280);

    await page.waitForFunction(() => document.querySelector('#cleany-display').dataset.mode === 'idle');
    const mutations = await page.evaluate(() => new Promise(resolve => {
      let count = 0;
      const observer = new MutationObserver(records => { count += records.length; });
      observer.observe(document.querySelector('#cleany-display'), {subtree: true, attributes: true, childList: true});
      setTimeout(() => { observer.disconnect(); resolve(count); }, 900);
    }));
    assert.equal(mutations, 0, 'Unchanged status should not rewrite the DOM');
    await page.emulateMedia({reducedMotion: 'reduce'});
    await page.waitForTimeout(100);
    const frozen = await page.locator('canvas').evaluate(c => c.toDataURL());
    await page.waitForTimeout(300);
    assert.equal(await page.locator('canvas').evaluate(c => c.toDataURL()), frozen);
    await page.emulateMedia({reducedMotion: 'no-preference'});
    await page.waitForTimeout(400);
    assert.notEqual(await page.locator('canvas').evaluate(c => c.toDataURL()), frozen);

    // A real 60Hz panel can run slightly above 60Hz. Avoid 3-frame/1-frame judder.
    state = {mode: 'idle', connected: true, rendering: {max_fps: 30}};
    const cadencePage = await browser.newPage({viewport: {width: 640, height: 360}});
    await cadencePage.addInitScript(() => {
      let pending = [], index = 0;
      const painted = [];
      window.requestAnimationFrame = callback => { pending.push(callback); return pending.length; };
      const original = CanvasRenderingContext2D.prototype.setTransform;
      CanvasRenderingContext2D.prototype.setTransform = function(...args) {
        if (this.canvas === document.querySelector('#cleany-display canvas')) painted.push(index);
        return original.apply(this, args);
      };
      window.checkCadence = () => {
        for (index = 0; index < 1200; index++) {
          const callbacks = pending; pending = [];
          callbacks.forEach(callback => callback(1000 + index * 1000 / 60.01));
        }
        const gaps = painted.slice(1).map((frame, i) => frame - painted[i]);
        return {paints: painted.length, minGap: Math.min(...gaps), maxGap: Math.max(...gaps)};
      };
    });
    await cadencePage.goto(url);
    await cadencePage.waitForResponse(response => response.url().endsWith('/api/state'));
    await cadencePage.waitForTimeout(50);
    const cadence = await cadencePage.evaluate(() => window.checkCadence());
    assert.equal(cadence.minGap, 2, JSON.stringify(cadence));
    assert.equal(cadence.maxGap, 2, JSON.stringify(cadence));
    results.push({cadence});
    await cadencePage.close();

    online = false;
    await page.waitForFunction(() => document.querySelector('.cd-help-request').textContent
      === '연결을 확인해 주세요', null, {timeout: 8000});
    online = true;
    state = {mode: 'idle', connected: true};
    await page.waitForFunction(() => document.querySelector('#cleany-display').dataset.mode === 'idle');
    assert.deepEqual(errors, []);
    assert.deepEqual(external, []);
    fs.writeFileSync(path.join(output, 'web-verification.json'), JSON.stringify({results, errors, external,
      unchangedStateMutations: mutations, reducedMotionResume: true, disconnectRecovery: true}, null, 2));
    console.log('PASS: 7 expressions at 60fps, Zzz timing, 4 viewports, 720p cap, live 15fps setting, motion resume, reconnect');
  } finally {
    if (browser) await browser.close();
    server.close();
  }
}
main().catch(error => { console.error(error); process.exitCode = 1; });
