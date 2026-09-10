const assert = require('node:assert/strict');
const {test} = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

// Run the shipped inline script with a fake DOM, clock and HTTP transport.
// These tests never connect to a robot or send real motor commands.
const html = fs.readFileSync(path.join(__dirname, '../src/main.cpp'), 'utf8')
  .split('R"HTML(')[1].split(')HTML";')[0];
const script = html.split('<script>')[1].split('</script>')[0];
const settle = async () => { for (let i = 0; i < 12; i++) await Promise.resolve(); };

function fixture() {
  class Element {
    constructor() { this.listeners = {}; this.style = {}; this.dataset = {}; this.value = '0'; this.textContent = ''; this.offsetWidth = 54; this.captures = new Set(); this.classes = new Set(); this.classList = {add: x => this.classes.add(x), remove: x => this.classes.delete(x)}; }
    addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
    emit(type, extra = {}) { const event = {pointerId: 1, pointerType: 'mouse', button: 0, buttons: 1, preventDefault() {}, ...extra}; for (const fn of this.listeners[type] || []) fn(event); }
    setPointerCapture(id) { this.captures.add(id); }
    hasPointerCapture(id) { return this.captures.has(id); }
    releasePointerCapture(id) { this.captures.delete(id); this.emit('lostpointercapture', {pointerId: id}); }
    getBoundingClientRect() { return {left: 0, top: 0, width: 200, height: 200}; }
  }
  const elements = new Map([...html.matchAll(/id="([^"]+)"/g)].map(m => ['#' + m[1], new Element()]));
  const rotations = [...html.matchAll(/class="rotate" data-w="([^"]+)"/g)].map(m => { const e = new Element(); e.dataset.w = m[1]; return e; });
  const motors = [1, 2, 3, 4].map(id => {
    const e = new Element(); e.dataset.id = id;
    e.parts = {input: new Element(), '.value': new Element(), button: new Element()};
    e.querySelector = selector => e.parts[selector]; return e;
  });
  const document = new Element(), window = new Element();
  document.hidden = false;
  document.querySelector = selector => { assert(elements.has(selector), selector); return elements.get(selector); };
  document.querySelectorAll = selector => ({'.rotate': rotations, '.motor': motors, '.joystick, .rotate': [elements.get('#joystick'), ...rotations]}[selector]);
  elements.get('#drive-speed').value = '40';
  elements.get('#encoder-graph').getContext = () => new Proxy({}, {get: () => () => {}, set: () => true});
  let now = 0, timerId = 0, outstanding = 0;
  const timers = new Map(), requests = [], releases = [];
  const transport = {hold: false, fail: false, maxOutstanding: 0};
  const schedule = (fn, delay, repeat = false) => { const id = ++timerId; timers.set(id, {fn, due: now + delay, delay, repeat}); return id; };
  const context = {document, devicePixelRatio: 1, navigator: {sendBeacon: url => requests.push(url)}, AbortController,
    addEventListener: window.addEventListener.bind(window),
    setTimeout: (fn, delay) => schedule(fn, delay), clearTimeout: id => timers.delete(id),
    setInterval: (fn, delay) => schedule(fn, delay, true),
    fetch: async (url, options) => {
      assert(url.startsWith('/api/'), 'Only relative mock API calls are allowed');
      if (url === '/api/status') return {json: async () => ({encoders: [0,0,0,0], target_rad_s: [0,0,0,0], commanded_rad_s: [0,0,0,0], omega_rad_s: [0,0,0,0], applied: [0,0,0,0], reverse_waiting: [false,false,false,false]})};
      assert.equal(options.method, 'POST'); requests.push(url); outstanding++;
      transport.maxOutstanding = Math.max(transport.maxOutstanding, outstanding);
      try {
        if (transport.hold) await new Promise((resolve, reject) => {
          releases.push(resolve);
          options.signal.addEventListener('abort', () => reject(Error('timeout')), {once:true});
        });
        if (transport.fail) throw Error('offline');
        return {ok: true};
      }
      finally { outstanding--; }
    }};
  vm.runInNewContext(script, context);
  const tick = async milliseconds => {
    const end = now + milliseconds;
    while (true) {
      const next = [...timers].filter(([,t]) => t.due <= end).sort((a,b) => a[1].due-b[1].due)[0];
      if (!next) break;
      const [id,t] = next; now = t.due;
      if (t.repeat) t.due += t.delay; else timers.delete(id);
      t.fn(); await settle();
    }
    now = end; await settle();
  };
  return {window, document, joystick: elements.get('#joystick'), knob: elements.get('#joystick-knob'), rotations, motors, stop: elements.get('#stop-all'), speed: elements.get('#drive-speed'), status: elements.get('#status'), requests, releases, transport, tick};
}

test('joystick maps screen directions, diagonals, dead zone and proportional speed', async () => {
  for (const [dx, dy, x, y] of [[0,-100,40,0], [0,100,-40,0], [-100,0,0,40], [100,0,0,-40], [-100,-100,28,28], [100,100,-28,-28], [0,0,0,0], [0,-5,0,0], [0,-36.85,20,0]]) {
    const f = fixture(); f.joystick.emit('pointerdown', {clientX:100+dx, clientY:100+dy}); await settle();
    assert.equal(f.requests.at(-1), `/api/drive?x=${x}&y=${y}&w=0`);
  }
});

test('pointer capture clamps outside drags and release clears pending updates', async () => {
  const f = fixture(); f.joystick.emit('pointerdown', {clientX:100,clientY:0}); await settle();
  assert(f.joystick.hasPointerCapture(1));
  f.joystick.emit('pointermove', {clientX:1000,clientY:100}); await f.tick(50);
  assert.equal(f.requests.at(-1), '/api/drive?x=0&y=-40&w=0');
  f.window.emit('pointerup', {pointerId:2}); await settle();
  assert(f.joystick.hasPointerCapture(1));
  f.window.emit('pointerup'); await settle(); const count = f.requests.length;
  await f.tick(1000);
  assert.equal(f.requests.length, count);
  assert.equal(f.requests.at(-1), '/api/drive?x=0&y=0&w=0');
  assert.equal(f.knob.style.transform, 'translate(0px, 0px)');
});

test('rotation icons send opposite angular commands without translation', async () => {
  for (const [index,w] of [[0,40],[1,-40]]) {
    const f = fixture(); f.rotations[index].emit('pointerdown'); await settle();
    assert.equal(f.requests.at(-1), `/api/drive?x=0&y=0&w=${w}`);
    await f.tick(250); assert.equal(f.requests.at(-1), `/api/drive?x=0&y=0&w=${w}`);
    f.window.emit('pointerup'); await settle();
    assert.equal(f.requests.at(-1), '/api/drive?x=0&y=0&w=0');
  }
});

test('second pointer cannot take over a held joystick', async () => {
  const f = fixture(); f.joystick.emit('pointerdown', {clientX:100,clientY:0}); await settle();
  f.rotations[0].emit('pointerdown', {pointerId:2}); await f.tick(250);
  assert(f.requests.every(url => url === '/api/drive?x=40&y=0&w=0'));
});

test('slow HTTP coalesces movement and sends only stop after the in-flight command', async () => {
  const f = fixture(); f.transport.hold = true;
  f.joystick.emit('pointerdown', {clientX:100,clientY:0});
  f.joystick.emit('pointermove', {clientX:0,clientY:100}); await f.tick(50);
  f.joystick.emit('pointermove', {clientX:200,clientY:100}); await f.tick(50);
  f.window.emit('pointerup');
  assert.equal(f.requests.length, 1);
  f.transport.hold = false; f.releases.shift()(); await settle();
  assert.deepEqual(f.requests, ['/api/drive?x=40&y=0&w=0', '/api/drive?x=0&y=0&w=0']);
  assert.equal(f.transport.maxOutstanding, 1);
});

test('STOP ALL discards queued movement and uses immediate-stop endpoint', async () => {
  const f = fixture(); f.transport.hold = true;
  f.rotations[0].emit('pointerdown'); await f.tick(250);
  f.stop.emit('click'); f.transport.hold = false; f.releases.shift()(); await settle();
  assert.equal(f.requests.at(-1), '/api/stop');
  const count = f.requests.length; await f.tick(1000); assert.equal(f.requests.length, count);
});

test('cancel, capture loss, mouse button loss and blur stop the joystick', async () => {
  for (const event of ['pointercancel','lostpointercapture','buttons','blur']) {
    const f = fixture(); f.joystick.emit('pointerdown', {clientX:100,clientY:0}); await settle();
    if (event === 'lostpointercapture') f.joystick.releasePointerCapture(1);
    else if (event === 'buttons') f.joystick.emit('pointermove', {clientX:0,clientY:0,buttons:0});
    else f.window.emit(event);
    await settle(); assert.equal(f.requests.at(-1), '/api/drive?x=0&y=0&w=0');
  }
});

test('hidden page, Escape and pagehide clear held controls', async () => {
  for (const event of ['visibilitychange','Escape','pagehide']) {
    const f = fixture(); f.rotations[0].emit('pointerdown'); await settle();
    if (event === 'visibilitychange') { f.document.hidden = true; f.document.emit(event); }
    else if (event === 'Escape') f.window.emit('keydown', {key:'Escape'});
    else f.window.emit(event);
    await settle(); const count = f.requests.length; await f.tick(1000);
    assert.equal(f.requests.at(-1), '/api/stop'); assert.equal(f.requests.length,count);
  }
});

test('rotation keyboard hold releases and a failed request never resumes itself', async () => {
  const f = fixture(); f.rotations[1].emit('keydown', {key:' '}); await settle();
  assert.equal(f.requests.at(-1), '/api/drive?x=0&y=0&w=-40');
  f.window.emit('keyup', {key:' '}); await settle();
  assert.equal(f.requests.at(-1), '/api/drive?x=0&y=0&w=0');
  f.transport.fail = true; f.rotations[0].emit('pointerdown'); await settle();
  const count = f.requests.length; f.transport.fail = false; await f.tick(1000);
  assert.equal(f.requests.length,count); assert.match(f.status.textContent,/Connection lost/);
});

test('individual motor input cancels joystick before commanding that motor', async () => {
  const f = fixture(); f.joystick.emit('pointerdown',{clientX:100,clientY:0}); await settle();
  f.motors[0].parts.input.value='25'; f.motors[0].parts.input.emit('input'); await settle();
  assert.deepEqual(f.requests.slice(-2),['/api/drive?x=0&y=0&w=0','/api/motor?id=1&speed=25']);
});

test('a request timeout clears held input and queued heartbeats', async () => {
  const f = fixture(); f.transport.hold = true;
  f.joystick.emit('pointerdown', {clientX:100,clientY:0});
  await f.tick(501);
  assert(!f.joystick.hasPointerCapture(1));
  assert.match(f.status.textContent,/Connection lost/);
  const count = f.requests.length; f.transport.hold = false; await f.tick(1000);
  assert.equal(f.requests.length,count);
});
