(() => {
  const root = document.getElementById('cleany-display');
  const screen = root.querySelector('.cd-screen');
  const scene = root.querySelector('.cd-scene');
  const canvas = root.querySelector('canvas');
  const pen = canvas.getContext('2d', { alpha: false });
  const background = document.createElement('canvas');
  const backgroundPen = background.getContext('2d', { alpha: false });
  let backgroundKey = '';
  const caption = root.querySelector('.cd-caption');
  const help = root.querySelector('.cd-help');
  const dreams = root.querySelector('.cd-dreams');
  const message = root.querySelector('.cd-message');
  const symbol = root.querySelector('.cd-symbol');
  const result = root.querySelector('.cd-result');
  const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
  const rendering = { max_canvas_width: 1280, max_pixel_ratio: 2, max_fps: 60 };
  const config = { eyeColor: '#8ff4cd', eyeSize: 112, glow: 14 };
  const modes = {
    idle: { name: '대기', emotion: 'idle', message: '' },
    moving: { name: '이동', emotion: 'idle', message: '지나갈게요' },
    checking: { name: '물건 확인', emotion: 'thinking', message: '물건을 확인하고 있어요' },
    collecting: { name: '물건 수거', emotion: 'focus', message: '물건을 수거하고 있어요' },
    complete: { name: '작업 완료', emotion: 'excited', message: '정리 끝!' },
    charging: { name: '충전', emotion: 'sleepy', message: '충전 중' },
    help: { name: '도움 요청', emotion: 'sad', message: '앞에 있는 물건을 옮겨 주세요' }
  };

  const base = { w: 103, h: 126, hR: 126, curve: 0, gap: 114, y: 0, tilt: 0, tiltR: 0, mouthW: 37, smile: 16, open: 0, mouthX: 0, mouthY: 88, cheeks: 0, gazeX: 0, gazeY: 0, lookX: 0, lookY: 0, bounce: 2, cx: 400, cy: 207, scale: 1, roll: 0, squash: 0, eyeLY: 0, eyeRY: 0, r: 143, g: 244, b: 205 };
  const motionPeriods = { moving: 1.35, checking: 3.2, collecting: 3.2, charging: 4.6, help: 3.6 };
  const idleRoutine = { interval: 10, actions: [{ name: 'smile', duration: 2.6 }, { name: 'peek', duration: 3.2 }, { name: 'nod', duration: 2.4 }] };
  const eyeTracks = {
    checking: [[0, 0, -4, 112], [.16, -46, -10, 104], [.28, -46, -10, 104], [.43, 46, 10, 98], [.53, 46, 10, 98], [.65, 0, -10, 138], [.76, 0, -10, 138], [.88, 0, -4, 112], [1, 0, -4, 112]],
    collecting: [[0, 0, 8, 80], [.18, 0, 32, 74], [.3, 10, 38, 44], [.43, 10, 38, 44], [.62, 0, -12, 86], [.72, 0, -12, 86], [.94, 0, 8, 80], [1, 0, 8, 80]]
  };
  const poses = {
    idle: { ...base },
    excited: { ...base, w: 116, h: 34, hR: 34, curve: 64, gap: 117, y: 10, mouthW: 59, smile: 26, open: 28, cheeks: .9, bounce: 8 },
    focus: { ...base, w: 103, h: 80, hR: 80, mouthW: 23, mouthY: 96, smile: 0, lookY: 16, bounce: 0 },
    thinking: { ...base, w: 98, h: 112, hR: 112, mouthW: 23, smile: 0, lookY: -4, bounce: 0 },
    sleepy: { ...base, w: 105, h: 13, hR: 13, curve: -8, y: 15, mouthW: 18, mouthY: 60, smile: 0, open: 6, bounce: 0 },
    sad: { ...base, w: 96, h: 88, hR: 88, tilt: -.15, tiltR: .15, y: 9, mouthW: 33, smile: -16, gazeY: 10, bounce: 1 }
  };
  const keys = Object.keys(base);
  let mode = 'idle';
  let direction = 'forward';
  let clock = 0;
  let modeStartedAt = 0;
  let idleSince = 0;
  let lastTime = 0;
  let nextFrameAt = 0;
  let nextBlink = 2.4;
  let blinkAt = -100;
  let nextGaze = 2.5;
  let celebrationAt = -100;
  let gaze = { x: 0, y: 0 };
  let gazeTarget = { x: 0, y: 0 };
  let target = { ...base };
  let current = { ...base };
  let velocity = Object.fromEntries(keys.map(key => [key, 0]));
  let visible = true;
  let dirty = true;
  let frame = 0;
  let helpTextTop = 450;
  let dreamAnimations = [];

  const clamp = (n, min, max) => Math.max(min, Math.min(max, n));
  const canAnimate = () => !reducedMotion.matches;
  const rgb = hex => [1, 3, 5].map(offset => parseInt(hex.slice(offset, offset + 2), 16));

  function syncDreams() {
    dreamAnimations = mode === 'charging' && canAnimate() ? dreams.getAnimations({subtree: true}) : [];
    for (const animation of dreamAnimations) {
      animation.currentTime = (clock - modeStartedAt) * 1000;
      if (rendering.max_fps >= 60 && visible && !document.hidden) animation.play();
      else animation.pause();
    }
  }

  function easedRamp(phase, start, end) {
    const t = clamp((phase - start) / (end - start), 0, 1);
    return t * t * (3 - 2 * t);
  }

  function followEyes(p, track, phase) {
    const end = track.findIndex(point => point[0] >= phase);
    const a = track[Math.max(0, end - 1)];
    const b = track[Math.max(0, end)];
    const progress = a === b ? 0 : (phase - a[0]) / (b[0] - a[0]);
    const eased = progress * progress * (3 - 2 * progress);
    p.lookX = a[1] + (b[1] - a[1]) * eased;
    p.lookY = a[2] + (b[2] - a[2]) * eased;
    p.h = p.hR = a[3] + (b[3] - a[3]) * eased;
  }

  function blinkAmount(age) {
    if (age < 0 || age >= .21) return 1;
    if (age < .06) return 1 - .96 * Math.sin(age / .06 * Math.PI / 2);
    if (age < .085) return .04;
    const progress = (age - .085) / .125;
    return .04 + .96 * progress * progress * (3 - 2 * progress);
  }

  function idleMoment() {
    if (mode !== 'idle') return null;
    const elapsed = clock - idleSince;
    const cycle = Math.floor(elapsed / idleRoutine.interval);
    if (cycle < 1) return null;
    const action = idleRoutine.actions[(cycle - 1) % idleRoutine.actions.length];
    const age = elapsed - cycle * idleRoutine.interval;
    return age < action.duration ? { name: action.name, phase: age / action.duration } : null;
  }

  function idlePose(p, moment) {
    const { name, phase } = moment;
    const amount = easedRamp(phase, 0, .22) * (1 - easedRamp(phase, .72, 1));
    p.bounce = 0;
    if (name === 'smile') {
      p.w += 10 * amount;
      p.h = p.hR = p.h - 98 * amount;
      p.curve = 50 * amount;
      p.mouthW += 12 * amount;
      p.smile += 12 * amount;
      p.cheeks = .5 * amount;
      p.y -= 6 * amount;
      p.roll = .05 * Math.sin(phase * Math.PI * 2) * amount;
    } else if (name === 'peek') {
      p.lookX = -36 * Math.sin(phase * Math.PI * 2) * amount;
      p.lookY = -15 * amount;
      p.roll = .07 * p.lookX / 36;
      p.h = p.hR = p.h + 8 * amount;
      p.w += 4 * amount;
      p.y -= 3 * amount;
      p.smile += 4 * amount;
    } else {
      const nod = easedRamp(phase, .12, .36) * (1 - easedRamp(phase, .55, .9));
      p.y += 16 * nod;
      p.lookY = 8 * nod;
      p.h = p.hR = p.h - 24 * nod;
      p.squash = .06 * nod;
      p.smile += 5 * nod;
      p.roll = -.04 * Math.sin(phase * Math.PI) * amount;
    }
    return p;
  }

  function actionPose() {
    const p = { ...target };
    const idle = idleMoment();
    if (idle) return idlePose(p, idle);
    if (!Object.hasOwn(motionPeriods, mode)) return p;
    const phase = ((clock - modeStartedAt) / motionPeriods[mode]) % 1;
    const angle = phase * Math.PI * 2;
    p.bounce = 0;
    if (mode === 'moving') {
      const stride = Math.sin(angle);
      p.roll = stride * .025 + (direction === 'left' ? -.012 : direction === 'right' ? .012 : 0);
      p.y -= 5 * (1 - Math.cos(angle * 2)) / 2;
      p.eyeLY = -3.5 * stride;
      p.eyeRY = 3.5 * stride;
      p.squash = .018 * Math.cos(angle * 2);
      p.smile = 15 + 3 * Math.cos(angle);
    } else if (mode === 'checking') {
      followEyes(p, eyeTracks.checking, phase);
      const notice = easedRamp(phase, .53, .65) * (1 - easedRamp(phase, .76, .88));
      const nod = easedRamp(phase, .7, .75) * (1 - easedRamp(phase, .79, .86));
      p.w += 22 * notice;
      p.gazeX = p.lookX * .48;
      p.roll = .105 * p.lookX / 46;
      p.y += 6 * Math.sin(angle * 2) * (1 - notice) - 16 * notice + 12 * nod;
      p.scale *= 1 + .055 * notice;
      p.squash = .018 * Math.sin(angle * 2) * (1 - notice) - .045 * notice;
      p.open = 13 * notice;
      p.mouthW -= 6 * notice;
      p.mouthX = p.lookX * .2;
    } else if (mode === 'collecting') {
      followEyes(p, eyeTracks.collecting, phase);
      const reach = easedRamp(phase, 0, .2) * (1 - easedRamp(phase, .43, .62));
      const brace = easedRamp(phase, .18, .3) * (1 - easedRamp(phase, .43, .62));
      const lift = easedRamp(phase, .43, .62) * (1 - easedRamp(phase, .72, .94));
      p.y += 16 * reach - 15 * lift;
      p.squash = .09 * brace - .03 * lift;
      p.tilt = .08 * brace;
      p.tiltR = -p.tilt;
      p.w += 5 * brace;
      p.mouthW -= 10 * brace;
      p.smile = -3 * brace;
    } else if (mode === 'charging') {
      const breath = Math.sin(angle);
      p.roll = -.025;
      p.y += breath * 2;
      p.squash = breath * .014;
      p.h = p.hR = 11 + breath;
      p.open = 7 + breath * 2;
      p.mouthW = 17 + breath * 2;
    } else if (mode === 'help') {
      const ask = (1 - Math.cos(angle)) / 2;
      const sway = Math.sin(angle);
      p.roll = .14 * sway;
      p.gazeX = 14 * sway;
      p.lookX = 10 * sway;
      p.lookY -= 6 * ask;
      p.y -= 8 * ask;
    }
    return p;
  }

  function retarget() {
    target = { ...poses[modes[mode].emotion] };
    [target.r, target.g, target.b] = rgb(config.eyeColor);
    if (mode !== 'idle') { target.cy = 184; target.scale = .94; }
    if (mode === 'moving') {
      target.gazeX = direction === 'left' ? -31 : direction === 'right' ? 31 : 0;
      target.gazeY = direction === 'forward' ? -10 : 0;
      target.smile = 11;
    }
    if (mode === 'help') {
      target.cx = 400; target.cy = 180; target.scale = .58;
      target.bounce = 0;
      target.r = 255; target.g = 210; target.b = 154;
    }
    if (!canAnimate()) {
      current = { ...target };
      velocity = Object.fromEntries(keys.map(key => [key, 0]));
    }
    dirty = true;
    schedule();
  }

  let liveData = {};
  let uiSignature;
  function updateUI() {
    const signature = JSON.stringify([mode, direction, liveData.connected === true,
      liveData.message, liveData.collected_count, liveData.battery_percent]);
    if (signature === uiSignature) return;
    uiSignature = signature;
    screen.dataset.mode = mode;
    root.dataset.mode = mode;
    root.dataset.connected = String(liveData.connected === true);
    canvas.setAttribute('aria-label', `Cleany ${modes[mode].name}`);
    message.textContent = liveData.message || modes[mode].message;
    result.hidden = mode !== 'complete' || !Number.isInteger(liveData.collected_count);
    result.textContent = Number.isInteger(liveData.collected_count) ? `수거 ${liveData.collected_count}개` : '';
    symbol.textContent = mode === 'complete' ? '✓' : '';
    symbol.hidden = mode !== 'complete';
    if (mode === 'moving' && !liveData.message) message.textContent = ({left: '왼쪽으로 지나갈게요', forward: '지나갈게요', right: '오른쪽으로 지나갈게요'})[direction];
    if (mode === 'charging') message.textContent = Number.isFinite(liveData.battery_percent) ? `충전 중 · ${liveData.battery_percent}%` : '충전 중';
    help.querySelector('.cd-help-request').textContent = liveData.message || '상태를 확인해 주세요';
    help.hidden = mode !== 'help';
    measureHelpLayout();
    if (mode !== 'help') delete help.dataset.ready;
    dreams.hidden = mode !== 'charging';
    syncDreams();
    caption.hidden = mode === 'help' || mode === 'idle';
  }

  function setMode(next) {
    if (!Object.hasOwn(modes, next)) return;
    mode = next;
    modeStartedAt = clock;
    idleSince = clock;
    blinkAt = -100;
    nextBlink = clock + 2.6 + Math.random() * 2.4;
    gazeTarget = { x: 0, y: 0 };
    celebrationAt = mode === 'complete' ? clock : -100;
    retarget();
    updateUI();
  }

  function eye(x, y, w, h, bend, tilt, blink, color) {
    pen.save();
    pen.translate(x, y);
    pen.rotate(tilt);
    h = Math.max(4, h * blink);
    bend *= blink;
    const r = Math.min(w * .34, h * .48);
    const span = w / 2 - r;
    pen.beginPath();
    pen.moveTo(-span, -h / 2);
    pen.bezierCurveTo(-span / 3, -h / 2 - bend, span / 3, -h / 2 - bend, span, -h / 2);
    pen.quadraticCurveTo(w / 2, -h / 2, w / 2, -h / 2 + r);
    pen.lineTo(w / 2, h / 2 - r);
    pen.quadraticCurveTo(w / 2, h / 2, span, h / 2);
    pen.bezierCurveTo(span / 3, h / 2 - bend, -span / 3, h / 2 - bend, -span, h / 2);
    pen.quadraticCurveTo(-w / 2, h / 2, -w / 2, h / 2 - r);
    pen.lineTo(-w / 2, -h / 2 + r);
    pen.quadraticCurveTo(-w / 2, -h / 2, -span, -h / 2);
    pen.closePath();
    const fill = pen.createLinearGradient(0, -h / 2 - Math.max(0, bend), 0, h / 2);
    fill.addColorStop(0, `rgb(${color.join(',')})`);
    fill.addColorStop(1, `rgb(${color.map(c => Math.round(c * .82)).join(',')})`);
    pen.fillStyle = fill;
    pen.shadowColor = `rgb(${color.join(',')})`;
    pen.shadowBlur = config.glow * canvas.width / 800;
    pen.fill();
    pen.restore();
  }

  function paintSweat(p, moving) {
    const phase = ((clock - modeStartedAt) / motionPeriods.collecting) % 1;
    const drops = [[.21, .41, 1, 11], [.4, .32, -1, 8]];
    drops.forEach(([start, duration, side, size], index) => {
      if (!moving && index > 0) return;
      const t = moving ? (phase - start) / duration : .3;
      if (t < 0 || t > 1) return;
      const opacity = easedRamp(t, 0, .12) * (1 - easedRamp(t, .78, 1));
      const x = side * (p.gap + p.w / 2 + 29 + t * 7) + p.lookX;
      const y = -p.h / 2 - 25 + p.lookY + 82 * Math.pow(t, 1.4);
      pen.save();
      pen.translate(x, y); pen.rotate(side * .14);
      const growth = .6 + .4 * easedRamp(t, 0, .18);
      pen.scale(growth, growth);
      pen.globalAlpha = opacity * .9;
      pen.fillStyle = '#9cdef3'; pen.shadowColor = '#9cdef3'; pen.shadowBlur = config.glow * .45;
      pen.beginPath(); pen.moveTo(0, -size * 1.8);
      pen.bezierCurveTo(size * .35, -size * .8, size, -size * .2, size, size * .45);
      pen.bezierCurveTo(size, size * 1.65, -size, size * 1.65, -size, size * .45);
      pen.bezierCurveTo(-size, -size * .2, -size * .35, -size * .8, 0, -size * 1.8);
      pen.fill();
      pen.shadowBlur = 0; pen.fillStyle = 'rgba(239,253,255,.65)';
      pen.beginPath(); pen.ellipse(-size * .27, size * .3, size * .16, size * .38, .2, 0, Math.PI * 2); pen.fill();
      pen.restore();
    });
  }

  function paintTravel(tint, moving) {
    pen.save(); pen.fillStyle = tint; pen.shadowColor = tint; pen.shadowBlur = config.glow * .5;
    for (let i = 0; i < 4; i++) {
      const t = moving ? ((clock - modeStartedAt) * .7 + i / 4) % 1 : (i + .5) / 4;
      const glow = Math.sin(t * Math.PI);
      const side = direction === 'left' ? -1 : 1;
      const x = direction === 'forward' ? 400 + (i - 1.5) * 16 * (1 - t * .4) : 400 + side * (t - .5) * 100;
      const y = direction === 'forward' ? 418 - 16 * t : 411 - 4 * glow;
      pen.globalAlpha = .18 + .65 * glow;
      pen.beginPath(); pen.arc(x, y, 2.4 + 1.6 * glow, 0, Math.PI * 2); pen.fill();
    }
    pen.restore();
  }

  function paintBackground(p, color) {
    const ratio = canvas.width / 800;
    // The faint ambient light only needs pixel precision; reuse its opaque bitmap.
    const cx = Math.round(p.cx * ratio) / ratio;
    const cy = Math.round(p.cy * ratio) / ratio;
    const key = [canvas.width, canvas.height, cx, cy, ...color].join(',');
    if (key !== backgroundKey) {
      if (background.width !== canvas.width || background.height !== canvas.height) {
        background.width = canvas.width;
        background.height = canvas.height;
      }
      backgroundPen.setTransform(ratio, 0, 0, canvas.height / 450, 0, 0);
      backgroundPen.fillStyle = '#030b08';
      backgroundPen.fillRect(0, 0, 800, 450);
      const ambient = backgroundPen.createRadialGradient(cx, cy, 10, cx, cy, 300);
      ambient.addColorStop(0, `rgba(${color.join(',')},.025)`);
      ambient.addColorStop(1, `rgba(${color.join(',')},0)`);
      backgroundPen.fillStyle = ambient;
      backgroundPen.fillRect(0, 0, 800, 450);
      backgroundKey = key;
    }
    pen.drawImage(background, 0, 0, 800, 450);
  }

  function paint() {
    if (rendering.max_fps < 60) {
      for (const animation of dreamAnimations) animation.currentTime = (clock - modeStartedAt) * 1000;
    }
    const p = current;
    const color = [p.r, p.g, p.b].map(n => Math.round(clamp(n, 0, 255)));
    const tint = `rgb(${color.join(',')})`;
    pen.setTransform(canvas.width / 800, 0, 0, canvas.height / 450, 0, 0);
    paintBackground(p, color);
    const moving = canAnimate();
    const wave = moving ? Math.sin(clock * (mode === 'complete' ? 5 : 1.7)) : 0;
    const checkingAge = ((clock - modeStartedAt) % motionPeriods.checking) - motionPeriods.checking * .92;
    const blink = !moving || mode === 'charging' || mode === 'collecting' || idleMoment() ? 1 : blinkAmount(mode === 'checking' ? checkingAge : clock - blinkAt);
    pen.save();
    pen.translate(p.cx + gaze.x + p.gazeX, p.cy + p.y + gaze.y + p.gazeY + wave * p.bounce);
    pen.rotate(p.roll);
    const scale = p.scale * config.eyeSize / 100;
    pen.scale(scale * (1 + p.squash), scale * (1 - p.squash * .6));
    eye(-p.gap + p.lookX, p.eyeLY + p.lookY, p.w, p.h, p.curve, p.tilt, blink, color);
    eye(p.gap + p.lookX, p.eyeRY + p.lookY, p.w, p.hR, p.curve, p.tiltR, blink, color);
    pen.strokeStyle = tint;
    pen.fillStyle = tint;
    pen.lineWidth = 7;
    pen.lineCap = 'round';
    pen.lineJoin = 'round';
    pen.shadowColor = tint;
    pen.shadowBlur = config.glow * .4 * canvas.width / 800;
    const mouthY = p.mouthY;
    pen.beginPath();
    if (p.open > 1) {
      const half = Math.max(6, p.mouthW * .52);
      pen.moveTo(p.mouthX - half, mouthY);
      pen.bezierCurveTo(p.mouthX - half * .5, mouthY + p.smile * .2 - 4, p.mouthX + half * .5, mouthY + p.smile * .2 - 4, p.mouthX + half, mouthY);
      pen.bezierCurveTo(p.mouthX + half, mouthY + p.open * 1.3, p.mouthX - half, mouthY + p.open * 1.3, p.mouthX - half, mouthY);
      pen.fill();
    } else {
      pen.moveTo(p.mouthX - p.mouthW * .5, mouthY);
      pen.quadraticCurveTo(p.mouthX, mouthY + p.smile, p.mouthX + p.mouthW * .5, mouthY);
      pen.stroke();
    }
    if (p.cheeks > .01) {
      pen.save(); pen.globalAlpha = p.cheeks * .8; pen.lineWidth = 5;
      for (const side of [-1, 1]) for (let i = 0; i < 3; i++) {
        const x = side * (p.gap + 47) + (i - 1) * 11;
        pen.beginPath(); pen.moveTo(x - 2, 53); pen.lineTo(x + 2, 64); pen.stroke();
      }
      pen.restore();
    }
    if (mode === 'collecting') paintSweat(p, moving);
    pen.restore();
    if (mode === 'moving') paintTravel(tint, moving);
    if (mode === 'help') {
      // Reveal the request only after the transitioning face clears the text area.
      const faceBottom = p.cy + p.y + gaze.y + p.gazeY + scale * (p.mouthY + Math.max(p.open * 1.3, p.smile, 0) + 9);
      const ready = faceBottom < helpTextTop - 18;
      if (help.dataset.ready !== String(ready)) help.dataset.ready = String(ready);
    }
    const partyAge = clock - celebrationAt;
    if (moving && partyAge >= 0 && partyAge < 1.5) {
      for (let i = 0; i < 8; i++) {
        const t = clamp((partyAge - i * .04) / 1.1, 0, 1);
        const a = i * Math.PI / 4;
        const radius = 183 + t * 40;
        const size = (5 + i % 3) * Math.sin(t * Math.PI);
        pen.save(); pen.globalAlpha = Math.sin(t * Math.PI) * .8;
        pen.strokeStyle = tint; pen.lineWidth = 2.5; pen.lineCap = 'round';
        pen.translate(400 + Math.cos(a) * radius, 180 + Math.sin(a) * radius * .55 - t * 15); pen.rotate(a + t);
        pen.beginPath(); pen.moveTo(-size, 0); pen.lineTo(size, 0); pen.moveTo(0, -size); pen.lineTo(0, size); pen.stroke(); pen.restore();
      }
    }
    dirty = false;
  }

  function measureHelpLayout() {
    if (help.hidden) return;
    const box = scene.getBoundingClientRect();
    if (box.height) helpTextTop = (help.getBoundingClientRect().top - box.top) * 450 / box.height;
  }

  function resize() {
    const box = scene.getBoundingClientRect();
    if (!box.width || !box.height) return;
    // Bound raster work independently of the monitor's physical resolution.
    const ratio = Math.min(devicePixelRatio || 1, rendering.max_pixel_ratio,
                           rendering.max_canvas_width / box.width);
    const width = Math.max(1, Math.round(box.width * ratio));
    const height = Math.max(1, Math.round(box.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    measureHelpLayout();
    dirty = true;
    schedule();
  }

  function schedule() {
    if (!frame && visible && !document.hidden) frame = requestAnimationFrame(tick);
  }

  function tick(now) {
    frame = 0;
    if (!visible || document.hidden || !root.isConnected) { lastTime = 0; return; }
    const interval = 1000 / rendering.max_fps;
    // Keep the paint cadence independent of the monitor's refresh rate.
    if (lastTime && !dirty && now + .5 < nextFrameAt) { schedule(); return; }
    // Re-anchor to the presented frame so a slightly fast panel cannot accumulate drift.
    nextFrameAt = now + interval;
    const dt = Math.min(.1, Math.max(0, (now - (lastTime || now)) / 1000));
    lastTime = now;
    if (canAnimate()) {
      clock += dt;

      if (clock >= nextBlink) { blinkAt = clock; nextBlink = clock + 2.6 + Math.random() * 2.4; }
      if (idleMoment()) {
        gazeTarget = { x: 0, y: 0 };
        nextGaze = clock + 2;
      } else if (mode === 'idle' && clock >= nextGaze) {
        gazeTarget = { x: (Math.random() - .5) * 17, y: (Math.random() - .5) * 8 };
        nextGaze = clock + 2 + Math.random() * 3;
      }
      const desired = actionPose();
      const steps = Math.max(1, Math.ceil(dt * 180));
      for (let sub = 0; sub < steps; sub++) {
        const h = dt / steps;
        for (const key of keys) {
          velocity[key] += (185 * (desired[key] - current[key]) - 24 * velocity[key]) * h;
          current[key] += velocity[key] * h;
        }
      }
      gaze.x += (gazeTarget.x - gaze.x) * Math.min(1, dt * 8);
      gaze.y += (gazeTarget.y - gaze.y) * Math.min(1, dt * 8);
      paint();
      schedule();
    } else if (dirty) paint();
  }

  reducedMotion.addEventListener('change', () => {
    lastTime = 0;
    blinkAt = -100;
    idleSince = clock;
    retarget();
    updateUI();
    syncDreams();
  });
  new ResizeObserver(resize).observe(scene);
  new IntersectionObserver(entries => {
    visible = entries[0].isIntersecting;
    syncDreams();
    if (visible) { lastTime = 0; dirty = true; schedule(); }
  }).observe(screen);
  retarget();
  current = { ...target };
  updateUI();
  resize();

  document.addEventListener('visibilitychange', () => { lastTime = 0; syncDreams(); schedule(); });
  let lastResponse = performance.now();
  async function poll() {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2000);
    let delay = 250;
    try {
      const response = await fetch('/api/state', {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw new Error('Display unavailable');
      const data = await response.json();
      if (!Object.hasOwn(modes, data.mode)) throw new Error('Invalid display mode');
      lastResponse = performance.now();
      liveData = data;
      let resized = false;
      for (const [key, min, max] of [['max_canvas_width', 320, 3840], ['max_pixel_ratio', 1, 2], ['max_fps', 15, 60]]) {
        const value = data.rendering?.[key];
        if (Number.isFinite(value)) {
          const next = clamp(value, min, max);
          if (rendering[key] !== next) { rendering[key] = next; resized = true; }
        }
      }
      if (resized) { resize(); syncDreams(); }
      const nextDirection = ['left', 'forward', 'right'].includes(data.direction) ? data.direction : 'forward';
      const directionChanged = direction !== nextDirection;
      direction = nextDirection;
      if (mode !== data.mode) setMode(data.mode);
      else { if (directionChanged) retarget(); updateUI(); }
      if (Number.isFinite(data.poll_ms)) delay = Math.max(100, Math.min(2000, data.poll_ms));
    } catch {
      if (performance.now() - lastResponse > 3000) {
        liveData = {mode: 'help', connected: false, message: '연결을 확인해 주세요'};
        if (mode !== 'help') setMode('help'); else updateUI();
      }
    } finally {
      clearTimeout(timeout);
      setTimeout(poll, delay);
    }
  }
  poll();
})();
