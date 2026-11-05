#!/usr/bin/env python3
"""Measure the visible kiosk; optional modes override only its HTTP response."""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright

MODES = ('idle', 'moving', 'checking', 'collecting', 'complete', 'charging', 'help')
URL = 'http://127.0.0.1:8770/'


def measure(page, browser, seconds: float) -> dict:
    session = browser.new_browser_cdp_session()
    try:
        system = session.send('SystemInfo.getInfo')
        before = session.send('SystemInfo.getProcessInfo')['processInfo']
        geometry = page.evaluate('''() => {const c=document.querySelector('canvas'); return {
          viewport:[innerWidth,innerHeight], screen:[screen.width,screen.height], dpr:devicePixelRatio,
          canvas:[c.width,c.height], context:c.getContext('2d').getContextAttributes(),
          mode:document.querySelector('#cleany-display').dataset.mode, hidden:document.hidden};}''')
        frames = page.evaluate('''duration => new Promise(resolve => {
          const context = document.querySelector('canvas').getContext('2d');
          const original = context.setTransform;
          const painted = [], raf = [];
          const start = performance.now();
          context.setTransform = function(...args) {
            painted.push(performance.now());
            return original.apply(this, args);
          };
          let request;
          function sample(now) { raf.push(now); request = requestAnimationFrame(sample); }
          request = requestAnimationFrame(sample);
          setTimeout(() => {
            cancelAnimationFrame(request);
            context.setTransform = original;
            const elapsed = performance.now() - start;
            function stats(times) {
              const intervals = times.slice(1).map((time, i) => time - times[i]);
              const sorted = [...intervals].sort((a, b) => a - b);
              return {frames: times.length, fps: times.length * 1000 / elapsed,
                median_ms: sorted[Math.floor(sorted.length * .5)] ?? null,
                p95_ms: sorted[Math.floor(sorted.length * .95)] ?? null,
                max_ms: sorted.at(-1) ?? null,
                over_25ms: intervals.filter(value => value > 25).length,
                over_50ms: intervals.filter(value => value > 50).length};
            }
            resolve({elapsed_ms: elapsed, canvas: stats(painted), raf: stats(raf)});
          }, duration);
        })''', seconds * 1000)
        after = session.send('SystemInfo.getProcessInfo')['processInfo']
        previous = {process['id']: process for process in before}
        cpu = [{'type': p['type'], 'id': p['id'], 'cpu_percent_one_core': round(
            100000 * (p['cpuTime'] - previous[p['id']]['cpuTime']) / frames['elapsed_ms'], 2)}
               for p in after if p['id'] in previous]
        return {'geometry': geometry, 'frames': frames, 'cpu': cpu,
                'gpu_devices': system['gpu']['devices'], 'gpu_features': system['gpu']['featureStatus']}
    finally:
        session.detach()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--modes', nargs='+', choices=MODES,
                        help='Exercise screen-only states, restoring live ROS display afterward.')
    options = parser.parse_args()
    if not 0 < options.seconds <= 60:
        parser.error('--seconds must be between 0 and 60')
    options.output.parent.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as playwright:
        browser = playwright.chromium.connect_over_cdp('http://127.0.0.1:9223', timeout=10000)
        page = next(page for context in browser.contexts for page in context.pages if page.url == URL)
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        results = []
        mode = None

        def override(route):
            response = route.fetch()
            data = response.json()
            data.update(mode=mode, message='', direction='forward')
            route.fulfill(response=response, json=data)

        try:
            if options.modes:
                page.route(URL + 'api/state', override)
            for mode in options.modes or [None]:
                if mode:
                    page.wait_for_function("mode => document.querySelector('#cleany-display').dataset.mode === mode", arg=mode)
                result = measure(page, browser, options.seconds)
                results.append(result)
                if mode:
                    page.screenshot(path=str(options.output.with_name(options.output.stem + '-' + mode + '.png')))
                print(json.dumps({'mode': result['geometry']['mode'], 'frames': result['frames']['canvas'],
                                  'cpu': result['cpu']}), flush=True)
            report = {'results': results, 'page_errors': errors} if options.modes else results[0]
            options.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        finally:
            if options.modes:
                page.unroute_all(behavior='wait')
                live_mode = page.request.get(URL + 'api/state').json()['mode']
                page.wait_for_function("mode => document.querySelector('#cleany-display').dataset.mode === mode", arg=live_mode)


if __name__ == '__main__':
    main()
