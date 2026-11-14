#!/usr/bin/env node
/** Capture desktop/mobile previews and check image loading and horizontal overflow. */
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';

const [runDirectory, playwrightModule] = process.argv.slice(2);
if (!runDirectory || !playwrightModule) {
  throw new Error('Usage: review_readme_media.mjs RUN_DIRECTORY PLAYWRIGHT_MODULE');
}
const {chromium} = await import(pathToFileURL(path.resolve(playwrightModule)));
const run = path.resolve(runDirectory);
const browser = await chromium.launch({executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome',
                                      headless:true, args:['--no-sandbox']});
const results = [];
try {
  for (const width of [1440, 390]) {
    const page = await browser.newPage({viewport:{width, height:1000}});
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    for (const [name, relative] of [['gallery', 'gallery/index.html'],
                                  ['readme-en', 'review/README.md.html'],
                                  ['readme-ko', 'review/README.ko.md.html']]) {
      await page.goto(pathToFileURL(path.join(run, relative)).href);
      await page.locator('img').evaluateAll(images => Promise.all(images.map(image => image.decode().catch(()=>{}))));
      if (name !== 'gallery') {
        await page.waitForFunction(() => window.previewReady === true);
        if (await page.locator('details, img[src*="phone-preview"]').count()) {
          throw new Error('README must not contain collapsed sections or Phone Controller');
        }
        if (await page.locator('.mermaid svg').count() !== 1) throw new Error('Missing architecture diagram');
        const rows = await page.locator('table tr').evaluateAll(rows => rows
          .filter(row => row.querySelector('img'))
          .map(row => [...row.querySelectorAll('img')].map(img => img.src.split('/').pop())));
        const expected = [['table-work.gif','map-growth.gif'],
                          ['gazebo-study-cafe.png','isaac-parallel.png'],
                          ['mujoco-work.gif','rviz-motion.gif'],
                          ['dashboard.png','freecad-workspace.png']];
        if (JSON.stringify(rows) !== JSON.stringify(expected)) throw new Error('Unexpected README image grouping');
      }
      const state = await page.evaluate(() => ({
        overflow:document.documentElement.scrollWidth > innerWidth,
        missing:[...document.images].filter(i=>i.src.startsWith('file:') && (!i.complete || !i.naturalWidth)).map(i=>i.src),
      }));
      if (state.overflow || state.missing.length || errors.length) throw new Error(JSON.stringify({name,width,state,errors}));
      if (name === 'gallery') {
        await page.getByRole('button', {name:'시뮬레이션',exact:true}).click();
        if (await page.locator('article:visible').count() !== 3) throw new Error('Simulation filter failed');
        await page.getByRole('button', {name:'전체',exact:true}).click();
        if (await page.locator('article:visible').count() !== 6) throw new Error('Missing media category');
      }
      await page.screenshot({path:path.join(run,'review',`${name}-${width}.png`), fullPage:true});
      results.push({name,width,...state,errors:[...errors]});
    }
    await page.close();
  }
  const sheet = await browser.newPage({viewport:{width:1680,height:1100}});
  await sheet.goto(pathToFileURL(path.join(run,'gallery/index.html')).href+'#sheet');
  await sheet.locator('img').evaluateAll(images=>Promise.all(images.map(i=>i.decode())));
  await sheet.locator('main').screenshot({path:path.join(run,'overview.png')});
  await fs.writeFile(path.join(run,'review/browser-check.json'),JSON.stringify({passed:true,results},null,2)+'\n');
  console.log('Desktop/mobile image, overflow and gallery-filter checks passed');
} finally {
  await browser.close();
}
