// 저장소 vendor Mermaid를 격리된 headless Chromium에서 렌더링합니다.
// 사용법: node render_depth_jira_diagram.mjs OUTPUT PUPPETEER_MODULE CHROME
import {pathToFileURL} from 'node:url';
import {resolve} from 'node:path';
import {writeFile} from 'node:fs/promises';

const [out, modulePath, chrome] = process.argv.slice(2);
if (!out || !modulePath || !chrome) throw new Error('OUTPUT, PUPPETEER_MODULE, CHROME 인수가 필요합니다');
const {default: puppeteer} = await import(pathToFileURL(resolve(modulePath)));
const browser = await puppeteer.launch({executablePath: chrome, headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage']});
try {
  const page = await browser.newPage();
  await page.setViewport({width: 1420, height: 1000, deviceScaleFactor: 1});
  const errors = [];
  page.on('pageerror', e => errors.push(String(e)));
  await page.goto(pathToFileURL(resolve(out, 'diagram.html')).href);
  await page.waitForFunction(() => document.body.dataset.ready === 'true');
  await page.evaluate(() => document.fonts.ready);
  await page.screenshot({path: resolve(out, '12-depth-pipeline.png'), fullPage: true});
  await writeFile(resolve(out, '12-depth-pipeline.svg'), await page.$eval('#diagram svg', el => el.outerHTML));
  await page.goto(pathToFileURL(resolve(out, 'index.html')).href);
  await page.waitForFunction(() => [...document.images].every(x => x.complete && x.naturalWidth > 0));
  const desktop = await page.evaluate(() => ({images: document.images.length,
    overflow: document.documentElement.scrollWidth > innerWidth,
    imagesLoaded: [...document.images].every(x => x.naturalWidth > 0)}));
  await page.screenshot({path: resolve(out, 'gallery-desktop.png')});
  await page.setViewport({width: 390, height: 844, deviceScaleFactor: 1});
  const mobile = await page.evaluate(() => ({overflow: document.documentElement.scrollWidth > innerWidth}));
  await page.screenshot({path: resolve(out, 'gallery-mobile.png')});
  if (errors.length || desktop.images !== 12 || !desktop.imagesLoaded || desktop.overflow || mobile.overflow)
    throw new Error(JSON.stringify({errors, desktop, mobile}));
  await writeFile(resolve(out, 'browser-check.json'), JSON.stringify({errors, desktop, mobile}, null, 2));
  console.log(JSON.stringify({errors, desktop, mobile}));
} finally {
  await browser.close();
}
