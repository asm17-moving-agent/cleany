// 로컬 Mermaid 원본을 렌더링하고 자료 목록의 이미지와 화면 넘침을 검사합니다.
import {pathToFileURL} from 'node:url';
import {resolve} from 'node:path';
import {readFile,writeFile} from 'node:fs/promises';
const [out,modulePath,chrome]=process.argv.slice(2);
if (!out || !modulePath || !chrome) throw new Error('OUTPUT, PUPPETEER_MODULE, CHROME 인수가 필요합니다');
const {default:puppeteer}=await import(pathToFileURL(resolve(modulePath)));
const browser=await puppeteer.launch({executablePath:chrome,headless:true,args:['--no-sandbox','--disable-dev-shm-usage']});
try {
  const manifest=JSON.parse(await readFile(resolve(out,'manifest.json'),'utf8'));
  const page=await browser.newPage();const errors=[];page.on('pageerror',e=>errors.push(String(e)));
  await page.setViewport({width:1420,height:200,deviceScaleFactor:1});
  for (const item of manifest.items.filter(x=>x.mermaid)) {
    const stem=item.file.replace(/\.png$/,'');
    await page.goto(pathToFileURL(resolve(out,stem+'.html')).href);
    await page.waitForFunction(()=>document.body.dataset.ready==='true');
    await page.evaluate(()=>document.fonts.ready);
    await page.screenshot({path:resolve(out,item.file),fullPage:true});
    await writeFile(resolve(out,stem+'.svg'),await page.$eval('#diagram svg',el=>el.outerHTML));
  }
  await page.setViewport({width:1420,height:1000,deviceScaleFactor:1});
  await page.goto(pathToFileURL(resolve(out,'index.html')).href);
  await page.waitForFunction(()=>[...document.images].every(x=>x.complete&&x.naturalWidth>0));
  const desktop=await page.evaluate(()=>({images:document.images.length,overflow:document.documentElement.scrollWidth>innerWidth}));
  await page.screenshot({path:resolve(out,'gallery-desktop.png')});
  await page.setViewport({width:390,height:844,deviceScaleFactor:1});
  const mobile=await page.evaluate(()=>({overflow:document.documentElement.scrollWidth>innerWidth}));
  await page.screenshot({path:resolve(out,'gallery-mobile.png')});
  if(errors.length||desktop.images!==manifest.items.length||desktop.overflow||mobile.overflow)throw new Error(JSON.stringify({errors,desktop,mobile}));
  await writeFile(resolve(out,'browser-check.json'),JSON.stringify({errors,desktop,mobile},null,2));
  console.log(JSON.stringify({errors,desktop,mobile}));
} finally {await browser.close();}
