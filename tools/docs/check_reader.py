#!/usr/bin/env python3
"""이미 실행 중인 Chromium CDP endpoint로 모든 문서 페이지를 검사합니다.

websockets가 필요합니다. 브라우저, ROS 노드, 서버나 로봇 프로세스를 시작하지
않습니다. 보고서와 도표별 스크린샷은 저장소 밖의 --output에 저장합니다.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
from pathlib import Path
import urllib.request

import websockets


ROOT = Path(__file__).resolve().parents[2]


async def check(endpoint: str, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(endpoint.rstrip("/") + "/json/list") as response:
        target = next(t for t in json.load(response) if t["type"] == "page")
    events, results = [], []
    sequence = 0
    async with websockets.connect(target["webSocketDebuggerUrl"], origin=endpoint,
                                  max_size=40_000_000) as websocket:
        async def rpc(method: str, params: dict | None = None) -> dict:
            nonlocal sequence
            sequence += 1
            ident = sequence
            await websocket.send(json.dumps({"id": ident, "method": method, "params": params or {}}))
            while True:
                message = json.loads(await websocket.recv())
                if message.get("method") == "Runtime.exceptionThrown":
                    events.append(message)
                if message.get("id") == ident:
                    if "error" in message:
                        raise RuntimeError(message)
                    return message.get("result", {})

        async def js(expression: str):
            response = await rpc("Runtime.evaluate", {"expression": expression,
                                "returnByValue": True, "awaitPromise": True})
            if "exceptionDetails" in response:
                raise RuntimeError(response["exceptionDetails"])
            return response.get("result", {}).get("value")

        await rpc("Runtime.enable")
        await rpc("Page.enable")
        await rpc("Page.navigate", {"url": (ROOT / "docs/index.html").as_uri()})
        for _ in range(100):
            if await js("typeof pages !== 'undefined' && typeof render === 'function'"):
                break
            await asyncio.sleep(.1)
        paths = await js("Object.keys(pages)")
        for mobile in [False, True]:
            await rpc("Emulation.setDeviceMetricsOverride", {
                "width": 390 if mobile else 1440, "height": 844 if mobile else 1050,
                "deviceScaleFactor": 1, "mobile": mobile})
            for path in paths:
                await js(f"history.replaceState(null, '', route({json.dumps(path)})); render()")
                await js("document.documentElement.style.scrollBehavior='auto'; "
                         "document.querySelectorAll('article details').forEach(e=>e.open=true)")
                await js("Promise.all([...article.querySelectorAll('img')].map(async i=>{"
                         "i.loading='eager';try{await i.decode()}catch(e){} }))")
                state = await js("({failedDiagrams:article.querySelectorAll('.diagram[aria-label]').length,"
                    "overflow:document.documentElement.scrollWidth>innerWidth+1,"
                    "brokenImages:[...article.querySelectorAll('img')].filter(i=>!i.naturalWidth).map(i=>i.src),"
                    "slots:[...article.querySelectorAll('[data-visual-id]')].map(e=>e.dataset.visualId)})")
                results.append({"page": path, "mobile": mobile, **state})
                for slot in state["slots"]:
                    selector = '[data-visual-id="' + slot + '"]'
                    await js(f"document.querySelector({json.dumps(selector)}).scrollIntoView({{block:'center',behavior:'instant'}})")
                    await asyncio.sleep(.05)
                    screenshot = await rpc("Page.captureScreenshot", {"format": "png"})
                    (output / f"{slot}-{'mobile' if mobile else 'desktop'}.png").write_bytes(base64.b64decode(screenshot["data"]))
                if state["failedDiagrams"] or state["overflow"] or state["brokenImages"]:
                    print("FAIL", path, mobile, state, flush=True)
            print(f"Rendered {len(paths)} pages at {'390' if mobile else '1440'} px", flush=True)
        await js("history.replaceState(null, '', route('ros2_ws/src/cleany_navigation/docs/planning.md')); render()")
        await js("document.querySelectorAll('article details').forEach(e=>e.open=true); article.querySelector('img').click()")
        zoom = await js("document.getElementById('image-dialog').open")
        await js("document.getElementById('image-dialog').close()")
        await rpc("Page.navigate", {"url": (ROOT / "docs/guide/index.html").as_uri()})
        for _ in range(100):
            if await js("typeof loadRoute === 'function' && Array.isArray(pages)"):
                break
            await asyncio.sleep(.1)
        guide_ids = await js("pages.map(p=>p.id)")
        guide_results = []
        for mobile in [False, True]:
            await rpc("Emulation.setDeviceMetricsOverride", {
                "width": 390 if mobile else 1440, "height": 844 if mobile else 1050,
                "deviceScaleFactor": 1, "mobile": mobile})
            for ident in guide_ids:
                await js(f"history.replaceState(null, '', route({json.dumps(ident)})); loadRoute()")
                await js("Promise.all([...article.querySelectorAll('img')].map(async i=>{"
                         "i.loading='eager';try{await i.decode()}catch(e){} }))")
                state = await js("({failedDiagrams:article.querySelectorAll('.mermaid-error,.error').length,"
                    "overflow:document.documentElement.scrollWidth>innerWidth+1,"
                    "brokenImages:[...article.querySelectorAll('img')].filter(i=>!i.naturalWidth).map(i=>i.alt)})")
                guide_results.append({"page": ident, "mobile": mobile, **state})
            print(f"Rendered {len(guide_ids)} guide pages at {'390' if mobile else '1440'} px", flush=True)
        report = {"pages": results, "guide": guide_results, "runtime_errors": events, "image_zoom": zoom}
        (output / "reader-report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        if events or not zoom or any(r["failedDiagrams"] or r["overflow"] or r["brokenImages"] for r in results + guide_results):
            raise SystemExit("Documentation reader checks failed; see reader-report.json")
        print("No Mermaid failures, missing images, horizontal overflow or runtime exceptions; zoom opens")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="http://localhost:9228")
    parser.add_argument("--output", type=Path, default=Path.home() / ".cache/cleany/docs-reader-check")
    args = parser.parse_args()
    asyncio.run(check(args.endpoint, args.output))


if __name__ == "__main__":
    main()
