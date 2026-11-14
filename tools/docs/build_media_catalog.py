#!/usr/bin/env python3
"""Build an offline review gallery from explicitly selected project media."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import shutil
from pathlib import Path


def escape(value: str) -> str:
    return html.escape(value, quote=True)


def file_link(value: str, label: str) -> str:
    path = Path(value)
    if path.is_file():
        return f'<a href="{escape(path.as_uri())}">{escape(label)}</a>'
    return f'<span>{escape(value)}</span>'


CSS = """
:root{color-scheme:light;--ink:#223d37;--muted:#65756e;--line:#dce6df;--green:#286657}
*{box-sizing:border-box}body{margin:0;background:#f4f7f3;color:var(--ink);font:15px/1.65 system-ui,sans-serif}
main{max-width:1240px;margin:auto;padding:56px 28px}header{max-width:850px;margin-bottom:28px}
.eyebrow{color:var(--green);font-size:12px;font-weight:700;letter-spacing:.16em}h1{font-size:38px;line-height:1.2;margin:12px 0}
h2{font-size:24px;margin:44px 0 18px}h3{font-size:19px;margin:10px 0 8px}p{margin:8px 0;color:var(--muted)}
nav{display:flex;gap:8px;flex-wrap:wrap;margin:28px 0}button{font:inherit;padding:6px 16px;border:1px solid var(--line);border-radius:30px;background:white;color:var(--ink);cursor:pointer}button.active{background:var(--green);color:white}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:22px}article{min-width:0;background:#fff;border:1px solid var(--line);border-radius:14px;overflow:hidden}
.info{padding:18px 22px}.number{font:12px ui-monospace,monospace;color:var(--green)}.tag{float:right;font-size:11px;padding:2px 8px;background:#edf3ef;border-radius:4px}
.media{display:flex;gap:4px;height:285px;padding:12px;background:#edf1ed}.media a{min-width:0;flex:1;display:flex;align-items:center;justify-content:center}.media img{width:100%;height:100%;object-fit:contain}
.media.parts{display:grid;grid-template-columns:1.2fr 1fr 1fr;grid-template-rows:1fr 1fr}.media.parts a:first-child{grid-row:span 2}.media.parts img{object-fit:contain}
.labels{display:flex;gap:6px;padding:0 12px 8px;background:#edf1ed;font-size:11px;color:var(--muted)}.labels span{flex:1;text-align:center}
details{font-size:12px;border-top:1px solid var(--line);padding-top:10px;margin-top:14px}summary{cursor:pointer;color:var(--green)}code{display:block;font:11px/1.6 ui-monospace,monospace;overflow-wrap:anywhere;margin:8px 0}a{color:var(--green)}
.ideas{display:grid;grid-template-columns:1fr 1fr;gap:16px}.idea{padding:20px;background:#e8f0eb;border-radius:10px}.idea h3{margin-top:0}.idea p{font-size:14px}.foot{font-size:12px;margin-top:28px}
[hidden]{display:none!important}.sheet-only{display:none}
body.sheet main{max-width:none;padding:28px}body.sheet header{margin:0 0 20px}body.sheet h1{font-size:28px}body.sheet .grid{grid-template-columns:repeat(4,1fr);gap:14px}body.sheet .media{height:190px;padding:8px}body.sheet .info{padding:12px 16px}body.sheet h3{font-size:16px;margin:5px 0}body.sheet .info p,body.sheet details,body.sheet nav,body.sheet .ideas,body.sheet h2,body.sheet .foot,body.sheet .labels{display:none}body.sheet header p{font-size:13px}
@media(max-width:700px){main{padding:28px 16px}h1{font-size:29px}.grid,.ideas{grid-template-columns:1fr}.media{height:240px}.info{padding:16px}.tag{float:none;margin-left:8px}}
"""


def build(config: dict, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    assets = output / "assets"
    assets.mkdir(exist_ok=True)
    cards: list[str] = []
    records: list[dict] = []
    completed = config.get('mode') == 'completed'
    for index, item in enumerate(config["items"], 1):
        images, labels, sources = [], [], []
        for media_index, media in enumerate(item["media"]):
            source = Path(media["path"]).resolve(strict=True)
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            name = f'{item["id"]}-{media_index}{source.suffix.lower()}'
            target = assets / name
            shutil.copyfile(source, target)
            if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
                raise ValueError(f"Copy hash mismatch: {target}")
            url = f"assets/{name}"
            label = escape(media["label"])
            images.append(f'<a href="{url}" target="_blank"><img src="{url}" alt="{label}"></a>')
            labels.append(f"<span>{label}</span>")
            sources.append(f'<code>{escape(str(source))}</code>')
            records.append({"id": item["id"], "source": str(source), "asset": url,
                            "sha256": digest, "bytes": target.stat().st_size,
                            "remote_source": item.get("remote_source"),
                            "related_video": item.get("video"),
                            "frame_seconds": item.get("frame_seconds"),
                            "evidence": item.get("evidence"),
                            "scope": item["scope"]})
        for key, label in [("video", "원본 영상"), ("evidence", "실험 기록")]:
            if key in item:
                sources.append(file_link(item[key], label))
        if "remote_source" in item:
            sources.append(f'<code>{escape(item["remote_source"])}</code>')
        cls = "media parts" if len(images) > 3 else "media"
        cards.append(f'''<article data-group="{escape(item['group'])}">
<div class="{cls}">{''.join(images)}</div><div class="labels">{''.join(labels)}</div>
<div class="info"><span class="number">{index:02d} / {escape(item['group'])}</span><span class="tag">{escape(item['priority'])}</span>
<h3>{escape(item['title'])}</h3><p>{escape(item['note'])}</p>
<details><summary>출처와 사용 범위</summary><p>{escape(item['scope'])}</p>{''.join(sources)}</details></div></article>''')
    groups = ["전체", *dict.fromkeys(item["group"] for item in config["items"])]
    buttons = ''.join(f'<button data-filter="{escape(g)}" class="{"active" if i == 0 else ""}">{escape(g)}</button>' for i, g in enumerate(groups))
    ideas = ''.join(f'<div class="idea"><span class="number">{escape(x["priority"])}</span><h3>{escape(x["title"])}</h3><p>{escape(x["how"])}</p></div>' for x in config["ideas"])
    ideas_section = f'<h2>새로 만들면 좋은 자료</h2><section class="ideas">{ideas}</section>' if config['ideas'] else ''
    count = f'{len(cards)}개 완성 자료' if completed else f'{len(cards)}개 후보 · 새로 만들 자료 {len(config["ideas"])}개'
    footer = '개별 원본, GIF, 출처는 각 카드에서 확인할 수 있습니다.' if completed else 'README에는 아직 추가하지 않은 후보 목록입니다.'
    sheet_css = 'body.sheet .grid{grid-template-columns:repeat(3,1fr)}body.sheet .media{height:240px}' if completed else ''
    document = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{escape(config['title'])}</title>
<style>{CSS}{sheet_css}</style><main><header><span class="eyebrow">CLEANY / MEDIA REVIEW</span><h1>{escape(config['title'])}</h1>
<p>{count}</p><p>{escape(config['description'])}</p></header>
<nav aria-label="분류">{buttons}</nav><section class="grid">{''.join(cards)}</section>
{ideas_section}
<div class="foot"><p>{escape(' / '.join(config['omitted']))}</p><p>{escape(config['created'])} · {footer}</p></div></main>
<script>for(const button of document.querySelectorAll('[data-filter]'))button.addEventListener('click',()=>{{for(const b of document.querySelectorAll('[data-filter]'))b.classList.toggle('active',b===button);for(const card of document.querySelectorAll('article'))card.hidden=button.dataset.filter!=='전체'&&card.dataset.group!==button.dataset.filter;}});function setSheet(){{document.body.classList.toggle('sheet',location.hash==='#sheet');}}window.addEventListener('hashchange',setSheet);setSheet();</script></html>'''
    (output / "index.html").write_text(document, encoding="utf-8")
    manifest = {"created": config["created"], "repo": config["repo"],
                "commit": config["commit"], "assets": records,
                "transform": "Original images copied unchanged; video previews are extracted frames.",
                "ideas": config["ideas"]}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = build(json.loads(args.config.read_text(encoding="utf-8")), args.output)
    print(f"Built {args.output / 'index.html'} ({len(manifest['assets'])} verified assets)")


if __name__ == "__main__":
    main()
