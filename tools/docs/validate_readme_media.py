#!/usr/bin/env python3
"""Decode the media set, check recorded evidence, and build README previews."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import markdown
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    media, review = args.run/'media', args.run/'review'
    review.mkdir(parents=True, exist_ok=True)
    results = []
    for path in sorted(media.glob('*')):
        if path.suffix in ('.png', '.gif'):
            with Image.open(path) as image:
                duration, digests = 0, set()
                for index in range(image.n_frames):
                    image.seek(index)
                    decoded = image.convert('RGB')
                    duration += image.info.get('duration', 0)
                    digests.add(hashlib.sha256(decoded.tobytes()).hexdigest())
                if path.suffix == '.gif' and len(digests) < 2:
                    raise ValueError(f'Static GIF: {path}')
                results.append({'file':path.name, 'dimensions':image.size,
                                'frames':image.n_frames, 'duration_ms':duration,
                                'unique_frames':len(digests)})
        elif path.suffix == '.mp4':
            subprocess.run(['ffmpeg', '-v', 'error', '-i', str(path), '-f', 'null', '-'], check=True)
    perception = json.loads((media/'perception-pipeline.json').read_text())
    assert all(.005 <= value < 1 for value in perception['size_m'])
    assert np.isclose(np.linalg.norm(perception['plane_normal']), 1)
    stamp = perception['frame']
    assert abs(stamp['color_timestamp_ms']-stamp['depth_timestamp_ms']) < 1
    for name in ('freecad-workspace', 'gazebo-study-cafe'):
        evidence = json.loads((media/f'{name}.json').read_text())
        for entry in evidence['sources']:
            assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest() == entry['sha256']
    cad = json.loads((media/'freecad-workspace.json').read_text())
    assert cad['source_model_sha256_before'] == cad['source_model_sha256_after']
    assert cad['capture_kind'] == 'actual FreeCAD application window'
    isaac = json.loads((media/'isaac-parallel.json').read_text())
    assert isaac['environments'] == 16 and isaac['independent_articulations'] == [1]*16
    assert isaac['trained_policy_executed'] is False
    origins = np.array([p['origin_m'] for p in isaac['placements']])
    assert len(np.unique(origins, axis=0)) == 16
    for before, after, origin in zip(isaac['initial_layout'], isaac['final_layout'], origins):
        index = before['names'].index('base_link')
        assert np.linalg.norm(np.array(after['links'][0][index][:3])-origin) < .001
    motion = json.loads((media/'mecanum-motion.json').read_text())
    for mode in ('vx', 'vy', 'wz'):
        samples = [s for s in motion['samples'] if s['mode'] == mode]
        assert len(samples) == motion['frames']
        assert np.linalg.norm(samples[-1]['base_xy_yaw']) < 1e-9
    for filename in ('README.md', 'README.ko.md'):
        text = (ROOT/filename).read_text()
        links = re.findall(r'(?:src=["\']|!\[[^\]]*\]\()([^"\')]+)', text)
        for link in links:
            if '://' not in link:
                assert (ROOT/link).is_file(), link
        html = markdown.markdown(text, extensions=['tables', 'fenced_code', 'toc'])
        if len(re.findall(r'<img\b', html)) != len(links):
            raise ValueError(f'Preview did not render every README image: {filename}')
        def preview_link(match: re.Match[str]) -> str:
            target = match[2]
            if target in ('README.md', 'README.ko.md'):
                target += '.html'
            else:
                target = (ROOT/target).as_uri()
            return f'{match[1]}="{target}"'

        html = re.sub(r'(src|href)="(?!https?:|#)([^"]+)"', preview_link, html)
        css = ('body{max-width:980px;margin:36px auto;padding:0 24px;font:16px/1.65 system-ui;color:#213b34}'
               'img{max-width:100%;height:auto}table{width:100%;border-collapse:collapse;table-layout:fixed}'
               'td,th{padding:9px;border:1px solid #dce6df;overflow-wrap:anywhere}'
               'pre{overflow:auto;padding:18px;background:#f3f7f3;font-size:13px}'
               'h2{margin-top:36px}a{color:#286657}.mermaid{padding:20px 0;text-align:center}'
               '.mermaid svg{max-width:100%;height:auto}'
               '@media(max-width:600px){body{font-size:14px;margin:20px auto;padding:0 16px}td,th{padding:5px}}')
        mermaid = (ROOT/'docs/guide/vendor/mermaid.tiny.js').as_uri()
        script = '''
<script>
window.previewReady = false;
(async () => {
  mermaid.initialize({startOnLoad:false,securityLevel:'strict',theme:'neutral'});
  for (const code of document.querySelectorAll('code.language-mermaid')) {
    const diagram = document.createElement('div');
    diagram.className = 'mermaid';
    diagram.textContent = code.textContent;
    code.parentElement.replaceWith(diagram);
  }
  await mermaid.run({querySelector:'.mermaid'});
  window.previewReady = true;
})();
</script>'''
        language = 'ko' if filename == 'README.ko.md' else 'en'
        (review/(filename+'.html')).write_text(
            f'<!doctype html><html lang="{language}"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>Cleany README · {language.upper()}</title><style>{css}</style></head>'
            f'<body>{html}<script src="{mermaid}"></script>{script}</body></html>')
    (review/'validation.json').write_text(json.dumps({'passed':True, 'decoded':results,
        'checks':['RGB-D timestamp alignment', '3D geometry', 'FreeCAD/Gazebo source hashes',
                  '16 independent Isaac bases', 'Mecanum loop closure', 'bilingual image links']}, indent=2)+'\n')
    print(f'Validated {len(results)} images/animations, MP4 decoding, evidence and README links')


if __name__ == '__main__':
    main()
