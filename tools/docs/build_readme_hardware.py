#!/usr/bin/env python3
"""Lay out an editable hardware guide from the original CAD renders."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


def embedded_image(path: Path, x: int, y: int, width: int, height: int) -> str:
    image = Image.open(path).convert('RGBA')
    array = np.asarray(image)
    selected = (array[:, :, 3] > 0) & (np.min(array[:, :, :3], axis=2) < 245)
    rows, cols = np.nonzero(selected)
    left, top = max(0, int(cols.min())-16), max(0, int(rows.min())-16)
    right, bottom = min(image.width, int(cols.max())+17), min(image.height, int(rows.max())+17)
    data = base64.b64encode(path.read_bytes()).decode()
    return (f'<svg x="{x}" y="{y}" width="{width}" height="{height}" '
            f'viewBox="{left} {top} {right-left} {bottom-top}">'
            f'<image width="{image.width}" height="{image.height}" href="data:image/png;base64,{data}"/></svg>')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renders', type=Path, required=True)
    parser.add_argument('--assembly', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    parts = [
        ('01', 'Dual arms', '07_dual_arms.png'),
        ('02', 'RGB-D camera', '08_head_camera.png'),
        ('03', 'LiDAR', '09_lidar.png'),
        ('04', 'Mecanum drive', '10_wheels_and_drives.png'),
    ]
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="880" viewBox="0 0 1400 880">',
           '<style>text{font-family:Arial,sans-serif;fill:#294e43}.label{font-size:24px;font-weight:600}.number{font-size:17px;fill:#578c78}</style>',
           '<rect width="1400" height="880" rx="24" fill="#f3f7f3"/>',
           '<text x="48" y="62" font-size="30" font-weight="700">Cleany hardware</text>',
           '<text x="48" y="92" font-size="16" fill="#638376">Original CAD assembly</text>',
           '<rect x="32" y="116" width="592" height="730" rx="20" fill="white"/>',
           embedded_image(args.assembly, 60, 138, 536, 680)]
    sources = [args.assembly]
    for index, (number, title, filename) in enumerate(parts):
        x, y = 648+(index%2)*360, 116+(index//2)*376
        svg += [f'<rect x="{x}" y="{y}" width="336" height="354" rx="20" fill="white"/>',
                f'<text x="{x+22}" y="{y+34}" class="number">{number}</text>',
                f'<text x="{x+22}" y="{y+65}" class="label">{title}</text>',
                embedded_image(args.renders/filename, x+24, y+91, 288, 237)]
        sources.append(args.renders/filename)
    svg.append('</svg>')
    (args.output/'hardware-guide.svg').write_text('\n'.join(svg))
    (args.output/'hardware-guide.json').write_text(json.dumps({
        'kind': 'original CAD render layout', 'geometry_redrawn': False,
        'sources': [{'path': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                    for p in sources],
        'wheel_mesh': 'simplified cylinders in original CAD; Mecanum behavior shown separately',
    }, indent=2)+'\n')


if __name__ == '__main__':
    main()
