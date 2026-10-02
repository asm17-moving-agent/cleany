#!/usr/bin/env python3
"""Render a deterministic SVG overlay of traced walls on the user's floor plan."""

from __future__ import annotations

import argparse
import base64
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_gazebo_sim'))
from cleany_gazebo_sim.world.facility_layout import load_facility_layout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', type=Path, default=ROOT / 'artifacts/facility/layout_overlay.svg'
    )
    args = parser.parse_args()
    layout = load_facility_layout(
        ROOT / 'ros2_ws/src/cleany_gazebo_sim/config/facility_18f/facility_layout.yaml'
    )
    source = layout.raw['source']
    pixels, maps = source['reference_pixel_anchors'], source['reference_map_anchors']
    sx = (pixels[1][0] - pixels[0][0]) / (maps[1][0] - maps[0][0])
    sy = (pixels[2][1] - pixels[0][1]) / (maps[2][1] - maps[0][1])

    def pixel(p):
        return (
            pixels[0][0] + (p[0] - maps[0][0]) * sx,
            pixels[0][1] + (p[1] - maps[0][1]) * sy,
        )

    from PIL import Image

    reference = ROOT / source['image']
    width, height = Image.open(reference).size
    image = base64.b64encode(reference.read_bytes()).decode()
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
        f'<image href="data:image/png;base64,{image}" width="{width}" height="{height}" opacity="0.65"/>',
    ]
    for segment in layout.segments:
        a, b = (pixel(layout.map_point(p)) for p in (segment.start, segment.end))
        color = '#d28c15' if segment.guard else (
            '#55bdda' if segment.material == 'glass' else '#008a9c'
        )
        svg.append(
            f'<path d="M{a[0]} {a[1]}L{b[0]} {b[1]}" stroke="{color}" stroke-width="3" fill="none"><title>{segment.name}</title></path>'
        )
    bounds = layout.raw['central_table']['bounds']
    a, b = pixel(bounds[:2]), pixel(bounds[2:])
    svg.append(
        f'<rect x="{a[0]}" y="{a[1]}" width="{b[0] - a[0]}" '
        f'height="{b[1] - a[1]}" fill="#b89964" fill-opacity="0.6">'
        '<title>THE GROUND table, 0.80 m</title></rect>'
    )
    for door in layout.raw['doors']:
        a, b = (pixel(p) for p in door['points'])
        svg.append(
            f'<path d="M{a[0]} {a[1]}L{b[0]} {b[1]}" stroke="#33bb55" stroke-width="5"><title>{door["id"]}</title></path>'
        )
    points = ' '.join(
        f'{x},{y}'
        for x, y in (pixel(layout.map_point(p)) for p in layout.raw['route_xy'])
    )
    svg.append(
        f'<polyline points="{points}" fill="none" stroke="#e13f83" stroke-width="3" stroke-dasharray="8 5"/>'
    )
    svg.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(svg), encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
