"""Correct straight primitive length metadata without changing lattice poses."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    document = json.loads(args.source.read_text())
    changed = []
    for primitive in document['primitives']:
        if primitive['arc_length'] or primitive['start_angle_index'] != primitive['end_angle_index']:
            continue
        points = [[0.0, 0.0], *primitive['poses']]
        distance = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
        if abs(distance - primitive['trajectory_length']) > 1e-4:
            changed.append(primitive['trajectory_id'])
            primitive['trajectory_length'] = primitive['straight_length'] = round(distance, 5)
    args.destination.write_text(json.dumps(document, indent='\t')+'\n')
    print(json.dumps({'corrected_trajectory_ids': changed}))


if __name__ == '__main__':
    main()
