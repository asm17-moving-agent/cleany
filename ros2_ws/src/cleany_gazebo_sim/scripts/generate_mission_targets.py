#!/usr/bin/env python3
"""Generate simulation approach targets and their clearance evidence."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from cleany_gazebo_sim.world.mission_targets import generate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--chair-mesh-root', type=Path, required=True,
                        help='Fuel OfficeChairGrey/1 directory containing meshes/')
    parser.add_argument('--home', type=float, nargs=3, default=[-1.865, -4.705, 1.5708])
    parser.add_argument('--half-extents', type=float, nargs=2, default=(.3185, .4074),
                        help='CAD body half-length and half-width in meters')
    parser.add_argument('--padding', type=float, default=.02)
    parser.add_argument('--localization-margin', type=float, default=.10,
                        help='extra XY footprint margin for localization uncertainty; simulation assumption')
    parser.add_argument('--reference-world', type=Path,
                        help='original chair layout used only to retain front direction for removed chairs')
    parser.add_argument('--front-offset', type=float, default=1.2,
                        help='preferred desk-to-goal offset in meters')
    parser.add_argument('--override', nargs=4, action='append', default=[],
                        metavar=('SEAT_ID', 'X', 'Y', 'YAW'),
                        help='planner-reviewed adjustment; rechecked against the footprint')
    args = parser.parse_args()
    targets, evidence = generate(args.world, args.map, args.home, args.chair_mesh_root,
                                 {name: [float(x), float(y), float(yaw)]
                                  for name, x, y, yaw in args.override},
                                 tuple(args.half_extents), args.padding, args.front_offset,
                                 args.localization_margin, args.reference_world)
    args.output.write_text('# Approximate simulation approach poses; arm reach is unvalidated.\n'+
                           yaml.safe_dump(targets, sort_keys=False))
    args.output.with_suffix('.validation.json').write_text(json.dumps(evidence, indent=2)+'\n')
    print(f'Generated {len(targets["seats"])} collision-checked approach poses: {args.output}')


if __name__ == '__main__':
    main()
