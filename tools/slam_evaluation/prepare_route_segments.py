#!/usr/bin/env python3
"""Create one route-follower parameter file per study-cafe route edge."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('route_config', type=Path)
    parser.add_argument('output_directory', type=Path)
    return parser.parse_args()


def main() -> None:
    options = parse_args()
    source = yaml.safe_load(options.route_config.read_text(encoding='utf-8'))
    parameters = source['ground_truth_route_follower']['ros__parameters']
    values = [float(value) for value in parameters['waypoints_xy']]
    points = list(zip(values[::2], values[1::2]))
    if len(points) < 2:
        raise ValueError('route must contain at least two waypoints')

    options.output_directory.mkdir(parents=True, exist_ok=True)
    manifest_lines = []
    initial_yaw = math.pi / 2.0
    for index, (start, target) in enumerate(zip(points, points[1:])):
        if index == 0:
            yaw = initial_yaw
        else:
            previous = points[index - 1]
            yaw = math.atan2(start[1] - previous[1], start[0] - previous[0])
        segment_parameters = dict(parameters)
        segment_parameters['waypoints_xy'] = [*start, *target]
        segment = {
            'ground_truth_route_follower': {
                'ros__parameters': segment_parameters,
            }
        }
        config_path = options.output_directory / f'segment_{index:02d}.yaml'
        config_path.write_text(
            yaml.safe_dump(segment, sort_keys=False), encoding='utf-8'
        )
        spawn = f'{start[0]},{start[1]},0.38,0.0,0.0,{yaw}'
        manifest_lines.append(f'{index:02d}\t{spawn}\t{config_path}')
    (options.output_directory / 'manifest.tsv').write_text(
        '\n'.join(manifest_lines) + '\n', encoding='utf-8'
    )


if __name__ == '__main__':
    main()
