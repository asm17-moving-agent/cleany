#!/usr/bin/env python3
"""RPP-only isolated tuning runner; retains the common safety pipeline."""
import argparse
import os
import sys
from pathlib import Path

import yaml

BASE = Path('/home/donglee/workspace/cleany/controller-comparison-20261013')
os.environ['CLEANY_COMPARISON_REPO'] = str(Path(__file__).parent / 'rpp-source')
sys.path.insert(0, str(Path(__file__).parent))
import run_rpp_common_v2 as common  # noqa: E402


TUNES = {
    'baseline': {},
    'short_ttc': {
        'max_allowed_time_to_collision_up_to_carrot': 0.35,
    },
    'short_lookahead_ttc': {
        'lookahead_dist': 0.45,
        'min_lookahead_dist': 0.20,
        'max_lookahead_dist': 0.60,
        'lookahead_time': 1.0,
        'max_allowed_time_to_collision_up_to_carrot': 0.35,
        'regulated_linear_scaling_min_radius': 0.65,
        'cost_scaling_dist': 0.45,
    },
    'terminal_rotation': {
        'lookahead_dist': 0.45,
        'min_lookahead_dist': 0.20,
        'max_lookahead_dist': 0.60,
        'lookahead_time': 1.0,
        'max_allowed_time_to_collision_up_to_carrot': 0.35,
        'regulated_linear_scaling_min_radius': 0.65,
        'cost_scaling_dist': 0.45,
        'rotate_to_heading_angular_vel': 0.10,
        'rotate_to_heading_min_angle': 0.55,
    },
}

ORIGINAL_SETUP = common.setup


def setup(out: Path, candidate: str, rtf: float):
    scenario = ORIGINAL_SETUP(out, candidate, rtf)
    tune_name = os.environ['CLEANY_RPP_TUNE']
    motion_file = out / 'config' / 'comparison_motion.yaml'
    motion = yaml.safe_load(motion_file.read_text())
    params = motion['controller_server']['ros__parameters']['FollowPath']
    params.update(TUNES[tune_name])
    motion_file.write_text(yaml.safe_dump(motion, sort_keys=False))
    scenario['rpp_tune_name'] = tune_name
    scenario['rpp_tune'] = TUNES[tune_name]
    (out / 'scenario.json').write_text(__import__('json').dumps(scenario, indent=2))
    return scenario


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--tune', choices=sorted(TUNES), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal-index', type=int, required=True)
    parser.add_argument('--domain', type=int, default=228)
    args = parser.parse_args()
    os.environ['CLEANY_RPP_TUNE'] = args.tune
    os.environ['CLEANY_COMPARISON_REPO'] = '/home/donglee/workspace/cleany/controller-optimization-20261013/rpp-source'
    os.environ['CLEANY_BENCHMARK_TIMING'] = '1'
    common.setup = setup
    if args.domain != 228:
        parser.error('This RPP campaign owns ROS domain 228 only')
    args.output.mkdir(parents=True, exist_ok=True)
    common.describe(args.output)
    common.run(args.output / f'{args.goal_index}_{common.GOALS[args.goal_index][0]}', 'rpp', args.goal_index, 1.0, args.domain)


if __name__ == '__main__':
    main()
