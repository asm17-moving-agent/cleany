#!/usr/bin/env python3
"""RPP-only isolated tuning runner; retains the common safety pipeline."""
import argparse
import os
import sys
from pathlib import Path

import yaml

BASE = Path('/home/donglee/workspace/cleany/controller-comparison-20261013')
os.environ['CLEANY_COMPARISON_REPO'] = str(Path(__file__).parents[2] / 'rpp-source')
sys.path.insert(0, str(Path(__file__).parent))
import common  # noqa: E402


TUNES = {
    'baseline': {},
    'curvature': {'wrapped_plugin': 'cleany_navigation::CurvatureLimitedRPP', 'tracking_yaw_limit': 0.14},
    'curvature_lookahead': {'wrapped_plugin':'cleany_navigation::CurvatureLimitedRPP','tracking_yaw_limit':0.14,'min_lookahead_dist':0.20},
    'curvature_lookahead12': {'wrapped_plugin':'cleany_navigation::CurvatureLimitedRPP','tracking_yaw_limit':0.14,'min_lookahead_dist':0.12},
    'native_lookahead20': {'min_lookahead_dist':0.20},
    'curvature_terminal10': {'wrapped_plugin':'cleany_navigation::CurvatureLimitedRPP','tracking_yaw_limit':0.14,'min_lookahead_dist':0.20,'rotate_to_heading_angular_vel':0.10},
    'aux_zero_refresh': {'max_allowed_time_to_collision_up_to_carrot':0.35},
    'curvature_lookahead18': {'wrapped_plugin':'cleany_navigation::CurvatureLimitedRPP','tracking_yaw_limit':0.14,'min_lookahead_dist':0.18},
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
    if tune_name == 'aux_zero_refresh':
        safety_file = out / 'config/nav2_safety.yaml'
        safety = yaml.safe_load(safety_file.read_text())
        safety['collision_monitor']['ros__parameters']['stop_pub_timeout'] = 300.0
        safety_file.write_text(yaml.safe_dump(safety, sort_keys=False))
        scenario['auxiliary_common_layer_change'] = {'collision_monitor.stop_pub_timeout':300.0, 'baseline_value':2.0, 'purpose':'retain sustained zero output for the 240s campaign; sensor/geometry/stop checks unchanged'}
    motion_file.write_text(yaml.safe_dump(motion, sort_keys=False))
    scenario['rpp_tune_name'] = tune_name
    scenario['rpp_tune'] = TUNES[tune_name]
    (out / 'scenario.json').write_text(__import__('json').dumps(scenario, indent=2))
    return scenario


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--tune', choices=sorted(TUNES), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal-index', type=int, choices=range(5))
    parser.add_argument('--sim-limit',type=float,default=240.0)
    parser.add_argument('--domain', type=int, default=228)
    args = parser.parse_args()
    if not 0 < args.sim_limit <= 240.0:parser.error('sim-limit must be between 0 and 240')
    os.environ['CLEANY_RPP_SIM_LIMIT'] = str(args.sim_limit)
    os.environ['CLEANY_RPP_TUNE'] = args.tune
    os.environ['CLEANY_COMPARISON_REPO'] = '/home/donglee/workspace/cleany/controller-optimization-20261013/rpp-source'
    os.environ['CLEANY_BENCHMARK_TIMING'] = '1'
    common.setup = setup
    if args.domain != 228:
        parser.error('This RPP campaign owns ROS domain 228 only')
    args.output.mkdir(parents=True, exist_ok=True)
    common.describe(args.output)
    for index in ([args.goal_index] if args.goal_index is not None else range(5)):
        result=common.run(args.output / f'{index}_{common.GOALS[index][0]}', 'rpp', index, 1.0, args.domain)
        if not result.get('completed'):raise RuntimeError('Infrastructure failed; preserve result and stop campaign')


if __name__ == '__main__':
    main()
