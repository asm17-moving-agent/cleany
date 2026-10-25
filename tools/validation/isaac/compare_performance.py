"""Compare the same route and unchanged robot/navigation inputs across adapters."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from analyze_profile import analyze
from compare_navigation import measure


def inspect(run: Path) -> dict:
    scene = json.loads((run/'config/room.json').read_text())
    profile = analyze(run)
    navigation = measure(run)
    inputs = {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in (
        'assets/cleany.urdf', 'config/controllers.yaml', 'config/motors.json', 'config/navigation.yaml',
        'config/world_geometry.json')}
    build = json.loads((run/'provenance/build.json').read_text())
    return {'run': str(run.resolve()), 'target_id': scene['target_id'],
        'configuration': profile['configuration'], 'source_digest': build['source_digest'],
        'input_hashes': inputs, 'physics_hz': scene['physics_hz'],
        'navigation_wall_s': sum(phase['wall_s'] for phase in navigation['phases'].values()),
        'navigation_simulation_s': navigation['navigation_simulation_s'],
        'rtf': profile['phases']['NAVIGATION']['rtf'],
        'physics_mean_ms': profile['phases']['NAVIGATION']['stages']['physics_step']['mean_wall_ms'],
        'ros_publish_wall_percent': sum(profile['phases']['NAVIGATION']['stages'][name]['wall_percent']
                                        for name in ('clock_publish', 'odometry_tf')),
        'cpu_by_role_percent_one_core': profile['navigation_cpu_by_role_percent_one_core'],
        'other_cpu_percent_one_core': profile['navigation_other_cpu_percent_one_core'],
        'passed': navigation['passed'], 'home_error_m': navigation['home_error_m'],
        'planner_failures': navigation['planner_failures'], 'recovery_behaviors': navigation['recovery_behaviors'],
        'costmap_clears': navigation['costmap_clears']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidates', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    baseline = inspect(args.baseline)
    candidates = [inspect(path) for path in args.candidates]
    for candidate in candidates:
        for key in ('target_id', 'source_digest', 'input_hashes', 'physics_hz'):
            assert candidate[key] == baseline[key], f'Comparison changed {key}: {candidate["run"]}'
        candidate['rtf_improvement_percent'] = 100*(candidate['rtf']/baseline['rtf']-1)
        candidate['navigation_wall_reduction_percent'] = 100*(1-candidate['navigation_wall_s']/baseline['navigation_wall_s'])
        candidate['physics_step_reduction_percent'] = 100*(1-candidate['physics_mean_ms']/baseline['physics_mean_ms'])
    args.output.write_text(json.dumps({'baseline': baseline, 'candidates': candidates,
        'limits': 'Same route, robot, controllers and navigation configuration; independently repeated closed-loop trajectories. Shared host load is measured but not controlled. Native-capture windows excluded from loop statistics.'}, indent=2)+'\n')
