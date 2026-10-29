"""Repeat isolated arm-return timing and collision checks without actuation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback


def interpolation_limits(trajectory: dict, joint_limits: dict, defaults: dict,
                         position_limits: dict | None = None) -> dict:
    """Check continuous quintic extrema using polynomial roots, not samples."""
    from numpy.polynomial import Polynomial

    ratios = {'velocity': 0., 'acceleration': 0.}
    points = trajectory['points']
    for first, last in zip(points, points[1:]):
        stamp = lambda p: p['time_from_start']['sec'] + p['time_from_start']['nanosec']/1e9
        dt = stamp(last)-stamp(first)
        assert dt > 0.
        for j, name in enumerate(trajectory['joint_names']):
            c0, c1, c2 = first['positions'][j], first['velocities'][j]*dt, first['accelerations'][j]*dt*dt/2
            dq = last['positions'][j]-c0-c1-c2
            dv = last['velocities'][j]*dt-c1-2*c2
            da = last['accelerations'][j]*dt*dt-2*c2
            position = Polynomial([c0,c1,c2,10*dq-4*dv+da/2,-15*dq+7*dv-da,6*dq-3*dv+da/2])
            if position_limits and name in position_limits:
                roots = position.deriv().roots()
                candidates = [0., 1., *(float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0. < r.real < 1.)]
                lower, upper = position_limits[name]
                assert all(lower-1e-10 <= position(u) <= upper+1e-10 for u in candidates), (name, 'position')
            for order, kind in [(1, 'velocity'), (2, 'acceleration')]:
                polynomial = position.deriv(order)/dt**order
                roots = polynomial.deriv().roots()
                candidates = [0., 1., *(float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0. < r.real < 1.)]
                settings = joint_limits[name]
                limit = settings['max_'+kind] if settings.get('has_'+kind+'_limits') else defaults['retiming_default_'+kind+'_limit']
                ratio = max(abs(polynomial(u)) for u in candidates)/limit
                assert ratio <= 1.+1e-6, (name, kind, ratio)
                ratios[kind] = max(ratios[kind], float(ratio))
    return ratios


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=12)
    parser.add_argument('--sides', nargs='+', choices=('left', 'right'), default=['left', 'right'])
    parser.add_argument('--test-directory', type=Path, default=Path(__file__).resolve().parents[2] /
                        'ros2_ws/src/cleany_moveit_config/test')
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    sys.path.insert(0, str(args.test_directory))
    import test_state_validity_batch_runtime as case
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ament_index_python.packages import get_package_share_directory
    import yaml

    configuration = Path(get_package_share_directory('cleany_moveit_config'))/'config'
    limits = yaml.safe_load((configuration/'joint_limits.yaml').read_text())['joint_limits']
    defaults = yaml.safe_load((configuration/'ompl_planning.yaml').read_text())

    args.output.mkdir(parents=True, exist_ok=True)
    original = case.call
    responses: list[dict] = []

    def capture(node, process, client, request, **kwargs):
        result = original(node, process, client, request, **kwargs)
        if client.srv_name == '/plan_kinematic_path':
            responses.append(message_to_ordereddict(result))
        return result

    rows = []
    case.call = capture
    try:
        for side in args.sides:
            for index in range(args.repeats):
                folder = args.output / f'{side}-{index:02d}'
                folder.mkdir(exist_ok=True)
                responses.clear()
                error = None
                ratios = None
                try:
                    case.test_return_planner_loads_fast_profile_and_rejects_a_blocked_scene(folder, side)
                    log = (folder/'moveit.log').read_text()
                    assert "Using planning request adapter 'Corrected Time Optimal Parameterization'" in log
                    assert 'Negative path velocity' not in log
                    assert 'Unable to parameterize trajectory' not in log
                    assert 'stop-at-corner' not in log
                    ratios = interpolation_limits(responses[0]['motion_plan_response']['trajectory']['joint_trajectory'], limits, defaults)
                except AssertionError:
                    error = traceback.format_exc()
                record = {'side': side, 'index': index, 'passed': error is None,
                          'error': error, 'responses': responses.copy(), 'interpolation_limit_ratios': ratios}
                metrics = folder / f'{side}-return-planning.json'
                if metrics.exists():
                    record['metrics'] = json.loads(metrics.read_text())
                rows.append(record)
                (args.output/'results.json').write_text(json.dumps(rows, indent=2)+'\n')
                print(side, index, 'PASS' if error is None else 'FAIL', flush=True)
    finally:
        case.call = original
    print(json.dumps({'tests': len(rows), 'passed': sum(r['passed'] for r in rows)}))
    return 0 if all(r['passed'] for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
