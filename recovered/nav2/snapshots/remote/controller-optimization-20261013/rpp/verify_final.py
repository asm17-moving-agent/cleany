#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

data = json.loads((ROOT / 'results.json').read_text())
checks = []
for phase in ('conservative', 'relaxed'):
    rows = data['phases'][phase]
    assert len(rows) == 5 and {r['goal_index'] for r in rows} == set(range(5))
    for r in rows:
        directory = WORKSPACE / r['run_directory']
        assert r['completed'] and r['measurement_completed']
        assert r['final_command_wheel_verified'] and r['actual_wheel_verified'] and r['final_stop_verified']
        assert r['actual_wheel_feedback_rad_s']['max'] <= 5.0 + 1e-6
        assert r['max_command_wheel_rad_s']['/cmd_vel'] <= 5.0 + 1e-6
        assert r['verified_planner']['GridBased.plugin'] == 'cleany_navigation::StraightFirstPlanner'
        assert r['footprint_geometry']['navigation']['projected_overlap_sample_count'] == 0
        assert (directory / 'bag').is_dir() and list((directory / 'bag').glob('*.db3'))
        assert (directory / 'compute_timing.csv').stat().st_size > 0
        if phase == 'conservative':
            assert r['final_command_box_verified']
            assert r['verified_yaw_limit']['FollowPath.tracking_yaw_limit'] == .14
            assert r['dynamic_limit_change_test']['rejected']
        else:
            assert r['action_status'] == 4
            parameters = r['verified_motion_parameters']
            assert parameters['FollowPath.use_collision_detection']
            assert parameters['goal_checker.xy_goal_tolerance'] == .15
            assert r['dynamic_wheel_cap_change_test']['rejected']
            assert r['actual_contact_count'] == 0
            contacts = r['physics_contacts']
            assert contacts['floor_contact_positive_control_observed']
            assert contacts['unclassified_entries'] == 0
        checks.append({'phase': phase, 'goal_index': r['goal_index'], 'arrived': r['action_status'] == 4, 'actual_wheel_max_rad_s': r['actual_wheel_feedback_rad_s']['max'], 'stopped': True})
frozen = {}
for name in ('runner.py', 'common.py', 'probe.py', 'wheel_limit.py'):
    hashes = [sha(WORKSPACE / r['run_directory'] / 'experiment_source' / name) for r in data['phases']['relaxed']]
    assert len(set(hashes)) == 1
    assert hashes[0] == sha(ROOT / 'relaxed_sol61_20261013' / name)
    frozen[name] = hashes[0]
parameters = [r['verified_motion_parameters'] for r in data['phases']['relaxed']]
assert all(p == parameters[0] for p in parameters)
cleanup = json.loads((ROOT / 'cleanup.json').read_text())
assert cleanup['all_new_experiment_processes_exited']
result = {'completed': True, 'final_trials': checks, 'frozen_relaxed_source_sha256': frozen, 'relaxed_parameters_identical_across_five': True, 'cleanup_verified': True, 'limits': 'Recorded samples only; one run per goal; no statistical guarantee.', 'cleanup_incident': cleanup['legacy_frictions_child_incident']}
(ROOT / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')
status = json.loads((ROOT / 'STATUS.json').read_text())
status.update(phase='complete', pending=[], conservative_final_completed=True, conservative_arrivals=3, conservative_trials=5, relaxed_final_completed=True, relaxed_arrivals=5, relaxed_trials=5, cleanup_verified=True, report=str(ROOT / 'REPORT.md'))
(ROOT / 'STATUS.json').write_text(json.dumps(status, indent=2) + '\n')
print('Verified ten final trials, frozen relaxed source and parameters, wheel limits, stops, contacts, and cleanup.')
