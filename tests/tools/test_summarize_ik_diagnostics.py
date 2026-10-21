from copy import deepcopy
import json

import pytest

from summarize_ik_diagnostics import read_diagnostics, summarize


def record(backend='gpu_primary', *, approach=42., outcome='pose_rejected'):
    return {
        'candidate_index': 17, 'arm': 'right', 'stage': 'PREGRASP_IK',
        'backend': backend, 'outcome': outcome,
        'target': {'position_m': [.4, -.2, .3], 'orientation_xyzw': [0., 0., 0., 1.]},
        'gripper_joint_position_rad': .4,
        'limits': {'position_error_mm': 10., 'approach_error_degrees': 30.,
                   'closing_error_degrees': 30.},
        'closest_alignment': {'position_error_mm': 1., 'approach_error_degrees': approach,
                              'closing_error_degrees': 0., 'violations': ['approach_error_degrees']},
    }


def test_prefixed_logs_preserve_best_gpu_pose_when_cpu_returns_no_solution():
    first = record()
    second = record('cpu_fallback', outcome='no_ik_solution')
    second['closest_alignment'] = None
    rows = ['phase=GPU_IK state=done\n', '[node] ik_diagnostic=' + json.dumps(first),
            '[node] ik_diagnostic=' + json.dumps(second)]
    result = summarize(read_diagnostics(rows))
    assert result['diagnostic_records'] == 2
    assert result['candidate_outcomes'] == {'no_ik_solution': 1}
    candidate = result['candidates'][0]
    assert candidate['best_recorded_alignment']['backend'] == 'gpu_primary'
    assert candidate['best_recorded_alignment']['approach_error_degrees'] == 42.


def test_repeated_candidate_index_with_different_target_is_not_merged():
    first = record()
    second = deepcopy(first)
    second['target']['position_m'][0] = .5
    assert len(summarize([first, second])['candidates']) == 2


def test_repeated_selector_goals_are_not_merged_even_with_identical_targets():
    first, second = record(), record()
    first['selection_id'], second['selection_id'] = 'first', 'second'
    assert len(summarize([first, second])['candidates']) == 2


def test_final_fk_mismatch_is_preserved_alongside_proposal_errors():
    first = record(outcome='final_fk_rejected')
    first['final_alignment'] = {'position_error_mm': 80., 'approach_error_degrees': 90.}
    candidate = summarize([first])['candidates'][0]
    assert candidate['last_final_alignment']['position_error_mm'] == 80.


def test_old_logs_report_missing_evidence_and_malformed_records_fail_clearly():
    assert not summarize(read_diagnostics(['no IK solution']))['available']
    with pytest.raises(ValueError, match='line 1'):
        read_diagnostics(['ik_diagnostic={broken'])
