from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('message,field', [
    ('Planning algorithm GridBased failed to generate a valid path to (-1.75, -0.55)', 'planner_failures'),
    ('GridBased: failed to create plan', 'planner_failures'),
    ('Optimizer fail to compute path', 'controller_failures'),
    ('MPPI: no collision-free emitted rollout', 'controller_failures'),
    ('Controller patience exceeded', 'controller_failures'),
    ('Failed to make progress', 'controller_failures'),
    ('Path progress timeout: 12.00s without 0.080m route advance', 'path_progress_timeouts'),
    ('Running spin', 'recovery_behaviors'),
    ('Received request to clear entirely the local_costmap', 'costmap_clears'),
    ('Continuous-footprint lattice refinement: 0 poses', 'continuous_footprint_refinement_failures'),
])
def test_actual_nav2_failure_messages_fail_no_recovery_gate(monkeypatch, message, field):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from navigation_log import FAILURE_COUNTS, summarize_navigation_log
    result = summarize_navigation_log('[server] [WARN] '+message, require_no_recovery=True)
    assert result[field] == 1
    assert any(result[name] for name in FAILURE_COUNTS)
    assert result['required_no_recovery'] is True


def test_normal_refinement_and_validated_standstill_are_separate_diagnostics(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from navigation_log import FAILURE_COUNTS, summarize_navigation_log
    result = summarize_navigation_log(
        'Continuous-footprint lattice refinement: 113 poses\n'
        'MPPI validated standstill: sampled rollouts rejected\nGoal succeeded',
        require_no_recovery=False)
    assert not any(result[name] for name in FAILURE_COUNTS)
    assert result['continuous_footprint_refinements'] == 1
    assert result['validated_standstill_cycles'] == 1
    assert result['required_no_recovery'] is False
