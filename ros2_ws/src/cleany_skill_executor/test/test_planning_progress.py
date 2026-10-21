import pytest

from cleany_skill_executor.core.planning_progress import PlanningProgress


def test_nested_progress_reports_active_phase_and_restores_parent():
    now = [10.]
    progress = PlanningProgress(lambda: now[0])
    messages = []
    progress.callback = messages.append
    with progress.stage('BATCH', 'targets=3'):
        now[0] = 12.
        with progress.stage('MOVEIT_COLLISION', 'checked=0'):
            now[0] = 15.
            progress.update_detail('checked=2/4')
            assert 'phase=MOVEIT_COLLISION checked=2/4 phase_elapsed=3.0s' == progress.current_message()
        assert progress.current_message() == 'phase=BATCH targets=3 phase_elapsed=5.0s'
    assert progress.current_message() == ''
    assert [r['wall_sec'] for r in progress.records] == [3., 5.]
    assert 'state=done wall_sec=3.000 checked=2/4' in messages[2]


def test_failure_clears_active_phase_and_diagnostics_are_bounded():
    progress = PlanningProgress()
    with pytest.raises(InterruptedError):
        with progress.stage('GPU_IK'):
            raise InterruptedError('deadline')
    assert progress.current_message() == ''
    assert progress.records[-1]['state'] == 'failed'
    for _ in range(130):
        with progress.stage('GPU_IK'):
            pass
    assert len(progress.records) == 128
    progress.reset()
    assert not progress.records
