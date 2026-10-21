import json
import os

from cleany_skill_executor.core.command_progress import CommandProgress


def test_command_history_and_execution_identity_survive_fast_phases(tmp_path, monkeypatch):
    path = tmp_path / 'request.json'
    progress = CommandProgress(str(path), 'one', os.getpid())
    progress.update('RUNTIME_PREPARING')
    progress.update('INSPECTING_SCENE')
    progress.update('SELECTING_TARGET')
    data = json.loads(path.read_text())
    assert [item['phase'] for item in data['history']] == ['RUNTIME_PREPARING', 'INSPECTING_SCENE', 'SELECTING_TARGET']
    assert data['owner_start'] and data['state'] == 'ACTIVE'
    monkeypatch.setenv('CLEANY_MANIPULATION_PROGRESS_FILE', str(path))
    assert CommandProgress.from_environment('other') is None
    assert CommandProgress.from_environment('one').execution_id == 'one'
    progress.update('COMMAND_FAILED', 'specific recognition error', state='FAILED')
    progress.update('COMMAND_FAILED', 'parent exit code', state='FAILED')
    assert json.loads(path.read_text())['message'] == 'specific recognition error'
    CommandProgress(str(path), 'two', os.getpid()).update('RUNTIME_PREPARING')
    assert len(json.loads(path.read_text())['history']) == 1


def test_unwritable_observation_never_raises_into_robot_control(tmp_path):
    blocker = tmp_path / 'not-a-directory'
    blocker.write_text('')
    CommandProgress(str(blocker / 'request.json'), 'one', os.getpid()).update('INSPECTING_SCENE')
