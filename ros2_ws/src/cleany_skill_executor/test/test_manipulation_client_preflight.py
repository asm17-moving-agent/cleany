from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from cleany_skill_executor import manipulation_client as module


@pytest.mark.parametrize('inhibited', [True, False])
def test_status_query_never_confirms_or_clears_recovery(inhibited):
    requests, destroyed = [], []
    def call(request):
        requests.append(request)
        future = Future()
        future.set_result(SimpleNamespace(success=True, inhibited=inhibited, message='status'))
        return future
    client = SimpleNamespace(wait_for_service=lambda **kw: True, call_async=call)
    node = SimpleNamespace(create_client=lambda *a: client, destroy_client=destroyed.append)
    if inhibited:
        with pytest.raises(RuntimeError, match='inhibited'):
            module.require_execution_available(node)
    else:
        module.require_execution_available(node)
    assert len(requests) == 1
    assert not requests[0].robot_stopped_confirmed and not requests[0].gripper_empty_confirmed
    assert destroyed == [client]


def test_blocked_auto_select_never_starts_inspection(monkeypatch):
    actions = []
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: None)
    node = SimpleNamespace(declare_parameter=lambda *a: None,
        create_client=lambda *a: SimpleNamespace(), destroy_node=lambda: None)
    monkeypatch.setattr(module, 'Node', lambda _: node)
    def action(node, kind, name):
        actions.append(name)
        return SimpleNamespace(wait_for_server=lambda **kw: True, destroy=lambda: None)
    monkeypatch.setattr(module, 'ActionClient', action)
    def blocked(*_):
        raise RuntimeError('inhibited')
    monkeypatch.setattr(module, 'require_execution_available', blocked)
    with pytest.raises(RuntimeError, match='inhibited'):
        module.main(['--namespace', '/', '--auto-select', '--collection-config', '/unused'])
    assert actions == ['/manipulation/execute_skill']


@pytest.mark.parametrize('inspection_success', [True, False])
def test_initial_inspection_is_reported_before_the_manipulation_goal(tmp_path, monkeypatch, inspection_success):
    import json
    import os
    from cleany_skill_executor.core.command_progress import CommandProgress

    path, execution_id = tmp_path / 'request.json', 'same-command-and-action'
    CommandProgress(str(path), execution_id, os.getpid()).update('RUNTIME_PREPARING')
    monkeypatch.setenv('CLEANY_MANIPULATION_PROGRESS_FILE', str(path))
    config = tmp_path / 'collection.json'
    config.write_text('{}')
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(module, 'require_execution_available', lambda *args: None)
    def future(value):
        result = Future()
        result.set_result(value)
        return result
    result = SimpleNamespace(status='SUCCESS', stop_confirmed=True,
                             get_fields_and_field_types=lambda: ['status', 'stop_confirmed'])
    record = SimpleNamespace(execution_id=execution_id, get_fields_and_field_types=lambda: ['execution_id'])
    service = SimpleNamespace(wait_for_service=lambda **kw: True,
        call_async=lambda request: future(SimpleNamespace(found=True, record=record)))
    node = SimpleNamespace(declare_parameter=lambda *a: None, create_client=lambda *a: service,
        get_parameter=lambda name: SimpleNamespace(value=[] if name == 'bt_denylist' else .1),
        destroy_node=lambda: None)
    monkeypatch.setattr(module, 'Node', lambda _: node)
    sent = []
    def action(node, kind, name):
        def send(goal, **kwargs):
            phase = json.loads(path.read_text())['phase']
            if name == '/perception/inspect_scene':
                assert phase == 'INSPECTING_SCENE'
                inspected = SimpleNamespace(success=inspection_success, detections=object(), message='recognition failure')
                return future(SimpleNamespace(accepted=True, get_result_async=lambda: future(SimpleNamespace(result=inspected))))
            assert phase == 'SUBMITTING_GOAL' and goal.execution_id == execution_id
            sent.append(goal)
            return future(SimpleNamespace(accepted=True, get_result_async=lambda: future(SimpleNamespace(result=result))))
        return SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=send, destroy=lambda: None)
    monkeypatch.setattr(module, 'ActionClient', action)
    target = SimpleNamespace(snapshot_id='observed', object_id=2, skill_name='collect_trash', destination_id='return_to_pick')
    monkeypatch.setattr(module, 'select_automatic_target', lambda *args, **kw: target)
    args = ['--namespace', '/', '--auto-select', '--execution-id', execution_id, '--collection-config', str(config)]
    if inspection_success:
        assert module.main(args) == 0
    else:
        with pytest.raises(RuntimeError, match='recognition failure'):
            module.main(args)
    data = json.loads(path.read_text())
    assert data['execution_id'] == execution_id
    phases = [item['phase'] for item in data['history']]
    assert phases.index('CHECKING_EXECUTION') < phases.index('INSPECTING_SCENE')
    assert bool(sent) == inspection_success
    assert data['state'] == ('FINISHED' if inspection_success else 'FAILED')
    if inspection_success:
        assert phases.index('SELECTING_TARGET') < phases.index('SUBMITTING_GOAL') < phases.index('EXECUTING')
