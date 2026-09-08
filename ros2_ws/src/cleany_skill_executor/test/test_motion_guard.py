from types import SimpleNamespace

import pytest

from cleany_skill_executor.motion_guard import enforce_motion_guard, guarded_action_result


def failing():
    raise RuntimeError('target lost')


def test_guard_fault_cancels_active_goal_and_waits_for_terminal_result(monkeypatch):
    events = []
    terminal = SimpleNamespace(done=lambda: False)
    def wait(future, timeout, label):
        events.append((future, label))
    node = SimpleNamespace(_check_motion_guard=failing, _future=wait,
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    handle = SimpleNamespace(cancel_goal_async=lambda: 'ack', get_result_async=lambda: terminal)
    monkeypatch.setattr('cleany_skill_executor.motion_guard.rclpy.spin_once', lambda *a, **k: None)
    with pytest.raises(RuntimeError, match='target lost'):
        guarded_action_result(node, handle, 60., 'transport')
    assert [event[0] for event in events] == ['ack', terminal]
    assert 'terminal result' in events[-1][1]


def test_terminal_timeout_does_not_claim_cancellation_completed():
    messages = []
    def wait(future, *_):
        if future == 'terminal':
            raise RuntimeError('terminal result timed out')
    node = SimpleNamespace(_check_motion_guard=failing, _future=wait,
        get_logger=lambda: SimpleNamespace(error=messages.append))
    handle = SimpleNamespace(cancel_goal_async=lambda: 'ack')
    with pytest.raises(RuntimeError, match='terminal result timed out'):
        enforce_motion_guard(node, handle, 'terminal', 'lift')
    assert len(messages) == 1 and 'cancel requested' in messages[0]


def test_guard_blocks_dispatch_before_a_goal_exists():
    with pytest.raises(RuntimeError, match='target lost'):
        enforce_motion_guard(SimpleNamespace(_check_motion_guard=failing))


def test_controller_stop_is_sent_before_waiting_for_blocked_moveit_cancel():
    events = []
    node = SimpleNamespace(_check_motion_guard=failing,
        _stop_guarded_controller=lambda future: events.append(('controller', future)),
        _future=lambda future, *_: events.append(('wait', future)),
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    handle = SimpleNamespace(cancel_goal_async=lambda: 'ack')
    with pytest.raises(RuntimeError, match='target lost'):
        enforce_motion_guard(node, handle, 'terminal')
    assert events == [('controller', 'terminal'), ('wait', 'ack'), ('wait', 'terminal')]


def test_success_and_legacy_actions_preserve_result():
    future = SimpleNamespace(done=lambda: True, result=lambda: 'result')
    handle = SimpleNamespace(get_result_async=lambda: future)
    assert guarded_action_result(SimpleNamespace(_check_motion_guard=lambda: None),
                                 handle, 60., 'return') == 'result'
    assert guarded_action_result(SimpleNamespace(_future=lambda f, *_: f.result()),
                                 handle, 60., 'legacy') == 'result'
