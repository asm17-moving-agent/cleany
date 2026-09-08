from types import SimpleNamespace

from action_msgs.msg import GoalInfo, GoalStatus
import pytest

from cleany_skill_executor.controller_stop import ControllerStop


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_cancels_only_selected_controller_specific_uuid_and_confirms_terminal(arm):
    stop = object.__new__(ControllerStop)
    info = GoalInfo()
    info.goal_id.uuid = [1]*16
    status = GoalStatus(goal_info=info, status=GoalStatus.STATUS_EXECUTING)
    stop.statuses = {arm: [status]}
    requests = []
    def cancel(request):
        requests.append(request)
        status.status = GoalStatus.STATUS_CANCELED
        return SimpleNamespace(return_code=0)
    stop.clients = {arm: SimpleNamespace(call_async=cancel)}
    stop.node = SimpleNamespace(_future=lambda future, *_: future,
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    stop.stop(arm, SimpleNamespace(done=lambda: False))
    assert len(requests) == 1
    assert bytes(requests[0].goal_info.goal_id.uuid) == bytes(info.goal_id.uuid)


def test_rejects_wildcard_goal_id_instead_of_stopping_unrelated_goals():
    stop = object.__new__(ControllerStop)
    stop.node = object()
    stop.statuses = {'left': [GoalStatus(status=GoalStatus.STATUS_EXECUTING)]}
    with pytest.raises(RuntimeError, match='wildcard'):
        stop.stop('left', SimpleNamespace(done=lambda: False))


def test_cancel_ack_alone_is_not_terminal_confirmation(monkeypatch):
    stop = object.__new__(ControllerStop)
    info = GoalInfo()
    info.goal_id.uuid = [2]*16
    stop.statuses = {'left': [GoalStatus(goal_info=info, status=GoalStatus.STATUS_EXECUTING)]}
    stop.clients = {'left': SimpleNamespace(call_async=lambda _: SimpleNamespace(return_code=0))}
    stop.node = SimpleNamespace(_future=lambda future, *_: future,
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    times = iter([0., 0., 4.])
    monkeypatch.setattr('cleany_skill_executor.controller_stop.time.monotonic', lambda: next(times))
    with pytest.raises(RuntimeError, match='did not confirm terminal'):
        stop.stop('left', SimpleNamespace(done=lambda: False))
