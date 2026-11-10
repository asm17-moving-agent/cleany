"""An abort must cancel late acknowledgements without losing stop evidence."""
from concurrent.futures import Future
from threading import Event
import pytest
from types import SimpleNamespace as NS
from action_msgs.msg import GoalStatus
from cleany_manipulation_bt.action_tracking import MotionClientPool, TrackedClient, quiescent


def test_late_goal_is_canceled_and_client_lives_until_terminal_result():
    acknowledgement, terminal = Future(), Future()
    abort = Event()
    owner = NS(submissions=[], handles=[], execution_context=NS(abort=abort),
               check_execution=lambda: None)
    owner.quiescent = lambda: quiescent(owner.submissions, owner.handles)
    canceled, destroyed = [], []
    handle = NS(accepted=True, get_result_async=lambda: terminal,
                cancel_goal_async=lambda: canceled.append(True))
    client = TrackedClient(owner, NS(send_goal_async=lambda goal: acknowledgement,
                                   destroy=lambda: destroyed.append(True)))
    client.send_goal_async(object())
    abort.set()
    client.destroy()
    assert not destroyed and not owner.quiescent()
    acknowledgement.set_result(handle)
    assert canceled == [True]
    assert not destroyed and not owner.quiescent()
    terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED))
    assert owner.quiescent() and destroyed == [True]


def test_lost_acknowledgement_cannot_be_reported_as_quiescent():
    acknowledgement = Future()
    acknowledgement.set_exception(RuntimeError('transport lost'))
    assert not quiescent([acknowledgement], [])


def test_completed_client_closes_while_other_controller_is_still_running():
    owner = NS(submissions=[], handles=[], execution_context=None, check_execution=lambda: None)
    owner.quiescent = lambda: quiescent(owner.submissions, owner.handles)
    results, clients, destroyed = [], [], []
    for index in range(2):
        acknowledgement, terminal = Future(), Future()
        handle = NS(accepted=True, get_result_async=lambda result=terminal: result)
        client = TrackedClient(owner, NS(send_goal_async=lambda goal, ack=acknowledgement: ack,
            destroy=lambda i=index: destroyed.append(i)))
        client.send_goal_async(object())
        acknowledgement.set_result(handle)
        client.destroy()
        clients.append(client)
        results.append(terminal)
    assert destroyed == []
    results[0].set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    assert destroyed == [0] and not owner.quiescent()
    clients[0].reset_completed()
    clients[1].reset_completed()
    assert not clients[0].handles and clients[1].handles
    results[1].set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    assert destroyed == [0, 1] and owner.quiescent()


def pooled_fixture():
    owner = NS(submissions=[], handles=[], execution_context=NS(abort=Event()),
               check_execution=lambda: None)
    created, destroyed, goals = [], [], []

    def factory(kind, name):
        def send(goal, **_kwargs):
            ack, terminal = Future(), Future()
            canceled = []
            handle = NS(accepted=True, get_result_async=lambda: terminal,
                        cancel_goal_async=lambda: canceled.append(True))
            goals.append((ack, terminal, handle, canceled))
            return ack
        raw = NS(send_goal_async=send, destroy=lambda: destroyed.append(name))
        created.append((kind, name, raw))
        return raw
    return owner, MotionClientPool(owner, factory), created, destroyed, goals


def test_pool_reuses_only_transport_and_closes_it_once_at_shutdown():
    owner, pool, created, destroyed, goals = pooled_fixture()
    first = pool.acquire(object, '/arm')
    first.send_goal_async(object())
    ack, terminal, handle, _ = goals[-1]
    ack.set_result(handle)
    terminal.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    first.destroy()
    owner.execution_context = NS(abort=Event())
    second = pool.acquire(object, '/arm')
    assert first is not second and first.client is second.client
    assert second.transport_reused and not first.transport_reused
    assert second.submissions == second.handles == []
    assert len(created) == 1 and not destroyed and quiescent(owner.submissions, owner.handles)
    with pytest.raises(RuntimeError, match='lease is closed'):
        first.send_goal_async(object())
    second.destroy()
    pool.close()
    pool.close()
    assert destroyed == ['/arm']
    with pytest.raises(RuntimeError, match='pool is closed'):
        pool.acquire(object, '/arm')


@pytest.mark.parametrize('phase', ['unsubmitted', 'pending_ack', 'accepted', 'ack_error'])
def test_pool_refuses_overlapping_or_unresolved_leases(phase):
    _, pool, _, destroyed, goals = pooled_fixture()
    first = pool.acquire(object, '/arm')
    if phase != 'unsubmitted':
        first.send_goal_async(object())
        ack, _, handle, _ = goals[-1]
        if phase == 'accepted':
            ack.set_result(handle)
        if phase == 'ack_error':
            ack.set_exception(RuntimeError('lost acknowledgement'))
        first.destroy()
    with pytest.raises(RuntimeError, match='not terminal'):
        pool.acquire(object, '/arm')
    assert destroyed == []
    other = pool.acquire(object, '/gripper')
    other.destroy()
    pool.close()
    assert sorted(destroyed) == ['/arm', '/gripper']


def test_pool_preserves_late_ack_cancellation_from_original_execution():
    owner, pool, _, destroyed, goals = pooled_fixture()
    first = pool.acquire(object, '/arm')
    first.send_goal_async(object())
    original_abort = owner.execution_context.abort
    original_abort.set()
    owner.execution_context = NS(abort=Event())
    first.destroy()
    ack, terminal, handle, canceled = goals[-1]
    ack.set_result(handle)
    assert canceled == [True] and not destroyed
    with pytest.raises(RuntimeError, match='not terminal'):
        pool.acquire(object, '/arm')
    terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED))
    second = pool.acquire(object, '/arm')
    second.destroy()
    pool.close()
    assert destroyed == ['/arm']


def test_rejected_goal_can_release_pool_without_recreating_transport():
    _, pool, created, destroyed, goals = pooled_fixture()
    first = pool.acquire(object, '/arm')
    first.send_goal_async(object())
    goals[-1][0].set_result(NS(accepted=False))
    first.destroy()
    second = pool.acquire(object, '/arm')
    assert len(created) == 1 and second.transport_reused and not destroyed
    second.destroy()
    pool.close()
