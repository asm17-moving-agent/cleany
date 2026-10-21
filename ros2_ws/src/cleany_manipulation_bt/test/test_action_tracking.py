"""An abort must cancel late acknowledgements without losing stop evidence."""
from concurrent.futures import Future
from threading import Event
from types import SimpleNamespace as NS

from action_msgs.msg import GoalStatus
from cleany_manipulation_bt.action_tracking import TrackedClient, quiescent


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
