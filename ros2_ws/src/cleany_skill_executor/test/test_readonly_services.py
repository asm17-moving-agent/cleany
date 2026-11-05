"""Independent read batches retain fresh reads, owner guards and failure cleanup."""
from types import SimpleNamespace as NS

import pytest
from rclpy.task import Future

from cleany_skill_executor.core.hardware_execution import ExecutionCanceled
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.readonly_services import ReadOnlyBatch, call_readonly_services


def batch_fixture():
    pending, removed, waits = [], [], []
    clients = []
    for index in range(4):
        def send(request, index=index):
            future = Future()
            pending.append((index, future))
            return future
        clients.append(NS(srv_name=f'/read_{index}', wait_for_service=lambda **_: True,
            call_async=send, remove_pending_request=lambda f: removed.append(f)))

    def wait(future, timeout):
        assert len(pending) == 4  # Every read precedes the first response wait.
        waits.append(timeout)
        if not future.done():
            for index, f in pending:
                f.set_result(NS(value=index))
        return future.result()

    node = NS(wait=wait, check_execution=lambda: None)
    return node, clients, pending, removed, waits


def test_readonly_batch_dispatches_first_and_uses_one_response_deadline():
    node, clients, pending, removed, waits = batch_fixture()
    responses = call_readonly_services(node, [(c, object()) for c in clients], timeout=1.)
    assert [response.value for response in responses] == list(range(4))
    assert all(0 < value <= 1. for value in waits)
    assert waits == sorted(waits, reverse=True)
    assert removed == [f for _, f in pending]


@pytest.mark.parametrize('failure', ['unavailable', 'dispatch', 'timeout', 'cancel', 'empty', 'negative', 'exception'])
def test_readonly_batch_failure_discards_responses_and_cleans_pending(failure):
    node, clients, pending, removed, _ = batch_fixture()
    if failure == 'unavailable':
        clients[2].wait_for_service = lambda **_: False
    elif failure == 'dispatch':
        def send(_):
            raise RuntimeError('send failed')
        clients[2].call_async = send
    else:
        def wait(future, timeout):
            if failure == 'cancel':
                raise ExecutionCanceled('stopped')
            if failure == 'timeout':
                future.cancel()  # The owner's wait may cancel before raising.
                raise RuntimeError('timeout')
            if failure == 'exception':
                future.set_exception(RuntimeError('remote failure'))
                return future.result()
            return None if failure == 'empty' else NS(success=False, message='failed')
        node.wait = wait
    with pytest.raises(RuntimeError):
        call_readonly_services(node, [(c, object()) for c in clients])
    assert all(f.done() or f.cancelled() for _, f in pending)
    assert removed == [f for _, f in pending]
    if failure == 'unavailable':
        assert not pending


def test_readonly_batch_checks_owner_between_submissions():
    node, clients, pending, removed, _ = batch_fixture()
    def check():
        if len(pending) == 2:
            raise ExecutionCanceled('stopped before third request')
    node.check_execution = check
    with pytest.raises(ExecutionCanceled):
        call_readonly_services(node, [(c, object()) for c in clients])
    assert len(pending) == len(removed) == 2


@pytest.mark.parametrize('invalid', ['', 'hardware', 'controller', 'model', 'execution', 'description'])
def test_parallel_model_verification_keeps_every_live_contract(monkeypatch, invalid):
    from cleany_skill_executor import hardware_execution as module
    node, clients, pending, _, _ = batch_fixture()
    monkeypatch.setattr(module, 'physical_limits', lambda *_: {'limit': 'fresh'})
    xml = '<robot name="same"/>'
    responses = [
        NS(component=[NS(class_type='cleany_feetech_hardware/FeetechSystem', state=NS(label='active'))]),
        NS(controller=[NS(name='right_'+part+'_controller', state='active') for part in ('arm', 'gripper')]),
        NS(values=[NS(string_value=xml)]),
        NS(values=[NS(string_value=xml), NS(bool_value=False)]),
    ]
    if invalid == 'hardware': responses[0].component[0].state.label = 'inactive'
    if invalid == 'controller': responses[1].controller = []
    if invalid == 'model': responses[3].values[0].string_value = '<robot name="different"/>'
    if invalid == 'execution': responses[3].values[1].bool_value = True
    if invalid == 'description': responses[2].values = []
    def wait(future, _):
        for index, f in pending:
            if not f.done(): f.set_result(responses[index])
        return future.result()
    node.wait = wait
    node.get_parameter = lambda _: NS(value=False)
    node.motion = NS(set_robot_description=lambda _: None)
    hardware = NS(node=node, parallel_model_queries=True, monitor=lambda: None,
                  hardware=clients[0], controllers=clients[1], parameters=clients[2], moveit_parameters=clients[3])
    if invalid:
        with pytest.raises(RuntimeError): HardwareExecution.verify(hardware, 'right')
    else:
        assert HardwareExecution.verify(hardware, 'right') == {'limit': 'fresh'}
        assert HardwareExecution.verify(hardware, 'right') == {'limit': 'fresh'}
        assert len(pending) == 8 and hardware._last_model_cache_hit


def test_pending_batch_can_be_closed_without_waiting():
    node, clients, pending, removed, waits = batch_fixture()
    batch = ReadOnlyBatch(node, [(client, object()) for client in clients])
    batch.close()
    batch.close()
    assert len(removed) == 4 and not waits
    assert all(future.cancelled() for _, future in pending)
    with pytest.raises(RuntimeError, match='consumed or closed'):
        batch.result()
