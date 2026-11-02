"""Cancellation must account for an operation's eventual terminal observation."""
from dataclasses import dataclass, field
import threading
import time
from typing import Callable

import pytest

from cleany_manipulation_bt._bt_runner import Runner
from cleany_manipulation_bt.backend import Observation, OperationError, WorkerBackend
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.manipulation.models import (
    CancelMode, Error, Goal, ObjectState, Placement, Status,
)
from test_bt import FakeBackend, TREE, finish, hold_at


def goal(execution_id: str) -> Goal:
    return Goal('mission', 'task', execution_id, 'collect_trash', 'snapshot', 1, 'trash_right')


@dataclass
class DeferredOperation:
    entered: threading.Event = field(default_factory=threading.Event)
    release: threading.Event = field(default_factory=threading.Event)
    outcome: Observation | Exception = field(default_factory=Observation)


def tick_until(core: ExecutionCore, condition: Callable[[], bool]) -> None:
    deadline = time.monotonic() + 2.
    while not condition():
        assert time.monotonic() < deadline, 'BT did not reach the expected condition'
        core.tick()
        time.sleep(.001)


@pytest.fixture
def worker_execution(tmp_path):
    control = DeferredOperation()
    started = []
    def operation(node, context):
        started.append(node)
        if node == 'MoveToPregrasp':
            control.entered.set()
            assert control.release.wait(3.), 'Test did not release the deferred operation'
            if isinstance(control.outcome, Exception):
                raise control.outcome
            return control.outcome
        values = {
            'SelectArmAndPath': dict(selected_arm='right'),
            'StopAndAssess': dict(stop_confirmed=True),
            'ReleaseInPlace': dict(object_state=ObjectState.LEFT_GRIPPER),
            'ReturnArmAfterCancel': dict(arm_recovered=True),
            'StopAfterRecovery': dict(stop_confirmed=True),
        }
        return Observation(**values.get(node, {}))
    backend = WorkerBackend(operation)
    store = BTExecutionStore(str(tmp_path / 'worker.sqlite3'))
    core = ExecutionCore(backend, store, TREE, Runner, timeout_sec=2., stop_timeout_sec=1.)
    try:
        assert core.accept(goal('worker'))[0]
        yield core, backend, store, control, started
    finally:
        control.release.set()
        if core.tree is not None:
            core.tree.halt()
        backend.close()
        store.close()


@pytest.mark.parametrize('mode', [CancelMode.IMMEDIATE, CancelMode.RETURN_ARM])
@pytest.mark.parametrize('error', [
    Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR, Error.MOTION_FAILED,
    Error.CANCELED, Error.NONE,
])
def test_cancel_collects_late_worker_outcome(worker_execution, mode: CancelMode, error: Error) -> None:
    core, backend, store, control, started = worker_execution
    tick_until(core, control.entered.is_set)
    control.outcome = (Observation() if error == Error.NONE else
                       RuntimeError('Late worker exception') if error == Error.INTERNAL_ERROR else
                       OperationError(error, 'Late operation outcome'))
    assert core.request_cancel('worker', mode)
    assert backend.context.abort.is_set()
    core.tick()
    assert core.failure.error == Error.CANCELED and core.record.result is None
    control.release.set()
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    expected = (Status.FATAL if error in (Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR)
                else Status.FAILED if error == Error.MOTION_FAILED else Status.CANCELED)
    assert result.status == expected
    assert result.error_code == (Error.CANCELED if error == Error.NONE else error)
    assert result.stop_confirmed and result.failed_substage == 'MoveToPregrasp'
    assert store.get('worker').result == result
    recover = mode == CancelMode.RETURN_ARM and expected == Status.CANCELED
    assert ('ReleaseInPlace' in started) == recover
    assert ('ReturnArmAfterCancel' in started) == recover
    if expected == Status.FATAL:
        assert core.inhibited and not core.accept(goal('next'))[0]
    core.tick()
    assert core.record.result == result
    assert sum(event.result is not None for event in core.drain_events()) == 1


@pytest.fixture
def deferred_execution(tmp_path):
    store = BTExecutionStore(str(tmp_path / 'deferred.sqlite3'))
    backend = FakeBackend(store)
    backend.defer_cancel_completion = True
    now = [0.]
    core = ExecutionCore(backend, store, TREE, Runner, clock=lambda: now[0],
                         timeout_sec=2., stop_timeout_sec=1.)
    backend.store = core.store
    try:
        assert core.accept(goal('deferred'))[0]
        yield core, backend, store, now
    finally:
        core.tree.halt()
        store.close()


@pytest.mark.parametrize('mode', [CancelMode.IMMEDIATE, CancelMode.RETURN_ARM])
@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp'])
def test_stop_waits_for_canceled_operation_before_recovery(
        deferred_execution, mode: CancelMode, node: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    assert core.request_cancel('deferred', mode)
    assert backend.canceled
    for _ in range(5):
        core.tick()
        now[0] += .05
    assert core.record.result is None and core.record.substage == 'StopAndAssess'
    assert 'ReleaseInPlace' not in backend.started
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Late hardware fault')
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.HARDWARE_ERROR
    assert result.failed_substage == node and result.stop_confirmed
    assert 'ReleaseInPlace' not in backend.started and core.inhibited


@pytest.mark.parametrize('mode', [CancelMode.IMMEDIATE, CancelMode.RETURN_ARM])
@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp'])
def test_unfinished_canceled_operation_hits_stop_deadline(
        deferred_execution, mode: CancelMode, node: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    core.request_cancel('deferred', mode)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and now[0] < 2.
    assert node in result.message
    assert 'ReleaseInPlace' not in backend.started and not core.accept(goal('next'))[0]
    backend.hold.remove(node)
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Arrived after finalization')
    core.tick()
    assert core.record.result == result


@pytest.mark.parametrize('mode', [CancelMode.IMMEDIATE, CancelMode.RETURN_ARM])
@pytest.mark.parametrize('node', ['ConfirmRelease', 'ReturnArm'])
def test_late_success_preserves_physical_progress_and_next_goal(
        deferred_execution, mode: CancelMode, node: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    core.request_cancel('deferred', mode)
    core.tick()
    core.tick()
    assert core.record.result is None
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.CANCELED and result.error_code == Error.CANCELED
    assert result.object_state == ObjectState.LEFT_GRIPPER
    assert result.placement_state == Placement.CONFIRMED and result.stop_confirmed
    assert result.arm_recovered == (node == 'ReturnArm' or mode == CancelMode.RETURN_ARM)
    assert node in core.record.completed_substages
    assert not core.inhibited and core.accept(goal('next'))[0]
    assert finish(core, now).status == Status.SUCCESS
    assert core.get('deferred').result == result


@pytest.mark.parametrize('mode', [CancelMode.IMMEDIATE, CancelMode.RETURN_ARM])
def test_cancel_before_motion_requires_confirmed_stop(deferred_execution, mode: CancelMode) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, 'ValidateGoal')
    core.request_cancel('deferred', mode)
    core.tick()
    backend.hold.remove('ValidateGoal')
    backend.failures['ValidateGoal'] = Observation(False, Error.CANCELED, 'Canceled before motion')
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=False)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and core.inhibited


@pytest.mark.parametrize('node', ['ReleaseInPlace', 'ReturnArmAfterCancel'])
def test_immediate_cancel_collects_late_recovery_fault(deferred_execution, node: str) -> None:
    core, backend, _, now = deferred_execution
    backend.defer_cancel_completion = False
    hold_at(core, backend, 'CarryObject')
    backend.hold.add(node)
    core.request_cancel('deferred', CancelMode.RETURN_ARM)
    for _ in range(80):
        core.tick()
        if core.record.substage == node:
            break
    else:
        pytest.fail('Recovery node not reached')
    backend.defer_cancel_completion = True
    core.request_cancel('deferred', CancelMode.IMMEDIATE)
    core.tick()
    core.tick()
    backend.hold.remove(node)
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Late recovery fault')
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.HARDWARE_ERROR
    assert result.failed_substage == node and result.stop_confirmed
    assert 'StopAfterRecovery' not in backend.started
    assert backend.started.count('StopAndAssess') == 2 and core.inhibited


@pytest.mark.parametrize('error', [
    Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR, Error.CANCELED, Error.NONE,
])
def test_timeout_collects_late_worker_outcome(worker_execution, monkeypatch, error: Error) -> None:
    core, backend, store, control, started = worker_execution
    now = [0.]
    monkeypatch.setattr(core, 'clock', lambda: now[0])
    tick_until(core, control.entered.is_set)
    control.outcome = (Observation() if error == Error.NONE else
                       RuntimeError('Late worker exception') if error == Error.INTERNAL_ERROR else
                       OperationError(error, 'Late operation outcome'))
    now[0] = 2.
    core.tick()
    assert backend.context.abort.is_set()
    assert core.failure.error == Error.TIMEOUT and core.record.result is None
    control.release.set()
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    fatal = error in (Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR)
    assert result.status == (Status.FATAL if fatal else Status.FAILED)
    assert result.error_code == (error if fatal else Error.TIMEOUT)
    assert result.stop_confirmed and result.failed_substage == 'MoveToPregrasp'
    assert store.get('worker').result == result
    assert core.record.human_confirmation_required == fatal
    assert core.inhibited == fatal
    if fatal:
        assert 'Late' in result.message and not core.accept(goal('next'))[0]
    assert 'ApproachObject' not in started and 'ReleaseInPlace' not in started
    assert 'ReturnArmAfterCancel' not in started
    core.tick()
    assert core.record.result == result
    assert sum(event.result is not None for event in core.drain_events()) == 1


@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp'])
def test_timeout_waits_for_worker_after_early_stop_evidence(deferred_execution, node: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    now[0] = 2.
    core.tick()
    for _ in range(5):
        core.tick()
    assert backend.canceled and core.record.result is None
    assert core.record.substage == 'StopAndAssess'
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Late hardware fault')
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.HARDWARE_ERROR
    assert result.failed_substage == node and result.stop_confirmed
    assert core.inhibited and not core.accept(goal('next'))[0]
    assert 'ReleaseInPlace' not in backend.started


@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp'])
@pytest.mark.parametrize('pending', ['worker', 'stop'])
def test_timeout_shutdown_is_bounded_and_result_is_final(deferred_execution, node: str, pending: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    if pending == 'stop':
        backend.hold.add('StopAndAssess')
    now[0] = 2.
    core.tick()
    if pending == 'stop':
        backend.failures[node] = Observation(False, Error.CANCELED, 'Worker canceled')
        backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and now[0] < 3.5
    if pending == 'worker':
        assert node in result.message
    assert core.inhibited and not core.accept(goal('next'))[0]
    assert 'ReleaseInPlace' not in backend.started
    backend.hold.clear()
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Arrived after finalization')
    core.tick()
    assert core.record.result == result
    assert sum(event.result is not None for event in core.drain_events()) == 1


@pytest.mark.parametrize('node', ['ConfirmRelease', 'ReturnArm'])
def test_timeout_preserves_late_physical_progress_without_resuming(deferred_execution, node: str) -> None:
    core, backend, store, now = deferred_execution
    hold_at(core, backend, node)
    now[0] = 2.
    core.tick()
    core.tick()
    assert core.record.result is None
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FAILED and result.error_code == Error.TIMEOUT
    assert result.object_state == ObjectState.LEFT_GRIPPER
    assert result.placement_state == Placement.CONFIRMED and result.stop_confirmed
    assert result.arm_recovered == (node == 'ReturnArm')
    assert result.failed_substage == node and node in core.record.completed_substages
    next_node = 'ReturnArm' if node == 'ConfirmRelease' else 'VerifyPlacedObject'
    assert next_node not in backend.started and 'ReleaseInPlace' not in backend.started
    assert store.get('deferred').result == result
    assert not core.inhibited and core.accept(goal('next'))[0]
    assert finish(core, now).status == Status.SUCCESS
    assert core.get('deferred').result == result


@pytest.mark.parametrize('elapsed', [2., 2.1])
def test_late_success_before_timeout_cancellation_is_still_accepted(deferred_execution, elapsed: float) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, 'CarryObject')
    now[0] = elapsed
    backend.hold.remove('CarryObject')
    result = finish(core, now)
    assert result.status == Status.SUCCESS and result.error_code == Error.NONE
    assert not backend.canceled and 'OpenGripperAtDestination' in backend.started


@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp'])
def test_timeout_cancellation_requires_confirmed_stop(deferred_execution, node: str) -> None:
    core, backend, _, now = deferred_execution
    hold_at(core, backend, node)
    now[0] = 2.
    core.tick()
    backend.failures[node] = Observation(False, Error.CANCELED, 'Worker canceled')
    backend.hold.remove(node)
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=False)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and core.inhibited
    assert not core.accept(goal('next'))[0]


@pytest.mark.parametrize('node', ['ReleaseInPlace', 'ReturnArmAfterCancel'])
def test_timeout_collects_late_cancel_recovery_fault(deferred_execution, node: str) -> None:
    core, backend, _, now = deferred_execution
    backend.defer_cancel_completion = False
    hold_at(core, backend, 'CarryObject')
    backend.hold.add(node)
    core.request_cancel('deferred', CancelMode.RETURN_ARM)
    for _ in range(80):
        core.tick()
        if core.record.substage == node:
            break
    else:
        pytest.fail('Recovery node not reached')
    backend.defer_cancel_completion = True
    now[0] = 2.
    core.tick()
    core.tick()
    assert core.record.result is None
    backend.failures[node] = Observation(False, Error.HARDWARE_ERROR, 'Late recovery fault')
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.HARDWARE_ERROR
    assert result.failed_substage == node and result.stop_confirmed
    assert 'StopAfterRecovery' not in backend.started
    if node == 'ReleaseInPlace':
        assert 'ReturnArmAfterCancel' not in backend.started
    assert backend.started.count('StopAndAssess') == 2 and core.inhibited
