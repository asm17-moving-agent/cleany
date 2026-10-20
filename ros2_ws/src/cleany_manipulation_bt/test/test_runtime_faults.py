"""Unexpected runtime errors must stop and finish without relying on the BT."""
from types import SimpleNamespace as NS
import threading
import time

import pytest

from cleany_manipulation_bt._bt_runner import Runner
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.manipulation.models import CancelMode, Error, Goal, ObjectState, Placement, Status
from test_bt import FakeBackend, TREE, finish, hold_at


@pytest.fixture
def execution(tmp_path):
    store = BTExecutionStore(str(tmp_path / 'runtime-faults.sqlite3'))
    backend = FakeBackend(store)
    now = [0.0]
    core = ExecutionCore(backend, store, TREE, Runner, clock=lambda: now[0],
                         timeout_sec=2.0, stop_timeout_sec=1.0)
    backend.store = core.store
    assert core.accept(Goal('m', 't', 'execution', 'collect_trash', 's', 1, 'trash_right'))[0]
    try:
        yield core, backend, store, now
    finally:
        tree = getattr(core.tree, 'native_tree', core.tree)
        try:
            tree.halt()
        except Exception:
            pass  # Some tests deliberately leave the native tree unable to halt.
        store.close()


@pytest.mark.parametrize('fault', ['start', 'poll', 'bt_status'])
def test_unexpected_execution_error_stops_and_finishes_once(execution, monkeypatch, fault):
    core, backend, store, now = execution
    original_start, original_poll, original_core_poll = backend.start, backend.poll, core.poll
    injected = []

    def start(node, execution_id):
        if fault == 'start' and node == 'MoveToPregrasp' and not injected:
            injected.append(True)
            raise RuntimeError('Injected worker submission error')
        return original_start(node, execution_id)

    def poll(token):
        if fault == 'poll' and backend.pending[token] == 'MoveToPregrasp' and not injected:
            injected.append(True)
            raise KeyError('Injected operation lookup error')
        return original_poll(token)

    def core_poll(token):
        if fault == 'bt_status' and core.ops[token][0] == 'MoveToPregrasp' and not injected:
            injected.append(True)
            return 'INVALID_STATUS'
        return original_core_poll(token)

    monkeypatch.setattr(backend, 'start', start)
    monkeypatch.setattr(backend, 'poll', poll)
    monkeypatch.setattr(core, 'poll', core_poll)
    result = finish(core, now)
    assert injected and result.status == Status.FATAL
    assert result.error_code == Error.INTERNAL_ERROR and result.stop_confirmed
    assert result.failed_substage == 'MoveToPregrasp'
    assert backend.started[-1] == 'StopAndAssess'
    assert 'ApproachObject' not in backend.started
    assert core.inhibited and core.record.human_confirmation_required
    assert store.get('execution').result == result
    assert not core.accept(Goal('m', 't', 'next', 'collect_trash', 's', 1, 'trash_right'))[0]
    for _ in range(3):
        core.tick()
    assert core.record.result == result
    assert sum(event.result is not None for event in core.drain_events()) == 1


def broken_tree(core, *, phase='tick', halt_error=False):
    tree = core.tree

    def fail():
        raise RuntimeError(f'Injected BT {phase} failure')

    def halt():
        raise RuntimeError('Injected BT halt failure')

    core.tree = NS(tick=fail if phase == 'tick' else tree.tick,
                   drain_transitions=fail if phase == 'drain' else tree.drain_transitions,
                   halt=halt if halt_error else tree.halt, snapshot=tree.snapshot, native_tree=tree)


@pytest.mark.parametrize('phase,halt_error', [('tick', False), ('drain', False), ('tick', True)])
def test_broken_runner_is_never_needed_to_stop_or_finalize(execution, phase, halt_error):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    broken_tree(core, phase=phase, halt_error=halt_error)
    core.tick()
    assert core.inhibited and core.record.human_confirmation_required and backend.aborted
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.stop_confirmed and result.failed_substage == 'MoveToPregrasp'
    assert backend.started[-1] == 'StopAndAssess'
    assert f'Injected BT {phase} failure' in result.message
    if halt_error:
        assert 'Injected BT halt failure' in result.message


@pytest.mark.parametrize('phase', ['abort', 'cancel'])
def test_stop_continues_when_an_abort_or_cancel_request_throws(execution, monkeypatch, phase):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')

    def fail(*_):
        raise RuntimeError(f'Injected {phase} request failure')

    monkeypatch.setattr(backend, phase, fail)
    if phase == 'cancel':
        assert core.request_cancel('execution', CancelMode.IMMEDIATE)
    else:
        broken_tree(core)
        core.tick()
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.stop_confirmed and 'StopAndAssess' in backend.started
    assert f'Injected {phase} request failure' in result.message


@pytest.mark.parametrize('fault,error', [
    ('start', Error.STOP_UNCONFIRMED), ('poll', Error.STOP_UNCONFIRMED),
    ('poll_once', Error.INTERNAL_ERROR), ('invalid', Error.STOP_UNCONFIRMED),
    ('false', Error.STOP_UNCONFIRMED), ('worker', Error.STOP_UNCONFIRMED),
])
def test_additional_stop_failure_keeps_original_error_and_finishes_within_deadline(
        execution, monkeypatch, fault, error):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    broken_tree(core)
    original_start, original_poll = backend.start, backend.poll
    thrown = []

    def start(node, execution_id):
        if node == 'StopAndAssess' and fault == 'start':
            raise RuntimeError('Injected stop submission failure')
        return original_start(node, execution_id)

    def poll(token):
        if backend.pending[token] == 'StopAndAssess':
            if fault == 'poll' or (fault == 'poll_once' and not thrown):
                thrown.append(True)
                raise KeyError('Injected stop lookup failure')
            if fault == 'invalid':
                return object()
            if fault == 'false':
                return Observation(stop_confirmed=False)
            if fault == 'worker':
                return Observation(False, Error.INTERNAL_ERROR, 'Injected stop worker failure')
        return original_poll(token)

    monkeypatch.setattr(backend, 'start', start)
    monkeypatch.setattr(backend, 'poll', poll)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == error
    assert result.stop_confirmed == (error == Error.INTERNAL_ERROR)
    assert 'Injected BT tick failure' in result.message
    assert core.inhibited and core.record.human_confirmation_required
    assert now[0] <= 1.05 + 1e-9
    assert backend.started.count('StopAndAssess') <= 1
    core.tick()
    assert sum(event.result is not None for event in core.drain_events()) == 1


def test_unreadable_motion_result_cannot_be_overlooked_by_successful_stop(execution, monkeypatch):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    original_poll = backend.poll

    def poll(token):
        if backend.pending[token] == 'MoveToPregrasp':
            raise KeyError('Injected permanent operation lookup failure')
        return original_poll(token)

    monkeypatch.setattr(backend, 'poll', poll)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and 'MoveToPregrasp' in result.message
    assert now[0] <= 1.05 + 1e-9


@pytest.mark.parametrize('node', ['ConfirmRelease', 'ReturnArm'])
def test_runtime_fault_collects_late_physical_progress_before_finishing(execution, node):
    core, backend, _, now = execution
    backend.defer_cancel_completion = True
    hold_at(core, backend, node)
    broken_tree(core)
    core.tick()
    for _ in range(3):
        core.tick()
        now[0] += .05
    assert core.record.result is None
    backend.hold.remove(node)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.object_state == ObjectState.LEFT_GRIPPER and result.placement_state == Placement.CONFIRMED
    assert result.arm_recovered == (node == 'ReturnArm')
    assert result.stop_confirmed and node in core.record.completed_substages
    assert 'ReleaseInPlace' not in backend.started and 'ReturnArmAfterCancel' not in backend.started


def test_runtime_fault_does_not_wait_forever_for_a_worker_after_stop_evidence(execution):
    core, backend, _, now = execution
    backend.defer_cancel_completion = True
    hold_at(core, backend, 'MoveToPregrasp')
    broken_tree(core)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and now[0] <= 1.05 + 1e-9
    assert 'operations still pending: MoveToPregrasp' in result.message


def test_fault_during_stop_reuses_assessment_and_preserves_original_deadline(execution, monkeypatch):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    backend.hold.add('StopAndAssess')
    core.request_cancel('execution', CancelMode.IMMEDIATE)
    for _ in range(4):
        core.tick()
        if backend.started[-1] == 'StopAndAssess':
            break
    assert backend.started[-1] == 'StopAndAssess'
    now[0] = .8
    original_poll = backend.poll
    thrown = []

    def poll(token):
        if backend.pending[token] == 'StopAndAssess' and not thrown:
            thrown.append(True)
            raise RuntimeError('Injected running stop lookup failure')
        return original_poll(token)

    monkeypatch.setattr(backend, 'poll', poll)
    core.tick()
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert backend.started.count('StopAndAssess') == 1 and now[0] <= 1.05 + 1e-9


def test_worker_submission_losing_its_token_still_receives_context_abort(tmp_path):
    entered, aborted = threading.Event(), threading.Event()
    started = []

    def operation(node, context):
        started.append(node)
        if node == 'MoveToPregrasp':
            entered.set()
            assert context.abort.wait(2.0), 'Lost-token operation was not interrupted'
            aborted.set()
            return Observation(False, Error.CANCELED, 'Worker interrupted')
        return Observation(selected_arm='right' if node == 'SelectArmAndPath' else None,
                           stop_confirmed=True if node == 'StopAndAssess' else None)

    backend = WorkerBackend(operation)
    original_start = backend.start

    def start(node, execution_id):
        token = original_start(node, execution_id)
        if node == 'MoveToPregrasp':
            assert entered.wait(1.0)
            raise RuntimeError('Injected lost submission token')
        return token

    backend.start = start
    store = BTExecutionStore(str(tmp_path / 'lost-token.sqlite3'))
    core = ExecutionCore(backend, store, TREE, Runner, stop_timeout_sec=1.0)
    try:
        assert core.accept(Goal('m', 't', 'worker', 'collect_trash', 's', 1, 'trash_right'))[0]
        deadline = time.monotonic() + 2.0
        while core.record.result is None and time.monotonic() < deadline:
            core.tick()
            time.sleep(.001)
        result = core.record.result
        assert result is not None and result.status == Status.FATAL
        assert result.error_code == Error.INTERNAL_ERROR and result.stop_confirmed
        assert aborted.is_set() and backend.context.abort.is_set()
        assert started[-2:] == ['MoveToPregrasp', 'StopAndAssess']
    finally:
        backend.abort()
        backend.close()
        store.close()


@pytest.mark.parametrize('phase', ['ready', 'begin', 'factory'])
def test_setup_error_rejects_or_finishes_reserved_execution_without_leaving_it_busy(tmp_path, phase):
    store = BTExecutionStore(str(tmp_path / 'setup-fault.sqlite3'))
    backend = FakeBackend(store)
    factory = Runner

    def fail(*_):
        raise RuntimeError(f'Injected {phase} setup failure')

    if phase == 'factory':
        factory = fail
    else:
        setattr(backend, phase, fail)
    core = ExecutionCore(backend, store, TREE, factory)
    backend.store = core.store
    try:
        accepted, reason = core.accept(Goal('m', 't', 'setup', 'collect_trash', 's', 1, 'trash_right'))
        if phase == 'begin':
            assert accepted
            core.tick()
            assert core.record.result.status == Status.FATAL
            assert core.record.result.error_code == Error.INTERNAL_ERROR
            assert core.record.result.stop_confirmed and core.inhibited
        else:
            assert not accepted and f'Injected {phase} setup failure' in reason
            assert core.record is None and core.get('setup') is None
        assert backend.started == (['StopAndAssess'] if phase == 'begin' else [])
    finally:
        store.close()


def test_stop_observation_returned_at_deadline_cannot_confirm_stop(execution, monkeypatch):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    broken_tree(core)
    original_poll = backend.poll

    def poll(token):
        if backend.pending[token] == 'StopAndAssess':
            now[0] = 1.0
        return original_poll(token)

    monkeypatch.setattr(backend, 'poll', poll)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed


@pytest.mark.parametrize('node', ['ReleaseInPlace', 'ReturnArmAfterCancel'])
def test_runtime_fault_during_cancel_recovery_stops_without_next_recovery_motion(execution, node):
    core, backend, _, now = execution
    hold_at(core, backend, 'CarryObject')
    backend.hold.add(node)
    assert core.request_cancel('execution', CancelMode.RETURN_ARM)
    for _ in range(80):
        core.tick()
        if core.record.substage == node:
            break
    else:
        pytest.fail('Recovery operation was not reached')
    broken_tree(core)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.stop_confirmed and result.failed_substage == node
    assert backend.started[-1] == 'StopAndAssess'
    assert ('ReturnArmAfterCancel' in backend.started) == (node == 'ReturnArmAfterCancel')
    assert 'StopAfterRecovery' not in backend.started


def test_runtime_fault_result_survives_persistent_record_write_failure(execution, monkeypatch):
    from cleany_skill_executor.manipulation.store import StoreError

    core, backend, store, now = execution
    hold_at(core, backend, 'MoveToPregrasp')

    def fail(*_, **__):
        raise StoreError('Injected fault journal write failure')

    monkeypatch.setattr(store, 'save', fail)
    broken_tree(core)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.stop_confirmed and core.get('execution').result == result
    assert core.inhibited and sum(event.result is not None for event in core.drain_events()) == 1


def test_native_halt_callback_error_is_reported_after_resetting_the_tree(execution, monkeypatch):
    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')

    def fail(_):
        raise RuntimeError('Injected native halt cancellation failure')

    monkeypatch.setattr(backend, 'cancel', fail)
    core.tree.halt()
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.stop_confirmed and 'Injected native halt cancellation failure' in result.message
    assert 'ApproachObject' not in backend.started


def test_action_timer_delivers_fault_result_once_even_if_bt_snapshot_throws(execution):
    from concurrent.futures import Future
    from cleany_manipulation_bt.node import MujocoManipulationNode
    import zmq

    core, backend, _, now = execution
    hold_at(core, backend, 'MoveToPregrasp')
    broken_tree(core)

    def snapshot():
        raise RuntimeError('Injected snapshot failure')

    def recv(**_):
        raise zmq.Again()

    core.tree.snapshot = snapshot
    node = object.__new__(MujocoManipulationNode)
    node.core = core
    delivered, warnings = [], []
    node._handle = NS(abort=lambda: delivered.append('abort'))
    future = Future()
    node._result_future = future
    node._publish_events = lambda: None
    node._monitor_execution = node._monitor_failed_execution = ''
    node.monitor = NS(update=lambda *_, **__: None)
    node._bt_events = NS(publish=lambda _: None)
    node.socket = NS(recv_multipart=recv)
    node.get_logger = lambda: NS(warning=warnings.append)
    for _ in range(20):
        node._tick()
        now[0] += .05
        if future.done():
            break
    assert future.done() and future.result().status == 'FATAL'
    assert future.result().error_code == 'INTERNAL_ERROR'
    assert future.result().stop_confirmed and delivered == ['abort']
    for _ in range(3):
        node._tick()
    assert delivered == ['abort'] and len(warnings) == 1
