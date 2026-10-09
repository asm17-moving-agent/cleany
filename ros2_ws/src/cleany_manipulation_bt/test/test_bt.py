from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
from cleany_manipulation_bt._bt_runner import Runner
from cleany_manipulation_bt.backend import Observation
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.monitor import LiveMonitor
from cleany_skill_executor.manipulation.models import (
    CancelMode, Error, Goal, ObjectState, Placement, RecordState, Status,
)
from cleany_manipulation_bt.store import BTExecutionStore as ExecutionStore

TREE = str(Path(__file__).parents[1] / 'trees/collect_trash_mujoco.xml')
NORMAL = [n.tag for n in ET.parse(TREE).findall('.//BehaviorTree/Fallback/Sequence')[0].iter()
          if n.tag != 'Sequence']


class FakeBackend:
    def __init__(self, store, failures=None):
        self.store = store
        self.failures = failures or {}
        self.started = []
        self.pending = {}
        self.canceled = []
        self.hold = set()
        self.defer_cancel_completion = False
        self.aborted = False

    def ready(self):
        return True

    def begin(self, goal):
        self.goal = goal

    def start(self, node, execution):
        # Runtime state is updated before actuation even when its DB write fails.
        record = self.store.get(execution)
        assert record.substage == node and record.result is None
        self.started.append(node)
        token = str(len(self.started))
        self.pending[token] = node
        return token

    def poll(self, token):
        node = self.pending[token]
        if node in self.hold:
            if ((token in self.canceled or (self.aborted and node not in ('StopAndAssess', 'StopAfterRecovery')))
                    and not self.defer_cancel_completion):
                return Observation(False, Error.CANCELED, 'Mock operation cancellation confirmed')
            return None
        if node in self.failures:
            return self.failures[node]
        values = {
            'SelectArmAndPath': dict(selected_arm='right'),
            'ConfirmGrasp': dict(object_state=ObjectState.HELD),
            'ConfirmRelease': dict(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED),
            'ReturnArm': dict(arm_recovered=True),
            'VerifyPlacedObject': dict(placement_state=Placement.CONFIRMED, stop_confirmed=True),
            'StopAndAssess': dict(stop_confirmed=True),
            'ReleaseInPlace': dict(object_state=ObjectState.LEFT_GRIPPER),
            'ReturnArmAfterCancel': dict(arm_recovered=True),
            'StopAfterRecovery': dict(stop_confirmed=True),
        }
        return Observation(**values.get(node, {}))

    def cancel(self, token):
        self.canceled.append(token)

    def abort(self):
        self.aborted = True


@pytest.fixture
def execution(tmp_path):
    store = ExecutionStore(str(tmp_path / 'executions.sqlite3'))
    backend = FakeBackend(store)
    now = [0.0]
    core = ExecutionCore(backend, store, TREE, Runner, clock=lambda: now[0], timeout_sec=2., stop_timeout_sec=1.)
    backend.store = core.store
    goal = Goal('mission', 'task', 'execution-1', 'collect_trash', 'snapshot', 1, 'trash_right')
    assert core.accept(goal)[0]
    yield core, backend, store, now
    core.tree.halt()
    store.close()


def finish(core, now):
    for _ in range(150):
        core.tick()
        now[0] += 0.05
        if core.record.result is not None:
            return core.record.result
    pytest.fail('BT did not terminate')


def test_lost_item_uses_the_same_tree_and_durable_contract(tmp_path):
    store = ExecutionStore(str(tmp_path / 'lost.sqlite3'))
    backend = FakeBackend(store)
    now = [0.]
    core = ExecutionCore(backend, store, TREE, Runner, clock=lambda: now[0])
    backend.store = core.store
    try:
        goal = Goal('mission', 'task', 'lost', 'collect_lost_item', 'snapshot', 1, 'lost_items_left')
        assert core.accept(goal)[0]
        result = finish(core, now)
        assert result.status == Status.SUCCESS and result.placement_state == Placement.CONFIRMED
        assert backend.started == [n for n in NORMAL if not n.startswith('Finalize')]
        assert store.get('lost').goal == goal
    finally:
        core.tree.halt()
        store.close()


def test_real_tree_owns_order_and_durable_success(execution):
    core, backend, store, now = execution
    result = finish(core, now)
    assert backend.started == [node for node in NORMAL if node != 'FinalizeSuccess']
    assert result.status == Status.SUCCESS and result.execution_profile == 'mujoco'
    assert result.stop_confirmed and result.arm_recovered
    assert store.get('execution-1').result == result
    assert store._db.execute("SELECT COUNT(*) FROM bt_transitions WHERE status='RUNNING'").fetchone()[0] > 0
    assert not core.accept(core.record.goal)[0]


@pytest.mark.parametrize('node', NORMAL[:-1])
def test_failed_operation_never_starts_following_normal_motion(execution, node):
    core, backend, store, now = execution
    backend.failures[node] = Observation(False, Error.MOTION_FAILED, f'{node} failed')
    result = finish(core, now)
    assert backend.started == NORMAL[:NORMAL.index(node)+1] + ['StopAndAssess']
    assert result.status == (Status.BLOCKED if NORMAL.index(node) < 5 else Status.FAILED)
    assert result.failed_substage == node
    assert 'FinalizeFailure' in store.get('execution-1').substage


def test_running_is_actual_native_state_and_monitor_uid(execution):
    core, backend, _, _ = execution
    backend.hold.add('ValidateGoal')
    core.tick()
    snapshot = core.tree.snapshot()
    running = next(n for n in snapshot['nodes'] if n['node_id'] == 'ValidateGoal')
    assert running['status'] == 'RUNNING'
    monitor = LiveMonitor()
    monitor.update(snapshot)
    xml = ET.fromstring(monitor.reply([b'\x02T1234'])[1])
    assert xml.find('.//ValidateGoal').get('_uid') == str(running['uid'])
    payload = monitor.reply([b'\x02S1234'])[1]
    statuses = {uid: status for uid, status in struct.iter_unpack('<HB', payload)}
    assert statuses[running['uid']] == 1
    assert any(event['node_id'] == 'ValidateGoal' and event['status'] == 'RUNNING' for event in core.transitions)


@pytest.mark.parametrize('node,checkpoint,next_node', [
    ('MoveToPregrasp', 'MoveToPregrasp', 'ApproachObject'),
    ('GraspObject', 'ConfirmGrasp', 'LiftObject'),
    ('LiftObject', 'ConfirmHeld', 'CarryObject'),
    ('OpenGripperAtDestination', 'ConfirmRelease', 'ReturnArm'),
])
def test_cancel_finishes_only_current_atomic_checkpoint(execution, node, checkpoint, next_node):
    core, backend, _, now = execution
    for _ in range(60):
        core.tick()
        if core.record.substage == node:
            assert core.request_cancel('execution-1')
            break
    else:
        pytest.fail('cancel node not reached')
    result = finish(core, now)
    assert result.status == Status.CANCELED
    assert checkpoint in backend.started and next_node not in backend.started
    assert backend.started[-1] == 'StopAndAssess'


def hold_at(core, backend, node):
    backend.hold.add(node)
    for _ in range(80):
        core.tick()
        if core.record.substage == node:
            return
    pytest.fail(f'{node} not reached')


def test_return_cancel_preempts_carry_releases_here_and_returns(execution):
    core, backend, store, now = execution
    hold_at(core, backend, 'CarryObject')
    assert core.request_cancel('execution-1', CancelMode.RETURN_ARM)
    result = finish(core, now)
    assert backend.canceled
    assert backend.started[-4:] == ['StopAndAssess', 'ReleaseInPlace', 'ReturnArmAfterCancel', 'StopAfterRecovery']
    assert 'CheckPlacementTarget' not in backend.started and 'VerifyPlacedObject' not in backend.started
    assert result.status == Status.CANCELED and result.cancel_mode == 'RETURN_ARM'
    assert result.object_state == ObjectState.LEFT_GRIPPER and result.placement_state == Placement.NOT_CHECKED
    assert result.arm_recovered and result.stop_confirmed
    assert not core.inhibited and store.get('execution-1').result == result


def test_immediate_cancel_preempts_held_atomic_operation(execution):
    core, backend, _, now = execution
    hold_at(core, backend, 'GraspObject')
    assert core.request_cancel('execution-1', CancelMode.IMMEDIATE)
    result = finish(core, now)
    assert backend.canceled and result.cancel_mode == 'IMMEDIATE'
    assert result.status == Status.CANCELED and result.stop_confirmed
    assert 'ConfirmGrasp' not in backend.started and 'ReleaseInPlace' not in backend.started
    assert result.object_state == ObjectState.UNKNOWN


def test_immediate_cancel_interrupts_recovery_before_arm_return(execution):
    core, backend, _, now = execution
    hold_at(core, backend, 'CarryObject')
    backend.hold.add('ReleaseInPlace')
    core.request_cancel('execution-1', CancelMode.RETURN_ARM)
    for _ in range(80):
        core.tick()
        if core.record.substage == 'ReleaseInPlace':
            break
    else:
        pytest.fail('release not reached')
    assert core.request_cancel('execution-1', CancelMode.IMMEDIATE)
    result = finish(core, now)
    assert result.cancel_mode == 'IMMEDIATE' and not result.arm_recovered
    assert 'ReturnArmAfterCancel' not in backend.started
    assert backend.started[-1] == 'StopAndAssess' and result.stop_confirmed


@pytest.mark.parametrize('node', ['ReleaseInPlace', 'ReturnArmAfterCancel'])
def test_recovery_failure_stops_again_and_is_not_successful_cancellation(execution, node):
    core, backend, _, now = execution
    hold_at(core, backend, 'CarryObject')
    backend.failures[node] = Observation(False, Error.MOTION_FAILED, 'recovery failed')
    core.request_cancel('execution-1', CancelMode.RETURN_ARM)
    result = finish(core, now)
    assert result.status == Status.FAILED and result.failed_substage == node
    assert not result.arm_recovered and result.stop_confirmed
    assert backend.started[-1] == 'StopAndAssess'


def test_unconfirmed_initial_stop_blocks_return_cancel_recovery(execution):
    core, backend, _, now = execution
    hold_at(core, backend, 'CarryObject')
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=False)
    core.request_cancel('execution-1', CancelMode.RETURN_ARM)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert 'ReleaseInPlace' not in backend.started and not result.arm_recovered


def test_actual_motion_failure_is_preserved_when_cancel_arrives(execution):
    core, backend, _, now = execution
    hold_at(core, backend, 'CarryObject')
    backend.hold.remove('CarryObject')
    backend.failures['CarryObject'] = Observation(False, Error.MOTION_FAILED, 'controller failed')
    core.request_cancel('execution-1', CancelMode.RETURN_ARM)
    result = finish(core, now)
    assert result.status == Status.FAILED and result.error_code == Error.MOTION_FAILED
    assert 'ReleaseInPlace' not in backend.started


@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp', 'ReturnArm'])
@pytest.mark.parametrize('error', [
    Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR, Error.STOP_UNCONFIRMED,
])
def test_fatal_operation_error_blocks_next_goal_after_confirmed_stop(
        execution, node: str, error: Error) -> None:
    core, backend, store, now = execution
    backend.failures[node] = Observation(False, error, 'Fatal operation fault')
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == error
    assert result.stop_confirmed and result.failed_substage == node
    # No held/unknown object can independently account for the inhibition.
    assert result.object_state in (ObjectState.NOT_TOUCHED, ObjectState.LEFT_GRIPPER)
    assert core.inhibited and core.record.human_confirmation_required
    assert store.get('execution-1').result == result
    assert not core.accept(Goal('mission', 'task', 'next', 'collect_trash',
                                'snapshot', 1, 'trash_right'))[0]


@pytest.mark.parametrize('error', [Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR])
def test_fatal_operation_error_inhibits_while_stop_is_pending(execution, error: Error) -> None:
    core, backend, _, _ = execution
    backend.failures['MoveToPregrasp'] = Observation(False, error, 'Fatal operation fault')
    hold_at(core, backend, 'StopAndAssess')
    assert core.record.result is None and core.failure.error == error
    assert core.inhibited and core.record.human_confirmation_required


@pytest.mark.parametrize('error', [Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR])
def test_unconfirmed_stop_takes_priority_over_fatal_operation_error(execution, error: Error) -> None:
    core, backend, _, now = execution
    backend.failures['MoveToPregrasp'] = Observation(False, error, 'Original fatal operation fault')
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=False)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and result.message == 'Original fatal operation fault'
    assert core.inhibited and core.record.human_confirmation_required


def test_fatal_operation_error_remains_inhibited_after_restart(execution) -> None:
    core, backend, store, now = execution
    backend.failures['MoveToPregrasp'] = Observation(False, Error.HARDWARE_ERROR, 'Controller fault')
    result = finish(core, now)
    core.tree.halt()
    store.close()
    recovered_store = ExecutionStore(str(store.path))
    try:
        recovered = ExecutionCore(FakeBackend(recovered_store), recovered_store, TREE, Runner)
        record = recovered.get('execution-1')
        assert record.result == result and record.result.status == Status.FATAL
        assert record.human_confirmation_required and recovered.inhibited
        assert not recovered.accept(Goal('mission', 'task', 'after-restart', 'collect_trash',
                                         'snapshot', 1, 'trash_right'))[0]
    finally:
        recovered_store.close()


def test_recording_failure_preserves_fatal_operation_error(execution, monkeypatch) -> None:
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    backend.failures['MoveToPregrasp'] = Observation(False, Error.HARDWARE_ERROR, 'Controller fault')
    def fail(*args, **kwargs):
        raise StoreError('Injected recording failure')
    monkeypatch.setattr(store, 'save', fail)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.HARDWARE_ERROR
    assert result.stop_confirmed and core.get('execution-1').result == result
    assert core.inhibited and core.record.human_confirmation_required
    assert not core.accept(Goal('mission', 'task', 'next', 'collect_trash',
                                'snapshot', 1, 'trash_right'))[0]


def test_stop_unconfirmed_is_fatal(execution):
    core, backend, _, now = execution
    backend.failures['CarryObject'] = Observation(False, Error.GRASP_LOST, 'Lost retention')
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=False)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and core.inhibited


def test_paused_operation_times_out_and_late_success_is_ignored(execution):
    core, backend, _, now = execution
    backend.hold.add('CarryObject')
    result = finish(core, now)
    assert result.error_code == Error.TIMEOUT and backend.canceled
    backend.hold.clear()
    core.tick()
    assert core.record.result == result and 'OpenGripperAtDestination' not in backend.started


def test_verification_timeout_keeps_unknown_state(execution):
    core, backend, _, now = execution
    backend.failures['ConfirmRelease'] = Observation(False, Error.VERIFICATION_TIMEOUT,
                                                   placement_state=Placement.UNKNOWN)
    result = finish(core, now)
    assert result.error_code == Error.VERIFICATION_TIMEOUT
    assert result.object_state == ObjectState.UNKNOWN and result.placement_state == Placement.UNKNOWN
    assert not result.arm_recovered and 'ReturnArm' not in backend.started


def test_success_requires_independent_release_return_and_stop_evidence(execution):
    core, backend, _, now = execution
    backend.failures['VerifyPlacedObject'] = Observation(stop_confirmed=False)
    result = finish(core, now)
    assert result.status == Status.FAILED
    assert result.error_code == Error.PLACEMENT_NOT_CONFIRMED


def test_restart_records_interruption_and_does_not_resume(tmp_path):
    path = str(tmp_path / 'restart.sqlite3')
    store = ExecutionStore(path)
    backend = FakeBackend(store)
    core = ExecutionCore(backend, store, TREE, Runner)
    goal = Goal('mission', 'task', 'interrupted', 'collect_trash', 'snapshot', 1, 'trash_right')
    assert core.accept(goal)[0]
    core.tick()
    core.tree.halt()
    store.close()
    store = ExecutionStore(path)
    backend = FakeBackend(store)
    recovered = ExecutionCore(backend, store, TREE, Runner)
    record = recovered.get('interrupted')
    assert record.record_state == RecordState.INTERRUPTED
    assert record.human_confirmation_required and not record.stop_confirmed
    assert recovered.tree is None and recovered.inhibited and backend.started == []
    assert not recovered.accept(Goal('mission', 'task', 'new', 'collect_trash', 'snapshot', 1, 'trash_right'))[0]
    store.close()


@pytest.mark.parametrize('failure_point', ['stage_start', 'contact', 'final', 'all'])
def test_checkpoint_failure_preserves_motion_result_and_runtime_lookup(execution, monkeypatch, failure_point):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    original = store.save
    def save(record, **kwargs):
        if (failure_point == 'all'
                or (failure_point == 'stage_start' and record.substage == 'GraspObject')
                or (failure_point == 'contact' and record.object_state == ObjectState.HELD)
                or (failure_point == 'final' and record.result is not None)):
            raise StoreError('injected recording failure')
        original(record, **kwargs)
    monkeypatch.setattr(store, 'save', save)
    result = finish(core, now)
    assert backend.started == [node for node in NORMAL if node != 'FinalizeSuccess']
    assert result.status == Status.SUCCESS and result.error_code == Error.NONE
    assert result.stop_confirmed and result.arm_recovered and not core.inhibited
    assert not core.record.human_confirmation_required
    assert core.get('execution-1').result == result
    if failure_point == 'final':
        assert store.get('execution-1').result is None
    assert not core.accept(core.record.goal)[0]
    assert core.accept(Goal('mission', 'task', 'new', 'collect_trash', 'snapshot', 2, 'trash_right'))[0]
    assert finish(core, now).status == Status.SUCCESS
    assert core.get('execution-1').result == result


def test_transition_recording_failure_preserves_success(execution, monkeypatch):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    def fail(*_):
        raise StoreError('transition log unavailable')
    monkeypatch.setattr(store, 'save_transitions', fail)
    result = finish(core, now)
    assert result.status == Status.SUCCESS and result.error_code == Error.NONE
    assert backend.started == [node for node in NORMAL if node != 'FinalizeSuccess']
    assert backend.canceled == [] and not core.inhibited
    assert store.get('execution-1').result == result


def test_initial_record_failure_accepts_goal_and_preserves_duplicate_check(execution, monkeypatch):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    finish(core, now)
    def fail(*args, **kwargs):
        raise StoreError('injected persistent write outage')
    monkeypatch.setattr(store, 'save', fail)
    goal = Goal('mission', 'task', 'new', 'collect_trash', 'snapshot', 2, 'trash_right')
    assert core.accept(goal)[0]
    assert store.get('new') is None
    result = finish(core, now)
    assert result.status == Status.SUCCESS and core.get('new').result == result
    assert not core.accept(goal)[0]


@pytest.mark.parametrize('stop_confirmed,status,error', [
    (True, Status.FAILED, Error.MOTION_FAILED),
    (False, Status.FATAL, Error.STOP_UNCONFIRMED),
])
def test_record_failure_preserves_physical_failure_and_stop_decision(
        execution, monkeypatch, stop_confirmed, status, error):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    backend.failures['MoveToPregrasp'] = Observation(False, Error.MOTION_FAILED, 'motion failed')
    backend.failures['StopAndAssess'] = Observation(stop_confirmed=stop_confirmed)
    def fail(*args, **kwargs):
        raise StoreError('injected persistent write outage')
    monkeypatch.setattr(store, 'save', fail)
    result = finish(core, now)
    assert (result.status, result.error_code) == (status, error)
    assert result.stop_confirmed == stop_confirmed
    assert core.inhibited == (not stop_confirmed)
