from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
from cleany_manipulation_bt._bt_runner import Runner
from cleany_manipulation_bt.backend import Observation
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.monitor import LiveMonitor
from cleany_skill_executor.manipulation.models import (
    Error, Goal, ObjectState, Placement, RecordState, Status,
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

    def ready(self):
        return True

    def begin(self, goal):
        self.goal = goal

    def start(self, node, execution):
        # A physical operation cannot start ahead of its durable checkpoint.
        record = self.store.get(execution)
        assert record.substage == node and record.result is None
        self.started.append(node)
        token = str(len(self.started))
        self.pending[token] = node
        return token

    def poll(self, token):
        node = self.pending[token]
        if node in self.hold:
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
        }
        return Observation(**values.get(node, {}))

    def cancel(self, token):
        self.canceled.append(token)


@pytest.fixture
def execution(tmp_path):
    store = ExecutionStore(str(tmp_path / 'executions.sqlite3'))
    backend = FakeBackend(store)
    now = [0.0]
    core = ExecutionCore(backend, store, TREE, Runner, clock=lambda: now[0], timeout_sec=2., stop_timeout_sec=1.)
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


def test_checkpoint_failure_prevents_actuation_and_persists_inhibition(execution, monkeypatch):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    original = store.save
    def save(record, **kwargs):
        if record.substage == 'GraspObject':
            raise StoreError('injected recording failure')
        original(record, **kwargs)
    monkeypatch.setattr(store, 'save', save)
    result = finish(core, now)
    assert 'GraspObject' not in backend.started and 'LiftObject' not in backend.started
    assert result.status == Status.FATAL and core.inhibited
    assert store.get('execution-1').human_confirmation_required


def test_transition_recording_failure_prevents_success(execution, monkeypatch):
    from cleany_skill_executor.manipulation.store import StoreError
    core, backend, store, now = execution
    def fail(*_):
        raise StoreError('transition log unavailable')
    monkeypatch.setattr(store, 'save_transitions', fail)
    result = finish(core, now)
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert 'MoveToPregrasp' not in backend.started
