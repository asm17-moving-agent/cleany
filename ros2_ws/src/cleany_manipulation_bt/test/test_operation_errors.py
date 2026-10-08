"""ROS operation exception classification through the real worker and native BT."""
from dataclasses import dataclass, field
from types import SimpleNamespace as NS
import time

import pytest

from cleany_manipulation_bt.backend import Observation, OperationError, WorkerBackend
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.joint_feedback import JointFeedback
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.core.grasp_selection import InfrastructureError, REQUIRED_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Error, Goal, ObjectState, Placement, Status
from cleany_skill_executor.nearest_pregrasp_coordinator import LiftRedetectionError
from cleany_skill_executor.seeded_cartesian import CartesianPlanningError
from test_bt import TREE, Runner
from test_cancel_completion import tick_until


@dataclass
class OperationControl:
    node: str = ''
    error: Exception | None = None
    stop_confirmed: bool = True
    started: list[str] = field(default_factory=list)


@pytest.fixture
def operation_execution(tmp_path):
    control = OperationControl()
    # Instantiate the actual dispatcher without starting a ROS/DDS node. Only
    # hardware calls are replaced; execute(), WorkerBackend and the BT are real.
    operations = object.__new__(MujocoOperations)
    operations.execution_context = None
    operations.handles, operations.submissions = [], []
    operations.get_parameter = lambda _: NS(value=180.)
    operations._guard = lambda: None
    values = {
        'SelectArmAndPath': dict(selected_arm='right'),
        'ConfirmGrasp': dict(object_state=ObjectState.HELD),
        'ConfirmRelease': dict(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED),
        'ReturnArm': dict(arm_recovered=True),
        'VerifyPlacedObject': dict(placement_state=Placement.CONFIRMED, stop_confirmed=True),
    }

    def replace_hardware(node):
        def operation(context):
            control.started.append(node)
            if node == control.node:
                raise control.error
            if node in ('StopAndAssess', 'StopAfterRecovery'):
                return Observation(stop_confirmed=control.stop_confirmed)
            return Observation(**values.get(node, {}))
        return operation

    methods = {
        'ValidateGoal': 'validate', 'PrepareTarget': 'prepare', 'ReconstructTarget': 'reconstruct',
        'GenerateGrasp': 'generate', 'SelectArmAndPath': 'select', 'MoveToPregrasp': 'pregrasp',
        'ApproachObject': 'approach', 'GraspObject': 'grasp', 'ConfirmGrasp': 'confirm_grasp',
        'LiftObject': 'lift', 'ConfirmHeld': 'confirm_held', 'CarryObject': 'carry',
        'CheckPlacementTarget': 'check_placement', 'OpenGripperAtDestination': 'open',
        'ConfirmRelease': 'confirm_release', 'ReturnArm': 'return_arm', 'VerifyPlacedObject': 'verify',
        'StopAndAssess': 'stop', 'ReleaseInPlace': 'release_in_place',
        'ReturnArmAfterCancel': 'return_after_cancel', 'StopAfterRecovery': 'stop_after_recovery',
    }
    for node, method in methods.items():
        setattr(operations, method, replace_hardware(node))
    backend = WorkerBackend(operations.execute)
    store = BTExecutionStore(str(tmp_path / 'operation-errors.sqlite3'))
    core = ExecutionCore(backend, store, TREE, Runner, timeout_sec=2., stop_timeout_sec=1.)
    try:
        assert core.accept(Goal('m', 't', 'operation-error', 'collect_trash', 's', 1, 'trash_right'))[0]
        yield core, operations, store, control
    finally:
        if core.tree is not None:
            core.tree.halt()
        backend.close()
        store.close()


@pytest.mark.parametrize('node', ['ValidateGoal', 'MoveToPregrasp', 'GraspObject', 'ConfirmHeld', 'ReturnArm'])
@pytest.mark.parametrize('exception_type', [AttributeError, TypeError, KeyError, NotImplementedError])
def test_unexpected_ros_operation_error_is_fatal(operation_execution, node, exception_type):
    core, _, store, control = operation_execution
    control.node = node
    control.error = exception_type('Injected programming error')
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert result.failed_substage == node and result.stop_confirmed
    assert exception_type.__name__ in result.message and 'Injected programming error' in result.message
    assert store.get('operation-error').result == result
    assert core.inhibited and core.record.human_confirmation_required
    assert not core.accept(Goal('m', 't', 'next', 'collect_trash', 's', 1, 'trash_right'))[0]
    assert control.started[-1] == 'StopAndAssess'
    assert 'ReleaseInPlace' not in control.started and 'ReturnArmAfterCancel' not in control.started
    core.tick()
    assert core.record.result == result
    assert sum(event.result is not None for event in core.drain_events()) == 1


class UnexpectedRuntimeError(RuntimeError):
    pass


@pytest.mark.parametrize('exception_type', [UnexpectedRuntimeError, RecursionError, IndexError, AssertionError, OSError])
def test_other_unexpected_operation_errors_are_not_motion_failures(operation_execution, exception_type):
    core, _, _, control = operation_execution
    control.node = 'MoveToPregrasp'
    control.error = exception_type('Unexpected operation error')
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.status == Status.FATAL and result.error_code == Error.INTERNAL_ERROR
    assert exception_type.__name__ in result.message and result.stop_confirmed
    assert core.inhibited and 'ApproachObject' not in control.started


@pytest.mark.parametrize('node,error,expected', [
    ('ValidateGoal', RuntimeError('Backend unavailable'), Error.BACKEND_NOT_READY),
    ('MoveToPregrasp', RuntimeError('Controller rejected movement'), Error.MOTION_FAILED),
    ('MoveToPregrasp', ValueError('Approved object changed'), Error.STALE_TARGET),
    ('ApproachObject', ValueError('Approved track changed'), Error.STALE_TARGET),
    ('GraspObject', RuntimeError('Gripper did not close'), Error.GRASP_FAILED),
    ('ConfirmGrasp', RuntimeError('No settled contact'), Error.GRASP_FAILED),
    ('ConfirmHeld', RuntimeError('Contact lost'), Error.GRASP_LOST),
    ('CarryObject', CartesianPlanningError('No bounded carry path'), Error.MOTION_FAILED),
    ('ConfirmHeld', LiftRedetectionError('Object absent after lift'), Error.GRASP_LOST),
    ('PrepareTarget', InfrastructureError('Inspection unavailable'), Error.MOTION_FAILED),
    ('MoveToPregrasp', InfrastructureError('IK unavailable'), Error.MOTION_FAILED),
])
def test_expected_operation_failures_keep_existing_classification(operation_execution, node, error, expected):
    core, _, _, control = operation_execution
    control.node, control.error = node, error
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.error_code == expected and result.stop_confirmed
    assert result.status == (Status.BLOCKED if node in ('ValidateGoal', 'PrepareTarget') else Status.FAILED)
    assert result.failed_substage == node and str(error) in result.message


@pytest.mark.parametrize('error', [Error.HARDWARE_ERROR, Error.E_STOP, Error.INTERNAL_ERROR, Error.MOTION_FAILED])
def test_explicit_operation_error_code_is_preserved(operation_execution, error):
    core, _, _, control = operation_execution
    control.node = 'MoveToPregrasp'
    control.error = OperationError(error, 'Explicit operation failure')
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.error_code == error and result.message == 'Explicit operation failure'
    assert result.status == (Status.FAILED if error == Error.MOTION_FAILED else Status.FATAL)
    assert result.stop_confirmed


def test_programming_error_with_unconfirmed_stop_blocks_next_goal(operation_execution):
    core, _, _, control = operation_execution
    control.node, control.error = 'MoveToPregrasp', AttributeError('Missing execution state')
    control.stop_confirmed = False
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and 'AttributeError' in result.message
    assert core.inhibited and not core.accept(Goal('m', 't', 'next', 'collect_trash', 's', 1, 'trash_right'))[0]


def test_movegroup_controller_failure_remains_motion_failure(operation_execution):
    from action_msgs.msg import GoalStatus
    from moveit_msgs.msg import MoveItErrorCodes
    from cleany_skill_executor.grasp_execution import GraspExecutionNode

    core, operations, _, _ = operation_execution
    handle = NS(accepted=True, get_result_async=lambda: NS(
        status=GoalStatus.STATUS_ABORTED,
        result=NS(error_code=NS(val=MoveItErrorCodes.CONTROL_FAILED))))
    motion = NS(_controller_retry_enabled=False, _execution_goal=lambda *_: object(),
                _move_group=NS(send_goal_async=lambda _: handle), _future=lambda future, *_: future,
                _log_joint_tracking_error=lambda *_: None,
                get_logger=lambda: NS(info=lambda _: None, warning=lambda _: None))
    operations.pregrasp = lambda _: GraspExecutionNode._move_to(motion, 'right', object(), 'pregrasp')
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.status == Status.FAILED and result.error_code == Error.MOTION_FAILED
    assert result.stop_confirmed and 'execution failed' in result.message
    assert not core.inhibited and core.accept(Goal('m', 't', 'next', 'collect_trash', 's', 1, 'trash_right'))[0]


def test_reobserved_target_failure_remains_stale_target(operation_execution):
    core, operations, _, _ = operation_execution
    operations._wrist_enabled = False

    def refresh(*_, **__):
        raise ValueError('Approved track is missing or its epoch changed')

    operations._refresh_selected_grasp = refresh
    operations.pregrasp = lambda context: MujocoOperations.refresh_approved(operations, context)
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    assert result.status == Status.FAILED and result.error_code == Error.STALE_TARGET
    assert result.stop_confirmed and 'Approved track' in result.message


@pytest.mark.parametrize('exception_type', [RuntimeError, NotImplementedError, RecursionError])
def test_contact_guard_preserves_only_expected_retention_failures(operation_execution, exception_type):
    core, operations, _, _ = operation_execution

    def contact(_):
        raise exception_type('Contact check failed')

    def confirm_held(context):
        wall = time.monotonic()
        operations._held_object = object()
        operations._joint_feedback = JointFeedback(REQUIRED_JOINT_NAMES)
        operations._joint_feedback.update(REQUIRED_JOINT_NAMES, [0.] * len(REQUIRED_JOINT_NAMES), 10**9, wall)
        operations.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10**9))
        operations._clock_value, operations._clock_progress_wall = 10**9, wall
        return MujocoOperations._guard(operations)

    operations._require_held_contact = contact
    operations.confirm_held = confirm_held
    tick_until(core, lambda: core.record.result is not None)
    result = core.record.result
    expected = exception_type is RuntimeError
    assert result.status == (Status.FAILED if expected else Status.FATAL)
    assert result.error_code == (Error.GRASP_LOST if expected else Error.INTERNAL_ERROR)
    assert result.failed_substage == 'ConfirmHeld' and result.stop_confirmed
    if not expected:
        assert exception_type.__name__ in result.message and core.record.human_confirmation_required
