from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from cleany_handeye_calibration.ik_port import ValidatedJointGoal
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import JointPose, TimedJointSample
from cleany_handeye_calibration.motion_config import (
    MujocoMotionConfig,
    StageTimeouts,
    ValidatedCurrentState,
    validate_dual_arm_current_state,
)
from cleany_handeye_calibration.motion_port import (
    EXECUTE_TRAJECTORY_ACTION_NAME,
    MOVE_GROUP_ACTION_NAME,
    MotionStatus,
    MoveItMotionAdapter,
)


class FakeFuture:
    def __init__(self, result=None, *, done=True, error=None):
        self._result = result
        self._done = done
        self._error = error
        self.cancel_called = False

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def cancel(self):
        self.cancel_called = True
        return True


class FakeGoalHandle:
    def __init__(
        self,
        wrapped_result=None,
        *,
        accepted=True,
        result_future=None,
        cancel_return_code=0,
        goal_uuid=bytes(range(16)),
        canceling_goal_uuids=None,
    ):
        self.accepted = accepted
        self.result_future = (
            FakeFuture(wrapped_result)
            if result_future is None
            else result_future
        )
        self.cancel_return_code = cancel_return_code
        self.goal_id = SimpleNamespace(uuid=list(goal_uuid))
        self.canceling_goal_uuids = (
            [goal_uuid]
            if canceling_goal_uuids is None
            else list(canceling_goal_uuids)
        )
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return FakeFuture(
            SimpleNamespace(
                return_code=self.cancel_return_code,
                goals_canceling=[
                    SimpleNamespace(
                        goal_id=SimpleNamespace(uuid=list(goal_uuid))
                    )
                    for goal_uuid in self.canceling_goal_uuids
                ],
            )
        )


class FakeActionClient:
    def __init__(self, goal_handles=(), *, ready=True):
        self.goal_handles = list(goal_handles)
        self.ready = ready
        self.goals = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return FakeFuture(self.goal_handles.pop(0))


class FakeNode:
    pass


class RecordingNode:
    def __init__(self):
        self.created = []

    def create_client(self, *args):
        self.created.append(args)


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def spin(self, timeout_sec):
        self.value += timeout_sec


def _config() -> MujocoMotionConfig:
    return MujocoMotionConfig(
        current_state_max_age_sec=0.2,
        right_park_position_tolerance_rad=0.02,
        stage_timeouts=StageTimeouts(
            ik_sec=0.05,
            state_validity_sec=0.05,
            plan_sec=0.05,
            execute_sec=0.05,
            cancel_sec=0.02,
            settle_sec=1.0,
        ),
    )


def _pose() -> JointPose:
    return JointPose(
        joint_names=LEFT_ARM_JOINT_NAMES,
        positions_rad=(0.1, 0.2, 0.3, 0.4, 0.5),
    )


def _validated_goal() -> ValidatedJointGoal:
    return ValidatedJointGoal(
        pose=_pose(),
        checked_state_stamp_ns=2_000_000_000,
    )


def _feedback(*, right_position=0.0) -> TimedJointSample:
    names = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    return TimedJointSample(
        stamp_ns=2_000_000_000,
        joint_names=names,
        positions_rad=tuple(
            right_position if name in RIGHT_ARM_JOINT_NAMES else 0.0
            for name in names
        ),
        velocities_rad_s=tuple(0.0 for _ in names),
    )


def _current_state() -> ValidatedCurrentState:
    return validate_dual_arm_current_state(
        _feedback(),
        now_stamp_ns=2_000_000_000,
        config=_config(),
    )


def _plan_result(
    *,
    error_code=MoveItErrorCodes.SUCCESS,
    positions=None,
):
    result = MoveGroup.Result()
    result.error_code.val = error_code
    result.planning_time = 0.01
    trajectory = result.planned_trajectory.joint_trajectory
    trajectory.joint_names = list(LEFT_ARM_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = list(
        _pose().positions_rad if positions is None else positions
    )
    trajectory.points = [point]
    return result


def _execute_result(*, error_code=MoveItErrorCodes.SUCCESS):
    result = ExecuteTrajectory.Result()
    result.error_code.val = error_code
    return result


def _wrapped(result, status=GoalStatus.STATUS_SUCCEEDED):
    return SimpleNamespace(status=status, result=result)


def _adapter(plan_client, execute_client, **kwargs):
    return MoveItMotionAdapter(
        FakeNode(),
        config=_config(),
        plan_client=plan_client,
        execute_client=execute_client,
        **kwargs,
    )


def test_plan_only_request_and_execute_are_distinct_actions():
    plan_client = FakeActionClient(
        [FakeGoalHandle(_wrapped(_plan_result()))]
    )
    execute_client = FakeActionClient(
        [FakeGoalHandle(_wrapped(_execute_result()))]
    )
    adapter = _adapter(plan_client, execute_client)

    result = adapter.move_to_joint_pose(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert result.success
    assert not result.ready_for_sample
    assert adapter.plan_action_name == MOVE_GROUP_ACTION_NAME == '/move_action'
    assert adapter.execute_action_name == (
        EXECUTE_TRAJECTORY_ACTION_NAME
    ) == '/execute_trajectory'
    assert len(plan_client.goals) == 1
    assert len(execute_client.goals) == 1

    plan_goal = plan_client.goals[0]
    assert isinstance(plan_goal, MoveGroup.Goal)
    assert plan_goal.planning_options.plan_only is True
    assert plan_goal.planning_options.look_around is False
    assert plan_goal.planning_options.replan is False
    assert plan_goal.request.group_name == 'left_arm'
    assert plan_goal.request.max_velocity_scaling_factor == 0.10
    assert plan_goal.request.max_acceleration_scaling_factor == 0.10
    constraints = plan_goal.request.goal_constraints
    assert len(constraints) == 1
    assert tuple(
        item.joint_name for item in constraints[0].joint_constraints
    ) == LEFT_ARM_JOINT_NAMES
    assert tuple(
        item.position for item in constraints[0].joint_constraints
    ) == _pose().positions_rad
    assert all(
        item.tolerance_above == 0.01
        and item.tolerance_below == 0.01
        and item.weight == 1.0
        for item in constraints[0].joint_constraints
    )

    execute_goal = execute_client.goals[0]
    assert isinstance(execute_goal, ExecuteTrajectory.Goal)
    assert (
        execute_goal.trajectory
        == result.plan_result.planned_motion.trajectory
    )


def test_raw_or_right_joint_pose_cannot_reach_plan_action():
    plan_client = FakeActionClient()
    execute_client = FakeActionClient()
    adapter = _adapter(plan_client, execute_client)

    with pytest.raises(ValueError, match='ValidatedJointGoal'):
        adapter.plan(_pose(), current_state=_current_state())

    right_pose = JointPose(
        joint_names=tuple(
            name.replace('left_', 'right_')
            for name in LEFT_ARM_JOINT_NAMES
        ),
        positions_rad=_pose().positions_rad,
    )
    with pytest.raises(ValueError, match='left-arm'):
        ValidatedJointGoal(
            pose=right_pose,
            checked_state_stamp_ns=2_000_000_000,
        )
    assert plan_client.goals == []


def test_stale_or_right_drift_state_cannot_reach_action():
    plan_client = FakeActionClient()
    adapter = _adapter(plan_client, FakeActionClient())
    unsafe = ValidatedCurrentState(
        sample=_feedback(right_position=0.1),
        validated_at_stamp_ns=2_000_000_000,
        age_ns=0,
    )

    with pytest.raises(RuntimeError, match='outside'):
        adapter.plan(_validated_goal(), current_state=unsafe)

    stale = ValidatedCurrentState(
        sample=_feedback(),
        validated_at_stamp_ns=2_200_000_001,
        age_ns=200_000_001,
    )
    with pytest.raises(RuntimeError, match='stale'):
        adapter.plan(_validated_goal(), current_state=stale)
    assert plan_client.goals == []


def test_plan_moveit_error_stops_before_execute_and_preserves_code():
    plan_client = FakeActionClient(
        [
            FakeGoalHandle(
                _wrapped(
                    _plan_result(
                        error_code=MoveItErrorCodes.PLANNING_FAILED
                    )
                )
            )
        ]
    )
    execute_client = FakeActionClient()
    adapter = _adapter(plan_client, execute_client)

    result = adapter.move_to_joint_pose(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert not result.success
    assert result.execution_result is None
    assert result.plan_result.status is MotionStatus.MOVEIT_ERROR
    assert result.plan_result.moveit_error_code == (
        MoveItErrorCodes.PLANNING_FAILED
    )
    assert execute_client.goals == []


@pytest.mark.parametrize(
    ('action_status', 'error_code', 'expected_status'),
    [
        (
            GoalStatus.STATUS_ABORTED,
            MoveItErrorCodes.CONTROL_FAILED,
            MotionStatus.ABORTED,
        ),
        (
            GoalStatus.STATUS_CANCELED,
            MoveItErrorCodes.PREEMPTED,
            MotionStatus.CANCELED,
        ),
    ],
)
def test_execute_status_mapping_preserves_controller_error_code(
    action_status,
    error_code,
    expected_status,
):
    plan_client = FakeActionClient(
        [FakeGoalHandle(_wrapped(_plan_result()))]
    )
    execute_client = FakeActionClient(
        [
            FakeGoalHandle(
                _wrapped(
                    _execute_result(error_code=error_code),
                    status=action_status,
                )
            )
        ]
    )
    adapter = _adapter(plan_client, execute_client)

    result = adapter.move_to_joint_pose(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert not result.success
    assert result.execution_result.status is expected_status
    assert result.execution_result.action_status == action_status
    assert result.execution_result.moveit_error_code == error_code


def test_plan_result_timeout_requests_bounded_action_cancel():
    clock = FakeClock()
    pending_result = FakeFuture(done=False)
    goal_handle = FakeGoalHandle(result_future=pending_result)
    plan_client = FakeActionClient([goal_handle])
    adapter = _adapter(
        plan_client,
        FakeActionClient(),
        monotonic=clock.monotonic,
        spin_once=clock.spin,
        poll_interval_sec=0.01,
    )

    result = adapter.plan(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert result.status is MotionStatus.TIMEOUT
    assert result.cancel_requested
    assert result.cancel_confirmed
    assert goal_handle.cancel_calls == 1
    assert clock.value == pytest.approx(0.05)


@pytest.mark.parametrize(
    'canceling_goal_uuids',
    [
        [],
        [bytes(reversed(range(16)))],
    ],
)
def test_cancel_confirmation_requires_the_exact_target_goal_uuid(
    canceling_goal_uuids,
):
    clock = FakeClock()
    goal_handle = FakeGoalHandle(
        result_future=FakeFuture(done=False),
        canceling_goal_uuids=canceling_goal_uuids,
    )
    adapter = _adapter(
        FakeActionClient([goal_handle]),
        FakeActionClient(),
        monotonic=clock.monotonic,
        spin_once=clock.spin,
        poll_interval_sec=0.01,
    )

    result = adapter.plan(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert result.status is MotionStatus.TIMEOUT
    assert result.cancel_requested
    assert not result.cancel_confirmed
    assert goal_handle.cancel_calls == 1


def test_cancel_confirmation_requires_humble_error_none_return_code():
    clock = FakeClock()
    goal_handle = FakeGoalHandle(
        result_future=FakeFuture(done=False),
        cancel_return_code=CancelGoal.Response.ERROR_REJECTED,
    )
    adapter = _adapter(
        FakeActionClient([goal_handle]),
        FakeActionClient(),
        monotonic=clock.monotonic,
        spin_once=clock.spin,
        poll_interval_sec=0.01,
    )

    result = adapter.plan(
        _validated_goal(),
        current_state=_current_state(),
    )

    assert result.status is MotionStatus.TIMEOUT
    assert result.cancel_requested
    assert not result.cancel_confirmed
    assert goal_handle.cancel_calls == 1


def test_right_scope_is_rejected_before_action_client_construction():
    node = RecordingNode()

    with pytest.raises(ValueError, match='exactly'):
        MoveItMotionAdapter(
            node,
            config=_config(),
            planning_group='right_arm',
            tip_link='right_gripper_frame',
        )

    assert node.created == []
