from types import SimpleNamespace

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import pytest

from cleany_handeye_calibration.ik_port import (
    CHECK_STATE_VALIDITY_SERVICE_NAME,
    COMPUTE_IK_SERVICE_NAME,
    IkFailure,
    MoveItPositionIKAdapter,
    MoveItStateValidityAdapter,
    StateValidityStatus,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    TimedJointSample,
)
from cleany_handeye_calibration.motion_config import (
    MujocoMotionConfig,
    StageTimeouts,
    ValidatedCurrentState,
    validate_dual_arm_current_state,
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


class FakeServiceClient:
    def __init__(self, responses=(), *, ready=True):
        self.responses = list(responses)
        self.ready = ready
        self.requests = []
        self.future = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        if self.future is not None:
            return self.future
        return FakeFuture(self.responses.pop(0))


class FakeNode:
    def __init__(self, client=None):
        self.client = client
        self.created = []

    def create_client(self, service_type, service_name):
        self.created.append((service_type, service_name))
        return self.client


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
            plan_sec=0.1,
            execute_sec=0.1,
            cancel_sec=0.05,
            settle_sec=1.0,
        ),
    )


def _feedback(
    *,
    right_position=0.0,
    reverse=False,
    with_extra=False,
) -> TimedJointSample:
    names = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    if with_extra:
        names = names + ('head_pan_joint',)
    if reverse:
        names = tuple(reversed(names))
    positions = tuple(
        right_position if name in RIGHT_ARM_JOINT_NAMES else index / 100.0
        for index, name in enumerate(names)
    )
    return TimedJointSample(
        stamp_ns=1_234_567_890,
        joint_names=names,
        positions_rad=positions,
        velocities_rad_s=tuple(0.0 for _ in names),
    )


def _current_state(
    sample: TimedJointSample | None = None,
) -> ValidatedCurrentState:
    return validate_dual_arm_current_state(
        _feedback() if sample is None else sample,
        now_stamp_ns=1_234_567_890,
        config=_config(),
    )


def _seed(offset=0.0) -> JointPose:
    return JointPose(
        joint_names=LEFT_ARM_JOINT_NAMES,
        positions_rad=tuple(
            offset + index / 10.0
            for index in range(len(LEFT_ARM_JOINT_NAMES))
        ),
    )


def _target() -> PositionTarget:
    return PositionTarget('base_link', (0.3, 0.2, 0.5))


def _ik_response(
    *,
    error_code=MoveItErrorCodes.SUCCESS,
    omit_joint=None,
    offset=1.0,
):
    response = GetPositionIK.Response()
    response.error_code.val = error_code
    names = tuple(
        name
        for name in DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
        if name != omit_joint
    )
    response.solution.joint_state.name = list(names)
    response.solution.joint_state.position = [
        offset + index / 10.0 for index, _ in enumerate(names)
    ]
    return response


def test_ik_request_contains_target_explicit_tip_and_full_feedback_seed():
    client = FakeServiceClient([_ik_response()])
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    feedback = _feedback(reverse=True, with_extra=True)
    current_state = _current_state(feedback)
    result = adapter.solve_position(
        _target(),
        _seed(),
        current_state=current_state,
    )

    assert result.success
    assert result.joint_pose is not None
    assert result.joint_pose.joint_names == LEFT_ARM_JOINT_NAMES
    assert adapter.service_name == COMPUTE_IK_SERVICE_NAME == '/compute_ik'
    assert len(client.requests) == 1
    request = client.requests[0].ik_request
    assert request.group_name == 'left_arm'
    assert request.ik_link_name == 'left_gripper_frame'
    assert request.avoid_collisions is True
    assert request.robot_state.is_diff is False
    assert request.pose_stamped.header.frame_id == 'base_link'
    assert request.pose_stamped.pose.position.x == 0.3
    assert request.pose_stamped.pose.position.y == 0.2
    assert request.pose_stamped.pose.position.z == 0.5
    assert request.pose_stamped.pose.orientation.x == 0.0
    assert request.pose_stamped.pose.orientation.y == 0.0
    assert request.pose_stamped.pose.orientation.z == 0.0
    assert request.pose_stamped.pose.orientation.w == 1.0
    assert request.timeout.sec == 0
    assert request.timeout.nanosec == 50_000_000
    assert request.pose_stamped.header.stamp.sec == 1
    assert request.pose_stamped.header.stamp.nanosec == 234_567_890

    state_names = tuple(request.robot_state.joint_state.name)
    assert state_names == (
        DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    )
    assert len(state_names) == 12
    assert tuple(request.robot_state.joint_state.velocity) == tuple(
        0.0 for _ in state_names
    )
    state_positions = dict(
        zip(
            state_names,
            request.robot_state.joint_state.position,
            strict=True,
        )
    )
    for name, expected in zip(
        LEFT_ARM_JOINT_NAMES,
        _seed().positions_rad,
        strict=True,
    ):
        assert state_positions[name] == expected
    feedback_positions = dict(
        zip(
            feedback.joint_names,
            feedback.positions_rad,
            strict=True,
        )
    )
    for name in (
        set(state_names) - set(LEFT_ARM_JOINT_NAMES)
    ):
        assert state_positions[name] == feedback_positions[name]


def test_seed_variation_is_preserved_in_distinct_full_robot_requests():
    client = FakeServiceClient(
        [_ik_response(offset=1.0), _ik_response(offset=2.0)]
    )
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    first = adapter.solve_position(
        _target(),
        _seed(0.0),
        current_state=_current_state(),
    )
    second = adapter.solve_position(
        _target(),
        _seed(0.5),
        current_state=_current_state(),
    )

    assert first.success and second.success
    first_state = dict(
        zip(
            client.requests[0].ik_request.robot_state.joint_state.name,
            client.requests[0].ik_request.robot_state.joint_state.position,
            strict=True,
        )
    )
    second_state = dict(
        zip(
            client.requests[1].ik_request.robot_state.joint_state.name,
            client.requests[1].ik_request.robot_state.joint_state.position,
            strict=True,
        )
    )
    assert tuple(first_state[name] for name in LEFT_ARM_JOINT_NAMES) != (
        tuple(second_state[name] for name in LEFT_ARM_JOINT_NAMES)
    )
    assert first.joint_pose != second.joint_pose


@pytest.mark.parametrize(
    ('response', 'reason'),
    [
        (
            _ik_response(error_code=MoveItErrorCodes.NO_IK_SOLUTION),
            IkFailure.NO_IK_SOLUTION.value,
        ),
        (
            _ik_response(omit_joint=LEFT_ARM_JOINT_NAMES[-1]),
            IkFailure.INVALID_RESPONSE.value,
        ),
    ],
)
def test_ik_maps_moveit_failure_and_incomplete_solution(response, reason):
    client = FakeServiceClient([response])
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    result = adapter.solve_position(
        _target(),
        _seed(),
        current_state=_current_state(),
    )

    assert not result.success
    assert result.failure_reason == reason


def test_ik_timeout_uses_monotonic_budget_and_cancels_service_future():
    clock = FakeClock()
    client = FakeServiceClient()
    client.future = FakeFuture(done=False)
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
        monotonic=clock.monotonic,
        spin_once=clock.spin,
        poll_interval_sec=0.01,
    )

    result = adapter.solve_position(
        _target(),
        _seed(),
        current_state=_current_state(),
    )

    assert not result.success
    assert result.failure_reason == IkFailure.TIMEOUT.value
    assert client.future.cancel_called
    assert clock.value == pytest.approx(0.05)


def test_right_scope_and_unsafe_state_are_rejected_before_ros_calls():
    node = FakeNode(FakeServiceClient([_ik_response()]))
    with pytest.raises(ValueError, match='exactly'):
        MoveItPositionIKAdapter(
            node,
            config=_config(),
            planning_group='right_arm',
            tip_link='right_gripper_frame',
        )
    assert node.created == []

    client = FakeServiceClient([_ik_response()])
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )
    unsafe = ValidatedCurrentState(
        sample=_feedback(right_position=0.1),
        validated_at_stamp_ns=1_234_567_890,
        age_ns=0,
    )
    with pytest.raises(RuntimeError, match='outside'):
        adapter.solve_position(
            _target(),
            _seed(),
            current_state=unsafe,
        )
    assert client.requests == []


def test_ik_rejects_non_base_target_before_service_call():
    client = FakeServiceClient([_ik_response()])
    adapter = MoveItPositionIKAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    with pytest.raises(ValueError, match='base_link'):
        adapter.solve_position(
            PositionTarget('map', (0.3, 0.2, 0.5)),
            _seed(),
            current_state=_current_state(),
        )

    assert client.requests == []


def test_state_validity_request_uses_full_state_with_resolved_left_joints():
    response = GetStateValidity.Response()
    response.valid = True
    client = FakeServiceClient([response])
    adapter = MoveItStateValidityAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    result = adapter.validate(
        _seed(0.4),
        current_state=_current_state(),
    )

    assert result.valid and result.checked
    assert result.validated_goal is not None
    assert result.validated_goal.pose == _seed(0.4)
    assert result.validated_goal.checked_state_stamp_ns == 1_234_567_890
    assert adapter.service_name == (
        CHECK_STATE_VALIDITY_SERVICE_NAME
    ) == '/check_state_validity'
    request = client.requests[0]
    assert request.group_name == 'left_arm'
    assert request.robot_state.is_diff is False
    assert tuple(request.robot_state.joint_state.name) == (
        DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    )
    assert len(request.robot_state.joint_state.name) == 12
    assert tuple(request.robot_state.joint_state.velocity) == tuple(
        0.0
        for _ in DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
    )
    positions = dict(
        zip(
            request.robot_state.joint_state.name,
            request.robot_state.joint_state.position,
            strict=True,
        )
    )
    assert tuple(positions[name] for name in LEFT_ARM_JOINT_NAMES) == (
        _seed(0.4).positions_rad
    )


def test_state_validity_maps_collision_and_limit_or_invalid_state():
    collision_response = SimpleNamespace(
        valid=False,
        contacts=[
            SimpleNamespace(
                contact_body_1='left_wrist_link',
                contact_body_2='target_board',
            )
        ],
        constraint_result=[],
    )
    limit_response = SimpleNamespace(
        valid=False,
        contacts=[],
        constraint_result=[],
    )
    client = FakeServiceClient([collision_response, limit_response])
    adapter = MoveItStateValidityAdapter(
        FakeNode(),
        config=_config(),
        client=client,
    )

    collision = adapter.validate(
        _seed(),
        current_state=_current_state(),
    )
    invalid = adapter.validate(
        _seed(),
        current_state=_current_state(),
    )

    assert collision.status is StateValidityStatus.COLLISION
    assert collision.contact_pairs == (
        ('left_wrist_link', 'target_board'),
    )
    assert collision.validated_goal is None
    assert not collision.valid and collision.checked
    assert invalid.status is (
        StateValidityStatus.JOINT_LIMIT_OR_STATE_INVALID
    )
    assert invalid.validated_goal is None
    assert invalid.checked
