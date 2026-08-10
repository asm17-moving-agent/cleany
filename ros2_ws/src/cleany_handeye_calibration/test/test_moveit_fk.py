from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
import pytest
from sensor_msgs.msg import JointState

from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    InterpolatedJointState,
)
from cleany_handeye_calibration.models import TimedJointSample
from cleany_handeye_calibration.moveit_fk import (
    BASE_FRAME,
    CALIBRATION_LINK,
    COMPUTE_FK_SERVICE_NAME,
    ForwardKinematicsError,
    ForwardKinematicsFailure,
    MoveItForwardKinematicsAdapter,
    ros_stamp_to_nanoseconds,
    timed_joint_sample_from_message,
)


JOINT_NAMES = (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
)


@dataclass
class _ManualClock:
    value: float = 100.0

    def __call__(self) -> float:
        return self.value

    def advance(self, duration: float) -> None:
        self.value += duration


class _FakeFuture:
    def __init__(
        self,
        response: GetPositionFK.Response | None,
        *,
        done: bool = True,
        error: Exception | None = None,
    ) -> None:
        self._response = response
        self._done = done
        self._error = error
        self.cancelled = False

    def done(self) -> bool:
        return self._done

    def result(self) -> GetPositionFK.Response | None:
        if self._error is not None:
            raise self._error
        return self._response

    def cancel(self) -> None:
        self.cancelled = True


class _FakeClient:
    def __init__(
        self,
        future: _FakeFuture,
        *,
        ready: bool = True,
        call_error: Exception | None = None,
    ) -> None:
        self.future = future
        self.ready = ready
        self.call_error = call_error
        self.requests: list[GetPositionFK.Request] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request: GetPositionFK.Request) -> _FakeFuture:
        self.requests.append(request)
        if self.call_error is not None:
            raise self.call_error
        return self.future


class _FakeNode:
    def __init__(self, client: _FakeClient) -> None:
        self.client = client
        self.created_service_type = None
        self.created_service_name = None

    def create_client(self, service_type, service_name: str) -> _FakeClient:
        self.created_service_type = service_type
        self.created_service_name = service_name
        return self.client


def _interpolation(
    *,
    stamp_ns: int = 12_345_678_901,
    joint_names: tuple[str, ...] = JOINT_NAMES,
) -> InterpolatedJointState:
    sample = TimedJointSample(
        stamp_ns=stamp_ns,
        joint_names=joint_names,
        positions_rad=tuple(
            0.25 * index for index in range(len(joint_names))
        ),
        velocities_rad_s=tuple(
            -0.1 * index for index in range(len(joint_names))
        ),
    )
    return InterpolatedJointState(
        sample=sample,
        before_stamp_ns=stamp_ns - 1,
        after_stamp_ns=stamp_ns + 1,
        ratio=0.5,
    )


def _response() -> GetPositionFK.Response:
    response = GetPositionFK.Response()
    response.error_code.val = MoveItErrorCodes.SUCCESS
    response.fk_link_names = [CALIBRATION_LINK]
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = 0.4
    pose.pose.position.y = -0.2
    pose.pose.position.z = 0.7
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    response.pose_stamped = [pose]
    return response


def _adapter(
    client: _FakeClient,
    *,
    clock: _ManualClock | None = None,
    spin_once: Callable[[float], None] | None = None,
) -> MoveItForwardKinematicsAdapter:
    actual_clock = clock or _ManualClock()
    return MoveItForwardKinematicsAdapter(
        _FakeNode(client),
        client=client,
        monotonic=actual_clock,
        spin_once=spin_once or actual_clock.advance,
    )


def test_joint_state_conversion_uses_header_ros_stamp() -> None:
    message = JointState()
    message.header.stamp.sec = 7
    message.header.stamp.nanosec = 123
    message.name = list(JOINT_NAMES)
    message.position = [0.1] * len(JOINT_NAMES)
    message.velocity = []

    sample = timed_joint_sample_from_message(message)

    assert sample.stamp_ns == 7_000_000_123
    assert sample.joint_names == JOINT_NAMES
    assert sample.velocities_rad_s is None


def test_joint_state_conversion_validates_velocity_shape() -> None:
    message = JointState()
    message.header.stamp.sec = 1
    message.name = list(JOINT_NAMES)
    message.position = [0.1] * len(JOINT_NAMES)
    message.velocity = [0.2] * len(JOINT_NAMES)

    assert timed_joint_sample_from_message(
        message
    ).velocities_rad_s == tuple(message.velocity)

    message.velocity = [0.2]
    with pytest.raises(ValueError, match='velocities_rad_s'):
        timed_joint_sample_from_message(message)


def test_ros_stamp_validation() -> None:
    message = JointState()
    message.header.stamp.sec = 2
    message.header.stamp.nanosec = 3
    assert ros_stamp_to_nanoseconds(message.header.stamp) == 2_000_000_003

    class InvalidStamp:
        sec = -1
        nanosec = 0

    with pytest.raises(ValueError, match='non-negative'):
        ros_stamp_to_nanoseconds(InvalidStamp())


def test_adapter_sends_full_absolute_robot_state() -> None:
    future = _FakeFuture(_response())
    client = _FakeClient(future)
    node = _FakeNode(client)
    adapter = MoveItForwardKinematicsAdapter(node)
    input_names = ('optional_wheel_joint',) + tuple(reversed(JOINT_NAMES))
    interpolation = _interpolation(joint_names=input_names)

    transform = adapter.compute(
        interpolation,
        CALIBRATION_LINK,
        timeout_sec=0.5,
    )

    assert node.created_service_type is GetPositionFK
    assert node.created_service_name == COMPUTE_FK_SERVICE_NAME
    assert len(client.requests) == 1
    request = client.requests[0]
    assert request.header.frame_id == BASE_FRAME
    assert request.header.stamp.sec == 12
    assert request.header.stamp.nanosec == 345_678_901
    assert request.fk_link_names == [CALIBRATION_LINK]
    assert request.robot_state.is_diff is False
    assert tuple(request.robot_state.joint_state.name) == JOINT_NAMES
    positions_by_name = dict(
        zip(
            interpolation.sample.joint_names,
            interpolation.sample.positions_rad,
            strict=True,
        )
    )
    velocities_by_name = dict(
        zip(
            interpolation.sample.joint_names,
            interpolation.sample.velocities_rad_s,
            strict=True,
        )
    )
    assert tuple(request.robot_state.joint_state.position) == tuple(
        positions_by_name[name] for name in JOINT_NAMES
    )
    assert tuple(request.robot_state.joint_state.velocity) == tuple(
        velocities_by_name[name] for name in JOINT_NAMES
    )
    assert len(request.robot_state.joint_state.name) == 12
    assert transform.parent_frame == BASE_FRAME
    assert transform.child_frame == CALIBRATION_LINK
    assert transform.translation_m == pytest.approx((0.4, -0.2, 0.7))


def test_adapter_rejects_partial_feedback_and_wrong_link() -> None:
    client = _FakeClient(_FakeFuture(_response()))
    adapter = _adapter(client)
    partial = tuple(
        name for name in JOINT_NAMES if name != 'right_gripper_joint'
    )

    with pytest.raises(ValueError, match='missing required'):
        adapter.compute(
            _interpolation(joint_names=partial),
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )
    with pytest.raises(ValueError, match='left_gripper_frame'):
        adapter.compute(
            _interpolation(),
            'right_gripper_frame',
            timeout_sec=0.5,
        )
    assert client.requests == []


def test_adapter_rejects_joint_data_without_interpolation_provenance() -> None:
    client = _FakeClient(_FakeFuture(_response()))
    adapter = _adapter(client)

    with pytest.raises(ValueError, match='feedback InterpolatedJointState'):
        adapter.compute(
            _interpolation().sample,
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )

    assert client.requests == []


def test_service_availability_timeout_uses_injected_monotonic_clock() -> None:
    clock = _ManualClock(value=50.0)
    client = _FakeClient(_FakeFuture(_response()), ready=False)
    adapter = _adapter(client, clock=clock)

    with pytest.raises(ForwardKinematicsError) as caught:
        adapter.compute(
            _interpolation(stamp_ns=9_000_000_000_000),
            CALIBRATION_LINK,
            timeout_sec=0.03,
        )

    assert caught.value.failure is (
        ForwardKinematicsFailure.SERVICE_UNAVAILABLE
    )
    assert clock.value == pytest.approx(50.03)
    assert client.requests == []


def test_response_timeout_cancels_local_future_on_monotonic_deadline() -> None:
    clock = _ManualClock()
    future = _FakeFuture(_response(), done=False)
    client = _FakeClient(future)
    adapter = _adapter(client, clock=clock)

    with pytest.raises(ForwardKinematicsError) as caught:
        adapter.compute(
            _interpolation(),
            CALIBRATION_LINK,
            timeout_sec=0.025,
        )

    assert caught.value.failure is ForwardKinematicsFailure.TIMEOUT
    assert future.cancelled


def test_moveit_error_code_is_preserved() -> None:
    response = _response()
    response.error_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
    adapter = _adapter(_FakeClient(_FakeFuture(response)))

    with pytest.raises(ForwardKinematicsError) as caught:
        adapter.compute(
            _interpolation(),
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )

    assert caught.value.failure is ForwardKinematicsFailure.MOVEIT_ERROR
    assert caught.value.moveit_error_code == (
        MoveItErrorCodes.INVALID_ROBOT_STATE
    )


@pytest.mark.parametrize(
    'mutate_response',
    [
        lambda response: setattr(
            response,
            'fk_link_names',
            ['right_gripper_frame'],
        ),
        lambda response: setattr(
            response.pose_stamped[0].header,
            'frame_id',
            'odom',
        ),
        lambda response: setattr(response, 'pose_stamped', []),
    ],
)
def test_link_frame_and_pose_count_are_validated(
    mutate_response: Callable[[GetPositionFK.Response], None],
) -> None:
    response = _response()
    mutate_response(response)
    adapter = _adapter(_FakeClient(_FakeFuture(response)))

    with pytest.raises(ForwardKinematicsError) as caught:
        adapter.compute(
            _interpolation(),
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )

    assert caught.value.failure is ForwardKinematicsFailure.INVALID_RESPONSE


def test_invalid_pose_and_failed_future_are_mapped() -> None:
    invalid_response = _response()
    invalid_response.pose_stamped[0].pose.orientation.w = 0.0
    adapter = _adapter(_FakeClient(_FakeFuture(invalid_response)))
    with pytest.raises(ForwardKinematicsError) as invalid:
        adapter.compute(
            _interpolation(),
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )
    assert invalid.value.failure is (
        ForwardKinematicsFailure.INVALID_RESPONSE
    )

    failed = _FakeFuture(None, error=RuntimeError('transport failed'))
    adapter = _adapter(_FakeClient(failed))
    with pytest.raises(ForwardKinematicsError) as service_failure:
        adapter.compute(
            _interpolation(),
            CALIBRATION_LINK,
            timeout_sec=0.5,
        )
    assert service_failure.value.failure is (
        ForwardKinematicsFailure.SERVICE_CALL_FAILED
    )
