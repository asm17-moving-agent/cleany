"""ROS adapters for timestamped joint feedback and MoveIt feedback FK."""

from __future__ import annotations

from enum import Enum
import math
import time
from typing import Any, Callable

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK
import rclpy

from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    DualArmJointContract,
    InterpolatedJointState,
)
from cleany_handeye_calibration.models import TimedJointSample
from cleany_handeye_calibration.transforms import RigidTransform


COMPUTE_FK_SERVICE_NAME = '/compute_fk'
BASE_FRAME = 'base_link'
CALIBRATION_LINK = 'left_gripper_frame'
_NANOSECONDS_PER_SECOND = 1_000_000_000
_MAX_ROS_TIME_SECONDS = 2_147_483_647


def ros_stamp_to_nanoseconds(stamp: Any) -> int:
    """Convert a ROS builtin time value without consulting a wall clock."""

    try:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
    except AttributeError as error:
        raise ValueError('stamp must provide sec and nanosec') from error
    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, int)
        or seconds < 0
    ):
        raise ValueError('stamp.sec must be a non-negative integer')
    if (
        isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or not 0 <= nanoseconds < _NANOSECONDS_PER_SECOND
    ):
        raise ValueError(
            'stamp.nanosec must be an integer in [0, 1000000000)'
        )
    return seconds * _NANOSECONDS_PER_SECOND + nanoseconds


def timed_joint_sample_from_message(message: Any) -> TimedJointSample:
    """Convert a ``JointState`` using its ROS header stamp as data time."""

    try:
        stamp = message.header.stamp
        joint_names = message.name
        positions = message.position
        message_velocities = message.velocity
    except AttributeError as error:
        raise ValueError(
            'message must have JointState header/name/position/velocity'
        ) from error
    velocities = (
        None if len(message_velocities) == 0 else message_velocities
    )
    return TimedJointSample(
        stamp_ns=ros_stamp_to_nanoseconds(stamp),
        joint_names=joint_names,
        positions_rad=positions,
        velocities_rad_s=velocities,
    )


class ForwardKinematicsFailure(str, Enum):
    """Stable failure categories for the synchronous MoveIt boundary."""

    SERVICE_UNAVAILABLE = 'service_unavailable'
    TIMEOUT = 'timeout'
    SERVICE_CALL_FAILED = 'service_call_failed'
    MOVEIT_ERROR = 'moveit_error'
    INVALID_RESPONSE = 'invalid_response'


class ForwardKinematicsError(RuntimeError):
    """A MoveIt service or response contract failure."""

    def __init__(
        self,
        failure: ForwardKinematicsFailure,
        message: str,
        *,
        moveit_error_code: int | None = None,
    ) -> None:
        self.failure = ForwardKinematicsFailure(failure)
        self.moveit_error_code = moveit_error_code
        super().__init__(message)


class MoveItForwardKinematicsAdapter:
    """Compute ``base_link_T_left_gripper_frame`` from feedback joints.

    The request always contains the complete interpolated state and explicitly
    sets ``RobotState.is_diff`` false.  ROS header stamps remain data time;
    the bounded service wait uses only the injected monotonic wall clock.
    ``client`` and ``spin_once`` are injectable so request/response behavior is
    testable without starting a ROS graph.
    """

    def __init__(
        self,
        node: Any,
        *,
        client: Any | None = None,
        joint_contract: DualArmJointContract = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT
        ),
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
        poll_interval_sec: float = 0.01,
    ) -> None:
        if node is None:
            raise ValueError('node is required')
        if not isinstance(joint_contract, DualArmJointContract):
            raise ValueError(
                'joint_contract must be a DualArmJointContract'
            )
        if not callable(monotonic):
            raise ValueError('monotonic must be callable')
        try:
            poll_interval = float(poll_interval_sec)
        except (TypeError, ValueError) as error:
            raise ValueError('poll_interval_sec must be numeric') from error
        if not math.isfinite(poll_interval) or poll_interval <= 0.0:
            raise ValueError(
                'poll_interval_sec must be positive and finite'
            )

        self._node = node
        self._client = (
            node.create_client(GetPositionFK, COMPUTE_FK_SERVICE_NAME)
            if client is None
            else client
        )
        self._joint_contract = joint_contract
        self._monotonic = monotonic
        self._poll_interval_sec = poll_interval
        if spin_once is None:
            self._spin_once = lambda timeout: rclpy.spin_once(
                self._node,
                timeout_sec=timeout,
            )
        elif callable(spin_once):
            self._spin_once = spin_once
        else:
            raise ValueError('spin_once must be callable')

    @property
    def service_name(self) -> str:
        return COMPUTE_FK_SERVICE_NAME

    def compute(
        self,
        joints: InterpolatedJointState,
        link_name: str,
        *,
        timeout_sec: float,
    ) -> RigidTransform:
        """Call MoveIt FK with a full state interpolated at image data time."""

        if not isinstance(joints, InterpolatedJointState):
            raise ValueError(
                'joints must be feedback InterpolatedJointState provenance'
            )
        if link_name != CALIBRATION_LINK:
            raise ValueError(
                f'link_name must be {CALIBRATION_LINK!r}, got '
                f'{link_name!r}'
            )
        self._joint_contract.validate(joints.sample)
        timeout = self._validated_timeout(timeout_sec)
        deadline = self._monotonic_now() + timeout

        while not self._client.service_is_ready():
            remaining = deadline - self._monotonic_now()
            if remaining <= 0.0:
                raise ForwardKinematicsError(
                    ForwardKinematicsFailure.SERVICE_UNAVAILABLE,
                    f'{COMPUTE_FK_SERVICE_NAME} was not ready before timeout',
                )
            self._spin_once(min(self._poll_interval_sec, remaining))

        request = self._build_request(joints, link_name)
        try:
            future = self._client.call_async(request)
        except Exception as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.SERVICE_CALL_FAILED,
                f'{COMPUTE_FK_SERVICE_NAME} call failed: {error}',
            ) from error

        while not future.done():
            remaining = deadline - self._monotonic_now()
            if remaining <= 0.0:
                cancel = getattr(future, 'cancel', None)
                if callable(cancel):
                    cancel()
                raise ForwardKinematicsError(
                    ForwardKinematicsFailure.TIMEOUT,
                    f'{COMPUTE_FK_SERVICE_NAME} response timed out',
                )
            self._spin_once(min(self._poll_interval_sec, remaining))

        try:
            response = future.result()
        except Exception as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.SERVICE_CALL_FAILED,
                f'{COMPUTE_FK_SERVICE_NAME} future failed: {error}',
            ) from error
        if response is None:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.SERVICE_CALL_FAILED,
                f'{COMPUTE_FK_SERVICE_NAME} returned no response',
            )
        return self._transform_from_response(response, link_name)

    @staticmethod
    def _validated_timeout(timeout_sec: float) -> float:
        try:
            timeout = float(timeout_sec)
        except (TypeError, ValueError) as error:
            raise ValueError('timeout_sec must be numeric') from error
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout_sec must be positive and finite')
        return timeout

    def _monotonic_now(self) -> float:
        try:
            now = float(self._monotonic())
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                'monotonic clock returned a non-numeric value'
            ) from error
        if not math.isfinite(now):
            raise RuntimeError('monotonic clock returned a non-finite value')
        return now

    def _build_request(
        self,
        joints: InterpolatedJointState,
        link_name: str,
    ) -> GetPositionFK.Request:
        request = GetPositionFK.Request()
        request.header.frame_id = BASE_FRAME
        MoveItForwardKinematicsAdapter._assign_stamp(
            request.header.stamp,
            joints.image_stamp_ns,
        )
        request.fk_link_names = [link_name]
        request.robot_state.is_diff = False

        joint_state = request.robot_state.joint_state
        MoveItForwardKinematicsAdapter._assign_stamp(
            joint_state.header.stamp,
            joints.image_stamp_ns,
        )
        positions_by_name = dict(
            zip(
                joints.sample.joint_names,
                joints.sample.positions_rad,
                strict=True,
            )
        )
        state_joint_names = self._joint_contract.required_joint_names
        joint_state.name = list(state_joint_names)
        joint_state.position = [
            positions_by_name[name] for name in state_joint_names
        ]
        if joints.sample.velocities_rad_s is not None:
            velocities_by_name = dict(
                zip(
                    joints.sample.joint_names,
                    joints.sample.velocities_rad_s,
                    strict=True,
                )
            )
            joint_state.velocity = [
                velocities_by_name[name] for name in state_joint_names
            ]
        return request

    @staticmethod
    def _assign_stamp(stamp: Any, stamp_ns: int) -> None:
        seconds, nanoseconds = divmod(
            stamp_ns,
            _NANOSECONDS_PER_SECOND,
        )
        if seconds > _MAX_ROS_TIME_SECONDS:
            raise ValueError('stamp_ns exceeds the ROS Time sec range')
        stamp.sec = seconds
        stamp.nanosec = nanoseconds

    @staticmethod
    def _transform_from_response(
        response: Any,
        link_name: str,
    ) -> RigidTransform:
        try:
            error_code = int(response.error_code.val)
        except (AttributeError, TypeError, ValueError) as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response does not contain a valid MoveIt error code',
            ) from error
        if error_code != MoveItErrorCodes.SUCCESS:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.MOVEIT_ERROR,
                f'MoveIt FK failed with error code {error_code}',
                moveit_error_code=error_code,
            )

        try:
            returned_links = list(response.fk_link_names)
            poses = list(response.pose_stamped)
        except (AttributeError, TypeError) as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response does not contain link and pose sequences',
            ) from error
        if returned_links != [link_name]:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response link mismatch: '
                f'expected {[link_name]!r}, got {returned_links!r}',
            )
        if len(poses) != 1:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response must contain exactly one pose, got '
                f'{len(poses)}',
            )

        pose_stamped = poses[0]
        try:
            frame_id = pose_stamped.header.frame_id
            pose = pose_stamped.pose
            translation = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        except AttributeError as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response pose is incomplete',
            ) from error
        if frame_id != BASE_FRAME:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                'FK response frame mismatch: '
                f'expected {BASE_FRAME!r}, got {frame_id!r}',
            )

        try:
            return RigidTransform.from_quaternion_xyzw(
                parent_frame=BASE_FRAME,
                child_frame=link_name,
                translation_m=translation,
                quaternion_xyzw=quaternion,
            )
        except (TypeError, ValueError) as error:
            raise ForwardKinematicsError(
                ForwardKinematicsFailure.INVALID_RESPONSE,
                f'FK response pose is not a rigid transform: {error}',
            ) from error
