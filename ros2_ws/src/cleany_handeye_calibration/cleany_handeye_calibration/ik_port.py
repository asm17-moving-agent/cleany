"""MoveIt position-IK and state-validity ROS adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Any, Callable

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy

from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
    DualArmJointContract,
)
from cleany_handeye_calibration.models import (
    IkResult,
    JointPose,
    PositionTarget,
    TimedJointSample,
)
from cleany_handeye_calibration.motion_config import (
    CALIBRATION_BASE_FRAME,
    CALIBRATION_PLANNING_GROUP,
    CALIBRATION_TIP_LINK,
    NANOSECONDS_PER_SECOND,
    MujocoMotionConfig,
    ValidatedCurrentState,
    validate_calibration_scope,
    validate_dual_arm_current_state,
)


COMPUTE_IK_SERVICE_NAME = '/compute_ik'
CHECK_STATE_VALIDITY_SERVICE_NAME = '/check_state_validity'
_MAX_ROS_TIME_SECONDS = 2_147_483_647


def _assign_stamp(stamp: Any, stamp_ns: int) -> None:
    seconds, nanoseconds = divmod(stamp_ns, NANOSECONDS_PER_SECOND)
    if seconds > _MAX_ROS_TIME_SECONDS:
        raise ValueError('stamp exceeds the ROS Time sec range')
    stamp.sec = seconds
    stamp.nanosec = nanoseconds


def _assign_duration(duration: Any, duration_sec: float) -> None:
    duration_ns = round(duration_sec * NANOSECONDS_PER_SECOND)
    seconds, nanoseconds = divmod(
        duration_ns,
        NANOSECONDS_PER_SECOND,
    )
    if seconds > _MAX_ROS_TIME_SECONDS:
        raise ValueError('duration exceeds the ROS Duration sec range')
    duration.sec = seconds
    duration.nanosec = nanoseconds


def _left_pose_positions(pose: JointPose) -> dict[str, float]:
    if not isinstance(pose, JointPose):
        raise ValueError('seed/resolved pose must be a JointPose')
    if set(pose.joint_names) != set(LEFT_ARM_JOINT_NAMES):
        raise ValueError(
            'seed/resolved pose must contain exactly the five left-arm '
            'joints; right-arm scope is forbidden'
        )
    positions = dict(
        zip(pose.joint_names, pose.positions_rad, strict=True)
    )
    return {name: positions[name] for name in LEFT_ARM_JOINT_NAMES}


def _validated_feedback(
    state: ValidatedCurrentState,
    *,
    config: MujocoMotionConfig,
    joint_contract: DualArmJointContract,
) -> TimedJointSample:
    if not isinstance(state, ValidatedCurrentState):
        raise ValueError(
            'current_state must come from '
            'validate_dual_arm_current_state'
        )
    checked = validate_dual_arm_current_state(
        state.sample,
        now_stamp_ns=state.validated_at_stamp_ns,
        config=config,
        joint_contract=joint_contract,
    )
    if checked.age_ns != state.age_ns:
        raise ValueError('current_state age provenance is inconsistent')
    return checked.sample


def _full_state_vectors(
    feedback: TimedJointSample,
    left_pose: JointPose,
    joint_contract: DualArmJointContract,
) -> tuple[list[str], list[float], list[float]]:
    left_positions = _left_pose_positions(left_pose)
    feedback_positions = dict(
        zip(
            feedback.joint_names,
            feedback.positions_rad,
            strict=True,
        )
    )
    names = list(joint_contract.required_joint_names)
    positions = [
        left_positions.get(name, feedback_positions[name])
        for name in names
    ]
    if feedback.velocities_rad_s is None:
        velocities = []
    else:
        feedback_velocities = dict(
            zip(
                feedback.joint_names,
                feedback.velocities_rad_s,
                strict=True,
            )
        )
        velocities = [feedback_velocities[name] for name in names]
    return names, positions, velocities


class IkFailure(str, Enum):
    """Stable ``InverseKinematicsPort`` failure reasons."""

    SERVICE_UNAVAILABLE = 'service_unavailable'
    TIMEOUT = 'timeout'
    SERVICE_CALL_FAILED = 'service_call_failed'
    NO_IK_SOLUTION = 'no_ik_solution'
    MOVEIT_ERROR = 'moveit_error'
    INVALID_RESPONSE = 'invalid_response'


class StateValidityStatus(str, Enum):
    """Result categories for joint-limit and collision validation."""

    VALID = 'valid'
    COLLISION = 'collision'
    CONSTRAINT_VIOLATION = 'constraint_violation'
    JOINT_LIMIT_OR_STATE_INVALID = 'joint_limit_or_state_invalid'
    SERVICE_UNAVAILABLE = 'service_unavailable'
    TIMEOUT = 'timeout'
    SERVICE_CALL_FAILED = 'service_call_failed'
    INVALID_RESPONSE = 'invalid_response'


@dataclass(frozen=True, slots=True)
class StateValidityResult:
    """A checked validity outcome with collision provenance when present."""

    status: StateValidityStatus
    contact_pairs: tuple[tuple[str, str], ...] = ()
    validated_goal: ValidatedJointGoal | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, 'status', StateValidityStatus(self.status))
        pairs = tuple(tuple(pair) for pair in self.contact_pairs)
        if any(len(pair) != 2 for pair in pairs):
            raise ValueError('each contact pair must contain two bodies')
        object.__setattr__(self, 'contact_pairs', pairs)
        if self.status is StateValidityStatus.VALID:
            if not isinstance(self.validated_goal, ValidatedJointGoal):
                raise ValueError(
                    'valid state requires a ValidatedJointGoal'
                )
        elif self.validated_goal is not None:
            raise ValueError(
                'invalid or unchecked state cannot include validated_goal'
            )

    @property
    def valid(self) -> bool:
        return self.status is StateValidityStatus.VALID

    @property
    def checked(self) -> bool:
        return self.status in {
            StateValidityStatus.VALID,
            StateValidityStatus.COLLISION,
            StateValidityStatus.CONSTRAINT_VIOLATION,
            StateValidityStatus.JOINT_LIMIT_OR_STATE_INVALID,
        }


@dataclass(frozen=True, slots=True)
class ValidatedJointGoal:
    """A left joint goal proven valid against one full feedback state."""

    pose: JointPose
    checked_state_stamp_ns: int

    def __post_init__(self) -> None:
        canonical = _left_pose_positions(self.pose)
        object.__setattr__(
            self,
            'pose',
            JointPose(
                joint_names=LEFT_ARM_JOINT_NAMES,
                positions_rad=tuple(
                    canonical[name] for name in LEFT_ARM_JOINT_NAMES
                ),
            ),
        )
        if (
            isinstance(self.checked_state_stamp_ns, bool)
            or not isinstance(self.checked_state_stamp_ns, int)
            or self.checked_state_stamp_ns < 0
        ):
            raise ValueError(
                'checked_state_stamp_ns must be a non-negative integer'
            )


class _MoveItServiceAdapter:
    def __init__(
        self,
        node: Any,
        *,
        config: MujocoMotionConfig,
        client: Any,
        monotonic: Callable[[], float],
        spin_once: Callable[[float], None] | None,
        poll_interval_sec: float,
        joint_contract: DualArmJointContract,
    ) -> None:
        if node is None:
            raise ValueError('node is required')
        if not isinstance(config, MujocoMotionConfig):
            raise ValueError('config must be MujocoMotionConfig')
        if not callable(monotonic):
            raise ValueError('monotonic must be callable')
        if not isinstance(joint_contract, DualArmJointContract):
            raise ValueError(
                'joint_contract must be DualArmJointContract'
            )
        try:
            poll_interval = float(poll_interval_sec)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'poll_interval_sec must be positive and finite'
            ) from error
        if not math.isfinite(poll_interval) or poll_interval <= 0.0:
            raise ValueError(
                'poll_interval_sec must be positive and finite'
            )

        self._node = node
        self._config = config
        self._client = client
        self._monotonic = monotonic
        self._poll_interval_sec = poll_interval
        self._joint_contract = joint_contract
        if spin_once is None:
            self._spin_once = lambda timeout: rclpy.spin_once(
                self._node,
                timeout_sec=timeout,
            )
        elif callable(spin_once):
            self._spin_once = spin_once
        else:
            raise ValueError('spin_once must be callable')

    def _now(self) -> float:
        try:
            now = float(self._monotonic())
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                'monotonic clock returned a non-numeric value'
            ) from error
        if not math.isfinite(now):
            raise RuntimeError(
                'monotonic clock returned a non-finite value'
            )
        return now

    def _wait_for_service(self, deadline: float) -> bool:
        while not self._client.service_is_ready():
            remaining = deadline - self._now()
            if remaining <= 0.0:
                return False
            self._spin_once(min(self._poll_interval_sec, remaining))
        return True

    def _wait_for_future(self, future: Any, deadline: float) -> bool:
        while not future.done():
            remaining = deadline - self._now()
            if remaining <= 0.0:
                cancel = getattr(future, 'cancel', None)
                if callable(cancel):
                    cancel()
                return False
            self._spin_once(min(self._poll_interval_sec, remaining))
        return True


class MoveItPositionIKAdapter(_MoveItServiceAdapter):
    """Resolve a left position target from a full feedback RobotState seed."""

    def __init__(
        self,
        node: Any,
        *,
        config: MujocoMotionConfig,
        planning_group: str = CALIBRATION_PLANNING_GROUP,
        tip_link: str = CALIBRATION_TIP_LINK,
        client: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
        poll_interval_sec: float = 0.01,
        joint_contract: DualArmJointContract = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT
        ),
    ) -> None:
        # Scope validation intentionally precedes client creation.
        validate_calibration_scope(planning_group, tip_link)
        if node is None:
            raise ValueError('node is required')
        resolved_client = (
            node.create_client(GetPositionIK, COMPUTE_IK_SERVICE_NAME)
            if client is None
            else client
        )
        super().__init__(
            node,
            config=config,
            client=resolved_client,
            monotonic=monotonic,
            spin_once=spin_once,
            poll_interval_sec=poll_interval_sec,
            joint_contract=joint_contract,
        )
        self._planning_group = planning_group
        self._tip_link = tip_link

    @property
    def service_name(self) -> str:
        return COMPUTE_IK_SERVICE_NAME

    def solve_position(
        self,
        target: PositionTarget,
        seed: JointPose,
        *,
        current_state: ValidatedCurrentState,
    ) -> IkResult:
        """Call ``/compute_ik`` after all local safety checks pass."""

        if not isinstance(target, PositionTarget):
            raise ValueError('target must be a PositionTarget')
        if target.frame_id != CALIBRATION_BASE_FRAME:
            raise ValueError(
                'position target frame_id must be exactly '
                f'{CALIBRATION_BASE_FRAME!r}'
            )
        _left_pose_positions(seed)
        feedback = _validated_feedback(
            current_state,
            config=self._config,
            joint_contract=self._joint_contract,
        )
        request = self._build_request(target, seed, feedback)
        deadline = self._now() + self._config.stage_timeouts.ik_sec

        if not self._wait_for_service(deadline):
            return IkResult(
                success=False,
                failure_reason=IkFailure.SERVICE_UNAVAILABLE.value,
            )
        try:
            future = self._client.call_async(request)
        except Exception:
            return IkResult(
                success=False,
                failure_reason=IkFailure.SERVICE_CALL_FAILED.value,
            )
        if not self._wait_for_future(future, deadline):
            return IkResult(
                success=False,
                failure_reason=IkFailure.TIMEOUT.value,
            )
        try:
            response = future.result()
        except Exception:
            return IkResult(
                success=False,
                failure_reason=IkFailure.SERVICE_CALL_FAILED.value,
            )
        return self._result_from_response(response)

    def _build_request(
        self,
        target: PositionTarget,
        seed: JointPose,
        feedback: TimedJointSample,
    ) -> GetPositionIK.Request:
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self._planning_group
        ik_request.ik_link_name = self._tip_link
        ik_request.avoid_collisions = True
        _assign_duration(
            ik_request.timeout,
            self._config.stage_timeouts.ik_sec,
        )

        names, positions, velocities = _full_state_vectors(
            feedback,
            seed,
            self._joint_contract,
        )
        robot_state = ik_request.robot_state
        robot_state.is_diff = False
        _assign_stamp(
            robot_state.joint_state.header.stamp,
            feedback.stamp_ns,
        )
        robot_state.joint_state.name = names
        robot_state.joint_state.position = positions
        robot_state.joint_state.velocity = velocities

        pose_stamped = ik_request.pose_stamped
        _assign_stamp(pose_stamped.header.stamp, feedback.stamp_ns)
        pose_stamped.header.frame_id = target.frame_id
        (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
        ) = target.position_m
        # KDL position-only IK ignores orientation; keep a valid identity
        # quaternion instead of smuggling an orientation constraint.
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        return request

    @staticmethod
    def _result_from_response(response: Any) -> IkResult:
        if response is None:
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        try:
            error_code = int(response.error_code.val)
        except (AttributeError, TypeError, ValueError):
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        if error_code != MoveItErrorCodes.SUCCESS:
            if error_code == MoveItErrorCodes.NO_IK_SOLUTION:
                failure_reason = IkFailure.NO_IK_SOLUTION.value
            else:
                failure_reason = (
                    f'{IkFailure.MOVEIT_ERROR.value}:{error_code}'
                )
            return IkResult(
                success=False,
                failure_reason=failure_reason,
            )

        try:
            names = tuple(response.solution.joint_state.name)
            positions = tuple(response.solution.joint_state.position)
        except (AttributeError, TypeError):
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        if (
            len(names) != len(positions)
            or len(names) != len(set(names))
            or not set(LEFT_ARM_JOINT_NAMES) <= set(names)
        ):
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        try:
            solution_by_name = {
                name: float(position)
                for name, position in zip(names, positions, strict=True)
            }
        except (TypeError, ValueError):
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        if not all(
            math.isfinite(solution_by_name[name])
            for name in LEFT_ARM_JOINT_NAMES
        ):
            return IkResult(
                success=False,
                failure_reason=IkFailure.INVALID_RESPONSE.value,
            )
        return IkResult(
            success=True,
            joint_pose=JointPose(
                joint_names=LEFT_ARM_JOINT_NAMES,
                positions_rad=tuple(
                    solution_by_name[name]
                    for name in LEFT_ARM_JOINT_NAMES
                ),
            ),
        )


class MoveItStateValidityAdapter(_MoveItServiceAdapter):
    """Check resolved left joints against MoveIt limits and collision scene."""

    def __init__(
        self,
        node: Any,
        *,
        config: MujocoMotionConfig,
        planning_group: str = CALIBRATION_PLANNING_GROUP,
        tip_link: str = CALIBRATION_TIP_LINK,
        client: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
        poll_interval_sec: float = 0.01,
        joint_contract: DualArmJointContract = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT
        ),
    ) -> None:
        validate_calibration_scope(planning_group, tip_link)
        if node is None:
            raise ValueError('node is required')
        resolved_client = (
            node.create_client(
                GetStateValidity,
                CHECK_STATE_VALIDITY_SERVICE_NAME,
            )
            if client is None
            else client
        )
        super().__init__(
            node,
            config=config,
            client=resolved_client,
            monotonic=monotonic,
            spin_once=spin_once,
            poll_interval_sec=poll_interval_sec,
            joint_contract=joint_contract,
        )
        self._planning_group = planning_group

    @property
    def service_name(self) -> str:
        return CHECK_STATE_VALIDITY_SERVICE_NAME

    def validate(
        self,
        resolved_pose: JointPose,
        *,
        current_state: ValidatedCurrentState,
    ) -> StateValidityResult:
        """Evaluate the full feedback state with resolved left joints."""

        _left_pose_positions(resolved_pose)
        feedback = _validated_feedback(
            current_state,
            config=self._config,
            joint_contract=self._joint_contract,
        )
        request = self._build_request(resolved_pose, feedback)
        deadline = (
            self._now()
            + self._config.stage_timeouts.state_validity_sec
        )

        if not self._wait_for_service(deadline):
            return StateValidityResult(
                StateValidityStatus.SERVICE_UNAVAILABLE
            )
        try:
            future = self._client.call_async(request)
        except Exception:
            return StateValidityResult(
                StateValidityStatus.SERVICE_CALL_FAILED
            )
        if not self._wait_for_future(future, deadline):
            return StateValidityResult(StateValidityStatus.TIMEOUT)
        try:
            response = future.result()
        except Exception:
            return StateValidityResult(
                StateValidityStatus.SERVICE_CALL_FAILED
            )
        return self._result_from_response(
            response,
            resolved_pose,
            feedback.stamp_ns,
        )

    check = validate

    def _build_request(
        self,
        resolved_pose: JointPose,
        feedback: TimedJointSample,
    ) -> GetStateValidity.Request:
        request = GetStateValidity.Request()
        request.group_name = self._planning_group
        names, positions, velocities = _full_state_vectors(
            feedback,
            resolved_pose,
            self._joint_contract,
        )
        request.robot_state.is_diff = False
        _assign_stamp(
            request.robot_state.joint_state.header.stamp,
            feedback.stamp_ns,
        )
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        request.robot_state.joint_state.velocity = velocities
        return request

    @staticmethod
    def _result_from_response(
        response: Any,
        resolved_pose: JointPose,
        feedback_stamp_ns: int,
    ) -> StateValidityResult:
        if response is None or not isinstance(
            getattr(response, 'valid', None),
            bool,
        ):
            return StateValidityResult(
                StateValidityStatus.INVALID_RESPONSE
            )
        if response.valid:
            return StateValidityResult(
                StateValidityStatus.VALID,
                validated_goal=ValidatedJointGoal(
                    pose=resolved_pose,
                    checked_state_stamp_ns=feedback_stamp_ns,
                ),
            )

        try:
            contacts = tuple(
                (
                    str(contact.contact_body_1),
                    str(contact.contact_body_2),
                )
                for contact in response.contacts
            )
            constraint_results = tuple(response.constraint_result)
        except (AttributeError, TypeError):
            return StateValidityResult(
                StateValidityStatus.INVALID_RESPONSE
            )
        if contacts:
            return StateValidityResult(
                StateValidityStatus.COLLISION,
                contact_pairs=contacts,
            )
        try:
            constraint_failed = any(
                not bool(result.result) for result in constraint_results
            )
        except AttributeError:
            return StateValidityResult(
                StateValidityStatus.INVALID_RESPONSE
            )
        if constraint_failed:
            return StateValidityResult(
                StateValidityStatus.CONSTRAINT_VIOLATION
            )
        return StateValidityResult(
            StateValidityStatus.JOINT_LIMIT_OR_STATE_INVALID
        )


InverseKinematicsPort = MoveItPositionIKAdapter
