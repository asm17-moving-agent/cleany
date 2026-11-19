"""Separate MoveIt PLAN and EXECUTE action adapters for calibration motion."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient

from cleany_handeye_calibration.ik_port import ValidatedJointGoal
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
    DualArmJointContract,
)
from cleany_handeye_calibration.models import JointPose
from cleany_handeye_calibration.motion_config import (
    CALIBRATION_PLANNING_GROUP,
    CALIBRATION_TIP_LINK,
    MujocoMotionConfig,
    ValidatedCurrentState,
    validate_calibration_scope,
    validate_dual_arm_current_state,
)


MOVE_GROUP_ACTION_NAME = '/move_action'
EXECUTE_TRAJECTORY_ACTION_NAME = '/execute_trajectory'


def _canonical_left_pose(pose: JointPose) -> JointPose:
    if not isinstance(pose, JointPose):
        raise ValueError('pose must be a JointPose')
    if set(pose.joint_names) != set(LEFT_ARM_JOINT_NAMES):
        raise ValueError(
            'motion pose must contain exactly the five left-arm joints; '
            'right-arm motion is forbidden'
        )
    positions = dict(
        zip(pose.joint_names, pose.positions_rad, strict=True)
    )
    return JointPose(
        joint_names=LEFT_ARM_JOINT_NAMES,
        positions_rad=tuple(
            positions[name] for name in LEFT_ARM_JOINT_NAMES
        ),
    )


class MotionStatus(str, Enum):
    """Stable action transport, status, and MoveIt result categories."""

    SUCCESS = 'success'
    ACTION_UNAVAILABLE = 'action_unavailable'
    TIMEOUT = 'timeout'
    GOAL_REJECTED = 'goal_rejected'
    CANCELED = 'canceled'
    ABORTED = 'aborted'
    MOVEIT_ERROR = 'moveit_error'
    INVALID_RESPONSE = 'invalid_response'
    TRANSPORT_ERROR = 'transport_error'


@dataclass(frozen=True, slots=True)
class PlannedMotion:
    """A plan-only trajectory tied to the resolved joint-space goal."""

    validated_goal: ValidatedJointGoal
    trajectory: Any
    planning_time_sec: float

    @property
    def joint_goal(self) -> JointPose:
        return self.validated_goal.pose


@dataclass(frozen=True, slots=True)
class PlanResult:
    status: MotionStatus
    planned_motion: PlannedMotion | None = None
    action_status: int | None = None
    moveit_error_code: int | None = None
    cancel_requested: bool = False
    cancel_confirmed: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, 'status', MotionStatus(self.status))
        if self.status is MotionStatus.SUCCESS:
            if self.planned_motion is None:
                raise ValueError(
                    'successful plan requires planned_motion'
                )
        elif self.planned_motion is not None:
            raise ValueError('failed plan must not include planned_motion')
        if self.cancel_confirmed and not self.cancel_requested:
            raise ValueError(
                'cancel_confirmed requires cancel_requested'
            )

    @property
    def success(self) -> bool:
        return self.status is MotionStatus.SUCCESS


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    status: MotionStatus
    action_status: int | None = None
    moveit_error_code: int | None = None
    cancel_requested: bool = False
    cancel_confirmed: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, 'status', MotionStatus(self.status))
        if self.cancel_confirmed and not self.cancel_requested:
            raise ValueError(
                'cancel_confirmed requires cancel_requested'
            )

    @property
    def success(self) -> bool:
        return self.status is MotionStatus.SUCCESS


@dataclass(frozen=True, slots=True)
class MotionResult:
    """PLAN/EXECUTE result; feedback settle remains a separate gate."""

    plan_result: PlanResult
    execution_result: ExecutionResult | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.plan_result, PlanResult):
            raise ValueError('plan_result must be a PlanResult')
        if self.execution_result is not None and not isinstance(
            self.execution_result,
            ExecutionResult,
        ):
            raise ValueError(
                'execution_result must be an ExecutionResult'
            )
        if not self.plan_result.success and self.execution_result is not None:
            raise ValueError('a failed plan cannot have an execution result')

    @property
    def success(self) -> bool:
        """Whether actions succeeded, not whether feedback has settled."""

        return (
            self.plan_result.success
            and self.execution_result is not None
            and self.execution_result.success
        )

    @property
    def ready_for_sample(self) -> bool:
        """Action success alone never authorizes image acquisition."""

        return False


@dataclass(frozen=True, slots=True)
class _ActionOutcome:
    status: MotionStatus
    result: Any = None
    action_status: int | None = None
    moveit_error_code: int | None = None
    cancel_requested: bool = False
    cancel_confirmed: bool = False


class MoveItMotionAdapter:
    """Plan with ``MoveGroup(plan_only=True)`` then execute explicitly."""

    def __init__(
        self,
        node: Any,
        *,
        config: MujocoMotionConfig,
        planning_group: str = CALIBRATION_PLANNING_GROUP,
        tip_link: str = CALIBRATION_TIP_LINK,
        plan_client: Any | None = None,
        execute_client: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
        poll_interval_sec: float = 0.01,
        joint_contract: DualArmJointContract = (
            DEFAULT_DUAL_ARM_JOINT_CONTRACT
        ),
    ) -> None:
        # Reject right scope before constructing either ROS ActionClient.
        validate_calibration_scope(planning_group, tip_link)
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
        self._planning_group = planning_group
        self._joint_contract = joint_contract
        self._plan_client = (
            ActionClient(node, MoveGroup, MOVE_GROUP_ACTION_NAME)
            if plan_client is None
            else plan_client
        )
        self._execute_client = (
            ActionClient(
                node,
                ExecuteTrajectory,
                EXECUTE_TRAJECTORY_ACTION_NAME,
            )
            if execute_client is None
            else execute_client
        )
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
    def plan_action_name(self) -> str:
        return MOVE_GROUP_ACTION_NAME

    @property
    def execute_action_name(self) -> str:
        return EXECUTE_TRAJECTORY_ACTION_NAME

    def plan(
        self,
        validated_goal: ValidatedJointGoal,
        *,
        current_state: ValidatedCurrentState,
    ) -> PlanResult:
        """Create a joint goal and obtain a trajectory without executing."""

        if not isinstance(validated_goal, ValidatedJointGoal):
            raise ValueError(
                'planning requires a ValidatedJointGoal from '
                '/check_state_validity'
            )
        feedback_stamp_ns = self._validate_current_state(current_state)
        if feedback_stamp_ns < validated_goal.checked_state_stamp_ns:
            raise ValueError(
                'current state predates the state-validity proof'
            )
        canonical_pose = _canonical_left_pose(validated_goal.pose)
        goal = self._build_plan_goal(canonical_pose)
        outcome = self._run_action(
            self._plan_client,
            goal,
            timeout_sec=self._config.stage_timeouts.plan_sec,
        )
        if outcome.status is not MotionStatus.SUCCESS:
            return PlanResult(
                status=outcome.status,
                action_status=outcome.action_status,
                moveit_error_code=outcome.moveit_error_code,
                cancel_requested=outcome.cancel_requested,
                cancel_confirmed=outcome.cancel_confirmed,
            )

        planned_motion = self._planned_motion_from_result(
            outcome.result,
            ValidatedJointGoal(
                pose=canonical_pose,
                checked_state_stamp_ns=(
                    validated_goal.checked_state_stamp_ns
                ),
            ),
        )
        if planned_motion is None:
            return PlanResult(
                status=MotionStatus.INVALID_RESPONSE,
                action_status=outcome.action_status,
                moveit_error_code=outcome.moveit_error_code,
            )
        return PlanResult(
            status=MotionStatus.SUCCESS,
            planned_motion=planned_motion,
            action_status=outcome.action_status,
            moveit_error_code=outcome.moveit_error_code,
        )

    def execute(
        self,
        planned_motion: PlannedMotion,
        *,
        current_state: ValidatedCurrentState,
    ) -> ExecutionResult:
        """Execute only a trajectory returned by the separate PLAN stage."""

        if not isinstance(planned_motion, PlannedMotion):
            raise ValueError('planned_motion must be a PlannedMotion')
        feedback_stamp_ns = self._validate_current_state(current_state)
        if (
            feedback_stamp_ns
            < planned_motion.validated_goal.checked_state_stamp_ns
        ):
            raise ValueError(
                'current state predates the state-validity proof'
            )
        _canonical_left_pose(planned_motion.joint_goal)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = planned_motion.trajectory
        outcome = self._run_action(
            self._execute_client,
            goal,
            timeout_sec=self._config.stage_timeouts.execute_sec,
        )
        return ExecutionResult(
            status=outcome.status,
            action_status=outcome.action_status,
            moveit_error_code=outcome.moveit_error_code,
            cancel_requested=outcome.cancel_requested,
            cancel_confirmed=outcome.cancel_confirmed,
        )

    def move_to_joint_pose(
        self,
        validated_goal: ValidatedJointGoal,
        *,
        current_state: ValidatedCurrentState,
    ) -> MotionResult:
        """Run PLAN then EXECUTE, never conflating either with settle."""

        plan_result = self.plan(
            validated_goal,
            current_state=current_state,
        )
        if not plan_result.success:
            return MotionResult(plan_result=plan_result)
        assert plan_result.planned_motion is not None
        execution_result = self.execute(
            plan_result.planned_motion,
            current_state=current_state,
        )
        return MotionResult(
            plan_result=plan_result,
            execution_result=execution_result,
        )

    def _build_plan_goal(self, pose: JointPose) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self._planning_group
        request.num_planning_attempts = self._config.planning_attempts
        request.allowed_planning_time = (
            self._config.stage_timeouts.plan_sec
        )
        request.max_velocity_scaling_factor = (
            self._config.max_velocity_scaling_factor
        )
        request.max_acceleration_scaling_factor = (
            self._config.max_acceleration_scaling_factor
        )
        request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = 'left_calibration_resolved_joint_goal'
        for joint_name, position in zip(
            pose.joint_names,
            pose.positions_rad,
            strict=True,
        ):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = (
                self._config.controller_goal_tolerance_rad
            )
            constraint.tolerance_below = (
                self._config.controller_goal_tolerance_rad
            )
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]

        options = goal.planning_options
        options.plan_only = True
        options.look_around = False
        options.replan = False
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _run_action(
        self,
        client: Any,
        goal: Any,
        *,
        timeout_sec: float,
    ) -> _ActionOutcome:
        deadline = self._now() + timeout_sec
        if not self._wait_for_server(client, deadline):
            return _ActionOutcome(MotionStatus.ACTION_UNAVAILABLE)

        try:
            goal_future = client.send_goal_async(goal)
        except Exception:
            return _ActionOutcome(MotionStatus.TRANSPORT_ERROR)
        if not self._wait_for_future(goal_future, deadline):
            cancel = getattr(goal_future, 'cancel', None)
            if callable(cancel):
                cancel()
            return _ActionOutcome(MotionStatus.TIMEOUT)
        try:
            goal_handle = goal_future.result()
        except Exception:
            return _ActionOutcome(MotionStatus.TRANSPORT_ERROR)
        if goal_handle is None or not isinstance(
            getattr(goal_handle, 'accepted', None),
            bool,
        ):
            return _ActionOutcome(MotionStatus.INVALID_RESPONSE)
        if not goal_handle.accepted:
            return _ActionOutcome(MotionStatus.GOAL_REJECTED)

        try:
            result_future = goal_handle.get_result_async()
        except Exception:
            return _ActionOutcome(MotionStatus.TRANSPORT_ERROR)
        if not self._wait_for_future(result_future, deadline):
            requested, confirmed = self._cancel_goal(goal_handle)
            return _ActionOutcome(
                MotionStatus.TIMEOUT,
                cancel_requested=requested,
                cancel_confirmed=confirmed,
            )
        try:
            wrapped_result = result_future.result()
        except Exception:
            return _ActionOutcome(MotionStatus.TRANSPORT_ERROR)
        return self._map_wrapped_result(wrapped_result)

    def _map_wrapped_result(self, wrapped_result: Any) -> _ActionOutcome:
        if wrapped_result is None:
            return _ActionOutcome(MotionStatus.INVALID_RESPONSE)
        try:
            action_status = int(wrapped_result.status)
        except (AttributeError, TypeError, ValueError):
            return _ActionOutcome(MotionStatus.INVALID_RESPONSE)
        result = getattr(wrapped_result, 'result', None)
        try:
            moveit_error_code = int(result.error_code.val)
        except (AttributeError, TypeError, ValueError):
            moveit_error_code = None
        if action_status == GoalStatus.STATUS_CANCELED:
            return _ActionOutcome(
                MotionStatus.CANCELED,
                result=result,
                action_status=action_status,
                moveit_error_code=moveit_error_code,
            )
        if action_status == GoalStatus.STATUS_ABORTED:
            return _ActionOutcome(
                MotionStatus.ABORTED,
                result=result,
                action_status=action_status,
                moveit_error_code=moveit_error_code,
            )
        if action_status != GoalStatus.STATUS_SUCCEEDED:
            return _ActionOutcome(
                MotionStatus.INVALID_RESPONSE,
                result=result,
                action_status=action_status,
                moveit_error_code=moveit_error_code,
            )

        if moveit_error_code is None:
            return _ActionOutcome(
                MotionStatus.INVALID_RESPONSE,
                action_status=action_status,
            )
        if moveit_error_code != MoveItErrorCodes.SUCCESS:
            return _ActionOutcome(
                MotionStatus.MOVEIT_ERROR,
                result=result,
                action_status=action_status,
                moveit_error_code=moveit_error_code,
            )
        return _ActionOutcome(
            MotionStatus.SUCCESS,
            result=result,
            action_status=action_status,
            moveit_error_code=moveit_error_code,
        )

    def _cancel_goal(self, goal_handle: Any) -> tuple[bool, bool]:
        goal_uuid = self._goal_uuid(goal_handle)
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return True, False
        cancel_deadline = (
            self._now() + self._config.stage_timeouts.cancel_sec
        )
        if not self._wait_for_future(cancel_future, cancel_deadline):
            return True, False
        try:
            response = cancel_future.result()
            return_code = int(response.return_code)
            canceling_goal_uuids = tuple(
                bytes(goal_info.goal_id.uuid)
                for goal_info in response.goals_canceling
            )
        except (AttributeError, TypeError, ValueError):
            return True, False
        return (
            True,
            return_code == CancelGoal.Response.ERROR_NONE
            and goal_uuid is not None
            and goal_uuid in canceling_goal_uuids,
        )

    @staticmethod
    def _goal_uuid(goal_handle: Any) -> bytes | None:
        goal_id = getattr(goal_handle, 'goal_id', None)
        if goal_id is None:
            goal_info = getattr(goal_handle, 'goal_info', None)
            goal_id = getattr(goal_info, 'goal_id', None)
        try:
            goal_uuid = bytes(goal_id.uuid)
        except (AttributeError, TypeError, ValueError):
            return None
        if len(goal_uuid) != 16:
            return None
        return goal_uuid

    def _wait_for_server(self, client: Any, deadline: float) -> bool:
        while not self._server_is_ready(client):
            remaining = deadline - self._now()
            if remaining <= 0.0:
                return False
            self._spin_once(min(self._poll_interval_sec, remaining))
        return True

    @staticmethod
    def _server_is_ready(client: Any) -> bool:
        ready = getattr(client, 'server_is_ready', None)
        if callable(ready):
            return bool(ready())
        wait = getattr(client, 'wait_for_server', None)
        if callable(wait):
            return bool(wait(timeout_sec=0.0))
        raise ValueError('action client does not expose server readiness')

    def _wait_for_future(self, future: Any, deadline: float) -> bool:
        while not future.done():
            remaining = deadline - self._now()
            if remaining <= 0.0:
                return False
            self._spin_once(min(self._poll_interval_sec, remaining))
        return True

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

    def _planned_motion_from_result(
        self,
        result: Any,
        validated_goal: ValidatedJointGoal,
    ) -> PlannedMotion | None:
        joint_goal = validated_goal.pose
        try:
            trajectory = result.planned_trajectory
            joint_trajectory = trajectory.joint_trajectory
            trajectory_names = tuple(joint_trajectory.joint_names)
            points = tuple(joint_trajectory.points)
            planning_time = float(result.planning_time)
        except (AttributeError, TypeError, ValueError):
            return None
        if (
            set(trajectory_names) != set(LEFT_ARM_JOINT_NAMES)
            or len(trajectory_names) != len(LEFT_ARM_JOINT_NAMES)
            or not points
            or not math.isfinite(planning_time)
            or planning_time < 0.0
        ):
            return None
        final_positions = tuple(points[-1].positions)
        if len(final_positions) != len(trajectory_names):
            return None
        if not all(math.isfinite(value) for value in final_positions):
            return None
        expected = dict(
            zip(
                joint_goal.joint_names,
                joint_goal.positions_rad,
                strict=True,
            )
        )
        actual = dict(
            zip(trajectory_names, final_positions, strict=True)
        )
        if any(
            abs(actual[name] - expected[name])
            > self._config.controller_goal_tolerance_rad
            for name in LEFT_ARM_JOINT_NAMES
        ):
            return None
        return PlannedMotion(
            validated_goal=validated_goal,
            trajectory=trajectory,
            planning_time_sec=planning_time,
        )

    def _validate_current_state(
        self,
        state: ValidatedCurrentState,
    ) -> int:
        if not isinstance(state, ValidatedCurrentState):
            raise ValueError(
                'current_state must come from '
                'validate_dual_arm_current_state'
            )
        checked = validate_dual_arm_current_state(
            state.sample,
            now_stamp_ns=state.validated_at_stamp_ns,
            config=self._config,
            joint_contract=self._joint_contract,
        )
        if checked.age_ns != state.age_ns:
            raise ValueError('current_state age provenance is inconsistent')
        return checked.sample.stamp_ns


MotionPort = MoveItMotionAdapter
