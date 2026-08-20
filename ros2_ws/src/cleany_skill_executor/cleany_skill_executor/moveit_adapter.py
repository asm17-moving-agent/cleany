"""MoveIt service/action adapter for plan-only grasp reachability checks."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.grasp_selection import (
    ARM_JOINT_NAMES,
    InfrastructureError,
    JointSolution,
    REQUIRED_JOINT_NAMES,
)


@dataclass(frozen=True, slots=True)
class MoveItAdapterConfig:
    base_frame: str = 'base_link'
    ik_timeout_sec: float = 0.15
    state_validity_timeout_sec: float = 1.0
    planning_timeout_sec: float = 4.0
    planning_attempts: int = 1
    velocity_scaling: float = 0.1
    acceleration_scaling: float = 0.1
    poll_interval_sec: float = 0.01


class MoveItGraspAdapter:
    """Preserve a complete feedback state in IK, validity, and plan requests."""

    def __init__(
        self,
        node: Any,
        config: MoveItAdapterConfig = MoveItAdapterConfig(),
        *,
        ik_client: Any | None = None,
        validity_client: Any | None = None,
        plan_client: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        spin_once: Callable[[float], None] | None = None,
    ) -> None:
        self._node = node
        self._config = config
        self._ik_client = ik_client or node.create_client(GetPositionIK, '/compute_ik')
        self._validity_client = validity_client or node.create_client(
            GetStateValidity, '/check_state_validity'
        )
        self._plan_client = plan_client or ActionClient(node, MoveGroup, '/move_action')
        self._monotonic = monotonic
        self._spin_once = spin_once or self._default_spin
        self._current_state: JointState | None = None
        self._active_goal_handle: Any | None = None

    def _default_spin(self, timeout: float) -> None:
        import rclpy

        rclpy.spin_once(self._node, timeout_sec=timeout)

    def set_current_state(self, state: JointState) -> None:
        positions = dict(zip(state.name, state.position, strict=True))
        missing = set(REQUIRED_JOINT_NAMES) - positions.keys()
        if missing:
            raise ValueError(f'incomplete joint state: {sorted(missing)}')
        canonical = JointState()
        canonical.header = state.header
        canonical.name = list(REQUIRED_JOINT_NAMES)
        canonical.position = [float(positions[name]) for name in canonical.name]
        if state.velocity and len(state.velocity) == len(state.name):
            velocities = dict(zip(state.name, state.velocity, strict=True))
            canonical.velocity = [float(velocities[name]) for name in canonical.name]
        self._current_state = canonical

    def _merged_state(self, solution: JointSolution | None) -> RobotState:
        if self._current_state is None:
            raise InfrastructureError('current joint state is unavailable')
        positions = dict(zip(self._current_state.name, self._current_state.position, strict=True))
        if solution is not None:
            positions.update(zip(solution.names, solution.positions, strict=True))
        result = RobotState()
        result.is_diff = False
        result.joint_state.header = self._current_state.header
        result.joint_state.name = list(REQUIRED_JOINT_NAMES)
        result.joint_state.position = [positions[name] for name in REQUIRED_JOINT_NAMES]
        if self._current_state.velocity:
            velocities = dict(
                zip(self._current_state.name, self._current_state.velocity, strict=True)
            )
            result.joint_state.velocity = [velocities[name] for name in REQUIRED_JOINT_NAMES]
        return result

    def _wait_service(self, client: Any, timeout: float) -> None:
        deadline = self._monotonic() + timeout
        while not client.service_is_ready():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                raise InfrastructureError('MoveIt service unavailable')
            self._spin_once(min(self._config.poll_interval_sec, remaining))

    def _call(self, client: Any, request: Any, timeout: float) -> Any:
        self._wait_service(client, timeout)
        future = client.call_async(request)
        deadline = self._monotonic() + timeout
        while not future.done():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                future.cancel()
                raise InfrastructureError('MoveIt service timed out')
            self._spin_once(min(self._config.poll_interval_sec, remaining))
        try:
            response = future.result()
        except Exception as error:
            raise InfrastructureError(f'MoveIt service failed: {error}') from error
        if response is None:
            raise InfrastructureError('MoveIt service returned no response')
        return response

    def solve_position_ik(
        self,
        arm: str,
        position: tuple[float, float, float],
        seed: JointSolution | None,
    ) -> JointSolution | None:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f'{arm}_grasp_arm'
        ik.ik_link_name = f'{arm}_grasp_tcp'
        ik.avoid_collisions = True
        ik.robot_state = self._merged_state(seed)
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = self._config.base_frame
        ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y, ik.pose_stamped.pose.position.z = position
        ik.pose_stamped.pose.orientation.w = 1.0
        seconds = int(self._config.ik_timeout_sec)
        ik.timeout.sec = seconds
        ik.timeout.nanosec = int((self._config.ik_timeout_sec - seconds) * 1e9)
        # MoveIt may legitimately consume the full solver timeout before
        # returning NO_IK_SOLUTION. Keep transport overhead outside that
        # algorithm budget so an unreachable pair remains a fallback result,
        # not an infrastructure timeout.
        response = self._call(
            self._ik_client,
            request,
            self._config.ik_timeout_sec + 1.0,
        )
        if response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            if response.error_code.val in (
                MoveItErrorCodes.INVALID_ROBOT_STATE,
                MoveItErrorCodes.INVALID_GROUP_NAME,
                MoveItErrorCodes.INVALID_LINK_NAME,
            ):
                raise InfrastructureError(f'IK configuration error: {response.error_code.val}')
            return None
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
                strict=True,
            )
        )
        names = ARM_JOINT_NAMES[arm]
        if not set(names) <= positions.keys():
            raise InfrastructureError('IK response omits arm joints')
        return JointSolution(names, tuple(float(positions[name]) for name in names))

    def state_is_valid(self, arm: str, solution: JointSolution) -> bool:
        request = GetStateValidity.Request()
        request.group_name = f'{arm}_grasp_arm'
        request.robot_state = self._merged_state(solution)
        response = self._call(
            self._validity_client, request, self._config.state_validity_timeout_sec
        )
        return bool(response.valid)

    def plan(
        self,
        arm: str,
        goal: JointSolution,
        start: JointSolution | None,
    ) -> bool:
        wait_timeout = self._config.planning_timeout_sec
        if not self._plan_client.wait_for_server(timeout_sec=wait_timeout):
            raise InfrastructureError('MoveGroup action unavailable')
        action_goal = MoveGroup.Goal()
        action_goal.planning_options.plan_only = True
        action_goal.planning_options.look_around = False
        action_goal.planning_options.replan = False
        action_goal.planning_options.planning_scene_diff.is_diff = True
        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        request = action_goal.request
        request.group_name = f'{arm}_grasp_arm'
        request.num_planning_attempts = self._config.planning_attempts
        request.allowed_planning_time = self._config.planning_timeout_sec
        request.max_velocity_scaling_factor = self._config.velocity_scaling
        request.max_acceleration_scaling_factor = self._config.acceleration_scaling
        request.start_state = self._merged_state(start)
        constraints = Constraints()
        constraints.name = f'{arm}_grasp_joint_goal'
        for name, position in zip(goal.names, goal.positions, strict=True):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 1e-4
            constraint.tolerance_below = 1e-4
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        future = self._plan_client.send_goal_async(action_goal)
        goal_handle = self._wait_future(future, wait_timeout)
        if goal_handle is None or not goal_handle.accepted:
            raise InfrastructureError('MoveGroup goal rejected')
        self._active_goal_handle = goal_handle
        try:
            wrapped = self._wait_future(goal_handle.get_result_async(), wait_timeout)
        finally:
            self._active_goal_handle = None
        if wrapped is None:
            raise InfrastructureError('MoveGroup action timed out')
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise InterruptedError('MoveGroup action canceled')
        return (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code.val == MoveItErrorCodes.SUCCESS
            and bool(wrapped.result.planned_trajectory.joint_trajectory.points)
        )

    def _wait_future(self, future: Any, timeout: float) -> Any | None:
        deadline = self._monotonic() + timeout
        while not future.done():
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                future.cancel()
                return None
            self._spin_once(min(self._config.poll_interval_sec, remaining))
        return future.result()

    def cancel_active(self) -> None:
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
