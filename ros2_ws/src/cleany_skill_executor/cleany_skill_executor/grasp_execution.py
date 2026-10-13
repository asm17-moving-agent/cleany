"""Shared MoveIt execution and joint-feedback helpers for study-cafe skills."""

from __future__ import annotations

import math
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import SelectReachableGrasp
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES


class GraspExecutionNode(Node):
    """Provide action clients and guarded motion to manipulation coordinators."""

    def __init__(self, node_name: str = 'grasp_execution') -> None:
        super().__init__(node_name)
        self.declare_parameter('selection_action', '/grasp/select_reachable')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.declare_parameter('demo_start_delay_sec', 5.0)
        self.declare_parameter('planning_timeout_sec', 5.0)
        self.declare_parameter('planning_attempts', 3)
        self.declare_parameter('replan_attempts', 2)
        self.declare_parameter('replan_delay_sec', 0.25)
        self.declare_parameter('velocity_scaling', 0.08)
        self.declare_parameter('acceleration_scaling', 0.08)
        self._selection = ActionClient(
            self,
            SelectReachableGrasp,
            str(self.get_parameter('selection_action').value),
        )
        self._move_group = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter('move_group_action').value),
        )
        self._joint_positions: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)

    def _spin_once(self, timeout_sec: float) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _on_joints(self, message: JointState) -> None:
        self._joint_positions.update(
            zip(message.name, message.position, strict=True)
        )


    def _wait_for_joint_state(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while (
            not set(REQUIRED_JOINT_NAMES) <= self._joint_positions.keys()
            and time.monotonic() < deadline
        ):
            self._spin_once(timeout_sec=0.05)
        missing = set(REQUIRED_JOINT_NAMES) - self._joint_positions.keys()
        if missing:
            self.get_logger().error(
                f'Missing required joint feedback: {sorted(missing)}'
            )
            raise RuntimeError('complete 12-joint feedback is unavailable')


    def _selection_feedback(self, message: Any) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f'candidate={feedback.candidate_index} arm={feedback.arm} '
            f'stage={feedback.stage}: {feedback.message}'
        )

    def _move_to(
        self,
        arm: str,
        joint_state: JointState,
        label: str,
        *,
        accept_control_failure: Callable[[], bool] | None = None,
    ) -> bool:
        attempts = (1, 2) if getattr(self, '_controller_retry_enabled', True) else (1,)
        for execution_attempt in attempts:
            goal = self._execution_goal(arm, joint_state, label)
            self.get_logger().info(
                f'MoveIt plan-and-execute: {label} attempt={execution_attempt}/{len(attempts)}'
            )
            handle = self._future(
                self._move_group.send_goal_async(goal),
                10.0,
                f'{label} goal response',
            )
            if not handle.accepted:
                raise RuntimeError(f'{label} MoveGroup goal was rejected')
            wrapped = self._future(handle.get_result_async(), 60.0, f'{label} execution result')
            code = wrapped.result.error_code.val
            if (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and code == MoveItErrorCodes.SUCCESS
            ):
                self.get_logger().info(f'MoveIt execution succeeded: {label}')
                return True
            self._log_joint_tracking_error(joint_state, label, code)
            if (
                code == MoveItErrorCodes.CONTROL_FAILED
                and accept_control_failure is not None
                and accept_control_failure()
            ):
                self.get_logger().info(
                    f'{label} stopped on verified target contact; '
                    'skipping endpoint retry'
                )
                return False
            if execution_attempt < len(attempts) and code == MoveItErrorCodes.CONTROL_FAILED:
                self.get_logger().warning(
                    f'{label} controller failed; replanning once from current state'
                )
                continue
            raise RuntimeError(
                f'{label} execution failed: status={wrapped.status} code={code}'
            )
        raise RuntimeError(f'{label} controller recovery was exhausted')

    def _execution_goal(
        self, arm: str, joint_state: JointState, label: str
    ) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = f'{arm}_grasp_arm'
        request.num_planning_attempts = int(
            self.get_parameter('planning_attempts').value
        )
        request.allowed_planning_time = float(
            self.get_parameter('planning_timeout_sec').value
        )
        request.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scaling').value
        )
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value
        )
        request.start_state.is_diff = True
        constraints = Constraints()
        constraints.name = f'grasp_{label}'
        for name, position in zip(
            joint_state.name, joint_state.position, strict=True
        ):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 1e-4
            constraint.tolerance_below = 1e-4
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = int(
            self.get_parameter('replan_attempts').value
        )
        goal.planning_options.replan_delay = float(
            self.get_parameter('replan_delay_sec').value
        )
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _log_joint_tracking_error(
        self, joint_state: JointState, label: str, error_code: int
    ) -> None:
        errors = {
            name: abs(float(target) - self._joint_positions[name])
            for name, target in zip(
                joint_state.name, joint_state.position, strict=True
            )
            if name in self._joint_positions
        }
        if not errors:
            self.get_logger().error(
                f'{label} failed code={error_code}; joint feedback unavailable'
            )
            return
        details = ', '.join(
            f'{name}={error:.4f}rad'
            for name, error in sorted(errors.items())
        )
        self.get_logger().error(
            f'{label} failed code={error_code}; joint tracking error: {details}'
        )

    def _verify_feedback(self, goal: JointState) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
            if all(
                abs(self._joint_positions.get(name, math.inf) - position) < 0.03
                for name, position in zip(goal.name, goal.position, strict=True)
            ):
                return
        raise RuntimeError('MuJoCo joint feedback did not converge to grasp')

    def _future(self, future: Any, timeout: float, label: str) -> Any:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'timed out waiting for {label}')
        result = future.result()
        if result is None:
            raise RuntimeError(f'{label} returned no result')
        return result

    def _hold(self, parameter_name: str) -> None:
        deadline = time.monotonic() + float(
            self.get_parameter(parameter_name).value
        )
        while time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
