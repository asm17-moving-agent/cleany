"""Operator-visible MuJoCo demo: select, then execute a reachable grasp."""

from __future__ import annotations

import math
import time
from typing import Any

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import GraspCandidate
from geometry_msgs.msg import Point
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class GraspExecutionDemo(Node):
    """Drive only the selected arm in a dedicated, operator-observed demo."""

    def __init__(self) -> None:
        super().__init__('grasp_execution_demo')
        self.declare_parameter('selection_action', '/grasp/select_reachable')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.declare_parameter('demo_start_delay_sec', 5.0)
        self.declare_parameter('planning_timeout_sec', 5.0)
        self.declare_parameter('stage_hold_sec', 3.0)
        self.declare_parameter('velocity_scaling', 0.12)
        self.declare_parameter('acceleration_scaling', 0.12)
        self.declare_parameter('target_position', [0.09, 0.6696, 0.6158])
        self.declare_parameter('target_size', [0.03, 0.03, 0.03])
        self.declare_parameter('approach_direction', [0.0, -0.186, 0.983])
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
        self._markers = self.create_publisher(
            MarkerArray,
            '/grasp_demo/markers',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._joint_positions: dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)

    def _on_joints(self, message: JointState) -> None:
        self._joint_positions.update(
            zip(message.name, message.position, strict=True)
        )

    def run(self) -> None:
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.get_logger().info('Waiting for reachable-grasp and MoveGroup actions')
        if not self._selection.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('reachable-grasp action is unavailable')
        if not self._move_group.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('MoveGroup action is unavailable')
        self._wait_for_joint_state(timeout)

        candidates = self._candidates()
        self._publish_markers(candidates, selected_index=None, status='evaluating')
        self.get_logger().info('GUI warm-up: grasp evaluation starts shortly')
        self._hold('demo_start_delay_sec')
        selection_goal = SelectReachableGrasp.Goal()
        selection_goal.candidates = candidates
        handle = self._future(
            self._selection.send_goal_async(
                selection_goal,
                feedback_callback=self._selection_feedback,
            ),
            timeout,
            'reachable-grasp goal response',
        )
        if not handle.accepted:
            raise RuntimeError('reachable-grasp goal was rejected')
        wrapped = self._future(
            handle.get_result_async(), timeout, 'reachable-grasp result'
        )
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise RuntimeError(
                f'grasp selection failed: code={result.error_code} {result.message}'
            )

        self.get_logger().info(
            f'Selected candidate={result.selected_candidate_index} '
            f'arm={result.selected_arm}; executing pre-grasp'
        )
        self._publish_markers(
            candidates,
            selected_index=result.selected_candidate_index,
            status=f'selected {result.selected_arm}',
        )
        self._hold('stage_hold_sec')
        self._move_to(result.selected_arm, result.pregrasp_joint_state, 'pre-grasp')
        self._hold('stage_hold_sec')
        self._move_to(result.selected_arm, result.grasp_joint_state, 'grasp')
        self._verify_feedback(result.grasp_joint_state)
        self._publish_markers(
            candidates,
            selected_index=result.selected_candidate_index,
            status=f'grasp reached with {result.selected_arm}',
        )
        self.get_logger().info(
            'DEMO COMPLETE: selected grasp reached in MuJoCo; '
            'trajectory execution and joint feedback both succeeded'
        )

    def _wait_for_joint_state(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while len(self._joint_positions) < 12 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if len(self._joint_positions) < 12:
            raise RuntimeError('complete 12-joint feedback is unavailable')

    def _candidates(self) -> list[GraspCandidate]:
        target = tuple(float(v) for v in self.get_parameter('target_position').value)
        size = tuple(float(v) for v in self.get_parameter('target_size').value)
        approach = tuple(
            float(v) for v in self.get_parameter('approach_direction').value
        )
        definitions = (
            ((2.0, 1.0, 1.5), (0.0, 0.0, -1.0), 0.9),
            (target, approach, 0.8),
        )
        candidates: list[GraspCandidate] = []
        for position, direction, score in definitions:
            candidate = GraspCandidate()
            candidate.header.frame_id = 'base_link'
            candidate.snapshot_id = 'mujoco-grasp-execution-demo'
            candidate.object_id = 1
            candidate.tcp_pose.position.x = position[0]
            candidate.tcp_pose.position.y = position[1]
            candidate.tcp_pose.position.z = position[2]
            candidate.tcp_pose.orientation.w = 1.0
            candidate.approach_direction.x = direction[0]
            candidate.approach_direction.y = direction[1]
            candidate.approach_direction.z = direction[2]
            candidate.required_opening_m = 0.03
            candidate.grasp_depth_m = 0.015
            candidate.score = score
            candidate.target_object.object_id = 1
            candidate.target_object.label = 'demo_grasp_target'
            candidate.target_object.confidence = 1.0
            candidate.target_object.obb_pose.position.x = target[0]
            candidate.target_object.obb_pose.position.y = target[1]
            candidate.target_object.obb_pose.position.z = target[2]
            candidate.target_object.obb_pose.orientation.w = 1.0
            candidate.target_object.obb_size.x = size[0]
            candidate.target_object.obb_size.y = size[1]
            candidate.target_object.obb_size.z = size[2]
            candidates.append(candidate)
        return candidates

    def _selection_feedback(self, message: Any) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f'candidate={feedback.candidate_index} arm={feedback.arm} '
            f'stage={feedback.stage}: {feedback.message}'
        )

    def _move_to(self, arm: str, joint_state: JointState, label: str) -> None:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = f'{arm}_grasp_arm'
        request.num_planning_attempts = 1
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
        constraints.name = f'demo_{label}'
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
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(f'MoveIt plan-and-execute: {label}')
        handle = self._future(
            self._move_group.send_goal_async(goal),
            10.0,
            f'{label} goal response',
        )
        if not handle.accepted:
            raise RuntimeError(f'{label} MoveGroup goal was rejected')
        wrapped = self._future(
            handle.get_result_async(), 60.0, f'{label} execution result'
        )
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'{label} execution failed: status={wrapped.status} '
                f'code={wrapped.result.error_code.val}'
            )
        self.get_logger().info(f'MoveIt execution succeeded: {label}')

    def _verify_feedback(self, goal: JointState) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(
                abs(self._joint_positions.get(name, math.inf) - position) < 0.03
                for name, position in zip(goal.name, goal.position, strict=True)
            ):
                return
        raise RuntimeError('MuJoCo joint feedback did not converge to grasp')

    def _future(self, future: Any, timeout: float, label: str) -> Any:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
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
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_markers(
        self,
        candidates: list[GraspCandidate],
        *,
        selected_index: int | None,
        status: str,
    ) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = stamp
            marker.ns = 'grasp_candidates'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = candidate.tcp_pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.035
            marker.color.a = 0.85
            if selected_index == index:
                marker.color.g = 1.0
            elif index == 0:
                marker.color.r = 1.0
            else:
                marker.color.r = marker.color.g = 0.9
            marker.lifetime = Duration(seconds=0).to_msg()
            markers.markers.append(marker)

        target = candidates[-1]
        norm = math.sqrt(
            target.approach_direction.x ** 2
            + target.approach_direction.y ** 2
            + target.approach_direction.z ** 2
        )
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.header.stamp = stamp
        arrow.ns = 'grasp_approach'
        arrow.id = 100
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.022
        arrow.color.b, arrow.color.a = 1.0, 1.0
        end = target.tcp_pose.position
        start = Point()
        start.x = end.x - target.approach_direction.x / norm * 0.08
        start.y = end.y - target.approach_direction.y / norm * 0.08
        start.z = end.z - target.approach_direction.z / norm * 0.08
        arrow.points = [start, end]
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = stamp
        text.ns = 'grasp_status'
        text.id = 200
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.09
        text.pose.position.y = 0.6696
        text.pose.position.z = 0.72
        text.scale.z = 0.045
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = status
        markers.markers.append(text)
        self._markers.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspExecutionDemo()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal(f'GRASP EXECUTION DEMO FAILED: {error}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
