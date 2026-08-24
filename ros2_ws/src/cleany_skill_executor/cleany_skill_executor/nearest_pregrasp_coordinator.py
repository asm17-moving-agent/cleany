"""Run nearest-first perception, grasp selection, and pre-grasp execution."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene, SelectReachableGrasp
from cleany_interfaces.srv import PlanGrasp
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint

import rclpy

from cleany_skill_executor.core.nearest_object import (
    ObjectAttempt,
    rank_object_attempts,
)
from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
from cleany_skill_executor.planning_scene import TargetSceneTransaction


class NearestPregraspCoordinator(GraspExecutionDemo):
    """Try distance-valid objects until one reaches pre-grasp."""

    def __init__(self) -> None:
        super().__init__(node_name='nearest_pregrasp_coordinator')
        self.declare_parameter(
            'inspection_action',
            '/perception/inspect_scene',
        )
        self.declare_parameter('grasp_service', '/grasp/plan')
        self.declare_parameter(
            'query',
            'Detect the box and can on the table.',
        )
        self.declare_parameter('inspection_timeout_sec', 60.0)
        self.declare_parameter('grasp_timeout_sec', 30.0)
        self.declare_parameter('selection_timeout_sec', 120.0)
        self.declare_parameter('gripper_open_position_rad', 1.2)
        self.declare_parameter('gripper_motion_sec', 2.0)
        self._inspection = ActionClient(
            self,
            InspectScene,
            str(self.get_parameter('inspection_action').value),
        )
        self._grasp = self.create_client(
            PlanGrasp,
            str(self.get_parameter('grasp_service').value),
        )
        self._grippers = {
            arm: ActionClient(
                self,
                FollowJointTrajectory,
                f'/{arm}_gripper_controller/follow_joint_trajectory',
            )
            for arm in ('left', 'right')
        }
        self._execution_scene = TargetSceneTransaction(
            self,
            timeout_sec=2.0,
            spin_once=lambda duration: rclpy.spin_once(
                self,
                timeout_sec=duration,
            ),
        )

    def destroy_node(self) -> None:
        try:
            if self._execution_scene.active:
                try:
                    self._execution_scene.restore()
                # MoveGroup may stop first during launch shutdown.
                except Exception as error:
                    self.get_logger().warning(
                        'Could not restore target scene during shutdown: '
                        f'{error}'
                    )
        finally:
            super().destroy_node()

    def run(self) -> None:
        startup_timeout = float(
            self.get_parameter('startup_timeout_sec').value
        )
        self.get_logger().info(
            'Waiting for perception, grasp, reachable-grasp, and MoveGroup'
        )
        if not self._inspection.wait_for_server(timeout_sec=startup_timeout):
            raise RuntimeError('inspection action is unavailable')
        if not self._grasp.wait_for_service(timeout_sec=startup_timeout):
            raise RuntimeError('grasp planning service is unavailable')
        if not self._selection.wait_for_server(timeout_sec=startup_timeout):
            raise RuntimeError('reachable-grasp action is unavailable')
        if not self._move_group.wait_for_server(timeout_sec=startup_timeout):
            raise RuntimeError('MoveGroup action is unavailable')
        self._wait_for_joint_state(startup_timeout)
        self._open_gripper('left')
        self._open_gripper('right')

        detection_result = self._detect_objects()
        attempts = self._attempts(detection_result.detections.detections)
        if not attempts:
            raise RuntimeError(
                'no detection has enough valid depth for autonomous selection'
            )

        failures: list[str] = []
        for attempt in attempts:
            self.get_logger().info(
                f'Trying nearest object_id={attempt.object_id} '
                f'label={attempt.label} distance={attempt.distance_m:.3f}m'
            )
            inspected = self._inspect_selected(
                detection_result.detections.snapshot_id,
                attempt,
            )
            if inspected is None:
                failures.append(f'{attempt.object_id}: inspection')
                continue
            planned = self._plan_grasps(inspected, attempt)
            if planned is None:
                failures.append(f'{attempt.object_id}: no grasp')
                continue
            selected = self._select_reachable(planned.candidates, attempt)
            if selected is None:
                failures.append(f'{attempt.object_id}: unreachable')
                continue
            self._execute_pregrasp(selected, attempt)
            self.get_logger().info(
                'NEAREST PREGRASP COMPLETE: '
                f'object_id={attempt.object_id} label={attempt.label} '
                f'distance={attempt.distance_m:.3f}m '
                f'candidate={selected.selected_candidate_index} '
                f'arm={selected.selected_arm}'
            )
            return
        raise RuntimeError(
            'no distance-valid object produced a reachable pre-grasp; '
            + ', '.join(failures)
        )

    def _detect_objects(self):
        goal = InspectScene.Goal()
        goal.query = str(self.get_parameter('query').value)
        timeout = float(self.get_parameter('inspection_timeout_sec').value)
        handle = self._future(
            self._inspection.send_goal_async(goal),
            10.0,
            'inspection detection goal response',
        )
        if not handle.accepted:
            raise RuntimeError('inspection detection goal was rejected')
        wrapped = self._future(
            handle.get_result_async(),
            timeout,
            'inspection detection result',
        )
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.success
        ):
            raise RuntimeError(
                'object detection failed: '
                f'code={wrapped.result.error_code} {wrapped.result.message}'
            )
        return wrapped.result

    @staticmethod
    def _attempts(detections) -> tuple[ObjectAttempt, ...]:
        attempts = [
            ObjectAttempt(
                object_id=int(item.object_id),
                label=item.label,
                confidence=float(item.confidence),
                distance_m=float(item.distance_m),
            )
            for item in detections
            if item.distance_valid
            and math.isfinite(float(item.distance_m))
            and float(item.distance_m) >= 0.0
        ]
        return rank_object_attempts(attempts)

    def _inspect_selected(
        self,
        snapshot_id: str,
        attempt: ObjectAttempt,
    ):
        goal = InspectScene.Goal()
        goal.snapshot_id = snapshot_id
        goal.selected_object_id = attempt.object_id
        handle = self._future(
            self._inspection.send_goal_async(goal),
            10.0,
            f'object {attempt.object_id} inspection goal response',
        )
        if not handle.accepted:
            raise RuntimeError('selected-object inspection goal was rejected')
        wrapped = self._future(
            handle.get_result_async(),
            float(self.get_parameter('inspection_timeout_sec').value),
            f'object {attempt.object_id} inspection result',
        )
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
            and len(wrapped.result.objects.objects) == 1
        ):
            return wrapped.result
        self.get_logger().warning(
            f'Skipping object {attempt.object_id}: inspection failed '
            f'code={wrapped.result.error_code} {wrapped.result.message}'
        )
        return None

    def _plan_grasps(self, inspected, attempt: ObjectAttempt):
        request = PlanGrasp.Request()
        request.snapshot_id = inspected.objects.snapshot_id
        request.object_id = attempt.object_id
        request.target_cloud = inspected.target_cloud
        request.context_cloud = inspected.context_cloud
        request.target_object = inspected.objects.objects[0]
        response = self._future(
            self._grasp.call_async(request),
            float(self.get_parameter('grasp_timeout_sec').value),
            f'object {attempt.object_id} grasp candidates',
        )
        if response.success and response.candidates:
            return response
        self.get_logger().warning(
            f'Skipping object {attempt.object_id}: grasp generation failed '
            f'code={response.error_code} {response.message}'
        )
        return None

    def _select_reachable(self, candidates, attempt: ObjectAttempt):
        goal = SelectReachableGrasp.Goal()
        goal.candidates = candidates
        handle = self._future(
            self._selection.send_goal_async(
                goal,
                feedback_callback=self._selection_feedback,
            ),
            10.0,
            f'object {attempt.object_id} reachable-grasp goal response',
        )
        if not handle.accepted:
            raise RuntimeError('reachable-grasp goal was rejected')
        wrapped = self._future(
            handle.get_result_async(),
            float(self.get_parameter('selection_timeout_sec').value),
            f'object {attempt.object_id} reachable-grasp result',
        )
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            raise RuntimeError('reachable-grasp selection was canceled')
        if (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.success
        ):
            return wrapped.result
        if (
            wrapped.result.error_code
            == SelectReachableGrasp.Result.ERROR_NO_REACHABLE_GRASP
        ):
            self.get_logger().warning(
                f'Skipping object {attempt.object_id}: no reachable grasp'
            )
            return None
        raise RuntimeError(
            'reachable-grasp infrastructure failed: '
            f'code={wrapped.result.error_code} {wrapped.result.message}'
        )

    def _execute_pregrasp(
        self,
        selected,
        attempt: ObjectAttempt,
    ) -> None:
        scene_id = (
            'nearest_pregrasp_target_'
            f'{selected.selected_candidate.snapshot_id}_'
            f'{attempt.object_id}'
        )
        self._execution_scene.begin(selected.selected_candidate, scene_id)
        try:
            self._execution_scene.disallow_target_contacts()
            self._move_to(
                selected.selected_arm,
                selected.pregrasp_joint_state,
                'nearest-object pre-grasp',
            )
            self._verify_feedback(selected.pregrasp_joint_state)
        except Exception:
            self._execution_scene.restore()
            raise

    def _open_gripper(self, arm: str) -> None:
        client = self._grippers[arm]
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'{arm} gripper controller is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'{arm}_gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [
            float(self.get_parameter('gripper_open_position_rad').value)
        ]
        point.time_from_start = Duration(
            seconds=float(self.get_parameter('gripper_motion_sec').value)
        ).to_msg()
        goal.trajectory.points = [point]
        handle = self._future(
            client.send_goal_async(goal),
            5.0,
            f'{arm} gripper goal response',
        )
        if not handle.accepted:
            raise RuntimeError(f'{arm} gripper command was rejected')
        wrapped = self._future(
            handle.get_result_async(),
            10.0,
            f'{arm} gripper result',
        )
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f'{arm} gripper failed: status={wrapped.status} '
                f'code={wrapped.result.error_code}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NearestPregraspCoordinator()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal(f'NEAREST PREGRASP FAILED: {error}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
