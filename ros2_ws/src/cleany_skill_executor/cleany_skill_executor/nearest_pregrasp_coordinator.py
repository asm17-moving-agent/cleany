"""Run nearest-first perception, grasp selection, and pre-grasp execution."""

from __future__ import annotations
from cleany_skill_executor.motion_guard import enforce_motion_guard

from collections.abc import Sequence
from copy import deepcopy
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene, SelectReachableGrasp
from cleany_interfaces.srv import PlanGrasp
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import Header, String
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
import rclpy

from cleany_skill_executor.core.can_rgbd import (
    CameraProjection,
    render_grasp_overlay,
    rotation_matrix_from_quaternion,
)
from cleany_skill_executor.core.nearest_object import (
    ObjectAttempt,
    rank_object_attempts,
)
from cleany_skill_executor.core.sensor_scene import validate_sensor_scene
from cleany_skill_executor.core.cartesian import (
    CartesianPose, execution_wall_timeout, line_corridor, sampled_time_scale, validate_corridor_samples, validate_pose_endpoint,
)
from cleany_skill_executor.core.gripper import (
    aperture_centering_offset,
    is_gripper_contact_stall,
    opening_to_gripper_position,
)
from cleany_skill_executor.core.grasp_selection import (
    REQUIRED_JOINT_NAMES,
    quaternion_axis,
)
from cleany_skill_executor.core.grasp_pipeline import (
    ContactDebounce,
    ContactLimits,
    ContactSample,
    ObservedGrasp,
    ReinspectionLimits,
    associate_refreshed_grasp,
    linear_approach_error_deg,
)
from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
from cleany_skill_executor.planning_scene import TargetSceneTransaction
from cleany_skill_executor.collision_geometry_cache import CollisionGeometryCache, subscribe_collision_geometry
from cleany_skill_executor.seeded_cartesian import (
    SeededCartesianConfig, SeededCartesianPlanner, load_joint_motion_limits,
)


class LiftRedetectionError(RuntimeError):
    """A fresh detector snapshot has no distance-valid target of the same label."""


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
        self.declare_parameter('tcp_fk_timeout_sec', 5.0)
        fk_timeout = float(self.get_parameter('tcp_fk_timeout_sec').value)
        if not math.isfinite(fk_timeout) or not 0 < fk_timeout <= 30:
            raise ValueError('tcp_fk_timeout_sec must be in (0, 30] seconds')
        self.declare_parameter('gripper_open_position_rad', 1.2)
        self.declare_parameter('gripper_close_position_rad', 0.30)
        self.declare_parameter('gripper_motion_sec', 3.0)
        self.declare_parameter('gripper_wall_timeout_factor', 2.0)
        self.declare_parameter('cartesian_execution_wall_timeout_factor', 2.0)
        self.declare_parameter('cartesian_execution_wall_timeout_margin_sec', 10.0)
        self.declare_parameter('cartesian_execution_wall_timeout_minimum_sec', 60.0)
        self._gripper_retry_steps = self.declare_parameter('gripper_contact_retry_steps', 0).value
        if not 0 <= self._gripper_retry_steps <= 5:
            raise ValueError('gripper_contact_retry_steps must be between 0 and 5')
        self.declare_parameter('gripper_contact_retry_step_rad', 0.10)
        self.declare_parameter('gripper_contact_retry_motion_sec', 1.2)
        self._grasp_close_override = None
        self.declare_parameter('execute_grasp_and_lift', False)
        self.declare_parameter('minimum_candidate_opening_m', 0.0)
        self.declare_parameter('gripper_force_full_close', False)
        self.declare_parameter('gripper_aperture_reference_m', 0.050)
        self.declare_parameter(
            'gripper_aperture_reference_position_rad', 0.30
        )
        self.declare_parameter('gripper_aperture_m_per_rad', 0.10)
        self.declare_parameter('gripper_close_opening_reduction_m', 0.010)
        self.declare_parameter('gripper_contact_min_motion_rad', 0.10)
        self.declare_parameter('gripper_contact_min_residual_rad', 0.05)
        self.declare_parameter('gripper_contact_max_velocity_rad_s', 0.05)
        self.declare_parameter('require_gripper_contact', True)
        self.declare_parameter('grasp_contact_stop_max_distance_m', 0.020)
        self.declare_parameter('pilz_pipeline_id', 'pilz_industrial_motion_planner')
        self.declare_parameter('pilz_planner_id', 'LIN')
        self.declare_parameter('approach_velocity_scaling', 0.2)
        self.declare_parameter('use_joint_corridor_grasp', False)
        self.declare_parameter('use_seeded_cartesian_grasp', False)
        self.declare_parameter('cartesian_ik_step_m', .01)
        self.declare_parameter('cartesian_validation_step_m', .001)
        self.declare_parameter('cartesian_validation_joint_step_rad', .005)
        self.declare_parameter('cartesian_maximum_points', 2000)
        self.declare_parameter('cartesian_local_refinement_iterations', 0)
        self.declare_parameter('cartesian_joint_acceleration_rad_s2', 1.0)
        self.declare_parameter('cartesian_joint_limits_file', str(
            Path(get_package_share_directory('cleany_moveit_config')) / 'config' / 'joint_limits.yaml'))
        self.declare_parameter('attachment_scene_timeout_sec', 5.0)
        self._direct_vertical_lift = self.declare_parameter('direct_vertical_lift', False).value
        self.declare_parameter('corridor_orientation_tolerance_deg', 5.0)
        self.declare_parameter('corridor_time_margin', 2.0)
        self.declare_parameter('cartesian_translation_speed_m_s', 0.10)
        self.declare_parameter('cartesian_translation_acceleration_m_s2', 0.20)
        self.declare_parameter('cartesian_rotation_speed_rad_s', 0.50)
        self._use_joint_corridor = bool(self.get_parameter('use_joint_corridor_grasp').value)
        self._seeded_cartesian = None
        if bool(self.get_parameter('use_seeded_cartesian_grasp').value):
            if not self._use_joint_corridor:
                raise ValueError('seeded Cartesian grasp requires joint corridor grasp')
            self._seeded_ik = self.create_client(GetPositionIK, '/compute_ik')
            self._seeded_validity = self.create_client(GetStateValidity, '/check_state_validity')
            self._seeded_cartesian = SeededCartesianPlanner(
                lambda req: self._future(self._seeded_ik.call_async(req), fk_timeout, 'seeded Cartesian IK'),
                lambda req: self._future(self._fk.call_async(req), fk_timeout, 'seeded Cartesian FK'),
                lambda req: self._future(self._seeded_validity.call_async(req), fk_timeout, 'seeded Cartesian collision'),
                load_joint_motion_limits(str(self.get_parameter('cartesian_joint_limits_file').value),
                                         float(self.get_parameter('cartesian_joint_acceleration_rad_s2').value)),
                SeededCartesianConfig(
                    ik_step_m=float(self.get_parameter('cartesian_ik_step_m').value),
                    validation_step_m=float(self.get_parameter('cartesian_validation_step_m').value),
                    validation_joint_step_rad=float(self.get_parameter('cartesian_validation_joint_step_rad').value),
                    local_refinement_iterations=int(self.get_parameter('cartesian_local_refinement_iterations').value),
                    maximum_points=int(self.get_parameter('cartesian_maximum_points').value)))
            self.create_subscription(String, '/robot_description',
                lambda message: self._seeded_cartesian.set_robot_description(message.data),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.declare_parameter('retreat_velocity_scaling', 0.4)
        self.declare_parameter('lin_acceleration_scaling', 0.4)
        self.declare_parameter('lin_position_tolerance_m', 0.001)
        self.declare_parameter('lin_orientation_tolerance_rad', 0.01)
        self.declare_parameter('lin_alignment_tolerance_deg', 10.0)
        self.declare_parameter('arm_stationary_velocity_rad_s', 0.02)
        self.declare_parameter('arm_stationary_samples', 5)
        self.declare_parameter('reinspection_max_center_shift_m', 0.030)
        self.declare_parameter('reinspection_max_axis_change_deg', 15.0)
        self.declare_parameter('reinspection_ambiguity_distance_m', 0.010)
        self.declare_parameter('contact_min_joint_error_rad', 0.10)
        self.declare_parameter('contact_max_joint_velocity_rad_s', 0.05)
        self.declare_parameter('contact_consecutive_samples', 5)
        self.declare_parameter('contact_effort_threshold', 0.0)
        self.declare_parameter('lift_distance_m', 0.06)
        self.declare_parameter('count_retreat_as_lift', False)
        self.declare_parameter('grasp_settle_sec', 1.0)
        self.declare_parameter('preapproach_maximum_geometry_shift_m', 0.005)
        self.declare_parameter('lift_hold_sec', 3.0)
        self.declare_parameter('lift_min_center_z_m', 0.0)
        self.declare_parameter(
            'debug_image_source_topic', '/perception/debug_image_latched'
        )
        self.declare_parameter(
            'grasp_debug_image_topic', '/grasp/debug_image_latched'
        )
        self.declare_parameter('grasp_debug_republish_period_sec', 0.5)
        self.declare_parameter('keep_debug_image_alive_on_failure', False)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('require_sensor_scene', False)
        self.declare_parameter('sensor_scene_maximum_age_sec', 2.0)
        self.declare_parameter(
            'sensor_scene_cloud_topic', '/perception/scene_cloud_filtered'
        )
        self.declare_parameter('sensor_scene_receipt_topic', '')
        self._scene_cloud_stamp_ns: int | None = None
        self.declare_parameter(
            'sensor_scene_topic', '/monitored_planning_scene'
        )
        self._sensor_map_id = ''
        self._sensor_map_bytes = 0
        self._sensor_map_received_at = float('-inf')
        self._scene_commit_received_at = float('-inf')
        self._scene_commit_stamp_ns = 0
        self.create_subscription(
            PlanningScene, str(self.get_parameter('sensor_scene_topic').value),
            self._on_sensor_scene, 10,
        )
        receipt_topic = str(self.get_parameter('sensor_scene_receipt_topic').value)
        if receipt_topic:
            self.create_subscription(
                Header, receipt_topic, self._on_scene_cloud_receipt,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        else:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter('sensor_scene_cloud_topic').value),
                self._on_scene_cloud, qos_profile_sensor_data,
            )
        self.declare_parameter(
            'camera_info_topic',
            '/cleany/internal/mujoco/left_wrist_camera/camera_info',
        )
        self.declare_parameter('selector_pregrasp_offset_m', 0.14)
        self.declare_parameter('support_patch_margin_m', 0.0)
        self.declare_parameter('planning_scene_timeout_sec', 2.0)
        self.declare_parameter('use_observed_collision_geometry', False)
        self.declare_parameter('collision_geometry_topic', '/grasp/collision_geometry')
        self.declare_parameter('selector_grasp_approach_offset_m', 0.0)
        self.declare_parameter('selector_grasp_lateral_offset_m', 0.0)
        self._aperture_centering = self.declare_parameter('grasp_use_aperture_centering', False).value
        self.declare_parameter('grasp_aperture_margin_m', 0.008)
        self.declare_parameter('grasp_fixed_jaw_inner_x_m', 0.008)
        self.declare_parameter('grasp_fixed_jaw_clearance_m', 0.0)
        self._inspection = ActionClient(
            self,
            InspectScene,
            str(self.get_parameter('inspection_action').value),
        )
        self._grasp = self.create_client(
            PlanGrasp,
            str(self.get_parameter('grasp_service').value),
        )
        self._fk = self.create_client(GetPositionFK, '/compute_fk')
        self._execute_trajectory = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )
        self._grippers = {
            arm: ActionClient(
                self,
                FollowJointTrajectory,
                f'/{arm}_gripper_controller/follow_joint_trajectory',
            )
            for arm in ('left', 'right')
        }
        use_mesh = bool(self.get_parameter('use_observed_collision_geometry').value)
        self._geometry_cache = CollisionGeometryCache()
        self._geometry_subscription = (subscribe_collision_geometry(self, self._geometry_cache,
            record=True) if use_mesh else None)
        self._execution_scene = TargetSceneTransaction(
            self,
            geometry_lookup=self._geometry_cache.get if use_mesh else None,
            support_patch_margin_m=float(self.get_parameter('support_patch_margin_m').value),
            timeout_sec=float(self.get_parameter('planning_scene_timeout_sec').value),
            spin_once=lambda duration: rclpy.spin_once(
                self,
                timeout_sec=duration,
            ),
        )
        self._joint_velocities: dict[str, float] = {}
        self._controller_states: dict[str, JointTrajectoryControllerState] = {}
        self._controller_state_counts = {'left': 0, 'right': 0}
        self._debug_image: Image | None = None
        self._last_grasp_debug_image: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._grasp_debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('grasp_debug_image_topic').value),
            latched_qos,
        )
        self._grasp_debug_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._grasp_debug_timer = self.create_timer(
            max(
                0.05,
                float(
                    self.get_parameter(
                        'grasp_debug_republish_period_sec'
                    ).value
                ),
            ),
            self._republish_grasp_debug_image,
            clock=self._grasp_debug_clock,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('debug_image_source_topic').value),
            self._on_debug_image,
            latched_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        for arm in ('left', 'right'):
            self.create_subscription(
                JointTrajectoryControllerState,
                f'/{arm}_arm_controller/controller_state',
                lambda message, side=arm: self._on_controller_state(
                    side, message
                ),
                20,
            )

    def _on_joints(self, message) -> None:
        super()._on_joints(message)
        if len(message.velocity) == len(message.name):
            self._joint_velocities.update(
                zip(message.name, message.velocity, strict=True)
            )

    def _on_debug_image(self, message: Image) -> None:
        self._debug_image = message

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _on_controller_state(
        self, arm: str, message: JointTrajectoryControllerState
    ) -> None:
        self._controller_states[arm] = message
        self._controller_state_counts[arm] += 1

    def _candidate_execution_positions(
        self,
        candidate,
    ) -> tuple[np.ndarray, np.ndarray]:
        approach = np.asarray(
            (
                candidate.approach_direction.x,
                candidate.approach_direction.y,
                candidate.approach_direction.z,
            ),
            dtype=float,
        )
        approach /= np.linalg.norm(approach)
        orientation = candidate.tcp_pose.orientation
        closing = np.asarray(
            quaternion_axis(
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
                (1.0, 0.0, 0.0),
            ),
            dtype=float,
        )
        source = np.asarray(
            (
                candidate.tcp_pose.position.x,
                candidate.tcp_pose.position.y,
                candidate.tcp_pose.position.z,
            ),
            dtype=float,
        )
        lateral = float(
            self.get_parameter('selector_grasp_lateral_offset_m').value
        )
        if getattr(self, '_aperture_centering', False):
            lateral = aperture_centering_offset(float(candidate.required_opening_m),
                float(self.get_parameter('grasp_aperture_margin_m').value),
                float(self.get_parameter('grasp_fixed_jaw_inner_x_m').value),
                float(self.get_parameter('grasp_fixed_jaw_clearance_m').value))
        pregrasp = (
            source
            - float(
                self.get_parameter('selector_pregrasp_offset_m').value
            )
            * approach
            + lateral * closing
        )
        grasp = (
            source
            + float(
                self.get_parameter(
                    'selector_grasp_approach_offset_m'
                ).value
            )
            * approach
            + lateral * closing
        )
        return pregrasp, grasp

    def _publish_grasp_overlay(
        self,
        candidates: Sequence,
        *,
        selected_index: int | None = None,
        selected_arm: str = '',
    ) -> None:
        """Publish candidate pre-grasp and grasp TCP points over the RGB view."""
        source = self._debug_image
        camera_info = self._camera_info
        if source is None or camera_info is None:
            self.get_logger().warning(
                'Grasp overlay unavailable: waiting for debug image/camera info'
            )
            return
        if source.encoding != 'rgb8' or source.step < source.width * 3:
            self.get_logger().warning(
                'Grasp overlay skipped: debug image must use rgb8'
            )
            return
        if not candidates:
            return
        if selected_index is not None and not 0 <= selected_index < len(candidates):
            self.get_logger().warning(
                f'Grasp overlay skipped: invalid selected index {selected_index}'
            )
            return
        try:
            stamped = self._tf_buffer.lookup_transform(
                candidates[0].header.frame_id,
                source.header.frame_id,
                Time.from_msg(source.header.stamp),
            )
            transform = stamped.transform
            rotation = rotation_matrix_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f'Grasp overlay skipped: camera TF unavailable: {error}'
            )
            return
        rows = np.frombuffer(source.data, dtype=np.uint8).reshape(
            source.height,
            source.step,
        )
        rgb = rows[:, : source.width * 3].reshape(
            source.height,
            source.width,
            3,
        ).copy()
        projection = CameraProjection(
            fx=float(camera_info.k[0]),
            fy=float(camera_info.k[4]),
            cx=float(camera_info.k[2]),
            cy=float(camera_info.k[5]),
            translation_base=(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            rotation_base_from_optical=tuple(rotation.reshape(-1)),
        )
        execution_positions = [
            self._candidate_execution_positions(item)
            for item in candidates
        ]
        grasp_positions = np.asarray(
            [grasp for _, grasp in execution_positions],
            dtype=float,
        )
        pregrasp_offset = float(
            self.get_parameter('selector_pregrasp_offset_m').value
        ) + float(
            self.get_parameter('selector_grasp_approach_offset_m').value
        )
        rendered = render_grasp_overlay(
            rgb,
            projection,
            grasp_positions,
            np.asarray(
                [
                    (
                        item.approach_direction.x,
                        item.approach_direction.y,
                        item.approach_direction.z,
                    )
                    for item in candidates
                ],
                dtype=float,
            ),
            np.asarray([item.score for item in candidates], dtype=float),
            np.asarray(
                [item.required_opening_m for item in candidates],
                dtype=float,
            ),
            selected_index=selected_index,
            selected_arm=selected_arm,
            pregrasp_offset_m=pregrasp_offset,
            title='GRASP DEBUG',
        )
        message = Image()
        message.header = source.header
        message.height, message.width = rendered.shape[:2]
        message.encoding = 'rgb8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = np.ascontiguousarray(rendered).tobytes()
        self._last_grasp_debug_image = message
        self._grasp_debug_publisher.publish(message)
        state = (
            'candidates'
            if selected_index is None
            else f'selected={selected_index} arm={selected_arm}'
        )
        self.get_logger().info(
            f'Published grasp RGB overlay: count={len(candidates)} {state}'
        )

    def _republish_grasp_debug_image(self) -> None:
        if self._last_grasp_debug_image is not None:
            self._grasp_debug_publisher.publish(
                self._last_grasp_debug_image
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
        self._wait_for_pipeline()
        self._run_nearest()

    def _wait_for_pipeline(self) -> None:
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
        if (
            bool(self.get_parameter('execute_grasp_and_lift').value)
            and not self._execute_trajectory.wait_for_server(
                timeout_sec=startup_timeout
            )
        ):
            raise RuntimeError('ExecuteTrajectory action is unavailable')
        self._wait_for_joint_state(startup_timeout)
        # Service availability does not imply that the RGB and depth streams
        # have produced a synchronised pair yet.  Honour the same startup hold
        # used by the single-object demo before asking perception for a
        # snapshot.
        self._hold('demo_start_delay_sec')

        if bool(self.get_parameter('require_sensor_scene').value):
            self._wait_for_sensor_scene(startup_timeout)

    def _run_nearest(self) -> None:
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
            self._publish_grasp_overlay(planned.candidates)
            selected = self._select_reachable(planned.candidates, attempt)
            if selected is None:
                failures.append(f'{attempt.object_id}: unreachable')
                continue
            self._publish_grasp_overlay(
                planned.candidates,
                selected_index=selected.selected_candidate_index,
                selected_arm=selected.selected_arm,
            )
            if bool(self.get_parameter('plan_only').value):
                self.get_logger().info(
                    'NEAREST PLAN COMPLETE (plan_only): no arm/gripper motion'
                )
                return
            self._execute_pregrasp(selected, attempt)
            if bool(self.get_parameter('execute_grasp_and_lift').value):
                # Every failure before physical gripper contact leaves the
                # selected gripper open at the stationary pre-grasp pose.
                self._open_gripper(selected.selected_arm)
                refreshed, refreshed_attempt = self._refresh_selected_grasp(
                    selected, attempt
                )
                self._execute_grasp_and_lift(
                    refreshed, refreshed_attempt
                )
                self.get_logger().info(
                    'NEAREST GRASP COMPLETE: '
                    f'object_id={attempt.object_id} label={attempt.label} '
                    f'distance={attempt.distance_m:.3f}m '
                    f'candidate={refreshed.selected_candidate_index} '
                    f'arm={refreshed.selected_arm}'
                )
                return
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

    def _on_scene_cloud(self, message: PointCloud2) -> None:
        if message.width * message.height:
            self._scene_cloud_stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )

    def _on_scene_cloud_receipt(self, message: Header) -> None:
        stamp = message.stamp.sec*1_000_000_000 + message.stamp.nanosec
        if message.frame_id and stamp > getattr(self, '_scene_commit_stamp_ns', 0):
            self._scene_commit_stamp_ns = stamp
            self._scene_commit_received_at = time.monotonic()
            self._scene_cloud_stamp_ns = max(self._scene_cloud_stamp_ns or 0, stamp)

    def _check_sensor_scene(self) -> None:
        # Humble's service serialization can race live OctoMap writes. The
        # scene publisher takes the OctoMap read lock: consume its output.
        age = float('inf') if self._scene_cloud_stamp_ns is None else (
            self.get_clock().now().nanoseconds - self._scene_cloud_stamp_ns
        ) / 1e9
        # The processed-cloud receipt is published after tree writes. A static
        # populated map need not be serialized again to prove a new integration.
        # Raw cloud arrival alone must never renew this evidence.
        map_age = time.monotonic() - max(self._sensor_map_received_at,
            getattr(self, '_scene_commit_received_at', float('-inf')))
        maximum_age = float(self.get_parameter('sensor_scene_maximum_age_sec').value)
        try:
            validate_sensor_scene(
                max(age, map_age) if age >= 0 else age,
                maximum_age, self._sensor_map_id, self._sensor_map_bytes,
            )
        except RuntimeError as error:
            raise RuntimeError(
                f'{error} (cloud_age={age:.3f}s map_receipt_age={map_age:.3f}s '
                f'maximum={maximum_age:.3f}s)'
            ) from error

    def _on_sensor_scene(self, message: PlanningScene) -> None:
        octomap = message.world.octomap.octomap
        if octomap.data or not message.is_diff:
            self._sensor_map_id = octomap.id
            self._sensor_map_bytes = len(octomap.data)
            self._sensor_map_received_at = time.monotonic()

    def _wait_for_sensor_scene(self, timeout_sec: float, *, after_stamp_ns: int | None = None) -> None:
        self.get_logger().info('Waiting for fresh depth and populated OctoMap')
        deadline = time.monotonic() + timeout_sec
        while True:
            try:
                self._check_sensor_scene()
                if after_stamp_ns is not None and (
                        self._scene_cloud_stamp_ns is None or self._scene_cloud_stamp_ns <= after_stamp_ns):
                    raise RuntimeError('No processed depth capture after the attachment scene update')
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                rclpy.spin_once(self, timeout_sec=0.2)

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
                sorting_category=getattr(item, 'sorting_category', ''),
                sorting_reason=getattr(item, 'sorting_reason', ''),
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
        minimum_opening = float(
            self.get_parameter('minimum_candidate_opening_m').value
        )
        if not math.isfinite(minimum_opening) or minimum_opening < 0.0:
            raise RuntimeError(
                'minimum_candidate_opening_m must be finite and non-negative'
            )
        candidates = [
            candidate
            for candidate in response.candidates
            if float(candidate.required_opening_m) >= minimum_opening
        ]
        if response.success and candidates:
            if len(candidates) != len(response.candidates):
                self.get_logger().info(
                    f'Filtered {len(response.candidates) - len(candidates)} '
                    'candidate(s) below the physical minimum opening '
                    f'{minimum_opening:.3f}m'
                )
            response.candidates = candidates
            return response
        self.get_logger().warning(
            f'Skipping object {attempt.object_id}: grasp generation failed '
            f'code={response.error_code} {response.message}'
        )
        return None

    def _select_reachable(
        self,
        candidates,
        attempt: ObjectAttempt,
        *,
        required_arm: str = '',
    ):
        if bool(self.get_parameter('require_sensor_scene').value):
            # Inference may span multiple sensor updates on a CPU. Drain the
            # subscriptions and wait for genuinely fresh geometry; do not
            # relax the age limit or use an old map to start motion.
            self._wait_for_sensor_scene(5.0)
        goal = SelectReachableGrasp.Goal()
        goal.candidates = candidates
        goal.required_arm = required_arm
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

    @staticmethod
    def _observed_grasp(candidate, key: int) -> ObservedGrasp:
        pose = candidate.tcp_pose
        orientation = pose.orientation
        return ObservedGrasp(
            key=key,
            center=(
                float(candidate.target_object.obb_pose.position.x),
                float(candidate.target_object.obb_pose.position.y),
                float(candidate.target_object.obb_pose.position.z),
            ),
            approach=(
                float(candidate.approach_direction.x),
                float(candidate.approach_direction.y),
                float(candidate.approach_direction.z),
            ),
            closing=quaternion_axis(
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
                (1.0, 0.0, 0.0),
            ),
        )

    def _reinspection_limits(self) -> ReinspectionLimits:
        return ReinspectionLimits(
            maximum_center_shift_m=float(
                self.get_parameter(
                    'reinspection_max_center_shift_m'
                ).value
            ),
            maximum_axis_change_deg=float(
                self.get_parameter(
                    'reinspection_max_axis_change_deg'
                ).value
            ),
            ambiguity_distance_m=float(
                self.get_parameter(
                    'reinspection_ambiguity_distance_m'
                ).value
            ),
        )

    def _refresh_selected_grasp(
        self,
        previous,
        attempt: ObjectAttempt,
    ):
        arm = previous.selected_arm
        self._wait_arm_stationary(arm)
        detected = self._detect_objects()
        reconstructions = []
        for detection in detected.detections.detections:
            if detection.label != attempt.label or not detection.distance_valid:
                continue
            refreshed_attempt = ObjectAttempt(
                object_id=int(detection.object_id),
                label=detection.label,
                confidence=float(detection.confidence),
                distance_m=float(detection.distance_m),
                sorting_category=detection.sorting_category,
                sorting_reason=detection.sorting_reason,
            )
            inspected = self._inspect_selected(
                detected.detections.snapshot_id, refreshed_attempt
            )
            if inspected is not None:
                reconstructions.append((refreshed_attempt, inspected))

        old = self._observed_grasp(previous.selected_candidate, attempt.object_id)
        center_candidates = []
        for refreshed_attempt, inspected in reconstructions:
            center = inspected.objects.objects[0].obb_pose.position
            center_candidates.append(
                ObservedGrasp(
                    key=refreshed_attempt.object_id,
                    center=(center.x, center.y, center.z),
                    approach=old.approach,
                    closing=old.closing,
                )
            )
        associated = associate_refreshed_grasp(
            old, center_candidates, self._reinspection_limits()
        )
        refreshed_attempt, inspected = next(
            pair
            for pair in reconstructions
            if pair[0].object_id == associated.key
        )
        planned = self._plan_grasps(inspected, refreshed_attempt)
        if planned is None:
            raise RuntimeError('refreshed object produced no grasp candidates')
        # Candidate ranking can change after a small segmentation/PCA change.
        # Enforce the existing continuity gate before reachability selection,
        # so an incompatible high-ranked candidate cannot hide a usable one.
        compatible = []
        for candidate in planned.candidates:
            try:
                associate_refreshed_grasp(old, [self._observed_grasp(
                    candidate, refreshed_attempt.object_id)], self._reinspection_limits())
            except ValueError:
                continue
            compatible.append(candidate)
        if not compatible:
            raise RuntimeError('refreshed object has no continuity-compatible grasp candidates')
        planned.candidates = compatible
        # The selector owns a temporary OBB for the refreshed observation.
        # Keeping the original execution OBB here represents the same object
        # twice: jaw contacts are permitted only against the new OBB, so IK
        # sees the old one as an unrelated obstacle. The arm is stationary
        # and no object has been grasped at this handoff.
        self._execution_scene.restore()
        selected = self._select_reachable(
            planned.candidates,
            refreshed_attempt,
            required_arm=arm,
        )
        if selected is None:
            raise RuntimeError(
                f'refreshed grasp is not reachable with required arm {arm}'
            )
        associate_refreshed_grasp(
            old,
            [
                self._observed_grasp(
                    selected.selected_candidate, refreshed_attempt.object_id
                )
            ],
            self._reinspection_limits(),
        )
        self._publish_grasp_overlay(
            planned.candidates,
            selected_index=selected.selected_candidate_index,
            selected_arm=arm,
        )

        # Replace the original snapshot's collision object with the refreshed
        # OBB before contact-enabled Cartesian planning.
        scene_id = (
            'nearest_grasp_target_'
            f'{selected.selected_candidate.snapshot_id}_'
            f'{refreshed_attempt.object_id}'
        )
        self._execution_scene.begin(selected.selected_candidate, scene_id)
        self._execution_scene.allow_contacts_for(arm)
        return selected, refreshed_attempt

    def _wait_arm_stationary(self, arm: str) -> None:
        names = tuple(
            name
            for name in REQUIRED_JOINT_NAMES
            if name.startswith(f'{arm}_') and 'gripper' not in name
        )
        maximum = float(
            self.get_parameter('arm_stationary_velocity_rad_s').value
        )
        required = int(self.get_parameter('arm_stationary_samples').value)
        deadline = time.monotonic() + 5.0
        consecutive = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(
                name in self._joint_velocities
                and abs(self._joint_velocities[name]) <= maximum
                for name in names
            ):
                consecutive += 1
                if consecutive >= required:
                    return
            else:
                consecutive = 0
        raise RuntimeError(f'{arm} arm did not settle before LIN planning')

    def _execute_grasp_and_lift(
        self,
        selected,
        attempt: ObjectAttempt,
    ) -> None:
        gripper_contact = False
        try:
            self._align_refreshed_pregrasp(selected, attempt)
            arm = selected.selected_arm
            approach_start = self._tcp_pose(arm)
            approach_start_joints = (self._arm_joint_state(arm)
                                     if getattr(self, '_use_joint_corridor', False) else None)
            # The selector's 5-DOF IK may only approximate the candidate
            # quaternion. Use FK of the accepted hard-constrained joint state
            # as the exact Pilz endpoint orientation.
            grasp_pose = self._tcp_pose(
                arm, selected.grasp_joint_state
            )
            alignment = linear_approach_error_deg(
                self._pose_position(approach_start),
                self._pose_position(grasp_pose),
                self._observed_grasp(
                    selected.selected_candidate, attempt.object_id
                ).approach,
            )
            maximum_alignment = float(
                self.get_parameter('lin_alignment_tolerance_deg').value
            )
            if alignment >= maximum_alignment:
                raise RuntimeError(
                    f'LIN approach is misaligned by {alignment:.2f}deg '
                    f'(maximum {maximum_alignment:.2f}deg)'
                )
            self.get_logger().info(
                f'Cartesian approach alignment={alignment:.2f}deg '
                f'maximum={maximum_alignment:.2f}deg'
            )
            arm_contact = self._execute_linear(
                arm,
                grasp_pose,
                'refreshed grasp approach',
                velocity_scaling=float(
                    self.get_parameter('approach_velocity_scaling').value
                ),
                contact_target=grasp_pose,
                joint_target=(selected.grasp_joint_state
                              if approach_start_joints is not None else None),
            )
            if arm_contact:
                self.get_logger().info(
                    'Pilz LIN approach stopped on debounced target contact'
                )
            gripper_start = self._joint_positions[f'{arm}_gripper_joint']
            self._grasp_close_override = None
            close_position = self._candidate_close_position(
                selected.selected_candidate
            )
            contact = self._command_gripper(
                arm,
                close_position,
                'close',
                allow_contact_stall=True,
            )
            contact, close_position = self._retry_gripper_contact(
                arm, gripper_start, close_position, contact)
            if not contact:
                raise RuntimeError(
                    f'gripper retention not confirmed from joint feedback on '
                    f'{attempt.label}'
                )
            self._hold('grasp_settle_sec')
            if not self._gripper_contact_stalled(arm, gripper_start, close_position):
                raise RuntimeError('gripper contact did not persist while settling')
            gripper_contact = True
            count_retreat = bool(self.get_parameter('count_retreat_as_lift').value)
            lift_distance = float(self.get_parameter('lift_distance_m').value)
            if not math.isfinite(lift_distance) or lift_distance <= 0.0:
                raise ValueError('lift distance must be finite and positive')
            minimum_object_z = None
            if count_retreat:
                contact_tcp_z = self._tcp_pose(arm).position.z
                initial_object_z = selected.selected_candidate.target_object.obb_pose.position.z
                if not all(math.isfinite(z) for z in (contact_tcp_z, initial_object_z)):
                    raise ValueError('lift reference heights must be finite')
                minimum_object_z = initial_object_z + lift_distance
            self._on_grasp_contact(selected, attempt)
            self._execution_scene.attach_to(arm)
            if bool(self.get_parameter('require_sensor_scene').value):
                self._wait_for_sensor_scene(
                    float(self.get_parameter('attachment_scene_timeout_sec').value),
                    after_stamp_ns=self.get_clock().now().nanoseconds)
            if not getattr(self, '_direct_vertical_lift', False):
                self._execute_linear(
                    arm,
                    approach_start,
                    'reverse grasp retreat',
                    joint_target=approach_start_joints,
                    velocity_scaling=float(
                        self.get_parameter('retreat_velocity_scaling').value
                    ),
                )
            lift_pose = deepcopy(self._tcp_pose(arm) if (
                count_retreat or getattr(self, '_direct_vertical_lift', False)) else approach_start)
            remaining_lift = (max(0.0, contact_tcp_z + lift_distance - lift_pose.position.z)
                              if count_retreat else lift_distance)
            if not math.isfinite(remaining_lift) or not math.isfinite(lift_pose.position.z):
                raise ValueError('lift feedback height must be finite')
            self.get_logger().info(
                f'Lift clearance: count_retreat={count_retreat} '
                f'remaining_tcp_rise={remaining_lift:.4f}m '
                f'required_observed_center_z={minimum_object_z}')
            if remaining_lift > 0.0:
                lift_pose.position.z += remaining_lift
                self._execute_linear(
                    arm, lift_pose, 'vertical grasp lift',
                    velocity_scaling=float(self.get_parameter('retreat_velocity_scaling').value))
            self._on_lift_motion_complete()
            self._hold('lift_hold_sec')
            if count_retreat:
                self._verify_lift_height(attempt, minimum_center_z_m=minimum_object_z)
            else:
                self._verify_lift_height(attempt)
        except Exception:
            if not gripper_contact:
                self._execution_scene.restore()
            raise

    def _on_grasp_contact(self, selected, attempt: ObjectAttempt) -> None:
        """Optional consumer hook after settled contact and before attachment."""

    def _on_lift_motion_complete(self) -> None:
        """Optional observation freshness barrier before the stabilization hold."""

    def _align_refreshed_pregrasp(self, selected, attempt: ObjectAttempt) -> None:
        """A refreshed selection may have planned from a different pregrasp."""
        arm = selected.selected_arm
        current = self._tcp_pose(arm)
        goal = self._tcp_pose(arm, selected.grasp_joint_state)
        error = linear_approach_error_deg(
            self._pose_position(current), self._pose_position(goal),
            self._observed_grasp(selected.selected_candidate, attempt.object_id).approach,
        )
        if error < float(self.get_parameter('lin_alignment_tolerance_deg').value):
            return
        self.get_logger().info(
            f'Repositioning to refreshed pregrasp: current LIN error={error:.2f}deg')
        self._execution_scene.disallow_target_contacts()
        self._move_to(arm, selected.pregrasp_joint_state, 'refreshed pre-grasp')
        self._verify_feedback(selected.pregrasp_joint_state)
        self._confirm_unchanged_target(selected, attempt)
        self._execution_scene.allow_contacts_for(arm)

    @staticmethod
    def _obb_corners(box) -> np.ndarray:
        size = box.obb_size
        pose = box.obb_pose
        extents = np.array((size.x, size.y, size.z))
        center = np.array((pose.position.x, pose.position.y, pose.position.z))
        if not np.isfinite(extents).all() or np.any(extents <= 0) or not np.isfinite(center).all():
            raise ValueError('OBB requires finite center and positive extents')
        q = pose.orientation
        rotation = rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)
        signs = np.array([[x, y, z] for x in (-0.5, 0.5)
                          for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
        return (signs * extents) @ rotation.T + center

    def _confirm_unchanged_target(self, selected, attempt: ObjectAttempt) -> None:
        after_stamp = self.get_clock().now().nanoseconds
        self._wait_arm_stationary(selected.selected_arm)
        detected = self._detect_objects()
        previous = self._obb_corners(selected.selected_candidate.target_object)
        tolerance = float(self.get_parameter('preapproach_maximum_geometry_shift_m').value)
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('pre-approach geometry tolerance must be finite and positive')
        matches = 0
        for detection in detected.detections.detections:
            if detection.label != attempt.label or not detection.distance_valid:
                continue
            fresh_attempt = ObjectAttempt(
                int(detection.object_id), detection.label,
                float(detection.confidence), float(detection.distance_m))
            inspected = self._inspect_selected(detected.detections.snapshot_id, fresh_attempt)
            if inspected is None or len(inspected.objects.objects) != 1:
                continue
            header = inspected.objects.header
            stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            if stamp <= after_stamp or header.frame_id != selected.selected_candidate.header.frame_id:
                continue
            current = self._obb_corners(inspected.objects.objects[0])
            distances = np.linalg.norm(previous[:, None] - current[None, :], axis=2)
            shift = max(float(distances.min(axis=0).max()),
                        float(distances.min(axis=1).max()))
            if shift <= tolerance:
                matches += 1
        if matches != 1:
            raise RuntimeError(
                'target geometry is stale, changed, missing or ambiguous after pregrasp reposition')

    @staticmethod
    def _pose_position(pose: Pose) -> tuple[float, float, float]:
        return (pose.position.x, pose.position.y, pose.position.z)

    def _tcp_pose(
        self, arm: str, override: JointState | None = None
    ) -> Pose:
        if not self._fk.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt FK service is unavailable')
        missing = set(REQUIRED_JOINT_NAMES) - self._joint_positions.keys()
        if missing:
            raise RuntimeError(f'joint feedback is incomplete: {sorted(missing)}')
        state = RobotState()
        state.joint_state.name = list(REQUIRED_JOINT_NAMES)
        positions = {
            name: self._joint_positions[name]
            for name in REQUIRED_JOINT_NAMES
        }
        if override is not None:
            positions.update(
                zip(override.name, override.position, strict=True)
            )
        state.joint_state.position = [positions[name] for name in REQUIRED_JOINT_NAMES]
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = [f'{arm}_grasp_tcp']
        request.robot_state = state
        response = self._future(
            self._fk.call_async(request),
            float(self.get_parameter('tcp_fk_timeout_sec').value), f'{arm} TCP FK'
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 1
        ):
            raise RuntimeError(f'MoveIt could not compute {arm} TCP pose')
        return response.pose_stamped[0].pose

    def _linear_goal(
        self,
        arm: str,
        target: Pose,
        label: str,
        velocity_scaling: float,
    ):
        goal = self._execution_goal(
            arm, self._arm_joint_state(arm), label
        )
        request = goal.request
        request.pipeline_id = str(
            self.get_parameter('pilz_pipeline_id').value
        )
        request.planner_id = str(
            self.get_parameter('pilz_planner_id').value
        )
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('lin_acceleration_scaling').value
        )
        request.goal_constraints = [
            self._pose_constraint(arm, target, label)
        ]
        request.start_state.is_diff = True
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        return goal

    def _arm_joint_state(self, arm: str):
        state = JointState()
        state.name = [
            name
            for name in REQUIRED_JOINT_NAMES
            if name.startswith(f'{arm}_') and 'gripper' not in name
        ]
        state.position = [self._joint_positions[name] for name in state.name]
        return state

    def _joint_corridor_goal(self, arm: str, joints: JointState, start: Pose,
                             target: Pose, label: str, velocity_scaling: float):
        goal = self._execution_goal(arm, joints, label)
        request = goal.request
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('lin_acceleration_scaling').value)
        request.start_state.joint_state.name = list(REQUIRED_JOINT_NAMES)
        request.start_state.joint_state.position = [self._joint_positions[n] for n in REQUIRED_JOINT_NAMES]
        request.start_state.is_diff = True  # keep held-object scene attachments
        corridor = line_corridor(self._pose_position(start), self._pose_position(target),
                                 float(self.get_parameter('lin_position_tolerance_m').value))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = corridor.center
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = corridor.orientation
        constraint = PositionConstraint(link_name=f'{arm}_grasp_tcp', weight=1.)
        constraint.header.frame_id = 'base_link'
        constraint.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[corridor.length, corridor.radius])]
        constraint.constraint_region.primitive_poses = [pose]
        request.path_constraints.position_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        return goal

    @staticmethod
    def _cartesian_pose(pose: Pose) -> CartesianPose:
        p, q = pose.position, pose.orientation
        return CartesianPose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _validate_cartesian_plan(self, arm: str, trajectory, start: Pose,
                                 target: Pose, label: str, *, corridor: bool,
                                 precomputed_samples: list[CartesianPose] | None = None) -> list[CartesianPose]:
        points, names = trajectory.joint_trajectory.points, trajectory.joint_trajectory.joint_names
        if (len(points) < 2 or len(names) != len(set(names)) or
                set(names) != set(self._arm_joint_state(arm).name)):
            raise RuntimeError(f'{label} plan has incomplete arm trajectory')
        for point in points:
            if len(point.positions) != len(names) or not all(math.isfinite(x) for x in point.positions):
                raise RuntimeError(f'{label} plan has invalid joint positions')
        samples = precomputed_samples if precomputed_samples is not None else [self._cartesian_pose(self._tcp_pose(arm, JointState(
            name=list(names), position=list(point.positions))))
                   for point in (points if corridor else [points[-1]])]
        if precomputed_samples is not None and len(samples) != len(points):
            raise RuntimeError('seeded trajectory/FK sample count mismatch')
        position_limit = float(self.get_parameter('lin_position_tolerance_m').value)
        orientation_limit = float(self.get_parameter('lin_orientation_tolerance_rad').value)
        try:
            if corridor:
                lateral, angle = validate_corridor_samples(
                    samples, self._cartesian_pose(start), self._cartesian_pose(target),
                    position_limit, math.radians(float(self.get_parameter(
                        'corridor_orientation_tolerance_deg').value)), orientation_limit)
                self.get_logger().info(f'{label} FK corridor verified: lateral={lateral:.6f}m '
                                       f'orientation={math.degrees(angle):.3f}deg samples={len(samples)}')
            else:
                # Position-only KDL can report success while ignoring the
                # requested Pilz orientation. Never execute such an endpoint.
                validate_pose_endpoint(samples[-1], self._cartesian_pose(target),
                                       position_limit, orientation_limit)
        except ValueError as error:
            raise RuntimeError(f'{label} plan rejected before execution: {error}') from error
        return samples

    def _slow_corridor_plan(self, trajectory, samples: list[CartesianPose],
                            velocity_scaling: float, *, acceleration_scaling: float | None = None) -> None:
        points = trajectory.joint_trajectory.points
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        factor = sampled_time_scale(
            samples, times,
            float(self.get_parameter('cartesian_translation_speed_m_s').value) * velocity_scaling,
            float(self.get_parameter('cartesian_rotation_speed_rad_s').value) * velocity_scaling,
            float(self.get_parameter('cartesian_translation_acceleration_m_s2').value) *
            (float(self.get_parameter('lin_acceleration_scaling').value)
             if acceleration_scaling is None else acceleration_scaling),
            float(self.get_parameter('corridor_time_margin').value))
        for point, stamp in zip(points, times, strict=True):
            point.time_from_start = Duration(nanoseconds=round(stamp * factor * 1e9)).to_msg()
            point.velocities = [x/factor for x in point.velocities]
            point.accelerations = [x/factor**2 for x in point.accelerations]
        self.get_logger().info(f'Corridor uniform time scale={factor:.3f} '
                               f'duration={times[-1]*factor:.3f}s; geometry unchanged')

    def _pose_constraint(
        self, arm: str, target: Pose, label: str
    ) -> Constraints:
        tolerance = float(
            self.get_parameter('lin_position_tolerance_m').value
        )
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [tolerance]
        position = PositionConstraint()
        position.header.frame_id = 'base_link'
        position.link_name = f'{arm}_grasp_tcp'
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [deepcopy(target)]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'base_link'
        orientation.link_name = f'{arm}_grasp_tcp'
        orientation.orientation = deepcopy(target.orientation)
        angular = float(
            self.get_parameter('lin_orientation_tolerance_rad').value
        )
        orientation.absolute_x_axis_tolerance = angular
        orientation.absolute_y_axis_tolerance = angular
        orientation.absolute_z_axis_tolerance = angular
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.name = f'pilz_lin_{label}'
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def _execute_linear(
        self,
        arm: str,
        target: Pose,
        label: str,
        *,
        velocity_scaling: float,
        contact_target: Pose | None = None,
        joint_target: JointState | None = None,
    ) -> bool:
        self._wait_arm_stationary(arm)
        start = self._tcp_pose(arm)
        goal = (self._linear_goal(arm, target, label, velocity_scaling)
                if joint_target is None else self._joint_corridor_goal(
                    arm, joint_target, start, target, label, velocity_scaling))
        seeded = getattr(self, '_seeded_cartesian', None) if joint_target is not None else None
        if seeded is not None:
            goal.request.pipeline_id = 'cleany_seeded_cartesian'
            goal.request.planner_id = 'monotone_cubic'
        trace = getattr(self, '_record_pipeline_message', None)
        if trace is not None:
            trace('motion_request', goal)
        precomputed = None
        if seeded is not None:
            result = MoveGroup.Result()
            begin = time.monotonic()
            try:
                result.planned_trajectory, precomputed = seeded.plan(
                    goal.request, self._cartesian_pose(start), self._cartesian_pose(target))
                result.error_code.val = MoveItErrorCodes.SUCCESS
                result.trajectory_start = goal.request.start_state
            except Exception:
                result.error_code.val = MoveItErrorCodes.FAILURE
                raise
            finally:
                result.planning_time = time.monotonic()-begin
                if trace is not None:
                    trace('motion_result', result)
        else:
            plan_handle = self._future(
                self._move_group.send_goal_async(goal), 10.0, f'{label} Cartesian plan response')
            if not plan_handle.accepted:
                raise RuntimeError(f'{label} Cartesian plan was rejected')
            planned = self._future(
                plan_handle.get_result_async(), 60.0, f'{label} Cartesian plan'
            )
            result = planned.result
            if trace is not None:
                trace('motion_result', result)
            if (planned.status != GoalStatus.STATUS_SUCCEEDED or
                    result.error_code.val != MoveItErrorCodes.SUCCESS):
                raise RuntimeError(f'{label} Cartesian planning failed: code={result.error_code.val}')
        samples = self._validate_cartesian_plan(
            arm, result.planned_trajectory, start, target,
            label, corridor=joint_target is not None, precomputed_samples=precomputed)
        if joint_target is not None:
            self._slow_corridor_plan(result.planned_trajectory, samples, velocity_scaling,
                acceleration_scaling=goal.request.max_acceleration_scaling_factor)
        if trace is not None:
            trace('motion_plan', result.planned_trajectory)
        last_time = result.planned_trajectory.joint_trajectory.points[-1].time_from_start
        planned_duration = last_time.sec + last_time.nanosec / 1e9
        execution_timeout = execution_wall_timeout(planned_duration,
            float(self.get_parameter('cartesian_execution_wall_timeout_factor').value),
            float(self.get_parameter('cartesian_execution_wall_timeout_margin_sec').value),
            float(self.get_parameter('cartesian_execution_wall_timeout_minimum_sec').value))
        self.get_logger().info(f'{label} execution deadline: trajectory={planned_duration:.3f}s '
                               f'wall_timeout={execution_timeout:.3f}s')
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = result.planned_trajectory
        enforce_motion_guard(self)
        execute_handle = self._future(
            self._execute_trajectory.send_goal_async(execute_goal),
            10.0,
            f'{label} execution response',
        )
        if not execute_handle.accepted:
            raise RuntimeError(f'{label} trajectory execution was rejected')
        contact = self._contact_monitor() if contact_target is not None else None
        last_sample = self._controller_state_counts[arm]
        result_future = execute_handle.get_result_async()
        deadline = time.monotonic() + execution_timeout
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            enforce_motion_guard(self, execute_handle, result_future, label)
            count = self._controller_state_counts[arm]
            if contact is None or count == last_sample:
                continue
            last_sample = count
            sample = self._contact_sample(arm, contact_target)
            if sample is not None and contact.update(sample):
                self._future(
                    execute_handle.cancel_goal_async(),
                    5.0,
                    f'{label} contact cancellation',
                )
                self._future(
                    result_future, 10.0, f'{label} canceled result'
                )
                return True
        if not result_future.done():
            self._future(
                execute_handle.cancel_goal_async(), 5.0, f'{label} timeout cancel'
            )
            self._future(result_future, 10.0, f'{label} timeout terminal result')
            raise RuntimeError(f'{label} trajectory execution timed out')
        enforce_motion_guard(self, execute_handle, result_future, label)
        wrapped = result_future.result()
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f'{label} trajectory execution failed: '
                f'status={wrapped.status} code={wrapped.result.error_code.val}'
            )
        return False

    def _contact_monitor(self) -> ContactDebounce:
        effort = float(self.get_parameter('contact_effort_threshold').value)
        return ContactDebounce(
            ContactLimits(
                maximum_tcp_distance_m=float(
                    self.get_parameter(
                        'grasp_contact_stop_max_distance_m'
                    ).value
                ),
                minimum_joint_error_rad=float(
                    self.get_parameter(
                        'contact_min_joint_error_rad'
                    ).value
                ),
                maximum_joint_velocity_rad_s=float(
                    self.get_parameter(
                        'contact_max_joint_velocity_rad_s'
                    ).value
                ),
                consecutive_samples=int(
                    self.get_parameter(
                        'contact_consecutive_samples'
                    ).value
                ),
                effort_threshold=effort if effort > 0.0 else None,
            )
        )

    def _contact_sample(
        self, arm: str, target: Pose
    ) -> ContactSample | None:
        state = self._controller_states.get(arm)
        if state is None:
            return None
        if (
            len(state.reference.positions) == len(state.feedback.positions)
            and state.reference.positions
        ):
            errors = tuple(
                float(reference) - float(feedback)
                for reference, feedback in zip(
                    state.reference.positions,
                    state.feedback.positions,
                    strict=True,
                )
            )
        else:
            errors = tuple(float(value) for value in state.error.positions)
        velocities = tuple(float(value) for value in state.feedback.velocities)
        if not errors or not velocities:
            return None
        effort = (
            tuple(float(value) for value in state.feedback.effort)
            if state.feedback.effort
            else None
        )
        # FK is queried only for a plausible stopped/error sample.
        distance = math.inf
        effort_threshold = float(
            self.get_parameter('contact_effort_threshold').value
        )
        plausible_signal = (
            max(abs(value) for value in errors)
            >= float(
                self.get_parameter('contact_min_joint_error_rad').value
            )
            or (
                effort_threshold > 0.0
                and effort is not None
                and max(abs(value) for value in effort) >= effort_threshold
            )
        )
        if plausible_signal and max(abs(value) for value in velocities) <= float(
            self.get_parameter(
                'contact_max_joint_velocity_rad_s'
            ).value
        ):
            distance = math.dist(
                self._pose_position(self._tcp_pose(arm)),
                self._pose_position(target),
            )
        return ContactSample(distance, errors, velocities, effort)

    def _verify_lift_height(self, attempt: ObjectAttempt, *,
                            minimum_center_z_m: float | None = None):
        minimum = float(
            self.get_parameter('lift_min_center_z_m').value
        )
        if not math.isfinite(minimum):
            raise ValueError('configured lift height must be finite')
        if minimum_center_z_m is not None:
            if not math.isfinite(minimum_center_z_m):
                raise ValueError('observed lift height must be finite')
            minimum = max(minimum, minimum_center_z_m)
        if minimum <= 0.0 and minimum_center_z_m is None:
            return
        detected = self._detect_objects()
        match = next(
            (
                item
                for item in detected.detections.detections
                if item.label == attempt.label and item.distance_valid
            ),
            None,
        )
        if match is None:
            raise LiftRedetectionError(
                f'lift verification could not redetect {attempt.label}'
            )
        post_lift = ObjectAttempt(
            object_id=int(match.object_id),
            label=match.label,
            confidence=float(match.confidence),
            distance_m=float(match.distance_m),
        )
        inspected = self._inspect_selected(
            detected.detections.snapshot_id,
            post_lift,
        )
        if inspected is None:
            raise RuntimeError(
                f'lift verification could not reconstruct {attempt.label}'
            )
        center_z = float(inspected.objects.objects[0].obb_pose.position.z)
        self.get_logger().info(
            f'Lift verification: label={attempt.label} '
            f'center_z={center_z:.3f}m minimum={minimum:.3f}m'
        )
        if not math.isfinite(center_z) or center_z < minimum:
            raise RuntimeError(
                f'{attempt.label} was not retained after lift: '
                f'center_z={center_z:.3f}m minimum={minimum:.3f}m'
            )
        return inspected

    def _open_gripper(self, arm: str) -> None:
        self._command_gripper(
            arm,
            float(self.get_parameter('gripper_open_position_rad').value),
            'open',
        )

    def _command_gripper(
        self,
        arm: str,
        position: float,
        command: str,
        *,
        allow_contact_stall: bool = False,
        motion_seconds: float | None = None,
        contact_start: float | None = None,
    ) -> bool:
        client = self._grippers[arm]
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'{arm} gripper controller is unavailable')
        goal = FollowJointTrajectory.Goal()
        joint = f'{arm}_gripper_joint'
        start = self._joint_positions.get(joint, math.nan)
        goal.trajectory.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [position]
        motion_sec = float(self.get_parameter('gripper_motion_sec').value)
        if motion_seconds is not None:
            motion_sec = motion_seconds
        if not math.isfinite(motion_sec) or motion_sec <= 0:
            raise ValueError('Gripper motion duration must be finite and positive')
        point.time_from_start = Duration(
            seconds=motion_sec
        ).to_msg()
        goal.trajectory.points = [point]
        if allow_contact_stall:
            tolerance = JointTolerance(name=joint, position=-1.0)
            goal.path_tolerance = [tolerance]
            goal.goal_tolerance = [tolerance]
        handle = self._future(
            client.send_goal_async(goal),
            5.0,
            f'{arm} gripper {command} goal response',
        )
        if not handle.accepted:
            raise RuntimeError(
                f'{arm} gripper {command} command was rejected'
            )
        timeout_factor = float(self.get_parameter('gripper_wall_timeout_factor').value)
        if not math.isfinite(timeout_factor) or timeout_factor < 1:
            raise ValueError('Gripper wall timeout factor must be finite and >= 1')
        wrapped = self._future(
            handle.get_result_async(),
            max(10.0, timeout_factor * motion_sec + 5.0),
            f'{arm} gripper {command} result',
        )
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        actual = self._joint_positions.get(joint, math.inf)
        self.get_logger().info(f'GRIPPER FEEDBACK arm={arm} operation={command} start={start:.5f} '
            f'actual={actual:.5f} command={position:.5f} residual={actual-position:.5f} '
            f'velocity={self._joint_velocities.get(joint, math.nan):.5f}')
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if allow_contact_stall and self._gripper_contact_stalled(
            arm, start if contact_start is None else contact_start, position
        ):
            self.get_logger().info(
                f'{arm} gripper contact detected: actual={actual:.3f}rad '
                f'command={position:.3f}rad'
            )
            return True
        if not succeeded:
            raise RuntimeError(
                f'{arm} gripper {command} failed: status={wrapped.status} '
                f'code={wrapped.result.error_code}'
            )
        if abs(actual - position) > 0.05:
            raise RuntimeError(
                f'{arm} gripper {command} feedback error='
                f'{abs(actual - position):.3f} rad'
            )
        return False

    def _gripper_contact_stalled(
        self, arm: str, start: float, command: float, *, allow_closing_motion: bool = False
    ) -> bool:
        joint = f'{arm}_gripper_joint'
        return is_gripper_contact_stall(
            allow_closing_motion=allow_closing_motion,
            start=start,
            actual=self._joint_positions.get(joint, math.inf),
            command=command,
            velocity=self._joint_velocities.get(joint, math.inf),
            minimum_motion=float(
                self.get_parameter('gripper_contact_min_motion_rad').value
            ),
            minimum_residual=float(
                self.get_parameter('gripper_contact_min_residual_rad').value
            ),
            maximum_velocity=float(
                self.get_parameter(
                    'gripper_contact_max_velocity_rad_s'
                ).value
            ),
        )

    def _retry_gripper_contact(
        self, arm: str, start: float, position: float, contact: bool,
    ) -> tuple[bool, float]:
        for retry in range(getattr(self, '_gripper_retry_steps', 0)):
            if contact:
                break
            step = float(self.get_parameter('gripper_contact_retry_step_rad').value)
            if not math.isfinite(step) or not 0 < step <= .2:
                raise ValueError('Gripper contact retry step must be in (0, 0.2] rad')
            lower = float(self.get_parameter('gripper_close_position_rad').value)
            next_position = max(lower, position - step)
            if next_position >= position:
                break
            position = next_position
            self._grasp_close_override = position
            self.get_logger().info(
                f'Bounded gripper contact retry {retry + 1}: command={position:.4f}rad')
            contact = self._command_gripper(
                arm, position, 'close', allow_contact_stall=True,
                motion_seconds=float(self.get_parameter('gripper_contact_retry_motion_sec').value),
                contact_start=start)
        return contact, position

    def _candidate_close_position(self, candidate) -> float:
        if getattr(self, '_grasp_close_override', None) is not None:
            return self._grasp_close_override
        fallback = float(
            self.get_parameter('gripper_close_position_rad').value
        )
        if bool(self.get_parameter('gripper_force_full_close').value):
            return fallback
        return opening_to_gripper_position(
            required_opening_m=float(candidate.required_opening_m),
            opening_reduction_m=float(
                self.get_parameter(
                    'gripper_close_opening_reduction_m'
                ).value
            ),
            reference_aperture_m=float(
                self.get_parameter('gripper_aperture_reference_m').value
            ),
            reference_position_rad=float(
                self.get_parameter(
                    'gripper_aperture_reference_position_rad'
                ).value
            ),
            aperture_m_per_rad=float(
                self.get_parameter('gripper_aperture_m_per_rad').value
            ),
            minimum_position_rad=fallback,
            maximum_position_rad=float(
                self.get_parameter('gripper_open_position_rad').value
            ),
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
        node.get_logger().fatal(f'NEAREST GRASP PIPELINE FAILED: {error}')
        if bool(
            node.get_parameter('keep_debug_image_alive_on_failure').value
        ):
            node.get_logger().warning(
                'Keeping the failed pipeline alive so the last grasp debug '
                'image remains available'
            )
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
        else:
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
