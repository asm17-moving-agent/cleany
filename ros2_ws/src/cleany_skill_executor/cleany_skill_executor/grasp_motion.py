"""인식 및 파지 작업을 위한 ROS 자원과 공통 연산을 관리합니다."""
from __future__ import annotations

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
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
import numpy as np
from cleany_skill_executor.core.rgbd_projection import (
    CameraProjection,
    render_grasp_overlay,
    rotation_matrix_from_quaternion,
)
from cleany_skill_executor.core.nearest_object import (
    ObjectAttempt,
    rank_object_attempts,
)
from cleany_skill_executor.core.sensor_scene import validate_sensor_scene
from cleany_skill_executor.core.gripper import approach_offset_for_label, aperture_centering_offset
from cleany_skill_executor.core.grasp_selection import (
    REQUIRED_JOINT_NAMES,
    quaternion_axis,
)
from cleany_skill_executor.core.grasp_pipeline import ObservedGrasp, ReinspectionLimits, associate_refreshed_grasp, linear_approach_error_deg
from cleany_skill_executor.grasp_execution import GraspExecutionNode
from cleany_skill_executor.pick_operations import GraspProgress
from cleany_skill_executor.planning_scene import TargetSceneTransaction
from cleany_skill_executor.collision_geometry_cache import CollisionGeometryCache, subscribe_collision_geometry
from cleany_skill_executor.seeded_cartesian import SeededCartesianConfig, SeededCartesianPlanner, load_joint_motion_limits
from .cartesian_motion import CartesianMotion
from .gripper_operations import GripperOperations


class GraspMotionOperations(CartesianMotion, GripperOperations, GraspExecutionNode):
    """인식, 파지 선택과 보호된 이동 연산을 제공합니다. 실행 순서는 호출자가 결정합니다.

    GraspExecutionNode의 관절 피드백과 MoveIt 및 선택 Client를 상속합니다.

    Subscriptions:
        /monitored_planning_scene (moveit_msgs/msg/PlanningScene): 장면 반영.
        /perception/scene_cloud_filtered (sensor_msgs/msg/PointCloud2): 최신 점군.
            receipt 토픽을 설정하면 std_msgs/msg/Header 구독으로 대체합니다.
        /perception/debug_image_latched (sensor_msgs/msg/Image): overlay 배경.
        /cleany/internal/mujoco/left_wrist_camera/camera_info
            (sensor_msgs/msg/CameraInfo): overlay 보정값.
        /{left,right}_arm_controller/controller_state
            (control_msgs/msg/JointTrajectoryControllerState): 실행 및 접촉 피드백.
        /robot_description (std_msgs/msg/String): 선택적 seed 기반 Cartesian 계획.
        /grasp/collision_geometry (cleany_interfaces/msg/ObservedObjectGeometry):
            선택적 관측 메시 입력. collision_geometry_topic 기본값이며
            reliable, transient_local입니다.

    Publishers:
        /grasp/debug_image_latched (sensor_msgs/msg/Image): 후보 overlay.

    Services:
        /grasp/plan (cleany_interfaces/srv/PlanGrasp): 파지 후보.
        /compute_fk (moveit_msgs/srv/GetPositionFK): 측정 TCP 자세.
        /compute_ik (moveit_msgs/srv/GetPositionIK), /check_state_validity
            (moveit_msgs/srv/GetStateValidity): 선택적 seed 기반 Cartesian 검사.
        /apply_planning_scene (moveit_msgs/srv/ApplyPlanningScene),
        /get_planning_scene (moveit_msgs/srv/GetPlanningScene):
            TargetSceneTransaction을 통한 임시 형상과 접촉 허용 갱신.

    Actions:
        /perception/inspect_scene (cleany_interfaces/action/InspectScene): 검사 Client.
        /execute_trajectory (moveit_msgs/action/ExecuteTrajectory): 궤적 Client.
        /{left,right}_gripper_controller/follow_joint_trajectory
            (control_msgs/action/FollowJointTrajectory): 그리퍼 Client.

    TF:
        카메라 및 TCP 변환을 읽으며 발행하지 않습니다.

    QoS:
        영상과 robot description은 transient_local이며 카메라 및 점군 입력은
        sensor_data입니다. 선택적 receipt 입력은 reliable, depth 1입니다.

    Notes:
        위 검사, 파지, 장면, 점군, 영상과 카메라 이름은 파라미터 기본값입니다.
        파지 및 들기 실행은 선택 사항이며 plan_only는 실행을 막습니다.
        필요한 경우 장면 수신 확인과 장면 반영을 별도로 검사합니다.
    """


    def __init__(self) -> None:
        super().__init__(node_name='nearest_pregrasp_coordinator')
        self.declare_parameter(
            'inspection_action',
            '/perception/inspect_scene',
        )
        self.declare_parameter('grasp_service', '/grasp/plan')
        self.declare_parameter(
            'query',
            'Detect the paper cup, LEGO brick, crumpled tissue, and mouse on the table.',
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
        self.declare_parameter('gripper_contact_feedback_timeout_sec', 5.0)
        self.declare_parameter('gripper_contact_stable_duration_sec', 0.10)
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
        self.declare_parameter('direct_vertical_lift_extra_m', 0.0)
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
        self.declare_parameter('publish_grasp_debug_overlay', True)
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
        self.declare_parameter('deeper_grasp_labels', [''])
        self.declare_parameter('deeper_grasp_offsets_m', [0.0])
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
            spin_once=lambda duration: self._spin_once(timeout_sec=duration),
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
        debug_period = float(self.get_parameter('grasp_debug_republish_period_sec').value)
        self._grasp_debug_timer = (
            self.create_timer(
                max(0.05, debug_period),
                self._republish_grasp_debug_image,
                clock=self._grasp_debug_clock,
            ) if debug_period > 0 else None
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
        if not hasattr(self, '_gripper_feedback_stamps'):
            self._gripper_feedback_stamps = {}
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        for joint in message.name:
            if joint.endswith('_gripper_joint'):
                self._gripper_feedback_stamps[joint] = stamp
                self._joint_velocities[joint] = math.inf
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
            + approach_offset_for_label(
                candidate.target_object.label,
                float(self.get_parameter('selector_grasp_approach_offset_m').value),
                list(self.get_parameter('deeper_grasp_labels').value),
                list(self.get_parameter('deeper_grasp_offsets_m').value))
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
        """RGB 화면 위에 후보 pregrasp와 grasp TCP 점을 발행합니다."""
        if not self.get_parameter('publish_grasp_debug_overlay').value:
            return
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
        self.get_logger().debug(
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
                # launch 종료 중 MoveGroup이 먼저 멈출 수 있습니다.
                except Exception as error:
                    self.get_logger().warning(
                        'Could not restore target scene during shutdown: '
                        f'{error}'
                    )
        finally:
            super().destroy_node()


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
        # 서비스 가용성이 RGB와 깊이의 동기 쌍 생성까지 의미하지는 않습니다.
        # 인식 snapshot 요청 전에 단일 물체 데모와 동일한 시작 대기를 적용합니다.
        self._hold('demo_start_delay_sec')

        if bool(self.get_parameter('require_sensor_scene').value):
            self._wait_for_sensor_scene(startup_timeout)


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
        # Humble 서비스 직렬화는 실시간 OctoMap 쓰기와 경쟁할 수 있습니다.
        # OctoMap 읽기 잠금을 사용하는 장면 publisher의 출력을 소비합니다.
        age = float('inf') if self._scene_cloud_stamp_ns is None else (
            self.get_clock().now().nanoseconds - self._scene_cloud_stamp_ns
        ) / 1e9
        # 처리된 점군 수신 확인은 트리 쓰기 이후 발행됩니다. 내용이 같은 정적 지도도
        # 새 통합을 입증하려고 다시 직렬화할 필요는 없습니다.
        # 원시 점군 도착만으로 이 근거를 갱신하지 않습니다.
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
        started = time.monotonic()
        deadline = started + timeout_sec
        while True:
            try:
                self._check_sensor_scene()
                if after_stamp_ns is not None and (
                        self._scene_cloud_stamp_ns is None or self._scene_cloud_stamp_ns <= after_stamp_ns):
                    raise RuntimeError('No processed depth capture after the attachment scene update')
                if after_stamp_ns is not None:
                    self.get_logger().debug(
                        f'Post-attachment depth ready: wait_wall_sec={time.monotonic()-started:.3f} '
                        f'capture_after_attachment_sec={(self._scene_cloud_stamp_ns-after_stamp_ns)/1e9:.3f}')
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self._spin_once(timeout_sec=0.2)


    def _detect_objects(self):
        return self._finish_object_detection(self._begin_object_detection())


    def _begin_object_detection(self):
        """추론을 기다리지 않고 검출을 제출한 handle/future를 반환합니다."""
        goal = InspectScene.Goal()
        goal.query = str(self.get_parameter('query').value)
        goal.tracking_session_id = getattr(self, '_tracking_session_id', '')
        handle = self._future(
            self._inspection.send_goal_async(goal),
            10.0,
            'inspection detection goal response',
        )
        if not handle.accepted:
            raise RuntimeError('inspection detection goal was rejected')
        return handle, handle.get_result_async()


    def _finish_object_detection(self, pending):
        timeout = float(self.get_parameter('inspection_timeout_sec').value)
        wrapped = self._future(
            pending[1],
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
        goal.tracking_session_id = getattr(self, '_tracking_session_id', '')
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
        request.target_object_header = inspected.objects.header
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
            # CPU 추론 중 센서가 여러 번 갱신될 수 있습니다. 구독을 처리하고 실제로
            # 최신인 형상을 기다리며 경과 시간 한계를 완화하거나 오래된 지도로
            # 이동을 시작하지 않습니다.
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


    def _validate_approved_tracking(self, observation) -> None:
        approved = getattr(self, '_approved_tracking', None)
        if approved is None:
            return
        if ((observation.tracking_session_id, observation.tracking_epoch) != approved[:2]
                or sum(d.track_id == approved[2] and d.tracking_state == 'TRACKED'
                       for d in observation.detections) != 1):
            raise ValueError('Approved track is missing, ambiguous or its epoch changed')


    def _matches_approved_tracking(self, detection) -> bool:
        approved = getattr(self, '_approved_tracking', None)
        return approved is None or (detection.track_id == approved[2] and detection.tracking_state == 'TRACKED')


    def _refresh_selected_grasp(
        self,
        previous,
        attempt: ObjectAttempt,
        *,
        match_label: bool = True,
    ):
        arm = previous.selected_arm
        self._wait_arm_stationary(arm)
        detected = self._detect_objects()
        if getattr(self, '_approved_tracking', None) is not None:
            self._validate_approved_tracking(detected.detections)
        reconstructions = []
        label_key = getattr(self, '_target_label_key', lambda label: label)
        if (getattr(self, '_require_unique_target_label', False)
                and sum(label_key(d.label) == label_key(attempt.label)
                        for d in detected.detections.detections) != 1):
            raise ValueError('Approved object type is missing or duplicated in refreshed snapshot')
        for detection in detected.detections.detections:
            if getattr(self, '_approved_tracking', None) is not None and not self._matches_approved_tracking(detection):
                continue
            if (match_label and label_key(detection.label) != label_key(attempt.label)) or not detection.distance_valid:
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
        if (not match_label and refreshed_attempt.sorting_category
                != attempt.sorting_category):
            raise RuntimeError('refreshed object sorting category changed')
        planned = self._plan_grasps(inspected, refreshed_attempt)
        if planned is None:
            raise RuntimeError('refreshed object produced no grasp candidates')
        # 작은 분할 또는 PCA 변화로 후보 순위가 바뀔 수 있습니다. 도달 가능성
        # 선택 전에 기존 연속성 검사를 적용해 호환되지 않는 상위 후보가
        # 사용 가능한 후보를 가리지 않게 합니다.
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
        # 선택기는 새 관측의 임시 OBB를 소유합니다. 여기서 원래 실행 OBB를 유지하면
        # 같은 물체를 두 번 표현합니다. 그리퍼 접촉은 새 OBB에만 허용되어 IK가
        # 이전 OBB를 별도 장애물로 보게 됩니다. 이 전달 시점에는 팔이 정지해 있고
        # 아직 물체를 잡지 않았습니다.
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

        # 접촉을 허용한 Cartesian 계획 전에 원래 snapshot의 충돌 물체를
        # 새 OBB로 교체합니다.
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
            self._spin_once(timeout_sec=0.05)
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


    def _execute_grasp_and_lift(self, selected, attempt: ObjectAttempt) -> None:
        progress = GraspProgress()
        try:
            self._approach_grasp(selected, attempt, progress)
            self._close_grasp(selected, attempt, progress)
            self._confirm_grasp_contact(selected, attempt, progress)
            self._lift_grasp(selected, attempt, progress)
            self._confirm_held_grasp(selected, attempt, progress)
        except Exception:
            if not progress.confirmed:
                self._execution_scene.restore()
            raise


    def _approach_grasp(self, selected, attempt, progress: GraspProgress) -> None:
        self._align_refreshed_pregrasp(selected, attempt)
        arm = selected.selected_arm
        approach_start = self._tcp_pose(arm)
        approach_start_joints = (self._arm_joint_state(arm)
                                 if getattr(self, '_use_joint_corridor', False) else None)
        # 선택기의 5자유도 IK는 후보 quaternion을 근사할 수 있습니다.
        # 엄격한 제약을 통과한 관절 상태의 FK를 정확한 Pilz 끝점 방향으로 사용합니다.
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
        self.get_logger().debug(
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
        progress.approach_start = approach_start
        progress.approach_start_joints = approach_start_joints


    def _close_grasp(self, selected, attempt, progress: GraspProgress, *, retry=True) -> None:
        arm = selected.selected_arm
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
        if retry:
            contact, close_position = self._retry_gripper_contact(
                arm, gripper_start, close_position, contact)
        if not contact:
            raise RuntimeError(
                f'gripper retention not confirmed from joint feedback on '
                f'{attempt.label}'
            )
        progress.gripper_start = gripper_start
        progress.close_position = close_position


    def _confirm_grasp_contact(self, selected, attempt, progress: GraspProgress) -> None:
        arm = selected.selected_arm
        gripper_start, close_position = progress.gripper_start, progress.close_position
        self._hold('grasp_settle_sec')
        if not self._gripper_contact_stalled(arm, gripper_start, close_position):
            raise RuntimeError('gripper contact did not persist while settling')
        progress.confirmed = True
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
        progress.count_retreat = count_retreat
        progress.contact_tcp_z = contact_tcp_z if count_retreat else 0.0
        progress.minimum_object_z = minimum_object_z


    def _lift_grasp(self, selected, attempt, progress: GraspProgress) -> None:
        if not progress.confirmed:
            raise RuntimeError('Lift requires confirmed gripper contact')
        arm = selected.selected_arm
        approach_start, approach_start_joints = progress.approach_start, progress.approach_start_joints
        count_retreat, contact_tcp_z = progress.count_retreat, progress.contact_tcp_z
        minimum_object_z = progress.minimum_object_z
        lift_distance = float(self.get_parameter('lift_distance_m').value)
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
        extra_lift = (float(self.get_parameter('direct_vertical_lift_extra_m').value)
                      if getattr(self, '_direct_vertical_lift', False) else 0.0)
        if not math.isfinite(extra_lift) or not 0.0 <= extra_lift <= 0.05:
            raise ValueError('direct_vertical_lift_extra_m must be in [0, 0.05]')
        remaining_lift += extra_lift
        if not math.isfinite(remaining_lift) or not math.isfinite(lift_pose.position.z):
            raise ValueError('lift feedback height must be finite')
        self.get_logger().debug(
            f'Lift clearance: count_retreat={count_retreat} '
            f'remaining_tcp_rise={remaining_lift:.4f}m '
            f'direct_extra={extra_lift:.4f}m '
            f'required_observed_center_z={minimum_object_z}')
        if remaining_lift > 0.0:
            lift_pose.position.z += remaining_lift
            self._execute_linear(
                arm, lift_pose, 'vertical grasp lift',
                velocity_scaling=float(self.get_parameter('retreat_velocity_scaling').value))
        self._on_lift_motion_complete()


    def _confirm_held_grasp(self, selected, attempt, progress: GraspProgress) -> None:
        count_retreat, minimum_object_z = progress.count_retreat, progress.minimum_object_z
        self._hold('lift_hold_sec')
        if count_retreat:
            self._verify_lift_height(attempt, minimum_center_z_m=minimum_object_z)
        else:
            self._verify_lift_height(attempt)


    def _on_grasp_contact(self, selected, attempt: ObjectAttempt) -> None:
        """접촉 정착 후 부착 전에 사용하는 선택적 소비자 hook입니다."""


    def _on_lift_motion_complete(self) -> None:
        """안정화 유지 전에 적용하는 선택적 관측 최신성 확인입니다."""


    def _align_refreshed_pregrasp(self, selected, attempt: ObjectAttempt) -> None:
        """갱신된 선택이 다른 pregrasp에서 계획되었을 수 있습니다."""
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
        if getattr(self, '_approved_tracking', None) is not None:
            self._validate_approved_tracking(detected.detections)
        previous = self._obb_corners(selected.selected_candidate.target_object)
        tolerance = float(self.get_parameter('preapproach_maximum_geometry_shift_m').value)
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('pre-approach geometry tolerance must be finite and positive')
        matches = 0
        evidence = []
        label_key = getattr(self, '_target_label_key', lambda label: label)
        label_count = sum(label_key(d.label) == label_key(attempt.label)
                          for d in detected.detections.detections)
        for detection in detected.detections.detections:
            if getattr(self, '_approved_tracking', None) is not None and not self._matches_approved_tracking(detection):
                continue
            if label_key(detection.label) != label_key(attempt.label) or not detection.distance_valid:
                continue
            fresh_attempt = ObjectAttempt(
                int(detection.object_id), detection.label,
                float(detection.confidence), float(detection.distance_m))
            inspected = self._inspect_selected(detected.detections.snapshot_id, fresh_attempt)
            if inspected is None or len(inspected.objects.objects) != 1:
                evidence.append('reconstruction unavailable')
                continue
            header = inspected.objects.header
            stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            if stamp <= after_stamp or header.frame_id != selected.selected_candidate.header.frame_id:
                evidence.append(f'stamp={stamp} barrier={after_stamp} frame={header.frame_id}')
                continue
            current = self._obb_corners(inspected.objects.objects[0])
            distances = np.linalg.norm(previous[:, None] - current[None, :], axis=2)
            shift = max(float(distances.min(axis=0).max()),
                        float(distances.min(axis=1).max()))
            evidence.append(f'geometry_shift={shift:.4f}m maximum={tolerance:.4f}m')
            if shift <= tolerance:
                matches += 1
        if matches != 1 or (getattr(self, '_require_unique_target_label', False) and label_count != 1):
            raise RuntimeError(
                'target geometry is stale, changed, missing or ambiguous after pregrasp reposition: '
                f'label_count={label_count}; ' + '; '.join(evidence))
