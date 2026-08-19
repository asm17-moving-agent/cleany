"""Detect a MuJoCo can from RGB-D, generate grasps, select, and execute."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from typing import Any

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import DetectedObject3D, GraspCandidate
from cleany_interfaces.srv import PlanGrasp
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from moveit_msgs.msg import (
    CollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetPositionIK
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from cleany_skill_executor.core.can_rgbd import (
    CameraProjection,
    SegmentedCanCloud,
    render_grasp_overlay,
    segment_red_can,
)
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo


PREGRASP_AIM_OFFSET_M = 0.14


@dataclass(frozen=True, slots=True)
class AimedPregrasp:
    joint_state: JointState
    tcp_position: tuple[float, float, float]
    approach_direction: tuple[float, float, float]
    facing_error_deg: float


class CanGraspExecutionDemo(GraspExecutionDemo):
    """End-to-end simulation demo driven by rendered RGB-D points."""

    def __init__(self) -> None:
        super().__init__(node_name='can_grasp_execution_demo')
        self.declare_parameter('grasp_service', '/grasp/plan')
        self.declare_parameter(
            'color_topic',
            '/cleany/internal/mujoco/left_wrist_camera/image_raw',
        )
        self.declare_parameter(
            'depth_topic',
            '/cleany/internal/mujoco/left_wrist_camera/depth',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/cleany/internal/mujoco/left_wrist_camera/camera_info',
        )
        self.declare_parameter(
            'camera_translation_base',
            [0.140938350461, -0.002, 0.774117299010],
        )
        self.declare_parameter(
            'camera_rotation_base_from_optical',
            [
                0.0, -0.7173560909, 0.696706709347,
                -1.0, 0.0, 0.0,
                0.0, -0.696706709347, -0.7173560909,
            ],
        )
        self.declare_parameter('table_top_z_base', 0.345)
        self.declare_parameter('can_diameter_m', 0.070)
        self.declare_parameter('can_height_m', 0.100)
        self.declare_parameter('pregrasp_aim_ik_timeout_sec', 1.0)
        self.declare_parameter('pregrasp_facing_tolerance_deg', 2.0)
        self.declare_parameter('gripper_open_position_rad', 1.2)
        self.declare_parameter('gripper_motion_sec', 2.0)
        self._grasp = self.create_client(
            PlanGrasp, str(self.get_parameter('grasp_service').value)
        )
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        self._aim_ik = self.create_client(GetPositionIK, '/compute_ik')
        self._fk = self.create_client(GetPositionFK, '/compute_fk')
        self._grippers = {
            arm: ActionClient(
                self,
                FollowJointTrajectory,
                f'/{arm}_gripper_controller/follow_joint_trajectory',
            )
            for arm in ('left', 'right')
        }
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._grasp_image = self.create_publisher(
            Image, '/grasp/can_grasp_image', latched_qos
        )
        self._camera_messages: dict[tuple[int, int], dict[str, Any]] = {}
        self._rgbd_frame: tuple[Image, Image, CameraInfo] | None = None
        self.create_subscription(
            Image,
            str(self.get_parameter('color_topic').value),
            lambda message: self._on_camera('color', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            lambda message: self._on_camera('depth', message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            lambda message: self._on_camera('info', message),
            qos_profile_sensor_data,
        )

    def _on_camera(self, kind: str, message: Any) -> None:
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        sample = self._camera_messages.setdefault(key, {})
        sample[kind] = message
        if {'color', 'depth', 'info'} <= sample.keys():
            self._rgbd_frame = (
                sample['color'], sample['depth'], sample['info']
            )
            self._camera_messages.clear()
        while len(self._camera_messages) > 12:
            self._camera_messages.pop(next(iter(self._camera_messages)))

    def run(self) -> None:
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.get_logger().info(
            'Waiting for RGB-D, grasp generation, selection, and MoveGroup'
        )
        if not self._grasp.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('grasp planning service is unavailable')
        if not self._selection.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('reachable-grasp action is unavailable')
        if not self._move_group.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('MoveGroup action is unavailable')
        self._wait_for_joint_state(timeout)
        color, depth, info = self._wait_for_rgbd(timeout)
        rgb = self._rgb_array(color)
        projection = self._camera_projection(info)
        cloud = segment_red_can(
            rgb, self._depth_array(depth), projection
        )
        target_object = self._target_object(cloud)
        request = PlanGrasp.Request()
        request.snapshot_id = (
            f'mujoco-can-{color.header.stamp.sec}-'
            f'{color.header.stamp.nanosec}'
        )
        request.object_id = 2
        request.target_cloud = self._cloud_message(
            cloud.target_points,
            cloud.target_colors,
            color,
        )
        request.context_cloud = self._cloud_message(
            cloud.context_points,
            cloud.context_colors,
            color,
        )
        request.target_object = target_object
        self.get_logger().info(
            f'RGB-D can segmented: target_points={len(cloud.target_points)} '
            f'context_points={len(cloud.context_points)}; '
            f'OBB center=('
            f'{target_object.obb_pose.position.x:.4f}, '
            f'{target_object.obb_pose.position.y:.4f}, '
            f'{target_object.obb_pose.position.z:.4f}) base_link'
        )
        response = self._future(
            self._grasp.call_async(request), timeout, 'grasp candidates'
        )
        if not response.success or not response.candidates:
            raise RuntimeError(
                f'grasp generation failed: code={response.error_code} '
                f'{response.message}'
            )
        self.get_logger().info(
            f'GEOMETRIC GRASP COMPLETE: generated '
            f'{len(response.candidates)} candidates from simulated can RGB-D'
        )
        self._publish_grasp_image(
            rgb,
            projection,
            response.candidates,
            color,
            selected_index=None,
            selected_arm='',
        )
        self._publish_can_markers(
            response.candidates,
            target_object,
            selected_index=None,
            status=f'{len(response.candidates)} can grasps found',
        )
        self._hold('demo_start_delay_sec')

        goal = SelectReachableGrasp.Goal()
        goal.candidates = response.candidates
        handle = self._future(
            self._selection.send_goal_async(
                goal, feedback_callback=self._selection_feedback
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
                f'grasp selection failed: code={result.error_code} '
                f'{result.message}'
            )
        self.get_logger().info(
            f'Selected generated candidate={result.selected_candidate_index} '
            f'arm={result.selected_arm}; TCP=('
            f'{result.selected_candidate.tcp_pose.position.x:.4f}, '
            f'{result.selected_candidate.tcp_pose.position.y:.4f}, '
            f'{result.selected_candidate.tcp_pose.position.z:.4f}); '
            'executing pre-grasp'
        )
        self._publish_grasp_image(
            rgb,
            projection,
            response.candidates,
            color,
            selected_index=result.selected_candidate_index,
            selected_arm=result.selected_arm,
        )
        self._publish_can_markers(
            response.candidates,
            target_object,
            selected_index=result.selected_candidate_index,
            status=f'can grasp selected: {result.selected_arm}',
        )
        self._register_execution_collision(target_object)
        self._open_gripper(result.selected_arm)
        aimed_pregrasp = self._solve_aimed_pregrasp(
            result.selected_arm,
            result.selected_candidate,
            result.pregrasp_joint_state,
        )
        self._publish_grasp_image(
            rgb,
            projection,
            response.candidates,
            color,
            selected_index=result.selected_candidate_index,
            selected_arm=result.selected_arm,
            selected_approach=aimed_pregrasp.approach_direction,
        )
        self._publish_can_markers(
            response.candidates,
            target_object,
            selected_index=result.selected_candidate_index,
            status=(
                f'aimed pre-grasp: {result.selected_arm}, '
                f'error {aimed_pregrasp.facing_error_deg:.1f} deg'
            ),
            selected_pregrasp=aimed_pregrasp.tcp_position,
        )
        self._hold('stage_hold_sec')
        self._move_to(
            result.selected_arm,
            aimed_pregrasp.joint_state,
            'collision-checked aimed pre-grasp',
        )
        self._verify_feedback(aimed_pregrasp.joint_state)
        self._publish_can_markers(
            response.candidates,
            target_object,
            selected_index=result.selected_candidate_index,
            status=f'open gripper at pre-grasp: {result.selected_arm}',
            selected_pregrasp=aimed_pregrasp.tcp_position,
        )
        self.get_logger().info(
            'CAN PREGRASP DEMO COMPLETE: gripper opened, can collision '
            'retained, and the TCP approach axis faces the grasp point'
        )

    def _solve_aimed_pregrasp(
        self,
        arm: str,
        candidate: GraspCandidate,
        seed: JointState,
    ) -> AimedPregrasp:
        """Aim the physical TCP at the grasp using a virtual offset tip."""
        if not self._aim_ik.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt IK service is unavailable for pre-grasp aiming')
        positions = {
            name: float(self._joint_positions[name])
            for name in REQUIRED_JOINT_NAMES
        }
        positions.update(
            (name, float(value))
            for name, value in zip(seed.name, seed.position, strict=True)
        )
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f'{arm}_pregrasp_aim_arm'
        ik.ik_link_name = f'{arm}_pregrasp_aim_tip'
        ik.avoid_collisions = True
        ik.robot_state.joint_state.name = list(REQUIRED_JOINT_NAMES)
        ik.robot_state.joint_state.position = [
            positions[name] for name in REQUIRED_JOINT_NAMES
        ]
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.pose.position = candidate.tcp_pose.position
        ik.pose_stamped.pose.orientation.w = 1.0
        timeout = float(
            self.get_parameter('pregrasp_aim_ik_timeout_sec').value
        )
        seconds = int(timeout)
        ik.timeout.sec = seconds
        ik.timeout.nanosec = int((timeout - seconds) * 1e9)
        response = self._future(
            self._aim_ik.call_async(request),
            timeout + 2.0,
            'direction-aware pre-grasp IK',
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                'no collision-free pre-grasp can face the can: '
                f'IK code={response.error_code.val}'
            )
        solved = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
                strict=True,
            )
        )
        missing = set(seed.name) - solved.keys()
        if missing:
            raise RuntimeError(
                'direction-aware IK response omitted arm joints: '
                f'{sorted(missing)}'
            )
        joint_state = JointState()
        joint_state.name = list(seed.name)
        joint_state.position = [
            float(solved[name]) for name in joint_state.name
        ]
        tcp_position, approach, angle_deg = self._verify_pregrasp_facing(
            arm, candidate, response.solution, timeout=2.0
        )
        return AimedPregrasp(
            joint_state=joint_state,
            tcp_position=tcp_position,
            approach_direction=approach,
            facing_error_deg=angle_deg,
        )

    def _verify_pregrasp_facing(
        self,
        arm: str,
        candidate: GraspCandidate,
        state: RobotState,
        *,
        timeout: float,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        float,
    ]:
        if not self._fk.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('MoveIt FK service is unavailable for aim verification')
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.fk_link_names = [
            f'{arm}_grasp_tcp',
            f'{arm}_pregrasp_aim_tip',
        ]
        request.robot_state = state
        response = self._future(
            self._fk.call_async(request), timeout, 'pre-grasp FK verification'
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or len(response.pose_stamped) != 2
        ):
            raise RuntimeError('MoveIt could not verify the aimed pre-grasp')
        tcp = response.pose_stamped[0].pose
        aim_tip = response.pose_stamped[1].pose.position
        grasp = candidate.tcp_pose.position
        target_error = math.sqrt(
            (aim_tip.x - grasp.x) ** 2
            + (aim_tip.y - grasp.y) ** 2
            + (aim_tip.z - grasp.z) ** 2
        )
        q = tcp.orientation
        # Rotate the TCP's local -Y approach axis into base_link.
        forward = (
            2.0 * (q.w * q.z - q.x * q.y),
            2.0 * (q.x * q.x + q.z * q.z) - 1.0,
            -2.0 * (q.y * q.z + q.w * q.x),
        )
        to_grasp = (
            grasp.x - tcp.position.x,
            grasp.y - tcp.position.y,
            grasp.z - tcp.position.z,
        )
        distance = math.sqrt(sum(value * value for value in to_grasp))
        if not math.isfinite(distance) or distance <= 1.0e-9:
            raise RuntimeError('pre-grasp TCP-to-can distance is invalid')
        direction = tuple(value / distance for value in to_grasp)
        cosine = max(
            -1.0,
            min(1.0, sum(a * b for a, b in zip(forward, direction, strict=True))),
        )
        angle_deg = math.degrees(math.acos(cosine))
        tolerance = float(
            self.get_parameter('pregrasp_facing_tolerance_deg').value
        )
        expected_offset = PREGRASP_AIM_OFFSET_M
        if (
            target_error > 0.002
            or abs(distance - expected_offset) > 0.002
            or angle_deg > tolerance
        ):
            raise RuntimeError(
                'pre-grasp does not face the can: '
                f'angle={angle_deg:.2f}deg distance={distance:.3f}m '
                f'target_error={target_error:.4f}m'
            )
        self.get_logger().info(
            'Direction-aware pre-grasp verified: '
            f'TCP-to-can={distance:.3f}m facing_error={angle_deg:.2f}deg'
        )
        return (
            (tcp.position.x, tcp.position.y, tcp.position.z),
            direction,
            angle_deg,
        )

    def _wait_for_rgbd(
        self, timeout: float
    ) -> tuple[Image, Image, CameraInfo]:
        deadline = time.monotonic() + timeout
        while self._rgbd_frame is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._rgbd_frame is None:
            raise RuntimeError('synchronized MuJoCo RGB-D is unavailable')
        return self._rgbd_frame

    def _camera_projection(self, info: CameraInfo) -> CameraProjection:
        return CameraProjection(
            fx=float(info.k[0]),
            fy=float(info.k[4]),
            cx=float(info.k[2]),
            cy=float(info.k[5]),
            translation_base=tuple(
                float(value)
                for value in self.get_parameter(
                    'camera_translation_base'
                ).value
            ),
            rotation_base_from_optical=tuple(
                float(value)
                for value in self.get_parameter(
                    'camera_rotation_base_from_optical'
                ).value
            ),
        )

    def _register_execution_collision(
        self, target: DetectedObject3D
    ) -> None:
        if not self._apply_scene.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                'planning-scene service unavailable for execution'
            )
        collision = CollisionObject()
        collision.header.frame_id = 'base_link'
        collision.id = 'detected_can_execution_collision'
        collision.operation = CollisionObject.ADD
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [target.obb_size.z, target.obb_size.x / 2.0]
        collision.primitives = [cylinder]
        collision.primitive_poses = [target.obb_pose]
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._future(
            self._apply_scene.call_async(request),
            2.0,
            'persistent can collision registration',
        )
        if not response.success:
            raise RuntimeError('MoveIt rejected persistent can collision')
        self.get_logger().info(
            'Execution planning scene retains detected can as a cylinder; '
            'no contact permissions are enabled'
        )

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
        self.get_logger().info(
            f'Opening {arm} gripper to {point.positions[0]:.3f} rad'
        )
        handle = self._future(
            client.send_goal_async(goal), 5.0, f'{arm} gripper goal response'
        )
        if not handle.accepted:
            raise RuntimeError(f'{arm} gripper command was rejected')
        wrapped = self._future(
            handle.get_result_async(), 10.0, f'{arm} gripper result'
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
        self.get_logger().info(f'{arm} gripper opened')

    def _publish_grasp_image(
        self,
        rgb: np.ndarray,
        projection: CameraProjection,
        candidates: list[GraspCandidate],
        source: Image,
        *,
        selected_index: int | None,
        selected_arm: str,
        selected_approach: tuple[float, float, float] | None = None,
    ) -> None:
        positions = np.asarray(
            [
                (
                    item.tcp_pose.position.x,
                    item.tcp_pose.position.y,
                    item.tcp_pose.position.z,
                )
                for item in candidates
            ],
            dtype=float,
        )
        approaches = np.asarray(
            [
                (
                    item.approach_direction.x,
                    item.approach_direction.y,
                    item.approach_direction.z,
                )
                for item in candidates
            ],
            dtype=float,
        )
        if selected_index is not None and selected_approach is not None:
            approaches[selected_index] = selected_approach
        rendered = render_grasp_overlay(
            rgb,
            projection,
            positions,
            approaches,
            np.asarray([item.score for item in candidates]),
            np.asarray([item.required_opening_m for item in candidates]),
            selected_index=selected_index,
            selected_arm=selected_arm,
            pregrasp_offset_m=PREGRASP_AIM_OFFSET_M,
        )
        message = Image()
        message.header = source.header
        message.height, message.width = rendered.shape[:2]
        message.encoding = 'rgb8'
        message.step = message.width * 3
        message.data = np.ascontiguousarray(rendered).tobytes()
        self._grasp_image.publish(message)

    @staticmethod
    def _rgb_array(message: Image) -> np.ndarray:
        if message.encoding not in ('rgb8', 'bgr8'):
            raise ValueError(f'unsupported RGB encoding: {message.encoding}')
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        rgb = rows[:, : message.width * 3].reshape(
            message.height, message.width, 3
        ).copy()
        if message.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
        return rgb

    @staticmethod
    def _depth_array(message: Image) -> np.ndarray:
        if message.encoding != '32FC1':
            raise ValueError(f'unsupported depth encoding: {message.encoding}')
        return np.ndarray(
            shape=(message.height, message.width),
            dtype='<f4',
            buffer=bytes(message.data),
            strides=(message.step, 4),
        ).copy()

    def _target_object(self, cloud: SegmentedCanCloud) -> DetectedObject3D:
        result = DetectedObject3D()
        result.object_id = 2
        result.label = 'can'
        result.confidence = 1.0
        # The visible surface median is biased toward the camera.  A trimmed
        # extent recovers the cylinder centre while rejecting a few red edge
        # pixels that can contain mixed depth.
        low_xy, high_xy = np.percentile(
            cloud.target_points[:, :2], [1.0, 99.0], axis=0
        )
        center_xy = (low_xy + high_xy) / 2.0
        result.obb_pose.position.x = float(center_xy[0])
        result.obb_pose.position.y = float(center_xy[1])
        height = float(self.get_parameter('can_height_m').value)
        result.obb_pose.position.z = (
            float(self.get_parameter('table_top_z_base').value) + height / 2.0
        )
        result.obb_pose.orientation.w = 1.0
        diameter = float(self.get_parameter('can_diameter_m').value)
        result.obb_size.x = result.obb_size.y = diameter
        result.obb_size.z = height
        return result

    @staticmethod
    def _cloud_message(
        points: np.ndarray, colors: np.ndarray, source: Image
    ) -> PointCloud2:
        message = PointCloud2()
        message.header.stamp = source.header.stamp
        message.header.frame_id = 'base_link'
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='rgb', offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        message.point_step = 16
        message.row_step = 16 * message.width
        payload = bytearray(message.row_step)
        for index, (point, color) in enumerate(
            zip(points, colors, strict=True)
        ):
            packed = (
                int(color[0]) << 16 | int(color[1]) << 8 | int(color[2])
            )
            struct.pack_into('<fffI', payload, index * 16, *point, packed)
        message.data = bytes(payload)
        message.is_dense = True
        return message

    def _publish_can_markers(
        self,
        candidates: list[GraspCandidate],
        target: DetectedObject3D,
        *,
        selected_index: int | None,
        status: str,
        selected_pregrasp: tuple[float, float, float] | None = None,
    ) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        can = Marker()
        can.header.frame_id = 'base_link'
        can.header.stamp = stamp
        can.ns = 'detected_can'
        can.id = 300
        can.type = Marker.CYLINDER
        can.action = Marker.ADD
        can.pose = target.obb_pose
        can.scale.x = target.obb_size.x
        can.scale.y = target.obb_size.y
        can.scale.z = target.obb_size.z
        can.color.r, can.color.a = 1.0, 0.35
        markers.markers.append(can)

        for index, candidate in enumerate(candidates):
            point = Marker()
            point.header.frame_id = 'base_link'
            point.header.stamp = stamp
            point.ns = 'can_grasp_candidates'
            point.id = index
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose = candidate.tcp_pose
            point.scale.x = point.scale.y = point.scale.z = 0.025
            point.color.a = 0.9
            if selected_index == index:
                point.color.g = 1.0
            else:
                point.color.r, point.color.g = 1.0, 0.75
            markers.markers.append(point)

        chosen_index = selected_index if selected_index is not None else 0
        chosen = candidates[chosen_index]
        norm = math.sqrt(
            chosen.approach_direction.x ** 2
            + chosen.approach_direction.y ** 2
            + chosen.approach_direction.z ** 2
        )
        arrow = Marker()
        arrow.header.frame_id = 'base_link'
        arrow.header.stamp = stamp
        arrow.ns = 'can_grasp_approach'
        arrow.id = 400
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.022
        arrow.color.b, arrow.color.a = 1.0, 1.0
        end = chosen.tcp_pose.position
        start = Point()
        offset = PREGRASP_AIM_OFFSET_M
        if selected_pregrasp is None:
            start.x = end.x - chosen.approach_direction.x / norm * offset
            start.y = end.y - chosen.approach_direction.y / norm * offset
            start.z = end.z - chosen.approach_direction.z / norm * offset
        else:
            start.x, start.y, start.z = selected_pregrasp
        arrow.points = [start, end]
        markers.markers.append(arrow)

        label = Marker()
        label.header.frame_id = 'base_link'
        label.header.stamp = stamp
        label.ns = 'can_grasp_status'
        label.id = 500
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = target.obb_pose.position.x
        label.pose.position.y = target.obb_pose.position.y
        label.pose.position.z = target.obb_pose.position.z + 0.15
        label.scale.z = 0.035
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = status
        markers.markers.append(label)
        for marker in markers.markers:
            marker.lifetime = Duration(seconds=0).to_msg()
        self._markers.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanGraspExecutionDemo()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().fatal(f'CAN GRASP EXECUTION DEMO FAILED: {error}')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
