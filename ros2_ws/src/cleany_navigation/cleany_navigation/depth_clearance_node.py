"""왜곡 보정된 깊이 영상을 hit와 범위가 제한된 costmap 공간 제거 ray로 투영합니다."""

from __future__ import annotations
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from cleany_navigation.body_geometry import BodyBox, load_body_boxes
from cleany_navigation.depth_obstacles import ObstacleFilter, filter_obstacles
from cleany_navigation.transforms import transform
import numpy as np
import time
from rclpy.clock import Clock, ClockType
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from cleany_navigation.depth_clearance import project_depth_image, decode_depth_image, obstacle_height_mask, rectified_intrinsics, self_observation_mask
from cleany_navigation.point_cloud import xyz_cloud


class DepthClearanceNode(Node):
    """깊이 영상을 공간 제거용과 장애물 표시용 점군으로 분리합니다.

    Subscriptions:
        camera/head/depth/image_raw (sensor_msgs/msg/Image): 깊이 표본.
        camera/head/depth/camera_info (sensor_msgs/msg/CameraInfo): 내부 파라미터.

    Publishers:
        camera/head/depth/points (sensor_msgs/msg/PointCloud2): 설정된 빈 공간 제거
            끝점을 포함한 변환 ray.
        camera/head/depth/obstacle_points (sensor_msgs/msg/PointCloud2):
            설정된 베이스 좌표계 높이 범위 안의 측정 장애물 점.

    TF:
        촬영 시각의 optical/camera/base 변환을 읽습니다. 좌표계 기본값은
        head_camera_depth_optical_frame, head_camera_depth_frame, base_link입니다.

    QoS:
        입력 영상은 best_effort, depth 1이며 보정값과 점군은 sensor_data입니다.

    Notes:
        토픽과 좌표계 이름은 변경할 수 있습니다. TF 누락 재시도는 tf_wait_timeout_s
        안으로 제한하며 자체 필터에는 명시적 body 프로파일이 필요합니다. 선택적
        spatial_filter_file을 쓰면 표시 hit는 base_frame으로 내보내고 제거 ray는
        camera_frame을 유지합니다. 실물 프로파일은 영상 경과 시간과 최소 유효 픽셀
        비율을 검사하며 무효 프레임은 하위 보호 상태의 최신성을 갱신하지 않습니다.
    """
    def __init__(self) -> None:
        super().__init__('depth_clearance')
        for name, default in [('input_topic', 'camera/head/depth/image_raw'),
                              ('output_topic', 'camera/head/depth/points'),
                              ('obstacle_topic', 'camera/head/depth/obstacle_points'),
                              ('tf_wait_timeout_s', .15), ('marking_min_z', -.33), ('marking_max_z', 1.62),
                              ('camera_info_topic', 'camera/head/depth/camera_info'),
                              ('camera_frame', 'head_camera_depth_frame'),
                              ('optical_frame', 'head_camera_depth_optical_frame'),
                              ('self_filter_profile', ''), ('self_filter_boxes_file', ''), ('spatial_filter_file', ''),
                              ('max_image_age_s', 0.), ('min_valid_fraction', 0.),
                              ('self_filter_margin_m', .001), ('allow_simulation_limits', False), ('base_frame', 'base_link'), ('pixel_stride', 4), ('positive_infinity_is_free', False),
                              ('clearing_distance', 5.0), ('marking_max_range', 4.0), ('sensor_far_clip', 10.0)]:
            self.declare_parameter(name, default)
        obstacle_height_mask(np.empty((0, 3)), self.get_parameter('marking_min_z').value, self.get_parameter('marking_max_z').value)
        self.stride = self.get_parameter('pixel_stride').value
        self.clearing_distance = self.get_parameter('clearing_distance').value
        if self.stride < 1 or not (self.get_parameter('marking_max_range').value < self.clearing_distance < self.get_parameter('sensor_far_clip').value):
            raise ValueError('Require stride >= 1 and marking range < clearing distance < far clip')
        profile = self.get_parameter('self_filter_profile').value
        self.self_filter_margin = self.get_parameter('self_filter_margin_m').value
        if not np.isfinite(self.self_filter_margin) or self.self_filter_margin < 0:
            raise ValueError('self_filter_margin_m must be finite and nonnegative')
        self.self_boxes = []
        self.spatial_filter = None
        filter_file = self.get_parameter('spatial_filter_file').value
        if filter_file:
            self.spatial_filter = ObstacleFilter(**json.loads(Path(filter_file).read_text()))
        boxes_file = self.get_parameter('self_filter_boxes_file').value
        if boxes_file and profile:
            raise ValueError('Select one source of self geometry')
        if boxes_file:
            body = json.loads(Path(boxes_file).read_text())
            if body['frame'] != self.get_parameter('base_frame').value:
                raise ValueError('Body geometry frame mismatch')
            self.self_boxes = [BodyBox(b['name'], np.asarray(b['center']), np.eye(3),
                                      np.asarray(b['half_size'])) for b in body['boxes']]
        self.max_image_age = self.get_parameter('max_image_age_s').value
        self.min_valid_fraction = self.get_parameter('min_valid_fraction').value
        if (not np.isfinite([self.max_image_age, self.min_valid_fraction]).all()
                or self.max_image_age < 0 or not 0 <= self.min_valid_fraction <= 1):
            raise ValueError('Invalid depth freshness or validity limit')
        if profile:
            self.self_boxes = [b for b in load_body_boxes(
                Path(get_package_share_directory('cleany_description')), Path(profile), merge=False, allow_simulation_limits=self.get_parameter('allow_simulation_limits').value)
                # 주차된 팔과 고정 프레임 및 기둥을 포함하며 회전하는 헤드 부품은 제외합니다.
                if b.name.startswith(('left_', 'right_', 'base_link/', 'top_base_link/'))]
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.camera = None
        self.pending = None
        self.pending_since = 0.
        self.tf_wait_timeout = self.get_parameter('tf_wait_timeout_s').value
        if not np.isfinite(self.tf_wait_timeout) or self.tf_wait_timeout <= 0:
            raise ValueError('tf_wait_timeout_s must be positive')
        self.create_timer(.01, self.retry_depth, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('input_topic').value, self.on_depth,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.obstacles = self.create_publisher(PointCloud2, self.get_parameter('obstacle_topic').value, qos_profile_sensor_data)
        self.publisher = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, qos_profile_sensor_data)

    def on_info(self, msg: CameraInfo) -> None:
        self.camera = msg

    def on_depth(self, msg: Image) -> None:
        self.pending, self.pending_since = msg, time.monotonic()
        self.retry_depth()

    def retry_depth(self) -> None:
        msg = self.pending
        if msg is None:
            return
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
        if self.max_image_age and not -.05 <= age <= self.max_image_age:
            self.pending = None
            return
        if time.monotonic()-self.pending_since > self.tf_wait_timeout:
            self.pending = None
            self.get_logger().warning('Depth TF wait expired; frame discarded', throttle_duration_sec=2)
            return
        when = Time.from_msg(msg.header.stamp)
        camera = self.get_parameter('camera_frame').value
        base = self.get_parameter('base_frame').value
        if self.camera is None or not self.buffer.can_transform(camera, msg.header.frame_id, when) or not self.buffer.can_transform(base, camera, when):
            return
        # 비블로킹 재시도로 executor가 TF를 수신하게 합니다. 원래 영상 stamp를
        # 유지하며 최신 또는 추측한 변환으로 대체하지 않습니다.
        self.pending = None
        self.process_depth(msg)

    def process_depth(self, msg: Image) -> None:
        camera = self.camera
        if (msg.width, msg.height) != (camera.width, camera.height) or msg.header.frame_id != self.get_parameter('optical_frame').value or camera.header.frame_id != msg.header.frame_id:
            self.get_logger().error('Depth image dimensions/frame do not match the configured camera', throttle_duration_sec=2)
            return
        if any(not np.isfinite(v) or abs(v) > 1e-9 for v in camera.d):
            self.get_logger().error('Rectified depth and matching CameraInfo are required', throttle_duration_sec=2)
            return
        try:
            depth = decode_depth_image(msg.data, msg.width, msg.height, msg.step, msg.encoding, msg.is_bigendian)
            valid_fraction = np.mean(np.isfinite(depth) & (depth > 0))
            if valid_fraction < self.min_valid_fraction:
                # 무효한 빈 영상으로 충돌 보호의 최신성을 갱신하지 않습니다.
                self.get_logger().warning('Insufficient valid depth; frame discarded', throttle_duration_sec=2.)
                return
            intrinsics = rectified_intrinsics(camera.k, camera.p)
            if intrinsics is None:
                raise ValueError('Expected a finite monocular rectified projection matrix')
            points = project_depth_image(depth, intrinsics,
                                          pixel_stride=self.stride,
                                          positive_infinity_is_free=self.get_parameter('positive_infinity_is_free').value,
                                          clearing_distance=self.clearing_distance)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2)
            return
        optical_points = np.column_stack((-points[:, 1], -points[:, 2], points[:, 0]))
        output_frame = self.get_parameter('camera_frame').value
        try:
            if output_frame == msg.header.frame_id:
                points = optical_points
            else:
                pose = self.buffer.lookup_transform(output_frame, msg.header.frame_id,
                                                    Time.from_msg(msg.header.stamp)).transform
                points = transform(optical_points, pose)
            points = np.ascontiguousarray(points, dtype='<f4')
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'Depth frame discarded: {error}', throttle_duration_sec=2)
            return
        try:
            pose = self.buffer.lookup_transform(self.get_parameter('base_frame').value, self.get_parameter('camera_frame').value,
                                                Time.from_msg(msg.header.stamp)).transform
        except TransformException as error:
            # 추측한 카메라 방향을 점군 좌표계로 표시하지 않습니다.
            self.get_logger().warning(f'Depth frame discarded: {error}', throttle_duration_sec=2)
            return
        body_points = transform(points, pose)
        sensor_origin = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
        keep = self_observation_mask(body_points, sensor_origin, self.self_boxes, self.self_filter_margin)
        # 자체 형상에 가린 ray는 버리며 빈 공간 ray로 바꾸지 않습니다.
        points = points[keep]
        body_points = body_points[keep]
        self.publisher.publish(xyz_cloud(points, output_frame, msg.header.stamp))
        # 표시용 점군에서 바닥과 천장을 제외하되 실제 관측 ray를 따라 점유 공간을
        # 제거할 수 있도록 전체 점군은 유지합니다.
        if self.spatial_filter is not None:
            hits, _ = filter_obstacles(body_points, sensor_origin, [], self.spatial_filter)
            hit_frame = self.get_parameter('base_frame').value
        else:
            hits = points[obstacle_height_mask(body_points,
                self.get_parameter('marking_min_z').value, self.get_parameter('marking_max_z').value)]
            hit_frame = output_frame
        self.obstacles.publish(xyz_cloud(hits, hit_frame, msg.header.stamp))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthClearanceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
