"""Project rectified depth images to hits and bounded costmap clearing rays."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from cleany_navigation.body_geometry import load_body_boxes
from cleany_navigation.transforms import transform
import numpy as np
import time
from rclpy.clock import Clock, ClockType
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from cleany_navigation.depth_clearance import project_depth_image, decode_depth_image, obstacle_height_mask, self_observation_mask


class DepthClearanceNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_clearance')
        for name, default in [('input_topic', 'camera/head/depth/image_raw'),
                              ('output_topic', 'camera/head/depth/points'),
                              ('obstacle_topic', 'camera/head/depth/obstacle_points'),
                              ('tf_wait_timeout_s', .15), ('marking_min_z', -.33), ('marking_max_z', 1.62),
                              ('camera_info_topic', 'camera/head/depth/camera_info'),
                              ('camera_frame', 'head_camera_depth_frame'),
                              ('optical_frame', 'head_camera_depth_optical_frame'),
                              ('self_filter_profile', ''), ('self_filter_margin_m', .001), ('allow_simulation_limits', False), ('base_frame', 'base_link'), ('pixel_stride', 4), ('positive_infinity_is_free', False),
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
        if profile:
            self.self_boxes = [b for b in load_body_boxes(
                Path(get_package_share_directory('cleany_description')), Path(profile), merge=False, allow_simulation_limits=self.get_parameter('allow_simulation_limits').value)
                # Parked arms and fixed frame/pole; rotating head parts are excluded.
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
        if time.monotonic()-self.pending_since > self.tf_wait_timeout:
            self.pending = None
            self.get_logger().warning('Depth TF wait expired; frame discarded', throttle_duration_sec=2)
            return
        when = Time.from_msg(msg.header.stamp)
        camera = self.get_parameter('camera_frame').value
        base = self.get_parameter('base_frame').value
        if self.camera is None or not self.buffer.can_transform(camera, msg.header.frame_id, when) or not self.buffer.can_transform(base, camera, when):
            return
        # Nonblocking retry lets this executor receive TF. Keep the original
        # image stamp and never substitute a latest or guessed transform.
        self.pending = None
        self.process_depth(msg)

    def process_depth(self, msg: Image) -> None:
        if self.camera is None:
            return
        camera = self.camera
        if (msg.width, msg.height) != (camera.width, camera.height) or msg.header.frame_id != self.get_parameter('optical_frame').value or camera.header.frame_id != msg.header.frame_id:
            self.get_logger().error('Depth image dimensions/frame do not match the configured camera', throttle_duration_sec=2)
            return
        if any(not np.isfinite(v) or abs(v) > 1e-9 for v in camera.d):
            self.get_logger().error('Rectified depth and matching CameraInfo are required', throttle_duration_sec=2)
            return
        try:
            depth = decode_depth_image(msg.data, msg.width, msg.height, msg.step, msg.encoding, msg.is_bigendian)
            intrinsics = (camera.k[0], camera.k[4], camera.k[2], camera.k[5])
            if any(camera.p):
                if not np.isfinite(camera.p).all() or abs(camera.p[3])+abs(camera.p[7]) > 1e-9:
                    raise ValueError('Expected a finite monocular rectified projection matrix')
                intrinsics = (camera.p[0], camera.p[5], camera.p[2], camera.p[6])
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
        except (TransformException, ValueError):
            return
        try:
            pose = self.buffer.lookup_transform(self.get_parameter('base_frame').value, self.get_parameter('camera_frame').value,
                                                Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return  # Never label a cloud with a guessed camera orientation.
        body_points = transform(points, pose)
        sensor_origin = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
        keep = self_observation_mask(body_points, sensor_origin, self.self_boxes, self.self_filter_margin)
        # Occluded self rays are discarded, never converted to free-space rays.
        points = np.ascontiguousarray(points[keep])
        body_points = body_points[keep]
        output = PointCloud2()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.get_parameter('camera_frame').value
        output.height, output.width = 1, len(points)
        output.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z'))]
        output.point_step = 12
        output.row_step = output.width*12
        output.is_dense = True
        output.data = points.tobytes()
        self.publisher.publish(output)
        # Marking cloud excludes floor/ceiling; the full cloud remains available
        # to clear occupied space along the actually observed rays.
        marked = deepcopy(output)
        hits = np.ascontiguousarray(points[obstacle_height_mask(body_points,
            self.get_parameter('marking_min_z').value, self.get_parameter('marking_max_z').value)])
        marked.width = len(hits); marked.row_step = marked.width * marked.point_step
        marked.data = hits.tobytes()
        self.obstacles.publish(marked)


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
