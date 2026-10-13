"""Project simulated depth images to hits and bounded costmap clearing rays."""

from __future__ import annotations

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from cleany_gazebo_sim.body_geometry import load_body_boxes
from cleany_gazebo_sim.obstacle_memory_node import transform
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from cleany_gazebo_sim.depth_clearance import project_depth_image


class DepthClearanceNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_clearance')
        for name, default in [('input_topic', '/camera/head/depth/image_raw'),
                              ('output_topic', '/camera/head/depth/points'),
                              ('camera_info_topic', '/camera/head/depth/camera_info'),
                              ('camera_frame', 'head_camera_depth_frame'),
                              ('optical_frame', 'head_camera_depth_optical_frame'),
                              ('self_filter_profile', ''), ('pixel_stride', 4), ('positive_infinity_is_free', False),
                              ('clearing_distance', 5.0), ('marking_max_range', 4.0), ('sensor_far_clip', 10.0)]:
            self.declare_parameter(name, default)
        self.stride = self.get_parameter('pixel_stride').value
        self.clearing_distance = self.get_parameter('clearing_distance').value
        if self.stride < 1 or not (self.get_parameter('marking_max_range').value < self.clearing_distance < self.get_parameter('sensor_far_clip').value):
            raise ValueError('Require stride >= 1 and marking range < clearing distance < far clip')
        profile = self.get_parameter('self_filter_profile').value
        self.self_boxes = []
        if profile:
            self.self_boxes = [b for b in load_body_boxes(
                Path(get_package_share_directory('cleany_description')), Path(profile), merge=False)
                if b.name.startswith(('left_', 'right_'))]
            self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.camera = None
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('input_topic').value, self.on_depth,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, qos_profile_sensor_data)

    def on_info(self, msg: CameraInfo) -> None:
        self.camera = msg

    def on_depth(self, msg: Image) -> None:
        if self.camera is None:
            return
        camera = self.camera
        if (msg.width, msg.height) != (camera.width, camera.height) or msg.header.frame_id != self.get_parameter('optical_frame').value:
            self.get_logger().error('Depth image dimensions/frame do not match the configured camera', throttle_duration_sec=2)
            return
        if any(abs(v) > 1e-9 for v in camera.d):
            self.get_logger().error('Only the undistorted simulation camera is supported', throttle_duration_sec=2)
            return
        if msg.encoding != '32FC1' or msg.step < msg.width*4 or len(msg.data) < msg.step*msg.height:
            self.get_logger().error('Expected a complete 32FC1 depth image', throttle_duration_sec=2)
            return
        endian = '>' if msg.is_bigendian else '<'
        depth = np.ndarray((msg.height, msg.width), dtype=endian+'f4', buffer=msg.data, strides=(msg.step, 4))
        try:
            points = project_depth_image(depth, (camera.k[0], camera.k[4], camera.k[2], camera.k[5]),
                                          pixel_stride=self.stride,
                                          positive_infinity_is_free=self.get_parameter('positive_infinity_is_free').value,
                                          clearing_distance=self.clearing_distance)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2)
            return
        if self.self_boxes:
            try:
                pose = self.buffer.lookup_transform('base_link', self.get_parameter('camera_frame').value,
                                                    Time.from_msg(msg.header.stamp)).transform
            except TransformException:
                return  # Never label a cloud with a guessed camera orientation.
            body_points = transform(points, pose)
            keep = np.ones(len(points), dtype=bool)
            for box in self.self_boxes:
                local = (body_points-box.center)@box.rotation
                keep &= ~np.all(np.abs(local) <= box.half_size+1e-5, axis=1)
            # Occluded self rays are discarded, never converted to free-space rays.
            points = np.ascontiguousarray(points[keep])
        output = PointCloud2()
        output.header = msg.header
        output.header.frame_id = self.get_parameter('camera_frame').value
        output.height, output.width = 1, len(points)
        output.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z'))]
        output.point_step = 12
        output.row_step = output.width*12
        output.is_dense = True
        output.data = points.tobytes()
        self.publisher.publish(output)


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
