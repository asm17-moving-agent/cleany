"""Publish sensor-only scene clouds for RViz and MoveIt."""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from cleany_perception.depth_scene_cloud import depth_scene_cloud


class DepthSceneNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_scene_node')
        defaults = {
            'depth_image_topic': '/camera/depth/image_rect_raw',
            'depth_info_topic': '/camera/depth/camera_info',
            'cloud_topic': '/perception/scene_cloud',
            'minimum_depth_m': 0.1,
            'maximum_depth_m': 2.0,
            'pixel_stride': 2,
            'publish_rate_hz': 2.0,
            'maximum_depth_age_sec': 1.0,
            'depth_16u_scale_m': 0.001,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be positive and finite')
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._published_stamp: tuple[int, int] | None = None
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('cloud_topic').value), 1
        )
        self.create_subscription(
            Image, str(self.get_parameter('depth_image_topic').value),
            self._on_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('depth_info_topic').value),
            self._on_info, qos_profile_sensor_data,
        )
        self.create_timer(1.0 / rate, self._publish)

    def _on_depth(self, message: Image) -> None:
        self._depth = message

    def _on_info(self, message: CameraInfo) -> None:
        self._info = message

    def _publish(self) -> None:
        if self._depth is None or self._info is None:
            return
        stamp = self._depth.header.stamp
        key = (stamp.sec, stamp.nanosec)
        age = (self.get_clock().now().nanoseconds
               - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
        maximum_age = float(self.get_parameter('maximum_depth_age_sec').value)
        if key == self._published_stamp or not 0 <= age <= maximum_age:
            return
        try:
            cloud = depth_scene_cloud(
                self._depth, self._info,
                float(self.get_parameter('minimum_depth_m').value),
                float(self.get_parameter('maximum_depth_m').value),
                int(self.get_parameter('pixel_stride').value),
                float(self.get_parameter('depth_16u_scale_m').value),
            )
        except ValueError as error:
            self.get_logger().warning(
                f'Scene cloud rejected: {error}', throttle_duration_sec=5
            )
            return
        if not cloud.width:
            return
        self._publisher.publish(cloud)
        self._published_stamp = key


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
