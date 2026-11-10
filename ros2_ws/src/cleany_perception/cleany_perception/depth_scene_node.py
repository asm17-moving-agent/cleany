"""RViz와 MoveIt에 센서 전용 장면 점군을 발행합니다."""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from cleany_perception.depth_scene_cloud import depth_scene_cloud


class DepthSceneNode(Node):
    """시각화와 mapping을 위해 센서 깊이를 장면 점군으로 발행합니다.

    Subscriptions:
        /camera/depth/image_rect_raw (sensor_msgs/msg/Image): 촬영 깊이.
        /camera/depth/camera_info (sensor_msgs/msg/CameraInfo): 투영 내부 파라미터.

    Publishers:
        /perception/scene_cloud (sensor_msgs/msg/PointCloud2): 촬영 좌표계의 비어 있지
            않은 측정 점군이며 원래 stamp를 유지합니다.

    QoS:
        입력은 best_effort, depth 1이고 출력은 reliable, depth 1입니다.

    Notes:
        토픽 이름은 변경 가능한 기본값입니다. 반복, 오래됨, 무효 또는 빈 표본은
        건너뛰며 출력 구독자가 없으면 점군을 계산하지 않습니다.
    """
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
        latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, str(self.get_parameter('depth_image_topic').value),
            self._on_depth, latest_sensor,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('depth_info_topic').value),
            self._on_info, latest_sensor,
        )
        self.create_timer(1.0 / rate, self._publish)

    def _on_depth(self, message: Image) -> None:
        self._depth = message

    def _on_info(self, message: CameraInfo) -> None:
        self._info = message

    def _publish(self) -> None:
        if self._publisher.get_subscription_count() == 0:
            return
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
