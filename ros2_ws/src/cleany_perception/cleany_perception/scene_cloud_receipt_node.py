"""Separate filtered-cloud reception from the coordinator's action callbacks."""
from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def receipt_for_cloud(cloud: PointCloud2) -> Header | None:
    """Forward only an actual nonempty complete cloud's unchanged capture time."""
    stamp = cloud.header.stamp.sec*1_000_000_000 + cloud.header.stamp.nanosec
    if (stamp <= 0 or not cloud.header.frame_id or cloud.width <= 0 or cloud.height <= 0
            or cloud.point_step <= 0 or cloud.row_step < cloud.width*cloud.point_step
            or len(cloud.data) < cloud.row_step*cloud.height):
        return None
    return deepcopy(cloud.header)


class SceneCloudReceiptNode(Node):
    def __init__(self) -> None:
        super().__init__('scene_cloud_receipt')
        self.declare_parameter('filtered_cloud_topic', '/perception/scene_cloud_filtered')
        self.declare_parameter('receipt_topic', '/perception/scene_cloud_receipt')
        self._publisher = self.create_publisher(
            Header, str(self.get_parameter('receipt_topic').value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            PointCloud2, str(self.get_parameter('filtered_cloud_topic').value),
            self._on_cloud, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _on_cloud(self, cloud: PointCloud2) -> None:
        receipt = receipt_for_cloud(cloud)
        if receipt is not None:
            self._publisher.publish(receipt)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneCloudReceiptNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
