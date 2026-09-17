"""ROS 2 wrapper for the dependency-free pose relay."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .relay import PoseRelay


class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("cleany_telemetry")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("url", "ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws")
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("input_timeout", 1.5)
        self.declare_parameter("reconnect_initial", 1.0)
        self.declare_parameter("reconnect_max", 30.0)
        self._relay = PoseRelay(
            self.get_parameter("url").value,
            float(self.get_parameter("rate").value),
            float(self.get_parameter("input_timeout").value),
            float(self.get_parameter("reconnect_initial").value),
            float(self.get_parameter("reconnect_max").value),
        )
        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._odom, odom_qos
        )
        self._relay.start()

    def _odom(self, message: Odometry) -> None:
        x, y = message.pose.pose.position.x, message.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        stamp = message.header.stamp
        sim_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._relay.update_pose(x, y, sim_stamp)

    def destroy_node(self) -> bool:
        self._relay.stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
