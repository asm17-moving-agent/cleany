"""의존성 없는 자세 relay의 ROS 2 래퍼입니다."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .relay import PoseRelay, quaternion_yaw


class TelemetryNode(Node):
    """유한한 odometry 자세를 telemetry WebSocket worker로 전달합니다.

    Subscriptions:
        /odom (nav_msgs/msg/Odometry): 자세와 촬영 시각이며 odom_topic 기본값입니다.

    QoS:
        입력은 best_effort, keep_last, depth 1입니다.

    Notes:
        콜백은 공유 최신 자세만 갱신합니다. 소켓 I/O, 속도 제한과 오래된 입력
        처리는 PoseRelay가 담당하며 좌표는 입력 좌표계를 유지합니다.
    """
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
        q = message.pose.pose.orientation
        self._relay.update_pose(x, y, sim_stamp, yaw=quaternion_yaw(q.x, q.y, q.z, q.w))

    def destroy_node(self) -> bool:
        self._relay.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
