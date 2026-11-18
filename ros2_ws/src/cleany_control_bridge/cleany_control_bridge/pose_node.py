"""Adapt read-only telemetry poses to the Backend pose WebSocket contract."""
from __future__ import annotations

import rclpy
from cleany_telemetry.pose import PoseFreshness, quaternion_yaw
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .pose_relay import PoseRelay


class PoseBackendBridge(Node):
    """Own Backend authentication, latest-only delivery and reconnects.

    Subscriptions:
        /telemetry/pose (geometry_msgs/msg/PoseStamped): pose_topic default,
        best effort, volatile. Original frame and source time are required.
    Notes:
        No ROS publishers, services, actions or TF. map_id is attached only
        to the configured map_frame; odom is never relabelled as map.
        Source freshness uses the ROS clock; delivery expiry uses monotonic time.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("pose_backend_bridge", **kwargs)
        for name, value in {
            "pose_topic": "/telemetry/pose",
            "url": "ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws",
            "device_token_file": "", "map_id": "", "map_frame": "map",
            "rate": 5.0, "input_timeout": 1.5, "future_tolerance": 0.1,
            "reconnect_initial": 1.0, "reconnect_max": 30.0,
        }.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.freshness = PoseFreshness(param("input_timeout"), param("future_tolerance"))
        self.map_id, self.map_frame = param("map_id"), param("map_frame")
        self.relay = PoseRelay(
            param("url"), param("rate"), param("input_timeout"),
            param("reconnect_initial"), param("reconnect_max"),
            device_token_file=param("device_token_file"),
        )
        self.create_subscription(PoseStamped, param("pose_topic"), self._pose, qos_profile_sensor_data)
        self.relay.start()

    def _pose(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        frame = message.header.frame_id
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.freshness.accept(frame, stamp, now):
            return
        self.relay.update_pose(
            p.x, p.y, stamp, yaw=quaternion_yaw(q.x, q.y, q.z, q.w),
            frame_id=frame, map_id=(self.map_id or None) if frame == self.map_frame else None,
            source_age=max(0.0, now - stamp),
        )

    def destroy_node(self) -> bool:
        self.relay.stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PoseBackendBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
