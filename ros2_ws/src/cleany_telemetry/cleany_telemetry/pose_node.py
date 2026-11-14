"""Read-only ROS pose provider, independent of external consumers."""
from __future__ import annotations

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .pose import PoseFreshness


class PoseObserver(Node):
    """Publish valid poses with the original frame and ROS timestamp.

    Subscriptions:
        /odom (nav_msgs/msg/Odometry): source=odom, odom_topic default.
    Publishers:
        /telemetry/pose (geometry_msgs/msg/PoseStamped): pose_topic default.
        Best effort, volatile; no replay or synthetic refresh of old poses.
    TF:
        source=tf reads target_frame -> base_frame (map -> base_link defaults).
        Does not broadcast transforms or request localization updates.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("pose_observer", **kwargs)
        for name, value in {
            "source": "odom", "odom_topic": "/odom", "pose_topic": "/telemetry/pose",
            "target_frame": "map", "base_frame": "base_link", "rate": 5.0,
            "input_timeout": 1.5, "future_tolerance": 0.1,
        }.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.freshness = PoseFreshness(param("input_timeout"), param("future_tolerance"))
        self.publisher = self.create_publisher(PoseStamped, param("pose_topic"), qos_profile_sensor_data)
        self.source = param("source")
        if self.source == "odom":
            self.create_subscription(Odometry, param("odom_topic"), self._odom, qos_profile_sensor_data)
        elif self.source == "tf":
            rate = float(param("rate"))
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("rate must be positive and finite")
            self.target_frame, self.base_frame = param("target_frame"), param("base_frame")
            if not self.target_frame or not self.base_frame or self.target_frame == self.base_frame:
                raise ValueError("distinct target_frame and base_frame are required")
            self.buffer = Buffer(node=self)
            self.listener = TransformListener(self.buffer, self)
            self.create_timer(1.0 / rate, self._tf)
        else:
            raise ValueError("source must be odom or tf")

    def _odom(self, message: Odometry) -> None:
        self._publish(PoseStamped(header=copy.deepcopy(message.header), pose=copy.deepcopy(message.pose.pose)))

    def _tf(self) -> None:
        try:
            transform = self.buffer.lookup_transform(self.target_frame, self.base_frame, Time())
        except TransformException:
            return
        message = PoseStamped(header=transform.header)
        p = transform.transform.translation
        message.pose.position.x, message.pose.position.y, message.pose.position.z = p.x, p.y, p.z
        message.pose.orientation = transform.transform.rotation
        self._publish(message)

    def _publish(self, message: PoseStamped) -> None:
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            return
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if not math.isfinite(norm) or norm < 1e-12:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.freshness.accept(message.header.frame_id, stamp, now):
            return
        q.x, q.y, q.z, q.w = (v / norm for v in (q.x, q.y, q.z, q.w))
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PoseObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
