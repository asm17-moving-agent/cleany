"""Publish the configured base-front scan while retaining the raw sensor topic."""

from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .scan_filter import mask_front_scan


class ScanFilter(Node):
    """Mask near/rear returns without altering the retained raw scan.

    Subscriptions:
        scan_raw (LaserScan): Sensor-data QoS, original sensor frame and stamp.
    Publishers:
        scan (LaserScan): Sensor-data QoS; masked rays are NaN.
    """

    def __init__(self):
        super().__init__("mapping_scan_filter")
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("front_center_rad", 0.0)
        self.declare_parameter("front_half_angle_rad", math.pi / 2)
        self.minimum = float(self.get_parameter("min_range_m").value)
        self.center = float(self.get_parameter("front_center_rad").value)
        self.half_angle = float(self.get_parameter("front_half_angle_rad").value)
        if not (
            math.isfinite(self.minimum)
            and self.minimum >= 0
            and math.isfinite(self.center)
            and 0 < self.half_angle <= math.pi
        ):
            raise ValueError("Invalid scan filter geometry")
        self.publisher = self.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "scan_raw", self.receive, qos_profile_sensor_data
        )

    def receive(self, message):
        output = deepcopy(message)
        output.ranges = mask_front_scan(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            center=self.center,
            half_angle=self.half_angle,
            minimum=max(float(message.range_min), self.minimum),
        )
        output.range_min = max(float(message.range_min), self.minimum)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
