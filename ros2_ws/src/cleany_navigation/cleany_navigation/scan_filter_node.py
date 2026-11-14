"""원시 센서 토픽을 유지하며 설정된 베이스 전방 스캔을 발행합니다."""

from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .scan_filter import mask_front_scan, mask_self_returns
from pathlib import Path
import yaml


class ScanFilter(Node):
    """보관된 원시 스캔을 바꾸지 않고 근거리 및 후방 반사값을 가립니다.

    Subscriptions:
        scan_raw (LaserScan): sensor_data QoS, 원래 센서 좌표계와 stamp.
    Publishers:
        scan (LaserScan): sensor_data QoS이며 가린 ray는 NaN입니다.
    """

    def __init__(self):
        super().__init__("mapping_scan_filter")
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("front_center_rad", 0.0)
        self.declare_parameter("front_half_angle_rad", math.pi / 2)
        self.declare_parameter('self_mask_file', '')
        self.declare_parameter('mount_x', 0.0)
        self.declare_parameter('mount_y', 0.0)
        self.boxes = []
        mask_file = self.get_parameter('self_mask_file').value
        if mask_file:
            self.boxes = yaml.safe_load(Path(mask_file).read_text())['boxes']
            if any(len(b)!=4 or not all(math.isfinite(v) for v in b) or b[0]>=b[2] or b[1]>=b[3] for b in self.boxes):
                raise ValueError('Invalid self-mask rectangles')
        self.translation = (float(self.get_parameter('mount_x').value), float(self.get_parameter('mount_y').value))
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
        if self.boxes:
            output.ranges = mask_self_returns(output.ranges, message.angle_min, message.angle_increment,
                translation=self.translation, yaw=-self.center, boxes=self.boxes)
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
