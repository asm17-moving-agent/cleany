"""ROS entry point for Jetson camera redaction."""
import fcntl
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .privacy import CameraRedactor
from .rgbd_snapshot import rgb_array


class CameraPrivacyNode(Node):
    """Publish only successfully redacted recent RGB frames with their original header.

    Subscriptions:
        /camera/color/image_raw (sensor_msgs/msg/Image): Configurable head RGB input.
    Publishers:
        /camera/head/color/image_redacted (sensor_msgs/msg/Image): Opaque BGR preview.
    QoS:
        Volatile, best effort, keep last one. Slow inference drops queued source frames.
    """

    def __init__(self):
        super().__init__('camera_privacy')
        root = Path('/var/opt/cleany/models/camera-privacy/active')
        profile = Path(get_package_share_directory('cleany_perception')) / 'config/yoloe_privacy.yaml'
        p = {k: self.declare_parameter(k, v).value for k, v in {
            'input_topic': '/camera/color/image_raw', 'output_topic': '/camera/head/color/image_redacted',
            'weights': str(root / 'privacy.engine'), 'face_model': str(root / 'face.onnx'),
            'text_model': str(root / 'text.plan'), 'device': '0', 'max_frame_age_s': 1.0,
            'max_fps': 5.0, 'gpu_lock_path': '',
        }.items()}
        source = self.resolve_topic_name(p['input_topic'])
        target = self.resolve_topic_name(p['output_topic'])
        if source == target or not 0 < p['max_frame_age_s'] or not 0 < p['max_fps']:
            raise ValueError('Privacy node needs distinct topics and positive timing limits')
        cv2.setNumThreads(2)
        self.redactor = CameraRedactor(Path(p['weights']), Path(p['face_model']),
                                      Path(p['text_model']), profile, device=p['device'])
        self.gpu_lock = open(p['gpu_lock_path'], 'a') if p['gpu_lock_path'] else None
        self.max_age = p['max_frame_age_s']
        self.period = 1.0 / p['max_fps']
        self.last_start = float('-inf')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.output = self.create_publisher(Image, target, qos)
        self.subscription = self.create_subscription(Image, source, self.receive, qos)

    def receive(self, message: Image) -> None:
        start = time.monotonic()
        if not self.output.get_subscription_count() or start - self.last_start < self.period:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not self.fresh(stamp):
            return
        self.last_start = start
        try:
            bgr = np.ascontiguousarray(rgb_array(message)[:, :, ::-1])
            if self.gpu_lock:
                try:
                    fcntl.flock(self.gpu_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    return
            try:
                frame = self.redactor.redact(bgr, stamp)
            finally:
                if self.gpu_lock:
                    fcntl.flock(self.gpu_lock, fcntl.LOCK_UN)
            if frame is None or not self.fresh(stamp):
                return
            result = Image(header=message.header, height=message.height, width=message.width,
                           encoding='bgr8', step=message.width * 3, data=frame.image.tobytes())
            self.output.publish(result)
        except Exception as exc:
            self.redactor.history.clear()
            self.get_logger().error(f'Privacy frame withheld ({type(exc).__name__})', throttle_duration_sec=5.)

    def fresh(self, stamp: float) -> bool:
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return stamp > 0 and 0 <= age <= self.max_age

    def destroy_node(self):
        if self.gpu_lock:
            self.gpu_lock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPrivacyNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
