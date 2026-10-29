"""Publish rectified USB wrist RGB using the existing intrinsic calibration."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def load_camera_profile(config_file: Path, repository_root: Path) -> tuple[dict, dict]:
    def resolve(raw: str) -> Path:
        path = Path(raw).expanduser()
        return path if path.is_absolute() else repository_root / path
    config = json.loads(config_file.read_text())
    if config['arm'] not in ('left', 'right') or config['frame_id'] != config['arm'] + '_wrist_rgb_optical_frame':
        raise ValueError('Wrist camera arm/frame mismatch')
    camera = json.loads(resolve(config['camera_config']).read_text())
    calibration = json.loads(resolve(config['calibration_config']).read_text())
    path = resolve(calibration['intrinsics_file'])
    intrinsic = json.loads(path.read_text())
    source = json.loads((path.parent / 'manifest.json').read_text())['config']
    for key in ('camera_device', 'width', 'height'):
        if source[key] != camera[key]:
            raise ValueError('Wrist calibration mismatch: ' + key)
    if (intrinsic['width'], intrinsic['height']) != (camera['width'], camera['height']):
        raise ValueError('Wrist calibration image dimensions mismatch')
    rate = config['publish_rate_hz']
    if not math.isfinite(rate) or not 0 < rate <= 30:
        raise ValueError('Wrist publish rate must be in (0, 30] Hz')
    return {**config, **camera}, intrinsic


class WristRgbCamera(Node):
    def __init__(self, **kwargs):
        super().__init__('wrist_rgb_camera', **kwargs)
        self.declare_parameter('repository_root', '')
        self.declare_parameter('config_file', '')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('preview_rate_hz', 5.0)
        self.declare_parameter('preview_jpeg_quality', 75)
        self.config, intrinsic = load_camera_profile(
            Path(self.get_parameter('config_file').value),
            Path(self.get_parameter('repository_root').value))
        width, height = self.config['width'], self.config['height']
        self.preview_rate = float(self.get_parameter('preview_rate_hz').value)
        self.preview_quality = self.get_parameter('preview_jpeg_quality').value
        if not math.isfinite(self.preview_rate) or not 0 < self.preview_rate <= 30:
            raise ValueError('Preview rate must be in (0, 30] Hz')
        self.preview_rate = min(self.preview_rate, self.config['publish_rate_hz'])
        if type(self.preview_quality) is not int or not 1 <= self.preview_quality <= 100:
            raise ValueError('Preview JPEG quality must be an integer in [1, 100]')
        self.next_preview_at = 0.0
        matrix = np.asarray(intrinsic['K'], dtype=float).reshape(3, 3)
        distortion = np.asarray(intrinsic['D'], dtype=float).ravel()
        if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0):
            raise ValueError('Invalid wrist camera intrinsics')
        self.maps = cv2.initUndistortRectifyMap(
            matrix, distortion, None, matrix, (width, height), cv2.CV_32FC1)
        self.info = CameraInfo(width=width, height=height, distortion_model='plumb_bob',
            d=[0.] * 5, k=matrix.ravel().tolist(), r=np.eye(3).ravel().tolist(),
            p=np.column_stack((matrix, np.zeros(3))).ravel().tolist())
        topic = '/' + self.config['arm'] + '_wrist_camera'
        self.images = self.create_publisher(Image, topic + '/image_raw', qos_profile_sensor_data)
        self.infos = self.create_publisher(CameraInfo, topic + '/camera_info', qos_profile_sensor_data)
        self.previews = self.create_publisher(
            CompressedImage, topic + '/preview/compressed', qos_profile_sensor_data)
        self.camera = None
        self.enabled_state = self.create_publisher(Bool, topic + '/enabled', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_timer = self.create_timer(1.0, self.publish_enabled)
        self.publish_enabled()
        self.enable_service = self.create_service(SetBool, topic + '/set_enabled', self.set_enabled)
        self.timer = self.create_timer(1. / self.config['publish_rate_hz'], self.publish_frame)
        if self.get_parameter('start_enabled').value:
            self.open_camera()

    def open_camera(self) -> None:
        if self.camera is not None:
            return
        camera = cv2.VideoCapture(self.config['camera_device'], cv2.CAP_V4L2)
        try:
            if not camera.isOpened():
                raise RuntimeError('Cannot open configured wrist USB camera')
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['width'])
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['height'])
            camera.set(cv2.CAP_PROP_FPS, self.config['publish_rate_hz'])
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            camera.release()
            raise
        self.camera = camera
        self.publish_enabled()
        self.get_logger().info(f"Wrist camera enabled: {self.config['arm']}")

    def close_camera(self) -> None:
        if self.camera is not None:
            self.camera.release()
            self.camera = None
            self.next_preview_at = 0.0
            self.publish_enabled()
            self.get_logger().info(f"Wrist camera disabled: {self.config['arm']}")

    def set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        try:
            if request.data:
                self.open_camera()
            else:
                self.close_camera()
        except Exception as error:
            response.success, response.message = False, str(error)
        else:
            response.success = True
            response.message = 'Wrist camera enabled' if self.camera is not None else 'Wrist camera disabled'
        self.publish_enabled()
        return response

    def publish_enabled(self) -> None:
        """Capture ownership, including an idle heartbeat; not proof of valid frames."""
        if self.context.ok():
            self.enabled_state.publish(Bool(data=self.camera is not None))

    def publish_frame(self) -> None:
        if self.camera is None:
            return
        # USB driver supplies no exposure timestamp here: stamp host acquisition.
        ok, frame = self.camera.read()
        expected = (self.config['height'], self.config['width'], 3)
        if not ok or frame.shape != expected:
            self.get_logger().error('Missing wrist RGB or unexpected image dimensions')
            return
        stamp = self.get_clock().now().to_msg()
        rectified = cv2.remap(frame, *self.maps, interpolation=cv2.INTER_LINEAR)
        message = Image(height=expected[0], width=expected[1], encoding='bgr8',
                        step=expected[1] * 3, data=rectified.tobytes())
        message.header.stamp, message.header.frame_id = stamp, self.config['frame_id']
        info = deepcopy(self.info)
        info.header = deepcopy(message.header)
        self.images.publish(message)
        self.infos.publish(info)
        now = time.monotonic()
        if self.previews.get_subscription_count() and now >= self.next_preview_at:
            period = 1. / self.preview_rate
            if not self.next_preview_at or now - self.next_preview_at >= period:
                self.next_preview_at = now + period
            else:
                self.next_preview_at += period
            encoded, jpeg = cv2.imencode(
                '.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, self.preview_quality])
            if encoded:
                preview = CompressedImage(
                    header=deepcopy(message.header), format='bgr8; jpeg compressed bgr8',
                    data=jpeg.tobytes())
                self.previews.publish(preview)
            else:
                self.get_logger().error('Wrist preview JPEG encoding failed')

    def destroy_node(self):
        self.close_camera()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristRgbCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
