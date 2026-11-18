"""Rectify RGB and register native depth with a measured RGB camera model."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


def align_depth(depth_m: np.ndarray, depth_k: np.ndarray, rgb_k: np.ndarray,
                rgb_from_depth: np.ndarray, rgb_size: tuple[int, int], *,
                native: bool = True) -> np.ndarray:
    """Project native rectified depth into rectified RGB pixels with a Z-buffer.

    Depth values are optical Z in metres. RGB distortion is handled by rectifying
    RGB separately; this projection therefore uses the new RGB K with zero D.
    """
    height, width = depth_m.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    if native and hasattr(cv2, 'rgbd') and hasattr(cv2.rgbd, 'registerDepth'):
        source = np.asarray(depth_m, dtype=np.float32).copy()
        t = rgb_from_depth
        color_z = ((t[2, 0] * (u - depth_k[0, 2]) / depth_k[0, 0]
                    + t[2, 1] * (v - depth_k[1, 2]) / depth_k[1, 1]
                    + t[2, 2]) * source + t[2, 3])
        # OpenCV does not reject negative transformed depths itself.
        source[~np.isfinite(source) | (source <= 0) | (color_z <= 0)] = 0
        out = cv2.rgbd.registerDepth(depth_k, rgb_k, None, rgb_from_depth,
                                     source, rgb_size, depthDilation=False)
        out[~np.isfinite(out) | (out <= 0)] = 0
        return out
    z = depth_m.ravel()
    valid = np.isfinite(z) & (z > 0)
    z = z[valid]
    dx = (u.ravel()[valid] - depth_k[0, 2]).astype(np.float32) * z / depth_k[0, 0]
    dy = (v.ravel()[valid] - depth_k[1, 2]).astype(np.float32) * z / depth_k[1, 1]
    # Explicit elementwise projection avoids costly threaded BLAS dispatch on Jetson.
    t = rgb_from_depth
    cx = t[0, 0]*dx + t[0, 1]*dy + t[0, 2]*z + t[0, 3]
    cy = t[1, 0]*dx + t[1, 1]*dy + t[1, 2]*z + t[1, 3]
    cz = t[2, 0]*dx + t[2, 1]*dy + t[2, 2]*z + t[2, 3]
    valid = np.isfinite(cx) & np.isfinite(cy) & np.isfinite(cz) & (cz > 0)
    cx, cy, cz = cx[valid], cy[valid], cz[valid]
    x = np.rint((rgb_k[0, 0]*cx + rgb_k[0, 1]*cy) / cz + rgb_k[0, 2]).astype(np.int64)
    y = np.rint((rgb_k[1, 0]*cx + rgb_k[1, 1]*cy) / cz + rgb_k[1, 2]).astype(np.int64)
    out_w, out_h = rgb_size
    valid = (0 <= x) & (x < out_w) & (0 <= y) & (y < out_h)
    out = np.full(out_h * out_w, np.inf, dtype=np.float32)
    np.minimum.at(out, y[valid] * out_w + x[valid], cz[valid])
    out[~np.isfinite(out)] = 0
    return out.reshape(out_h, out_w)


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from cleany_perception.rgbd_snapshot import _depth_array, _rgb_array, stamp_nanoseconds

    class CalibratedRgbd(Node):
        """Publish matched RGB, registered depth and CameraInfo on /camera topics.

        Native depth uses the SDK's depth model and depth-to-color extrinsics.
        The measured RGB model owns both RGB rectification and depth projection.
        """

        def __init__(self):
            super().__init__('calibrated_rgbd')
            self.declare_parameter('intrinsics_file', '')
            self.declare_parameter('maximum_pair_skew_s', .025)
            self.path = Path(self.get_parameter('intrinsics_file').value).expanduser()
            intr = json.loads(self.path.read_text())
            self.k = np.array(intr['K'], dtype=float).reshape(3, 3)
            self.d = np.array(intr['D'], dtype=float)
            self.size = (int(intr['width']), int(intr['height']))
            if (not np.isfinite(self.k).all() or not np.isfinite(self.d).all()
                    or self.k[0, 0] <= 0 or self.k[1, 1] <= 0):
                raise ValueError('Invalid measured RGB intrinsics')
            self.maps = cv2.initUndistortRectifyMap(self.k, self.d, None, self.k,
                                                  self.size, cv2.CV_32FC1)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.depths = []
            self.colors = []
            self.depth_info = None
            self.last_stamp = 0
            self.last_preview = 0.
            self.maximum_skew = float(self.get_parameter('maximum_pair_skew_s').value)
            if not 0 <= self.maximum_skew < 1 / 30:
                raise ValueError('Pair skew must be below one 30Hz frame interval')
            self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
            self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
            self.color_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
            self.aligned_info = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
            self.preview = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', qos_profile_sensor_data)
            self.status = self.create_publisher(String, '/camera_calibration/rgbd_status', 1)
            self.create_subscription(Image, '/camera/sdk/color/image_raw', self.color, qos_profile_sensor_data)
            self.create_subscription(Image, '/camera/raw_depth/image_raw', self.depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '/camera/raw_depth/camera_info', self.info, qos_profile_sensor_data)
            self.get_logger().info(f'Measured RGB calibration: {self.path}')

        def info(self, msg):
            self.depth_info = msg

        def color(self, msg):
            self.colors.append(msg)
            self.colors = self.colors[-4:]
            self.publish_pairs()

        def depth(self, msg):
            self.depths.append(msg)
            self.depths = self.depths[-4:]
            self.publish_pairs()

        def publish_pairs(self):
            if self.depth_info is None or not self.depths:
                return
            for color in self.colors[:]:
                stamp = stamp_nanoseconds(color)
                if stamp <= self.last_stamp:
                    self.colors.remove(color)
                    continue
                depth = min(self.depths, key=lambda x: abs(stamp_nanoseconds(x) - stamp))
                skew = abs(stamp_nanoseconds(depth) - stamp) / 1e9
                if skew > self.maximum_skew:
                    continue
                try:
                    transform = self.buffer.lookup_transform(color.header.frame_id,
                        depth.header.frame_id, rclpy.time.Time()).transform
                except TransformException:
                    continue
                if (color.width, color.height) != self.size:
                    raise ValueError('RGB resolution differs from measured intrinsics')
                info = self.depth_info
                if (depth.width, depth.height) != (info.width, info.height):
                    raise ValueError('Native depth CameraInfo size mismatch')
                q = transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                matrix = np.eye(4)
                matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
                p = transform.translation
                matrix[:3, 3] = [p.x, p.y, p.z]
                # image_rect_raw uses the rectified projection matrix, not raw D.
                depth_k = np.array(info.p).reshape(3, 4)[:, :3]
                if not np.isfinite(depth_k).all() or min(depth_k[0, 0], depth_k[1, 1]) <= 0:
                    raise ValueError('Invalid native rectified depth projection')
                rgb = cv2.remap(_rgb_array(color), *self.maps, cv2.INTER_LINEAR)
                registered = align_depth(_depth_array(depth, .001), depth_k, self.k,
                                         matrix, self.size)
                header = deepcopy(color.header)
                rgb_msg = Image(header=header, width=self.size[0], height=self.size[1],
                    encoding='rgb8', is_bigendian=0, step=self.size[0]*3, data=rgb.tobytes())
                dep_msg = Image(header=header, width=self.size[0], height=self.size[1],
                    encoding='32FC1', is_bigendian=0, step=self.size[0]*4, data=registered.tobytes())
                camera = CameraInfo(header=header, width=self.size[0], height=self.size[1],
                    distortion_model='plumb_bob', d=[0.]*5, k=self.k.ravel().tolist(),
                    r=np.eye(3).ravel().tolist(),
                    p=np.column_stack((self.k, np.zeros(3))).ravel().tolist())
                self.color_pub.publish(rgb_msg)
                self.depth_pub.publish(dep_msg)
                self.color_info.publish(camera)
                self.aligned_info.publish(camera)
                now = time.monotonic()
                if now - self.last_preview >= .2:
                    ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                            [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        self.preview.publish(CompressedImage(header=header, format='jpeg', data=jpeg.tobytes()))
                    self.status.publish(String(data=json.dumps({'intrinsics_file': str(self.path),
                        'intrinsics_sha256': hashlib.sha256(self.path.read_bytes()).hexdigest(),
                        'source_skew_s': skew, 'depth_valid_fraction': float(np.mean(registered > 0)),
                        'stamp_ns': stamp, 'sdk_depth_model_retained': True})))
                    self.last_preview = now
                self.last_stamp = stamp
                self.colors.remove(color)

    cv2.setNumThreads(1)
    rclpy.init(args=args)
    node = CalibratedRgbd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
