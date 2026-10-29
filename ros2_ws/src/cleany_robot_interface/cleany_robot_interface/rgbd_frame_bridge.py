"""Attach the RealSense SDK tree to the existing calibrated RGB optical frame."""
import json
import math
from pathlib import Path
import time


def inverse_transform(xyz: list[float], quaternion: list[float]) -> tuple[list[float], list[float]]:
    if (len(xyz) != 3 or len(quaternion) != 4
            or not all(math.isfinite(v) for v in xyz + quaternion)
            or not math.isclose(sum(v*v for v in quaternion), 1.0, abs_tol=1e-6)):
        raise ValueError('Invalid SDK rigid transform')
    x, y, z, w = quaternion
    rotation = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    return [-sum(rotation[j][i]*xyz[j] for j in range(3)) for i in range(3)], [-x, -y, -z, w]


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

    rclpy.init(args=args)
    node = rclpy.create_node('read_only_rgbd_frame_bridge')
    node.declare_parameter('config_file', '')
    node.declare_parameter('receipt_file', '')
    cfg = json.loads(Path(node.get_parameter('config_file').value).read_text())
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = StaticTransformBroadcaster(node)
    try:
        deadline = time.monotonic() + 20
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                sdk = buffer.lookup_transform(cfg['sdk_base_frame'], cfg['sdk_color_optical_frame'],
                                              rclpy.time.Time()).transform
                break
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError('Missing RealSense SDK internal RGB transform')
        else:
            return
        translation, quaternion = inverse_transform(
            [sdk.translation.x, sdk.translation.y, sdk.translation.z],
            [sdk.rotation.x, sdk.rotation.y, sdk.rotation.z, sdk.rotation.w])
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = cfg['calibrated_rgb_frame']
        msg.child_frame_id = cfg['sdk_base_frame']
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = quaternion
        publisher.sendTransform(msg)
        receipt = node.get_parameter('receipt_file').value
        if receipt:
            Path(receipt).write_text(json.dumps({
                'parent': msg.header.frame_id, 'child': msg.child_frame_id,
                'translation': translation, 'quaternion_xyzw': quaternion,
                'method': 'inverse of SDK camera_link to color optical transform',
                'physical_calibration_verified': False}, indent=2) + '\n')
        node.get_logger().info('Calibrated RGB frame connected to the RealSense SDK frame tree')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
