#!/usr/bin/env python3
"""Capture stationary depth/LiDAR evidence without commanding the robot."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage


def stamp(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('depth_lidar_wall_capture')
    depths, depth_times, scans, odom, transforms, infos = [], [], [], [], {}, []
    dynamic, head = [], []
    last_depth = [0.0]

    def image_callback(msg: Image) -> None:
        if time.monotonic() - last_depth[0] < 0.3:
            return
        if msg.encoding != '16UC1':
            raise ValueError(f'Expected millimetre depth: {msg.encoding}')
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        depth = np.frombuffer(rows[:, :msg.width * 2].copy().tobytes(), dtype=dtype)
        depths.append(depth.reshape(msg.height, msg.width).copy())
        depth_times.append({'stamp': stamp(msg), 'received': time.monotonic(), 'frame': msg.header.frame_id})
        last_depth[0] = time.monotonic()

    def scan_callback(msg: LaserScan) -> None:
        scans.append({'stamp': stamp(msg), 'received': time.monotonic(),
                      'frame': msg.header.frame_id, 'angle_min': msg.angle_min,
                      'angle_increment': msg.angle_increment, 'scan_time': msg.scan_time,
                      'range_min': msg.range_min, 'range_max': msg.range_max,
                      'ranges': [float(x) if np.isfinite(x) else None for x in msg.ranges]})

    def info_callback(msg: CameraInfo) -> None:
        infos[:] = [{'frame': msg.header.frame_id, 'k': list(msg.k),
                     'd': list(msg.d), 'width': msg.width, 'height': msg.height}]

    def odom_callback(msg: Odometry) -> None:
        v, w, p = msg.twist.twist.linear, msg.twist.twist.angular, msg.pose.pose.position
        odom.append({'stamp': stamp(msg), 'v': [v.x, v.y, v.z],
                     'w': [w.x, w.y, w.z], 'position': [p.x, p.y, p.z]})

    def tf_callback(msg: TFMessage) -> None:
        for t in msg.transforms:
            p, q = t.transform.translation, t.transform.rotation
            transforms[t.child_frame_id] = {'parent': t.header.frame_id,
                                           'xyz': [p.x, p.y, p.z],
                                           'xyzw': [q.x, q.y, q.z, q.w]}

    def dynamic_callback(msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id not in ('d435_pan_link', 'd435_tilt_link', 'd435_link'):
                continue
            p, q = t.transform.translation, t.transform.rotation
            dynamic.append({'stamp': stamp(t), 'parent': t.header.frame_id,
                            'child': t.child_frame_id, 'xyz': [p.x, p.y, p.z],
                            'xyzw': [q.x, q.y, q.z, q.w]})

    def head_callback(msg: JointState) -> None:
        head.append({'stamp': stamp(msg), 'names': list(msg.name), 'position': list(msg.position)})

    node.create_subscription(Image, '/camera/d435/depth/image_rect_raw', image_callback, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/d435/depth/camera_info', info_callback, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan_raw', scan_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom_callback, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', tf_callback,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf', dynamic_callback, 100)
    node.create_subscription(JointState, '/head/joint_states', head_callback, 100)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not depths or not scans or not infos:
        raise RuntimeError('Missing depth, scan_raw or CameraInfo')
    np.savez_compressed(args.output / 'depth.npz', depth_mm=np.stack(depths))
    evidence = {'depth_times': depth_times, 'scans': scans, 'odom': odom,
                'camera_info': infos[0], 'tf_static': transforms,
                'tf_dynamic': dynamic, 'head_joint_states': head}
    (args.output / 'capture.json').write_text(json.dumps(evidence, indent=2))
    print(json.dumps({'depth_frames': len(depths), 'scans': len(scans), 'odom': len(odom),
                      'tf_children': list(transforms)}))


if __name__ == '__main__':
    main()
