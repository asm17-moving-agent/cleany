#!/usr/bin/env python3
"""Record live base-frame obstacle bounds without commanding the robot."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from cleany_navigation.point_cloud import cloud_xyz


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--min-z', type=float, required=True)
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('depth_cutoff_probe')
    samples = []

    def observe(message: PointCloud2) -> None:
        points = cloud_xyz(message)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        samples.append({'stamp': stamp, 'frame': message.header.frame_id,
                        'count': len(points),
                        'min_z': float(points[:, 2].min()) if len(points) else None,
                        'below_cutoff': int((points[:, 2] < args.min_z-1e-6).sum()),
                        'nonfinite': int((~np.isfinite(points).all(axis=1)).sum()),
                        'age_s': node.get_clock().now().nanoseconds*1e-9-stamp})

    node.create_subscription(PointCloud2, '/perception/depth/obstacle_points',
                             observe, qos_profile_sensor_data)
    end = time.monotonic()+args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    valid = [s for s in samples if s['count']]
    passed = len(valid) >= 5 and all(s['frame'] == 'base_link' and
              s['below_cutoff'] == 0 and s['nonfinite'] == 0 and
              -.05 <= s['age_s'] <= .5 for s in valid)
    result = {'passed': passed, 'cutoff_base_z': args.min_z, 'samples': samples}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'passed': passed, 'nonempty_frames': len(valid),
                      'min_z': min((s['min_z'] for s in valid), default=None),
                      'below_cutoff': sum(s['below_cutoff'] for s in samples)}))
    node.destroy_node()
    rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
