#!/usr/bin/env python3
"""Capture stationary depth/costmap evidence; optionally restore a saved pose."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from cleany_interfaces.msg import TeleopStatus
from cleany_navigation.point_cloud import cloud_xyz


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--restore-pose', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('depth_navigation_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    result = {'clouds': {}, 'odom': [], 'teleop': None}
    latest = {}

    def on_cloud(key, message):
        points = cloud_xyz(message)
        result['clouds'].setdefault(key, []).append(dict(
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            frame=message.header.frame_id, count=len(points),
            min_z=float(points[:, 2].min()) if len(points) else None))
        latest[key] = points
        latest[key+'_msg'] = message

    for key, topic in [('marking', '/perception/depth/nav_obstacle_points'),
                       ('clearing', '/perception/depth/nav_points')]:
        node.create_subscription(PointCloud2, topic, lambda msg, k=key: on_cloud(k, msg), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: latest.update(costmap=msg), 10)
    node.create_subscription(Odometry, '/odom', lambda msg: result['odom'].append(message_to_ordereddict(msg.twist.twist)), qos_profile_sensor_data)
    node.create_subscription(TeleopStatus, '/teleop/status', lambda msg: result.update(teleop=message_to_ordereddict(msg)), 10)
    if args.restore_pose:
        pose = json.loads(args.restore_pose.read_text())['map_to_base']
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = pose['translation']['x']
        message.pose.pose.position.y = pose['translation']['y']
        for key, value in pose['rotation'].items():
            setattr(message.pose.pose.orientation, key, value)
        message.pose.covariance[0] = message.pose.covariance[7] = .01
        message.pose.covariance[35] = .01
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        deadline = time.monotonic()+5.
        def amcl_subscribed():
            return any(info.node_name == 'amcl' for info in node.get_subscriptions_info_by_topic('/initialpose'))
        while not amcl_subscribed() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not amcl_subscribed():
            raise RuntimeError('No AMCL initial-pose subscriber')
        # Discovery of another consumer is not confirmation that AMCL received it.
        for _ in range(3):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            until = time.monotonic()+.4
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
    deadline = time.monotonic()+args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if buffer.can_transform('map', 'base_link', Time()):
        result['map_to_base'] = message_to_ordereddict(buffer.lookup_transform('map', 'base_link', Time()).transform)
    if 'costmap' in latest:
        msg = latest['costmap']
        if buffer.can_transform(msg.header.frame_id, 'base_link', Time()):
            result['costmap_to_base'] = message_to_ordereddict(
                buffer.lookup_transform(msg.header.frame_id, 'base_link', Time()).transform)
        result['costmap'] = dict(frame=msg.header.frame_id, info=message_to_ordereddict(msg.info),
                                 occupied=int(np.sum(np.asarray(msg.data) == 100)))
        np.savez_compressed(args.output.with_suffix('.npz'),
                            costmap=np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                            marking=latest.get('marking', np.empty((0, 3))),
                            clearing=latest.get('clearing', np.empty((0, 3))))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'cloud_frames': {k: len(v) for k, v in result['clouds'].items()},
                      'costmap': result.get('costmap'), 'map_to_base': result.get('map_to_base'),
                      'teleop': result['teleop']}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
