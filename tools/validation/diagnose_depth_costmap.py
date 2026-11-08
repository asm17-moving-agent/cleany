#!/usr/bin/env python3
"""Passively measure sensor/costmap timing and current obstacle support."""

import argparse
from collections import defaultdict, deque
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial import cKDTree

from cleany_navigation.point_cloud import cloud_xyz
from cleany_navigation.transforms import transform


def process_ticks():
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry/'cmdline').read_bytes().decode().split('\0')
            stat = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            if any(name in ' '.join(cmd[:2]) for name in ('depth_clearance', 'depth_obstacles_preview', 'controller_server', 'rviz2', 'realsense2_camera_node')):
                result[entry.name] = dict(ticks=int(stat[11])+int(stat[12]), command=' '.join(cmd[:2]))
        except (OSError, ValueError, UnicodeError):
            pass
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('depth_costmap_diagnostic')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    timing = defaultdict(list)
    history = {name: deque(maxlen=60) for name in ('marking', 'scan')}
    latest = {}

    def receive(name, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now = node.get_clock().now().nanoseconds
        timing[name].append([time.monotonic(), stamp, (now-stamp)*1e-9])
        if name in history:
            history[name].append(msg)
        latest[name] = msg

    topics = [('image', Image, '/camera/d435/depth/image_rect_raw'),
              ('marking', PointCloud2, '/perception/depth/nav_obstacle_points'),
              ('clearing', PointCloud2, '/perception/depth/nav_points'),
              ('costmap', OccupancyGrid, '/local_costmap/costmap'),
              ('update_footprint', PolygonStamped, '/local_costmap/published_footprint'),
              ('scan', LaserScan, '/scan')]
    for key, cls, topic in topics:
        node.create_subscription(cls, topic, lambda msg, k=key: receive(k, msg), qos_profile_sensor_data)
    start = time.monotonic()
    cpu_before = process_ticks()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
    elapsed = time.monotonic()-start
    cpu_after = process_ticks()
    result = {'duration_s': elapsed, 'timing': {}, 'process_cpu_percent_one_core': {}}
    for key, samples in timing.items():
        values = np.asarray(samples)
        intervals = np.diff(values[:, 0])
        result['timing'][key] = dict(count=len(samples),
            received_hz=float(1/intervals.mean()) if len(intervals) else None,
            interval_ms_p50_p95_max=(np.percentile(intervals*1000, [50, 95, 100]).tolist() if len(intervals) else []),
            source_age_ms_p50_p95_max=np.percentile(values[:, 2]*1000, [50, 95, 100]).tolist())
    for pid in cpu_before.keys() & cpu_after.keys():
        result['process_cpu_percent_one_core'][pid] = dict(command=cpu_before[pid]['command'],
            percent=100*(cpu_after[pid]['ticks']-cpu_before[pid]['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed)
    image_arrivals = {row[1]: row[0] for row in timing['image']}
    delta = [1000*(row[0]-image_arrivals[row[1]]) for row in timing['marking'] if row[1] in image_arrivals]
    result['matching_image_to_marking_arrival_ms_p50_p95_max'] = np.percentile(delta, [50, 95, 100]).tolist() if delta else []
    clouds = {}
    for name, messages in history.items():
        arrays = []
        for msg in messages:
            if isinstance(msg, LaserScan):
                ranges = np.asarray(msg.ranges)
                angle = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
                keep = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
                points = np.column_stack((ranges[keep]*np.cos(angle[keep]), ranges[keep]*np.sin(angle[keep]), np.zeros(keep.sum())))
            else:
                points = cloud_xyz(msg)
            try:
                pose = buffer.lookup_transform('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
                arrays.append(transform(points, pose))
            except TransformException:
                continue
        clouds[name] = np.vstack(arrays) if arrays else np.empty((0, 3))
    if 'costmap' in latest:
        msg = latest['costmap']
        result['map_info'] = message_to_ordereddict(msg.info)
        cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        rows, cols = np.nonzero(cells == 100)
        xy = np.column_stack((msg.info.origin.position.x+(cols+.5)*msg.info.resolution,
                              msg.info.origin.position.y+(rows+.5)*msg.info.resolution))
        distances = {key: cKDTree(points[:, :2]).query(xy)[0] if len(points) else np.full(len(xy), np.inf)
                     for key, points in clouds.items()}
        result['occupied_cell_support'] = {'tolerance_m': .075, 'occupied_cells': len(xy),
            'depth_supported': int((distances['marking'] < .075).sum()),
            'lidar_supported': int((distances['scan'] < .075).sum()),
            'no_recent_hit': int(((distances['marking'] >= .075) & (distances['scan'] >= .075)).sum())}
        np.savez_compressed(args.output.with_suffix('.npz'), costmap=cells,
                            occupied_xy=xy, depth_distance=distances['marking'],
                            scan_distance=distances['scan'], **clouds)
        if buffer.can_transform('odom', 'base_link', Time()):
            result['odom_to_base'] = message_to_ordereddict(buffer.lookup_transform('odom', 'base_link', Time()).transform)
    result['timing_samples'] = timing
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('timing_samples', 'map_info')}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
