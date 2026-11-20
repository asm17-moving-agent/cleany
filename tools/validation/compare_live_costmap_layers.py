#!/usr/bin/env python3
"""Observe fresh single-layer costmaps without changing the navigation costmap."""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import VoxelGrid
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--layer', choices=['depth', 'lidar'], required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base = yaml.safe_load(args.parameters.read_text())['local_costmap']['local_costmap']['ros__parameters']
    rclpy.init()
    node = rclpy.create_node('costmap_layer_comparison')
    latest = {}
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: latest.update(live=msg), 10)
    node.create_subscription(OccupancyGrid, '/diagnostic_costmap/grid', lambda msg: latest.update(shadow=msg), 10)
    node.create_subscription(VoxelGrid, '/costmap/voxel_grid', lambda msg: latest.update(voxels=msg), 10)

    def pump(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)

    summaries = {}
    try:
        pump(.5)
        if any(name == 'costmap' and namespace == '/costmap' for name, namespace in node.get_node_names_and_namespaces()):
            raise RuntimeError('An unrelated standalone costmap already exists')
        for key, layer in [(args.layer, 'depth_layer' if args.layer == 'depth' else 'obstacle_layer')]:
            parameters = deepcopy(base)
            parameters.update(plugins=[layer], publish_frequency=5.)
            if key == 'depth':
                parameters[layer]['publish_voxel_map'] = True
            config = args.output/f'{key}-parameters.yaml'
            config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
            binary = Path(get_package_prefix('nav2_costmap_2d'))/'lib/nav2_costmap_2d/nav2_costmap_2d'
            log = (args.output/f'{key}.log').open('w')
            process = subprocess.Popen(
                [str(binary), '--ros-args', '--params-file', str(config), '-r', 'costmap:=/diagnostic_costmap/grid'],
                stdout=log, stderr=log)
            try:
                client = node.create_client(ChangeState, '/costmap/costmap/change_state')
                if not client.wait_for_service(timeout_sec=5.):
                    raise RuntimeError('Diagnostic costmap lifecycle service unavailable')
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
                    if not future.done() or not future.result().success:
                        raise RuntimeError('Diagnostic costmap transition failed')
                latest.pop('shadow', None)
                pump(7.)
                record = {}
                arrays = {}
                for name in ('live', 'shadow'):
                    msg = latest[name]
                    cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
                    arrays[name] = cells
                    record[name] = dict(info=message_to_ordereddict(msg.info),
                                        stamp=message_to_ordereddict(msg.header), occupied=int((cells == 100).sum()))
                if key == 'depth' and 'voxels' in latest:
                    record['voxels'] = message_to_ordereddict(latest['voxels'])
                np.savez_compressed(args.output/f'{key}.npz', **arrays)
                (args.output/f'{key}.json').write_text(json.dumps(record, indent=2)+'\n')
                summaries[key] = {name: record[name]['occupied'] for name in ('live', 'shadow')}
                node.destroy_client(client)
                for transition in (4, 2):
                    client = node.create_client(ChangeState, '/costmap/costmap/change_state')
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
                    node.destroy_client(client)
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.)
                log.close()
            pump(2.)
        print(json.dumps(summaries))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
