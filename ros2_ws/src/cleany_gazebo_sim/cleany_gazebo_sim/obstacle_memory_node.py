"""Persistent obstacle observations in the saved SLAM map, without inflation."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from cleany_gazebo_sim.point_cloud import cloud_xyz, xyz_cloud
from tf2_ros import Buffer, TransformException, TransformListener

from cleany_gazebo_sim.obstacle_memory import MemoryGeometry, ObstacleMemory


def transform(points: np.ndarray, pose) -> np.ndarray:
    q, t = pose.rotation, pose.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return points@matrix.T+[t.x, t.y, t.z]


class ObstacleMemoryNode(Node):
    def __init__(self):
        super().__init__('obstacle_memory')
        for name, value in [('map_topic', '/map'), ('scan_topic', '/scan'),
                            ('depth_topic', '/camera/head/depth/points'), ('output_directory', ''),
                            ('restore_path', ''), ('floor_z', -0.38), ('min_height', 0.05),
                            ('max_height', 2.0), ('z_resolution', 0.10), ('update_period', 0.5),
                            ('save_period', 5.0), ('mark_range', 4.0), ('clear_range', 5.0),
                            ('scan_stride', 2), ('max_depth_rays', 1200), ('publish_3d', False),
                            ('mark_all_depth_points', False), ('source_timeout_s', .8)]:
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in (
            'output_directory', 'restore_path', 'floor_z', 'min_height', 'max_height', 'z_resolution',
            'update_period', 'save_period', 'mark_range', 'clear_range', 'scan_stride', 'max_depth_rays',
            'publish_3d', 'mark_all_depth_points', 'source_timeout_s')}
        if not self.values['output_directory']:
            raise ValueError('An explicit output_directory is required')
        if min(self.values[k] for k in ('update_period', 'save_period', 'scan_stride', 'max_depth_rays', 'mark_range')) <= 0:
            raise ValueError('Periods, sampling counts and marking range must be positive')
        if self.values['clear_range'] < self.values['mark_range']:
            raise ValueError('Clearing range must cover marking range')
        self.directory = Path(self.values['output_directory'])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.memory = None
        self.latest = {}
        self.processed = {}
        self.counts = {'lidar': 0, 'depth': 0, 'tf_waits': 0}
        self.source_stamps = {}
        self.voxel_revision = 0
        self.previous_grid = None
        self.restored = False
        self.enabled = True
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.voxel_publisher = self.create_publisher(PointCloud2, '/obstacle_memory/voxels', durable)
        self.voxel_status = self.create_publisher(String, '/obstacle_memory/voxel_status', durable)
        self.grid_publishers = {key: self.create_publisher(OccupancyGrid, '/obstacle_memory/'+key, durable)
                           for key in ('grid', 'lidar', 'depth')}
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value, self.on_map, durable)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 lambda m: self.latest.update(lidar=m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter('depth_topic').value,
                                 lambda m: self.latest.update(depth=m), qos_profile_sensor_data)
        self.create_timer(self.values['update_period'], self.update)
        self.create_timer(self.values['save_period'], self.save)
        self.create_service(Trigger, '/obstacle_memory/save', self.save_service)
        self.create_service(SetBool, '/obstacle_memory/set_enabled', self.set_enabled)

    def set_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = 'Recording enabled' if self.enabled else 'Recording frozen'
        self.save()
        if self.memory is not None and self.values['publish_3d']:
            self.publish_voxels()
        return response

    def on_map(self, msg):
        if self.memory is not None:
            return
        q = msg.info.origin.orientation
        if msg.header.frame_id != 'map' or max(abs(q.x), abs(q.y), abs(q.z)) > 1e-8:
            raise ValueError('Obstacle memory requires an axis-aligned saved map')
        self.info = deepcopy(msg.info)
        v = self.values
        geometry = MemoryGeometry(msg.info.width, msg.info.height, msg.info.resolution,
                                  msg.info.origin.position.x, msg.info.origin.position.y,
                                  v['floor_z']+v['min_height'], v['floor_z']+v['max_height'],
                                  v['z_resolution'], hashlib.sha256(np.array(msg.data, dtype=np.int8).tobytes()).hexdigest())
        self.memory = (ObstacleMemory.restore(Path(v['restore_path']), geometry)
                       if v['restore_path'] else ObstacleMemory(geometry))
        self.restored = bool(v['restore_path'])
        self.publish()
        self.save()

    def observe(self, source, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        stamp_s = stamp[0]+stamp[1]*1e-9
        now = self.get_clock().now().nanoseconds/1e9
        if self.values['publish_3d'] and not 0 <= now-stamp_s <= self.values['source_timeout_s']:
            return
        if self.processed.get(source) == stamp:
            return
        try:
            pose = self.buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                                timeout=Duration(seconds=0.03)).transform
        except TransformException:
            self.counts['tf_waits'] += 1
            return
        v = self.values
        full_hits = None
        if source == 'lidar':
            indices = np.arange(0, len(msg.ranges), v['scan_stride'])
            ranges = np.asarray(msg.ranges)[indices]
            valid = (np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)) | np.isposinf(ranges)
            ranges, indices = ranges[valid], indices[valid]
            hits = np.isfinite(ranges) & (ranges < min(msg.range_max, v['mark_range']))
            ranges = np.minimum(ranges, min(msg.range_max, v['clear_range']))
            angles = msg.angle_min+indices*msg.angle_increment
            points = np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles), np.zeros(len(ranges))))
        else:
            full_points = cloud_xyz(msg)
            full_points = full_points[np.isfinite(full_points).all(axis=1)]
            if v['mark_all_depth_points']:
                full_distances = np.linalg.norm(full_points, axis=1)
                full_hits = full_points[(full_distances > .01) & (full_distances < v['mark_range'])]
            stride = max(1, int(np.ceil(len(full_points)/v['max_depth_rays'])))
            # Cycle the clearing subset so permanently skipped pixels cannot
            # leave occupied voxels behind after an object disappears.
            phase = self.counts['depth'] % stride if v['publish_3d'] else 0
            points = full_points[phase::stride]
            distances = np.linalg.norm(points, axis=1)
            valid = distances > 0.01
            points, distances = points[valid], distances[valid]
            hits = distances < v['mark_range']
            points *= np.minimum(1.0, v['clear_range']/distances)[:, None]
        sensor = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
        if not len(points):
            return
        self.memory.observe(source, sensor, transform(points, pose), hits, time.time(),
                            transform(full_hits, pose) if full_hits is not None else None)
        self.source_stamps[source] = stamp_s
        self.voxel_revision += 1
        self.processed[source] = stamp
        self.counts[source] += 1

    def update(self):
        if self.memory is None or not self.enabled:
            return
        for source, msg in list(self.latest.items()):
            self.observe(source, msg)
        self.publish()

    def publish(self):
        if self.values['publish_3d']:
            self.publish_voxels()
        fused = None
        for name, publisher in self.grid_publishers.items():
            grid = self.memory.grid(None if name == 'grid' else name)
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info = self.info
            msg.data = grid.ravel().tolist()
            publisher.publish(msg)
            if name == 'grid':
                fused = grid.ravel()
        changed = np.arange(len(fused)) if self.previous_grid is None else np.flatnonzero(fused != self.previous_grid)
        if len(changed):
            with (self.directory/'changes.jsonl').open('a') as stream:
                stream.write(json.dumps({'unix_s': time.time(), 'sim_s': self.get_clock().now().nanoseconds/1e9,
                                         'indices': changed.tolist(), 'values': fused[changed].tolist()})+'\n')
        self.previous_grid = fused.copy()

    def publish_voxels(self):
        g = self.memory.geometry
        stamp = self.get_clock().now().to_msg()
        points = self.memory.occupied_centers()
        self.voxel_publisher.publish(xyz_cloud(points, 'map', stamp))
        status = dict(enabled=self.enabled, revision=self.voxel_revision,
                      stamp_s=stamp.sec+stamp.nanosec*1e-9, source_stamps=self.source_stamps,
                      voxel_size=[g.resolution, g.resolution, g.z_resolution],
                      occupied_voxels=len(points), map_id=g.map_id,
                      min_xyz=[g.origin_x, g.origin_y, g.min_z],
                      max_xyz=[g.origin_x+g.width*g.resolution, g.origin_y+g.height*g.resolution, g.max_z])
        self.voxel_status.publish(String(data=json.dumps(status)))
        (self.directory/'voxel_status.json').write_text(json.dumps(status, indent=2))

    def save(self):
        if self.memory is None:
            return
        self.memory.save(self.directory/'obstacles.npz')
        grid = self.memory.grid()
        report = dict(counts=self.counts, restored=self.restored, enabled=self.enabled,
                      observed_cells=int(np.count_nonzero(grid >= 0)), occupied_cells=int(np.count_nonzero(grid == 100)),
                      total_cells=grid.size, observed_area_m2=float(np.count_nonzero(grid >= 0)*self.info.resolution**2),
                      occupied_by_source={s: int(np.count_nonzero(self.memory.grid(s) == 100)) for s in self.memory.sources})
        (self.directory/'status.json').write_text(json.dumps(report, indent=2))

    def save_service(self, request, response):
        self.save()
        response.success = self.memory is not None
        response.message = str(self.directory/'obstacles.npz')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
