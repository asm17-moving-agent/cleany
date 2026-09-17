"""Simulation-only 3-D gate after Nav2 Collision Monitor.

/nav2/cmd_vel -> Collision Monitor -> /safety_2d/cmd_vel -> this gate -> /cmd_vel.
The downstream Gazebo command watchdog remains independent of this process.
"""
from __future__ import annotations
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from cleany_gazebo_sim.body_geometry import load_body_boxes
from cleany_gazebo_sim.obstacle_memory_node import transform
from cleany_gazebo_sim.point_cloud import cloud_xyz, xyz_cloud
from cleany_gazebo_sim.voxel_guard import GuardConfig, VoxelGuard


def twist_vector(msg: Twist) -> np.ndarray:
    values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
    if not np.isfinite(values).all() or np.any(np.abs(np.array(values)[[2, 3, 4]]) > 1e-8):
        raise ValueError('Non-planar or non-finite command')
    return np.array([msg.linear.x, msg.linear.y, msg.angular.z])


def pose_matrix(pose) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, 3] = transform(np.zeros((1, 3)), pose)[0]
    matrix[:3, :3] = (transform(np.eye(3), pose)-matrix[:3, 3]).T
    return matrix


class VoxelGuardNode(Node):
    def __init__(self):
        super().__init__('voxel_guard')
        share = Path(get_package_share_directory('cleany_gazebo_sim'))
        defaults = dict(profile=str(share/'config/cad_frame.yaml'), output_directory='',
                        command_topic='/safety_2d/cmd_vel', output_topic='/cmd_vel',
                        odom_topic='/wheel/odom', voxel_topic='/obstacle_memory/voxels',
                        status_topic='/obstacle_memory/voxel_status',
                        update_period_s=.05, command_timeout_s=.5, source_timeout_s=.8,
                        wall_source_timeout_s=5., processing_budget_s=.15,
                        localization_jump_m=.25, localization_jump_rad=.3,
                        **vars(GuardConfig()))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.config = GuardConfig(**{key: self.p[key] for key in vars(GuardConfig())})
        for key in ('update_period_s', 'command_timeout_s', 'source_timeout_s', 'wall_source_timeout_s',
                    'processing_budget_s', 'localization_jump_m', 'localization_jump_rad'):
            if not np.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError('Invalid timing or localization bound')
        if self.p['command_topic'] == self.p['output_topic'] or not self.p['output_directory']:
            raise ValueError('Distinct command topics and explicit output directory required')
        self.directory = Path(self.p['output_directory'])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.boxes = load_body_boxes(Path(get_package_share_directory('cleany_description')), Path(self.p['profile']))
        self.guard = VoxelGuard(self.boxes, self.config)
        (self.directory/'parameters.json').write_text(json.dumps(self.p, indent=2))
        self.command = None
        self.command_wall = -1e9
        self.odom = None
        self.odom_wall = -1e9
        self.cloud = None
        self.cloud_wall = -1e9
        self.status = None
        self.status_wall = -1e9
        self.fault = None
        self.previous_map_odom = None
        self.last_sim = None
        self.previous_decision = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.velocity = self.create_publisher(Twist, self.p['output_topic'], 10)
        self.state = self.create_publisher(String, '/body_guard_3d/state', durable)
        self.risks = self.create_publisher(PointCloud2, '/body_guard_3d/risk_voxels', durable)
        self.create_subscription(Twist, self.p['command_topic'], self.on_command, 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.p['voxel_topic'], self.on_cloud, durable)
        self.create_subscription(String, self.p['status_topic'], self.on_status, durable)
        # A steady timer can still send zero when /clock or a data publisher stalls.
        self.create_timer(self.p['update_period_s'], self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command, self.command_wall = msg, time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_wall = msg, time.monotonic()

    def on_cloud(self, msg):
        try:
            if msg.header.frame_id != 'map':
                raise ValueError('Voxel cloud must use map frame')
            points = cloud_xyz(msg)
            if not np.isfinite(points).all():
                raise ValueError('Non-finite voxel center')
            self.cloud = (Time.from_msg(msg.header.stamp).nanoseconds/1e9, points)
            self.cloud_wall = time.monotonic()
        except ValueError as exc:
            self.cloud = None
            self.get_logger().warning(str(exc))

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_wall = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def check_inputs(self, now: float, wall: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        if self.cloud is None or self.status is None or self.odom is None:
            raise ValueError('Missing voxel map, status or wheel odometry')
        if self.fault:
            raise ValueError(self.fault)
        if not self.status['enabled']:
            raise ValueError('Voxel mapping is frozen')
        for stamp in (self.cloud_wall, self.status_wall, self.odom_wall):
            if wall-stamp > self.p['wall_source_timeout_s']:
                raise ValueError('Source wall-clock timeout')
        stamps = [self.cloud[0], self.status['stamp_s'],
                  Time.from_msg(self.odom.header.stamp).nanoseconds/1e9,
                  *[self.status['source_stamps'][name] for name in ('lidar', 'depth')]]
        ages = now-np.array(stamps)
        if not np.isfinite(ages).all() or (ages < -.02).any() or (ages > self.p['source_timeout_s']).any():
            raise ValueError('Stale or future sensor/voxel/odom stamp')
        if abs(self.cloud[0]-self.status['stamp_s']) > self.p['source_timeout_s']:
            raise ValueError('Inconsistent voxel metadata')
        if self.odom.child_frame_id != 'base_link':
            raise ValueError('Wheel velocity must use base_link')
        measured = twist_vector(self.odom.twist.twist)
        pose_msg = self.buffer.lookup_transform('map', 'base_link', Time())
        pose_age = now-Time.from_msg(pose_msg.header.stamp).nanoseconds/1e9
        if not -.02 <= pose_age <= self.p['source_timeout_s']:
            raise ValueError('Stale base pose')
        pose = pose_matrix(pose_msg.transform)
        if not np.isfinite(pose).all() or np.linalg.norm(pose[:2, 2]) > .05:
            raise ValueError('Non-planar base pose')
        map_odom = pose_matrix(self.buffer.lookup_transform('map', 'odom', Time()).transform)
        if self.previous_map_odom is not None:
            change = np.linalg.inv(self.previous_map_odom)@map_odom
            if np.linalg.norm(change[:2, 3]) > self.p['localization_jump_m'] or abs(np.arctan2(change[1, 0], change[0, 0])) > self.p['localization_jump_rad']:
                self.fault = 'Localization jumped; rebuild voxel map before restarting guard'
                raise ValueError(self.fault)
        self.previous_map_odom = map_odom
        size = np.asarray(self.status['voxel_size'], dtype=float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid voxel dimensions')
        return self.cloud[1], size, pose, measured, float(max(0., ages.max(), pose_age))

    def update(self):
        start = time.monotonic()
        now = self.get_clock().now().nanoseconds/1e9
        output = np.zeros(3)
        report = dict(mode='simulation_serial_3d_guard', sim_s=now,
                      observation_scope='observed_voxels_only; unknown space is not certified free')
        risk_points = np.empty((0, 3))
        try:
            if self.last_sim is not None and now < self.last_sim-.01:
                self.fault = 'Simulation clock reset; rebuild voxel map and restart guard'
            self.last_sim = now
            points, size, pose, measured, source_age = self.check_inputs(now, start)
            if self.command is None or start-self.command_wall > self.p['command_timeout_s']:
                raise ValueError('Command timeout')
            requested = twist_vector(self.command)
            # Include observed input age in the prediction, in addition to reaction time.
            config = replace(self.config, reaction_s=self.config.reaction_s+source_age)
            self.guard.config = config
            max_speed = max(np.linalg.norm(requested[:2]), np.linalg.norm(measured[:2]))
            max_yaw = max(abs(requested[2]), abs(measured[2]))
            horizon = config.reaction_s+max(max_speed/config.linear_deceleration, max_yaw/config.angular_deceleration)+config.slowdown_lookahead_s
            radius = self.guard.radius+config.margin+max_speed*horizon+np.linalg.norm(size)
            nearby = points[np.linalg.norm(points[:, :2]-pose[:2, 3], axis=1) <= radius]
            output, collision = self.guard.evaluate(nearby, size, pose, requested, measured)
            risk_points = nearby[collision.pop('hit_indices')]
            report.update(collision, requested=requested.tolist(), measured=measured.tolist(),
                          nearby_voxels=len(nearby), total_voxels=len(points), source_age_s=source_age)
        except (ValueError, KeyError, TypeError, TransformException) as exc:
            report.update(decision='STOP_UNAVAILABLE', reason=str(exc))
        elapsed = time.monotonic()-start
        if elapsed > self.p['processing_budget_s']:
            output[:] = 0
            report.update(decision='STOP_PROCESSING_OVERRUN')
        report.update(output=output.tolist(), processing_ms=elapsed*1000)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, output)
        self.velocity.publish(msg)
        self.state.publish(String(data=json.dumps(report)))
        self.risks.publish(xyz_cloud(risk_points, 'map', self.get_clock().now().to_msg()))
        temporary = self.directory/'latest.tmp'
        temporary.write_text(json.dumps(report, indent=2))
        temporary.replace(self.directory/'latest.json')
        if report['decision'] != self.previous_decision:
            with (self.directory/'transitions.jsonl').open('a') as stream:
                stream.write(json.dumps(report)+'\n')
            self.previous_decision = report['decision']


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.velocity.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
