"""Final command owner using direct LiDAR/depth and a steady-clock watchdog."""
from __future__ import annotations

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from cleany_navigation.body_geometry import load_body_boxes
from cleany_navigation.point_cloud import cloud_xyz
from cleany_navigation.reactive_guard import InputWatchdog, PlanarBrakingGuard
from cleany_navigation.transforms import stamp, transform
from cleany_navigation.voxel_guard import GuardConfig
from cleany_navigation.voxel_guard_node import localization_jumped, pose_matrix, twist_vector


class ReactiveGuardNode(Node):
    def __init__(self) -> None:
        super().__init__('reactive_guard')
        defaults = dict(profile='', allow_simulation_limits=False,
            command_topic='safety_2d/cmd_vel', raw_command_topic='nav2/cmd_vel', output_topic='cmd_vel',
            scan_topic='scan', depth_topic='camera/head/depth/points', odom_topic='odom',
            base_frame='base_link', odom_frame='odom', map_frame='map', depth_frame='head_camera_depth_frame',
            require_pan_feedback=True, pan_joint='head_pan_joint', joint_states_topic='joint_states',
            min_height=-.33, max_height=1.62, source_timeout_s=.5, command_timeout_s=.5,
            update_period_s=.05, processing_budget_s=.15, localization_jump_m=.25,
            localization_jump_rad=.3, **vars(GuardConfig()))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('source_timeout_s', 'command_timeout_s', 'update_period_s', 'processing_budget_s',
                    'localization_jump_m', 'localization_jump_rad'):
            if not np.isfinite(p[key]) or p[key] <= 0:
                raise ValueError('Invalid timing or localization bound')
        if (len({p['command_topic'], p['raw_command_topic'], p['output_topic']}) != 3
                or not p['profile'] or not np.isfinite([p['min_height'], p['max_height']]).all()
                or p['min_height'] >= p['max_height']):
            raise ValueError('Distinct command topics, body profile and height interval required')
        boxes = load_body_boxes(Path(get_package_share_directory('cleany_description')), Path(p['profile']),
                                allow_simulation_limits=p['allow_simulation_limits'])
        corners = np.concatenate([b.corners() for b in boxes])
        self.guard = PlanarBrakingGuard(corners[:, :2].min(axis=0), corners[:, :2].max(axis=0),
                                       GuardConfig(**{k: p[k] for k in vars(GuardConfig())}))
        self.watchdog = InputWatchdog(p['source_timeout_s'], p['command_timeout_s'])
        self.command = self.raw = self.odom = None
        self.command_wall = self.raw_wall = -np.inf
        self.pending = {}
        self.observations = {}
        self.previous_map_odom = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(Twist, p['output_topic'], 1)
        self.state = self.create_publisher(String, 'reactive_guard/state', 10)
        self.create_subscription(Twist, p['command_topic'], self.on_command, 1)
        self.create_subscription(Twist, p['raw_command_topic'], self.on_raw, 1)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, p['scan_topic'], lambda m: self.on_sensor('scan', m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p['depth_topic'], lambda m: self.on_sensor('depth', m), qos_profile_sensor_data)
        if p['require_pan_feedback']:
            self.create_subscription(JointState, p['joint_states_topic'], self.on_joint, qos_profile_sensor_data)
        self.create_timer(p['update_period_s'], self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg: Twist) -> None:
        self.command, self.command_wall = msg, time.monotonic()

    def on_raw(self, msg: Twist) -> None:
        self.raw, self.raw_wall = msg, time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.watchdog.sources['odom'] = (stamp(msg.header.stamp), time.monotonic())

    def on_joint(self, msg: JointState) -> None:
        if self.p['pan_joint'] not in msg.name:
            return
        i = msg.name.index(self.p['pan_joint'])
        valid = i < len(msg.position) and i < len(msg.velocity) and np.isfinite([msg.position[i], msg.velocity[i]]).all()
        self.watchdog.sources['joint'] = (stamp(msg.header.stamp) if valid else -np.inf, time.monotonic())

    def on_sensor(self, name: str, msg) -> None:
        received = time.monotonic()
        try:
            if not msg.header.frame_id:
                raise ValueError('Missing sensor frame')
            if name == 'depth':
                points = cloud_xyz(msg)
                if msg.header.frame_id != self.p['depth_frame'] or not len(points) or not np.isfinite(points).all():
                    raise ValueError('Invalid depth cloud')
            else:
                ranges = np.asarray(msg.ranges)
                if (not len(ranges) or not np.isfinite([msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max]).all()
                        or msg.range_min < 0 or msg.range_min >= msg.range_max or msg.angle_increment == 0):
                    raise ValueError('Invalid scan geometry')
                valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
                if not (valid | np.isposinf(ranges)).any():
                    raise ValueError('No valid scan rays')
                angles = msg.angle_min+np.flatnonzero(valid)*msg.angle_increment
                points = np.column_stack((ranges[valid]*np.cos(angles), ranges[valid]*np.sin(angles), np.zeros(valid.sum())))
            self.pending[name] = (msg.header, received, points)
        except (ValueError, TypeError):
            self.pending.pop(name, None)
            self.observations.pop(name, None)
            self.watchdog.sources.pop(name, None)

    def observations_in_base(self, now: float) -> tuple[np.ndarray, float]:
        # Nonblocking TF retry. Retain the previous exactly transformed sample
        # only within the unchanged freshness bound; never use latest camera TF
        # for an older depth image while the pan is moving.
        for name, (header, received, points) in list(self.pending.items()):
            try:
                tf = self.buffer.lookup_transform(self.p['odom_frame'], header.frame_id, Time.from_msg(header.stamp))
                self.observations[name] = transform(points, tf.transform)
                self.watchdog.sources[name] = (stamp(header.stamp), received)
                del self.pending[name]
            except TransformException:
                pass
        pose_msg = self.buffer.lookup_transform(self.p['odom_frame'], self.p['base_frame'], Time())
        pose_age = now-stamp(pose_msg.header.stamp)
        if not 0 <= pose_age <= self.p['source_timeout_s']:
            raise ValueError('STALE_BASE_TF')
        pose = pose_matrix(pose_msg.transform)
        if np.linalg.norm(pose[:2, 2]) > .05:
            raise ValueError('NON_PLANAR_BASE')
        map_odom = pose_matrix(self.buffer.lookup_transform(self.p['map_frame'], self.p['odom_frame'], Time()).transform)
        if localization_jumped(self.previous_map_odom, map_odom, self.p['localization_jump_m'], self.p['localization_jump_rad']):
            self.watchdog.fault = 'LOCALIZATION_JUMP_RESTART_REQUIRED'
        self.previous_map_odom = map_odom
        points = np.concatenate([self.observations.get(k, np.empty((0, 3))) for k in ('scan', 'depth')])
        points = (points-pose[:3, 3])@pose[:3, :3]
        points = points[(points[:, 2] >= self.p['min_height']) & (points[:, 2] <= self.p['max_height'])]
        return points[:, :2], pose_age

    def update(self) -> None:
        start = time.monotonic()
        now = self.get_clock().now().nanoseconds/1e9
        output = np.zeros(3)
        requested = np.zeros(3)
        measured = np.zeros(3)
        reason, source_age = 'MISSING_INPUT', None
        try:
            # Clock faults must latch even while TF or sensors are unavailable.
            self.watchdog.tick(now, start)
            points, pose_age = self.observations_in_base(now)
            required = ('scan', 'depth', 'odom')+ (('joint',) if self.p['require_pan_feedback'] else ())
            source_age = max(pose_age, self.watchdog.check(now, start, self.command_wall, self.raw_wall, required))
            if self.command is None or self.raw is None or self.odom is None:
                raise ValueError('MISSING_COMMAND_OR_ODOM')
            if self.odom.child_frame_id != self.p['base_frame']:
                raise ValueError('INVALID_ODOM_FRAME')
            requested, raw, measured = twist_vector(self.command), twist_vector(self.raw), twist_vector(self.odom.twist.twist)
            # A raw stop cannot be resurrected by a delayed monitor sample.
            if not np.any(raw):
                requested = np.zeros(3)
            output, reason = self.guard.evaluate(points, requested, measured, source_age)
            if not np.any(requested) and reason == 'PASS':
                reason = 'STOP_COMMAND'
            if time.monotonic()-start > self.p['processing_budget_s']:
                raise ValueError('PROCESSING_TIMEOUT')
        except (ValueError, TransformException) as exc:
            output, reason = np.zeros(3), str(exc)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = map(float, output)
        self.output.publish(out)
        self.state.publish(String(data=json.dumps(dict(mode='direct_sensor_planar_guard', sim_s=now,
            decision=reason, requested=requested.tolist(), output=output.tolist(),
            measured=measured.tolist(),
            source_age_s=source_age, processing_s=time.monotonic()-start,
            observation_scope='observed_surfaces_only; blind space is not certified free'))))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ReactiveGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
