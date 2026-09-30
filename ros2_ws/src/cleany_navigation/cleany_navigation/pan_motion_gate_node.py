"""Velocity interlock using backend feedback; this node does not own TF."""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float64, String

from cleany_navigation.pan_motion_gate import PanGate
from cleany_navigation.point_cloud import cloud_xyz


def stamp(msg) -> float:
    return (msg.sec * 1_000_000_000 + msg.nanosec) / 1e9


def velocity(msg) -> tuple[float, float, float]:
    return msg.linear.x, msg.linear.y, msg.angular.z


class PanMotionGate(Node):
    def __init__(self) -> None:
        super().__init__('pan_motion_gate')
        defaults = dict(allow_reverse=False, angle_tolerance=.04, settle_s=.2,
                        source_timeout_s=.5, command_timeout_s=.5,
                        linear_stopped=.01, angular_stopped=.02,
                        pan_stopped=.05, update_period_s=.05,
                        command_topic='nav2/cmd_vel', output_topic='pan_ready/cmd_vel',
                        pan_command_topic='head_pan/command', state_topic='head_pan/state',
                        joint_states_topic='joint_states', odom_topic='odom',
                        depth_topic='camera/head/depth/points', pan_joint='head_pan_joint',
                        base_frame='base_link', depth_frame='head_camera_depth_frame')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('angle_tolerance', 'settle_s', 'source_timeout_s', 'command_timeout_s',
                    'linear_stopped', 'angular_stopped', 'pan_stopped', 'update_period_s'):
            if not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f'Invalid {key}')
        if p['command_topic'] == p['output_topic']:
            raise ValueError('Distinct input/output command topics required')
        self.gate = PanGate(**{key: p[key] for key in ('allow_reverse', 'angle_tolerance',
            'settle_s', 'source_timeout_s', 'linear_stopped', 'angular_stopped', 'pan_stopped')})
        self.command = self.measured = (0., 0., 0.)
        self.command_wall = self.joint_wall = self.odom_wall = self.depth_wall = -math.inf
        self.clock_stamp = None
        self.clock_advanced_wall = time.monotonic()
        self.joint_stamp = self.odom_stamp = self.depth_stamp = -math.inf
        self.pan = self.pan_speed = 0.
        self.output = self.create_publisher(Twist, p['output_topic'], 10)
        self.pan_pub = self.create_publisher(Float64, p['pan_command_topic'], 10)
        self.state = self.create_publisher(String, p['state_topic'], 10)
        self.create_subscription(Twist, p['command_topic'], self.on_command, 10)
        self.create_subscription(JointState, p['joint_states_topic'], self.on_joint, qos_profile_sensor_data)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, p['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_timer(p['update_period_s'], self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg: Twist) -> None:
        self.command = velocity(msg)
        if any(abs(v) > 1e-8 or not math.isfinite(v) for v in
               (msg.linear.z, msg.angular.x, msg.angular.y)):
            self.command = (math.nan, 0., 0.)
        self.command_wall = time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        if msg.child_frame_id != self.p['base_frame']:
            self.odom_stamp = -math.inf
            return
        self.measured = velocity(msg.twist.twist)
        self.odom_stamp, self.odom_wall = stamp(msg.header.stamp), time.monotonic()

    def on_depth(self, msg: PointCloud2) -> None:
        try:
            points = cloud_xyz(msg)
            if msg.header.frame_id != self.p['depth_frame'] or not len(points):
                raise ValueError('Missing depth geometry or wrong frame')
            if not all(math.isfinite(v) for v in points.ravel()):
                raise ValueError('Invalid depth points')
        except ValueError:
            self.depth_stamp = -math.inf
            return
        self.depth_stamp, self.depth_wall = stamp(msg.header.stamp), time.monotonic()

    def on_joint(self, msg: JointState) -> None:
        if self.p['pan_joint'] not in msg.name:
            return
        index = msg.name.index(self.p['pan_joint'])
        if index >= len(msg.position) or index >= len(msg.velocity):
            self.joint_stamp = -math.inf
            return
        self.pan, self.pan_speed = msg.position[index], msg.velocity[index]
        if not all(math.isfinite(v) for v in (self.pan, self.pan_speed)):
            self.joint_stamp = -math.inf
            return
        self.joint_stamp, self.joint_wall = stamp(msg.header.stamp), time.monotonic()

    def update(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        if self.clock_stamp != now:
            self.clock_stamp, self.clock_advanced_wall = now, wall
        target, command, reason = self.gate.step(now, self.command, self.measured,
            self.pan, self.pan_speed, self.joint_stamp, self.odom_stamp, self.depth_stamp)
        if wall-self.command_wall > self.p['command_timeout_s'] or any(
            wall-t > self.p['source_timeout_s'] for t in (self.joint_wall, self.odom_wall, self.depth_wall)):
            command, reason = (0., 0., 0.), 'WALL_TIMEOUT'
        if wall-self.clock_advanced_wall > self.p['command_timeout_s']:
            command, reason = (0., 0., 0.), 'CLOCK_TIMEOUT'
        if reason in ('WALL_TIMEOUT', 'CLOCK_TIMEOUT'):
            self.gate.aligned_since = None
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = command
        self.output.publish(out)
        self.pan_pub.publish(Float64(data=target))
        self.state.publish(String(data=json.dumps(dict(stamp_s=now, sim_s=now,
            target=target, pan=self.pan, reason=reason, requested=self.command,
            output=command, depth_stamp=self.depth_stamp))))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PanMotionGate()
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
