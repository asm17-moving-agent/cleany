"""Shared ROS frame, odometry and parameter validation."""
from dataclasses import fields
import math
import time
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cleany_interfaces.msg import TofFrame
from .core import Settings, observe
from .proximity import model_id


def settings(node):
    values = {}
    for f in fields(Settings):
        default = getattr(Settings(), f.name)
        node.declare_parameter(f.name, list(default) if isinstance(default, tuple) else default)
        value = node.get_parameter(f.name).value
        values[f.name] = tuple(value) if isinstance(default, tuple) else value
    # Old per-zone validity thresholds cannot silently change the new contract.
    if node.declare_parameter('min_valid', -1).value != -1 or node.declare_parameter('required_zones', [-1]).value != [-1]:
        raise ValueError('remove obsolete min_valid/required_zones parameters')
    return Settings(**values)


def steady_timer(node, period, callback):
    return node.create_timer(period, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


def velocity(message):
    values = (message.linear.x, message.linear.y, message.angular.z)
    if any(not math.isfinite(v) for v in (*values, message.linear.z, message.angular.x, message.angular.y)) or any(
            v != 0 for v in (message.linear.z, message.angular.x, message.angular.y)):
        return (math.nan, math.nan, math.nan)
    return values


def twist(values=(0., 0., 0.)):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = map(float, values)
    return msg


class Inputs:
    def __init__(self, node, config):
        self.node, self.s = node, config
        self.frame = self.odom = None
        self.frame_at = self.odom_at = -math.inf
        self.sequence = -1
        node.create_subscription(TofFrame, 'tof/frame', self.on_frame, QoSProfile(depth=1))
        node.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    def on_frame(self, msg):
        if not msg.sensor_ready:
            self.frame = None
            return
        # A restart must expire the old sample before a new sequence can be accepted.
        now = time.monotonic()
        if msg.sequence <= self.sequence and now-self.frame_at <= self.s.source_timeout_s:
            self.frame = None
            return
        self.sequence, self.frame_at, self.frame = msg.sequence, now, msg

    def on_odom(self, msg):
        self.odom, self.odom_at = msg, time.monotonic()

    def age(self, stamp):
        return self.node.get_clock().now().nanoseconds*1e-9-(stamp.sec+stamp.nanosec*1e-9)

    def snapshot(self):
        now = time.monotonic()
        if self.frame is None or self.odom is None or self.frame.header.frame_id != 'tof_link':
            raise ValueError('missing sensor or odometry')
        ages = (self.age(self.frame.header.stamp), self.age(self.odom.header.stamp),
                now-self.frame_at, now-self.odom_at)
        if any(not math.isfinite(a) or not 0 <= a <= self.s.source_timeout_s for a in ages):
            raise ValueError('stale or future sensor/odometry')
        measured = velocity(self.odom.twist.twist)
        if not all(math.isfinite(v) for v in measured):
            raise ValueError('invalid odometry')
        msg = self.frame
        observation = observe(msg.distance_m, msg.sigma_m, msg.target_status,
                              msg.target_count, msg.valid, self.s)
        return observation, max(ages[0], ages[2]), measured


class SupervisorView:
    """Interlock against fresh Supervisor state, never acquire or renew a lease."""
    def __init__(self, node):
        from cleany_interfaces.msg import SystemHeartbeat, SystemState
        from rclpy.qos import DurabilityPolicy
        self.node = node
        self.required = node.declare_parameter('require_supervision', True).value
        self.epoch = ''
        self.heartbeat_at = self.state_at = -math.inf
        self.state = None
        node.create_subscription(SystemHeartbeat, 'system/heartbeat', self.heartbeat, 1)
        node.create_subscription(SystemState, 'system/state', self.on_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def fresh_stamp(self, stamp):
        age = self.node.get_clock().now().nanoseconds*1e-9-(stamp.sec+stamp.nanosec*1e-9)
        return math.isfinite(age) and 0 <= age <= .3

    def heartbeat(self, msg):
        if not self.fresh_stamp(msg.header.stamp):
            return
        if msg.epoch != self.epoch:
            self.state = None
        self.epoch, self.heartbeat_at = msg.epoch, time.monotonic()

    def on_state(self, msg):
        if msg.epoch == self.epoch and self.fresh_stamp(msg.header.stamp):
            self.state, self.state_at = msg, time.monotonic()

    def operable(self):
        if not self.required:
            return True
        now = time.monotonic()
        return bool(self.epoch and self.state and now-self.heartbeat_at <= .3
                    and now-self.state_at <= .3 and self.fresh_stamp(self.state.header.stamp)
                    and self.state.operating_state in ('OPERATIONAL', 'DEGRADED'))

    def authorized(self, request):
        if not self.required:
            return True
        if not self.operable() or request.lease_epoch != self.epoch:
            return False
        elapsed = max(time.monotonic()-self.state_at,
                      self.node.get_clock().now().nanoseconds*1e-9-
                      (self.state.header.stamp.sec+self.state.header.stamp.nanosec*1e-9))
        return any(p.mission_id == request.mission_id and p.lease_id == request.lease_id
                   and p.version == request.lease_version and p.granted
                   and math.isfinite(p.valid_for_s) and p.valid_for_s > elapsed
                   and ('NAVIGATION' in p.capabilities or 'MOBILE_MANIPULATION' in p.capabilities)
                   for p in self.state.permissions)


class DepthProof:
    """Consume live RGB-D coverage evidence without accepting retained/stale truth."""
    def __init__(self, node, config):
        from std_msgs.msg import String
        self.node, self.s = node, config
        self.proof = None
        self.received = -math.inf
        node.create_subscription(String, 'tof/depth_coverage', self.on_report, 1)

    def on_report(self, msg):
        import json
        self.proof = None
        if len(msg.data) > 16384:
            return
        try:
            report = json.loads(msg.data)
            if not isinstance(report, dict) or report.get('model_id') != model_id(self.s):
                return
            stamp, age = report['stamp_s'], report['source_age_s']
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in (stamp, age)) or age < 0:
                return
            self.proof, self.received = report, time.monotonic()
        except (KeyError, TypeError, ValueError):
            return

    def snapshot(self):
        if self.proof is None or self.node.count_publishers('tof/depth_coverage') != 1:
            return None
        now = self.node.get_clock().now().nanoseconds*1e-9
        report = self.proof
        ages = (now-report['stamp_s'], now-report['stamp_s']+report['source_age_s'],
                time.monotonic()-self.received)
        if any(not 0 <= a <= self.s.source_timeout_s for a in ages):
            return None
        return report
