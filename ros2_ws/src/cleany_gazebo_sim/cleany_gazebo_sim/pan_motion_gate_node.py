"""Measured Gazebo pan TF plus a velocity interlock upstream of safety guards."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster
from cleany_gazebo_sim.pan_motion_gate import PanGate
from cleany_gazebo_sim.world.cad_frame import rotation
from cleany_gazebo_sim.body_guard_observer import quaternion


def stamp(msg):
    return msg.sec+msg.nanosec*1e-9


def velocity(msg):
    return msg.linear.x, msg.linear.y, msg.angular.z


class PanMotionGate(Node):
    def __init__(self):
        super().__init__('pan_motion_gate')
        defaults = dict(translation=[0., 0., 0.], rotation_rpy=[0., 0., 0.],
                        pan_pivot=[0., 0., 0.], allow_reverse=False,
                        angle_tolerance=.04, settle_s=.2, source_timeout_s=.5,
                        command_timeout_s=.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        self.gate = PanGate(angle_tolerance=p['angle_tolerance'], settle_s=p['settle_s'],
                            source_timeout_s=p['source_timeout_s'], allow_reverse=p['allow_reverse'])
        self.timeout = p['command_timeout_s']
        self.camera = np.array(p['translation']); self.pivot = np.array(p['pan_pivot'])
        self.rotation = rotation(p['rotation_rpy'])
        self.command = (0., 0., 0.); self.measured = (0., 0., 0.)
        self.command_wall = self.joint_wall = self.odom_wall = self.depth_wall = -1e9
        self.joint_stamp = self.odom_stamp = self.depth_stamp = -1e9
        self.pan = self.pan_speed = 0.
        self.tf = TransformBroadcaster(self)
        self.output = self.create_publisher(Twist, '/pan_ready/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/head_pan/command', 10)
        self.state = self.create_publisher(String, '/head_pan/state', 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self.on_command, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/head/depth/points', self.on_depth, qos_profile_sensor_data)
        self.create_timer(.05, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command = velocity(msg); self.command_wall = time.monotonic()

    def on_odom(self, msg):
        self.measured = velocity(msg.twist.twist)
        self.odom_stamp = stamp(msg.header.stamp); self.odom_wall = time.monotonic()

    def on_depth(self, msg):
        if msg.width*msg.height and msg.data:
            self.depth_stamp = stamp(msg.header.stamp); self.depth_wall = time.monotonic()

    def on_joint(self, msg):
        if 'head_pan_joint' not in msg.name:
            return
        i = msg.name.index('head_pan_joint')
        if i >= len(msg.position) or i >= len(msg.velocity):
            return
        self.pan, self.pan_speed = msg.position[i], msg.velocity[i]
        if not np.isfinite([self.pan, self.pan_speed]).all():
            return
        self.joint_stamp = stamp(msg.header.stamp); self.joint_wall = time.monotonic()
        turn = rotation([0., 0., self.pan])
        xyz = self.pivot+turn@(self.camera-self.pivot)
        q = quaternion(turn@self.rotation)
        tf = TransformStamped(); tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'base_link'; tf.child_frame_id = 'head_camera_depth_frame'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        self.tf.sendTransform(tf)

    def update(self):
        now = self.get_clock().now().nanoseconds/1e9
        wall = time.monotonic()
        target, command, reason = self.gate.step(now, self.command, self.measured,
            self.pan, self.pan_speed, self.joint_stamp, self.odom_stamp, self.depth_stamp)
        if wall-self.command_wall > self.timeout or any(wall-t > 1. for t in (self.joint_wall, self.odom_wall, self.depth_wall)):
            command = (0., 0., 0.); reason = 'WALL_TIMEOUT'
            self.gate.aligned_since = None
        out = Twist(); out.linear.x, out.linear.y, out.angular.z = command
        self.output.publish(out)
        self.pan_pub.publish(Float64(data=target))
        self.state.publish(String(data=json.dumps(dict(sim_s=now, target=target,
            pan=self.pan, reason=reason, requested=self.command, output=command,
            depth_stamp=self.depth_stamp))))


def main(args=None):
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
