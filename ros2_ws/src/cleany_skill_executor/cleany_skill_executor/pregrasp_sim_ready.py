"""Wait for measured simulation inputs before starting the planning runtime."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener

from cleany_skill_executor.core.physical_grasp_selection import REQUIRED_JOINT_NAMES


class SimulationInputsReady(Node):
    def __init__(self):
        super().__init__('pregrasp_sim_inputs_ready')
        self.declare_parameter('startup_timeout_sec', 60.)
        self.joints, self.infos = None, {}
        self.last_reason = 'Waiting for simulation input'
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        for topic in ('/camera/color/camera_info', '/camera/depth/camera_info',
                      '/left_wrist_camera/camera_info', '/right_wrist_camera/camera_info'):
            self.create_subscription(CameraInfo, topic,
                lambda msg, key=topic: self.infos.__setitem__(key, msg), qos_profile_sensor_data)

    def on_joints(self, message):
        self.joints = message

    def inputs_ready(self):
        joints = self.joints
        if (joints is None or len(self.infos) != 4
                or len(joints.name) != len(joints.position)
                or not set(REQUIRED_JOINT_NAMES) <= set(joints.name)
                or not all(math.isfinite(q) for q in joints.position)):
            self.last_reason = 'Missing camera info or complete measured joints'
            return False
        now = self.get_clock().now().nanoseconds
        for message in (joints, *self.infos.values()):
            stamp = Time.from_msg(message.header.stamp)
            if stamp.nanoseconds <= 0 or not 0 <= now-stamp.nanoseconds <= 500_000_000:
                self.last_reason = f'Stale/future input: {message.header.frame_id}'
                return False
            if isinstance(message, CameraInfo) and not self.tf.can_transform(
                    'base_link', message.header.frame_id, stamp):
                self.last_reason = f'Missing capture-time TF: base_link <- {message.header.frame_id}'
                return False
        self.last_reason = 'Waiting for paired head camera info and active controllers'
        return (self.infos['/camera/color/camera_info'].header.stamp
                == self.infos['/camera/depth/camera_info'].header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationInputsReady()
    deadline = time.monotonic()+float(node.get_parameter('startup_timeout_sec').value)
    pending, ready = None, False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if not node.inputs_ready():
                continue
            if pending is None and node.controllers.service_is_ready():
                pending = node.controllers.call_async(ListControllers.Request())
            if pending is not None and pending.done():
                response = pending.result()
                active = {c.name for c in response.controller if c.state == 'active'} if response else set()
                if {f'{arm}_{part}_controller' for arm in ('left', 'right')
                        for part in ('arm', 'gripper')} <= active:
                    ready = True
                    break
                pending = None
        node.get_logger().info('Simulation inputs and controllers ready' if ready
                               else f'Simulation inputs timed out: {node.last_reason}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ready else 1)
