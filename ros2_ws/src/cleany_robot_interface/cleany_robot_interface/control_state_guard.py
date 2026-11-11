"""Publish complete position feedback and deactivate controllers on stale/invalid state."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


EXPECTED_JOINTS = {f'{side}_{suffix}_joint' for side in ('left', 'right') for suffix in
                   ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')}
EXPECTED_JOINTS.update(('head_pan_joint', 'head_tilt_joint'))
CONTROLLERS = [f'{side}_{part}_controller' for side in ('left', 'right') for part in ('arm', 'gripper')]


def valid_positions(message: JointState, now_ns: int, maximum_age: float) -> bool:
    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    return (len(message.name) == len(message.position) == 14
            and set(message.name) == EXPECTED_JOINTS
            and all(math.isfinite(value) for value in message.position)
            and 0 <= now_ns - stamp <= maximum_age * 1e9)


class ControlStateGuard(Node):
    def __init__(self):
        super().__init__('real_control_state_guard')
        self.declare_parameter('max_state_age_s', .25)
        self.declare_parameter('startup_timeout_s', 30.0)
        self.declare_parameter('controller_names', CONTROLLERS)
        self.controllers = list(self.get_parameter('controller_names').value)
        if not self.controllers or not set(self.controllers) <= set(CONTROLLERS):
            raise ValueError('Invalid controller_names')
        self.max_age = float(self.get_parameter('max_state_age_s').value)
        self.startup = float(self.get_parameter('startup_timeout_s').value)
        if not 0 < self.max_age <= 1 or not 1 <= self.startup <= 60:
            raise ValueError('Invalid state timeout')
        self.created = time.monotonic()
        self.last_received = None
        self.last_stamp = -1
        self.fault = ''
        self.stop_confirmed = False
        self.request = None
        self.cancel_requests = {}
        self.actions_cancelled = False
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.status = self.create_publisher(String, 'real_control/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.create_subscription(JointState,
            'joint_state_broadcaster/joint_states', self.receive, 10)
        self.switch = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.cancel = {name: self.create_client(
            CancelGoal, f'/{name}/follow_joint_trajectory/_action/cancel_goal')
            for name in self.controllers}
        self.stop_service = self.create_service(Trigger, 'real_control/stop', self.stop)
        self.timer = self.create_timer(.05, self.tick)

    def receive(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.fault:
            return
        if not valid_positions(message, self.get_clock().now().nanoseconds, self.max_age):
            self.latch('Invalid/incomplete/stale hardware position feedback')
            return
        if stamp <= self.last_stamp:
            return  # A repeated stamp does not refresh the watchdog.
        self.last_stamp = stamp
        self.last_received = time.monotonic()
        result = JointState()
        result.header = message.header
        result.name, result.position = message.name, message.position
        # Humble broadcaster fills unsupported velocity/effort with NaN; omit them.
        self.publisher.publish(result)

    def latch(self, reason):
        if not self.fault:
            self.fault = reason
            self.get_logger().error(reason + '; controller stop requested; restart required')

    def stop(self, _request, response):
        self.latch('Operator software stop')
        response.success = True
        response.message = 'Stop latched; awaiting controller deactivation. See real_control/status.'
        return response

    def tick(self):
        now = time.monotonic()
        if self.last_received is None:
            if now - self.created > self.startup:
                self.latch('Hardware feedback startup timeout')
        elif now - self.last_received > self.max_age:
            self.latch('Hardware feedback timeout')
        if self.fault:
            for name, client in self.cancel.items():
                future = self.cancel_requests.get(name)
                if future is None and client.service_is_ready():
                    # A zero GoalInfo is the ROS action protocol request to
                    # cancel every active goal on this action server.
                    self.cancel_requests[name] = client.call_async(CancelGoal.Request())
            if self.cancel_requests and all(future.done() for future in self.cancel_requests.values()):
                self.actions_cancelled = all(
                    future.exception() is None and
                    future.result().return_code != CancelGoal.Response.ERROR_REJECTED
                    for future in self.cancel_requests.values())
        if self.fault and not self.stop_confirmed:
            if self.request is not None and self.request.done():
                try:
                    self.stop_confirmed = bool(self.request.result().ok)
                except Exception as error:
                    self.get_logger().error(str(error))
                self.request = None
            if not self.stop_confirmed and self.request is None and self.switch.service_is_ready():
                request = SwitchController.Request()
                request.deactivate_controllers = self.controllers
                request.strictness = SwitchController.Request.BEST_EFFORT
                request.timeout.sec = 1
                self.request = self.switch.call_async(request)
        message = String()
        message.data = json.dumps({'state': 'fault' if self.fault else
                                  ('ready' if self.last_received is not None else 'waiting'),
                                  'reason': self.fault, 'controllers_stopped': self.stop_confirmed,
                                  'actions_cancelled': self.actions_cancelled,
                                  'physical_stop_verified': False})
        self.status.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControlStateGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
