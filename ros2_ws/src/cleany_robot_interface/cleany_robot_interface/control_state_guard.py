"""Publish complete position feedback and deactivate controllers on stale/invalid state."""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


EXPECTED_JOINTS = {f'{side}_{suffix}_joint' for side in ('left', 'right') for suffix in
                   ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')}
EXPECTED_JOINTS.update(('head_pan_joint', 'head_tilt_joint'))
CONTROLLERS = [f'{side}_{part}_controller' for side in ('left', 'right') for part in ('arm', 'gripper')]
DEFAULT_MAX_STATE_AGE_S = .25


def decode_feedback(message: DynamicJointState, expected: set[str],
                    require_driver_fault: bool) -> tuple[dict[str, float], int]:
    """Read positions and the common driver timestamp from one snapshot."""
    names = message.joint_names
    if (len(names) != len(message.interface_values) or len(set(names)) != len(names)
            or set(names) != expected):
        raise ValueError('Incomplete or duplicate hardware feedback joints')
    positions, stamps = {}, set()
    for name, row in zip(names, message.interface_values):
        if (len(row.interface_names) != len(row.values)
                or len(set(row.interface_names)) != len(row.values)):
            raise ValueError('Malformed hardware feedback interfaces')
        values = dict(zip(row.interface_names, row.values))
        position = values['position']
        if not math.isfinite(position):
            raise ValueError('Invalid hardware position')
        positions[name] = position
        if name.startswith('head_'):
            continue
        fault = values['hardware_fault'] if require_driver_fault else values.get('hardware_fault', 0.)
        if not math.isfinite(fault) or fault != 0:
            raise ValueError('Hardware driver latched a fault')
        stamp = values['feedback_stamp_ns']
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('Invalid hardware feedback timestamp')
        stamps.add(int(stamp))
    if len(stamps) != 1:
        raise ValueError('Inconsistent hardware feedback timestamps')
    return positions, stamps.pop()


class ControlStateGuard(Node):
    """Forward verified actuator state and latch faults that require stopping.

    Subscriptions:
        joint_state_broadcaster/dynamic_joint_states (control_msgs/msg/DynamicJointState):
            Complete motor positions and driver feedback age; latest sample only.

    Publishers:
        joint_states (sensor_msgs/msg/JointState): Verified positions stamped at acquisition.
        real_control/status (std_msgs/msg/String): Ready or latched fault; transient local.

    Services:
        real_control/stop (std_srvs/srv/Trigger): Server for a latched software stop.
        controller_manager/switch_controller (controller_manager_msgs/srv/SwitchController):
            Client for deactivating command controllers on fault.
    """

    def __init__(self):
        super().__init__('real_control_state_guard')
        self.declare_parameter('max_state_age_s', DEFAULT_MAX_STATE_AGE_S)
        self.declare_parameter('startup_timeout_s', 30.0)
        self.declare_parameter('controller_names', CONTROLLERS)
        self.declare_parameter('cad_calibration_json', '')
        cad_json = self.get_parameter('cad_calibration_json').value
        self.cad_coordinates = None
        if cad_json:
            from cleany_description.cad_coordinates import CadCoordinates
            self.cad_coordinates = CadCoordinates(json.loads(cad_json))
        self.declare_parameter('expected_joint_names', sorted(EXPECTED_JOINTS))
        self._gripper_positions = None
        self._dependent_positions = {}
        self.declare_parameter('require_driver_fault', False)
        self.require_driver_fault = self.get_parameter('require_driver_fault').value
        self.expected_joint_names = set(self.get_parameter('expected_joint_names').value)
        head = {'head_pan_joint', 'head_tilt_joint'}
        allowed = [head | {f'{side}_{suffix}_joint' for side in sides for suffix in
                   ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')}
                   for sides in (('left',), ('right',), ('left', 'right'))]
        if self.expected_joint_names not in allowed:
            raise ValueError('Expected feedback must cover complete present arms and head')
        self.controllers = list(self.get_parameter('controller_names').value)
        if not self.controllers or not set(self.controllers) <= set(CONTROLLERS):
            raise ValueError('Invalid controller_names')
        self.max_age = float(self.get_parameter('max_state_age_s').value)
        self.startup = float(self.get_parameter('startup_timeout_s').value)
        if not 0 < self.max_age <= 1 or not 1 <= self.startup <= 60:
            raise ValueError('Invalid state timeout')
        self.created = time.monotonic()
        self.last_stamp = None
        self.fault = ''
        self.stop_confirmed = False
        self.request = None
        self.cancel_requests = {}
        self.actions_cancelled = False
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.status = self.create_publisher(String, 'real_control/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.hardware_subscription = self.create_subscription(DynamicJointState,
            'joint_state_broadcaster/dynamic_joint_states', self.receive_hardware,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.switch = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.cancel = {name: self.create_client(
            CancelGoal, f'/{name}/follow_joint_trajectory/_action/cancel_goal')
            for name in self.controllers}
        self.stop_service = self.create_service(Trigger, 'real_control/stop', self.stop)
        self.timer = self.create_timer(.05, self.tick)

    def receive_hardware(self, message: DynamicJointState) -> None:
        """Forward a fresh complete data frame using its hardware acquisition time."""
        if self.fault:
            return
        try:
            positions, stamp = decode_feedback(message, self.expected_joint_names, self.require_driver_fault)
            if self.last_stamp is not None and stamp < self.last_stamp:
                raise ValueError('Hardware feedback timestamp regressed')
        except (KeyError, ValueError, TypeError) as error:
            self.latch(f'Invalid hardware feedback: {error}')
            return
        if not self.feedback_is_fresh(stamp) or stamp == self.last_stamp:
            return
        if self.cad_coordinates is not None:
            try:
                self.expand_positions(positions)
            except (ValueError, KeyError) as error:
                self.latch(f'CAD pin closure failed: {error}')
                return
        if not self.feedback_is_fresh(stamp):
            return
        result = JointState(name=list(positions), position=list(positions.values()))
        result.header.frame_id = message.header.frame_id
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.last_stamp = stamp
        self.publisher.publish(result)

    def expand_positions(self, positions: dict[str, float]) -> None:
        # The linkage depends only on the two grippers, even while the arms move.
        grippers = positions['left_gripper_joint'], positions['right_gripper_joint']
        if grippers != self._gripper_positions:
            self._dependent_positions = self.cad_coordinates.dependent_positions(positions)
            self._gripper_positions = grippers
        positions.update(self._dependent_positions)

    def feedback_is_fresh(self, stamp: int) -> bool:
        age_ns = self.get_clock().now().nanoseconds - stamp
        if 0 <= age_ns <= self.max_age * 1e9:
            return True
        if not self.fault:
            self.get_logger().error(f'Last motor snapshot age {age_ns / 1e6:.1f} ms')
            self.latch('Hardware feedback timeout')
        return False

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
        if self.last_stamp is None:
            if now - self.created > self.startup:
                self.latch('Hardware feedback startup timeout')
        elif not self.fault:
            self.feedback_is_fresh(self.last_stamp)
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
                                  ('ready' if self.last_stamp is not None else 'waiting'),
                                  'reason': self.fault, 'controllers_stopped': self.stop_confirmed,
                                  'actions_cancelled': self.actions_cancelled,
                                  'physical_stop_verified': False})
        self.status.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControlStateGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
