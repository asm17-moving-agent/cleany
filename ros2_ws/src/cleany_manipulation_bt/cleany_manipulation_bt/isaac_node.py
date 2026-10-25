"""Isaac profile of the existing sensor/MoveIt BT, with measured drive readiness."""
import json
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .drive_readiness import DriveGate
from .node import MujocoManipulationNode


class IsaacManipulationNode(MujocoManipulationNode):
    """Reuse sorting actions and expose a read-only base-motion gate.

    Subscriptions:
        /joint_states (sensor_msgs/msg/JointState): Measured articulated state.
    Services:
        manipulation/ready_to_drive (std_srvs/srv/Trigger): Fresh stowed joints,
            empty payload state, terminal actions and a non-inhibited BT.
    Notes:
        stowed_positions_json must describe the installed navigation posture.
        The gate never commands joints or resets an execution fault.
    """
    def __init__(self):
        super().__init__(execution_profile='isaac')
        self.declare_parameter('stowed_positions_json', '')
        self.drive_gate = DriveGate(json.loads(self.get_parameter('stowed_positions_json').value))
        self.create_subscription(JointState, '/joint_states', self._drive_joints, qos_profile_sensor_data)
        self.create_service(Trigger, 'manipulation/ready_to_drive', self._drive_ready)

    def _drive_joints(self, message):
        self.drive_gate.update(message.name, message.position, message.velocity,
            message.header.stamp.sec*10**9 + message.header.stamp.nanosec, time.monotonic())

    def _drive_ready(self, request, response):
        with self.core.lock:
            record = self.core.record
            terminal = record is None or record.result is not None
            empty = record is None or record.result is not None and record.result.object_state in (
                'NOT_TOUCHED', 'LEFT_GRIPPER')
            response.success = bool(terminal and empty and not self.core.inhibited and self.backend.ready()
                and self.drive_gate.ready(time.monotonic(), self.get_clock().now().nanoseconds))
        response.message = '' if response.success else 'Arm, payload or execution state is unsafe or unobserved'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = IsaacManipulationNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
