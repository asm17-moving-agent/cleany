"""Read-only ROS telemetry recorder. Never opens a motor port or sends goals."""
import json
import math

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data


def telemetry_record(message: DynamicJointState, receipt_ns: int) -> dict:
    joints = {}
    for name, row in zip(message.joint_names, message.interface_values, strict=True):
        if not name.startswith(('left_', 'right_')):
            continue
        joints[name] = {
            key: float(value) if math.isfinite(value) else None
            for key, value in zip(row.interface_names, row.values, strict=True)
        }
    return {
        'stage': 'motor_telemetry',
        'state_stamp_ns': message.header.stamp.sec*10**9+message.header.stamp.nanosec,
        'receipt_stamp_ns': receipt_ns,
        'joints': joints,
    }


class MotorTelemetryRecorder(Node):
    def __init__(self):
        super().__init__('motor_telemetry_recorder')
        self.subscription = self.create_subscription(
            DynamicJointState, '/joint_state_broadcaster/dynamic_joint_states',
            self.record, qos_profile_sensor_data)

    def record(self, message: DynamicJointState) -> None:
        # Runs outside the motor control thread. No logging/serial reads are
        # added to the real-time driver, and no NaN JSON is emitted.
        print(json.dumps(telemetry_record(message, self.get_clock().now().nanoseconds),
                         allow_nan=False, separators=(',', ':')), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTelemetryRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
