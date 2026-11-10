"""Publish both arms and head through exclusive READ-only serial ownership."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from cleany_robot_interface.core import mappings
from cleany_robot_interface.reader import ReadOnlyBus, verify_arm_calibration
from cleany_robot_interface.state_profile import head_positions


class RobotStateReader(Node):
    def __init__(self):
        super().__init__('read_only_robot_state')
        self.buses = []
        for name in ('arm_config_file', 'head_config_file', 'head_mapping_file'):
            self.declare_parameter(name, '')
        self.declare_parameter('rate_hz', 20.0)
        read = lambda name: yaml.safe_load(Path(self.get_parameter(name).value).read_text())
        arm, head = read('arm_config_file'), read('head_config_file')
        self.head_mapping = read('head_mapping_file')
        head_positions([0, 0], self.head_mapping)
        if head['motor_ids'] != {'pan': 13, 'tilt': 14}:
            raise ValueError('Expected head pan/tilt motor IDs 13/14')
        shared_bus = Path(arm['port']).resolve() == Path(head['serial_port']).resolve()
        if shared_bus and int(arm['baudrate']) != int(head['baudrate']):
            raise ValueError('Shared arm/head bus requires the same baudrate')
        directory = Path(arm['calibration_dir']).expanduser()
        calibrations = {side: json.loads((directory / cfg['calibration']).read_text())
                        for side, cfg in arm['arms'].items()}
        self.joints = mappings(arm, calibrations)
        rate = float(self.get_parameter('rate_hz').value)
        if not 0 < rate <= 50:
            raise ValueError('rate_hz must be in (0, 50]')
        try:
            self.arm = ReadOnlyBus(arm['port'], int(arm['baudrate']))
            self.buses.append(self.arm)
            verify_arm_calibration(self.arm, self.joints)
            if shared_bus:
                self.head = self.arm
            else:
                self.head = ReadOnlyBus(head['serial_port'], int(head['baudrate']))
                self.buses.append(self.head)
            for motor in (13, 14):
                if self.head.read(motor, 3, 2) != 777:
                    raise ValueError(f'Head ID {motor}: expected STS3215')
            self.publisher = self.create_publisher(JointState, '/joint_states', 10)
            self.last_warning = 0.0
            self.timer = self.create_timer(1.0 / rate, self.poll)
            self.get_logger().warning(
                'READ ONLY: 14 live joint positions for provisional planning. '
                'Absolute calibration and operating limits remain unverified. No motor writes.')
        except Exception:
            self.close_buses()
            raise

    def poll(self):
        # Motors are sampled sequentially; this timestamp is the start of the read.
        stamp = self.get_clock().now().to_msg()
        try:
            positions = [joint.radians(self.arm.read(joint.motor_id, 56, 2))
                         for joint in self.joints]
            positions += head_positions([self.head.read(i, 56, 2) for i in (13, 14)],
                                        self.head_mapping)
        except (RuntimeError, ValueError, OSError) as error:
            if time.monotonic() - self.last_warning > 2:
                self.get_logger().error(f'No complete fresh state published: {error}')
                self.last_warning = time.monotonic()
            return
        message = JointState()
        message.header.stamp = stamp
        message.name = [joint.name for joint in self.joints] + ['head_pan_joint', 'head_tilt_joint']
        message.position = positions
        # Velocity/effort are absent: no unverified values are manufactured.
        self.publisher.publish(message)

    def close_buses(self):
        for bus in reversed(self.buses):
            bus.close()
        self.buses.clear()

    def destroy_node(self):
        self.close_buses()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotStateReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
