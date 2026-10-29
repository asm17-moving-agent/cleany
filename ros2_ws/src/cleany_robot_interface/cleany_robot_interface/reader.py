"""Read-only SDK access: no Robot.connect/configure or motor write operations."""
import fcntl
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from cleany_robot_interface.core import mappings


class ReadOnlyBus:
    def __init__(self, port: str, baudrate: int):
        from scservo_sdk import PacketHandler, PortHandler
        self.port = PortHandler(port)
        self.packet = PacketHandler(0)
        self.exclusive = False
        try:
            if not self.port.setBaudRate(baudrate):
                raise RuntimeError('Cannot open motor port')
            fcntl.flock(self.port.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Prevent subsequent ordinary opens of this serial device.
            import termios
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            self.exclusive = True
        except Exception:
            self.close()
            raise

    def read(self, motor: int, address: int, size: int) -> int:
        method = self.packet.read1ByteTxRx if size == 1 else self.packet.read2ByteTxRx
        value, status, error = method(self.port, motor, address)
        if status != 0 or error != 0:
            raise RuntimeError(f'ID {motor}: read failed comm={status} error={error}')
        return value

    def close(self):
        serial = getattr(self.port, 'ser', None)
        if serial is not None and serial.is_open:
            from contextlib import suppress
            import termios
            if self.exclusive:
                with suppress(OSError):
                    fcntl.ioctl(serial.fileno(), termios.TIOCNXCL)
            self.exclusive = False
            self.port.closePort()


def verify_arm_calibration(bus, joints):
    for joint in joints:
        if bus.read(joint.motor_id, 3, 2) != joint.model:
            raise ValueError(f'{joint.name}: unexpected motor model')
        homing = bus.read(joint.motor_id, 31, 2)
        homing = -(homing & 0x7ff) if homing & 0x800 else homing
        actual = (homing, bus.read(joint.motor_id, 9, 2), bus.read(joint.motor_id, 11, 2))
        if actual != (joint.homing_offset, joint.range_min, joint.range_max):
            raise ValueError(f'{joint.name}: EEPROM calibration mismatch')


class ArmStateReader(Node):
    def __init__(self):
        super().__init__('arm_state_reader')
        self.bus = None
        self.declare_parameter('config_file', '')
        config = yaml.safe_load(Path(self.get_parameter('config_file').value).read_text())
        directory = Path(config['calibration_dir']).expanduser()
        calibrations = {
            side: json.loads((directory / arm['calibration']).read_text())
            for side, arm in config['arms'].items()
        }
        self.joints = mappings(config, calibrations)
        rate = float(config['rate_hz'])
        if not 0 < rate <= 50:
            raise ValueError('rate_hz must be in (0, 50]')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        try:
            self.bus = ReadOnlyBus(config['port'], int(config['baudrate']))
            verify_arm_calibration(self.bus, self.joints)
            self.get_logger().warning(
                'READ ONLY visualization: URDF zero/direction UNVERIFIED. '
                'No torque or goal writes. Do not use for motion planning.')
            self.last_warning = 0.0
            self.timer = self.create_timer(1.0 / rate, self.poll)
        except Exception:
            if self.bus:
                self.bus.close()
            raise

    def poll(self):
        stamp = self.get_clock().now().to_msg()
        try:
            positions = [j.radians(self.bus.read(j.motor_id, 56, 2)) for j in self.joints]
        except (RuntimeError, ValueError, OSError) as error:
            if time.monotonic() - self.last_warning > 2:
                self.get_logger().error(f'No fresh state published: {error}')
                self.last_warning = time.monotonic()
            return
        message = JointState()
        message.header.stamp = stamp
        message.name = [j.name for j in self.joints]
        message.position = positions
        self.publisher.publish(message)

    def destroy_node(self):
        if self.bus:
            self.bus.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmStateReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
