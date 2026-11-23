"""Relay fresh Joy input to the Jetson manual controller."""

import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml

from cleany_teleop.gamepad import GamepadControl, Mapping
from cleany_teleop.gamepad_relay import GamepadRelay


class GamepadTeleop(Node):
    """Send left-stick translation and right-stick yaw from /joy to the operator relay."""

    def __init__(self):
        super().__init__("gamepad_teleop")
        defaults = Mapping()
        base_share = Path(get_package_share_directory("cleany_base_driver"))
        config = self.declare_parameter(
            "base_config", str(base_share / "config" / "base_hardware.yaml")
        ).value
        self.control = GamepadControl(
            Mapping(
                **{
                    name: self.declare_parameter(name, getattr(defaults, name)).value
                    for name in vars(defaults)
                }
            ),
            yaml.safe_load(Path(config).read_text()),
        )
        self.get_logger().info(
            f"패드 병진 속도 한도 {self.control.max_translation_mps:g} m/s "
            f"(RT 요청 {self.control.mapping.fast_mps:g} m/s)"
        )
        directory = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
        self.path = self.declare_parameter(
            "socket_path", str(Path(directory) / "cleany-gamepad.sock")
        ).value
        self.relay = GamepadRelay(
            self.control, self.path, lambda msg: self.get_logger().warning(msg, throttle_duration_sec=2)
        )
        self.create_subscription(Joy, "joy", self.sample, 10)
        self.create_timer(0.02, self.watchdog)

    def sample(self, msg):
        if msg.header.frame_id != "leadjoy_xeno_plus":
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.relay.sample(msg.axes, msg.buttons, stamp, time.time() - stamp, time.monotonic())

    def watchdog(self):
        self.relay.tick(time.monotonic())


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.relay.close()
        node.destroy_node()
        rclpy.try_shutdown()
