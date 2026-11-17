"""Publish CAD preview coordinates without opening a motor device."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from .cad_coordinates import CadCoordinates


def initial_positions(description: str) -> dict[str, float]:
    """Choose an explicitly nominal display pose inside the actuator limits."""
    positions = {}
    for joint in ET.fromstring(description).findall('joint'):
        if joint.get('type') != 'revolute' or joint.find('mimic') is not None:
            continue
        limit = joint.find('limit')
        lower, upper = (float(limit.get(k)) for k in ('lower', 'upper'))
        positions[joint.get('name')] = upper if joint.get('name').endswith('_gripper_joint') else (lower+upper)/2
    return positions


def resolve_positions(description: str, measured: dict[str, float]) -> dict[str, float]:
    required = set(initial_positions(description))
    if not required <= measured.keys() or not all(math.isfinite(measured[n]) for n in required):
        raise ValueError('A finite sample of all fourteen actuator positions is required')
    cad = CadCoordinates.from_urdf(description)
    if cad is None:
        raise ValueError('The runtime preview requires the calibrated CAD model')
    return cad.expanded({name: float(measured[name]) for name in required})


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class RuntimeModelState(Node):
        """Resolve preview finger states from subscribed actuator feedback.

        Subscriptions:
            /joint_states (sensor_msgs/msg/JointState): Measured actuator states;
                input_topic default, sensor-data QoS. No device access.
        Publishers:
            joint_states (sensor_msgs/msg/JointState): Namespace-local display
                state including passive CAD fingers; never actuator commands.
            status (std_msgs/msg/String): Nominal, measured or stale pose source.
        """

        def __init__(self):
            super().__init__('runtime_model_state')
            self.declare_parameter('model_file', '')
            self.declare_parameter('input_topic', '/joint_states')
            if self.resolve_topic_name('joint_states') == self.resolve_topic_name(self.get_parameter('input_topic').value):
                raise ValueError('Preview output must be separate from measured joint_states')
            self.description = Path(self.get_parameter('model_file').value).read_text()
            self.positions = initial_positions(self.description)
            self.received = None
            self.publisher = self.create_publisher(JointState, 'joint_states', 10)
            self.status = self.create_publisher(String, 'status', 10)
            self.subscription = self.create_subscription(JointState,
                self.get_parameter('input_topic').value, self.observe, qos_profile_sensor_data)
            self.timer = self.create_timer(.1, self.publish)

        def observe(self, message):
            if len(message.name) != len(message.position):
                return
            sample = dict(zip(message.name, message.position))
            try:
                resolved = resolve_positions(self.description, sample)
            except ValueError:
                return
            self.positions = {name: resolved[name] for name in self.positions}
            self.received = time.monotonic()

        def publish(self):
            values = resolve_positions(self.description, self.positions)
            message = JointState(name=list(values), position=list(values.values()))
            message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)
            mode = 'nominal_preview' if self.received is None else (
                'measured_feedback' if time.monotonic()-self.received < 1. else 'stale_feedback')
            self.status.publish(String(data=json.dumps({'source': mode, 'motor_commands_sent': 0})))

    rclpy.init()
    node = RuntimeModelState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
