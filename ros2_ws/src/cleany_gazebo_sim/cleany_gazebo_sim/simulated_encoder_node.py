from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from cleany_gazebo_sim.simulated_encoder import SimulatedQuadratureEncoder


class SimulatedEncoderNode(Node):
    """Publish synthetic quadrature-encoder joint states for Gazebo wheels."""

    def __init__(self) -> None:
        super().__init__('simulated_encoder')
        self.declare_parameter('input_topic', 'joint_states')
        self.declare_parameter(
            'output_topic', 'wheel_encoder/joint_states'
        )
        self.declare_parameter('ticks_per_revolution', 3172)
        self.declare_parameter('tick_noise_stddev', 0.0)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter(
            'front_left_joint', 'front_left_wheel_joint'
        )
        self.declare_parameter(
            'front_right_joint', 'front_right_wheel_joint'
        )
        self.declare_parameter('rear_left_joint', 'rear_left_wheel_joint')
        self.declare_parameter('rear_right_joint', 'rear_right_wheel_joint')
        self.declare_parameter('front_left_scale', 1.0)
        self.declare_parameter('front_right_scale', 1.0)
        self.declare_parameter('rear_left_scale', 1.0)
        self.declare_parameter('rear_right_scale', 1.0)

        self._joint_names = (
            str(self.get_parameter('front_left_joint').value),
            str(self.get_parameter('front_right_joint').value),
            str(self.get_parameter('rear_left_joint').value),
            str(self.get_parameter('rear_right_joint').value),
        )
        self._encoder = SimulatedQuadratureEncoder(
            ticks_per_revolution=int(
                self.get_parameter('ticks_per_revolution').value
            ),
            wheel_scales=(
                float(self.get_parameter('front_left_scale').value),
                float(self.get_parameter('front_right_scale').value),
                float(self.get_parameter('rear_left_scale').value),
                float(self.get_parameter('rear_right_scale').value),
            ),
            tick_noise_stddev=float(
                self.get_parameter('tick_noise_stddev').value
            ),
            random_seed=int(self.get_parameter('random_seed').value),
        )
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(
            JointState, output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.position) < len(message.name):
            self.get_logger().warning(
                'Ignoring JointState with fewer positions than names',
                throttle_duration_sec=5.0,
            )
            return

        position_by_name = dict(zip(message.name, message.position))
        missing = [
            joint_name
            for joint_name in self._joint_names
            if joint_name not in position_by_name
        ]
        if missing:
            self.get_logger().warning(
                f'Ignoring JointState missing wheel joints: {missing}',
                throttle_duration_sec=5.0,
            )
            return

        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        reading = self._encoder.update(
            tuple(
                float(position_by_name[joint_name])
                for joint_name in self._joint_names
            ),
            stamp_s,
        )

        output = JointState()
        output.header.stamp = stamp
        output.name = list(self._joint_names)
        output.position = list(reading.positions_rad)
        output.velocity = list(reading.velocities_rad_s)
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedEncoderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
