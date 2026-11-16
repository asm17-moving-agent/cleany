import os

import pytest

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter

from cleany_base_odometry.wheel_odom_node import WheelOdometryNode


@pytest.mark.parametrize(
    'value',
    [[0.0] * 5, [0.0] * 7, [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
     [0.0, 0.0, float('nan'), 0.0, 0.0, 0.0],
     [0.0, 0.0, float('inf'), 0.0, 0.0, 0.0]],
)
def test_invalid_covariance_rejected(value):
    import rclpy

    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError, match='twist_covariance_diagonal'):
            WheelOdometryNode(parameter_overrides=[
                Parameter('twist_covariance_diagonal', value=value)
            ])
    finally:
        rclpy.shutdown()


def test_covariance_diagonal_is_published():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    rclpy.init(args=[])
    node = WheelOdometryNode(parameter_overrides=[
        Parameter('twist_covariance_diagonal',
                  value=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
        Parameter('velocity_window_s', value=0.0),
    ])
    monitor = rclpy.create_node('wheel_covariance_test')
    received = []
    monitor.create_subscription(Odometry, 'wheel/odom', received.append, 10)
    publisher = monitor.create_publisher(JointState, 'joint_states', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(monitor)
    try:
        import time
        deadline = time.monotonic() + 3
        message = JointState()
        message.name = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
        ]
        message.position = [0.0] * 4
        while time.monotonic() < deadline and not received:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
        assert received
        covariance = received[0].twist.covariance
        assert [covariance[index * 7] for index in range(6)] == [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0
        ]
        assert all(
            covariance[index] == 0.0
            for index in range(36) if index not in {0, 7, 14, 21, 28, 35}
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown()
