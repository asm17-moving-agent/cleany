"""ROS message/odom/TF boundary test using synthetic counts on an isolated domain."""

from math import pi
from time import monotonic

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cleany_interfaces.msg')

from cleany_interfaces.msg import WheelEncoderTicks
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from cleany_base_odometry.hardware_odom_node import HardwareOdometryNode


def test_raw_counts_reach_odom_tf_and_reboot_does_not_move_pose():
    rclpy.init(args=[])
    hardware = HardwareOdometryNode()
    observer = Node('hardware_odom_test')
    executor = SingleThreadedExecutor()
    executor.add_node(hardware)
    executor.add_node(observer)
    wheel, canonical, transforms, joints = [], [], [], []
    observer.create_subscription(Odometry, 'wheel/odom', wheel.append, 10)
    observer.create_subscription(Odometry, 'odom', canonical.append, 10)
    observer.create_subscription(JointState, 'wheel_encoder/joint_states', joints.append, 10)
    observer.create_subscription(TFMessage, '/tf', transforms.append, 10)
    publisher = observer.create_publisher(WheelEncoderTicks, 'wheel/encoder_ticks', 10)
    boot_start = monotonic()
    sequence = 0

    def spin(duration):
        end = monotonic() + duration
        while monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def send(ticks, boot='0123456789abcdef'):
        nonlocal sequence
        message = WheelEncoderTicks()
        message.header.stamp = observer.get_clock().now().to_msg()
        message.ticks = ticks
        message.has_mcu_time = True
        message.boot_id = boot
        message.sample_seq = sequence
        message.sample_time_us = int((monotonic() - boot_start) * 1e6)
        publisher.publish(message)
        sequence += 1
        spin(0.08)

    try:
        deadline = monotonic() + 5
        while publisher.get_subscription_count() == 0 and monotonic() < deadline:
            spin(0.02)
        assert publisher.get_subscription_count() == 1
        for _ in range(4):
            send([0, 0, 0, 0])
        for _ in range(4):
            send([100, -100, -100, 100])
        expected_x = 100 * 2 * pi / 3172 * 0.0635
        assert wheel and canonical and joints and transforms
        assert wheel[-1].pose.pose.position.x == pytest.approx(expected_x)
        assert wheel[-1].pose.pose.position.y == pytest.approx(0.0)
        assert wheel[-1].pose.pose.orientation.z == pytest.approx(0.0)
        assert wheel[-1].header.frame_id == 'odom'
        assert wheel[-1].child_frame_id == 'base_link'
        assert canonical[-1].pose == wheel[-1].pose
        assert wheel[-1].pose.covariance[0] > 0
        transform = transforms[-1].transforms[0]
        assert transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link'
        assert transform.transform.translation.x == pytest.approx(expected_x)
        assert joints[-1].position == pytest.approx([100 * 2 * pi / 3172] * 4)

        boot_start = monotonic()
        sequence = 0
        for _ in range(4):
            send([0, 0, 0, 0], boot='fedcba9876543210')
        assert wheel[-1].pose.pose.position.x == pytest.approx(expected_x)
        assert wheel[-1].twist.twist.linear.x == pytest.approx(0.0)
        spin(0.2)
        before = (len(wheel), len(canonical), len(transforms))
        spin(0.6)
        assert (len(wheel), len(canonical), len(transforms)) == before
    finally:
        executor.shutdown()
        hardware.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
