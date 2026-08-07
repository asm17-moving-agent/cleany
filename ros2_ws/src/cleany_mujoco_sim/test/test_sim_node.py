import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, LaserScan

from cleany_mujoco_sim.base_command import ChassisCommand, stopped_command
from cleany_mujoco_sim.mecanum_kinematics import stopped_wheel_speeds
from cleany_mujoco_sim.sim_node import MujocoSimNode
from cleany_mujoco_sim.state import joint_positions


def _make_node(scene_path: Path, **overrides) -> MujocoSimNode:
    params = {
        'scene_path': str(scene_path),
        'publish_rate_hz': 1000.0,
        'headless': True,
        'scan_samples': 8,
        'base_drive_enabled': False,
    }
    params.update(overrides)
    return MujocoSimNode(
        namespace='test_mujoco_sim',
        parameter_overrides=[Parameter(name, value=value) for name, value in params.items()]
    )


def test_sim_node_publishes_joint_states(scene_path: Path):
    rclpy.init(args=[])
    try:
        node = _make_node(scene_path)
        received: list[JointState] = []
        node.create_subscription(JointState, 'joint_states', received.append, 10)

        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received
        assert received[0].name == ['shoulder']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_publishes_odometry_and_scan(scene_path: Path):
    rclpy.init(args=[])
    try:
        node = _make_node(scene_path)
        odom_received: list[Odometry] = []
        scan_received: list[LaserScan] = []
        node.create_subscription(Odometry, 'odom', odom_received.append, 10)
        node.create_subscription(LaserScan, 'scan', scan_received.append, 10)

        deadline = time.time() + 2.0
        while (not odom_received or not scan_received) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert odom_received
        assert odom_received[0].header.frame_id == 'odom'
        assert odom_received[0].child_frame_id == 'base_link'
        assert scan_received
        assert scan_received[0].header.frame_id == 'laser'
        assert len(scan_received[0].ranges) == 8
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_applies_joint_cmd(scene_path: Path):
    rclpy.init(args=[])
    try:
        node = _make_node(scene_path)
        commander = rclpy.create_node('test_commander')
        cmd_pub = commander.create_publisher(
            JointState, '/test_mujoco_sim/mujoco_sim/joint_cmd', 10
        )

        cmd = JointState()
        cmd.name = ['shoulder']
        cmd.position = [0.4]

        applied = False
        deadline = time.time() + 2.0
        while time.time() < deadline:
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            if joint_positions(node._model, node._data) == pytest.approx([0.4]):
                applied = True
                break

        assert applied
        commander.destroy_node()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_accepts_and_bounds_supported_cmd_vel_axes(scene_path: Path):
    rclpy.init(args=[])
    commander = None
    try:
        node = _make_node(
            scene_path,
            max_linear_x=0.2,
            max_linear_y=0.15,
            max_angular_z=0.5,
            cmd_vel_timeout_sec=1.0,
            wheel_radius=0.1,
            wheelbase_length=0.4,
            track_width=0.2,
            max_wheel_speed=10.0,
        )
        commander = rclpy.create_node('test_cmd_vel_commander')
        cmd_pub = commander.create_publisher(
            Twist, '/test_mujoco_sim/cmd_vel', 10
        )

        cmd = Twist()
        cmd.linear.x = 0.4
        cmd.linear.y = -0.1
        cmd.linear.z = 1.0
        cmd.angular.z = -0.8

        expected = ChassisCommand(
            linear_x=0.2,
            linear_y=-0.1,
            angular_z=-0.5,
        )
        deadline = time.time() + 2.0
        while (
            node._current_chassis_command != expected
            and time.time() < deadline
        ):
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)

        assert node._current_chassis_command == expected
        assert (
            node._target_wheel_speeds.front_left,
            node._target_wheel_speeds.front_right,
            node._target_wheel_speeds.rear_left,
            node._target_wheel_speeds.rear_right,
        ) == pytest.approx((4.5, -0.5, 2.5, 1.5))
        assert node._last_cmd_vel_time is not None
    finally:
        if commander is not None:
            commander.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_drives_xlerobot_from_cmd_vel(cleany_scene_path: Path):
    rclpy.init(args=[])
    commander = None
    node = None
    try:
        node = _make_node(
            cleany_scene_path,
            publish_rate_hz=200.0,
            scan_enabled=False,
            base_drive_enabled=True,
        )
        commander = rclpy.create_node('test_drive_cmd_vel_commander')
        cmd_pub = commander.create_publisher(
            Twist, '/test_mujoco_sim/cmd_vel', 10
        )

        start_x = float(node._data.xpos[node._base_body_id, 0])
        cmd = Twist()
        cmd.linear.x = 0.1
        deadline = time.time() + 3.0
        while (
            node._data.xpos[node._base_body_id, 0] - start_x <= 0.05
            and time.time() < deadline
        ):
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.01)

        assert node._data.xpos[node._base_body_id, 0] - start_x > 0.05

        deadline = time.time() + 3.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            measured = node._mujoco_drive.measured_speeds(node._data)
            peak_speed = max(
                abs(measured.front_left),
                abs(measured.front_right),
                abs(measured.rear_left),
                abs(measured.rear_right),
            )
            if node._last_cmd_vel_time is None and peak_speed < 0.1:
                break

        assert node._last_cmd_vel_time is None
        assert peak_speed < 0.1
    finally:
        if commander is not None:
            commander.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_stops_on_non_finite_cmd_vel(scene_path: Path):
    rclpy.init(args=[])
    commander = None
    try:
        node = _make_node(scene_path, cmd_vel_timeout_sec=1.0)
        commander = rclpy.create_node('test_invalid_cmd_vel_commander')
        cmd_pub = commander.create_publisher(
            Twist, '/test_mujoco_sim/cmd_vel', 10
        )

        valid_cmd = Twist()
        valid_cmd.linear.x = 0.1
        deadline = time.time() + 2.0
        while (
            node._current_chassis_command.linear_x != 0.1
            and time.time() < deadline
        ):
            cmd_pub.publish(valid_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node._current_chassis_command.linear_x == pytest.approx(0.1)

        invalid_cmd = Twist()
        invalid_cmd.angular.x = float('nan')
        deadline = time.time() + 2.0
        while node._last_cmd_vel_time is not None and time.time() < deadline:
            cmd_pub.publish(invalid_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)

        assert node._current_chassis_command == stopped_command()
        assert node._target_wheel_speeds == stopped_wheel_speeds()
        assert node._last_cmd_vel_time is None
    finally:
        if commander is not None:
            commander.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_stops_after_cmd_vel_timeout(scene_path: Path):
    rclpy.init(args=[])
    commander = None
    try:
        node = _make_node(
            scene_path,
            cmd_vel_timeout_sec=0.05,
            timeout_check_rate_hz=1000.0,
        )
        commander = rclpy.create_node('test_timeout_cmd_vel_commander')
        cmd_pub = commander.create_publisher(
            Twist, '/test_mujoco_sim/cmd_vel', 10
        )

        cmd = Twist()
        cmd.linear.x = 0.1
        deadline = time.time() + 2.0
        while node._last_cmd_vel_time is None and time.time() < deadline:
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.01)
        assert node._last_cmd_vel_time is not None

        deadline = time.time() + 2.0
        while node._last_cmd_vel_time is not None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

        assert node._current_chassis_command == stopped_command()
        assert node._target_wheel_speeds == stopped_wheel_speeds()
        assert node._last_cmd_vel_time is None
    finally:
        if commander is not None:
            commander.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_sim_node_rejects_non_positive_publish_rate(scene_path: Path):
    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError):
            _make_node(scene_path, publish_rate_hz=0.0)
    finally:
        rclpy.shutdown()


def test_sim_node_rejects_missing_scene_path(tmp_path: Path):
    rclpy.init(args=[])
    try:
        with pytest.raises(FileNotFoundError):
            _make_node(tmp_path / "does-not-exist.xml")
    finally:
        rclpy.shutdown()


def test_sim_node_allows_zero_scan_rate_when_scan_disabled(scene_path: Path):
    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(scene_path, scan_enabled=False, scan_rate_hz=0.0)
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ('parameter_name', 'value'),
    [
        ('max_linear_x', 0.0),
        ('max_linear_y', -0.1),
        ('max_angular_z', float('inf')),
        ('cmd_vel_timeout_sec', float('nan')),
        ('timeout_check_rate_hz', 0.0),
        ('wheel_radius', 0.0),
        ('wheelbase_length', -0.1),
        ('track_width', float('nan')),
        ('max_wheel_speed', float('inf')),
        ('wheel_kp', -0.1),
        ('wheel_ki', float('nan')),
        ('wheel_kd', float('inf')),
        ('motor_voltage_limit', 0.0),
        ('motor_no_load_speed', -0.1),
    ],
)
def test_sim_node_rejects_invalid_command_parameters(
    scene_path: Path,
    parameter_name: str,
    value: float,
):
    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError):
            _make_node(scene_path, **{parameter_name: value})
    finally:
        rclpy.shutdown()
