import time
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan

from cleany_mujoco_sim.sim_node import MujocoSimNode
from cleany_mujoco_sim.state import joint_positions


def _make_node(scene_path: Path, **overrides) -> MujocoSimNode:
    params = {
        'scene_path': str(scene_path),
        'publish_rate_hz': 1000.0,
        'headless': True,
        'scan_samples': 8,
        'camera_enabled': False,
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


def test_sim_node_publishes_camera_image(scene_path: Path):
    rclpy.init(args=[])
    node = None
    try:
        try:
            node = _make_node(
                scene_path,
                camera_enabled=True,
                camera_width=64,
                camera_height=48,
                camera_rate_hz=100.0,
            )
        except Exception as exc:  # noqa: BLE001 - GL backend may be unavailable
            pytest.skip(f'MuJoCo offscreen rendering unavailable: {exc}')

        received: list[Image] = []
        node.create_subscription(Image, 'image_raw', received.append, 10)

        deadline = time.time() + 3.0
        while not received and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert received
        assert received[0].encoding == 'rgb8'
        assert received[0].width == 64
        assert received[0].height == 48
        assert received[0].header.frame_id == 'head_camera_rgb_optical_frame'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_publishes_depth_and_camera_info(scene_path: Path):
    rclpy.init(args=[])
    node = None
    try:
        try:
            node = _make_node(
                scene_path,
                camera_enabled=True,
                camera_width=64,
                camera_height=48,
                camera_rate_hz=100.0,
            )
        except Exception as exc:  # noqa: BLE001 - GL backend may be unavailable
            pytest.skip(f'MuJoCo offscreen rendering unavailable: {exc}')

        depth_received: list[Image] = []
        info_received: list[CameraInfo] = []
        node.create_subscription(Image, 'depth', depth_received.append, 10)
        node.create_subscription(CameraInfo, 'camera_info', info_received.append, 10)

        deadline = time.time() + 3.0
        while (not depth_received or not info_received) and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        assert depth_received
        assert depth_received[0].encoding == '32FC1'
        assert depth_received[0].width == 64
        assert depth_received[0].height == 48
        assert depth_received[0].header.frame_id == 'head_camera_rgb_optical_frame'
        assert info_received
        assert info_received[0].width == 64
        assert info_received[0].height == 48
        assert info_received[0].k[0] > 0.0
        assert info_received[0].header.frame_id == 'head_camera_rgb_optical_frame'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_skips_depth_when_disabled(scene_path: Path):
    rclpy.init(args=[])
    node = None
    try:
        try:
            node = _make_node(
                scene_path,
                camera_enabled=True,
                camera_width=64,
                camera_height=48,
                camera_rate_hz=100.0,
                depth_enabled=False,
            )
        except Exception as exc:  # noqa: BLE001 - GL backend may be unavailable
            pytest.skip(f'MuJoCo offscreen rendering unavailable: {exc}')

        assert node._depth_renderer is None
        assert node._depth_pub is None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_rejects_missing_camera(scene_path: Path):
    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError):
            _make_node(scene_path, camera_enabled=True, camera_name='no_such_camera')
    finally:
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
