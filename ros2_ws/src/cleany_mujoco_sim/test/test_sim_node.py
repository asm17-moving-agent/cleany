import time
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan

from cleany_mujoco_sim.sim_node import MujocoSimNode
from cleany_mujoco_sim.state import joint_positions


def _make_node(scene_path: Path, **overrides) -> MujocoSimNode:
    step_observers = overrides.pop('step_observers', None)
    params = {
        'scene_path': str(scene_path),
        'publish_rate_hz': 1000.0,
        'headless': True,
        'scan_samples': 8,
    }
    params.update(overrides)
    return MujocoSimNode(
        step_observers=step_observers,
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


def test_sim_node_exposes_context_and_notifies_observer_after_step(
    scene_path: Path,
):
    class RecordingObserver:
        def __init__(self) -> None:
            self.calls = []

        def after_step(self, context, stamp) -> None:
            self.calls.append((context, stamp, context.data.time))

    rclpy.init(args=[])
    node = None
    try:
        observer = RecordingObserver()
        node = _make_node(scene_path, step_observers=[observer])

        assert node.simulation_context.model is node._model
        assert node.simulation_context.data is node._data
        assert node.simulation_context.data.time == pytest.approx(0.0)

        node._on_timer()

        assert len(observer.calls) == 1
        context, stamp, simulation_time = observer.calls[0]
        assert context is node.simulation_context
        assert isinstance(stamp, Time)
        assert simulation_time > 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_can_register_observer_after_construction(scene_path: Path):
    class CountingObserver:
        def __init__(self) -> None:
            self.call_count = 0

        def after_step(self, context, stamp) -> None:
            self.call_count += 1

    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(scene_path)
        observer = CountingObserver()

        node.add_step_observer(observer)
        node._on_timer()

        assert observer.call_count == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_propagates_step_observer_errors(scene_path: Path):
    class FailingObserver:
        def after_step(self, context, stamp) -> None:
            raise RuntimeError('sensor failure')

    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(scene_path, step_observers=[FailingObserver()])

        with pytest.raises(RuntimeError, match='sensor failure'):
            node._on_timer()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_applies_opt_in_initial_joint_positions(scene_path: Path):
    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(
            scene_path,
            initial_joint_names=['shoulder'],
            initial_joint_positions=[0.01],
        )

        assert joint_positions(node._model, node._data) == pytest.approx(
            [0.01]
        )
        assert node._data.ctrl[0] == pytest.approx(0.01)

        node._on_timer()

        assert joint_positions(node._model, node._data) == pytest.approx(
            [0.01], abs=1e-6
        )
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_sim_node_keeps_model_defaults_without_initial_joint_params(
    scene_path: Path,
):
    rclpy.init(args=[])
    node = None
    try:
        node = _make_node(scene_path)

        assert joint_positions(node._model, node._data) == pytest.approx([0.0])
        assert node._data.ctrl[0] == pytest.approx(0.0)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
