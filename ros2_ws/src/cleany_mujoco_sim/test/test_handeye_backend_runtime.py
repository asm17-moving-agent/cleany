from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from typing import Callable, TypeVar
from uuid import uuid4

import pytest


if os.environ.get('ROS_DISTRO') is None:
    pytest.skip('ROS 2 environment is not active', allow_module_level=True)

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINT_SUFFIXES = (
    'shoulder_yaw_joint',
    'shoulder_pitch_joint',
    'elbow_pitch_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)
ARM_JOINTS = {
    side: tuple(f'{side}_{suffix}' for suffix in ARM_JOINT_SUFFIXES)
    for side in ('left', 'right')
}
GRIPPER_JOINTS = ('left_gripper_joint', 'right_gripper_joint')
EXPOSED_JOINTS = frozenset(
    (*ARM_JOINTS['left'], *ARM_JOINTS['right'], *GRIPPER_JOINTS)
)
TARGETS = {
    'left': (0.15, 0.20, 0.25, -0.15, 0.12),
    'right': (-0.15, 0.18, 0.22, 0.14, -0.10),
}
T = TypeVar('T')


class BackendProbe(Node):
    def __init__(self) -> None:
        super().__init__('handeye_backend_runtime_probe')
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.joint_state_samples = 0
        self.simulation_time = 0.0
        self.create_subscription(
            Clock,
            '/clock',
            self._on_clock,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.actions = {
            side: ActionClient(
                self,
                FollowJointTrajectory,
                f'/{side}_arm_controller/follow_joint_trajectory',
            )
            for side in ('left', 'right')
        }
        self.list_controllers = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )

    def _on_clock(self, message: Clock) -> None:
        self.simulation_time = (
            float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        )

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        self.positions = dict(zip(message.name, message.position, strict=True))
        if len(message.name) == len(message.velocity):
            self.velocities = dict(
                zip(message.name, message.velocity, strict=True)
            )
        self.joint_state_samples += 1


def _log_tail(path: Path, lines: int = 160) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<could not read launch log: {error}>'


def _assert_launch_running(
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    return_code = process.poll()
    if return_code is not None:
        pytest.fail(
            f'hand-eye backend exited early with code {return_code}\n'
            f'launch log tail:\n{_log_tail(log_path)}'
        )


def _wait_for(
    predicate: Callable[[], bool],
    *,
    timeout_sec: float,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
    description: str,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail(
        f'timed out waiting for {description}\n'
        f'launch log tail:\n{_log_tail(log_path)}'
    )


def _future_result(
    future,
    *,
    timeout_sec: float,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
    description: str,
) -> T:
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        pytest.fail(
            f'timed out waiting for {description}\n'
            f'launch log tail:\n{_log_tail(log_path)}'
        )
    exception = future.exception()
    if exception is not None:
        pytest.fail(f'{description} raised {exception!r}')
    result = future.result()
    assert result is not None
    return result


def _wait_for_active_arm_controllers(
    probe: BackendProbe,
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> dict[str, ControllerState]:
    expected_names = {
        f'{side}_arm_controller' for side in ('left', 'right')
    }
    deadline = time.monotonic() + 20.0
    last_states: dict[str, str] = {}
    while time.monotonic() < deadline:
        response = _future_result(
            probe.list_controllers.call_async(ListControllers.Request()),
            timeout_sec=5.0,
            node=probe,
            process=process,
            log_path=log_path,
            description='controller activation inspection',
        )
        controllers_by_name = {
            controller.name: controller for controller in response.controller
        }
        last_states = {
            name: controllers_by_name[name].state
            for name in expected_names
            if name in controllers_by_name
        }
        if last_states == {name: 'active' for name in expected_names}:
            return controllers_by_name
        rclpy.spin_once(probe, timeout_sec=0.1)
    pytest.fail(
        'timed out waiting for active arm controllers; '
        f'last states: {last_states}'
    )


def _execute_trajectory(
    probe: BackendProbe,
    side: str,
    target: tuple[float, ...],
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS[side])
    point = JointTrajectoryPoint()
    point.positions = list(target)
    point.velocities = [0.0] * len(target)
    point.time_from_start.sec = 3
    goal.trajectory.points = [point]
    goal.goal_time_tolerance.sec = 2

    goal_handle = _future_result(
        probe.actions[side].send_goal_async(goal),
        timeout_sec=10.0,
        node=probe,
        process=process,
        log_path=log_path,
        description=f'{side} trajectory goal acceptance',
    )
    assert goal_handle.accepted
    wrapped_result = _future_result(
        goal_handle.get_result_async(),
        timeout_sec=15.0,
        node=probe,
        process=process,
        log_path=log_path,
        description=f'{side} trajectory result',
    )
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped_result.result.error_code == (
        FollowJointTrajectory.Result.SUCCESSFUL
    ), wrapped_result.result.error_string


def _wait_for_settle(
    probe: BackendProbe,
    side: str,
    target: tuple[float, ...],
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    settled_since: float | None = None

    def settled_for_half_second() -> bool:
        nonlocal settled_since
        positions_ok = all(
            abs(probe.positions.get(name, float('inf')) - expected) <= 0.005
            for name, expected in zip(ARM_JOINTS[side], target, strict=True)
        )
        velocities_ok = all(
            abs(probe.velocities.get(name, float('inf'))) <= 0.01
            for name in ARM_JOINTS[side]
        )
        if positions_ok and velocities_ok:
            if settled_since is None:
                settled_since = time.monotonic()
            return time.monotonic() - settled_since >= 0.5
        settled_since = None
        return False

    _wait_for(
        settled_for_half_second,
        timeout_sec=10.0,
        node=probe,
        process=process,
        log_path=log_path,
        description=f'{side} arm convergence',
    )


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def test_raw_per_arm_trajectories_and_joint_state_feedback() -> None:
    with tempfile.TemporaryDirectory(
        prefix='cleany_handeye_backend_test_'
    ) as tmp:
        temp_root = Path(tmp)
        log_path = temp_root / 'launch.log'
        domain_id = 20 + uuid4().int % 180
        environment = os.environ.copy()
        environment.update(
            {
                'ROS_DOMAIN_ID': str(domain_id),
                'ROS_HOME': str(temp_root / 'ros_home'),
                'ROS_LOG_DIR': str(temp_root / 'ros_log'),
            }
        )
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'cleany_mujoco_sim',
                    'handeye_backend.launch.py',
                    'headless:=true',
                    'sim_speed_factor:=1.0',
                ],
                env=environment,
                cwd=temp_root,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init(args=None, domain_id=domain_id)
            probe = BackendProbe()
            try:
                for side, action in probe.actions.items():
                    _wait_for(
                        lambda action=action: action.wait_for_server(
                            timeout_sec=0.0
                        ),
                        timeout_sec=90.0,
                        node=probe,
                        process=process,
                        log_path=log_path,
                        description=f'{side} FollowJointTrajectory action',
                    )
                _wait_for(
                    lambda: EXPOSED_JOINTS <= probe.positions.keys()
                    and EXPOSED_JOINTS <= probe.velocities.keys(),
                    timeout_sec=30.0,
                    node=probe,
                    process=process,
                    log_path=log_path,
                    description='all arm and gripper joint states',
                )
                assert set(probe.positions) == EXPOSED_JOINTS
                _wait_for(
                    lambda: probe.simulation_time > 0.5,
                    timeout_sec=10.0,
                    node=probe,
                    process=process,
                    log_path=log_path,
                    description='advancing MuJoCo simulation clock',
                )
                clock_before_trajectories = probe.simulation_time

                _wait_for(
                    lambda: probe.list_controllers.wait_for_service(
                        timeout_sec=0.0
                    ),
                    timeout_sec=20.0,
                    node=probe,
                    process=process,
                    log_path=log_path,
                    description='/controller_manager/list_controllers',
                )
                controllers_by_name = _wait_for_active_arm_controllers(
                    probe,
                    process=process,
                    log_path=log_path,
                )
                for side in ('left', 'right'):
                    controller = controllers_by_name[f'{side}_arm_controller']
                    assert set(controller.claimed_interfaces) == {
                        f'{joint_name}/position'
                        for joint_name in ARM_JOINTS[side]
                    }

                topic_names = {
                    name for name, _ in probe.get_topic_names_and_types()
                }
                assert not any(
                    name.endswith('/joint_cmd') for name in topic_names
                )

                initial_positions = dict(probe.positions)
                _execute_trajectory(
                    probe,
                    'left',
                    TARGETS['left'],
                    process=process,
                    log_path=log_path,
                )
                _wait_for_settle(
                    probe,
                    'left',
                    TARGETS['left'],
                    process=process,
                    log_path=log_path,
                )
                assert all(
                    abs(probe.positions[name] - initial_positions[name])
                    <= 0.005
                    for name in ARM_JOINTS['right']
                )

                left_positions = {
                    name: probe.positions[name]
                    for name in ARM_JOINTS['left']
                }
                _execute_trajectory(
                    probe,
                    'right',
                    TARGETS['right'],
                    process=process,
                    log_path=log_path,
                )
                _wait_for_settle(
                    probe,
                    'right',
                    TARGETS['right'],
                    process=process,
                    log_path=log_path,
                )
                assert all(
                    abs(probe.positions[name] - left_positions[name]) <= 0.005
                    for name in ARM_JOINTS['left']
                )
                assert all(
                    abs(probe.positions[name]) <= 1.0e-6
                    and abs(probe.velocities[name]) <= 0.01
                    for name in GRIPPER_JOINTS
                )
                assert probe.joint_state_samples > 10
                assert probe.simulation_time > clock_before_trajectories + 5.0
            finally:
                probe.destroy_node()
                rclpy.shutdown()
                _stop_launch(process)
                launch_log.flush()

        log_text = log_path.read_text(encoding='utf-8', errors='replace')
        incomplete_state_warnings = [
            line
            for line in log_text.splitlines()
            if 'incomplete' in line.lower() and 'state' in line.lower()
        ]
        assert incomplete_state_warnings == []
        instability_warnings = [
            line
            for line in log_text.splitlines()
            if 'simulation is unstable' in line.lower()
            or 'nan, inf or huge value in ctrl' in line.lower()
            or 'position state is not finite' in line.lower()
        ]
        assert instability_warnings == []
