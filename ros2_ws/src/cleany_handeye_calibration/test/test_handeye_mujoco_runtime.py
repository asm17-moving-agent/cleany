from __future__ import annotations

import math
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

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState


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
EXPOSED_JOINTS = tuple(
    joint_name
    for side in ('left', 'right')
    for joint_name in (*ARM_JOINTS[side], f'{side}_gripper_joint')
)
SUCCESS_TARGETS = {
    'left': (0.15, 0.20, 0.25, -0.15, 0.12),
    'right': (-0.15, 0.18, 0.22, 0.14, -0.10),
}
T = TypeVar('T')


def test_handeye_mujoco_launch_shows_viewer_by_default() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'handeye_mujoco.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    assert "'headless',\n                default_value='false'" in source


class MotionProbe(Node):
    def __init__(self) -> None:
        super().__init__(f'handeye_mujoco_probe_{uuid4().hex[:8]}')
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.joint_state_samples = 0
        self.controller_statuses: dict[
            str, dict[bytes, set[int]]
        ] = {side: {} for side in ('left', 'right')}

        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        for side in ('left', 'right'):
            self.create_subscription(
                GoalStatusArray,
                (
                    f'/{side}_arm_controller/'
                    'follow_joint_trajectory/_action/status'
                ),
                lambda message, side=side: self._on_controller_status(
                    side, message
                ),
                qos_profile_action_status_default,
            )

        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.execute_trajectory = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )
        self.controllers = {
            side: ActionClient(
                self,
                FollowJointTrajectory,
                f'/{side}_arm_controller/follow_joint_trajectory',
            )
            for side in ('left', 'right')
        }
        self.controller_cancel = {
            side: self.create_client(
                CancelGoal,
                (
                    f'/{side}_arm_controller/'
                    'follow_joint_trajectory/_action/cancel_goal'
                ),
            )
            for side in ('left', 'right')
        }
        self.get_planning_scene = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        self.positions = dict(
            zip(message.name, message.position, strict=True)
        )
        if len(message.name) == len(message.velocity):
            self.velocities = dict(
                zip(message.name, message.velocity, strict=True)
            )
        self.joint_state_samples += 1

    def _on_controller_status(
        self, side: str, message: GoalStatusArray
    ) -> None:
        history = self.controller_statuses[side]
        for status in message.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            history.setdefault(goal_id, set()).add(status.status)

    def controller_goal_ids(self, side: str) -> set[bytes]:
        return set(self.controller_statuses[side])


def _log_tail(path: Path, lines: int = 180) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<could not read launch log: {error}>'


def _assert_launch_running(
    process: subprocess.Popen[bytes], log_path: Path
) -> None:
    return_code = process.poll()
    if return_code is not None:
        pytest.fail(
            f'hand-eye MuJoCo launch exited early with code {return_code}\n'
            f'launch log tail:\n{_log_tail(log_path)}'
        )


def _wait_for(
    predicate: Callable[[], bool],
    *,
    timeout_sec: float,
    probe: MotionProbe,
    process: subprocess.Popen[bytes],
    log_path: Path,
    description: str,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        if predicate():
            return
        rclpy.spin_once(probe, timeout_sec=0.1)
    pytest.fail(
        f'timed out waiting for {description}\n'
        f'launch log tail:\n{_log_tail(log_path)}'
    )


def _future_result(
    future,
    *,
    timeout_sec: float,
    probe: MotionProbe,
    process: subprocess.Popen[bytes],
    log_path: Path,
    description: str,
) -> T:
    _wait_for(
        future.done,
        timeout_sec=timeout_sec,
        probe=probe,
        process=process,
        log_path=log_path,
        description=description,
    )
    exception = future.exception()
    if exception is not None:
        pytest.fail(f'{description} raised {exception!r}')
    result = future.result()
    assert result is not None
    return result


def _future_finishes_within(
    future,
    *,
    timeout_sec: float,
    probe: MotionProbe,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        rclpy.spin_once(probe, timeout_sec=0.05)
    return future.done()


def _spin_for(
    duration_sec: float,
    *,
    probe: MotionProbe,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        _assert_launch_running(process, log_path)
        rclpy.spin_once(probe, timeout_sec=0.05)


def _wait_for_interfaces(
    probe: MotionProbe,
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    action_clients = (
        ('/move_action', probe.move_group),
        ('/execute_trajectory', probe.execute_trajectory),
        *(
            (
                f'/{side}_arm_controller/follow_joint_trajectory',
                probe.controllers[side],
            )
            for side in ('left', 'right')
        ),
    )
    for action_name, client in action_clients:
        _wait_for(
            lambda client=client: client.wait_for_server(timeout_sec=0.0),
            timeout_sec=90.0,
            probe=probe,
            process=process,
            log_path=log_path,
            description=action_name,
        )
    for side, client in probe.controller_cancel.items():
        _wait_for(
            lambda client=client: client.wait_for_service(timeout_sec=0.0),
            timeout_sec=30.0,
            probe=probe,
            process=process,
            log_path=log_path,
            description=f'{side} controller cancel service',
        )
    _wait_for(
        lambda: probe.get_planning_scene.wait_for_service(timeout_sec=0.0),
        timeout_sec=60.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description='/get_planning_scene',
    )
    _wait_for(
        lambda: set(EXPOSED_JOINTS) <= probe.positions.keys()
        and set(EXPOSED_JOINTS) <= probe.velocities.keys(),
        timeout_sec=30.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description='complete arm and gripper feedback',
    )


def _plan_joint_goal(
    probe: MotionProbe,
    side: str,
    target: tuple[float, ...],
    *,
    velocity_scaling: float,
    process: subprocess.Popen[bytes],
    log_path: Path,
):
    goal = MoveGroup.Goal()
    goal.request.group_name = f'{side}_arm'
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = velocity_scaling
    goal.request.max_acceleration_scaling_factor = velocity_scaling
    goal.request.start_state.is_diff = True

    constraints = Constraints()
    constraints.name = f'{side}_joint_goal'
    for joint_name, position in zip(
        ARM_JOINTS[side], target, strict=True
    ):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = position
        constraint.tolerance_above = 1.0e-4
        constraint.tolerance_below = 1.0e-4
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)
    goal.request.goal_constraints = [constraints]

    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    goal_handle = _future_result(
        probe.move_group.send_goal_async(goal),
        timeout_sec=20.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} plan goal acceptance',
    )
    assert goal_handle.accepted
    wrapped_result = _future_result(
        goal_handle.get_result_async(),
        timeout_sec=60.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} plan result',
    )
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    result = wrapped_result.result
    assert result.error_code.val == MoveItErrorCodes.SUCCESS

    trajectory = result.planned_trajectory.joint_trajectory
    assert set(trajectory.joint_names) == set(ARM_JOINTS[side])
    assert trajectory.points
    final_positions = dict(
        zip(
            trajectory.joint_names,
            trajectory.points[-1].positions,
            strict=True,
        )
    )
    for joint_name, expected in zip(
        ARM_JOINTS[side], target, strict=True
    ):
        assert final_positions[joint_name] == pytest.approx(
            expected, abs=1.0e-3
        )
    assert not result.executed_trajectory.joint_trajectory.points
    return result


def _execute_plan(
    probe: MotionProbe,
    side: str,
    trajectory,
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> bytes:
    previous_goal_ids = probe.controller_goal_ids(side)
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = trajectory
    goal_handle = _future_result(
        probe.execute_trajectory.send_goal_async(goal),
        timeout_sec=10.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} execute goal acceptance',
    )
    assert goal_handle.accepted
    wrapped_result = _future_result(
        goal_handle.get_result_async(),
        timeout_sec=60.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} execute result',
    )
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def controller_succeeded() -> bool:
        new_goal_ids = probe.controller_goal_ids(side) - previous_goal_ids
        return len(new_goal_ids) == 1 and all(
            GoalStatus.STATUS_SUCCEEDED
            in probe.controller_statuses[side][goal_id]
            for goal_id in new_goal_ids
        )

    _wait_for(
        controller_succeeded,
        timeout_sec=10.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} controller success status',
    )
    return (probe.controller_goal_ids(side) - previous_goal_ids).pop()


def _wait_for_settle(
    probe: MotionProbe,
    side: str,
    target: tuple[float, ...],
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    settled_since: float | None = None

    def settled() -> bool:
        nonlocal settled_since
        position_ok = all(
            abs(probe.positions.get(name, math.inf) - expected) <= 0.005
            for name, expected in zip(
                ARM_JOINTS[side], target, strict=True
            )
        )
        velocity_ok = all(
            abs(probe.velocities.get(name, math.inf)) <= 0.01
            for name in ARM_JOINTS[side]
        )
        if position_ok and velocity_ok:
            if settled_since is None:
                settled_since = time.monotonic()
            return time.monotonic() - settled_since >= 0.25
        settled_since = None
        return False

    _wait_for(
        settled,
        timeout_sec=15.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description=f'{side} feedback settle',
    )


def _assert_complete_current_state(
    state: RobotState, feedback: dict[str, float]
) -> None:
    assert len(state.joint_state.name) == len(state.joint_state.position)
    state_positions = dict(
        zip(
            state.joint_state.name,
            state.joint_state.position,
            strict=True,
        )
    )
    assert set(EXPOSED_JOINTS) <= state_positions.keys()
    for joint_name in EXPOSED_JOINTS:
        assert state_positions[joint_name] == pytest.approx(
            feedback[joint_name], abs=0.01
        )


def _planning_scene_current_state(
    probe: MotionProbe,
    *,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> RobotState:
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.ROBOT_STATE
    response = _future_result(
        probe.get_planning_scene.call_async(request),
        timeout_sec=20.0,
        probe=probe,
        process=process,
        log_path=log_path,
        description='MoveIt planning-scene current state',
    )
    return response.scene.robot_state


def _signal_process_group(process: subprocess.Popen[bytes], sig: int) -> None:
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def _stop_launch(process: subprocess.Popen[bytes]) -> None:
    _signal_process_group(process, signal.SIGINT)
    if process.poll() is None:
        try:
            process.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            pass
    if process.poll() is None:
        _signal_process_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            pass
    if process.poll() is None:
        _signal_process_group(process, signal.SIGKILL)
        process.wait(timeout=5.0)
    else:
        # ros2 launch can exit before one of its children. The process group is
        # private to this test, so clean any such survivor as well.
        _signal_process_group(process, signal.SIGTERM)


def test_moveit_plans_executes_and_cancels_mujoco_trajectories() -> None:
    domain_id = 1 + uuid4().int % 231
    with tempfile.TemporaryDirectory(
        prefix='cleany_handeye_mujoco_test_'
    ) as temp_dir:
        temp_root = Path(temp_dir)
        log_path = temp_root / 'launch.log'
        environment = os.environ.copy()
        environment.update(
            {
                'ROS_DOMAIN_ID': str(domain_id),
                'ROS_HOME': str(temp_root / 'ros_home'),
                'ROS_LOG_DIR': str(temp_root / 'ros_log'),
            }
        )

        probe: MotionProbe | None = None
        with log_path.open('wb') as launch_log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'cleany_handeye_calibration',
                    'handeye_mujoco.launch.py',
                    'headless:=true',
                    'sim_speed_factor:=1.0',
                ],
                env=environment,
                cwd=temp_root,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                rclpy.init(args=None, domain_id=domain_id)
                probe = MotionProbe()
                _wait_for_interfaces(
                    probe, process=process, log_path=log_path
                )

                node_names = {
                    name for name, _ in probe.get_node_names_and_namespaces()
                }
                assert {
                    'controller_manager',
                    'move_group',
                    'robot_state_publisher',
                } <= node_names
                assert 'mujoco_sim_node' not in node_names
                assert not any('gazebo' in name.lower() for name in node_names)
                topic_names = {
                    name for name, _ in probe.get_topic_names_and_types()
                }
                assert not any(
                    name.endswith('/joint_cmd') for name in topic_names
                )

                initial_feedback = dict(probe.positions)
                initial_controller_goals = {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                }

                # The calibration motion boundary is left-only. Planning must
                # not contact either controller, and execution must not create
                # a right-controller goal.
                left_plan = _plan_joint_goal(
                    probe,
                    'left',
                    SUCCESS_TARGETS['left'],
                    velocity_scaling=0.1,
                    process=process,
                    log_path=log_path,
                )
                _spin_for(
                    0.25,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                )
                assert {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                } == initial_controller_goals
                _assert_complete_current_state(
                    left_plan.trajectory_start, initial_feedback
                )

                _execute_plan(
                    probe,
                    'left',
                    left_plan.planned_trajectory,
                    process=process,
                    log_path=log_path,
                )
                _wait_for_settle(
                    probe,
                    'left',
                    SUCCESS_TARGETS['left'],
                    process=process,
                    log_path=log_path,
                )
                assert probe.controller_goal_ids('right') == (
                    initial_controller_goals['right']
                )
                for joint_name in ARM_JOINTS['right']:
                    assert probe.positions[joint_name] == pytest.approx(
                        initial_feedback[joint_name], abs=0.005
                    )

                # A separate right-arm plan starts from the complete feedback
                # state, including the already moved left arm and grippers.
                feedback_after_left = dict(probe.positions)
                controller_goals_before_right_plan = {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                }
                right_plan = _plan_joint_goal(
                    probe,
                    'right',
                    SUCCESS_TARGETS['right'],
                    velocity_scaling=0.1,
                    process=process,
                    log_path=log_path,
                )
                _spin_for(
                    0.25,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                )
                assert {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                } == controller_goals_before_right_plan
                _assert_complete_current_state(
                    right_plan.trajectory_start, feedback_after_left
                )

                _execute_plan(
                    probe,
                    'right',
                    right_plan.planned_trajectory,
                    process=process,
                    log_path=log_path,
                )
                _wait_for_settle(
                    probe,
                    'right',
                    SUCCESS_TARGETS['right'],
                    process=process,
                    log_path=log_path,
                )
                for joint_name in ARM_JOINTS['left']:
                    assert probe.positions[joint_name] == pytest.approx(
                        feedback_after_left[joint_name], abs=0.005
                    )

                feedback_after_both = dict(probe.positions)
                scene_state = _planning_scene_current_state(
                    probe, process=process, log_path=log_path
                )
                _assert_complete_current_state(
                    scene_state, feedback_after_both
                )

                # Exercise the timeout path with a deliberately slow left wrist
                # trajectory. Humble MoveIt's ExecuteTrajectory capability
                # cannot service its cancel callback while execution blocks.
                # The compatibility fallback cancels the active side's standard
                # FollowJointTrajectory goal directly and verifies controller
                # and upstream MoveIt terminal statuses explicitly.
                cancel_target = tuple(
                    1.5
                    if joint_name == 'left_wrist_roll_joint'
                    else feedback_after_both[joint_name]
                    for joint_name in ARM_JOINTS['left']
                )
                controller_goals_before_cancel_plan = {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                }
                cancel_plan = _plan_joint_goal(
                    probe,
                    'left',
                    cancel_target,
                    velocity_scaling=0.01,
                    process=process,
                    log_path=log_path,
                )
                _assert_complete_current_state(
                    cancel_plan.trajectory_start, feedback_after_both
                )
                assert {
                    side: probe.controller_goal_ids(side)
                    for side in ('left', 'right')
                } == controller_goals_before_cancel_plan

                execute_goal = ExecuteTrajectory.Goal()
                execute_goal.trajectory = cancel_plan.planned_trajectory
                execute_handle = _future_result(
                    probe.execute_trajectory.send_goal_async(execute_goal),
                    timeout_sec=10.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='cancel-path execute goal acceptance',
                )
                assert execute_handle.accepted
                execute_result_future = execute_handle.get_result_async()

                def left_controller_executing() -> bool:
                    new_goal_ids = probe.controller_goal_ids('left') - (
                        controller_goals_before_cancel_plan['left']
                    )
                    return len(new_goal_ids) == 1 and all(
                        GoalStatus.STATUS_EXECUTING
                        in probe.controller_statuses['left'][goal_id]
                        for goal_id in new_goal_ids
                    )

                _wait_for(
                    left_controller_executing,
                    timeout_sec=10.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='left controller executing cancel-path goal',
                )
                assert not _future_finishes_within(
                    execute_result_future,
                    timeout_sec=0.25,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                )

                new_left_goal_ids = probe.controller_goal_ids('left') - (
                    controller_goals_before_cancel_plan['left']
                )
                assert len(new_left_goal_ids) == 1
                canceled_controller_goal = next(iter(new_left_goal_ids))

                cancel_response = _future_result(
                    probe.controller_cancel['left'].call_async(
                        CancelGoal.Request()
                    ),
                    timeout_sec=10.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='left controller action cancel response',
                )
                assert cancel_response.return_code == (
                    CancelGoal.Response.ERROR_NONE
                )
                assert any(
                    bytes(goal.goal_id.uuid) == canceled_controller_goal
                    for goal in cancel_response.goals_canceling
                )

                canceled_result = _future_result(
                    execute_result_future,
                    timeout_sec=20.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='canceled execute trajectory terminal result',
                )
                assert canceled_result.status == GoalStatus.STATUS_ABORTED
                assert canceled_result.result.error_code.val == (
                    MoveItErrorCodes.PREEMPTED
                )

                _wait_for(
                    lambda: GoalStatus.STATUS_CANCELED
                    in probe.controller_statuses['left'][
                        canceled_controller_goal
                    ],
                    timeout_sec=10.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='left controller canceled terminal status',
                )
                assert probe.controller_goal_ids('right') == (
                    controller_goals_before_cancel_plan['right']
                )

                _wait_for(
                    lambda: all(
                        abs(probe.velocities.get(name, math.inf)) <= 0.01
                        for name in ARM_JOINTS['left']
                    ),
                    timeout_sec=5.0,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                    description='left controller cancel hold',
                )
                held_positions = {
                    name: probe.positions[name]
                    for name in ARM_JOINTS['left']
                }
                _spin_for(
                    0.5,
                    probe=probe,
                    process=process,
                    log_path=log_path,
                )
                for joint_name, held_position in held_positions.items():
                    assert probe.positions[joint_name] == pytest.approx(
                        held_position, abs=0.005
                    )
                assert probe.joint_state_samples > 20
                _assert_launch_running(process, log_path)
            finally:
                if probe is not None:
                    probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                _stop_launch(process)
                launch_log.flush()

        log_text = log_path.read_text(encoding='utf-8', errors='replace')
        instability_markers = (
            'simulation is unstable',
            'nan, inf or huge value in ctrl',
            'position state is not finite',
        )
        assert not any(
            marker in log_text.lower() for marker in instability_markers
        ), _log_tail(log_path)
