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

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


ARM_JOINT_SUFFIXES = (
    'shoulder_yaw_joint',
    'shoulder_pitch_joint',
    'elbow_pitch_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)
ALL_JOINTS = tuple(
    joint
    for side in ('left', 'right')
    for joint in (
        *(f'{side}_{suffix}' for suffix in ARM_JOINT_SUFFIXES),
        f'{side}_gripper_joint',
    )
)
T = TypeVar('T')


def _arm_joints(side: str) -> tuple[str, ...]:
    return tuple(f'{side}_{suffix}' for suffix in ARM_JOINT_SUFFIXES)


def _log_tail(path: Path, lines: int = 120) -> str:
    try:
        return '\n'.join(
            path.read_text(encoding='utf-8', errors='replace').splitlines()[
                -lines:
            ]
        )
    except OSError as error:
        return f'<could not read launch log: {error}>'


def _assert_launch_running(process: subprocess.Popen[bytes], log_path: Path) -> None:
    return_code = process.poll()
    if return_code is not None:
        pytest.fail(
            f'mock launch exited early with code {return_code}\n'
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


def _wait_for_service(
    client: Client,
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    _wait_for(
        lambda: client.wait_for_service(timeout_sec=0.0),
        timeout_sec=60.0,
        node=node,
        process=process,
        log_path=log_path,
        description=client.srv_name,
    )


def _wait_for_action(
    client: ActionClient,
    action_name: str,
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    _wait_for(
        lambda: client.wait_for_server(timeout_sec=0.0),
        timeout_sec=60.0,
        node=node,
        process=process,
        log_path=log_path,
        description=action_name,
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


def _call_service(
    client: Client,
    request,
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
):
    return _future_result(
        client.call_async(request),
        timeout_sec=20.0,
        node=node,
        process=process,
        log_path=log_path,
        description=client.srv_name,
    )


def _robot_state(positions: dict[str, float]) -> RobotState:
    state = RobotState()
    state.joint_state.name = list(ALL_JOINTS)
    state.joint_state.position = [positions.get(name, 0.0) for name in ALL_JOINTS]
    return state


def _positions_by_name(state: RobotState) -> dict[str, float]:
    return dict(
        zip(
            state.joint_state.name,
            state.joint_state.position,
            strict=True,
        )
    )


def _compute_fk(
    client: Client,
    side: str,
    state: RobotState,
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
):
    request = GetPositionFK.Request()
    request.header.frame_id = 'base_link'
    request.fk_link_names = [f'{side}_gripper_frame']
    request.robot_state = state
    response = _call_service(
        client,
        request,
        node=node,
        process=process,
        log_path=log_path,
    )
    assert response.error_code.val == MoveItErrorCodes.SUCCESS
    assert response.fk_link_names == [f'{side}_gripper_frame']
    assert len(response.pose_stamped) == 1
    return response.pose_stamped[0]


def _solve_position_ik(
    client: Client,
    side: str,
    seed: RobotState,
    target_pose,
    orientation_xyzw: tuple[float, float, float, float],
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> RobotState:
    request = GetPositionIK.Request()
    request.ik_request.group_name = f'{side}_arm'
    request.ik_request.ik_link_name = f'{side}_gripper_frame'
    request.ik_request.robot_state = seed
    request.ik_request.avoid_collisions = True
    request.ik_request.pose_stamped.header.frame_id = 'base_link'
    request.ik_request.pose_stamped.pose.position = target_pose.pose.position
    orientation = request.ik_request.pose_stamped.pose.orientation
    orientation.x, orientation.y, orientation.z, orientation.w = orientation_xyzw
    request.ik_request.timeout.sec = 5
    response = _call_service(
        client,
        request,
        node=node,
        process=process,
        log_path=log_path,
    )
    assert response.error_code.val == MoveItErrorCodes.SUCCESS
    return response.solution


def _assert_state_valid(
    client: Client,
    state: RobotState,
    group_name: str,
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    request = GetStateValidity.Request()
    request.robot_state = state
    request.group_name = group_name
    response = _call_service(
        client,
        request,
        node=node,
        process=process,
        log_path=log_path,
    )
    assert response.valid, [
        (contact.contact_body_1, contact.contact_body_2)
        for contact in response.contacts
    ]


def _execute_joint_goal(
    client: ActionClient,
    side: str,
    positions: dict[str, float],
    *,
    node: Node,
    process: subprocess.Popen[bytes],
    log_path: Path,
) -> None:
    goal = MoveGroup.Goal()
    goal.request.group_name = f'{side}_arm'
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.2
    goal.request.max_acceleration_scaling_factor = 0.2

    constraints = Constraints()
    constraints.name = f'{side}_resolved_ik_goal'
    for joint_name in _arm_joints(side):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = positions[joint_name]
        constraint.tolerance_above = 1e-4
        constraint.tolerance_below = 1e-4
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)
    goal.request.goal_constraints = [constraints]
    goal.request.start_state.is_diff = True
    goal.planning_options.plan_only = False
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    goal_handle = _future_result(
        client.send_goal_async(goal),
        timeout_sec=20.0,
        node=node,
        process=process,
        log_path=log_path,
        description=f'{side} MoveGroup goal acceptance',
    )
    assert goal_handle.accepted
    wrapped_result = _future_result(
        goal_handle.get_result_async(),
        timeout_sec=60.0,
        node=node,
        process=process,
        log_path=log_path,
        description=f'{side} MoveGroup execution result',
    )
    assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS
    trajectory = wrapped_result.result.planned_trajectory.joint_trajectory
    assert set(trajectory.joint_names) == set(_arm_joints(side))
    assert trajectory.points


def _stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5.0)


def test_per_arm_position_ik_plan_and_execute() -> None:
    domain_id = 60 + os.getpid() % 120
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    with tempfile.TemporaryDirectory(prefix='cleany-moveit-smoke-') as temp_dir:
        temp_path = Path(temp_dir)
        os.environ['ROS_HOME'] = str(temp_path / 'ros-home')
        log_path = temp_path / 'mock_planning.log'
        log_stream = log_path.open('wb')
        process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'cleany_moveit_config',
                'mock_planning.launch.py',
                'use_rviz:=false',
            ],
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        rclpy.init()
        node = rclpy.create_node(f'cleany_moveit_smoke_{uuid4().hex[:8]}')
        latest_joint_positions: dict[str, float] = {}

        def joint_state_callback(message: JointState) -> None:
            latest_joint_positions.update(
                zip(message.name, message.position, strict=True)
            )

        node.create_subscription(
            JointState,
            '/joint_states',
            joint_state_callback,
            qos_profile_sensor_data,
        )
        ik_client = node.create_client(GetPositionIK, '/compute_ik')
        fk_client = node.create_client(GetPositionFK, '/compute_fk')
        validity_client = node.create_client(
            GetStateValidity, '/check_state_validity'
        )
        move_group_client = ActionClient(node, MoveGroup, '/move_action')
        trajectory_clients = {
            side: ActionClient(
                node,
                FollowJointTrajectory,
                f'/{side}_arm_controller/follow_joint_trajectory',
            )
            for side in ('left', 'right')
        }

        try:
            for client in (ik_client, fk_client, validity_client):
                _wait_for_service(
                    client, node=node, process=process, log_path=log_path
                )
            _wait_for_action(
                move_group_client,
                '/move_action',
                node=node,
                process=process,
                log_path=log_path,
            )
            for side, client in trajectory_clients.items():
                _wait_for_action(
                    client,
                    f'/{side}_arm_controller/follow_joint_trajectory',
                    node=node,
                    process=process,
                    log_path=log_path,
                )

            _wait_for(
                lambda: set(ALL_JOINTS) <= set(latest_joint_positions),
                timeout_sec=30.0,
                node=node,
                process=process,
                log_path=log_path,
                description='all modeled /joint_states',
            )

            home_state = _robot_state({})
            _assert_state_valid(
                validity_client,
                home_state,
                '',
                node=node,
                process=process,
                log_path=log_path,
            )

            for side, wrist_roll_seed in (('left', 0.35), ('right', -0.35)):
                opposite_side = 'right' if side == 'left' else 'left'
                opposite_before = {
                    name: latest_joint_positions[name]
                    for name in _arm_joints(opposite_side)
                }
                target = _compute_fk(
                    fk_client,
                    side,
                    home_state,
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                seed = _robot_state(
                    {f'{side}_wrist_roll_joint': wrist_roll_seed}
                )
                identity_solution = _solve_position_ik(
                    ik_client,
                    side,
                    seed,
                    target,
                    (0.0, 0.0, 0.0, 1.0),
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                rotated_solution = _solve_position_ik(
                    ik_client,
                    side,
                    seed,
                    target,
                    (1.0, 0.0, 0.0, 0.0),
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                identity_positions = _positions_by_name(identity_solution)
                rotated_positions = _positions_by_name(rotated_solution)
                assert identity_positions[f'{side}_wrist_roll_joint'] == (
                    pytest.approx(wrist_roll_seed, abs=1e-6)
                )
                for joint_name in _arm_joints(side):
                    assert identity_positions[joint_name] == pytest.approx(
                        rotated_positions[joint_name], abs=1e-6
                    )

                _assert_state_valid(
                    validity_client,
                    identity_solution,
                    f'{side}_arm',
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                solved_fk = _compute_fk(
                    fk_client,
                    side,
                    identity_solution,
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                target_position = target.pose.position
                solved_position = solved_fk.pose.position
                position_error = math.sqrt(
                    (target_position.x - solved_position.x) ** 2
                    + (target_position.y - solved_position.y) ** 2
                    + (target_position.z - solved_position.z) ** 2
                )
                assert position_error < 1e-4

                _execute_joint_goal(
                    move_group_client,
                    side,
                    identity_positions,
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                _wait_for(
                    lambda: all(
                        abs(
                            latest_joint_positions.get(joint_name, math.inf)
                            - identity_positions[joint_name]
                        )
                        < 0.02
                        for joint_name in _arm_joints(side)
                    ),
                    timeout_sec=20.0,
                    node=node,
                    process=process,
                    log_path=log_path,
                    description=f'{side} feedback convergence',
                )
                for joint_name, previous_position in opposite_before.items():
                    assert latest_joint_positions[joint_name] == pytest.approx(
                        previous_position, abs=1e-6
                    )

                feedback_state = _robot_state(latest_joint_positions)
                _assert_state_valid(
                    validity_client,
                    feedback_state,
                    '',
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                feedback_fk = _compute_fk(
                    fk_client,
                    side,
                    feedback_state,
                    node=node,
                    process=process,
                    log_path=log_path,
                )
                feedback_position = feedback_fk.pose.position
                feedback_error = math.sqrt(
                    (target_position.x - feedback_position.x) ** 2
                    + (target_position.y - feedback_position.y) ** 2
                    + (target_position.z - feedback_position.z) ** 2
                )
                assert feedback_error < 1e-3
        finally:
            node.destroy_node()
            rclpy.shutdown()
            _stop_process(process)
            log_stream.close()
