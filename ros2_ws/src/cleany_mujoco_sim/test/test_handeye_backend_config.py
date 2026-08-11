from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
ARM_JOINT_SUFFIXES = (
    'shoulder_yaw_joint',
    'shoulder_pitch_joint',
    'elbow_pitch_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)


def _arm_joints(side: str) -> list[str]:
    return [f'{side}_{suffix}' for suffix in ARM_JOINT_SUFFIXES]


def test_handeye_controllers_claim_disjoint_arm_joints() -> None:
    config = yaml.safe_load(
        (
            PACKAGE_ROOT / 'config' / 'handeye_ros2_controllers.yaml'
        ).read_text(encoding='utf-8')
    )

    manager = config['controller_manager']['ros__parameters']
    assert manager['update_rate'] == 100
    assert manager['joint_state_broadcaster']['type'] == (
        'joint_state_broadcaster/JointStateBroadcaster'
    )

    claimed_joints = {}
    for side in ('left', 'right'):
        controller_name = f'{side}_arm_controller'
        assert manager[controller_name]['type'] == (
            'joint_trajectory_controller/JointTrajectoryController'
        )
        parameters = config[controller_name]['ros__parameters']
        claimed_joints[side] = set(parameters['joints'])
        assert parameters['joints'] == _arm_joints(side)
        assert parameters['command_interfaces'] == ['position']
        assert parameters['state_interfaces'] == ['position', 'velocity']
        assert parameters['allow_partial_joints_goal'] is False
        assert parameters['open_loop_control'] is False
        assert parameters['allow_nonzero_velocity_at_trajectory_end'] is False
        assert parameters['constraints']['stopped_velocity_tolerance'] == 0.01
        for joint_name in parameters['joints']:
            assert parameters['constraints'][joint_name] == {
                'trajectory': 0.05,
                'goal': 0.01,
            }

    assert claimed_joints['left'].isdisjoint(claimed_joints['right'])
    assert all(
        'gripper' not in name
        for names in claimed_joints.values()
        for name in names
    )


def test_handeye_backend_does_not_compose_the_custom_simulator() -> None:
    launch_path = PACKAGE_ROOT / 'launch' / 'handeye_backend.launch.py'
    launch_source = launch_path.read_text(encoding='utf-8')
    tree = ast.parse(launch_source)

    imported_names = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert 'IncludeLaunchDescription' not in imported_names
    assert 'mujoco_sim_node' not in launch_source
    assert "package='mujoco_ros2_control'" in launch_source
    assert "executable='ros2_control_node'" in launch_source
