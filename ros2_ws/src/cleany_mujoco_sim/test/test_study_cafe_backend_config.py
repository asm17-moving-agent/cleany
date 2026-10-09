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


def test_study_cafe_controllers_claim_disjoint_arm_joints() -> None:
    config = yaml.safe_load(
        (
            PACKAGE_ROOT / 'config' / 'study_cafe_ros2_controllers.yaml'
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
                'trajectory': 0.12,
                'goal': 0.01,
            }

    assert claimed_joints['left'].isdisjoint(claimed_joints['right'])
    assert all(
        'gripper' not in name
        for names in claimed_joints.values()
        for name in names
    )

    for side in ('left', 'right'):
        controller_name = f'{side}_gripper_controller'
        assert manager[controller_name]['type'] == (
            'joint_trajectory_controller/JointTrajectoryController'
        )
        parameters = config[controller_name]['ros__parameters']
        assert parameters['joints'] == [f'{side}_gripper_joint']
        assert parameters['command_interfaces'] == ['position']
        assert parameters['state_interfaces'] == ['position', 'velocity']


def test_xim_workaround_is_configurable_and_simulator_only() -> None:
    source = (PACKAGE_ROOT / 'launch' / 'study_cafe_backend.launch.py').read_text()
    tree = ast.parse(source)
    overrides = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if isinstance(call.func, ast.Name) and call.func.id == 'Node':
            keywords = {kw.arg: kw.value for kw in call.keywords}
            if 'additional_env' in keywords:
                overrides.append(keywords)
    assert len(overrides) == 1
    assert ast.literal_eval(overrides[0]['executable']) == 'ros2_control_node'
    env = overrides[0]['additional_env']
    assert isinstance(env, ast.Dict)
    assert [ast.literal_eval(key) for key in env.keys] == ['XMODIFIERS']
    assert ast.unparse(env.values[0]) == "LaunchConfiguration('sim_xmodifiers').perform(context)"
    assert "'sim_xmodifiers', default_value='@im=none'" in source
    assert 'SetEnvironmentVariable' not in source
