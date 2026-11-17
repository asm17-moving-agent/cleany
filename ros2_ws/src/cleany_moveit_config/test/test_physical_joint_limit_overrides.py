"""Materialized physical position limits take precedence over stale YAML values."""
import importlib.util
from pathlib import Path

import yaml


def test_physical_limits_file_keeps_dynamics_and_uses_urdf_positions(tmp_path):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('physical_limits_launch',
        package / 'launch/physical_move_group.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = tmp_path / 'robot.urdf'
    model.write_text('''<robot name="test"><link name="base"/><link name="wrist"/>
<joint name="left_wrist_roll_joint" type="revolute"><parent link="base"/>
<child link="wrist"/><limit lower="-0.05" upper="3.09" effort="1" velocity="1"/>
</joint></robot>''')
    limits = tmp_path / 'limits.yaml'
    limits.write_text(yaml.safe_dump({'joint_limits': {'left_wrist_roll_joint': {
        'has_position_limits': True, 'min_position': -0.01, 'max_position': 0.01,
        'has_velocity_limits': True, 'max_velocity': 0.3,
        'has_acceleration_limits': True, 'max_acceleration': 0.5}}}))
    config = module._moveit_config(robot_description_file=str(model),
                                    joint_limits_file=str(limits))
    values = config.joint_limits['robot_description_planning']['joint_limits']['left_wrist_roll_joint']
    assert (values['min_position'], values['max_position']) == (-0.05, 3.09)
    assert (values['max_velocity'], values['max_acceleration']) == (0.3, 0.5)
