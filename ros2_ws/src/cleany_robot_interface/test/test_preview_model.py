"""Check preview assembly independently of its moving-joint transforms."""
import importlib.util
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import pytest


def _preview_defaults():
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        'arm_preview_launch', package / 'launch/arm_preview.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    return context.launch_configurations


def test_legacy_preview_shoulders_remain_on_fixed_motor_shafts():
    package = Path(__file__).parents[1]
    defaults = {'model_profile': 'jetson_20261009',
                'include_head_camera': 'true', 'head_mast_rear_gap_m': '0.05'}
    model = package.parent / 'cleany_description/urdf/cleany.urdf.xacro'
    root = ET.fromstring(subprocess.check_output([
        'xacro', str(model), 'model_profile:=' + defaults['model_profile'],
        'include_head_camera:=' + defaults['include_head_camera'],
        'head_mast_rear_gap_m:=' + defaults['head_mast_rear_gap_m'],
    ], text=True))
    motors = [visual for visual in root.find("link[@name='base_link']").findall('visual')
              if visual.find('geometry/mesh') is not None
              and visual.find('geometry/mesh').get('filename').endswith('/Base_Motor.stl')]
    assert len(motors) == 2
    for side in ('left', 'right'):
        motor = next(visual for visual in motors
                     if (float(visual.find('origin').get('xyz').split()[1]) > 0)
                     == (side == 'left'))
        origin = motor.find('origin')
        xyz = [float(value) for value in origin.get('xyz').split()]
        roll, pitch, yaw = [float(value) for value in origin.get('rpy').split()]
        assert abs(roll) < 1e-8 and abs(pitch) < 1e-8
        # Shaft datum in the existing Base_Motor CAD frame, also used by MJCF.
        shaft = [xyz[0] + .0452 * math.sin(yaw),
                 xyz[1] - .0452 * math.cos(yaw), xyz[2] + .0165]
        shoulder = root.find(f"joint[@name='{side}_shoulder_yaw_joint']/origin")
        assert [float(value) for value in shoulder.get('xyz').split()] == pytest.approx(
            shaft, abs=1e-8)


def test_legacy_preview_keeps_head_on_repositioned_mast():
    defaults = {'model_profile': 'jetson_20261009',
                'include_head_camera': 'true', 'head_mast_rear_gap_m': '0.05'}
    model = Path(__file__).parents[2] / 'cleany_description/urdf/cleany.urdf.xacro'
    nominal = ET.fromstring(subprocess.check_output(['xacro', str(model), 'model_profile:=' + defaults['model_profile']]))
    preview = ET.fromstring(subprocess.check_output([
        'xacro', str(model), 'model_profile:=' + defaults['model_profile'],
        'head_mast_rear_gap_m:=' + defaults['head_mast_rear_gap_m'],
    ]))
    for name in ('top_base_joint', 'head_pan_joint'):
        path = f"joint[@name='{name}']/origin"
        assert preview.find(path).attrib == nominal.find(path).attrib


def test_default_preview_includes_common_calibrated_model_without_motor_reader():
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        'arm_preview_default', package / 'launch/arm_preview.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    assert not any(isinstance(action, Node) for action in actions)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    context = LaunchContext()
    for action in includes[0].launch_description_source.get_launch_description(context).entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    assert context.launch_configurations['model_variant'] == 'freecad_calibrated'
    assert context.launch_configurations['input_joint_states'] == '/joint_states'
    assert dict(includes[0].launch_arguments)['namespace'] == 'arm_preview'
