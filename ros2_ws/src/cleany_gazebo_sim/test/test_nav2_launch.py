"""ROS 프로세스를 시작하지 않고 Gazebo overlay의 실제 launch 전달값을 검사합니다."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from cleany_navigation.runtime_config import load_navigation_profile

PACKAGES = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('name', ['amcl_nav2', 'amcl_nav2_safety'])
def test_gazebo_launch_passes_complete_parameters_and_explicit_overlay(monkeypatch, tmp_path, name):
    path = PACKAGES/'cleany_gazebo_sim/launch'/f'{name}.launch.py'
    spec = importlib.util.spec_from_file_location('gazebo_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(PACKAGES/package))
    monkeypatch.setattr(module, 'mkdtemp', lambda **kw: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations["map"] = str(tmp_path/"map.yaml")
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    overlay = tmp_path/'custom.yaml'
    overlay.write_text('controller_server:\n  ros__parameters:\n    controller_frequency: 13.0\n')
    context.launch_configurations['params_file'] = str(overlay)
    include = next(action for action in module._launch(context) if isinstance(action, IncludeLaunchDescription))
    arguments = {key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for key, value in include.launch_arguments}
    merged = yaml.safe_load(Path(arguments['params_file']).read_text())
    assert merged == load_navigation_profile(PACKAGES/'cleany_navigation/config/nav2_amcl.yaml', overlay)
    assert merged['controller_server']['ros__parameters']['controller_frequency'] == 13.
    assert 'amcl' in merged and 'global_costmap' in merged
    if name == 'amcl_nav2_safety':
        assert Path(arguments['robot_params_file']).is_file()
        assert arguments['safety_params_file'] == str(PACKAGES/'cleany_navigation/config/nav2_safety.yaml')
