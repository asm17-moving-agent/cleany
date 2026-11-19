"""Check launch defaults and manual motion policy without starting hardware."""
import json
from pathlib import Path
import runpy
import sys

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction


@pytest.mark.parametrize('arguments, expected', [
    ({}, 'omni'),
    ({'motion_profile': 'longitudinal'}, 'longitudinal'),
    ({'phone_motion_profile': 'longitudinal'}, 'longitudinal'),
])
def test_hardware_launch_selects_motion_profile(arguments, expected):
    launch_file = Path(__file__).resolve().parents[1]/'launch/hardware_mapping.launch.py'
    description = runpy.run_path(str(launch_file))['generate_launch_description']()
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name in ('motion_profile', 'phone_motion_profile'):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            action.execute(context)
            break
    assert context.launch_configurations['motion_profile'] == expected


@pytest.mark.parametrize('arguments, forward_only', [
    ([], False),
    (['--motion-profile', 'longitudinal'], True),
    (['--phone-motion-profile', 'longitudinal'], True),
    (['--motion-profile', 'omni'], False),
])
def test_runtime_manual_motion_follows_selected_profile(tmp_path, monkeypatch, arguments, forward_only):
    import rclpy
    from cleany_bringup import hardware_runtime

    class BeforeROSStart(Exception):
        pass

    def stop_before_ros(**kwargs):
        raise BeforeROSStart

    map_file = tmp_path/'map.yaml'
    map_file.touch()
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    monkeypatch.setattr(rclpy, 'init', stop_before_ros)
    monkeypatch.setattr(hardware_runtime, 'load_profile', lambda *a, **kw: {
        'state_root': str(tmp_path/'state'), 'base_config': 'unused', 'base_settings': {},
        'logger_config': str(tmp_path/'logger.yaml'), 'recordings_directory': str(tmp_path/'recordings'),
    })
    monkeypatch.setattr(sys, 'argv', ['hardware_runtime', '--profile', 'unused',
        '--navigation-map', str(map_file), '--robot-params', str(map_file), *arguments])
    with pytest.raises(BeforeROSStart):
        hardware_runtime.main()
    assert json.loads((tmp_path/'state/teleop-motion.json').read_text()) == {'forward_only': forward_only}


def test_runtime_rejects_conflicting_alias_before_loading_hardware(monkeypatch):
    from cleany_bringup import hardware_runtime

    def unexpected_load(*args, **kwargs):
        pytest.fail('Hardware profile must not be loaded for conflicting arguments')

    monkeypatch.setattr(hardware_runtime, 'load_profile', unexpected_load)
    monkeypatch.setattr(sys, 'argv', ['hardware_runtime', '--profile', 'unused',
        '--motion-profile', 'omni', '--phone-motion-profile', 'longitudinal'])
    with pytest.raises(SystemExit) as error:
        hardware_runtime.main()
    assert error.value.code == 2
