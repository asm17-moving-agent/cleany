"""Launch contract checks without starting ROS nodes or loading model weights."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
import pytest
import yaml


@pytest.mark.parametrize('tracking', ['true', 'false'])
def test_standalone_uses_shared_profile_and_tracking_settings(monkeypatch, tracking):
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('learned_launch', package / 'launch/learned_rgbd.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(package))
    captured = []
    original = module.Node

    def node(**kwargs):
        captured.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', node)
    context = LaunchContext()
    context.launch_configurations.update(sam2_tracking_enabled=tracking, use_sim_time='true', device='cuda:0')
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
        elif isinstance(entity, OpaqueFunction):
            for argument in entity.execute(context):
                argument.execute(context)
    profile = Path(context.launch_configurations['model_profile'])
    settings = yaml.safe_load(profile.read_text())['perception_inspector']['ros__parameters']
    assert profile.name == 'gemini_flash_lite_sam2_tiny.yaml'
    assert settings['gemini_model'] == 'gemini-3.1-flash-lite'
    assert context.launch_configurations['sam2_checkpoint'] == settings['sam2_checkpoint']
    assert context.launch_configurations['sam2_model_config'] == settings['sam2_model_config']
    values = captured[0]['parameters'][-1]
    for key in ('enable_reference_observation', 'enable_wrist_observation', 'wrist_continuous_tracking'):
        assert isinstance(values[key], ParameterValue)
        assert values[key].evaluate(context) is (tracking == 'true')
    assert values['use_sim_time'].evaluate(context) is True
    assert values['sam2_device'].perform(context) == 'cuda:0'


def test_selected_profile_checkpoint_is_not_overridden_by_tiny_defaults(tmp_path):
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('custom_learned_launch', package / 'launch/learned_rgbd.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({'perception_inspector': {'ros__parameters': {
        'sam2_checkpoint': 'custom/model.pt', 'sam2_model_config': 'custom/config.yaml'}}}))
    context = LaunchContext()
    context.launch_configurations['model_profile'] = str(profile)
    for argument in module._profile_defaults(context):
        argument.execute(context)
    assert context.launch_configurations['sam2_checkpoint'] == 'custom/model.pt'
    assert context.launch_configurations['sam2_model_config'] == 'custom/config.yaml'


def test_native_install_guide_pins_the_container_sam2_commit():
    root = Path(__file__).parents[4]
    dockerfile = (root / 'containers/vision/Dockerfile').read_text()
    commit = next(line.split('=', 1)[1] for line in dockerfile.splitlines()
                  if line.startswith('ARG SAM2_COMMIT='))
    assert f'checkout {commit}' in (root / 'docs/DEVELOPMENT_SETUP.md').read_text()
