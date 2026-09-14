import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
import pytest
import yaml


"""Launch contract checks without starting ROS nodes or loading model weights."""


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


def test_large_rgbd_profile_keeps_udp_and_bounded_shared_memory_without_qos_override():
    source = Path(__file__).parents[1] / 'config' / 'fastdds_rgbd.xml'
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(source).getroot()
    transports = root.findall('d:transport_descriptors/d:transport_descriptor', ns)
    by_type = {item.findtext('d:type', namespaces=ns): item for item in transports}
    assert set(by_type) == {'UDPv4', 'SHM'}
    assert int(by_type['SHM'].findtext('d:segment_size', namespaces=ns)) == 16*1024*1024
    participant = root.find('d:participant', ns)
    assert participant.attrib['is_default_profile'] == 'true'
    assert participant.findtext('d:rtps/d:useBuiltinTransports', namespaces=ns) == 'false'
    assert len(participant.findall('d:rtps/d:userTransports/d:transport_id', ns)) == 2
    assert not root.findall('.//d:reliability', ns) and not root.findall('.//d:history', ns)
