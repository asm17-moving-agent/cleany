import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


"""Launch contract checks without starting ROS nodes or loading model weights."""


@pytest.mark.parametrize('wrist', ['true', 'false'])
def test_standalone_uses_shared_yoloe_profile(monkeypatch, wrist):
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
    context.launch_configurations.update(use_sim_time='true', device='cuda:0',
                                        enable_wrist_observation=wrist)
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    profile = Path(context.launch_configurations['model_profile'])
    settings = yaml.safe_load(profile.read_text())['perception_inspector']['ros__parameters']
    assert profile.name == 'yoloe_seg_gemini.yaml'
    assert settings['gemini_model'] == 'gemini-3.1-flash-lite'
    values = captured[0]['parameters'][-1]
    assert values['use_sim_time'].evaluate(context) is True
    assert values['enable_wrist_observation'].evaluate(context) is (wrist == 'true')
    assert settings['detector_type'] == 'yoloe_gemini'
    assert settings['segmenter_type'] == 'yoloe_seg'
    assert values['yoloe_device'].perform(context) == 'cuda:0'




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
