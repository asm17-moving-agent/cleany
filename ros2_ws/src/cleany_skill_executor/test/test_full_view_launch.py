import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


@pytest.mark.parametrize('restricted_scene', [False, True])
def test_hardware_scene_roi_is_independent_of_target_roi(tmp_path, monkeypatch, restricted_scene):
    package = Path(__file__).resolve().parents[1]
    repo = package.parents[2]
    spec = importlib.util.spec_from_file_location('full_view_hardware_support',
        package/'launch/pregrasp_hardware_support.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'prepare_state_profile',
        lambda *args: SimpleNamespace(camera_serial='synthetic-camera'))
    monkeypatch.setattr(module, 'get_package_share_directory',
        lambda name: str(package.parent/name))
    profile = tmp_path/'profile.json'
    profile.write_text(json.dumps({'rgbd_bridge': 'configs/robot/rgbd_bridge.json'}))
    context = LaunchContext()
    context.launch_configurations.update(repository_root=str(repo), profile_file=str(profile))
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations['workspace_config_file'] = str(repo/'configs/robot/plan_only_box_workspace.yaml')
    if restricted_scene:
        scene = tmp_path/'scene.yaml'
        scene.write_text(yaml.safe_dump({'depth_scene_node': {'ros__parameters': {
            'workspace_roi_pixels': [10., 20., 600., 450.]}}}))
        context.launch_configurations['scene_config_file'] = str(scene)
    recorded = {}
    def record(**kwargs):
        recorded[kwargs['executable']] = kwargs
        return Node(**kwargs)
    monkeypatch.setattr(module, 'Node', record)
    module.setup(context)
    parameters = {}
    for value in recorded['depth_scene_node']['parameters']:
        if isinstance(value, str):
            parameters.update(yaml.safe_load(Path(value).read_text())['depth_scene_node']['ros__parameters'])
        else:
            parameters.update(value)
    assert parameters['workspace_roi_pixels'] == (
        [10., 20., 600., 450.] if restricted_scene else [-1.]*4)
    assert str(repo/'configs/robot/plan_only_box_workspace.yaml') in recorded['validate_pick_server']['parameters']
    assert parameters['pixel_stride'] == 4
