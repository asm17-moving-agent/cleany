"""Verify head camera sharing without launching cameras or controllers."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


@pytest.mark.parametrize('shared,runtime', [(False, 'container'), (True, 'native')])
def test_shared_head_camera_keeps_gpu_selector_and_omits_duplicate_sensor_nodes(monkeypatch, tmp_path, shared, runtime):
    package = Path(__file__).resolve().parents[1]
    root = package.parents[2]
    spec = importlib.util.spec_from_file_location('shared_head_launch',
        package/'launch/pregrasp_hardware_support.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = root / 'ros2_ws/src/cleany_description/urdf/cleany.urdf'
    monkeypatch.setattr(module, 'prepare_state_profile',
                        lambda *args: SimpleNamespace(camera_serial='TEST', model=model))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(root/'ros2_ws/src'/name))
    records, cameras = [], []
    real_node, real_include = module.Node, module.IncludeLaunchDescription
    def node(**kwargs):
        records.append(kwargs)
        return real_node(**kwargs)
    def include(*args, **kwargs):
        cameras.append(kwargs)
        return real_include(*args, **kwargs)
    monkeypatch.setattr(module, 'Node', node)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', include)
    context = LaunchContext()
    context.launch_configurations.update(repository_root=str(root),
        profile_file=str(root/'configs/robot/camera_preview.json'), output_root=str(tmp_path),
        motion_backend='curobo_ik_moveit', motion_runtime=runtime, grasp_predictor='geometric',
        curobo_python=str(tmp_path/'curobo/bin/python'),
        start_head_camera='false' if shared else 'true')
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    module.setup(context)
    assert len(cameras) == int(not shared)
    assert sum(r['executable']=='rgbd_frame_bridge' for r in records) == int(not shared)
    selector = next(r for r in records if r['executable']=='physical_grasp_selection_server')
    prefix = selector['prefix'].perform(context)
    assert prefix == (str(tmp_path/'curobo/bin/python') if runtime == 'native'
                      else str(root/'tools/launch-motion-container'))
    assert {'curobo_sphere_model': 'cad_cached'} in selector['parameters']
    assert not any(r['executable']=='ros2_control_node' for r in records)


def test_head_rgbd_launch_starts_only_camera_and_bridge(monkeypatch):
    import json

    package = Path(__file__).resolve().parents[1]
    root = package.parents[2]
    spec = importlib.util.spec_from_file_location('head_rgbd_launch', package/'launch/head_rgbd.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    perception = root/'ros2_ws/src/cleany_perception'
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(perception))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda source, **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations['repository_root'] = str(root)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    camera, bridge = module.setup(context)
    profile = json.loads((root/'configs/robot/cad_grasp.json').read_text())
    head = json.loads((root/profile['head_config']).read_text())
    assert dict(camera['launch_arguments']) == {
        'serial_no': str(head['camera_serial']),
        'config_file': str(perception/'config/realsense_pregrasp.yaml'),
    }
    assert bridge['executable'] == 'rgbd_frame_bridge'
    assert bridge['parameters'] == [{'config_file': str(root/profile['rgbd_bridge'])}]
