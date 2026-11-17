"""Verify head camera sharing without launching cameras or controllers."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


@pytest.mark.parametrize('shared', [False, True])
def test_shared_head_camera_keeps_gpu_selector_and_omits_duplicate_sensor_nodes(monkeypatch, tmp_path, shared):
    package = Path(__file__).resolve().parents[1]
    root = package.parents[2]
    spec = importlib.util.spec_from_file_location('shared_head_launch',
        package/'launch/pregrasp_hardware_support.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'prepare_state_profile', lambda *args: SimpleNamespace(camera_serial='TEST'))
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
        motion_backend='curobo_ik_moveit', motion_runtime='container', grasp_predictor='geometric',
        start_head_camera='false' if shared else 'true')
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    module.setup(context)
    assert len(cameras) == int(not shared)
    assert sum(r['executable']=='rgbd_frame_bridge' for r in records) == int(not shared)
    selector = next(r for r in records if r['executable']=='physical_grasp_selection_server')
    assert 'launch-motion-container' in selector['prefix'].perform(context)
    assert not any(r['executable']=='ros2_control_node' for r in records)
