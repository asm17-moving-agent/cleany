"""Validate physical CPU/GPU launch wiring without starting ROS or devices."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


@pytest.mark.parametrize('backend,predictor', [
    ('moveit', 'geometric'), ('moveit', 'anygrasp'),
    ('curobo_ik_moveit', 'geometric'), ('curobo_ik_moveit', 'anygrasp'),
])
def test_hardware_launch_selects_predictor_and_keeps_shared_limits(monkeypatch, tmp_path, backend, predictor):
    package = Path(__file__).resolve().parents[1]
    root = package.parents[2]
    spec = importlib.util.spec_from_file_location(
        'hardware_support_launch', package / 'launch/pregrasp_hardware_support.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'prepare_state_profile',
                        lambda *args: SimpleNamespace(camera_serial='TEST'))
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(root / 'ros2_ws/src' / name))
    records = []
    node_class = module.Node

    def capture_node(**kwargs):
        records.append(kwargs)
        return node_class(**kwargs)

    monkeypatch.setattr(module, 'Node', capture_node)
    context = LaunchContext()
    context.launch_configurations.update(
        repository_root=str(root), profile_file=str(root / 'configs/robot/camera_preview.json'),
        output_root=str(tmp_path), motion_backend=backend, grasp_predictor=predictor,
        motion_runtime='native' if backend == 'moveit' else 'container')
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    module.setup(context)
    grasp_nodes = [item for item in records if item['executable'] == 'grasp_server']
    assert len(grasp_nodes) == int(predictor == 'geometric')
    if grasp_nodes:
        config = {}
        for path in grasp_nodes[0]['parameters']:
            config.update(yaml.safe_load(Path(path).read_text())['grasp_server']['ros__parameters'])
        assert config['predictor_type'] == 'geometric'
        assert config['anygrasp_preload'] is False
        assert config['geometric.maximum_candidates'] == 96
        assert config['geometric.defer_support_plane_collision'] is False
    selector = next(item for item in records if item['executable'] == 'physical_grasp_selection_server')
    config = {}
    for item in selector['parameters']:
        config.update(item if isinstance(item, dict) else
                      yaml.safe_load(Path(item).read_text())['grasp_selection_server']['ros__parameters'])
    assert config['motion_backend'] == backend
    assert config['position_tolerance_m'] == config['pregrasp_position_tolerance_m'] == .010
    assert config['approach_tolerance_degrees'] == config['closing_tolerance_degrees'] == 30.
    assert config['maximum_candidates'] == 32
    assert config['grasp_ik_first'] is True
    assert config['grasp_approach_offset_m'] == 0.
    assert config['pregrasp_fallback_offsets_m'] == [.06, .04]
    assert config['stop_after_first_feasible'] is True
    assert config['prefer_near_wrist_grasp_direction'] is True
    assert config['wrist_direction_tie_tolerance_degrees'] == 5.0
    assert config['grasp_direction_pair_position_tolerance_m'] == .00001
    assert config['grasp_direction_pair_axis_tolerance_degrees'] == 1.0
    assert config['require_sensor_scene'] is True
    assert config['require_gripper_closure_clearance'] is True
    assert config['asymmetric_gripper_enabled'] is True
    assert config['fixed_jaw_additional_bias_m'] == .020
    assert config['fixed_jaw_clearance_enabled'] is False
    assert config['fixed_jaw_minimum_clearance_m'] == .005
    assert config['fixed_jaw_maximum_correction_m'] == .010
    assert config['fixed_jaw_collision_meshes'] == ['Fixed_Jaw_part2.ply.convex.stl']
    assert config['validate_motion_paths'] is True
    assert config['planning_timeout_sec'] == 4.0
    assert config['planning_response_margin_sec'] == 20.0
    prefix = selector['prefix'].perform(context)
    if backend == 'moveit':
        assert prefix == ''
    else:
        assert prefix.endswith('/tools/launch-motion-container')
