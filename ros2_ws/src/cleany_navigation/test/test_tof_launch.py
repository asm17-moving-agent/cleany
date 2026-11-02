"""Launch assembly only; robot geometry is synthetic and no processes start."""
from xml.dom.minidom import parseString
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import cleany_navigation.launch_runtime as module


@pytest.mark.parametrize('enabled,observer', [(True, False), (False, False), (True, True), (False, True)])
def test_tof_launch_preserves_independent_depth_observer(tmp_path, monkeypatch, enabled, observer):
    body = tmp_path/'body.yaml'
    body.write_text('park_joints: {head_pan_joint: 0.0}\n')
    robot = tmp_path/'robot.yaml'
    robot.write_text(yaml.safe_dump({
        'voxel_guard': {'ros__parameters': dict(profile=str(body), max_speed=.15,
            max_yaw_rate=.3, reaction_s=.25, linear_deceleration=.4, angular_deceleration=.6)},
        'depth_clearance': {'ros__parameters': dict(camera_frame='head_camera_depth_frame',
            optical_frame='head_camera_depth_optical_frame')},
        'planner_server': {'ros__parameters': {'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner'}}}}))
    map_file = tmp_path/'map.yaml'
    map_file.write_text('image: unused.pgm\n')
    merged = tmp_path/'merged'
    merged.mkdir()
    monkeypatch.setattr(module, 'mkdtemp', lambda **_k: str(merged))
    monkeypatch.setattr(module, 'validate_robot_parameters', lambda *_a, **_k: None)
    monkeypatch.setattr(module, 'forward_kinematics', lambda *_a, **_k: None)
    monkeypatch.setattr(module, 'apply_simulation_geometry', lambda _u, profile, **_k: profile)
    monkeypatch.setattr(module.xacro, 'process_file', lambda _p: parseString('<robot name="synthetic"/>'))
    monkeypatch.setattr(module, 'nav2_nodes', lambda *_a: [])
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    description = module.navigation_launch_description(safety=True)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    if observer:
        p = yaml.safe_load(robot.read_text())
        p['depth_coverage'] = {'ros__parameters': {'enabled': True, 'update_period_s': .2}}
        robot.write_text(yaml.safe_dump(p))
    context.launch_configurations.update(map=str(map_file), robot_params_file=str(robot),
                                         tof_enabled=str(enabled).lower())
    actions = module._launch(context, safety=True)
    nodes = [a for a in actions if isinstance(a, dict)]
    coverage = [n for n in nodes if n['executable'] == 'depth_coverage']
    assert len(coverage) == int(observer)
    parameters = yaml.safe_load((merged/'parameters.yaml').read_text())
    if observer:
        p = parameters['depth_coverage']['ros__parameters']
        assert 'tof_coverage_enabled' not in p and p['profile'] == str(body)
        assert p['update_period_s'] == .2
    if enabled:
        assert parameters['voxel_guard']['ros__parameters']['output_topic'] == 'proximity/input'
        assert {'tof_driver', 'proximity_guard', 'desk_approach'} <= {n['executable'] for n in nodes}
    else:
        assert not any(n['package'] == 'cleany_tof' for n in nodes)
