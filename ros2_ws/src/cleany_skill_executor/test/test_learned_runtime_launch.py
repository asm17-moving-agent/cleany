import importlib.util
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


@pytest.mark.parametrize('parameter', [
    'state_validity_timeout_sec', 'fk_timeout_sec',
    'ik_response_margin_sec', 'planning_response_margin_sec'])
def test_sorting_selector_collision_rpc_has_cpu_response_margin(parameter):
    source = (Path(__file__).resolve().parents[1] / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    assert f"'{parameter}': ParameterValue(PythonExpression([\n" in source
    setting = source.split(f"'{parameter}':", 1)[1].split('\n', 2)
    assert "5.0 if" in setting[1]
    assert "sorting_mode" in setting[1]
    assert "else 1.0" in setting[1]


def test_sorting_generates_more_geometry_without_expanding_ik_candidate_budget():
    source = (Path(__file__).resolve().parents[1] / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    setting = source.split("'geometric.maximum_candidates':", 1)[1].split('\n', 2)
    assert "96 if" in setting[1] and "sorting_mode" in setting[1]
    assert "else 24" in setting[1]
    assert "'maximum_candidates': 24," in source


@pytest.fixture
def runtime(monkeypatch):
    # LaunchContext.environment is os.environ in Humble. Register both keys
    # with monkeypatch before any SetEnvironmentVariable action executes.
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', '')
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE')
    monkeypatch.setenv('MUJOCO_GL', os.environ.get('MUJOCO_GL', 'egl'))
    path = (Path(__file__).resolve().parents[1] / 'launch' /
            'study_cafe_nearest_grasp_demo.launch.py')
    spec = importlib.util.spec_from_file_location('learned_runtime', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    original = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package:
                        '/plugin' if package == 'moveit_ros_perception'
                        else original(package))
    return module, context


def test_no_arguments_selects_learned_sensor_only_plan_mode(runtime):
    _, context = runtime
    values = context.launch_configurations
    assert values['perception_detector_type'] == 'gemini'
    assert values['gemini_model'] == 'gemini-3.1-flash-lite'
    assert values['gemini_api_key_environment'] == 'GEMINI_API_KEY'
    assert values['perception_segmenter_type'] == 'sam2'
    assert values['yoloe_model_path'].endswith('yoloe-26s-seg.pt')
    assert values['sam2_checkpoint'].endswith('sam2.1_t.pt')
    assert values['preload_models'] == 'true'
    assert values['sorting_contact_diagnostics'] == 'false'
    assert values['sorting_use_reference_observation'] == 'true'
    assert values['sorting_bins_config'].endswith('/config/robot_top_bins.yaml')
    assert values['sorting_head_reference_refresh_age_sec'] == '30.0'
    assert values['grasp_maximum_top_contact_depth_m'] == '0.0'
    assert values['sim_speed_factor'] == '1.0'
    assert values['gripper_open_position_rad'] == '1.2'
    assert values['lin_acceleration_scaling'] == '0.4'
    assert values['sensor_scene'] == values['plan_only'] == 'true'
    assert values['depth_octomap_plugin'] == (
        'occupancy_map_monitor/PointCloudOctomapUpdater'
    )
    assert values['perception_device'] == 'auto'
    assert values['depth_image_topic'] == (
        '/camera/aligned_depth_to_color/image_raw'
    )


def test_tracking_off_exposes_both_reference_and_wrist_controls(runtime):
    module, context = runtime
    for name in ('sorting_use_reference_observation', 'sorting_use_wrist_camera',
                 'sorting_async_carry_monitor', 'wrist_continuous_tracking'):
        context.launch_configurations[name] = 'false'
    source = Path(module.__file__).read_text()
    assert "'enable_reference_observation': ParameterValue(PythonExpression([" in source
    assert "'sorting_use_reference_observation': ParameterValue(LaunchConfiguration('sorting_use_reference_observation'), value_type=bool)" in source


@pytest.mark.parametrize('tracking', ['true', 'false'])
def test_single_tracking_switch_sets_all_sorting_defaults(runtime, tracking):
    module, context = runtime
    context.launch_configurations.update(sorting_mode='true', sam2_tracking_enabled=tracking)
    names = ('sorting_use_wrist_camera', 'sorting_use_reference_observation',
             'wrist_continuous_tracking', 'sorting_async_carry_monitor')
    for name in names:
        context.launch_configurations.pop(name)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name in names:
            entity.execute(context)
    assert all(context.launch_configurations[name] == tracking for name in names)


def test_external_perception_skips_only_local_models_and_credentials(runtime, monkeypatch):
    from launch_ros.actions import Node
    module, context = runtime
    context.launch_configurations.update(start_perception='false', sam2_checkpoint='/missing/model.pt')
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    assert module._preflight(context) == []
    nodes = [item for item in module.generate_launch_description().entities
             if isinstance(item, Node) and item.node_package == 'cleany_perception'
             and item.condition is not None]
    assert any(not item.condition.evaluate(context) for item in nodes)
    context.launch_configurations['use_sim_time'] = 'false'
    with pytest.raises(RuntimeError, match='MuJoCo requires'):
        module._preflight(context)


@pytest.mark.parametrize('sorting,expected', [
    ('false', ('0.2', '0.4', '2.0')),
    ('true', ('1.0', '0.8', '1.05')),
])
def test_motion_defaults_are_scoped_to_sorting(runtime, sorting, expected):
    module, context = runtime
    context.launch_configurations['sorting_mode'] = sorting
    names = ('approach_velocity_scaling', 'retreat_velocity_scaling', 'corridor_time_margin')
    for name in names:
        context.launch_configurations.pop(name)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name in names:
            entity.execute(context)
    assert tuple(context.launch_configurations[name] for name in names) == expected


@pytest.mark.parametrize('sorting', ['false', 'true'])
def test_speed_profile_overrides_base_caps_and_preserves_generic_defaults(runtime, sorting):
    module, context = runtime
    expected = {
        'velocity_scaling': (.08, .30), 'acceleration_scaling': (.08, .50),
        'sorting_payload_velocity_scaling': (.04, .24),
        'sorting_payload_acceleration_scaling': (.02, .20),
        'lin_acceleration_scaling': (.4, .8),
        'cartesian_translation_speed_m_s': (.10, .30),
        'cartesian_translation_acceleration_m_s2': (.20, .80),
        'cartesian_rotation_speed_rad_s': (.50, 1.50),
        'cartesian_joint_acceleration_rad_s2': (1., 4.),
        'gripper_motion_sec': (3., 2.),
    }
    context.launch_configurations['sorting_mode'] = sorting
    for name in expected:
        context.launch_configurations.pop(name)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name in expected:
            entity.execute(context)
    values = {name: float(context.launch_configurations[name]) for name in expected}
    assert values == {name: options[sorting == 'true'] for name, options in expected.items()}
    assert values['sorting_payload_velocity_scaling'] <= values['velocity_scaling']
    assert values['sorting_payload_acceleration_scaling'] <= values['acceleration_scaling']
    source = (Path(__file__).resolve().parents[1] / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    for name in expected:
        assert f"'{name}': ParameterValue(LaunchConfiguration('{name}'), value_type=float)" in source


def test_dds_profile_is_installed_and_applied_before_any_process(runtime):
    from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
    from launch_ros.actions import Node
    module, context = runtime
    path = Path(context.launch_configurations['fastdds_profiles_file'])
    assert path.name == 'fastdds_rgbd.xml' and path.is_file()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, (Node, IncludeLaunchDescription)):
            break
        if isinstance(entity, SetEnvironmentVariable):
            entity.execute(context)
    assert context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(path)


def test_existing_user_dds_profile_is_preserved(runtime, tmp_path):
    module, context = runtime
    custom = str(tmp_path / 'user.xml')
    context.environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = custom
    context.launch_configurations.pop('fastdds_profiles_file')
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == 'fastdds_profiles_file':
            entity.execute(context)
    assert context.launch_configurations['fastdds_profiles_file'] == custom


def test_invalid_dds_profile_fails_before_starting_backend(runtime):
    module, context = runtime
    context.launch_configurations['fastdds_profiles_file'] = '/missing/cleany-dds.xml'
    with pytest.raises(RuntimeError, match='profiles file'):
        module._preflight(context)


def test_preflight_rejects_missing_models_before_backend(runtime, tmp_path):
    module, context = runtime
    context.launch_configurations['model_directory'] = str(tmp_path)
    with pytest.raises(ValueError, match='not found'):
        module._preflight(context)


def test_gemini_preflight_rejects_missing_credentials(runtime, monkeypatch):
    module, context = runtime
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    with pytest.raises(RuntimeError, match='GEMINI_API_KEY must be set'):
        module._preflight(context)


def test_gemini_needs_only_sam2_local_assets(runtime, tmp_path, monkeypatch):
    module, context = runtime
    monkeypatch.setenv('GEMINI_API_KEY', 'unit-test-placeholder-not-a-real-key')
    (tmp_path / 'sam2').mkdir()
    (tmp_path / 'sam2/sam2.1_t.pt').touch()
    context.launch_configurations['model_directory'] = str(tmp_path)
    assert module._preflight(context) == []
    context.launch_configurations['perception_detector_type'] = 'yoloe'
    with pytest.raises(ValueError, match='yoloe_model_path not found'):
        module._preflight(context)


@pytest.mark.parametrize('field,value', [
    ('plan_only', 'false'), ('sensor_scene', 'false'),
])
def test_external_robot_rejects_demo_modes(runtime, field, value):
    module, context = runtime
    context.launch_configurations['start_simulator'] = 'false'
    context.launch_configurations[field] = value
    with pytest.raises(RuntimeError, match='External robot'):
        module._preflight(context)


def test_simulator_requires_consistent_sensor_clock(runtime):
    module, context = runtime
    context.launch_configurations['use_sim_time'] = 'false'
    with pytest.raises(RuntimeError, match='use_sim_time'):
        module._preflight(context)


def test_sorting_requires_execution_and_explicit_bins(runtime):
    module, context = runtime
    context.launch_configurations['sorting_mode'] = 'true'
    with pytest.raises(RuntimeError, match='simulation with execution'):
        module._preflight(context)
    context.launch_configurations['plan_only'] = 'false'
    context.launch_configurations['sorting_bins_config'] = ''
    with pytest.raises(RuntimeError, match='bin configuration'):
        module._preflight(context)


@pytest.mark.parametrize('wrist,tracking', [('false', 'true'), ('true', 'false')])
def test_async_monitor_requires_enabled_wrist_stream(runtime, wrist, tracking):
    module, context = runtime
    context.launch_configurations.update(sorting_mode='true', plan_only='false',
        sorting_bins_config=str(Path(__file__).parents[2] / 'cleany_mujoco_sim/config/robot_top_bins.yaml'),
        sorting_async_carry_monitor='true', sorting_use_wrist_camera=wrist,
        wrist_continuous_tracking=tracking)
    with pytest.raises(RuntimeError, match='requires wrist camera and continuous tracking'):
        module._preflight(context)

def test_tool_depth_offset_is_shared_by_selection_and_execution():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    assert "'grasp_approach_offset_m': grasp_approach_offset" in source
    assert "'selector_grasp_approach_offset_m': grasp_approach_offset" in source
    assert "DeclareLaunchArgument('grasp_approach_offset_m', default_value='0.016')" in source


def test_sorting_support_deferral_is_paired_with_robot_checks_and_local_patches():
    source = (Path(__file__).parents[1] / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    for parameter in ('geometric.defer_support_plane_collision',
                      'require_open_grasp_clearance', 'require_gripper_closure_clearance'):
        assert f"'{parameter}': ParameterValue(sorting_mode, value_type=bool)" in source
    assert source.count("'support_patch_margin_m': ParameterValue(PythonExpression([") == 2
    assert source.count('"0.02 if \'", sorting_mode, "\' == \'true\' else 0.0"]), value_type=float)') == 2


def test_only_head_sorting_requires_head_pregrasp_visibility():
    import yaml
    package = Path(__file__).parents[1]
    source = (package / 'launch' / 'study_cafe_nearest_grasp_demo.launch.py').read_text()
    assert "'require_pregrasp_visibility': ParameterValue(PythonExpression([" in source
    assert "LaunchConfiguration('sorting_use_wrist_camera'), \"' != 'true'\"" in source
    defaults = yaml.safe_load((package / 'config' / 'grasp_selection.yaml').read_text())
    assert defaults['grasp_selection_server']['ros__parameters']['require_pregrasp_visibility'] is False


def test_nominal_wrist_frames_only_start_in_simulation_sorting(runtime):
    module, context = runtime
    assert module._wrist_camera_transforms(context) == []
    context.launch_configurations['sorting_use_wrist_camera'] = 'true'
    with pytest.raises(RuntimeError, match='simulation sorting'):
        module._wrist_camera_transforms(context)
    context.launch_configurations['sorting_mode'] = 'true'
    context.launch_configurations['sorting_wrist_cameras_config'] = str(
        Path(__file__).parents[2] / 'cleany_mujoco_sim/config/sorting_wrist_cameras.yaml')
    assert len(module._wrist_camera_transforms(context)) == 2


def test_sorting_uses_joint_corridor_with_generic_mode_opt_in():
    import yaml
    package = Path(__file__).parents[1]
    source = (package / 'launch' / 'study_cafe_nearest_grasp_demo.launch.py').read_text()
    assert "'use_joint_corridor_grasp': ParameterValue(sorting_mode, value_type=bool)" in source
    assert "'use_seeded_cartesian_grasp': ParameterValue(sorting_mode, value_type=bool)" in source
    assert "'count_retreat_as_lift': ParameterValue(sorting_mode, value_type=bool)" in source
    assert "'align_grasp_wrist_roll': ParameterValue(sorting_mode, value_type=bool)" in source
    defaults = yaml.safe_load((package / 'config' / 'nearest_pregrasp.yaml').read_text())
    assert defaults['nearest_pregrasp_coordinator']['ros__parameters']['use_joint_corridor_grasp'] is False
    assert defaults['nearest_pregrasp_coordinator']['ros__parameters']['use_seeded_cartesian_grasp'] is False
    assert defaults['nearest_pregrasp_coordinator']['ros__parameters']['count_retreat_as_lift'] is False


def test_optional_octomap_plugin_is_forwarded_to_moveit(runtime):
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    module, context = runtime
    custom = 'cleany_scene_mapping/KnownGeometryOctomapUpdater'
    context.launch_configurations['depth_octomap_plugin'] = custom
    includes = [entity for entity in module.generate_launch_description().entities
                if isinstance(entity, IncludeLaunchDescription)]
    moveit = next(entity for entity in includes
                  if 'depth_octomap_plugin' in dict(entity.launch_arguments))
    value = dict(moveit.launch_arguments)['depth_octomap_plugin']
    assert perform_substitutions(context, [value]) == custom
