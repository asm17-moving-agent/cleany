import importlib.util
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


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


def test_sorting_wrapper_uses_yoloe_seg_gemini_with_wrist():
    from launch.actions import IncludeLaunchDescription

    path = (Path(__file__).resolve().parents[1] / 'launch' /
            'study_cafe_sorting.launch.py')
    spec = importlib.util.spec_from_file_location('sorting_wrapper', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    include = next(entity for entity in module.generate_launch_description().entities
                   if isinstance(entity, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments['perception_detector_type'] == 'yoloe_gemini'
    assert arguments['perception_segmenter_type'] == 'yoloe_seg'
    assert arguments['sorting_use_wrist_camera'] == 'true'


def test_study_cafe_simulation_defaults_to_fine_tuned_yoloe(runtime):
    _, context = runtime
    assert context.launch_configurations['yoloe_model_path'] == (
        'yoloe/study_cafe_sim_yoloe26s_seg.pt'
    )
    assert context.launch_configurations['wrist_yoloe_model_path'] == (
        'yoloe/study_cafe_sim_all_views_yoloe26s_seg.pt'
    )
    assert context.launch_configurations['wrist_right_yoloe_model_path'] == (
        'yoloe/study_cafe_sim_yoloe26s_seg.pt'
    )
    assert context.launch_configurations['wrist_check_yoloe_model_path'] == (
        'yoloe/study_cafe_sim_held_yoloe26s_seg.pt'
    )
    assert context.launch_configurations['wrist_mouse_check_yoloe_model_path'] == (
        'yoloe/study_cafe_sim_left_held_mouse_v2_yoloe26s_seg.pt'
    )
    assert context.launch_configurations['wrist_right_check_yoloe_model_path'] == (
        'yoloe/study_cafe_sim_right_held_cup_v2_yoloe26s_seg.pt')
    assert context.launch_configurations['sorting_release_edge_margin_m'] == '0.005'
    assert context.launch_configurations[
        'wrist_right_check_yoloe_class_confidence_thresholds'] == (
            '[0.08, 0.25, 0.25, 0.08]')
    assert context.launch_configurations[
        'perception_minimum_detection_confidence'
    ] == '0.08'
    assert context.launch_configurations['yoloe_class_confidence_thresholds'] == (
        '[0.25, 0.25, 0.25, 0.08]'
    )


@pytest.mark.parametrize('simulator,sorting,known', [
    ('true', 'true', True), ('false', 'true', False),
    ('true', 'false', False), ('false', 'false', False),
])
def test_known_geometry_default_is_scoped_to_simulation_sorting(
        runtime, simulator, sorting, known):
    module, context = runtime
    context.launch_configurations.update(start_simulator=simulator, sorting_mode=sorting)
    del context.launch_configurations['depth_octomap_plugin']
    argument = next(entity for entity in module.generate_launch_description().entities
                    if isinstance(entity, DeclareLaunchArgument)
                    and entity.name == 'depth_octomap_plugin')
    argument.execute(context)
    expected = ('cleany_scene_mapping/KnownGeometryOctomapUpdater' if known else
                'occupancy_map_monitor/PointCloudOctomapUpdater')
    assert context.launch_configurations['depth_octomap_plugin'] == expected


@pytest.mark.parametrize('simulator,sorting,depth,verify', [
    ('true', 'true', '0.004', 'false'),
    ('false', 'true', '0.0', 'true'),
    ('true', 'false', '0.0', 'true'),
])
def test_operator_observation_and_tissue_depth_are_simulation_scoped(runtime, simulator, sorting, depth, verify):
    module, context = runtime
    context.launch_configurations.update(start_simulator=simulator, sorting_mode=sorting)
    for name, expected in [('tissue_grasp_extra_depth_m', depth), ('sorting_verify_placement', verify)]:
        context.launch_configurations.pop(name)
        argument = next(action for action in module.generate_launch_description().entities
                        if isinstance(action, DeclareLaunchArgument) and action.name == name)
        argument.execute(context)
        assert context.launch_configurations[name] == expected


def test_no_arguments_selects_yoloe_seg_gemini_plan_mode(runtime):
    _, context = runtime
    values = context.launch_configurations
    assert values['perception_detector_type'] == 'yoloe_gemini'
    assert values['perception_segmenter_type'] == 'yoloe_seg'
    assert values['sorting_contact_diagnostics'] == 'false'
    assert values['sim_performance_profile'] == 'baseline'
    assert values['sensor_scene'] == values['plan_only'] == 'true'
    assert values['depth_octomap_plugin'] == (
        'occupancy_map_monitor/PointCloudOctomapUpdater'
    )
    assert values['perception_device'] == 'auto'
    assert values['depth_image_topic'] == (
        '/camera/aligned_depth_to_color/image_raw'
    )




def test_external_perception_skips_only_local_models_and_credentials(runtime, monkeypatch):
    from launch_ros.actions import Node
    module, context = runtime
    context.launch_configurations.update(start_perception='false', yoloe_model_path='/missing/model.pt')
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    assert module._preflight(context) == []
    nodes = [item for item in module.generate_launch_description().entities
             if isinstance(item, Node) and item.node_package == 'cleany_perception'
             and item.condition is not None]
    assert any(not item.condition.evaluate(context) for item in nodes)
    context.launch_configurations['use_sim_time'] = 'false'
    with pytest.raises(RuntimeError, match='MuJoCo requires'):
        module._preflight(context)


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




def test_default_yoloe_gemini_preflight_uses_local_yoloe_assets(runtime, tmp_path, monkeypatch):
    module, context = runtime
    monkeypatch.setenv('GEMINI_API_KEY', 'unit-test-placeholder-not-a-real-key')
    model_dir = tmp_path / 'yoloe'
    model_dir.mkdir()
    (model_dir / 'study_cafe_sim_yoloe26s_seg.pt').touch()
    (model_dir / 'mobileclip2_b.ts').touch()
    context.launch_configurations['model_directory'] = str(tmp_path)
    assert module._preflight(context) == []


def test_sorting_preflight_requires_separate_wrist_yoloe_checkpoint(runtime, tmp_path, monkeypatch):
    module, context = runtime
    monkeypatch.setenv('GEMINI_API_KEY', 'unit-test-placeholder-not-a-real-key')
    model_dir = tmp_path / 'yoloe'
    model_dir.mkdir()
    (model_dir / 'study_cafe_sim_yoloe26s_seg.pt').touch()
    (model_dir / 'mobileclip2_b.ts').touch()
    context.launch_configurations.update(model_directory=str(tmp_path),
        sorting_mode='true', plan_only='false', sorting_use_wrist_camera='true')
    with pytest.raises(ValueError, match='yoloe_model_path not found'):
        module._preflight(context)
    (model_dir / 'study_cafe_sim_all_views_yoloe26s_seg.pt').touch()
    with pytest.raises(ValueError, match='yoloe_model_path not found'):
        module._preflight(context)
    (model_dir / 'study_cafe_sim_held_yoloe26s_seg.pt').touch()
    (model_dir / 'study_cafe_sim_right_held_cup_v2_yoloe26s_seg.pt').touch()
    (model_dir / 'study_cafe_sim_left_held_mouse_v2_yoloe26s_seg.pt').touch()
    assert module._preflight(context) == []


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




@pytest.mark.parametrize('simulator,sorting,expected', [
    ('true', 'true', '0.018'),
    ('false', 'true', '0.016'),
    ('true', 'false', '0.016'),
    ('false', 'false', '0.016'),
])
def test_deeper_base_grasp_is_simulation_sorting_only(runtime, simulator, sorting, expected):
    module, context = runtime
    context.launch_configurations.update(start_simulator=simulator, sorting_mode=sorting)
    context.launch_configurations.pop('grasp_approach_offset_m')
    argument = next(action for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)
                    and action.name == 'grasp_approach_offset_m')
    argument.execute(context)
    assert context.launch_configurations['grasp_approach_offset_m'] == expected
    # A caller can restore the previous value without changing other offsets.
    context.launch_configurations['grasp_approach_offset_m'] = '0.016'
    argument.execute(context)
    assert context.launch_configurations['grasp_approach_offset_m'] == '0.016'


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


@pytest.mark.parametrize('argument,custom', [
    ('depth_octomap_plugin', 'cleany_scene_mapping/KnownGeometryOctomapUpdater'),
    ('sim_performance_profile', 'tabletop_fast'),
])
def test_backend_options_are_forwarded(runtime, argument, custom):
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    module, context = runtime
    context.launch_configurations[argument] = custom
    includes = [entity for entity in module.generate_launch_description().entities
                if isinstance(entity, IncludeLaunchDescription)]
    backend = next(entity for entity in includes
                   if argument in dict(entity.launch_arguments))
    value = dict(backend.launch_arguments)[argument]
    assert perform_substitutions(context, [value]) == custom


@pytest.mark.parametrize('sorting,wrist', [('false', 'false'), ('true', 'false'), ('true', 'true')])
def test_resolved_nodes_share_grasp_geometry_and_mode_guards(runtime, monkeypatch, sorting, wrist):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    module, context = runtime
    context.launch_configurations.update(sorting_mode=sorting, sorting_use_wrist_camera=wrist,
                                        grasp_approach_offset_m='0.019')
    parameters = {}
    original = module.Node

    def capture(**kwargs):
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(kwargs['executable']))
        values = [p for p in kwargs.get('parameters', []) if isinstance(p, dict)]
        parameters[executable] = {key: value for p in evaluate_parameters(context, normalize_parameters(values))
                                  for key, value in p.items()}
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', capture)
    module.generate_launch_description()
    selector = parameters['grasp_selection_server']
    coordinator = parameters['sorting_coordinator' if sorting == 'true' else 'nearest_pregrasp_coordinator']
    assert selector['grasp_approach_offset_m'] == coordinator['selector_grasp_approach_offset_m'] == .019
    for key in ('deeper_grasp_labels', 'deeper_grasp_offsets_m', 'support_patch_margin_m'):
        assert selector[key] == coordinator[key]
    for key in ('require_open_grasp_clearance', 'require_gripper_closure_clearance'):
        assert selector[key] is (sorting == 'true')
    assert selector['require_pregrasp_visibility'] is (sorting == 'true' and wrist == 'false')
    assert coordinator['use_seeded_cartesian_grasp'] is (sorting == 'true')
    assert parameters['grasp_server']['geometric.defer_support_plane_collision'] is (sorting == 'true')
