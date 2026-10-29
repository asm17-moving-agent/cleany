"""Deployment wiring checks; no ROS nodes, GPU allocation or device access."""
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'ros2_ws/src/cleany_skill_executor'


def test_physical_runtime_defaults_to_hybrid_and_preserves_thirty_two_candidates():
    script=(ROOT/'tools/collection-runtime').read_text()
    assert '${CLEANY_COLLECTION_MOTION_BACKEND:-curobo_ik_moveit}' in script
    assert '${CLEANY_COLLECTION_GRASP_PREDICTOR:-geometric}' in script
    assert 'maximum_candidates:=32' in script
    launch=(PACKAGE/'launch/pregrasp_hardware_support.launch.py').read_text()
    assert "config/hybrid_motion.yaml" in launch
    assert 'curobo_ik_moveit) motion_runtime=container' in script
    assert 'motion_runtime_prefix()' in launch
    assert "'motion_backend': backend" in launch
    assert "DeclareLaunchArgument('maximum_candidates', default_value='32')" in launch
    params=yaml.safe_load((PACKAGE/'config/hybrid_motion.yaml').read_text())['grasp_selection_server']['ros__parameters']
    assert params['motion_backend']=='curobo_ik_moveit'
    assert params['maximum_candidates']==32
    assert params['curobo_ik_exclude_opposite_arm_cost'] is True
    assert (params['curobo_ik_batch_size'],params['curobo_ik_primary_seeds'],
            params['curobo_ik_fallback_seeds'],params['curobo_ik_iterations'])==(8,16,32,50)
    assert params['curobo_ik_retry_failed_candidates'] is True
    assert params['curobo_ik_pregrasp_retry_passes']==0
    assert params['curobo_ik_adaptive_batch_size'] is False
    assert params['curobo_ik_progressive_pregrasp'] is True
    assert params['curobo_ik_group_gripper_openings'] is True
    assert params['curobo_ik_defer_candidate_retries'] is True
    assert params['curobo_ik_warmup_enabled'] is True
    assert params['curobo_ik_vlm_warmup_enabled'] is False
    assert params['curobo_ik_vlm_warmup_arms'] == ['right', 'left']
    assert params['curobo_ik_vlm_warmup_window_sec'] == 30.0
    assert params['curobo_ik_warmup_arms'] == ['right', 'left']
    assert params['curobo_ik_warmup_include_aim'] is False
    assert params['stop_after_first_feasible'] is True
    assert params['curobo_spheres_per_link']==64
    assert params['curobo_sphere_budget_per_link']==16
    assert params['curobo_ik_cpu_fallback'] is False
    assert params['curobo_planning_retries']==1


def test_cpu_and_gpu_physical_profiles_keep_the_same_alignment_limits():
    launch=(PACKAGE/'launch/pregrasp_hardware_support.launch.py').read_text()
    assert launch.index('config/collection_motion.yaml') > launch.index('config/hybrid_motion.yaml')
    params=yaml.safe_load((PACKAGE/'config/collection_motion.yaml').read_text())['grasp_selection_server']['ros__parameters']
    assert 'motion_backend' not in params
    assert params['position_tolerance_m']==params['pregrasp_position_tolerance_m']==.010
    assert params['approach_tolerance_degrees']==params['closing_tolerance_degrees']==30.
    assert params['maximum_candidates']==32
    assert params['validate_motion_paths'] is True
    assert params['align_wrist_roll'] is True


def test_standalone_and_sensor_launches_load_hybrid_profile_and_cuda_python():
    for name in ('physical_grasp_selection.launch.py','pregrasp_runtime.launch.py'):
        launch=(PACKAGE/'launch'/name).read_text()
        assert "hybrid_motion.yaml" in launch
        assert 'motion_runtime_prefix()' in launch
        assert "'motion_backend': backend" in launch
        assert "curobo_ik_moveit" in launch


def test_moveit_execution_is_disabled_for_guarded_executor():
    script=(ROOT/'tools/collection-runtime').read_text()
    assert 'read_only:=false with_moveit:=true allow_trajectory_execution:=false' in script
