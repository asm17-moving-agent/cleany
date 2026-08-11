from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import pytest

from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
)
from cleany_handeye_calibration.single_pose_runtime_config import (
    SCHEMA_VERSION,
    SinglePoseConfigError,
    load_single_pose_runtime_config,
)


def _mapping(tmp_path):
    stage_timeouts = {
        'resolve_position_ik': 3.0,
        'validate_resolved_pose': 3.0,
        'plan': 5.0,
        'execute': 8.0,
        'wait_settled': 3.0,
        'acquire_image': 3.0,
        'detect_target': 1.0,
        'compute_feedback_fk': 3.0,
        'record_sample': 2.0,
    }
    soft_limits = {
        name: [-2.0, 2.0] for name in LEFT_ARM_JOINT_NAMES
    }
    return {
        'schema_version': SCHEMA_VERSION,
        'artifact_root': str(tmp_path / 'artifacts'),
        'sample': {
            'sample_id': 'sample_001',
            'pose_id': 'calibration_001',
            'split': 'calibration',
            'target_position_m': [0.5, 0.2, 0.4],
            'ik_seed_positions_rad': [-1.0, 0.2, 0.4, -0.2, -1.5],
        },
        'safety_profile': {
            'profile_id': 'explicit_mujoco_test',
            'soft_joint_limits_rad': soft_limits,
            'collision_margin_m': 0.005,
        },
        'expected_resolved_evidence': {
            'joint_positions_rad': [-1.0, 0.2, 0.4, -0.2, -1.5],
            'match_tolerance_rad': 0.001,
            'collision_clearance_m': 0.01,
        },
        'motion': {
            'current_state_max_age_sec': 0.5,
            'right_park_position_tolerance_rad': 0.01,
            'stage_timeouts_sec': {
                'ik': 2.0,
                'state_validity': 2.0,
                'plan': 4.0,
                'execute': 7.0,
                'cancel': 1.0,
                'settle': 2.0,
            },
            'max_velocity_scaling_factor': 0.1,
            'max_acceleration_scaling_factor': 0.1,
            'controller_path_tolerance_rad': 0.05,
            'controller_goal_tolerance_rad': 0.01,
            'settle_position_tolerance_rad': 0.005,
            'settle_velocity_tolerance_rad_s': 0.01,
            'settle_duration_sec': 1.0,
            'planning_attempts': 1,
        },
        'orchestration_timeouts_sec': stage_timeouts,
        'feedback_buffer': {
            'capacity': 256,
            'max_sample_distance_ns': 50_000_000,
            'clock_reset_threshold_ns': 500_000_000,
            'startup_state_timeout_sec': 30.0,
            'startup_planning_scene_timeout_sec': 30.0,
        },
        'dataset_manifest': {
            'run_id': 'run_001',
            'git': {
                'commit': '0123456789abcdef0123456789abcdef01234567',
                'dirty': True,
            },
            'source_hashes': {
                'urdf_sha256': 'a' * 64,
                'mjcf_sha256': 'b' * 64,
                'pose_manifest_sha256': 'c' * 64,
            },
            'software_versions': {
                'ros_distro': 'humble',
                'moveit': '2.5.9',
                'opencv': '4.5.4',
                'mujoco': '3.4.0',
                'mujoco_ros2_control': '0.0.3',
                'vendor': {'opencv-contrib': '4.5.4'},
            },
            'target': {
                'board_svg_sha256': 'd' * 64,
                'board_pdf_sha256': 'e' * 64,
                'size_provenance': 'simulation_manifest_exact_geometry',
            },
            'timing': {
                'simulation_timestep_s': 0.002,
                'controller_update_rate_hz': 50.0,
                'image_rate_hz': 10.0,
                'joint_state_rate_hz': 50.0,
            },
            'calibration_parameters': {
                'profile': 'explicit_mujoco_test'
            },
            'random_seed': 20260810,
        },
    }


def _write(tmp_path, mapping, name='request.json'):
    path = tmp_path / name
    path.write_text(json.dumps(mapping), encoding='utf-8')
    return path


def test_loads_fully_explicit_single_pose_runtime_config(tmp_path):
    config = load_single_pose_runtime_config(
        _write(tmp_path, _mapping(tmp_path))
    )

    assert config.request.pose_id == 'calibration_001'
    assert config.request.ik_seed.joint_names == LEFT_ARM_JOINT_NAMES
    assert config.request.timeouts.acquire_image_sec == 3.0
    assert config.motion.stage_timeouts.execute_sec == 7.0
    assert config.feedback.startup_planning_scene_timeout_sec == 30.0
    assert config.expected.observed_collision_clearance_m == 0.01
    assert config.dataset_manifest.run_id == 'run_001'


@pytest.mark.parametrize(
    'mutation',
    [
        lambda value: value['safety_profile'].__setitem__(
            'collision_margin_m', None
        ),
        lambda value: value['safety_profile'].__setitem__(
            'soft_joint_limits_rad', None
        ),
        lambda value: value['expected_resolved_evidence'].__setitem__(
            'collision_clearance_m', None
        ),
        lambda value: value['orchestration_timeouts_sec'].__setitem__(
            'acquire_image', None
        ),
    ],
)
def test_unresolved_safety_or_timeout_placeholder_blocks_execution(
    tmp_path, mutation
):
    mapping = _mapping(tmp_path)
    mutation(mapping)

    with pytest.raises((SinglePoseConfigError, ValueError, TypeError)):
        load_single_pose_runtime_config(_write(tmp_path, mapping))


def test_adapter_budget_must_fit_inside_orchestrator_stage(tmp_path):
    mapping = _mapping(tmp_path)
    mapping['orchestration_timeouts_sec']['execute'] = 1.0

    with pytest.raises(ValueError, match='must cover'):
        load_single_pose_runtime_config(_write(tmp_path, mapping))


def test_resolved_pose_must_match_clearance_evidence(tmp_path):
    config = load_single_pose_runtime_config(
        _write(tmp_path, _mapping(tmp_path))
    )
    different = deepcopy(config.expected.pose)
    object.__setattr__(
        different,
        'positions_rad',
        (-0.5, 0.2, 0.4, -0.2, -1.5),
    )

    with pytest.raises(ValueError, match='differs'):
        config.expected.validate_match(different)


def test_duplicate_json_key_is_rejected(tmp_path):
    path = tmp_path / 'duplicate.json'
    path.write_text(
        '{"schema_version":"a","schema_version":"b"}',
        encoding='utf-8',
    )

    with pytest.raises(SinglePoseConfigError, match='duplicate JSON key'):
        load_single_pose_runtime_config(path)


def test_installed_template_is_deliberately_not_runnable():
    template = (
        Path(__file__).parents[1]
        / 'config'
        / 'single_pose_request.template.json'
    )

    with pytest.raises((SinglePoseConfigError, ValueError, TypeError)):
        load_single_pose_runtime_config(template)
