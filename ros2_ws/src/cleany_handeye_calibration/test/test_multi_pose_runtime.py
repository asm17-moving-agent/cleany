import hashlib
import json
from pathlib import Path

import pytest

from cleany_handeye_calibration.multi_pose_runtime import (
    validate_multi_pose_runtime_profile,
)
from cleany_handeye_calibration.pose_manifest import write_pose_manifest
from cleany_handeye_calibration.single_pose_runtime_config import (
    load_single_pose_runtime_config,
)
from pose_test_support import materialized_manifest
from test_single_pose_runtime_config import _mapping


PACKAGE_ROOT = Path(__file__).parents[1]


def _runtime(tmp_path, manifest_path, manifest):
    mapping = _mapping(tmp_path)
    first = manifest.poses[0]
    timeouts = manifest.run_config.stage_timeouts
    mapping['sample'] = {
        'sample_id': first.pose_id,
        'pose_id': first.pose_id,
        'split': first.split.value,
        'target_position_m': list(first.target.position_m),
        'ik_seed_positions_rad': list(first.ik_seed.positions_rad),
    }
    limits = manifest.run_config.soft_joint_limits
    assert limits is not None
    mapping['safety_profile']['soft_joint_limits_rad'] = {
        name: [low, high]
        for name, low, high in zip(
            limits.joint_names,
            limits.lower_rad,
            limits.upper_rad,
            strict=True,
        )
    }
    mapping['safety_profile']['collision_margin_m'] = (
        manifest.run_config.collision_margin_m
    )
    mapping['expected_resolved_evidence'] = {
        'joint_positions_rad': list(
            first.resolved_joint_pose.positions_rad
        ),
        'match_tolerance_rad': 0.001,
        'collision_clearance_m': (
            first.validation.minimum_collision_distance_m
        ),
    }
    mapping['motion']['right_park_position_tolerance_rad'] = (
        manifest.run_config.right_park_position_tolerance_rad
    )
    mapping['motion']['stage_timeouts_sec'] = {
        'ik': timeouts.ik_sec,
        'state_validity': timeouts.state_validity_sec,
        'plan': timeouts.plan_sec,
        'execute': timeouts.execute_sec,
        'cancel': timeouts.cancel_sec,
        'settle': timeouts.settle_sec,
    }
    mapping['orchestration_timeouts_sec'] = {
        'resolve_position_ik': timeouts.ik_sec,
        'validate_resolved_pose': timeouts.state_validity_sec,
        'plan': timeouts.plan_sec,
        'execute': timeouts.execute_sec,
        'wait_settled': timeouts.settle_sec,
        'acquire_image': timeouts.image_acquisition_sec,
        'detect_target': timeouts.target_detection_sec,
        'compute_feedback_fk': timeouts.feedback_fk_sec,
        'record_sample': timeouts.record_sample_sec,
    }
    mapping['dataset_manifest']['source_hashes'][
        'pose_manifest_sha256'
    ] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    mapping['dataset_manifest']['random_seed'] = (
        manifest.generator.random_seed
    )
    path = tmp_path / 'runtime.json'
    path.write_text(json.dumps(mapping), encoding='utf-8')
    return load_single_pose_runtime_config(path), mapping


def test_multi_pose_profile_is_hash_anchored_and_exact(tmp_path):
    manifest = materialized_manifest()
    manifest_path = tmp_path / 'poses.yaml'
    write_pose_manifest(manifest_path, manifest)
    runtime, _ = _runtime(tmp_path, manifest_path, manifest)

    validate_multi_pose_runtime_profile(
        manifest_path, manifest, runtime
    )

    manifest_path.write_text('changed', encoding='utf-8')
    with pytest.raises(ValueError, match='SHA-256'):
        validate_multi_pose_runtime_profile(
            manifest_path, manifest, runtime
        )


def test_multi_pose_launch_shows_viewer_by_default_and_node_is_installed():
    source = (
        PACKAGE_ROOT / 'launch' / 'multi_pose_mujoco.launch.py'
    ).read_text(encoding='utf-8')
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "'headless',\n                default_value='false'" in source
    assert "'use_rviz',\n                default_value='true'" in source
    assert "'use_rviz': use_rviz" in source
    assert "executable='multi_pose_calibration'" in source
    assert 'multi_pose_runtime:main' in setup
