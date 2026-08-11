from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from cleany_handeye_calibration.models import JointPose, SampleSplit
from cleany_handeye_calibration.pose_diversity import (
    evaluate_rotation_diversity,
    rotation_observations,
)
from cleany_handeye_calibration.pose_manifest import (
    PoseManifestError,
    PoseRunConfiguration,
    RequiredStageTimeouts,
    UnresolvedRunConfiguration,
    load_pose_manifest,
    pose_manifest_from_mapping,
    pose_manifest_to_mapping,
    preflight_pose_manifest,
    write_pose_manifest,
)
from cleany_handeye_calibration.transforms import (
    RigidTransform,
    rotation_matrix_from_rodrigues,
)
from pose_test_support import materialized_manifest


PACKAGE_ROOT = Path(__file__).parents[1]


def test_materialized_manifest_preflights_exact_20_plus_5_and_round_trips(
    tmp_path,
):
    manifest = materialized_manifest()

    validated = preflight_pose_manifest(manifest)
    path = tmp_path / 'poses.yaml'
    write_pose_manifest(path, manifest)
    loaded = load_pose_manifest(path)

    assert len(manifest.poses_for_split(SampleSplit.CALIBRATION)) == 20
    assert len(manifest.poses_for_split(SampleSplit.HELD_OUT)) == 5
    assert len(validated.computed_diversity.nonparallel_axis_pose_ids) >= 5
    assert [pose.pose_id for pose in loaded.poses] == [
        pose.pose_id for pose in manifest.poses
    ]
    assert loaded.selection.diversity.maximum_axis_parallelism == (
        pytest.approx(
            manifest.selection.diversity.maximum_axis_parallelism,
            abs=1.0e-12,
        )
    )


def test_manifest_rejects_unresolved_motion_values_before_preflight():
    manifest = materialized_manifest()
    timeouts = RequiredStageTimeouts(
        **{
            name: None
            for name in RequiredStageTimeouts.FIELD_NAMES
        }
    )
    unresolved = replace(
        manifest,
        run_config=PoseRunConfiguration(
            max_retries=None,
            stage_timeouts=timeouts,
            right_park_position_tolerance_rad=None,
            soft_joint_limits=None,
            collision_margin_m=None,
            target_position_tolerance_m=None,
            duplicate_target_position_tolerance_m=None,
            duplicate_ik_seed_tolerance_rad=None,
            duplicate_resolved_joint_tolerance_rad=None,
            axis_parallelism_tolerance=None,
            covariance_rank_tolerance=None,
        ),
    )

    with pytest.raises(UnresolvedRunConfiguration) as caught:
        preflight_pose_manifest(unresolved)
    assert 'record_sample_sec' in ' '.join(caught.value.field_paths)
    assert 'soft_joint_limits_rad' in ' '.join(caught.value.field_paths)


def test_manifest_rejects_duplicate_resolved_pose_and_wrong_arm():
    manifest = materialized_manifest()
    poses = list(manifest.poses)
    poses[1] = replace(
        poses[1],
        resolved_joint_pose=JointPose(
            poses[0].resolved_joint_pose.joint_names,
            poses[0].resolved_joint_pose.positions_rad,
        ),
    )
    duplicate = replace(manifest, poses=tuple(poses))

    with pytest.raises(PoseManifestError, match='duplicate resolved'):
        preflight_pose_manifest(duplicate)
    mapping = pose_manifest_to_mapping(manifest)
    mapping['calibration_arm'] = 'right'
    with pytest.raises(PoseManifestError, match='exactly left'):
        pose_manifest_from_mapping(mapping)


def test_manifest_rejects_rank_deficient_calibration_rotation_set():
    manifest = materialized_manifest()
    poses = list(manifest.poses)
    for index in range(20):
        original = poses[index]
        poses[index] = replace(
            original,
            base_T_gripper=RigidTransform(
                parent_frame='base_link',
                child_frame='left_gripper_frame',
                rotation_matrix=rotation_matrix_from_rodrigues(
                    (0.1 + 0.01 * index, 0.0, 0.0)
                ),
                translation_m=original.base_T_gripper.translation_m,
            ),
        )
    observations = rotation_observations(
        [pose.pose_id for pose in poses[:20]],
        [pose.base_T_gripper.rotation_matrix for pose in poses[:20]],
        reference_rotation_matrix=np.eye(3),
    )
    config = manifest.run_config
    deficient = evaluate_rotation_diversity(
        observations,
        log_det_epsilon=manifest.generator.log_det_epsilon,
        axis_parallelism_tolerance=config.axis_parallelism_tolerance,
        covariance_rank_tolerance=config.covariance_rank_tolerance,
    )
    invalid = replace(
        manifest,
        poses=tuple(poses),
        selection=replace(manifest.selection, diversity=deficient),
    )

    with pytest.raises(ValueError, match='rank'):
        preflight_pose_manifest(invalid)


def test_loader_rejects_duplicate_yaml_keys(tmp_path):
    path = tmp_path / 'duplicate.yaml'
    path.write_text('schema_version: 1\nschema_version: 1\n', encoding='utf-8')

    with pytest.raises(PoseManifestError, match='duplicate key'):
        load_pose_manifest(path, require_preflight=False)


def test_generation_template_keeps_unapproved_values_explicitly_null():
    template = (
        PACKAGE_ROOT / 'config' / 'pose_generation.template.yaml'
    ).read_text(encoding='utf-8')

    assert 'poses: null' in template
    assert 'random_seed: null' in template
    assert 'record_sample_sec: null' in template
