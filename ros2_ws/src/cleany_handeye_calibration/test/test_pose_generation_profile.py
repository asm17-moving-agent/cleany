from pathlib import Path

import pytest

from cleany_handeye_calibration.pose_generation_profile import (
    PoseGenerationProfileError,
    load_mujoco_pose_generation_profile,
)


PACKAGE_ROOT = Path(__file__).parents[1]
PROFILE_PATH = PACKAGE_ROOT / 'config' / 'pose_generation.mujoco.yaml'


def test_mujoco_profile_materializes_seed_bounds_and_relaxed_settle():
    profile = load_mujoco_pose_generation_profile(PROFILE_PATH)

    assert profile.random_seed == 20260810
    assert profile.candidate_pool_size == 30
    assert profile.max_generation_attempts == 9000
    assert profile.run_config.max_retries == 3
    assert profile.run_config.collision_margin_m == 0.01
    assert profile.run_config.stage_timeouts.execute_sec == 12.0
    assert profile.motion.controller_goal_tolerance_rad == 0.01
    assert profile.motion.settle_position_tolerance_rad == 0.015
    assert profile.seed_sampling_limits.lower_rad[0] == -1.75
    assert profile.seed_sampling_limits.upper_rad[4] == 2.75


def test_profile_rejects_duplicate_yaml_keys(tmp_path):
    duplicate = tmp_path / 'duplicate.yaml'
    duplicate.write_text(
        'schema_version: cleany.pose_generation_mujoco/v1\n'
        'schema_version: cleany.pose_generation_mujoco/v1\n',
        encoding='utf-8',
    )

    with pytest.raises(PoseGenerationProfileError, match='duplicate'):
        load_mujoco_pose_generation_profile(duplicate)
