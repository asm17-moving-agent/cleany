import pytest

from cleany_handeye_calibration.models import SampleSplit
from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.pose_generation import (
    PoseGenerationConfig,
    PoseGenerationError,
    generate_pose_manifest,
)
from cleany_handeye_calibration.pose_manifest import (
    SoftJointLimits,
    pose_manifest_to_mapping,
)
from pose_test_support import bounds, evaluated_candidate, run_config


class AcceptingEvaluator:
    def __init__(self, reject_first=0):
        self.reject_first = reject_first
        self.requests = []

    def evaluate(self, request, run):
        assert run is run_config_value
        self.requests.append(request)
        if len(self.requests) <= self.reject_first:
            return None
        return evaluated_candidate(request)


run_config_value = run_config()


def _config(seed=42, pool_size=25, attempt_cap=30):
    return PoseGenerationConfig(
        random_seed=seed,
        candidate_pool_size=pool_size,
        max_generation_attempts=attempt_cap,
        log_det_epsilon=1.0e-9,
        target_position_bounds_m=bounds(),
        run_config=run_config_value,
        reference_rotation_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        multistart_count=3,
    )


def test_generation_is_seeded_bounded_and_materializes_fixed_splits():
    first = generate_pose_manifest(_config(seed=123), AcceptingEvaluator())
    second = generate_pose_manifest(_config(seed=123), AcceptingEvaluator())

    assert pose_manifest_to_mapping(first) == pose_manifest_to_mapping(second)
    assert first.generator.random_seed == 123
    assert first.generator.attempts_used == 25
    assert [pose.split for pose in first.poses[:20]] == [
        SampleSplit.CALIBRATION
    ] * 20
    assert [pose.split for pose in first.poses[20:]] == [
        SampleSplit.HELD_OUT
    ] * 5
    assert len(first.selection.diversity.nonparallel_axis_pose_ids) >= 5


def test_generation_records_rejections_and_stops_at_attempt_cap():
    evaluator = AcceptingEvaluator(reject_first=2)
    manifest = generate_pose_manifest(
        _config(pool_size=25, attempt_cap=27), evaluator
    )
    assert manifest.generator.attempts_used == 27

    with pytest.raises(PoseGenerationError, match='attempt cap'):
        generate_pose_manifest(
            _config(pool_size=25, attempt_cap=25),
            AcceptingEvaluator(reject_first=1),
        )


def test_generation_samples_random_seeds_from_explicit_workspace_prior():
    evaluator = AcceptingEvaluator()
    config = PoseGenerationConfig(
        random_seed=81,
        candidate_pool_size=25,
        max_generation_attempts=25,
        log_det_epsilon=1.0e-9,
        target_position_bounds_m=bounds(),
        run_config=run_config_value,
        reference_rotation_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        multistart_count=3,
        seed_sampling_limits=SoftJointLimits(
            joint_names=LEFT_ARM_JOINT_NAMES,
            lower_rad=(-1.8, 0.1, 0.2, 0.3, -1.0),
            upper_rad=(-1.7, 0.2, 0.3, 0.4, -0.9),
        ),
    )

    generate_pose_manifest(config, evaluator)

    for request in evaluator.requests:
        assert config.seed_sampling_limits.contains(request.ik_seed)


def test_generation_rejects_seed_prior_outside_safety_soft_limits():
    with pytest.raises(ValueError, match='inside soft limits'):
        PoseGenerationConfig(
            random_seed=81,
            candidate_pool_size=25,
            max_generation_attempts=25,
            log_det_epsilon=1.0e-9,
            target_position_bounds_m=bounds(),
            run_config=run_config_value,
            reference_rotation_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            seed_sampling_limits=SoftJointLimits(
                joint_names=LEFT_ARM_JOINT_NAMES,
                lower_rad=(-2.1, -1.0, -1.0, -1.0, -2.0),
                upper_rad=(2.0, 2.0, 2.0, 2.0, 2.0),
            ),
        )
