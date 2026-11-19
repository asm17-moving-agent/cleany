"""Deterministic candidate generation and rotation-diverse pose selection."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol, Sequence

import numpy as np

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
)
from cleany_handeye_calibration.pose_diversity import (
    RotationDiversity,
    RotationObservation,
    evaluate_rotation_diversity,
    require_rotation_diversity,
    rotation_observations,
    rotation_selection_objective,
)
from cleany_handeye_calibration.pose_manifest import (
    CALIBRATION_ARM,
    CALIBRATION_FRAME,
    CALIBRATION_POSE_COUNT,
    HELD_OUT_POSE_COUNT,
    POSE_MANIFEST_SCHEMA_VERSION,
    RANDOM_ENGINE,
    SELECTION_STRATEGY,
    TOTAL_POSE_COUNT,
    CartesianBounds,
    GeneratorRecord,
    MaterializedPose,
    PoseManifest,
    PoseRunConfiguration,
    PoseSelectionRecord,
    PoseValidationEvidence,
    SoftJointLimits,
    preflight_pose_manifest,
)
from cleany_handeye_calibration.transforms import (
    RigidTransform,
    quaternion_xyzw_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
)


@dataclass(frozen=True, slots=True)
class PoseCandidateRequest:
    candidate_id: str
    target: PositionTarget
    ik_seed: JointPose

    def __post_init__(self) -> None:
        if (
            not isinstance(self.candidate_id, str)
            or not self.candidate_id
            or self.candidate_id != self.candidate_id.strip()
        ):
            raise ValueError('candidate_id must be non-empty trimmed text')
        if not isinstance(self.target, PositionTarget):
            raise ValueError('target must be PositionTarget')
        if self.target.frame_id != CALIBRATION_FRAME:
            raise ValueError('candidate target frame must be base_link')
        if not isinstance(self.ik_seed, JointPose):
            raise ValueError('ik_seed must be JointPose')
        if self.ik_seed.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('IK seed must use canonical left-arm order')


@dataclass(frozen=True, slots=True)
class EvaluatedPoseCandidate:
    request: PoseCandidateRequest
    resolved_joint_pose: JointPose
    base_T_gripper: RigidTransform
    validation: PoseValidationEvidence

    def __post_init__(self) -> None:
        if not isinstance(self.request, PoseCandidateRequest):
            raise ValueError('request must be PoseCandidateRequest')
        if not isinstance(self.resolved_joint_pose, JointPose):
            raise ValueError('resolved_joint_pose must be JointPose')
        if self.resolved_joint_pose.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('resolved pose must use canonical left-arm order')
        if not isinstance(self.base_T_gripper, RigidTransform):
            raise ValueError('base_T_gripper must be RigidTransform')
        if (
            self.base_T_gripper.parent_frame != CALIBRATION_FRAME
            or self.base_T_gripper.child_frame != 'left_gripper_frame'
        ):
            raise ValueError(
                'candidate FK must be base_link_T_left_gripper_frame'
            )
        if not isinstance(self.validation, PoseValidationEvidence):
            raise ValueError('validation must be PoseValidationEvidence')


class PoseCandidateEvaluator(Protocol):
    """Resolve and validate one sampled target/seed through MoveIt/FK."""

    def evaluate(
        self,
        request: PoseCandidateRequest,
        run_config: PoseRunConfiguration,
    ) -> EvaluatedPoseCandidate | None: ...


class PoseTargetSampler(Protocol):
    """Sample a bounded random target conditional on one random IK seed."""

    def sample_target(
        self,
        seed: JointPose,
        rng: np.random.Generator,
        bounds: CartesianBounds,
    ) -> PositionTarget | None: ...


@dataclass(frozen=True, slots=True)
class PoseGenerationConfig:
    random_seed: int
    candidate_pool_size: int
    max_generation_attempts: int
    log_det_epsilon: float
    target_position_bounds_m: CartesianBounds
    run_config: PoseRunConfiguration
    reference_rotation_quaternion_xyzw: tuple[float, float, float, float]
    multistart_count: int = 8
    seed_sampling_limits: SoftJointLimits | None = None

    def __post_init__(self) -> None:
        # Reuse the manifest record's strict seed/pool/epsilon validation.
        GeneratorRecord(
            random_seed=self.random_seed,
            random_engine=RANDOM_ENGINE,
            candidate_pool_size=self.candidate_pool_size,
            max_generation_attempts=self.max_generation_attempts,
            attempts_used=self.candidate_pool_size,
            log_det_epsilon=self.log_det_epsilon,
            target_position_bounds_m=self.target_position_bounds_m,
        )
        if not isinstance(self.run_config, PoseRunConfiguration):
            raise ValueError('run_config must be PoseRunConfiguration')
        self.run_config.require_ready()
        quaternion = tuple(
            float(value)
            for value in self.reference_rotation_quaternion_xyzw
        )
        rotation_matrix_from_quaternion_xyzw(quaternion)
        if (
            isinstance(self.multistart_count, bool)
            or not isinstance(self.multistart_count, int)
            or self.multistart_count <= 0
        ):
            raise ValueError('multistart_count must be a positive integer')
        if self.seed_sampling_limits is not None:
            sampling = self.seed_sampling_limits
            if sampling.joint_names != LEFT_ARM_JOINT_NAMES:
                raise ValueError(
                    'seed sampling limits must use canonical left-arm order'
                )
            assert self.run_config.soft_joint_limits is not None
            safety = self.run_config.soft_joint_limits
            for sample_lower, sample_upper, safe_lower, safe_upper in zip(
                sampling.lower_rad,
                sampling.upper_rad,
                safety.lower_rad,
                safety.upper_rad,
                strict=True,
            ):
                if sample_lower < safe_lower or sample_upper > safe_upper:
                    raise ValueError(
                        'seed sampling limits must remain inside soft limits'
                    )
        object.__setattr__(
            self,
            'reference_rotation_quaternion_xyzw',
            quaternion,
        )


class PoseGenerationError(RuntimeError):
    pass


def _observation(
    candidate: EvaluatedPoseCandidate,
    reference_rotation: np.ndarray,
) -> RotationObservation:
    return rotation_observations(
        (candidate.request.candidate_id,),
        (candidate.base_T_gripper.rotation_matrix,),
        reference_rotation_matrix=reference_rotation,
    )[0]


def _objective(
    candidates: Sequence[EvaluatedPoseCandidate],
    observations: dict[str, RotationObservation],
    *,
    log_det_epsilon: float,
) -> tuple[float, float]:
    values = tuple(
        observations[item.request.candidate_id] for item in candidates
    )
    maximum_parallelism, log_det = rotation_selection_objective(
        values,
        log_det_epsilon=log_det_epsilon,
    )
    return maximum_parallelism, -log_det


def select_diverse_candidates(
    candidates: Sequence[EvaluatedPoseCandidate],
    count: int,
    *,
    reference_rotation_matrix: Sequence[Sequence[float]] | np.ndarray,
    log_det_epsilon: float,
    multistart_count: int = 8,
) -> tuple[EvaluatedPoseCandidate, ...]:
    """Select a deterministic subset using the approved lexicographic score."""

    ordered = tuple(
        sorted(candidates, key=lambda item: item.request.candidate_id)
    )
    if count <= 0 or len(ordered) < count:
        raise ValueError('candidate selection count is out of range')
    reference = np.asarray(reference_rotation_matrix, dtype=np.float64)
    observations = {
        item.request.candidate_id: _observation(item, reference)
        for item in ordered
    }
    starts = ordered[:min(multistart_count, len(ordered))]
    best: tuple[EvaluatedPoseCandidate, ...] | None = None
    best_key: tuple[float, float, tuple[str, ...]] | None = None

    for start in starts:
        selected = [start]
        available = [item for item in ordered if item is not start]
        while len(selected) < count:
            chosen = min(
                available,
                key=lambda item: (
                    *_objective(
                        (*selected, item),
                        observations,
                        log_det_epsilon=log_det_epsilon,
                    ),
                    item.request.candidate_id,
                ),
            )
            selected.append(chosen)
            available.remove(chosen)

        # A deterministic one-swap local improvement avoids depending on one
        # greedy insertion order while keeping generation bounded.
        while True:
            current_key = (
                *_objective(
                    selected,
                    observations,
                    log_det_epsilon=log_det_epsilon,
                ),
                tuple(sorted(item.request.candidate_id for item in selected)),
            )
            replacement = None
            replacement_key = current_key
            unselected = [item for item in ordered if item not in selected]
            for index, _ in enumerate(selected):
                for candidate in unselected:
                    trial = [*selected]
                    trial[index] = candidate
                    key = (
                        *_objective(
                            trial,
                            observations,
                            log_det_epsilon=log_det_epsilon,
                        ),
                        tuple(
                            sorted(item.request.candidate_id for item in trial)
                        ),
                    )
                    if key < replacement_key:
                        replacement = (index, candidate)
                        replacement_key = key
            if replacement is None:
                break
            selected[replacement[0]] = replacement[1]

        result = tuple(
            sorted(selected, key=lambda item: item.request.candidate_id)
        )
        key = (
            *_objective(
                result,
                observations,
                log_det_epsilon=log_det_epsilon,
            ),
            tuple(item.request.candidate_id for item in result),
        )
        if best_key is None or key < best_key:
            best = result
            best_key = key
    assert best is not None
    return best


def _is_duplicate(
    candidate: EvaluatedPoseCandidate,
    pool: Sequence[EvaluatedPoseCandidate],
    config: PoseRunConfiguration,
) -> bool:
    assert config.duplicate_target_position_tolerance_m is not None
    assert config.duplicate_ik_seed_tolerance_rad is not None
    assert config.duplicate_resolved_joint_tolerance_rad is not None
    for existing in pool:
        target_distance = float(
            np.linalg.norm(
                np.asarray(candidate.request.target.position_m)
                - np.asarray(existing.request.target.position_m)
            )
        )
        seed_difference = max(
            abs(left - right)
            for left, right in zip(
                candidate.request.ik_seed.positions_rad,
                existing.request.ik_seed.positions_rad,
                strict=True,
            )
        )
        resolved_difference = max(
            abs(left - right)
            for left, right in zip(
                candidate.resolved_joint_pose.positions_rad,
                existing.resolved_joint_pose.positions_rad,
                strict=True,
            )
        )
        if (
            target_distance
            <= config.duplicate_target_position_tolerance_m
            and seed_difference <= config.duplicate_ik_seed_tolerance_rad
        ):
            return True
        if (
            resolved_difference
            <= config.duplicate_resolved_joint_tolerance_rad
        ):
            return True
    return False


def _validate_accepted_candidate(
    candidate: EvaluatedPoseCandidate,
    request: PoseCandidateRequest,
    config: PoseGenerationConfig,
) -> None:
    if candidate.request != request:
        raise PoseGenerationError('evaluator returned a different request')
    run = config.run_config
    assert run.soft_joint_limits is not None
    assert run.collision_margin_m is not None
    assert run.target_position_tolerance_m is not None
    if not config.target_position_bounds_m.contains(
        request.target.position_m
    ):
        raise PoseGenerationError('sampled target escaped configured bounds')
    if not run.soft_joint_limits.contains(request.ik_seed):
        raise PoseGenerationError('sampled IK seed escaped soft limits')
    if not run.soft_joint_limits.contains(candidate.resolved_joint_pose):
        raise PoseGenerationError(
            'accepted resolved pose violates soft limits'
        )
    error = float(
        np.linalg.norm(
            np.asarray(request.target.position_m)
            - np.asarray(candidate.base_T_gripper.translation_m)
        )
    )
    evidence = candidate.validation
    if not math.isclose(
        error,
        evidence.target_position_error_m,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PoseGenerationError('target-error evidence does not match FK')
    if error > run.target_position_tolerance_m:
        raise PoseGenerationError('accepted candidate misses target tolerance')
    if evidence.minimum_collision_distance_m < run.collision_margin_m:
        raise PoseGenerationError(
            'accepted candidate violates collision margin'
        )
    if not (
        evidence.planning_succeeded
        and evidence.target_visible
        and evidence.camera_front
    ):
        raise PoseGenerationError(
            'accepted candidate lacks validation evidence'
        )


def _materialized_pose(
    candidate: EvaluatedPoseCandidate,
    *,
    pose_id: str,
    split: SampleSplit,
) -> MaterializedPose:
    return MaterializedPose(
        pose_id=pose_id,
        source_candidate_id=candidate.request.candidate_id,
        split=split,
        target=candidate.request.target,
        ik_seed=candidate.request.ik_seed,
        resolved_joint_pose=candidate.resolved_joint_pose,
        base_T_gripper=candidate.base_T_gripper,
        validation=candidate.validation,
    )


def generate_pose_manifest(
    config: PoseGenerationConfig,
    evaluator: PoseCandidateEvaluator,
    target_sampler: PoseTargetSampler | None = None,
) -> PoseManifest:
    """Sample a bounded pool, select 20+5 poses, and preflight the result."""

    if not isinstance(config, PoseGenerationConfig):
        raise ValueError('config must be PoseGenerationConfig')
    if evaluator is None:
        raise ValueError('evaluator is required')
    run = config.run_config.require_ready()
    assert run.soft_joint_limits is not None
    rng = np.random.Generator(np.random.PCG64(config.random_seed))
    sampling_limits = config.seed_sampling_limits or run.soft_joint_limits
    lower = np.asarray(sampling_limits.lower_rad)
    upper = np.asarray(sampling_limits.upper_rad)
    bounds = config.target_position_bounds_m
    target_low = np.asarray(
        (bounds.x_m.minimum, bounds.y_m.minimum, bounds.z_m.minimum)
    )
    target_high = np.asarray(
        (bounds.x_m.maximum, bounds.y_m.maximum, bounds.z_m.maximum)
    )
    reference = rotation_matrix_from_quaternion_xyzw(
        config.reference_rotation_quaternion_xyzw
    )
    pool: list[EvaluatedPoseCandidate] = []
    attempts_used = 0
    for attempt in range(1, config.max_generation_attempts + 1):
        attempts_used = attempt
        seed = JointPose(
            joint_names=LEFT_ARM_JOINT_NAMES,
            positions_rad=tuple(rng.uniform(lower, upper)),
        )
        if target_sampler is None:
            target = PositionTarget(
                frame_id=CALIBRATION_FRAME,
                position_m=tuple(rng.uniform(target_low, target_high)),
            )
        else:
            target = target_sampler.sample_target(seed, rng, bounds)
            if target is None:
                continue
        request = PoseCandidateRequest(
            candidate_id=f'candidate_{attempt:06d}',
            target=target,
            ik_seed=seed,
        )
        candidate = evaluator.evaluate(request, run)
        if candidate is None:
            continue
        _validate_accepted_candidate(candidate, request, config)
        try:
            _observation(candidate, reference)
        except ValueError:
            continue
        if _is_duplicate(candidate, pool, run):
            continue
        pool.append(candidate)
        if len(pool) == config.candidate_pool_size:
            break
    if len(pool) != config.candidate_pool_size:
        raise PoseGenerationError(
            'candidate pool did not reach the configured size before the '
            f'attempt cap: {len(pool)}/{config.candidate_pool_size}'
        )

    selected = select_diverse_candidates(
        pool,
        TOTAL_POSE_COUNT,
        reference_rotation_matrix=reference,
        log_det_epsilon=config.log_det_epsilon,
        multistart_count=config.multistart_count,
    )
    calibration = select_diverse_candidates(
        selected,
        CALIBRATION_POSE_COUNT,
        reference_rotation_matrix=reference,
        log_det_epsilon=config.log_det_epsilon,
        multistart_count=config.multistart_count,
    )
    calibration_ids = {item.request.candidate_id for item in calibration}
    held_out = tuple(
        item
        for item in selected
        if item.request.candidate_id not in calibration_ids
    )
    if len(held_out) != HELD_OUT_POSE_COUNT:
        raise PoseGenerationError('held-out split selection is inconsistent')
    poses = tuple(
        _materialized_pose(
            item,
            pose_id=f'calibration_{index:03d}',
            split=SampleSplit.CALIBRATION,
        )
        for index, item in enumerate(calibration, start=1)
    ) + tuple(
        _materialized_pose(
            item,
            pose_id=f'held_out_{index:03d}',
            split=SampleSplit.HELD_OUT,
        )
        for index, item in enumerate(held_out, start=1)
    )
    materialized_calibration = poses[:CALIBRATION_POSE_COUNT]
    observations = rotation_observations(
        [pose.pose_id for pose in materialized_calibration],
        [
            pose.base_T_gripper.rotation_matrix
            for pose in materialized_calibration
        ],
        reference_rotation_matrix=reference,
    )
    assert run.axis_parallelism_tolerance is not None
    assert run.covariance_rank_tolerance is not None
    diversity: RotationDiversity = evaluate_rotation_diversity(
        observations,
        log_det_epsilon=config.log_det_epsilon,
        axis_parallelism_tolerance=run.axis_parallelism_tolerance,
        covariance_rank_tolerance=run.covariance_rank_tolerance,
    )
    require_rotation_diversity(diversity)
    manifest = PoseManifest(
        schema_version=POSE_MANIFEST_SCHEMA_VERSION,
        calibration_arm=CALIBRATION_ARM,
        frame_id=CALIBRATION_FRAME,
        joint_names=LEFT_ARM_JOINT_NAMES,
        generator=GeneratorRecord(
            random_seed=config.random_seed,
            random_engine=RANDOM_ENGINE,
            candidate_pool_size=config.candidate_pool_size,
            max_generation_attempts=config.max_generation_attempts,
            attempts_used=attempts_used,
            log_det_epsilon=config.log_det_epsilon,
            target_position_bounds_m=config.target_position_bounds_m,
        ),
        run_config=run,
        selection=PoseSelectionRecord(
            strategy=SELECTION_STRATEGY,
            reference_rotation_quaternion_xyzw=(
                quaternion_xyzw_from_rotation_matrix(reference)
            ),
            diversity=diversity,
        ),
        poses=poses,
    )
    preflight_pose_manifest(manifest)
    return manifest


__all__ = [
    'EvaluatedPoseCandidate',
    'PoseCandidateEvaluator',
    'PoseCandidateRequest',
    'PoseGenerationConfig',
    'PoseGenerationError',
    'PoseTargetSampler',
    'generate_pose_manifest',
    'select_diverse_candidates',
]
