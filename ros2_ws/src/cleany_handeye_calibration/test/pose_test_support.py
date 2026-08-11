from __future__ import annotations

import math

import numpy as np

from cleany_handeye_calibration.joint_state_sync import LEFT_ARM_JOINT_NAMES
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
)
from cleany_handeye_calibration.pose_diversity import (
    evaluate_rotation_diversity,
    rotation_observations,
)
from cleany_handeye_calibration.pose_generation import (
    EvaluatedPoseCandidate,
    PoseCandidateRequest,
)
from cleany_handeye_calibration.pose_manifest import (
    CALIBRATION_ARM,
    CALIBRATION_FRAME,
    POSE_MANIFEST_SCHEMA_VERSION,
    RANDOM_ENGINE,
    SELECTION_STRATEGY,
    CartesianBounds,
    ClosedInterval,
    GeneratorRecord,
    MaterializedPose,
    PoseManifest,
    PoseRunConfiguration,
    PoseSelectionRecord,
    PoseValidationEvidence,
    RequiredStageTimeouts,
    SoftJointLimits,
)
from cleany_handeye_calibration.transforms import (
    RigidTransform,
    rotation_matrix_from_rodrigues,
)


def stage_timeouts() -> RequiredStageTimeouts:
    return RequiredStageTimeouts(
        ik_sec=1.0,
        state_validity_sec=1.0,
        plan_sec=2.0,
        execute_sec=3.0,
        cancel_sec=1.0,
        settle_sec=2.0,
        image_acquisition_sec=1.0,
        target_detection_sec=1.0,
        feedback_fk_sec=1.0,
        record_sample_sec=1.0,
    )


def run_config() -> PoseRunConfiguration:
    return PoseRunConfiguration(
        max_retries=3,
        stage_timeouts=stage_timeouts(),
        right_park_position_tolerance_rad=0.01,
        soft_joint_limits=SoftJointLimits(
            joint_names=LEFT_ARM_JOINT_NAMES,
            lower_rad=(-2.0, -1.0, -1.0, -1.0, -2.0),
            upper_rad=(2.0, 2.0, 2.0, 2.0, 2.0),
        ),
        collision_margin_m=0.10,
        target_position_tolerance_m=1.0e-6,
        duplicate_target_position_tolerance_m=1.0e-8,
        duplicate_ik_seed_tolerance_rad=1.0e-8,
        duplicate_resolved_joint_tolerance_rad=1.0e-8,
        axis_parallelism_tolerance=0.01,
        covariance_rank_tolerance=1.0e-10,
    )


def bounds() -> CartesianBounds:
    return CartesianBounds(
        x_m=ClosedInterval(0.40, 0.80),
        y_m=ClosedInterval(-0.30, 0.40),
        z_m=ClosedInterval(0.40, 0.90),
    )


def rotation_vector(index: int) -> tuple[float, float, float]:
    axis = np.asarray(
        (
            math.sin(0.71 * (index + 1)) + 0.25,
            math.cos(0.43 * (index + 1)) - 0.15,
            0.35 + 0.04 * (index % 5),
        )
    )
    axis /= np.linalg.norm(axis)
    vector = axis * (0.20 + 0.017 * index)
    return tuple(float(value) for value in vector)


def evaluated_candidate(
    request: PoseCandidateRequest,
) -> EvaluatedPoseCandidate:
    index = int(request.candidate_id.rsplit('_', 1)[1])
    return EvaluatedPoseCandidate(
        request=request,
        resolved_joint_pose=request.ik_seed,
        base_T_gripper=RigidTransform(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            rotation_matrix=rotation_matrix_from_rodrigues(
                rotation_vector(index)
            ),
            translation_m=request.target.position_m,
        ),
        validation=PoseValidationEvidence(
            target_position_error_m=0.0,
            minimum_collision_distance_m=0.20,
            planning_succeeded=True,
            target_visible=True,
            camera_front=True,
        ),
    )


def materialized_manifest() -> PoseManifest:
    poses = []
    for index in range(25):
        split = (
            SampleSplit.CALIBRATION
            if index < 20
            else SampleSplit.HELD_OUT
        )
        split_index = index + 1 if index < 20 else index - 19
        target = (0.45 + 0.01 * (index % 10), 0.05 + 0.008 * index, 0.60)
        positions = (
            -1.5 + 0.03 * index,
            0.2 + 0.02 * index,
            0.3 + 0.015 * index,
            0.4 + 0.01 * index,
            -1.0 + 0.025 * index,
        )
        poses.append(
            MaterializedPose(
                pose_id=(
                    f'calibration_{split_index:03d}'
                    if split is SampleSplit.CALIBRATION
                    else f'held_out_{split_index:03d}'
                ),
                source_candidate_id=f'candidate_{index + 1:06d}',
                split=split,
                target=PositionTarget('base_link', target),
                ik_seed=JointPose(LEFT_ARM_JOINT_NAMES, positions),
                resolved_joint_pose=JointPose(
                    LEFT_ARM_JOINT_NAMES, positions
                ),
                base_T_gripper=RigidTransform(
                    parent_frame='base_link',
                    child_frame='left_gripper_frame',
                    rotation_matrix=rotation_matrix_from_rodrigues(
                        rotation_vector(index + 1)
                    ),
                    translation_m=target,
                ),
                validation=PoseValidationEvidence(
                    target_position_error_m=0.0,
                    minimum_collision_distance_m=0.20,
                    planning_succeeded=True,
                    target_visible=True,
                    camera_front=True,
                ),
            )
        )
    calibration = poses[:20]
    observations = rotation_observations(
        [pose.pose_id for pose in calibration],
        [pose.base_T_gripper.rotation_matrix for pose in calibration],
        reference_rotation_matrix=np.eye(3),
    )
    config = run_config()
    diversity = evaluate_rotation_diversity(
        observations,
        log_det_epsilon=1.0e-9,
        axis_parallelism_tolerance=config.axis_parallelism_tolerance,
        covariance_rank_tolerance=config.covariance_rank_tolerance,
    )
    return PoseManifest(
        schema_version=POSE_MANIFEST_SCHEMA_VERSION,
        calibration_arm=CALIBRATION_ARM,
        frame_id=CALIBRATION_FRAME,
        joint_names=LEFT_ARM_JOINT_NAMES,
        generator=GeneratorRecord(
            random_seed=20260810,
            random_engine=RANDOM_ENGINE,
            candidate_pool_size=25,
            max_generation_attempts=25,
            attempts_used=25,
            log_det_epsilon=1.0e-9,
            target_position_bounds_m=bounds(),
        ),
        run_config=config,
        selection=PoseSelectionRecord(
            strategy=SELECTION_STRATEGY,
            reference_rotation_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            diversity=diversity,
        ),
        poses=tuple(poses),
    )
