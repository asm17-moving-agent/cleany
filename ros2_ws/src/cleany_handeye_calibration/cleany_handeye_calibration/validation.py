"""Ground-truth accuracy and held-out consistency metrics."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Sequence

import numpy as np

from cleany_handeye_calibration.models import (
    CalibrationSample,
    SampleSplit,
)
from cleany_handeye_calibration.solver import HandEyeMethod, HandEyeResult
from cleany_handeye_calibration.transforms import RigidTransform


@dataclass(frozen=True, slots=True)
class TransformErrorMetrics:
    translation_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True, slots=True)
class HandEyeEvaluation:
    method: HandEyeMethod
    translation_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True, slots=True)
class HeldOutConsistencyMetrics:
    sample_count: int
    pair_count: int
    translation_median_m: float
    translation_p95_m: float
    rotation_median_rad: float
    rotation_p95_rad: float


def _rotation_distance_rad(
    first_rotation: np.ndarray,
    second_rotation: np.ndarray,
) -> float:
    relative = first_rotation.T @ second_rotation
    skew_vector = np.array(
        (
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        )
    )
    sine = float(np.linalg.norm(skew_vector) / 2.0)
    cosine = float((np.trace(relative) - 1.0) / 2.0)
    return math.atan2(sine, max(-1.0, min(1.0, cosine)))


def transform_error_metrics(
    estimate: RigidTransform,
    reference: RigidTransform,
) -> TransformErrorMetrics:
    """Compare transforms with identical directions, in metres and radians."""

    if not isinstance(estimate, RigidTransform) or not isinstance(
        reference,
        RigidTransform,
    ):
        raise ValueError(
            'estimate and reference must be RigidTransform values'
        )
    if (
        estimate.parent_frame != reference.parent_frame
        or estimate.child_frame != reference.child_frame
    ):
        raise ValueError('transform directions must match before comparison')
    translation_error = float(
        np.linalg.norm(
            estimate.translation_array() - reference.translation_array()
        )
    )
    rotation_error = _rotation_distance_rad(
        reference.rotation_array(),
        estimate.rotation_array(),
    )
    return TransformErrorMetrics(
        translation_error_m=translation_error,
        rotation_error_rad=rotation_error,
    )


def evaluate_hand_eye_result(
    result: HandEyeResult,
    ground_truth: RigidTransform,
) -> HandEyeEvaluation:
    """Evaluate one valid result; ground truth never enters the solver API."""

    if not isinstance(result, HandEyeResult) or not result.valid:
        raise ValueError('only a valid HandEyeResult can be evaluated')
    if result.gripper_T_camera is None:
        raise ValueError('valid HandEyeResult lacks gripper_T_camera')
    metrics = transform_error_metrics(
        result.gripper_T_camera,
        ground_truth,
    )
    return HandEyeEvaluation(
        method=result.method,
        translation_error_m=metrics.translation_error_m,
        rotation_error_rad=metrics.rotation_error_rad,
    )


def held_out_base_target_consistency(
    samples: Sequence[CalibrationSample],
    gripper_T_camera: RigidTransform,
) -> HeldOutConsistencyMetrics:
    """Measure all pairwise held-out ``base_T_target`` disagreements."""

    held_out_samples = tuple(samples)
    if len(held_out_samples) < 2:
        raise ValueError('held-out consistency requires at least two samples')
    if not isinstance(gripper_T_camera, RigidTransform):
        raise ValueError('gripper_T_camera must be a RigidTransform')

    base_target_transforms: list[RigidTransform] = []
    for sample in held_out_samples:
        if not isinstance(sample, CalibrationSample):
            raise ValueError(
                'held-out inputs must be CalibrationSample values'
            )
        if sample.split is not SampleSplit.HELD_OUT:
            raise ValueError(
                f'{sample.sample_id} is not in the held_out split'
            )
        try:
            base_target = (
                sample.base_T_gripper
                @ gripper_T_camera
                @ sample.camera_T_target
            )
        except ValueError as error:
            raise ValueError(
                f'{sample.sample_id} violates the held-out frame contract'
            ) from error
        if base_target_transforms and (
            base_target.parent_frame
            != base_target_transforms[0].parent_frame
            or base_target.child_frame
            != base_target_transforms[0].child_frame
        ):
            raise ValueError(
                'held-out samples do not share base/target frames'
            )
        base_target_transforms.append(base_target)

    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for first, second in combinations(base_target_transforms, 2):
        metrics = transform_error_metrics(first, second)
        translation_errors.append(metrics.translation_error_m)
        rotation_errors.append(metrics.rotation_error_rad)

    return HeldOutConsistencyMetrics(
        sample_count=len(base_target_transforms),
        pair_count=len(translation_errors),
        translation_median_m=float(np.median(translation_errors)),
        translation_p95_m=float(np.percentile(translation_errors, 95.0)),
        rotation_median_rad=float(np.median(rotation_errors)),
        rotation_p95_rad=float(np.percentile(rotation_errors, 95.0)),
    )
