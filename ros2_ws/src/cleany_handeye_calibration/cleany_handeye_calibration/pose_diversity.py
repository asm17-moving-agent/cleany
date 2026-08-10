"""Rotation-diversity metrics for materialized calibration pose sets.

The selection objective is deliberately relative: lower maximum pairwise
axis parallelism wins, then higher covariance log-determinant wins.  Neither
quantity is used as an absolute acceptance threshold.  Acceptance uses only
the plan's explicit minimum of five pairwise non-parallel axes and covariance
rank three, with caller-supplied numerical tolerances.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Sequence

import numpy as np

from cleany_handeye_calibration.transforms import (
    rodrigues_from_rotation_matrix,
    validate_rotation_matrix,
)


MINIMUM_NONPARALLEL_AXES = 5
REQUIRED_ROTATION_COVARIANCE_RANK = 3


def _positive_finite(value: float, *, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field_name} must be positive and finite')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must be positive and finite'
        ) from error
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{field_name} must be positive and finite')
    return result


def _axis_parallelism_tolerance(value: float) -> float:
    result = _positive_finite(
        value,
        field_name='axis_parallelism_tolerance',
    )
    if result >= 1.0:
        raise ValueError('axis_parallelism_tolerance must be in (0, 1)')
    return result


@dataclass(frozen=True, slots=True)
class RotationObservation:
    """One relative SO(3) log vector and its normalized rotation axis."""

    pose_id: str
    rotation_vector_rad: tuple[float, float, float]
    axis: tuple[float, float, float]

    def __post_init__(self) -> None:
        if (
            not isinstance(self.pose_id, str)
            or not self.pose_id
            or self.pose_id != self.pose_id.strip()
        ):
            raise ValueError('pose_id must be a non-empty trimmed string')
        vector = np.asarray(self.rotation_vector_rad, dtype=np.float64)
        axis = np.asarray(self.axis, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(
                'rotation_vector_rad must contain three finite values'
            )
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError('axis must contain three finite values')
        vector_norm = float(np.linalg.norm(vector))
        if vector_norm == 0.0:
            raise ValueError('rotation_vector_rad must be nonzero')
        axis_norm = float(np.linalg.norm(axis))
        if not math.isclose(axis_norm, 1.0, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError('axis must have unit norm')
        expected_axis = vector / vector_norm
        if not np.allclose(axis, expected_axis, rtol=0.0, atol=1.0e-12):
            raise ValueError('axis must be the normalized rotation vector')
        object.__setattr__(
            self,
            'rotation_vector_rad',
            tuple(float(component) for component in vector),
        )
        object.__setattr__(
            self,
            'axis',
            tuple(float(component) for component in axis),
        )


@dataclass(frozen=True, slots=True)
class RotationDiversity:
    """Auditable metrics and a concrete five-axis diversity witness."""

    maximum_axis_parallelism: float
    rotation_covariance_log_det: float
    rotation_covariance_rank: int
    nonparallel_axis_pose_ids: tuple[str, ...]

    def __post_init__(self) -> None:
        maximum = float(self.maximum_axis_parallelism)
        log_det = float(self.rotation_covariance_log_det)
        if not math.isfinite(maximum) or not 0.0 <= maximum <= 1.0:
            raise ValueError(
                'maximum_axis_parallelism must be finite and in [0, 1]'
            )
        if not math.isfinite(log_det):
            raise ValueError('rotation_covariance_log_det must be finite')
        if (
            isinstance(self.rotation_covariance_rank, bool)
            or not isinstance(self.rotation_covariance_rank, int)
            or not 0 <= self.rotation_covariance_rank <= 3
        ):
            raise ValueError('rotation_covariance_rank must be in [0, 3]')
        ids = tuple(self.nonparallel_axis_pose_ids)
        if len(set(ids)) != len(ids):
            raise ValueError('nonparallel_axis_pose_ids must be unique')
        if any(
            not isinstance(pose_id, str)
            or not pose_id
            or pose_id != pose_id.strip()
            for pose_id in ids
        ):
            raise ValueError(
                'nonparallel_axis_pose_ids must contain trimmed strings'
            )
        object.__setattr__(self, 'maximum_axis_parallelism', maximum)
        object.__setattr__(self, 'rotation_covariance_log_det', log_det)
        object.__setattr__(self, 'nonparallel_axis_pose_ids', ids)

    @property
    def lexicographic_objective(self) -> tuple[float, float]:
        """A minimization key: parallelism first, negative log-det second."""

        return (
            self.maximum_axis_parallelism,
            -self.rotation_covariance_log_det,
        )


def rotation_observations(
    pose_ids: Sequence[str],
    rotation_matrices: Sequence[Sequence[Sequence[float]] | np.ndarray],
    *,
    reference_rotation_matrix: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[RotationObservation, ...]:
    """Express every gripper rotation relative to one recorded reference."""

    ids = tuple(pose_ids)
    rotations = tuple(rotation_matrices)
    if len(ids) != len(rotations):
        raise ValueError(
            'pose_ids and rotation_matrices must have equal lengths'
        )
    if not ids:
        raise ValueError('at least one rotation is required')
    if len(set(ids)) != len(ids):
        raise ValueError('pose_ids must be unique')
    reference = validate_rotation_matrix(reference_rotation_matrix)
    observations: list[RotationObservation] = []
    for pose_id, rotation_value in zip(ids, rotations, strict=True):
        rotation = validate_rotation_matrix(rotation_value)
        relative = reference.T @ rotation
        vector = np.asarray(
            rodrigues_from_rotation_matrix(relative),
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(vector))
        if norm == 0.0:
            raise ValueError(
                f'pose {pose_id!r} has zero rotation relative to the reference'
            )
        axis = vector / norm
        observations.append(
            RotationObservation(
                pose_id=pose_id,
                rotation_vector_rad=tuple(float(value) for value in vector),
                axis=tuple(float(value) for value in axis),
            )
        )
    return tuple(observations)


def _pairwise_nonparallel(
    observations: Sequence[RotationObservation],
    *,
    axis_parallelism_tolerance: float,
) -> bool:
    tolerance = _axis_parallelism_tolerance(axis_parallelism_tolerance)
    parallel_boundary = 1.0 - tolerance
    for left, right in combinations(observations, 2):
        parallelism = abs(float(np.dot(left.axis, right.axis)))
        if parallelism >= parallel_boundary:
            return False
    return True


def find_nonparallel_axis_witness(
    observations: Sequence[RotationObservation],
    *,
    axis_parallelism_tolerance: float,
    minimum_count: int = MINIMUM_NONPARALLEL_AXES,
) -> tuple[str, ...]:
    """Return the lexically first concrete pairwise non-parallel witness."""

    if (
        isinstance(minimum_count, bool)
        or not isinstance(minimum_count, int)
        or minimum_count <= 0
    ):
        raise ValueError('minimum_count must be a positive integer')
    ordered = tuple(sorted(observations, key=lambda item: item.pose_id))
    if len(ordered) < minimum_count:
        return ()
    for subset in combinations(ordered, minimum_count):
        if _pairwise_nonparallel(
            subset,
            axis_parallelism_tolerance=axis_parallelism_tolerance,
        ):
            return tuple(item.pose_id for item in subset)
    return ()


def evaluate_rotation_diversity(
    observations: Sequence[RotationObservation],
    *,
    log_det_epsilon: float,
    axis_parallelism_tolerance: float,
    covariance_rank_tolerance: float,
) -> RotationDiversity:
    """Compute the plan's lexicographic objective and acceptance evidence."""

    values = tuple(observations)
    if not values:
        raise ValueError('at least one rotation observation is required')
    if len({item.pose_id for item in values}) != len(values):
        raise ValueError('rotation observation pose IDs must be unique')
    epsilon = _positive_finite(
        log_det_epsilon,
        field_name='log_det_epsilon',
    )
    _axis_parallelism_tolerance(axis_parallelism_tolerance)
    rank_tolerance = _positive_finite(
        covariance_rank_tolerance,
        field_name='covariance_rank_tolerance',
    )

    maximum_parallelism, log_det = rotation_selection_objective(
        values,
        log_det_epsilon=epsilon,
    )
    vectors = np.asarray(
        [item.rotation_vector_rad for item in values],
        dtype=np.float64,
    )
    centered = vectors - np.mean(vectors, axis=0, keepdims=True)
    covariance = (centered.T @ centered) / float(len(values))
    singular_values = np.linalg.svd(covariance, compute_uv=False)
    rank = int(np.count_nonzero(singular_values > rank_tolerance))
    witness = find_nonparallel_axis_witness(
        values,
        axis_parallelism_tolerance=axis_parallelism_tolerance,
    )
    return RotationDiversity(
        maximum_axis_parallelism=maximum_parallelism,
        rotation_covariance_log_det=float(log_det),
        rotation_covariance_rank=rank,
        nonparallel_axis_pose_ids=witness,
    )


def rotation_selection_objective(
    observations: Sequence[RotationObservation],
    *,
    log_det_epsilon: float,
) -> tuple[float, float]:
    """Return ``(maximum parallelism, covariance log-det)`` cheaply.

    Unlike :func:`evaluate_rotation_diversity`, this helper does not search
    for the five-axis witness and is therefore suitable for repeated subset
    comparisons inside the deterministic selector.
    """

    values = tuple(observations)
    if not values:
        raise ValueError('at least one rotation observation is required')
    epsilon = _positive_finite(
        log_det_epsilon,
        field_name='log_det_epsilon',
    )
    axes = np.asarray([item.axis for item in values], dtype=np.float64)
    if len(values) < 2:
        maximum_parallelism = 0.0
    else:
        gram = np.abs(axes @ axes.T)
        np.fill_diagonal(gram, 0.0)
        maximum_parallelism = float(np.max(gram))

    vectors = np.asarray(
        [item.rotation_vector_rad for item in values],
        dtype=np.float64,
    )
    centered = vectors - np.mean(vectors, axis=0, keepdims=True)
    covariance = (centered.T @ centered) / float(len(values))
    sign, log_det = np.linalg.slogdet(covariance + epsilon * np.eye(3))
    if sign <= 0.0 or not math.isfinite(float(log_det)):
        raise ValueError(
            'regularized rotation covariance is not positive definite'
        )
    return maximum_parallelism, float(log_det)


def require_rotation_diversity(diversity: RotationDiversity) -> None:
    """Apply only the two absolute acceptance rules approved by the plan."""

    if not isinstance(diversity, RotationDiversity):
        raise ValueError('diversity must be RotationDiversity')
    if (
        diversity.rotation_covariance_rank
        != REQUIRED_ROTATION_COVARIANCE_RANK
    ):
        raise ValueError('rotation-vector covariance rank must be exactly 3')
    if len(diversity.nonparallel_axis_pose_ids) < MINIMUM_NONPARALLEL_AXES:
        raise ValueError('pose set requires at least five non-parallel axes')
