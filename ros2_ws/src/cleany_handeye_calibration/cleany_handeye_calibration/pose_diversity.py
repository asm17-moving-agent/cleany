"""구체화된 보정 자세 집합의 회전 다양성 지표입니다.

선택 목적은 상대 비교입니다. 쌍별 축 평행성의 최댓값이 작을수록 우선하며,
같으면 공분산 log-determinant가 큰 쪽을 택합니다. 두 값 모두 절대 수락 임계값으로
쓰지 않습니다. 수락에는 쌍별로 평행하지 않은 축이 최소 다섯 개이고 공분산 rank가
3이어야 한다는 조건만 적용하며 수치 허용 오차는 호출자가 제공합니다.
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
    """상대 SO(3) 로그 벡터 하나와 정규화된 회전축입니다."""

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
    """검토 가능한 지표와 다양성을 입증하는 구체적인 축 다섯 개입니다."""

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
        """최소화 정렬 키로 평행성을 우선하고 음의 log-det을 다음으로 사용합니다."""

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
    """기록된 기준 자세 하나에 대한 상대값으로 모든 그리퍼 회전을 표현합니다."""

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
    """쌍별로 평행하지 않은 구체적인 축 집합 중 사전순 첫 집합을 반환합니다."""

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
    """사전식 선택 목적값과 수락 근거를 계산합니다."""

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
    """(최대 평행성, 공분산 log-det)을 적은 비용으로 계산합니다.

    evaluate_rotation_diversity와 달리 다섯 축의 입증 집합을 탐색하지 않으므로
    결정적 선택기에서 부분 집합을 반복 비교할 때 적합합니다.
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
    """명시된 두 가지 절대 수락 규칙만 적용합니다."""

    if not isinstance(diversity, RotationDiversity):
        raise ValueError('diversity must be RotationDiversity')
    if (
        diversity.rotation_covariance_rank
        != REQUIRED_ROTATION_COVARIANCE_RANK
    ):
        raise ValueError('rotation-vector covariance rank must be exactly 3')
    if len(diversity.nonparallel_axis_pose_ids) < MINIMUM_NONPARALLEL_AXES:
        raise ValueError('pose set requires at least five non-parallel axes')
