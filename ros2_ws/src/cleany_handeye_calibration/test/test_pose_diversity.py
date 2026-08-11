import numpy as np
import pytest

from cleany_handeye_calibration.pose_diversity import (
    RotationObservation,
    evaluate_rotation_diversity,
    require_rotation_diversity,
    rotation_observations,
)
from cleany_handeye_calibration.transforms import (
    rotation_matrix_from_rodrigues,
)


def _observation(pose_id, vector):
    value = np.asarray(vector, dtype=np.float64)
    return RotationObservation(
        pose_id=pose_id,
        rotation_vector_rad=tuple(value),
        axis=tuple(value / np.linalg.norm(value)),
    )


def test_rotation_diversity_records_rank_objective_and_five_axis_witness():
    observations = tuple(
        _observation(f'pose_{index}', vector)
        for index, vector in enumerate(
            (
                (0.30, 0.02, 0.01),
                (0.02, 0.35, 0.03),
                (0.01, 0.04, 0.40),
                (0.25, 0.20, 0.05),
                (-0.10, 0.22, 0.30),
                (0.18, -0.16, 0.24),
            )
        )
    )

    result = evaluate_rotation_diversity(
        observations,
        log_det_epsilon=1.0e-9,
        axis_parallelism_tolerance=0.01,
        covariance_rank_tolerance=1.0e-10,
    )

    assert result.rotation_covariance_rank == 3
    assert len(result.nonparallel_axis_pose_ids) == 5
    assert 0.0 <= result.maximum_axis_parallelism < 1.0
    require_rotation_diversity(result)


def test_rotation_diversity_rejects_rank_deficient_parallel_set():
    values = tuple(
        _observation(f'pose_{index}', (0.1 + 0.01 * index, 0.0, 0.0))
        for index in range(6)
    )

    result = evaluate_rotation_diversity(
        values,
        log_det_epsilon=1.0e-9,
        axis_parallelism_tolerance=0.01,
        covariance_rank_tolerance=1.0e-10,
    )

    assert result.rotation_covariance_rank == 1
    assert result.nonparallel_axis_pose_ids == ()
    with pytest.raises(ValueError, match='rank'):
        require_rotation_diversity(result)


def test_relative_rotation_observations_reject_reference_duplicate():
    with pytest.raises(ValueError, match='zero rotation'):
        rotation_observations(
            ('same_as_reference',),
            (np.eye(3),),
            reference_rotation_matrix=np.eye(3),
        )

    result = rotation_observations(
        ('rotated',),
        (rotation_matrix_from_rodrigues((0.1, 0.2, 0.3)),),
        reference_rotation_matrix=np.eye(3),
    )
    assert result[0].pose_id == 'rotated'
