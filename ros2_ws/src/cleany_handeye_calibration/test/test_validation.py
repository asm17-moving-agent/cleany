from dataclasses import replace

import numpy as np
import pytest

from cleany_handeye_calibration.models import SampleSplit
from cleany_handeye_calibration.solver import (
    HandEyeTransformValidityPolicy,
    solve_all_hand_eye_methods,
)
from cleany_handeye_calibration.synthetic import (
    generate_synthetic_hand_eye_dataset,
)
from cleany_handeye_calibration.transforms import RigidTransform
from cleany_handeye_calibration.validation import (
    evaluate_hand_eye_result,
    held_out_base_target_consistency,
    transform_error_metrics,
)


def test_synthetic_dataset_has_20_calibration_and_5_held_out_poses():
    dataset = generate_synthetic_hand_eye_dataset()

    assert len(dataset.calibration_samples) == 20
    assert len(dataset.held_out_samples) == 5
    assert all(
        sample.split is SampleSplit.CALIBRATION
        for sample in dataset.calibration_samples
    )
    assert all(
        sample.split is SampleSplit.HELD_OUT
        for sample in dataset.held_out_samples
    )
    for sample in (
        *dataset.calibration_samples,
        *dataset.held_out_samples,
    ):
        reconstructed = (
            sample.base_T_gripper
            @ dataset.gripper_T_camera_ground_truth
            @ sample.camera_T_target
        )
        np.testing.assert_allclose(
            reconstructed.as_homogeneous_matrix(),
            dataset.base_T_target_ground_truth.as_homogeneous_matrix(),
            rtol=0.0,
            atol=1.0e-10,
        )


def test_transform_error_reports_metres_and_radians():
    reference = RigidTransform.identity('same_frame')
    estimate = RigidTransform.from_rodrigues(
        parent_frame='same_frame',
        child_frame='same_frame',
        translation_m=(0.001, 0.0, 0.0),
        rodrigues_vector=(0.0, 0.0, 0.01),
    )

    metrics = transform_error_metrics(estimate, reference)

    assert metrics.translation_error_m == pytest.approx(0.001)
    assert metrics.rotation_error_rad == pytest.approx(0.01)


def test_all_methods_have_accuracy_metrics_only_in_evaluator():
    dataset = generate_synthetic_hand_eye_dataset()
    results = solve_all_hand_eye_methods(
        dataset.calibration_samples,
        validity_policy=HandEyeTransformValidityPolicy(
            max_translation_norm_m=1.0,
        ),
    )

    evaluations = tuple(
        evaluate_hand_eye_result(
            result,
            dataset.gripper_T_camera_ground_truth,
        )
        for result in results
    )

    assert len(evaluations) == 5
    assert all(value.translation_error_m < 1.0e-7 for value in evaluations)
    assert all(value.rotation_error_rad < 1.0e-6 for value in evaluations)


def test_exact_held_out_samples_have_near_zero_pairwise_consistency():
    dataset = generate_synthetic_hand_eye_dataset()

    metrics = held_out_base_target_consistency(
        dataset.held_out_samples,
        dataset.gripper_T_camera_ground_truth,
    )

    assert metrics.sample_count == 5
    assert metrics.pair_count == 10
    assert metrics.translation_median_m < 1.0e-12
    assert metrics.translation_p95_m < 1.0e-12
    assert metrics.rotation_median_rad < 1.0e-12
    assert metrics.rotation_p95_rad < 1.0e-12


def test_held_out_consistency_detects_one_perturbed_observation():
    dataset = generate_synthetic_hand_eye_dataset()
    samples = list(dataset.held_out_samples)
    last = samples[-1]
    camera_T_target = last.camera_T_target
    perturbed_camera_T_target = RigidTransform(
        parent_frame=camera_T_target.parent_frame,
        child_frame=camera_T_target.child_frame,
        rotation_matrix=camera_T_target.rotation_matrix,
        translation_m=(
            camera_T_target.translation_m[0] + 0.01,
            camera_T_target.translation_m[1],
            camera_T_target.translation_m[2],
        ),
    )
    samples[-1] = replace(
        last,
        camera_T_target=perturbed_camera_T_target,
    )

    metrics = held_out_base_target_consistency(
        samples,
        dataset.gripper_T_camera_ground_truth,
    )

    assert metrics.translation_p95_m > 0.009


def test_held_out_consistency_rejects_calibration_split():
    dataset = generate_synthetic_hand_eye_dataset()

    with pytest.raises(ValueError, match='held_out split'):
        held_out_base_target_consistency(
            dataset.calibration_samples[:2],
            dataset.gripper_T_camera_ground_truth,
        )
