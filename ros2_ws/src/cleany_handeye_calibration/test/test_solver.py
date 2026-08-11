from dataclasses import replace
import inspect
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from cleany_handeye_calibration.solver import (
    DEFAULT_HAND_EYE_FRAMES,
    HAND_EYE_METHOD_REGISTRY,
    HandEyeFailure,
    HandEyeMethod,
    HandEyeTransformValidityPolicy,
    InvalidHandEyeDataset,
    solve_all_hand_eye_methods,
)
from cleany_handeye_calibration.synthetic import (
    generate_synthetic_hand_eye_dataset,
)
from cleany_handeye_calibration.validation import (
    evaluate_hand_eye_result,
)


VALIDITY_POLICY = HandEyeTransformValidityPolicy(
    max_translation_norm_m=1.0,
)


def _fake_cv_module(calibrate_hand_eye):
    return SimpleNamespace(
        CALIB_HAND_EYE_TSAI=0,
        CALIB_HAND_EYE_PARK=1,
        CALIB_HAND_EYE_HORAUD=2,
        CALIB_HAND_EYE_ANDREFF=3,
        CALIB_HAND_EYE_DANIILIDIS=4,
        calibrateHandEye=calibrate_hand_eye,
    )


def test_all_five_methods_recover_known_20_pose_transform():
    dataset = generate_synthetic_hand_eye_dataset()

    results = solve_all_hand_eye_methods(
        dataset.calibration_samples,
        validity_policy=VALIDITY_POLICY,
    )

    assert tuple(result.method for result in results) == tuple(HandEyeMethod)
    assert len(results) == 5
    for result in results:
        assert result.valid, result.failure_detail
        assert result.gripper_T_camera is not None
        assert result.gripper_T_camera.parent_frame == 'left_gripper_frame'
        assert (
            result.gripper_T_camera.child_frame
            == 'left_wrist_rgb_optical_frame'
        )
        metrics = evaluate_hand_eye_result(
            result,
            dataset.gripper_T_camera_ground_truth,
        )
        assert metrics.translation_error_m < 1.0e-7
        assert metrics.rotation_error_rad < 1.0e-6
        assert result.runtime_ms >= 0.0


def test_each_method_receives_identical_directionally_correct_samples(
    monkeypatch,
):
    dataset = generate_synthetic_hand_eye_dataset()
    original = cv2.calibrateHandEye
    observed_inputs = []

    def recording_calibrate(*args, **kwargs):
        observed_inputs.append(
            tuple(
                tuple(np.array(value, copy=True) for value in group)
                for group in args
            )
        )
        return original(*args, **kwargs)

    monkeypatch.setattr(cv2, 'calibrateHandEye', recording_calibrate)

    results = solve_all_hand_eye_methods(
        dataset.calibration_samples,
        validity_policy=VALIDITY_POLICY,
    )

    assert all(result.valid for result in results)
    assert len(observed_inputs) == 5
    first = observed_inputs[0]
    for method_inputs in observed_inputs[1:]:
        for first_group, method_group in zip(first, method_inputs):
            for first_value, method_value in zip(first_group, method_group):
                np.testing.assert_array_equal(first_value, method_value)
    first_sample = dataset.calibration_samples[0]
    np.testing.assert_array_equal(
        first[0][0],
        first_sample.base_T_gripper.rotation_array(),
    )
    np.testing.assert_array_equal(
        first[1][0].reshape(3),
        first_sample.base_T_gripper.translation_array(),
    )
    np.testing.assert_array_equal(
        first[2][0],
        first_sample.camera_T_target.rotation_array(),
    )
    np.testing.assert_array_equal(
        first[3][0].reshape(3),
        first_sample.camera_T_target.translation_array(),
    )


@pytest.mark.parametrize('inverted_field', ['base', 'camera'])
def test_inverted_transform_direction_is_rejected_before_opencv(
    inverted_field,
):
    dataset = generate_synthetic_hand_eye_dataset()
    samples = list(dataset.calibration_samples)
    first = samples[0]
    if inverted_field == 'base':
        samples[0] = replace(
            first,
            base_T_gripper=first.base_T_gripper.inverse(),
        )
    else:
        samples[0] = replace(
            first,
            camera_T_target=first.camera_T_target.inverse(),
        )
    call_count = 0

    def unexpected_calibrate(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        raise AssertionError('OpenCV must not be called')

    with pytest.raises(InvalidHandEyeDataset):
        solve_all_hand_eye_methods(
            samples,
            validity_policy=VALIDITY_POLICY,
            cv_module=_fake_cv_module(unexpected_calibrate),
        )
    assert call_count == 0


def test_held_out_samples_are_rejected_before_opencv():
    dataset = generate_synthetic_hand_eye_dataset()
    call_count = 0

    def unexpected_calibrate(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        raise AssertionError('OpenCV must not be called')

    with pytest.raises(InvalidHandEyeDataset, match='held_out'):
        solve_all_hand_eye_methods(
            dataset.held_out_samples,
            validity_policy=VALIDITY_POLICY,
            cv_module=_fake_cv_module(unexpected_calibrate),
        )
    assert call_count == 0


def test_method_failures_are_isolated_and_explicit():
    dataset = generate_synthetic_hand_eye_dataset()
    constants = {
        method_spec.method: index
        for index, method_spec in enumerate(HAND_EYE_METHOD_REGISTRY)
    }

    def scripted_calibrate(*args, method):
        if method == constants[HandEyeMethod.TSAI]:
            raise RuntimeError('forced failure')
        if method == constants[HandEyeMethod.PARK]:
            return np.eye(3), np.full((3, 1), float('nan'))
        if method == constants[HandEyeMethod.HORAUD]:
            return np.diag((1.0, 1.0, -1.0)), np.zeros((3, 1))
        if method == constants[HandEyeMethod.ANDREFF]:
            return None
        return np.eye(3), np.zeros((3, 1))

    results = solve_all_hand_eye_methods(
        dataset.calibration_samples,
        validity_policy=VALIDITY_POLICY,
        cv_module=_fake_cv_module(scripted_calibrate),
    )
    by_method = {result.method: result for result in results}

    assert len(results) == 5
    assert (
        by_method[HandEyeMethod.TSAI].failure_reason
        is HandEyeFailure.METHOD_EXCEPTION
    )
    assert (
        by_method[HandEyeMethod.PARK].failure_reason
        is HandEyeFailure.INVALID_TRANSFORM
    )
    assert (
        by_method[HandEyeMethod.HORAUD].failure_reason
        is HandEyeFailure.INVALID_TRANSFORM
    )
    assert (
        by_method[HandEyeMethod.ANDREFF].failure_reason
        is HandEyeFailure.MALFORMED_OUTPUT
    )
    assert by_method[HandEyeMethod.DANIILIDIS].valid
    assert all(result.runtime_ms >= 0.0 for result in results)


def test_solver_api_has_no_ground_truth_parameter_or_ros_dependency():
    parameters = inspect.signature(solve_all_hand_eye_methods).parameters
    solver_module = inspect.getmodule(solve_all_hand_eye_methods)

    assert 'ground_truth' not in parameters
    assert parameters['validity_policy'].default is inspect.Parameter.empty
    assert solver_module is not None
    assert 'rclpy' not in inspect.getsource(solver_module)
    assert DEFAULT_HAND_EYE_FRAMES.base_frame == 'base_link'


@pytest.mark.parametrize('invalid_maximum', [True, 0.0, -1.0, float('inf')])
def test_translation_validity_policy_requires_positive_finite_bound(
    invalid_maximum,
):
    with pytest.raises(ValueError, match='positive finite'):
        HandEyeTransformValidityPolicy(
            max_translation_norm_m=invalid_maximum,
        )


def test_finite_translation_outside_physical_bound_is_isolated():
    dataset = generate_synthetic_hand_eye_dataset()
    call_index = 0

    def scripted_calibrate(*args, method):
        nonlocal call_index
        call_index += 1
        if call_index == 2:
            return np.eye(3), np.array((1.000001, 0.0, 0.0))
        if call_index == 3:
            return np.eye(3), np.array((1.0e300, 0.0, 0.0))
        return np.eye(3), np.array((1.0, 0.0, 0.0))

    results = solve_all_hand_eye_methods(
        dataset.calibration_samples,
        validity_policy=VALIDITY_POLICY,
        cv_module=_fake_cv_module(scripted_calibrate),
    )

    assert results[0].valid
    assert not results[1].valid
    assert results[1].failure_reason is HandEyeFailure.INVALID_TRANSFORM
    assert '1.000001 m > 1 m' in (results[1].failure_detail or '')
    assert not results[2].valid
    assert results[2].failure_reason is HandEyeFailure.INVALID_TRANSFORM
    assert all(result.valid for result in (*results[:1], *results[3:]))
