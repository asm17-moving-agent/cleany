import cv2
import numpy as np
import pytest

import cleany_handeye_calibration.pnp as pnp_module
from cleany_handeye_calibration.pnp import (
    PnpCandidate,
    PnpCandidateFailure,
    PnpFailure,
    select_pnp_candidates,
    solve_planar_pnp,
)
from cleany_handeye_calibration.target_detector import (
    QUADRANTS,
    CharucoDetection,
    analyze_charuco_corners,
    charuco_object_points_m,
)
from cleany_handeye_calibration.transforms import RigidTransform


CAMERA_MATRIX = np.array(
    [
        [227.751496, 0.0, 319.5],
        [0.0, 227.751496, 239.5],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DISTORTION = np.zeros(5, dtype=np.float64)
OBJECT_POINTS = charuco_object_points_m()


def _project(
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
) -> np.ndarray:
    projected, _ = cv2.projectPoints(
        OBJECT_POINTS,
        rotation_vector,
        translation_vector,
        CAMERA_MATRIX,
        DISTORTION,
    )
    return projected.reshape(-1, 2)


def _detection(
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
):
    return analyze_charuco_corners(
        np.arange(24, dtype=np.int32),
        _project(rotation_vector, translation_vector),
    )


def _rotation_error_rad(actual: RigidTransform, expected_rvec) -> float:
    expected_rotation, _ = cv2.Rodrigues(
        np.asarray(expected_rvec, dtype=np.float64)
    )
    difference = expected_rotation.T @ actual.rotation_array()
    cosine = np.clip((np.trace(difference) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def test_ippe_recovers_oblique_camera_target_pose_and_records_candidates():
    expected_rvec = np.array((0.35, -0.2, 0.1), dtype=np.float64)
    expected_tvec = np.array((-0.10, -0.07, 0.55), dtype=np.float64)

    result = solve_planar_pnp(
        _detection(expected_rvec, expected_tvec),
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='left_wrist_rgb_optical_frame',
        target_frame='charuco_target',
    )

    assert result.valid
    assert not result.ambiguous
    assert result.camera_T_target is not None
    assert len(result.candidates) == 2
    assert all(
        candidate.raw_camera_T_target is not None
        for candidate in result.candidates
    )
    assert all(
        candidate.refined_camera_T_target is not None
        for candidate in result.candidates
    )
    np.testing.assert_allclose(
        result.camera_T_target.translation_m,
        expected_tvec,
        rtol=0.0,
        atol=1.0e-8,
    )
    assert _rotation_error_rad(result.camera_T_target, expected_rvec) < 1.0e-7


def test_solver_calls_generic_pnp_with_ippe_flag(monkeypatch):
    expected_rvec = np.array((0.35, -0.2, 0.1), dtype=np.float64)
    expected_tvec = np.array((-0.10, -0.07, 0.55), dtype=np.float64)
    original = cv2.solvePnPGeneric
    observed_flags = []

    def recording_solve(*args, **kwargs):
        observed_flags.append(kwargs.get('flags'))
        return original(*args, **kwargs)

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', recording_solve)

    result = solve_planar_pnp(
        _detection(expected_rvec, expected_tvec),
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.valid
    assert observed_flags == [cv2.SOLVEPNP_IPPE]


def test_fronto_parallel_equal_solutions_are_rejected_as_ambiguous():
    expected_rvec = np.zeros(3, dtype=np.float64)
    expected_tvec = np.array((-0.105, -0.075, 0.55), dtype=np.float64)

    result = solve_planar_pnp(
        _detection(expected_rvec, expected_tvec),
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert not result.valid
    assert result.ambiguous
    assert result.failure_reason is PnpFailure.AMBIGUOUS_PNP
    assert result.camera_T_target is None


def _valid_candidate(index: int, rmse: float) -> PnpCandidate:
    pose = RigidTransform(
        parent_frame='camera',
        child_frame='target',
        rotation_matrix=np.eye(3),
        translation_m=(0.0, 0.0, 1.0),
    )
    return PnpCandidate(
        index=index,
        valid=True,
        failure_reason=None,
        raw_camera_T_target=pose,
        raw_min_depth_m=1.0,
        raw_reprojection_rmse_px=rmse,
        refined_camera_T_target=pose,
        refined_min_depth_m=1.0,
        refined_reprojection_rmse_px=rmse,
    )


@pytest.mark.parametrize(
    ('second_rmse', 'ambiguous'),
    [(1.049, True), (1.05, False)],
)
def test_ambiguity_ratio_uses_strict_1_05_threshold(
    second_rmse,
    ambiguous,
):
    result = select_pnp_candidates(
        (_valid_candidate(0, 1.0), _valid_candidate(1, second_rmse))
    )

    assert result.ambiguous is ambiguous
    assert result.valid is not ambiguous


def test_candidate_with_point_behind_camera_is_rejected(monkeypatch):
    rvec = np.zeros((3, 1), dtype=np.float64)
    front_tvec = np.array((0.0, 0.0, 1.0), dtype=np.float64)
    detection = _detection(rvec.reshape(3), front_tvec)

    def fake_generic(*args, **kwargs):
        return (
            2,
            (rvec.copy(), rvec.copy()),
            (
                np.array((0.0, 0.0, -1.0), dtype=np.float64).reshape(3, 1),
                front_tvec.reshape(3, 1),
            ),
            np.zeros((2, 1), dtype=np.float64),
        )

    def no_refinement(*args, **kwargs):
        return np.array(args[4], copy=True), np.array(args[5], copy=True)

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', fake_generic)
    monkeypatch.setattr(pnp_module.cv2, 'solvePnPRefineVVS', no_refinement)

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.valid
    assert not result.candidates[0].valid
    assert (
        result.candidates[0].failure_reason
        is PnpCandidateFailure.RAW_POINT_BEHIND_CAMERA
    )
    assert result.candidates[1].valid


def test_non_finite_candidate_is_rejected_without_losing_valid_one(
    monkeypatch,
):
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.array((0.0, 0.0, 1.0), dtype=np.float64).reshape(3, 1)
    detection = _detection(rvec.reshape(3), tvec.reshape(3))

    def fake_generic(*args, **kwargs):
        invalid_rvec = np.full((3, 1), float('nan'))
        return 2, (invalid_rvec, rvec.copy()), (tvec.copy(), tvec.copy()), None

    def no_refinement(*args, **kwargs):
        return np.array(args[4], copy=True), np.array(args[5], copy=True)

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', fake_generic)
    monkeypatch.setattr(pnp_module.cv2, 'solvePnPRefineVVS', no_refinement)

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.valid
    assert (
        result.candidates[0].failure_reason
        is PnpCandidateFailure.INVALID_RAW_POSE
    )
    assert result.candidates[1].valid


@pytest.mark.parametrize(
    ('corner_ids', 'covered_quadrants'),
    [
        (tuple(range(15)), QUADRANTS),
        (tuple(reversed(range(24))), QUADRANTS),
        ((*range(23), 22), QUADRANTS),
        ((*range(23), 24), QUADRANTS),
        (
            tuple(
                value
                for value in range(24)
                if value not in {15, 16, 17, 21, 22, 23}
            ),
            ('top_left', 'top_right', 'bottom_left'),
        ),
    ],
)
def test_pnp_revalidates_forged_valid_detection_contract(
    corner_ids,
    covered_quadrants,
    monkeypatch,
):
    count = len(corner_ids)
    object_points = tuple(
        tuple(float(value) for value in point)
        for point in OBJECT_POINTS[:count]
    )
    forged = CharucoDetection(
        valid=True,
        failure_reason=None,
        corner_ids=corner_ids,
        image_points_px=tuple((float(index), 0.0) for index in range(count)),
        object_points_m=object_points,
        covered_quadrants=covered_quadrants,
    )

    def unexpected_call(*args, **kwargs):
        raise AssertionError('forged detection must not reach OpenCV')

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', unexpected_call)

    result = solve_planar_pnp(
        forged,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert not result.valid
    assert result.failure_reason is PnpFailure.INVALID_CORRESPONDENCES


def test_pnp_rejects_forged_noncanonical_object_points(monkeypatch):
    detection = _detection(
        np.array((0.2, -0.1, 0.05)),
        np.array((-0.1, -0.07, 0.6)),
    )
    forged_points = list(detection.object_points_m)
    forged_points[0] = (9.0, 9.0, 0.0)
    forged = CharucoDetection(
        valid=True,
        failure_reason=None,
        corner_ids=detection.corner_ids,
        image_points_px=detection.image_points_px,
        object_points_m=tuple(forged_points),
        covered_quadrants=detection.covered_quadrants,
    )

    def unexpected_call(*args, **kwargs):
        raise AssertionError('forged detection must not reach OpenCV')

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', unexpected_call)

    result = solve_planar_pnp(
        forged,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert not result.valid
    assert result.failure_reason is PnpFailure.INVALID_CORRESPONDENCES


@pytest.mark.parametrize(
    'generic_result',
    [
        None,
        (),
        (2, None, None, None),
        (
            1,
            (np.zeros((3, 1)),),
            (np.array((0.0, 0.0, 1.0)).reshape(3, 1),),
            None,
        ),
    ],
)
def test_malformed_generic_pnp_return_maps_to_candidate_count_failure(
    generic_result,
    monkeypatch,
):
    detection = _detection(
        np.array((0.2, -0.1, 0.05)),
        np.array((-0.1, -0.07, 0.6)),
    )

    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPGeneric',
        lambda *args, **kwargs: generic_result,
    )

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert not result.valid
    assert result.failure_reason is PnpFailure.UNEXPECTED_CANDIDATE_COUNT


def _two_front_candidates():
    rotation = np.zeros((3, 1), dtype=np.float64)
    translation = np.array((0.0, 0.0, 1.0)).reshape(3, 1)
    return (
        2,
        (rotation.copy(), rotation.copy()),
        (translation.copy(), translation.copy()),
        np.zeros((2, 1), dtype=np.float64),
    )


def test_refinement_failure_is_recorded_per_candidate(monkeypatch):
    detection = _detection(np.zeros(3), np.array((0.0, 0.0, 1.0)))

    def fail_refinement(*args, **kwargs):
        raise cv2.error('forced refinement failure')

    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPGeneric',
        lambda *args, **kwargs: _two_front_candidates(),
    )
    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPRefineVVS',
        fail_refinement,
    )

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.failure_reason is PnpFailure.NO_VALID_CANDIDATE
    assert all(
        candidate.failure_reason is PnpCandidateFailure.REFINEMENT_FAILED
        for candidate in result.candidates
    )


def test_refined_behind_camera_pose_is_recorded_per_candidate(monkeypatch):
    detection = _detection(np.zeros(3), np.array((0.0, 0.0, 1.0)))

    def refine_behind(*args, **kwargs):
        return (
            np.array(args[4], copy=True),
            np.array((0.0, 0.0, -1.0), dtype=np.float64).reshape(3, 1),
        )

    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPGeneric',
        lambda *args, **kwargs: _two_front_candidates(),
    )
    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPRefineVVS',
        refine_behind,
    )

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.failure_reason is PnpFailure.NO_VALID_CANDIDATE
    assert all(
        candidate.failure_reason
        is PnpCandidateFailure.REFINED_POINT_BEHIND_CAMERA
        for candidate in result.candidates
    )


def test_refined_non_finite_pose_is_recorded_per_candidate(monkeypatch):
    detection = _detection(np.zeros(3), np.array((0.0, 0.0, 1.0)))

    def refine_non_finite(*args, **kwargs):
        return (
            np.full((3, 1), float('nan')),
            np.array(args[5], copy=True),
        )

    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPGeneric',
        lambda *args, **kwargs: _two_front_candidates(),
    )
    monkeypatch.setattr(
        pnp_module.cv2,
        'solvePnPRefineVVS',
        refine_non_finite,
    )

    result = solve_planar_pnp(
        detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert result.failure_reason is PnpFailure.NO_VALID_CANDIDATE
    assert all(
        candidate.failure_reason
        is PnpCandidateFailure.INVALID_REFINED_POSE
        for candidate in result.candidates
    )


def test_invalid_detection_is_not_sent_to_opencv(monkeypatch):
    invalid_detection = analyze_charuco_corners(
        np.arange(15, dtype=np.int32),
        np.zeros((15, 2), dtype=np.float64),
    )

    def unexpected_call(*args, **kwargs):
        raise AssertionError('OpenCV must not be called')

    monkeypatch.setattr(pnp_module.cv2, 'solvePnPGeneric', unexpected_call)

    result = solve_planar_pnp(
        invalid_detection,
        camera_matrix=CAMERA_MATRIX,
        distortion_coefficients=DISTORTION,
        camera_frame='camera',
        target_frame='target',
    )

    assert not result.valid
    assert result.failure_reason is PnpFailure.INVALID_DETECTION
