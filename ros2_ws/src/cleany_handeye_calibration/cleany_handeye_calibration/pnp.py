"""Planar ChArUco PnP with explicit IPPE candidate diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

import cv2
import numpy as np

from cleany_handeye_calibration.target_detector import (
    CharucoDetection,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


PNP_METHOD_NAME = 'SOLVEPNP_IPPE'
AMBIGUITY_RATIO_MIN = 1.05
ZERO_RMSE_TOLERANCE_PX = 1.0e-12


class PnpCandidateFailure(str, Enum):
    INVALID_RAW_POSE = 'invalid_raw_pose'
    RAW_POINT_BEHIND_CAMERA = 'raw_point_behind_camera'
    RAW_REPROJECTION_FAILED = 'raw_reprojection_failed'
    REFINEMENT_FAILED = 'refinement_failed'
    INVALID_REFINED_POSE = 'invalid_refined_pose'
    REFINED_POINT_BEHIND_CAMERA = 'refined_point_behind_camera'
    REFINED_REPROJECTION_FAILED = 'refined_reprojection_failed'


class PnpFailure(str, Enum):
    INVALID_DETECTION = 'invalid_detection'
    INVALID_CORRESPONDENCES = 'invalid_correspondences'
    OPENCV_ERROR = 'opencv_error'
    UNEXPECTED_CANDIDATE_COUNT = 'unexpected_candidate_count'
    NO_VALID_CANDIDATE = 'no_valid_candidate'
    AMBIGUOUS_PNP = 'ambiguous_pnp'


@dataclass(frozen=True, slots=True)
class PnpCandidate:
    index: int
    valid: bool
    failure_reason: PnpCandidateFailure | None
    raw_camera_T_target: RigidTransform | None
    raw_min_depth_m: float | None
    raw_reprojection_rmse_px: float | None
    refined_camera_T_target: RigidTransform | None
    refined_min_depth_m: float | None
    refined_reprojection_rmse_px: float | None


@dataclass(frozen=True, slots=True)
class PnpResult:
    valid: bool
    method: str
    failure_reason: PnpFailure | None
    failure_detail: str | None
    ambiguous: bool
    selected_candidate_index: int | None
    camera_T_target: RigidTransform | None
    candidates: tuple[PnpCandidate, ...]


def _invalid_result(
    reason: PnpFailure,
    *,
    detail: str | None = None,
    ambiguous: bool = False,
    candidates: tuple[PnpCandidate, ...] = (),
) -> PnpResult:
    return PnpResult(
        valid=False,
        method=PNP_METHOD_NAME,
        failure_reason=reason,
        failure_detail=detail,
        ambiguous=ambiguous,
        selected_candidate_index=None,
        camera_T_target=None,
        candidates=candidates,
    )


def _camera_parameters(
    camera_matrix: Sequence[Sequence[float]] | np.ndarray,
    distortion_coefficients: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        matrix = np.asarray(camera_matrix, dtype=np.float64)
        distortion = np.asarray(
            distortion_coefficients,
            dtype=np.float64,
        )
    except (TypeError, ValueError) as error:
        raise ValueError('camera parameters must be numeric') from error
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('camera_matrix must be a finite 3 x 3 matrix')
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    if not np.allclose(
        matrix[2],
        np.array((0.0, 0.0, 1.0)),
        rtol=0.0,
        atol=1.0e-12,
    ):
        raise ValueError('camera_matrix bottom row must be [0, 0, 1]')
    if distortion.ndim == 2 and 1 in distortion.shape:
        distortion = distortion.reshape(-1)
    if distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14):
        raise ValueError(
            'distortion_coefficients must contain 4, 5, 8, 12, or 14 values'
        )
    if not np.all(np.isfinite(distortion)):
        raise ValueError(
            'distortion_coefficients must contain only finite values'
        )
    return (
        np.ascontiguousarray(matrix),
        np.ascontiguousarray(distortion.reshape(-1, 1)),
    )


def _correspondences(
    detection: CharucoDetection,
) -> tuple[np.ndarray, np.ndarray]:
    if not isinstance(detection, CharucoDetection) or not detection.valid:
        raise ValueError('PnP requires a valid ChArUco detection')
    validated = analyze_charuco_corners(
        detection.corner_ids,
        detection.image_points_px,
    )
    if not validated.valid:
        reason = (
            None
            if validated.failure_reason is None
            else validated.failure_reason.value
        )
        raise ValueError(
            f'PnP detection contract validation failed: {reason}'
        )
    if detection.corner_ids != validated.corner_ids:
        raise ValueError('PnP corner IDs must be sorted in ascending order')
    if detection.covered_quadrants != validated.covered_quadrants:
        raise ValueError('PnP board quadrant coverage is inconsistent')
    object_points = np.ascontiguousarray(
        detection.object_points_array(),
        dtype=np.float64,
    )
    image_points = np.ascontiguousarray(
        detection.image_points_array(),
        dtype=np.float64,
    )
    if (
        object_points.ndim != 2
        or object_points.shape[1:] != (3,)
        or image_points.ndim != 2
        or image_points.shape[1:] != (2,)
        or object_points.shape[0] != image_points.shape[0]
        or object_points.shape[0] < 4
    ):
        raise ValueError('PnP correspondences have invalid shapes')
    if not np.all(np.isfinite(object_points)) or not np.all(
        np.isfinite(image_points)
    ):
        raise ValueError('PnP correspondences must be finite')
    if not np.allclose(
        object_points,
        validated.object_points_array(),
        rtol=0.0,
        atol=1.0e-12,
    ):
        raise ValueError(
            'PnP object points do not match the canonical ChArUco IDs'
        )
    centered = object_points - np.mean(object_points, axis=0)
    if np.linalg.matrix_rank(centered, tol=1.0e-12) != 2:
        raise ValueError('SOLVEPNP_IPPE requires planar non-collinear points')
    return object_points, image_points


def _vector3(values, *, field_name: str) -> np.ndarray:
    try:
        vector = np.asarray(values, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{field_name} must be numeric') from error
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f'{field_name} must contain three finite values')
    return vector.reshape(3, 1)


def _pose_from_vectors(
    rotation_vector,
    translation_vector,
    *,
    camera_frame: str,
    target_frame: str,
) -> tuple[RigidTransform, np.ndarray, np.ndarray]:
    rotation = _vector3(rotation_vector, field_name='rotation_vector')
    translation = _vector3(
        translation_vector,
        field_name='translation_vector',
    )
    transform = RigidTransform.from_rodrigues(
        parent_frame=camera_frame,
        child_frame=target_frame,
        translation_m=translation.reshape(3),
        rodrigues_vector=rotation.reshape(3),
    )
    return transform, rotation, translation


def _minimum_depth_m(
    pose: RigidTransform,
    object_points: np.ndarray,
) -> float:
    camera_points = (
        pose.rotation_array() @ object_points.T
        + pose.translation_array().reshape(3, 1)
    )
    return float(np.min(camera_points[2]))


def _reprojection_rmse_px(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion_coefficients,
    )
    projected = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    if projected.shape != image_points.shape or not np.all(
        np.isfinite(projected)
    ):
        raise ValueError('OpenCV returned invalid projected points')
    residuals = projected - image_points
    return float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))


def _invalid_candidate(
    index: int,
    reason: PnpCandidateFailure,
    *,
    raw_pose: RigidTransform | None = None,
    raw_min_depth_m: float | None = None,
    raw_rmse_px: float | None = None,
    refined_pose: RigidTransform | None = None,
    refined_min_depth_m: float | None = None,
) -> PnpCandidate:
    return PnpCandidate(
        index=index,
        valid=False,
        failure_reason=reason,
        raw_camera_T_target=raw_pose,
        raw_min_depth_m=raw_min_depth_m,
        raw_reprojection_rmse_px=raw_rmse_px,
        refined_camera_T_target=refined_pose,
        refined_min_depth_m=refined_min_depth_m,
        refined_reprojection_rmse_px=None,
    )


def evaluate_pnp_candidate(
    *,
    index: int,
    rotation_vector,
    translation_vector,
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    camera_frame: str,
    target_frame: str,
) -> PnpCandidate:
    """Validate and VVS-refine one raw IPPE candidate."""

    try:
        raw_pose, raw_rotation, raw_translation = _pose_from_vectors(
            rotation_vector,
            translation_vector,
            camera_frame=camera_frame,
            target_frame=target_frame,
        )
    except (ValueError, cv2.error):
        return _invalid_candidate(
            index,
            PnpCandidateFailure.INVALID_RAW_POSE,
        )
    raw_min_depth = _minimum_depth_m(raw_pose, object_points)
    if not math.isfinite(raw_min_depth) or raw_min_depth <= 0.0:
        return _invalid_candidate(
            index,
            PnpCandidateFailure.RAW_POINT_BEHIND_CAMERA,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
        )
    try:
        raw_rmse = _reprojection_rmse_px(
            object_points,
            image_points,
            raw_rotation,
            raw_translation,
            camera_matrix,
            distortion_coefficients,
        )
    except (ValueError, cv2.error):
        return _invalid_candidate(
            index,
            PnpCandidateFailure.RAW_REPROJECTION_FAILED,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
        )

    try:
        refined_rotation, refined_translation = cv2.solvePnPRefineVVS(
            object_points,
            image_points,
            camera_matrix,
            distortion_coefficients,
            raw_rotation.copy(),
            raw_translation.copy(),
        )
    except cv2.error:
        return _invalid_candidate(
            index,
            PnpCandidateFailure.REFINEMENT_FAILED,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
            raw_rmse_px=raw_rmse,
        )
    try:
        refined_pose, refined_rotation, refined_translation = (
            _pose_from_vectors(
                refined_rotation,
                refined_translation,
                camera_frame=camera_frame,
                target_frame=target_frame,
            )
        )
    except (ValueError, cv2.error):
        return _invalid_candidate(
            index,
            PnpCandidateFailure.INVALID_REFINED_POSE,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
            raw_rmse_px=raw_rmse,
        )
    refined_min_depth = _minimum_depth_m(refined_pose, object_points)
    if not math.isfinite(refined_min_depth) or refined_min_depth <= 0.0:
        return _invalid_candidate(
            index,
            PnpCandidateFailure.REFINED_POINT_BEHIND_CAMERA,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
            raw_rmse_px=raw_rmse,
            refined_pose=refined_pose,
            refined_min_depth_m=refined_min_depth,
        )
    try:
        refined_rmse = _reprojection_rmse_px(
            object_points,
            image_points,
            refined_rotation,
            refined_translation,
            camera_matrix,
            distortion_coefficients,
        )
    except (ValueError, cv2.error):
        return _invalid_candidate(
            index,
            PnpCandidateFailure.REFINED_REPROJECTION_FAILED,
            raw_pose=raw_pose,
            raw_min_depth_m=raw_min_depth,
            raw_rmse_px=raw_rmse,
            refined_pose=refined_pose,
            refined_min_depth_m=refined_min_depth,
        )
    return PnpCandidate(
        index=index,
        valid=True,
        failure_reason=None,
        raw_camera_T_target=raw_pose,
        raw_min_depth_m=raw_min_depth,
        raw_reprojection_rmse_px=raw_rmse,
        refined_camera_T_target=refined_pose,
        refined_min_depth_m=refined_min_depth,
        refined_reprojection_rmse_px=refined_rmse,
    )


def select_pnp_candidates(
    candidates: Sequence[PnpCandidate],
) -> PnpResult:
    """Select the lowest-RMSE valid candidate or reject ambiguity."""

    candidate_tuple = tuple(candidates)
    valid_candidates = sorted(
        (
            candidate
            for candidate in candidate_tuple
            if candidate.valid
            and candidate.refined_camera_T_target is not None
            and candidate.refined_reprojection_rmse_px is not None
            and math.isfinite(candidate.refined_reprojection_rmse_px)
        ),
        key=lambda candidate: (
            candidate.refined_reprojection_rmse_px,
            candidate.index,
        ),
    )
    if not valid_candidates:
        return _invalid_result(
            PnpFailure.NO_VALID_CANDIDATE,
            candidates=candidate_tuple,
        )
    if len(valid_candidates) >= 2:
        best_rmse = valid_candidates[0].refined_reprojection_rmse_px
        second_rmse = valid_candidates[1].refined_reprojection_rmse_px
        assert best_rmse is not None
        assert second_rmse is not None
        both_numerically_zero = (
            best_rmse <= ZERO_RMSE_TOLERANCE_PX
            and second_rmse <= ZERO_RMSE_TOLERANCE_PX
        )
        insufficient_ratio = (
            best_rmse > ZERO_RMSE_TOLERANCE_PX
            and second_rmse / best_rmse < AMBIGUITY_RATIO_MIN
        )
        if both_numerically_zero or insufficient_ratio:
            return _invalid_result(
                PnpFailure.AMBIGUOUS_PNP,
                ambiguous=True,
                candidates=candidate_tuple,
            )

    selected = valid_candidates[0]
    return PnpResult(
        valid=True,
        method=PNP_METHOD_NAME,
        failure_reason=None,
        failure_detail=None,
        ambiguous=False,
        selected_candidate_index=selected.index,
        camera_T_target=selected.refined_camera_T_target,
        candidates=candidate_tuple,
    )


def solve_planar_pnp(
    detection: CharucoDetection,
    *,
    camera_matrix: Sequence[Sequence[float]] | np.ndarray,
    distortion_coefficients: Sequence[float] | np.ndarray,
    camera_frame: str,
    target_frame: str,
) -> PnpResult:
    """Run two-solution IPPE, refine both candidates, and select by RMSE."""

    if not isinstance(detection, CharucoDetection) or not detection.valid:
        return _invalid_result(PnpFailure.INVALID_DETECTION)
    try:
        matrix, distortion = _camera_parameters(
            camera_matrix,
            distortion_coefficients,
        )
        object_points, image_points = _correspondences(detection)
    except ValueError as error:
        return _invalid_result(
            PnpFailure.INVALID_CORRESPONDENCES,
            detail=str(error),
        )
    try:
        generic_result = cv2.solvePnPGeneric(
            object_points,
            image_points,
            matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error as error:
        return _invalid_result(
            PnpFailure.OPENCV_ERROR,
            detail=str(error),
        )
    try:
        if not isinstance(generic_result, tuple) or len(generic_result) != 4:
            raise ValueError('expected a four-element result tuple')
        count, rotation_vectors, translation_vectors, _ = generic_result
        candidate_count = int(count)
        rotation_vectors = tuple(rotation_vectors)
        translation_vectors = tuple(translation_vectors)
    except (TypeError, ValueError, OverflowError) as error:
        return _invalid_result(
            PnpFailure.UNEXPECTED_CANDIDATE_COUNT,
            detail=f'invalid solvePnPGeneric return: {error}',
        )
    if (
        candidate_count != 2
        or len(rotation_vectors) != 2
        or len(translation_vectors) != 2
    ):
        return _invalid_result(
            PnpFailure.UNEXPECTED_CANDIDATE_COUNT,
            detail=(
                f'expected 2 candidates, got count={candidate_count}, '
                f'rvecs={len(rotation_vectors)}, '
                f'tvecs={len(translation_vectors)}'
            ),
        )

    candidates = tuple(
        evaluate_pnp_candidate(
            index=index,
            rotation_vector=rotation_vector,
            translation_vector=translation_vector,
            object_points=object_points,
            image_points=image_points,
            camera_matrix=matrix,
            distortion_coefficients=distortion,
            camera_frame=camera_frame,
            target_frame=target_frame,
        )
        for index, (rotation_vector, translation_vector) in enumerate(
            zip(rotation_vectors, translation_vectors)
        )
    )
    return select_pnp_candidates(candidates)
