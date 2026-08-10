import cv2
import numpy as np
import pytest

from cleany_handeye_calibration.target_detector import (
    INNER_CORNER_COUNT,
    QUADRANTS,
    CharucoTargetDetector,
    CharucoTargetSpec,
    DetectionFailure,
    analyze_charuco_corners,
    charuco_object_points_m,
)


def _image_points(count: int) -> np.ndarray:
    values = np.arange(count * 2, dtype=np.float64)
    return values.reshape(count, 2)


def test_humble_charuco_detector_recovers_all_inner_corners():
    detector = CharucoTargetDetector()
    image = detector.board.draw(
        (700, 500),
        marginSize=20,
        borderBits=1,
    )

    result = detector.detect(image)

    assert hasattr(cv2.aruco, 'CharucoBoard_create')
    assert hasattr(cv2.aruco, 'DetectorParameters_create')
    assert hasattr(cv2.aruco, 'detectMarkers')
    assert hasattr(cv2.aruco, 'interpolateCornersCharuco')
    assert hasattr(detector.board, 'chessboardCorners')
    assert result.valid
    assert result.failure_reason is None
    assert result.corner_ids == tuple(range(INNER_CORNER_COUNT))
    assert len(result.object_points_m) == 24
    assert result.covered_quadrants == QUADRANTS


def test_charuco_object_points_match_7_by_5_board_geometry():
    points = charuco_object_points_m()

    assert points.shape == (24, 3)
    np.testing.assert_allclose(points[0], (0.03, 0.03, 0.0), atol=1.0e-8)
    np.testing.assert_allclose(points[-1], (0.18, 0.12, 0.0), atol=1.0e-8)
    assert np.unique(points[:, 0]).size == 6
    assert np.unique(points[:, 1]).size == 4


@pytest.mark.parametrize(
    'kwargs',
    [
        {'squares_x': 6},
        {'squares_y': 4},
        {'square_length_m': 0.031},
        {'marker_length_m': 0.014},
        {'dictionary_name': 'DICT_4X4_50'},
        {'legacy_pattern': True},
        {'minimum_corner_count': 15},
    ],
)
def test_target_spec_rejects_contract_mismatch(kwargs):
    with pytest.raises(ValueError):
        CharucoTargetSpec(**kwargs)


def test_detection_rejects_fewer_than_16_corners():
    result = analyze_charuco_corners(
        np.arange(15, dtype=np.int32),
        _image_points(15),
    )

    assert not result.valid
    assert result.failure_reason is DetectionFailure.INSUFFICIENT_CORNERS


def test_detection_requires_all_four_board_quadrants():
    bottom_right_ids = {15, 16, 17, 21, 22, 23}
    ids = np.array(
        [value for value in range(24) if value not in bottom_right_ids],
        dtype=np.int32,
    )

    result = analyze_charuco_corners(ids, _image_points(ids.size))

    assert ids.size == 18
    assert not result.valid
    assert (
        result.failure_reason
        is DetectionFailure.INCOMPLETE_BOARD_COVERAGE
    )
    assert 'bottom_right' not in result.covered_quadrants


def test_detection_sorts_ids_and_preserves_correspondence_order():
    ids = np.arange(24, dtype=np.int32)[::-1]
    image_points = np.column_stack((ids, ids + 0.5))

    result = analyze_charuco_corners(ids, image_points)

    assert result.valid
    assert result.corner_ids == tuple(range(24))
    assert result.image_points_px[3] == (3.0, 3.5)


@pytest.mark.parametrize(
    ('ids', 'reason'),
    [
        (
            np.array([*range(23), 22], dtype=np.int32),
            DetectionFailure.DUPLICATE_CORNER_IDS,
        ),
        (
            np.array([*range(23), 24], dtype=np.int32),
            DetectionFailure.CORNER_ID_OUT_OF_RANGE,
        ),
        (
            np.arange(24, dtype=np.float64),
            DetectionFailure.INVALID_CORNER_IDS,
        ),
    ],
)
def test_detection_rejects_invalid_corner_ids(ids, reason):
    result = analyze_charuco_corners(ids, _image_points(24))

    assert not result.valid
    assert result.failure_reason is reason


def test_detector_reports_no_markers_without_throwing():
    image = np.full((480, 640), 255, dtype=np.uint8)

    result = CharucoTargetDetector().detect(image)

    assert not result.valid
    assert result.failure_reason is DetectionFailure.MARKERS_NOT_DETECTED
