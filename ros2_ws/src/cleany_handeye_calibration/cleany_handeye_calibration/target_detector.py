"""ROS-independent ChArUco target detection and correspondence checks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

import cv2
import numpy as np


SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_M = 0.030
MARKER_LENGTH_M = 0.015
DICTIONARY_NAME = 'DICT_5X5_100'
MINIMUM_CORNER_COUNT = 16
INNER_CORNER_COUNT = (SQUARES_X - 1) * (SQUARES_Y - 1)
QUADRANTS = (
    'top_left',
    'top_right',
    'bottom_left',
    'bottom_right',
)


class DetectionFailure(str, Enum):
    INVALID_IMAGE = 'invalid_image'
    MARKERS_NOT_DETECTED = 'markers_not_detected'
    CHARUCO_INTERPOLATION_FAILED = 'charuco_interpolation_failed'
    INVALID_CORNER_IDS = 'invalid_corner_ids'
    INVALID_IMAGE_POINTS = 'invalid_image_points'
    CORRESPONDENCE_COUNT_MISMATCH = 'correspondence_count_mismatch'
    DUPLICATE_CORNER_IDS = 'duplicate_corner_ids'
    CORNER_ID_OUT_OF_RANGE = 'corner_id_out_of_range'
    INSUFFICIENT_CORNERS = 'insufficient_corners'
    INCOMPLETE_BOARD_COVERAGE = 'incomplete_board_coverage'


@dataclass(frozen=True, slots=True)
class CharucoTargetSpec:
    """The fixed Cleany calibration target contract."""

    squares_x: int = SQUARES_X
    squares_y: int = SQUARES_Y
    square_length_m: float = SQUARE_LENGTH_M
    marker_length_m: float = MARKER_LENGTH_M
    dictionary_name: str = DICTIONARY_NAME
    legacy_pattern: bool = False
    minimum_corner_count: int = MINIMUM_CORNER_COUNT

    def __post_init__(self) -> None:
        if self.squares_x != SQUARES_X or self.squares_y != SQUARES_Y:
            raise ValueError('ChArUco target must contain 7 x 5 squares')
        if not math.isclose(
            self.square_length_m,
            SQUARE_LENGTH_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError('ChArUco square length must be 0.030 m')
        if not math.isclose(
            self.marker_length_m,
            MARKER_LENGTH_M,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError('ChArUco marker length must be 0.015 m')
        if self.dictionary_name != DICTIONARY_NAME:
            raise ValueError('ChArUco dictionary must be DICT_5X5_100')
        if self.legacy_pattern is not False:
            raise ValueError('ChArUco legacy_pattern must be false')
        if self.minimum_corner_count != MINIMUM_CORNER_COUNT:
            raise ValueError('minimum ChArUco corner count must be 16')

    @property
    def inner_corner_count(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)


@dataclass(frozen=True, slots=True)
class CharucoDetection:
    """Detected correspondences and their deterministic validation result."""

    valid: bool
    failure_reason: DetectionFailure | None
    corner_ids: tuple[int, ...]
    image_points_px: tuple[tuple[float, float], ...]
    object_points_m: tuple[tuple[float, float, float], ...]
    covered_quadrants: tuple[str, ...]

    def image_points_array(self) -> np.ndarray:
        return np.asarray(self.image_points_px, dtype=np.float64)

    def object_points_array(self) -> np.ndarray:
        return np.asarray(self.object_points_m, dtype=np.float64)


def create_charuco_board(
    spec: CharucoTargetSpec | None = None,
):
    """Create the board through the OpenCV 4.5.4 Python API."""

    target = spec or CharucoTargetSpec()
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('installed OpenCV does not provide cv2.aruco')
    aruco = cv2.aruco
    required_symbols = (
        'CharucoBoard_create',
        'DICT_5X5_100',
        'getPredefinedDictionary',
    )
    missing = [name for name in required_symbols if not hasattr(aruco, name)]
    if missing:
        raise RuntimeError(
            'installed OpenCV lacks Humble ChArUco API: '
            + ', '.join(missing)
        )
    dictionary_id = getattr(aruco, target.dictionary_name)
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    board = aruco.CharucoBoard_create(
        target.squares_x,
        target.squares_y,
        target.square_length_m,
        target.marker_length_m,
        dictionary,
    )
    corners = np.asarray(board.chessboardCorners)
    if corners.shape != (target.inner_corner_count, 3):
        raise RuntimeError(
            'OpenCV ChArUco board produced an unexpected corner layout'
        )
    return board


def charuco_object_points_m(
    spec: CharucoTargetSpec | None = None,
) -> np.ndarray:
    """Return object points indexed by ChArUco corner ID."""

    board = create_charuco_board(spec)
    return np.asarray(board.chessboardCorners, dtype=np.float64).copy()


def _invalid_detection(
    reason: DetectionFailure,
    *,
    corner_ids: tuple[int, ...] = (),
    image_points_px: tuple[tuple[float, float], ...] = (),
    object_points_m: tuple[tuple[float, float, float], ...] = (),
    covered_quadrants: tuple[str, ...] = (),
) -> CharucoDetection:
    return CharucoDetection(
        valid=False,
        failure_reason=reason,
        corner_ids=corner_ids,
        image_points_px=image_points_px,
        object_points_m=object_points_m,
        covered_quadrants=covered_quadrants,
    )


def _normalized_corner_ids(
    corner_ids: Sequence[int] | np.ndarray,
) -> np.ndarray | None:
    ids = np.asarray(corner_ids)
    if ids.ndim == 2 and ids.shape[1:] == (1,):
        ids = ids.reshape(-1)
    if ids.ndim != 1 or not np.issubdtype(ids.dtype, np.integer):
        return None
    if np.issubdtype(ids.dtype, np.bool_):
        return None
    return ids.astype(np.int64, copy=False)


def _normalized_image_points(
    image_points_px: Sequence[Sequence[float]] | np.ndarray,
) -> np.ndarray | None:
    try:
        points = np.asarray(image_points_px, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if points.ndim == 3 and points.shape[1:] == (1, 2):
        points = points.reshape(-1, 2)
    if points.ndim != 2 or points.shape[1:] != (2,):
        return None
    if not np.all(np.isfinite(points)):
        return None
    return points


def _covered_quadrants(
    object_points: np.ndarray,
    spec: CharucoTargetSpec,
) -> tuple[str, ...]:
    center_x = spec.squares_x * spec.square_length_m / 2.0
    center_y = spec.squares_y * spec.square_length_m / 2.0
    covered: set[str] = set()
    for x, y, _ in object_points:
        vertical = 'top' if y < center_y else 'bottom'
        horizontal = 'left' if x < center_x else 'right'
        covered.add(f'{vertical}_{horizontal}')
    return tuple(name for name in QUADRANTS if name in covered)


def analyze_charuco_corners(
    corner_ids: Sequence[int] | np.ndarray,
    image_points_px: Sequence[Sequence[float]] | np.ndarray,
    *,
    spec: CharucoTargetSpec | None = None,
) -> CharucoDetection:
    """Validate IDs, ordering, minimum count, and board-region coverage."""

    target = spec or CharucoTargetSpec()
    ids = _normalized_corner_ids(corner_ids)
    if ids is None:
        return _invalid_detection(DetectionFailure.INVALID_CORNER_IDS)
    image_points = _normalized_image_points(image_points_px)
    if image_points is None:
        return _invalid_detection(DetectionFailure.INVALID_IMAGE_POINTS)
    if ids.size != image_points.shape[0]:
        return _invalid_detection(
            DetectionFailure.CORRESPONDENCE_COUNT_MISMATCH
        )
    if np.unique(ids).size != ids.size:
        return _invalid_detection(DetectionFailure.DUPLICATE_CORNER_IDS)
    if np.any(ids < 0) or np.any(ids >= target.inner_corner_count):
        return _invalid_detection(DetectionFailure.CORNER_ID_OUT_OF_RANGE)

    order = np.argsort(ids)
    ids = ids[order]
    image_points = image_points[order]
    all_object_points = charuco_object_points_m(target)
    object_points = all_object_points[ids]
    covered_quadrants = _covered_quadrants(object_points, target)

    id_values = tuple(int(value) for value in ids)
    image_values = tuple(
        (float(point[0]), float(point[1])) for point in image_points
    )
    object_values = tuple(
        (float(point[0]), float(point[1]), float(point[2]))
        for point in object_points
    )
    if ids.size < target.minimum_corner_count:
        return _invalid_detection(
            DetectionFailure.INSUFFICIENT_CORNERS,
            corner_ids=id_values,
            image_points_px=image_values,
            object_points_m=object_values,
            covered_quadrants=covered_quadrants,
        )
    if covered_quadrants != QUADRANTS:
        return _invalid_detection(
            DetectionFailure.INCOMPLETE_BOARD_COVERAGE,
            corner_ids=id_values,
            image_points_px=image_values,
            object_points_m=object_values,
            covered_quadrants=covered_quadrants,
        )
    return CharucoDetection(
        valid=True,
        failure_reason=None,
        corner_ids=id_values,
        image_points_px=image_values,
        object_points_m=object_values,
        covered_quadrants=covered_quadrants,
    )


class CharucoTargetDetector:
    """OpenCV 4.5.4 detector that exposes no ROS message types."""

    def __init__(self, spec: CharucoTargetSpec | None = None) -> None:
        self.spec = spec or CharucoTargetSpec()
        self._board = create_charuco_board(self.spec)
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.spec.dictionary_name)
        )
        if not hasattr(cv2.aruco, 'DetectorParameters_create'):
            raise RuntimeError(
                'installed OpenCV lacks DetectorParameters_create'
            )
        self._detector_parameters = cv2.aruco.DetectorParameters_create()

    @property
    def board(self):
        return self._board

    def detect(self, image: np.ndarray) -> CharucoDetection:
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            return _invalid_detection(DetectionFailure.INVALID_IMAGE)
        if image.ndim == 2:
            grayscale = image
        elif image.ndim == 3 and image.shape[2] == 3:
            grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim == 3 and image.shape[2] == 4:
            grayscale = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        else:
            return _invalid_detection(DetectionFailure.INVALID_IMAGE)
        grayscale = np.ascontiguousarray(grayscale)

        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            grayscale,
            self._dictionary,
            parameters=self._detector_parameters,
        )
        if marker_ids is None or len(marker_ids) == 0:
            return _invalid_detection(
                DetectionFailure.MARKERS_NOT_DETECTED
            )
        count, charuco_corners, charuco_ids = (
            cv2.aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                grayscale,
                self._board,
            )
        )
        if (
            count is None
            or int(count) == 0
            or charuco_corners is None
            or charuco_ids is None
        ):
            return _invalid_detection(
                DetectionFailure.CHARUCO_INTERPOLATION_FAILED
            )
        return analyze_charuco_corners(
            charuco_ids,
            charuco_corners,
            spec=self.spec,
        )
