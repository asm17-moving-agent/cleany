from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]
MaskArray = NDArray[np.bool_]
RgbArray = NDArray[np.uint8]
DepthArray = NDArray[np.float32]


class FailureKind(str, Enum):
    RGBD_TIMEOUT = 'rgbd_timeout'
    DETECTOR_API = 'detector_api'
    DETECTOR_RESPONSE = 'detector_response'
    MASK = 'mask'
    DEPTH = 'depth'
    PLANE = 'plane'
    TF = 'tf'
    CANCELLED = 'cancelled'
    INTERNAL = 'internal'


class InspectionStage(str, Enum):
    WAITING_FOR_RGBD = 'waiting_for_rgbd'
    DETECTING = 'detecting'
    SEGMENTING = 'segmenting'
    RECONSTRUCTING = 'reconstructing'
    TRANSFORMING = 'transforming'


class InspectionFailure(RuntimeError):
    def __init__(self, kind: FailureKind, message: str) -> None:
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class BoundingBox2D:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        values = (self.x_min, self.y_min, self.x_max, self.y_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Bounding-box coordinates must be finite')
        if self.x_min < 0.0 or self.y_min < 0.0:
            raise ValueError(
                'Bounding-box minimum coordinates must be non-negative'
            )
        if self.x_max <= self.x_min or self.y_max <= self.y_min:
            raise ValueError('Bounding box must have positive area')


@dataclass(frozen=True)
class Detection2D:
    label: str
    confidence: float
    bbox: BoundingBox2D

    def __post_init__(self) -> None:
        if not self.label.strip():
            raise ValueError('Detection label must not be empty')
        if (
            not math.isfinite(self.confidence)
            or not 0.0 <= self.confidence <= 1.0
        ):
            raise ValueError('Detection confidence must be in [0, 1]')


@dataclass(frozen=True)
class ObjectMask:
    detection: Detection2D
    mask: MaskArray
    score: float

    def __post_init__(self) -> None:
        mask = np.asarray(self.mask, dtype=np.bool_)
        if mask.ndim != 2:
            raise ValueError('Object mask must be two-dimensional')
        if not math.isfinite(self.score) or not 0.0 <= self.score <= 1.0:
            raise ValueError('Mask score must be in [0, 1]')
        object.__setattr__(self, 'mask', mask)


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Camera dimensions must be positive')
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Camera intrinsics must be finite')
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError('Camera focal lengths must be positive')


@dataclass(frozen=True)
class RgbdSnapshot:
    rgb: RgbArray
    depth_m: DepthArray
    intrinsics: CameraIntrinsics
    stamp_ns: int
    source_frame: str

    def __post_init__(self) -> None:
        rgb = np.asarray(self.rgb)
        depth = np.asarray(self.depth_m, dtype=np.float32)
        expected_rgb = (
            self.intrinsics.height,
            self.intrinsics.width,
            3,
        )
        expected_depth = (
            self.intrinsics.height,
            self.intrinsics.width,
        )
        if rgb.shape != expected_rgb or rgb.dtype != np.uint8:
            raise ValueError('RGB image does not match camera intrinsics')
        if depth.shape != expected_depth:
            raise ValueError('Depth image does not match camera intrinsics')
        if self.stamp_ns < 0:
            raise ValueError('Snapshot timestamp must not be negative')
        if not self.source_frame:
            raise ValueError('Snapshot source frame must not be empty')
        object.__setattr__(self, 'rgb', rgb)
        object.__setattr__(self, 'depth_m', depth)


@dataclass(frozen=True)
class Plane:
    normal: FloatArray
    offset: float
    inlier_count: int

    def __post_init__(self) -> None:
        normal = np.asarray(self.normal, dtype=np.float64)
        if normal.shape != (3,) or not np.isfinite(normal).all():
            raise ValueError('Plane normal must be a finite 3-vector')
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-12:
            raise ValueError('Plane normal must not be zero')
        if not math.isfinite(self.offset):
            raise ValueError('Plane offset must be finite')
        if self.inlier_count < 0:
            raise ValueError('Plane inlier count must not be negative')
        object.__setattr__(self, 'normal', normal / norm)
        object.__setattr__(self, 'offset', float(self.offset) / norm)


@dataclass(frozen=True)
class RigidTransform:
    translation: FloatArray
    rotation: FloatArray

    def __post_init__(self) -> None:
        translation = np.asarray(self.translation, dtype=np.float64)
        rotation = np.asarray(self.rotation, dtype=np.float64)
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError('Transform translation must be a finite 3-vector')
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError('Transform rotation must be a finite 3x3 matrix')
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError('Transform rotation must be orthonormal')
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
            raise ValueError('Transform rotation must be right-handed')
        object.__setattr__(self, 'translation', translation)
        object.__setattr__(self, 'rotation', rotation)


@dataclass(frozen=True)
class OrientedBox3D:
    center: FloatArray
    rotation: FloatArray
    size: FloatArray

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=np.float64)
        rotation = np.asarray(self.rotation, dtype=np.float64)
        size = np.asarray(self.size, dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError('OBB center must be a finite 3-vector')
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            raise ValueError('OBB rotation must be a finite 3x3 matrix')
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError('OBB rotation must be orthonormal')
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
            raise ValueError('OBB rotation must be right-handed')
        if size.shape != (3,) or not np.isfinite(size).all():
            raise ValueError('OBB size must be a finite 3-vector')
        if np.any(size <= 0.0):
            raise ValueError('OBB size must be positive')
        object.__setattr__(self, 'center', center)
        object.__setattr__(self, 'rotation', rotation)
        object.__setattr__(self, 'size', size)


@dataclass(frozen=True)
class InspectedObject:
    label: str
    confidence: float
    box: OrientedBox3D


@dataclass(frozen=True)
class InspectionOutput:
    objects: tuple[InspectedObject, ...]
    detections: tuple[Detection2D, ...]
    masks: tuple[ObjectMask, ...]
    target_frame: str
    plane: Plane | None


@dataclass(frozen=True)
class PipelineConfig:
    minimum_detection_confidence: float = 0.25
    maximum_detections: int = 10
    minimum_depth_m: float = 0.1
    maximum_depth_m: float = 3.0
    support_margin_pixels: int = 40
    support_sample_stride: int = 4
    plane_ransac_iterations: int = 200
    plane_distance_threshold_m: float = 0.006
    plane_minimum_inliers: int = 100
    plane_minimum_inlier_ratio: float = 0.35
    maximum_plane_tilt_degrees: float = 20.0
    minimum_object_points: int = 30
    minimum_object_height_m: float = 0.005
    minimum_obb_extent_m: float = 0.005

    def __post_init__(self) -> None:
        if not 0.0 <= self.minimum_detection_confidence <= 1.0:
            raise ValueError('Detection threshold must be in [0, 1]')
        if self.maximum_detections <= 0:
            raise ValueError('Maximum detections must be positive')
        if not 0.0 < self.minimum_depth_m < self.maximum_depth_m:
            raise ValueError('Depth range is invalid')
        if self.support_margin_pixels < 0 or self.support_sample_stride <= 0:
            raise ValueError('Support sampling parameters are invalid')
        if self.plane_ransac_iterations <= 0:
            raise ValueError('Plane RANSAC iterations must be positive')
        if self.plane_distance_threshold_m <= 0.0:
            raise ValueError('Plane distance threshold must be positive')
        if self.plane_minimum_inliers < 3:
            raise ValueError('Plane fitting needs at least three inliers')
        if not 0.0 < self.plane_minimum_inlier_ratio <= 1.0:
            raise ValueError('Plane inlier ratio must be in (0, 1]')
        if not 0.0 <= self.maximum_plane_tilt_degrees < 90.0:
            raise ValueError('Maximum plane tilt must be in [0, 90)')
        if self.minimum_object_points < 3:
            raise ValueError(
                'Object reconstruction needs at least three points'
            )
        if self.minimum_object_height_m <= 0.0:
            raise ValueError('Minimum object height must be positive')
        if self.minimum_obb_extent_m <= 0.0:
            raise ValueError('Minimum OBB extent must be positive')
