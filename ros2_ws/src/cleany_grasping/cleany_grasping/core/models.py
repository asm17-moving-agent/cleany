from __future__ import annotations

from dataclasses import dataclass, field

import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class PointCloud:
    points: NDArray[np.float64]
    colors: NDArray[np.float64]
    viewpoint: NDArray[np.float64] | None = field(default=None, kw_only=True)

    def __post_init__(self) -> None:
        points = np.asarray(self.points, dtype=np.float64)
        colors = np.asarray(self.colors, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
            raise ValueError('Points and colors must have shape Nx3')
        if not np.isfinite(points).all() or not np.isfinite(colors).all():
            raise ValueError('Point cloud must be finite')
        if self.viewpoint is not None:
            origin = np.asarray(self.viewpoint, dtype=float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError('Cloud viewpoint must be a finite 3-vector')
            object.__setattr__(self, 'viewpoint', origin)
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'colors', colors)


@dataclass(frozen=True)
class RawGrasp:
    rotation: NDArray[np.float64]
    translation: NDArray[np.float64]
    width_m: float
    depth_m: float
    score: float
    pad_offset_m: float = 0.0
    face_width_m: float | None = None

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation, dtype=np.float64)
        translation = np.asarray(self.translation, dtype=np.float64)
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError('Grasp pose dimensions are invalid')
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError('Grasp pose must be finite')
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError('Grasp rotation must be orthonormal')
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
            raise ValueError('Grasp rotation must be right-handed')
        if not all(
            math.isfinite(value)
            for value in (self.width_m, self.depth_m, self.score)
        ):
            raise ValueError('Grasp dimensions and score must be finite')
        if not math.isfinite(self.pad_offset_m) or self.pad_offset_m < 0:
            raise ValueError('Pad contact offset must be finite and non-negative')
        if self.width_m <= 0.0 or self.depth_m < 0.0:
            raise ValueError('Grasp width must be positive and depth non-negative')
        object.__setattr__(self, 'rotation', rotation)
        object.__setattr__(self, 'translation', translation)


@dataclass(frozen=True)
class GraspPose:
    rotation: NDArray[np.float64]
    translation: NDArray[np.float64]
    approach_direction: NDArray[np.float64]
    required_opening_m: float
    depth_m: float
    score: float
    face_width_m: float | None = field(default=None, kw_only=True)
