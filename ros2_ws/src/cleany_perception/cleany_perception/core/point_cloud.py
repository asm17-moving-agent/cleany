from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from cleany_perception.core.models import CameraIntrinsics


@dataclass(frozen=True)
class ColoredPointCloud:
    points: NDArray[np.float32]
    colors: NDArray[np.uint8]

    def __post_init__(self) -> None:
        points = np.asarray(self.points, dtype=np.float32)
        colors = np.asarray(self.colors, dtype=np.uint8)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('Cloud points must have shape Nx3')
        if colors.shape != points.shape:
            raise ValueError('Cloud colors must have shape Nx3')
        if not np.isfinite(points).all():
            raise ValueError('Cloud points must be finite')
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'colors', colors)


def colored_cloud_from_selection(
    depth_m: NDArray[np.float32],
    rgb: NDArray[np.uint8],
    intrinsics: CameraIntrinsics,
    selection: NDArray[np.bool_],
    minimum_depth_m: float,
    maximum_depth_m: float,
    voxel_size_m: float,
    maximum_points: int,
) -> ColoredPointCloud:
    if depth_m.shape != selection.shape or rgb.shape[:2] != depth_m.shape:
        raise ValueError('RGB, depth, and selection dimensions must match')
    if voxel_size_m <= 0.0 or maximum_points <= 0:
        raise ValueError('Cloud limits must be positive')
    valid = np.asarray(selection, dtype=np.bool_).copy()
    valid &= np.isfinite(depth_m)
    valid &= depth_m >= minimum_depth_m
    valid &= depth_m <= maximum_depth_m
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return ColoredPointCloud(
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.uint8),
        )
    depth = depth_m[rows, columns].astype(np.float32)
    points = np.column_stack(
        (
            (columns.astype(np.float32) - intrinsics.cx)
            * depth
            / intrinsics.fx,
            (rows.astype(np.float32) - intrinsics.cy)
            * depth
            / intrinsics.fy,
            depth,
        )
    ).astype(np.float32)
    colors = rgb[rows, columns].astype(np.uint8, copy=True)

    voxel_keys = np.floor(points / voxel_size_m).astype(np.int64)
    _, indices = np.unique(voxel_keys, axis=0, return_index=True)
    indices.sort()
    if indices.size > maximum_points:
        sample = np.linspace(0, indices.size - 1, maximum_points, dtype=int)
        indices = indices[sample]
    return ColoredPointCloud(points[indices], colors[indices])
