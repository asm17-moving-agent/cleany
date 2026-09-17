"""Gazebo depth-camera no-return rays, in camera X-forward coordinates."""

from __future__ import annotations

import math

import numpy as np


def project_depth_image(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    *,
    pixel_stride: int = 4,
    positive_infinity_is_free: bool = False,
    clearing_distance: float = 5.0,
) -> np.ndarray:
    """Project undistorted optical Z-depth to camera X-forward XYZ."""
    if depth.ndim != 2 or pixel_stride < 1:
        raise ValueError('Expected a 2-D depth image and positive pixel stride')
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(v) for v in intrinsics) or fx <= 0 or fy <= 0:
        raise ValueError('Camera intrinsics must be finite with positive focal lengths')
    rows = np.arange(0, depth.shape[0], pixel_stride)
    columns = np.arange(0, depth.shape[1], pixel_stride)
    yy, xx = np.meshgrid(rows, columns, indexing='ij')
    d = depth[::pixel_stride, ::pixel_stride]
    with np.errstate(invalid='ignore'):
        points = np.stack((d, (cx-xx)*d/fx, (cy-yy)*d/fy), axis=-1)
    return prepare_depth_points(points, rows, columns, intrinsics,
                                positive_infinity_is_free=positive_infinity_is_free,
                                clearing_distance=clearing_distance)


def prepare_depth_points(
    points: np.ndarray,
    rows: np.ndarray,
    columns: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    *,
    positive_infinity_is_free: bool = False,
    clearing_distance: float = 5.0,
) -> np.ndarray:
    """Keep finite hits; optionally give Gazebo +inf pixels finite ray ends.

    Gazebo Ogre2 encodes beyond-far as +inf and below-near as -inf. Only
    explicitly enabled +inf is free. NaN, -inf and nonpositive depth never
    become clearing observations. Endpoints must lie beyond marking range.
    """
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(v) for v in intrinsics) or fx <= 0 or fy <= 0:
        raise ValueError('Camera intrinsics must be finite with positive focal lengths')
    if not math.isfinite(clearing_distance) or clearing_distance <= 0:
        raise ValueError('clearing_distance must be finite and positive')
    xyz = np.array(points, dtype=np.float32, copy=True)
    if xyz.shape != (len(rows), len(columns), 3):
        raise ValueError('Point grid and pixel coordinates disagree')
    if positive_infinity_is_free:
        no_return = np.isposinf(xyz[:, :, 0])
        yy, xx = np.meshgrid(rows, columns, indexing='ij')
        endpoints = np.stack((np.full_like(xx, clearing_distance, dtype=float),
                              (cx-xx)*clearing_distance/fx, (cy-yy)*clearing_distance/fy), axis=-1)
        xyz[no_return] = endpoints[no_return]
    valid = np.isfinite(xyz).all(axis=2) & (xyz[:, :, 0] > 0)
    return np.ascontiguousarray(xyz[valid], dtype='<f4')
