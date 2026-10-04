"""Rectified depth projection and explicit simulation no-return policy."""

from __future__ import annotations

import math

import numpy as np

from cleany_navigation.body_geometry import BodyBox


def self_observation_mask(points: np.ndarray, sensor_origin: np.ndarray,
                          boxes: list[BodyBox], margin: float) -> np.ndarray:
    """Keep rays whose finite segment does not intersect fixed self geometry.

    A mixed edge pixel can report a hit behind a rail. Checking only the hit
    position misses this self occlusion. Discard the ray instead of certifying
    any space behind the robot as free.
    """
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('Self-filter margin must be finite and nonnegative')
    keep = np.isfinite(points).all(axis=1)
    rays = points-sensor_origin
    for box in boxes:
        origin = (sensor_origin-box.center)@box.rotation
        direction = rays@box.rotation
        half = box.half_size+margin
        # Accumulate the same closed-segment slab intersection by axis. This
        # avoids allocating/reducing several N x 3 temporary arrays per box;
        # parallel rays and boundary contact keep their conservative semantics.
        outside = np.zeros(len(points), dtype=bool)
        enter = np.zeros(len(points), dtype=direction.dtype)
        leave = np.ones(len(points), dtype=direction.dtype)
        for axis in range(3):
            component = direction[:, axis]
            parallel = np.abs(component) < 1e-12
            outside |= parallel & (abs(origin[axis]) > half[axis])
            low = np.full(len(points), -np.inf, dtype=direction.dtype)
            high = np.full(len(points), np.inf, dtype=direction.dtype)
            np.divide(-half[axis]-origin[axis], component, out=low, where=~parallel)
            np.divide(half[axis]-origin[axis], component, out=high, where=~parallel)
            np.maximum(enter, np.minimum(low, high), out=enter)
            np.minimum(leave, np.maximum(low, high), out=leave)
        keep &= outside | (enter > leave)
    return keep


def decode_depth_image(data: bytes, width: int, height: int, step: int,
                       encoding: str, is_bigendian: bool = False) -> np.ndarray:
    """Decode REP-118 depth, including padded rows, to meters."""
    formats = {'32FC1': ('f4', 4, 1.), '16UC1': ('u2', 2, .001)}
    if encoding not in formats:
        raise ValueError('Expected 32FC1 or 16UC1 depth')
    dtype, size, scale = formats[encoding]
    if min(width, height) <= 0 or step < width * size or len(data) < step * height:
        raise ValueError('Truncated depth image')
    array = np.ndarray((height, width), dtype=('>' if is_bigendian else '<')+dtype,
                       buffer=data, strides=(step, size))
    return np.asarray(array, dtype=np.float32) * scale


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


def obstacle_height_mask(body_points: np.ndarray, min_z: float, max_z: float) -> np.ndarray:
    """Select obstacle hits in base_link while retaining original clearing rays."""
    if not math.isfinite(min_z) or not math.isfinite(max_z) or min_z >= max_z:
        raise ValueError('Require finite ordered obstacle height bounds')
    return np.isfinite(body_points).all(axis=1) & (body_points[:, 2] >= min_z) & (body_points[:, 2] <= max_z)
