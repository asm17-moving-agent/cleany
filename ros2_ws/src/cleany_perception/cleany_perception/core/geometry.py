from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from cleany_perception.core.models import (
    CameraIntrinsics,
    OrientedBox3D,
    Plane,
    RigidTransform,
)


def deproject_masked_depth(
    depth_m: NDArray[np.float32],
    intrinsics: CameraIntrinsics,
    mask: NDArray[np.bool_],
    minimum_depth_m: float,
    maximum_depth_m: float,
    stride: int = 1,
) -> NDArray[np.float64]:
    if depth_m.shape != mask.shape:
        raise ValueError('Depth and selection mask shapes do not match')
    if stride <= 0:
        raise ValueError('Depth sampling stride must be positive')
    selected = np.asarray(mask, dtype=np.bool_).copy()
    selected &= np.isfinite(depth_m)
    selected &= depth_m >= minimum_depth_m
    selected &= depth_m <= maximum_depth_m
    if stride > 1:
        sampling = np.zeros_like(selected)
        sampling[::stride, ::stride] = True
        selected &= sampling
    rows, columns = np.nonzero(selected)
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    depth = depth_m[rows, columns].astype(np.float64)
    x = (columns.astype(np.float64) - intrinsics.cx) * depth / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * depth / intrinsics.fy
    return np.column_stack((x, y, depth))


def fit_plane_ransac(
    points: NDArray[np.float64],
    iterations: int,
    distance_threshold_m: float,
    minimum_inliers: int,
    minimum_inlier_ratio: float,
    random_seed: int = 0,
) -> Plane:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError('Plane point cloud must have shape Nx3')
    if cloud.shape[0] < max(3, minimum_inliers):
        raise ValueError('Not enough support points for plane fitting')
    if not np.isfinite(cloud).all():
        raise ValueError('Plane point cloud must be finite')

    generator = np.random.default_rng(random_seed)
    best_inliers: NDArray[np.bool_] | None = None
    best_count = 0
    for _ in range(iterations):
        sample = cloud[generator.choice(cloud.shape[0], 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-10:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        inliers = np.abs(cloud @ normal + offset) <= distance_threshold_m
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_inliers = inliers
            best_count = count

    required_count = max(
        minimum_inliers,
        math.ceil(minimum_inlier_ratio * cloud.shape[0]),
    )
    if best_inliers is None or best_count < required_count:
        raise ValueError(
            f'Support plane has {best_count} inliers; '
            f'{required_count} required'
        )

    inlier_points = cloud[best_inliers]
    centroid = np.mean(inlier_points, axis=0)
    _, _, right_vectors = np.linalg.svd(
        inlier_points - centroid,
        full_matrices=False,
    )
    normal = right_vectors[-1]
    normal /= np.linalg.norm(normal)
    offset = -float(normal @ centroid)

    # The support surface normal points toward the camera origin. Objects on
    # the table then have positive signed height above the plane.
    point_on_plane = -offset * normal
    if float(normal @ (-point_on_plane)) < 0.0:
        normal = -normal
        offset = -offset

    refined_distances = np.abs(cloud @ normal + offset)
    refined_count = int(
        np.count_nonzero(refined_distances <= distance_threshold_m)
    )
    if refined_count < required_count:
        raise ValueError(
            'Refined support plane does not retain enough inliers'
        )
    return Plane(normal=normal, offset=offset, inlier_count=refined_count)


def reconstruct_supported_obb(
    points: NDArray[np.float64],
    plane: Plane,
    minimum_height_m: float,
    minimum_extent_m: float,
    minimum_points: int,
) -> OrientedBox3D:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError('Object point cloud must have shape Nx3')
    if not np.isfinite(cloud).all():
        raise ValueError('Object point cloud must be finite')
    heights = cloud @ plane.normal + plane.offset
    above_plane = heights >= minimum_height_m
    object_points = cloud[above_plane]
    object_heights = heights[above_plane]
    if object_points.shape[0] < minimum_points:
        raise ValueError('Not enough object points above the support plane')

    projected = object_points - object_heights[:, None] * plane.normal
    plane_origin = -plane.offset * plane.normal
    centered = projected - np.mean(projected, axis=0)
    covariance = centered.T @ centered / max(1, centered.shape[0] - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    x_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    x_axis -= float(x_axis @ plane.normal) * plane.normal
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm <= 1e-10:
        raise ValueError('Object footprint does not define a stable axis')
    x_axis /= x_norm
    dominant_index = int(np.argmax(np.abs(x_axis)))
    if x_axis[dominant_index] < 0.0:
        x_axis = -x_axis
    y_axis = np.cross(plane.normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    relative = projected - plane_origin
    x_coordinates = relative @ x_axis
    y_coordinates = relative @ y_axis
    x_min, x_max = np.min(x_coordinates), np.max(x_coordinates)
    y_min, y_max = np.min(y_coordinates), np.max(y_coordinates)
    height = float(np.max(object_heights))
    size = np.array(
        (x_max - x_min, y_max - y_min, height),
        dtype=np.float64,
    )
    if np.any(size < minimum_extent_m):
        raise ValueError(
            'Reconstructed OBB is smaller than the minimum extent'
        )

    center_on_plane = (
        plane_origin
        + 0.5 * (x_min + x_max) * x_axis
        + 0.5 * (y_min + y_max) * y_axis
    )
    center = center_on_plane + 0.5 * height * plane.normal
    rotation = np.column_stack((x_axis, y_axis, plane.normal))
    return OrientedBox3D(center=center, rotation=rotation, size=size)


def transform_box(
    box: OrientedBox3D,
    transform: RigidTransform,
) -> OrientedBox3D:
    return OrientedBox3D(
        center=transform.rotation @ box.center + transform.translation,
        rotation=transform.rotation @ box.rotation,
        size=box.size.copy(),
    )


def transform_plane_normal(
    plane: Plane,
    transform: RigidTransform,
) -> NDArray[np.float64]:
    return transform.rotation @ plane.normal


def quaternion_xyzw_from_rotation(
    rotation: NDArray[np.float64],
) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError('Rotation matrix must have shape 3x3')
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return tuple(float(value) for value in quaternion)
