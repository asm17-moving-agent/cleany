"""Quaternion, TCP-axis, and trajectory sampling helpers."""

from __future__ import annotations

import math

import numpy as np


TCP_APPROACH_AXIS = np.array((0.0, -1.0, 0.0), dtype=float)
TCP_CLOSING_AXIS = np.array((1.0, 0.0, 0.0), dtype=float)


def rotation_from_quaternion(
    quaternion_xyzw: tuple[float, float, float, float],
) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('quaternion must be finite and non-zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def tcp_axes(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    rotation = rotation_from_quaternion(quaternion_xyzw)
    return rotation @ TCP_APPROACH_AXIS, rotation @ TCP_CLOSING_AXIS


def quaternion_from_rotation(matrix: np.ndarray) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        permutations = ((0, 1, 2), (1, 2, 0), (2, 0, 1))
        i, j, k = permutations[index]
        scale = math.sqrt(
            1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k]
        ) * 2.0
        quaternion = [0.0, 0.0, 0.0, (rotation[k, j] - rotation[j, k]) / scale]
        quaternion[i] = 0.25 * scale
        quaternion[j] = (rotation[i, j] + rotation[j, i]) / scale
        quaternion[k] = (rotation[i, k] + rotation[k, i]) / scale
        values = tuple(quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(float(value / norm) for value in values)


def tcp_quaternion_for_approach(
    approach: tuple[float, float, float],
    preferred_closing: tuple[float, float, float] = (1.0, 0.0, 0.0),
) -> tuple[float, float, float, float]:
    """Choose a right-handed TCP pose whose local -Y follows approach."""
    approach_axis = np.asarray(approach, dtype=float)
    approach_axis /= np.linalg.norm(approach_axis)
    closing = np.asarray(preferred_closing, dtype=float)
    closing -= float(np.dot(closing, approach_axis)) * approach_axis
    if np.linalg.norm(closing) <= 1e-9:
        closing = np.array((0.0, 1.0, 0.0))
        closing -= float(np.dot(closing, approach_axis)) * approach_axis
    closing /= np.linalg.norm(closing)
    local_y = -approach_axis
    local_z = np.cross(closing, local_y)
    rotation = np.column_stack((closing, local_y, local_z))
    return quaternion_from_rotation(rotation)


def vector_error_degrees(left: np.ndarray, right: np.ndarray) -> float:
    a = np.asarray(left, dtype=float)
    b = np.asarray(right, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def axis_errors_degrees(
    requested_xyzw: tuple[float, float, float, float],
    actual_xyzw: tuple[float, float, float, float],
) -> tuple[float, float]:
    requested_approach, requested_closing = tcp_axes(requested_xyzw)
    actual_approach, actual_closing = tcp_axes(actual_xyzw)
    return (
        vector_error_degrees(requested_approach, actual_approach),
        vector_error_degrees(requested_closing, actual_closing),
    )


def sampled_indices(length: int, maximum: int) -> tuple[int, ...]:
    if length <= 0 or maximum <= 0:
        return ()
    if length <= maximum:
        return tuple(range(length))
    if maximum == 1:
        return (length - 1,)
    return tuple(
        sorted(
            {
                int(round(index * (length - 1) / (maximum - 1)))
                for index in range(maximum)
            }
        )
    )


def quaternion_axis(
    quaternion: tuple[float, float, float, float],
    local_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a local unit axis by an xyzw quaternion."""
    norm = math.sqrt(sum(v*v for v in quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = (v/norm for v in quaternion)
    vx, vy, vz = local_axis
    # Expanded q * v * q^-1.
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )
