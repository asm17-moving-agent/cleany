from __future__ import annotations

import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from cleany_perception.core.models import (
    FailureKind,
    InspectionFailure,
    RigidTransform,
)


def rotation_matrix_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    quaternion = np.array((x, y, z, w), dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError('TF quaternion must not be zero')
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


class Tf2TransformAdapter:
    def __init__(
        self,
        node: Node,
        timeout_seconds: float = 0.5,
        cache_seconds: float = 60.0,
        buffer: Buffer | None = None,
    ) -> None:
        if timeout_seconds < 0.0:
            raise ValueError('TF timeout must not be negative')
        if cache_seconds <= 0.0:
            raise ValueError('TF cache duration must be positive')
        self._timeout = Duration(seconds=timeout_seconds)
        self._buffer = buffer or Buffer(
            cache_time=Duration(seconds=cache_seconds)
        )
        self._listener = None
        if buffer is None:
            self._listener = TransformListener(
                self._buffer,
                node,
                spin_thread=False,
            )

    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        stamp_ns: int,
    ) -> RigidTransform:
        try:
            transform = self._buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(nanoseconds=stamp_ns),
                timeout=self._timeout,
            ).transform
            rotation = rotation_matrix_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            return RigidTransform(
                translation=np.array(
                    (
                        transform.translation.x,
                        transform.translation.y,
                        transform.translation.z,
                    ),
                    dtype=np.float64,
                ),
                rotation=rotation,
            )
        except (TransformException, ValueError) as error:
            raise InspectionFailure(
                FailureKind.TF,
                f'TF lookup failed: {error}',
            ) from error
