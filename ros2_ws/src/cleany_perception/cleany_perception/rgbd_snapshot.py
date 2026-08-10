from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.core.models import (
    CameraIntrinsics,
    FailureKind,
    InspectionFailure,
    RgbdSnapshot,
)


@dataclass(frozen=True)
class SynchronizedRgbdMessages:
    sequence: int
    stamp_ns: int
    color: Image
    color_info: CameraInfo
    depth: Image
    depth_info: CameraInfo


def stamp_nanoseconds(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RgbdSnapshotBuffer:
    _KINDS = ('color', 'color_info', 'depth', 'depth_info')

    def __init__(self, maximum_pending_stamps: int = 8) -> None:
        if maximum_pending_stamps <= 0:
            raise ValueError('Maximum pending RGB-D stamps must be positive')
        self._maximum_pending_stamps = maximum_pending_stamps
        self._condition = threading.Condition()
        self._pending: dict[int, dict[str, Any]] = {}
        self._latest: SynchronizedRgbdMessages | None = None
        self._sequence = 0

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    def add_color(self, message: Image) -> None:
        self._add('color', message)

    def add_color_info(self, message: CameraInfo) -> None:
        self._add('color_info', message)

    def add_depth(self, message: Image) -> None:
        self._add('depth', message)

    def add_depth_info(self, message: CameraInfo) -> None:
        self._add('depth_info', message)

    def wait_for_new(
        self,
        after_sequence: int,
        timeout_seconds: float,
        cancelled: Callable[[], bool] | None = None,
    ) -> SynchronizedRgbdMessages:
        if timeout_seconds <= 0.0:
            raise ValueError('RGB-D timeout must be positive')
        is_cancelled = cancelled or (lambda: False)
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            while self._sequence <= after_sequence:
                if is_cancelled():
                    raise InspectionFailure(
                        FailureKind.CANCELLED,
                        'Inspection was cancelled while waiting for RGB-D',
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise InspectionFailure(
                        FailureKind.RGBD_TIMEOUT,
                        'Timed out waiting for an exact-timestamp '
                        'RGB-D snapshot',
                    )
                self._condition.wait(timeout=min(remaining, 0.05))
            if self._latest is None:
                raise RuntimeError(
                    'RGB-D sequence advanced without a snapshot'
                )
            return self._latest

    def _add(self, kind: str, message: Any) -> None:
        stamp_ns = stamp_nanoseconds(message)
        with self._condition:
            if self._latest is not None and stamp_ns <= self._latest.stamp_ns:
                return
            bundle = self._pending.setdefault(stamp_ns, {})
            bundle[kind] = message
            if all(name in bundle for name in self._KINDS):
                self._sequence += 1
                self._latest = SynchronizedRgbdMessages(
                    sequence=self._sequence,
                    stamp_ns=stamp_ns,
                    color=bundle['color'],
                    color_info=bundle['color_info'],
                    depth=bundle['depth'],
                    depth_info=bundle['depth_info'],
                )
                self._pending = {
                    key: value
                    for key, value in self._pending.items()
                    if key > stamp_ns
                }
                self._condition.notify_all()
            while len(self._pending) > self._maximum_pending_stamps:
                oldest_stamp = min(self._pending)
                del self._pending[oldest_stamp]


def snapshot_from_messages(
    messages: SynchronizedRgbdMessages,
    depth_16u_scale_m: float = 0.001,
) -> RgbdSnapshot:
    try:
        rgb = _rgb_array(messages.color)
        depth_m = _depth_array(messages.depth, depth_16u_scale_m)
        _validate_camera_info(messages.color_info, messages.color)
        _validate_camera_info(messages.depth_info, messages.depth)
        if rgb.shape[:2] != depth_m.shape:
            raise ValueError('Aligned RGB and depth dimensions do not match')
        color_matrix = np.asarray(messages.color_info.k, dtype=np.float64)
        depth_matrix = np.asarray(messages.depth_info.k, dtype=np.float64)
        if not np.allclose(color_matrix, depth_matrix, rtol=0.0, atol=1e-6):
            raise ValueError('Aligned RGB and depth intrinsics do not match')
        if (
            messages.color.header.frame_id
            != messages.color_info.header.frame_id
        ):
            raise ValueError(
                'Color image and CameraInfo frame IDs do not match'
            )
        if (
            messages.depth.header.frame_id
            != messages.depth_info.header.frame_id
        ):
            raise ValueError(
                'Depth image and CameraInfo frame IDs do not match'
            )
        intrinsics = CameraIntrinsics(
            width=messages.depth.width,
            height=messages.depth.height,
            fx=float(messages.depth_info.k[0]),
            fy=float(messages.depth_info.k[4]),
            cx=float(messages.depth_info.k[2]),
            cy=float(messages.depth_info.k[5]),
        )
        return RgbdSnapshot(
            rgb=rgb,
            depth_m=depth_m,
            intrinsics=intrinsics,
            stamp_ns=messages.stamp_ns,
            source_frame=messages.depth.header.frame_id,
        )
    except InspectionFailure:
        raise
    except (TypeError, ValueError) as error:
        raise InspectionFailure(
            FailureKind.DEPTH,
            f'Invalid aligned RGB-D snapshot: {error}',
        ) from error


def _rgb_array(message: Image) -> np.ndarray:
    if message.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported color encoding: {message.encoding}')
    row_bytes = int(message.width) * 3
    if message.step < row_bytes:
        raise ValueError('Color image step is smaller than one row')
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    expected_bytes = int(message.height) * int(message.step)
    if raw.size != expected_bytes:
        raise ValueError('Color image data length does not match metadata')
    rows = raw.reshape(int(message.height), int(message.step))
    rgb = rows[:, :row_bytes].reshape(
        int(message.height),
        int(message.width),
        3,
    )
    if message.encoding == 'bgr8':
        rgb = rgb[:, :, ::-1]
    return np.ascontiguousarray(rgb)


def _depth_array(message: Image, depth_16u_scale_m: float) -> np.ndarray:
    if depth_16u_scale_m <= 0.0:
        raise ValueError('16-bit depth scale must be positive')
    if message.encoding == '32FC1':
        bytes_per_pixel = 4
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        scale = 1.0
    elif message.encoding in ('16UC1', 'mono16'):
        bytes_per_pixel = 2
        dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
        scale = depth_16u_scale_m
    else:
        raise ValueError(f'Unsupported depth encoding: {message.encoding}')
    row_bytes = int(message.width) * bytes_per_pixel
    if message.step < row_bytes:
        raise ValueError('Depth image step is smaller than one row')
    raw_bytes = bytes(message.data)
    expected_bytes = int(message.height) * int(message.step)
    if len(raw_bytes) != expected_bytes:
        raise ValueError('Depth image data length does not match metadata')
    rows = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(
        int(message.height),
        int(message.step),
    )
    packed = np.ascontiguousarray(rows[:, :row_bytes])
    depth = packed.view(dtype).reshape(int(message.height), int(message.width))
    return depth.astype(np.float32) * np.float32(scale)


def _validate_camera_info(info: CameraInfo, image: Image) -> None:
    if info.width != image.width or info.height != image.height:
        raise ValueError('CameraInfo dimensions do not match the image')
    matrix = np.asarray(info.k, dtype=np.float64)
    if matrix.shape != (9,) or not np.isfinite(matrix).all():
        raise ValueError('CameraInfo intrinsic matrix must be finite')
    if matrix[0] <= 0.0 or matrix[4] <= 0.0:
        raise ValueError('CameraInfo focal lengths must be positive')
