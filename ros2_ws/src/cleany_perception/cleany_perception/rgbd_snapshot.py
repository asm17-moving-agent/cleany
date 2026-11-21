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
    """색상, 깊이와 두 보정값을 정확히 같은 타임스탬프로 묶습니다.

    Condition으로 보호한 콜백은 maximum_pending_stamps개 이하의 부분 묶음을
    유지합니다. 완성 묶음이 이전 것을 대체한 뒤 오래되거나 같은 stamp는 무시합니다.
    메시지를 복사하지 않고 보관하므로 생성자와 소비자는 삽입 이후 변경하면 안 됩니다.
    """
    _KINDS = ('color', 'color_info', 'depth', 'depth_info')

    def __init__(self, maximum_pending_stamps: int = 8) -> None:
        if maximum_pending_stamps <= 0:
            raise ValueError('Maximum pending RGB-D stamps must be positive')
        self._maximum_pending_stamps = maximum_pending_stamps
        self._condition = threading.Condition()
        self._pending: dict[int, dict[str, Any]] = {}
        self._latest: SynchronizedRgbdMessages | None = None
        self._sequence = 0
        self._received = {kind: 0 for kind in self._KINDS}
        self._last_received_stamp = {kind: 0 for kind in self._KINDS}

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    @property
    def latest_stamp_ns(self) -> int | None:
        with self._condition:
            return None if self._latest is None else self._latest.stamp_ns

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
        """호출자의 순서 cursor보다 새로 완성된 묶음을 기다립니다.

        시간 제한은 ROS 시계와 독립적인 단조 경과 시간입니다.
        대기 중 취소는 최대 50ms 간격으로 확인합니다.

        Raises:
            ValueError: 시간 제한이 양수가 아닌 경우.
            InspectionFailure: 대기 시간이 초과되거나 취소를 확인한 경우.
        """
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
                        'RGB-D snapshot; '
                        f'received={self._received}, '
                        f'last_stamp_ns={self._last_received_stamp}, '
                        f'completed={self._sequence}, pending={len(self._pending)}',
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
            self._received[kind] += 1
            self._last_received_stamp[kind] = stamp_ns
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
    """이미 짝지어 정렬한 RGB-D 촬영을 디코딩합니다.

    영상 크기, 광학 좌표계와 내부 파라미터가 일치해야 합니다.
    변환 전 시각 짝짓기는 RgbdSnapshotBuffer가 담당합니다.

    Args:
        depth_16u_scale_m: 16UC1 저장 깊이 한 단위당 미터 수.
            32FC1 깊이는 이미 미터 단위입니다.

    Returns:
        촬영 원본 좌표계의 RGB 픽셀, 미터 깊이와 내부 파라미터이며
        나노초 타임스탬프를 유지합니다.

    Raises:
        InspectionFailure: 인코딩, 보정 또는 정렬이 잘못된 경우.
    """
    try:
        rgb = rgb_array(messages.color)
        depth_m = _depth_array(messages.depth, depth_16u_scale_m)
        _validate_camera_info(messages.color_info, messages.color)
        _validate_camera_info(messages.depth_info, messages.depth)
        if rgb.shape[:2] != depth_m.shape:
            raise ValueError('Aligned RGB and depth dimensions do not match')
        if messages.color.header.frame_id != messages.depth.header.frame_id:
            raise ValueError('Aligned RGB and depth optical frames differ')
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


def rgb_array(message: Image) -> np.ndarray:
    if message.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'Unsupported color encoding: {message.encoding}')
    row_bytes = int(message.width) * 3
    if message.step < row_bytes:
        raise ValueError('Color image step is smaller than one row')
    raw = np.frombuffer(message.data, dtype=np.uint8)
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
    result = np.ascontiguousarray(rgb)
    result.flags.writeable = False
    return result


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
    raw_bytes = memoryview(message.data)
    expected_bytes = int(message.height) * int(message.step)
    if len(raw_bytes) != expected_bytes:
        raise ValueError('Depth image data length does not match metadata')
    rows = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(
        int(message.height),
        int(message.step),
    )
    packed = np.ascontiguousarray(rows[:, :row_bytes])
    depth = packed.view(dtype).reshape(int(message.height), int(message.width))
    if dtype == np.dtype('<f4') and scale == 1.0:
        depth.flags.writeable = False
        return depth
    result = depth.astype(np.float32) * np.float32(scale)
    result.flags.writeable = False
    return result


def _validate_camera_info(info: CameraInfo, image: Image) -> None:
    if info.width != image.width or info.height != image.height:
        raise ValueError('CameraInfo dimensions do not match the image')
    matrix = np.asarray(info.k, dtype=np.float64)
    if matrix.shape != (9,) or not np.isfinite(matrix).all():
        raise ValueError('CameraInfo intrinsic matrix must be finite')
    if matrix[0] <= 0.0 or matrix[4] <= 0.0:
        raise ValueError('CameraInfo focal lengths must be positive')
