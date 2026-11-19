"""왜곡 보정된 깊이 투영과 명시적 시뮬레이션 무반사 정책입니다."""

from __future__ import annotations

import math

import numpy as np

from cleany_navigation.body_geometry import BodyBox

try:
    from _cleany_depth_filter import filter as _native_filter
except ImportError:
    _native_filter = None


def self_observation_mask(points: np.ndarray, sensor_origin: np.ndarray,
                          boxes: list[BodyBox], margin: float) -> np.ndarray:
    """유한 선분이 고정 자체 형상과 교차하지 않는 ray만 유지합니다.

    경계의 혼합 픽셀은 레일 뒤쪽 hit를 보고할 수 있습니다. hit 위치만 검사하면
    자체 가림을 놓치므로 로봇 뒤 공간을 비었다고 판단하는 대신 해당 ray를 버립니다.
    """
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('Self-filter margin must be finite and nonnegative')
    if _native_filter is not None and boxes:
        rays = np.ascontiguousarray(points-sensor_origin, dtype=np.float64)
        origins = np.ascontiguousarray([(sensor_origin-b.center)@b.rotation for b in boxes])
        rotations = np.ascontiguousarray([b.rotation for b in boxes], dtype=np.float64)
        halves = np.ascontiguousarray([b.half_size+margin for b in boxes], dtype=np.float64)
        visible = np.isfinite(points).all(axis=1)
        return np.frombuffer(_native_filter(rays, origins, rotations, halves, visible), dtype=bool).copy()
    return _self_observation_mask_python(points, sensor_origin, boxes, margin)


def _self_observation_mask_python(points: np.ndarray, sensor_origin: np.ndarray,
                          boxes: list[BodyBox], margin: float) -> np.ndarray:
    """self_observation_mask()의 ray 선분 계약을 구현한 Python 대체 경로입니다."""
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('Self-filter margin must be finite and nonnegative')
    keep = np.isfinite(points).all(axis=1)
    rays = points-sensor_origin
    for box in boxes:
        origin = (sensor_origin-box.center)@box.rotation
        direction = rays@box.rotation
        half = box.half_size+margin
        # 닫힌 선분과 slab의 같은 교차 검사를 축별로 누적합니다.
        # 상자마다 여러 N×3 임시 배열을 할당하고 축약하는 비용을 피하며
        # 평행 ray와 경계 접촉의 보수적 의미는 유지합니다.
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
    """행 padding을 포함한 REP-118 깊이를 미터 단위로 디코딩합니다."""
    formats = {'32FC1': ('f4', 4, 1.), '16UC1': ('u2', 2, .001)}
    if encoding not in formats:
        raise ValueError('Expected 32FC1 or 16UC1 depth')
    dtype, size, scale = formats[encoding]
    if min(width, height) <= 0 or step < width * size or len(data) < step * height:
        raise ValueError('Truncated depth image')
    array = np.ndarray((height, width), dtype=('>' if is_bigendian else '<')+dtype,
                       buffer=data, strides=(step, size))
    return np.asarray(array, dtype=np.float32) * scale


def rectified_intrinsics(k, p) -> tuple[float, float, float, float] | None:
    """단안 보정 P가 발행되면 해당 fx, fy, cx, cy를, 없으면 K를 사용하며 무효 P에는 None을 반환합니다."""
    if not any(p):
        return k[0], k[4], k[2], k[5]
    if not np.isfinite(p).all() or abs(p[3])+abs(p[7]) > 1e-9:
        return None
    return p[0], p[5], p[2], p[6]


def project_depth_image(
    depth: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    *,
    pixel_stride: int = 4,
    positive_infinity_is_free: bool = False,
    clearing_distance: float = 5.0,
) -> np.ndarray:
    """왜곡 없는 광학 Z 깊이를 X 전방 카메라 XYZ로 투영합니다."""
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
    """유한 hit를 유지하고 선택적으로 Gazebo +inf 픽셀에 유한 ray 끝점을 부여합니다.

    Gazebo Ogre2는 원거리 밖을 +inf, 근거리 안쪽을 -inf로 인코딩합니다. 명시적으로
    활성화한 +inf만 빈 공간으로 취급합니다. NaN, -inf와 0 이하 깊이는 공간 제거
    관측이 되지 않습니다. 끝점은 장애물 표시 범위 밖에 있어야 합니다.
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
    """원래 공간 제거 ray를 유지하며 base_link에서 장애물 hit를 선택합니다."""
    if not math.isfinite(min_z) or not math.isfinite(max_z) or min_z >= max_z:
        raise ValueError('Require finite ordered obstacle height bounds')
    return np.isfinite(body_points).all(axis=1) & (body_points[:, 2] >= min_z) & (body_points[:, 2] <= max_z)
