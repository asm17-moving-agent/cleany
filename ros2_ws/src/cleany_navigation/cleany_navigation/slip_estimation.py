"""Shared wheel/LiDAR comparison, with no ROS or control side effects."""
from __future__ import annotations
from dataclasses import asdict, dataclass
import math
from typing import Any
import numpy as np

@dataclass(frozen=True)
class Settings:
    window_s: float = 1.0
    stride_s: float = 0.5
    min_range_m: float = 0.5
    max_range_m: float = 8.0
    far_range_m: float = 1.5
    voxel_m: float = 0.05
    max_scan_duration_s: float = 0.25
    max_odom_gap_s: float = 0.12
    max_cycle_m: float = 0.02
    max_cycle_rad: float = 0.02
    max_far_disagreement_m: float = 0.03
    max_far_disagreement_rad: float = 0.035
    translation_error_m: float = 0.03
    translation_error_fraction: float = 0.35
    rotation_error_rad: float = 0.07
    rotation_error_fraction: float = 0.35
    minimum_wheel_translation_m: float = 0.025
    minimum_wheel_rotation_rad: float = 0.08
    minimum_consecutive_windows: int = 2

    def __post_init__(self) -> None:
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('All settings must be positive finite numbers')
        if not self.min_range_m < self.far_range_m < self.max_range_m:
            raise ValueError('Require min_range < far_range < max_range')
        if self.stride_s > self.window_s or type(self.minimum_consecutive_windows) is not int:
            raise ValueError('Invalid sampling or persistence settings')


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], wrap(a[2]+b[2])])


def relative_pose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    c, s = math.cos(a[2]), math.sin(a[2])
    d = b-a
    return np.array([c*d[0]+s*d[1], -s*d[0]+c*d[1], wrap(d[2])])


def odom_delta(odom: np.ndarray, t0: float, t1: float, maximum_gap: float) -> np.ndarray:
    """Interpolate bracketing header stamps; reject gaps and pose discontinuities."""
    if len(odom) < 2 or not np.all(np.isfinite(odom)):
        raise ValueError('invalid_odometry')
    if t0 < odom[0, 0] or t1 > odom[-1, 0] or t1 <= t0:
        raise ValueError('unbracketed_odometry')
    left = max(0, int(np.searchsorted(odom[:, 0], t0))-1)
    right = min(len(odom), int(np.searchsorted(odom[:, 0], t1))+1)
    segment = odom[left:right]
    dt = np.diff(segment[:, 0])
    if len(dt) == 0 or np.any(dt <= 0) or np.any(dt > maximum_gap):
        raise ValueError('odometry_gap')
    if np.any(np.linalg.norm(np.diff(segment[:, 1:3], axis=0), axis=1) > .2):
        raise ValueError('odometry_reset_or_discontinuity')
    values = segment[:, 1:4].copy()
    values[:, 2] = np.unwrap(values[:, 2])
    if np.any(abs(np.diff(values[:, 2])) > .2):
        raise ValueError('odometry_reset_or_discontinuity')
    a = np.array([np.interp(t0, segment[:, 0], values[:, i]) for i in range(3)])
    b = np.array([np.interp(t1, segment[:, 0], values[:, i]) for i in range(3)])
    return relative_pose(a, b)


def classify(wheel: np.ndarray, lidar: np.ndarray, valid: bool, config: Settings) -> dict[str, Any]:
    """Flag persistent traction disagreement, not a proven physical slip cause."""
    if not valid or not np.all(np.isfinite(np.r_[wheel, lidar])):
        return {'status': 'unknown', 'kind': 'unreliable_measurement'}
    error = wheel-lidar
    error[2] = wrap(error[2])
    wheel_distance = float(np.linalg.norm(wheel[:2]))
    measured_distance = float(np.linalg.norm(lidar[:2]))
    translation_error = float(np.linalg.norm(error[:2]))
    translation_fraction = translation_error / max(wheel_distance, measured_distance, .01)
    rotation_fraction = abs(error[2]) / max(abs(wheel[2]), abs(lidar[2]), .01)
    translation = (wheel_distance >= config.minimum_wheel_translation_m
                   and translation_error >= config.translation_error_m
                   and translation_fraction >= config.translation_error_fraction)
    rotation = (abs(wheel[2]) >= config.minimum_wheel_rotation_rad
                and abs(error[2]) >= config.rotation_error_rad
                and rotation_fraction >= config.rotation_error_fraction)
    kind = 'translation_and_rotation' if translation and rotation else 'translation' if translation else 'rotation'
    if translation and measured_distance < .25*wheel_distance:
        kind = 'wheel_spin_or_stuck'
    elif translation and measured_distance > .015:
        dot = float(wheel[:2] @ lidar[:2]) / max(wheel_distance*measured_distance, 1e-9)
        if dot < math.cos(math.radians(35)):
            kind = 'direction_mismatch'
    return {'status': 'suspect' if translation or rotation else 'no_evidence',
            'kind': kind if translation or rotation else 'none',
            'error_body': error.tolist(), 'translation_error_m': translation_error,
            'translation_error_fraction': translation_fraction, 'rotation_error_rad': abs(float(error[2]))}


def events(rows: list[dict[str, Any]], config: Settings) -> list[dict[str, Any]]:
    """Require consecutive reliable suspect windows; unknown samples break events."""
    result, active = [], []
    def finish() -> None:
        if len(active) >= config.minimum_consecutive_windows:
            result.append({'start': active[0]['t0'], 'end': active[-1]['t1'],
                           'windows': len(active), 'kinds': sorted({r['kind'] for r in active}),
                           'max_error_m': max(r['translation_error_m'] for r in active)})
        active.clear()
    for row in rows:
        if row['status'] != 'suspect' or (active and row['t0']-active[-1]['t0'] > config.stride_s*1.75):
            finish()
        if row['status'] == 'suspect':
            active.append(row)
    finish()
    return result
