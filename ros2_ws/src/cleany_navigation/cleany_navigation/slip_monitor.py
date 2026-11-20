"""Bounded live history and persistent, timestamped slip observations."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict
import math
from typing import Any

import numpy as np

from .lidar_scan_matcher import LidarScanMatcher
from .slip_estimation import Settings, classify, compose, odom_delta, relative_pose


def base_cloud(points: np.ndarray, mount: np.ndarray, boxes: list,
               config: Settings, far: bool = False) -> np.ndarray:
    distance = np.linalg.norm(points, axis=1)
    minimum = config.far_range_m if far else config.min_range_m
    cloud = points[(distance >= minimum) & (distance <= config.max_range_m)]
    x, y, angle = mount
    c, s = math.cos(angle), math.sin(angle)
    cloud = cloud @ np.array([[c, s], [-s, c]]) + [x, y]
    for x0, y0, x1, y1 in boxes:
        cloud = cloud[~((cloud[:, 0] >= x0) & (cloud[:, 0] <= x1)
                       & (cloud[:, 1] >= y0) & (cloud[:, 1] <= y1))]
    return cloud


def compare_clouds(a: np.ndarray, b: np.ndarray, far_a: np.ndarray, far_b: np.ndarray,
                   wheel: np.ndarray, matcher: LidarScanMatcher, config: Settings) -> dict[str, Any]:
    matches = [matcher.match(a, b), matcher.match(b, a), matcher.match(far_a, far_b)]
    vectors = [np.array([r.delta_x_m, r.delta_y_m, r.delta_yaw_rad]) for r in matches]
    lidar, reverse, distant = vectors
    cycle, gap = compose(lidar, reverse), relative_pose(lidar, distant)
    reasons = [name+': '+r.rejection_reason for name, r in zip(('forward', 'reverse', 'far'), matches) if not r.is_valid]
    if np.linalg.norm(cycle[:2]) > config.max_cycle_m or abs(cycle[2]) > config.max_cycle_rad:
        reasons.append('forward_reverse_disagree')
    if np.linalg.norm(gap[:2]) > config.max_far_disagreement_m or abs(gap[2]) > config.max_far_disagreement_rad:
        reasons.append('near_far_disagree')
    return dict(classify(wheel, lidar, not reasons, config), wheel_delta=wheel.tolist(),
                lidar_delta=lidar.tolist(), far_delta=distant.tolist(), cycle=cycle.tolist(),
                far_gap=gap.tolist(), reason='; '.join(reasons), match=asdict(matches[0]),
                reverse_match=asdict(matches[1]), far_match=asdict(matches[2]))


class ObservationHistory:
    """Use only consecutive, non-overlapping-in-order results for persistence."""

    def __init__(self, config: Settings):
        self.config = config
        self.last_start = -math.inf
        self.count = 0
        self.observation: dict[str, Any] = dict(state='unknown', t0=0., t1=0., reason='warming_up')

    def update(self, row: dict[str, Any]) -> dict[str, Any]:
        if row['t0'] <= self.last_start:
            return self.observation
        if row['t0']-self.last_start > self.config.stride_s*1.75:
            self.count = 0
        self.last_start = row['t0']
        self.count = self.count+1 if row['status'] == 'suspect' else 0
        state = ('confirmed' if self.count >= self.config.minimum_consecutive_windows else 'suspect') if self.count else ('clear' if row['status'] == 'no_evidence' else 'unknown')
        self.observation = dict(row, state=state, consecutive=self.count)
        return self.observation


class ScanHistory:
    """Keep four seconds of sensor headers; reject clock/frame discontinuities."""

    def __init__(self, config: Settings):
        self.config = config
        self.scans: deque = deque(maxlen=64)
        self.odom: deque = deque(maxlen=512)
        self.last_pair = -math.inf

    def add_odom(self, sample: list[float]) -> None:
        if self.odom and sample[0] <= self.odom[-1][0]:
            self.odom.clear()
        self.odom.append(sample)
        while self.odom and sample[0]-self.odom[0][0] > 4:
            self.odom.popleft()

    def add_scan(self, stamp: float, duration: float, points: np.ndarray) -> None:
        if self.scans and stamp <= self.scans[-1][0]:
            self.scans.clear()
            self.last_pair = -math.inf
        self.scans.append((stamp, duration, points))
        while self.scans and stamp-self.scans[0][0] > 4:
            self.scans.popleft()

    def pair(self) -> tuple | None:
        if len(self.scans) < 2 or len(self.odom) < 2:
            return None
        b = self.scans[-1]
        if b[0] > self.odom[-1][0] or b[0]-self.last_pair < self.config.stride_s:
            return None
        a = min(self.scans, key=lambda s: abs(s[0]-(b[0]-self.config.window_s)))
        if abs(b[0]-a[0]-self.config.window_s) > .15:
            return None
        self.last_pair = b[0]
        return a, b, np.array(self.odom)


def evaluate_pair(a: tuple, b: tuple, odom: np.ndarray, mount: np.ndarray,
                  boxes: list, config: Settings, matcher: LidarScanMatcher) -> dict[str, Any]:
    row = dict(t0=a[0], t1=b[0], status='unknown', kind='unreliable_measurement')
    if any(not math.isfinite(s[1]) or not 0 < s[1] <= config.max_scan_duration_s for s in (a, b)):
        return dict(row, reason='scan_timing')
    try:
        wheel = odom_delta(odom, a[0], b[0], config.max_odom_gap_s)
    except ValueError as error:
        return dict(row, reason=str(error))
    clouds = [base_cloud(s[2], mount, boxes, config, far) for far in (False, True) for s in (a, b)]
    return dict(row, **compare_clouds(*clouds, wheel, matcher, config))
