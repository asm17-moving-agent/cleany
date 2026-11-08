"""ROS-independent scan/map consistency and bounded stationary pose search."""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Callable

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import least_squares


@dataclass(frozen=True)
class AlignmentSettings:
    min_points: int = 60
    max_points: int = 240
    inlier_distance: float = .15
    min_inlier_ratio: float = .55
    max_mean_distance: float = .18
    distance_cap: float = .6
    bad_duration: float = .8
    good_duration: float = 1.
    degraded_timeout_s: float = 4.
    maximum_degraded_s: float = 8.
    severe_inlier_ratio: float = .10
    severe_mean_distance: float = .35
    progress_distance_m: float = .02
    progress_inlier_ratio: float = .05
    search_radius: float = .8
    search_yaw: float = .7
    coarse_xy: float = .1
    coarse_yaw: float = .0872664626
    recovery_inlier_ratio: float = .65
    improvement: float = .04
    ambiguity_distance: float = .3
    ambiguity_yaw: float = .2
    ambiguity_gap: float = .025

    def __post_init__(self):
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Alignment settings must be finite and positive')
        if not 0 < self.min_inlier_ratio <= self.recovery_inlier_ratio <= 1:
            raise ValueError('Invalid alignment inlier ratios')
        if self.min_points < 6 or self.max_points < 2*self.min_points:
            raise ValueError('Too few independent search and validation points')
        if type(self.min_points) is not int or type(self.max_points) is not int:
            raise ValueError('Point counts must be integers')
        if self.inlier_distance > self.distance_cap or self.max_mean_distance > self.distance_cap:
            raise ValueError('Distance thresholds exceed the residual cap')
        if self.coarse_xy > self.search_radius or self.coarse_yaw > self.search_yaw:
            raise ValueError('Search steps exceed bounds')
        if not self.bad_duration < self.degraded_timeout_s <= self.maximum_degraded_s:
            raise ValueError('Recovery observation durations must be ordered')
        if not self.severe_inlier_ratio < self.min_inlier_ratio or not (
                self.max_mean_distance < self.severe_mean_distance <= self.distance_cap):
            raise ValueError('Severe mismatch must exceed ordinary mismatch')


@dataclass(frozen=True)
class AlignmentScore:
    points: int
    inlier_ratio: float
    mean_distance: float
    median_distance: float
    known_ratio: float

    def acceptable(self, settings: AlignmentSettings, *, recovery=False) -> bool:
        ratio = settings.recovery_inlier_ratio if recovery else settings.min_inlier_ratio
        return (self.points >= settings.min_points and self.inlier_ratio >= ratio and
                self.mean_distance <= settings.max_mean_distance)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


class AlignmentMap:
    """Immutable metric distance field; unknown/outside endpoints remain mismatches."""

    def __init__(self, cells: np.ndarray, resolution: float, origin: tuple[float, float, float]):
        cells = np.asarray(cells)
        if cells.ndim != 2 or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid occupancy grid geometry')
        if not np.isfinite(origin).all() or not np.any(cells >= 65):
            raise ValueError('Map must contain finite geometry and occupied cells')
        self.cells = cells.copy()
        self.resolution = resolution
        self.origin = np.array(origin)
        self.distance = distance_transform_edt(cells < 65)*resolution

    def residuals(self, points: np.ndarray, pose: np.ndarray, cap: float) -> tuple[np.ndarray, np.ndarray]:
        world = transform(points, pose)
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        local = (world-self.origin[:2]) @ np.array([[c, -s], [s, c]]) / self.resolution
        ix, iy = np.floor(local).astype(int).T
        height, width = self.cells.shape
        valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        known = np.zeros(len(points), dtype=bool)
        known[valid] = self.cells[iy[valid], ix[valid]] >= 0
        values = map_coordinates(self.distance, [local[:, 1]-.5, local[:, 0]-.5],
                                 order=1, mode='constant', cval=cap)
        return np.where(known, np.minimum(values, cap), cap), known

    def score(self, points: np.ndarray, pose: np.ndarray, settings: AlignmentSettings) -> AlignmentScore:
        if not np.isfinite(points).all() or not np.isfinite(pose).all():
            raise ValueError('Alignment inputs must be finite')
        if len(points) == 0:
            return AlignmentScore(0, 0., settings.distance_cap, settings.distance_cap, 0.)
        values, known = self.residuals(points, pose, settings.distance_cap)
        # Trim only the largest quarter, to tolerate limited dynamic clutter.
        trimmed = np.sort(values)[:max(1, int(math.ceil(len(values)*.75)))]
        return AlignmentScore(len(points), float(np.mean(values <= settings.inlier_distance)),
                              float(np.mean(trimmed)), float(np.median(values)), float(np.mean(known)))

    def free_position(self, pose: np.ndarray) -> bool:
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        xy = (pose[:2]-self.origin[:2]) @ np.array([[c, -s], [s, c]]) / self.resolution
        x, y = np.floor(xy).astype(int)
        return bool(0 <= y < self.cells.shape[0] and 0 <= x < self.cells.shape[1] and
                    0 <= self.cells[y, x] < 65)


class AlignmentHealth:
    """Retain a verified pose through bounded, speed-limited reconvergence."""

    def __init__(self, settings: AlignmentSettings):
        self.settings = settings
        self.state = 'unknown'
        self.last_stamp = -math.inf
        self.good_since = self.bad_since = None
        self.trusted = False
        self.severe_since = self.progress_at = None
        self.best_distance, self.best_ratio = math.inf, 0.
        self.reason = 'unverified'

    def invalidate(self) -> None:
        """Missing or malformed evidence never receives reconvergence grace."""
        self.state, self.trusted, self.reason = 'unknown', False, 'unverified'
        self.good_since = self.bad_since = self.severe_since = self.progress_at = None
        self.best_distance, self.best_ratio = math.inf, 0.

    def observe(self, stamp: float, score: AlignmentScore | None) -> str:
        if stamp <= self.last_stamp:
            return self.state
        if stamp-self.last_stamp > .6:
            self.invalidate()
        self.last_stamp = stamp
        if score is None or score.points < self.settings.min_points:
            self.invalidate()
        elif score.acceptable(self.settings):
            self.severe_since = None
            # A valid match is progress even before the stable-good interval ends.
            # Keep bad_since so alternating good/bad samples cannot extend the budget.
            self.progress_at = stamp
            if self.good_since is None:
                self.good_since = stamp
            if stamp-self.good_since >= self.settings.good_duration:
                self.state, self.trusted, self.reason = 'available', True, 'aligned'
                self.bad_since = self.progress_at = None
                self.best_distance, self.best_ratio = math.inf, 0.
            elif self.state == 'degraded':
                if stamp-self.bad_since >= self.settings.maximum_degraded_s:
                    self.state, self.reason = 'unavailable', 'reconvergence_budget_exhausted'
                else:
                    self.reason = 'verifying_reconvergence'
        else:
            self.good_since = None
            if self.bad_since is None:
                self.bad_since = stamp
                self.progress_at = stamp
                self.best_distance, self.best_ratio = score.mean_distance, score.inlier_ratio
            if (score.mean_distance <= self.best_distance-self.settings.progress_distance_m or
                    score.inlier_ratio >= self.best_ratio+self.settings.progress_inlier_ratio):
                self.progress_at = stamp
                self.best_distance = min(self.best_distance, score.mean_distance)
                self.best_ratio = max(self.best_ratio, score.inlier_ratio)
            severe = (score.inlier_ratio < self.settings.severe_inlier_ratio and
                      score.mean_distance > self.settings.severe_mean_distance)
            self.severe_since = (stamp if self.severe_since is None else self.severe_since) if severe else None
            if self.state == 'unavailable':
                return self.state
            if self.severe_since is not None and stamp-self.severe_since >= self.settings.bad_duration:
                self.state, self.reason = 'unavailable', 'severe_mismatch'
            elif stamp-self.bad_since >= self.settings.maximum_degraded_s:
                self.state, self.reason = 'unavailable', 'reconvergence_budget_exhausted'
            elif stamp-self.progress_at >= self.settings.degraded_timeout_s:
                self.state, self.reason = 'unavailable', 'reconvergence_stalled'
            elif self.trusted:
                self.state, self.reason = 'degraded', 'reconverging'
        return self.state


def search_pose(model: AlignmentMap, points: np.ndarray, seed: np.ndarray,
                settings: AlignmentSettings, canceled: Callable[[], bool] = lambda: False,
                *, require_improvement: bool = True) -> dict:
    """Validate a unique bounded SE(2) match using held-out beams.

    Confidence-only recovery may retain an already aligned pose; it still needs
    the stricter recovery score and an unambiguous spatial match.
    """
    points, seed = np.asarray(points, dtype=float), np.asarray(seed, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 2*settings.min_points or
            not np.isfinite(points).all() or seed.shape != (3,) or not np.isfinite(seed).all()):
        return {'accepted': False, 'reason': 'insufficient_search_evidence'}
    points = points[np.linspace(0, len(points)-1, min(len(points), settings.max_points), dtype=int)]
    train, holdout = points[::2], points[1::2]
    bounds = np.array([settings.search_radius, settings.search_radius, settings.search_yaw])
    lower, upper = seed-bounds, seed+bounds
    coarse = []
    for yaw in np.arange(-settings.search_yaw, settings.search_yaw+1e-6, settings.coarse_yaw):
        if canceled():
            return {'accepted': False, 'reason': 'search_canceled'}
        for x in np.arange(-settings.search_radius, settings.search_radius+1e-6, settings.coarse_xy):
            for y in np.arange(-settings.search_radius, settings.search_radius+1e-6, settings.coarse_xy):
                if math.hypot(x, y) > settings.search_radius:
                    continue
                pose = seed+np.array([x, y, yaw])
                if not model.free_position(pose):
                    continue
                coarse.append((model.score(train, pose, settings).mean_distance, pose))
    candidates = []
    for _, guess in sorted(coarse, key=lambda item: item[0]):
        if canceled():
            return {'accepted': False, 'reason': 'search_canceled'}
        if any(np.linalg.norm(guess[:2]-p[:2]) < settings.ambiguity_distance/2 and
               abs(wrap(guess[2]-p[2])) < settings.ambiguity_yaw/2 for _, p in candidates):
            continue
        fit = least_squares(lambda p: model.residuals(train, p, settings.distance_cap)[0],
                            np.clip(guess, lower+1e-8, upper-1e-8), bounds=(lower, upper),
                            loss='soft_l1', f_scale=.06, max_nfev=45, diff_step=1e-4)
        pose = fit.x
        if np.linalg.norm(pose[:2]-seed[:2]) > settings.search_radius or not model.free_position(pose):
            continue
        candidates.append((model.score(train, pose, settings).mean_distance, pose))
        if len(candidates) >= 12:
            break
    if not candidates:
        return {'accepted': False, 'reason': 'no_bounded_candidate'}
    candidates.sort(key=lambda item: item[0])
    cost, best = candidates[0]
    alternative = next((score for score, pose in candidates[1:] if
                        np.linalg.norm(pose[:2]-best[:2]) >= settings.ambiguity_distance or
                        abs(wrap(pose[2]-best[2])) >= settings.ambiguity_yaw), None)
    # Include unrefined coarse alternatives, so optimizer convergence cannot hide ambiguity.
    alternatives = [score for score, pose in coarse if
                    np.linalg.norm(pose[:2]-best[:2]) >= settings.ambiguity_distance or
                    abs(wrap(pose[2]-best[2])) >= settings.ambiguity_yaw]
    if alternatives:
        alternative = min([min(alternatives), *([] if alternative is None else [alternative])])
    validation = model.score(holdout, best, settings)
    baseline = model.score(holdout, seed, settings)
    reason = ('candidate_not_well_aligned' if not validation.acceptable(settings, recovery=True) else
              'ambiguous_candidate' if alternative is not None and alternative-cost < settings.ambiguity_gap else
              'insufficient_improvement' if require_improvement and
              baseline.mean_distance-validation.mean_distance < settings.improvement else
              'accepted')
    best[2] = wrap(best[2])
    return {'accepted': reason == 'accepted', 'reason': reason, 'pose': best.tolist(),
            'baseline': asdict(baseline), 'validation': asdict(validation),
            'alternative_gap': None if alternative is None else alternative-cost}
