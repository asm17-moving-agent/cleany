"""Measured floor fitting and spatial filtering for depth obstacle previews."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.depth_clearance import self_observation_mask


@dataclass(frozen=True)
class ObstacleFilter:
    """Metre-based preview limits; floor plane is n dot p = offset in base_link."""

    floor_normal: tuple[float, float, float]
    floor_offset: float
    min_height: float = .05
    max_height: float = 1.4
    min_range: float = .25
    max_range: float = 3.0
    neighbor_radius: float = .04
    min_neighbors: int = 3
    voxel_size: float = .02
    self_margin: float = .005
    min_base_z: float | None = None

    def __post_init__(self) -> None:
        n = np.asarray(self.floor_normal)
        values = [self.floor_offset, self.min_height, self.max_height, self.min_range,
                  self.max_range, self.neighbor_radius, self.voxel_size, self.self_margin]
        if n.shape != (3,) or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n), 1) or n[2] < .98:
            raise ValueError('Floor must have a unit near-up normal')
        if not np.isfinite(values).all() or not 0 <= self.min_height < self.max_height or not 0 < self.min_range < self.max_range:
            raise ValueError('Invalid range or height limits')
        if self.neighbor_radius <= 0 or self.voxel_size <= 0 or self.min_neighbors < 1 or self.self_margin < 0:
            raise ValueError('Invalid spatial filter limits')
        if self.min_base_z is not None and not np.isfinite(self.min_base_z):
            raise ValueError('Base-frame height cutoff must be finite')


def fit_floor(points: np.ndarray, *, expected_z: float = -.0955,
              search_band: float = .2, tolerance: float = .02) -> dict:
    """Fit a broad near-ground plane; reject tabletop and sparse candidates."""
    p = np.asarray(points)
    mask = np.isfinite(p).all(axis=1) & (abs(p[:, 2]-expected_z) < search_band)
    mask &= (p[:, 0] > .4) & (p[:, 0] < 3) & (abs(p[:, 1]) < 1.5)
    p = p[mask]
    if len(p) < 300:
        raise ValueError('Insufficient near-ground points')
    rng = np.random.default_rng(7)
    if len(p) > 6000:
        p = p[rng.choice(len(p), 6000, replace=False)]
    best = np.zeros(len(p), dtype=bool)
    for _ in range(250):
        sample = p[rng.choice(len(p), 3, replace=False)]
        normal = np.cross(sample[1]-sample[0], sample[2]-sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if normal[2] < 0:
            normal *= -1
        if normal[2] < .985:
            continue
        offset = normal @ sample[0]
        if abs(offset/normal[2]-expected_z) > search_band:
            continue
        inliers = abs(p @ normal-offset) < tolerance
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 300 or best.mean() < .5:
        raise ValueError('No supported near-ground plane')
    for _ in range(3):
        centre = p[best].mean(axis=0)
        _, _, vh = np.linalg.svd(p[best]-centre, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal *= -1
        offset = float(normal @ centre)
        best = abs(p @ normal-offset) < tolerance
    span = np.ptp(p[best], axis=0)
    if best.mean() < .5 or normal[2] < .985 or span[0] < .5 or span[1] < .5:
        raise ValueError('Floor lacks spatial support')
    residual = p[best] @ normal-offset
    return {'normal': normal.tolist(), 'offset': offset, 'z_at_origin': offset/normal[2],
            'inlier_fraction': float(best.mean()), 'inlier_count': int(best.sum()),
            'span_m': span.tolist(), 'rmse_m': float(np.sqrt(np.mean(residual**2))),
            'p95_m': float(np.percentile(abs(residual), 95))}


def filter_obstacles(points: np.ndarray, origin: np.ndarray, boxes: list[BodyBox],
                     config: ObstacleFilter) -> tuple[np.ndarray, dict]:
    """Keep supported measured hits; excluded points never imply free space."""
    p = np.asarray(points, dtype=float)
    p = p[np.isfinite(p).all(axis=1)]
    distance = np.linalg.norm(p-origin, axis=1)
    p = p[(distance >= config.min_range) & (distance <= config.max_range)]
    counts = {'in_range': len(p)}
    keep = self_observation_mask(p, origin, boxes, config.self_margin)
    counts['self_removed'] = int((~keep).sum())
    p = p[keep]
    height = p @ np.asarray(config.floor_normal)-config.floor_offset
    # A fixed LiDAR-plane cutoff replaces the floor-relative lower bound.
    lower = height >= config.min_height if config.min_base_z is None else p[:, 2] >= config.min_base_z
    keep = lower & (height <= config.max_height)
    counts['height_removed'] = int((~keep).sum())
    p = p[keep]
    if len(p):
        counts_per_point = cKDTree(p).query_ball_point(p, config.neighbor_radius, return_length=True)
        keep = counts_per_point >= config.min_neighbors
        counts['isolated_removed'] = int((~keep).sum())
        p = p[keep]
    else:
        counts['isolated_removed'] = 0
    if len(p):
        # Keep an actually observed representative, not an averaged point through a surface.
        _, first = np.unique(np.floor(p/config.voxel_size).astype(np.int64), axis=0, return_index=True)
        p = p[np.sort(first)]
    counts['output'] = len(p)
    return np.asarray(p, dtype=np.float32).reshape(-1, 3), counts
