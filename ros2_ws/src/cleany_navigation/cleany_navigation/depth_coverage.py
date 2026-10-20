"""Conservative depth-image coverage diagnostics, with no motion authority.

Cubes are certified only when their entire projected rectangle has finite,
farther depth. FOV alone, sparse rays, +inf and persisted occupancy never count
as free. This is a rectified pixel model, not a physical sensor guarantee.
"""
from __future__ import annotations
from dataclasses import dataclass
from itertools import product
import numpy as np
from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.voxel_guard import GuardConfig, VoxelGuard

SIGNS = np.array(list(product((-1., 1.), repeat=3)))


@dataclass(frozen=True)
class DepthView:
    stamp: float
    depth: np.ndarray
    intrinsics: tuple[float, float, float, float]
    optical_in_fixed: np.ndarray

    def __post_init__(self):
        d = np.asarray(self.depth, dtype=np.float32).copy()
        pose = np.asarray(self.optical_in_fixed, dtype=float).copy()
        fx, fy, cx, cy = self.intrinsics
        if (d.ndim != 2 or min(d.shape) < 2 or pose.shape != (4, 4)
                or not np.isfinite([self.stamp, fx, fy, cx, cy]).all() or min(fx, fy) <= 0
                or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3))
                or not np.isclose(np.linalg.det(pose[:3, :3]), 1)):
            raise ValueError('Invalid rectified depth observation')
        d[~np.isfinite(d) | (d <= 0)] = 0  # Unknown, including simulated +inf.
        d.setflags(write=False); pose.setflags(write=False)
        object.__setattr__(self, 'depth', d)
        object.__setattr__(self, 'optical_in_fixed', pose)

    def classify(self, centers: np.ndarray, size: float, fixed_from_base: np.ndarray,
               *, depth_margin: float = .02, pixel_margin: int = 2) -> tuple[np.ndarray, np.ndarray]:
        if not np.isfinite([size, depth_margin]).all() or size <= 0 or depth_margin < 0 or pixel_margin < 0:
            raise ValueError('Invalid coverage margin')
        corners = np.asarray(centers)[:, None, :]+SIGNS[None, :, :]*size/2
        optical_from_base = np.linalg.inv(self.optical_in_fixed)@fixed_from_base
        p = corners@optical_from_base[:3, :3].T+optical_from_base[:3, 3]
        valid = (p[:, :, 2] > 0).all(axis=1) & np.isfinite(p).all(axis=(1, 2))
        fx, fy, cx, cy = self.intrinsics
        uv = p[:, :, :2]/np.maximum(p[:, :, 2:], 1e-12)*[fx, fy]+[cx, cy]
        lo = np.floor(uv.min(axis=1)).astype(int)-pixel_margin
        hi = np.ceil(uv.max(axis=1)).astype(int)+pixel_margin
        height, width = self.depth.shape
        valid &= (lo >= 0).all(axis=1) & (hi < [width, height]).all(axis=1)
        # New information anywhere in the projected rectangle supersedes old
        # free-space evidence, including invalid pixels or a new occluder.
        visible = ((hi >= 0).all(axis=1) & (lo < [width, height]).all(axis=1)
                   & (p[:, :, 2] > 0).any(axis=1))
        result = np.zeros(len(p), dtype=bool)
        far = p[:, :, 2].max(axis=1)+depth_margin
        for i in np.flatnonzero(valid):
            # Includes every pixel and occlusion boundary; no sampled-ray fill.
            result[i] = self.depth[lo[i, 1]:hi[i, 1]+1, lo[i, 0]:hi[i, 0]+1].min() > far[i]
        return visible, result

    def covers(self, centers: np.ndarray, size: float, fixed_from_base: np.ndarray,
               **kwargs) -> np.ndarray:
        return self.classify(centers, size, fixed_from_base, **kwargs)[1]


class SweptDepthCoverage:
    """Bounded, transient observation history; reused guard braking geometry."""
    def __init__(self, boxes: list[BodyBox], config: GuardConfig, *, cell_size: float = .1,
                 timeout: float = .5, max_cells: int = 20000, max_views: int = 8,
                 pose_jump_m: float = .25, pose_jump_rad: float = .3):
        if not np.isfinite([cell_size, timeout, pose_jump_m, pose_jump_rad]).all() or min(cell_size, timeout, max_cells, max_views, pose_jump_m, pose_jump_rad) <= 0:
            raise ValueError('Invalid coverage bounds')
        self.guard = VoxelGuard(boxes, config)
        self.cell_size, self.timeout = cell_size, timeout
        self.max_cells, self.max_views = max_cells, max_views
        self.views: list[DepthView] = []
        self.last_time = -np.inf
        self.last_pose = None
        self.pose_jump_m, self.pose_jump_rad = pose_jump_m, pose_jump_rad

    def clear(self) -> None:
        self.views.clear()
        self.last_time = -np.inf
        self.last_pose = None

    def observe(self, view: DepthView) -> None:
        if self.views and view.stamp <= self.views[-1].stamp:
            return
        self.views = [v for v in self.views if 0 <= view.stamp-v.stamp <= self.timeout]
        self.views.append(view)
        self.views = self.views[-self.max_views:]

    def evaluate(self, now: float, fixed_from_base: np.ndarray,
                 requested: np.ndarray, measured: np.ndarray) -> dict:
        if now < self.last_time:
            self.clear()
        self.last_time = now
        self.views = [v for v in self.views if 0 <= now-v.stamp <= self.timeout]
        report = dict(covered=False, reason='NO_FRESH_DEPTH', required_cells=0, covered_cells=0)
        pose = np.asarray(fixed_from_base)
        if (pose.shape != (4, 4) or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3))
                or not np.isclose(np.linalg.det(pose[:3, :3]), 1)):
            self.clear()
            return dict(report, reason='INVALID_POSE')
        if self.last_pose is not None:
            change = np.linalg.inv(self.last_pose)@pose
            if np.linalg.norm(change[:3, 3]) > self.pose_jump_m or abs(np.arctan2(change[1, 0], change[0, 0])) > self.pose_jump_rad:
                self.clear()
                return dict(report, reason='POSE_JUMP')
        self.last_pose = pose.copy()
        if not self.views:
            return report
        c = self.guard.config
        velocities = [np.asarray(requested), np.asarray(measured)]
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in velocities):
            return dict(report, reason='INVALID_MOTION')
        speed = max(np.linalg.norm(v[:2]) for v in velocities)
        yaw = max(abs(v[2]) for v in velocities)
        if speed > c.max_speed+1e-9 or yaw > c.max_yaw_rate+1e-9:
            return dict(report, reason='MOTION_OUT_OF_BOUNDS')
        age = max(now-v.stamp for v in self.views)
        horizon = c.reaction_s+age+max(speed/c.linear_deceleration, yaw/c.angular_deceleration)
        travel = (speed+yaw*self.guard.radius)*(horizon+c.step_s)+c.margin
        low = np.min([b.center-np.abs(b.rotation)@b.half_size for b in self.guard.boxes], axis=0)-travel
        high = np.max([b.center+np.abs(b.rotation)@b.half_size for b in self.guard.boxes], axis=0)+travel
        size = self.cell_size
        lower, upper = np.floor(low/size).astype(int), np.ceil(high/size).astype(int)
        counts = upper-lower
        if np.prod(counts) > self.max_cells:
            return dict(report, reason='CELL_BUDGET')
        cells = (np.indices(tuple(counts)).reshape(3, -1).T+lower+.5)*size
        required = np.zeros(len(cells), dtype=bool)
        for v in velocities:
            required |= self.guard.sweep(cells, np.full(3, size), np.eye(4), v, extra_s=age)
        # Only cells wholly inside the current physical body are exempt. Partial
        # cells, safety margin and the entering leading surfaces require depth.
        inside = np.zeros(len(cells), dtype=bool)
        for b in self.guard.boxes:
            local = (cells-b.center)@b.rotation
            inside |= (np.abs(local)+np.abs(b.rotation).T@np.full(3, size/2) <= b.half_size).all(axis=1)
        cells = cells[required & ~inside]
        covered = np.zeros(len(cells), dtype=bool)
        resolved = np.zeros(len(cells), dtype=bool)
        for view in reversed(self.views):
            missing = np.flatnonzero(~resolved)
            if not len(missing):
                break
            visible, free = view.classify(cells[missing], size, fixed_from_base, depth_margin=c.margin)
            covered[missing] = free
            resolved[missing] = visible
        return dict(covered=bool(covered.all() and len(cells)),
                    reason='COVERED' if covered.all() and len(cells) else 'UNOBSERVED_SWEEP',
                    required_cells=len(cells), covered_cells=int(covered.sum()),
                    source_age_s=age, views=len(self.views))
