"""Observed-voxel collision checks for a planar, fixed-posture simulation robot.

Voxel cubes retain their extent; centers are not treated as point obstacles.
Unknown space is not certified free by this evaluator.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from cleany_gazebo_sim.body_geometry import BodyBox


def voxel_box_hits(centers: np.ndarray, voxel_size: np.ndarray, box: BodyBox,
                   margin: float = 0.) -> np.ndarray:
    """Separating-axis test: map-aligned voxel boxes against an oriented body box."""
    centers = np.asarray(centers, dtype=float).reshape(-1, 3)
    size = np.asarray(voxel_size, dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Invalid voxel size')
    if not np.isfinite(centers).all() or not np.isfinite(margin) or margin < 0:
        raise ValueError('Invalid collision input')
    half = box.half_size+margin
    delta = centers-box.center
    # Cheap map-axis broad phase before the remaining separating axes.
    active = np.flatnonzero((np.abs(delta) <= np.abs(box.rotation)@half+size/2+1e-10).all(axis=1))
    result = np.zeros(len(centers), dtype=bool)
    if not len(active):
        return result
    axes = [*np.eye(3), *box.rotation.T]
    axes.extend(np.cross(a, b) for a in np.eye(3) for b in box.rotation.T)
    axes = np.array([axis for axis in axes if np.linalg.norm(axis) > 1e-10])
    radius = np.abs(axes)@(size/2)+np.abs(axes@box.rotation)@half
    result[active] = (np.abs(delta[active]@axes.T) <= radius+1e-10).all(axis=1)
    return result


def planar_pose(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    pose = np.eye(4)
    pose[:2, :2] = [[c, -s], [s, c]]
    pose[:2, 3] = [x, y]
    return pose


def twist_pose(velocity: np.ndarray, seconds: float) -> np.ndarray:
    vx, vy, wz = velocity
    angle = wz*seconds
    if abs(wz) < 1e-8:
        dx, dy = vx*seconds, vy*seconds
    else:
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = (vx*s+vy*(c-1))/wz, (vx*(1-c)+vy*s)/wz
    return planar_pose(dx, dy, angle)


@dataclass(frozen=True)
class GuardConfig:
    margin: float = .02
    reaction_s: float = .25
    linear_deceleration: float = .4
    angular_deceleration: float = .6
    step_s: float = .05
    slowdown_lookahead_s: float = .5
    slowdown_ratio: float = .35
    max_speed: float = .15
    max_yaw_rate: float = .3

    def __post_init__(self):
        values = list(vars(self).values())
        if not np.isfinite(values).all() or min(values) <= 0 or self.slowdown_ratio > 1:
            raise ValueError('Invalid 3D guard configuration')


class VoxelGuard:
    def __init__(self, boxes: list[BodyBox], config: GuardConfig):
        if not boxes:
            raise ValueError('Body geometry is empty')
        self.boxes, self.config = boxes, config
        self.radius = max(np.linalg.norm(b.center[:2])+np.linalg.norm(b.half_size) for b in boxes)

    def sweep(self, centers: np.ndarray, size: np.ndarray, base_in_map: np.ndarray,
              velocity: np.ndarray, extra_s: float = 0.) -> np.ndarray:
        """Conservative path length for proportional braking at fixed curvature.

        This does not bound arbitrary slip, independent-axis braking or moving
        obstacles. Those assumptions require separate real-robot validation.
        """
        c = self.config
        speed = float(np.linalg.norm(velocity[:2]))
        horizon = c.reaction_s+max(speed/c.linear_deceleration, abs(velocity[2])/c.angular_deceleration)+extra_s
        if not np.isfinite(horizon) or horizon > 5 or horizon < 0:
            raise ValueError('Unbounded prediction horizon')
        times = [0.] if np.all(velocity == 0) else np.linspace(0, horizon, int(np.ceil(horizon/c.step_s))+1)
        # Maximum surface travel between samples, added to the body extent.
        padding = (speed+abs(velocity[2])*self.radius)*c.step_s/2
        hits = np.zeros(len(centers), dtype=bool)
        for seconds in times:
            pose = base_in_map@twist_pose(velocity, seconds)
            for box in self.boxes:
                moved = BodyBox(box.name, pose[:3, :3]@box.center+pose[:3, 3],
                                pose[:3, :3]@box.rotation, box.half_size)
                hits |= voxel_box_hits(centers, size, moved, c.margin+padding)
        return hits

    def evaluate(self, centers: np.ndarray, size: np.ndarray, base_in_map: np.ndarray,
                 requested: np.ndarray, measured: np.ndarray) -> tuple[np.ndarray, dict]:
        c = self.config
        arrays = (centers, size, base_in_map, requested, measured)
        if any(not np.isfinite(a).all() for a in arrays):
            raise ValueError('Non-finite guard input')
        if np.shape(base_in_map) != (4, 4) or np.shape(requested) != (3,) or np.shape(measured) != (3,):
            raise ValueError('Invalid pose or velocity shape')
        command = np.array(requested, dtype=float).copy()
        command[:2] *= min(1., c.max_speed/max(np.linalg.norm(command[:2]), 1e-12))
        command[2] = np.clip(command[2], -c.max_yaw_rate, c.max_yaw_rate)
        near = self.sweep(centers, size, base_in_map, measured)
        near |= self.sweep(centers, size, base_in_map, command)
        far = np.zeros(len(centers), dtype=bool)
        if near.any():
            output, decision = np.zeros(3), 'STOP_OBSTACLE'
        else:
            far = self.sweep(centers, size, base_in_map, command, c.slowdown_lookahead_s)
            output = command*(c.slowdown_ratio if far.any() else 1.)
            decision = 'SLOW_OBSTACLE' if far.any() else 'PASS_OBSERVED_SPACE'
        return output, dict(decision=decision, stop_voxels=int(near.sum()),
                            slow_voxels=int(far.sum()), hit_indices=np.flatnonzero(near|far).tolist())
