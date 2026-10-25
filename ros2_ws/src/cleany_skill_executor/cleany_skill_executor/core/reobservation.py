"""Bounded camera-frustum targets from perceived geometry, not scene truth."""
from __future__ import annotations

import math

import numpy as np

from cleany_skill_executor.core.rgbd_projection import CameraProjection


def sphere_in_view(center: np.ndarray, radius: float, camera: CameraProjection,
                   width: int, height: int, margin_px: float) -> bool:
    """Test the entire sphere against the four perspective side planes."""
    optical = np.asarray(camera.rotation_base_from_optical).reshape(3, 3).T @ (
        center - np.asarray(camera.translation_base))
    x, y, z = optical
    if z <= radius:
        return False
    left, right = (margin_px-camera.cx)/camera.fx, (width-1-margin_px-camera.cx)/camera.fx
    top, bottom = (margin_px-camera.cy)/camera.fy, (height-1-margin_px-camera.cy)/camera.fy
    return bool(all(distance >= radius * math.hypot(1., slope) for distance, slope in (
        (x-left*z, left), (right*z-x, right), (y-top*z, top), (bottom*z-y, bottom))))


def reobservation_centers(camera: CameraProjection, width: int, height: int,
                         current_center: np.ndarray, radius_m: float, *,
                         minimum_z_m: float, margin_px: float,
                         maximum_translation_m: float, maximum_lowering_m: float = 0.,
                         height_clearance_m: float = 0.02,
                         maximum_raising_m: float = 0.) -> tuple[np.ndarray, ...]:
    """Propose bounded whole-object views on still-lifted height planes.

    The sphere encloses the observed OBB at any orientation. These are proposals,
    NOT reachability/collision certificates or evidence of object retention.
    If central rays produce no candidate, sample off-center image rows.
    Optional lowering never crosses the unchanged required lift height plus a
    clearance. Optional raising searches additional views within the same total
    translation bound. All candidates require a fresh observed-height check.
    """
    center = np.asarray(current_center, dtype=float)
    scalars = (radius_m, minimum_z_m, margin_px, maximum_translation_m,
               maximum_lowering_m, height_clearance_m, maximum_raising_m)
    if (center.shape != (3,) or not np.isfinite(center).all()
            or not all(math.isfinite(v) for v in scalars)
            or radius_m <= 0. or maximum_translation_m <= 0.
            or maximum_lowering_m < 0. or maximum_raising_m < 0. or height_clearance_m <= 0.
            or width < 2 or height < 2 or margin_px < 0.
            or 2*margin_px >= min(width-1, height-1)):
        raise ValueError('invalid reobservation geometry or limits')
    if center[2] < minimum_z_m:
        return ()
    rotation = np.asarray(camera.rotation_base_from_optical).reshape(3, 3)
    translation = np.asarray(camera.translation_base)
    result = []
    lower = max(minimum_z_m + height_clearance_m, center[2]-maximum_lowering_m)
    heights = ((center[2], (center[2]+lower)/2, lower) if lower < center[2]-1e-9
               else (center[2],))
    if maximum_raising_m:
        heights += (center[2]+maximum_raising_m/2, center[2]+maximum_raising_m)
    for rows in ((0.5,), (0.25, 0.75, 0.15, 0.85)):
        for target_z in heights:
            for vertical in rows:
                for fraction in (0.5, 0.35, 0.65):
                    u, v = fraction*(width-1), vertical*(height-1)
                    direction = rotation @ np.array(((u-camera.cx)/camera.fx,
                                                     (v-camera.cy)/camera.fy, 1.))
                    if abs(direction[2]) <= 1e-9:
                        continue
                    distance = (target_z-translation[2])/direction[2]
                    if distance <= 0.:
                        continue
                    target = translation + direction * distance
                    if (np.linalg.norm(target-center) <= maximum_translation_m
                            and sphere_in_view(target, radius_m, camera, width, height, margin_px)):
                        result.append(target)
        if result:
            if rows != (0.5,):
                result.sort(key=lambda target: float(np.linalg.norm(target-center)))
            return tuple(result)
    return ()
