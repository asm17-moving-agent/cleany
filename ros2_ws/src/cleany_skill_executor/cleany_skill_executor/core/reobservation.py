"""장면 참값이 아닌 인식 기하에서 카메라 시야 안의 제한된 목표를 생성합니다."""
from __future__ import annotations

import math

import numpy as np

from cleany_skill_executor.core.rgbd_projection import CameraProjection


def sphere_in_view(center: np.ndarray, radius: float, camera: CameraProjection,
                   width: int, height: int, margin_px: float) -> bool:
    """전체 구를 원근 시야의 측면 평면 네 개와 대조합니다."""
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
                         height_clearance_m: float = 0.02) -> tuple[np.ndarray, ...]:
    """중앙 ray 세 개와 들어 올린 상태의 높이 평면 최대 세 개를 교차시킵니다.

    구는 어떤 방향에서든 관측 OBB를 둘러쌉니다. 후보 제안이며 도달 가능성이나
    충돌 검증 또는 물체 유지의 근거가 아닙니다. 선택적으로 낮추더라도 기존 필수
    들림 높이와 여유의 합 아래로 내리지 않습니다. 모든 후보는 새 관측 높이 검사가 필요합니다.
    """
    center = np.asarray(current_center, dtype=float)
    scalars = (radius_m, minimum_z_m, margin_px, maximum_translation_m,
               maximum_lowering_m, height_clearance_m)
    if (center.shape != (3,) or not np.isfinite(center).all()
            or not all(math.isfinite(v) for v in scalars)
            or radius_m <= 0. or maximum_translation_m <= 0.
            or maximum_lowering_m < 0. or height_clearance_m <= 0.
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
    for target_z in heights:
        for fraction in (0.5, 0.35, 0.65):
            u, v = fraction*(width-1), 0.5*(height-1)
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
    return tuple(result)
