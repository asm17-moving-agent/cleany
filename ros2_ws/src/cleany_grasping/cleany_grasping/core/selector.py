from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cleany_grasping.core.models import GraspPose, PointCloud, RawGrasp
from cleany_grasping.core.ports import GraspPredictor


@dataclass(frozen=True)
class GraspConfig:
    workspace_margin_m: float = 0.04
    target_contact_margin_m: float = 0.015
    maximum_gripper_width_m: float = 0.10
    nms_translation_threshold_m: float = 0.02
    nms_rotation_threshold_rad: float = math.radians(20.0)
    canonical_to_tcp_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    tcp_approach_axis: np.ndarray = field(
        default_factory=lambda: np.array((1.0, 0.0, 0.0))
    )

    def __post_init__(self) -> None:
        conversion = np.asarray(self.canonical_to_tcp_rotation, dtype=float)
        approach = np.asarray(self.tcp_approach_axis, dtype=float)
        if conversion.shape != (3, 3) or not np.allclose(
            conversion.T @ conversion, np.eye(3), atol=1e-5
        ):
            raise ValueError('Canonical-to-TCP rotation must be orthonormal')
        if approach.shape != (3,) or not np.isfinite(approach).all():
            raise ValueError('TCP approach axis must be a finite 3-vector')
        if np.linalg.norm(approach) <= 1e-12:
            raise ValueError('TCP approach axis must not be zero')
        limits = (
            self.workspace_margin_m,
            self.target_contact_margin_m,
            self.maximum_gripper_width_m,
            self.nms_translation_threshold_m,
            self.nms_rotation_threshold_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError('Grasp limits must be finite and positive')
        object.__setattr__(self, 'canonical_to_tcp_rotation', conversion)
        object.__setattr__(self, 'tcp_approach_axis', approach)


def _rotation_distance(left: np.ndarray, right: np.ndarray) -> float:
    cosine = np.clip((np.trace(left.T @ right) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def select_grasp(
    predictor: GraspPredictor,
    target_cloud: PointCloud,
    context_cloud: PointCloud,
    config: GraspConfig | None = None,
) -> GraspPose | None:
    settings = config or GraspConfig()
    if target_cloud.points.shape[0] == 0 or context_cloud.points.shape[0] == 0:
        raise ValueError('Target and context clouds must not be empty')
    target_min = target_cloud.points.min(axis=0)
    target_max = target_cloud.points.max(axis=0)
    workspace_min = target_min - settings.workspace_margin_m
    workspace_max = target_max + settings.workspace_margin_m
    # AnyGrasp SDK order: xmin, xmax, ymin, ymax, zmin, zmax.
    workspace = np.column_stack((workspace_min, workspace_max)).reshape(-1)
    candidates = sorted(
        predictor.predict(context_cloud, workspace),
        key=lambda item: item.score,
        reverse=True,
    )
    kept: list[RawGrasp] = []
    contact_min = target_min - settings.target_contact_margin_m
    contact_max = target_max + settings.target_contact_margin_m
    for candidate in candidates:
        if not 0.0 < candidate.width_m <= settings.maximum_gripper_width_m:
            continue
        contact = candidate.translation + candidate.depth_m * candidate.rotation[:, 0]
        if not np.all((contact >= contact_min) & (contact <= contact_max)):
            continue
        if any(
            np.linalg.norm(candidate.translation - prior.translation)
            < settings.nms_translation_threshold_m
            and _rotation_distance(candidate.rotation, prior.rotation)
            < settings.nms_rotation_threshold_rad
            for prior in kept
        ):
            continue
        kept.append(candidate)
    if not kept:
        return None
    best = kept[0]
    tcp_rotation = best.rotation @ np.asarray(settings.canonical_to_tcp_rotation)
    approach = tcp_rotation @ np.asarray(settings.tcp_approach_axis)
    approach /= np.linalg.norm(approach)
    return GraspPose(
        rotation=tcp_rotation,
        translation=best.translation.copy(),
        approach_direction=approach,
        required_opening_m=best.width_m,
        depth_m=best.depth_m,
        score=best.score,
    )
