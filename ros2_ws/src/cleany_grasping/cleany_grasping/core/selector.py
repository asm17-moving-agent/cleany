from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cleany_grasping.core.models import GraspPose, PointCloud, RawGrasp


@dataclass(frozen=True)
class GraspConfig:
    workspace_margin_m: float = 0.04
    target_contact_margin_m: float = 0.015
    maximum_gripper_width_m: float = 0.10
    nms_translation_threshold_m: float = 0.02
    nms_rotation_threshold_rad: float = math.radians(20.0)
    # GraspNet +X를 Cleany 로컬 -Y(접근), GraspNet +Y를 로컬 +X
    # (평행 그리퍼 닫힘)로 변환합니다.
    canonical_to_tcp_rotation: np.ndarray = field(
        default_factory=lambda: np.array(
            ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        )
    )
    tcp_approach_axis: np.ndarray = field(
        default_factory=lambda: np.array((0.0, -1.0, 0.0))
    )

    def __post_init__(self) -> None:
        conversion = np.asarray(self.canonical_to_tcp_rotation, dtype=float).copy()
        approach = np.asarray(self.tcp_approach_axis, dtype=float).copy()
        if conversion.shape != (3, 3) or not np.allclose(
            conversion.T @ conversion, np.eye(3), atol=1e-5
        ):
            raise ValueError('Canonical-to-TCP rotation must be orthonormal')
        if not math.isclose(float(np.linalg.det(conversion)), 1.0, abs_tol=1e-5):
            raise ValueError('Canonical-to-TCP rotation must be right-handed')
        if approach.shape != (3,) or not np.isfinite(approach).all():
            raise ValueError('TCP approach axis must be a finite 3-vector')
        if np.linalg.norm(approach) <= 1e-12:
            raise ValueError('TCP approach axis must not be zero')
        if not np.allclose(approach / np.linalg.norm(approach), (0.0, -1.0, 0.0)):
            raise ValueError('Cleany TCP approach axis must be local -Y')
        if not np.allclose(conversion @ approach, (1.0, 0.0, 0.0)) or not np.allclose(
            conversion @ np.array((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0)
        ):
            raise ValueError(
                'GraspNet +X/+Y must map to Cleany local -Y/+X'
            )
        limits = (
            self.workspace_margin_m,
            self.target_contact_margin_m,
            self.maximum_gripper_width_m,
            self.nms_translation_threshold_m,
            self.nms_rotation_threshold_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError('Grasp limits must be finite and positive')
        conversion.setflags(write=False)
        approach.setflags(write=False)
        object.__setattr__(self, 'canonical_to_tcp_rotation', conversion)
        object.__setattr__(self, 'tcp_approach_axis', approach)


def _rotation_distance(left: np.ndarray, right: np.ndarray) -> float:
    cosine = np.clip((np.trace(left.T @ right) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def rank_grasps(
    candidates: tuple[RawGrasp, ...],
    target_cloud: PointCloud,
    config: GraspConfig | None = None,
) -> tuple[GraspPose, ...]:
    """로봇별 도달 가능성 검사 전에 점수가 매겨진 파지를 필터링하고 중복을 억제합니다.

    예측 접촉점이 여유를 포함한 대상 경계 안에 있는지 확인하고 벌림 한계를 적용하며
    가까운 자세를 억제합니다. 입력 점군과 후보 위치는 같은 좌표계의 미터 단위입니다.

    Returns:
        모델 규약에서 TCP 규약으로 변환한 파지를 점수순으로 반환합니다.
        모든 후보가 탈락하면 빈 튜플입니다. 남은 후보도 로봇 IK와 충돌 검사가 필요합니다.

    Raises:
        ValueError: 대상 점군이 비어 있는 경우.
    """
    settings = config or GraspConfig()
    if target_cloud.points.shape[0] == 0:
        raise ValueError('Target cloud must not be empty')
    target_min = target_cloud.points.min(axis=0)
    target_max = target_cloud.points.max(axis=0)
    sorted_candidates = sorted(candidates, key=lambda item: item.score, reverse=True)
    kept: list[RawGrasp] = []
    contact_min = target_min - settings.target_contact_margin_m
    contact_max = target_max + settings.target_contact_margin_m
    for candidate in sorted_candidates:
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
    ranked = []
    for candidate in kept:
        tcp_rotation = candidate.rotation @ np.asarray(
            settings.canonical_to_tcp_rotation
        )
        approach = tcp_rotation @ np.asarray(settings.tcp_approach_axis)
        approach /= np.linalg.norm(approach)
        ranked.append(
            GraspPose(
                rotation=tcp_rotation,
                translation=candidate.translation.copy(),
                approach_direction=approach,
                required_opening_m=candidate.width_m,
                depth_m=candidate.depth_m,
                score=candidate.score,
            )
        )
    return tuple(ranked)
