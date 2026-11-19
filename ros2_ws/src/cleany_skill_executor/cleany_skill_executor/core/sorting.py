"""명시적 시뮬레이션 분류 규칙과 확인된 집기 및 놓기 순서입니다.

label 규칙은 데모 정책이며 소유권이나 실제 컵의 폐기 가능성을 의미하지 않습니다.
알 수 없거나 위험하거나 불확실한 물체는 검토가 필요합니다.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol

import yaml


class Category(str, Enum):
    TRASH = 'trash'
    LOST_ITEM = 'lost_item'
    REVIEW = 'review'


COLLECTION_SKILLS = {'trash': 'collect_trash', 'lost_item': 'collect_lost_item'}


def classify_model(label: str, confidence: float, category: str, reason: str, *,
                   hazardous_labels: frozenset[str], minimum_confidence: float,
                   trash_destination: str, lost_item_destination: str) -> Decision:
    """공통 의미 판정입니다. label은 안전을 이유로 거부할 수 있지만 허가를 부여하지는 않습니다."""
    if normalize_label(label) in hazardous_labels:
        return Decision(label, Category.REVIEW, None, 'hazardous_label')
    if not math.isfinite(confidence) or not minimum_confidence <= confidence <= 1:
        return Decision(label, Category.REVIEW, None, 'low_confidence')
    if category not in COLLECTION_SKILLS or not isinstance(reason, str) or not reason.strip():
        return Decision(label, Category.REVIEW, None, 'missing_or_uncertain_model_classification')
    destination = trash_destination if category == 'trash' else lost_item_destination
    return Decision(label, Category(category), destination or None, 'gemini: ' + reason.strip())


def bin_release_region(
    center_xy: tuple[float, float], outside_size: tuple[float, float, float],
    wall: float, top_z: float, radius: float, minimum_clearance: float,
    maximum_clearance: float, edge_margin: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """전체 경계 구가 입구 안에 남도록 제한하는 물체 중심 범위입니다."""
    values = (*center_xy, *outside_size[:2], wall, top_z, radius,
              minimum_clearance, maximum_clearance, edge_margin)
    if (not all(math.isfinite(v) for v in values)
            or min(*outside_size[:2], wall, radius, minimum_clearance, edge_margin) <= 0
            or maximum_clearance < minimum_clearance):
        raise ValueError('Invalid bin release region dimensions or clearances')
    half = [s / 2 - wall - radius - edge_margin for s in outside_size[:2]]
    if min(half) <= 0:
        raise ValueError('Payload bounding sphere does not fit inside bin opening')
    lower = (center_xy[0]-half[0], center_xy[1]-half[1], top_z+radius+minimum_clearance)
    upper = (center_xy[0]+half[0], center_xy[1]+half[1], top_z+radius+maximum_clearance)
    return lower, upper


def validate_release_point(point, lower, upper, *, tolerance_m: float = .001) -> None:
    """계획 오차 범위 전체가 허용된 놓기 영역 안에 있는지 검사합니다."""
    if (len(point) != 3 or not all(math.isfinite(v) for v in point)
            or not math.isfinite(tolerance_m) or tolerance_m < 0
            or any(p - tolerance_m < lo or p + tolerance_m > hi
                   for p, lo, hi in zip(point, lower, upper, strict=True))):
        raise ValueError('Configured release point does not fit observed payload')


def validate_payload_above_opening(center, radius: float, center_xy, outside_size,
                                   wall: float, top_z: float) -> None:
    """관측된 물체 경계 구가 수거함 내부 폭과 입구 높이를 모두 만족해야 합니다."""
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('Release payload radius must be positive and finite')
    if (len(center) != 3
            or not all(math.isfinite(v) for v in (*center, *center_xy, *outside_size, wall, top_z))):
        raise ValueError('Release payload geometry must be positive and finite')
    if (any(abs(center[i] - center_xy[i]) + radius >= outside_size[i] / 2 - wall
            for i in range(2)) or center[2] - radius <= top_z):
        raise ValueError('Object is not safely above the bin opening')


@dataclass(frozen=True)
class Decision:
    label: str
    category: Category
    destination: str | None
    reason: str


@dataclass(frozen=True)
class SortingPolicy:
    trash_labels: frozenset[str]
    lost_item_labels: frozenset[str]
    hazardous_labels: frozenset[str]
    minimum_confidence: float
    trash_destination: str
    lost_item_destination: str

    def __post_init__(self) -> None:
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in (0, 1]')
        groups = (self.trash_labels, self.lost_item_labels,
                  self.hazardous_labels)
        if any(not group for group in groups):
            raise ValueError('Each policy label group must be non-empty')
        if any(not label or label != normalize_label(label)
               for group in groups for label in group):
            raise ValueError('Policy labels must be normalized and non-empty')
        if any(groups[i] & groups[j]
               for i in range(3) for j in range(i + 1, 3)):
            raise ValueError('Conflicting label rules are not allowed')
        if (not self.trash_destination or not self.lost_item_destination
                or self.trash_destination == self.lost_item_destination):
            raise ValueError('Two distinct destinations are required')


    def classify_model(self, label: str, confidence: float,
                       category: str, reason: str) -> Decision:
        """모델 의미를 사용하되 보수적 안전 거부 조건만 유지합니다."""
        return classify_model(label, confidence, category, reason,
            hazardous_labels=self.hazardous_labels, minimum_confidence=self.minimum_confidence,
            trash_destination=self.trash_destination, lost_item_destination=self.lost_item_destination)


def normalize_label(label: str) -> str:
    return ' '.join(label.strip().lower().split())


def load_sorting_policy(path: str | Path) -> SortingPolicy:
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(raw, Mapping) or raw.get('schema_version') != 1:
        raise ValueError('Unsupported sorting policy schema')
    rules = raw['rules']

    def labels(key: str) -> frozenset[str]:
        values = rules[key]
        if not isinstance(values, list) or not all(
            isinstance(value, str) for value in values
        ):
            raise ValueError(f'{key} must be a list of labels')
        return frozenset(normalize_label(value) for value in values)

    return SortingPolicy(
        labels('trash'), labels('lost_item'), labels('hazardous'),
        float(raw['minimum_confidence']),
        str(raw['destinations']['trash']),
        str(raw['destinations']['lost_item']),
    )


class SortingPort(Protocol):
    def pick(self, target: Any) -> Any: ...
    def transport(self, held: Any, destination: str) -> None: ...
    def release(self, held: Any, destination: str) -> None: ...
    def verify_placement(self, target: Any, destination: str) -> bool: ...
    def retreat(self, held: Any, destination: str) -> None: ...


def execute_sort(
    target: Any,
    decision: Decision,
    port: SortingPort,
    stage: Callable[[str], None],
    *,
    verify_placement: bool = True,
) -> bool:
    """그리퍼 열기 명령이 성공했다는 이유만으로 배치 성공을 보고하지 않습니다.

    예외는 순서를 중단합니다. 특히 운반 실패가 바닥 위 방출로 이어져서는 안 됩니다.
    복구는 물리 포트가 담당합니다. 검증을 끄면 True는 이동 순서 완료만 의미하며
    별도의 complete_unverified 단계로 배치 확인을 운영자에게 남깁니다.
    """
    if decision.category == Category.REVIEW or decision.destination is None:
        stage('review')
        return False
    stage('pick')
    held = port.pick(target)
    stage('transport')
    port.transport(held, decision.destination)
    stage('release')
    port.release(held, decision.destination)
    stage('retreat')
    port.retreat(held, decision.destination)
    if not verify_placement:
        # 운영자가 관측하는 시뮬레이션입니다. 이동 완료는 배치 검증이 아닙니다.
        stage('complete_unverified')
        return True
    stage('verify')
    if not port.verify_placement(target, decision.destination):
        raise RuntimeError('Placement could not be verified in destination')
    stage('complete')
    return True
