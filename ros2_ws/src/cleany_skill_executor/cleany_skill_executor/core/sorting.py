"""Explicit simulation sorting rules and verified pick/place sequencing.

These label rules are a demo policy, not a claim about ownership or whether a
real cup is disposable. Unknown, dangerous and uncertain items need review.
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


def table_placement_slots(
    center_xy: tuple[float, float], size_xy: tuple[float, float], radius: float,
    *, edge_margin: float = .01, step: float = .04,
) -> list[tuple[float, float]]:
    """Bounded interior sampling, including narrow intervals and the centerline."""
    if (not all(math.isfinite(v) for v in (*center_xy, *size_xy, radius, edge_margin, step))
            or min(*size_xy, radius, edge_margin, step) <= 0):
        raise ValueError('Placement dimensions and sampling parameters must be finite and positive')
    half = [s / 2 - radius - edge_margin for s in size_xy]
    if min(half) < 0:
        return []
    axes = []
    for center, extent in zip(center_xy, half):
        count = min(16, max(1, math.ceil(2 * extent / step)))
        axes.append(sorted({center, *(center - extent + 2 * extent * i / count
                                       for i in range(count + 1))}))
    ys = sorted(axes[1], key=lambda y: abs(y - center_xy[1]))
    return [(x, y) for x in axes[0] for y in ys]


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
    trash_destination: str = 'trash_left'
    lost_item_destination: str = 'lost_items_right'

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

    def classify(self, label: str, confidence: float) -> Decision:
        normalized = normalize_label(label)
        if normalized in self.hazardous_labels:
            return Decision(label, Category.REVIEW, None, 'hazardous_label')
        if (not math.isfinite(confidence) or confidence > 1.0
                or confidence < self.minimum_confidence):
            return Decision(label, Category.REVIEW, None, 'low_confidence')
        if normalized in self.trash_labels:
            return Decision(label, Category.TRASH,
                            self.trash_destination, 'configured_trash_label')
        if normalized in self.lost_item_labels:
            return Decision(
                label, Category.LOST_ITEM,
                self.lost_item_destination, 'configured_lost_label',
            )
        return Decision(label, Category.REVIEW, None, 'unknown_label')

    def classify_model(self, label: str, confidence: float,
                       category: str, reason: str) -> Decision:
        """Consume model semantics, retaining conservative safety vetoes only."""
        if normalize_label(label) in self.hazardous_labels:
            return Decision(label, Category.REVIEW, None, 'hazardous_label')
        if not math.isfinite(confidence) or not self.minimum_confidence <= confidence <= 1:
            return Decision(label, Category.REVIEW, None, 'low_confidence')
        if category not in ('trash', 'lost_item') or not isinstance(reason, str) or not reason.strip():
            return Decision(label, Category.REVIEW, None, 'missing_or_uncertain_model_classification')
        destination = self.trash_destination if category == 'trash' else self.lost_item_destination
        return Decision(label, Category(category), destination, 'gemini: ' + reason.strip())


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
) -> bool:
    """Do not report placement just because a gripper-open command succeeded.

    Exceptions stop the sequence. In particular, a failed transport must not
    trigger release over the floor. Recovery belongs to the physical port.
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
    stage('verify')
    if not port.verify_placement(target, decision.destination):
        raise RuntimeError('Placement could not be verified in destination')
    stage('complete')
    return True
