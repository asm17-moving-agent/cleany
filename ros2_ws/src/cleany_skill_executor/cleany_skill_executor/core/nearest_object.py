"""ROS-independent nearest-object attempt policy."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True, slots=True)
class ObjectAttempt:
    object_id: int
    label: str
    confidence: float
    distance_m: float
    sorting_category: str = ''
    sorting_reason: str = ''

    def __post_init__(self) -> None:
        if self.object_id <= 0:
            raise ValueError('object_id must be positive')
        if not self.label.strip():
            raise ValueError('label must not be empty')
        if (
            not math.isfinite(self.confidence)
            or not 0.0 <= self.confidence <= 1.0
        ):
            raise ValueError('confidence must be in [0, 1]')
        if not math.isfinite(self.distance_m) or self.distance_m < 0.0:
            raise ValueError('distance_m must be finite and non-negative')


def rank_object_attempts(
    objects: Sequence[ObjectAttempt],
) -> tuple[ObjectAttempt, ...]:
    """Return deterministic nearest-first autonomous attempt order."""

    object_ids = [item.object_id for item in objects]
    if len(set(object_ids)) != len(object_ids):
        raise ValueError('object_id values must be unique')
    return tuple(
        sorted(
            objects,
            key=lambda item: (
                item.distance_m,
                -item.confidence,
                item.object_id,
            ),
        )
    )
