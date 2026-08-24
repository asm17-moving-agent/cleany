from types import SimpleNamespace

import pytest

from cleany_skill_executor.core.nearest_object import (
    ObjectAttempt,
    rank_object_attempts,
)
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    NearestPregraspCoordinator,
)


def _attempt(
    object_id: int,
    distance_m: float,
    confidence: float,
) -> ObjectAttempt:
    return ObjectAttempt(
        object_id=object_id,
        label=f'object-{object_id}',
        confidence=confidence,
        distance_m=distance_m,
    )


def test_attempts_rank_by_distance_confidence_then_object_id():
    ranked = rank_object_attempts(
        (
            _attempt(3, 0.8, 0.99),
            _attempt(2, 0.4, 0.7),
            _attempt(1, 0.4, 0.9),
            _attempt(4, 0.4, 0.9),
        )
    )

    assert [item.object_id for item in ranked] == [1, 4, 2, 3]


def test_attempts_reject_duplicate_object_ids():
    with pytest.raises(ValueError, match='unique'):
        rank_object_attempts((_attempt(1, 0.2, 0.8), _attempt(1, 0.3, 0.7)))


def test_coordinator_ignores_invalid_depth_detections():
    detections = [
        SimpleNamespace(
            object_id=1,
            label='invalid-near',
            confidence=1.0,
            distance_valid=False,
            distance_m=0.0,
        ),
        SimpleNamespace(
            object_id=2,
            label='far',
            confidence=0.9,
            distance_valid=True,
            distance_m=0.8,
        ),
        SimpleNamespace(
            object_id=3,
            label='near',
            confidence=0.7,
            distance_valid=True,
            distance_m=0.4,
        ),
    ]

    attempts = NearestPregraspCoordinator._attempts(detections)

    assert [item.object_id for item in attempts] == [3, 2]
