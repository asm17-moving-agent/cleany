from __future__ import annotations

import pytest

from cleany_skill_executor.core.grasp_pipeline import (
    ContactDebounce,
    ContactLimits,
    ContactSample,
    ObservedGrasp,
    associate_refreshed_grasp,
    linear_approach_error_deg,
)


def _grasp(key, center=(0.4, 0.2, 0.3), approach=(1.0, 0.0, 0.0), closing=(0.0, 1.0, 0.0)):
    return ObservedGrasp(key, center, approach, closing)


def test_reinspection_associates_nearest_same_label_candidate():
    assert associate_refreshed_grasp(
        _grasp(1),
        [_grasp(3, (0.42, 0.2, 0.3)), _grasp(2, (0.405, 0.2, 0.3))],
    ).key == 2


def test_reinspection_rejects_motion_axis_change_and_ambiguity():
    with pytest.raises(ValueError, match='shifted'):
        associate_refreshed_grasp(_grasp(1), [_grasp(2, (0.431, 0.2, 0.3))])
    with pytest.raises(ValueError, match='axes'):
        associate_refreshed_grasp(
            _grasp(1), [_grasp(2, approach=(0.0, 1.0, 0.0))]
        )
    with pytest.raises(ValueError, match='ambiguous'):
        associate_refreshed_grasp(
            _grasp(1),
            [_grasp(2, (0.405, 0.2, 0.3)), _grasp(3, (0.412, 0.2, 0.3))],
        )


def test_linear_approach_alignment_is_directed():
    assert linear_approach_error_deg((0, 0, 0), (1, 0, 0), (1, 0, 0)) == 0.0
    assert linear_approach_error_deg((0, 0, 0), (-1, 0, 0), (1, 0, 0)) == 180.0


def test_contact_requires_five_consecutive_near_stopped_samples():
    monitor = ContactDebounce(ContactLimits(consecutive_samples=5))
    contact = ContactSample(0.01, (0.11,), (0.01,))
    assert [monitor.update(contact) for _ in range(5)] == [False] * 4 + [True]
    monitor = ContactDebounce(ContactLimits(consecutive_samples=2))
    assert monitor.update(contact) is False
    assert monitor.update(ContactSample(0.03, (0.11,), (0.01,))) is False
    assert monitor.update(contact) is False


def test_missing_effort_falls_back_and_effort_threshold_is_optional():
    no_effort = ContactDebounce(ContactLimits(consecutive_samples=1))
    assert no_effort.update(ContactSample(0.01, (0.11,), (0.01,), None))
    effort_only = ContactDebounce(
        ContactLimits(consecutive_samples=1, effort_threshold=2.0)
    )
    assert effort_only.update(ContactSample(0.01, (0.0,), (0.01,), (2.1,)))


def test_endpoint_without_contact_is_not_a_contact_failure():
    monitor = ContactDebounce(ContactLimits(consecutive_samples=1))
    assert not monitor.update(ContactSample(0.0, (0.0,), (0.0,)))
