import math

import pytest

from icp_convergence import BoundedCycleDetector


def test_every_pair_of_corresponding_cycles_must_match_strict_tolerance():
    detector = BoundedCycleDetector()
    for offset in (0., 9e-6, -9e-6):
        for phase in (0., .0003):
            assert not detector.observe((phase + offset, 0., 0.), final_stage=True)


def feed(detector, poses):
    return [detector.observe(pose, final_stage=True) for pose in poses]


def test_requires_three_complete_cycles_and_detects_bounded_cycle():
    detector = BoundedCycleDetector()
    poses = [(0.0, 0.0, 0.0), (1e-6, 0.0, 1e-6)] * 3
    assert feed(detector, poses[:4]) == [False] * 4
    assert feed(detector, poses[4:]) == [False, True]


def test_angular_wrap_is_treated_as_nearby():
    detector = BoundedCycleDetector()
    poses = [(0.0, 0.0, math.pi - 1e-6), (0.0, 0.0, -math.pi + 1e-6)] * 3
    assert feed(detector, poses) == [False] * 5 + [True]


@pytest.mark.parametrize("poses", [
    [(i * 2e-5, 0.0, 0.0) for i in range(30)],
    [(i % 2 * 0.002, 0.0, 0.0) for i in range(6)],
    [(0.0, 0.0, (i % 2) * 0.001) for i in range(6)],
    [(i % 7, 0.0, 0.0) for i in range(21)],
])
def test_rejects_drift_large_span_and_period_over_limit(poses):
    assert not any(feed(BoundedCycleDetector(), poses))


def test_non_final_stage_resets_history():
    detector = BoundedCycleDetector()
    cycle = [(0.0, 0.0, 0.0), (1e-6, 0.0, 1e-6)]
    assert not any(feed(detector, cycle * 2))
    assert not detector.observe(cycle[0], final_stage=False)
    assert not any(feed(detector, cycle * 2))
    assert not detector.observe(cycle[0], final_stage=True)
    assert detector.observe(cycle[1], final_stage=True)


def test_nonfinite_pose_clears_history():
    detector = BoundedCycleDetector()
    assert not any(feed(detector, [(0, 0, 0), (1, 0, 0), (0, 0, 0), (1, 0, 0)]))
    assert not detector.observe((math.nan, 0, 0), final_stage=True)
    assert not any(feed(detector, [(0, 0, 0), (1, 0, 0)]))


@pytest.mark.parametrize("kwargs", [
    {"translation_tolerance_m": 0},
    {"rotation_tolerance_rad": math.inf},
    {"max_translation_span_m": True},
    {"max_rotation_span_rad": -1},
    {"max_period": 1},
    {"max_period": 2.0},
    {"translation_tolerance_m": 2e-3},
    {"rotation_tolerance_rad": 1.0},
])
def test_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        BoundedCycleDetector(**kwargs)
