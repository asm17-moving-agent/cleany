"""Check straight controller segments against dense collision samples and per-axis limits."""

from bisect import bisect_right
import numpy as np
import pytest

from cleany_skill_executor.core.hardware_execution import (
    PhysicalJointLimit,
    prepare_bounded_path,
)
from cleany_skill_executor.core.joint_path import spline_coefficients


NAMES = ["joint_a", "joint_b", "jaw"]
CAPS = (2.0, 8.0)
LIMITS = {
    n: PhysicalJointLimit(-4.0, 4.0, v, a, 0.0, 1.0, 0.0, -4000.0, 4000.0)
    for n, v, a in zip(NAMES, [2.6, 0.85, 1.0], [7.0, 2.0, 5.0])
}


def prepared(points, *, cruise=False, scale=1.0, maximum=10000, floor=.01):
    return prepare_bounded_path(
        NAMES,
        points,
        LIMITS,
        velocity=CAPS[0],
        acceleration=CAPS[1],
        limit_scale=scale,
        straight_cruise=cruise,
        step=0.02,
        maximum_points=maximum,
        minimum_duration_sec=floor,
    )


@pytest.mark.parametrize(
    "destination", [(3.5, -0.8, 0.5), (-3.6, 1.7, 0.5), (0.0, -1.8, 0.5)]
)
@pytest.mark.parametrize("scale", [0.55, 1.0])
def test_every_collision_sample_and_continuous_rate_limits(destination, scale):
    positions = [(0.0, 0.0, 0.5), destination]
    source = prepared(positions, scale=scale)
    result = prepared(positions, cruise=True, scale=scale)
    assert result.duration < source.duration
    assert np.allclose(result.command_knots[0].derivatives, 0.0)
    assert np.allclose(result.command_knots[-1].derivatives, 0.0)
    knots = result.command_knots
    times = [k.progress for k in knots]
    polynomials = []
    for left, right in zip(knots, knots[1:]):
        step = right.progress - left.progress
        sec = step * result.duration
        c = np.array(
            spline_coefficients(
                np.array(left.positions),
                np.array(right.positions),
                np.array(left.derivatives) * step,
                np.array(right.derivatives) * step,
            )
        )
        polynomials.append(c)
        for j, name in enumerate(NAMES):
            _, v, b, h = c[:, j]
            extrema = [0.0, 1.0]
            if abs(h) > 1e-12 and 0 < -b / (3 * h) < 1:
                extrema.append(-b / (3 * h))
            velocity = max(abs((v + 2 * b * u + 3 * h * u * u) / sec) for u in extrema)
            acceleration = max(abs((2 * b + 6 * h * u) / sec**2) for u in (0.0, 1.0))
            assert velocity <= min(CAPS[0], LIMITS[name].velocity * scale) / 1.05 + 1e-8
            assert (
                acceleration <= min(CAPS[1], LIMITS[name].acceleration * scale) / 1.05**2 + 1e-8
            )
    for sample in result.samples:
        i = min(bisect_right(times, sample.progress) - 1, len(polynomials) - 1)
        u = (sample.progress - times[i]) / (times[i + 1] - times[i])
        actual = np.polynomial.polynomial.polyval(u, polynomials[i])
        np.testing.assert_allclose(actual, sample.positions, atol=1e-9, rtol=0)
    preserved = np.array([sample.positions for sample in result.samples])
    for previous in source.samples:
        assert np.any(np.max(np.abs(preserved - previous.positions), axis=1) < 1e-10)


def test_preserve_waypoints_and_stationary_command():
    for positions in (
        [(0.0, 0.0, 0.0)] * 2,
        [(0.0, 0.0, 0.0), (1.0, 0.4, 0.0), (2.0, 0.0, 0.0)],
    ):
        assert prepared(positions, cruise=True) == prepared(positions)


def test_preserve_duration_floor_and_sample_budget():
    positions = [(0.0, 0.0, 0.5), (3.5, -0.8, 0.5)]
    assert prepared(positions, cruise=True, floor=12).duration == 12
    baseline = prepared(positions)
    assert prepared(positions, cruise=True, maximum=len(baseline.samples)) == baseline
