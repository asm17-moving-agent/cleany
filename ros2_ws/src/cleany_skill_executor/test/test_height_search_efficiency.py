"""Height search keeps joint-step bounds and useful positive downward travel."""
import math
import random

import pytest

from cleany_skill_executor.core.hardware_execution import CommandPathValidationError
from cleany_skill_executor.core.recorded_trajectory import (
    CommandPoint, RecordedTrajectory, _cubic_slope_bound, descending_height_time,
)


def test_analytical_slope_bound_covers_interior_extrema_and_both_directions():
    rng = random.Random(26)
    coefficients = [(2., -3., 0.), (-2., 3., 0.), (0., 2., -3.), (0., 0., 0.)]
    coefficients += [tuple(rng.uniform(-10., 10.) for _ in range(3)) for _ in range(100)]
    for a, b, c in coefficients:
        bound = _cubic_slope_bound(a, b, c)
        dense = max(abs(3*a*(i/1000)**2+2*b*(i/1000)+c) for i in range(1001))
        assert dense <= bound+1e-12
        assert bound <= 3*abs(a)+2*abs(b)+abs(c)+1e-12
    assert _cubic_slope_bound(2., -3., 0.) == pytest.approx(1.5)


@pytest.mark.parametrize('start_ns', [0, 100_000_000])
@pytest.mark.parametrize('tolerance', [0., .0001, .005])
def test_reduced_samples_preserve_joint_step_and_height_from_a_partial_window(start_ns, tolerance):
    path = RecordedTrajectory(('q',), (CommandPoint(0, (1.,), (0.,)),
                                       CommandPoint(1_000_000_000, (0.,), (0.,))))
    observed = []
    end = descending_height_time(path, lambda q: observed.append(q[0]) or q[0], .4321,
                                 start_ns=start_ns, height_tolerance_m=tolerance)
    assert start_ns < end < path.points[-1].time_ns
    assert 0 < path.sample_at(end).positions[0]-.4321 <= max(tolerance, 1e-8)
    # Check the coarse forward scan before the first below-target bracket;
    # bisection intentionally visits earlier points inside that bracket.
    crossing = next(i for i, z in enumerate(observed) if z <= .4321)
    assert max(abs(a-b) for a,b in zip(observed[:crossing],observed[1:crossing+1])) <= .01+1e-8
    assert len(observed) < 120  # Old coefficient-sum sampling needed > 600 FK calls.


@pytest.mark.parametrize('tolerance', [0., .005])
def test_equal_height_endpoints_do_not_hide_the_first_downward_crossing(tolerance):
    # Both endpoints are at 1m, while the cubic dips to zero halfway through.
    path = RecordedTrajectory(('q',), (CommandPoint(0, (1.,), (-4.,)),
                                       CommandPoint(1_000_000_000, (1.,), (4.,))))
    end = descending_height_time(path, lambda q:q[0], .5, height_tolerance_m=tolerance)
    assert 0 < end < 500_000_000
    assert 0 < path.sample_at(end).positions[0]-.5 <= max(tolerance, 1e-8)


@pytest.mark.parametrize('start, target', [(.005, 0.), (1., .995), (.5, .496)])
def test_five_mm_tolerance_cannot_return_the_current_start_for_a_small_correction(start, target):
    path = RecordedTrajectory(('q',), (CommandPoint(0, (start,), (0.,)),
                                       CommandPoint(1_000_000_000, (start-1.,), (0.,))))
    end = descending_height_time(path, lambda q:q[0], target,
                                 step_rad=.5, height_tolerance_m=.005)
    assert end > 0
    assert target < path.sample_at(end).positions[0] < start


def test_nonfinite_height_budget_and_cancellation_still_stop_search():
    path = RecordedTrajectory(('q',), (CommandPoint(0, (1.,), (0.,)),
                                       CommandPoint(1_000_000_000, (0.,), (0.,))))
    with pytest.raises(CommandPathValidationError, match='not finite'):
        descending_height_time(path, lambda _: math.nan, .5, height_tolerance_m=.005)
    with pytest.raises(CommandPathValidationError, match='budget'):
        descending_height_time(path, lambda q:q[0], .5, maximum_points=2, height_tolerance_m=.005)
    calls = []
    def canceled(q):
        calls.append(q)
        if len(calls) == 3:
            raise InterruptedError('cancel')
        return q[0]
    with pytest.raises(InterruptedError, match='cancel'):
        descending_height_time(path, canceled, .5, height_tolerance_m=.005)
    assert len(calls) == 3
