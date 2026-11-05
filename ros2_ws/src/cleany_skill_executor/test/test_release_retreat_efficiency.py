import pytest
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, bounded_path


def test_small_retreat_reduces_minimum_wait_without_exceeding_dynamic_limits():
    names = ('q',)
    limits = {'q': PhysicalJointLimit(-2., 2., 2., 2., 2048., 1., 0., 0., 4095.)}
    durations = []
    for floor in (1., .25):
        samples, duration = bounded_path(names, [(0.,), (.018,)], limits,
            velocity=.8, acceleration=1.2, minimum_duration_sec=floor)
        assert samples[0].positions == (0.,) and samples[-1].positions == pytest.approx((.018,))
        assert max(abs(s.derivatives[0]/duration) for s in samples) <= .8
        # A rest-to-rest two-point cubic has max acceleration 6*delta/T^2.
        assert 6*.018/duration**2 <= 1.2
        durations.append(duration)
    assert durations[0] == 1. and .25 <= durations[1] < .4
