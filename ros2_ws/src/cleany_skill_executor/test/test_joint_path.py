import math

import numpy as np
import pytest

from cleany_skill_executor.core.joint_path import CubicJointPath


@pytest.mark.parametrize('positions', [[], [(0.,)], [(0.,), (math.nan,)], [(0.,), (1., 2.)]])
def test_invalid_path_rejected(positions):
    with pytest.raises(ValueError):
        CubicJointPath(positions)


def test_starts_and_ends_at_rest_without_joint_overshoot():
    path = CubicJointPath([(0., 1.), (.2, .9), (.5, .6), (.51, .7)])
    samples = path.samples(.002, 5)
    assert samples[0].positions == (0., 1.)
    assert samples[-1].positions == pytest.approx((.51, .7))
    assert samples[0].derivatives == (0., 0.)
    assert samples[-1].derivatives == pytest.approx((0., 0.), abs=1e-14)
    for segment in range(3):
        for fraction in np.linspace(0., 1., 100):
            sample = path.sample(segment, fraction)
            for j in range(2):
                low, high = sorted(path.positions[segment:segment+2, j])
                assert low-1e-12 <= sample.positions[j] <= high+1e-12


def test_analytical_velocity_and_acceleration_bounds_cover_dense_samples():
    path = CubicJointPath([(0., 0.), (.2, -.05), (.21, -.08), (.8, -.05)])
    velocity, acceleration = path.derivative_bounds()
    duration = path.minimum_duration([.1, .08], [.2, .15])
    assert np.all(velocity/duration <= np.array([.1, .08])+1e-12)
    assert np.all(acceleration/duration**2 <= np.array([.2, .15])+1e-12)
    for segment in range(3):
        previous = None
        for fraction in np.linspace(0., 1., 1001):
            sample = path.sample(segment, fraction)
            assert np.all(np.abs(sample.derivatives) <= velocity+1e-12)
            if previous is not None:
                measured = np.abs(np.subtract(sample.derivatives, previous.derivatives))/.001*3
                assert np.all(measured <= acceleration+1e-10)
            previous = sample


def test_dense_samples_reconstruct_same_cubic_used_by_jtc():
    path = CubicJointPath([(0.,), (.2,), (.4,)])
    samples = path.samples(.01, 10)
    for first, second in zip(samples, samples[1:]):
        # Hermite midpoint reconstructed from emitted positions and velocities.
        h = second.progress-first.progress
        midpoint = .5*(first.positions[0]+second.positions[0]) + h/8*(first.derivatives[0]-second.derivatives[0])
        progress = (first.progress+second.progress)/2
        segment = min(int(progress*2), 1)
        assert midpoint == pytest.approx(path.sample(segment, progress*2-segment).positions[0], abs=1e-12)
        assert abs(second.positions[0]-first.positions[0]) <= .01+1e-12


def test_sampling_and_timing_limits_fail_closed():
    path = CubicJointPath([(0.,), (1.,)])
    with pytest.raises(ValueError, match='sample budget'):
        path.samples(.001, 1, 100)
    for limit in (0., -1., math.nan, math.inf):
        with pytest.raises(ValueError):
            path.minimum_duration([limit], [1.])
        with pytest.raises(ValueError):
            path.samples(limit, 1)


def test_local_derivative_sampling_avoids_global_oversampling():
    path = CubicJointPath([(float(i)*.001,) for i in range(100)] + [(1.,)])
    samples = path.samples(.002, 2, maximum_points=2000)
    assert len(samples) < 1000
    assert max(abs(b.positions[0]-a.positions[0]) for a, b in zip(samples, samples[1:])) <= .002+1e-12
    assert samples[0].progress == 0 and samples[-1].progress == 1
