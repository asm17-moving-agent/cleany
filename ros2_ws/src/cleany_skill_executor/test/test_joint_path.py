import math

import numpy as np
import pytest

from cleany_skill_executor.core.joint_path import CubicJointPath, executable_joint_path


def test_samples_preserve_boundary_goals_exactly():
    # Measured left-gripper release start and configured upper limit from the
    # 2026-10-16 failure: Horner evaluation added 4.44e-16 rad at sample 274.
    positions = [(-0.07694070477797149,), (1.74532919957,)]
    path = executable_joint_path(positions)
    samples = path.samples(.01, 2)
    assert len(samples) == 275
    assert samples[0].positions == positions[0]
    assert samples[-1].positions == positions[-1]
    assert all(positions[0][0] <= sample.positions[0] <= positions[-1][0] for sample in samples)


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


def test_executable_path_uses_joint_distance_instead_of_waypoint_index():
    path = executable_joint_path(
        [(0.,), (.001,), (.5,), (1.,)], simplification_tolerance=0.)
    assert path.progressions == pytest.approx((0., .001, .5, 1.))
    # All three secants represent the same dq/ds despite very different input
    # waypoint spacing, so the interior spline does not inject a speed pulse.
    assert path.sample(0, 1.).derivatives[0] == pytest.approx(1.)
    assert path.sample(1, 0.).derivatives[0] == pytest.approx(1.)
    assert path.sample(1, 1.).derivatives[0] == pytest.approx(1.)


def test_executable_path_removes_only_sub_tolerance_collinear_noise():
    straight = executable_joint_path([
        (0., 0.), (.2, .0005), (.4, -.0004), (.6, 0.),
    ], simplification_tolerance=.001)
    assert len(straight.positions) == 2
    bend = executable_joint_path([
        (0., 0.), (.2, .01), (.4, 0.),
    ], simplification_tolerance=.001)
    assert len(bend.positions) == 3
    endpoint = executable_joint_path([
        (0.,), (.5,), (.50000001,),
    ], simplification_tolerance=0., duplicate_tolerance=1e-6)
    assert endpoint.positions[-1, 0] == .50000001


@pytest.mark.parametrize('progressions', [
    (0., .5), (0., 1., .5), (0., 0., 1.), (0., math.nan, 1.),
])
def test_invalid_nonuniform_progressions_are_rejected(progressions):
    with pytest.raises(ValueError, match='progressions'):
        CubicJointPath([(0.,), (.5,), (1.,)], progressions)


def test_local_retiming_keeps_the_tiny_entry_knot_without_slowing_the_whole_arm():
    # 2026-10-21 ffba743f pregrasp: the 0.47-degree entry segment previously
    # stretched the synchronized arm/jaw command to 81.59 seconds.
    points = [
        (1.5968740001889525, 3.449922791954807, 3.133922749650365,
         -1.744136155825974, 1.5247769031583274),
        (1.5978523830182387, 3.441735511185412, 3.128212685831445,
         -1.7377200584202257, 1.522670606857331),
        (-.25156481213915877, 2.2085029827901934, .770669014855935,
         -1.4109097699702258, -.10514646871656987),
        (-1.0965014218881726, 2.7890012646094897, -.1004673557340166,
         -1.7101768756016158, -.6389970755618308),
    ]
    arm = executable_joint_path(points)
    start, end = -.33925141950641613, 1.1134223409647397
    path = executable_joint_path([
        (*q, start+(end-start)*s*s*(3-2*s)) for s, q in zip(arm.progressions, arm.positions)])
    assert path.minimum_duration([.8]*6, [1.2]*6)*1.05 == pytest.approx(81.5887646765867)
    retimed = path.retimed([.8]*6, [1.2]*6)
    assert np.array_equal(retimed.positions, path.positions)
    assert retimed.minimum_duration([.8]*6, [1.2]*6)*1.05 < 8.
    assert retimed.sample(0, 0.).derivatives == (0.,)*6
    assert retimed.sample(2, 1.).derivatives == pytest.approx((0.,)*6, abs=1e-12)
    # Every retained junction has a single velocity; there is no artificial
    # stop at the small waypoint and no joint overshoot between the knots.
    for segment in range(2):
        assert retimed.sample(segment, 1.).derivatives == pytest.approx(
            retimed.sample(segment+1, 0.).derivatives, abs=1e-11)
    assert any(abs(v) > .1 for v in retimed.sample(0, 1.).derivatives)
    verify_retimed_limits_and_jtc(retimed, [.8]*6, [1.2]*6)


def verify_retimed_limits_and_jtc(path, velocity_limits, acceleration_limits):
    duration = path.minimum_duration(velocity_limits, acceleration_limits)
    velocity, acceleration = path.derivative_bounds()
    assert np.all(velocity/duration <= np.asarray(velocity_limits)+1e-12)
    assert np.all(acceleration/duration**2 <= np.asarray(acceleration_limits)+1e-12)
    samples = path.samples(.01, 2, 10000)
    for first, second in zip(samples, samples[1:]):
        h = second.progress-first.progress
        midpoint = .5*np.add(first.positions, second.positions)+h/8*np.subtract(
            first.derivatives, second.derivatives)
        progress = (first.progress+second.progress)/2
        segment = min(np.searchsorted(path.progressions, progress, side='right')-1,
                      len(path.positions)-2)
        u = (progress-path.progressions[segment])/(path.progressions[segment+1]-path.progressions[segment])
        assert midpoint == pytest.approx(path.sample(segment, u).positions, abs=1e-11)
        low, high = np.min(path.positions[segment:segment+2], axis=0), np.max(
            path.positions[segment:segment+2], axis=0)
        assert np.all(midpoint >= low-1e-12) and np.all(midpoint <= high+1e-12)


def test_dense_straight_path_does_not_confine_acceleration_to_one_tiny_segment():
    path = executable_joint_path([(i/100.,) for i in range(101)], simplification_tolerance=0.)
    retimed = path.retimed([.8], [1.2])
    assert np.array_equal(retimed.positions, path.positions)
    assert retimed.minimum_duration([.8], [1.2]) < 4.
    verify_retimed_limits_and_jtc(retimed, [.8], [1.2])


@pytest.mark.parametrize('points', [
    [(0., 0.), (.00001, -.0003), (.8, -.02), (.801, -.01), (.7, .4)],
    [(0., 0.), (0., 0.)], [(0., 0.), (.2, -.8)],
])
def test_local_retiming_handles_reversals_stationary_joints_and_unequal_limits(points):
    path = executable_joint_path(points, simplification_tolerance=0.)
    limits = ([.08, .3], [.2, .1])
    retimed = path.retimed(*limits)
    assert np.array_equal(retimed.positions, path.positions)
    assert retimed.minimum_duration(*limits) <= path.minimum_duration(*limits)
    verify_retimed_limits_and_jtc(retimed, *limits)


@pytest.mark.parametrize('limits', [([0.], [1.]), ([1.], [math.nan]), ([1., 2.], [1.])])
def test_local_retiming_rejects_invalid_limits(limits):
    with pytest.raises(ValueError, match='limits'):
        CubicJointPath([(0.,), (1.,)]).retimed(*limits)
