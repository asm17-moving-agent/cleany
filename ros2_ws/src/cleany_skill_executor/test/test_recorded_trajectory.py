"""Exact controller-spline reversal and validation, independent of ROS."""

from dataclasses import replace
import math
import random

import pytest

from cleany_skill_executor.core.hardware_execution import (
    CommandPathValidationError, JointRangeValidationError, PhysicalJointLimit, bounded_path,
)
from cleany_skill_executor.core.recorded_trajectory import (
    CommandPoint, RecordedTrajectory, _cubic_slope_bound, descending_height_time,
    recorded_window, reversed_command_history, stopped_recorded_window,
    validate_command_history, validate_recorded_trajectory,
)


def recorded_path():
    names = ('yaw', 'pitch', 'roll')
    limits = {name: PhysicalJointLimit(-3., 3., 2., 4., 2048., 1., 0., 0., 4095.) for name in names}
    samples, duration = bounded_path(names, [(0., .1, .2), (.2, .15, .5), (.15, .4, .8)], limits,
                                     velocity=.8, acceleration=1.2, step=.03)
    points = tuple(CommandPoint(round(s.progress*duration*1e9), s.positions,
                                tuple(v/duration for v in s.derivatives)) for s in samples)
    return RecordedTrajectory(names, points), limits


def interpolate(path, time_ns):
    for first, last in zip(path.points, path.points[1:]):
        if first.time_ns <= time_ns <= last.time_ns:
            dt = (last.time_ns-first.time_ns)/1e9
            u = (time_ns-first.time_ns)/(last.time_ns-first.time_ns)
            return tuple((2*u**3-3*u**2+1)*q0+(u**3-2*u**2+u)*dt*v0
                         +(-2*u**3+3*u**2)*q1+(u**3-u**2)*dt*v1
                         for q0, q1, v0, v1 in zip(first.positions, last.positions,
                                                   first.velocities, last.velocities, strict=True))
    raise AssertionError('Time outside path')


def test_replay_checks_cubic_position_extrema_between_valid_endpoints():
    # This Hermite segment overshoots to 1.5 although both positions are zero.
    path = RecordedTrajectory(('yaw',), (CommandPoint(0, (0.,), (6.,)),
                                       CommandPoint(10**9, (0.,), (-6.,))))
    limits = {'yaw': PhysicalJointLimit(-1., 1., 10., 20., 2048., 1., 0., 0., 4095.)}
    with pytest.raises(JointRangeValidationError) as rejected:
        validate_recorded_trajectory(path, limits, velocity=10., acceleration=20., step=.8)
    assert rejected.value.joint_name == 'yaw' and rejected.value.sample_index > 0


def test_replay_enforces_current_velocity_and_acceleration_limits():
    path, limits = recorded_path()
    with pytest.raises(CommandPathValidationError, match='rate limits'):
        validate_recorded_trajectory(path, limits, velocity=.001, acceleration=1.2, step=.03)
    with pytest.raises(CommandPathValidationError, match='rate limits'):
        validate_recorded_trajectory(path, limits, velocity=.8, acceleration=.001, step=.03)


@pytest.mark.parametrize('duration_ns,density,end_ns', [
    (1031994523,25,492759368), (1424806516,76,1338189622),
    (1284412919,28,585917499), (1091969690,66,794820702),
])
def test_nearly_coincident_warp_samples_do_not_invent_acceleration(duration_ns,density,end_ns):
    limits = {'q':PhysicalJointLimit(-3.,3.,2.,4.,2048.,1.,0.,0.,4095.)}
    source = RecordedTrajectory(('q',),(CommandPoint(0,(0.,),(0.,)),
                                       CommandPoint(duration_ns,(.3,),(0.,))))
    dense = RecordedTrajectory(('q',),tuple(source.sample_at(round(duration_ns*i/density))
                                            for i in range(density+1)))
    result = stopped_recorded_window(dense,0,end_ns,limits,velocity=.8,acceleration=1.2)
    validate_recorded_trajectory(result,limits,velocity=.8,acceleration=1.2,step=.01)
    assert result.points[-1].time_ns < end_ns*3
    assert result.points[-1].positions == pytest.approx(source.sample_at(end_ns).positions, abs=1e-12)


@pytest.mark.parametrize('arm,sign,bounds', [
    ('left', 1., (-2.16, 0.)),
    ('right', -1., (0., 2.0033789089786476)),
])
def test_front_shoulder_replay_rejects_forbidden_interior_with_legal_endpoints(arm, sign, bounds):
    name = f'{arm}_shoulder_yaw_joint'
    # Endpoints are legal; nonzero endpoint velocities cross the zero boundary.
    path = RecordedTrajectory((name,), (
        CommandPoint(0, (-sign*.001,), (sign*.1,)),
        CommandPoint(10**9, (-sign*.001,), (-sign*.1,)),
    ))
    limits = {name: PhysicalJointLimit(*bounds, 1., 2., 2048., -1., 0., 0., 4095.)}
    with pytest.raises(JointRangeValidationError) as rejected:
        validate_recorded_trajectory(path, limits, velocity=1., acceleration=2., step=.5)
    assert rejected.value.joint_name == name


@pytest.mark.parametrize('bad', ['time', 'width', 'nan'])
def test_invalid_recording_cannot_authorize_replay(bad):
    path, _ = recorded_path()
    if bad == 'time':
        path = replace(path, points=(path.points[0], replace(path.points[1], time_ns=0)))
    elif bad == 'width':
        path = replace(path, points=(path.points[0], replace(path.points[1], positions=(0.,))))
    else:
        path = replace(path, points=(path.points[0], replace(path.points[1], velocities=(float('nan'), 0., 0.))))
    with pytest.raises(CommandPathValidationError):
        path.validate()


def test_replay_rejects_encoder_range_violation():
    path, limits = recorded_path()
    path = replace(path, points=tuple(replace(point, positions=(*point.positions[:2], 3.5),
                                                   velocities=(*point.velocities[:2], 0.))
                                     for point in path.points))
    with pytest.raises(JointRangeValidationError) as rejected:
        validate_recorded_trajectory(path, limits, velocity=.8, acceleration=1.2, step=.03)
    assert rejected.value.joint_name == 'roll'


def test_partial_replay_duration_is_independent_of_knot_density():
    names = ('height', 'side', 'roll')
    limits = {name: PhysicalJointLimit(-3., 3., 2., 4., 2048., 1., 0., 0., 4095.) for name in names}
    source = RecordedTrajectory(names, tuple(
        CommandPoint(round(seconds*1e9), positions, (0.,)*3) for seconds, positions in (
            (0., (.44, .01, .4)), (1., (.435, .01, .4)), (2., (.43, .01, .4)),
            (3., (.38, .01, .4)), (5., (.3, 0., .4)))))
    end = 4500000000
    durations = []
    for density in (1, 20, 80):
        points = [source.points[0]]
        for first, last in zip(source.points, source.points[1:]):
            points.extend(source.sample_at(first.time_ns + round((last.time_ns-first.time_ns)*i/density))
                          for i in range(1, density+1))
        dense = RecordedTrajectory(names, tuple(points))
        replay = stopped_recorded_window(dense, 0, end, limits, velocity=.8, acceleration=1.2)
        validate_recorded_trajectory(replay, limits, velocity=.8, acceleration=1.2, step=.01)
        assert replay.points[0].positions == source.points[0].positions
        assert replay.points[-1].positions == pytest.approx(source.sample_at(end).positions)
        assert replay.points[-1].velocities == (0.,)*3
        durations.append(replay.points[-1].time_ns/1e9)
        assert durations[-1] < 6.
    assert max(durations)-min(durations) < .03


def test_partial_replay_starts_and_stops_at_rest_with_current_limits():
    path, limits = recorded_path()
    start = path.points[-1].time_ns // 3
    end = start + path.points[-1].time_ns // 100
    assert any(abs(v) > .001 for v in path.sample_at(start).velocities)
    replay = stopped_recorded_window(path, start, end, limits, velocity=.2, acceleration=.3)
    assert replay.points[0].velocities == replay.points[-1].velocities == (0.,)*3
    validate_recorded_trajectory(replay, limits, velocity=.2, acceleration=.3, step=.01)
    assert replay.points[0].positions == path.sample_at(start).positions
    assert replay.points[-1].positions == path.sample_at(end).positions


def test_reversal_preserves_actual_cubics_and_nonuniform_timing_except_fixed_roll():
    forward, limits = recorded_path()
    backward = forward.reversed_with_roll('roll', .73)
    end = forward.points[-1].time_ns
    assert len(backward.points) == len(forward.points)
    for first, last in zip(forward.points, reversed(backward.points), strict=True):
        assert first.time_ns+last.time_ns == end
        assert first.positions[:2] == last.positions[:2]
        assert tuple(-v for v in first.velocities[:2]) == last.velocities[:2]
        assert last.positions[2] == .73 and last.velocities[2] == 0.
    for i in range(101):
        time_ns = round(end*i/100)
        assert interpolate(backward, end-time_ns)[:2] == pytest.approx(interpolate(forward, time_ns)[:2])
        assert interpolate(backward, end-time_ns)[2] == pytest.approx(.73)
    samples = validate_recorded_trajectory(backward, limits, velocity=.8, acceleration=1.2, step=.03)
    assert samples[0].positions == backward.points[0].positions
    assert samples[-1].positions == pytest.approx(backward.points[-1].positions)


def test_fixed_roll_still_has_to_fit_physical_encoder_range():
    path, limits = recorded_path()
    with pytest.raises(JointRangeValidationError) as rejected:
        validate_recorded_trajectory(path.reversed_with_roll('roll', 3.5), limits,
                                     velocity=.8, acceleration=1.2, step=.03)
    assert rejected.value.joint_name == 'roll'


def test_reverse_prefix_follows_recorded_cubics_and_cuts_before_original_pickup():
    # Nonlinear second joint distinguishes replay from a straight joint chord.
    path = RecordedTrajectory(('height', 'side'), (
        CommandPoint(0, (.3, 0.), (0., 0.)),
        CommandPoint(10**9, (.34, .02), (.03, 0.)),
        CommandPoint(2*10**9, (.40, .01), (0., 0.))))
    correction = RecordedTrajectory(path.joint_names, (
        CommandPoint(0, (.40, .01), (0., 0.)),
        CommandPoint(10**9, (.41, .01), (0., 0.))))
    reverse = reversed_command_history([path, correction])
    end = descending_height_time(reverse, lambda q:q[0]-.017, .32, step_rad=.01)
    points = tuple(p.positions for p in recorded_window(reverse,0,end).points)
    assert points[0] == correction.points[-1].positions
    assert points[-1][0] == pytest.approx(.337, abs=1e-7)
    assert all(q[0] >= .337-1e-9 for q in points)
    assert any(q == path.points[1].positions for q in points)
    # Interior points and endpoint remain on the originally sent spline.
    for q in points:
        if q[0] > .4:
            assert q[1] == .01
            continue
        low, high = 0, 2*10**9
        for _ in range(40):
            middle = (low+high)/2
            if interpolate(path, middle)[0] < q[0]:
                low = middle
            else:
                high = middle
        assert q == pytest.approx(interpolate(path, (low+high)/2), abs=1e-8)


@pytest.mark.parametrize('invalid', ['empty', 'wrong_arm', 'discontinuous', 'not_stopped',
                                    'nonfinite_fk', 'below_goal', 'unreachable', 'budget'])
def test_reverse_prefix_rejects_invalid_history_or_unreachable_release_before_motion(invalid):
    path, _ = recorded_path()
    paths, target, height, budget = [path], .1, lambda q: q[0], 2000
    if invalid == 'empty':
        paths = []
    elif invalid == 'wrong_arm':
        paths.append(replace(path, joint_names=('other', 'pitch', 'roll')))
    elif invalid == 'discontinuous':
        paths.append(path)
    elif invalid == 'not_stopped':
        paths = [replace(path, points=(*path.points[:-1], replace(path.points[-1], velocities=(.1, 0., 0.))))]
    elif invalid == 'nonfinite_fk':
        height = lambda q: float('nan')
    elif invalid == 'below_goal':
        target = .3
    elif invalid == 'unreachable':
        target = -.1
    else:
        budget = 2
    with pytest.raises(CommandPathValidationError):
        descending_height_time(reversed_command_history(paths), height, target,
                               step_rad=.01, maximum_points=budget)


def test_full_reversal_preserves_roll_and_velocity_direction():
    path, _ = recorded_path()
    reverse = path.reversed()
    for before, after in zip(path.points, reversed(reverse.points)):
        assert after.positions == before.positions
        assert after.velocities == tuple(-v for v in before.velocities)
    assert reverse.reversed() == path


def test_reverse_history_validates_every_path_before_sampling():
    path, _ = recorded_path()
    invalid = replace(path, points=(path.points[0], replace(path.points[-1], time_ns=0)))
    with pytest.raises(CommandPathValidationError):
        validate_command_history([invalid, path], path.joint_names)


def test_timed_partial_return_avoids_dense_waypoint_duration_inflation():
    names = ('height', 'side', 'roll')
    limits = {name: PhysicalJointLimit(-3.,3.,2.,4.,2048.,1.,0.,0.,4095.) for name in names}
    paths = []
    # Recorded 2s retreat, 1s lift, two 1s corrections. Densify knots without
    # changing the source splines: density must not alter replay timing.
    for start,end,duration in [((.3,0.,.4),(.38,.01,.4),2.),
                               ((.38,.01,.4),(.43,.01,.4),1.),
                               ((.43,.01,.4),(.435,.01,.4),1.),
                               ((.435,.01,.4),(.44,.01,.4),1.)]:
        paths.append(RecordedTrajectory(names,(CommandPoint(0,start,(0.,)*3),
                                              CommandPoint(round(duration*1e9),end,(0.,)*3))))
    durations = []
    for density in (1, 20, 80):
        sampled = [RecordedTrajectory(names, tuple(p.sample_at(round(p.points[-1].time_ns*i/density))
                                                   for i in range(density+1))) for p in paths]
        reverse = reversed_command_history(sampled)
        end = descending_height_time(reverse, lambda q:q[0], .337)
        replay = stopped_recorded_window(reverse, 0, end, limits, velocity=.8, acceleration=1.2)
        validate_recorded_trajectory(replay, limits, velocity=.8, acceleration=1.2, step=.01)
        assert replay.points[0].positions == reverse.points[0].positions
        assert replay.points[-1].positions[0] == pytest.approx(.337, abs=1e-7)
        assert replay.points[-1].velocities == (0.,)*3
        durations.append(replay.points[-1].time_ns/1e9)
        assert durations[-1] < 6.  # Previously the dense path grew to ~18 seconds.
    assert max(durations)-min(durations) < .03


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


@pytest.mark.parametrize('tolerance', [0., .0001, .005])
def test_reduced_samples_preserve_joint_step_and_release_height(tolerance):
    path = RecordedTrajectory(('q',), (CommandPoint(0, (1.,), (0.,)),
                                       CommandPoint(1_000_000_000, (0.,), (0.,))))
    observed = []
    end = descending_height_time(path, lambda q: observed.append(q[0]) or q[0], .4321,
                                 height_tolerance_m=tolerance)
    assert 0 < end < path.points[-1].time_ns
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
