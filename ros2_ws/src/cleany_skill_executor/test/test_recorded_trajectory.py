"""Exact controller-spline reversal and validation, independent of ROS."""
from dataclasses import replace

import pytest

from cleany_skill_executor.core.hardware_execution import (
    CommandPathValidationError, JointRangeValidationError, PhysicalJointLimit, bounded_path,
)
from cleany_skill_executor.core.recorded_trajectory import (
    CommandPoint, RecordedTrajectory, validate_recorded_trajectory,
    validate_command_history,
    reversed_command_history, recorded_window, descending_height_time, stopped_recorded_window,
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


def test_fixed_roll_still_has_to_fit_physical_encoder_range():
    path, limits = recorded_path()
    with pytest.raises(JointRangeValidationError) as rejected:
        validate_recorded_trajectory(path.reversed_with_roll('roll', 3.5), limits,
                                     velocity=.8, acceleration=1.2, step=.03)
    assert rejected.value.joint_name == 'roll'


@pytest.mark.parametrize('bad', ['time', 'width', 'nan', 'missing_roll'])
def test_invalid_recording_cannot_authorize_replay(bad):
    path, _ = recorded_path()
    if bad == 'time':
        path = replace(path, points=(path.points[0], replace(path.points[1], time_ns=0)))
    elif bad == 'width':
        path = replace(path, points=(path.points[0], replace(path.points[1], positions=(0.,))))
    elif bad == 'nan':
        path = replace(path, points=(path.points[0], replace(path.points[1], velocities=(float('nan'), 0., 0.))))
    with pytest.raises(CommandPathValidationError):
        path.reversed_with_roll('missing' if bad == 'missing_roll' else 'roll', .5)


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


def test_timed_reverse_keeps_original_joint_curve_and_remainder_for_feedback():
    forward, _ = recorded_path()
    reverse = reversed_command_history([forward])
    cut = descending_height_time(reverse, lambda q:q[1], .2)
    point = reverse.sample_at(cut)
    assert point.positions[1] == pytest.approx(.2, abs=1e-7)
    later = descending_height_time(reverse, lambda q:q[1], .195, start_ns=cut)
    assert later > cut
    assert recorded_window(reverse, cut, later).points[0].positions == point.positions


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


def test_timed_feedback_continuation_starts_and_stops_at_rest_with_current_limits():
    forward, limits = recorded_path()
    reverse = reversed_command_history([forward])
    start = descending_height_time(reverse,lambda q:q[1],.25)
    end = descending_height_time(reverse,lambda q:q[1],.245,start_ns=start)
    assert any(abs(v) > .001 for v in reverse.sample_at(start).velocities)
    replay = stopped_recorded_window(reverse,start,end,limits,velocity=.2,acceleration=.3)
    assert replay.points[0].velocities == replay.points[-1].velocities == (0.,)*3
    validate_recorded_trajectory(replay,limits,velocity=.2,acceleration=.3,step=.01)
    assert replay.points[0].positions == reverse.sample_at(start).positions
    assert replay.points[-1].positions == reverse.sample_at(end).positions


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
