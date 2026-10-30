"""Exact controller-spline reversal and validation, independent of ROS."""

from dataclasses import replace

import pytest

from cleany_skill_executor.core.hardware_execution import (
    CommandPathValidationError, JointRangeValidationError, PhysicalJointLimit, bounded_path,
)
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory, validate_recorded_trajectory, stopped_recorded_window


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
