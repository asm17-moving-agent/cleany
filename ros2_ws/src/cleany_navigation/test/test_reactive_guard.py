import math

import numpy as np
import pytest

from cleany_navigation.pan_motion_gate import PanGate, PanTracker
from cleany_navigation.reactive_guard import InputWatchdog, PlanarBrakingGuard
from cleany_navigation.voxel_guard import GuardConfig


def guard():
    return PlanarBrakingGuard(np.array([-.25, -.35]), np.array([.25, .35]), GuardConfig())


def test_pan_tracks_asynchronously_and_holds_five_degree_correction():
    tracker = PanTracker(PanGate(pan_min_position_rad=-3.2, pan_max_position_rad=3.2))
    def command(degrees):
        a = math.radians(degrees)
        return .1*math.cos(a), .1*math.sin(a), 0.
    target, reason = tracker.step(command(30), 0.)
    assert target == pytest.approx(math.radians(30)) and reason == 'TRACKING'
    assert tracker.step(command(25), 0.) == (target, 'HOLD_DIRECTION')
    assert tracker.step(command(90), 0.)[0] == pytest.approx(math.pi/2)
    assert tracker.step((0., 0., .1), 0.)[1] == 'HOLD_IDLE'
    tracker.target = math.pi
    target, reason = tracker.step(command(-160), math.pi)
    assert -3.2 <= target <= 3.2 and reason == 'TRACKING'


def test_clear_observations_pass_diagonal_motion_without_pan_or_voxel_input():
    for degrees in (30, 25, -172):
        a = math.radians(degrees)
        command = np.array([.1*math.cos(a), .1*math.sin(a), .02])
        output, decision = guard().evaluate(np.array([[2., 2.]]), command, command, .1)
        np.testing.assert_allclose(output, command)
        assert decision == 'PASS'


def test_obstacle_in_braking_path_stops_including_measured_motion_after_raw_stop():
    g = guard()
    measured = np.array([.15, 0., 0.])
    for requested in (measured, np.zeros(3)):
        output, decision = g.evaluate(np.array([[.32, 0.]]), requested, measured, .1)
        assert not output.any() and decision == 'STOP_OBSTACLE'


def test_source_age_expands_braking_envelope_and_far_obstacle_slows():
    g = guard()
    command = np.array([.15, 0., 0.])
    points = np.array([[.4, 0.]])
    output, decision = g.evaluate(points, command, command, 0.)
    assert decision == 'SLOW_OBSTACLE'
    np.testing.assert_allclose(output, command*.35)
    assert g.evaluate(points, command, command, .4)[1] == 'STOP_OBSTACLE'


def test_measured_overshoot_is_used_for_braking_without_speed_threshold_chatter():
    g = guard()
    measured = np.array([.151, 0., 0.])
    command = np.array([.15, 0., 0.])
    output, reason = g.evaluate(np.empty((0, 2)), command, measured, .1)
    np.testing.assert_allclose(output, command)
    assert reason == 'PASS'
    assert g.evaluate(np.array([[.32, 0.]]), np.zeros(3), measured, .1)[1] == 'STOP_OBSTACLE'


def test_lateral_rotation_and_speed_limit_are_checked():
    g = guard()
    assert g.evaluate(np.array([[0., .42]]), np.array([0., .15, 0.]), np.zeros(3), .1)[1] == 'STOP_OBSTACLE'
    assert g.evaluate(np.array([[.3, .30]]), np.array([0., 0., -.3]), np.zeros(3), .1)[1] == 'STOP_OBSTACLE'
    limited, _ = g.evaluate(np.empty((0, 2)), np.array([.16, .16, -.4]), np.zeros(3), 0.)
    assert np.linalg.norm(limited[:2]) == pytest.approx(.15)
    assert limited[2] == pytest.approx(-.3)
    with pytest.raises(ValueError):
        g.evaluate(np.array([[np.nan, 0.]]), np.zeros(3), np.zeros(3), 0.)


def fresh_watchdog():
    w = InputWatchdog()
    w.sources = {k: (10., 100.) for k in ('scan', 'depth', 'odom', 'joint')}
    return w


@pytest.mark.parametrize('source', ['scan', 'depth', 'odom', 'joint'])
def test_missing_stale_future_and_wall_stale_sources_stop(source):
    for value in (None, (9., 100.), (10.2, 100.), (10., 99.)):
        w = fresh_watchdog()
        if value is None:
            del w.sources[source]
        else:
            w.sources[source] = value
        with pytest.raises(ValueError, match='STALE_'+source.upper()):
            w.check(10.1, 100.1, 100., 100., ('scan', 'depth', 'odom', 'joint'))


def test_command_monitor_clock_failure_stops_and_reset_latches():
    w = fresh_watchdog()
    assert w.check(10.1, 100.1, 100., 100., tuple(w.sources)) == pytest.approx(.1)
    for command_wall, raw_wall in ((99., 100.), (100., 99.)):
        with pytest.raises(ValueError, match='COMMAND_TIMEOUT'):
            w.check(10.1, 100.1, command_wall, raw_wall, ())
    with pytest.raises(ValueError, match='CLOCK_TIMEOUT'):
        w.check(10.1, 100.7, 100.7, 100.7, ())
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        w.check(9., 100.8, 100.8, 100.8, ())
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        w.check(11., 101., 101., 101., ())


def test_clock_reset_latches_before_sensor_or_tf_validation():
    w = fresh_watchdog()
    w.tick(10., 100.)
    # The adapter ticks first, even if a following TF lookup would throw.
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        w.tick(9., 100.1)
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        w.check(11., 100.2, 100.2, 100.2, ())
