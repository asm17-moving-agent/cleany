"""Encoder response, feedback loss and watchdog wiring without motor access."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from cleany_skill_executor.core.motion_monitor import MotionMonitor, MotionMonitorConfig
from cleany_skill_executor.hardware_execution import HardwareExecution


CONFIG = MotionMonitorConfig(.07, .5, 1., .03, .005, 1.)
JOINT = 'left_wrist_roll_joint'


@pytest.fixture
def operational_config():
    root = Path(__file__).resolve().parents[4]
    values = yaml.safe_load((root/'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml').read_text())[
        'pregrasp_pipeline']['ros__parameters']
    return MotionMonitorConfig(
        values['hardware_motion_max_error_rad'], values['hardware_motion_error_duration_sec'],
        values['hardware_motion_response_window_sec'], values['hardware_motion_response_error_rad'],
        values['hardware_motion_minimum_motion_rad'], values['hardware_motion_feedback_timeout_sec'],
        values['hardware_motion_abort_on_deviation'])


@pytest.mark.parametrize('error_rad', [.036016, .099, .10])
def test_operational_near_target_hold_does_not_trigger_stall(operational_config, error_rad):
    monitor = MotionMonitor((JOINT,), operational_config, 0.)
    for step in range(21):
        now = step*.1
        monitor.observe((JOINT,), (error_rad,), (0.,), now)
        monitor.check(now)
    assert monitor.report()['fault'] == ''


def test_recorded_shoulder_small_motion_is_accepted_by_operational_deadband(operational_config):
    monitor = MotionMonitor(('right_shoulder_pitch_joint',), operational_config, 0.)
    # The recorded final one-second window moved three encoder ticks while
    # following a similarly small command change with about 0.036 rad bias.
    for step in range(25):
        now = step*.05
        progress = min(now, 1.)
        desired = 2.806466867969594-.0046579570570927*progress
        actual = 2.842466399952093-.004601942363657*progress
        monitor.observe(monitor.names, (desired,), (actual,), now)
        monitor.check(now)
    assert monitor.report()['fault'] == ''


@pytest.mark.parametrize('error_rad', [.10001, .12, .25])
def test_operational_large_stall_still_aborts(operational_config, error_rad):
    monitor = MotionMonitor((JOINT,), operational_config, 0.)
    for now in (0., .25, .5, .75, 1.):
        monitor.observe((JOINT,), (error_rad,), (0.,), now)
    with pytest.raises(RuntimeError, match='Joint did not respond'):
        monitor.check(1.)


def test_operational_relaxed_deadband_keeps_feedback_timeout(operational_config):
    monitor = MotionMonitor((JOINT,), operational_config, 0.)
    monitor.observe((JOINT,), (.036016,), (0.,), 0.)
    with pytest.raises(RuntimeError, match='missing or stale'):
        monitor.check(operational_config.feedback_timeout_sec+.001)


def test_tracking_and_completed_hold_do_not_raise():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    for step in range(101):
        desired = min(step * .01, .5)
        monitor.observe((JOINT,), (desired,), (desired - .02,), step * .05)
        monitor.check(step * .05)
    assert monitor.report()['feedback_samples'] == 101
    assert monitor.report()['fault'] == ''


def test_sustained_error_faults_even_when_encoder_is_moving():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    monitor.observe((JOINT,), (.08,), (0.,), 0.)
    monitor.observe((JOINT,), (.10,), (.02,), .25)
    monitor.check(.25)
    monitor.observe((JOINT,), (.12,), (.04,), .5)
    with pytest.raises(RuntimeError, match='Sustained trajectory deviation'):
        monitor.check(.5)


def test_supervised_deviation_warns_once_per_episode_and_keeps_monitoring():
    monitor = MotionMonitor((JOINT,), replace(CONFIG, abort_on_deviation=False), 0.)
    for step in range(21):
        now = step*.1
        monitor.observe((JOINT,), (now*.1+.1,), (now*.1,), now)
        monitor.check(now)
    warnings = monitor.take_warnings()
    assert len(warnings) == 1
    assert 'Sustained trajectory deviation' in warnings[0]
    assert not monitor.take_warnings()
    assert monitor.report()['deviation_warning_counts_by_joint'] == {JOINT: 1}
    monitor.observe((JOINT,), (.3,), (.3,), 2.1)
    for step in range(7):
        now = 2.2+step*.1
        monitor.observe((JOINT,), (now*.1+.1,), (now*.1,), now)
    assert len(monitor.take_warnings()) == 1
    assert monitor.report()['deviation_warning_counts_by_joint'] == {JOINT: 2}


def test_supervised_warning_does_not_disable_stall_or_feedback_faults():
    monitor = MotionMonitor((JOINT,), replace(CONFIG, abort_on_deviation=False), 0.)
    for now in (0., .5, 1.):
        monitor.observe((JOINT,), (.1,), (0.,), now)
    with pytest.raises(RuntimeError, match='Joint did not respond'):
        monitor.check(1.)
    missing = MotionMonitor((JOINT,), replace(CONFIG, abort_on_deviation=False), 0.)
    with pytest.raises(RuntimeError, match='missing or stale'):
        missing.check(1.1)


def test_short_deviation_resets_before_a_later_deviation():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    for now, desired, actual in ((0., .08, 0.), (.25, .08, .08), (.5, .2, .12),
                                 (.75, .22, .14)):
        monitor.observe((JOINT,), (desired,), (actual,), now)
        monitor.check(now)
    monitor.observe((JOINT,), (.24,), (.16,), 1.)
    with pytest.raises(RuntimeError, match='Sustained trajectory deviation'):
        monitor.check(1.)


def test_stall_faults_below_the_large_error_limit():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    for now in (0., .5):
        monitor.observe((JOINT,), (.04,), (0.,), now)
        monitor.check(now)
    monitor.observe((JOINT,), (.04,), (0.,), 1.)
    with pytest.raises(RuntimeError, match='Joint did not respond'):
        monitor.check(1.)


def test_arrival_clears_stall_history_before_a_new_motion():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    for now, desired, actual in ((0., .04, 0.), (.75, .04, .04),
                                 (1.5, .08, .04), (2., .08, .04)):
        monitor.observe((JOINT,), (desired,), (actual,), now)
        monitor.check(now)
    monitor.observe((JOINT,), (.08,), (.04,), 2.5)
    with pytest.raises(RuntimeError, match='Joint did not respond'):
        monitor.check(2.5)


@pytest.mark.parametrize('send_feedback', [False, True])
def test_missing_or_stale_feedback_faults(send_feedback):
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    if send_feedback:
        monitor.observe((JOINT,), (0.,), (0.,), 0.)
    monitor.check(1.)
    with pytest.raises(RuntimeError, match='feedback missing or stale'):
        monitor.check(1.001)


@pytest.mark.parametrize('names,desired,actual', [
    (('right_wrist_roll_joint',), (0.,), (0.,)),
    ((JOINT,), (), (0.,)),
    ((JOINT,), (0.,), (float('nan'),)),
    ((JOINT,), (float('inf'),), (0.,)),
])
def test_malformed_feedback_fault_is_latched(names, desired, actual):
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    monitor.observe(names, desired, actual, .1)
    monitor.observe((JOINT,), (0.,), (0.,), .2)
    with pytest.raises(RuntimeError, match='Invalid physical trajectory feedback'):
        monitor.check(.2)


def test_replayed_feedback_does_not_renew_the_watchdog():
    monitor = MotionMonitor((JOINT,), CONFIG, 0.)
    monitor.observe((JOINT,), (0.,), (0.,), 1.)
    monitor.observe((JOINT,), (0.,), (0.,), 1.)
    monitor.observe((JOINT,), (0.,), (0.,), .5)
    assert monitor.report()['feedback_samples'] == 1
    with pytest.raises(RuntimeError, match='feedback missing or stale'):
        monitor.check(2.001)


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_limits_are_rejected(value):
    with pytest.raises(ValueError, match='Invalid motion-monitor limits'):
        replace(CONFIG, feedback_timeout_sec=value)


def test_hardware_monitor_propagates_bad_feedback_and_removes_finished_watchdogs(monkeypatch):
    from cleany_skill_executor import hardware_execution as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    state = NS(name=['head_pan_joint', 'head_tilt_joint'], position=[0., 0.])
    execution = NS(_motion_config=CONFIG, _motion_monitors=set(), fault='',
                   guard={'state': 'ready'}, guard_time=10., uncommanded=None,
                   node=NS(measured=lambda: state), value=lambda _: .5)
    monitor = HardwareExecution._start_motion_monitor(execution, (JOINT,))
    HardwareExecution._motion_feedback(monitor, NS(feedback=NS(
        joint_names=[JOINT], desired=NS(positions=[0.]), actual=NS(positions=[0.]))))
    assert HardwareExecution.monitor(execution) is state
    HardwareExecution._motion_feedback(monitor, NS(feedback=NS()))
    with pytest.raises(RuntimeError, match='Invalid physical trajectory feedback'):
        HardwareExecution.monitor(execution)
    report = HardwareExecution._finish_motion_monitor(execution, monitor)
    assert report['feedback_samples'] == 1
    assert report['fault']
    assert not execution._motion_monitors
    assert HardwareExecution.monitor(execution) is state
