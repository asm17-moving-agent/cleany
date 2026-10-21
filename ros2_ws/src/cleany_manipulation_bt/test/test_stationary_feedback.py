"""A stop needs fresh velocity or measured position changes and terminal Actions."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from cleany_manipulation_bt.action_tracking import quiescent
from cleany_manipulation_bt.joint_feedback import JointFeedback, StationaryWindow


@pytest.mark.parametrize('fault,reason', [
    ('repeated', 'repeated_stamp'), ('backward', 'backward_stamp'),
    ('missing_velocity', 'missing_velocity'), ('nan', 'non_finite_velocity'),
])
def test_feedback_validation_failure_is_distinct_from_receipt_delay(fault, reason):
    feedback = JointFeedback(('joint',))
    feedback.update(('joint',), (0.,), 10**9, 10.)
    stamp = 10**9 if fault == 'repeated' else 10**9 - 1 if fault == 'backward' else 1_100_000_000
    velocities = () if fault == 'missing_velocity' else (float('nan'),) if fault == 'nan' else (.1,)
    feedback.update(('joint',), velocities, stamp, 10.1)
    assert feedback.invalid_reasons(10.101, 1_100_000_000, .5) == {'joint': reason}
    assert feedback.invalid_joints(10.101, 1_100_000_000, .5) == ('joint',)
    feedback.update(('joint',), (0.,), 1_200_000_000, 10.2)
    assert not feedback.invalid_reasons(10.201, 1_200_000_000, .5)


def test_feedback_diagnostics_preserve_expiry_and_missing_joint_checks():
    feedback = JointFeedback(('a', 'b'))
    feedback.update(('a',), (0.,), 10**9, 10.)
    assert feedback.invalid_reasons(10.6, 10**9, .5) == {'a': 'stale_receipt', 'b': 'missing_joint'}
    assert feedback.invalid_reasons(10.1, 2 * 10**9, .5) == {'a': 'clock_skew', 'b': 'missing_joint'}
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Error, Goal


def window():
    return StationaryWindow(barrier_ns=10_000_000_000, started_at=10.0,
        max_age_sec=.2, maximum_velocity=.02, required_samples=3, minimum_duration_sec=.1)


def update(feedback, now, names=('left', 'right'), velocities=(0., 0.)):
    feedback.update(names, velocities, int(now * 1e9), now)


def test_stop_accepts_separate_arm_messages_only_after_full_fresh_rounds():
    feedback, stopped = JointFeedback(('left', 'right')), window()
    for n in range(5):
        now = 10.01 + n * .05
        update(feedback, now, ('left',), (0.,))
        left_result = stopped.observe(feedback, now=now, clock_ns=int(now*1e9), terminal=True)
        if n < 3:
            assert not left_result
        update(feedback, now, ('right',), (0.,))
        result = stopped.observe(feedback, now=now, clock_ns=int(now*1e9), terminal=True) or left_result
    assert result


@pytest.mark.parametrize('velocities,terminal', [((.1,0.),True), ((),True), ((0.,0.),False)])
def test_movement_missing_velocities_or_active_action_cannot_confirm(velocities, terminal):
    feedback, stopped = JointFeedback(('left', 'right')), window()
    for n in range(10):
        now = 10.01 + n*.05
        update(feedback, now, velocities=velocities)
        assert not stopped.observe(feedback, now=now, clock_ns=int(now*1e9), terminal=terminal)


def test_replayed_feedback_cannot_refresh_stop_evidence():
    feedback, stopped = JointFeedback(('left', 'right')), window()
    update(feedback, 10.01)
    for n in range(10):
        feedback.update(('left', 'right'), (0.,0.), 10_010_000_000, 10.01+n*.05)
        assert not stopped.observe(feedback, now=10.01+n*.05, clock_ns=int((10.01+n*.05)*1e9), terminal=True)


def test_future_stamped_feedback_cannot_prove_physical_stop():
    feedback, stopped = JointFeedback(('left', 'right')), window()
    for n in range(10):
        now = 10.01 + n * .05
        feedback.update(('left', 'right'), (0., 0.), int((now + .05) * 1e9), now)
        assert not stopped.observe(feedback, now=now, clock_ns=int(now * 1e9), terminal=True)


@pytest.mark.parametrize('velocities', [(), (float('nan'), float('nan'))])
def test_position_only_feedback_proves_stop_after_two_post_barrier_samples(velocities):
    feedback, stopped = JointFeedback(('left', 'right')), window()
    feedback.update(('left', 'right'), velocities, 9_990_000_000, 9.99, positions=(.4, -.2))
    confirmed = False
    for n in range(7):
        now = 10.01 + n * .05
        feedback.update(('left', 'right'), velocities, int(now * 1e9), now,
                        positions=(.4, -.2), maximum_position_interval_sec=.2)
        confirmed = stopped.observe(feedback, now=now, clock_ns=int(now * 1e9), terminal=True)
        if n < 3:
            assert not confirmed
    assert confirmed


@pytest.mark.parametrize('issue', ['moving', 'nan', 'missing', 'gap'])
def test_bad_position_feedback_cannot_replace_missing_velocity(issue):
    feedback, stopped = JointFeedback(('left', 'right')), window()
    for n in range(10):
        now = 10.01 + n * (.3 if issue == 'gap' else .05)
        positions = (.1 * n if issue == 'moving' else .4, -.2)
        if issue == 'nan':
            positions = (float('nan'), -.2)
        elif issue == 'missing':
            positions = ()
        feedback.update(('left', 'right'), (), int(now * 1e9), now,
                        positions=positions, maximum_position_interval_sec=.2)
        assert not stopped.observe(feedback, now=now, clock_ns=int(now * 1e9), terminal=True)


def test_late_acknowledgement_and_terminal_result_are_both_required():
    submitted, result = Future(), Future()
    handle = NS(accepted=True)
    assert not quiescent([submitted], [])
    submitted.set_result(handle)
    assert not quiescent([submitted], [])
    assert not quiescent([submitted], [(handle,result)])
    result.set_result(NS(status=4))
    assert quiescent([submitted], [(handle,result)])
