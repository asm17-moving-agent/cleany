from dataclasses import replace

import pytest

from cleany_skill_executor.core.carry_guard import CarryGuard, ContactLossGuard, TrackingEvidence


def evidence(**changes):
    return replace(TrackingEvidence('ref', 'left', 'head', 1, 10_000_000_000,
        'left_wrist_rgb_optical_frame', True, True, 'visible'), **changes)


def bound():
    guard = CarryGuard()
    guard.bind('ref', 'left', 'head', 1)
    return guard


@pytest.mark.parametrize('changes', [dict(visible=False), dict(valid=False),
    dict(stamp_ns=0), dict(frame_id='right_wrist_rgb_optical_frame')])
def test_bad_evidence_latches_during_carry_until_new_target(changes):
    guard = bound()
    guard.update(evidence(), 20.)
    guard.arm(15_000_000_000, 20.)
    guard.update(evidence(**changes), 21.)
    guard.update(evidence(stamp_ns=16_000_000_000), 22.)
    with pytest.raises(RuntimeError, match='suspected'):
        guard.check(17_000_000_000, 22.)
    guard.bind('ref', 'left', 'head', 1)
    guard.update(evidence(), 23.)
    guard.arm(15_000_000_000, 23.)


def test_no_initial_evidence_fails_without_waiting():
    with pytest.raises(RuntimeError, match='No continuous'):
        bound().arm(15_000_000_000, 20.)


@pytest.mark.parametrize('changes', [dict(reference_id='old'), dict(arm='right'),
    dict(source_snapshot_id='other'), dict(source_object_id=2)])
def test_unrelated_targets_cannot_authorize_carry(changes):
    guard = bound()
    guard.update(evidence(**changes), 20.)
    with pytest.raises(RuntimeError, match='No continuous'):
        guard.arm(15_000_000_000, 20.)


@pytest.mark.parametrize('now_ns,now_sec,reason', [
    (23_000_000_000, 20., 'capture'), (9_000_000_000, 20., 'capture'),
    (15_000_000_000, 29., 'updates'), (15_000_000_000, 19., 'updates')])
def test_capture_and_wall_heartbeat_have_independent_limits(now_ns, now_sec, reason):
    guard = bound()
    guard.update(evidence(), 20.)
    with pytest.raises(RuntimeError, match=reason):
        guard.arm(now_ns, now_sec)


def test_duplicate_and_reordered_results_do_not_refresh_camera_heartbeat():
    guard = bound()
    guard.update(evidence(), 20.)
    guard.arm(15_000_000_000, 20.)
    guard.update(evidence(), 27.)
    guard.update(evidence(stamp_ns=9_000_000_000), 28.)
    with pytest.raises(RuntimeError, match='updates stopped'):
        guard.check(15_000_000_000, 29.)


def test_intentional_release_disarms_missing_target_monitor():
    guard = bound()
    guard.update(evidence(), 20.)
    guard.arm(15_000_000_000, 20.)
    guard.disarm()
    guard.update(evidence(visible=False), 21.)
    guard.check(100_000_000_000, 100.)
    assert guard.fault is None


@pytest.mark.parametrize('limit', [0., -1., float('nan'), float('inf')])
def test_invalid_limits_rejected(limit):
    with pytest.raises(ValueError):
        CarryGuard(maximum_capture_age_sec=limit)
    with pytest.raises(ValueError):
        CarryGuard(maximum_update_age_sec=limit)


def test_contact_loss_debounce_runs_without_waiting_and_resets_on_recovery():
    guard = ContactLossGuard(.3)
    guard.check(False, 10.)
    guard.check(False, 10.2)
    guard.check(True, 10.25)
    guard.check(False, 11.)
    guard.check(False, 11.2)
    with pytest.raises(RuntimeError, match='persistently'):
        guard.check(False, 11.31)
