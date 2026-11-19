"""A persisted link failure can clear only with fresh, disarmed startup evidence."""
import json
from copy import deepcopy

import pytest

from cleany_system_supervisor.config import validate
from cleany_system_supervisor.faults import BASE_FEEDBACK_LOST
from cleany_system_supervisor.models import Availability as A
from cleany_system_supervisor.store import LatchStore
from cleany_system_supervisor.supervisor import SystemSupervisor
from test_core import Clock, Recovery, Stop, advance, permission, profile, ready


def reconnect_profile():
    config = profile()
    config['startup_base_reconnect'] = {'enabled': True, 'stable_s': 1.0}
    config['safety_sources'] = list(dict.fromkeys([*config['safety_sources'], 'lidar', 'publisher_trust']))
    config['sources']['base_disarmed'] = dict(
        config['sources']['stopped'], fault_code='BASE_DISARM_UNVERIFIED', monitor_only=True)
    return config


def restored(tmp_path, *, cause=BASE_FEEDBACK_LOST, leases=(), enabled=True):
    config = reconnect_profile()
    config['startup_base_reconnect']['enabled'] = enabled
    store = LatchStore(tmp_path / 'state.json')
    store.save({'faults': [dict(code='BASE_UNAVAILABLE', source='base', severity='CRITICAL',
                               capabilities=['BASE_MOTION'], recovery_policy='', locks_operation=True,
                               detail='MCU feedback lost', **({'cause': cause} if cause is not None else {}))],
                'leases': list(leases)})
    return SystemSupervisor(config, clock=Clock(), stop_port=Stop(), recovery_port=Recovery(), store=store)


def active(system):
    return system.faults.faults['BASE_UNAVAILABLE'].active


def test_restored_link_fault_clears_once_after_stable_disarmed_startup(tmp_path):
    system = restored(tmp_path)
    for _ in range(8):
        advance(system)
    assert active(system)  # Valid samples alone cannot skip the full stability interval.
    for _ in range(8):
        advance(system)
    assert not active(system)
    assert sum(e.kind == 'BASE_STARTUP_RECONCILED' for e in system.events) == 1
    assert json.loads(system.store.path.read_text())['faults'] == []
    assert system.permissions.leases == {}  # No automatic goal/lease resumption.
    assert permission(system, mission_id='new-user-goal').granted


@pytest.mark.parametrize('missing', ['base', 'base_disarmed', 'lidar', 'safety', 'publisher_trust'])
def test_stale_or_absent_evidence_cannot_reconcile(tmp_path, missing):
    system = restored(tmp_path)
    for _ in range(20):
        advance(system, missing=(missing,))
    assert active(system)


def test_moving_base_breaks_stability_window(tmp_path):
    system = restored(tmp_path)
    for _ in range(8):
        advance(system)
    advance(system, bad=('base_disarmed',))
    for _ in range(8):
        advance(system)
    assert active(system)
    for _ in range(8):
        advance(system)
    assert not active(system)


@pytest.mark.parametrize('cause', [None, '', 'base_hardware_or_protocol_fault', 'base_unverified'])
def test_legacy_or_non_transport_fault_still_requires_operator(tmp_path, cause):
    system = restored(tmp_path, cause=cause)
    for _ in range(20):
        advance(system)
    assert active(system)


def test_default_policy_keeps_manual_reset(tmp_path):
    system = restored(tmp_path, enabled=False)
    for _ in range(20):
        advance(system)
    assert active(system)


def test_hardware_fault_during_startup_permanently_disqualifies_link_recovery(tmp_path):
    system = restored(tmp_path)
    system.clock.now += .1
    system.observe('base', A.UNAVAILABLE, 'MCU fault bits: 0x2', cause='base_hardware_or_protocol_fault')
    for _ in range(20):
        advance(system)
    assert active(system)
    assert system.faults.faults['BASE_UNAVAILABLE'].cause == 'base_hardware_or_protocol_fault'


def test_pending_mission_stop_blocks_until_confirmed_and_does_not_resume(tmp_path):
    system = restored(tmp_path, leases=[{'mission_id': 'old-goal', 'lease_id': 'old-lease'}])
    for _ in range(20):
        advance(system)
    assert active(system)
    assert system.stop_port.requests == ['old-lease']
    system.stop_port.status = 'STOPPED'
    advance(system)
    assert not active(system)
    assert not system.permissions.leases['old-lease'].granted


def test_new_runtime_link_failure_is_not_a_startup_recovery(tmp_path):
    system = SystemSupervisor(reconnect_profile(), clock=Clock(), stop_port=Stop(),
                              store=LatchStore(tmp_path / 'state.json'))
    ready(system)
    for _ in range(6):
        system.clock.now += .1
        system.observe('base', A.UNAVAILABLE, 'MCU feedback lost', cause=BASE_FEEDBACK_LOST)
        system.tick()
    assert active(system)
    for _ in range(20):
        advance(system)
    assert active(system) and not system.faults.faults['BASE_UNAVAILABLE'].restored
    saved = {f['code']: f for f in json.loads(system.store.path.read_text())['faults']}
    assert saved['BASE_UNAVAILABLE']['cause'] == BASE_FEEDBACK_LOST


def test_hardware_fault_cannot_be_reclassified_by_later_link_loss(tmp_path):
    system = restored(tmp_path, cause='base_hardware_or_protocol_fault')
    for _ in range(6):
        system.clock.now += .1
        system.observe('base', A.UNAVAILABLE, 'MCU feedback lost', cause=BASE_FEEDBACK_LOST)
        system.tick()
    for _ in range(20):
        advance(system)
    assert active(system)


def test_policy_rejects_missing_disarmed_monitor_and_invalid_timing():
    config = reconnect_profile()
    validate(config)
    for value in (0, -1, float('nan'), True):
        invalid = deepcopy(config)
        invalid['startup_base_reconnect']['stable_s'] = value
        with pytest.raises(ValueError):
            validate(invalid)
    del config['sources']['base_disarmed']
    with pytest.raises(ValueError, match='base_disarmed'):
        validate(config)
