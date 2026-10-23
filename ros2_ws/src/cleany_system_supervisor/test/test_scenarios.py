"""Core scenarios: boot delays, transient/persistent loss, restart persistence, retention."""
import json

import pytest

from cleany_system_supervisor.faults import STATE_FAULT
from cleany_system_supervisor.models import Availability as A, OperatingState as O, RecoveryState as R
from cleany_system_supervisor.recovery import UnsupportedRecovery
from cleany_system_supervisor.store import LatchStore
from cleany_system_supervisor.supervisor import SystemSupervisor
from test_core import OPERATOR, Clock, Recovery, Stop, advance, permission, profile, ready


def make(store=None, clock=None, config=None):
    return SystemSupervisor(config or profile(), clock=clock or Clock(), recovery_port=Recovery(),
                            stop_port=Stop(), store=store)


def kinds(system):
    return [event.kind for event in system.events]


def test_boot_waits_for_first_mcu_feedback_without_latching():
    system = make()
    # Base driver reports "waiting for first feedback" (UNKNOWN) for 12 s of a 15 s grace.
    for _ in range(120):
        advance(system, bad=(), missing=('base',))
        system.observe('base', A.UNKNOWN, 'waiting for first MCU feedback')
    assert 'BASE_UNAVAILABLE' not in system.faults.faults
    assert system.state == O.FAULT_LOCKED  # Essential BASE_MOTION is unverified after 5 s.
    assert not permission(system).granted
    for _ in range(10):
        advance(system)
    assert system.state == O.OPERATIONAL and 'BASE_UNAVAILABLE' not in system.faults.faults


def test_mcu_that_never_connects_latches_after_grace():
    system = make()
    for _ in range(151):
        advance(system, missing=('base',))
        system.observe('base', A.UNKNOWN, 'waiting for first MCU feedback')
    fault = system.faults.faults['BASE_UNAVAILABLE']
    assert fault.active and fault.latched


def test_explicit_base_fault_during_boot_is_not_delayed():
    system = make()
    for _ in range(5):
        advance(system, bad=('base',))
    assert system.faults.faults['BASE_UNAVAILABLE'].active


def test_transient_feedback_gap_blocks_motion_but_does_not_latch():
    system = make()
    ready(system)
    lease = permission(system)
    assert lease.granted
    advance(system, bad=('base',))
    advance(system, bad=('base',))
    # BASE_MOTION is withdrawn immediately and the lease revoked, independent of raise_s.
    assert system.capabilities.states['BASE_MOTION'] == A.UNAVAILABLE
    assert not system.permissions.leases[lease.lease_id].granted
    for _ in range(8):
        advance(system)
    assert 'BASE_UNAVAILABLE' not in system.faults.faults
    system.stop_port.status = 'STOPPED'
    advance(system)
    assert system.state == O.OPERATIONAL and permission(system, mission_id='next').granted


def test_persistent_feedback_loss_latches_until_operator_reset():
    system = make()
    ready(system)
    for _ in range(5):
        advance(system, bad=('base',))
    assert system.faults.faults['BASE_UNAVAILABLE'].active
    for _ in range(10):
        advance(system)
    assert system.state == O.FAULT_LOCKED
    assert system.reset_fault(system.epoch, 'BASE_UNAVAILABLE', OPERATOR)[0]
    assert system.state == O.OPERATIONAL
    reset = [e for e in system.events if e.kind == 'FAULT_RESET_ACCEPTED']
    assert reset and reset[-1].source == OPERATOR


def test_restart_preserves_latched_faults_and_unconfirmed_leases(tmp_path):
    store, clock = LatchStore(tmp_path / 'state.json'), Clock()
    first = make(store, clock)
    ready(first)
    lease = permission(first)
    for _ in range(5):
        advance(first, bad=('estop',))
    assert first.faults.faults['ESTOP'].active
    saved = json.loads((tmp_path / 'state.json').read_text())
    assert [f['code'] for f in saved['faults']] == ['ESTOP']
    assert [entry['lease_id'] for entry in saved['leases']] == [lease.lease_id]

    restarted = make(store, clock)
    assert restarted.epoch != first.epoch
    assert restarted.faults.faults['ESTOP'].active and restarted.faults.faults['ESTOP'].restored
    assert {'FAULT_RESTORED', 'LEASE_RESTORED'} <= set(kinds(restarted))
    for _ in range(15):
        advance(restarted)
    # Healthy evidence alone does not clear a restored latch, and the old lease is re-stopped.
    assert restarted.state == O.FAULT_LOCKED
    assert restarted.stop_port.requests == [lease.lease_id]
    assert not permission(restarted, mission_id='synthetic_mission').granted
    assert restarted.reset_fault(restarted.epoch, 'ESTOP', OPERATOR)[0]
    restarted.stop_port.status = 'STOPPED'
    advance(restarted)
    assert restarted.state == O.OPERATIONAL
    assert permission(restarted, mission_id='synthetic_mission').granted
    assert json.loads((tmp_path / 'state.json').read_text())['faults'] == []


def test_restart_with_unconfirmable_old_stop_stays_locked(tmp_path):
    store, clock = LatchStore(tmp_path / 'state.json'), Clock()
    store.save({'faults': [], 'leases': [{'mission_id': 'm', 'lease_id': 'old'}]})
    system = make(store, clock)
    system.stop_port.status = 'FAILED'
    for _ in range(15):
        advance(system)
    assert system.faults.faults['MISSION_STOP_FAILED:old'].active
    assert system.state == O.FAULT_LOCKED


def test_unreadable_state_file_fails_closed_until_operator_acknowledges(tmp_path):
    path = tmp_path / 'state.json'
    path.write_text('{not json')
    system = make(LatchStore(path))
    for _ in range(15):
        advance(system)
    assert system.faults.faults[STATE_FAULT].active and system.state == O.FAULT_LOCKED
    assert not system.reset_fault(system.epoch, STATE_FAULT)[0]
    assert system.reset_fault(system.epoch, STATE_FAULT, OPERATOR)[0]
    assert system.state == O.OPERATIONAL


def test_persistence_failure_locks_operation(tmp_path):
    class BrokenStore(LatchStore):
        def save(self, state):
            raise OSError('synthetic read-only filesystem')
    system = make(BrokenStore(tmp_path / 'state.json'))
    for _ in range(15):
        advance(system)
    assert system.faults.faults[STATE_FAULT].active and system.state == O.FAULT_LOCKED
    assert kinds(system).count('STATE_PERSIST_FAILED') == 1


def test_retention_prunes_finished_records_but_keeps_recovery_budget():
    config = profile()
    config['retention_s'] = 1.
    system = make(config=config)
    ready(system)
    lease = permission(system)
    assert not permission(system, lease_id=lease.lease_id, version=lease.version, release=True).granted
    advance(system, bad=('navigation',))
    advance(system, .3, bad=('navigation',))
    job = system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE')
    for _ in range(2):
        system.tick()
        system.recoveries.port.statuses[job.job_id] = 'FAILED'
        advance(system, bad=('navigation',))
    assert job.state == R.FAILED
    for _ in range(20):
        advance(system, bad=('navigation',))
    assert lease.lease_id not in system.permissions.leases
    assert system.stop_port.forgotten == [lease.lease_id]
    # The fault episode is still active: the exhausted job must survive to block retries.
    assert system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE') is job
    for _ in range(30):
        advance(system)
    assert job.job_id not in system.recoveries.jobs
    assert 'NAVIGATION_UNAVAILABLE' not in system.faults.faults


def test_duplicate_recovery_request_does_not_start_a_second_operation():
    system = make()
    ready(system)
    advance(system, bad=('navigation',))
    advance(system, .3, bad=('navigation',))
    job = system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE')
    system.tick()
    assert system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE') is job
    advance(system, bad=('navigation',))
    assert system.recoveries.port.started == [(job.job_id, 1)]


def test_unsupported_recovery_is_explicit_and_consumes_no_budget():
    system = make()
    ready(system)
    advance(system, bad=('lidar',))
    advance(system, .3, bad=('lidar',))
    with pytest.raises(UnsupportedRecovery, match='operator action required'):
        system.request_recovery(system.epoch, 'LIDAR_UNAVAILABLE')
    assert not system.recoveries.jobs and 'RECOVERY_UNSUPPORTED' in kinds(system)


def test_events_cover_state_fault_lease_and_recovery_transitions():
    system = make()
    ready(system)
    lease = permission(system)
    advance(system, bad=('navigation',))
    advance(system, .3, bad=('navigation',))
    job = system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE')
    system.tick()
    system.stop_port.status = 'STOPPED'
    advance(system, bad=('navigation',))
    observed = kinds(system)
    for kind in ('STATE_CHANGED', 'LEASE_GRANTED', 'LEASE_REVOKED', 'MISSION_STOP_ACCEPTED', 'FAULT_RAISED',
                 'RECOVERY_REQUESTED', 'RECOVERY_RUNNING', 'MISSION_STOPPED'):
        assert kind in observed, kind
    sequences = [event.sequence for event in system.events]
    assert sequences == sorted(set(sequences))
    raised = next(e for e in system.events if e.kind == 'FAULT_RAISED')
    assert raised.code == 'NAVIGATION_UNAVAILABLE' and raised.severity == 'ERROR'
    assert any(e.code == lease.lease_id for e in system.events if e.kind == 'LEASE_REVOKED')
    assert job.job_id in next(e.detail for e in system.events if e.kind == 'RECOVERY_RUNNING')


def test_stop_request_retries_until_the_stop_server_is_reachable():
    class LateServer(Stop):
        reachable = False

        def request(self, lease, epoch):
            if not self.reachable:
                raise RuntimeError('mission/stop action unavailable')
            super().request(lease, epoch)
    system = make()
    system.stop_port = LateServer()
    ready(system)
    lease = permission(system)
    advance(system, bad=('localization',))
    advance(system, bad=('localization',))
    assert not any(code.startswith('MISSION_STOP_FAILED') for code in system.faults.faults)
    system.stop_port.reachable, system.stop_port.status = True, 'STOPPED'
    advance(system, bad=('localization',))
    assert system.permissions.leases[lease.lease_id].stopped
    assert system.stop_port.requests == [lease.lease_id]


def test_unreachable_stop_server_escalates_at_the_stop_deadline():
    from cleany_system_supervisor.supervisor import UnavailablePort
    system = make()
    system.stop_port = UnavailablePort()
    ready(system)
    lease = permission(system)
    for _ in range(int(system.config['stop_timeout_s'] * 10) + 2):
        advance(system, bad=('localization',))
    assert system.faults.faults['MISSION_STOP_FAILED:' + lease.lease_id].active


def test_grant_is_persisted_before_the_reply(tmp_path):
    store = LatchStore(tmp_path / 'state.json')
    system = make(store)
    ready(system)
    lease = permission(system)
    assert lease.granted
    saved = json.loads((tmp_path / 'state.json').read_text())
    assert [entry['lease_id'] for entry in saved['leases']] == [lease.lease_id]


def test_grant_is_refused_when_it_cannot_be_persisted(tmp_path):
    class FailingAfterBoot(LatchStore):
        broken = False

        def save(self, state):
            if self.broken:
                raise OSError('synthetic disk full')
            super().save(state)
    store = FailingAfterBoot(tmp_path / 'state.json')
    system = make(store)
    ready(system)
    store.broken = True
    decision = permission(system)
    assert not decision.granted and decision.reason == 'lease could not be persisted'
    assert not any(lease.granted for lease in system.permissions.leases.values())
