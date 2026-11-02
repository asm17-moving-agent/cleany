from copy import deepcopy
from pathlib import Path

import pytest

from cleany_system_supervisor.config import load, validate
from cleany_system_supervisor.models import Availability as A, OperatingState as O, RecoveryState as R
from cleany_system_supervisor.recovery import UnsupportedRecovery
from cleany_system_supervisor.store import LatchStore
from cleany_system_supervisor.supervisor import SystemSupervisor

OPERATOR = 'synthetic_operator'
# Fixed test timings, independent of the robot profile's recovery deadlines.
RECOVERY = dict(automatic=False, max_attempts=2, attempt_timeout_s=5., total_timeout_s=20.,
                verify_timeout_s=3., cancel_timeout_s=2.)


class Clock:
    now = 100.

    def __call__(self):
        return self.now


class Recovery:
    def __init__(self):
        self.started, self.canceled, self.statuses = [], [], {}

    def start(self, job):
        self.started.append((job.job_id, job.attempt))
        self.statuses[job.job_id] = 'RUNNING'

    def poll(self, job):
        return self.statuses[job.job_id]

    def cancel(self, job):
        self.canceled.append(job.job_id)


class Stop:
    def __init__(self):
        self.requests, self.status, self.forgotten = [], 'ACCEPTED', []

    def request(self, lease, epoch):
        self.requests.append(lease.lease_id)

    def poll(self, lease_id):
        return self.status

    def forget(self, lease_id):
        self.forgotten.append(lease_id)


def profile():
    config = load(Path(__file__).resolve().parents[1] / 'config/system_supervisor.yaml')
    for policy in config['recovery_policies'].values():
        policy.update(RECOVERY)
    return config


@pytest.fixture
def system():
    return SystemSupervisor(profile(), clock=Clock(), recovery_port=Recovery(), stop_port=Stop())


def advance(system, seconds=.1, bad=(), missing=()):
    system.clock.now += seconds
    for source in system.config['sources']:
        if source not in missing:
            system.observe(source, A.UNAVAILABLE if source in bad else A.AVAILABLE)
    system.tick()


def ready(system):
    for _ in range(15):
        advance(system)
    assert system.state == O.OPERATIONAL


def permission(system, caps=('NAVIGATION',), **kwargs):
    values = dict(epoch=system.epoch, mission_id='synthetic_mission', capabilities=list(caps),
                  lease_id='', version=0, release=False)
    values.update(kwargs)
    return system.check_permission(**values)


def test_boot_unknown_then_operational_and_optional_degradation(system):
    assert system.state == O.INITIALIZING
    assert set(system.capabilities.states.values()) == {A.UNKNOWN}
    assert not permission(system).granted
    ready(system)
    advance(system, bad=('camera',))
    assert system.state == O.DEGRADED
    assert system.capabilities.states['VISUAL_PERCEPTION'] == A.UNAVAILABLE
    assert system.capabilities.states['MOBILE_MANIPULATION'] == A.UNAVAILABLE
    assert permission(system).granted
    assert not permission(system, ('VISUAL_PERCEPTION',), mission_id='other').granted


def test_stale_cannot_remain_available_and_missing_safety_locks(system):
    ready(system)
    advance(system, .6, missing=('navigation',))
    assert system.capabilities.states['NAVIGATION'] != A.AVAILABLE
    advance(system, .4, missing=('safety',))
    assert system.state == O.FAULT_LOCKED
    assert not permission(system, ('MANIPULATION',)).granted


def test_fault_hysteresis_multiple_faults_and_conjunctive_reset(system):
    ready(system)
    advance(system, bad=('navigation', 'camera'))
    advance(system, .3, bad=('navigation', 'camera'))
    assert {f.code for f in system.faults.faults.values() if f.active} == {
        'NAVIGATION_UNAVAILABLE', 'CAMERA_UNAVAILABLE'}
    advance(system)
    assert system.faults.faults['NAVIGATION_UNAVAILABLE'].active
    for _ in range(7):
        advance(system)
    assert not system.faults.faults['NAVIGATION_UNAVAILABLE'].active
    for _ in range(5):
        advance(system, bad=('estop', 'base'))
    assert system.state == O.FAULT_LOCKED
    assert not system.reset_fault(system.epoch, 'ESTOP', OPERATOR)[0]
    for _ in range(8):
        advance(system)
    assert system.state == O.FAULT_LOCKED
    assert not system.reset_fault(system.epoch, 'ESTOP')[0]  # Operator identity is mandatory.
    assert system.reset_fault(system.epoch, 'ESTOP', OPERATOR)[0]
    assert system.state == O.FAULT_LOCKED
    assert system.reset_fault(system.epoch, 'BASE_UNAVAILABLE', OPERATOR)[0]
    assert system.state == O.OPERATIONAL


def test_revocation_acceptance_is_not_stopped_and_timeout_escalates(system):
    ready(system)
    lease = permission(system)
    assert lease.granted
    advance(system, bad=('localization',))
    record = system.permissions.leases[lease.lease_id]
    assert not record.granted and record.stop_accepted and not record.stopped
    assert not permission(system, lease_id=lease.lease_id, version=lease.version).granted
    for _ in range(int(system.config['stop_timeout_s'] * 10) + 2):
        advance(system, bad=('localization',))
    assert system.state == O.FAULT_LOCKED
    assert system.faults.faults['MISSION_STOP_FAILED:' + lease.lease_id].active
    system.stop_port.status = 'STOPPED'
    advance(system, bad=('localization',))
    assert record.stopped
    assert system.state == O.DEGRADED


def test_expiry_epoch_and_identity_cannot_reauthorize(system):
    ready(system)
    lease = permission(system)
    assert not permission(system, lease_id=lease.lease_id, version=lease.version + 1).granted
    assert not permission(system, ('MANIPULATION',), lease_id=lease.lease_id, version=lease.version).granted
    for _ in range(16):
        advance(system)
    assert not permission(system, lease_id=lease.lease_id, version=lease.version).granted
    restarted = SystemSupervisor(deepcopy(system.config), clock=system.clock)
    assert restarted.epoch != system.epoch and restarted.state == O.INITIALIZING
    assert not permission(restarted, epoch=system.epoch, lease_id=lease.lease_id, version=lease.version).granted
    assert not permission(system).granted  # Previous stop is unconfirmed.


def make_recovery(system, source='navigation'):
    advance(system, bad=(source,))
    advance(system, .3, bad=(source,))
    code = system.config['sources'][source]['fault_code']
    job = system.request_recovery(system.epoch, code)
    system.tick()
    assert job.state == R.RUNNING
    return job


def test_recovery_result_requires_new_stable_health_and_keeps_permission_blocked(system):
    ready(system)
    job = make_recovery(system)
    system.recoveries.port.statuses[job.job_id] = 'SUCCEEDED'
    advance(system, bad=('navigation',))
    assert job.state == R.RUNNING and job.verify_after is not None
    assert not permission(system).granted
    for _ in range(4):
        advance(system)
    assert job.state == R.RUNNING
    assert not permission(system).granted
    for _ in range(4):
        advance(system)
    assert job.state == R.SUCCEEDED
    advance(system)
    assert permission(system).granted


def test_failed_recovery_exhausts_budget_without_clearing_fault(system):
    ready(system)
    job = make_recovery(system)
    for _ in range(2):
        system.recoveries.port.statuses[job.job_id] = 'FAILED'
        advance(system, bad=('navigation',))
    assert job.state == R.FAILED and job.attempt == 2
    assert system.request_recovery(system.epoch, job.fault_code) is job
    assert system.faults.faults[job.fault_code].active
    assert not permission(system).granted


def test_timeout_quarantines_until_terminal_cancel_and_serializes_jobs(system):
    ready(system)
    job = make_recovery(system)
    advance(system, bad=('navigation', 'localization'))
    advance(system, .3, bad=('navigation', 'localization'))
    other = system.request_recovery(system.epoch, 'LOCALIZATION_UNAVAILABLE')
    for _ in range(52):
        advance(system, bad=('navigation', 'localization'))
    assert system.recoveries.port.canceled == [job.job_id]
    assert other.state == R.IDLE
    for _ in range(21):
        advance(system, bad=('navigation', 'localization'))
    assert job.state == R.FAILED and job.resource_held
    assert other.state == R.IDLE and not permission(system).granted
    system.recoveries.port.statuses[job.job_id] = 'CANCELED'
    advance(system, bad=('navigation', 'localization'))
    assert len(system.recoveries.port.started) == 2


def test_safety_fault_cancels_running_recovery_without_releasing_resource(system):
    ready(system)
    job = make_recovery(system)
    advance(system, bad=('navigation', 'estop'))
    assert job.cancel_requested and job.resource_held
    system.recoveries.port.statuses[job.job_id] = 'CANCELED'
    advance(system, bad=('navigation', 'estop'))
    assert job.state == R.CANCELED and not job.resource_held


def test_automatic_recovery_needs_confirmed_mission_stop(system):
    ready(system)
    lease = permission(system)
    system.config['recovery_policies']['localization']['automatic'] = True
    advance(system, bad=('localization',))
    advance(system, .3, bad=('localization',))
    job = next(iter(system.recoveries.jobs.values()))
    assert job.state == R.IDLE
    system.stop_port.status = 'STOPPED'
    advance(system, bad=('localization',))
    assert job.state == R.RUNNING
    assert system.permissions.leases[lease.lease_id].stopped


def test_missing_component_action_fails_honestly(system):
    from cleany_system_supervisor.supervisor import UnavailablePort
    ready(system)
    system.recoveries.port = UnavailablePort()
    job = make_recovery_unavailable(system)
    assert job.state == R.FAILED
    assert system.faults.faults[job.fault_code].active


def make_recovery_unavailable(system):
    advance(system, bad=('navigation',))
    advance(system, .3, bad=('navigation',))
    job = system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE')
    system.tick()
    system.tick()
    return job


def test_bad_config_is_rejected(system):
    config = deepcopy(system.config)
    config['capabilities']['BASE_MOTION']['depends_on'] = ['MOBILE_MANIPULATION']
    with pytest.raises(ValueError, match='cycle'):
        validate(config)
    config = deepcopy(system.config)
    config['lease_s'] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        validate(config)


def test_no_health_never_passes_startup_and_latched_unknown_does_not_reset(system):
    system.clock.now += 6.
    system.tick()
    assert system.state == O.FAULT_LOCKED
    assert not system.reset_fault(system.epoch, 'ESTOP', OPERATOR)[0]
    assert not permission(system).granted


def test_recovery_success_without_health_times_out_and_exhausts_budget(system):
    ready(system)
    job = make_recovery(system)
    for _ in range(2):
        system.recoveries.port.statuses[job.job_id] = 'SUCCEEDED'
        advance(system, bad=('navigation',))
        for _ in range(32):
            advance(system, bad=('navigation',))
    assert job.state == R.FAILED and job.attempt == 2
    assert not permission(system).granted


def test_recovery_requires_independent_stop_evidence_but_owner_may_move_during_recovery(system):
    ready(system)
    advance(system, bad=('navigation', 'stopped'))
    advance(system, .3, bad=('navigation', 'stopped'))
    job = system.request_recovery(system.epoch, 'NAVIGATION_UNAVAILABLE')
    system.tick()
    assert job.state == R.IDLE
    for _ in range(8):
        advance(system, bad=('navigation',))
    assert job.state == R.RUNNING
    advance(system, bad=('navigation', 'stopped'))
    assert job.state == R.RUNNING and not job.cancel_requested
    assert 'STOP_UNVERIFIED' not in system.faults.faults


def test_reset_does_not_invent_mission_stop_proof(system):
    ready(system)
    lease = permission(system)
    system.stop_port.status = 'FAILED'
    advance(system, bad=('localization',))
    code = 'MISSION_STOP_FAILED:' + lease.lease_id
    assert not system.reset_fault(system.epoch, code, OPERATOR)[0]
    assert system.state == O.FAULT_LOCKED
    system.stop_port.status = 'STOPPED'
    advance(system, bad=('localization',))
    assert not system.faults.faults[code].active


def test_operation_lock_is_independent_of_recovery_job_state(system):
    ready(system)
    system.config['sources']['base']['recovery'] = 'navigation'
    for _ in range(5):
        advance(system, bad=('base',))
    job = system.request_recovery(system.epoch, 'BASE_UNAVAILABLE')
    system.tick()
    assert system.state == O.FAULT_LOCKED
    assert job.state == R.RUNNING  # Fresh independent safety permits component recovery while locked.


def test_tof_requirement_revokes_motion_without_turning_obstacles_into_faults():
    from cleany_system_supervisor.config import with_tof
    config = with_tof(profile())
    system = SystemSupervisor(config, clock=Clock(), recovery_port=Recovery(), stop_port=Stop())
    ready(system)
    lease = permission(system)
    assert lease.granted
    # Obstacle limiting leaves health AVAILABLE. Only protection availability is reported.
    advance(system, bad=('tof_proximity',))
    advance(system, bad=('tof_proximity',))
    assert system.capabilities.states['BASE_MOTION'] == A.UNAVAILABLE
    assert not system.permissions.leases[lease.lease_id].granted
    assert system.stop_port.requests
