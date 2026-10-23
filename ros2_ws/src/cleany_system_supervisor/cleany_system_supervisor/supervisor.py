from collections import deque
from time import monotonic
from uuid import uuid4

from .capabilities import CapabilityManager, operating_state
from .config import validate
from .faults import STATE_FAULT, FaultManager
from .health import HealthMonitoring
from .models import (Availability, HealthReport, OperatingState, PermissionDecision, RecoveryState,
                     SupervisorEvent)
from .permissions import MissionPermissionManager
from .ports import MissionStopTransport, RecoveryTransport
from .recovery import RecoveryCoordinator, UnsupportedRecovery


class UnavailablePort:
    """An absent transport is an explicit failure, never synthetic success."""
    def request(self, *args):
        raise RuntimeError('mission stop transport unavailable')

    def start(self, *args):
        raise RuntimeError('component recovery transport unavailable')

    def poll(self, *args):
        return 'FAILED'

    def cancel(self, *args):
        pass

    def forget(self, *args):
        pass


class SystemSupervisor:
    def __init__(self, config: dict, *, clock=monotonic,
                 recovery_port: RecoveryTransport | None = None,
                 stop_port: MissionStopTransport | None = None,
                 store=None):
        self.config, self.clock = validate(config), clock
        self.epoch, self.revision = uuid4().hex, 0
        self.started_at = clock()
        self.health = HealthMonitoring(config['sources'], self.started_at)
        self.faults = FaultManager(config, self.started_at)
        self.capabilities = CapabilityManager(config['capabilities'])
        self.permissions = MissionPermissionManager(self.epoch, config)
        self.recoveries = RecoveryCoordinator(config, recovery_port or UnavailablePort())
        self.stop_port = stop_port or UnavailablePort()
        self.state = OperatingState.INITIALIZING
        self.events: deque[SupervisorEvent] = deque(maxlen=1000)
        self._event_sequence = 0
        self.store, self._persisted, self._persist_failed = store, None, False
        if store is not None:
            self._restore()
        self._seen_state = self.state
        self._seen_faults = {code: f.active for code, f in self.faults.faults.items()}
        self._seen_leases, self._seen_jobs = {}, {}

    def observe(self, source: str, status: Availability, detail: str = '') -> None:
        self.health.observe(HealthReport(source, status, self.clock(), detail))

    def _emit(self, kind: str, *, code='', source='', severity='', detail='') -> None:
        self._event_sequence += 1
        self.events.append(SupervisorEvent(self._event_sequence, kind, self.clock(), code, source,
                                           severity, detail))

    def _restore(self) -> None:
        now = self.clock()
        try:
            data = self.store.load()
        except Exception as exc:
            # Unknown previous latches must not silently become "no latches".
            self.faults.raise_state_fault(now, f'persisted latch state unreadable: {exc}')
            self._emit('STATE_RESTORE_FAILED', code=STATE_FAULT, severity='CRITICAL', detail=str(exc))
            return
        if not data:
            return
        self.faults.restore(data['faults'], now)
        self.permissions.restore(data['leases'])
        for entry in data['faults']:
            self._emit('FAULT_RESTORED', code=entry['code'], source=entry['source'],
                       severity=entry['severity'], detail=entry['detail'])
        for entry in data['leases']:
            self._emit('LEASE_RESTORED', code=entry['lease_id'], source=entry['mission_id'],
                       detail='stop confirmation required before new authority for this mission')

    def _availability(self, now):
        self.capabilities.update(self.health, self.faults.faults, now)
        self.state = operating_state(self.config, self.capabilities.states, self.health,
                                     self.faults.faults, now, self.started_at)

    def tick(self) -> None:
        now = self.clock()
        self.faults.update(self.health, now)
        for job in self.recoveries.jobs.values():
            if job.resource_held or job.state in (RecoveryState.IDLE, RecoveryState.RUNNING):
                self.faults.faults[job.fault_code].active = True
        self._availability(now)
        self.permissions.update(self.capabilities.states, self.state, now,
                                self.stop_port, self.faults)
        self._availability(now)  # Stop failure may have introduced an operation lock.
        # Recovery cannot race an unconfirmed mission stop or unready independent safety.
        safe = (all(self.health.healthy(source, now) for source in self.config['safety_sources']) and
                not any(f.active and (f.source in self.config['safety_sources'] or f.source == 'mission_stop')
                        for f in self.faults.faults.values()) and
                not any(not lease.granted and not lease.stopped
                        for lease in self.permissions.leases.values()))
        stopped = all(self.health.healthy(source, now) for source in self.config['recovery_safety_sources'])
        self.recoveries.update(self.faults.faults, self.health, now, safe and stopped, safe)
        self._prune(now)
        self._record()
        self._persist(now)
        self.revision += 1

    def _prune(self, now) -> None:
        retention = self.config['retention_s']
        for lease_id in self.permissions.prune(now, retention):
            getattr(self.stop_port, 'forget', lambda _: None)(lease_id)
        for job in self.recoveries.prune(self.faults.faults, now, retention):
            getattr(self.recoveries.port, 'forget', lambda _: None)(job)
        self.faults.prune(now, retention, {job.fault_code for job in self.recoveries.jobs.values()})

    def _record(self) -> None:
        """Derive audit events from state differences, so no transition path can skip logging."""
        if self.state != self._seen_state:
            self._emit('STATE_CHANGED', detail=f'{self._seen_state.value} -> {self.state.value}')
            self._seen_state = self.state
        current = {}
        for code, fault in self.faults.faults.items():
            current[code] = fault.active
            if fault.active != self._seen_faults.get(code, False):
                self._emit('FAULT_RAISED' if fault.active else 'FAULT_CLEARED', code=code, source=fault.source,
                           severity=fault.severity, detail=fault.detail)
        self._seen_faults = current
        leases = {}
        for lease_id, lease in self.permissions.leases.items():
            key = leases[lease_id] = (lease.granted, lease.stop_accepted, lease.stopped)
            previous = self._seen_leases.get(lease_id, (False, False, lease.restored and lease.stopped))
            if key == previous:
                continue
            fields = dict(code=lease_id, source=lease.mission_id, detail=lease.reason)
            if lease.granted and not previous[0] and lease_id not in self._seen_leases:
                self._emit('LEASE_GRANTED', code=lease_id, source=lease.mission_id,
                           detail=','.join(lease.capabilities))
            if previous[0] and not lease.granted:
                self._emit('LEASE_RELEASED' if lease.reason.startswith('released') else 'LEASE_REVOKED',
                           **fields)
            if lease.stop_accepted and not previous[1]:
                self._emit('MISSION_STOP_ACCEPTED', **fields)
            if lease.stopped and not previous[2] and not lease.reason.startswith('released'):
                self._emit('MISSION_STOPPED', **fields)
        self._seen_leases = leases
        jobs = {}
        for job_id, job in self.recoveries.jobs.items():
            key = jobs[job_id] = (job.state, job.attempt)
            if self._seen_jobs.get(job_id) != key:
                self._emit('RECOVERY_' + job.state.value, code=job.fault_code, source=job.source,
                           detail=f'job {job_id} attempt {job.attempt}: {job.detail}')
        self._seen_jobs = jobs

    def _persist(self, now) -> None:
        if self.store is None:
            return
        state = {'faults': self.faults.persisted(), 'leases': self.permissions.persisted()}
        if state == self._persisted:
            return
        try:
            self.store.save(state)
            self._persisted, self._persist_failed = state, False
        except Exception as exc:
            # A latch that cannot be persisted could vanish on restart: lock instead.
            self.faults.raise_state_fault(now, f'latch state persistence failed: {exc}')
            if not self._persist_failed:
                self._emit('STATE_PERSIST_FAILED', code=STATE_FAULT, severity='CRITICAL', detail=str(exc))
            self._persist_failed = True

    def check_permission(self, **request):
        self.tick()
        decision = self.permissions.check(**request, states=self.capabilities.states,
                                          operating=self.state, now=self.clock())
        self._record()
        # Write-ahead: a grant becomes durable before the client can move under it, so a
        # crash right after the reply still leaves an unconfirmed lease to re-verify.
        self._persist(self.clock())
        if decision.granted and self._persist_failed:
            lease = self.permissions.leases[decision.lease_id]
            lease.granted, lease.reason = False, 'lease could not be persisted'
            return PermissionDecision(False, self.epoch, reason='lease could not be persisted')
        return decision

    def reset_fault(self, epoch: str, code: str, operator: str = '') -> tuple[bool, str]:
        result = self._reset(epoch, code, operator)
        self._emit('FAULT_RESET_' + ('ACCEPTED' if result[0] else 'REJECTED'), code=code,
                   source=operator, detail=result[1])
        return result

    def _reset(self, epoch: str, code: str, operator: str) -> tuple[bool, str]:
        if epoch != self.epoch:
            return False, 'supervisor epoch mismatch'
        if not operator.strip():
            return False, 'operator identity required for fault reset'
        if code.startswith('MISSION_STOP_FAILED:'):
            lease = self.permissions.leases.get(code.removeprefix('MISSION_STOP_FAILED:'))
            if lease is None:
                return False, 'unknown mission stop fault'
            if not lease.stopped:
                # Re-query Mission through the same idempotent stop Action. The service
                # itself never claims that a previous failed stop has completed.
                try:
                    self.stop_port.request(lease, self.epoch)
                except Exception as exc:
                    return False, str(exc)
                return False, 'mission stop verification requested; await confirmed Action result'
            self.faults.confirm_stop(lease.lease_id, self.clock())
            self.tick()
            return True, 'mission stop confirmed'
        if any(job.fault_code == code and (job.resource_held or job.state in (
                RecoveryState.IDLE, RecoveryState.RUNNING)) for job in self.recoveries.jobs.values()):
            return False, 'component recovery is still in progress or quarantined'
        result = self.faults.reset(code, self.health, self.clock())
        self.tick()
        return result

    def request_recovery(self, epoch: str, code: str):
        if epoch != self.epoch:
            raise ValueError('supervisor epoch mismatch')
        fault = self.faults.faults.get(code)
        if fault is None:
            raise ValueError('unknown fault')
        try:
            job = self.recoveries.request(fault, self.clock())
        except UnsupportedRecovery as exc:
            self._emit('RECOVERY_UNSUPPORTED', code=code, source=fault.source, detail=str(exc))
            raise
        self._emit('RECOVERY_REQUESTED', code=code, source=fault.source, detail=f'job {job.job_id}')
        return job
