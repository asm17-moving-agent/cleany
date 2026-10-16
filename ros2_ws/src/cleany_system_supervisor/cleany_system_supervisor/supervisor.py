from time import monotonic
from uuid import uuid4

from .capabilities import CapabilityManager, operating_state
from .config import validate
from .faults import FaultManager
from .health import HealthMonitoring
from .models import Availability, HealthReport, OperatingState, RecoveryState
from .permissions import MissionPermissionManager
from .ports import MissionStopTransport, RecoveryTransport
from .recovery import RecoveryCoordinator


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


class SystemSupervisor:
    def __init__(self, config: dict, *, clock=monotonic,
                 recovery_port: RecoveryTransport | None = None,
                 stop_port: MissionStopTransport | None = None):
        self.config, self.clock = validate(config), clock
        self.epoch, self.revision = uuid4().hex, 0
        self.started_at = clock()
        self.health = HealthMonitoring(config['sources'], self.started_at)
        self.faults = FaultManager(config)
        self.capabilities = CapabilityManager(config['capabilities'])
        self.permissions = MissionPermissionManager(self.epoch, config)
        self.recoveries = RecoveryCoordinator(config, recovery_port or UnavailablePort())
        self.stop_port = stop_port or UnavailablePort()
        self.state = OperatingState.INITIALIZING

    def observe(self, source: str, status: Availability, detail: str = '') -> None:
        self.health.observe(HealthReport(source, status, self.clock(), detail))

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
        self.revision += 1

    def check_permission(self, **request):
        self.tick()
        return self.permissions.check(**request, states=self.capabilities.states,
                                      operating=self.state, now=self.clock())

    def reset_fault(self, epoch: str, code: str) -> tuple[bool, str]:
        if epoch != self.epoch:
            return False, 'supervisor epoch mismatch'
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
            self.faults.confirm_stop(lease.lease_id)
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
        return self.recoveries.request(fault, self.clock())
