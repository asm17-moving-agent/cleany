from .models import Availability, Fault

STATE_FAULT = 'SUPERVISOR_STATE_UNVERIFIED'


class FaultManager:
    def __init__(self, config: dict, started_at: float):
        self.config = config
        self.started_at = started_at
        self.faults: dict[str, Fault] = {}

    def update(self, health, now: float) -> None:
        for source, policy in self.config['sources'].items():
            if policy.get('monitor_only', False):
                continue
            status = health.status(source, now)
            fault = self.faults.get(policy['fault_code'])
            if status != Availability.AVAILABLE:
                # A never-verified source during the boot grace is not yet a failure. The
                # capability stays UNKNOWN meanwhile. Explicit bad evidence, or losing a
                # source that was verified, is not delayed.
                grace = policy.get('startup_grace_s', self.config['startup_grace_s'])
                if (status == Availability.UNKNOWN and source not in health.verified
                        and now - self.started_at < grace):
                    continue
                if now - health.bad_since[source] >= policy['raise_s']:
                    report = health.reports.get(source)
                    fresh = report is not None and now - report.observed_at <= policy['stale_s']
                    detail = (report.detail or report.status.value) if fresh else 'missing or stale health'
                    if fault is None or not fault.active:
                        affected = [name for name, entry in self.config['capabilities'].items()
                                    if source in entry['sources']]
                        fault = Fault(policy['fault_code'], source, policy['severity'], affected,
                                      now, now, True, policy['latched'], policy.get('recovery', ''),
                                      detail, policy['locks_operation'])
                        self.faults[fault.code] = fault
                    fault.last_seen = now
                    if not fault.restored:
                        fault.detail = detail
            elif fault and fault.active and not fault.latched and health.healthy(source, now):
                self._clear(fault, now)

    @staticmethod
    def _clear(fault: Fault, now: float) -> None:
        fault.active, fault.cleared_at = False, now

    def reset(self, code: str, health, now: float) -> tuple[bool, str]:
        fault = self.faults.get(code)
        if fault is None or not fault.active:
            return False, 'no active fault'
        if code == STATE_FAULT:
            # No sensor can prove persisted latches; the operator request is the confirmation.
            self._clear(fault, now)
            return True, 'operator acknowledged unverifiable supervisor latch state'
        if not health.healthy(fault.source, now):
            return False, 'cause unresolved or safety/health verification incomplete'
        self._clear(fault, now)
        return True, 'fault reset after fresh stable health verification'

    def escalate_stop(self, lease_id: str, now: float, detail: str) -> None:
        code = 'MISSION_STOP_FAILED:' + lease_id
        if code not in self.faults:
            self.faults[code] = Fault(code, 'mission_stop', 'CRITICAL', [], now, now,
                                      True, True, '', detail, True)

    def confirm_stop(self, lease_id: str, now: float) -> None:
        # Only confirmed completion, never Action acceptance, resolves stop escalation.
        fault = self.faults.get('MISSION_STOP_FAILED:' + lease_id)
        if fault and fault.active:
            self._clear(fault, now)

    def raise_state_fault(self, now: float, detail: str) -> None:
        fault = self.faults.get(STATE_FAULT)
        if fault is None or not fault.active:
            self.faults[STATE_FAULT] = Fault(STATE_FAULT, 'supervisor_state', 'CRITICAL', [], now, now,
                                             True, True, '', detail, True)
        else:
            fault.last_seen, fault.detail = now, detail

    def persisted(self) -> list[dict]:
        """Active latched faults survive restart; nonlatched ones are re-derived from health."""
        return [{'code': f.code, 'source': f.source, 'severity': f.severity, 'capabilities': f.capabilities,
                 'recovery_policy': f.recovery_policy, 'locks_operation': f.locks_operation, 'detail': f.detail}
                for f in self.faults.values() if f.active and f.latched]

    def restore(self, entries: list[dict], now: float) -> None:
        for entry in entries:
            self.faults[entry['code']] = Fault(
                entry['code'], entry['source'], entry['severity'], list(entry['capabilities']), now, now,
                True, True, entry['recovery_policy'],
                'restored after supervisor restart; operator reset required: ' + entry['detail'],
                bool(entry['locks_operation']), restored=True)

    def prune(self, now: float, retention_s: float, referenced: set[str]) -> None:
        for code in [code for code, fault in self.faults.items()
                     if not fault.active and code not in referenced and fault.cleared_at is not None
                     and now - fault.cleared_at >= retention_s]:
            del self.faults[code]
