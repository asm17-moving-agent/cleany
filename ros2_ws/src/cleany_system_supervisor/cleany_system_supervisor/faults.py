from .models import Availability, Fault


class FaultManager:
    def __init__(self, config: dict):
        self.config = config
        self.faults: dict[str, Fault] = {}

    def update(self, health, now: float) -> None:
        for source, policy in self.config['sources'].items():
            if policy.get('monitor_only', False):
                continue
            status = health.status(source, now)
            fault = self.faults.get(policy['fault_code'])
            if status != Availability.AVAILABLE:
                # A missing source gets a boot grace; explicit bad evidence does not.
                missing = source not in health.reports
                delay = max(policy['raise_s'], self.config['startup_grace_s'] if missing else 0.)
                if now - health.bad_since[source] >= delay:
                    report = health.reports.get(source)
                    detail = report.detail if report and status != Availability.UNKNOWN else 'missing or stale health'
                    if fault is None or not fault.active:
                        affected = [name for name, entry in self.config['capabilities'].items()
                                    if source in entry['sources']]
                        fault = Fault(policy['fault_code'], source, policy['severity'], affected,
                                      now, now, True, policy['latched'], policy.get('recovery', ''),
                                      detail, policy['locks_operation'])
                        self.faults[fault.code] = fault
                    fault.last_seen, fault.detail = now, detail
            elif fault and fault.active and not fault.latched and health.healthy(source, now):
                fault.active = False

    def reset(self, code: str, health, now: float) -> tuple[bool, str]:
        fault = self.faults.get(code)
        if fault is None or not fault.active:
            return False, 'no active fault'
        if not health.healthy(fault.source, now):
            return False, 'cause unresolved or safety/health verification incomplete'
        fault.active = False
        return True, 'fault reset after fresh stable health verification'

    def escalate_stop(self, lease_id: str, now: float, detail: str) -> None:
        code = 'MISSION_STOP_FAILED:' + lease_id
        if code not in self.faults:
            self.faults[code] = Fault(code, 'mission_stop', 'CRITICAL', [], now, now,
                                      True, True, '', detail, True)

    def confirm_stop(self, lease_id: str) -> None:
        # Only confirmed completion, never Action acceptance, resolves stop escalation.
        fault = self.faults.get('MISSION_STOP_FAILED:' + lease_id)
        if fault:
            fault.active = False
