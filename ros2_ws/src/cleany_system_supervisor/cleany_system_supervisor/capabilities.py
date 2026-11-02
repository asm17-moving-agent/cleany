from .models import Availability, OperatingState


class CapabilityManager:
    def __init__(self, definitions: dict):
        self.definitions = definitions
        self.states = {name: Availability.UNKNOWN for name in definitions}

    def update(self, health, faults, now: float) -> None:
        resolved = {}

        def resolve(name):
            if name in resolved:
                return resolved[name]
            entry = self.definitions[name]
            evidence = [health.status(source, now) for source in entry['sources']]
            # AVAILABLE requires stable evidence, including at initial boot.
            evidence = [value if value != Availability.AVAILABLE or health.healthy(source, now)
                        else Availability.UNKNOWN for source, value in zip(entry['sources'], evidence)]
            evidence += [resolve(dependency) for dependency in entry['depends_on']]
            if any(f.active and name in f.capabilities for f in faults.values()):
                evidence.append(Availability.UNAVAILABLE)
            resolved[name] = (Availability.UNAVAILABLE if Availability.UNAVAILABLE in evidence else
                              Availability.UNKNOWN if Availability.UNKNOWN in evidence else
                              Availability.AVAILABLE)
            return resolved[name]
        for name in self.definitions:
            resolve(name)
        self.states = resolved


def operating_state(config, capabilities, health, faults, now, started_at):
    if any(f.active and f.locks_operation for f in faults.values()):
        return OperatingState.FAULT_LOCKED
    required = [capabilities[name] for name in config['essential_capabilities']]
    required += [Availability.AVAILABLE if health.healthy(source, now) else health.status(source, now)
                 for source in config['safety_sources']]
    # Stable health is required even if status is nominally available.
    ready = (all(value == Availability.AVAILABLE for value in required) and
             all(health.healthy(source, now) for source in config['safety_sources']))
    if not ready:
        if Availability.UNAVAILABLE in required or now - started_at >= config['startup_grace_s']:
            return OperatingState.FAULT_LOCKED
        return OperatingState.INITIALIZING
    if any(value != Availability.AVAILABLE for value in capabilities.values()):
        return OperatingState.DEGRADED
    return OperatingState.OPERATIONAL
