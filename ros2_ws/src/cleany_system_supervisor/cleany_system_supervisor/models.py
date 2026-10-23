"""ROS-independent contracts. All deadlines use the injected monotonic clock."""
from dataclasses import dataclass
from enum import Enum


class Availability(str, Enum):
    AVAILABLE = 'AVAILABLE'
    UNAVAILABLE = 'UNAVAILABLE'
    UNKNOWN = 'UNKNOWN'


class OperatingState(str, Enum):
    INITIALIZING = 'INITIALIZING'
    OPERATIONAL = 'OPERATIONAL'
    DEGRADED = 'DEGRADED'
    FAULT_LOCKED = 'FAULT_LOCKED'


class RecoveryState(str, Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


@dataclass
class HealthReport:
    source: str
    status: Availability
    observed_at: float
    detail: str = ''


@dataclass
class Fault:
    code: str
    source: str
    severity: str
    capabilities: list[str]
    first_seen: float
    last_seen: float
    active: bool
    latched: bool
    recovery_policy: str
    detail: str = ''
    locks_operation: bool = False
    cleared_at: float | None = None
    restored: bool = False


@dataclass
class Lease:
    mission_id: str
    lease_id: str
    version: int
    capabilities: tuple[str, ...]
    expires_at: float
    granted: bool = True
    reason: str = ''
    stop_requested_at: float | None = None
    stop_accepted: bool = False
    stopped: bool = False
    ended_at: float | None = None
    restored: bool = False


@dataclass
class RecoveryJob:
    job_id: str
    fault_code: str
    source: str
    policy: dict
    state: RecoveryState = RecoveryState.IDLE
    attempt: int = 0
    started_at: float = 0.
    attempt_at: float = 0.
    verify_after: float | None = None
    cancel_at: float | None = None
    resource_held: bool = False
    cancel_requested: bool = False
    detail: str = ''
    finished_at: float | None = None


@dataclass
class PermissionDecision:
    granted: bool
    epoch: str
    lease_id: str = ''
    version: int = 0
    valid_for_s: float = 0.
    reason: str = ''


@dataclass
class SupervisorEvent:
    """Structured audit record; monotonic_s belongs to the emitting process epoch."""
    sequence: int
    kind: str
    monotonic_s: float
    code: str = ''
    source: str = ''
    severity: str = ''
    detail: str = ''
