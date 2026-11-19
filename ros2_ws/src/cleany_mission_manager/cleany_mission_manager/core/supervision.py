"""Mission-side view of System Supervisor authority; no ROS dependency."""

from enum import Enum
from typing import Protocol


class Authorization(str, Enum):
    PENDING = "PENDING"
    GRANTED = "GRANTED"
    DENIED = "DENIED"


class SupervisionPort(Protocol):
    """Nonblocking lease client. The Supervisor owns system faults and capabilities.

    The Runtime asks it whether the system is available, requests one lease per
    mission and stops when authority is lost. Releasing is allowed only after the
    Runtime itself confirmed that its operations stopped.
    """

    def available(self) -> tuple[bool, str]:
        """Cached system view used for admission; never a lease."""
        ...

    def acquire(self, mission_id: str) -> None: ...

    def authorization(self) -> tuple[Authorization, str]: ...

    def release(self) -> None:
        """Return a lease after a terminal outcome with confirmed stop."""
        ...

    def abandon(self) -> None:
        """Stop renewing without claiming a stop; the Supervisor then verifies it."""
        ...

    def maintain(self) -> None: ...
