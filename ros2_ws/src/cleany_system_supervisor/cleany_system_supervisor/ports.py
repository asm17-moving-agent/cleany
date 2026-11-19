"""Component-owned operations. Every method must return without blocking."""
from typing import Protocol

from .models import Lease, RecoveryJob


class RecoveryTransport(Protocol):
    def start(self, job: RecoveryJob) -> None:
        ...

    def poll(self, job: RecoveryJob) -> str:
        """RUNNING/ACCEPTED/UNKNOWN or a confirmed terminal outcome."""
        ...

    def cancel(self, job: RecoveryJob) -> None:
        ...

    def forget(self, job: RecoveryJob) -> None:
        """Drop bookkeeping for a pruned terminal job."""
        ...


class MissionStopTransport(Protocol):
    def request(self, lease: Lease, epoch: str) -> None:
        ...

    def poll(self, lease_id: str) -> str:
        """ACCEPTED is receipt; STOPPED requires component completion proof."""
        ...

    def forget(self, lease_id: str) -> None:
        ...
