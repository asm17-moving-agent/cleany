"""컴포넌트가 소유하는 작업이며 모든 메서드는 블로킹 없이 반환해야 합니다."""
from typing import Protocol

from .models import Lease, RecoveryJob


class RecoveryTransport(Protocol):
    def start(self, job: RecoveryJob) -> None:
        ...

    def poll(self, job: RecoveryJob) -> str:
        """RUNNING/ACCEPTED/UNKNOWN 또는 확인된 종료 결과입니다."""
        ...

    def cancel(self, job: RecoveryJob) -> None:
        ...

    def forget(self, job: RecoveryJob) -> None:
        """정리 대상 종료 작업의 관리 기록을 제거합니다."""
        ...


class MissionStopTransport(Protocol):
    def request(self, lease: Lease, epoch: str) -> None:
        ...

    def poll(self, lease_id: str) -> str:
        """ACCEPTED는 수신 확인이며 STOPPED에는 컴포넌트 완료 근거가 필요합니다."""
        ...

    def forget(self, lease_id: str) -> None:
        ...
