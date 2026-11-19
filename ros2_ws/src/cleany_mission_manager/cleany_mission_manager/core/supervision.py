"""Mission 측에서 보는 System Supervisor 권한이며 ROS 의존성이 없습니다."""

from enum import Enum
from typing import Protocol


class Authorization(str, Enum):
    PENDING = "PENDING"
    GRANTED = "GRANTED"
    DENIED = "DENIED"


class SupervisionPort(Protocol):
    """비블로킹 임대 client입니다. Supervisor가 시스템 fault와 capability를 관리합니다.

    Runtime은 시스템 가용성을 조회하고 임무별 임대 하나를 요청하며 권한을 잃으면
    정지합니다. Runtime이 자체 작업의 정지를 확인한 이후에만 반환할 수 있습니다.
    """

    def available(self) -> tuple[bool, str]:
        """수락에 쓰는 캐시된 시스템 상태이며 임대가 아닙니다."""
        ...

    def acquire(self, mission_id: str) -> None: ...

    def authorization(self) -> tuple[Authorization, str]: ...

    def release(self) -> None:
        """종료 결과와 정지 확인 이후 임대를 반환합니다."""
        ...

    def abandon(self) -> None:
        """정지를 주장하지 않고 갱신을 중단하며 이후 Supervisor가 정지를 검증합니다."""
        ...

    def maintain(self) -> None: ...
