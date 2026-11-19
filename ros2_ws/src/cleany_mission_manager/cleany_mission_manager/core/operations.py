"""작업은 한 번 시작하고 비블로킹으로 polling하며 명시적으로 취소합니다."""

from typing import Callable, Protocol
from uuid import uuid4

from .result import ModuleResult, ResultStatus


class OperationPort(Protocol):
    def start(self, command: object) -> str: ...
    def poll(self, operation_id: str) -> ModuleResult | None: ...
    def cancel(self, operation_id: str) -> None: ...
    def ready(self) -> tuple[bool, str]: ...
    def stopped(self) -> bool: ...


class ExecutorPort(OperationPort, Protocol):
    """물리적 정지와 베이스 이동 허가는 서로 다른 사실입니다."""

    def safe_to_drive(self) -> tuple[bool, str]: ...
    def maintain(self) -> None: ...


class DeferredPort:
    """테스트와 명시적인 mock 책상 작업을 위한 결정적 비동기 어댑터입니다."""

    def __init__(self, handler: Callable[[object], ModuleResult], polls: int = 2) -> None:
        self.handler = handler
        self.polls = polls
        self.pending: dict[str, tuple[object, int]] = {}
        self.results: dict[str, ModuleResult] = {}
        self.commands: list[object] = []

    def start(self, command: object) -> str:
        if self.pending:
            raise RuntimeError("adapter already has an active operation")
        operation_id = str(uuid4())
        self.commands.append(command)
        self.pending[operation_id] = (command, self.polls)
        return operation_id

    def poll(self, operation_id: str) -> ModuleResult | None:
        if operation_id in self.results:
            return self.results[operation_id]
        command, remaining = self.pending[operation_id]
        if remaining > 0:
            self.pending[operation_id] = (command, remaining - 1)
            return None
        del self.pending[operation_id]
        self.results[operation_id] = self.handler(command)
        return self.results[operation_id]

    def cancel(self, operation_id: str) -> None:
        if operation_id in self.pending:
            del self.pending[operation_id]
            self.results[operation_id] = ModuleResult(False, ResultStatus.CANCELLED)

    def ready(self) -> tuple[bool, str]:
        return True, ""

    def stopped(self) -> bool:
        return not self.pending

    def safe_to_drive(self) -> tuple[bool, str]:
        return self.stopped(), "" if self.stopped() else "OPERATION_RUNNING"

    def maintain(self) -> None:
        pass
