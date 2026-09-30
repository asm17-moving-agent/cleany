"""An operation is started once, polled without blocking, and explicitly cancelled."""

from typing import Callable, Protocol
from uuid import uuid4

from .result import ModuleResult, ResultStatus


class OperationPort(Protocol):
    def start(self, command: object) -> str: ...
    def poll(self, operation_id: str) -> ModuleResult | None: ...
    def cancel(self, operation_id: str) -> None: ...
    def ready(self) -> tuple[bool, str]: ...
    def stopped(self) -> bool: ...


class DeferredPort:
    """Deterministic asynchronous adapter for tests and explicitly mock desk work."""

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
