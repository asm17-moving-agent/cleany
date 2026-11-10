"""직렬화된 비동기 작업입니다. 백엔드와 worker 모두 작업 순서를 결정하지 않습니다."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from threading import Event
from typing import Any, Callable, Protocol
from uuid import uuid4

from cleany_skill_executor.manipulation.models import Error, Goal, ObjectState, Placement


@dataclass(frozen=True)
class Observation:
    success: bool = True
    error: Error = Error.NONE
    message: str = ''
    selected_arm: str | None = None
    object_state: ObjectState | None = None
    placement_state: Placement | None = None
    stop_confirmed: bool | None = None
    arm_recovered: bool | None = None


class OperationError(RuntimeError):
    def __init__(self, error: Error, message: str) -> None:
        super().__init__(message)
        self.error = error


@dataclass
class ExecutionContext:
    goal: Goal
    abort: Event = field(default_factory=Event)
    verification_id: str = ''
    snapshot: Any = None
    attempt: Any = None
    inspected: Any = None
    grasps: Any = None
    selected: Any = None
    target: Any = None
    grasp_progress: Any = None
    held: Any = None
    release_stamp_ns: int = 0
    release_confirmed: bool = False
    arm_recovered: bool = False
    stop_confirmed: bool = False
    gripper_engaged: bool = False


class Backend(Protocol):
    def ready(self) -> bool: ...
    def begin(self, goal: Goal) -> None: ...
    def start(self, node_id: str, execution_id: str) -> str: ...
    def poll(self, operation_id: str) -> Observation | None: ...
    def cancel(self, operation_id: str) -> None: ...
    def abort(self) -> None: ...


class WorkerBackend:
    def __init__(self, operation: Callable[[str, ExecutionContext], Observation],
                 *, quiescent: Callable[[], bool] = lambda: True) -> None:
        self.operation = operation
        self.quiescent = quiescent
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='manipulation-ops')
        self.context: ExecutionContext | None = None
        self.pending: dict[str, tuple[Future, ExecutionContext]] = {}

    def ready(self) -> bool:
        return (all(future.done() for future, _ in self.pending.values()) and self.quiescent())

    def begin(self, goal: Goal) -> None:
        if not self.ready():
            raise RuntimeError('Previous operation has not quiesced')
        self.pending.clear()
        self.context = ExecutionContext(goal)

    def start(self, node_id: str, execution_id: str) -> str:
        context = self.context
        if context is None or context.goal.execution_id != execution_id:
            raise RuntimeError('Execution identity mismatch')
        if node_id == 'ReleaseInPlace':
            # core는 StopAndAssess 이후에만 복구를 시작합니다. 제출 전에 이전 작업의
            # abort를 지워 이후 IMMEDIATE 요청이 새 작업에도 abort를 설정할 수 있게 합니다.
            context.abort.clear()
        token = uuid4().hex
        self.pending[token] = (self.worker.submit(self.operation, node_id, context), context)
        return token

    def poll(self, operation_id: str) -> Observation | None:
        future, context = self.pending[operation_id]
        if not future.done():
            return None
        try:
            return future.result()
        except OperationError as error:
            return Observation(False, error.error, str(error))
        except Exception as error:
            return Observation(False, Error.INTERNAL_ERROR, str(error))

    def cancel(self, operation_id: str) -> None:
        # Future 취소를 정지 근거로 삼지 않습니다. 작업은 보호된 ROS 경계를 통해
        # 빠져나오며 StopAndAssess가 이를 관측합니다.
        _, context = self.pending[operation_id]
        context.abort.set()

    def abort(self) -> None:
        """작업 제출 중 token을 잃었더라도 컨텍스트를 중단합니다."""
        if self.context is not None:
            self.context.abort.set()

    def close(self) -> None:
        if self.context is not None:
            self.context.abort.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
