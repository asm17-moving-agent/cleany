"""Bounded timing and live progress for serial planning operations."""
from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import threading
import time


@dataclass
class _Operation:
    phase: str
    detail: str
    started: float


class PlanningProgress:
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._active: list[_Operation] = []
        self.records: list[dict] = []
        self.callback: Callable[[str], None] | None = None

    def reset(self) -> None:
        with self._lock:
            self.records.clear()
            self._active.clear()

    def current_message(self) -> str:
        with self._lock:
            if not self._active:
                return ''
            operation = self._active[-1]
            return (f'phase={operation.phase} {operation.detail} '
                    f'phase_elapsed={self._clock()-operation.started:.1f}s')

    def update_detail(self, detail: str) -> None:
        with self._lock:
            if self._active:
                self._active[-1].detail = detail

    @contextmanager
    def stage(self, phase: str, detail: str = '') -> Iterator[None]:
        operation = _Operation(phase, detail, self._clock())
        with self._lock:
            self._active.append(operation)
        outcome = 'done'
        try:
            self._emit(f'phase={phase} state=started {detail}')
            yield
        except BaseException:
            outcome = 'failed'
            raise
        finally:
            seconds = self._clock()-operation.started
            with self._lock:
                self._active.pop()
                self.records.append(dict(phase=phase, detail=operation.detail,
                                         state=outcome, wall_sec=seconds))
                del self.records[:-128]
            self._emit(f'phase={phase} state={outcome} wall_sec={seconds:.3f} '
                       f'{operation.detail}')

    def _emit(self, message: str) -> None:
        if self.callback is not None:
            self.callback(message)
