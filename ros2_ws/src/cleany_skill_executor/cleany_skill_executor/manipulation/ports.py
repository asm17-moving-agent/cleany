"""비블로킹 백엔드 경계입니다. 각 poll은 새 근거나 None을 반환합니다."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .models import Error, Goal, ObjectState, Placement, Stage


@dataclass(frozen=True)
class Evidence:
    error: Error = Error.NONE
    object_state: ObjectState | None = None
    placement_state: Placement | None = None
    selected_arm: str | None = None
    stop_confirmed: bool | None = None
    arm_recovered: bool | None = None
    message: str = ''
    substage: str = ''
    completed_substage: str = ''
    stage_complete: bool = True


class ManipulationPort(Protocol):
    def begin(self, stage: Stage, goal: Goal, now: float) -> None: ...
    def poll(self, now: float) -> Evidence | None: ...
    def stop(self, now: float) -> None: ...
    def fault(self, now: float) -> Error | None: ...
