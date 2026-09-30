"""Nonblocking backend boundary. Each poll returns fresh evidence or None."""

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


class ManipulationPort(Protocol):
    def begin(self, stage: Stage, goal: Goal, now: float) -> None: ...
    def poll(self, now: float) -> Evidence | None: ...
    def stop(self, now: float) -> None: ...
    def fault(self, now: float) -> Error | None: ...
