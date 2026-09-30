"""Stateful mock desk. No camera, arm, VLA, or physical cleaning is claimed."""

from typing import Callable
from uuid import uuid4

from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import ModuleResult
from cleany_mission_manager.core.runtime_models import SceneObject, SceneSnapshot, TaskProposal


class MockDesk:
    def __init__(self, clock: Callable[[], float], objects: tuple[SceneObject, ...] | None = None,
                 polls: int = 2) -> None:
        self.clock = clock
        self.initial = objects if objects is not None else (
            SceneObject("trash-1"), SceneObject("trash-2"),
        )
        self.objects = list(self.initial)
        self.perception = DeferredPort(self.observe, polls)
        self.planner = DeferredPort(self.plan, polls)
        self.executor = DeferredPort(self.execute, polls)

    def begin(self) -> None:
        self.objects = list(self.initial)

    def observe(self, mission_id: object) -> ModuleResult:
        snapshot = str(uuid4())
        return ModuleResult.success(SceneSnapshot(
            snapshot, self.clock(), tuple(self.objects),
            f"mock://observations/{mission_id}/{snapshot}",
        ))

    def plan(self, command: object) -> ModuleResult:
        scene, records = command
        skipped = {record.object_id for record in records if record.status == "SKIPPED"}
        obj = next((obj for obj in scene.objects if obj.object_id not in skipped), None)
        return ModuleResult.success(TaskProposal(
            obj.disposition if obj else "done", scene.snapshot_id,
            obj.object_id if obj else "",
        ))

    def execute(self, command: object) -> ModuleResult:
        assert isinstance(command, TaskProposal)
        self.objects = [obj for obj in self.objects if obj.object_id != command.object_id]
        return ModuleResult.success(message="mock collection completed")
