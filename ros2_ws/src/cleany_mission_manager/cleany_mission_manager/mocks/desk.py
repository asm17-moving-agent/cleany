"""Stateful mock desk. No camera, arm, VLA, or physical cleaning is claimed."""

from typing import Callable
from uuid import uuid4

from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import ModuleResult
from cleany_mission_manager.core.runtime_models import SceneObject, SceneSnapshot, TaskProposal


class MockDesk:
    def __init__(self, clock: Callable[[], float], objects: tuple[SceneObject, ...] | None = None,
                 polls: int = 2, destination_id: str = "mock_trash_bin",
                 snapshot_ids: tuple[str, ...] = ()) -> None:
        self.clock = clock
        self.destination_id = destination_id
        self.snapshot_ids = snapshot_ids
        self.observations = 0
        self.initial = objects if objects is not None else (
            SceneObject("trash-1"), SceneObject("trash-2"),
        )
        self.objects = list(self.initial)
        self.perception = DeferredPort(self.observe, polls)
        self.planner = DeferredPort(self.plan, polls)
        self.executor = DeferredPort(self.execute, polls)

    def begin(self) -> None:
        self.objects = list(self.initial)
        self.observations = 0

    def observe(self, mission_id: object) -> ModuleResult:
        if self.snapshot_ids and self.observations >= len(self.snapshot_ids):
            raise RuntimeError("mock snapshot fixture exhausted; no observation invented")
        snapshot = self.snapshot_ids[self.observations] if self.snapshot_ids else str(uuid4())
        self.observations += 1
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
            self.destination_id if obj and obj.disposition == "collect_trash" else "",
            obj.wire_object_id if obj else None,
        ))

    def execute(self, command: object) -> ModuleResult:
        assert isinstance(command, TaskProposal)
        self.objects = [obj for obj in self.objects if obj.object_id != command.object_id]
        return ModuleResult.success(message="mock collection completed")

    def manipulation_succeeded(self, goal: dict) -> None:
        # Explicit mock bookkeeping, never used as camera/physical evidence.
        self.objects = [obj for obj in self.objects if obj.wire_object_id != goal["object_id"]]
