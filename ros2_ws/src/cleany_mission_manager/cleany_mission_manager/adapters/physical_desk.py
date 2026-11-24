"""Desk operations for a supervised field mission."""

from cleany_mission_manager.core.operations import DeferredPort
from cleany_mission_manager.core.result import FailureCode, ModuleResult
from cleany_mission_manager.core.runtime_models import TaskProposal

from .physical_scene import PhysicalScenePort


class PhysicalDesk:
    def __init__(self, node, value):
        self.destination = value("manipulation_destination")
        self.perception = PhysicalScenePort(
            node,
            value("inspection_action"),
            value("inspection_image_topic"),
            value("observation_directory"),
            value("allowed_trash_labels"),
            value("scene_max_age_s"),
        )
        self.planner = DeferredPort(self.plan, polls=0)

    def plan(self, command):
        scene, records = command
        completed = {r.object_id for r in records if r.status == "OK"}
        skipped = {r.object_id for r in records if r.status == "SKIPPED"}
        if any(obj.object_id in completed for obj in scene.objects):
            return ModuleResult.blocked(
                FailureCode.PLAN_BLOCKED, message="Collected track is still on the desk"
            )
        remaining = [obj for obj in scene.objects if obj.object_id not in skipped]
        obj = next(
            (o for o in remaining if o.disposition == "collect_trash"), next(iter(remaining), None)
        )
        if obj is None:
            return ModuleResult.success(TaskProposal("done", scene.snapshot_id))
        return ModuleResult.success(
            TaskProposal(
                obj.disposition,
                scene.snapshot_id,
                obj.object_id,
                self.destination if obj.disposition == "collect_trash" else "",
                obj.wire_object_id,
            )
        )
