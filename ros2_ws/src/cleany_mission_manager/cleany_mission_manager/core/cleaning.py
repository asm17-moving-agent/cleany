"""Desk BT: one proposal, one operation, then a fresh observation."""

from dataclasses import dataclass, field
from typing import Callable

import py_trees

from .operations import OperationPort
from .result import FailureCode, ModuleResult, ResultStatus
from .runtime_models import ActionRecord, RuntimePolicy, SceneSnapshot, TaskProposal


@dataclass
class CleaningContext:
    scene: SceneSnapshot | None = None
    proposal: TaskProposal | None = None
    before: str = ""
    after: str = ""
    records: list[ActionRecord] = field(default_factory=list)
    seen_snapshots: set[str] = field(default_factory=set)
    completed: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    needs_review: bool = False
    complete: bool = False
    actions: int = 0
    failures: dict[str, int] = field(default_factory=dict)
    failure: ModuleResult | None = None
    abort_after_observe: ModuleResult | None = None
    stage: str = ""


class OperationLeaf(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, tree: "CleaningTree", port: OperationPort,
                 command: Callable[[], object], consume: Callable[[ModuleResult], bool]) -> None:
        super().__init__(name)
        self.tree = tree
        self.port = port
        self.command = command
        self.consume = consume
        self.operation_id: str | None = None
        self.started_at = 0.0

    def initialise(self) -> None:
        self.operation_id = None
        self.started_at = self.tree.clock()

    def update(self) -> py_trees.common.Status:
        self.tree.context.stage = self.name
        if self.operation_id is None:
            command = self.command()
            if command is None:
                return py_trees.common.Status.SUCCESS
            self.operation_id = self.port.start(command)
        result = self.port.poll(self.operation_id)
        if result is None:
            if self.tree.clock() - self.started_at > self.tree.policy.operation_timeout:
                self.port.cancel(self.operation_id)
                self.tree.context.failure = ModuleResult.failed(
                    FailureCode.TIMEOUT, message=f"{self.name} timed out"
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        return (py_trees.common.Status.SUCCESS if self.consume(result)
                else py_trees.common.Status.FAILURE)

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.INVALID and self.operation_id:
            self.port.cancel(self.operation_id)


class CheckLeaf(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, tree: "CleaningTree", check: Callable[[], bool]) -> None:
        super().__init__(name)
        self.tree, self.check = tree, check

    def update(self) -> py_trees.common.Status:
        self.tree.context.stage = self.name
        return (py_trees.common.Status.SUCCESS if self.check()
                else py_trees.common.Status.FAILURE)


class UntilVerified(py_trees.decorators.Decorator):
    def __init__(self, child: py_trees.behaviour.Behaviour, context: CleaningContext) -> None:
        super().__init__(name="UntilVerified", child=child)
        self.context = context

    def update(self) -> py_trees.common.Status:
        status = self.decorated.status
        if status == py_trees.common.Status.SUCCESS and not self.context.complete:
            self.decorated.stop(py_trees.common.Status.INVALID)
            return py_trees.common.Status.RUNNING
        return status


class CleaningTree:
    def __init__(self, *, perception: OperationPort, planner: OperationPort,
                 executor: OperationPort, clock: Callable[[], float], policy: RuntimePolicy,
                 mission_id: str, mode: str = "mock") -> None:
        self.context = CleaningContext()
        self.clock, self.policy, self.mode = clock, policy, mode
        self.ports = (perception, planner, executor)
        observe = lambda: mission_id
        cycle = py_trees.composites.Sequence(name="ActionCheckpoint", memory=True, children=[
            OperationLeaf("PlanNext", self, planner,
                          lambda: (self.context.scene, tuple(self.context.records)), self._plan),
            CheckLeaf("ValidateProposal", self, self._validate),
            OperationLeaf("ExecuteOne", self, executor, self._action_command, self._executed),
            OperationLeaf("Reobserve", self, perception, observe, self._observed),
            CheckLeaf("VerifyCompletion", self, self._verify),
        ])
        self.root = py_trees.composites.Sequence(name="CleanDesk", memory=True, children=[
            OperationLeaf("ObserveBefore", self, perception, observe, self._observed),
            UntilVerified(cycle, self.context),
        ])

    def tick(self) -> py_trees.common.Status:
        self.root.tick_once()
        return self.root.status

    def halt(self) -> None:
        self.root.stop(py_trees.common.Status.INVALID)

    def stopped(self) -> bool:
        return all(port.stopped() for port in self.ports)

    def _reject(self, code: FailureCode, message: str) -> bool:
        self.context.failure = ModuleResult.blocked(code, message=message)
        return False

    def _observed(self, result: ModuleResult) -> bool:
        if result.status != ResultStatus.OK:
            self.context.failure = result
            return False
        scene = result.data
        if not isinstance(scene, SceneSnapshot):
            return self._reject(FailureCode.PERCEPTION_FAIL, "expected a typed SceneSnapshot")
        if (not scene.snapshot_id or scene.snapshot_id in self.context.seen_snapshots
                or scene.source != self.mode or not scene.reference
                or not 0 <= self.clock() - scene.captured_at <= self.policy.operation_timeout
                or len({obj.object_id for obj in scene.objects}) != len(scene.objects)):
            return self._reject(FailureCode.PERCEPTION_FAIL, "invalid or stale observation")
        self.context.seen_snapshots.add(scene.snapshot_id)
        self.context.scene = scene
        if not self.context.before:
            self.context.before = scene.reference
        self.context.after = scene.reference
        return True

    def _plan(self, result: ModuleResult) -> bool:
        if result.status != ResultStatus.OK:
            self.context.failure = result
            return False
        if not isinstance(result.data, TaskProposal):
            return self._reject(FailureCode.PLAN_BLOCKED, "expected one typed TaskProposal")
        self.context.proposal = result.data
        return True

    def _validate(self) -> bool:
        proposal, scene = self.context.proposal, self.context.scene
        if proposal is None or scene is None or proposal.snapshot_id != scene.snapshot_id:
            return self._reject(FailureCode.PLAN_BLOCKED, "proposal references another scene")
        if proposal.action not in ("collect_trash", "skip", "human_review", "done"):
            return self._reject(FailureCode.PLAN_BLOCKED, "capability is not allowed")
        if proposal.action == "done":
            return not proposal.object_id or self._reject(
                FailureCode.PLAN_BLOCKED, "done must not contain an object"
            )
        obj = next((obj for obj in scene.objects if obj.object_id == proposal.object_id), None)
        if obj is None or not obj.object_id or obj.object_id in self.context.skipped:
            return self._reject(FailureCode.PLAN_BLOCKED, "object is unavailable")
        if proposal.action == "collect_trash" and obj.disposition != "collect_trash":
            return self._reject(FailureCode.PLAN_BLOCKED, "object is not permitted for collection")
        if self.context.actions >= self.policy.max_actions:
            return self._reject(FailureCode.TIMEOUT, "cleaning action budget exhausted")
        return True

    def _action_command(self) -> TaskProposal | None:
        proposal = self.context.proposal
        assert proposal is not None
        if proposal.action == "done":
            return None
        self.context.actions += 1
        if proposal.action in ("skip", "human_review"):
            self.context.skipped.append(proposal.object_id)
            self.context.needs_review |= proposal.action == "human_review"
            self.context.records.append(ActionRecord(
                proposal.object_id, proposal.action, proposal.snapshot_id, "SKIPPED"
            ))
            return None
        return proposal

    def _executed(self, result: ModuleResult) -> bool:
        proposal = self.context.proposal
        assert proposal is not None
        self.context.records.append(ActionRecord(
            proposal.object_id, proposal.action, proposal.snapshot_id,
            result.status.value, result.message,
        ))
        if result.status == ResultStatus.OK:
            self.context.completed.append(proposal.object_id)
            return True
        if result.status == ResultStatus.FATAL:
            self.context.failure = result
            return False
        attempts = self.context.failures.get(proposal.object_id, 0) + 1
        self.context.failures[proposal.object_id] = attempts
        if (not result.retryable or result.status != ResultStatus.FAILED
                or attempts > self.policy.max_skill_retries):
            self.context.abort_after_observe = result
        # Even exhausted failures are recorded and reobserved before termination.
        return True

    def _verify(self) -> bool:
        if self.context.abort_after_observe:
            self.context.failure = self.context.abort_after_observe
            return False
        proposal, scene = self.context.proposal, self.context.scene
        assert proposal is not None and scene is not None
        if proposal.action == "done":
            self.context.complete = all(
                obj.object_id in self.context.skipped for obj in scene.objects
            )
        return True
