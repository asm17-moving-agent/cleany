"""One-object action boundary, independent of the executor's internal BT."""

import json
from dataclasses import asdict
from typing import Any, Callable, Protocol

from .journal import MissionJournal
from .result import FailureCode, ModuleResult, ResultStatus
from .runtime_models import ManipulationOutcome, TaskProposal


class Future(Protocol):
    def done(self) -> bool: ...
    def result(self) -> Any: ...
    def cancel(self) -> bool: ...


class ManipulationTransport(Protocol):
    def ready(self) -> bool: ...
    def send(self, goal: dict) -> Future: ...
    def cancel(self, execution_id: str) -> Future: ...
    def lookup(self, execution_id: str) -> Future: ...


def validate_backend(backend: str, journal: MissionJournal) -> None:
    if backend not in ("mock", "action_mock"):
        raise ValueError("manipulation_backend must be mock or action_mock")
    if backend == "mock" and journal.value(ManipulationPort.KEY):
        raise ValueError("persisted manipulation execution requires action_mock recovery; "
                         "switching to a local mock cannot bypass the saved arm state")


def goal_fields(command: TaskProposal) -> dict:
    """Numeric IDs come explicitly from perception; never parse display IDs."""
    goal = dict(mission_id=command.mission_id, task_id=command.task_id,
                execution_id=command.execution_id, skill_name=command.action,
                snapshot_id=command.snapshot_id, object_id=command.wire_object_id,
                destination_id=command.destination_id)
    if (command.action != "collect_trash" or type(command.wire_object_id) is not int
            or not 0 < command.wire_object_id <= 0xffffffff
            or any(not value.strip() for key, value in goal.items() if key != "object_id")):
        raise ValueError("invalid approved manipulation request")
    return goal


def map_outcome(outcome: ManipulationOutcome, execution_id: str,
                ros_status: int | None = None) -> ModuleResult:
    """ROS SUCCEEDED also contains BLOCKED; physical payload is authoritative."""
    status = {"SUCCESS": ResultStatus.OK, "BLOCKED": ResultStatus.BLOCKED,
              "FAILED": ResultStatus.FAILED, "CANCELED": ResultStatus.CANCELLED,
              "FATAL": ResultStatus.FATAL}.get(outcome.status)
    valid = (outcome.execution_id == execution_id and outcome.execution_profile == "mock"
             and status is not None
             and outcome.object_state in ("NOT_TOUCHED", "HELD", "LEFT_GRIPPER", "UNKNOWN")
             and outcome.placement_state in ("NOT_CHECKED", "CONFIRMED", "NOT_CONFIRMED", "UNKNOWN"))
    if ros_status is not None:
        valid &= ros_status == {"SUCCESS": 4, "BLOCKED": 4, "FAILED": 6,
                                "CANCELED": 5, "FATAL": 6}.get(outcome.status)
    if outcome.status == "SUCCESS":
        valid &= (outcome.error_code == "NONE" and not outcome.failed_stage
                  and outcome.last_completed_stage == "VERIFYING_PLACEMENT"
                  and outcome.object_state == "LEFT_GRIPPER"
                  and outcome.placement_state == "CONFIRMED" and bool(outcome.selected_arm)
                  and outcome.stop_confirmed and outcome.arm_recovered and not outcome.retryable)
    if not valid:
        raise ValueError("inconsistent manipulation result")
    code = {"GRASP_FAILED": FailureCode.GRASP_FAIL, "GRASP_LOST": FailureCode.GRASP_FAIL,
            "PLACEMENT_NOT_CONFIRMED": FailureCode.PLACE_FAIL,
            "TIMEOUT": FailureCode.TIMEOUT, "VERIFICATION_TIMEOUT": FailureCode.TIMEOUT,
            "E_STOP": FailureCode.E_STOP, "HARDWARE_ERROR": FailureCode.HARDWARE_ERROR}.get(
                outcome.error_code, FailureCode.SKILL_FAIL)
    return ModuleResult(status == ResultStatus.OK, status,
                        None if status in (ResultStatus.OK, ResultStatus.CANCELLED) else code,
                        outcome.retryable and outcome.safe_to_drive,
                        outcome.message, outcome)


class ManipulationPort:
    """Persist before send. Uncertain acceptance/result never permits movement/replay.

    Transport futures complete on the ROS executor. maintain() only checks done futures.
    A lookup miss is ambiguous with the peer service's current storage-error response.
    It therefore never clears the execution or proves a physical stop.
    """

    KEY = "manipulation_boundary"

    def __init__(self, transport: ManipulationTransport, journal: MissionJournal, clock: Callable[[], float],
                 *, initial_mock_safe: bool = False, on_success: Callable[[dict], None] | None = None) -> None:
        self.transport, self.journal, self.clock = transport, journal, clock
        self.on_success = on_success
        saved = json.loads(journal.value(self.KEY, "{}"))
        self.goal = saved.get("goal")
        self.outcome = (ManipulationOutcome(**saved["outcome"])
                        if saved.get("outcome") else None)
        self.human_review = saved.get("human_review", False)
        self.initial_mock_safe = initial_mock_safe and not saved
        self.result = map_outcome(self.outcome, self.goal["execution_id"]) if self.outcome else None
        self.send_future = self.result_future = self.lookup_future = self.cancel_future = None
        self.cancel_requested = bool(self.goal and self.outcome is None)  # restart: stop, no resume
        self.last_query = self.last_cancel = float("-inf")
        self.query_started = self.cancel_started = 0.0

    def _save(self) -> None:
        self.journal.set(self.KEY, json.dumps(dict(
            goal=self.goal, outcome=asdict(self.outcome) if self.outcome else None,
            human_review=self.human_review,
        )))

    def ready(self) -> tuple[bool, str]:
        safe, reason = self.safe_to_drive()
        return (False, reason) if not safe else (
            (True, "") if self.transport.ready() else (False, "MANIPULATION_NOT_READY"))

    def start(self, command: object) -> str:
        if not isinstance(command, TaskProposal):
            raise ValueError("expected approved TaskProposal")
        goal = goal_fields(command)
        if not self.ready()[0]:
            raise RuntimeError("manipulation is not ready for another execution")
        self.goal, self.outcome, self.result = goal, None, None
        self.human_review = False
        self.initial_mock_safe = False
        self.cancel_requested = False
        self.send_future = self.result_future = self.lookup_future = self.cancel_future = None
        self._save()  # If this fails, no actuator request has been sent.
        try:
            self.send_future = self.transport.send(goal)
        except Exception:
            # Sending may already have crossed the boundary; query this exact ID.
            self.cancel_requested = True
        return goal["execution_id"]

    def _complete(self, outcome: ManipulationOutcome, ros_status=None, human_review=False) -> None:
        assert self.goal is not None
        result = map_outcome(outcome, self.goal["execution_id"], ros_status)
        self.outcome, self.human_review = outcome, human_review
        self._save()  # Durable physical evidence before releasing any gate.
        self.result = result
        if outcome.status == "SUCCESS" and self.on_success:
            self.on_success(self.goal)

    def maintain(self) -> None:
        if not self.goal or self.outcome is not None:
            return
        if self.send_future and self.send_future.done():
            future, self.send_future = self.send_future, None
            try:
                handle = future.result()
                if handle.accepted:
                    self.result_future = handle.get_result_async()
                else:
                    self.cancel_requested = True  # Could be a duplicate execution.
            except Exception:
                self.cancel_requested = True
        if self.result_future and self.result_future.done():
            future, self.result_future = self.result_future, None
            try:
                response = future.result()
                self._complete(ManipulationOutcome(**response["result"]), response["status"])
            except Exception:
                self.outcome = None
                self.cancel_requested = True
        if self.outcome is not None:
            return
        now = self.clock()
        if self.cancel_requested:
            if self.cancel_future and (self.cancel_future.done() or now - self.cancel_started > 1):
                self.cancel_future.cancel()
                self.cancel_future = None
            if self.cancel_future is None and now - self.last_cancel >= 1:
                self.last_cancel = self.cancel_started = now
                try:
                    self.cancel_future = self.transport.cancel(self.goal["execution_id"])
                except Exception:
                    pass
        if self.lookup_future and self.lookup_future.done():
            future, self.lookup_future = self.lookup_future, None
            try:
                record = future.result()
                if (record and all(record.get(k) == v for k, v in self.goal.items())
                        and record.get("record_state") == "FINISHED" and record.get("has_result")):
                    outcome = ManipulationOutcome(**{k: record[k]
                                                     for k in ManipulationOutcome.__dataclass_fields__})
                    self._complete(outcome, human_review=record.get("human_confirmation_required", True))
            except Exception:
                self.outcome = None
        if self.outcome is not None:
            return
        if self.lookup_future and now - self.query_started > 1:
            self.lookup_future.cancel()
            self.lookup_future = None
        if self.lookup_future is None and now - self.last_query >= 1:
            self.last_query = self.query_started = now
            try:
                self.lookup_future = self.transport.lookup(self.goal["execution_id"])
            except Exception:
                pass

    def poll(self, operation_id: str) -> ModuleResult | None:
        self.maintain()
        return self.result if self.goal and self.goal["execution_id"] == operation_id else None

    def cancel(self, operation_id: str) -> None:
        if self.goal and self.goal["execution_id"] == operation_id and self.outcome is None:
            self.cancel_requested = True
            self.maintain()

    def stopped(self) -> bool:
        return self.initial_mock_safe or bool(self.outcome and self.outcome.stop_confirmed)

    def safe_to_drive(self) -> tuple[bool, str]:
        if self.initial_mock_safe:
            return True, ""
        if self.outcome and self.outcome.safe_to_drive and not self.human_review:
            return True, ""
        return False, "MANIPULATION_STATE_UNSAFE_OR_UNKNOWN"
