"""Single-owner FSM. tick() never waits for ROS actions or network I/O."""

from dataclasses import asdict, replace, is_dataclass
from collections import deque
from typing import Callable
from uuid import uuid4

import py_trees

from .cleaning import CleaningTree
from .journal import MissionJournal
from .operations import ExecutorPort, OperationPort
from .result import FailureCode, ModuleResult, ResultStatus
from .runtime_models import Admission, RuntimePolicy, RuntimeReport, RuntimeRequest, RuntimeState
from .supervision import Authorization, SupervisionPort


class MissionRuntime:
    """Own mission admission, progress and stop confirmation on one thread.

    Call tick() regularly with nonblocking operation ports. Construction turns
    unfinished journal entries into INTERRUPTED outcomes requiring operator
    review. Admission and terminal outcomes are persisted synchronously.

    With a supervision port the System Supervisor owns system faults and
    capabilities: admission needs its availability, motion needs a granted lease
    and lost authority stops the mission. Losing authority is not a Runtime
    execution error, so it ends the mission without entering ERROR. The Runtime
    still owns stop confirmation of its own operations and its ERROR/reset rules.
    """
    def __init__(self, *, navigator: OperationPort, perception: OperationPort,
                 planner: OperationPort, executor: ExecutorPort, clock: Callable[[], float],
                 supported_targets: set[str], policy: RuntimePolicy | None = None,
                 journal: MissionJournal | None = None,
                 on_accept: Callable[[], None] | None = None,
                 navigation_mode: str = "mock",
                 supervision: SupervisionPort | None = None) -> None:
        if navigation_mode not in ("sim", "mock"):
            raise ValueError("this runtime supports simulation or explicit mocks only")
        self.execution_profile = {"navigation": navigation_mode, "perception": "mock",
                                  "planning": "mock", "execution": "mock"}
        self.supervision = supervision
        self.revoked = False
        self.authorized_once = False
        self.navigator = navigator
        self.desk_ports = (perception, planner, executor)
        self.executor = executor
        self.clock = clock
        self.targets = supported_targets
        self.policy = policy or RuntimePolicy()
        self.journal = journal or MissionJournal()
        self.on_accept = on_accept
        self.request: RuntimeRequest | None = None
        self.boot_id = str(uuid4())
        self.debug_sequence = 0
        self.debug_events: deque[dict] = deque(maxlen=2048)
        self.external_phase = "ACCEPTED"
        self.cleaning: CleaningTree | None = None
        self.operation_id: str | None = None
        self.navigation_is_home = False
        self.started_at = clock()
        self.navigation_result = "NOT_STARTED"
        self.return_result = "NOT_STARTED"
        self.stop_result: ModuleResult | None = None
        self.cancel_requested = False
        self.return_after_cancel = False
        self.error_code = self.journal.value("error_code")
        for request in self.journal.unfinished():
            self.journal.finish(RuntimeReport(
                request, "INTERRUPTED", "Runtime restarted; operator confirmation required.",
                failure_code="INTERRUPTED", needs_human_review=True,
                execution_profile=self.journal.profile(request.mission_id),
            ))
            self.error_code = "INTERRUPTED"
            self.journal.set("error_code", self.error_code)
        self.state = RuntimeState.ERROR if self.error_code else RuntimeState.IDLE
        self.last_report = self.journal.latest()
        self.sequence = int(self.journal.value("sequence", "0"))
        self.event_id = ""
        self._last_status: tuple | None = None
        self._refresh_status()

    def readiness(self) -> tuple[bool, str]:
        if self.state == RuntimeState.ERROR:
            return False, self.error_code or "ERROR"
        if self.state != RuntimeState.IDLE:
            return False, "BUSY"
        safe, reason = self._safe_to_drive()
        if not safe:
            return False, reason
        if self.supervision is not None:
            try:
                available, reason = self.supervision.available()
            except Exception:
                return False, "SUPERVISION_ERROR"
            if not available:
                return False, reason or "SUPERVISION_UNAVAILABLE"
        for port in (self.navigator, *self.desk_ports):
            try:
                ready, reason = port.ready()
                if not ready or not port.stopped():
                    return False, reason or "NOT_STOPPED"
            except Exception:
                return False, "ADAPTER_ERROR"
        return True, ""

    def offer(self, request: RuntimeRequest) -> Admission:
        """Admit a supported mission only after readiness and journal persistence.

        An identical request replays its admission and any terminal report without
        restarting work. Reusing its mission ID with different fields is rejected.
        """
        if any(not value.strip() for value in (
            request.mission_id, request.target_id, request.requested_by,
        )):
            return Admission(False, "INVALID_REQUEST")
        existing = self.journal.get(request.mission_id)
        if existing:
            if existing[0] != request:
                return Admission(False, "MISSION_ID_CONFLICT")
            return Admission(True, duplicate=True, report=existing[1])
        if request.mission_type != "clean_desk" or request.target_kind != "SEAT":
            return Admission(False, "UNSUPPORTED_MISSION")
        if request.target_id not in self.targets:
            return Admission(False, "UNKNOWN_TARGET")
        ready, reason = self.readiness()
        if not ready:
            return Admission(False, reason)
        if self.on_accept:
            self.on_accept()
        self.journal.accept(request, self.execution_profile)
        self.request = request
        self.cancel_requested = False
        self.return_after_cancel = False
        self.stop_result = None
        self.operation_id = None
        self.cleaning = None
        self.navigation_result = self.return_result = "NOT_STARTED"
        self.revoked = self.authorized_once = False
        self._enter(RuntimeState.NAVIGATE_TO_TARGET)
        if self.supervision is not None:
            try:
                self.supervision.acquire(request.mission_id)
            except Exception as exc:
                self._revoke(f"lease request failed: {exc}")
        return Admission(True)

    def cancel(self, mission_id: str) -> Admission:
        """Request cancellation or replay an existing terminal report.

        Acceptance confirms the request, not physical stop; tick() must continue
        polling the adapters until stop is confirmed or the stop deadline expires.
        """
        existing = self.journal.get(mission_id)
        if existing and existing[1]:
            return Admission(True, duplicate=True, report=existing[1])
        if self.request is None or self.request.mission_id != mission_id:
            return Admission(False, "NO_ACTIVE_MISSION")
        if self.state in (RuntimeState.CANCELLING, RuntimeState.ERROR):
            return Admission(self.state == RuntimeState.CANCELLING, "ALREADY_STOPPING")
        self.cancel_requested = True
        self.return_after_cancel = (
            self.policy.post_mission == "return_home" and self.state != RuntimeState.RETURN_HOME
        )
        self._begin_stop(ModuleResult(False, ResultStatus.CANCELLED, message="Mission cancelled."))
        return Admission(True)

    def reset_error(self) -> Admission:
        if self.state != RuntimeState.ERROR:
            return Admission(False, "NOT_IN_ERROR")
        if self.error_code in ("E_STOP", "HARDWARE_ERROR"):
            return Admission(False, "HARDWARE_RELEASE_REQUIRED")
        if not self._stopped():
            return Admission(False, "NOT_STOPPED")
        safe, reason = self._safe_to_drive()
        if not safe:
            return Admission(False, reason)
        ready, reason = self.navigator.ready()
        if not ready:
            return Admission(False, reason)
        self.error_code = ""
        self.journal.set("error_code", "")
        self.request = None
        self._enter(RuntimeState.IDLE)
        return Admission(True)

    def safety_fault(self, code: FailureCode) -> None:
        if code not in (FailureCode.E_STOP, FailureCode.HARDWARE_ERROR):
            raise ValueError("unknown independent safety fault")
        self.return_after_cancel = False
        if self.request and not self.journal.get(self.request.mission_id)[1]:
            self._begin_stop(ModuleResult.fatal(code, message="Independent safety stop reported."))
        else:
            self.error_code = code.value
            self.journal.set("error_code", self.error_code)
            self._enter(RuntimeState.ERROR)

    def safety_released(self, code: FailureCode) -> Admission:
        # A notification from the safety owner permits a later explicit operator reset.
        if (self.state != RuntimeState.ERROR or self.error_code != code.value
                or code not in (FailureCode.E_STOP, FailureCode.HARDWARE_ERROR)):
            return Admission(False, "NO_MATCHING_SAFETY_FAULT")
        if not self._stopped():
            return Admission(False, "NOT_STOPPED")
        self.error_code = "SAFETY_RELEASED"
        self.journal.set("error_code", self.error_code)
        return Admission(True)

    def supervisor_stop(self, mission_id: str, reason: str) -> Admission:
        """Accept a Supervisor stop request for the active or a finished mission.

        A mission not running here is answered from stop evidence only, because a
        restarted Runtime or Supervisor may still need confirmation of an old lease.
        """
        if self.request is None:
            return Admission(True)
        if self.request.mission_id != mission_id:
            return Admission(False, "OTHER_MISSION_ACTIVE")
        if self.state not in (RuntimeState.CANCELLING, RuntimeState.ERROR):
            self._revoke(reason or "Supervisor requested stop")
        return Admission(True)

    def supervisor_stop_confirmed(self, mission_id: str) -> bool:
        """True only when no work runs here and every operation reports stopped."""
        return self.request is None and self._stopped()

    def tick(self) -> None:
        """Advance pending operations without waiting for ROS or network replies."""
        try:
            self._tick()
        except Exception as exc:
            if self.request and not self.journal.get(self.request.mission_id)[1]:
                failure = ModuleResult.fatal(
                    FailureCode.UNKNOWN_ERROR, message=f"adapter failure: {exc}"
                )
                if self.state == RuntimeState.CANCELLING:
                    # Repeated adapter exceptions cannot extend the cancellation budget.
                    if self.clock() - self.started_at >= self.policy.cancel_timeout:
                        self._finish(failure, force_error=True)
                else:
                    self._begin_stop(failure)
            else:
                self.error_code = "UNKNOWN_ERROR"
                self.journal.set("error_code", self.error_code)
                self._enter(RuntimeState.ERROR)
        self._refresh_status()

    def _tick(self) -> None:
        self.executor.maintain()
        if self.supervision is not None:
            self.supervision.maintain()
        if self.cleaning and self.state in (RuntimeState.CANCELLING, RuntimeState.ERROR):
            self.cleaning.settle()
        if self.state == RuntimeState.ERROR and self.operation_id:
            self.navigator.poll(self.operation_id)
        if self.state in (RuntimeState.IDLE, RuntimeState.ERROR):
            return
        assert self.request is not None
        if self.state != RuntimeState.CANCELLING and not self._authorized():
            return
        if self.state == RuntimeState.CANCELLING:
            # Poll cancellation too: a cancel request alone is never a stopped acknowledgement.
            if self.operation_id:
                result = self.navigator.poll(self.operation_id)
                if result:
                    if self.navigation_is_home:
                        self.return_result = result.status.value
                    else:
                        self.navigation_result = result.status.value
            if self._stopped():
                self.operation_id = None
                safe, _ = self._safe_to_drive()
                if not safe:
                    self._finish(self.stop_result, force_error=True)
                elif self.return_after_cancel and self.navigator.ready()[0]:
                    self.return_after_cancel = False
                    self._enter(RuntimeState.RETURN_HOME)
                else:
                    self._finish(self.stop_result)
            elif self.clock() - self.started_at >= self.policy.cancel_timeout:
                failure = (replace(self.stop_result, message="Stop acknowledgement timed out.")
                           if self.stop_result and self.stop_result.status == ResultStatus.FATAL
                           else ModuleResult.fatal(FailureCode.TIMEOUT,
                                                   message="Stop acknowledgement timed out."))
                self._finish(failure, force_error=True)
            return
        if self.state in (RuntimeState.NAVIGATE_TO_TARGET, RuntimeState.RETURN_HOME):
            returning = self.state == RuntimeState.RETURN_HOME
            if self.operation_id is None:
                safe, reason = self._safe_to_drive()
                if not safe:
                    self._begin_stop(ModuleResult.blocked(FailureCode.SKILL_FAIL, message=reason))
                    return
                self.navigation_is_home = returning
                self.operation_id = self.navigator.start(
                    "home" if returning else self.request.target_id
                )
                self.observe("action_start", {"module": "navigation",
                             "operation_id": self.operation_id,
                             "target": "home" if returning else self.request.target_id})
            result = self.navigator.poll(self.operation_id)
            if result is None:
                if self.clock() - self.started_at >= self.policy.navigation_timeout:
                    self._begin_stop(ModuleResult.failed(
                        FailureCode.TIMEOUT, message="Navigation timed out."
                    ))
                return
            self.observe("action_result", {"module": "navigation",
                         "operation_id": self.operation_id, "status": result.status.value,
                         "data": result.data, "message": result.message})
            if returning:
                self.return_result = result.status.value
            else:
                self.navigation_result = result.status.value
            if result.status != ResultStatus.OK or not self.navigator.stopped():
                self._begin_stop(result if result.status != ResultStatus.OK else
                                 ModuleResult.fatal(FailureCode.NAVIGATION_FAIL,
                                                    message="Base is not stopped."))
                return
            self.operation_id = None
            if returning:
                self._finish(self.stop_result if self.cancel_requested else None)
            else:
                self.cleaning = CleaningTree(
                    perception=self.desk_ports[0], planner=self.desk_ports[1],
                    executor=self.desk_ports[2], clock=self.clock, policy=self.policy,
                    mission_id=self.request.mission_id, observer=self.observe,
                )
                self._enter(RuntimeState.WORKING)
        elif self.state == RuntimeState.WORKING:
            assert self.cleaning is not None
            if self.clock() - self.started_at >= self.policy.cleaning_timeout:
                self._begin_stop(ModuleResult.failed(FailureCode.TIMEOUT,
                                                    message="Cleaning budget exhausted."))
                return
            status = self.cleaning.tick()
            if status == py_trees.common.Status.SUCCESS:
                self._enter(RuntimeState.POST_MISSION)
            elif status == py_trees.common.Status.FAILURE:
                self._begin_stop(self.cleaning.context.failure or ModuleResult.failed(
                    FailureCode.UNKNOWN_ERROR, message="Cleaning BT failed."
                ))
        elif self.state == RuntimeState.POST_MISSION:
            if self.policy.post_mission == "return_home":
                self._enter(RuntimeState.RETURN_HOME)
            else:
                self.return_result = "NOT_REQUESTED"
                self._finish()

    def _authorized(self) -> bool:
        """Gate every active tick on the Supervisor lease; wait only before first motion."""
        if self.supervision is None:
            return True
        status, reason = self.supervision.authorization()
        if status == Authorization.GRANTED:
            self.authorized_once = True
            return True
        if (status == Authorization.PENDING and not self.authorized_once
                and self.state == RuntimeState.NAVIGATE_TO_TARGET and self.operation_id is None
                and self.clock() - self.started_at < self.policy.permission_timeout):
            return False
        self._revoke(reason or status.value)
        return False

    def _revoke(self, reason: str) -> None:
        self.revoked = True
        self.return_after_cancel = False
        self._begin_stop(ModuleResult.blocked(FailureCode.PERMISSION_REVOKED,
                                              message=f"Supervisor authority unavailable: {reason}"))

    def _begin_stop(self, result: ModuleResult) -> None:
        self.stop_result = result
        self.observe("cancel_requested", {"operation_id": self.operation_id,
                                         "reason": result.message})
        if self.state != RuntimeState.CANCELLING:
            self._enter(RuntimeState.CANCELLING)
        for halt in (
            lambda: self.navigator.cancel(self.operation_id) if self.operation_id else None,
            lambda: self.cleaning.halt() if self.cleaning else None,
        ):
            try:
                halt()
            except Exception as exc:
                if result.status != ResultStatus.FATAL:
                    self.stop_result = ModuleResult.fatal(
                        FailureCode.UNKNOWN_ERROR, message=f"stop adapter failure: {exc}"
                    )

    def _stopped(self) -> bool:
        try:
            return all(port.stopped() for port in (self.navigator, *self.desk_ports))
        except Exception:
            return False

    def _safe_to_drive(self) -> tuple[bool, str]:
        try:
            return self.executor.safe_to_drive()
        except Exception:
            return False, "MANIPULATION_STATE_UNAVAILABLE"

    def _enter(self, state: RuntimeState) -> None:
        previous = self.state.value
        self.state = state
        self.observe("transition", {"from": previous, "to": state.value,
                                   "active_request": asdict(self.request) if self.request else None})
        self.external_phase = {
            RuntimeState.NAVIGATE_TO_TARGET: "NAVIGATING", RuntimeState.WORKING: "WORKING",
            RuntimeState.POST_MISSION: "WORKING", RuntimeState.RETURN_HOME: "RETURNING",
        }.get(state, self.external_phase)
        self.started_at = self.clock()
        self._refresh_status()

    def _finish(self, result: ModuleResult | None = None, force_error: bool = False) -> None:
        assert self.request is not None
        context = self.cleaning.context if self.cleaning else None
        force_error |= not self._safe_to_drive()[0]
        if result:
            outcome = ("CANCELLED" if result.status == ResultStatus.CANCELLED else
                       "BLOCKED" if result.status == ResultStatus.BLOCKED else "FAILED")
        else:
            outcome = ("HUMAN_REVIEW_REQUIRED" if context and context.needs_review else
                       "PARTIAL_SUCCESS" if context and context.skipped else "SUCCESS")
        report = RuntimeReport(
            self.request, outcome, result.message if result else "Mission flow completed.",
            failure_code=(result.failure_code.value if result and result.failure_code else
                          "MANIPULATION_UNSAFE" if not self._safe_to_drive()[0] else ""),
            completed_tasks=list(context.completed) if context else [],
            skipped_tasks=list(context.skipped) if context else [],
            failed_task=(context.proposal.object_id if context and context.proposal and result else ""),
            needs_human_review=bool(force_error or result and result.status == ResultStatus.FATAL
                                   or context and context.needs_review
                                   or self.revoked and self.cleaning is not None),
            before_observation=context.before if context else "",
            after_observation=context.after if context else "",
            navigation_result=self.navigation_result, return_result=self.return_result,
            actions=list(context.records) if context else [],
            execution_profile=self.journal.profile(self.request.mission_id),
        )
        self.journal.finish(report)
        self.last_report = report
        if self.supervision is not None:
            try:
                # Only confirmed stops return the lease; otherwise the Supervisor
                # lets it expire and verifies the stop itself.
                self.supervision.release() if self._stopped() else self.supervision.abandon()
            except Exception:
                pass
        self.observe("mission_result", report.to_dict())
        fatal = force_error or result is not None and result.status == ResultStatus.FATAL
        self.error_code = report.failure_code or "ERROR" if fatal else ""
        self.journal.set("error_code", self.error_code)
        self.request = None
        self._enter(RuntimeState.ERROR if fatal else RuntimeState.IDLE)

    def _refresh_status(self) -> None:
        ready, reason = self.readiness()
        signature = (self.state, self.request, ready, reason,
                     self.cleaning.context.stage if self.cleaning else "")
        if signature != self._last_status:
            self.sequence += 1
            self.journal.set("sequence", str(self.sequence))
            self.event_id = str(uuid4())
            self._last_status = signature

    def snapshot(self, *, include_completed: bool = True) -> dict:
        self._refresh_status()
        ready, reason = self.readiness()
        phase = self.external_phase if self.request else "TERMINAL"
        return {
            "boot_id": self.boot_id,
            "event_id": self.event_id, "sequence": self.sequence,
            "state": self.state.value, "phase": phase,
            "robot_state": "ERROR" if self.state == RuntimeState.ERROR else
                           "BUSY" if self.request else "IDLE",
            "ready": ready, "reason": reason,
            "bt_stage": self.cleaning.context.stage if self.cleaning else "",
            "before_observation": self.cleaning.context.before if self.cleaning else "",
            "active_request": asdict(self.request) if self.request else None,
            "last_result": self.last_report.to_dict() if self.last_report else None,
            "completed_reports": [report.to_dict() for report in self.journal.reports()] if include_completed else [],
            "supported_seat_ids": sorted(self.targets),
            "execution_profile": dict(self.execution_profile),
            "supervision": "required" if self.supervision is not None else "disabled",
        }

    def _supervision_snapshot(self) -> dict | None:
        if self.supervision is None:
            return None
        try:
            available, reason = self.supervision.available()
            status, detail = self.supervision.authorization()
            return {"available": available, "reason": reason, "authorization": status.value,
                    "detail": detail, "revoked": self.revoked}
        except Exception as exc:
            return {"error": str(exc)}

    def observe(self, kind: str, data: dict) -> None:
        """Bounded in-memory observation; no I/O or mission control from consumers."""
        self.debug_sequence += 1
        proposal = self.cleaning.context.proposal if self.cleaning else None
        self.debug_events.append({
            "boot_id": self.boot_id, "sequence": self.debug_sequence,
            "monotonic_time": self.clock(), "kind": kind,
            "mission_id": self.request.mission_id if self.request else
                          self.last_report.request.mission_id if self.last_report else "",
            "task_id": proposal.task_id if proposal else "",
            "execution_id": proposal.execution_id if proposal else "", "data": data,
        })

    def debug_snapshot(self) -> dict:
        snapshot = self.snapshot(include_completed=False)
        snapshot.pop("completed_reports", None)
        modules = {}
        for name, port in zip(("navigation", "perception", "planning", "execution"),
                              (self.navigator, *self.desk_ports)):
            try:
                ready, reason = port.ready()
                modules[name] = {"ready": ready, "reason": reason, "stopped": port.stopped()}
            except Exception as exc:
                modules[name] = {"ready": None, "stopped": None, "reason": str(exc)}
        safe, reason = self._safe_to_drive()
        proposal = self.cleaning.context.proposal if self.cleaning else None
        outcome = getattr(self.executor, "outcome", None)
        snapshot.update({"event_sequence": self.debug_sequence,
            "fsm_states": [state.value for state in RuntimeState],
            "fsm_edges": [[a, b] for a, b in (
                ("IDLE", "NAVIGATE_TO_TARGET"), ("NAVIGATE_TO_TARGET", "WORKING"),
                ("WORKING", "POST_MISSION"), ("POST_MISSION", "RETURN_HOME"),
                ("POST_MISSION", "IDLE"), ("RETURN_HOME", "IDLE"),
                ("NAVIGATE_TO_TARGET", "CANCELLING"), ("WORKING", "CANCELLING"),
                ("RETURN_HOME", "CANCELLING"), ("POST_MISSION", "CANCELLING"),
                ("CANCELLING", "RETURN_HOME"), ("CANCELLING", "IDLE"),
                ("CANCELLING", "ERROR"), ("IDLE", "ERROR"), ("ERROR", "IDLE"))],
            "bt_nodes": self.cleaning.debug_nodes() if self.cleaning else [],
            "modules": modules, "safe_to_drive": safe, "drive_block_reason": reason,
            "supervision": self._supervision_snapshot(),
            "proposal": asdict(proposal) if proposal else None,
            "manipulation": asdict(outcome) if is_dataclass(outcome) else None,
            "navigation": {"operation_id": self.operation_id,
                "target": "home" if self.navigation_is_home else
                          self.request.target_id if self.request else None},
        })
        return snapshot
