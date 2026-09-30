"""Tick-driven execution with bounded cancellation and durable physical progress."""

from __future__ import annotations

from dataclasses import replace
import threading
import time
from typing import Callable

from .models import Error, Goal, ObjectState, Placement, Record, RecordState, Result, Stage, Status
from .ports import Evidence, ManipulationPort
from .store import DuplicateExecution, ExecutionStore, StoreError, inhibits_execution


NORMAL_STAGES = tuple(stage for stage in Stage if stage not in (Stage.STOPPING, Stage.FINALIZING))
ATOMIC_STAGES = (Stage.GRASPING, Stage.LIFTING, Stage.PLACING)
MOTION_STAGES = NORMAL_STAGES[2:8]
FAULTS = (Error.HARDWARE_ERROR, Error.E_STOP, Error.STOP_UNCONFIRMED, Error.INTERNAL_ERROR)


class ExecutionCore:
    def __init__(self, port: ManipulationPort, store: ExecutionStore,
                 timeout: Callable[[Stage], float], *,
                 clock: Callable[[], float] = time.monotonic,
                 wall_clock_ns: Callable[[], int] = time.time_ns) -> None:
        self.lock = threading.RLock()
        self.port, self.store, self.timeout = port, store, timeout
        self.clock, self.wall_clock_ns = clock, wall_clock_ns
        self.events = store.recover(wall_clock_ns())
        self.inhibited = any(inhibits_execution(record) for record in store.all_records())
        self.record: Record | None = None
        self._next_stage: Stage | None = None
        self._started_at = 0.0
        self._running = False
        self._cancel_stage: str | None = None
        self._pending: tuple[Status, Error, str, str] | None = None
        self._storage_failed = False

    def accept(self, goal: Goal) -> tuple[bool, str]:
        with self.lock:
            if not goal.valid():
                return False, 'Invalid IDs, object_id or unsupported skill'
            if self.inhibited:
                return False, 'Execution inhibited; human confirmation is required'
            if self.record is not None and self.record.result is None:
                return False, 'Another execution is active'
            try:
                existing = self.store.get(goal.execution_id)
                if existing is not None:
                    return False, ('Duplicate execution_id' if existing.goal == goal
                                   else 'execution_id conflicts with recorded arguments')
                now_ns = self.wall_clock_ns()
                record = Record(goal, accepted_at_ns=now_ns, updated_at_ns=now_ns,
                                message='Goal accepted; motion has not started')
                self.store.save(record, create=True)
            except DuplicateExecution:
                return False, 'Duplicate execution_id'
            except StoreError as exc:
                self.inhibited = True
                return False, f'Execution recording unavailable: {exc}'
            self.record = record
            self.events.append(record)
            self._next_stage = Stage.VALIDATING
            self._running, self._storage_failed = False, False
            self._cancel_stage, self._pending = None, None
            return True, 'Accepted'

    def request_cancel(self, execution_id: str) -> bool:
        with self.lock:
            if (self.record is None or self.record.goal.execution_id != execution_id
                    or self.record.result is not None):
                return False
            if self._cancel_stage is None:
                self._cancel_stage = self.record.stage.value
                self._persist(replace(self.record, message='Cancellation accepted'))
            return True

    def get(self, execution_id: str) -> Record | None:
        with self.lock:
            if self.record is not None and self.record.goal.execution_id == execution_id:
                return self.record
            return self.store.get(execution_id)

    def drain_events(self) -> list[Record]:
        with self.lock:
            events, self.events = self.events, []
            return events

    def _persist(self, record: Record) -> bool:
        record = replace(record, revision=record.revision + 1, updated_at_ns=self.wall_clock_ns())
        self.record = record
        if not self._storage_failed:
            try:
                self.store.save(record)
            except StoreError as exc:
                self._storage_failed = self.inhibited = True
                self.record = replace(record, record_state=RecordState.RECORDING_FAILED,
                                      human_confirmation_required=True,
                                      message=f'Execution recording failed: {exc}')
        if self._storage_failed:
            result = self.record.result
            if result is not None:
                error = (result.error_code if result.status == Status.FATAL
                         else Error.INTERNAL_ERROR)
                result = replace(result, status=Status.FATAL, error_code=error,
                                 failed_stage=result.failed_stage or Stage.FINALIZING.value,
                                 retryable=False,
                                 message=f'{result.message}; durable outcome unavailable')
            self.record = replace(self.record, result=result,
                                  record_state=RecordState.RECORDING_FAILED,
                                  human_confirmation_required=True)
        self.events.append(self.record)
        return not self._storage_failed

    def tick(self) -> None:
        with self.lock:
            if self.record is None or self.record.result is not None:
                return
            now = self.clock()
            try:
                self._tick(now)
            except Exception as exc:
                # Unexpected adapter failures are faults, never fabricated success.
                self.inhibited = True
                if self.record.stage == Stage.STOPPING:
                    self._finish(Status.FATAL, Error.STOP_UNCONFIRMED,
                                 self._pending[2] if self._pending else Stage.STOPPING.value,
                                 f'Stop adapter failed: {exc}')
                else:
                    if self._running and self.record.stage in ATOMIC_STAGES:
                        self._persist(replace(self.record, object_state=ObjectState.UNKNOWN))
                    self._stop(Status.FATAL, Error.INTERNAL_ERROR, self.record.stage.value,
                               f'Adapter failed: {exc}', now)

    def _tick(self, now: float) -> None:
        record = self.record
        if record.stage == Stage.STOPPING and self._running:
            fault = self.port.fault(now)
            if fault:
                self.inhibited = True
                self._pending = (Status.FATAL, fault, self._pending[2],
                                 f'{self._pending[3]}; fault during stop: {fault.value}')
            evidence = self.port.poll(now)
            if now - self._started_at >= self.timeout(Stage.STOPPING):
                self._finish(Status.FATAL, Error.STOP_UNCONFIRMED, self._pending[2],
                             f'{self._pending[3]}; stop deadline exceeded')
            elif evidence is not None and evidence.stop_confirmed and evidence.error == Error.NONE:
                self._apply(evidence)
                self._finish(*self._pending)
            elif evidence is not None:
                self._finish(Status.FATAL, Error.STOP_UNCONFIRMED, self._pending[2],
                             f'{self._pending[3]}; stop evidence unavailable')
            return
        fault = self.port.fault(now) if self._running else None
        if fault:
            if record.stage in ATOMIC_STAGES:
                self._persist(replace(record, object_state=ObjectState.UNKNOWN))
            self._stop(Status.FATAL, fault, record.stage.value, f'Mock/local fault: {fault.value}', now)
            return
        if self._storage_failed:
            if self._running and record.stage in ATOMIC_STAGES:
                self._persist(replace(record, object_state=ObjectState.UNKNOWN))
            self._stop(Status.FATAL, Error.INTERNAL_ERROR, record.stage.value,
                       'Execution recording failed; durable outcome unavailable', now)
            return
        if self._cancel_stage is not None and (not self._running or record.stage not in ATOMIC_STAGES):
            self._stop(Status.CANCELED, Error.CANCELED, self._cancel_stage,
                       'Canceled at checkpoint; no recovery motion requested', now)
            return
        if self._next_stage is not None:
            stage, self._next_stage = self._next_stage, None
            motion = stage in MOTION_STAGES
            if not self._persist(replace(record, stage=stage,
                                         stop_confirmed=False if motion else record.stop_confirmed,
                                         arm_recovered=False if motion else record.arm_recovered,
                                         message=f'Starting {stage.value}')):
                return
            self._started_at, self._running = now, True
            self.port.begin(stage, record.goal, now)
            return
        evidence = self.port.poll(now)
        # Late evidence cannot turn a timed-out step into success.
        if now - self._started_at >= self.timeout(record.stage):
            if record.stage in ATOMIC_STAGES:
                self._persist(replace(record, object_state=ObjectState.UNKNOWN))
            elif record.stage == Stage.VERIFYING_PLACEMENT:
                self._persist(replace(record, placement_state=Placement.UNKNOWN))
            error = (Error.VERIFICATION_TIMEOUT if record.stage == Stage.VERIFYING_PLACEMENT
                     else Error.TIMEOUT)
            self._fail(error, record.stage.value, 'Stage deadline exceeded', now)
            return
        if evidence is None:
            return
        # A poll outcome ends the current operation, even if storing its evidence fails.
        # Keep that fresh physical evidence when entering the stop path.
        self._running = False
        self._apply(evidence)
        if self._storage_failed:
            return
        if evidence.error != Error.NONE:
            self._fail(evidence.error, record.stage.value, evidence.message, now)
            return
        self._persist(replace(self.record, last_completed_stage=record.stage.value,
                              message=f'Stage completed: {record.stage.value}'))
        if self._cancel_stage is not None:
            self._stop(Status.CANCELED, Error.CANCELED, self._cancel_stage,
                       'Canceled after atomic checkpoint', now)
        elif record.stage == Stage.VERIFYING_PLACEMENT:
            current = self.record
            if (current.object_state == ObjectState.LEFT_GRIPPER
                    and current.placement_state == Placement.CONFIRMED
                    and current.selected_arm and current.stop_confirmed and current.arm_recovered):
                self._finish(Status.SUCCESS, Error.NONE, '', 'Mock collection verified')
            else:
                self._stop(Status.FATAL, Error.INTERNAL_ERROR, record.stage.value,
                           'Success evidence is incomplete', now)
        else:
            self._next_stage = NORMAL_STAGES[NORMAL_STAGES.index(record.stage) + 1]

    def _apply(self, evidence: Evidence) -> None:
        updates = {name: getattr(evidence, name) for name in (
            'object_state', 'placement_state', 'selected_arm', 'stop_confirmed', 'arm_recovered',
        ) if getattr(evidence, name) is not None}
        self._persist(replace(self.record, **updates, evidence_at_ns=self.wall_clock_ns(),
                              message=f'Observation received: {evidence.message}'))

    def _fail(self, error: Error, stage: str, message: str, now: float) -> None:
        if error in FAULTS:
            self._stop(Status.FATAL, error, stage, message, now)
        elif self.record.stage in (Stage.VALIDATING, Stage.PREPARING_TARGET):
            if self._cancel_stage is not None:
                self._stop(Status.CANCELED, Error.CANCELED, self._cancel_stage, message, now)
            else:
                self._finish(Status.BLOCKED, error, stage, message)
        else:
            self._stop(Status.FAILED, error, stage, message, now)

    def _stop(self, status: Status, error: Error, stage: str, message: str, now: float) -> None:
        self._pending = (status, error, stage, message)
        self._next_stage = None
        if status == Status.FATAL:
            self.inhibited = True
        self._persist(replace(self.record, stage=Stage.STOPPING, stop_confirmed=False,
                              message=message, human_confirmation_required=self.inhibited))
        self._started_at, self._running = now, True
        try:
            self.port.stop(now)
        except Exception as exc:
            self._finish(Status.FATAL, Error.STOP_UNCONFIRMED, stage,
                         f'{message}; stop command failed: {exc}')

    def _finish(self, status: Status, error: Error, stage: str, message: str) -> None:
        self._persist(replace(self.record, stage=Stage.FINALIZING,
                              message=f'Finalizing result: {message}'))
        if self._storage_failed:
            if status != Status.FATAL:
                error = Error.INTERNAL_ERROR
            status = Status.FATAL
            message = f'{message}; execution recording failed; durable outcome unavailable'
        record = self.record
        attention = (self._storage_failed or status == Status.FATAL
                     or record.object_state in (ObjectState.HELD, ObjectState.UNKNOWN))
        result = Result(record.goal.execution_id, record.execution_profile, status, error, stage,
                        record.last_completed_stage, record.object_state, record.placement_state,
                        record.selected_arm, record.stop_confirmed, record.arm_recovered,
                        status == Status.BLOCKED and record.stop_confirmed and not attention, message)
        final = replace(record, result=result, human_confirmation_required=attention,
                        record_state=RecordState.RECORDING_FAILED if self._storage_failed
                        else RecordState.FINISHED,
                        message=f'Execution finished: {status.value}; {message}')
        self._persist(final)
        self.inhibited = self.inhibited or inhibits_execution(self.record)
        self._running = False
