"""Action lifecycle and durable observations. Physical order belongs to the XML tree."""
from __future__ import annotations

from dataclasses import replace
import threading
import time
import weakref
import math
from typing import Callable
from uuid import uuid4

from cleany_skill_executor.manipulation.models import (
    Error, Goal, ObjectState, Placement, Record, RecordState, Result, Stage, Status,
)
from cleany_skill_executor.manipulation.steps import STAGE_STEPS
from cleany_skill_executor.manipulation.store import StoreError, inhibits_execution
from .store import BTExecutionStore
from .backend import Backend, Observation

NODE_STAGE = {step.node: stage for stage, steps in STAGE_STEPS.items() for step in steps}
NODE_STAGE.update(StopAndAssess=Stage.STOPPING, FinalizeSuccess=Stage.FINALIZING,
                  FinalizeFailure=Stage.FINALIZING)
ATOMIC = {Stage.GRASPING, Stage.LIFTING, Stage.PLACING}
PREPARATION = {Stage.VALIDATING, Stage.PREPARING_TARGET}


class ExecutionCore:
    def __init__(self, backend: Backend, store: BTExecutionStore, tree_xml: str,
                 runner_factory: Callable, *, timeout_sec: float = 180.0,
                 stop_timeout_sec: float = 15.0, clock: Callable = time.monotonic) -> None:
        self.backend, self.store, self.tree_xml = backend, store, tree_xml
        self.runner_factory, self.clock = runner_factory, clock
        self.timeout_sec, self.stop_timeout_sec = timeout_sec, stop_timeout_sec
        if any(not math.isfinite(value) or value <= 0 for value in (timeout_sec, stop_timeout_sec)):
            raise ValueError('Operation and stop deadlines must be finite and positive')
        self.recording_failed = False
        self.lock = threading.RLock()
        self.record: Record | None = None
        self.tree = None
        self.events: list[Record] = store.recover(time.time_ns())
        self.inhibited = any(inhibits_execution(record) for record in store.all_records())
        self.cancel_requested = False
        self.failure: Observation | None = None
        self.failed_stage = ''
        self.failed_substage = ''
        self.ops: dict[str, tuple[str, float, str | Observation]] = {}
        self.completed_ops: set[str] = set()
        self.motion_started = False
        self.transitions: list[dict] = []

    def accept(self, goal: Goal) -> tuple[bool, str]:
        if (self.inhibited or not self.backend.ready()
                or (self.record is not None and self.record.result is None)):
            return False, 'Execution inhibited, busy or awaiting physical-state confirmation'
        if not goal.valid():
            return False, 'Invalid goal'
        try:
            if self.store.get(goal.execution_id) is not None:
                return False, 'Duplicate execution_id'
            now = time.time_ns()
            record = Record(goal, execution_profile='mujoco', accepted_at_ns=now,
                            updated_at_ns=now, message='Accepted; awaiting BT validation')
            # Construct before acceptance, so malformed XML cannot leave an
            # accepted goal without an executable tree. No nodes are ticked here.
            tree = self.runner_factory(self.tree_xml, weakref.proxy(self), goal.execution_id)
            self.store.save(record, create=True)
        except StoreError as error:
            self.inhibited = True
            self.recording_failed = True
            return False, str(error)
        self.record, self.tree = record, tree
        self.backend.begin(goal)
        self.cancel_requested, self.failure = False, None
        self.failed_stage = self.failed_substage = ''
        self.motion_started = False
        self.ops.clear()
        self.completed_ops.clear()
        self.events.append(record)
        return True, ''

    def get(self, execution_id: str) -> Record | None:
        return self.store.get(execution_id)

    def request_cancel(self, execution_id: str) -> bool:
        if (self.record is None or self.record.goal.execution_id != execution_id
                or self.record.result is not None or self.record.stage == Stage.FINALIZING):
            return False
        self.cancel_requested = True
        return True

    def _save(self, **changes) -> bool:
        record = replace(self.record, **changes, revision=self.record.revision + 1,
                         updated_at_ns=time.time_ns())
        try:
            self.store.save(record)
        except StoreError as error:
            self.inhibited = True
            self.recording_failed = True
            self.record = replace(record, record_state=RecordState.RECORDING_FAILED,
                                  human_confirmation_required=True,
                                  message=f'Recording failed: {error}')
            return False
        self.record = record
        self.events.append(record)
        return True

    def _fail(self, node: str, observation: Observation) -> None:
        if self.failure is None:
            self.failure = observation
            self.failed_stage, self.failed_substage = NODE_STAGE[node].value, node
            changes = {}
            if (observation.error == Error.GRASP_LOST
                    or (self.record.object_state == ObjectState.HELD and observation.error in
                        (Error.HARDWARE_ERROR, Error.MOTION_FAILED, Error.TIMEOUT))):
                changes['object_state'] = ObjectState.UNKNOWN
            self._save(failed_substage=node, message=observation.message, **changes)

    def start(self, node: str, execution_id: str) -> str:
        if self.record.goal.execution_id != execution_id:
            raise RuntimeError('Execution identity mismatch')
        token = uuid4().hex
        stage = NODE_STAGE[node]
        failure_path = node in ('StopAndAssess', 'FinalizeFailure')
        same_atomic = stage in ATOMIC and self.record.stage == stage
        blocked = self.cancel_requested and not same_atomic and not failure_path
        if self.failure is not None and not failure_path:
            self.ops[token] = (node, self.clock(), self.failure)
            return token
        if blocked:
            observation = Observation(False, Error.CANCELED, 'Canceled at atomic checkpoint')
            self._fail(node, observation)
            self.ops[token] = (node, self.clock(), observation)
            return token
        if not self._save(stage=stage, substage=node, message=f'BT started {node}') and not failure_path:
            observation = Observation(False, Error.INTERNAL_ERROR, 'Checkpoint could not be persisted')
            self._fail(node, observation)
            self.ops[token] = (node, self.clock(), observation)
            return token
        if node in ('FinalizeSuccess', 'FinalizeFailure'):
            observation = Observation()
        else:
            if stage not in PREPARATION and not failure_path:
                self.motion_started = True
            if node in ('GraspObject', 'OpenGripperAtDestination'):
                if not self._save(object_state=ObjectState.UNKNOWN):
                    observation = Observation(False, Error.INTERNAL_ERROR, 'Physical checkpoint could not be persisted')
                    self._fail(node, observation)
                    self.ops[token] = (node, self.clock(), observation)
                    return token
            observation = self.backend.start(node, execution_id)
        self.ops[token] = (node, self.clock(), observation)
        return token

    def poll(self, token: str) -> str:
        node, started, operation = self.ops[token]
        if token in self.completed_ops:
            return 'FAILURE' if node == self.failed_substage else 'SUCCESS'
        observation = operation if isinstance(operation, Observation) else self.backend.poll(operation)
        limit = self.stop_timeout_sec if node == 'StopAndAssess' else self.timeout_sec
        if observation is None and self.clock() - started >= limit:
            self.backend.cancel(operation)
            observation = Observation(False, Error.TIMEOUT, f'{node} exceeded wall-clock deadline')
        if observation is None:
            return 'RUNNING'
        self.completed_ops.add(token)
        if node in ('FinalizeSuccess', 'FinalizeFailure'):
            observation = self._finalize(success=node == 'FinalizeSuccess')
            return 'SUCCESS' if observation.success else 'FAILURE'
        if node == 'StopAndAssess':
            # Assessment completion is SUCCESS in the BT even when physical
            # stop failed, so FinalizeFailure always persists an honest Result.
            self._save(stop_confirmed=observation.success and observation.stop_confirmed is True,
                       message=observation.message)
            return 'SUCCESS'
        if not observation.success:
            changes = {name: getattr(observation, name) for name in
                       ('object_state', 'placement_state')
                       if getattr(observation, name) is not None}
            if changes:
                self._save(**changes)
            self._fail(node, observation)
            return 'FAILURE'
        changes = {name: getattr(observation, name) for name in (
            'selected_arm', 'object_state', 'placement_state', 'stop_confirmed', 'arm_recovered')
                   if getattr(observation, name) is not None}
        changes.update(completed_substages=(*self.record.completed_substages, node),
                       evidence_at_ns=time.time_ns(), message=observation.message or f'{node} completed')
        stage = NODE_STAGE[node]
        if node == STAGE_STEPS[stage][-1].node:
            changes['last_completed_stage'] = stage.value
        if not self._save(**changes):
            self._fail(node, Observation(False, Error.INTERNAL_ERROR, 'Observation persistence failed'))
            return 'FAILURE'
        return 'SUCCESS'

    def cancel(self, token: str) -> None:
        _, _, operation = self.ops[token]
        if isinstance(operation, str):
            self.backend.cancel(operation)

    def _finalize(self, *, success: bool) -> Observation:
        record = self.record
        if success and not (record.object_state == ObjectState.LEFT_GRIPPER
                            and record.placement_state == Placement.CONFIRMED
                            and record.arm_recovered and record.stop_confirmed):
            observation = Observation(False, Error.PLACEMENT_NOT_CONFIRMED,
                                      'Success evidence is incomplete')
            self._fail('FinalizeSuccess', observation)
            return observation
        if success:
            status, error = Status.SUCCESS, Error.NONE
        else:
            error = self.failure.error if self.failure else Error.INTERNAL_ERROR
            status = (Status.CANCELED if error == Error.CANCELED else
                      Status.BLOCKED if self.failed_stage in {s.value for s in PREPARATION}
                      else Status.FAILED)
            if self.motion_started and not record.stop_confirmed:
                status, error = Status.FATAL, Error.STOP_UNCONFIRMED
        if self.recording_failed:
            status, error = Status.FATAL, Error.INTERNAL_ERROR
        result = Result(record.goal.execution_id, 'mujoco', status, error,
                        self.failed_stage, record.last_completed_stage, record.object_state,
                        record.placement_state, record.selected_arm, record.stop_confirmed,
                        record.arm_recovered, False,
                        'Verified single-object collection' if success else
                        (self.failure.message if self.failure else 'Execution failed'),
                        failed_substage=self.failed_substage)
        if not self._save(result=result, record_state=(RecordState.RECORDING_FAILED if self.recording_failed
                                                      else RecordState.FINISHED),
                          human_confirmation_required=(status == Status.FATAL or record.object_state in
                                                       (ObjectState.HELD, ObjectState.UNKNOWN))):
            result = replace(result, status=Status.FATAL, error_code=Error.INTERNAL_ERROR,
                             message=self.record.message)
            self.record = replace(self.record, result=result)
            self.events.append(self.record)
        self.inhibited = self.inhibited or inhibits_execution(self.record)
        return Observation()

    def tick(self) -> None:
        if self.tree is not None and self.record.result is None:
            self.tree.tick()
            transitions = self.tree.drain_transitions()
            self.transitions.extend(transitions)
            try:
                self.store.save_transitions(self.record.goal.execution_id, transitions)
            except StoreError:
                self.inhibited = True
                self.recording_failed = True
                # Prevent the next operation; active work must go through stop
                # rather than continuing with a broken journal.
                for token in self.ops.keys() - self.completed_ops:
                    self.cancel(token)
                if self.record.result is None and self.failure is None:
                    self._fail(self.record.substage, Observation(False, Error.INTERNAL_ERROR,
                                                               'BT transition persistence failed'))
                elif self.record.result is not None:
                    result = replace(self.record.result, status=Status.FATAL, error_code=Error.INTERNAL_ERROR,
                                     message='BT transition persistence failed')
                    self._save(result=result, record_state=RecordState.RECORDING_FAILED,
                               human_confirmation_required=True)

    def drain_events(self) -> list[Record]:
        events, self.events = self.events, []
        return events
