"""Action 생명주기와 런타임 관측입니다. 물리 작업 순서는 XML 트리가 관리합니다."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import threading
import time
import weakref
import math
from typing import Callable
from uuid import uuid4

from cleany_skill_executor.manipulation.models import (
    CancelMode, Error, Goal, ObjectState, Placement, Record, RecordState, Result, Stage, Status,
    resolve_cancel_mode,
)
from cleany_skill_executor.manipulation.steps import STAGE_STEPS
from cleany_skill_executor.manipulation.store import DuplicateExecution, inhibits_execution
from .store import BTExecutionJournal, BTExecutionStore
from .backend import Backend, Observation

NODE_STAGE = {step.node: stage for stage, steps in STAGE_STEPS.items() for step in steps}
NODE_STAGE.update(StopAndAssess=Stage.STOPPING, FinalizeSuccess=Stage.FINALIZING,
                  FinalizeFailure=Stage.FINALIZING, StopAfterRecovery=Stage.STOPPING)
ATOMIC = {Stage.GRASPING, Stage.LIFTING, Stage.PLACING}
PREPARATION = {Stage.VALIDATING, Stage.PREPARING_TARGET}
FATAL_ERRORS = {Error.HARDWARE_ERROR, Error.E_STOP, Error.STOP_UNCONFIRMED, Error.INTERNAL_ERROR}
RECOVERY_NODES = {'ReleaseInPlace', 'ReturnArmAfterCancel', 'StopAfterRecovery'}
STOP_NODES = {'StopAndAssess', 'StopAfterRecovery'}
FAILURE_NODES = {*STOP_NODES, *RECOVERY_NODES, 'FinalizeFailure'}


@dataclass
class FaultShutdown:
    deadline: float
    pending: set[str]
    issues: list[str] = field(default_factory=list)
    stop_operation: str | None = None
    stop_observation: Observation | None = None


class ExecutionCore:
    """단일 물체 실행 기록과 native BT의 비동기 작업 생명주기를 관리합니다.

    물리 작업 순서는 XML 트리가 소유합니다. 호출자는 공유 상태에 접근할 때
    lock으로 직렬화하며 backend 연산은 신속히 반환해야 합니다. 생성 시 journal을
    복구하고 미확인 물리 상태가 남으면 새 실행을 금지합니다.
    BT 취소, worker 종료와 정지 확인을 구분하며 기한 초과도 정지 근거로 간주하지
    않습니다. 런타임 fault가 나면 손상된 트리 대신 별도 정지 절차를 진행합니다.
    """

    def __init__(self, backend: Backend, store: BTExecutionStore, tree_xml: str,
                 runner_factory: Callable, *, timeout_sec: float = 180.0,
                 stop_timeout_sec: float = 15.0, clock: Callable = time.monotonic,
                 execution_profile: str = 'mujoco', allow_assumed_placement: bool = False,
                 operation_timeouts: dict[str, float] | None = None) -> None:
        self.backend, self.store, self.tree_xml = backend, BTExecutionJournal(store), tree_xml
        self.runner_factory, self.clock = runner_factory, clock
        self.timeout_sec, self.stop_timeout_sec = timeout_sec, stop_timeout_sec
        self.operation_timeouts = dict(operation_timeouts or {})
        if (set(self.operation_timeouts) - NODE_STAGE.keys()
                or any(not math.isfinite(value) or value <= 0
                       for value in self.operation_timeouts.values())):
            raise ValueError('Operation timeout overrides must name a BT operation and be positive')
        if execution_profile not in ('mujoco', 'jetson'):
            raise ValueError('Unknown execution profile')
        self.execution_profile = execution_profile
        if allow_assumed_placement and execution_profile != 'jetson':
            raise ValueError('Assumed placement is only available for the temporary Jetson mode')
        self.allow_assumed_placement = allow_assumed_placement
        if any(not math.isfinite(value) or value <= 0 for value in (timeout_sec, stop_timeout_sec)):
            raise ValueError('Operation and stop deadlines must be finite and positive')
        self.lock = threading.RLock()
        self.record: Record | None = None
        self.tree = None
        self.events: list[Record] = self.store.recover(time.time_ns())
        self.inhibited = bool(self.recovery_blocks())
        self.cancel_requested = False
        self._timeout_cancel_requested = False
        self.failure: Observation | None = None
        self.failed_stage = ''
        self.failed_substage = ''
        self.ops: dict[str, tuple[str, float, str | Observation]] = {}
        self.completed_ops: set[str] = set()
        self._pending_cancel_results: set[str] = set()
        self._stop_observations: dict[str, Observation] = {}
        self.motion_started = False
        self.transitions: list[dict] = []
        self.skipped_ops: set[str] = set()
        self.recovering = False
        self._fault_shutdown: FaultShutdown | None = None
        self._deferred_fault: Exception | None = None
        self._recovery_in_progress = False
        self.arm_return_active = False

    def recovery_blocks(self) -> list[Record]:
        return [record for record in self.store.all_records()
                if inhibits_execution(record) and not self.store.recovery_confirmed(record)]

    def confirm_recovery(self, execution_id: str, revision: int, *, stopped: bool,
                         gripper_empty: bool, note: str,
                         verify_stationary: Callable[[], bool]) -> tuple[bool, str]:
        with self.lock:
            if self._recovery_in_progress or self.arm_return_active:
                return False, 'Recovery confirmation or arm return is already in progress'
            if self.record is not None and self.record.result is None:
                return False, 'An execution is active; finish or cancel it first'
            record = self.store.get(execution_id)
            if record is None:
                return False, 'Execution not found'
            if record.revision != revision:
                return False, 'Execution revision changed; query it again'
            if record.record_state == RecordState.ACTIVE:
                return False, 'Execution is active; restart recovery cannot acknowledge it'
            if not stopped or not gripper_empty or not note.strip():
                return False, 'Operator must confirm a stopped robot, empty grippers and provide a note'
            if self.store.recovery_confirmed(record):
                return True, 'This execution revision was already acknowledged'
            if not inhibits_execution(record):
                return False, 'This execution does not inhibit new work'
            self._recovery_in_progress = True
        try:
            # Let timer/status callbacks run while the operation node receives
            # fresh feedback. Waiting under core.lock can exhaust executor threads.
            if not self.backend.ready():
                return False, 'Operations or controller Actions have not quiesced'
            stationary = verify_stationary()
            with self.lock:
                if not stationary or not self.backend.ready():
                    return False, 'Fresh stationary joint feedback and terminal Actions are required'
                if self.store.get(execution_id) != record:
                    return False, 'Execution changed during verification; query it again'
                self.store.confirm_recovery(record, note.strip())
                self.inhibited = bool(self.recovery_blocks())
                return True, ('Recovery acknowledged; fresh Goals are allowed' if not self.inhibited
                              else 'Recovery acknowledged; other execution records still inhibit new work')
        except Exception as error:
            return False, f'Recovery confirmation failed: {error}'
        finally:
            with self.lock:
                self._recovery_in_progress = False

    def accept(self, goal: Goal) -> tuple[bool, str]:
        if (self.inhibited or self._recovery_in_progress or self.arm_return_active
                or (self.record is not None and self.record.result is None)):
            return False, 'Execution inhibited, busy or awaiting physical-state confirmation'
        try:
            ready = self.backend.ready()
        except Exception as error:
            self.inhibited = True
            return False, f'Backend readiness unavailable: {error}'
        if not ready:
            return False, 'Execution inhibited, busy or awaiting physical-state confirmation'
        if not goal.valid():
            return False, 'Invalid goal'
        try:
            if self.store.get(goal.execution_id) is not None:
                return False, 'Duplicate execution_id'
            now = time.time_ns()
            record = Record(goal, execution_profile=self.execution_profile, accepted_at_ns=now,
                            updated_at_ns=now, message='Accepted; awaiting BT validation')
            # 잘못된 XML 때문에 실행할 트리 없는 목표가 수락되지 않도록 수락 전에 생성합니다.
            # 여기서는 어떤 노드도 tick하지 않습니다.
            tree = self.runner_factory(self.tree_xml, weakref.proxy(self), goal.execution_id)
            self.store.save(record, create=True)
        except DuplicateExecution as error:
            return False, str(error)
        except Exception as error:
            return False, f'Execution setup failed: {error}'
        self.record, self.tree = record, tree
        self.cancel_requested, self.failure = False, None
        self._timeout_cancel_requested = False
        self.failed_stage = self.failed_substage = ''
        self.motion_started = False
        self.ops.clear()
        self.completed_ops.clear()
        self._pending_cancel_results.clear()
        self._stop_observations.clear()
        self.skipped_ops.clear()
        self.recovering = False
        self._fault_shutdown = None
        self._deferred_fault = None
        self.events.append(record)
        try:
            self.backend.begin(goal)
        except Exception as error:
            # 예약된 실행에도 최종 Action Result를 전달합니다.
            self._begin_fault(error)
        return True, ''

    def get(self, execution_id: str) -> Record | None:
        return self.store.get(execution_id)

    def request_cancel(self, execution_id: str, mode: CancelMode | str | None = None) -> bool:
        if (self.record is None or self.record.goal.execution_id != execution_id
                or self.record.result is not None or self.record.stage == Stage.FINALIZING
                or self._fault_shutdown is not None):
            return False
        try:
            selected = resolve_cancel_mode(self.record.cancel_mode, mode)
        except ValueError:
            return False
        self.cancel_requested = True
        self._save(cancel_mode=selected.value, message=f'Cancellation accepted: {selected.value}')
        if selected != CancelMode.CHECKPOINT:
            try:
                for token, (node, _, operation) in tuple(self.ops.items()):
                    if (token not in self.completed_ops and node not in STOP_NODES
                            and isinstance(operation, str)):
                        self.backend.cancel(operation)
            except Exception as error:
                self._begin_fault(error)
        return True

    def _save(self, **changes) -> None:
        record = replace(self.record, **changes, revision=self.record.revision + 1,
                         updated_at_ns=time.time_ns())
        self.record = record
        self.store.save(record)
        self.events.append(record)

    def _fail(self, node: str, observation: Observation) -> None:
        # 실제 실패는 잠정 취소보다, fault는 일반 실패보다 우선합니다.
        # 이미 관측한 fault를 더 낮은 상태로 바꾸지 않습니다.
        if (self.failure is None
                or (self.failure.error == Error.CANCELED and observation.error != Error.CANCELED)
                or (observation.error in FATAL_ERRORS and self.failure.error not in FATAL_ERRORS)):
            self.failure = observation
            self.failed_stage, self.failed_substage = NODE_STAGE.get(node, self.record.stage).value, node
            changes = {}
            if observation.error in FATAL_ERRORS:
                self.inhibited = True
                changes['human_confirmation_required'] = True
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
        failure_path = node in FAILURE_NODES
        if node in RECOVERY_NODES:
            recover = (self.record.cancel_mode == CancelMode.RETURN_ARM
                       and self.failure is not None and self.failure.error == Error.CANCELED
                       and not self._pending_cancel_results
                       and bool(self.record.selected_arm)
                       and (self.record.stop_confirmed or self.recovering))
            if not recover:
                self.skipped_ops.add(token)
                self.ops[token] = (node, self.clock(), Observation())
                return token
            self.recovering = True
        same_atomic = (self.record.cancel_mode == CancelMode.CHECKPOINT
                       and stage in ATOMIC and self.record.stage == stage)
        blocked = self.cancel_requested and not same_atomic and not failure_path
        if self.failure is not None and not failure_path:
            self.ops[token] = (node, self.clock(), self.failure)
            return token
        if blocked:
            observation = Observation(False, Error.CANCELED, 'Canceled at atomic checkpoint')
            self._fail(node, observation)
            self.ops[token] = (node, self.clock(), observation)
            return token
        changes = {'stop_confirmed': False, 'arm_recovered': False} if node in (
            'ReleaseInPlace', 'ReturnArmAfterCancel') else {}
        self._save(stage=stage, substage=node, message=f'BT started {node}', **changes)
        if node in ('FinalizeSuccess', 'FinalizeFailure'):
            observation = Observation()
        else:
            if (stage not in PREPARATION and not failure_path) or node in (
                    'ReleaseInPlace', 'ReturnArmAfterCancel'):
                self.motion_started = True
            if node in ('GraspObject', 'OpenGripperAtDestination'):
                self._save(object_state=ObjectState.UNKNOWN)
            observation = self.backend.start(node, execution_id)
        self.ops[token] = (node, self.clock(), observation)
        return token

    def poll(self, token: str) -> str:
        """작업 관측을 BT 상태로 바꾸고 취소된 worker의 후속 결과도 추적합니다.

        시간 초과나 선점은 FAILURE를 만들 수 있지만 worker 종료를 입증하지 않습니다.
        아직 결과가 없는 취소 작업은 별도 대기 집합에 남기며 정지 노드는 독립 평가합니다.
        """

        node, started, operation = self.ops[token]
        if token in self.completed_ops:
            return 'FAILURE' if node == self.failed_substage and node not in STOP_NODES else 'SUCCESS'
        if token in self.skipped_ops:
            self.completed_ops.add(token)
            return 'SUCCESS'
        if node in STOP_NODES:
            return self._poll_stop(token, node, started, operation)
        preempted = (self.cancel_requested and node not in STOP_NODES
                     and node not in ('FinalizeSuccess', 'FinalizeFailure')
                     and (self.record.cancel_mode == CancelMode.IMMEDIATE
                          or (self.record.cancel_mode == CancelMode.RETURN_ARM
                              and node not in RECOVERY_NODES)))
        observed = operation if isinstance(operation, Observation) else self.backend.poll(operation)
        if (preempted and not isinstance(operation, Observation)
                and (observed is None or observed.success or observed.error == Error.CANCELED)):
            if isinstance(operation, str):
                self.backend.cancel(operation)
                if observed is None:
                    # 논리적 BT 취소가 worker 종료를 의미하지는 않습니다.
                    self._pending_cancel_results.add(token)
            observation = Observation(False, Error.CANCELED, 'Current operation canceled',
                object_state=(observed.object_state if observed is not None and observed.object_state is not None
                              else ObjectState.UNKNOWN if NODE_STAGE[node] in ATOMIC else None),
                selected_arm=observed.selected_arm if observed is not None else None,
                placement_state=observed.placement_state if observed is not None else None,
                arm_recovered=observed.arm_recovered if observed is not None else None)
        else:
            observation = observed
        if observation is None and self.clock() - started >= self.operation_timeouts.get(node, self.timeout_sec):
            # Requesting cancellation does not prove the worker has ended.
            self._timeout_cancel_requested = True
            self._pending_cancel_results.add(token)
            self.backend.cancel(operation)
            observation = Observation(False, Error.TIMEOUT, f'{node} exceeded wall-clock deadline')
        if observation is None:
            return 'RUNNING'
        self.completed_ops.add(token)
        if node in ('FinalizeSuccess', 'FinalizeFailure'):
            observation = self._finalize(success=node == 'FinalizeSuccess')
            return 'SUCCESS' if observation.success else 'FAILURE'
        if not observation.success:
            self._record_failure(node, observation)
            return 'FAILURE'
        self._record_success(node, observation)
        return 'SUCCESS'

    def _record_failure(self, node: str, observation: Observation) -> None:
        changes = {name: getattr(observation, name) for name in
                   ('object_state', 'placement_state', 'selected_arm', 'arm_recovered')
                   if getattr(observation, name) is not None}
        if changes:
            self._save(**changes)
        self._fail(node, observation)

    def _record_success(self, node: str, observation: Observation) -> None:
        changes = {name: getattr(observation, name) for name in (
            'selected_arm', 'object_state', 'placement_state', 'stop_confirmed', 'arm_recovered')
                   if getattr(observation, name) is not None}
        changes.update(completed_substages=(*self.record.completed_substages, node),
                       evidence_at_ns=time.time_ns(), message=observation.message or f'{node} completed')
        stage = NODE_STAGE[node]
        if node == STAGE_STEPS[stage][-1].node and node not in RECOVERY_NODES:
            changes['last_completed_stage'] = stage.value
        self._save(**changes)

    def _collect_cancel_results(self) -> None:
        for token in tuple(self._pending_cancel_results):
            node, _, operation = self.ops[token]
            observation = operation if isinstance(operation, Observation) else self.backend.poll(operation)
            if observation is None:
                continue
            self._pending_cancel_results.remove(token)
            if observation.success:
                # 물리적 완료 상태는 보존하되 취소 전 근거를 재사용하지 않고
                # 취소 후 정지를 별도로 평가합니다.
                self._record_success(node, replace(observation, stop_confirmed=None))
            else:
                self._record_failure(node, observation)

    def _poll_stop(self, token: str, node: str, started: float,
                   operation: str | Observation) -> str:
        self._collect_cancel_results()
        if token not in self._stop_observations:
            observation = operation if isinstance(operation, Observation) else self.backend.poll(operation)
            if observation is not None:
                self._stop_observations[token] = observation
        observation = self._stop_observations.get(token)
        if observation is not None and not observation.success:
            self._fail(node, observation)
        if self.clock() - started >= self.stop_timeout_sec:
            if isinstance(operation, str):
                self.backend.cancel(operation)
            pending = ', '.join(sorted(self.ops[t][0] for t in self._pending_cancel_results))
            message = f'{node} exceeded stop deadline'
            if pending:
                message += f'; canceled operations still pending: {pending}'
            observation = Observation(False, Error.STOP_UNCONFIRMED, message, stop_confirmed=False)
            self._fail(node, observation)
        elif observation is None or self._pending_cancel_results:
            return 'RUNNING'
        self.completed_ops.add(token)
        self._save(stop_confirmed=observation.success and observation.stop_confirmed is True,
                   message=observation.message)
        # 정지 근거가 실패해도 FinalizeFailure까지 도달해 실제 결과를 반환해야 합니다.
        return 'SUCCESS'

    def cancel(self, token: str) -> None:
        # C++ 소멸 중에도 native halt 콜백은 노드 초기화를 마쳐야 합니다.
        # BT 밖으로 돌아온 뒤 실패를 처리합니다.
        try:
            node, _, operation = self.ops[token]
            if self._fault_shutdown is not None and node in STOP_NODES:
                return  # 이미 진행 중인 물리 상태 평가는 계속 유지합니다.
            if isinstance(operation, str):
                self.backend.cancel(operation)
        except Exception as error:
            if self._fault_shutdown is not None:
                self._fault_issue('BT operation halt', error)
            else:
                self._deferred_fault = error

    def _finalize(self, *, success: bool) -> Observation:
        record = self.record
        if record.result is not None:
            return Observation()
        if success and not (record.object_state == ObjectState.LEFT_GRIPPER
                            and (record.placement_state == Placement.CONFIRMED or
                                 (self.allow_assumed_placement and record.placement_state == Placement.ASSUMED))
                            and record.arm_recovered and record.stop_confirmed):
            observation = Observation(False, Error.PLACEMENT_NOT_CONFIRMED,
                                      'Success evidence is incomplete')
            self._fail('FinalizeSuccess', observation)
            return observation
        if success:
            status, error = Status.SUCCESS, Error.NONE
        else:
            error = self.failure.error if self.failure else Error.INTERNAL_ERROR
            status = (Status.FATAL if error in FATAL_ERRORS else
                      Status.CANCELED if error == Error.CANCELED else
                      Status.BLOCKED if self.failed_stage in {s.value for s in PREPARATION}
                      else Status.FAILED)
            if ((self.motion_started or self.cancel_requested or self._timeout_cancel_requested)
                    and not record.stop_confirmed):
                status, error = Status.FATAL, Error.STOP_UNCONFIRMED
        result = Result(record.goal.execution_id, record.execution_profile, status, error,
                        self.failed_stage, record.last_completed_stage, record.object_state,
                        record.placement_state, record.selected_arm, record.stop_confirmed,
                        record.arm_recovered, False,
                        ('Temporary return-to-pick completed; release and placement assumed'
                         if record.placement_state == Placement.ASSUMED else 'Verified single-object collection') if success else
                        (self.failure.message if self.failure else 'Execution failed'),
                        failed_substage=self.failed_substage, cancel_mode=record.cancel_mode)
        self._save(result=result, record_state=RecordState.FINISHED,
                   human_confirmation_required=(status == Status.FATAL or record.object_state in
                                                (ObjectState.HELD, ObjectState.UNKNOWN)))
        self.inhibited = self.inhibited or inhibits_execution(self.record)
        return Observation()

    def tick(self) -> None:
        if self.record is None or self.record.result is not None:
            return
        if self._fault_shutdown is not None:
            self._tick_fault()
            return
        if self._deferred_fault is not None:
            self._begin_fault(self._deferred_fault)
            self._deferred_fault = None
            return
        try:
            if self.tree is None:
                raise RuntimeError('Accepted execution has no BT runner')
            self.tree.tick()
            if self._deferred_fault is not None:
                raise self._deferred_fault
            transitions = self.tree.drain_transitions()
            self.transitions.extend(transitions)
            self.store.save_transitions(self.record.goal.execution_id, transitions)
        except Exception as error:
            self._deferred_fault = None
            if self.record.result is None:
                self._begin_fault(error)

    def _fault_issue(self, phase: str, error: Exception) -> None:
        message = f'{phase}: {type(error).__name__}: {error}'
        if message not in self._fault_shutdown.issues:
            self._fault_shutdown.issues.append(message)

    def _begin_fault(self, error: Exception) -> None:
        """새 실행을 금지하고 손상된 BT 밖에서 제한된 정지 절차를 시작합니다.

        token 반환 전 시작된 작업도 컨텍스트 abort로 중단 요청합니다. 기존 정지 작업이
        있으면 재사용하며 추가 오류가 나도 정지 마감을 연장하지 않습니다.
        """

        if self.record.result is not None:
            return
        if self._fault_shutdown is not None:
            self._fault_issue('Additional runtime error', error)
            return
        self.inhibited = True
        pending = {token for token, (node, _, operation) in self.ops.items()
                   if node not in STOP_NODES and isinstance(operation, str)
                   and (token not in self.completed_ops or token in self._pending_cancel_results)}
        fault = FaultShutdown(self.clock() + self.stop_timeout_sec, pending)
        self._fault_shutdown = fault
        self._fault_issue(f'Runtime error at {self.record.substage or "ValidateGoal"}', error)
        if self.failure is not None:
            fault.issues.append(self.failure.message)
        self._fail(self.record.substage or 'ValidateGoal', Observation(
            False, Error.INTERNAL_ERROR, fault.issues[0]))
        self._save(stop_confirmed=False, human_confirmation_required=True)
        # 컨텍스트 전체에 적용하는 이 신호는 작업을 시작한 뒤 예외가 발생해
        # core에 작업 token을 반환하지 못한 제출도 처리합니다.
        try:
            self.backend.abort()
        except Exception as abort_error:
            self._fault_issue('Context abort', abort_error)
        for token in tuple(pending):
            try:
                self.backend.cancel(self.ops[token][2])
            except Exception as cancel_error:
                self._fault_issue(f'Cancel {self.ops[token][0]}', cancel_error)
        existing = [(started, operation) for token, (node, started, operation) in self.ops.items()
                    if node in STOP_NODES and token not in self.completed_ops and isinstance(operation, str)]
        if existing:
            started, fault.stop_operation = existing[0]
            fault.deadline = min(fault.deadline, started + self.stop_timeout_sec)
        try:
            if self.tree is not None:
                self.tree.halt()
        except Exception as halt_error:
            self._fault_issue('BT halt', halt_error)
        # 이후 실행은 여기서 전담하며 손상된 BT를 다시 tick하지 않습니다.
        self._save(stage=Stage.STOPPING, substage='StopAndAssess', message=fault.issues[0])
        if fault.stop_operation is None:
            try:
                operation = self.backend.start('StopAndAssess', self.record.goal.execution_id)
                if not isinstance(operation, str):
                    raise TypeError('StopAndAssess did not return an operation token')
                fault.stop_operation = operation
            except Exception as stop_error:
                self._fault_issue('Start StopAndAssess', stop_error)

    def _fault_poll(self, operation: str, phase: str) -> Observation | None:
        try:
            observation = self.backend.poll(operation)
            if observation is not None and not isinstance(observation, Observation):
                raise TypeError('Backend returned an invalid observation')
            return observation
        except Exception as error:
            self._fault_issue(phase, error)
            return None

    def _tick_fault(self) -> None:
        fault = self._fault_shutdown
        if self._fault_timed_out():
            return
        for token in tuple(fault.pending):
            node, _, operation = self.ops[token]
            observation = self._fault_poll(operation, f'Poll {node}')
            if self._fault_timed_out():
                return
            if observation is None:
                continue
            fault.pending.remove(token)
            self._pending_cancel_results.discard(token)
            self.completed_ops.add(token)
            if observation.success:
                self._record_success(node, replace(observation, stop_confirmed=None))
            else:
                self._record_failure(node, observation)
                if observation.message:
                    fault.issues.append(f'{node} result: {observation.message}')
        if fault.stop_operation is None:
            self._finish_fault(confirmed=False)
            return
        if fault.stop_observation is None:
            fault.stop_observation = self._fault_poll(fault.stop_operation, 'Poll StopAndAssess')
        if self._fault_timed_out():
            return
        observation = fault.stop_observation
        if observation is None or fault.pending:
            return
        if observation.message:
            fault.issues.append(f'StopAndAssess result: {observation.message}')
        self._finish_fault(confirmed=observation.success and observation.stop_confirmed is True)

    def _fault_timed_out(self) -> bool:
        fault = self._fault_shutdown
        if self.clock() < fault.deadline:
            return False
        pending = ', '.join(sorted(self.ops[token][0] for token in fault.pending))
        fault.issues.append('Stop deadline exceeded' + (f'; operations still pending: {pending}' if pending else ''))
        self._finish_fault(confirmed=False)
        return True

    def _finish_fault(self, *, confirmed: bool) -> None:
        message = '; '.join(self._fault_shutdown.issues)
        self.failure = replace(self.failure, error=self.failure.error if confirmed else Error.STOP_UNCONFIRMED,
                               message=message)
        self._save(stop_confirmed=confirmed, stage=Stage.FINALIZING, substage='FinalizeFailure', message=message)
        self._finalize(success=False)

    def drain_events(self) -> list[Record]:
        events, self.events = self.events, []
        return events
