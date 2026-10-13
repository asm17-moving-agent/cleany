"""String contracts shared by the execution core and its ROS adapter."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class Stage(str, Enum):
    VALIDATING = 'VALIDATING'
    PREPARING_TARGET = 'PREPARING_TARGET'
    APPROACHING = 'APPROACHING'
    GRASPING = 'GRASPING'
    LIFTING = 'LIFTING'
    TRANSPORTING = 'TRANSPORTING'
    PLACING = 'PLACING'
    RETURNING_ARM = 'RETURNING_ARM'
    VERIFYING_PLACEMENT = 'VERIFYING_PLACEMENT'
    RELEASING_IN_PLACE = 'RELEASING_IN_PLACE'
    RECOVERING_ARM = 'RECOVERING_ARM'
    STOPPING = 'STOPPING'
    FINALIZING = 'FINALIZING'


class Status(str, Enum):
    SUCCESS = 'SUCCESS'
    BLOCKED = 'BLOCKED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'
    FATAL = 'FATAL'


class CancelMode(str, Enum):
    IMMEDIATE = 'IMMEDIATE'
    CHECKPOINT = 'CHECKPOINT'
    RETURN_ARM = 'RETURN_ARM'


def resolve_cancel_mode(current: str, requested: CancelMode | str | None) -> CancelMode:
    """Repeated requests are idempotent; only immediate stop may change a mode."""
    mode = CancelMode(requested) if requested is not None else CancelMode(current or 'CHECKPOINT')
    if current == CancelMode.IMMEDIATE:
        return CancelMode.IMMEDIATE
    if current and mode != current and mode != CancelMode.IMMEDIATE:
        raise ValueError('Cancellation already requested; only IMMEDIATE can override it')
    return mode


class Error(str, Enum):
    NONE = 'NONE'
    INVALID_ARGUMENT = 'INVALID_ARGUMENT'
    TARGET_UNAVAILABLE = 'TARGET_UNAVAILABLE'
    STALE_TARGET = 'STALE_TARGET'
    DESTINATION_UNAVAILABLE = 'DESTINATION_UNAVAILABLE'
    BACKEND_NOT_READY = 'BACKEND_NOT_READY'
    VERIFICATION_UNAVAILABLE = 'VERIFICATION_UNAVAILABLE'
    GRASP_FAILED = 'GRASP_FAILED'
    GRASP_LOST = 'GRASP_LOST'
    MOTION_FAILED = 'MOTION_FAILED'
    PLACEMENT_NOT_CONFIRMED = 'PLACEMENT_NOT_CONFIRMED'
    VERIFICATION_TIMEOUT = 'VERIFICATION_TIMEOUT'
    TIMEOUT = 'TIMEOUT'
    CANCELED = 'CANCELED'
    E_STOP = 'E_STOP'
    HARDWARE_ERROR = 'HARDWARE_ERROR'
    STOP_UNCONFIRMED = 'STOP_UNCONFIRMED'
    INTERNAL_ERROR = 'INTERNAL_ERROR'


class ObjectState(str, Enum):
    NOT_TOUCHED = 'NOT_TOUCHED'
    HELD = 'HELD'
    LEFT_GRIPPER = 'LEFT_GRIPPER'
    UNKNOWN = 'UNKNOWN'


class Placement(str, Enum):
    NOT_CHECKED = 'NOT_CHECKED'
    CONFIRMED = 'CONFIRMED'
    NOT_CONFIRMED = 'NOT_CONFIRMED'
    UNKNOWN = 'UNKNOWN'


class RecordState(str, Enum):
    ACTIVE = 'ACTIVE'
    FINISHED = 'FINISHED'
    INTERRUPTED = 'INTERRUPTED'
    RECORDING_FAILED = 'RECORDING_FAILED'


@dataclass(frozen=True)
class Goal:
    mission_id: str
    task_id: str
    execution_id: str
    skill_name: str
    snapshot_id: str
    object_id: int
    destination_id: str

    def valid(self) -> bool:
        return (all(value.strip() for value in (
            self.mission_id, self.task_id, self.execution_id, self.snapshot_id,
            self.destination_id,
        )) and self.skill_name in ('collect_trash', 'collect_lost_item') and 0 < self.object_id <= 0xffffffff)


@dataclass(frozen=True)
class Result:
    execution_id: str
    execution_profile: str
    status: Status
    error_code: Error
    failed_stage: str
    last_completed_stage: str
    object_state: ObjectState
    placement_state: Placement
    selected_arm: str
    stop_confirmed: bool
    arm_recovered: bool
    retryable: bool
    message: str
    failed_substage: str = ''
    cancel_mode: str = ''


@dataclass(frozen=True)
class Record:
    goal: Goal
    execution_profile: str = 'mock'
    record_state: RecordState = RecordState.ACTIVE
    stage: Stage = Stage.VALIDATING
    last_completed_stage: str = ''
    object_state: ObjectState = ObjectState.NOT_TOUCHED
    placement_state: Placement = Placement.NOT_CHECKED
    selected_arm: str = ''
    stop_confirmed: bool = False
    arm_recovered: bool = False
    human_confirmation_required: bool = False
    revision: int = 0
    accepted_at_ns: int = 0
    updated_at_ns: int = 0
    evidence_at_ns: int = 0
    message: str = ''
    result: Result | None = None
    substage: str = ''
    completed_substages: tuple[str, ...] = ()
    failed_substage: str = ''
    cancel_mode: str = ''

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> Record:
        data = dict(value)
        data['completed_substages'] = tuple(data.get('completed_substages', ()))
        data['goal'] = Goal(**data['goal'])
        for name, enum in (
            ('record_state', RecordState), ('stage', Stage),
            ('object_state', ObjectState), ('placement_state', Placement),
        ):
            data[name] = enum(data[name])
        if data['result'] is not None:
            result = dict(data['result'])
            for name, enum in (
                ('status', Status), ('error_code', Error),
                ('object_state', ObjectState), ('placement_state', Placement),
            ):
                result[name] = enum(result[name])
            data['result'] = Result(**result)
        return cls(**data)
