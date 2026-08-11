"""ROS-independent orchestration for one eye-in-hand calibration pose.

The reducer deliberately exposes every externally blocking operation as one
stage.  ROS adapters may implement the effects, while unit tests use small
fakes and can prove that no later effect runs after a failure.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Callable, Protocol

from cleany_handeye_calibration.camera_acquisition import CameraFramePair
from cleany_handeye_calibration.ik_port import ValidatedJointGoal
from cleany_handeye_calibration.joint_state_sync import (
    LEFT_ARM_JOINT_NAMES,
    InterpolatedJointState,
)
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
)
from cleany_handeye_calibration.pnp import PnpResult
from cleany_handeye_calibration.target_detector import CharucoDetection
from cleany_handeye_calibration.transforms import RigidTransform


class SinglePoseStage(str, Enum):
    RESOLVE_POSITION_IK = 'resolve_position_ik'
    VALIDATE_RESOLVED_POSE = 'validate_resolved_pose'
    PLAN = 'plan'
    EXECUTE = 'execute'
    WAIT_SETTLED = 'wait_settled'
    ACQUIRE_IMAGE = 'acquire_image'
    DETECT_TARGET = 'detect_target'
    COMPUTE_FEEDBACK_FK = 'compute_feedback_fk'
    RECORD_SAMPLE = 'record_sample'


ORDERED_STAGES = tuple(SinglePoseStage)


def _text(value: str, *, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f'{field_name} must be a non-empty trimmed string')
    return value


def _positive_finite(value: float, *, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{field_name} must be positive and finite')
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'{field_name} must be positive and finite'
        ) from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{field_name} must be positive and finite')
    return number


@dataclass(frozen=True, slots=True)
class SinglePoseTimeouts:
    """Required monotonic budgets for all nine single-pose stages."""

    resolve_position_ik_sec: float
    validate_resolved_pose_sec: float
    plan_sec: float
    execute_sec: float
    wait_settled_sec: float
    acquire_image_sec: float
    detect_target_sec: float
    compute_feedback_fk_sec: float
    record_sample_sec: float

    def __post_init__(self) -> None:
        for stage in ORDERED_STAGES:
            field_name = f'{stage.value}_sec'
            object.__setattr__(
                self,
                field_name,
                _positive_finite(
                    getattr(self, field_name),
                    field_name=field_name,
                ),
            )

    def for_stage(self, stage: SinglePoseStage) -> float:
        return float(getattr(self, f'{SinglePoseStage(stage).value}_sec'))


@dataclass(frozen=True, slots=True)
class JointSoftLimit:
    joint_name: str
    lower_rad: float
    upper_rad: float

    def __post_init__(self) -> None:
        name = _text(self.joint_name, field_name='joint_name')
        try:
            lower = float(self.lower_rad)
            upper = float(self.upper_rad)
        except (TypeError, ValueError) as error:
            raise ValueError('joint soft limits must be numeric') from error
        if not all(math.isfinite(value) for value in (lower, upper)):
            raise ValueError('joint soft limits must be finite')
        if lower >= upper:
            raise ValueError('joint soft-limit lower must be below upper')
        object.__setattr__(self, 'joint_name', name)
        object.__setattr__(self, 'lower_rad', lower)
        object.__setattr__(self, 'upper_rad', upper)


@dataclass(frozen=True, slots=True)
class SinglePoseSafetyProfile:
    """Explicit decision-gate values; there are intentionally no defaults."""

    profile_id: str
    soft_joint_limits: tuple[JointSoftLimit, ...]
    required_collision_margin_m: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'profile_id',
            _text(self.profile_id, field_name='profile_id'),
        )
        limits = tuple(self.soft_joint_limits)
        names = tuple(limit.joint_name for limit in limits)
        if names != LEFT_ARM_JOINT_NAMES:
            raise ValueError(
                'soft_joint_limits must contain the canonical five '
                'left-arm joints in canonical order'
            )
        object.__setattr__(self, 'soft_joint_limits', limits)
        object.__setattr__(
            self,
            'required_collision_margin_m',
            _positive_finite(
                self.required_collision_margin_m,
                field_name='required_collision_margin_m',
            ),
        )

    def validate(
        self,
        pose: JointPose,
        *,
        observed_collision_clearance_m: float,
    ) -> None:
        if pose.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError(
                'resolved pose must use the canonical left-arm order'
            )
        positions = dict(
            zip(pose.joint_names, pose.positions_rad, strict=True)
        )
        violations = tuple(
            limit.joint_name
            for limit in self.soft_joint_limits
            if not (
                limit.lower_rad
                <= positions[limit.joint_name]
                <= limit.upper_rad
            )
        )
        if violations:
            raise ValueError(
                f'resolved pose violates soft joint limits: {violations!r}'
            )
        clearance = _positive_finite(
            observed_collision_clearance_m,
            field_name='observed_collision_clearance_m',
        )
        if clearance < self.required_collision_margin_m:
            raise ValueError(
                'resolved pose collision clearance is below the required '
                f'margin: {clearance:g} < '
                f'{self.required_collision_margin_m:g} m'
            )


@dataclass(frozen=True, slots=True)
class SinglePoseRequest:
    sample_id: str
    pose_id: str
    split: SampleSplit
    target: PositionTarget
    ik_seed: JointPose
    timeouts: SinglePoseTimeouts
    safety_profile: SinglePoseSafetyProfile

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            'sample_id',
            _text(self.sample_id, field_name='sample_id'),
        )
        object.__setattr__(
            self,
            'pose_id',
            _text(self.pose_id, field_name='pose_id'),
        )
        object.__setattr__(self, 'split', SampleSplit(self.split))
        if not isinstance(self.target, PositionTarget):
            raise ValueError('target must be PositionTarget')
        if self.target.frame_id != 'base_link':
            raise ValueError('target frame must be base_link')
        if not isinstance(self.ik_seed, JointPose):
            raise ValueError('ik_seed must be JointPose')
        if self.ik_seed.joint_names != LEFT_ARM_JOINT_NAMES:
            raise ValueError('ik_seed must use the canonical left-arm order')
        if not isinstance(self.timeouts, SinglePoseTimeouts):
            raise ValueError('timeouts must be SinglePoseTimeouts')
        if not isinstance(self.safety_profile, SinglePoseSafetyProfile):
            raise ValueError(
                'safety_profile must be SinglePoseSafetyProfile'
            )


@dataclass(frozen=True, slots=True)
class ResolvedPoseValidation:
    validated_goal: ValidatedJointGoal
    observed_collision_clearance_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.validated_goal, ValidatedJointGoal):
            raise ValueError('validated_goal must be ValidatedJointGoal')
        _positive_finite(
            self.observed_collision_clearance_m,
            field_name='observed_collision_clearance_m',
        )


@dataclass(frozen=True, slots=True)
class TargetObservation:
    pair: CameraFramePair
    detection: CharucoDetection
    pnp: PnpResult

    def __post_init__(self) -> None:
        if not isinstance(self.pair, CameraFramePair):
            raise ValueError('pair must be CameraFramePair')
        if not isinstance(self.detection, CharucoDetection):
            raise ValueError('detection must be CharucoDetection')
        if not self.detection.valid:
            raise ValueError('detection must be valid')
        if not isinstance(self.pnp, PnpResult) or not self.pnp.valid:
            raise ValueError('pnp must be a valid PnpResult')


@dataclass(frozen=True, slots=True)
class FeedbackFkObservation:
    interpolation: InterpolatedJointState
    base_T_gripper: RigidTransform

    def __post_init__(self) -> None:
        if not isinstance(self.interpolation, InterpolatedJointState):
            raise ValueError(
                'interpolation must be InterpolatedJointState'
            )
        if not isinstance(self.base_T_gripper, RigidTransform):
            raise ValueError('base_T_gripper must be RigidTransform')
        if self.interpolation.image_stamp_ns <= 0:
            raise ValueError('image feedback stamp must be nonzero')


class SinglePoseEffects(Protocol):
    def resolve_position_ik(
        self, request: SinglePoseRequest, timeout_sec: float
    ) -> JointPose: ...

    def validate_resolved_pose(
        self,
        request: SinglePoseRequest,
        resolved_pose: JointPose,
        timeout_sec: float,
    ) -> ResolvedPoseValidation: ...

    def plan(
        self,
        validation: ResolvedPoseValidation,
        timeout_sec: float,
    ) -> Any: ...

    def execute(self, planned_motion: Any, timeout_sec: float) -> None: ...

    def wait_settled(
        self, resolved_pose: JointPose, timeout_sec: float
    ) -> int: ...

    def acquire_image(
        self, settled_stamp_ns: int, timeout_sec: float
    ) -> CameraFramePair: ...

    def detect_target(
        self, pair: CameraFramePair, timeout_sec: float
    ) -> TargetObservation: ...

    def compute_feedback_fk(
        self, observation: TargetObservation, timeout_sec: float
    ) -> FeedbackFkObservation: ...

    def record_sample(
        self,
        request: SinglePoseRequest,
        resolved_pose: JointPose,
        observation: TargetObservation,
        feedback_fk: FeedbackFkObservation,
        timeout_sec: float,
    ) -> Any: ...


class JournalStatus(str, Enum):
    STARTED = 'started'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


@dataclass(frozen=True, slots=True)
class StageJournalEntry:
    stage: SinglePoseStage
    status: JournalStatus
    reason: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, 'stage', SinglePoseStage(self.stage))
        object.__setattr__(self, 'status', JournalStatus(self.status))
        if self.status is JournalStatus.FAILED:
            object.__setattr__(
                self,
                'reason',
                _text(self.reason, field_name='reason'),
            )
        elif self.reason is not None:
            raise ValueError('only failed journal entries may have a reason')

    def to_mapping(self) -> dict[str, str]:
        value = {'stage': self.stage.value, 'status': self.status.value}
        if self.reason is not None:
            value['reason'] = self.reason
        return value


class StageJournal(Protocol):
    def append(self, entry: StageJournalEntry) -> None: ...


@dataclass(slots=True)
class MemoryStageJournal:
    entries: list[StageJournalEntry] = field(default_factory=list)

    def append(self, entry: StageJournalEntry) -> None:
        if not isinstance(entry, StageJournalEntry):
            raise ValueError('entry must be StageJournalEntry')
        self.entries.append(entry)


class JsonlStageJournal:
    """Durable append-only stage evidence inside one run directory."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.is_symlink():
            raise ValueError('stage journal must not be a symbolic link')

    def append(self, entry: StageJournalEntry) -> None:
        if not isinstance(entry, StageJournalEntry):
            raise ValueError('entry must be StageJournalEntry')
        payload = (
            json.dumps(
                entry.to_mapping(),
                allow_nan=False,
                separators=(',', ':'),
                sort_keys=True,
            )
            + '\n'
        ).encode('ascii')
        descriptor = os.open(
            self.path,
            os.O_APPEND | os.O_CREAT | os.O_WRONLY | os.O_CLOEXEC,
            0o600,
        )
        try:
            os.write(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


class SinglePoseFailure(RuntimeError):
    def __init__(self, stage: SinglePoseStage, reason: str) -> None:
        self.stage = SinglePoseStage(stage)
        self.reason = _text(reason, field_name='reason')
        super().__init__(f'{self.stage.value}: {self.reason}')


@dataclass(frozen=True, slots=True)
class SinglePoseResult:
    resolved_pose: JointPose
    observation: TargetObservation
    feedback_fk: FeedbackFkObservation
    stored_sample: Any


class SinglePoseOrchestrator:
    def __init__(
        self,
        effects: SinglePoseEffects,
        journal: StageJournal,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if effects is None:
            raise ValueError('effects is required')
        if journal is None:
            raise ValueError('journal is required')
        if not callable(monotonic):
            raise ValueError('monotonic must be callable')
        self._effects = effects
        self._journal = journal
        self._monotonic = monotonic

    def _run_stage(
        self,
        stage: SinglePoseStage,
        timeout_sec: float,
        operation: Callable[[], Any],
    ) -> Any:
        self._journal.append(
            StageJournalEntry(stage, JournalStatus.STARTED)
        )
        started = float(self._monotonic())
        try:
            result = operation()
            elapsed = float(self._monotonic()) - started
            if not math.isfinite(elapsed) or elapsed < 0.0:
                raise RuntimeError('monotonic clock moved backwards')
            if elapsed > timeout_sec:
                raise TimeoutError(
                    f'stage exceeded timeout {timeout_sec:g} s '
                    f'(elapsed {elapsed:g} s)'
                )
        except Exception as error:
            reason = f'{type(error).__name__}: {error}'
            self._journal.append(
                StageJournalEntry(stage, JournalStatus.FAILED, reason)
            )
            raise SinglePoseFailure(stage, reason) from error
        self._journal.append(
            StageJournalEntry(stage, JournalStatus.SUCCEEDED)
        )
        return result

    def run(self, request: SinglePoseRequest) -> SinglePoseResult:
        if not isinstance(request, SinglePoseRequest):
            raise ValueError('request must be SinglePoseRequest')
        budget = request.timeouts.for_stage

        resolved = self._run_stage(
            SinglePoseStage.RESOLVE_POSITION_IK,
            budget(SinglePoseStage.RESOLVE_POSITION_IK),
            lambda: self._effects.resolve_position_ik(
                request,
                budget(SinglePoseStage.RESOLVE_POSITION_IK),
            ),
        )
        validation = self._run_stage(
            SinglePoseStage.VALIDATE_RESOLVED_POSE,
            budget(SinglePoseStage.VALIDATE_RESOLVED_POSE),
            lambda: self._validate_resolved(
                request,
                resolved,
                budget(SinglePoseStage.VALIDATE_RESOLVED_POSE),
            ),
        )
        planned = self._run_stage(
            SinglePoseStage.PLAN,
            budget(SinglePoseStage.PLAN),
            lambda: self._effects.plan(
                validation,
                budget(SinglePoseStage.PLAN),
            ),
        )
        self._run_stage(
            SinglePoseStage.EXECUTE,
            budget(SinglePoseStage.EXECUTE),
            lambda: self._effects.execute(
                planned,
                budget(SinglePoseStage.EXECUTE),
            ),
        )
        settled_stamp_ns = self._run_stage(
            SinglePoseStage.WAIT_SETTLED,
            budget(SinglePoseStage.WAIT_SETTLED),
            lambda: self._effects.wait_settled(
                resolved,
                budget(SinglePoseStage.WAIT_SETTLED),
            ),
        )
        pair = self._run_stage(
            SinglePoseStage.ACQUIRE_IMAGE,
            budget(SinglePoseStage.ACQUIRE_IMAGE),
            lambda: self._effects.acquire_image(
                settled_stamp_ns,
                budget(SinglePoseStage.ACQUIRE_IMAGE),
            ),
        )
        observation = self._run_stage(
            SinglePoseStage.DETECT_TARGET,
            budget(SinglePoseStage.DETECT_TARGET),
            lambda: self._effects.detect_target(
                pair,
                budget(SinglePoseStage.DETECT_TARGET),
            ),
        )
        feedback_fk = self._run_stage(
            SinglePoseStage.COMPUTE_FEEDBACK_FK,
            budget(SinglePoseStage.COMPUTE_FEEDBACK_FK),
            lambda: self._effects.compute_feedback_fk(
                observation,
                budget(SinglePoseStage.COMPUTE_FEEDBACK_FK),
            ),
        )
        stored = self._run_stage(
            SinglePoseStage.RECORD_SAMPLE,
            budget(SinglePoseStage.RECORD_SAMPLE),
            lambda: self._effects.record_sample(
                request,
                resolved,
                observation,
                feedback_fk,
                budget(SinglePoseStage.RECORD_SAMPLE),
            ),
        )
        return SinglePoseResult(
            resolved_pose=resolved,
            observation=observation,
            feedback_fk=feedback_fk,
            stored_sample=stored,
        )

    def _validate_resolved(
        self,
        request: SinglePoseRequest,
        resolved: JointPose,
        timeout_sec: float,
    ) -> ResolvedPoseValidation:
        if not isinstance(resolved, JointPose):
            raise ValueError('IK effect must return JointPose')
        validation = self._effects.validate_resolved_pose(
            request,
            resolved,
            timeout_sec,
        )
        if not isinstance(validation, ResolvedPoseValidation):
            raise ValueError(
                'validation effect must return ResolvedPoseValidation'
            )
        if validation.validated_goal.pose != resolved:
            raise ValueError('state-validity proof does not match IK result')
        request.safety_profile.validate(
            resolved,
            observed_collision_clearance_m=(
                validation.observed_collision_clearance_m
            ),
        )
        return validation


__all__ = [
    'FeedbackFkObservation',
    'JointSoftLimit',
    'JsonlStageJournal',
    'JournalStatus',
    'MemoryStageJournal',
    'ORDERED_STAGES',
    'ResolvedPoseValidation',
    'SinglePoseEffects',
    'SinglePoseFailure',
    'SinglePoseOrchestrator',
    'SinglePoseRequest',
    'SinglePoseResult',
    'SinglePoseSafetyProfile',
    'SinglePoseStage',
    'SinglePoseTimeouts',
    'StageJournalEntry',
    'TargetObservation',
]
