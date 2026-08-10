"""Durable multi-pose traversal, retry policy, cancellation, and resume."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import os
from pathlib import Path
import threading
from typing import Any, Callable, Protocol, Sequence

from cleany_handeye_calibration.models import SampleSplit
from cleany_handeye_calibration.pose_manifest import (
    MAX_RETRIES,
    MaterializedPose,
    PoseManifest,
    RequiredStageTimeouts,
    preflight_pose_manifest,
)


MAX_ATTEMPTS = MAX_RETRIES + 1


class PoseFailureCategory(str, Enum):
    IK = 'ik'
    PLANNING = 'planning'
    SETTLE = 'settle'
    IMAGE_ACQUISITION = 'image_acquisition'
    TARGET_DETECTION = 'target_detection'
    LIMIT = 'limit'
    COLLISION = 'collision'
    CONTROLLER = 'controller'
    ESTOP = 'estop'
    HARDWARE_FAULT = 'hardware_fault'
    DATA_INTEGRITY = 'data_integrity'
    INTERNAL = 'internal'


RETRYABLE_FAILURES = frozenset(
    {
        PoseFailureCategory.IK,
        PoseFailureCategory.PLANNING,
        PoseFailureCategory.SETTLE,
        PoseFailureCategory.IMAGE_ACQUISITION,
        PoseFailureCategory.TARGET_DETECTION,
    }
)


class PoseAttemptFailure(RuntimeError):
    def __init__(self, category: PoseFailureCategory, reason: str) -> None:
        self.category = PoseFailureCategory(category)
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('pose-attempt failure reason is required')
        self.reason = reason.strip()
        super().__init__(f'{self.category.value}: {self.reason}')


@dataclass(frozen=True, slots=True)
class CommittedPoseSample:
    pose_id: str
    sample_id: str
    split: SampleSplit

    def __post_init__(self) -> None:
        for field_name in ('pose_id', 'sample_id'):
            value = getattr(self, field_name)
            if (
                not isinstance(value, str)
                or not value
                or value != value.strip()
            ):
                raise ValueError(
                    f'{field_name} must be non-empty trimmed text'
                )
        object.__setattr__(self, 'split', SampleSplit(self.split))


class PoseRunExecutor(Protocol):
    def execute_pose(
        self,
        pose: MaterializedPose,
        *,
        attempt: int,
        stage_timeouts: RequiredStageTimeouts,
        cancel_requested: Callable[[], bool],
    ) -> CommittedPoseSample: ...


class RunJournalStatus(str, Enum):
    STARTED = 'started'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    SKIPPED = 'skipped'
    CANCELED = 'canceled'


@dataclass(frozen=True, slots=True)
class PoseRunJournalEntry:
    pose_id: str
    split: SampleSplit
    attempt: int
    status: RunJournalStatus
    category: PoseFailureCategory | None = None
    reason: str | None = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.pose_id, str)
            or not self.pose_id
            or self.pose_id != self.pose_id.strip()
        ):
            raise ValueError('pose_id must be non-empty trimmed text')
        object.__setattr__(self, 'split', SampleSplit(self.split))
        object.__setattr__(self, 'status', RunJournalStatus(self.status))
        if (
            isinstance(self.attempt, bool)
            or not isinstance(self.attempt, int)
            or self.attempt < 0
        ):
            raise ValueError('attempt must be a non-negative integer')
        if self.status in {
            RunJournalStatus.STARTED,
            RunJournalStatus.SUCCEEDED,
            RunJournalStatus.FAILED,
        } and not 1 <= self.attempt <= MAX_ATTEMPTS:
            raise ValueError('attempt status requires attempt in [1, 4]')
        if self.status in {
            RunJournalStatus.SKIPPED,
            RunJournalStatus.CANCELED,
        } and self.attempt != 0:
            raise ValueError('skip/cancel status requires attempt zero')
        if self.status is RunJournalStatus.FAILED:
            if self.category is None or self.reason is None:
                raise ValueError('failed journal row requires category/reason')
            object.__setattr__(
                self, 'category', PoseFailureCategory(self.category)
            )
            if not isinstance(self.reason, str) or not self.reason.strip():
                raise ValueError('failed journal reason is required')
            object.__setattr__(self, 'reason', self.reason.strip())
        elif self.category is not None:
            raise ValueError('only failed rows may have a category')
        elif self.status in {
            RunJournalStatus.SKIPPED,
            RunJournalStatus.CANCELED,
        }:
            if not isinstance(self.reason, str) or not self.reason.strip():
                raise ValueError('skip/cancel row requires a reason')
            object.__setattr__(self, 'reason', self.reason.strip())
        elif self.reason is not None:
            raise ValueError('started/succeeded rows cannot have a reason')

    def to_mapping(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            'pose_id': self.pose_id,
            'split': self.split.value,
            'attempt': self.attempt,
            'status': self.status.value,
        }
        if self.category is not None:
            result['category'] = self.category.value
        if self.reason is not None:
            result['reason'] = self.reason
        return result

    @classmethod
    def from_mapping(cls, value: Any) -> PoseRunJournalEntry:
        if not isinstance(value, dict):
            raise ValueError('pose-run journal row must be an object')
        expected = {'pose_id', 'split', 'attempt', 'status'}
        optional = {'category', 'reason'}
        if not expected.issubset(value) or set(value) - expected - optional:
            raise ValueError('pose-run journal row has invalid fields')
        return cls(
            pose_id=value['pose_id'],
            split=SampleSplit(value['split']),
            attempt=value['attempt'],
            status=RunJournalStatus(value['status']),
            category=(
                None
                if value.get('category') is None
                else PoseFailureCategory(value['category'])
            ),
            reason=value.get('reason'),
        )


class JsonlPoseRunJournal:
    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.is_symlink():
            raise ValueError('pose-run journal must not be a symbolic link')
        self._lock = threading.Lock()

    def append(self, entry: PoseRunJournalEntry) -> None:
        if not isinstance(entry, PoseRunJournalEntry):
            raise ValueError('entry must be PoseRunJournalEntry')
        payload = (
            json.dumps(
                entry.to_mapping(),
                allow_nan=False,
                separators=(',', ':'),
                sort_keys=True,
            )
            + '\n'
        ).encode('ascii')
        with self._lock:
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

    def read(self) -> tuple[PoseRunJournalEntry, ...]:
        if not self.path.exists():
            return ()
        payload = self.path.read_bytes()
        if payload and not payload.endswith(b'\n'):
            raise ValueError('pose-run journal has a partial final row')
        result = []
        for line_number, line in enumerate(payload.splitlines(), start=1):
            try:
                value = json.loads(line.decode('ascii'))
                result.append(PoseRunJournalEntry.from_mapping(value))
            except (
                UnicodeDecodeError,
                json.JSONDecodeError,
                ValueError,
            ) as error:
                raise ValueError(
                    f'invalid pose-run journal line {line_number}: {error}'
                ) from error
        return tuple(result)


class RunCancelToken:
    def __init__(self) -> None:
        self._event = threading.Event()

    def request(self) -> None:
        self._event.set()

    def is_requested(self) -> bool:
        return self._event.is_set()


@dataclass(frozen=True, slots=True)
class PoseRunSummary:
    completed_pose_ids: tuple[str, ...]
    skipped_pose_ids: tuple[str, ...]
    failed_pose_ids: tuple[str, ...]
    canceled: bool
    aborted_reason: str | None

    @property
    def success(self) -> bool:
        return not self.failed_pose_ids and not self.canceled and (
            self.aborted_reason is None
        )


def _completed_index(
    samples: Sequence[CommittedPoseSample],
    manifest: PoseManifest,
) -> dict[str, CommittedPoseSample]:
    poses = {pose.pose_id: pose for pose in manifest.poses}
    result: dict[str, CommittedPoseSample] = {}
    for sample in samples:
        if not isinstance(sample, CommittedPoseSample):
            raise ValueError('completed samples must be CommittedPoseSample')
        pose = poses.get(sample.pose_id)
        if pose is None:
            raise ValueError(
                'completed sample references unknown pose '
                f'{sample.pose_id}'
            )
        if sample.split is not pose.split:
            raise ValueError(
                f'completed sample split differs for {sample.pose_id}'
            )
        if sample.pose_id in result:
            raise ValueError(
                f'duplicate completed sample for {sample.pose_id}'
            )
        result[sample.pose_id] = sample
    return result


def _attempts_used(
    entries: Sequence[PoseRunJournalEntry],
    manifest: PoseManifest,
) -> dict[str, int]:
    pose_ids = {pose.pose_id for pose in manifest.poses}
    used = {pose_id: 0 for pose_id in pose_ids}
    seen: set[tuple[str, int]] = set()
    for entry in entries:
        if entry.pose_id not in pose_ids:
            raise ValueError(
                f'pose-run journal references unknown pose {entry.pose_id}'
            )
        if entry.status is RunJournalStatus.STARTED:
            key = (entry.pose_id, entry.attempt)
            if key in seen:
                raise ValueError('pose-run journal repeats an attempt start')
            seen.add(key)
            expected = used[entry.pose_id] + 1
            if entry.attempt != expected:
                raise ValueError(
                    'pose-run journal attempts are not sequential'
                )
            used[entry.pose_id] = entry.attempt
    return used


class MultiPoseRunOrchestrator:
    def __init__(
        self,
        executor: PoseRunExecutor,
        journal: JsonlPoseRunJournal,
        cancel_token: RunCancelToken | None = None,
    ) -> None:
        if executor is None:
            raise ValueError('executor is required')
        if not isinstance(journal, JsonlPoseRunJournal):
            raise ValueError('journal must be JsonlPoseRunJournal')
        self._executor = executor
        self._journal = journal
        self._cancel = cancel_token or RunCancelToken()

    def run(
        self,
        manifest: PoseManifest,
        *,
        completed_samples: Sequence[CommittedPoseSample] = (),
    ) -> PoseRunSummary:
        preflight_pose_manifest(manifest)
        completed = _completed_index(completed_samples, manifest)
        attempts_used = _attempts_used(self._journal.read(), manifest)
        completed_now: list[str] = []
        skipped: list[str] = []
        failed: list[str] = []

        for pose in manifest.poses:
            if pose.pose_id in completed:
                skipped.append(pose.pose_id)
                self._journal.append(
                    PoseRunJournalEntry(
                        pose.pose_id,
                        pose.split,
                        0,
                        RunJournalStatus.SKIPPED,
                        reason='sample already committed',
                    )
                )
                continue
            if self._cancel.is_requested():
                self._journal.append(
                    PoseRunJournalEntry(
                        pose.pose_id,
                        pose.split,
                        0,
                        RunJournalStatus.CANCELED,
                        reason='run cancellation requested',
                    )
                )
                return PoseRunSummary(
                    tuple(completed_now),
                    tuple(skipped),
                    tuple(failed),
                    True,
                    None,
                )

            first_attempt = attempts_used[pose.pose_id] + 1
            if first_attempt > MAX_ATTEMPTS:
                failed.append(pose.pose_id)
                continue
            pose_succeeded = False
            for attempt in range(first_attempt, MAX_ATTEMPTS + 1):
                if self._cancel.is_requested():
                    self._journal.append(
                        PoseRunJournalEntry(
                            pose.pose_id,
                            pose.split,
                            0,
                            RunJournalStatus.CANCELED,
                            reason='run cancellation requested',
                        )
                    )
                    return PoseRunSummary(
                        tuple(completed_now),
                        tuple(skipped),
                        tuple(failed),
                        True,
                        None,
                    )
                self._journal.append(
                    PoseRunJournalEntry(
                        pose.pose_id,
                        pose.split,
                        attempt,
                        RunJournalStatus.STARTED,
                    )
                )
                try:
                    committed = self._executor.execute_pose(
                        pose,
                        attempt=attempt,
                        stage_timeouts=manifest.run_config.stage_timeouts,
                        cancel_requested=self._cancel.is_requested,
                    )
                    if (
                        committed.pose_id != pose.pose_id
                        or committed.split is not pose.split
                    ):
                        raise PoseAttemptFailure(
                            PoseFailureCategory.DATA_INTEGRITY,
                            'executor committed a different pose or split',
                        )
                except PoseAttemptFailure as error:
                    self._journal.append(
                        PoseRunJournalEntry(
                            pose.pose_id,
                            pose.split,
                            attempt,
                            RunJournalStatus.FAILED,
                            category=error.category,
                            reason=error.reason,
                        )
                    )
                    if error.category not in RETRYABLE_FAILURES:
                        failed.append(pose.pose_id)
                        return PoseRunSummary(
                            tuple(completed_now),
                            tuple(skipped),
                            tuple(failed),
                            False,
                            str(error),
                        )
                    if attempt == MAX_ATTEMPTS:
                        failed.append(pose.pose_id)
                    continue
                except Exception as error:
                    failure = PoseAttemptFailure(
                        PoseFailureCategory.INTERNAL,
                        f'{type(error).__name__}: {error}',
                    )
                    self._journal.append(
                        PoseRunJournalEntry(
                            pose.pose_id,
                            pose.split,
                            attempt,
                            RunJournalStatus.FAILED,
                            category=failure.category,
                            reason=failure.reason,
                        )
                    )
                    failed.append(pose.pose_id)
                    return PoseRunSummary(
                        tuple(completed_now),
                        tuple(skipped),
                        tuple(failed),
                        False,
                        str(failure),
                    )
                else:
                    self._journal.append(
                        PoseRunJournalEntry(
                            pose.pose_id,
                            pose.split,
                            attempt,
                            RunJournalStatus.SUCCEEDED,
                        )
                    )
                    completed[pose.pose_id] = committed
                    completed_now.append(pose.pose_id)
                    pose_succeeded = True
                    break
            if not pose_succeeded and pose.pose_id not in failed:
                failed.append(pose.pose_id)

        return PoseRunSummary(
            tuple(completed_now),
            tuple(skipped),
            tuple(failed),
            False,
            None,
        )


__all__ = [
    'CommittedPoseSample',
    'JsonlPoseRunJournal',
    'MAX_ATTEMPTS',
    'MultiPoseRunOrchestrator',
    'PoseAttemptFailure',
    'PoseFailureCategory',
    'PoseRunExecutor',
    'PoseRunJournalEntry',
    'PoseRunSummary',
    'RETRYABLE_FAILURES',
    'RunCancelToken',
    'RunJournalStatus',
]
