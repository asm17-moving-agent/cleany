from collections import Counter

from cleany_handeye_calibration.run_recovery import (
    CommittedPoseSample,
    JsonlPoseRunJournal,
    MultiPoseRunOrchestrator,
    PoseAttemptFailure,
    PoseFailureCategory,
    PoseRunJournalEntry,
    RunCancelToken,
    RunJournalStatus,
)
from pose_test_support import materialized_manifest


class FakeExecutor:
    def __init__(self, failures=None, cancel_token=None):
        self.failures = {
            pose_id: list(values)
            for pose_id, values in (failures or {}).items()
        }
        self.cancel_token = cancel_token
        self.calls = []
        self.timeouts = []

    def execute_pose(
        self,
        pose,
        *,
        attempt,
        stage_timeouts,
        cancel_requested,
    ):
        assert not cancel_requested()
        self.calls.append((pose.pose_id, pose.split, attempt))
        self.timeouts.append(stage_timeouts)
        queue = self.failures.get(pose.pose_id, [])
        if queue:
            raise PoseAttemptFailure(queue.pop(0), f'failure {attempt}')
        if self.cancel_token is not None and len(self.calls) == 1:
            self.cancel_token.request()
        return CommittedPoseSample(
            pose_id=pose.pose_id,
            sample_id=pose.pose_id,
            split=pose.split,
        )


def test_retry_three_means_four_total_attempts_and_uses_manifest_timeouts(
    tmp_path,
):
    manifest = materialized_manifest()
    first = manifest.poses[0].pose_id
    executor = FakeExecutor(
        {first: [PoseFailureCategory.IK] * 3}
    )
    journal = JsonlPoseRunJournal(tmp_path / 'pose_run.jsonl')

    summary = MultiPoseRunOrchestrator(executor, journal).run(manifest)

    assert summary.success
    counts = Counter(pose_id for pose_id, _, _ in executor.calls)
    assert counts[first] == 4
    assert len(summary.completed_pose_ids) == 25
    assert all(
        item is manifest.run_config.stage_timeouts
        for item in executor.timeouts
    )
    first_rows = [
        row for row in journal.read() if row.pose_id == first
    ]
    assert [
        row.attempt
        for row in first_rows
        if row.status is RunJournalStatus.STARTED
    ] == [1, 2, 3, 4]


def test_resume_skips_committed_samples_without_duplicate_execution(tmp_path):
    manifest = materialized_manifest()
    completed = tuple(
        CommittedPoseSample(pose.pose_id, pose.pose_id, pose.split)
        for pose in manifest.poses[:7]
    )
    executor = FakeExecutor()
    journal = JsonlPoseRunJournal(tmp_path / 'pose_run.jsonl')

    summary = MultiPoseRunOrchestrator(executor, journal).run(
        manifest,
        completed_samples=completed,
    )

    assert summary.success
    assert summary.skipped_pose_ids == tuple(
        pose.pose_id for pose in manifest.poses[:7]
    )
    assert [pose_id for pose_id, _, _ in executor.calls] == [
        pose.pose_id for pose in manifest.poses[7:]
    ]
    assert executor.calls[-1][1].value == 'held_out'


def test_retry_exhaustion_is_partial_but_nonretryable_failure_aborts(tmp_path):
    manifest = materialized_manifest()
    first = manifest.poses[0].pose_id
    exhausted = FakeExecutor(
        {first: [PoseFailureCategory.PLANNING] * 4}
    )
    partial = MultiPoseRunOrchestrator(
        exhausted,
        JsonlPoseRunJournal(tmp_path / 'retry.jsonl'),
    ).run(manifest)

    assert partial.failed_pose_ids == (first,)
    assert len(partial.completed_pose_ids) == 24
    assert partial.aborted_reason is None

    collision = FakeExecutor({first: [PoseFailureCategory.COLLISION]})
    aborted = MultiPoseRunOrchestrator(
        collision,
        JsonlPoseRunJournal(tmp_path / 'collision.jsonl'),
    ).run(manifest)
    assert aborted.failed_pose_ids == (first,)
    assert aborted.aborted_reason is not None
    assert len(collision.calls) == 1


def test_resume_counts_crashed_started_attempt_and_cancel_stops_next_pose(
    tmp_path,
):
    manifest = materialized_manifest()
    first = manifest.poses[0]
    journal = JsonlPoseRunJournal(tmp_path / 'resume.jsonl')
    journal.append(
        PoseRunJournalEntry(
            first.pose_id,
            first.split,
            1,
            RunJournalStatus.STARTED,
        )
    )
    executor = FakeExecutor()
    summary = MultiPoseRunOrchestrator(executor, journal).run(manifest)
    assert summary.success
    assert executor.calls[0][2] == 2

    cancel = RunCancelToken()
    cancel_executor = FakeExecutor(cancel_token=cancel)
    canceled = MultiPoseRunOrchestrator(
        cancel_executor,
        JsonlPoseRunJournal(tmp_path / 'cancel.jsonl'),
        cancel,
    ).run(manifest)
    assert canceled.canceled
    assert canceled.completed_pose_ids == (first.pose_id,)
    assert len(cancel_executor.calls) == 1
