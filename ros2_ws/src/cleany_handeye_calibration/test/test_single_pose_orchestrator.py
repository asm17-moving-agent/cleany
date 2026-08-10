from __future__ import annotations

from dataclasses import dataclass
import json

import pytest

from cleany_handeye_calibration.camera_acquisition import (
    CAMERA_D,
    CAMERA_DISTORTION_MODEL,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_K,
    CAMERA_P,
    CAMERA_R,
    CAMERA_WIDTH,
    CameraFramePair,
    CameraInfoFrame,
    ImageFrame,
)
from cleany_handeye_calibration.ik_port import ValidatedJointGoal
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    LEFT_ARM_JOINT_NAMES,
    InterpolatedJointState,
)
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    SampleSplit,
    TimedJointSample,
)
from cleany_handeye_calibration.pnp import PnpCandidate, PnpResult
from cleany_handeye_calibration.single_pose_orchestrator import (
    FeedbackFkObservation,
    JointSoftLimit,
    JournalStatus,
    JsonlStageJournal,
    MemoryStageJournal,
    ORDERED_STAGES,
    ResolvedPoseValidation,
    SinglePoseFailure,
    SinglePoseOrchestrator,
    SinglePoseRequest,
    SinglePoseSafetyProfile,
    SinglePoseStage,
    SinglePoseTimeouts,
    TargetObservation,
)
from cleany_handeye_calibration.target_detector import (
    INNER_CORNER_COUNT,
    analyze_charuco_corners,
)
from cleany_handeye_calibration.transforms import RigidTransform


RESOLVED = JointPose(
    LEFT_ARM_JOINT_NAMES,
    (-1.0, 0.2, 0.4, -0.2, -1.5),
)
ALL_JOINTS = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names


def _timeouts(value: float = 1.0) -> SinglePoseTimeouts:
    return SinglePoseTimeouts(*([value] * len(ORDERED_STAGES)))


def _profile() -> SinglePoseSafetyProfile:
    return SinglePoseSafetyProfile(
        profile_id='explicit_test_profile',
        soft_joint_limits=tuple(
            JointSoftLimit(name, -2.0, 2.0)
            for name in LEFT_ARM_JOINT_NAMES
        ),
        required_collision_margin_m=0.01,
    )


def _request(timeouts: SinglePoseTimeouts | None = None):
    return SinglePoseRequest(
        sample_id='sample_001',
        pose_id='calibration_001',
        split=SampleSplit.CALIBRATION,
        target=PositionTarget('base_link', (0.5, 0.2, 0.4)),
        ik_seed=RESOLVED,
        timeouts=timeouts or _timeouts(),
        safety_profile=_profile(),
    )


def _pair() -> CameraFramePair:
    stamp_ns = 1_500
    return CameraFramePair(
        image=ImageFrame(
            stamp_ns=stamp_ns,
            frame_id=CAMERA_FRAME_ID,
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            encoding='rgb8',
            is_bigendian=False,
            step=CAMERA_WIDTH * 3,
            data=b'\xff' * (CAMERA_WIDTH * CAMERA_HEIGHT * 3),
        ),
        camera_info=CameraInfoFrame(
            stamp_ns=stamp_ns,
            frame_id=CAMERA_FRAME_ID,
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            distortion_model=CAMERA_DISTORTION_MODEL,
            d=CAMERA_D,
            k=CAMERA_K,
            r=CAMERA_R,
            p=CAMERA_P,
        ),
    )


def _observation() -> TargetObservation:
    detection = analyze_charuco_corners(
        tuple(range(INNER_CORNER_COUNT)),
        tuple(
            (100.0 + (index % 6) * 40.0, 80.0 + (index // 6) * 50.0)
            for index in range(INNER_CORNER_COUNT)
        ),
    )
    transform = RigidTransform.from_quaternion_xyzw(
        parent_frame=CAMERA_FRAME_ID,
        child_frame='charuco_target',
        translation_m=(0.0, 0.0, 0.4),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    candidate = PnpCandidate(
        index=0,
        valid=True,
        failure_reason=None,
        raw_camera_T_target=transform,
        raw_min_depth_m=0.4,
        raw_reprojection_rmse_px=0.1,
        refined_camera_T_target=transform,
        refined_min_depth_m=0.4,
        refined_reprojection_rmse_px=0.1,
    )
    return TargetObservation(
        pair=_pair(),
        detection=detection,
        pnp=PnpResult(
            valid=True,
            method='SOLVEPNP_IPPE',
            failure_reason=None,
            failure_detail=None,
            ambiguous=False,
            selected_candidate_index=0,
            camera_T_target=transform,
            candidates=(candidate,),
        ),
    )


def _feedback_fk() -> FeedbackFkObservation:
    sample = TimedJointSample(
        stamp_ns=1_500,
        joint_names=ALL_JOINTS,
        positions_rad=(0.0,) * len(ALL_JOINTS),
        velocities_rad_s=(0.0,) * len(ALL_JOINTS),
    )
    return FeedbackFkObservation(
        interpolation=InterpolatedJointState(
            sample=sample,
            before_stamp_ns=1_000,
            after_stamp_ns=2_000,
            ratio=0.5,
        ),
        base_T_gripper=RigidTransform.from_quaternion_xyzw(
            parent_frame='base_link',
            child_frame='left_gripper_frame',
            translation_m=(0.5, 0.2, 0.4),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    )


@dataclass
class FakeEffects:
    fail_stage: SinglePoseStage | None = None

    def __post_init__(self):
        self.calls = []

    def _called(self, stage):
        self.calls.append(stage)
        if self.fail_stage is stage:
            raise RuntimeError(f'forced {stage.value} failure')

    def resolve_position_ik(self, request, timeout_sec):
        self._called(SinglePoseStage.RESOLVE_POSITION_IK)
        return RESOLVED

    def validate_resolved_pose(self, request, resolved_pose, timeout_sec):
        self._called(SinglePoseStage.VALIDATE_RESOLVED_POSE)
        return ResolvedPoseValidation(
            validated_goal=ValidatedJointGoal(
                pose=resolved_pose,
                checked_state_stamp_ns=1_000,
            ),
            observed_collision_clearance_m=0.02,
        )

    def plan(self, validation, timeout_sec):
        self._called(SinglePoseStage.PLAN)
        return 'planned-motion'

    def execute(self, planned_motion, timeout_sec):
        self._called(SinglePoseStage.EXECUTE)

    def wait_settled(self, resolved_pose, timeout_sec):
        self._called(SinglePoseStage.WAIT_SETTLED)
        return 1_000

    def acquire_image(self, settled_stamp_ns, timeout_sec):
        self._called(SinglePoseStage.ACQUIRE_IMAGE)
        return _pair()

    def detect_target(self, pair, timeout_sec):
        self._called(SinglePoseStage.DETECT_TARGET)
        return _observation()

    def compute_feedback_fk(self, observation, timeout_sec):
        self._called(SinglePoseStage.COMPUTE_FEEDBACK_FK)
        return _feedback_fk()

    def record_sample(
        self,
        request,
        resolved_pose,
        observation,
        feedback_fk,
        timeout_sec,
    ):
        self._called(SinglePoseStage.RECORD_SAMPLE)
        return 'stored-sample'


def test_single_pose_runs_exact_stage_order_and_records_success():
    effects = FakeEffects()
    journal = MemoryStageJournal()

    result = SinglePoseOrchestrator(effects, journal).run(_request())

    assert effects.calls == list(ORDERED_STAGES)
    assert result.resolved_pose == RESOLVED
    assert result.stored_sample == 'stored-sample'
    assert [entry.stage for entry in journal.entries] == [
        stage for stage in ORDERED_STAGES for _ in range(2)
    ]
    assert [entry.status for entry in journal.entries] == [
        status
        for _ in ORDERED_STAGES
        for status in (JournalStatus.STARTED, JournalStatus.SUCCEEDED)
    ]


@pytest.mark.parametrize('failed_stage', ORDERED_STAGES)
def test_each_stage_failure_is_journaled_and_stops_later_effects(
    failed_stage,
):
    effects = FakeEffects(fail_stage=failed_stage)
    journal = MemoryStageJournal()

    with pytest.raises(SinglePoseFailure) as caught:
        SinglePoseOrchestrator(effects, journal).run(_request())

    assert caught.value.stage is failed_stage
    assert 'forced' in caught.value.reason
    assert effects.calls == list(
        ORDERED_STAGES[: ORDERED_STAGES.index(failed_stage) + 1]
    )
    assert journal.entries[-1].stage is failed_stage
    assert journal.entries[-1].status is JournalStatus.FAILED
    assert 'forced' in journal.entries[-1].reason


def test_stage_overrun_is_reported_as_that_stage_timeout():
    ticks = iter((0.0, 1.1))
    journal = MemoryStageJournal()
    orchestrator = SinglePoseOrchestrator(
        FakeEffects(),
        journal,
        monotonic=lambda: next(ticks),
    )

    with pytest.raises(SinglePoseFailure) as caught:
        orchestrator.run(_request(_timeouts(1.0)))

    assert caught.value.stage is SinglePoseStage.RESOLVE_POSITION_IK
    assert 'exceeded timeout' in caught.value.reason


def test_safety_profile_rejects_missing_margin_before_planning():
    effects = FakeEffects()
    effects.validate_resolved_pose = lambda *args, **kwargs: (
        ResolvedPoseValidation(
            validated_goal=ValidatedJointGoal(
                pose=RESOLVED,
                checked_state_stamp_ns=1_000,
            ),
            observed_collision_clearance_m=0.005,
        )
    )
    journal = MemoryStageJournal()

    with pytest.raises(SinglePoseFailure) as caught:
        SinglePoseOrchestrator(effects, journal).run(_request())

    assert caught.value.stage is SinglePoseStage.VALIDATE_RESOLVED_POSE
    assert 'below the required margin' in caught.value.reason
    assert SinglePoseStage.PLAN not in effects.calls


def test_jsonl_journal_durably_records_failure_reason(tmp_path):
    path = tmp_path / 'run' / 'orchestration.jsonl'

    with pytest.raises(SinglePoseFailure):
        SinglePoseOrchestrator(
            FakeEffects(fail_stage=SinglePoseStage.ACQUIRE_IMAGE),
            JsonlStageJournal(path),
        ).run(_request())

    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[-1]['stage'] == 'acquire_image'
    assert rows[-1]['status'] == 'failed'
    assert 'forced acquire_image failure' in rows[-1]['reason']
