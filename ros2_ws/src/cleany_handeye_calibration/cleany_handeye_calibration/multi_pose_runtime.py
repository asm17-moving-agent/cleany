"""ROS executable for a fresh, preflighted 20+5 pose MuJoCo run."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import threading
from typing import Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from cleany_handeye_calibration.dataset_writer import DatasetWriter
from cleany_handeye_calibration.pose_manifest import (
    MaterializedPose,
    PoseManifest,
    PoseRunConfiguration,
    RequiredStageTimeouts,
    load_pose_manifest,
)
from cleany_handeye_calibration.pose_run import (
    CommittedPoseSample,
    JsonlPoseRunJournal,
    MultiPoseRunOrchestrator,
    PoseAttemptFailure,
    PoseFailureCategory,
    PoseRunExecutor,
    RunCancelToken,
)
from cleany_handeye_calibration.single_pose_orchestrator import (
    JointSoftLimit,
    JsonlStageJournal,
    SinglePoseFailure,
    SinglePoseOrchestrator,
    SinglePoseRequest,
    SinglePoseSafetyProfile,
    SinglePoseStage,
    SinglePoseTimeouts,
)
from cleany_handeye_calibration.single_pose_runtime import (
    SinglePoseRuntimeEffects,
)
from cleany_handeye_calibration.single_pose_runtime_config import (
    ExpectedResolvedPoseEvidence,
    SinglePoseRuntimeConfig,
    load_single_pose_runtime_config,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _same(left: float, right: float) -> bool:
    return math.isclose(
        float(left), float(right), rel_tol=0.0, abs_tol=1.0e-12
    )


def _required_timeout(value: float | None, name: str) -> float:
    if value is None:
        raise ValueError(f'pose manifest timeout {name} is unresolved')
    return float(value)


def _single_pose_timeouts(values: RequiredStageTimeouts) -> SinglePoseTimeouts:
    return SinglePoseTimeouts(
        resolve_position_ik_sec=_required_timeout(values.ik_sec, 'ik_sec'),
        validate_resolved_pose_sec=_required_timeout(
            values.state_validity_sec, 'state_validity_sec'
        ),
        plan_sec=_required_timeout(values.plan_sec, 'plan_sec'),
        execute_sec=_required_timeout(values.execute_sec, 'execute_sec'),
        wait_settled_sec=_required_timeout(values.settle_sec, 'settle_sec'),
        acquire_image_sec=_required_timeout(
            values.image_acquisition_sec, 'image_acquisition_sec'
        ),
        detect_target_sec=_required_timeout(
            values.target_detection_sec, 'target_detection_sec'
        ),
        compute_feedback_fk_sec=_required_timeout(
            values.feedback_fk_sec, 'feedback_fk_sec'
        ),
        record_sample_sec=_required_timeout(
            values.record_sample_sec, 'record_sample_sec'
        ),
    )


def _fresh_pose_run_journal(writer: DatasetWriter) -> Path:
    if writer.read_samples():
        raise RuntimeError(
            'run already contains committed samples; choose a new run_id'
        )
    path = writer.run_directory / 'pose_run.jsonl'
    if path.exists():
        raise RuntimeError(
            'run already contains a pose journal; choose a new run_id'
        )
    return path


def _safety_profile(run: PoseRunConfiguration) -> SinglePoseSafetyProfile:
    if run.soft_joint_limits is None or run.collision_margin_m is None:
        raise ValueError('pose manifest safety profile is unresolved')
    return SinglePoseSafetyProfile(
        profile_id='materialized_pose_manifest',
        soft_joint_limits=tuple(
            JointSoftLimit(name, low, high)
            for name, low, high in zip(
                run.soft_joint_limits.joint_names,
                run.soft_joint_limits.lower_rad,
                run.soft_joint_limits.upper_rad,
                strict=True,
            )
        ),
        required_collision_margin_m=run.collision_margin_m,
    )


def validate_multi_pose_runtime_profile(
    manifest_path: Path,
    manifest: PoseManifest,
    runtime: SinglePoseRuntimeConfig,
) -> None:
    """Reject duplicated profile values that disagree before ROS motion."""

    run = manifest.run_config.require_ready()
    if runtime.dataset_manifest.source_hashes.pose_manifest_sha256 != _sha256(
        manifest_path
    ):
        raise ValueError('dataset provenance pose-manifest SHA-256 differs')
    if runtime.dataset_manifest.random_seed != manifest.generator.random_seed:
        raise ValueError('dataset random seed differs from pose manifest')
    first = manifest.poses[0]
    if (
        runtime.request.pose_id != first.pose_id
        or runtime.request.split is not first.split
        or runtime.request.target != first.target
        or runtime.request.ik_seed != first.ik_seed
        or runtime.expected.pose != first.resolved_joint_pose
    ):
        raise ValueError('runtime profile must be anchored to the first pose')
    if not _same(
        runtime.expected.observed_collision_clearance_m,
        first.validation.minimum_collision_distance_m,
    ):
        raise ValueError('first-pose collision evidence differs')
    if run.right_park_position_tolerance_rad is None or not _same(
        runtime.motion.right_park_position_tolerance_rad,
        run.right_park_position_tolerance_rad,
    ):
        raise ValueError('right-arm park tolerance differs')
    expected_safety = _safety_profile(run)
    actual_safety = runtime.request.safety_profile
    if (
        actual_safety.soft_joint_limits != expected_safety.soft_joint_limits
        or not _same(
            actual_safety.required_collision_margin_m,
            expected_safety.required_collision_margin_m,
        )
    ):
        raise ValueError('single-pose and pose-manifest safety values differ')

    values = run.stage_timeouts
    adapter_pairs = (
        (runtime.motion.stage_timeouts.ik_sec, values.ik_sec, 'ik'),
        (
            runtime.motion.stage_timeouts.state_validity_sec,
            values.state_validity_sec,
            'state_validity',
        ),
        (runtime.motion.stage_timeouts.plan_sec, values.plan_sec, 'plan'),
        (
            runtime.motion.stage_timeouts.execute_sec,
            values.execute_sec,
            'execute',
        ),
        (
            runtime.motion.stage_timeouts.cancel_sec,
            values.cancel_sec,
            'cancel',
        ),
        (
            runtime.motion.stage_timeouts.settle_sec,
            values.settle_sec,
            'settle',
        ),
    )
    for actual, expected, name in adapter_pairs:
        if expected is None or not _same(actual, expected):
            raise ValueError(f'{name} timeout differs between profiles')
    if runtime.request.timeouts != _single_pose_timeouts(values):
        raise ValueError('orchestration timeouts differ between profiles')


def _failure_category(error: SinglePoseFailure) -> PoseFailureCategory:
    mapping = {
        SinglePoseStage.RESOLVE_POSITION_IK: PoseFailureCategory.IK,
        SinglePoseStage.PLAN: PoseFailureCategory.PLANNING,
        SinglePoseStage.EXECUTE: PoseFailureCategory.CONTROLLER,
        SinglePoseStage.WAIT_SETTLED: PoseFailureCategory.SETTLE,
        SinglePoseStage.ACQUIRE_IMAGE: PoseFailureCategory.IMAGE_ACQUISITION,
        SinglePoseStage.DETECT_TARGET: PoseFailureCategory.TARGET_DETECTION,
        SinglePoseStage.COMPUTE_FEEDBACK_FK: (
            PoseFailureCategory.DATA_INTEGRITY
        ),
        SinglePoseStage.RECORD_SAMPLE: PoseFailureCategory.DATA_INTEGRITY,
    }
    if error.stage is SinglePoseStage.VALIDATE_RESOLVED_POSE:
        return (
            PoseFailureCategory.LIMIT
            if 'limit' in error.reason.lower()
            else PoseFailureCategory.COLLISION
        )
    return mapping[error.stage]


class RosPoseRunExecutor(PoseRunExecutor):
    def __init__(
        self,
        effects: SinglePoseRuntimeEffects,
        writer: DatasetWriter,
        safety_profile: SinglePoseSafetyProfile,
    ) -> None:
        self._effects = effects
        self._writer = writer
        self._safety = safety_profile

    def execute_pose(
        self,
        pose: MaterializedPose,
        *,
        attempt: int,
        stage_timeouts: RequiredStageTimeouts,
        cancel_requested: Callable[[], bool],
    ) -> CommittedPoseSample:
        if cancel_requested():
            raise PoseAttemptFailure(
                PoseFailureCategory.INTERNAL,
                'run canceled before pose attempt',
            )
        request = SinglePoseRequest(
            sample_id=pose.pose_id,
            pose_id=pose.pose_id,
            split=pose.split,
            target=pose.target,
            ik_seed=pose.ik_seed,
            timeouts=_single_pose_timeouts(stage_timeouts),
            safety_profile=self._safety,
            attempt=attempt,
        )
        stage_path = (
            self._writer.run_directory
            / 'pose_stages'
            / f'{pose.pose_id}.attempt_{attempt}.jsonl'
        )
        try:
            result = SinglePoseOrchestrator(
                self._effects,
                JsonlStageJournal(stage_path),
            ).run(request)
        except SinglePoseFailure as error:
            raise PoseAttemptFailure(
                _failure_category(error),
                error.reason,
            ) from error
        stored = result.stored_sample
        return CommittedPoseSample(
            pose_id=stored.record.sample.pose_id,
            sample_id=stored.record.sample.sample_id,
            split=stored.record.sample.split,
        )


def _run(pose_manifest_path: str, runtime_config_path: str) -> int:
    manifest_path = Path(pose_manifest_path).expanduser().resolve()
    manifest = load_pose_manifest(manifest_path)
    runtime = load_single_pose_runtime_config(runtime_config_path)
    validate_multi_pose_runtime_profile(manifest_path, manifest, runtime)
    expected = {
        pose.pose_id: ExpectedResolvedPoseEvidence(
            pose=pose.resolved_joint_pose,
            match_tolerance_rad=runtime.expected.match_tolerance_rad,
            observed_collision_clearance_m=(
                pose.validation.minimum_collision_distance_m
            ),
        )
        for pose in manifest.poses
    }
    node = Node(
        'multi_pose_handeye_orchestrator',
        parameter_overrides=[Parameter('use_sim_time', value=True)],
        automatically_declare_parameters_from_overrides=True,
    )
    writer = DatasetWriter(
        artifact_root=runtime.artifact_root,
        manifest=runtime.dataset_manifest,
    )
    pose_run_journal = _fresh_pose_run_journal(writer)
    effects = SinglePoseRuntimeEffects(
        node,
        runtime,
        writer,
        expected_by_pose_id=expected,
    )
    cancel_token = RunCancelToken()

    def cancel_callback(request, response):
        del request
        cancel_token.request()
        response.success = True
        response.message = 'calibration run cancellation requested'
        return response

    node.create_service(Trigger, '/handeye/cancel_run', cancel_callback)
    ros_executor = MultiThreadedExecutor(num_threads=4)
    ros_executor.add_node(node)
    executor_thread = threading.Thread(
        target=ros_executor.spin,
        name='multi-pose-ros-executor',
        daemon=True,
    )
    executor_thread.start()
    try:
        effects.wait_for_startup_state()
        effects.wait_for_planning_scene()
        pose_executor = RosPoseRunExecutor(
            effects,
            writer,
            _safety_profile(manifest.run_config),
        )
        summary = MultiPoseRunOrchestrator(
            pose_executor,
            JsonlPoseRunJournal(pose_run_journal),
            cancel_token,
        ).run(manifest)
        node.get_logger().info(
            'multi-pose calibration finished: '
            f'completed={len(summary.completed_pose_ids)}, '
            f'failed={len(summary.failed_pose_ids)}, '
            f'canceled={summary.canceled}'
        )
        return 0 if summary.success else 1
    except Exception as error:
        node.get_logger().error(f'multi-pose calibration failed: {error}')
        return 1
    finally:
        ros_executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        node.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    bootstrap = Node('multi_pose_handeye_config')
    bootstrap.declare_parameter('pose_manifest', '')
    bootstrap.declare_parameter('runtime_config', '')
    pose_manifest = bootstrap.get_parameter(
        'pose_manifest'
    ).get_parameter_value().string_value
    runtime_config = bootstrap.get_parameter(
        'runtime_config'
    ).get_parameter_value().string_value
    bootstrap.destroy_node()
    try:
        if not pose_manifest or not runtime_config:
            raise ValueError(
                'pose_manifest and runtime_config ROS parameters are required'
            )
        exit_code = _run(pose_manifest, runtime_config)
    except Exception as error:
        print(f'multi-pose calibration startup failed: {error}')
        exit_code = 2
    finally:
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
