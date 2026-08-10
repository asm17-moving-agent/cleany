"""ROS executable that wires the single-pose core to MoveIt and MuJoCo."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping

import numpy as np
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState

from cleany_handeye_calibration.camera_acquisition import CameraFramePair
from cleany_handeye_calibration.dataset_writer import DatasetWriter
from cleany_handeye_calibration.ik_port import (
    MoveItPositionIKAdapter,
    MoveItStateValidityAdapter,
)
from cleany_handeye_calibration.joint_state_sync import (
    DEFAULT_DUAL_ARM_JOINT_CONTRACT,
    InterpolatedJointState,
    JointStateRingBuffer,
)
from cleany_handeye_calibration.models import (
    CalibrationSample,
    JointPose,
    TimedJointSample,
)
from cleany_handeye_calibration.motion_config import (
    ValidatedCurrentState,
    validate_dual_arm_current_state,
)
from cleany_handeye_calibration.motion_port import (
    MoveItMotionAdapter,
    PlannedMotion,
)
from cleany_handeye_calibration.moveit_fk import (
    MoveItForwardKinematicsAdapter,
    timed_joint_sample_from_message,
)
from cleany_handeye_calibration.pnp import solve_planar_pnp
from cleany_handeye_calibration.ros_camera_adapter import (
    RosExactCameraPairAdapter,
)
from cleany_handeye_calibration.schema import CalibrationSampleRecord
from cleany_handeye_calibration.settle_detector import JointSettleDetector
from cleany_handeye_calibration.single_pose_orchestrator import (
    FeedbackFkObservation,
    JsonlStageJournal,
    ResolvedPoseValidation,
    SinglePoseOrchestrator,
    SinglePoseRequest,
    TargetObservation,
)
from cleany_handeye_calibration.single_pose_runtime_config import (
    ExpectedResolvedPoseEvidence,
    SinglePoseRuntimeConfig,
    load_single_pose_runtime_config,
)
from cleany_handeye_calibration.target_detector import CharucoTargetDetector


IMAGE_TOPIC = '/left_wrist_camera/image_raw'
CAMERA_INFO_TOPIC = '/left_wrist_camera/camera_info'
JOINT_STATE_TOPIC = '/joint_states'
REQUIRED_COLLISION_OBJECT_IDS = {
    'handeye_table',
    'handeye_target_stand',
    'charuco_target',
}


class SinglePoseRuntimeEffects:
    """Synchronous effects driven by a background ROS executor."""

    def __init__(
        self,
        node: Node,
        config: SinglePoseRuntimeConfig,
        writer: DatasetWriter,
        *,
        expected_by_pose_id: Mapping[
            str, ExpectedResolvedPoseEvidence
        ] | None = None,
    ) -> None:
        self._node = node
        self._config = config
        self._writer = writer
        expected_values = (
            {config.request.pose_id: config.expected}
            if expected_by_pose_id is None
            else dict(expected_by_pose_id)
        )
        if not expected_values or any(
            not isinstance(pose_id, str)
            or not pose_id
            or not isinstance(evidence, ExpectedResolvedPoseEvidence)
            for pose_id, evidence in expected_values.items()
        ):
            raise ValueError('expected_by_pose_id is invalid')
        self._expected_by_pose_id = expected_values
        self._state_condition = threading.Condition()
        feedback = config.feedback
        self._buffer = JointStateRingBuffer(
            capacity=feedback.capacity,
            max_sample_distance_ns=feedback.max_sample_distance_ns,
            clock_reset_threshold_ns=feedback.clock_reset_threshold_ns,
        )
        self._latest_sample: TimedJointSample | None = None
        self._settle_detector = JointSettleDetector(config.motion)
        self._settled_stamp_ns: int | None = None
        self._camera = RosExactCameraPairAdapter()
        self._detector = CharucoTargetDetector()

        # The executor thread resolves ROS futures and subscription callbacks.
        # Adapter polling therefore only yields the caller thread.
        def poll(timeout: float) -> None:
            time.sleep(min(float(timeout), 0.01))

        self._ik = MoveItPositionIKAdapter(
            node, config=config.motion, spin_once=poll
        )
        self._validity = MoveItStateValidityAdapter(
            node, config=config.motion, spin_once=poll
        )
        self._motion = MoveItMotionAdapter(
            node, config=config.motion, spin_once=poll
        )
        self._fk = MoveItForwardKinematicsAdapter(
            node, spin_once=poll
        )
        self._planning_scene_client = node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )

        node.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Image,
            IMAGE_TOPIC,
            self._camera.on_image,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self._camera.on_camera_info,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            sample = timed_joint_sample_from_message(message)
        except ValueError as error:
            self._node.get_logger().warning(
                f'rejected invalid JointState: {error}'
            )
            return
        with self._state_condition:
            insertion = self._buffer.add(sample)
            if not insertion.accepted:
                return
            self._latest_sample = sample
            if self._settle_detector.update(sample):
                self._settled_stamp_ns = sample.stamp_ns
            self._state_condition.notify_all()

    def wait_for_startup_state(self) -> ValidatedCurrentState:
        deadline = (
            time.monotonic()
            + self._config.feedback.startup_state_timeout_sec
        )
        last_error: Exception | None = None
        with self._state_condition:
            while True:
                if self._latest_sample is not None:
                    try:
                        return self._validated_current_state_locked()
                    except Exception as error:
                        last_error = error
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    detail = '' if last_error is None else f': {last_error}'
                    raise TimeoutError(
                        'timed out waiting for valid dual-arm feedback'
                        + detail
                    )
                self._state_condition.wait(timeout=remaining)

    def wait_for_planning_scene(self) -> None:
        deadline = (
            time.monotonic()
            + self._config.feedback.startup_planning_scene_timeout_sec
        )
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        while time.monotonic() < deadline:
            if not self._planning_scene_client.service_is_ready():
                time.sleep(0.02)
                continue
            future = self._planning_scene_client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if future.done():
                response = future.result()
                if response is not None:
                    present = {
                        item.id
                        for item in response.scene.world.collision_objects
                    }
                    if REQUIRED_COLLISION_OBJECT_IDS <= present:
                        return
            time.sleep(0.02)
        raise TimeoutError(
            'required hand-eye collision objects were not applied'
        )

    def _validated_current_state_locked(self) -> ValidatedCurrentState:
        assert self._latest_sample is not None
        return validate_dual_arm_current_state(
            self._latest_sample,
            now_stamp_ns=self._node.get_clock().now().nanoseconds,
            config=self._config.motion,
        )

    def _current_state(self) -> ValidatedCurrentState:
        with self._state_condition:
            if self._latest_sample is None:
                raise RuntimeError('joint feedback is unavailable')
            return self._validated_current_state_locked()

    def resolve_position_ik(
        self, request: SinglePoseRequest, timeout_sec: float
    ) -> JointPose:
        result = self._ik.solve_position(
            request.target,
            request.ik_seed,
            current_state=self._current_state(),
        )
        if not result.success:
            raise RuntimeError(f'position IK failed: {result.failure_reason}')
        assert result.joint_pose is not None
        return result.joint_pose

    def validate_resolved_pose(
        self,
        request: SinglePoseRequest,
        resolved_pose: JointPose,
        timeout_sec: float,
    ) -> ResolvedPoseValidation:
        try:
            expected = self._expected_by_pose_id[request.pose_id]
        except KeyError as error:
            raise ValueError(
                f'no resolved-pose evidence for {request.pose_id}'
            ) from error
        expected.validate_match(resolved_pose)
        result = self._validity.validate(
            resolved_pose,
            current_state=self._current_state(),
        )
        if not result.valid:
            raise RuntimeError(
                f'MoveIt state validity failed: {result.status.value}'
            )
        assert result.validated_goal is not None
        return ResolvedPoseValidation(
            validated_goal=result.validated_goal,
            observed_collision_clearance_m=(
                expected.observed_collision_clearance_m
            ),
        )

    def plan(
        self,
        validation: ResolvedPoseValidation,
        timeout_sec: float,
    ) -> PlannedMotion:
        result = self._motion.plan(
            validation.validated_goal,
            current_state=self._current_state(),
        )
        if not result.success:
            raise RuntimeError(
                'MoveIt planning failed: '
                f'{result.status.value}, error={result.moveit_error_code}'
            )
        assert result.planned_motion is not None
        return result.planned_motion

    def execute(self, planned_motion: Any, timeout_sec: float) -> None:
        if not isinstance(planned_motion, PlannedMotion):
            raise ValueError('planned motion has an invalid type')
        result = self._motion.execute(
            planned_motion,
            current_state=self._current_state(),
        )
        if not result.success:
            raise RuntimeError(
                'MoveIt execution failed: '
                f'{result.status.value}, error={result.moveit_error_code}, '
                f'cancel_confirmed={result.cancel_confirmed}'
            )

    def wait_settled(
        self, resolved_pose: JointPose, timeout_sec: float
    ) -> int:
        deadline = time.monotonic() + timeout_sec
        with self._state_condition:
            self._settled_stamp_ns = None
            self._settle_detector.begin(
                resolved_pose,
                action_succeeded=True,
            )
            while self._settled_stamp_ns is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    latest = self._latest_sample
                    diagnostic = ''
                    if latest is not None:
                        positions = dict(
                            zip(
                                latest.joint_names,
                                latest.positions_rad,
                                strict=True,
                            )
                        )
                        target = dict(
                            zip(
                                resolved_pose.joint_names,
                                resolved_pose.positions_rad,
                                strict=True,
                            )
                        )
                        max_position_error = max(
                            abs(positions[name] - target[name])
                            for name in target
                        )
                        max_velocity = None
                        if latest.velocities_rad_s is not None:
                            velocities = dict(
                                zip(
                                    latest.joint_names,
                                    latest.velocities_rad_s,
                                    strict=True,
                                )
                            )
                            max_velocity = max(
                                abs(velocities[name]) for name in target
                            )
                        diagnostic = (
                            f'; last_reset='
                            f'{self._settle_detector.last_reset_reason}, '
                            f'max_position_error={max_position_error:g}, '
                            f'max_velocity={max_velocity}'
                        )
                    raise TimeoutError(
                        'left arm did not satisfy the continuous feedback '
                        'settle contract' + diagnostic
                    )
                self._state_condition.wait(timeout=remaining)
            return self._settled_stamp_ns

    def acquire_image(
        self, settled_stamp_ns: int, timeout_sec: float
    ) -> CameraFramePair:
        return self._camera.wait_for_first_compatible_frame(
            settled_stamp_ns=settled_stamp_ns,
            timeout_sec=timeout_sec,
        )

    @staticmethod
    def _image_array(pair: CameraFramePair) -> np.ndarray:
        image = pair.image
        array = np.frombuffer(image.data, dtype=np.uint8)
        if image.encoding == 'rgb8':
            return array.reshape((image.height, image.width, 3))
        if image.encoding == 'mono8':
            return array.reshape((image.height, image.width))
        raise ValueError(f'unsupported image encoding: {image.encoding}')

    def detect_target(
        self, pair: CameraFramePair, timeout_sec: float
    ) -> TargetObservation:
        detection = self._detector.detect(self._image_array(pair))
        if not detection.valid:
            reason = (
                'unknown'
                if detection.failure_reason is None
                else detection.failure_reason.value
            )
            raise RuntimeError(
                'ChArUco detection failed: '
                f'{reason}; corner_count={len(detection.corner_ids)}; '
                f'quadrants={detection.covered_quadrants}'
            )
        camera = pair.camera_info
        pnp = solve_planar_pnp(
            detection,
            camera_matrix=np.asarray(camera.k).reshape((3, 3)),
            distortion_coefficients=camera.d,
            camera_frame=camera.frame_id,
            target_frame='charuco_target',
        )
        if not pnp.valid:
            reason = (
                'unknown'
                if pnp.failure_reason is None
                else pnp.failure_reason.value
            )
            raise RuntimeError(f'planar PnP failed: {reason}')
        return TargetObservation(pair=pair, detection=detection, pnp=pnp)

    def _wait_for_interpolation(
        self, image_stamp_ns: int, timeout_sec: float
    ) -> InterpolatedJointState:
        deadline = time.monotonic() + timeout_sec
        with self._state_condition:
            while True:
                result = self._buffer.interpolate(image_stamp_ns)
                if result.success:
                    assert result.interpolation is not None
                    return result.interpolation
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    assert result.failure is not None
                    raise TimeoutError(
                        'cannot bracket image with joint feedback: '
                        f'{result.failure.value}'
                    )
                self._state_condition.wait(
                    timeout=min(remaining, 0.05)
                )

    @staticmethod
    def _canonical_interpolation(
        interpolation: InterpolatedJointState,
    ) -> InterpolatedJointState:
        names = DEFAULT_DUAL_ARM_JOINT_CONTRACT.required_joint_names
        source = interpolation.sample
        positions = dict(
            zip(source.joint_names, source.positions_rad, strict=True)
        )
        velocities = None
        if source.velocities_rad_s is not None:
            source_velocities = dict(
                zip(
                    source.joint_names,
                    source.velocities_rad_s,
                    strict=True,
                )
            )
            velocities = tuple(source_velocities[name] for name in names)
        return InterpolatedJointState(
            sample=TimedJointSample(
                stamp_ns=source.stamp_ns,
                joint_names=names,
                positions_rad=tuple(positions[name] for name in names),
                velocities_rad_s=velocities,
            ),
            before_stamp_ns=interpolation.before_stamp_ns,
            after_stamp_ns=interpolation.after_stamp_ns,
            ratio=interpolation.ratio,
        )

    def compute_feedback_fk(
        self, observation: TargetObservation, timeout_sec: float
    ) -> FeedbackFkObservation:
        interpolation = self._canonical_interpolation(
            self._wait_for_interpolation(
                observation.pair.stamp_ns,
                timeout_sec,
            )
        )
        transform = self._fk.compute(
            interpolation,
            'left_gripper_frame',
            timeout_sec=timeout_sec,
        )
        return FeedbackFkObservation(
            interpolation=interpolation,
            base_T_gripper=transform,
        )

    def record_sample(
        self,
        request: SinglePoseRequest,
        resolved_pose: JointPose,
        observation: TargetObservation,
        feedback_fk: FeedbackFkObservation,
        timeout_sec: float,
    ):
        pnp = observation.pnp
        assert pnp.camera_T_target is not None
        assert pnp.selected_candidate_index is not None
        candidate = next(
            candidate
            for candidate in pnp.candidates
            if candidate.index == pnp.selected_candidate_index
        )
        rmse = candidate.refined_reprojection_rmse_px
        if rmse is None or not math.isfinite(rmse):
            raise RuntimeError('selected PnP candidate has no finite RMSE')
        interpolation = feedback_fk.interpolation
        record = CalibrationSampleRecord(
            sample=CalibrationSample(
                sample_id=request.sample_id,
                pose_id=request.pose_id,
                split=request.split,
                base_T_gripper=feedback_fk.base_T_gripper,
                camera_T_target=pnp.camera_T_target,
            ),
            calibration_arm='left',
            planning_group='left_arm',
            position_target=request.target,
            ik_seed=request.ik_seed,
            resolved_ik=resolved_pose,
            image_stamp_ns=observation.pair.stamp_ns,
            joint_state_before_stamp_ns=(
                interpolation.before_stamp_ns
            ),
            joint_state_after_stamp_ns=interpolation.after_stamp_ns,
            joint_interpolation_ratio=interpolation.ratio,
            interpolated_joints=interpolation.sample,
            camera_info=observation.pair.camera_info,
            target_detection=observation.detection,
            pnp_method=pnp.method,
            pnp_reprojection_rmse_px=rmse,
            pnp_ambiguous=pnp.ambiguous,
            pnp_selected_candidate_index=pnp.selected_candidate_index,
            image_path=f'images/{request.sample_id}.png',
        )
        return self._writer.append_sample(record, observation.pair)


def _run(request_file: str) -> int:
    config = load_single_pose_runtime_config(request_file)
    node = Node(
        'single_pose_handeye_orchestrator',
        parameter_overrides=[
            Parameter('use_sim_time', value=True)
        ],
        automatically_declare_parameters_from_overrides=True,
    )
    writer = DatasetWriter(
        artifact_root=config.artifact_root,
        manifest=config.dataset_manifest,
    )
    journal = JsonlStageJournal(
        writer.run_directory / 'orchestration.jsonl'
    )
    effects = SinglePoseRuntimeEffects(node, config, writer)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name='single-pose-ros-executor',
        daemon=True,
    )
    executor_thread.start()
    try:
        effects.wait_for_startup_state()
        effects.wait_for_planning_scene()
        result = SinglePoseOrchestrator(effects, journal).run(
            config.request
        )
        node.get_logger().info(
            'Stored single-pose hand-eye sample '
            f'{result.stored_sample.record.sample.sample_id}'
        )
        return 0
    except Exception as error:
        node.get_logger().error(f'single-pose calibration failed: {error}')
        return 1
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        node.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    bootstrap = Node('single_pose_handeye_config')
    bootstrap.declare_parameter('request_file', '')
    request_file = bootstrap.get_parameter(
        'request_file'
    ).get_parameter_value().string_value
    bootstrap.destroy_node()
    try:
        if not request_file:
            raise ValueError('request_file ROS parameter is required')
        exit_code = _run(request_file)
    except Exception as error:
        print(f'single-pose calibration startup failed: {error}')
        exit_code = 2
    finally:
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
