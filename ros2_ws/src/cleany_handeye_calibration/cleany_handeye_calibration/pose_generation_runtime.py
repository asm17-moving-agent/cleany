"""Live MoveIt evaluator and executable for deterministic pose generation."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import time

from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from cleany_handeye_calibration.ik_port import (
    MoveItPositionIKAdapter,
    MoveItStateValidityAdapter,
)
from cleany_handeye_calibration.joint_state_sync import (
    InterpolatedJointState,
)
from cleany_handeye_calibration.models import (
    JointPose,
    PositionTarget,
    TimedJointSample,
)
from cleany_handeye_calibration.motion_config import (
    ValidatedCurrentState,
    validate_dual_arm_current_state,
)
from cleany_handeye_calibration.motion_port import MoveItMotionAdapter
from cleany_handeye_calibration.moveit_fk import (
    CALIBRATION_LINK,
    MoveItForwardKinematicsAdapter,
    timed_joint_sample_from_message,
)
from cleany_handeye_calibration.mujoco_pose_evidence import (
    MujocoPoseEvidenceEvaluator,
    MujocoRenderedTargetEvaluator,
)
from cleany_handeye_calibration.pose_generation import (
    EvaluatedPoseCandidate,
    PoseCandidateRequest,
    PoseTargetSampler,
    generate_pose_manifest,
)
from cleany_handeye_calibration.pose_generation_artifacts import (
    materialize_pose_generation_artifacts,
)
from cleany_handeye_calibration.pose_generation_profile import (
    MujocoPoseGenerationProfile,
    load_mujoco_pose_generation_profile,
)
from cleany_handeye_calibration.pose_manifest import (
    CartesianBounds,
    PoseRunConfiguration,
    PoseValidationEvidence,
)
from cleany_handeye_calibration.transforms import (
    quaternion_xyzw_from_rotation_matrix,
)


REQUIRED_COLLISION_OBJECT_IDS = {
    'handeye_table',
    'handeye_target_stand',
    'charuco_target',
}


class MujocoSeedLocalTargetSampler(PoseTargetSampler):
    """Keep random targets near visible, clear random seed configurations."""

    def __init__(
        self,
        scene: MujocoPoseEvidenceEvaluator,
        rendered_target: MujocoRenderedTargetEvaluator,
        profile: MujocoPoseGenerationProfile,
    ) -> None:
        self._scene = scene
        self._rendered_target = rendered_target
        self._profile = profile
        self.rejections: Counter[str] = Counter()

    def sample_target(
        self,
        seed: JointPose,
        rng: np.random.Generator,
        bounds: CartesianBounds,
    ) -> PositionTarget | None:
        evidence = self._scene.evaluate(seed)
        assert self._profile.run_config.collision_margin_m is not None
        if not evidence.target_visible:
            self.rejections['seed_target_not_visible'] += 1
            return None
        if (
            evidence.minimum_collision_distance_m
            < self._profile.run_config.collision_margin_m
        ):
            self.rejections['seed_fixture_clearance'] += 1
            return None
        detection = self._rendered_target.evaluate(seed)
        if not detection.detected:
            reason = detection.failure_reason or 'invalid'
            self.rejections[f'seed_charuco:{reason}'] += 1
            return None
        if not detection.pnp_valid:
            reason = detection.pnp_failure_reason or 'invalid'
            self.rejections[f'seed_pnp:{reason}'] += 1
            return None
        center = np.asarray(evidence.base_gripper_position_m)
        radius = np.asarray(self._profile.target_seed_local_radius_m)
        global_lower = np.asarray(
            (
                bounds.x_m.minimum,
                bounds.y_m.minimum,
                bounds.z_m.minimum,
            )
        )
        global_upper = np.asarray(
            (
                bounds.x_m.maximum,
                bounds.y_m.maximum,
                bounds.z_m.maximum,
            )
        )
        lower = np.maximum(global_lower, center - radius)
        upper = np.minimum(global_upper, center + radius)
        if np.any(lower >= upper):
            self.rejections['seed_outside_target_bounds'] += 1
            return None
        return PositionTarget(
            frame_id='base_link',
            position_m=tuple(rng.uniform(lower, upper)),
        )


class RosPoseCandidateEvaluator:
    def __init__(
        self,
        node: Node,
        profile: MujocoPoseGenerationProfile,
        scene: MujocoPoseEvidenceEvaluator,
        rendered_target: MujocoRenderedTargetEvaluator,
    ) -> None:
        self._node = node
        self._profile = profile
        self._latest_sample: TimedJointSample | None = None
        self._rejections: Counter[str] = Counter()
        self._accepted = 0
        node.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            20,
        )
        self._planning_scene = node.create_client(
            GetPlanningScene,
            '/get_planning_scene',
        )
        self._ik = MoveItPositionIKAdapter(
            node,
            config=profile.motion,
        )
        self._validity = MoveItStateValidityAdapter(
            node,
            config=profile.motion,
        )
        self._motion = MoveItMotionAdapter(
            node,
            config=profile.motion,
        )
        self._fk = MoveItForwardKinematicsAdapter(node)
        self._scene = scene
        self._rendered_target = rendered_target

    @property
    def rejection_counts(self) -> dict[str, int]:
        return dict(sorted(self._rejections.items()))

    def _on_joint_state(self, message: JointState) -> None:
        try:
            self._latest_sample = timed_joint_sample_from_message(message)
        except ValueError as error:
            self._node.get_logger().warning(
                f'rejected invalid JointState during pose generation: {error}'
            )

    def wait_until_ready(self) -> ValidatedCurrentState:
        deadline = (
            time.monotonic()
            + self._profile.feedback.startup_state_timeout_sec
        )
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            try:
                state = self._current_state()
            except (RuntimeError, ValueError):
                continue
            if self._required_scene_objects_present(deadline):
                return state
        raise TimeoutError(
            'joint feedback or required MoveIt collision scene was not ready'
        )

    def _required_scene_objects_present(self, deadline: float) -> bool:
        if not self._planning_scene.service_is_ready():
            self._planning_scene.wait_for_service(timeout_sec=0.05)
            return False
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = self._planning_scene.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            return False
        present = {
            item.id
            for item in future.result().scene.world.collision_objects
        }
        return REQUIRED_COLLISION_OBJECT_IDS <= present

    def _current_state(self) -> ValidatedCurrentState:
        if self._latest_sample is None:
            raise RuntimeError('joint feedback is unavailable')
        return validate_dual_arm_current_state(
            self._latest_sample,
            now_stamp_ns=self._node.get_clock().now().nanoseconds,
            config=self._profile.motion,
        )

    @staticmethod
    def _hypothetical_state(
        feedback: TimedJointSample,
        left_pose: JointPose | None,
    ) -> InterpolatedJointState:
        positions = dict(
            zip(
                feedback.joint_names,
                feedback.positions_rad,
                strict=True,
            )
        )
        if left_pose is not None:
            positions.update(
                zip(
                    left_pose.joint_names,
                    left_pose.positions_rad,
                    strict=True,
                )
            )
        sample = TimedJointSample(
            stamp_ns=feedback.stamp_ns,
            joint_names=feedback.joint_names,
            positions_rad=tuple(
                positions[name] for name in feedback.joint_names
            ),
            velocities_rad_s=feedback.velocities_rad_s,
        )
        before = max(0, sample.stamp_ns - 1)
        after = sample.stamp_ns + 1
        ratio = (sample.stamp_ns - before) / (after - before)
        return InterpolatedJointState(
            sample=sample,
            before_stamp_ns=before,
            after_stamp_ns=after,
            ratio=ratio,
        )

    def reference_rotation_quaternion(
        self,
    ) -> tuple[float, float, float, float]:
        current = self._current_state()
        transform = self._fk.compute(
            self._hypothetical_state(current.sample, None),
            CALIBRATION_LINK,
            timeout_sec=(
                self._profile.run_config.stage_timeouts.feedback_fk_sec
            ),
        )
        return quaternion_xyzw_from_rotation_matrix(
            transform.rotation_matrix
        )

    def _reject(self, reason: str) -> None:
        self._rejections[reason] += 1
        total = sum(self._rejections.values()) + self._accepted
        if total % 25 == 0:
            self._node.get_logger().info(
                f'pose generation progress: attempts={total}, '
                f'accepted={self._accepted}, '
                f'rejections={self.rejection_counts}'
            )

    def evaluate(
        self,
        request: PoseCandidateRequest,
        run_config: PoseRunConfiguration,
    ) -> EvaluatedPoseCandidate | None:
        current = self._current_state()
        ik = self._ik.solve_position(
            request.target,
            request.ik_seed,
            current_state=current,
        )
        if not ik.success:
            self._reject(f'ik:{ik.failure_reason}')
            return None
        assert ik.joint_pose is not None
        resolved = ik.joint_pose
        assert run_config.soft_joint_limits is not None
        if not run_config.soft_joint_limits.contains(resolved):
            self._reject('soft_joint_limit')
            return None

        validity = self._validity.validate(
            resolved,
            current_state=self._current_state(),
        )
        if not validity.valid:
            self._reject(f'state_validity:{validity.status.value}')
            return None
        assert validity.validated_goal is not None

        transform = self._fk.compute(
            self._hypothetical_state(self._current_state().sample, resolved),
            CALIBRATION_LINK,
            timeout_sec=run_config.stage_timeouts.feedback_fk_sec,
        )
        target_error = float(
            np.linalg.norm(
                np.asarray(transform.translation_m)
                - np.asarray(request.target.position_m)
            )
        )
        assert run_config.target_position_tolerance_m is not None
        if target_error > run_config.target_position_tolerance_m:
            self._reject('target_position_error')
            return None

        scene = self._scene.evaluate(resolved)
        assert run_config.collision_margin_m is not None
        if scene.minimum_collision_distance_m < run_config.collision_margin_m:
            self._reject('fixture_clearance')
            return None
        if not scene.camera_front:
            self._reject('camera_back_side')
            return None
        if not scene.target_visible:
            self._reject('target_out_of_frame')
            return None
        detection = self._rendered_target.evaluate(resolved)
        if not detection.detected:
            reason = detection.failure_reason or 'invalid'
            self._reject(f'charuco:{reason}')
            return None
        if not detection.pnp_valid:
            reason = detection.pnp_failure_reason or 'invalid'
            self._reject(f'pnp:{reason}')
            return None

        plan = self._motion.plan(
            validity.validated_goal,
            current_state=self._current_state(),
        )
        if not plan.success:
            self._reject(f'planning:{plan.status.value}')
            return None

        self._accepted += 1
        self._node.get_logger().info(
            f'accepted {request.candidate_id}: '
            f'pool={self._accepted}/{self._profile.candidate_pool_size}, '
            f'clearance={scene.minimum_collision_distance_m:.4f} m, '
            f'target_error={target_error:.6f} m, '
            f'charuco_corners={detection.corner_count}, '
            'pnp_rmse='
            f'{detection.selected_reprojection_rmse_px:.6f} px'
        )
        return EvaluatedPoseCandidate(
            request=request,
            resolved_joint_pose=resolved,
            base_T_gripper=transform,
            validation=PoseValidationEvidence(
                target_position_error_m=target_error,
                minimum_collision_distance_m=(
                    scene.minimum_collision_distance_m
                ),
                planning_succeeded=True,
                target_visible=True,
                camera_front=True,
            ),
        )


def _required_string(node: Node, name: str) -> str:
    value = node.get_parameter(name).get_parameter_value().string_value
    if not value:
        raise ValueError(f'{name} parameter is required')
    return value


def _run(node: Node) -> None:
    profile_path = Path(_required_string(node, 'profile')).resolve(strict=True)
    scene_path = Path(
        _required_string(node, 'scene_path')
    ).resolve(strict=True)
    output_directory = Path(_required_string(node, 'output_directory'))
    artifact_root = Path(_required_string(node, 'artifact_root'))
    repository_root = Path(_required_string(node, 'repository_root'))
    run_id = _required_string(node, 'run_id')
    profile = load_mujoco_pose_generation_profile(profile_path)
    scene = MujocoPoseEvidenceEvaluator(
        scene_path,
        minimum_camera_depth_m=(
            profile.visibility.minimum_camera_depth_m
        ),
        image_border_fraction=profile.visibility.image_border_fraction,
    )
    rendered_target = MujocoRenderedTargetEvaluator(scene_path)
    evaluator = RosPoseCandidateEvaluator(
        node,
        profile,
        scene,
        rendered_target,
    )
    target_sampler = MujocoSeedLocalTargetSampler(
        scene,
        rendered_target,
        profile,
    )
    try:
        evaluator.wait_until_ready()
        reference = evaluator.reference_rotation_quaternion()
        manifest = generate_pose_manifest(
            profile.generation_config(reference),
            evaluator,
            target_sampler,
        )
    finally:
        rendered_target.close()
    manifest_path, runtime_path, urdf_path = (
        materialize_pose_generation_artifacts(
            output_directory=output_directory,
            artifact_root=artifact_root,
            run_id=run_id,
            manifest=manifest,
            profile=profile,
            repository_root=repository_root,
            scene_template_path=scene_path,
        )
    )
    diversity = manifest.selection.diversity
    node.get_logger().info(
        'materialized random pose set: '
        f'manifest={manifest_path}, runtime={runtime_path}, urdf={urdf_path}, '
        f'max_axis_parallelism={diversity.maximum_axis_parallelism:.9f}, '
        f'covariance_log_det={diversity.rotation_covariance_log_det:.9f}, '
        f'covariance_rank={diversity.rotation_covariance_rank}, '
        f'sampler_rejections={dict(target_sampler.rejections)}, '
        f'evaluator_rejections={evaluator.rejection_counts}'
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node(
        'handeye_pose_generator',
        parameter_overrides=[Parameter('use_sim_time', value=True)],
        automatically_declare_parameters_from_overrides=True,
    )
    for name in (
        'profile',
        'scene_path',
        'output_directory',
        'artifact_root',
        'repository_root',
        'run_id',
    ):
        if not node.has_parameter(name):
            node.declare_parameter(name, '')
    try:
        _run(node)
        exit_code = 0
    except Exception as error:
        node.get_logger().error(f'pose generation failed: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
