"""ROS action server selecting the first MoveIt-reachable grasp pair."""

from __future__ import annotations

import math
import threading
import time
import uuid

import rclpy
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import GraspCandidate
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.msg import VisibilityConstraint
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from cleany_skill_executor.core.grasp_selection import (
    Candidate,
    EvaluationStage,
    GraspSelectionConfig,
    GraspSelector,
    InfrastructureError,
    REQUIRED_JOINT_NAMES,
)
from cleany_skill_executor.moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter
from cleany_skill_executor.planning_scene import SceneAwarePort, TargetSceneTransaction
from cleany_skill_executor.collision_geometry_cache import CollisionGeometryCache, subscribe_collision_geometry
from cleany_skill_executor.service_trace import ServiceTrace
from cleany_skill_executor.core.visibility import enclosing_visibility_cone
from cleany_skill_executor.core.gripper import aperture_centering_offset


STAGE_CONSTANT = {
    EvaluationStage.PREGRASP_IK: SelectReachableGrasp.Feedback.STAGE_PREGRASP_IK,
    EvaluationStage.GRASP_IK: SelectReachableGrasp.Feedback.STAGE_GRASP_IK,
    EvaluationStage.STATE_VALIDITY: SelectReachableGrasp.Feedback.STAGE_STATE_VALIDITY,
    EvaluationStage.PLAN_PREGRASP: SelectReachableGrasp.Feedback.STAGE_PLAN_PREGRASP,
    EvaluationStage.PLAN_GRASP: SelectReachableGrasp.Feedback.STAGE_PLAN_GRASP,
}


class GraspSelectionNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('grasp_selection_server', **kwargs)
        defaults = {
            'action_name': 'grasp/select_reachable',
            'joint_state_topic': 'joint_states',
            'planning_frame': 'base_link',
            'joint_state_max_age_sec': 0.5,
            'service_artifact_directory': '',
            'ik_timeout_sec': 0.15,
            'ik_response_margin_sec': 1.0,
            'pregrasp_aim_ik_timeout_sec': 1.0,
            'state_validity_timeout_sec': 1.0,
            'fk_timeout_sec': 1.0,
            'pregrasp_position_tolerance_m': 0.005,
            'grasp_position_tolerance_m': 0.005,
            'pregrasp_preferred_approach_tolerance_deg': 5.0,
            'pregrasp_approach_tolerance_deg': 15.0,
            'pregrasp_closing_tolerance_deg': 30.0,
            'grasp_closing_tolerance_deg': 30.0,
            'grasp_closing_sign_invariant': True,
            'pregrasp_aim_attempts': 8,
            'grasp_pose_seed_attempts': 0,
            'align_grasp_wrist_roll': False,
            'pose_refinement_iterations': 0,
            'pose_refinement_position_weight': 1.0,
            'joint_limit_margin_rad': 0.0,
            'wrist_roll_lower_rad': -2.743847297,
            'wrist_roll_upper_rad': 2.84120630938,
            'planning_timeout_sec': 4.0,
            'planning_response_margin_sec': 1.0,
            'planning_attempts': 3,
            'velocity_scaling': 0.08,
            'acceleration_scaling': 0.08,
            'maximum_candidates': 12,
            'action_timeout_sec': 120.0,
            'pregrasp_offset_m': 0.14,
            'pregrasp_seed_offset_m': 0.08,
            'grasp_approach_offset_m': 0.0,
            'grasp_lateral_offset_m': 0.0,
            'grasp_use_aperture_centering': False,
            'grasp_aperture_margin_m': 0.008,
            'grasp_fixed_jaw_inner_x_m': 0.008,
            'grasp_fixed_jaw_clearance_m': 0.0,
            'grasp_execution_lateral_offset_m': math.nan,
            'require_pregrasp_visibility': False,
            'require_open_grasp_clearance': False,
            'require_gripper_closure_clearance': False,
            'support_patch_margin_m': 0.0,
            'planning_scene_timeout_sec': 1.0,
            'use_observed_collision_geometry': False,
            'collision_geometry_topic': '/grasp/collision_geometry',
            'selection_gripper_close_position_rad': -0.3,
            'gripper_sweep_step_rad': .05,
            'selection_gripper_open_position_rad': 1.4,
            'visibility_camera_frame': 'head_camera_rgb_optical_frame',
            'visibility_camera_max_age_sec': 0.5,
            'visibility_padding_m': 0.003,
            'visibility_cone_sides': 16,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._visibility_tf = None
        if bool(self.get_parameter('require_pregrasp_visibility').value):
            if not str(self.get_parameter('visibility_camera_frame').value):
                raise ValueError('visibility_camera_frame is required')
            self._visibility_tf = Buffer()
            self._visibility_listener = TransformListener(self._visibility_tf, self)
        self._joint_state: JointState | None = None
        self._joint_state_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_active = False
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state,
            20,
            callback_group=callback_group,
        )

        def wait(duration: float) -> None:
            time.sleep(min(duration, 0.01))

        directory = str(self.get_parameter('service_artifact_directory').value)
        trace = ServiceTrace(directory, self.get_logger().warning) if directory else None
        self._adapter = MoveItGraspAdapter(
            self,
            MoveItAdapterConfig(
                base_frame=str(self.get_parameter('planning_frame').value),
                ik_timeout_sec=float(self.get_parameter('ik_timeout_sec').value),
                ik_response_margin_sec=float(self.get_parameter('ik_response_margin_sec').value),
                planning_response_margin_sec=float(self.get_parameter('planning_response_margin_sec').value),
                pregrasp_aim_ik_timeout_sec=float(
                    self.get_parameter('pregrasp_aim_ik_timeout_sec').value
                ),
                state_validity_timeout_sec=float(self.get_parameter('state_validity_timeout_sec').value),
                fk_timeout_sec=float(self.get_parameter('fk_timeout_sec').value),
                pregrasp_position_tolerance_m=float(
                    self.get_parameter('pregrasp_position_tolerance_m').value
                ),
                grasp_position_tolerance_m=float(self.get_parameter('grasp_position_tolerance_m').value),
                pose_refinement_position_weight=float(self.get_parameter('pose_refinement_position_weight').value),
                pregrasp_preferred_approach_tolerance_deg=float(
                    self.get_parameter(
                        'pregrasp_preferred_approach_tolerance_deg'
                    ).value
                ),
                pregrasp_approach_tolerance_deg=float(
                    self.get_parameter('pregrasp_approach_tolerance_deg').value
                ),
                pregrasp_closing_tolerance_deg=float(
                    self.get_parameter('pregrasp_closing_tolerance_deg').value
                ),
                grasp_closing_tolerance_deg=float(
                    self.get_parameter('grasp_closing_tolerance_deg').value
                ),
                grasp_closing_sign_invariant=bool(
                    self.get_parameter('grasp_closing_sign_invariant').value
                ),
                pregrasp_aim_attempts=int(
                    self.get_parameter('pregrasp_aim_attempts').value
                ),
                grasp_pose_seed_attempts=int(
                    self.get_parameter('grasp_pose_seed_attempts').value
                ),
                align_grasp_wrist_roll=bool(self.get_parameter('align_grasp_wrist_roll').value),
                pose_refinement_iterations=int(self.get_parameter('pose_refinement_iterations').value),
                joint_limit_margin_rad=float(self.get_parameter('joint_limit_margin_rad').value),
                wrist_roll_lower_rad=float(
                    self.get_parameter('wrist_roll_lower_rad').value
                ),
                wrist_roll_upper_rad=float(
                    self.get_parameter('wrist_roll_upper_rad').value
                ),
                planning_timeout_sec=float(self.get_parameter('planning_timeout_sec').value),
                planning_attempts=int(self.get_parameter('planning_attempts').value),
                velocity_scaling=float(self.get_parameter('velocity_scaling').value),
                acceleration_scaling=float(self.get_parameter('acceleration_scaling').value),
            ),
            spin_once=wait,
            service_trace=trace,
        )
        use_mesh = bool(self.get_parameter('use_observed_collision_geometry').value)
        self._geometry_cache = CollisionGeometryCache()
        self._geometry_subscription = (subscribe_collision_geometry(self, self._geometry_cache,
            callback_group=callback_group) if use_mesh else None)
        self._scene = TargetSceneTransaction(self, spin_once=wait,
            timeout_sec=float(self.get_parameter('planning_scene_timeout_sec').value),
            support_patch_margin_m=float(self.get_parameter('support_patch_margin_m').value),
            geometry_lookup=self._geometry_cache.get if use_mesh else None)
        self._description_subscription = self.create_subscription(
            String, '/robot_description', self._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=callback_group)
        self._scene_port = SceneAwarePort(self._adapter, self._scene)
        execution_lateral_offset = float(
            self.get_parameter('grasp_execution_lateral_offset_m').value
        )
        self._selector = GraspSelector(
            self._scene_port,
            GraspSelectionConfig(
                pregrasp_offset_m=float(self.get_parameter('pregrasp_offset_m').value),
                pregrasp_seed_offset_m=float(
                    self.get_parameter('pregrasp_seed_offset_m').value
                ),
                grasp_approach_offset_m=float(
                    self.get_parameter('grasp_approach_offset_m').value
                ),
                grasp_lateral_offset_m=float(
                    self.get_parameter('grasp_lateral_offset_m').value
                ),
                grasp_execution_lateral_offset_m=(
                    None
                    if math.isnan(execution_lateral_offset)
                    else execution_lateral_offset
                ),
                maximum_candidates=int(self.get_parameter('maximum_candidates').value),
                closed_gripper_position_rad=(float(self.get_parameter(
                    'selection_gripper_close_position_rad').value)
                    if self.get_parameter('require_gripper_closure_clearance').value else None),
                gripper_sweep_step_rad=float(self.get_parameter('gripper_sweep_step_rad').value),
                require_pregrasp_visibility=bool(
                    self.get_parameter('require_pregrasp_visibility').value),
                open_gripper_position_rad=(float(self.get_parameter(
                    'selection_gripper_open_position_rad').value)
                    if self.get_parameter('require_open_grasp_clearance').value else None),
            ),
        )
        self._server = ActionServer(
            self,
            SelectReachableGrasp,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callback_group,
        )

    def _on_robot_description(self, message: String) -> None:
        try:
            self._adapter.set_robot_description(message.data)
        except (ValueError, KeyError) as error:
            self.get_logger().error(f'Invalid runtime URDF for pose refinement: {error}')

    def _on_joint_state(self, message: JointState) -> None:
        with self._joint_state_lock:
            self._joint_state = message

    def _on_goal(self, request) -> GoalResponse:
        try:
            self._validate_candidates(
                request.candidates,
                str(self.get_parameter('planning_frame').value),
            )
        except ValueError as error:
            self.get_logger().warning(f'Rejecting invalid grasp goal: {error}')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._adapter.cancel_active()
        return CancelResponse.ACCEPT

    @staticmethod
    def _validate_candidates(candidates, planning_frame: str) -> None:
        if not candidates:
            raise ValueError('at least one candidate is required')
        if not planning_frame:
            raise ValueError('planning_frame must not be empty')
        first = candidates[0]
        if not first.snapshot_id or first.object_id == 0 or not first.header.frame_id:
            raise ValueError('candidate snapshot, object, and frame are required')
        if first.target_object.object_id != first.object_id:
            raise ValueError('target OBB object ID does not match candidate')
        if first.header.frame_id != planning_frame:
            raise ValueError(
                'candidate frame must match configured planning_frame '
                f'{planning_frame!r}'
            )
        size = first.target_object.obb_size
        pose = first.target_object.obb_pose
        values = (
            size.x, size.y, size.z,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values) or min(size.x, size.y, size.z) <= 0:
            raise ValueError('target OBB must have finite positive dimensions and pose')
        norm = math.sqrt(sum(value * value for value in values[-4:]))
        if not math.isclose(norm, 1.0, abs_tol=1e-4):
            raise ValueError('target OBB quaternion must be normalized')
        for candidate in candidates:
            if (
                candidate.snapshot_id != first.snapshot_id
                or candidate.object_id != first.object_id
                or candidate.header.frame_id != first.header.frame_id
                or candidate.target_object != first.target_object
            ):
                raise ValueError('all candidates must share snapshot, object, frame, and OBB')

    def _current_joint_state(self) -> JointState:
        with self._joint_state_lock:
            state = self._joint_state
        if state is None:
            raise ValueError('joint state is incomplete')
        missing = set(REQUIRED_JOINT_NAMES) - set(state.name)
        if missing or len(state.position) != len(state.name):
            raise ValueError(f'joint state is incomplete: {sorted(missing)}')
        stamp_ns = state.header.stamp.sec * 1_000_000_000 + state.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(float(self.get_parameter('joint_state_max_age_sec').value) * 1e9)
        if stamp_ns <= 0 or now_ns - stamp_ns > max_age_ns:
            raise TimeoutError('joint state is stale')
        return state

    def _visibility_constraint(self, candidate) -> VisibilityConstraint:
        frame = str(self.get_parameter('planning_frame').value)
        camera_frame = str(self.get_parameter('visibility_camera_frame').value)
        try:
            transform = self._visibility_tf.lookup_transform(
                frame, camera_frame, Time(), timeout=Duration(seconds=1.0))
        except Exception as error:
            raise InfrastructureError(f'visibility camera TF unavailable: {error}') from error
        stamp = transform.header.stamp.sec*1_000_000_000 + transform.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp)/1e9
        maximum_age = float(self.get_parameter('visibility_camera_max_age_sec').value)
        if (not math.isfinite(maximum_age) or maximum_age <= 0.
                or (stamp != 0 and not -0.05 <= age <= maximum_age)):
            raise InfrastructureError('visibility camera TF is stale or its age limit is invalid')
        camera = transform.transform.translation
        obj = candidate.target_object
        p, q, size = obj.obb_pose.position, obj.obb_pose.orientation, obj.obb_size
        cone = enclosing_visibility_cone(
            (camera.x, camera.y, camera.z), (p.x, p.y, p.z),
            (size.x, size.y, size.z), (q.x, q.y, q.z, q.w),
            padding_m=float(self.get_parameter('visibility_padding_m').value),
            sides=int(self.get_parameter('visibility_cone_sides').value))
        message = VisibilityConstraint()
        message.target_radius = cone.radius_m
        message.cone_sides = cone.sides
        message.target_pose.header.frame_id = frame
        message.target_pose.pose.position.x, message.target_pose.pose.position.y, message.target_pose.pose.position.z = cone.target
        (message.target_pose.pose.orientation.x, message.target_pose.pose.orientation.y,
         message.target_pose.pose.orientation.z, message.target_pose.pose.orientation.w) = cone.orientation
        message.sensor_pose.header.frame_id = frame
        message.sensor_pose.pose.position.x, message.sensor_pose.pose.position.y, message.sensor_pose.pose.position.z = cone.camera
        message.sensor_pose.pose.orientation.w = 1.
        message.weight = 1.
        self.get_logger().info(f'Pregrasp visibility envelope radius={cone.radius_m:.4f}m camera={camera_frame}')
        return message

    def _execute(self, goal_handle):
        result = SelectReachableGrasp.Result()
        result.selected_candidate_index = -1
        deadline = time.monotonic() + float(
            self.get_parameter('action_timeout_sec').value
        )
        candidate_messages = list(goal_handle.request.candidates)
        scene_started = False
        terminal_state = 'abort'
        try:
            if goal_handle.request.required_arm not in ('', 'left', 'right'):
                raise ValueError('required_arm must be empty, left, or right')
            try:
                state = self._current_joint_state()
            except TimeoutError as error:
                self._set_failure(
                    result,
                    result.ERROR_JOINT_STATE_STALE,
                    str(error),
                )
            except ValueError as error:
                self._set_failure(
                    result,
                    result.ERROR_JOINT_STATE_INCOMPLETE,
                    str(error),
                )
            else:
                self._adapter.set_current_state(state)
                if getattr(self, '_visibility_tf', None) is not None:
                    self._adapter.set_visibility_constraint(
                        self._visibility_constraint(candidate_messages[0]))
                if self._scene.active:
                    self._scene.restore()
                object_id = f'grasp_target_{uuid.uuid4().hex}'
                scene_started = True
                self._scene.begin(candidate_messages[0], object_id)
                self._scene_port.reset()
                candidates = [
                    Candidate(
                        position=(
                            item.tcp_pose.position.x,
                            item.tcp_pose.position.y,
                            item.tcp_pose.position.z,
                        ),
                        approach_direction=(
                            item.approach_direction.x,
                            item.approach_direction.y,
                            item.approach_direction.z,
                        ),
                        score=float(item.score),
                        source_index=index,
                        lateral_offset_m=(aperture_centering_offset(float(item.required_opening_m),
                            float(self.get_parameter('grasp_aperture_margin_m').value),
                            float(self.get_parameter('grasp_fixed_jaw_inner_x_m').value),
                            float(self.get_parameter('grasp_fixed_jaw_clearance_m').value))
                            if self.get_parameter('grasp_use_aperture_centering').value else None),
                        orientation=(
                            item.tcp_pose.orientation.x,
                            item.tcp_pose.orientation.y,
                            item.tcp_pose.orientation.z,
                            item.tcp_pose.orientation.w,
                        ),
                    )
                    for index, item in enumerate(candidate_messages)
                ]

                def canceled() -> bool:
                    cancel = (
                        goal_handle.is_cancel_requested
                        or time.monotonic() >= deadline
                    )
                    if cancel:
                        self._adapter.cancel_active()
                    return cancel

                def feedback(index, arm, stage, message) -> None:
                    update = SelectReachableGrasp.Feedback()
                    update.candidate_index = index
                    update.arm = arm
                    update.stage = STAGE_CONSTANT[stage]
                    update.message = message
                    goal_handle.publish_feedback(update)
                    self.get_logger().info(
                        f'candidate={index} arm={arm} '
                        f'stage={stage.value}: {message}'
                    )

                selection = self._selector.select(
                    candidates,
                    required_arm=getattr(
                        goal_handle.request, 'required_arm', ''
                    ),
                    cancel_requested=canceled,
                    feedback=feedback,
                )
                if selection is None:
                    self._set_failure(
                        result,
                        result.ERROR_NO_REACHABLE_GRASP,
                        'No candidate-arm pair passed IK, validity, and both plans',
                    )
                else:
                    result.success = True
                    result.error_code = result.ERROR_NONE
                    result.message = 'Selected reachable grasp (plan-only)'
                    result.selected_candidate_index = selection.candidate_index
                    result.selected_arm = selection.arm
                    result.selected_candidate = candidate_messages[
                        selection.candidate_index
                    ]
                    result.pregrasp_joint_state = self._joint_message(
                        selection.pregrasp
                    )
                    result.grasp_joint_state = self._joint_message(
                        selection.grasp
                    )
                    terminal_state = 'succeed'
        except InterruptedError:
            self._set_failure(
                result,
                result.ERROR_CANCELED,
                'Grasp selection canceled or timed out',
            )
            terminal_state = 'canceled'
        except InfrastructureError as error:
            message = str(error)
            code = self._infrastructure_error_code(result, message)
            self._set_failure(result, code, message)
        except (ValueError, TypeError) as error:
            self._set_failure(result, result.ERROR_INVALID_INPUT, str(error))
        except Exception as error:
            self._set_failure(
                result,
                result.ERROR_INTERNAL,
                f'Unexpected error: {error}',
            )
        finally:
            if getattr(self, '_visibility_tf', None) is not None:
                self._adapter.set_visibility_constraint(None)
            if scene_started:
                try:
                    self._scene.restore()
                except Exception as error:
                    self.get_logger().error(
                        f'Failed to restore planning scene: {error}'
                    )
                    self._set_failure(
                        result,
                        result.ERROR_PLANNING_SCENE,
                        f'Failed to restore planning scene: {error}',
                    )
                    terminal_state = 'abort'
            with self._goal_lock:
                self._goal_active = False

        if terminal_state == 'succeed':
            goal_handle.succeed()
        elif terminal_state == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _joint_message(solution) -> JointState:
        message = JointState()
        message.name = list(solution.names)
        message.position = list(solution.positions)
        return message

    @staticmethod
    def _set_failure(result, code, message) -> None:
        result.success = False
        result.error_code = code
        result.message = message
        result.selected_candidate_index = -1
        result.selected_arm = ''
        result.selected_candidate = GraspCandidate()
        result.pregrasp_joint_state = JointState()
        result.grasp_joint_state = JointState()

    @staticmethod
    def _infrastructure_error_code(result, message: str) -> int:
        planning_scene_terms = (
            'planning-scene',
            'planning scene',
            'target OBB',
            'collision permissions',
        )
        if any(term in message for term in planning_scene_terms):
            return result.ERROR_PLANNING_SCENE
        return result.ERROR_MOVEIT_UNAVAILABLE


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspSelectionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
