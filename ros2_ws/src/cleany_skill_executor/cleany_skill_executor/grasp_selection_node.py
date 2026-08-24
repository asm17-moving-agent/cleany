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
            'ik_timeout_sec': 0.15,
            'state_validity_timeout_sec': 1.0,
            'planning_timeout_sec': 4.0,
            'planning_attempts': 1,
            'velocity_scaling': 0.1,
            'acceleration_scaling': 0.1,
            'maximum_candidates': 12,
            'action_timeout_sec': 120.0,
            'pregrasp_offset_m': 0.08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
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

        self._adapter = MoveItGraspAdapter(
            self,
            MoveItAdapterConfig(
                base_frame=str(self.get_parameter('planning_frame').value),
                ik_timeout_sec=float(self.get_parameter('ik_timeout_sec').value),
                state_validity_timeout_sec=float(self.get_parameter('state_validity_timeout_sec').value),
                planning_timeout_sec=float(self.get_parameter('planning_timeout_sec').value),
                planning_attempts=int(self.get_parameter('planning_attempts').value),
                velocity_scaling=float(self.get_parameter('velocity_scaling').value),
                acceleration_scaling=float(self.get_parameter('acceleration_scaling').value),
            ),
            spin_once=wait,
        )
        self._scene = TargetSceneTransaction(self, spin_once=wait)
        self._scene_port = SceneAwarePort(self._adapter, self._scene)
        self._selector = GraspSelector(
            self._scene_port,
            GraspSelectionConfig(
                pregrasp_offset_m=float(self.get_parameter('pregrasp_offset_m').value),
                maximum_candidates=int(self.get_parameter('maximum_candidates').value),
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
