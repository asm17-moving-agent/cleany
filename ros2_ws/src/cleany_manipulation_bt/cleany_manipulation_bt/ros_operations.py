"""Sensor/MoveIt operations for one approved object, on a dedicated ROS worker node."""
from __future__ import annotations

import math
import time
from typing import Any
from action_msgs.msg import GoalStatus

from cleany_interfaces.srv import GetSceneSnapshot, ObserveWristTarget, VerifyPlacement
from controller_manager_msgs.srv import ListControllers
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.core.sorting import Category, bin_release_region
from cleany_skill_executor.collision_geometry_cache import candidate_bounding_radius
from cleany_skill_executor.manipulation.models import Error, ObjectState, Placement
from cleany_skill_executor.pick_operations import GraspProgress
from cleany_skill_executor.sorting_coordinator import SortTarget, SortingCoordinator
import rclpy

from .backend import ExecutionContext, Observation, OperationError
from .target_identity import VERIFICATION_LABELS, label_key


class TrackedClient:
    """Track both pending acceptance and terminal responses, including late goals."""
    def __init__(self, node: MujocoOperations, client: Any) -> None:
        self.node, self.client = node, client

    def __getattr__(self, name: str) -> Any:
        return getattr(self.client, name)

    def send_goal_async(self, goal, **kwargs):
        self.node._guard()
        context = self.node.execution_context
        future = self.client.send_goal_async(goal, **kwargs)
        self.node.submissions.append(future)

        def accepted(response):
            if response.cancelled() or response.exception() is not None:
                return
            handle = response.result()
            if handle.accepted:
                terminal = handle.get_result_async()
                self.node.handles.append((handle, terminal))
                if context.abort.is_set():
                    handle.cancel_goal_async()
        future.add_done_callback(accepted)
        return future


class MujocoOperations(SortingCoordinator):
    """Reuse calibrated sorting motion without invoking run(), pick() or execute_sort()."""
    def __init__(self) -> None:
        super().__init__()
        self._controller_retry_enabled = False
        self._require_unique_target_label = True
        self._geometry_association_lift = False
        self.execution_context: ExecutionContext | None = None
        self.current_node = ''
        self.deadline = math.inf
        self._last_joint_wall = 0.0
        self._joint_stamp_ns = 0
        self._clock_value = 0
        self._clock_progress_wall = time.monotonic()
        self.declare_parameter('bt_feedback_max_age_sec', 2.0)
        self.declare_parameter('bt_operation_timeout_sec', 180.0)
        self.declare_parameter('bt_verification_timeout_sec', 10.0)
        self.declare_parameter('bt_stop_timeout_sec', 10.0)
        self.declare_parameter('bt_supported_trash_labels', list(VERIFICATION_LABELS))
        self._target_label_key = label_key
        self._snapshot = self.create_client(GetSceneSnapshot, '/perception/get_scene_snapshot')
        self._controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.submissions: list[Any] = []
        self.handles: list[tuple[Any, Any]] = []
        for name in ('_inspection', '_selection', '_move_group', '_execute_trajectory'):
            setattr(self, name, TrackedClient(self, getattr(self, name)))
        self._grippers = {arm: TrackedClient(self, client) for arm, client in self._grippers.items()}
        self._transport_adapter._spin_once = lambda t: self._spin_once(timeout_sec=t)

    def _on_joints(self, message) -> None:
        super()._on_joints(message)
        self._last_joint_wall = time.monotonic()
        self._joint_stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def quiescent(self) -> bool:
        def valid(future):
            return future.done() and not future.cancelled() and future.exception() is None
        terminals = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        # A response Future can become done before its acceptance callback is
        # dispatched. Until its accepted handle is tracked, there is no stop
        # evidence for that action and no permission to begin a new context.
        return (all(valid(future) and future.result() is not None
                    and (not future.result().accepted
                         or any(handle is future.result() for handle, _ in self.handles))
                    for future in self.submissions)
                and all(valid(future) and future.result() is not None
                        and future.result().status in terminals for _, future in self.handles))

    def _execution_goal(self, *args):
        goal = super()._execution_goal(*args)
        goal.planning_options.replan = False
        return goal

    def _guard(self) -> None:
        if self.execution_context is None or self.current_node == 'StopAndAssess':
            return
        if self.execution_context.abort.is_set():
            raise OperationError(Error.CANCELED, 'Operation halted; awaiting stop assessment')
        if time.monotonic() >= self.deadline:
            raise OperationError(Error.TIMEOUT, 'Operation wall-clock deadline exceeded')
        motion = self.current_node in (
            'MoveToPregrasp', 'ApproachObject', 'GraspObject', 'ConfirmGrasp', 'LiftObject',
            'ConfirmHeld', 'CarryObject', 'CheckPlacementTarget', 'OpenGripperAtDestination', 'ReturnArm')
        if motion:
            maximum = float(self.get_parameter('bt_feedback_max_age_sec').value)
            now = self.get_clock().now().nanoseconds
            if now != self._clock_value:
                self._clock_value, self._clock_progress_wall = now, time.monotonic()
            if (time.monotonic() - self._last_joint_wall > maximum
                    or time.monotonic() - self._clock_progress_wall > maximum
                    or self._joint_stamp_ns <= 0 or abs(now-self._joint_stamp_ns)/1e9 > maximum):
                # /clock and /joint_states arrive independently. A small newer
                # joint stamp is possible before the next clock callback; large
                # skew, a stopped clock or stale receipt still fails closed.
                raise OperationError(Error.HARDWARE_ERROR,
                    'Joint feedback or simulation clock is stale: '
                    f'receipt_age={time.monotonic()-self._last_joint_wall:.3f}s '
                    f'clock_idle={time.monotonic()-self._clock_progress_wall:.3f}s '
                    f'stamp_skew={(now-self._joint_stamp_ns)/1e9:.3f}s')
        if self.current_node in ('LiftObject', 'ConfirmHeld', 'CarryObject', 'CheckPlacementTarget'):
            if self._held_object is None:
                raise OperationError(Error.GRASP_LOST, 'Missing held object')
            try:
                self._require_held_contact(self._held_object)
            except RuntimeError as error:
                raise OperationError(Error.GRASP_LOST, str(error)) from error

    def _spin_once(self, timeout_sec: float) -> None:
        self._guard()
        super()._spin_once(timeout_sec)
        self._guard()

    def _future(self, future, timeout: float, label: str):
        # Keep late action acceptance observable so StopAndAssess can cancel it.
        deadline = min(self.deadline, time.monotonic() + timeout)
        while not future.done() and time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.02)
        self._guard()
        if not future.done():
            raise OperationError(Error.TIMEOUT, f'Timed out waiting for {label}')
        result = future.result()
        if result is None:
            raise OperationError(Error.INTERNAL_ERROR, f'{label} returned no result')
        return result

    def execute(self, node: str, context: ExecutionContext) -> Observation:
        if self.execution_context is not context:
            self.execution_context = context
            self.handles.clear()
            self.submissions.clear()
            self._held_object = None
            self._fixed_release_wrist = {}
            self._wrist_reference = None
            self._lift_completion_stamp_ns = None
        self.current_node = node
        self.deadline = time.monotonic() + float(self.get_parameter('bt_operation_timeout_sec').value)
        operations = {
            'ValidateGoal': self.validate, 'PrepareTarget': self.prepare,
            'ReconstructTarget': self.reconstruct, 'GenerateGrasp': self.generate,
            'SelectArmAndPath': self.select, 'MoveToPregrasp': self.pregrasp,
            'ApproachObject': self.approach, 'GraspObject': self.grasp,
            'ConfirmGrasp': self.confirm_grasp, 'LiftObject': self.lift,
            'ConfirmHeld': self.confirm_held, 'CarryObject': self.carry,
            'CheckPlacementTarget': self.check_placement, 'OpenGripperAtDestination': self.open,
            'ConfirmRelease': self.confirm_release, 'ReturnArm': self.return_arm,
            'VerifyPlacedObject': self.verify, 'StopAndAssess': self.stop,
        }
        try:
            self._guard()
            return operations[node](context)
        except OperationError:
            raise
        except Exception as error:
            code = (Error.BACKEND_NOT_READY if node == 'ValidateGoal' else
                    Error.STALE_TARGET if node == 'ApproachObject' and isinstance(error, ValueError) else
                    Error.GRASP_FAILED if node in ('GraspObject', 'ConfirmGrasp') else
                    Error.GRASP_LOST if node == 'ConfirmHeld' else Error.MOTION_FAILED)
            raise OperationError(code, str(error)) from error

    def validate(self, context: ExecutionContext) -> Observation:
        clients = [self._inspection, self._selection, self._move_group, self._execute_trajectory,
                   *self._grippers.values()]
        deadline = time.monotonic() + float(self.get_parameter('startup_timeout_sec').value)
        while time.monotonic() < deadline:
            self._spin_once(timeout_sec=0.05)
            if (all(client.server_is_ready() for client in clients)
                    and all(client.service_is_ready() for client in
                            (self._snapshot, self._grasp, self._verification, self._controllers))
                    and set(REQUIRED_JOINT_NAMES) <= self._joint_positions.keys()):
                break
        else:
            raise OperationError(Error.BACKEND_NOT_READY, 'Perception, motion, verifier or joint state unavailable')
        response = self._future(self._controllers.call_async(ListControllers.Request()), 3., 'controller state')
        required = {'left_arm_controller', 'right_arm_controller',
                    'left_gripper_controller', 'right_gripper_controller'}
        active = {controller.name for controller in response.controller if controller.state == 'active'}
        if not required <= active:
            raise OperationError(Error.BACKEND_NOT_READY, 'Required controllers are not active')
        # Only a quiescent, non-inhibited prior execution can reach this node.
        # Remove a temporary world target left by a failure before gripper
        # closure. Held/unknown records require confirmation and cannot restart.
        self._execution_scene.restore()
        if bool(self.get_parameter('require_sensor_scene').value):
            self._wait_for_sensor_scene(5.)
        self._register_bins()
        self._home = {arm: self._arm_joint_state(arm) for arm in ('left', 'right')}
        if self._wrist_enabled:
            if not self._wrist_client.service_is_ready():
                raise OperationError(Error.BACKEND_NOT_READY, 'Wrist observation unavailable')
            self._switch_camera('head')
        return Observation(message='Sensor, controllers, MoveIt and independent verifier ready')

    def prepare(self, context: ExecutionContext) -> Observation:
        goal = context.goal
        response = self._future(self._snapshot.call_async(
            GetSceneSnapshot.Request(snapshot_id=goal.snapshot_id)), 3., 'snapshot lookup')
        if not response.found:
            raise OperationError(Error.STALE_TARGET, response.message)
        context.snapshot = response.detections
        matches = [d for d in response.detections.detections if d.object_id == goal.object_id]
        if len(matches) != 1 or response.detections.snapshot_id != goal.snapshot_id:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Approved object is absent or ambiguous')
        detection = matches[0]
        supported = self.get_parameter('bt_supported_trash_labels').value
        if (detection.label not in supported or detection.label not in VERIFICATION_LABELS
                or sum(label_key(d.label) == label_key(detection.label)
                       for d in response.detections.detections) != 1):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Verifier requires a supported, unique label')
        if not detection.distance_valid:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Target has no valid sensor depth')
        decision = self._policy.classify_model(detection.label, detection.confidence,
                                              detection.sorting_category, detection.sorting_reason)
        if decision.category != Category.TRASH:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Target is not approved trash by model and policy')
        if goal.destination_id != decision.destination or goal.destination_id not in self._bins:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, 'Destination does not match trash policy')
        context.attempt = ObjectAttempt(detection.object_id, detection.label, detection.confidence,
                                        detection.distance_m, detection.sorting_category, detection.sorting_reason)
        return Observation(message=f'Approved {goal.snapshot_id}/{goal.object_id}: {detection.label}')

    def reconstruct(self, context: ExecutionContext) -> Observation:
        context.inspected = self._inspect_selected(context.goal.snapshot_id, context.attempt)
        if context.inspected is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Selected-object reconstruction failed or expired')
        obj = context.inspected.objects.objects[0]
        if (context.inspected.objects.snapshot_id != context.goal.snapshot_id
                or obj.object_id != context.goal.object_id or obj.label != context.attempt.label):
            raise OperationError(Error.TARGET_UNAVAILABLE, 'Reconstructed identity mismatch')
        return Observation(message='Selected object reconstructed from cached RGB-D')

    def generate(self, context: ExecutionContext) -> Observation:
        context.grasps = self._plan_grasps(context.inspected, context.attempt)
        if context.grasps is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'No grasp candidates')
        return Observation()

    def select(self, context: ExecutionContext) -> Observation:
        center = context.inspected.objects.objects[0].obb_pose.position
        arms = ('left', 'right') if center.y >= 0 else ('right', 'left')
        for arm in arms:
            context.selected = self._select_reachable(context.grasps.candidates, context.attempt, required_arm=arm)
            if context.selected is not None:
                break
        if context.selected is None:
            raise OperationError(Error.TARGET_UNAVAILABLE, 'No collision-valid reachable grasp')
        self.check_payload_fit(context)
        context.target = SortTarget(context.attempt, context.selected)
        context.grasp_progress = GraspProgress()
        return Observation(selected_arm=context.selected.selected_arm)

    def check_payload_fit(self, context: ExecutionContext) -> None:
        cache = self._geometry_cache if self._geometry_subscription is not None else None
        radius = candidate_bounding_radius(context.selected.selected_candidate, cache)
        destination = self._bins[context.goal.destination_id]
        try:
            low, high = bin_release_region(destination.center_xy, destination.outside_size,
                destination.wall, destination.top_z, radius,
                float(self.get_parameter('sorting_release_clearance_m').value),
                float(self.get_parameter('sorting_release_maximum_clearance_m').value),
                float(self.get_parameter('sorting_release_edge_margin_m').value))
            point = self._fixed_release_points[context.goal.destination_id]
            if any(point - .001 < low) or any(point + .001 > high):
                raise ValueError('Configured release point does not fit observed payload')
        except ValueError as error:
            raise OperationError(Error.DESTINATION_UNAVAILABLE, str(error)) from error

    def pregrasp(self, context: ExecutionContext) -> Observation:
        self._execute_pregrasp(context.selected, context.attempt)
        return Observation()

    def approach(self, context: ExecutionContext) -> Observation:
        # Reobserve ONLY the approved label, with existing geometric continuity
        # and ambiguity gates; never use geometry-only automatic reassignment.
        self._switch_camera('head') if self._wrist_enabled else None
        selected, attempt = self._refresh_selected_grasp(context.selected, context.attempt, match_label=True)
        old = self._policy.classify_model(context.attempt.label, context.attempt.confidence,
                                         context.attempt.sorting_category, context.attempt.sorting_reason)
        new = self._policy.classify_model(attempt.label, attempt.confidence,
                                         attempt.sorting_category, attempt.sorting_reason)
        if new.category != old.category or new.destination != old.destination or new.category != Category.TRASH:
            raise OperationError(Error.STALE_TARGET, 'Approved-object classification changed')
        context.selected, context.attempt = selected, attempt
        self.check_payload_fit(context)
        context.target = SortTarget(attempt, selected)
        if self._wrist_enabled:
            self._switch_camera(selected.selected_arm)
            self._wrist_reference = self._observe_wrist(selected, ObserveWristTarget.Request.HANDOFF)
            self._set_head_depth_boost(True)
        self._approach_grasp(selected, attempt, context.grasp_progress)
        return Observation(message='Approved-object continuity and approach confirmed')

    def grasp(self, context: ExecutionContext) -> Observation:
        self._close_grasp(context.selected, context.attempt, context.grasp_progress, retry=False)
        return Observation()

    def confirm_grasp(self, context: ExecutionContext) -> Observation:
        self._confirm_grasp_contact(context.selected, context.attempt, context.grasp_progress)
        context.held = self._held_object
        context.held.target = context.target
        return Observation(object_state=ObjectState.HELD, message='Settled gripper contact and attachment confirmed')

    def lift(self, context: ExecutionContext) -> Observation:
        self._lift_grasp(context.selected, context.attempt, context.grasp_progress)
        return Observation(object_state=ObjectState.HELD)

    def confirm_held(self, context: ExecutionContext) -> Observation:
        self._confirm_held_grasp(context.selected, context.attempt, context.grasp_progress)
        if self._wrist_enabled:
            self._set_head_depth_boost(False)
        return Observation(object_state=ObjectState.HELD, message='Post-lift observation and contact confirmed')

    def carry(self, context: ExecutionContext) -> Observation:
        self.transport(context.held, context.goal.destination_id)
        return Observation(object_state=ObjectState.HELD)

    def check_placement(self, context: ExecutionContext) -> Observation:
        self.check_release_target(context.held, context.goal.destination_id)
        return Observation()

    def open(self, context: ExecutionContext) -> Observation:
        self.check_release_target(context.held, context.goal.destination_id)
        self.open_at_destination(context.held)
        context.release_stamp_ns = self._release_stamp_ns
        return Observation(message='Gripper opening confirmed by controller and joint feedback')

    def _verify_after(self, context: ExecutionContext, after_ns: int) -> Observation:
        request = VerifyPlacement.Request(label=label_key(context.target.attempt.label),
            destination_id=context.goal.destination_id, after_stamp_ns=after_ns)
        deadline = time.monotonic() + float(self.get_parameter('bt_verification_timeout_sec').value)
        while time.monotonic() < deadline:
            response = self._future(self._verification.call_async(request), 2., 'independent placement verification')
            if response.success:
                return Observation(message=response.message, placement_state=Placement.CONFIRMED)
            self._spin_once(timeout_sec=0.1)
        return Observation(False, Error.VERIFICATION_TIMEOUT, 'Awaiting settled post-release evidence timed out',
                           placement_state=Placement.UNKNOWN)

    def confirm_release(self, context: ExecutionContext) -> Observation:
        observation = self._verify_after(context, context.release_stamp_ns)
        if not observation.success:
            return observation
        self.finish_release(context.held)
        context.release_confirmed = True
        return Observation(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED,
                           message=observation.message)

    def return_arm(self, context: ExecutionContext) -> Observation:
        if not context.release_confirmed:
            raise OperationError(Error.PLACEMENT_NOT_CONFIRMED, 'Return requires independently confirmed release')
        self.retreat(context.held, context.goal.destination_id, prefetch=False)
        context.arm_recovered = True
        return Observation(arm_recovered=True)

    def verify(self, context: ExecutionContext) -> Observation:
        observation = self._verify_after(context, self.get_clock().now().nanoseconds)
        if not observation.success:
            return observation
        settled = self._assess_stationary(context, cancel=False)
        return Observation(success=settled, error=Error.NONE if settled else Error.STOP_UNCONFIRMED,
                           stop_confirmed=settled, placement_state=Placement.CONFIRMED,
                           message=observation.message if settled else 'Final stationary feedback unavailable')

    def _assess_stationary(self, context: ExecutionContext, *, cancel: bool) -> bool:
        deadline = time.monotonic() + float(self.get_parameter('bt_stop_timeout_sec').value)
        barrier = self.get_clock().now().nanoseconds
        canceled: set[int] = set()
        previous_stamp = 0
        samples = 0
        required = int(self.get_parameter('arm_stationary_samples').value)
        maximum = float(self.get_parameter('arm_stationary_velocity_rad_s').value)
        while time.monotonic() < deadline:
            # Assessment bypasses the abort guard, but still has its own deadline.
            rclpy.spin_once(self, timeout_sec=0.02)
            if cancel:
                for handle, terminal in self.handles:
                    if not terminal.done() and id(handle) not in canceled:
                        handle.cancel_goal_async()
                        canceled.add(id(handle))
            terminal = self.quiescent()
            stamp = self._joint_stamp_ns
            fresh = (stamp > barrier and stamp > previous_stamp
                     and time.monotonic() - self._last_joint_wall < 0.5)
            if fresh:
                previous_stamp = stamp
                stationary = all(math.isfinite(self._joint_velocities.get(n, math.nan))
                                 and abs(self._joint_velocities[n]) <= maximum for n in REQUIRED_JOINT_NAMES)
                samples = samples + 1 if stationary and terminal else 0
                if samples >= required:
                    context.stop_confirmed = True
                    return True
        return False

    def stop(self, context: ExecutionContext) -> Observation:
        context.abort.set()
        confirmed = self._assess_stationary(context, cancel=True)
        return Observation(stop_confirmed=confirmed, message='Stationary feedback and terminal actions confirmed'
                           if confirmed else 'Stop could not be confirmed from fresh feedback')
