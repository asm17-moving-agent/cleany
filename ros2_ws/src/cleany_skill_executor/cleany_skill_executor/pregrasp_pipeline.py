"""One-shot validation, optional MuJoCo pregrasp arrival, and YOLOE-seg observation.

The default mode only calls read-only perception/planning services. Physical
execution is exposed exclusively by the configured collection_pipeline subclass.
This CLI continues to expose only plan_only and mujoco.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import ValidatePick
from cleany_interfaces.srv import ObserveObjectReference, ObserveWristTarget
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution, PlannedPath, REQUIRED_JOINT_NAMES
from cleany_skill_executor.core.cartesian import CartesianPose, execution_wall_timeout
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter
from cleany_skill_executor.planning_scene import TargetSceneTransaction
from cleany_skill_executor.sensor_scene import SensorSceneMonitor
from cleany_skill_executor.trajectory_io import path_to_message


def check_joint_snapshot(state, now_ns, *, maximum_age=0.5):
    stamp = state.header.stamp.sec*10**9+state.header.stamp.nanosec
    age = (now_ns-stamp)/1e9
    if (stamp <= 0 or not 0 <= (now_ns-stamp)/1e9 <= maximum_age
            or len(state.name) != len(state.position) or len(set(state.name)) != len(state.name)
            or not set(REQUIRED_JOINT_NAMES) <= set(state.name)
            or not all(math.isfinite(q) for q in state.position)):
        raise RuntimeError(f'Missing, stale, or malformed measured joint state (age={age:.3f}s)')
    return dict(zip(state.name, state.position))


def check_start_state(measured, planned, tolerance=0.02, *, commanded=(), commanded_tolerance=None):
    if not set(REQUIRED_JOINT_NAMES) <= set(planned) or not set(planned) <= measured.keys():
        raise RuntimeError('Incomplete planning start state')
    commanded = set(commanded)
    if any(not math.isfinite(q) or abs(measured[name]-q) > (
            commanded_tolerance if name in commanded and commanded_tolerance is not None else tolerance)
           for name, q in planned.items()):
        raise RuntimeError('Robot moved since planning; request a new plan')


def check_path_start(trajectory, planned, tolerance=0.002):
    """Reject an unvalidated jump from the measured state to the first knot."""
    if not trajectory.points:
        raise RuntimeError('Empty pregrasp trajectory')
    first = trajectory.points[0]
    if (len(first.positions) != len(trajectory.joint_names)
            or any(name not in planned or not math.isfinite(q)
                   or abs(planned[name]-q) > tolerance
                   for name, q in zip(trajectory.joint_names, first.positions))):
        raise RuntimeError('Trajectory does not begin at the planning start state')


def reference_observation_metrics(observation):
    return {
        'stamp_ns': observation.header.stamp.sec*10**9+observation.header.stamp.nanosec,
        'frame_id': observation.header.frame_id,
        'center_m': [
            observation.observed_center.x,
            observation.observed_center.y,
            observation.observed_center.z,
        ],
        'extent_m': [
            observation.observed_extent.x,
            observation.observed_extent.y,
            observation.observed_extent.z,
        ],
        'mask_pixels': int(observation.mask_pixels),
        'valid_depth_points': int(observation.valid_depth_points),
        'valid_depth_fraction': float(observation.valid_depth_fraction),
    }


class ReferenceObservationFailure(RuntimeError):
    """Preserve reference-service error codes for bounded occlusion handling."""

    def __init__(self, response: ObserveObjectReference.Response) -> None:
        super().__init__(response.message or 'Reference observation failed')
        self.response = response
        self.error_code = int(response.error_code)


class PregraspPipeline(Node):
    def __init__(self, *, mode='plan_only', **kwargs):
        if mode not in ('plan_only', 'mujoco', 'hardware'):
            raise ValueError('Unknown execution mode')
        super().__init__('pregrasp_pipeline', **kwargs)
        self.mode = mode
        defaults = {'pipeline_timeout_sec': 360., 'maximum_joint_age_sec': .5,
                    'maximum_target_translation_m': .01, 'wrist_checks': 3,
                    'planning_scene_timeout_sec': 5.,
                    'planning_timeout_sec': 4., 'planning_response_margin_sec': 1.,
                    'execution_timeout_factor': 10., 'execution_timeout_margin_sec': 10.,
                    'execution_timeout_minimum_sec': 60., 'wrist_camera_control_timeout_sec': 5.}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('planning_timeout_sec', 'planning_response_margin_sec'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.scene_monitor = SensorSceneMonitor(self)
        self.validation = ActionClient(self, ValidatePick, '/grasp/validate_pick')
        self.reference = self.create_client(ObserveObjectReference, '/perception/observe_object_reference')
        self.wrist = self.create_client(ObserveWristTarget, '/perception/observe_wrist_target')
        self.state, self.description = None, ''
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.on_state, 20)
        self.create_subscription(String, '/robot_description', self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.motion = None
        self.real = None
        self._grasp_scene_freeze_held = False
        self.wrist_cameras = None
        if mode in ('mujoco', 'hardware'):
            if mode == 'mujoco' and not self.get_parameter('use_sim_time').value:
                raise ValueError('MuJoCo execution requires use_sim_time')
            self.hardware = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
            self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
            self.motion = MoveItGraspAdapter(self, MoveItAdapterConfig(
                validate_motion_paths=True,
                planning_timeout_sec=float(self.get_parameter('planning_timeout_sec').value),
                planning_response_margin_sec=float(
                    self.get_parameter('planning_response_margin_sec').value)),
                spin_once=lambda delay: time.sleep(min(delay, .01)))
            self.scene = TargetSceneTransaction(
                self,
                timeout_sec=float(
                    self.get_parameter('planning_scene_timeout_sec').value),
                support_patch_margin_m=.08,
            )
        if mode == 'hardware':
            from .wrist_camera_control import WristCameraSession
            from .hardware_execution import HardwareExecution
            from .core.execution_ownership import ExecutionOwnership
            self.real = HardwareExecution(self)
            self._execution_ownership = ExecutionOwnership()
            self.wrist_cameras = WristCameraSession(self,
                float(self.get_parameter('wrist_camera_control_timeout_sec').value), self.wait)

    def start_wrist_camera(self, arm: str) -> None:
        if self.wrist_cameras is not None:
            self.wrist_cameras.activate(arm)

    def stop_wrist_camera(self) -> None:
        if self.wrist_cameras is not None:
            self.wrist_cameras.close()

    def request_wrist_camera_stop(self) -> None:
        if self.wrist_cameras is not None:
            self.wrist_cameras.request_close()

    def destroy_node(self):
        cameras = getattr(self, 'wrist_cameras', None)
        if cameras is not None:
            try:
                cameras.request_close()
            except Exception as error:
                self.get_logger().error(f'Wrist camera shutdown request failed: {error}')
        ownership = getattr(self, '_execution_ownership', None)
        if ownership is not None:
            ownership.close()
        return super().destroy_node()

    def on_state(self, message):
        with self.lock:
            self.state = message

    def create_motion_client(self, action_type, name):
        """Let an execution owner track even late goal acknowledgements."""
        return ActionClient(self, action_type, name)

    def on_description(self, message):
        with self.lock:
            self.description = message.data

    def measured(self):
        with self.lock:
            state = deepcopy(self.state)
        if state is None:
            raise RuntimeError('No measured joint state')
        # JointState and /clock travel independently. Give a simulation clock
        # callback a bounded chance to catch up; never accept a future stamp.
        stamp = state.header.stamp.sec*10**9+state.header.stamp.nanosec
        deadline = time.monotonic()+.05
        while (self.mode == 'mujoco'
               and 0 < stamp-self.get_clock().now().nanoseconds <= 50_000_000
               and time.monotonic() < deadline):
            time.sleep(.001)
        check_joint_snapshot(state, self.get_clock().now().nanoseconds,
                             maximum_age=self.get_parameter('maximum_joint_age_sec').value)
        return state

    def wait(self, future, timeout, *, monitor=False, cancel_on_timeout=True):
        deadline = time.monotonic()+timeout
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        while not future.done():
            if not rclpy.ok() or time.monotonic() >= deadline:
                if cancel_on_timeout:
                    future.cancel()
                raise RuntimeError('Pipeline operation timed out or was interrupted')
            if monitor:
                real = getattr(self, 'real', None)
                if real is None or not real.sensor_scene_frozen:
                    self.scene_monitor.check()
                self.measured()
                if real is not None:
                    real.monitor()
            # Wake on the response instead of adding up to 10 ms to every
            # sequential collision check. Keep the same monitoring cadence.
            completed.wait(max(0., min(.01, deadline-time.monotonic())))
        return future.result()

    def call(self, client, request, timeout=30., *, monitor=False):
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        response = self.wait(client.call_async(request), timeout, monitor=monitor)
        if response is None or (hasattr(response, 'success') and not response.success):
            if response is None:
                raise RuntimeError('Empty service response')
            if isinstance(response, ObserveObjectReference.Response):
                raise ReferenceObservationFailure(response)
            detail = getattr(response, 'message', '')
            raise RuntimeError(detail or f'Service call failed: {client.srv_name}')
        return response

    def run_action(self, client, goal, timeout, *, monitor=False, allow_aborted=False,
                   feedback_callback=None):
        if not client.wait_for_server(timeout_sec=10.):
            raise RuntimeError('Action server unavailable')
        send_options = {} if feedback_callback is None else {
            'feedback_callback': feedback_callback}
        handle = self.wait(client.send_goal_async(goal, **send_options), 10.)
        if handle is None or not handle.accepted:
            raise RuntimeError('Action goal rejected')
        result_future = handle.get_result_async()
        try:
            wrapped = self.wait(result_future, timeout, monitor=monitor, cancel_on_timeout=False)
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED and not (
                    allow_aborted and wrapped.status == GoalStatus.STATUS_ABORTED):
                detail = getattr(wrapped.result, 'message', None) or getattr(wrapped.result, 'error_string', '')
                raise RuntimeError(f'Action failed: {detail or wrapped.status}')
            return wrapped.result
        except BaseException as interrupted:
            from .core.hardware_execution import CommunicationRecovered
            if isinstance(interrupted, CommunicationRecovered):
                real = getattr(self, 'real', None)
                if real is not None:
                    # Carrying keeps hardware monitoring active during cancel
                    # waits. Consume this event so those waits can prove stop.
                    real._active_recovery_generation = None
            try:
                self.wait(handle.cancel_goal_async(), 5.)
                if not result_future.done():
                    self.wait(result_future, 5., cancel_on_timeout=False)
                if isinstance(interrupted, CommunicationRecovered):
                    interrupted.cancellation_confirmed = result_future.result().status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            except Exception as error:
                self.get_logger().error(f'Could not confirm action cancellation: {error}')
            raise

    def observe(self, reference_id, after):
        request = ObserveObjectReference.Request(operation=ObserveObjectReference.Request.OBSERVE,
                                                reference_id=reference_id, after_stamp_ns=after)
        return self.call(self.reference, request, 60.)

    def verify_simulator(self, arm):
        if getattr(self, 'real', None) is not None:
            self.real.verify(arm)
            return
        hardware = self.call(self.hardware, ListHardwareComponents.Request())
        if (len(hardware.component) != 1
                or hardware.component[0].class_type != 'mujoco_ros2_control/MujocoSystemInterface'
                or hardware.component[0].state.label != 'active'):
            raise RuntimeError('Execution requires exactly one active MuJoCo hardware component')
        controllers = self.call(self.controllers, ListControllers.Request())
        active = {item.name for item in controllers.controller if item.state == 'active'}
        if not {f'{arm}_arm_controller', f'{arm}_gripper_controller'} <= active:
            raise RuntimeError('MuJoCo arm/gripper controllers are not active')
        if not self.description:
            raise RuntimeError('No runtime robot_description')
        self.motion.set_robot_description(self.description)

    def execute_trajectory(self, arm, trajectory, controller, *, check_orientation=True,
                           check_collisions=True, allow_endpoint_error=False,
                           held_command=None, command_path_validator=None,
                           defer_height_shortfall: bool = False, minimum_duration_sec=1.,
                           start_from_measured: bool = False,
                           endpoint_pose: CartesianPose | None = None):
        if getattr(self, 'real', None) is not None:
            endpoint_options = {'allow_endpoint_error': True} if allow_endpoint_error else {}
            if held_command is not None:
                endpoint_options['held_command'] = held_command
            if endpoint_pose is not None:
                endpoint_options['endpoint_pose'] = endpoint_pose
            if command_path_validator is not None:
                endpoint_options['command_path_validator'] = command_path_validator
            if type(defer_height_shortfall) is not bool:
                raise ValueError('defer_height_shortfall must be boolean')
            if defer_height_shortfall:
                endpoint_options['defer_height_shortfall'] = True
            if minimum_duration_sec != 1.:
                endpoint_options['minimum_duration_sec'] = minimum_duration_sec
            if type(start_from_measured) is not bool:
                raise ValueError('start_from_measured must be boolean')
            if start_from_measured:
                endpoint_options['start_from_measured'] = True
            return self.real.execute(
                arm, trajectory, controller,
                check_orientation=check_orientation, check_collisions=check_collisions,
                **endpoint_options)
        if allow_endpoint_error or defer_height_shortfall:
            raise ValueError('Relaxed grasp arrival requires physical collection mode')
        self.verify_simulator(arm)
        client = ActionClient(self, FollowJointTrajectory, f'/{arm}_{controller}_controller/follow_joint_trajectory')
        try:
            end = trajectory.points[-1].time_from_start
            duration = end.sec+end.nanosec/1e9
            timeout = execution_wall_timeout(duration,
                self.get_parameter('execution_timeout_factor').value,
                self.get_parameter('execution_timeout_margin_sec').value,
                self.get_parameter('execution_timeout_minimum_sec').value)
            result = self.run_action(client, FollowJointTrajectory.Goal(trajectory=trajectory), timeout, monitor=True)
            if result.error_code != result.SUCCESSFUL:
                raise RuntimeError(f'MuJoCo trajectory rejected: {result.error_string}')
            deadline = time.monotonic()+5.
            while True:
                state = self.measured()
                actual = dict(zip(state.name, state.position))
                if all(abs(actual[name]-q) <= .02 for name, q in zip(trajectory.joint_names, trajectory.points[-1].positions)):
                    return state
                if time.monotonic() >= deadline:
                    raise RuntimeError('Controller completed without measured arrival')
                time.sleep(.02)
        finally:
            client.destroy()

    def execute_pregrasp(self, selected):
        arm = selected.selected_arm
        self.verify_simulator(arm)
        measured = self.measured()
        planned = dict(zip(selected.planning_start_state.name, selected.planning_start_state.position))
        real = getattr(self, 'real', None)
        start_options = ({} if real is None else {
            'commanded': (*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint'),
            'commanded_tolerance': real.value('hardware_start_blend_max_rad')})
        check_start_state(dict(zip(measured.name, measured.position)), planned, **start_options)
        trajectory = selected.pregrasp_trajectory
        if (tuple(trajectory.joint_names) != ARM_JOINT_NAMES[arm] or len(trajectory.points) < 2
                or not all(len(p.velocities) == len(trajectory.joint_names) for p in trajectory.points)):
            raise RuntimeError('Selection does not contain a validated executable cubic path')
        check_path_start(trajectory, planned)
        self.scene.begin(selected.selected_candidate, f'pregrasp_execution_{time.monotonic_ns()}')
        primary_error = None
        try:
            # Hardware dispatch obtains a new frame immediately before checking
            # the exact synchronized command. Preliminary checks only need a
            # currently fresh map; waiting for another new frame here serializes
            # two camera cycles without replacing the authoritative final check.
            if real is not None:
                self.scene_monitor.wait()
            else:
                self.scene_monitor.wait(after_stamp_ns=self.get_clock().now().nanoseconds)
            self.motion.set_current_state(measured)
            current = JointSolution(ARM_JOINT_NAMES[arm], tuple(planned[n] for n in ARM_JOINT_NAMES[arm]))
            minimum_opening = float(selected.gripper_joint_position)
            commanded_opening = minimum_opening
            if getattr(self, 'real', None) is not None:
                required_width = float(selected.selected_candidate.required_opening_m)
                margin = float(self.config['pregrasp_opening_margin_m'])
                commanded_opening = self.arm_config.opening(
                    min(self.arm_config.width, required_width + margin))
            if not self.motion.gripper_sweep_is_valid(
                    arm, current, planned[f'{arm}_gripper_joint'], commanded_opening, .05):
                raise RuntimeError('Opening the gripper intersects the current scene')
            self.motion.set_gripper_position(arm, commanded_opening)
            path = PlannedPath(tuple(trajectory.joint_names), tuple(tuple(p.positions) for p in trajectory.points))
            checked = self.motion._validated_cubic_path(arm, path)
            if checked is None:
                raise RuntimeError('Pregrasp path is no longer collision-free')
            # Recheck the original start after potentially slow scene validation.
            state = self.measured()
            check_start_state(dict(zip(state.name, state.position)), planned, **start_options)
            jaw = JointTrajectory(joint_names=[f'{arm}_gripper_joint'])
            first = JointTrajectoryPoint(positions=[planned[f'{arm}_gripper_joint']], velocities=[0.])
            last = JointTrajectoryPoint(positions=[commanded_opening], velocities=[0.])
            last.time_from_start.sec = 5
            jaw.points = [first, last]
            arm_command = path_to_message(checked)
            if getattr(self, 'real', None) is not None:
                # Continuous depth is the default collection policy. The
                # updater masks measured robot/attached geometry each frame.
                if not getattr(self, 'config', {}).get('continuous_scene_updates', True):
                    self.real.begin_sensor_scene_freeze()
                    self._grasp_scene_freeze_held = True
                try:
                    state = self.real.execute_pregrasp_parallel(
                        arm, arm_command, jaw, minimum_opening,
                        1 if self.arm_config.opened > self.arm_config.closed else -1)
                except BaseException as motion_error:
                    try:
                        self.release_grasp_scene_freeze()
                    except Exception as cleanup_error:
                        self._scene_cleanup_error = str(cleanup_error)
                        self.get_logger().error(
                            'Scene-freeze cleanup failed after primary error '
                            f'{type(motion_error).__name__}: {motion_error}; '
                            f'cleanup={cleanup_error}'
                        )
                    raise
            else:
                self.execute_trajectory(arm, jaw, 'gripper')
                state = self.execute_trajectory(arm, arm_command, 'arm')
            return state.header.stamp.sec*10**9+state.header.stamp.nanosec
        except BaseException as error:
            primary_error = error
            raise
        finally:
            # A hardware execution failure may already have latched controller
            # stop.  MoveIt's scene service then depends on stopped joint/TF
            # publishers and can time out, masking the actionable motion error.
            # The collection process is exiting and control recovery restarts
            # MoveIt, so defer this cleanup instead of replacing the primary
            # exception with a secondary planning-scene timeout.
            real = getattr(self, 'real', None)
            if primary_error is not None and real is not None and real.fault:
                self._scene_cleanup_error = (
                    'Planning-scene restore deferred after latched physical stop; '
                    'recover control before replanning'
                )
                self.get_logger().error(self._scene_cleanup_error)
            else:
                try:
                    self.scene.restore()
                except Exception as cleanup_error:
                    self._scene_cleanup_error = str(cleanup_error)
                    if primary_error is None:
                        raise
                    self.get_logger().error(
                        'Planning-scene cleanup failed after primary error '
                        f'{type(primary_error).__name__}: {primary_error}; '
                        f'cleanup={cleanup_error}'
                    )

    def before_execution(self, selected):
        if getattr(self, 'real', None) is not None:
            raise RuntimeError('Physical execution requires the configured collection_pipeline entrypoint')

    def release_grasp_scene_freeze(self):
        """Resume OctoMap updates after the full collection cycle or failure."""
        if not getattr(self, '_grasp_scene_freeze_held', False):
            return
        self.real.end_sensor_scene_freeze()
        self._grasp_scene_freeze_held = False

    def target_translation_limit(self) -> float:
        return float(self.get_parameter('maximum_target_translation_m').value)

    def validate_target_translation(self, selected, before, after,
                                    displacement: float, report: dict) -> None:
        """Reject a changed target before handing control to an execution subclass."""
        del selected, before, after, report
        if displacement > self.target_translation_limit():
            raise RuntimeError('Target surface changed; a new grasp plan is required')

    def after_pregrasp(self, selected, wrist_request, report,
                       before_observation=None, after_observation=None):
        pass

    def wrist_observation_required(self) -> bool:
        return True

    def head_reobservation_required(self) -> bool:
        return True

    def observe_arrival(self, selected, before, arrival, report):
        return self.observe(selected.reference_id, arrival)

    def run(self, query):
        report = {'mode': self.mode, 'success': False, 'pregrasp_executed': False,
                  'grasp_executed': False, 'wrist_checks': 0}
        self._scene_cleanup_error = ''
        selected, wrist_request = None, None
        try:
            self.scene_monitor.wait()
            selected = self.run_action(self.validation, ValidatePick.Goal(query=query),
                                       self.get_parameter('pipeline_timeout_sec').value, allow_aborted=True)
            report['capture_stamp_ns'] = selected.detections.header.stamp.sec*10**9+selected.detections.header.stamp.nanosec
            if not selected.success:
                report.update(error_code=selected.error_code, detection_count=len(selected.detections.detections))
                raise RuntimeError(selected.message)
            report.update(arm=selected.selected_arm, candidate_index=selected.selected_candidate_index,
                          planner_id=selected.planner_id, evidence_json_path=selected.evidence_json_path)
            report['pregrasp_joint_state'] = dict(zip(
                selected.pregrasp_joint_state.name,
                (float(value) for value in selected.pregrasp_joint_state.position),
                strict=True,
            ))
            report['grasp_joint_state'] = dict(zip(
                selected.grasp_joint_state.name,
                (float(value) for value in selected.grasp_joint_state.position),
                strict=True,
            ))
            if not selected.reference_id:
                raise RuntimeError('Enable pin_object_reference and YOLOE-seg reference observation')
            before = after = None
            head_reobservation = self.head_reobservation_required()
            if head_reobservation:
                before = self.observe(selected.reference_id, self.get_clock().now().nanoseconds)
                report['reference_capture_ns'] = before.header.stamp.sec*10**9+before.header.stamp.nanosec
                report['target_observation_before'] = reference_observation_metrics(before)
            else:
                report.update(target_motion_assumed_stationary=True,
                              head_reobservation_performed=False)
            if self.mode == 'plan_only':
                report.update(success=True, message='Plan and current-view observation verified; no motion requested')
                return report
            if head_reobservation:
                center = selected.selected_object.obb_pose.position
                observed = before.observed_center
                extent = selected.selected_object.obb_size
                # A visible-surface center is not the object's completed center.
                # Require it to remain inside the planned OBB's enclosing sphere.
                bound = math.sqrt(extent.x**2+extent.y**2+extent.z**2)/2
                if math.dist((center.x, center.y, center.z),
                             (observed.x, observed.y, observed.z)) > bound + .01:
                    raise RuntimeError('Fresh target surface is outside the planned object; replan required')
            self.before_execution(selected)
            if self.wrist_observation_required():
                self.start_wrist_camera(selected.selected_arm)
            arrival = self.execute_pregrasp(selected)
            report['pregrasp_executed'] = getattr(self, 'config', {}).get(
                'pregrasp_motion_enabled', True)
            report['arrival_stamp_ns'] = arrival
            if report['pregrasp_executed'] and getattr(self, 'real', None) is not None:
                report['pregrasp_execution_metrics'] = deepcopy(
                    self.real.last_execution_metrics)
            if head_reobservation:
                after = self.observe_arrival(selected, before, arrival, report)
                report['target_observation_after'] = reference_observation_metrics(after)
                old = (before.observed_center.x, before.observed_center.y, before.observed_center.z)
                new = (after.observed_center.x, after.observed_center.y, after.observed_center.z)
                displacement = math.dist(old, new)
                report['observed_surface_displacement_m'] = displacement
                self.validate_target_translation(
                    selected, before, after, displacement, report)
            if self.wrist_observation_required():
                wrist_request = ObserveWristTarget.Request(operation=ObserveWristTarget.Request.HANDOFF,
                    arm=selected.selected_arm, source_snapshot_id=after.source_snapshot_id,
                    source_object_id=after.source_object_id, label=after.source_label,
                    detector_label=after.source_detector_label,
                    source_confidence=float(after.source_confidence), after_stamp_ns=arrival)
                wrist_request.expected_pose.header = deepcopy(after.header)
                wrist_request.expected_pose.pose.position = deepcopy(after.observed_center)
                wrist_request.expected_pose.pose.orientation.w = 1.
                wrist_request.size = deepcopy(after.observed_extent)
                handoff = self.call(self.wrist, wrist_request, 60.)
                wrist_request.reference_id = handoff.reference_id
                wrist_request.operation = wrist_request.CHECK
                wrist_request.after_stamp_ns = handoff.header.stamp.sec*10**9+handoff.header.stamp.nanosec
                for _ in range(int(self.get_parameter('wrist_checks').value)):
                    observation = self.call(self.wrist, wrist_request, 60.)
                    stamp = observation.header.stamp.sec*10**9+observation.header.stamp.nanosec
                    if stamp <= wrist_request.after_stamp_ns:
                        raise RuntimeError('Wrist tracking did not advance')
                    wrist_request.after_stamp_ns = stamp
                    report['wrist_checks'] += 1
            self.after_pregrasp(
                selected, wrist_request, report,
                before_observation=before,
                after_observation=after,
            )
            message = (f'{self.mode} execution and fresh observations verified' if head_reobservation else
                       f'{self.mode} execution completed; stationary target assumed without head reobservation')
            report.update(success=True, message=message)
        except Exception as error:
            report['message'] = str(error)
            report['error_type'] = type(error).__name__
            if self._scene_cleanup_error:
                report['cleanup_error'] = self._scene_cleanup_error
            real = getattr(self, 'real', None)
            if real is not None and real.last_execution_metrics:
                report['pregrasp_motion_attempted'] = True
                report['pregrasp_execution_metrics'] = deepcopy(
                    real.last_execution_metrics)
            if (getattr(self, 'real', None) is not None
                    and not (report.get('error_code') == 2 and report.get('detection_count') == 0)):
                report['stop'] = self.real.fail_stop()
        finally:
            try:
                self.stop_wrist_camera()
            except Exception as error:
                report.setdefault('cleanup_error', str(error))
                report['success'] = False
            try:
                self.release_grasp_scene_freeze()
            except Exception as error:
                report.setdefault('cleanup_error', str(error))
                report['success'] = False
            for client, request in (
                (self.reference, ObserveObjectReference.Request(operation=ObserveObjectReference.Request.CLEAR,
                    reference_id=selected.reference_id) if selected and selected.reference_id else None),
                (self.wrist, wrist_request if wrist_request and wrist_request.reference_id else None)):
                if request is not None:
                    request.operation = request.CLEAR
                    try:
                        self.call(client, request, 10.)
                    except Exception as error:
                        report.setdefault('cleanup_error', str(error))
                        report['success'] = False
        return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('plan_only', 'mujoco'), default='plan_only')
    parser.add_argument('--query', default='')
    parser.add_argument('--output', required=True)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = PregraspPipeline(mode=options.mode)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    report = {'success': False, 'message': 'Interrupted'}
    try:
        report = node.run(options.query)
    finally:
        output = Path(options.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
        print(json.dumps(report, ensure_ascii=False))
        node.scene_monitor.close()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if report['success'] else 1)
