"""Guarded ROS trajectory transport to an already activated Feetech runtime.

Never starts hardware, enables torque, clears faults, or changes driver limits.
"""
from copy import deepcopy
from collections import deque
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState, JointTolerance
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .core.physical_grasp_selection import ARM_JOINT_NAMES, JointSolution
from .state_validity_batch import StateValidityBatch
from .execution_timing import timed_operation
from .readonly_services import ReadOnlyBatch, call_readonly_services
from .trajectory_io import command_samples_to_message
from .core.cartesian import CartesianPose, orientation_error_rad
from .core.height_correction import validate_height_correctable_endpoint
from .core.hardware_execution import (
    ArmHold, CollisionValidationError, CommandPathValidationError, JointRangeValidationError,
    CommunicationRecovered, ExecutionCanceled, bounded_path, bridge_joint_path,
    continue_joint_path, physical_limits, prepare_bounded_path, remaining_joint_path,
)
from .core.joint_path import executable_joint_path
from .core.joint_stability import JointStabilityHistory
from .core.recorded_trajectory import (
    ArmCommandRecording, CommandPoint, RecordedTrajectory, validate_recorded_trajectory,
    stopped_recorded_window,
)
from .core.motion_monitor import MotionMonitor, MotionMonitorConfig
from .core.gripper_contact import ContactState, LiveContactHistory, load_contact_criteria


@dataclass(frozen=True)
class ReturnEndpoint:
    target: CartesianPose
    measured: CartesianPose
    positions: tuple[tuple[str, float], ...]
    models: tuple[str, str]
    submissions: int
    generation: int
    checked_at: float


@dataclass
class ReturnPreparation:
    arm: str
    phase: str = 'retreat'
    endpoint: ReturnEndpoint | None = None
    endpoint_reused: bool = False
    batch: ReadOnlyBatch | None = None
    submissions: int = 0
    generation: int = 0


def path_validation_drift(baseline, latest, commanded, commanded_tolerance, other_tolerance):
    """Return joints that moved beyond their role-specific validation tolerance."""
    current = dict(zip(latest.name, latest.position))
    commanded = set(commanded)
    return {
        name: abs(current[name] - start)
        for name, start in baseline.items()
        if abs(current[name] - start) > (
            commanded_tolerance if name in commanded else other_tolerance
        )
    }


def collision_contact_details(response):
    """Return JSON-safe MoveIt contact evidence for operator diagnostics."""
    return [
        {
            'body_1': contact.contact_body_1,
            'body_2': contact.contact_body_2,
            'depth_m': float(contact.depth),
            'position_m': [
                float(contact.position.x),
                float(contact.position.y),
                float(contact.position.z),
            ],
        }
        for contact in response.contacts
    ]


def transient_octomap_arm_collision(response, arm):
    """True only for an OctoMap contact against the commanded arm links."""
    if response.valid or not response.contacts or arm not in ARM_JOINT_NAMES:
        return False
    for contact in response.contacts:
        bodies = (contact.contact_body_1, contact.contact_body_2)
        if bodies.count('<octomap>') != 1:
            return False
        robot_link = bodies[1] if bodies[0] == '<octomap>' else bodies[0]
        if not robot_link.startswith(f'{arm}_'):
            return False
    return True


class HardwareExecution:
    def __init__(self, node):
        self.node = node
        self.guard = None
        self.guard_time = 0.
        self.fault = ''
        self.uncommanded = None
        self.stop_report = None
        self.last_execution_metrics = {}
        self.motion_submissions = 0
        self.arm_holds: dict[str, ArmHold] = {}
        self.communication_recovery_count = 0
        self.complete_feedback_stamp_ns: int | None = None
        self._active_recovery_generation: int | None = None
        node.declare_parameter('hardware_reuse_settle_history', True)
        node.declare_parameter('hardware_reuse_scene_barrier', False)
        for name in ('parallel_model_queries', 'sparse_cubic_commands', 'share_return_preparation'):
            parameter = 'hardware_'+name
            node.declare_parameter(parameter, False)
            enabled = node.get_parameter(parameter).value
            if type(enabled) is not bool:
                raise ValueError(parameter+' must be boolean')
            setattr(self, name, enabled)
        node.declare_parameter('hardware_state_preparation_monitor_period_sec', .01)
        self._state_preparation_monitor_period_sec = self.value(
            'hardware_state_preparation_monitor_period_sec')
        if self._state_preparation_monitor_period_sec > .01:
            raise ValueError('State preparation monitoring period must be at most 0.01 seconds')
        self.reuse_scene_barrier = node.get_parameter('hardware_reuse_scene_barrier').value
        if type(self.reuse_scene_barrier) is not bool:
            raise ValueError('hardware_reuse_scene_barrier must be boolean')
        reuse_history = node.get_parameter('hardware_reuse_settle_history').value
        if type(reuse_history) is not bool:
            raise ValueError('hardware_reuse_settle_history must be boolean')
        self._stability_history = JointStabilityHistory() if reuse_history else None
        node.declare_parameter('gripper_contact_rule_file', str(
            Path(get_package_share_directory('cleany_skill_executor')) /
            'config/gripper_contact_decision.yaml'))
        self.contact_criteria = load_contact_criteria(
            node.get_parameter('gripper_contact_rule_file').value)
        self.contact_histories = {
            arm: LiveContactHistory(criteria) for arm, criteria in self.contact_criteria.items()
        }
        self.contact_lock = threading.Lock()
        self.contact_subscription = node.create_subscription(
            DynamicJointState, '/joint_state_broadcaster/dynamic_joint_states',
            self.on_contact_feedback, 10)
        self._motion_monitors: set[MotionMonitor] = set()
        self._sensor_scene_freeze_depth = 0
        node.declare_parameter('continuous_sensor_scene_updates', True)
        node.declare_parameter(
            'sensor_scene_update_service',
            '/perception/scene_cloud_updates_enabled',
        )
        self.subscription = node.create_subscription(String, '/real_control/status', self.on_guard, 10)
        self.hardware = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.parameters = node.create_client(GetParameters, '/controller_manager/get_parameters')
        self.moveit_parameters = node.create_client(GetParameters, '/move_group/get_parameters')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        node.declare_parameter('hardware_state_validity_batch_size', 32)
        self._batch_validity = StateValidityBatch(node,
            batch_size=node.get_parameter('hardware_state_validity_batch_size').value)
        self.stop = node.create_client(Trigger, '/real_control/stop')
        self.scene_updates = node.create_client(
            SetBool,
            str(node.get_parameter('sensor_scene_update_service').value),
        )
        for name, value in {'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
                            'hardware_path_step_rad': .01,
                            'hardware_pickup_height_search_tolerance_m': 0.,
                            'hardware_path_simplification_tolerance_rad': .002,
                            'hardware_start_delay_sec': .8,
                            'hardware_return_start_delay_sec': .8,
                            'hardware_guard_max_age_sec': .25, 'hardware_start_tolerance_rad': .005,
                            'hardware_start_blend_max_rad': .03,
                            'hardware_path_refresh_attempts': 1,
                            'hardware_communication_resume_limit': 1,
                            'hardware_arrival_tolerance_rad': .07,
                            'hardware_uncommanded_tolerance_rad': .03,
                            'hardware_settle_time_sec': .5,
                            'hardware_height_correction_min_duration_sec': 1.,
                            'hardware_release_retreat_min_duration_sec': 1.,
                            'hardware_release_retreat_position_tolerance_m': 1e-9,
                            'hardware_release_retreat_orientation_tolerance_deg': 1e-9,
                            'hardware_gripper_contact_timeout_sec': 3.,
                            'hardware_endpoint_settle_duration_sec': .3,
                            'hardware_endpoint_settle_timeout_sec': 2.,
                            'hardware_endpoint_settle_velocity_rad_s': .02,
                            'hardware_tcp_position_tolerance_m': .008,
                            'hardware_tcp_orientation_tolerance_deg': 6.,
                            'hardware_command_continuity_max_offset_rad': .06,
                            'hardware_motion_max_error_rad': .07,
                            'hardware_motion_error_duration_sec': .5,
                            'hardware_motion_response_window_sec': 1.,
                            'hardware_motion_response_error_rad': .03,
                            'hardware_motion_minimum_motion_rad': .005,
                            'hardware_motion_feedback_timeout_sec': 1.}.items():
            node.declare_parameter(name, value)
        node.declare_parameter('hardware_motion_monitor_enabled', True)
        node.declare_parameter('hardware_motion_abort_on_deviation', True)
        node.declare_parameter('hardware_uncommanded_motion_check_enabled', True)
        self.uncommanded_motion_check_enabled = node.get_parameter(
            'hardware_uncommanded_motion_check_enabled').value
        if type(self.uncommanded_motion_check_enabled) is not bool:
            raise ValueError('hardware_uncommanded_motion_check_enabled must be boolean')
        node.declare_parameter('hardware_gripper_opening_position_range_check_enabled', True)
        self.gripper_opening_position_range_check_enabled = node.get_parameter(
            'hardware_gripper_opening_position_range_check_enabled').value
        if type(self.gripper_opening_position_range_check_enabled) is not bool:
            raise ValueError('hardware_gripper_opening_position_range_check_enabled must be boolean')
        abort_on_deviation = node.get_parameter('hardware_motion_abort_on_deviation').value
        if type(abort_on_deviation) is not bool:
            raise ValueError('hardware_motion_abort_on_deviation must be boolean')
        enabled = node.get_parameter('hardware_motion_monitor_enabled').value
        if type(enabled) is not bool:
            raise ValueError('hardware_motion_monitor_enabled must be boolean')
        self._motion_config = None if not enabled else MotionMonitorConfig(
            self.value('hardware_motion_max_error_rad'),
            self.value('hardware_motion_error_duration_sec'),
            self.value('hardware_motion_response_window_sec'),
            self.value('hardware_motion_response_error_rad'),
            self.value('hardware_motion_minimum_motion_rad'),
            self.value('hardware_motion_feedback_timeout_sec'), abort_on_deviation)

    def motion_client(self, arm, controller):
        name = f'/{arm}_{controller}_controller/follow_joint_trajectory'
        factory = getattr(self.node, 'create_motion_client', None)
        return (factory(FollowJointTrajectory, name) if factory is not None
                else ActionClient(self.node, FollowJointTrajectory, name))

    def motion_clients(self, arm, controllers):
        """Release earlier leases if acquiring a later controller fails."""
        clients = {}
        try:
            for controller in controllers:
                clients[controller] = HardwareExecution.motion_client(self, arm, controller)
        except BaseException:
            for client in clients.values():
                client.destroy()
            raise
        return clients

    def value(self, name):
        result = float(self.node.get_parameter(name).value)
        if not math.isfinite(result) or result <= 0:
            raise ValueError('Invalid parameter '+name)
        return result

    def positive_integer(self, name):
        raw = self.node.get_parameter(name).value
        if type(raw) is not int or raw < 1:
            raise ValueError('Invalid parameter '+name)
        return raw

    def on_guard(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError('Malformed guard')
            self.guard = status
            self.guard_time = time.monotonic()
            if status.get('state') == 'fault':
                self.fault = status.get('reason') or 'Hardware fault'
        except (ValueError, TypeError):
            self.fault = 'Malformed hardware guard status'

    def _start_motion_monitor(self, names):
        config = getattr(self, '_motion_config', None)
        if config is None:
            return None
        monitor = MotionMonitor(tuple(names), config, time.monotonic())
        self._motion_monitors.add(monitor)
        return monitor

    @staticmethod
    def _motion_feedback(monitor, message):
        if monitor is None:
            return
        try:
            feedback = message.feedback
            monitor.observe(feedback.joint_names, feedback.desired.positions,
                            feedback.actual.positions, time.monotonic())
        except (AttributeError, TypeError, ValueError):
            monitor.observe((), (), (), time.monotonic())

    def _finish_motion_monitor(self, monitor):
        if monitor is None:
            return {}
        self._motion_monitors.discard(monitor)
        return monitor.report()

    def monitor(self):
        check_execution = getattr(self.node, 'check_execution', None)
        if check_execution is not None:
            check_execution()
        if (self.fault or self.guard is None or self.guard.get('state') != 'ready'
                or time.monotonic()-self.guard_time > self.value('hardware_guard_max_age_sec')):
            raise RuntimeError(self.fault or 'Physical guard unavailable/stale/not ready')
        state = self.node.measured()
        feedback_stamp = getattr(self, 'complete_feedback_stamp_ns', None)
        if feedback_stamp is not None:
            age_ns = self.node.get_clock().now().nanoseconds-feedback_stamp
            if age_ns < 0 or age_ns > self.value('hardware_guard_max_age_sec')*1e9:
                raise RuntimeError('Physical complete-feedback acquisition stale; broadcaster replay cannot confirm recovery')
        q = dict(zip(state.name, state.position))
        if not {'head_pan_joint', 'head_tilt_joint'} <= q.keys():
            raise RuntimeError('Physical execution requires all 14 measured joints')
        if self.uncommanded and getattr(self, 'uncommanded_motion_check_enabled', True):
            tolerance = self.value('hardware_uncommanded_tolerance_rad')
            moved = {n: q[n]-v for n, v in self.uncommanded.items() if abs(q[n]-v) > tolerance}
            if moved:
                details = ', '.join(f'{name} delta={delta:+.6f}rad'
                                    for name, delta in sorted(moved.items()))
                raise RuntimeError(f'Uncommanded joint moved: {details}; tolerance={tolerance:.6f}rad')
        for monitor in tuple(getattr(self, '_motion_monitors', ())):
            monitor.check(time.monotonic())
            for warning in monitor.take_warnings():
                self.node.get_logger().warning(warning+'; continuing supervised motion')
        if hasattr(self.node, 'monitor_payload'):
            self.node.monitor_payload()
        generation = getattr(self, '_active_recovery_generation', None)
        if generation is not None and self.communication_recovery_count > generation:
            raise CommunicationRecovered(self.communication_recovery_count)
        return state

    @timed_operation('settle_wait', node_attribute='node')
    def wait_for_settle(self, names, *, duration_sec=None):
        """Return a fresh state after the commanded joints remain nearly still."""
        duration = (self.value('hardware_endpoint_settle_duration_sec')
                    if duration_sec is None else duration_sec)
        timeout = self.value('hardware_endpoint_settle_timeout_sec')
        velocity = self.value('hardware_endpoint_settle_velocity_rad_s')
        deadline = time.monotonic()+timeout
        samples = deque()
        history = getattr(self, '_stability_history', None)
        while True:
            now = time.monotonic()
            state = self.monitor()
            positions = dict(zip(state.name, state.position))
            if not set(names) <= positions.keys() or any(
                    not math.isfinite(positions[name]) for name in names):
                raise RuntimeError('Endpoint settling requires finite measured joints')
            if history is not None:
                if history.stable(names, positions,
                        now_ns=self.node.get_clock().now().nanoseconds,
                        duration_sec=duration,
                        maximum_age_sec=self.value('hardware_guard_max_age_sec'),
                        velocity_rad_s=velocity):
                    return state
            samples.append((now, tuple(float(positions[name]) for name in names)))
            while len(samples) > 1 and samples[1][0] <= now-duration:
                samples.popleft()
            if history is None and samples and now-samples[0][0] >= duration:
                span_limit = velocity*(now-samples[0][0])
                if all(max(values)-min(values) <= span_limit for values in zip(
                        *(positions for _, positions in samples), strict=True)):
                    return state
            if now >= deadline:
                raise RuntimeError('Arm did not settle before endpoint verification')
            time.sleep(.02)

    def _wait_before_execution(self, names):
        began = time.monotonic()
        try:
            if getattr(self, '_stability_history', None) is not None:
                return self.wait_for_settle(names, duration_sec=self.value('hardware_settle_time_sec'))
            time.sleep(self.value('hardware_settle_time_sec'))
            return self.monitor()
        finally:
            self._last_start_settle_sec = time.monotonic()-began

    def _reset_stability_history(self):
        history = getattr(self, '_stability_history', None)
        if history is not None:
            history.clear()

    def endpoint_metrics(self, arm, target, actual, *, correction_attempt, command):
        names = ARM_JOINT_NAMES[arm]
        positions = dict(zip(actual.name, actual.position))
        measured = tuple(float(positions[name]) for name in names)
        if any(not math.isfinite(value) for value in measured):
            raise RuntimeError('Endpoint verification received nonfinite joints')
        target_pose = self.node.motion.forward_kinematics(
            arm, JointSolution(names, tuple(target)))
        actual_pose = self.node.motion.forward_kinematics(
            arm, JointSolution(names, measured))
        if any(not math.isfinite(value) for value in (*target_pose.position, *actual_pose.position)):
            raise RuntimeError('Endpoint verification received nonfinite TCP positions')
        orientation_error = orientation_error_rad(
            tuple(target_pose.orientation_xyzw),
            tuple(actual_pose.orientation_xyzw))
        return {
            'correction_attempt': correction_attempt,
            'target_arm_joint_rad': list(target),
            'commanded_arm_joint_rad': list(command),
            'actual_arm_joint_rad': list(measured),
            'final_arm_joint_error_rad': [
                value-desired for value, desired in zip(measured, target, strict=True)
            ],
            'target_tcp_position_m': list(target_pose.position),
            'actual_tcp_position_m': list(actual_pose.position),
            'target_tcp_orientation_xyzw': list(target_pose.orientation_xyzw),
            'actual_tcp_orientation_xyzw': list(actual_pose.orientation_xyzw),
            'final_tcp_position_error_m': math.dist(
                target_pose.position, actual_pose.position),
            'final_tcp_orientation_error_deg': math.degrees(orientation_error),
        }

    def capture_scene_barrier(self, arm):
        """Allow cloud reception during planning while the settled pose stays identical."""
        if not getattr(self, 'reuse_scene_barrier', False):
            return None
        state = self.wait_for_settle((*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint'))
        stamp_ns = self.node.get_clock().now().nanoseconds
        positions = dict(zip(state.name, state.position, strict=True))
        return dict(stamp_ns=stamp_ns,
                    positions=positions,
                    submissions=self.motion_submissions,
                    generation=self.communication_recovery_count)

    def wait_for_fresh_sensor_scene(self, stage, *, barrier=None):
        """Wait for a cloud captured after the settled robot pose."""
        began = time.monotonic()
        after_stamp_ns = self.node.get_clock().now().nanoseconds
        reused = False
        if barrier is not None:
            state = self.monitor()
            if (0 < barrier['stamp_ns'] <= after_stamp_ns
                    and barrier['positions'] == dict(zip(state.name, state.position, strict=True))
                    and barrier['submissions'] == self.motion_submissions
                    and barrier['generation'] == self.communication_recovery_count):
                after_stamp_ns = barrier['stamp_ns']
                reused = True

        def canceled():
            self.monitor()
            return False

        self.node.scene_monitor.wait(
            after_stamp_ns=after_stamp_ns,
            canceled=canceled,
        )
        self.monitor()
        self.node.get_logger().info(json.dumps({
            'stage': 'hardware_sensor_scene_refreshed',
            'for_stage': stage,
            'after_stamp_ns': after_stamp_ns,
            'planning_barrier_reused': reused,
            'planning_barrier_age_sec': (
                (self.node.get_clock().now().nanoseconds-after_stamp_ns)/1e9 if reused else None),
            'wall_sec': time.monotonic() - began,
        }))

    @property
    def sensor_scene_frozen(self):
        return self._sensor_scene_freeze_depth > 0

    def set_sensor_scene_updates(self, enabled):
        """Control the sole point-cloud feed used by the MoveIt OctoMap."""
        request = SetBool.Request(data=bool(enabled))
        response = self.node.call(self.scene_updates, request, 5.)
        if not response.success:
            raise RuntimeError(response.message or 'Could not change sensor scene updates')
        self.node.get_logger().info(json.dumps({
            'stage': 'hardware_sensor_scene_updates',
            'enabled': bool(enabled),
            'message': response.message,
        }))

    def begin_sensor_scene_freeze(self):
        """Acquire a nested hold on one fresh pre-motion OctoMap."""
        if not self.sensor_scene_frozen:
            self.wait_for_fresh_sensor_scene('pre_motion_octomap')
            self.set_sensor_scene_updates(False)
        self._sensor_scene_freeze_depth += 1

    def end_sensor_scene_freeze(self):
        """Release one hold and resume updates after the outermost user."""
        if self._sensor_scene_freeze_depth <= 0:
            raise RuntimeError('Sensor scene freeze is not active')
        self._sensor_scene_freeze_depth -= 1
        if not self.sensor_scene_frozen:
            try:
                self.set_sensor_scene_updates(True)
            except Exception:
                # Keep ownership so the caller can retry the resume operation.
                self._sensor_scene_freeze_depth = 1
                raise

    @contextmanager
    def frozen_sensor_scene(self, *, barrier=None):
        """Require a fresh map; freeze only when the configured policy requests it."""
        config = getattr(self.node, 'config', {})
        continuous = config.get('continuous_scene_updates',
            self.node.get_parameter('continuous_sensor_scene_updates').value)
        if continuous:
            self.wait_for_fresh_sensor_scene('continuous_pre_motion_octomap',
                                            **({'barrier': barrier} if barrier is not None else {}))
            yield
            return
        self.begin_sensor_scene_freeze()
        failure = None
        try:
            yield
        except BaseException as error:
            failure = error
            raise
        finally:
            try:
                self.end_sensor_scene_freeze()
            except Exception as error:
                if failure is None:
                    raise
                self.node.get_logger().error(
                    'Could not resume sensor scene updates after failure: '
                    + str(error)
                )

    def _collision_responses(self, states):
        batch = getattr(self, '_batch_validity', None)
        size = batch.batch_size if batch is not None else 1
        for start in range(0, len(states), size):
            self.monitor()
            chunk = states[start:start+size]
            response = (batch.check(chunk, lambda client, request:
                self.node.call(client, request, timeout=5. * len(chunk), monitor=True))
                if batch is not None else None)
            self.monitor()
            if response is None:
                for offset, state in enumerate(chunk):
                    self.monitor()
                    result = self.node.call(self.validity, GetStateValidity.Request(robot_state=state))
                    yield start+offset, result
                    if not result.valid:
                        return
            else:
                for offset, valid in enumerate(response.valid):
                    yield start+offset, GetStateValidity.Response(valid=valid,
                        contacts=list(response.contacts) if not valid else [])
                if response.first_invalid_index != -1:
                    return

    @timed_operation('collision_state_preparation', node_attribute='node')
    def _prepare_collision_states(self, arm, expected, samples, baseline, measured,
                                  *, hold_bridge_samples=0):
        """Build identical full states with bounded checks before any RPC or goal.

        Only unsent message construction uses the reduced monitor cadence.
        Collision RPCs and physical execution retain their own monitoring.
        """
        names = list(measured.name)
        slots = [names.index(name) for name in expected]
        positions = [baseline[name] for name in names]
        guard = getattr(self, '_approach_clearance_guard', None)
        use_guard = guard is not None and tuple(expected) == ARM_JOINT_NAMES[arm]
        period = getattr(self, '_state_preparation_monitor_period_sec', .01)
        last_check = -math.inf
        states = []
        for index, sample in enumerate(samples):
            if index % 64 == 0 or time.monotonic()-last_check >= period:
                self.monitor()
                last_check = time.monotonic()
            if len(sample.positions) != len(slots):
                raise ValueError('Collision sample has incorrect joint count')
            values = list(positions)
            for slot, value in zip(slots, sample.positions, strict=True):
                values[slot] = value
            if use_guard:
                phase = 'hold_bridge' if index < hold_bridge_samples else 'command_path'
                guard({**baseline, **dict(zip(expected, sample.positions))}, index, phase)
            state = JointState(header=deepcopy(measured.header), name=list(names),
                               position=values, velocity=list(measured.velocity),
                               effort=list(measured.effort))
            states.append(RobotState(joint_state=state, is_diff=True))
        self.monitor()
        return states

    @timed_operation('collision_validation', node_attribute='node')
    def validate_collision_samples(
        self,
        arm,
        expected,
        samples,
        baseline,
        measured,
        *,
        hold_bridge_samples=0,
        trajectory_stage='hardware',
    ):
        """Validate the held-command entry and path against the current scene."""
        states = HardwareExecution._prepare_collision_states(
            self, arm, expected, samples, baseline, measured,
            hold_bridge_samples=hold_bridge_samples)
        for index, response in HardwareExecution._collision_responses(self, states):
            if response.valid:
                continue
            sample = samples[index]
            phase = 'hold_bridge' if index < hold_bridge_samples else 'command_path'
            contacts = collision_contact_details(response)
            octomap_arm_contact = transient_octomap_arm_collision(response, arm)
            self.node.get_logger().warning(json.dumps({
                'stage': 'hardware_collision_validation_failed',
                'trajectory_stage': trajectory_stage,
                'arm': arm,
                'sample_index': index,
                'sample_joint_names': list(expected),
                'sample_joint_rad': list(sample.positions),
                'full_joint_names': list(states[index].joint_state.name),
                'full_joint_rad': list(states[index].joint_state.position),
                'phase': phase,
                'hold_bridge_samples': hold_bridge_samples,
                'frozen_pre_motion_octomap': self.sensor_scene_frozen,
                'octomap_arm_contact': octomap_arm_contact,
                'contacts': contacts,
            }))
            raise CollisionValidationError(
                f'{trajectory_stage} trajectory failed collision validation: '
                f'phase={phase} sample={index} contacts={contacts}',
                phase=phase, sample_index=index, contacts=tuple(contacts),
                octomap_arm_contact=octomap_arm_contact)

    @timed_operation('endpoint_verification', node_attribute='node')
    def verify_arm_endpoint(self, arm, target, limits, actual=None, *, check_orientation=True,
                            allow_endpoint_error=False, defer_height_shortfall: bool = False,
                            endpoint_pose: CartesianPose | None = None):
        """Verify the settled TCP once without issuing additional arm goals."""
        if type(defer_height_shortfall) is not bool:
            raise ValueError('defer_height_shortfall must be boolean')
        names = ARM_JOINT_NAMES[arm]
        target = tuple(float(value) for value in target)
        if len(target) != len(names) or any(not math.isfinite(value) for value in target):
            raise RuntimeError('Invalid arm endpoint target')
        hold = self.arm_holds.get(arm)
        command = hold.command if hold is not None else target
        actual = self.wait_for_settle(names)
        metrics = self.endpoint_metrics(
            arm, target, actual, correction_attempt=0, command=command)
        if endpoint_pose is not None:
            if not isinstance(endpoint_pose, CartesianPose):
                raise ValueError('Arm endpoint pose must be a CartesianPose')
            # The supporting joint command may intentionally exceed a loaded
            # physical goal. Keep its FK diagnostic, but judge measured arrival
            # against the destination rather than the compensation command.
            metrics['command_tcp_position_m'] = metrics['target_tcp_position_m']
            metrics['command_tcp_orientation_xyzw'] = metrics['target_tcp_orientation_xyzw']
            metrics['target_tcp_position_m'] = list(endpoint_pose.position)
            metrics['target_tcp_orientation_xyzw'] = list(endpoint_pose.orientation)
            metrics['endpoint_target_source'] = 'physical_goal'
            metrics['final_tcp_position_error_m'] = math.dist(
                endpoint_pose.position, metrics['actual_tcp_position_m'])
            metrics['final_tcp_orientation_error_deg'] = math.degrees(orientation_error_rad(
                endpoint_pose.orientation, tuple(metrics['actual_tcp_orientation_xyzw'])))
        within = (
            metrics['final_tcp_position_error_m']
            <= self.value('hardware_tcp_position_tolerance_m')
            and (not check_orientation
                 or metrics['final_tcp_orientation_error_deg']
                 <= self.value('hardware_tcp_orientation_tolerance_deg')))
        height_deferred = False
        if not within and defer_height_shortfall:
            try:
                validate_height_correctable_endpoint(
                    CartesianPose(tuple(metrics['actual_tcp_position_m']),
                                  tuple(metrics['actual_tcp_orientation_xyzw'])),
                    CartesianPose(tuple(metrics['target_tcp_position_m']),
                                  tuple(metrics['target_tcp_orientation_xyzw'])),
                    self.value('hardware_tcp_position_tolerance_m'),
                    math.radians(self.value('hardware_tcp_orientation_tolerance_deg'))
                    if check_orientation else math.pi)
                height_deferred = True
            except ValueError:
                pass
        summary = {**metrics, 'endpoint_corrections': 0,
                   'endpoint_history': [metrics],
                   'endpoint_accuracy_checks_enabled': not allow_endpoint_error,
                   'endpoint_within_tolerance': within,
                   'endpoint_height_shortfall_deferred': height_deferred}
        self.last_execution_metrics = {
            'stage': ('hardware_endpoint_verified' if within
                      else 'hardware_endpoint_height_shortfall_deferred' if height_deferred
                      else 'hardware_endpoint_error_ignored' if allow_endpoint_error
                      else 'hardware_endpoint_validation_failed'),
            **summary,
        }
        self.node.get_logger().info(json.dumps(self.last_execution_metrics))
        if not within and not height_deferred:
            message = ('Physical TCP endpoint outside tolerance: '
                f"position={metrics['final_tcp_position_error_m']:.6f}m "
                f"orientation={metrics['final_tcp_orientation_error_deg']:.3f}deg")
            if not allow_endpoint_error:
                raise RuntimeError(message)
            self.node.get_logger().warning(message+'; grasp trial will attempt gripper close')
        self.arm_holds[arm] = ArmHold(target, command)
        preparation = getattr(self, '_return_preparation', None)
        if (preparation is not None and preparation.arm == arm and preparation.phase == 'retreat'
                and within and check_orientation and not allow_endpoint_error
                and not height_deferred and endpoint_pose is not None):
            preparation.endpoint = ReturnEndpoint(endpoint_pose,
                CartesianPose(tuple(metrics['actual_tcp_position_m']),
                              tuple(metrics['actual_tcp_orientation_xyzw'])),
                tuple(sorted(zip(actual.name, map(float, actual.position), strict=True))),
                self._verified_models, self.motion_submissions,
                self.communication_recovery_count, time.monotonic())
            # These are new reads for the next command. They may overlap only
            # the remaining endpoint checks and return planning, never motion.
            HardwareExecution.prepare_return_phase(self, arm)
        return actual, summary

    @contextmanager
    def return_preparation(self, arm):
        """Share bounded preparation only inside one release-retreat/home operation."""
        if not getattr(self, 'share_return_preparation', False):
            yield
            return
        if arm not in ARM_JOINT_NAMES or getattr(self, '_return_preparation', None) is not None:
            raise RuntimeError('Return preparation requires one exclusive arm operation')
        preparation = ReturnPreparation(arm)
        self._return_preparation = preparation
        try:
            yield
        finally:
            if preparation.batch is not None:
                preparation.batch.close()
            self._return_preparation = None

    def _model_requests(self):
        return ((self.hardware, ListHardwareComponents.Request()),
                (self.controllers, ListControllers.Request()),
                (self.parameters, GetParameters.Request(names=['robot_description'])),
                (self.moveit_parameters, GetParameters.Request(
                    names=['robot_description', 'allow_trajectory_execution'])))

    def prepare_return_phase(self, arm):
        """Start the next fresh query batch once, without delaying for discovery."""
        preparation = getattr(self, '_return_preparation', None)
        if preparation is None or preparation.arm != arm or preparation.phase != 'retreat':
            return
        preparation.phase = 'home'
        self.monitor()
        calls = HardwareExecution._model_requests(self)
        if not all(client.service_is_ready() for client, _ in calls):
            self.node.event('return_model_prefetch', outcome='services_not_ready', arm=arm)
            return
        preparation.submissions = self.motion_submissions
        preparation.generation = self.communication_recovery_count
        preparation.batch = ReadOnlyBatch(self.node, calls, availability_timeout=0.)
        self.node.event('return_model_prefetch', outcome='started', arm=arm)

    def reuse_release_retreat_endpoint(self, arm, target):
        """Reuse FK only when the entire measured robot still matches the receipt."""
        preparation = getattr(self, '_return_preparation', None)
        if preparation is None or preparation.arm != arm or preparation.endpoint is None:
            return None
        receipt = preparation.endpoint
        state = self.monitor()
        current = tuple(sorted(zip(state.name, map(float, state.position), strict=True)))
        valid = (not preparation.endpoint_reused and receipt.target == target
                 and current == receipt.positions and receipt.models == self._verified_models
                 and receipt.submissions == self.motion_submissions
                 and receipt.generation == self.communication_recovery_count
                 and time.monotonic()-receipt.checked_at <= self.value('hardware_guard_max_age_sec'))
        self.node.event('return_endpoint_handoff', arm=arm, reused=valid)
        if not valid:
            return None
        preparation.endpoint_reused = True
        return receipt.measured

    def _return_model_responses(self, arm):
        preparation = getattr(self, '_return_preparation', None)
        self._last_return_model_prefetch_used = False
        if preparation is None or preparation.batch is None:
            return None
        batch, preparation.batch = preparation.batch, None
        valid = (preparation.arm == arm and preparation.submissions == self.motion_submissions
                 and preparation.generation == self.communication_recovery_count
                 and time.monotonic()-batch.started <= self.value('hardware_guard_max_age_sec'))
        if not valid:
            batch.close()
            self.node.event('return_model_prefetch', arm=arm, outcome='discarded')
            return None
        responses = batch.result()
        age = time.monotonic()-batch.started
        if (age > self.value('hardware_guard_max_age_sec')
                or preparation.submissions != self.motion_submissions
                or preparation.generation != self.communication_recovery_count):
            self.node.event('return_model_prefetch', arm=arm, outcome='invalidated_during_wait')
            return None
        self._last_return_model_prefetch_used = True
        self.node.event('return_model_prefetch', arm=arm, outcome='consumed', age_sec=age)
        return responses

    @timed_operation('model_verification', node_attribute='node')
    def verify(self, arm):
        began = time.monotonic()
        if self.node.get_parameter('use_sim_time').value:
            raise RuntimeError('Physical execution forbids simulation time')
        self.monitor()
        query_began = time.monotonic()
        responses = HardwareExecution._return_model_responses(self, arm)
        if responses is None and getattr(self, 'parallel_model_queries', False):
            responses = call_readonly_services(self.node, HardwareExecution._model_requests(self))
        hardware = (responses[0] if responses is not None else
                    self.node.call(self.hardware, ListHardwareComponents.Request()))
        if (len(hardware.component) != 1
                or hardware.component[0].class_type != 'cleany_feetech_hardware/FeetechSystem'
                or hardware.component[0].state.label != 'active'):
            raise RuntimeError('Expected one active Feetech hardware component')
        controllers = (responses[1] if responses is not None else
                       self.node.call(self.controllers, ListControllers.Request()))
        active = {c.name for c in controllers.controller if c.state == 'active'}
        if not {f'{arm}_arm_controller', f'{arm}_gripper_controller'} <= active:
            raise RuntimeError('Selected physical controllers are not active')
        physical = (responses[2] if responses is not None else self.node.call(
            self.parameters, GetParameters.Request(names=['robot_description']))).values
        if len(physical) != 1:
            raise RuntimeError('Physical robot description response is incomplete')
        description = physical[0].string_value
        planner = (responses[3] if responses is not None else self.node.call(self.moveit_parameters,
            GetParameters.Request(names=['robot_description', 'allow_trajectory_execution']))).values
        self._last_model_queries_sec = time.monotonic()-query_began
        if len(planner) != 2 or planner[1].bool_value:
            raise RuntimeError('MoveIt must be configured for planning only')
        signature = (description, planner[0].string_value)
        preparation = getattr(self, '_return_preparation', None)
        if (preparation is not None and preparation.endpoint is not None
                and preparation.endpoint.models != signature):
            raise RuntimeError('Robot model changed after release retreat; fresh retreat verification required')
        self._last_model_cache_hit = signature == getattr(self, '_verified_models', None)
        if not self._last_model_cache_hit:
            models = [ET.fromstring(xml) for xml in signature]
            for model in models:
                for system in model.findall('ros2_control'):
                    model.remove(system)
            if ET.tostring(models[0]) != ET.tostring(models[1]):
                raise RuntimeError('Physical and planning robot models differ')
            self._verified_models = signature
            self._physical_limits = {}
        if arm not in self._physical_limits:
            self._physical_limits[arm] = physical_limits(description, arm)
        limits = self._physical_limits[arm]
        self.node.description = description
        self.node.motion.set_robot_description(description)
        self._last_model_verification_sec = time.monotonic()-began
        return limits

    def fail_stop(self):
        """Request stop, then report confirmation separately from request acceptance."""
        HardwareExecution._reset_stability_history(self)
        if (self.stop_report is not None
                and (self.stop_report.get('requested')
                     or self.stop_report.get('controllers_stopped'))):
            return deepcopy(self.stop_report)
        self.fault = self.fault or 'Physical execution interrupted'
        self.arm_holds.clear()
        report = {'requested': False, 'controllers_stopped': False, 'physical_stop_verified': False}
        try:
            if not self.stop.wait_for_service(timeout_sec=1.):
                raise RuntimeError('Stop service unavailable')
            future = self.stop.call_async(Trigger.Request())
            deadline = time.monotonic()+3.
            while not future.done() and time.monotonic()<deadline:
                time.sleep(.01)
            if not future.done():
                raise RuntimeError('Stop service timeout')
            report['requested'] = bool(future.result().success)
            deadline = time.monotonic()+2.
            while time.monotonic() < deadline:
                if self.guard and self.guard.get('controllers_stopped'):
                    report['controllers_stopped'] = True
                    break
                time.sleep(.02)
        except Exception as error:
            report['error'] = str(error)
        self.stop_report = report
        return report

    def stop_after_execution_error(self, error: BaseException) -> None:
        """Let a tracked BT cancel/assess scene failures before latching stop."""
        if isinstance(error, (ExecutionCanceled, CommunicationRecovered)):
            return
        assess = getattr(self.node, 'defer_stop_assessment', None)
        try:
            if assess is not None and assess(error):
                return
        except Exception as assessment_error:
            self.node.get_logger().error(f'Could not request BT stop assessment: {assessment_error}')
        self.fail_stop()

    def on_contact_feedback(self, message):
        with self.contact_lock:
            if (len(message.joint_names) != len(message.interface_values)
                    or len(set(message.joint_names)) != len(message.joint_names)):
                HardwareExecution._reset_stability_history(self)
                for history in self.contact_histories.values():
                    history.invalidate('malformed_joint_feedback')
                return
            rows = dict(zip(message.joint_names, message.interface_values))
            recovery_counts = []
            feedback_stamps = {}
            for name, row in rows.items():
                if len(row.interface_names) != len(row.values):
                    continue
                values = dict(zip(row.interface_names, row.values))
                count = values.get('communication_recovery_count')
                if count is not None:
                    if not math.isfinite(count) or count < 0 or count != int(count):
                        self.fault = 'Invalid communication recovery telemetry'
                        return
                    recovery_counts.append(int(count))
                    stamp = values.get('feedback_stamp_ns')
                    if (stamp is None or not math.isfinite(stamp)
                            or stamp <= 0 or stamp != int(stamp)):
                        self.fault = 'Invalid complete-feedback acquisition telemetry'
                        return
                    feedback_stamps[name] = int(stamp)
            if recovery_counts:
                required = {name for names in ARM_JOINT_NAMES.values() for name in names}
                required.update(f'{arm}_gripper_joint' for arm in ARM_JOINT_NAMES)
                if (set(feedback_stamps) != required
                        or len(set(feedback_stamps.values())) != 1
                        or len(set(recovery_counts)) != 1):
                    self.fault = 'Incomplete or inconsistent communication recovery frame'
                    return
                self.complete_feedback_stamp_ns = next(iter(feedback_stamps.values()))
                self.communication_recovery_count = max(
                    getattr(self, 'communication_recovery_count', 0), max(recovery_counts))
                stability = getattr(self, '_stability_history', None)
                if stability is not None:
                    positions = {name: dict(zip(rows[name].interface_names,
                        rows[name].values)).get('position', math.nan) for name in required}
                    stability.observe(self.complete_feedback_stamp_ns, positions,
                                      max(recovery_counts))
            else:
                HardwareExecution._reset_stability_history(self)
            for arm, history in self.contact_histories.items():
                row = rows.get(f'{arm}_gripper_joint')
                if (row is None or len(row.interface_names) != len(row.values)
                        or len(set(row.interface_names)) != len(row.interface_names)):
                    history.invalidate('missing_or_malformed_gripper_feedback')
                else:
                    history.observe(dict(zip(row.interface_names, row.values)))

    def contact_decision(self, arm, *, after_ns=0):
        with self.contact_lock:
            return self.contact_histories[arm].evaluate(
                now_ns=self.node.get_clock().now().nanoseconds, after_ns=after_ns)

    def closed_contact_position(self, arm, limits=None):
        """Propose the calibrated close goal; dispatch rechecks it against fresh limits."""
        if limits is None:
            limits = getattr(self, '_physical_limits', {}).get(arm)
            if limits is None:
                limits = self.verify(arm)
        limit = limits[f'{arm}_gripper_joint']
        goal = self.contact_criteria[arm].closed_goal_raw
        value = limit.offset + limit.direction * (goal-limit.zero) * math.tau/4096
        limit.check(value)
        return value

    @timed_operation('gripper_contact_wait', node_attribute='node')
    def wait_for_contact(self, arm, after_ns):
        deadline = time.monotonic()+self.value('hardware_gripper_contact_timeout_sec')
        while True:
            self.monitor()
            decision = self.contact_decision(arm, after_ns=after_ns)
            if decision.state == ContactState.CONTACT:
                return decision
            if decision.state == ContactState.EMPTY:
                raise RuntimeError('Gripper closed empty; grasp contact was not confirmed')
            if time.monotonic() >= deadline:
                raise RuntimeError('Grasp contact remained unknown: '+decision.reason)
            time.sleep(.01)

    def execute(
        self,
        arm,
        trajectory,
        controller,
        *,
        terminal_check=True,
        held_command=None,
        check_orientation=True,
        contact_close=False,
        check_collisions=True,
        allow_endpoint_error=False,
        command_path_validator=None,
        recorded_trajectory=None,
        recorded_window: tuple[int, int] | None = None,
        defer_height_shortfall: bool = False,
        minimum_duration_sec=1.,
        start_from_measured: bool = False,
        endpoint_pose: CartesianPose | None = None,
    ):
        if type(defer_height_shortfall) is not bool:
            raise ValueError('defer_height_shortfall must be boolean')
        height_options = {'defer_height_shortfall': True} if defer_height_shortfall else {}
        if recorded_window is not None:
            height_options['recorded_window'] = recorded_window
        if minimum_duration_sec != 1.:
            height_options['minimum_duration_sec'] = minimum_duration_sec
        if type(start_from_measured) is not bool:
            raise ValueError('start_from_measured must be boolean')
        if start_from_measured:
            height_options['start_from_measured'] = True
        if endpoint_pose is not None:
            height_options['endpoint_pose'] = endpoint_pose
        if controller == 'arm' and check_collisions:
            with self.frozen_sensor_scene():
                return self._execute_with_recovery(
                    arm,
                    trajectory,
                    controller,
                    terminal_check=terminal_check,
                    held_command=held_command,
                    check_orientation=check_orientation,
                    contact_close=contact_close,
                    check_collisions=check_collisions,
                    allow_endpoint_error=allow_endpoint_error,
                    command_path_validator=command_path_validator,
                    recorded_trajectory=recorded_trajectory,
                    **height_options,
                )
        return self._execute_with_recovery(
            arm,
            trajectory,
            controller,
            terminal_check=terminal_check,
            held_command=held_command,
            check_orientation=check_orientation,
            contact_close=contact_close,
            check_collisions=check_collisions,
            allow_endpoint_error=allow_endpoint_error,
            command_path_validator=command_path_validator,
            recorded_trajectory=recorded_trajectory,
            **height_options,
        )

    @contextmanager
    def record_arm_commands(self, arm: str):
        """Capture completed controller goals within one motion phase."""
        if getattr(self, '_arm_command_recording', None) is not None:
            raise RuntimeError('Arm command recording is already active')
        recording = ArmCommandRecording(arm)
        self._arm_command_recording = recording
        try:
            yield recording
        finally:
            self._arm_command_recording = None

    def execute_recorded_trajectory(self, arm: str, recorded: RecordedTrajectory, *,
                                   check_collisions: bool = True,
                                   defer_height_shortfall: bool = False,
                                   recorded_window: tuple[int, int] | None = None,
                                   endpoint_pose: CartesianPose | None = None,
                                   check_arrival: bool = True):
        if type(check_arrival) is not bool:
            raise ValueError('check_arrival must be boolean')
        if not check_arrival and (endpoint_pose is not None or defer_height_shortfall):
            raise ValueError('Unchecked arrival cannot request endpoint verification')
        recorded.validate()
        trajectory = JointTrajectory(joint_names=list(recorded.joint_names), points=[
            JointTrajectoryPoint(positions=list(point.positions)) for point in recorded.points])
        if type(defer_height_shortfall) is not bool:
            raise ValueError('defer_height_shortfall must be boolean')
        options = {'defer_height_shortfall': True} if defer_height_shortfall else {}
        if recorded_window is not None:
            options['recorded_window'] = recorded_window
        if not check_arrival:
            options.update(terminal_check=False, allow_endpoint_error=True)
        if endpoint_pose is not None:
            options['endpoint_pose'] = endpoint_pose
        return self.execute(arm, trajectory, 'arm', recorded_trajectory=recorded,
                            held_command=recorded.points[0].positions,
                            check_collisions=check_collisions, **options)

    def execute_return_segment(self, arm: str, path: RecordedTrajectory, start_ns: int, end_ns: int, *,
                               check_collisions: bool = True,
                               endpoint_pose: CartesianPose | None = None,
                               check_arrival: bool = True):
        """Execute one bounded return window, optionally omitting arrival acceptance."""
        if type(check_arrival) is not bool:
            raise ValueError('check_arrival must be boolean')
        return self.execute_recorded_trajectory(arm, path, check_collisions=check_collisions,
            recorded_window=(start_ns, end_ns),
            endpoint_pose=endpoint_pose, **({'check_arrival': False} if not check_arrival else {}))

    def _execute_with_recovery(self, arm, trajectory, controller, **options):
        working = trajectory
        resumes = []
        limit = self.positive_integer('hardware_communication_resume_limit')
        for attempt in range(limit+1):
            try:
                actual = self._execute(arm, working, controller, **options)
                if resumes:
                    self.last_execution_metrics['communication_resumes'] = resumes
                return actual
            except (CollisionValidationError, JointRangeValidationError, CommandPathValidationError) as error:
                # A communication resume can reject its remaining path after
                # the original trajectory already moved. Do not start detours.
                error.motion_started = bool(resumes)
                raise
            except CommunicationRecovered as error:
                if options.get('recorded_trajectory') is not None:
                    self.fail_stop()
                    raise RuntimeError('Interrupted recorded retreat cannot resume its exact path') from error
                if not error.cancellation_confirmed or attempt >= limit:
                    self.fail_stop()
                    raise RuntimeError('Communication recovery could not confirm stop or resume limit exceeded') from error
                self._active_recovery_generation = None
                state = self.wait_for_settle(tuple(working.joint_names))
                q = dict(zip(state.name, state.position))
                points = remaining_joint_path([p.positions for p in working.points],
                    tuple(q[name] for name in working.joint_names),
                    maximum_delta=self.value('hardware_start_blend_max_rad'))
                working = deepcopy(working)
                working.points = [JointTrajectoryPoint(positions=list(point)) for point in points]
                options['held_command'] = None
                self.arm_holds.pop(arm, None)
                resumes.append({'generation': error.count, 'remaining_waypoints': len(points)})
                self.node.get_logger().warning(
                    f'Communication recovered: rebuilding {controller} remaining path from fresh encoders')

    def _bridge_start(self, names, positions, baseline):
        actual = tuple(float(baseline[name]) for name in names)
        points = bridge_joint_path(positions, actual, self.value('hardware_start_blend_max_rad'))
        if max(abs(a-b) for a, b in zip(actual, positions[0], strict=True)) > self.value(
                'hardware_start_tolerance_rad'):
            self.node.get_logger().info('Small start drift: connecting measured joints to the planned path')
        return points

    def _can_refresh_path(self, drift, commanded, attempt):
        return (attempt < self.positive_integer('hardware_path_refresh_attempts')
                and set(drift) <= set(commanded)
                and max(drift.values(), default=0.) <= self.value('hardware_start_blend_max_rad'))

    def opening_position_range_exemptions(self, arm, names, positions, limits, *,
                                         contact_close=False, opening_direction=None):
        """Exempt only monotone jaw-opening samples; retain raw encoder checks."""
        joint = f'{arm}_gripper_joint'
        if (getattr(self, 'gripper_opening_position_range_check_enabled', True)
                or contact_close or joint not in names or len(positions) < 2):
            return ()
        if opening_direction is None:
            opening_direction = self.contact_criteria[arm].opening_direction * limits[joint].direction
        if opening_direction not in (-1, 1):
            raise ValueError('Invalid gripper opening direction')
        index = names.index(joint)
        values = [point[index] for point in positions]
        if (all(math.isfinite(value) for value in values)
                and opening_direction*(values[-1]-values[0]) > 0
                and all(opening_direction*(b-a) >= 0 for a, b in zip(values, values[1:]))):
            return (joint,)
        return ()

    @timed_operation('trajectory_execution', node_attribute='node')
    def _execute(
        self,
        arm,
        trajectory,
        controller,
        *,
        terminal_check=True,
        held_command=None,
        check_orientation=True,
        contact_close=False,
        check_collisions=True,
        allow_endpoint_error=False,
        command_path_validator=None,
        recorded_trajectory=None,
        recorded_window: tuple[int, int] | None = None,
        _refresh_attempt=0,
        defer_height_shortfall: bool = False,
        minimum_duration_sec=1.,
        start_from_measured: bool = False,
        endpoint_pose: CartesianPose | None = None,
    ):
        preparation_began = time.monotonic()
        if endpoint_pose is not None and (not isinstance(endpoint_pose, CartesianPose)
                                         or controller != 'arm' or not terminal_check):
            raise ValueError('Physical arm endpoint requires a CartesianPose and terminal verification')
        if type(start_from_measured) is not bool:
            raise ValueError('start_from_measured must be boolean')
        if start_from_measured and (controller != 'arm' or held_command is not None
                                    or recorded_trajectory is not None):
            raise ValueError('Measured arm start cannot use a held or recorded command')
        if type(defer_height_shortfall) is not bool:
            raise ValueError('defer_height_shortfall must be boolean')
        if defer_height_shortfall and controller != 'arm':
            raise ValueError('Height shortfall deferral requires an arm trajectory')
        if type(allow_endpoint_error) is not bool:
            raise ValueError('allow_endpoint_error must be boolean')
        if allow_endpoint_error and controller != 'arm':
            raise ValueError('Relaxed arrival is only allowed for an arm trajectory')
        if type(check_collisions) is not bool:
            raise ValueError('check_collisions must be boolean')
        limits = self.verify(arm)
        if recorded_window is not None:
            if controller != 'arm' or recorded_trajectory is None:
                raise ValueError('A recorded window requires an arm command history')
            start_ns, end_ns = recorded_window
            recorded_trajectory = stopped_recorded_window(
                recorded_trajectory, start_ns, end_ns, limits,
                velocity=self.value('hardware_velocity_rad_s'),
                acceleration=self.value('hardware_acceleration_rad_s2'))
            trajectory = JointTrajectory(joint_names=list(recorded_trajectory.joint_names), points=[
                JointTrajectoryPoint(positions=list(point.positions))
                for point in recorded_trajectory.points])
            held_command = recorded_trajectory.points[0].positions
            self.node.event('return_replay_timing', source_duration_sec=(end_ns-start_ns)/1e9,
                            command_duration_sec=recorded_trajectory.points[-1].time_ns/1e9,
                            command_points=len(recorded_trajectory.points))
        requested_held_command = held_command
        expected = ARM_JOINT_NAMES[arm] if controller == 'arm' else (f'{arm}_gripper_joint',)
        if controller not in ('arm', 'gripper') or tuple(trajectory.joint_names) != expected:
            raise RuntimeError('Wrong trajectory joint/controller mapping')
        # A completed gripper/arm goal can still settle by a few encoder ticks.
        # Capture the execution baseline after that settling period, not immediately
        # after the preceding action result arrives.
        measured = HardwareExecution._wait_before_execution(self, expected)
        path_preparation_began = time.monotonic()
        baseline = dict(zip(measured.name, measured.position))
        positions = [tuple(p.positions) for p in trajectory.points]
        if len(positions) < 2 or any(len(p) != len(expected) for p in positions):
            raise RuntimeError('Malformed physical trajectory positions')
        nominal_target = positions[-1]
        if contact_close and (controller != 'gripper' or abs(
                nominal_target[0]-self.closed_contact_position(arm, limits)) > 1e-9):
            raise RuntimeError('Contact closing requires the calibrated final gripper goal')
        # Continue from the previous held command at normal action boundaries
        # so the controller does not unload the gravity-loaded gear train.
        if recorded_trajectory is not None:
            if (controller != 'arm' or recorded_trajectory.joint_names != expected
                    or tuple(positions) != tuple(p.positions for p in recorded_trajectory.points)):
                raise CommandPathValidationError('Recorded trajectory joint/controller mapping mismatch')
            # These are the already transformed commands. Preserve their exact
            # timestamps and velocities; do not add an old hold bias or simplify.
            replay_collision_samples = validate_recorded_trajectory(
                recorded_trajectory, limits,
                velocity=self.value('hardware_velocity_rad_s'),
                acceleration=self.value('hardware_acceleration_rad_s2'),
                step=self.value('hardware_path_step_rad'))
            samples = recorded_trajectory.command_samples()
            duration = recorded_trajectory.points[-1].time_ns/1e9
        elif (controller == 'arm' and held_command is None and arm in self.arm_holds
              and not start_from_measured):
            positions = HardwareExecution._bridge_start(self, expected, positions, baseline)
            hold = self.arm_holds[arm]
            positions = continue_joint_path(positions, hold, self.value(
                'hardware_command_continuity_max_offset_rad'))
            held_command = hold.command
        elif held_command is None:
            positions = HardwareExecution._bridge_start(self, expected, positions, baseline)
        if recorded_trajectory is None:
            range_exemptions = HardwareExecution.opening_position_range_exemptions(
                self, arm, expected, positions, limits, contact_close=contact_close)
            prepared = prepare_bounded_path(expected, positions, limits,
                velocity=self.value('hardware_velocity_rad_s'),
                acceleration=self.value('hardware_acceleration_rad_s2'),
                step=self.value('hardware_path_step_rad'),
                simplification_tolerance=self.value(
                    'hardware_path_simplification_tolerance_rad'),
                position_range_exemptions=range_exemptions,
                minimum_duration_sec=minimum_duration_sec)
            samples, duration = prepared.samples, prepared.duration
            if range_exemptions:
                self.node.event('gripper_opening_position_range_check_disabled', arm=arm,
                                joints=list(range_exemptions), raw_encoder_check_enabled=True)
        collision_samples = samples if recorded_trajectory is None else replay_collision_samples
        hold_bridge_samples = 0
        if held_command is None:
            if any(abs(baseline[n]-q) > self.value('hardware_start_tolerance_rad')
                   for n, q in zip(expected, samples[0].positions)):
                raise RuntimeError('Physical start changed; replan required')
        else:
            held_command = tuple(float(value) for value in held_command)
            if (controller != 'arm' or len(held_command) != len(expected)
                    or any(not math.isfinite(value) for value in held_command)
                    or any(abs(value-start) > 1e-9 for value, start in zip(
                        held_command, samples[0].positions, strict=True))):
                raise RuntimeError('Invalid held-command trajectory continuation')
            measured_start = tuple(float(baseline[name]) for name in expected)
            if any(abs(measured-command) > self.value(
                    'hardware_arrival_tolerance_rad') for measured, command in zip(
                        measured_start, held_command, strict=True)):
                raise RuntimeError(
                    'Physical hold departed too far for continuous path')
            # The controller must keep ``held_command`` as its first desired
            # point, but collision validation must also cover the small
            # gravity/backlash displacement from the measured robot state back
            # toward that still-active setpoint.
            if max(abs(measured-command) for measured, command in zip(
                    measured_start, held_command, strict=True)) > 1e-7:
                bridge, _ = bounded_path(
                    expected, [measured_start, held_command], limits,
                    velocity=self.value('hardware_velocity_rad_s'),
                    acceleration=self.value('hardware_acceleration_rad_s2'),
                    step=self.value('hardware_path_step_rad'),
                    simplification_tolerance=self.value(
                        'hardware_path_simplification_tolerance_rad'))
                collision_samples = [*bridge[:-1], *collision_samples]
                # Include the junction: samples[0] is the held setpoint and
                # remains mandatory for every later waypoint candidate.
                hold_bridge_samples = len(bridge)
        # Validate the exact samples that will be sent, after hold continuation
        # and retiming. A planner's unmodified knots cannot authorize this command.
        path_preparation_sec = time.monotonic()-path_preparation_began
        corridor_began = time.monotonic()
        if command_path_validator is not None:
            command_path_validator(samples, measured)
        corridor_sec = time.monotonic()-corridor_began
        # Full-robot collisions are checked with measured gripper, other arm,
        # attached objects, and the pre-motion sensor map. The normal hardware
        # profile keeps updates enabled; only the explicit legacy freeze mode
        # pauses the sensor map for the enclosing operation.
        collision_began = time.monotonic()
        if check_collisions:
            HardwareExecution.validate_collision_samples(
                self,
                arm,
                expected,
                collision_samples,
                baseline,
                measured,
                hold_bridge_samples=hold_bridge_samples,
            )
        else:
            self.node.get_logger().info(json.dumps({
                'stage': 'hardware_collision_validation_skipped',
                'arm': arm,
                'controller': controller,
                'command_samples': len(samples),
                'hold_bridge_samples': hold_bridge_samples,
            }))
        collision_sec = time.monotonic()-collision_began
        drift_began = time.monotonic()
        latest = self.monitor()
        drift = path_validation_drift(
            baseline, latest, expected,
            self.value('hardware_start_tolerance_rad'),
            self.value('hardware_uncommanded_tolerance_rad'))
        if drift:
            if HardwareExecution._can_refresh_path(self, drift, expected, _refresh_attempt):
                self.node.get_logger().info('Small drift during collision validation: refreshing the path once')
                return self._execute(arm, trajectory, controller, terminal_check=terminal_check,
                    held_command=requested_held_command, check_orientation=check_orientation,
                    contact_close=contact_close, check_collisions=check_collisions,
                    allow_endpoint_error=allow_endpoint_error,
                    command_path_validator=command_path_validator,
                    recorded_trajectory=recorded_trajectory,
                    _refresh_attempt=_refresh_attempt+1,
                    defer_height_shortfall=defer_height_shortfall,
                    minimum_duration_sec=minimum_duration_sec,
                    start_from_measured=start_from_measured,
                    endpoint_pose=endpoint_pose)
            details = ', '.join(f'{name}={delta:.6f}rad' for name, delta in sorted(drift.items()))
            raise RuntimeError('Robot moved during path validation: '+details)
        drift_sec = time.monotonic()-drift_began
        serialization_began = time.monotonic()
        command = JointTrajectory(joint_names=list(expected))
        # Humble JTC interpolates from *measured* state before a future-stamped
        # first point. Start now, with the held command at t=0, to avoid that
        # otherwise hidden reset even when the first point itself is correct.
        start_ns = 0
        # JTC's two endpoint Hermite cubic reproduces this one-segment jaw path.
        # Keep all samples above for physical bounds and collision validation.
        compact_gripper = (controller == 'gripper' and recorded_trajectory is None
                           and len(positions) == 2)
        compact_cubic = getattr(self, 'sparse_cubic_commands', False) and recorded_trajectory is None
        command_samples = (prepared.command_knots if compact_cubic else
                           [samples[0], samples[-1]] if compact_gripper else samples)
        for index, sample in enumerate(command_samples):
            point = JointTrajectoryPoint(positions=list(sample.positions),
                velocities=[v/duration for v in sample.derivatives])
            ns = round(sample.progress*duration*1e9)
            if recorded_trajectory is not None:
                saved = recorded_trajectory.points[index]
                ns, point.velocities = saved.time_ns, list(saved.velocities)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            command.points.append(point)
        self.node.get_logger().info(json.dumps({
            'stage': 'hardware_trajectory_prepared',
            'controller_points': len(command.points),
            'two_point_gripper_command': compact_gripper,
            'sparse_cubic_command': compact_cubic,
            'preparation_wall_sec': time.monotonic() - preparation_began,
            'path_preparation_wall_sec': path_preparation_sec,
            'command_corridor_wall_sec': corridor_sec,
            'collision_validation_wall_sec': collision_sec,
            'post_validation_drift_wall_sec': drift_sec,
            'command_serialization_wall_sec': time.monotonic()-serialization_began,
            'model_verification_wall_sec': getattr(self, '_last_model_verification_sec', None),
            'model_queries_wall_sec': getattr(self, '_last_model_queries_sec', None),
            'parallel_model_queries': getattr(self, 'parallel_model_queries', False),
            'model_cache_hit': getattr(self, '_last_model_cache_hit', False),
            'start_settle_wall_sec': getattr(self, '_last_start_settle_sec', None),
            'minimum_duration_sec': minimum_duration_sec if recorded_trajectory is None else None,
            'controller': controller,
            'input_waypoints': len(trajectory.points),
            'command_samples': len(samples),
            'duration_sec': duration,
            'start_stamp_ns': start_ns,
            'held_command_continuation': held_command is not None,
            'start_from_measured': start_from_measured,
            'collision_checks_enabled': check_collisions,
            'recorded_trajectory_replay': recorded_trajectory is not None,
            'nominal_endpoint_rad': list(nominal_target),
            'command_start_rad': list(samples[0].positions),
            'command_endpoint_rad': list(samples[-1].positions),
        }))
        client = HardwareExecution.motion_client(self, arm, controller)
        previous_uncommanded = self.uncommanded
        self.uncommanded = {n:q for n,q in baseline.items() if n not in expected}
        # Gripper resistance may be intentional contact; this motion watchdog
        # applies to arm joints, while gripper contact uses a separate evidence gate.
        motion_monitor = (HardwareExecution._start_motion_monitor(self, expected)
                          if controller == 'arm' else None)
        previous_generation = getattr(self, '_active_recovery_generation', None)
        self._active_recovery_generation = getattr(self, 'communication_recovery_count', 0)
        action_terminal = False
        submitted = False
        skip_arrival = controller == 'arm' and not terminal_check and allow_endpoint_error
        try:
            # Explicit trial policies may omit endpoint accuracy. Trajectory
            # validity, path limits, cancellation and faults still apply.
            action_options = {'monitor': True}
            if motion_monitor is not None:
                action_options['feedback_callback'] = lambda message: (
                    HardwareExecution._motion_feedback(motion_monitor, message))
            goal = FollowJointTrajectory.Goal(trajectory=command)
            if allow_endpoint_error:
                accuracy = {'velocity': -1., 'acceleration': -1.} if skip_arrival else {}
                goal.goal_tolerance = [
                    JointTolerance(name=name, position=-1., **accuracy) for name in expected]
            elif contact_close:
                # Intentional object contact prevents endpoint arrival. Only this
                # calibrated closing command delegates success to the raw gate.
                goal.goal_tolerance = [JointTolerance(name=expected[0], position=-1.)]
            self.motion_submissions = getattr(self, 'motion_submissions', 0) + 1
            HardwareExecution._reset_stability_history(self)
            submitted = True
            result = self.node.run_action(client, goal,
                                          duration+10., **action_options)
            action_terminal = True
            if motion_monitor is not None:
                motion_monitor.check(time.monotonic())
            motion_report = HardwareExecution._finish_motion_monitor(self, motion_monitor)
            motion_monitor = None
            if result.error_code != result.SUCCESSFUL:
                raise RuntimeError('Physical controller failure: '+result.error_string)
            actual = self.monitor()
            q = dict(zip(actual.name, actual.position))
            if any(name not in q or not math.isfinite(q[name]) for name in expected):
                raise RuntimeError('Physical arrival requires complete finite encoder feedback')
            arrival_errors = [abs(q[name]-target) for name, target in zip(expected, samples[-1].positions)]
            joint_arrival = (None if skip_arrival else all(
                error <= self.value('hardware_arrival_tolerance_rad') for error in arrival_errors))
            if not skip_arrival and not contact_close and not joint_arrival:
                if not allow_endpoint_error:
                    raise RuntimeError('Physical arrival not confirmed by encoders')
                self.node.get_logger().warning(
                    f'Endpoint accuracy checks disabled: encoder error {max(arrival_errors):.6f}rad')
            if controller == 'arm' and (terminal_check or allow_endpoint_error):
                self.arm_holds[arm] = ArmHold(nominal_target, tuple(samples[-1].positions))
            if controller == 'arm' and terminal_check:
                endpoint_options = {'allow_endpoint_error': True} if allow_endpoint_error else {}
                if defer_height_shortfall:
                    endpoint_options['defer_height_shortfall'] = True
                if endpoint_pose is not None:
                    endpoint_options['endpoint_pose'] = endpoint_pose
                actual, metrics = self.verify_arm_endpoint(
                    arm, nominal_target, limits, actual,
                    check_orientation=check_orientation, **endpoint_options)
                self.last_execution_metrics = metrics
            elif skip_arrival:
                self.last_execution_metrics = {}
            self.last_execution_metrics['joint_arrival_error_rad'] = arrival_errors
            self.last_execution_metrics['joint_arrival_within_tolerance'] = joint_arrival
            self.last_execution_metrics['endpoint_accuracy_checks_enabled'] = not allow_endpoint_error
            self.last_execution_metrics['motion_monitor'] = motion_report
            self.last_execution_metrics['collision_checks_enabled'] = check_collisions
            self.last_execution_metrics['start_from_measured'] = start_from_measured
            recording = getattr(self, '_arm_command_recording', None)
            if controller == 'arm' and recording is not None and recording.arm == arm:
                recording.trajectories.append(RecordedTrajectory(tuple(command.joint_names), tuple(
                    CommandPoint(p.time_from_start.sec*10**9+p.time_from_start.nanosec,
                                 tuple(p.positions), tuple(p.velocities)) for p in command.points)))
            return actual
        except BaseException as error:
            recording = getattr(self, '_arm_command_recording', None)
            if submitted and controller == 'arm' and recording is not None and recording.arm == arm:
                recording.interrupted = True
            if isinstance(error, CommunicationRecovered) and action_terminal:
                error.cancellation_confirmed = True
            HardwareExecution.stop_after_execution_error(self, error)
            raise
        finally:
            HardwareExecution._finish_motion_monitor(self, motion_monitor)
            self._active_recovery_generation = previous_generation
            self.uncommanded = previous_uncommanded
            client.destroy()

    def execute_pregrasp_parallel(
            self, arm, arm_trajectory, gripper_trajectory, minimum_gripper_opening,
            gripper_opening_direction, *, gripper_goal_required=False,
            stage_prefix='synchronized_pregrasp', scene_barrier=None, check_collisions=True):
        if type(check_collisions) is not bool:
            raise ValueError('check_collisions must be boolean')
        scene = (self.frozen_sensor_scene(
            **({'barrier': scene_barrier} if scene_barrier is not None else {}))
            if check_collisions else nullcontext())
        with scene:
            resumes = []
            options = ({'gripper_goal_required': True, 'stage_prefix': stage_prefix}
                       if gripper_goal_required else {})
            if not check_collisions:
                options['check_collisions'] = False
            limit = self.positive_integer('hardware_communication_resume_limit')
            for attempt in range(limit+1):
                try:
                    actual = self._execute_pregrasp_parallel(arm, arm_trajectory, gripper_trajectory,
                        minimum_gripper_opening, gripper_opening_direction, **options)
                    if resumes:
                        self.last_execution_metrics['communication_resumes'] = resumes
                    return actual
                except CollisionValidationError as error:
                    error.motion_started = bool(resumes)
                    raise
                except CommunicationRecovered as error:
                    if not error.cancellation_confirmed or attempt >= limit:
                        self.fail_stop()
                        raise RuntimeError('Synchronized communication recovery could not confirm stop or resume limit exceeded') from error
                    self._active_recovery_generation = None
                    names = (*arm_trajectory.joint_names, *gripper_trajectory.joint_names)
                    actual = self.wait_for_settle(names)
                    q = dict(zip(actual.name, actual.position))
                    points = remaining_joint_path([p.positions for p in arm_trajectory.points],
                        tuple(q[name] for name in arm_trajectory.joint_names),
                        maximum_delta=self.value('hardware_start_blend_max_rad'))
                    arm_trajectory = deepcopy(arm_trajectory)
                    arm_trajectory.points = [JointTrajectoryPoint(positions=list(p)) for p in points]
                    gripper_trajectory = deepcopy(gripper_trajectory)
                    gripper_trajectory.points[0].positions = [q[name] for name in gripper_trajectory.joint_names]
                    self.arm_holds.pop(arm, None)
                    resumes.append({'generation': error.count, 'remaining_waypoints': len(points)})
                    self.node.get_logger().warning('Communication recovered: rebuilding synchronized pregrasp from fresh encoders')

    def execute_arm_gripper(self, arm, arm_trajectory, gripper_trajectory, *, scene_barrier=None,
                            check_collisions=True):
        """Return an empty arm and close its jaw; joint limits always apply."""
        if not gripper_trajectory.points:
            raise ValueError('Missing synchronized gripper goal')
        target = float(gripper_trajectory.points[-1].positions[0])
        return self.execute_pregrasp_parallel(arm, arm_trajectory, gripper_trajectory,
            target, 1, gripper_goal_required=True, stage_prefix='synchronized_return',
            check_collisions=check_collisions,
            **({'scene_barrier': scene_barrier} if scene_barrier is not None else {}))

    @timed_operation('synchronized_execution', node_attribute='node')
    def _execute_pregrasp_parallel(
            self, arm, arm_trajectory, gripper_trajectory, minimum_gripper_opening,
            gripper_opening_direction, *, _refresh_attempt=0,
            gripper_goal_required=False, stage_prefix='synchronized_pregrasp',
            check_collisions=True):
        """Start arm and gripper on one timeline with explicit collision policy."""
        if type(check_collisions) is not bool:
            raise ValueError('check_collisions must be boolean')
        preparation_began = time.monotonic()
        self.last_execution_metrics = {}
        limits = self.verify(arm)
        arm_names = ARM_JOINT_NAMES[arm]
        gripper_name = f'{arm}_gripper_joint'
        if (tuple(arm_trajectory.joint_names) != arm_names
                or tuple(gripper_trajectory.joint_names) != (gripper_name,)
                or len(arm_trajectory.points) < 2
                or len(gripper_trajectory.points) != 2):
            raise RuntimeError('Wrong synchronized pregrasp trajectory mapping')
        measured = HardwareExecution._wait_before_execution(self, (*arm_names, gripper_name))
        path_preparation_began = time.monotonic()
        baseline = dict(zip(measured.name, measured.position))
        combined_names = (*arm_names, gripper_name)
        gripper_start = float(gripper_trajectory.points[0].positions[0])
        gripper_end = float(gripper_trajectory.points[-1].positions[0])
        simplification_tolerance = self.value(
            'hardware_path_simplification_tolerance_rad')
        arm_positions = [tuple(point.positions) for point in arm_trajectory.points]
        nominal_target = arm_positions[-1]
        arm_positions = HardwareExecution._bridge_start(self, arm_names, arm_positions, baseline)
        bridge_joint_path([(gripper_start,), (gripper_end,)], (baseline[gripper_name],),
                          self.value('hardware_start_blend_max_rad'))
        gripper_start = float(baseline[gripper_name])
        hold = self.arm_holds.get(arm)
        if hold is not None:
            if any(abs(baseline[n]-q) > self.value('hardware_arrival_tolerance_rad')
                   for n, q in zip(arm_names, hold.command)):
                raise RuntimeError('Physical hold departed too far for continuous pregrasp')
            arm_positions = continue_joint_path(arm_positions, hold, self.value(
                'hardware_command_continuity_max_offset_rad'))
        arm_path = executable_joint_path(
            arm_positions,
            simplification_tolerance=simplification_tolerance)
        combined_positions = []
        for progress, positions in zip(
                arm_path.progressions, arm_path.positions, strict=True):
            # Rest-to-rest cubic used by JTC for the two-point gripper path.
            blend = progress * progress * (3.0 - 2.0 * progress)
            gripper = gripper_start + (gripper_end - gripper_start) * blend
            combined_positions.append((*tuple(positions), gripper))
        range_exemptions = HardwareExecution.opening_position_range_exemptions(
            self, arm, combined_names, combined_positions, limits,
            opening_direction=gripper_opening_direction)
        prepared = prepare_bounded_path(
            combined_names, combined_positions, limits,
            velocity=self.value('hardware_velocity_rad_s'),
            acceleration=self.value('hardware_acceleration_rad_s2'),
            step=self.value('hardware_path_step_rad'),
            simplification_tolerance=simplification_tolerance,
            position_range_exemptions=range_exemptions)
        samples, duration = prepared.samples, prepared.duration
        if range_exemptions:
            self.node.event('gripper_opening_position_range_check_disabled', arm=arm,
                            joints=list(range_exemptions), raw_encoder_check_enabled=True)
        collision_samples = samples
        hold_bridge_samples = 0
        if hold is not None:
            bridge, _ = bounded_path(
                combined_names,
                [tuple(baseline[n] for n in combined_names), samples[0].positions], limits,
                velocity=self.value('hardware_velocity_rad_s'),
                acceleration=self.value('hardware_acceleration_rad_s2'),
                step=self.value('hardware_path_step_rad'),
                simplification_tolerance=simplification_tolerance,
                position_range_exemptions=range_exemptions)
            hold_bridge_samples = len(bridge)-1
            collision_samples = [*bridge[:-1], *samples]
        path_preparation_sec = time.monotonic()-path_preparation_began
        collision_began = time.monotonic()
        if check_collisions:
            HardwareExecution.validate_collision_samples(
                self, arm, combined_names, collision_samples, baseline, measured,
                hold_bridge_samples=hold_bridge_samples, trajectory_stage=stage_prefix)
        else:
            self.node.get_logger().info(json.dumps({
                'stage': 'hardware_collision_validation_skipped',
                'trajectory_stage': stage_prefix, 'arm': arm,
                'controller': 'arm_gripper', 'command_samples': len(samples),
                'hold_bridge_samples': hold_bridge_samples,
            }))
        collision_sec = time.monotonic()-collision_began
        drift_began = time.monotonic()
        latest = self.monitor()
        drift = path_validation_drift(
            baseline, latest, combined_names,
            self.value('hardware_start_tolerance_rad'),
            self.value('hardware_uncommanded_tolerance_rad'))
        if drift:
            if HardwareExecution._can_refresh_path(self, drift, combined_names, _refresh_attempt):
                self.node.get_logger().info('Small synchronized start drift: refreshing collision validation once')
                return self._execute_pregrasp_parallel(arm, arm_trajectory, gripper_trajectory,
                    minimum_gripper_opening, gripper_opening_direction,
                    _refresh_attempt=_refresh_attempt+1,
                    **({'check_collisions': False} if not check_collisions else {}),
                    **({'gripper_goal_required': True, 'stage_prefix': stage_prefix}
                       if gripper_goal_required else {}))
            details = ', '.join(f'{name}={delta:.6f}rad' for name, delta in sorted(drift.items()))
            raise RuntimeError('Robot moved during synchronized path validation: ' + details)

        drift_sec = time.monotonic()-drift_began
        serialization_began = time.monotonic()
        header_ns = self.node.get_clock().now().nanoseconds
        delay_parameter = ('hardware_return_start_delay_sec'
                           if stage_prefix == 'synchronized_return' else 'hardware_start_delay_sec')
        delay_ns = round(self.value(delay_parameter)*1e9)
        start_ns = header_ns+delay_ns
        compact_cubic = getattr(self, 'sparse_cubic_commands', False)
        command_samples = prepared.command_knots if compact_cubic else samples
        # Retain the common current epoch and explicit hold; both controllers
        # reconstruct exactly the same retimed curve used for dense validation.
        commands = {
            'arm': command_samples_to_message(arm_names, command_samples, duration,
                indices=range(len(arm_names)), header_ns=header_ns, delay_ns=delay_ns),
            'gripper': command_samples_to_message((gripper_name,), command_samples, duration,
                indices=(len(arm_names),), header_ns=header_ns, delay_ns=delay_ns),
        }
        self.node.get_logger().info(json.dumps({
            'stage': stage_prefix + '_trajectory_prepared',
            'controller_points': {key: len(value.points) for key, value in commands.items()},
            'sparse_cubic_command': compact_cubic,
            'collision_checks_enabled': check_collisions,
            'preparation_wall_sec': time.monotonic() - preparation_began,
            'path_preparation_wall_sec': path_preparation_sec,
            'collision_validation_wall_sec': collision_sec,
            'post_validation_drift_wall_sec': drift_sec,
            'command_serialization_wall_sec': time.monotonic()-serialization_began,
            'model_verification_wall_sec': getattr(self, '_last_model_verification_sec', None),
            'model_queries_wall_sec': getattr(self, '_last_model_queries_sec', None),
            'parallel_model_queries': getattr(self, 'parallel_model_queries', False),
            'model_cache_hit': getattr(self, '_last_model_cache_hit', False),
            'start_settle_wall_sec': getattr(self, '_last_start_settle_sec', None),
            'input_waypoints': len(arm_trajectory.points),
            'command_samples': len(samples),
            'duration_sec': duration,
            'start_stamp_ns': start_ns,
            'header_stamp_ns': header_ns,
            'scheduled_start_delay_sec': delay_ns / 1e9,
            'held_command_continuation': hold is not None,
            'nominal_endpoint_rad': nominal_target,
            'command_endpoint_rad': samples[-1].positions[:-1],
        }))

        commanded_gripper_opening = samples[-1].positions[-1]
        gripper_tolerance = (
            abs(commanded_gripper_opening - minimum_gripper_opening)
            + self.value('hardware_arrival_tolerance_rad'))

        clients_began = time.monotonic()
        clients = HardwareExecution.motion_clients(self, arm, ('arm', 'gripper'))
        client_acquisition_sec = time.monotonic()-clients_began
        handles = {}
        result_futures = {}
        tracking = {
            controller: {
                'samples': 0,
                'maximum_error_rad': 0.0,
                'maximum_error_rad_by_joint': {
                    name: 0.0 for name in commands[controller].joint_names
                },
            }
            for controller in clients
        }

        motion_monitor = None
        motion_report = {}

        def feedback(controller, message):
            if controller == 'arm':
                HardwareExecution._motion_feedback(motion_monitor, message)
            item = message.feedback
            if len(item.desired.positions) != len(item.actual.positions):
                return
            errors = [abs(desired-actual) for desired, actual in zip(
                item.desired.positions, item.actual.positions)]
            error = max(errors, default=0.0)
            tracking[controller]['samples'] += 1
            tracking[controller]['maximum_error_rad'] = max(
                tracking[controller]['maximum_error_rad'], error)
            for name, value in zip(commands[controller].joint_names, errors):
                current = tracking[controller]['maximum_error_rad_by_joint'][name]
                tracking[controller]['maximum_error_rad_by_joint'][name] = max(current, value)

        self.uncommanded = {name: value for name, value in baseline.items()
                            if name not in combined_names}
        previous_generation = getattr(self, '_active_recovery_generation', None)
        self._active_recovery_generation = getattr(self, 'communication_recovery_count', 0)
        try:
            for controller, client in clients.items():
                if not client.wait_for_server(timeout_sec=10.):
                    raise RuntimeError(f'{controller} action server unavailable')
            motion_monitor = HardwareExecution._start_motion_monitor(self, arm_names)
            # Dispatch both before waiting for either controller to finish.
            self.motion_submissions = getattr(self, 'motion_submissions', 0) + 1
            HardwareExecution._reset_stability_history(self)
            dispatch_began = time.monotonic()
            goal_futures = {
                controller: client.send_goal_async(FollowJointTrajectory.Goal(
                    trajectory=commands[controller],
                    goal_tolerance=([] if controller == 'arm' else [JointTolerance(
                        name=gripper_name, position=gripper_tolerance)])),
                    feedback_callback=lambda message, name=controller: feedback(name, message))
                for controller, client in clients.items()
            }
            for controller, future in goal_futures.items():
                handle = self.node.wait(future, 10.)
                if handle is None or not handle.accepted:
                    raise RuntimeError(f'{controller} synchronized goal rejected')
                handles[controller] = handle
                self.node.get_logger().info(json.dumps({
                    'stage': 'synchronized_goal_accepted',
                    'for_stage': stage_prefix,
                    'controller': controller,
                    'transport_reused': getattr(clients[controller], 'transport_reused', False),
                    'client_acquisition_wall_sec': client_acquisition_sec,
                    'dispatch_to_observed_accept_sec': time.monotonic()-dispatch_began,
                    'header_to_observed_accept_sec': (
                        self.node.get_clock().now().nanoseconds-header_ns)/1e9,
                    'scheduled_start_delay_sec': delay_ns/1e9,
                }))
            if self.node.get_clock().now().nanoseconds >= start_ns:
                raise RuntimeError('Both synchronized goals must be accepted before their start time')
            result_futures = {name: handle.get_result_async() for name, handle in handles.items()}
            deadline = time.monotonic() + duration + 10.
            completed = {}
            while len(completed) != len(result_futures):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Synchronized pregrasp execution timed out')
                for controller, future in result_futures.items():
                    if controller in completed or not future.done():
                        continue
                    wrapped = future.result()
                    result = wrapped.result
                    if (wrapped.status != GoalStatus.STATUS_SUCCEEDED
                            or result.error_code != result.SUCCESSFUL):
                        detail = result.error_string or str(wrapped.status)
                        raise RuntimeError(
                            f'{controller} synchronized controller failure: {detail}')
                    if controller == 'arm':
                        if motion_monitor is not None:
                            motion_monitor.check(time.monotonic())
                        motion_report = HardwareExecution._finish_motion_monitor(
                            self, motion_monitor)
                        motion_monitor = None
                    completed[controller] = wrapped
                if len(completed) == len(result_futures):
                    break
                self.monitor()
                time.sleep(.01)
            actual = self.monitor()
            q = dict(zip(actual.name, actual.position))
            arrival_tolerance = self.value('hardware_arrival_tolerance_rad')
            target_arm = tuple(float(value) for value in samples[-1].positions[:-1])
            actual_arm = tuple(float(q[name]) for name in arm_names)
            joint_errors = tuple(actual_value-target for actual_value, target in zip(
                actual_arm, target_arm))
            if any(abs(error) > arrival_tolerance for error in joint_errors):
                raise RuntimeError('Arm pregrasp arrival not confirmed by encoders')
            if gripper_opening_direction not in (-1, 1):
                raise RuntimeError('Invalid gripper opening direction')
            if gripper_goal_required and abs(q[gripper_name] - gripper_end) > arrival_tolerance:
                raise RuntimeError('Synchronized gripper goal not confirmed by encoder')
            if (not gripper_goal_required
                    and gripper_opening_direction * (q[gripper_name] - minimum_gripper_opening)
                    < -arrival_tolerance):
                raise RuntimeError('Minimum gripper opening not confirmed by encoder')
            self.arm_holds[arm] = ArmHold(nominal_target, target_arm)
            initial_endpoint = self.endpoint_metrics(
                arm, nominal_target, actual, correction_attempt=0,
                command=target_arm)
            actual, endpoint = self.verify_arm_endpoint(
                arm, nominal_target, limits, actual)
            metrics = {
                'stage': stage_prefix + '_tracking',
                'collision_checks_enabled': check_collisions,
                'controllers': tracking,
                'motion_monitor': motion_report,
                'duration_sec': duration,
                'command_samples': len(samples),
                'arm_joint_names': list(arm_names),
                'gripper_goal_required': gripper_goal_required,
                'gripper_goal_rad': gripper_end,
                'gripper_actual_rad': float(q[gripper_name]),
                'initial_endpoint': initial_endpoint,
                'uncommanded_joint_change_rad': {
                    name: float(q[name]-value)
                    for name, value in baseline.items()
                    if name not in combined_names
                },
                **endpoint,
            }
            metrics['maximum_final_arm_joint_error_rad'] = max(
                (abs(value) for value in endpoint['final_arm_joint_error_rad']),
                default=0.)
            self.last_execution_metrics = metrics
            self.node.get_logger().info(json.dumps(metrics))
            return actual
        except BaseException as interrupted:
            if isinstance(interrupted, CommunicationRecovered):
                self._active_recovery_generation = None
            cancellation_confirmed = True
            for controller, handle in handles.items():
                future = result_futures.get(controller)
                if future is None or not future.done():
                    try:
                        self.node.wait(handle.cancel_goal_async(), 5.)
                        if future is None:
                            future = handle.get_result_async()
                        wrapped = self.node.wait(future, 5., cancel_on_timeout=False)
                        if wrapped.status not in (GoalStatus.STATUS_SUCCEEDED,
                                GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                            raise RuntimeError('Controller did not reach a terminal action state')
                    except Exception as error:
                        cancellation_confirmed = False
                        self.node.get_logger().error(
                            f'Could not confirm {controller} goal cancellation: {error}')
            if isinstance(interrupted, CommunicationRecovered):
                interrupted.cancellation_confirmed = cancellation_confirmed and len(handles) == len(clients)
            HardwareExecution.stop_after_execution_error(self, interrupted)
            raise
        finally:
            HardwareExecution._finish_motion_monitor(self, motion_monitor)
            self._active_recovery_generation = previous_generation
            self.uncommanded = None
            for client in clients.values():
                client.destroy()
