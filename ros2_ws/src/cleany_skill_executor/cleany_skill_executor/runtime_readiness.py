"""Check the collection graph and optionally finish deferred cold activation."""
from __future__ import annotations

from pathlib import Path
import argparse
from collections import Counter
import json
import math
import time

import rclpy
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from cleany_skill_executor.startup_cpu import StartupCpuWindow
from cleany_robot_interface.control_feedback_startup import (
    ControlFeedbackStartupProbe, activate_cold_controllers, require_stable_feedback,
)
from cleany_robot_interface.control_state_guard import DEFAULT_MAX_STATE_AGE_S


REQUIRED_SERVICES = (
    '/perception/observe_object_reference', '/perception/observe_wrist_target',
    '/grasp/plan', '/compute_ik', '/compute_fk', '/check_state_validity',
    '/get_planning_scene', '/perception/scene_cloud_updates_enabled',
    '/compute_cartesian_path', '/plan_kinematic_path',
    '/controller_manager/list_controllers', '/controller_manager/list_hardware_components',
    '/real_control/stop', '/grasp/warmup_status',
)
PLANNING_ACTIONS = (
    '/perception/inspect_scene', '/grasp/select_reachable', '/grasp/validate_pick',
    '/move_action',
)
REQUIRED_ACTIONS = (*PLANNING_ACTIONS, '/left_arm_controller/follow_joint_trajectory',
                    '/left_gripper_controller/follow_joint_trajectory')
FEEDBACK = {'/joint_states': JointState, '/camera/depth/image_raw': Image,
            '/perception/scene_cloud_receipt': Header}
FEEDBACK_TIMEOUTS = {'/joint_states': 15., '/camera/depth/image_raw': 20.,
                     '/perception/scene_cloud_receipt': 20.}


CONFIG_QUERY_TIMEOUT_S = 10.
READINESS_POLL_PERIOD_S = .05
PLANNING_STARTUP_STAGES = ('scene', 'grasp', 'validation')


def moveit_execution_disabled(node, ready_since):
    """Verify MoveIt's boolean reply, retrying an unanswered read once after discovery."""
    request = lambda: node.parameter_client.call_async(
        GetParameters.Request(names=['allow_trajectory_execution']))
    if node.parameter_future is None:
        if not node.parameter_client.service_is_ready():
            return False
        node.parameter_future = request()
    if (not node.parameter_future.done() and node.parameter_retry is None
            and ready_since is not None
            and time.monotonic() - ready_since >= CONFIG_QUERY_TIMEOUT_S / 2
            and node.parameter_client.service_is_ready()):
        node.parameter_retry = request()
    replies = [f for f in (node.parameter_future, node.parameter_retry)
               if f is not None and f.done()]
    for future in replies:
        result = future.result()
        if (result is None or len(result.values) != 1
                or result.values[0].type != ParameterType.PARAMETER_BOOL
                or result.values[0].bool_value):
            raise RuntimeError('MoveIt must have allow_trajectory_execution=false (boolean)')
    return bool(replies)


def graph_pending(nodes, services, action_servers, *, gui: bool,
                  required_actions=REQUIRED_ACTIONS) -> list[str]:
    """Action clients alone must never satisfy server readiness."""
    duplicates = [name for name, count in Counter(nodes).items() if count > 1]
    if duplicates:
        raise RuntimeError('Duplicate ROS node names: ' + ', '.join(sorted(duplicates)))
    pending = [name for name in REQUIRED_SERVICES if name not in services]
    for name in required_actions:
        count = action_servers.get(name, 0)
        if count > 1:
            raise RuntimeError(f'Expected exactly one action server for {name}: {count}')
        if count == 0:
            pending.append(name + ' (server)')
    for name in ('/perception_inspector', *(['/moveit_rviz'] if gui else [])):
        if name not in nodes:
            pending.append(name)
    return pending


class ControlStartupProbe(Node):
    """Wait for live guard feedback and a loaded planning scene before starting IK."""

    def __init__(self):
        super().__init__('collection_control_startup')
        self.guard_state = None
        self.guard_received = 0.
        self.create_subscription(String, '/real_control/status', self.receive_guard,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.scene_future = None
        self.scene_ready_at = None
        self.parameter_client = self.create_client(GetParameters, '/move_group/get_parameters')
        self.parameter_future = self.parameter_retry = None

    def receive_guard(self, message):
        self.guard_state = json.loads(message.data)['state']
        self.guard_received = time.monotonic()

    def pending(self):
        if self.guard_state not in (None, 'waiting', 'ready'):
            raise RuntimeError(f'Real-control guard is not ready: {self.guard_state}')
        if self.scene_future is None and self.scene.service_is_ready():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE
            self.scene_future = self.scene.call_async(request)
        if self.scene_future is None or not self.scene_future.done():
            return ['/get_planning_scene']
        result = self.scene_future.result()
        if result is None or not result.scene.robot_state.joint_state.name:
            raise RuntimeError('MoveIt returned an empty joint state')
        if self.scene_ready_at is None:
            self.scene_ready_at = time.monotonic()
        if not moveit_execution_disabled(self, self.scene_ready_at):
            if time.monotonic() - self.scene_ready_at >= CONFIG_QUERY_TIMEOUT_S:
                raise RuntimeError('Required configuration/status query timed out: MoveIt execution mode')
            return ['MoveIt execution mode']
        if self.guard_state != 'ready' or self.guard_received < self.scene_ready_at:
            return ['/real_control/status ready']
        return []


class PlanningInputsStartupProbe(Node):
    """Require fresh protected feedback and the current stage's planning inputs."""

    def __init__(self, stage):
        super().__init__(f'collection_planning_{stage}_startup')
        self.stage = stage
        self.validation = None
        if stage in ('control', 'validation'):
            from cleany_interfaces.action import ValidatePick
            from rclpy.action import ActionClient
            self.validation = ActionClient(self, ValidatePick, '/grasp/validate_pick')
        self.guard = None
        self.guard_ready_at = None
        self.guard_received = 0.
        self.inputs_ready_at = None
        self.received = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sensor_subscriptions = []
        for topic, kind in FEEDBACK.items():
            sub = self.create_subscription(kind, topic,
                lambda message, topic=topic: self.received.__setitem__(topic, time.monotonic()), qos,
                raw=(kind is Image))
            if topic != '/joint_states':
                self.sensor_subscriptions.append(sub)
        self.create_subscription(String, '/real_control/status', self._guard, qos)

    def _guard(self, message):
        try:
            self.guard = json.loads(message.data)['state']
        except (ValueError, KeyError, TypeError):
            self.guard = 'invalid'
        self.guard_received = time.monotonic()
        if self.guard != 'ready':
            self.guard_ready_at = None
            self.inputs_ready_at = None
        elif self.guard_ready_at is None:
            self.guard_ready_at = self.guard_received

    def pending(self):
        if self.guard not in (None, 'waiting', 'ready'):
            raise RuntimeError(f'Real-control guard is not ready: {self.guard}')
        if self.guard_ready_at is None:
            return ['/real_control/status ready']
        now = time.monotonic()
        topics = FEEDBACK_TIMEOUTS
        if self.stage == 'control':
            topics = {'/joint_states': FEEDBACK_TIMEOUTS['/joint_states']}
        elif self.stage == 'receipt':
            topics = {k: v for k, v in FEEDBACK_TIMEOUTS.items() if k != '/perception/scene_cloud_receipt'}
        pending = [topic for topic, timeout in topics.items()
                   if self.received.get(topic, -1.) < self.guard_ready_at
                   or now - self.received[topic] >= timeout]
        if self.stage == 'receipt' and not self.count_publishers('/perception/scene_cloud_receipt'):
            pending.append('/perception/scene_cloud_receipt (publisher)')
        if (self.stage in ('grasp', 'validation')
                and '/grasp/plan' not in {name for name, _ in self.get_service_names_and_types()}):
            pending.append('/grasp/plan')
        if self.stage == 'validation' and not self.validation.server_is_ready():
            pending.append('/grasp/validate_pick (server)')
        if pending:
            self.inputs_ready_at = None
            return pending
        if self.inputs_ready_at is None:
            self.inputs_ready_at = now
        if (self.guard_received < self.inputs_ready_at or self.received['/joint_states'] < self.inputs_ready_at
                or now - min(self.guard_received, self.received['/joint_states']) > DEFAULT_MAX_STATE_AGE_S):
            return ['Fresh controller feedback after planning inputs']
        return []


def warmup_ready(probe, now):
    """Poll the selector and retry status queries that never receive a response."""
    if not probe.warmup_ready:
        future = probe.warmup_future
        if future is not None and not future.done() and now >= probe.warmup_poll_after:
            probe.warmup_client.remove_pending_request(future)
            future.cancel()
            probe.warmup_future = None
        if (probe.warmup_future is None and now >= probe.warmup_poll_after
                and probe.warmup_client.service_is_ready()):
            probe.warmup_future = probe.warmup_client.call_async(Trigger.Request())
            probe.warmup_poll_after = now + 3.
        if probe.warmup_future is not None and probe.warmup_future.done():
            result = probe.warmup_future.result()
            probe.warmup_ready = result is not None and result.success
            if result is not None and result.message.startswith('IK warmup failed:'):
                raise RuntimeError(result.message)
            probe.warmup_future = None
            probe.warmup_poll_after = now + .5
    return probe.warmup_ready


class PlanningSequenceProbe(PlanningInputsStartupProbe):
    """Keep the same DDS participant while advancing the hardware launch one stage at a time."""

    def __init__(self):
        super().__init__('control')
        self.warmup_client = self.create_client(Trigger, '/grasp/warmup_status')
        self.warmup_future = None
        self.warmup_ready = False
        self.warmup_poll_after = 0.

    def pending(self):
        if self.stage != 'warmup':
            return super().pending()
        if self.guard not in (None, 'waiting', 'ready'):
            raise RuntimeError(f'Real-control guard is not ready: {self.guard}')
        now = time.monotonic()
        if self.guard != 'ready' or now - self.guard_received > DEFAULT_MAX_STATE_AGE_S:
            return ['/real_control/status ready']
        return [] if warmup_ready(self, now) else ['Primary GPU IK warmup']

    def advance(self):
        """Emit one line for the launch event handler after each verified stage."""
        transitions = {'control': 'receipt', 'receipt': 'scene', 'scene': 'grasp',
                       'grasp': 'validation', 'validation': 'warmup'}
        self.stage = transitions[self.stage]
        self.guard_ready_at = time.monotonic()
        self.inputs_ready_at = None
        if self.stage == 'warmup':
            for subscription in self.sensor_subscriptions:
                self.destroy_subscription(subscription)
        print('CLEANY_START_STAGE ' + self.stage, flush=True)


class ReadinessProbe(Node):
    def __init__(self, *, gui: bool, cold_controllers=()):
        super().__init__('collection_readiness')
        self.gui = gui
        self.cold_controllers = cold_controllers
        self.received = {}
        self.feedback_after = None
        self.graph_ready_at = None
        self.guard = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [self.create_subscription(kind, topic,
            lambda message, topic=topic: self.received.__setitem__(topic, time.monotonic()), qos)
            for topic, kind in FEEDBACK.items()]
        self.subscriptions_owned.append(self.create_subscription(
            String, '/real_control/status', self._guard, qos))
        self.parameter_client = self.create_client(GetParameters, '/move_group/get_parameters')
        self.controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.parameter_future = self.controllers_future = None
        self.controllers_retry = None
        self.controllers_query_at = 0.
        self.parameter_retry = None
        self.warmup_client = self.create_client(Trigger, '/grasp/warmup_status')
        self.warmup_future = None
        self.warmup_ready = False
        self.warmup_poll_after = 0.

    def _guard(self, message):
        try:
            self.guard = json.loads(message.data)['state']
        except (ValueError, KeyError, TypeError):
            self.guard = 'invalid'

    def pending(self) -> list[str]:
        if self.cold_controllers and self.guard not in (None, 'waiting', 'ready'):
            raise RuntimeError(f'Real-control guard is not ready: {self.guard}')
        nodes = self.get_node_names_and_namespaces()
        full_names = [namespace.rstrip('/') + '/' + name for name, namespace in nodes]
        servers = Counter()
        discovery_pending = []
        for name, namespace in set(nodes):
            try:
                actions = get_action_server_names_and_types_by_node(self, name, namespace)
            except _rclpy.RCLError as error:
                # The graph can change between listing nodes and querying their
                # actions. Retry within the main deadline; an incomplete scan
                # must never establish readiness, even for an unrelated node.
                full_name = namespace.rstrip('/') + '/' + name
                discovery_pending.append(f'Action server discovery for {full_name}: {error}')
                continue
            servers.update(action for action, _ in actions)
        required_actions = REQUIRED_ACTIONS
        if self.cold_controllers:
            required_actions = (*PLANNING_ACTIONS,
                                *(f'/{name}/follow_joint_trajectory' for name in self.cold_controllers))
        pending = graph_pending(full_names, {name for name, _ in self.get_service_names_and_types()},
                                servers, gui=self.gui, required_actions=required_actions)
        pending.extend(discovery_pending)
        now = time.monotonic()
        if pending:
            self.graph_ready_at = None
        elif self.graph_ready_at is None:
            self.graph_ready_at = now
        # Requests and subscriptions progress together, including during graph discovery.
        query_window_open = (self.graph_ready_at is None
                             or now - self.graph_ready_at < CONFIG_QUERY_TIMEOUT_S)
        retry_due = (self.controllers_future is not None and not self.controllers_future.done()
                     and self.controllers_retry is None
                     and now - self.controllers_query_at >= CONFIG_QUERY_TIMEOUT_S / 2)
        if ((self.controllers_future is None or retry_due) and query_window_open
                and self.controllers_client.service_is_ready()):
            query = self.controllers_client.call_async(ListControllers.Request())
            if retry_due:
                self.controllers_retry = query
            else:
                self.controllers_future = query
                self.controllers_query_at = now
        if not moveit_execution_disabled(self, self.graph_ready_at):
            pending.append('MoveIt execution mode')
        queries = (self.controllers_future, self.controllers_retry)
        replies = [future for future in queries if future is not None and future.done()]
        if not replies:
            pending.append('Controller state')
        else:
            result = replies[-1].result()
            state = 'inactive' if self.cold_controllers else 'active'
            matching = {c.name for c in result.controller if c.state == state} if result else set()
            required = set(self.cold_controllers or ('left_arm_controller', 'left_gripper_controller'))
            if not required <= matching:
                # State can change during discovery, so request a new snapshot.
                for future in queries:
                    if future is not None and not future.done():
                        future.cancel()
                self.controllers_future = self.controllers_retry = None
                pending.append(f'{state.capitalize()} arm/gripper controllers')
        if self.guard is None:
            pending.append('/real_control/status')
        elif self.guard == 'waiting':
            pending.append('/real_control/status ready')
        elif self.guard != 'ready':
            raise RuntimeError(f'Real-control guard is not ready: {self.guard}')
        if pending:
            self.feedback_after = None
            if self.graph_ready_at is not None and now - self.graph_ready_at >= CONFIG_QUERY_TIMEOUT_S:
                raise RuntimeError('Required configuration/status query timed out: ' + ', '.join(pending))
        # GPU initialization has its own configured budget, not the 10-second
        # configuration-query timeout. A solver allocation alone is not ready.
        if not warmup_ready(self, now):
            pending.append('Primary GPU IK warmup')
        if pending:
            self.feedback_after = None
        elif self.feedback_after is None:
            # Match the old post-graph topic checks, while waiting for all three
            # streams concurrently. Early startup messages cannot satisfy this.
            self.feedback_after = time.monotonic()
        for topic in FEEDBACK:
            if self.feedback_after is None or self.received.get(topic, -1.) < self.feedback_after:
                if self.feedback_after is not None and now - self.feedback_after >= FEEDBACK_TIMEOUTS[topic]:
                    raise RuntimeError('Required feedback unavailable: ' + topic)
                pending.append(topic)
        return pending


class ManipulationStartupProbe(ControlFeedbackStartupProbe):
    """Confirm a unique execute-skill server and stable feedback after it appears."""

    def __init__(self, startup_marker=None):
        super().__init__()
        self.startup_marker = startup_marker
        self.startup_cpu = None

    def pending(self):
        feedback_ready = self.ready()
        endpoints = self.get_publishers_info_by_topic('/manipulation/execute_skill/_action/status')
        if len(endpoints) > 1:
            raise RuntimeError('Multiple /manipulation/execute_skill servers')
        if self.startup_marker is not None:
            if endpoints:
                raise RuntimeError('Unexpected execute-skill server before managed startup')
            if not feedback_ready:
                self.startup_cpu = None
            else:
                if self.startup_cpu is None:
                    self.startup_cpu = StartupCpuWindow()
                if self.startup_cpu.ready():
                    self.startup_marker.touch(exist_ok=False)
                    self.startup_marker = None
                    self.stable_since = None
            return ['Stable protected feedback and CPU before executor startup']
        if not endpoints:
            self.stable_since = None
            return ['/manipulation/execute_skill (server)']
        owner = endpoints[0]
        try:
            actions = get_action_server_names_and_types_by_node(self, owner.node_name, owner.node_namespace)
        except _rclpy.RCLError:
            actions = []
        if not any(name == '/manipulation/execute_skill' and 'cleany_interfaces/action/ExecuteManipulationSkill' in types
                   for name, types in actions):
            self.stable_since = None
            return ['Action server discovery']
        return [] if feedback_ready else ['Stable protected feedback after executor startup']


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--gui', action='store_true')
    stages = parser.add_mutually_exclusive_group()
    parser.add_argument('--settle-cpu', action='store_true')
    stages.add_argument('--planning-sequence', action='store_true')
    parser.add_argument('--before-action-marker', type=Path)
    stages.add_argument('--action-startup', action='store_true')
    stages.add_argument('--control-startup', action='store_true')
    stages.add_argument('--activate-cold-controllers', nargs='+', metavar='NAME')
    stages.add_argument('--planning-inputs-startup', nargs='?', const='grasp',
                        choices=PLANNING_STARTUP_STAGES)
    options = parser.parse_args(args)
    if options.before_action_marker and not options.action_startup:
        parser.error('--before-action-marker requires --action-startup')
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    began = time.monotonic()
    rclpy.init(args=[])
    if options.planning_sequence:
        node, event = PlanningSequenceProbe(), 'planning_sequence_ready'
    elif options.action_startup:
        node, event = ManipulationStartupProbe(options.before_action_marker), 'manipulation_startup_ready'
    elif options.control_startup:
        node, event = ControlStartupProbe(), 'control_startup_ready'
    elif options.planning_inputs_startup:
        node = PlanningInputsStartupProbe(options.planning_inputs_startup)
        event = 'planning_inputs_startup_ready'
    else:
        node = ReadinessProbe(gui=options.gui, cold_controllers=options.activate_cold_controllers or ())
        event = 'collection_readiness'
    cpu_window = None
    last_log = began - 5.
    next_check = began
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0., next_check - time.monotonic()))
            now = time.monotonic()
            if now < next_check:
                continue
            pending = node.pending()
            checked_at = time.monotonic()
            # Keep joint callbacks responsive independently of expensive whole-graph scans.
            period = .5 if event == 'collection_readiness' else READINESS_POLL_PERIOD_S
            if options.action_startup:
                period = .2
            next_check = min(began + options.timeout, checked_at + period)
            elapsed = checked_at - began
            if options.settle_cpu:
                if pending:
                    cpu_window = None
                else:
                    if cpu_window is None:
                        cpu_window = StartupCpuWindow()
                    if not cpu_window.ready():
                        pending = [f'CPU <= {cpu_window.limit:g}% for two samples']
            if elapsed >= options.timeout:
                raise RuntimeError('Required readiness unavailable: ' + ', '.join(pending or [event]))
            if not pending and options.planning_sequence and node.stage != 'warmup':
                node.advance()
                cpu_window = None
                continue
            if not pending:
                if options.activate_cold_controllers:
                    print('Planning and GPU ready; checking feedback before torque activation', flush=True)
                    planning_probe = node
                    node = ControlFeedbackStartupProbe()
                    planning_probe.destroy_node()
                    deadline = began + options.timeout
                    activate_cold_controllers(node, options.activate_cold_controllers, deadline)
                    require_stable_feedback(node, deadline)
                result = {'event': event, 'wall_sec': time.monotonic() - began, 'status': 'ready'}
                if options.planning_inputs_startup:
                    result['stage'] = options.planning_inputs_startup
                print(json.dumps(result), flush=True)
                return 0
            if time.monotonic() - last_log >= 5.:
                print(f'Waiting for collection readiness ({elapsed:.1f}s/{options.timeout:g}s): '
                      + ', '.join(pending), flush=True)
                last_log = time.monotonic()
        return 1
    except Exception as error:
        print(f'Collection readiness failed: {error}', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
