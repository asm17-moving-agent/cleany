"""Read-only collection preflight using one ROS participant and concurrent discovery."""
from __future__ import annotations

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


REQUIRED_SERVICES = (
    '/perception/observe_object_reference', '/perception/observe_wrist_target',
    '/grasp/plan', '/compute_ik', '/compute_fk', '/check_state_validity',
    '/get_planning_scene', '/perception/scene_cloud_updates_enabled',
    '/compute_cartesian_path', '/plan_kinematic_path',
    '/controller_manager/list_controllers', '/controller_manager/list_hardware_components',
    '/real_control/stop', '/grasp/warmup_status',
)
REQUIRED_ACTIONS = (
    '/perception/inspect_scene', '/grasp/select_reachable', '/grasp/validate_pick',
    '/move_action', '/left_arm_controller/follow_joint_trajectory',
    '/left_gripper_controller/follow_joint_trajectory',
)
FEEDBACK = {'/joint_states': JointState, '/camera/depth/image_raw': Image,
            '/perception/scene_cloud_receipt': Header}
FEEDBACK_TIMEOUTS = {'/joint_states': 15., '/camera/depth/image_raw': 20.,
                     '/perception/scene_cloud_receipt': 20.}


CONFIG_QUERY_TIMEOUT_S = 10.


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


def graph_pending(nodes, services, action_servers, *, gui: bool) -> list[str]:
    """Action clients alone must never satisfy server readiness."""
    duplicates = [name for name, count in Counter(nodes).items() if count > 1]
    if duplicates:
        raise RuntimeError('Duplicate ROS node names: ' + ', '.join(sorted(duplicates)))
    pending = [name for name in REQUIRED_SERVICES if name not in services]
    for name in REQUIRED_ACTIONS:
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


class ReadinessProbe(Node):
    def __init__(self, *, gui: bool):
        super().__init__('collection_readiness')
        self.gui = gui
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
        pending = graph_pending(full_names, {name for name, _ in self.get_service_names_and_types()},
                                servers, gui=self.gui)
        pending.extend(discovery_pending)
        now = time.monotonic()
        if pending:
            self.graph_ready_at = None
        elif self.graph_ready_at is None:
            self.graph_ready_at = now
        # Requests and subscriptions progress together, including during graph discovery.
        if self.controllers_future is None and self.controllers_client.service_is_ready():
            self.controllers_future = self.controllers_client.call_async(ListControllers.Request())
        if not moveit_execution_disabled(self, self.graph_ready_at):
            pending.append('MoveIt execution mode')
        if self.controllers_future is None or not self.controllers_future.done():
            pending.append('Controller state')
        else:
            result = self.controllers_future.result()
            active = {c.name for c in result.controller if c.state == 'active'} if result else set()
            required = {'left_arm_controller', 'left_gripper_controller'}
            if not required <= active:
                # Controllers can still be activating; never accept an old inactive response.
                self.controllers_future = None
                pending.append('Active arm/gripper controllers')
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
        if not self.warmup_ready:
            if (self.warmup_future is None and now >= self.warmup_poll_after
                    and self.warmup_client.service_is_ready()):
                self.warmup_future = self.warmup_client.call_async(Trigger.Request())
            if self.warmup_future is not None and self.warmup_future.done():
                result = self.warmup_future.result()
                self.warmup_ready = result is not None and result.success
                if result is not None and result.message.startswith('IK warmup failed:'):
                    raise RuntimeError(result.message)
                self.warmup_future = None
                self.warmup_poll_after = now + .5
            if not self.warmup_ready:
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


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--control-startup', action='store_true')
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    began = time.monotonic()
    rclpy.init(args=[])
    node = ControlStartupProbe() if options.control_startup else ReadinessProbe(gui=options.gui)
    last_log = began - 5.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            pending = node.pending()
            elapsed = time.monotonic() - began
            if not pending:
                event = 'control_startup_ready' if options.control_startup else 'collection_readiness'
                print(json.dumps({'event': event, 'wall_sec': elapsed,
                                  'status': 'ready'}), flush=True)
                return 0
            if elapsed >= options.timeout:
                raise RuntimeError('Required readiness unavailable: ' + ', '.join(pending))
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
