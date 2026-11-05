"""End-to-end ROS graph: production supervision and mission nodes against a synthetic plant.

Real nodes: SupervisorNode, SafetyAdapter, BaseDriver + MockMcu, MissionRuntimeNode (Nav2
backend, supervised) and, when the navigation install base is sourced, LifecycleRecovery.
`SyntheticPlant` stands in for sensors, localization TF, the Nav2 action server, Nav2
lifecycle services and safety-hardware evidence. Its responses are synthetic: these tests
prove software contracts and latencies on this host, not physical stopping.
"""
import json
import os
from copy import deepcopy
from pathlib import Path
from statistics import median
from threading import Thread
from time import monotonic, sleep
from uuid import uuid4

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from cleany_base_driver.driver_node import BaseDriver
from cleany_base_driver.mock_node import MockMcu
from cleany_interfaces.action import RequestRecovery
from cleany_interfaces.msg import SafetyStatus, SystemEvent
from cleany_interfaces.srv import OfferMission, RequestFaultReset
from cleany_mission_manager.core.runtime_models import RuntimeState
from cleany_mission_manager.node import MissionRuntimeNode
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from cleany_safety_adapter.node import SafetyAdapter
from cleany_system_supervisor.config import load
from cleany_system_supervisor.models import Availability, OperatingState
from cleany_system_supervisor.node import SupervisorNode

try:
    from cleany_navigation.lifecycle_recovery_node import LifecycleRecovery
except ImportError:  # The navigation install base is built separately.
    LifecycleRecovery = None

PACKAGE = Path(__file__).resolve().parents[1]
MISSION = PACKAGE.parent / 'cleany_mission_manager'
LIFECYCLE_NODES = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator',
                   'collision_monitor')
OPERATOR = 'synthetic_operator'
OPERABLE = (OperatingState.OPERATIONAL, OperatingState.DEGRADED)


def wait(predicate, timeout=8., message='condition did not become true'):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        sleep(.005)
    assert predicate(), message() if callable(message) else message


class SyntheticPlant(Node):
    """Synthetic sensors, localization, Nav2 servers and safety-hardware evidence."""

    def __init__(self):
        super().__init__('synthetic_plant')
        self.group = ReentrantCallbackGroup()
        self.bad, self.hold, self.ignore_cancel = set(), False, False
        self.estop_released, self.protection_ready = True, True
        self.pose, self.moving = None, False
        self.goals, self.cancels, self.lifecycle_commands = [], [], []
        self.bad_published_at = {}
        self.pubs = {
            'scan': self.create_publisher(LaserScan, 'scan', 10),
            'pose': self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10),
            'odom': self.create_publisher(Odometry, 'odom', 10),
            'estop': self.create_publisher(Bool, 'plant/estop_released', 10),
            'protection': self.create_publisher(Bool, 'plant/protection_ready', 10),
        }
        self.tf = TransformBroadcaster(self)
        for name in LIFECYCLE_NODES:
            self.create_service(GetState, name + '/get_state', self._lifecycle, callback_group=self.group)
        for manager in ('lifecycle_manager_navigation', 'lifecycle_manager_localization'):
            self.create_service(ManageLifecycleNodes, manager + '/manage_nodes',
                                lambda request, response, manager=manager: self._manage(manager, request, response),
                                callback_group=self.group)
        self.nav = ActionServer(self, NavigateToPose, 'navigate_to_pose', self._navigate,
                                cancel_callback=self._cancel, callback_group=self.group)
        self.timer = self.create_timer(.02, self._publish, callback_group=self.group)

    def _lifecycle(self, request, response):
        response.current_state = (State(id=2, label='inactive') if 'navigation' in self.bad
                                  else State(id=3, label='active'))
        return response

    def _manage(self, manager, request, response):
        self.lifecycle_commands.append((manager, request.command))
        sleep(.2)  # A lifecycle transition takes time; the owner must not overlap recoveries.
        if request.command == ManageLifecycleNodes.Request.STARTUP and manager.endswith('navigation'):
            self.bad.discard('navigation')
        response.success = True
        return response

    def _cancel(self, _goal):
        self.cancels.append(monotonic())
        return CancelResponse.ACCEPT

    def _navigate(self, goal):
        self.goals.append(monotonic())
        self.moving, started = True, monotonic()
        while rclpy.ok():
            if goal.is_cancel_requested and not self.ignore_cancel:
                self.moving = False
                goal.canceled()
                return NavigateToPose.Result()
            if not self.hold and not goal.is_cancel_requested and monotonic() - started > .2:
                self.pose, self.moving = goal.request.pose.pose, False
                sleep(.05)  # Stopped feedback is published before the result.
                goal.succeed()
                return NavigateToPose.Result()
            sleep(.005)
        goal.abort()
        return NavigateToPose.Result()

    def _publish(self):
        stamp, now = self.get_clock().now().to_msg(), monotonic()
        for name in self.bad:
            self.bad_published_at.setdefault(name, now)
        scan = LaserScan(angle_increment=.1, range_min=.1, range_max=10.,
                         ranges=[float('nan') if 'lidar' in self.bad else 1.] * 10)
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        self.pubs['scan'].publish(scan)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp, pose.header.frame_id = stamp, 'map'
        pose.pose.pose.orientation.w = 1.
        self.pubs['pose'].publish(pose)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.twist.twist.linear.x = .2 if self.moving else 0.
        self.pubs['odom'].publish(odom)
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'map', 'base_link'
        transform.transform.rotation.w = 1.
        if self.pose is not None:
            transform.transform.translation.x = self.pose.position.x
            transform.transform.translation.y = self.pose.position.y
            transform.transform.rotation = self.pose.orientation
        self.tf.sendTransform(transform)
        self.pubs['estop'].publish(Bool(data=self.estop_released))
        self.pubs['protection'].publish(Bool(data=self.protection_ready))


def supervisor_config(tmp_path):
    config = load(PACKAGE / 'config/system_supervisor.yaml')
    config['persistence']['state_file'] = str(tmp_path / 'supervisor_state.json')
    config['ros']['trusted_publishers'] = {'safety/status': ['/safety_adapter'], 'system/component_health': []}
    return config


class Graph:
    def __init__(self, tmp_path, *, mcu=True, safety_evidence=True, runtime_parameters=()):
        self.tmp_path, self.nodes = tmp_path, []
        self.executor = MultiThreadedExecutor(num_threads=12)
        self.plant = self.add(SyntheticPlant())
        self.driver = self.add(BaseDriver(parameter_overrides=[Parameter(k, value=v) for k, v in {
            'geometry.wheel_radius_m': .05, 'geometry.wheelbase_m': .30, 'geometry.wheel_separation_m': .30,
            'limits.linear_x_mps': .4, 'limits.linear_y_mps': .4, 'limits.angular_z_rad_s': .5,
            'limits.wheel_rad_s': 10., 'limits.command_timeout_s': .4}.items()]))
        self.mcu = self.add(MockMcu()) if mcu else None
        evidence = {'estop_released_topic': 'plant/estop_released',
                    'protection_ready_topic': 'plant/protection_ready'} if safety_evidence else {}
        self.add(SafetyAdapter(parameter_overrides=[Parameter(k, value=v) for k, v in evidence.items()]))
        if LifecycleRecovery is not None:
            self.add(LifecycleRecovery())
        self.config = supervisor_config(tmp_path)
        self.supervisor = self.add(SupervisorNode(config=deepcopy(self.config)))
        self.events, self.diagnostics = [], []
        self.observer = self.add(rclpy.create_node('e2e_observer'))
        self.observer.create_subscription(SystemEvent, 'system/events', self.events.append, 100)
        self.observer.create_subscription(DiagnosticArray, 'diagnostics', self.diagnostics.append, 10)
        self.mission_executor, self.mission = SingleThreadedExecutor(), None
        parameters = [Parameter(k, value=v) for k, v in {
            'targets_file': str(MISSION / 'config/study_cafe_targets.yaml'),
            'journal_path': str(tmp_path / 'missions.db'), 'odom_topic': 'odom', 'use_sim_time': False,
            'post_mission': 'wait_for_next', 'navigation_timeout': 20.0, 'mock_operation_polls': 1,
            **dict(runtime_parameters)}.items()]
        self.threads = [Thread(target=self.executor.spin, daemon=True),
                        Thread(target=self._run_mission, args=(parameters,), daemon=True)]
        for thread in self.threads:
            thread.start()
        wait(lambda: self.mission is not None)

    def _run_mission(self, parameters):
        # The Runtime and its SQLite journal live on one thread, as with rclpy.spin in production.
        node = MissionRuntimeNode(parameter_overrides=parameters)
        self.mission_executor.add_node(node)
        self.mission = node
        try:
            self.mission_executor.spin()
        finally:
            node.destroy_node()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def remove(self, node):
        # Let in-flight callbacks of the removed node finish before its handles are destroyed.
        self.executor.remove_node(node)
        self.nodes.remove(node)
        sleep(.3)
        node.destroy_node()

    @property
    def core(self):
        return self.supervisor.core

    @property
    def runtime(self):
        return self.mission.runtime

    def restart_supervisor(self):
        self.remove(self.supervisor)
        self.supervisor = self.add(SupervisorNode(config=deepcopy(self.config)))

    def call(self, client, request, timeout=5.):
        assert client.wait_for_service(timeout_sec=timeout)
        future = client.call_async(request)
        wait(future.done, timeout)
        return future.result()

    def offer(self, mission_id=None):
        client = self.observer.create_client(OfferMission, 'mission/offer')
        return self.call(client, OfferMission.Request(
            mission_id=mission_id or str(uuid4()), mission_type='clean_desk', target_kind='SEAT',
            target_id='seat-12', requested_by=OPERATOR))

    def reset(self, code, operator=OPERATOR):
        client = self.observer.create_client(RequestFaultReset, 'system/request_fault_reset')
        return self.call(client, RequestFaultReset.Request(epoch=self.core.epoch, fault_code=code,
                                                           operator_id=operator))

    def shutdown(self):
        self.plant.timer.cancel()
        self.mission_executor.shutdown(timeout_sec=4.)
        self.executor.shutdown(timeout_sec=4.)
        for thread in self.threads:
            thread.join(4.)
        for node in self.nodes:
            node.destroy_node()


# A fresh DDS domain per test: participants of a finished graph can linger in discovery
# and would otherwise appear as untrusted publishers to the next graph.
_DOMAINS = iter(range(int(os.environ.get('SUPERVISOR_E2E_DOMAIN_BASE', '200')), 225))


@pytest.fixture
def graph(tmp_path):
    rclpy.init(domain_id=next(_DOMAINS))
    graphs = []

    def make(**kwargs):
        graphs.append(Graph(tmp_path, **kwargs))
        return graphs[-1]
    try:
        yield make
    finally:
        for created in graphs:
            created.shutdown()
        rclpy.shutdown()


def admissible(g, timeout=10.):
    wait(lambda: g.core.state in OPERABLE and g.runtime.readiness()[0], timeout,
         f'supervisor {g.core.state.value}, runtime {g.runtime.readiness()}: ' +
         str({c: f.detail for c, f in g.core.faults.faults.items() if f.active and f.locks_operation}))


def record(name, values):
    path = os.environ.get('SUPERVISOR_LATENCY_REPORT')
    if path:
        report = json.loads(Path(path).read_text()) if Path(path).exists() else {}
        report[name] = values
        Path(path).write_text(json.dumps(report, indent=2, sort_keys=True))
    print(name, json.dumps(values, sort_keys=True))


def test_boot_waits_for_mcu_link_and_runtime_admission_follows_the_supervisor(graph):
    g = graph(mcu=False)
    sleep(3.)
    assert 'BASE_UNAVAILABLE' not in g.core.faults.faults  # Waiting for the MCU is not a latch.
    assert g.core.capabilities.states['BASE_MOTION'] == Availability.UNKNOWN
    ready, reason = g.runtime.readiness()
    assert not ready and reason in ('SYSTEM_INITIALIZING', 'CAPABILITY_UNAVAILABLE:NAVIGATION')
    reply = g.offer()
    assert not reply.accepted and reply.reason == reason
    g.mcu = g.add(MockMcu())
    admissible(g)
    assert 'BASE_UNAVAILABLE' not in g.core.faults.faults
    wait(lambda: any(s.name == 'system_supervisor: operating state'
                     for msg in g.diagnostics for s in msg.status))
    wait(lambda: any(e.kind == 'STATE_CHANGED' for e in g.events))


def test_mcu_link_gaps_transient_blocks_motion_persistent_latches(graph):
    g = graph()
    admissible(g)
    transport = g.observer.create_client(SetBool, 'mock/transport')
    assert g.call(transport, SetBool.Request(data=False)).success
    wait(lambda: g.core.capabilities.states['BASE_MOTION'] != Availability.AVAILABLE, 2.)
    assert g.call(transport, SetBool.Request(data=True)).success
    admissible(g)
    assert 'BASE_UNAVAILABLE' not in g.core.faults.faults
    assert g.call(transport, SetBool.Request(data=False)).success
    wait(lambda: 'BASE_UNAVAILABLE' in g.core.faults.faults, 3.)
    assert g.call(transport, SetBool.Request(data=True)).success
    sleep(1.)
    assert g.core.state == OperatingState.FAULT_LOCKED  # Latched until an operator reset.
    assert not g.runtime.readiness()[0]
    response = g.reset('BASE_UNAVAILABLE')
    assert response.reset, response.reason
    admissible(g)


def test_cmd_vel_timeout_is_a_safe_disable_not_a_base_fault(graph):
    g = graph()
    admissible(g)
    enable = g.observer.create_client(SetBool, 'base/enable')
    assert g.call(enable, SetBool.Request(data=True)).success
    wait(lambda: g.driver.enabled, 2.)
    g.observer.create_publisher(Twist, 'cmd_vel', 10).publish(Twist())
    wait(lambda: g.driver.diagnostic_error == 'cmd_vel timeout', 3.)
    sleep(1.)  # The diagnostics level stays ERROR for the whole period.
    assert 'BASE_UNAVAILABLE' not in g.core.faults.faults
    assert g.core.capabilities.states['BASE_MOTION'] == Availability.AVAILABLE


def test_supervised_mission_completes_and_returns_its_lease(graph):
    g = graph()
    admissible(g)
    reply = g.offer()
    assert reply.accepted, reply.reason
    wait(lambda: g.runtime.last_report is not None, 15.)
    report = g.runtime.last_report
    assert report.outcome == 'SUCCESS' and g.runtime.supervision is not None
    wait(lambda: any(e.kind == 'LEASE_RELEASED' for e in g.events))
    assert not any(lease.granted for lease in g.core.permissions.leases.values())


def test_capability_loss_stops_the_nav2_mission_with_measured_latency(graph):
    g = graph()
    admissible(g)
    g.plant.hold = True
    trials = []
    for _ in range(5):
        goals, cancels = len(g.plant.goals), len(g.plant.cancels)
        mission_id = str(uuid4())
        assert g.offer(mission_id).accepted
        wait(lambda: len(g.plant.goals) > goals and g.plant.moving, 5.)
        lease = next(lease for lease in g.core.permissions.leases.values() if lease.granted)
        g.plant.bad_published_at.pop('lidar', None)
        g.plant.bad.add('lidar')
        wait(lambda: len(g.plant.cancels) > cancels, 3.)
        wait(lambda: lease.stopped, 8.)
        fault_at = g.plant.bad_published_at['lidar']
        wait(lambda: any(e.kind == 'MISSION_STOPPED' and e.code == lease.lease_id for e in g.events), 2.)
        events = [e for e in g.events if e.code == lease.lease_id]
        report = g.runtime.last_report
        assert report.request.mission_id == mission_id and report.failure_code == 'PERMISSION_REVOKED'
        assert g.runtime.state == RuntimeState.IDLE and not g.plant.moving
        trials.append({
            'revoke_s': next(e.monotonic_s for e in events if e.kind == 'LEASE_REVOKED') - fault_at,
            'nav2_cancel_s': g.plant.cancels[-1] - fault_at,
            'stop_confirmed_s': next(e.monotonic_s for e in events if e.kind == 'MISSION_STOPPED') - fault_at})
        g.plant.bad.discard('lidar')
        admissible(g)
    summary = {key: {'median': median(t[key] for t in trials), 'max': max(t[key] for t in trials)}
               for key in trials[0]}
    record('capability_loss_to_stop', {'trials': trials, 'summary': summary, 'runtime_tick_hz': 10.0})
    assert summary['revoke_s']['max'] < .5
    assert summary['nav2_cancel_s']['max'] < 1.
    assert summary['stop_confirmed_s']['max'] < 2.
    assert not any(f.active and f.code.startswith('MISSION_STOP_FAILED') for f in g.core.faults.faults.values())


def test_supervisor_crash_stops_the_mission_and_restart_reconfirms_the_old_lease(graph):
    g = graph()
    admissible(g)
    g.plant.hold = True
    assert g.offer().accepted
    wait(lambda: g.plant.moving, 5.)
    lease_id = next(lease.lease_id for lease in g.core.permissions.leases.values() if lease.granted)
    cancels, crashed_at = len(g.plant.cancels), monotonic()
    g.remove(g.supervisor)
    wait(lambda: len(g.plant.cancels) > cancels, 3.)
    latency = g.plant.cancels[-1] - crashed_at
    wait(lambda: g.runtime.state == RuntimeState.IDLE and g.runtime.last_report is not None, 6.)
    assert g.runtime.last_report.failure_code == 'PERMISSION_REVOKED'
    record('supervisor_crash_to_nav2_cancel', {'nav2_cancel_s': latency})
    assert latency < 1.7  # Heartbeat timeout 1.0 s plus one Runtime tick and DDS delivery.
    g.supervisor = g.add(SupervisorNode(config=deepcopy(g.config)))
    wait(lambda: lease_id in g.core.permissions.leases and g.core.permissions.leases[lease_id].stopped, 8.,
         lambda: f'restored lease {g.core.permissions.leases.get(lease_id)}, runtime {g.runtime.state.value}')
    assert g.core.permissions.leases[lease_id].restored
    admissible(g)


def test_stop_failure_escalates_and_faults_reset_by_their_owners(graph):
    g = graph(runtime_parameters={'cancel_timeout': 1.0})
    admissible(g)
    g.plant.hold, g.plant.ignore_cancel = True, True
    assert g.offer().accepted
    wait(lambda: g.plant.moving, 5.)
    lease_id = next(lease.lease_id for lease in g.core.permissions.leases.values() if lease.granted)
    g.plant.bad.add('lidar')
    code = 'MISSION_STOP_FAILED:' + lease_id
    wait(lambda: code in g.core.faults.faults and g.core.faults.faults[code].active, 6.)
    assert g.core.state == OperatingState.FAULT_LOCKED
    assert g.runtime.state == RuntimeState.ERROR  # The Runtime's own stop-deadline rule.
    g.plant.bad.discard('lidar')
    g.plant.ignore_cancel = False  # The base finally stops.
    response = g.reset(code)
    assert not response.reset and 'verification requested' in response.reason
    wait(lambda: not g.core.faults.faults[code].active, 8.)
    reset = g.observer.create_client(Trigger, 'mission/reset_error')
    wait(lambda: g.call(reset, Trigger.Request()).success, 5.)
    admissible(g)


@pytest.mark.skipif(LifecycleRecovery is None, reason='source the navigation install base')
def test_navigation_recovery_via_lifecycle_manager_and_unsupported_lidar(graph):
    g = graph()
    admissible(g)
    client = ActionClient(g.observer, RequestRecovery, 'system/request_recovery')
    assert client.wait_for_server(timeout_sec=5.)
    g.plant.bad.add('navigation')
    wait(lambda: g.core.faults.faults.get('NAVIGATION_UNAVAILABLE') is not None and
         g.core.faults.faults['NAVIGATION_UNAVAILABLE'].active, 4.)
    assert g.runtime.readiness()[1] == 'CAPABILITY_UNAVAILABLE:NAVIGATION'
    goals = [client.send_goal_async(RequestRecovery.Goal(epoch=g.core.epoch, fault_code='NAVIGATION_UNAVAILABLE'))
             for _ in range(2)]
    wait(lambda: all(goal.done() for goal in goals))
    results = [goal.result().get_result_async() for goal in goals]
    wait(lambda: all(result.done() for result in results), 20.)
    outcomes = [result.result().result for result in results]
    assert all(outcome.restored for outcome in outcomes), [o.detail for o in outcomes]
    assert outcomes[0].job_id == outcomes[1].job_id  # A duplicate request joins the same job.
    assert [command for _, command in g.plant.lifecycle_commands] == [
        ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP]
    admissible(g)
    g.plant.bad.add('lidar')
    wait(lambda: g.core.faults.faults.get('LIDAR_UNAVAILABLE') is not None, 3.)
    sent = client.send_goal_async(RequestRecovery.Goal(epoch=g.core.epoch, fault_code='LIDAR_UNAVAILABLE'))
    wait(sent.done)
    result = sent.result().get_result_async()
    wait(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.state == 'UNSUPPORTED'


def test_restart_keeps_estop_latch_until_operator_reset(graph):
    g = graph()
    admissible(g)
    g.plant.estop_released = False
    wait(lambda: g.core.faults.faults.get('ESTOP') is not None and g.core.faults.faults['ESTOP'].active, 3.)
    # Admission is a cached precheck (one snapshot behind); the lease is the authority.
    wait(lambda: g.runtime.readiness()[1] == 'SYSTEM_FAULT_LOCKED', 2.)
    assert g.offer().reason == 'SYSTEM_FAULT_LOCKED'
    g.plant.estop_released = True
    g.restart_supervisor()
    sleep(2.)
    assert g.core.faults.faults['ESTOP'].restored and g.core.state == OperatingState.FAULT_LOCKED
    missing = g.reset('ESTOP', operator='')
    assert not missing.reset and 'operator' in missing.reason
    accepted = g.reset('ESTOP')
    assert accepted.reset, accepted.reason
    admissible(g)


def test_untrusted_safety_publisher_locks_operation(graph):
    g = graph()
    admissible(g)
    rogue = g.add(rclpy.create_node('rogue_safety'))
    publisher = rogue.create_publisher(SafetyStatus, 'safety/status', 10)
    timer = rogue.create_timer(.05, lambda: publisher.publish(SafetyStatus(
        protection_ready=True, physical_release_confirmed=True, stopped=True)))
    wait(lambda: g.core.faults.faults.get('UNTRUSTED_PUBLISHER') is not None, 4.)
    assert g.core.state == OperatingState.FAULT_LOCKED
    assert '/rogue_safety' in g.core.faults.faults['UNTRUSTED_PUBLISHER'].detail
    wait(lambda: not g.runtime.readiness()[0])
    timer.cancel()


def test_missing_safety_hardware_evidence_locks_and_runtime_rejects(graph):
    g = graph(safety_evidence=False)
    sleep(6.)
    assert g.core.state == OperatingState.FAULT_LOCKED
    assert 'no independent protection evidence' in g.core.faults.faults['SAFETY_UNVERIFIED'].detail
    reply = g.offer()
    assert not reply.accepted and reply.reason == 'SYSTEM_FAULT_LOCKED'
    assert not g.plant.goals
