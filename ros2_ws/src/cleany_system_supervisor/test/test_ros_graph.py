"""ROS graph evidence with synthetic peers; no hardware or navigation is driven."""
from copy import deepcopy
from pathlib import Path
from threading import Event, Thread
from time import monotonic, sleep

import pytest
import rclpy
from cleany_interfaces.action import RecoverComponent, RequestRecovery
from cleany_interfaces.msg import ComponentHealth, SafetyStatus, SystemState
from cleany_interfaces.srv import CheckPermission, RequestFaultReset
from cleany_mission_manager.core.manager import MissionManager
from cleany_mission_manager.core.models import MissionRequest
from cleany_mission_manager.core.result import ModuleResult
from cleany_mission_manager.core.states import MissionState
from cleany_mission_manager.mocks.components import (
    MockNavigator, MockPerception, MockPlanner, MockSkillExecutor, InMemoryReporter,
)
from cleany_mission_manager.ros_supervision import SupervisedMissionNode
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from cleany_system_supervisor.config import load
from cleany_system_supervisor.models import OperatingState, RecoveryState
from cleany_system_supervisor.node import SupervisorNode
from cleany_system_supervisor.sensors import StampGuard, valid_image, valid_pose, valid_scan


def wait(predicate, timeout=5.):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        sleep(.01)
    assert predicate(), 'condition did not become true'


class SyntheticPeers(Node):
    def __init__(self, config):
        super().__init__('synthetic_supervisor_peers')
        self.config, self.bad, self.messages = config, set(), []
        self.group = ReentrantCallbackGroup()
        settings = config['ros']
        self.pubs = {
            'scan': self.create_publisher(LaserScan, settings['scan_topic'], 10),
            'pose': self.create_publisher(PoseWithCovarianceStamped, settings['pose_topic'], 10),
            'camera': self.create_publisher(Image, settings['camera_topic'], 10),
            'base': self.create_publisher(DiagnosticArray, settings['diagnostics_topic'], 10),
            'owner': self.create_publisher(ComponentHealth, settings['component_health_topic'], 10),
            'safety': self.create_publisher(SafetyStatus, settings['safety_topic'], 10),
        }
        for name in settings['navigation_nodes'] + settings['localization_nodes']:
            self.create_service(GetState, name + '/get_state', self.state)
        self.create_subscription(SystemState, 'system/state', self.messages.append, 10)
        self.recoveries = []
        self.recovery = ActionServer(self, RecoverComponent, 'lidar/recover', self.recover,
                                     callback_group=self.group)
        self.timer = self.create_timer(.025, self.publish)

    @staticmethod
    def state(request, response):
        response.current_state = State(id=3, label='active')
        return response

    def recover(self, goal):
        self.recoveries.append(goal.request.job_id)
        goal.succeed()
        return RecoverComponent.Result(success=True, detail='synthetic action success only')

    def publish(self):
        stamp = self.get_clock().now().to_msg()

        def stamped(kind, **kwargs):
            msg = kind(**kwargs)
            msg.header.stamp, msg.header.frame_id = stamp, 'synthetic_frame'
            return msg
        scan = stamped(LaserScan, angle_increment=.1, range_min=.1, range_max=10.,
                       ranges=[float('nan') if 'lidar' in self.bad else 1.] * 10)
        self.pubs['scan'].publish(scan)
        pose = stamped(PoseWithCovarianceStamped)
        pose.pose.pose.orientation.w = 1.
        if 'localization' in self.bad:
            pose.pose.covariance[0] = 100.
        self.pubs['pose'].publish(pose)
        self.pubs['camera'].publish(stamped(Image, height=1, width=1, step=3,
                                            encoding='rgb8', data=[0, 0, 0]))
        status = DiagnosticStatus(name='base_driver', level=DiagnosticStatus.ERROR if 'base' in self.bad else DiagnosticStatus.OK,
                                  values=[KeyValue(key='feedback_age_s', value='0.01')])
        self.pubs['base'].publish(stamped(DiagnosticArray, status=[status]))
        for source in self.config['ros']['component_sources']:
            self.pubs['owner'].publish(stamped(ComponentHealth, source=source,
                status=ComponentHealth.UNAVAILABLE if source in self.bad else ComponentHealth.AVAILABLE,
                detail='synthetic semantic owner report'))
        self.pubs['safety'].publish(stamped(SafetyStatus,
            protection_ready='safety' not in self.bad, estop_engaged='estop' in self.bad,
            physical_release_confirmed='estop' not in self.bad, stopped=True))


@pytest.fixture
def graph():
    config = load(Path(__file__).resolve().parents[1] / 'config/system_supervisor.yaml')
    config['ros']['navigation_nodes'] = ['synthetic_nav']
    config['ros']['localization_nodes'] = ['synthetic_amcl']
    config['ros']['lifecycle_poll_s'] = .1
    config['lease_s'], config['heartbeat_timeout_s'], config['stop_timeout_s'] = .5, .3, .7
    for entry in config['sources'].values():
        entry['clear_s'], entry['raise_s'] = .08, 0.
    rclpy.init()
    peers, supervisor = SyntheticPeers(config), SupervisorNode(config=config)
    executor = MultiThreadedExecutor(num_threads=6)
    nodes = [peers, supervisor]
    for node in nodes:
        executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait(lambda: supervisor.core.state == OperatingState.OPERATIONAL)
        yield peers, supervisor, executor, nodes
    finally:
        peers.timer.cancel()
        executor.shutdown(timeout_sec=4.)
        thread.join(4.)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_ros_permission_service_and_recovery_verification(graph):
    peers, supervisor, _, _ = graph
    client = peers.create_client(CheckPermission, 'system/check_permission', callback_group=peers.group)
    wait(client.service_is_ready)
    future = client.call_async(CheckPermission.Request(epoch=supervisor.core.epoch,
        mission_id='synthetic_permission_only', capabilities=['UNKNOWN_CAPABILITY']))
    wait(future.done)
    assert not future.result().granted
    peers.bad.add('lidar')
    wait(lambda: 'LIDAR_UNAVAILABLE' in supervisor.core.faults.faults)
    request = ActionClient(peers, RequestRecovery, 'system/request_recovery', callback_group=peers.group)
    wait(request.server_is_ready)
    sent = request.send_goal_async(RequestRecovery.Goal(epoch=supervisor.core.epoch,
                                                       fault_code='LIDAR_UNAVAILABLE'))
    wait(sent.done)
    handle = sent.result()
    assert handle.accepted
    result = handle.get_result_async()
    wait(lambda: bool(peers.recoveries))
    job = next(iter(supervisor.core.recoveries.jobs.values()))
    wait(lambda: job.verify_after is not None)
    assert not result.done() and job.state == RecoveryState.RUNNING
    peers.bad.remove('lidar')
    wait(result.done)
    assert result.result().result.restored and job.state == RecoveryState.SUCCEEDED
    wait(lambda: supervisor.core.state == OperatingState.OPERATIONAL)
    assert peers.messages[-1].epoch == supervisor.core.epoch


class BlockingNavigator(MockNavigator):
    def __init__(self):
        self.entered, self.finish = Event(), Event()

    def navigate_to_target(self, request):
        self.entered.set()
        assert self.finish.wait(5.)
        return ModuleResult.success()


class SyntheticStop:
    def __init__(self, navigator):
        self.navigator, self.confirmed, self.requested = navigator, False, False

    def request_stop(self):
        self.requested = True
        self.navigator.finish.set()
        return True

    def stopped(self):
        return self.confirmed


def start_mission(graph):
    peers, supervisor, executor, nodes = graph
    navigator = BlockingNavigator()
    stopper = SyntheticStop(navigator)
    manager = MissionManager(navigator=navigator, perception=MockPerception(), planner=MockPlanner(),
                            skill_executor=MockSkillExecutor(), reporter=InMemoryReporter(), stop_controller=stopper)
    bridge = SupervisedMissionNode(manager, stop_timeout_s=.6)
    bridge.permission.heartbeat_timeout_s = .3
    executor.add_node(bridge)
    nodes.append(bridge)
    wait(lambda: bool(bridge.permission.epoch) and bridge.permission.client.service_is_ready())
    bridge.submit(MissionRequest('synthetic_mission', 'clean_seat', 'seat_01', 'synthetic_operator'))
    wait(navigator.entered.is_set)
    return bridge, manager, stopper


def test_ros_revocation_ack_and_actual_stop_are_distinct(graph):
    peers, supervisor, _, _ = graph
    bridge, manager, stopper = start_mission(graph)
    lease = next(iter(supervisor.core.permissions.leases.values()))
    peers.bad.add('localization')
    wait(lambda: lease.stop_accepted)
    assert stopper.requested and not lease.stopped and manager.state == MissionState.STOPPING
    stopper.confirmed = True
    wait(lambda: lease.stopped)
    assert manager.state == MissionState.INTERRUPTED
    peers.bad.remove('localization')
    wait(lambda: supervisor.core.state == OperatingState.OPERATIONAL)
    assert manager.state == MissionState.INTERRUPTED and not lease.granted


def test_ros_supervisor_restart_invalidates_active_permission(graph):
    peers, supervisor, executor, nodes = graph
    bridge, manager, stopper = start_mission(graph)
    previous_epoch = supervisor.core.epoch
    executor.remove_node(supervisor)
    nodes.remove(supervisor)
    supervisor.destroy_node()
    restarted = SupervisorNode(config=deepcopy(peers.config))
    executor.add_node(restarted)
    nodes.append(restarted)
    assert restarted.core.epoch != previous_epoch
    wait(lambda: stopper.requested)
    assert not bridge.permission.valid()
    stopper.confirmed = True
    wait(lambda: manager.state == MissionState.INTERRUPTED)
    assert not restarted.core.permissions.leases


def test_ros_supervisor_exit_expires_lease_without_a_revocation_message(graph):
    _, supervisor, executor, nodes = graph
    bridge, manager, stopper = start_mission(graph)
    # Exercise successful renewals for more than the original grant lifetime.
    sleep(.8)
    assert bridge.permission.valid() and not stopper.requested
    executor.remove_node(supervisor)
    nodes.remove(supervisor)
    supervisor.destroy_node()
    wait(lambda: stopper.requested, timeout=1.)
    assert not bridge.permission.valid()
    stopper.confirmed = True
    wait(lambda: manager.state == MissionState.INTERRUPTED)


def test_ros_stop_failure_escalates_without_fabricating_success(graph):
    peers, supervisor, _, _ = graph
    bridge, manager, stopper = start_mission(graph)
    peers.bad.add('localization')
    wait(lambda: any(f.code.startswith('MISSION_STOP_FAILED:') and f.active
                     for f in supervisor.core.faults.faults.values()))
    assert supervisor.core.state == OperatingState.FAULT_LOCKED
    assert not any(lease.stopped for lease in supervisor.core.permissions.leases.values())
    stopper.confirmed = True
    wait(lambda: manager.state == MissionState.INTERRUPTED)
    reset = peers.create_client(RequestFaultReset, 'system/request_fault_reset', callback_group=peers.group)
    wait(reset.service_is_ready)
    code = next(f.code for f in supervisor.core.faults.faults.values()
                if f.code.startswith('MISSION_STOP_FAILED:'))
    result = reset.call_async(RequestFaultReset.Request(epoch=supervisor.core.epoch, fault_code=code))
    wait(result.done)
    assert not result.result().reset  # Service dispatches verification, not a fabricated stop result.
    wait(lambda: all(lease.stopped for lease in supervisor.core.permissions.leases.values()))
    assert supervisor.core.state == OperatingState.DEGRADED


def test_sensor_validation_rejects_publish_only_health():
    scan = LaserScan(angle_increment=.1, range_min=.1, range_max=10., ranges=[float('nan')] * 10)
    scan.header.frame_id = 'laser'
    assert not valid_scan(scan, .1)
    scan.ranges = [float('inf')] * 10
    assert valid_scan(scan, .1)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id, pose.pose.pose.orientation.w = 'map', 1.
    assert valid_pose(pose, .5)
    pose.pose.covariance[0] = float('nan')
    assert not valid_pose(pose, .5)
    image = Image(width=1, height=1, step=3, encoding='rgb8', data=[1])
    image.header.frame_id = 'camera'
    assert not valid_image(image)
    guard = StampGuard(1.)
    scan.header.stamp.sec = 10
    assert guard.accept('scan', scan.header.stamp, 10 * 10**9)
    assert not guard.accept('scan', scan.header.stamp, 10 * 10**9)
    assert not guard.accept('scan', scan.header.stamp, 12 * 10**9)
