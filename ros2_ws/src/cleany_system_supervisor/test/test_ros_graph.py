"""Supervisor-only ROS graph with synthetic peers; mission integration is in test_e2e.py."""
import os
from pathlib import Path
from threading import Thread
from time import monotonic, sleep

import pytest
import rclpy
from cleany_interfaces.action import RecoverComponent, RequestRecovery
from cleany_interfaces.msg import ComponentHealth, SafetyStatus, SystemState
from cleany_interfaces.srv import CheckPermission
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
        for name in settings['navigation_nodes']:
            self.create_service(GetState, name + '/get_state', self.navigation_state)
        for name in settings['localization_nodes']:
            self.create_service(GetState, name + '/get_state', self.state)
        self.create_subscription(SystemState, 'system/state', self.messages.append, 10)
        self.recoveries = []
        self.recovery = ActionServer(self, RecoverComponent, 'navigation/recover', self.recover,
                                     callback_group=self.group)
        self.timer = self.create_timer(.025, self.publish)

    @staticmethod
    def state(request, response):
        response.current_state = State(id=3, label='active')
        return response

    def navigation_state(self, request, response):
        response.current_state = (State(id=2, label='inactive') if 'navigation' in self.bad
                                  else State(id=3, label='active'))
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


_DOMAINS = iter(range(int(os.environ.get('SUPERVISOR_GRAPH_DOMAIN_BASE', '190')), 200))


@pytest.fixture
def graph():
    config = load(Path(__file__).resolve().parents[1] / 'config/system_supervisor.yaml')
    config['ros']['navigation_nodes'] = ['synthetic_nav']
    config['ros']['localization_nodes'] = ['synthetic_amcl']
    config['ros']['lifecycle_poll_s'] = .1
    config['lease_s'], config['heartbeat_timeout_s'], config['stop_timeout_s'] = .5, .3, .7
    config['persistence']['state_file'] = ''  # Never touch the operator's real latch state.
    config['ros']['trusted_publishers'] = {topic: ['/synthetic_supervisor_peers']
                                           for topic in config['ros']['trusted_publishers']}
    for entry in config['sources'].values():
        entry['clear_s'], entry['raise_s'] = .08, 0.
    rclpy.init(domain_id=next(_DOMAINS))
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
    peers.bad.add('navigation')
    wait(lambda: 'NAVIGATION_UNAVAILABLE' in supervisor.core.faults.faults)
    request = ActionClient(peers, RequestRecovery, 'system/request_recovery', callback_group=peers.group)
    wait(request.server_is_ready)
    sent = request.send_goal_async(RequestRecovery.Goal(epoch=supervisor.core.epoch,
                                                       fault_code='NAVIGATION_UNAVAILABLE'))
    wait(sent.done)
    handle = sent.result()
    assert handle.accepted
    result = handle.get_result_async()
    wait(lambda: bool(peers.recoveries))
    job = next(iter(supervisor.core.recoveries.jobs.values()))
    wait(lambda: job.verify_after is not None)
    assert not result.done() and job.state == RecoveryState.RUNNING
    peers.bad.remove('navigation')
    wait(result.done)
    assert result.result().result.restored and job.state == RecoveryState.SUCCEEDED
    wait(lambda: supervisor.core.state == OperatingState.OPERATIONAL)
    assert peers.messages[-1].epoch == supervisor.core.epoch


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
