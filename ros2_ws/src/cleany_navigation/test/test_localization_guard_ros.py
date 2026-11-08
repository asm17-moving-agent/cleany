"""Isolated recovery with real guard/Supervisor and a synthetic stationary LiDAR plant."""
from copy import deepcopy
from pathlib import Path
from threading import Thread
from time import monotonic, sleep
import math

import numpy as np
import pytest
import rclpy
from cleany_interfaces.action import RecoverComponent, StopMission
from cleany_interfaces.srv import CheckPermission
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from cleany_navigation.localization_guard_node import LocalizationGuard
from cleany_navigation.localization_alignment import AlignmentMap
from cleany_system_supervisor.config import load
from cleany_system_supervisor.node import SupervisorNode
from test_localization_alignment import scene


def wait(predicate, timeout=16.):
    deadline = monotonic()+timeout
    while monotonic() < deadline:
        if predicate():return
        sleep(.02)
    assert predicate()


class Plant(Node):
    def __init__(self, ambiguous=False):
        super().__init__('synthetic_localization_plant')
        self.group = ReentrantCallbackGroup()
        self.model, points, self.truth = scene()
        if ambiguous:
            cells=np.zeros((240,240),dtype=int);cells[:,80]=cells[:,160]=100
            self.model=AlignmentMap(cells,.05,(0.,0.,0.));self.truth=np.array([6.,6.,0.])
            y=np.linspace(3.,9.,100)
            points=np.vstack((np.column_stack((np.full(100,4.025),y)),
                              np.column_stack((np.full(100,8.025),y))))-self.truth[:2]
        self.estimate = self.truth.copy()
        self.variance = .001
        self.stopped = True
        self.pose_updates, self.stop_requests = [], []
        self.speed_limits = []
        self.create_subscription(SpeedLimit,'navigation/localization_speed_limit',
                                 lambda m:self.speed_limits.append(m.speed_limit),10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose, 10)
        for name in ('fake_nav', 'fake_amcl'):
            self.create_service(GetState, name+'/get_state', self.lifecycle)
        self.stop_server = ActionServer(self, StopMission, 'navigation/stop', self.stop,
                                        callback_group=self.group)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        tr = TransformStamped();tr.header.frame_id='base_link';tr.child_frame_id='laser'
        tr.transform.rotation.w=1.;self.static.sendTransform(tr)
        self.ranges = np.full(720, np.inf)
        for p in points:
            index = int(round((math.atan2(p[1], p[0])+math.pi)/(2*math.pi/720))) % 720
            self.ranges[index] = min(self.ranges[index], float(np.linalg.norm(p)))
        message = OccupancyGrid()
        message.header.frame_id='map';message.info.width=self.model.cells.shape[1];message.info.height=self.model.cells.shape[0]
        message.info.resolution=.05;message.info.origin.orientation.w=1.
        message.data=self.model.cells.astype(int).ravel().tolist();self.map_pub.publish(message)
        self.timer = self.create_timer(.05, self.publish)

    def lifecycle(self, request, response):
        response.current_state=State(id=3,label='active');return response

    def initial_pose(self, message):
        p=message.pose.pose
        self.pose_updates.append((monotonic(), self.stopped))
        self.estimate=np.array([p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)])
        self.variance = .001

    def stop(self, handle):
        self.stop_requests.append(monotonic())
        self.stopped=True
        handle.succeed()
        return StopMission.Result(stopped=True,detail='synthetic stop completion')

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id='map';tr.child_frame_id='base_link'
        x,y,a=self.estimate
        tr.transform.translation.x=x;tr.transform.translation.y=y
        tr.transform.rotation.z=math.sin(a/2);tr.transform.rotation.w=math.cos(a/2);self.tf.sendTransform(tr)
        pose=PoseWithCovarianceStamped();pose.header=tr.header
        pose.pose.pose.position.x=x;pose.pose.pose.position.y=y;pose.pose.pose.orientation=tr.transform.rotation
        pose.pose.covariance[0]=pose.pose.covariance[7]=self.variance;pose.pose.covariance[35]=self.variance
        self.pose_pub.publish(pose)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser'
        scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.range_min=.1;scan.range_max=12.
        scan.ranges=self.ranges.tolist();self.scan_pub.publish(scan)
        status=DiagnosticStatus(name='base_driver',values=[KeyValue(key='feedback_age_s',value='0.01'),
            KeyValue(key='feedback_received',value='True'),KeyValue(key='fault',value=''),
            KeyValue(key='disarmed',value=str(self.stopped))])
        diagnostic=DiagnosticArray(status=[status]);diagnostic.header.stamp=stamp;self.diag_pub.publish(diagnostic)


@pytest.fixture
def graph(tmp_path,request):
    rclpy.init(domain_id=182)
    guard=LocalizationGuard(parameter_overrides=[Parameter('evidence_directory',value=str(tmp_path/'evidence'))])
    plant=Plant(getattr(request,'param',False))
    config=load(Path(__file__).resolve().parents[1]/'config/phone_lidar_supervisor.yaml')
    config['persistence']['state_file']=str(tmp_path/'supervisor.json')
    config['ros']['navigation_nodes']=['fake_nav'];config['ros']['localization_nodes']=['fake_amcl']
    config['ros']['lifecycle_poll_s']=.1
    supervisor=SupervisorNode(config=deepcopy(config))
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (guard,plant,supervisor):executor.add_node(node)
    thread=Thread(target=executor.spin,daemon=True);thread.start()
    try:yield guard,plant,supervisor
    finally:
        plant.timer.cancel();executor.shutdown(timeout_sec=4.);thread.join(4.)
        for node in (guard,plant,supervisor):node.destroy_node()
        rclpy.shutdown()


def acquire_renewed_lease(plant, supervisor, mission_id):
    """Keep the normal lease protocol alive while testing longer recovery windows."""
    client=plant.create_client(CheckPermission,'system/check_permission',callback_group=plant.group)
    wait(client.service_is_ready)
    initial=client.call_async(CheckPermission.Request(epoch=supervisor.core.epoch,mission_id=mission_id,
                                                      capabilities=['NAVIGATION']))
    wait(initial.done);assert initial.result().granted
    current=[initial.result()];pending=[None]
    def renew():
        if pending[0] is not None:
            if not pending[0].done():return
            current[0]=pending[0].result();pending[0]=None
        if current[0] is None or not current[0].granted:return
        pending[0]=client.call_async(CheckPermission.Request(epoch=current[0].epoch,mission_id=mission_id,
            capabilities=['NAVIGATION'],lease_id=current[0].lease_id,version=current[0].version))
    timer=plant.create_timer(.2,renew,callback_group=plant.group)
    return current[0].lease_id,timer


def test_low_covariance_wrong_pose_revokes_lease_and_recovers_only_after_stop(graph):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    wait(lambda:supervisor.core.capabilities.states['NAVIGATION'].value=='AVAILABLE')
    lease_id,timer=acquire_renewed_lease(plant,supervisor,'synthetic-drive')
    plant.stopped=False
    plant.estimate=plant.truth+np.array([.4,-.3,.12])
    wait(lambda:bool(plant.stop_requests))
    wait(lambda:bool(plant.pose_updates))
    assert plant.pose_updates[0][0] > plant.stop_requests[0] and plant.pose_updates[0][1]
    wait(lambda:guard.phase=='recovered',20.)
    wait(lambda:supervisor.core.capabilities.states['NAVIGATION'].value=='AVAILABLE')
    assert np.linalg.norm(plant.estimate[:2]-plant.truth[:2]) < .06
    assert not supervisor.core.permissions.leases[lease_id].granted
    assert supervisor.core.permissions.leases[lease_id].stopped
    assert all(not lease.granted for lease in supervisor.core.permissions.leases.values())
    assert any(job.state.value=='SUCCEEDED' for job in supervisor.core.recoveries.jobs.values())
    timer.cancel()


def test_mild_mismatch_slows_and_reconverges_without_canceling_goal(graph):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    wait(lambda:supervisor.core.capabilities.states['NAVIGATION'].value=='AVAILABLE')
    lease_id,timer=acquire_renewed_lease(plant,supervisor,'mild-reconvergence')
    plant.stopped=False
    plant.estimate=plant.truth+np.array([.22,0.,0.])
    wait(lambda:guard.state=='degraded')
    wait(lambda:30. in plant.speed_limits)
    sleep(1.)
    assert not plant.stop_requests and not plant.pose_updates
    assert supervisor.core.permissions.leases[lease_id].granted
    plant.estimate=plant.truth.copy()
    wait(lambda:guard.state=='available')
    wait(lambda:plant.speed_limits[-1]==100.)
    assert supervisor.core.permissions.leases[lease_id].granted
    assert not plant.stop_requests and not plant.pose_updates
    timer.cancel()


def test_missing_scan_becomes_unknown_and_never_creates_motion_authority(graph):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    plant.timer.cancel()
    wait(lambda:guard.state=='unknown')
    wait(lambda:supervisor.core.capabilities.states['NAVIGATION'].value!='AVAILABLE')
    assert not plant.pose_updates
    assert not supervisor.core.permissions.leases


def test_covariance_roundoff_remains_healthy_but_invalid_input_clears_evidence(graph):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    plant.variance=-1.4016105994451956e-11
    sleep(1.2)
    assert guard.state=='available'
    assert supervisor.core.capabilities.states['NAVIGATION'].value=='AVAILABLE'
    assert not plant.pose_updates
    plant.variance=-1e-8
    wait(lambda:guard.reason=='invalid_pose_covariance')
    assert guard.observation is None
    assert not plant.pose_updates


@pytest.mark.parametrize('displacement',[(.4, -.3, .12), (0., 0., 0.)])
def test_uncertain_amcl_keeps_scan_evidence_for_guarded_recovery(graph, displacement):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    plant.variance=.8
    plant.estimate=plant.truth+np.array(displacement)
    wait(lambda:guard.reason=='amcl_covariance_unverified')
    wait(lambda:bool(plant.pose_updates),20.)
    wait(lambda:guard.phase=='recovered',20.)
    wait(lambda:supervisor.core.capabilities.states['NAVIGATION'].value=='AVAILABLE')
    assert all(stopped for _,stopped in plant.pose_updates)
    assert np.linalg.norm(plant.estimate[:2]-plant.truth[:2]) < .06
    assert not supervisor.core.permissions.leases


def test_recovery_refuses_unconfirmed_stop_and_concurrent_owner(graph):
    guard,plant,supervisor=graph
    with supervisor.lock:supervisor.core.config['recovery_policies']['localization']['automatic']=False
    wait(lambda:guard.state=='available')
    guard.options['operation_timeout_s']=1.
    plant.stopped=False
    wait(lambda:not guard.is_stopped())
    client=ActionClient(plant,RecoverComponent,'localization/recover',callback_group=plant.group)
    wait(client.server_is_ready)
    request=RecoverComponent.Goal(job_id='stop-required',source='localization_alignment',
                                   fault_code='LOCALIZATION_ALIGNMENT_UNAVAILABLE')
    first=client.send_goal_async(request);wait(first.done);assert first.result().accepted
    duplicate=client.send_goal_async(request);wait(duplicate.done);assert not duplicate.result().accepted
    result=first.result().get_result_async();wait(result.done)
    assert not result.result().result.success
    assert not plant.pose_updates
    assert guard.phase=='failed'


def test_manual_pose_update_preempts_pending_automatic_search(graph):
    guard,plant,supervisor=graph
    with supervisor.lock:supervisor.core.config['recovery_policies']['localization']['automatic']=False
    wait(lambda:guard.state=='available')
    plant.estimate=plant.truth+np.array([.4,-.3,.12])
    wait(lambda:guard.state=='unavailable')
    client=ActionClient(plant,RecoverComponent,'localization/recover',callback_group=plant.group)
    wait(client.server_is_ready)
    future=client.send_goal_async(RecoverComponent.Goal(job_id='manual-preempt',source='localization_alignment',
                                                       fault_code='LOCALIZATION_ALIGNMENT_UNAVAILABLE'))
    wait(future.done);assert future.result().accepted
    wait(lambda:guard.phase=='searching')
    manual=PoseWithCovarianceStamped();manual.header.stamp=plant.get_clock().now().to_msg()
    manual.header.frame_id='map';manual.pose.pose.position.x=plant.truth[0]
    manual.pose.pose.position.y=plant.truth[1];manual.pose.pose.orientation.z=math.sin(plant.truth[2]/2)
    manual.pose.pose.orientation.w=math.cos(plant.truth[2]/2)
    pub=plant.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
    wait(lambda:pub.get_subscription_count() >= 2)
    pub.publish(manual)
    result=future.result().get_result_async();wait(result.done)
    assert not result.result().result.success
    assert len(plant.pose_updates)==1  # Only the explicitly published manual pose.


@pytest.mark.parametrize('graph',[True],indirect=True)
def test_ambiguous_corridor_exhausts_bounded_recovery_without_pose_jump(graph):
    guard,plant,supervisor=graph
    wait(lambda:guard.state=='available')
    plant.estimate=plant.truth+np.array([.4,0.,0.])
    wait(lambda:bool(supervisor.core.recoveries.jobs))
    wait(lambda:any(job.state.value=='FAILED' for job in supervisor.core.recoveries.jobs.values()),20.)
    assert not plant.pose_updates
    assert guard.recovery_reason=='ambiguous_candidate'
    assert supervisor.core.capabilities.states['NAVIGATION'].value!='AVAILABLE'
