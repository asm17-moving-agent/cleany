"""Synthetic ROS graph only: no serial device, Nav2 controller or motors."""
import time
import json
from itertools import count
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_action_status_default
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cleany_tof.proximity import model_id
from nav_msgs.msg import Odometry
from cleany_interfaces.action import DeskApproach
from cleany_interfaces.msg import TofFrame, SystemState, SystemHeartbeat, MissionPermission, ComponentHealth
from cleany_tof.approach import DeskApproachNode
from cleany_tof.guard import ProximityGuard
from cleany_tof.core import Settings, project


_DOMAINS = count(190)


class World(Node):
    def __init__(self):
        super().__init__('synthetic_tof_world')
        self.distance = 1.
        self.sensor = self.authority = True
        self.active_nav = False
        self.lease_granted = True
        self.operating = 'OPERATIONAL'
        self.seq = 0
        self.outputs, self.selected, self.health = [], [], []
        self.frame = self.create_publisher(TofFrame, 'tof/frame', 1)
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.nav_status = self.create_publisher(GoalStatusArray, 'navigate_to_pose/_action/status', qos_profile_action_status_default)
        self.nav = self.create_publisher(Twist, 'nav2/cmd_vel', 1)
        self.upstream = self.create_publisher(Twist, 'proximity/input', 1)
        self.state = self.create_publisher(SystemState, 'system/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.heartbeat = self.create_publisher(SystemHeartbeat, 'system/heartbeat', 1)
        self.create_subscription(Twist, 'motion/cmd_vel', self.forward, 1)
        self.create_subscription(Twist, 'cmd_vel', self.outputs.append, 1)
        self.create_subscription(ComponentHealth, 'system/component_health', self.health.append, 10)
        self.client = ActionClient(self, DeskApproach, 'desk/approach')
        self.timer = self.create_timer(.03, self.publish)
        self.nav_speed = .0
        self.frame_mode = 'range'
        self.sensor_ready = True
        self.publish_proof = False
        self.proof_covered = True
        self.proof_age = .01
        self.proof_model = model_id(Settings(calibrated=True))
        self.reasons = []
        self.coverage = self.create_publisher(String, 'tof/depth_coverage', 1)
        self.create_subscription(String, 'proximity/status', lambda msg: self.reasons.append(msg.data), 1)

    def forward(self, msg):
        self.selected.append(msg)
        self.upstream.publish(msg)  # Explicit identity substitute for the existing safety chain.

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        self.odom.publish(odom)
        if self.sensor:
            self.seq += 1
            points, _ = project([1.]*64, [.001]*64, [5]*64, [1]*64, [True]*64, Settings())
            msg = TofFrame(sensor_ready=True, sequence=self.seq, distance_m=[float(v) for v in self.distance/points[:, 0]],
                           sigma_m=[.001]*64, target_status=[5]*64, target_count=[1]*64, valid=[True]*64)
            msg.sensor_ready = self.sensor_ready
            if self.frame_mode in ('no_return', 'bad'):
                msg.target_count = [0 if self.frame_mode == 'no_return' else 1]*64
                msg.target_status = [255 if self.frame_mode == 'no_return' else 4]*64
                msg.valid = [False]*64
                msg.distance_m = [.338]*64
            msg.header.stamp, msg.header.frame_id = stamp, 'tof_link'
            self.frame.publish(msg)
        if self.publish_proof:
            self.coverage.publish(String(data=json.dumps(dict(contract='tof_sweep_v1',
                model_id=self.proof_model, covered=self.proof_covered, reason='COVERED',
                stamp_s=self.get_clock().now().nanoseconds*1e-9, source_age_s=self.proof_age,
                requested=[self.nav_speed, 0., 0.], measured=[0., 0., 0.]))))
        if self.authority:
            hb = SystemHeartbeat(epoch='test-epoch')
            hb.header.stamp = stamp
            self.heartbeat.publish(hb)
            state = SystemState(epoch='test-epoch', operating_state=self.operating, permissions=[
                MissionPermission(mission_id='test-mission', lease_id='test-lease', version=1,
                                  capabilities=['NAVIGATION'], granted=self.lease_granted, valid_for_s=1.)])
            state.header.stamp = stamp
            self.state.publish(state)
        msg = GoalStatusArray(status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)] if self.active_nav else [])
        self.nav_status.publish(msg)
        cmd = Twist()
        cmd.linear.x = self.nav_speed
        self.nav.publish(cmd)

    def goal(self, **kwargs):
        fields = dict(distance_m=.75, mission_id='test-mission', lease_id='test-lease',
                      lease_epoch='test-epoch', lease_version=1)
        fields.update(kwargs)
        return self.client.send_goal_async(DeskApproach.Goal(**fields))


@pytest.fixture
def graph():
    rclpy.init(domain_id=next(_DOMAINS))
    parameters = [Parameter('calibrated', value=True)]
    guard, approach, world = ProximityGuard(parameter_overrides=parameters), DeskApproachNode(parameter_overrides=parameters), World()
    executor = SingleThreadedExecutor()
    for node in (guard, approach, world):
        executor.add_node(node)

    def until(predicate, timeout=4.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        assert predicate(), 'synthetic graph condition timed out'

    until(lambda: approach.graph_ok and guard.graph_ok and approach.nav_seen and approach.supervision.operable())
    world.approach_node = approach
    yield world, until
    executor.shutdown()
    for node in (world, approach, guard):
        node.destroy_node()
    rclpy.shutdown()


def stopped(msg):
    return msg.linear.x == msg.linear.y == msg.angular.z == 0.


def test_fresh_navigation_stale_sensor_and_health(graph):
    world, until = graph
    world.nav_speed = .1
    until(lambda: world.outputs and world.outputs[-1].linear.x > .05)
    until(lambda: world.health and world.health[-1].status == ComponentHealth.AVAILABLE)
    world.sensor = False
    until(lambda: world.health[-1].status == ComponentHealth.UNAVAILABLE)
    until(lambda: stopped(world.outputs[-1]))


def test_ownership_rejection_cancellation_and_settled_success(graph):
    world, until = graph
    world.active_nav = True
    until(lambda: world.approach_node.nav_active)
    future = world.goal()
    until(future.done)
    assert not future.result().accepted
    world.active_nav = False
    until(lambda: not world.approach_node.nav_active)
    future = world.goal(lease_id='wrong')
    until(future.done)
    assert not future.result().accepted
    future = world.goal()
    until(future.done)
    handle = future.result()
    assert handle.accepted
    until(lambda: world.selected[-1].linear.x > 0)
    cancel = handle.cancel_goal_async()
    until(cancel.done)
    result = handle.get_result_async()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_CANCELED
    assert not result.result().result.success
    world.distance = .75
    # Wait until new samples replace the old plane before starting the second action.
    baseline = world.seq
    until(lambda: world.seq > baseline+4)
    future = world.goal()
    until(future.done)
    assert future.result().accepted
    result = future.result().get_result_async()
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED and result.result().result.success
    until(lambda: stopped(world.outputs[-1]))


def test_authority_loss_aborts_approach_and_stops_final_output(graph):
    world, until = graph
    future = world.goal()
    until(future.done)
    assert future.result().accepted
    result = future.result().get_result_async()
    until(lambda: world.outputs and world.outputs[-1].linear.x > 0)
    world.authority = False
    until(result.done)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.reason == 'SUPERVISOR_AUTHORITY_LOST'
    until(lambda: stopped(world.outputs[-1]))
    # Loss of authority must not masquerade as a sensor failure.
    assert world.health[-1].status == ComponentHealth.AVAILABLE


def test_revoked_live_lease_aborts_without_waiting_for_heartbeat_timeout(graph):
    world, until = graph
    future = world.goal()
    until(future.done)
    assert future.result().accepted
    result = future.result().get_result_async()
    until(lambda: world.outputs and world.outputs[-1].linear.x > 0)
    world.lease_granted = False
    until(result.done)
    assert result.result().result.reason == 'SUPERVISOR_AUTHORITY_LOST'
    until(lambda: stopped(world.outputs[-1]))


def test_obstacle_is_not_a_system_health_fault_and_locked_state_stops_navigation(graph):
    world, until = graph
    world.nav_speed = .1
    until(lambda: world.outputs and world.outputs[-1].linear.x > 0)
    world.distance = .62
    until(lambda: stopped(world.outputs[-1]))
    assert world.health[-1].status == ComponentHealth.AVAILABLE
    world.distance = 1.
    until(lambda: world.outputs[-1].linear.x > 0)
    world.operating = 'FAULT_LOCKED'
    until(lambda: stopped(world.outputs[-1]))


def test_mission_client_waits_for_stopped_feedback_and_clears_journal(graph, tmp_path):
    from types import SimpleNamespace
    from cleany_mission_manager.adapters.desk_approach import DeskApproachPort
    from cleany_mission_manager.core.journal import MissionJournal
    world, until = graph
    journal = MissionJournal(str(tmp_path/'approach.db'))
    stopped_feedback = [True]
    lease = SimpleNamespace(mission_id='test-mission', lease_id='test-lease', lease_epoch='test-epoch', version=1)
    port = DeskApproachPort(world, journal, lambda: stopped_feedback[0], .75, lease)
    try:
        until(lambda: port.ready()[0])
        operation = port.start('seat-12')
        assert journal.value('desk_goal_uuid') == operation
        assert port.poll(operation) is None
        until(lambda: world.selected[-1].linear.x > 0)
        stopped_feedback[0] = False
        world.distance = .75
        until(lambda: port.result is not None)
        assert port.result.ok and port.poll(operation) is None
        stopped_feedback[0] = True
        assert port.poll(operation).ok and journal.value('desk_goal_uuid') == ''
    finally:
        journal.close()


def test_restarted_client_cancels_exact_journaled_goal_before_readiness(graph, tmp_path):
    from uuid import UUID
    from cleany_mission_manager.adapters.desk_approach import DeskApproachPort
    from cleany_mission_manager.core.journal import MissionJournal
    world, until = graph
    future = world.goal()
    until(future.done)
    handle = future.result()
    assert handle.accepted
    journal = MissionJournal(str(tmp_path/'interrupted.db'))
    identity = str(UUID(bytes=bytes(handle.goal_id.uuid)))
    journal.set('desk_goal_uuid', identity)
    port = DeskApproachPort(world, journal, lambda: True, .75)
    result = handle.get_result_async()
    try:
        assert not port.ready()[0]
        def recovered():
            port.maintain()
            return port.ready()[0]
        until(recovered)
        until(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        assert journal.value('desk_goal_uuid') == ''
    finally:
        journal.close()


def test_no_return_requires_live_matching_depth_coverage_without_faulting_sensor(graph):
    world, until = graph
    world.frame_mode, world.nav_speed = 'no_return', .2
    until(lambda: world.reasons and world.reasons[-1] == 'NO_RETURN_UNCOVERED')
    assert stopped(world.outputs[-1]) and world.health[-1].status == ComponentHealth.AVAILABLE
    rejected = world.goal()
    until(rejected.done)
    assert not rejected.result().accepted  # No measured desk plane.
    world.publish_proof = True
    until(lambda: world.reasons[-1] == 'NO_RETURN_DEPTH_COVERED')
    assert 0 < world.outputs[-1].linear.x <= .080001
    world.proof_covered = False
    until(lambda: world.reasons[-1] == 'NO_RETURN_UNCOVERED')
    assert stopped(world.outputs[-1])
    world.proof_covered = True
    until(lambda: world.reasons[-1] == 'NO_RETURN_DEPTH_COVERED')
    world.publish_proof = False
    until(lambda: world.reasons[-1] == 'NO_RETURN_UNCOVERED')
    assert stopped(world.outputs[-1]) and world.health[-1].status == ComponentHealth.AVAILABLE


def test_stale_wrong_model_and_unreliable_frames_cannot_borrow_depth_proof(graph):
    world, until = graph
    world.frame_mode, world.nav_speed, world.publish_proof = 'no_return', .1, True
    until(lambda: world.reasons and world.reasons[-1] == 'NO_RETURN_DEPTH_COVERED')
    world.proof_age = 1.
    until(lambda: world.reasons[-1] == 'NO_RETURN_UNCOVERED')
    world.proof_age, world.proof_model = .01, 'wrong-profile'
    baseline = world.seq
    until(lambda: world.seq > baseline+3)
    assert world.reasons[-1] == 'NO_RETURN_UNCOVERED'
    world.proof_model = model_id(Settings(calibrated=True))
    until(lambda: world.reasons[-1] == 'NO_RETURN_DEPTH_COVERED')
    world.frame_mode = 'bad'
    until(lambda: world.health[-1].status == ComponentHealth.UNAVAILABLE)
    assert stopped(world.outputs[-1])
    world.frame_mode = 'no_return'
    until(lambda: world.reasons[-1] == 'NO_RETURN_DEPTH_COVERED')
    world.sensor_ready = False
    until(lambda: world.health[-1].status == ComponentHealth.UNAVAILABLE)
    assert stopped(world.outputs[-1])
