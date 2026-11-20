"""Real ROS action transport, with synthetic control and Supervisor ports only."""
import math
import threading
import time
from types import SimpleNamespace as Obj
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import ActionClient
from rclpy.task import Future
from cleany_interfaces.action import StopMission
from nav2_msgs.action import NavigateToPose
from cleany_teleop.map_view import MapView
from cleany_mission_manager.core.supervision import Authorization
from cleany_teleop import navigation_bridge as module


class Authority:
    epoch='synthetic'
    lease_id='synthetic'
    def __init__(self,*args,**kwargs):pass
    def available(self):return True,''
    def acquire(self,*args):pass
    def maintain(self):pass
    def authorization(self):return Authorization.GRANTED,''
    def release(self):pass


class Control:
    def __init__(self):
        self.owner=None;self.phase='idle';self.stop_confirmed=True;self.forward_only=True
        self.clock=Obj(received_at=time.monotonic());self.state=Obj(velocity_rad_s=[0.]*4)
        self.transport=Obj(geometry=Obj(radius=.0635,wheelbase=.35,separation=.59))
        self.requests=[]
    def stop(self,reason):self.phase='idle';self.owner=None;self.stop_confirmed=True
    def stopped(self):return True
    def request(self,payload):
        self.requests.append(payload);self.owner=payload['client'];self.phase='moving'


def test_restarted_idle_bridge_accepts_only_current_epoch_stop(monkeypatch):
    monkeypatch.setattr(module,'RosSupervision',Authority)
    rclpy.init();node=rclpy.create_node('phone_restart_stop_test')
    control=Control();b=module.NavigationBridge(node,control,threading.RLock())
    request=Obj(epoch='synthetic',mission_id='old-goal',lease_id='old-lease')
    try:
        assert b.admit_stop(request)==GoalResponse.ACCEPT
        request.epoch='old-supervisor'
        assert b.admit_stop(request)==GoalResponse.REJECT
        request.epoch='synthetic';b.goal_id='new-goal';b.phase='active'
        assert b.admit_stop(request)==GoalResponse.REJECT
        request.mission_id='new-goal';request.lease_id='synthetic'
        assert b.admit_stop(request)==GoalResponse.ACCEPT
        assert control.requests==[]
    finally:
        node.destroy_node();rclpy.shutdown()


def test_previous_lease_stop_completes_through_ros_action_without_motion(monkeypatch):
    monkeypatch.setattr(module,'RosSupervision',Authority)
    rclpy.init();node=rclpy.create_node('phone_previous_lease_test')
    control=Control();b=module.NavigationBridge(node,control,threading.RLock())
    client=ActionClient(node,StopMission,'navigation/stop')
    try:
        assert client.wait_for_server(timeout_sec=2.)
        sent=client.send_goal_async(StopMission.Goal(epoch='synthetic',mission_id='old-goal',lease_id='old-lease'))
        end=time.monotonic()+3
        while not sent.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert sent.done() and sent.result().accepted
        result=sent.result().get_result_async()
        while not result.done() and time.monotonic()<end:
            control.clock.received_at=time.monotonic();b.tick();rclpy.spin_once(node,timeout_sec=.01)
        assert result.done() and result.result().result.stopped
        assert control.requests==[] and b.goal_id=='' and b.phase=='idle'
    finally:
        client.destroy();b.stop_server.destroy();node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('blocker', ['moving','stale','pending_goal','pending_cancel'])
def test_restored_stop_waits_for_current_standstill_and_finished_cancellation(monkeypatch,blocker):
    monkeypatch.setattr(module,'RosSupervision',Authority)
    rclpy.init();node=rclpy.create_node('phone_stop_proof_test')
    control=Control();b=module.NavigationBridge(node,control,threading.RLock())
    result=Future();outcomes=[]
    handle=Obj(succeed=lambda:outcomes.append('stopped'),abort=lambda:outcomes.append('failed'))
    b.stop_waiters=[(handle,result,time.monotonic())]
    try:
        if blocker=='moving':control.state.velocity_rad_s=[.1,0.,0.,0.]
        if blocker=='stale':control.clock.received_at=time.monotonic()-1.
        if blocker=='pending_goal':b.pending=Future()
        if blocker=='pending_cancel':b.cancel_future=Future()
        b.tick()
        assert not result.done() and not outcomes
        control.state.velocity_rad_s=[0.]*4;control.clock.received_at=time.monotonic()
        b.pending=b.cancel_future=None
        b.tick()
        assert result.done() and result.result().stopped and outcomes==['stopped']
        assert control.requests==[]
    finally:
        node.destroy_node();rclpy.shutdown()


def test_goal_transport_and_confirmed_result_without_browser_heartbeat(monkeypatch):
    monkeypatch.setattr(module,'RosSupervision',Authority)
    rclpy.init();client=rclpy.create_node('phone_action_test');server=rclpy.create_node('synthetic_nav2')
    observed=[]
    def execute(handle):
        observed.append(handle.request.pose)
        handle.succeed()
        return NavigateToPose.Result()
    action=ActionServer(server,NavigateToPose,'navigate_to_pose',execute_callback=execute)
    control=Control();b=module.NavigationBridge(client,control,threading.RLock())
    b.readiness=lambda:(True,'')
    b.map=MapView(10,10,.1,0.,0.,0.,[0]*100)
    b.pose=[.5,.5,0.]
    b.request('goal',dict(map_id=b.map.revision,x=.5,y=.5,yaw=0.,request_id='one'))
    b.heartbeat=0.
    end=time.monotonic()+5
    try:
        while time.monotonic()<end and b.phase!='succeeded':
            b.scan_at=b.odom_at=b.pose_at=control.clock.received_at=time.monotonic()
            b.tick()
            rclpy.spin_once(client,timeout_sec=.01);rclpy.spin_once(server,timeout_sec=.01)
        assert b.phase=='succeeded'
        assert observed[0].header.frame_id=='map'
        assert observed[0].pose.position.x==.5
        assert control.requests and all(p['vector']==[0.,0.,0.] for p in control.requests)
    finally:
        action.destroy();client.destroy_node();server.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('velocity', [(0.,.127,0.),(0.,-.127,0.),
    (.127,.127,0.),(.127,-.127,0.),(-.127,.127,0.),(-.127,-.127,0.),
    (-.127,0.,0.),(.127,.127,.27)])
def test_omni_bridge_preserves_lateral_direction_and_limits_each_wheel(monkeypatch,velocity):
    from cleany_teleop.driver_transport import limit_wheel_velocity
    monkeypatch.setattr(module,'RosSupervision',Authority)
    rclpy.init();node=rclpy.create_node('phone_omni_bridge_test')
    control=Control();control.forward_only=False
    b=module.NavigationBridge(node,control,threading.RLock())
    b.phase='active';b.goal_id='synthetic-omni';b.velocity=velocity
    b.odom_at=b.scan_at=b.pose_at=b.velocity_at=time.monotonic()
    try:
        b.tick()
        assert len(control.requests)==1
        request=control.requests[0]
        g=control.transport.geometry;arm=(g.wheelbase+g.separation)/2
        x,y,z=request['vector'];speed=request['speed']
        actual=(x*g.radius*speed,y*g.radius*speed,z*g.radius*speed/arm)
        assert actual==pytest.approx(limit_wheel_velocity(velocity,g,2.))
        wheels=[(actual[0]-actual[1]-arm*actual[2])/g.radius,
                (actual[0]+actual[1]+arm*actual[2])/g.radius,
                (actual[0]+actual[1]-arm*actual[2])/g.radius,
                (actual[0]-actual[1]+arm*actual[2])/g.radius]
        assert max(abs(v) for v in wheels)<=2.+1e-6
    finally:
        node.destroy_node();rclpy.shutdown()
