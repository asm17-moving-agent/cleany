import threading
from types import SimpleNamespace as Obj
from unittest.mock import Mock
import pytest
from cleany_teleop.navigation_bridge import NavigationBridge
from cleany_teleop.map_view import MapView


def bridge():
    b=NavigationBridge.__new__(NavigationBridge)
    b.lock=threading.RLock();b.phase='active';b.epoch=3;b.outcome='';b.reason='';b.handle=Mock();b.control=Mock();b.map=MapView(1,1,1.,0.,0.,0.,[0]);b.requests={};b.readiness=lambda:(False,'not ready')
    return b


def test_stop_precedes_action_cancel_and_invalidates_old_callbacks():
    b=bridge();events=[]
    b.control.stop.side_effect=lambda _:events.append('stop')
    b.handle.cancel_goal_async.side_effect=lambda:events.append('cancel')
    b.cancel('test')
    assert events==['stop','cancel'] and b.epoch==4 and b.phase=='stopping'
    b.finished(Mock(),3)
    assert b.phase=='stopping'


def test_late_goal_acceptance_is_cancelled_not_activated():
    b=bridge();h=Mock(accepted=True);future=Mock();future.result.return_value=h
    b.accepted(future,2)
    h.cancel_goal_async.assert_called_once()
    h.get_result_async.assert_not_called()
    assert b.phase=='active'


def test_not_ready_does_not_request_motion():
    b=bridge();b.phase='idle'
    with pytest.raises(ValueError,match='not ready'):
        b.request('goal',dict(request_id='one',map_id=b.map.revision,x=.5,y=.5,yaw=0.))
    b.control.stop.assert_not_called()


def path_message(points, frame='map'):
    return Obj(header=Obj(frame_id=frame), poses=[Obj(pose=Obj(position=Obj(x=x,y=y))) for x,y in points])


@pytest.mark.parametrize('phase', ['idle','preparing','active','succeeded'])
def test_global_plan_is_visible_before_motion_and_after_success(phase):
    b=bridge();b.phase=phase;b.path=[]
    b.on_path(path_message([(0.,0.),(1.,2.)]))
    assert b.path==[[0.,0.],[1.,2.]] and b.path_at>0


@pytest.mark.parametrize('phase', ['stopping','cancelled','failed'])
def test_late_plan_cannot_reappear_after_cancel(phase):
    b=bridge();b.phase=phase;b.path=[]
    b.on_path(path_message([(1.,2.)]))
    assert b.path==[]


def test_global_plan_rejects_wrong_frame_and_nonfinite_points():
    b=bridge();b.path=[]
    b.on_path(path_message([(1.,2.)],frame='odom'))
    b.on_path(path_message([(float('nan'),2.)]))
    assert b.path==[]


def test_empty_global_plan_clears_display():
    b=bridge();b.path=[[1.,2.]]
    b.on_path(path_message([]))
    assert b.path==[]


@pytest.mark.parametrize('field,label', [('odom_at','오도메트리'),
                                        ('scan_at','LiDAR'),('pose_at','위치 추정/TF')])
def test_timeout_reason_identifies_the_input_without_relaxing_limits(field,label):
    b=NavigationBridge.__new__(NavigationBridge)
    b.heartbeat=b.odom_at=b.scan_at=b.pose_at=100.
    b.odom_timeout=.3;b.scan_timeout=b.pose_timeout=.5
    assert b.input_failure(100.1)[0]==''
    setattr(b,field,98.)
    reason,ages=b.input_failure(100.1)
    assert label in reason and 2.1 in ages.values()


def test_accepted_autonomous_goal_does_not_depend_on_browser_presence():
    b=NavigationBridge.__new__(NavigationBridge)
    b.heartbeat=0.
    b.odom_at=b.scan_at=b.pose_at=100.
    b.odom_timeout=.3;b.scan_timeout=b.pose_timeout=.5
    assert b.input_failure(100.1)[0]==''


def test_success_outside_selected_goal_tolerance_remains_failure():
    import time
    from action_msgs.msg import GoalStatus
    b=bridge();b.target=(1.025,1.025,.463647609)
    b.pose=(1.065,1.025,.463647609);b.pose_at=time.monotonic();b.pose_timeout=.5
    b.goal_distance=.03;b.goal_angle=.1
    f=Mock();f.result.return_value.status=GoalStatus.STATUS_SUCCEEDED
    b.finished(f,3)
    assert b.outcome=='failed'
    b.control.stop.assert_called_once_with('Nav2 완료')
