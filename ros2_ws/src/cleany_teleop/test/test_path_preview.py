"""Read-only global-plan previews must not arm the base or revive older targets."""
from concurrent.futures import Future
import threading
import time
from types import SimpleNamespace as Obj
from unittest.mock import Mock

from rclpy.time import Time
from action_msgs.msg import GoalStatus

from cleany_teleop.map_view import MapView
from cleany_teleop.navigation_bridge import NavigationBridge


def bridge():
    b=NavigationBridge.__new__(NavigationBridge)
    b.lock=threading.RLock();b.map=MapView(10,10,1.,0.,0.,0.,[0]*100)
    b.phase='idle';b.pose_at=time.monotonic();b.pose_timeout=.5
    b.preview_generation=0;b.preview_id='';b.preview_path=[];b.preview_handle=None
    b.preview_phase='idle';b.preview_error='';b.path=[]
    b.control=Mock();b.supervision=Mock();b.node=Mock()
    b.node.get_clock.return_value.now.return_value.to_msg.return_value=Time().to_msg()
    b.preview_client=Mock();b.preview_client.server_is_ready.return_value=True
    b.status=lambda:{'preview_phase':b.preview_phase}
    return b


def target(b,request_id='one',x=1.):
    return dict(request_id=request_id,map_id=b.map.revision,x=x,y=1.,yaw=0.)


def result(points):
    poses=[Obj(pose=Obj(position=Obj(x=x,y=y),orientation=Obj(x=0.,y=0.,z=0.,w=1.))) for x,y in points]
    return Obj(status=GoalStatus.STATUS_SUCCEEDED,result=Obj(path=Obj(header=Obj(frame_id='map'),poses=poses)))


def test_preview_uses_planner_only_and_never_enables_base():
    b=bridge();accept=Future();complete=Future();handle=Mock(accepted=True)
    handle.get_result_async.return_value=complete
    b.preview_client.send_goal_async.return_value=accept
    assert b.preview(target(b))['preview_phase']=='planning'
    msg=b.preview_client.send_goal_async.call_args.args[0]
    assert msg.goal.header.frame_id=='map' and msg.goal.pose.position.x==1.
    assert msg.planner_id=='GridBased' and not msg.use_start
    accept.set_result(handle);complete.set_result(result([(0.,0.),(1.,1.)]))
    assert b.preview_phase=='ready' and b.preview_path==[[0.,0.],[1.,1.]]
    assert b.preview_target==[1.,1.,0.]
    assert b.control.mock_calls==[] and b.supervision.mock_calls==[]


def test_old_target_result_does_not_overwrite_new_target():
    b=bridge();first=Future();second=Future();done=Future();h=Mock(accepted=True)
    h.get_result_async.return_value=done;b.preview_client.send_goal_async.side_effect=[first,second]
    b.preview(target(b));first.set_result(h)
    b.preview(target(b,'two',2.))
    h.cancel_goal_async.assert_called_once()
    done.set_result(result([(0.,0.),(1.,1.)]))
    assert b.preview_id=='two' and b.preview_path==[] and b.preview_phase=='planning'


def test_late_acceptance_after_cancel_is_cancelled():
    b=bridge();f=Future();h=Mock(accepted=True);b.preview_client.send_goal_async.return_value=f
    b.preview(target(b));b.clear_preview();f.set_result(h)
    h.cancel_goal_async.assert_called_once();h.get_result_async.assert_not_called()
    assert b.preview_path==[]


def test_no_route_clears_preview_and_reports_failure():
    b=bridge();f=Future();done=Future();h=Mock(accepted=True)
    h.get_result_async.return_value=done;b.preview_client.send_goal_async.return_value=f
    b.preview(target(b));f.set_result(h);done.set_result(result([]))
    assert b.preview_phase=='failed' and b.preview_path==[] and b.preview_error


def test_preview_exposes_quantized_endpoint_without_changing_arrival_tolerance():
    import math
    b=bridge();b.preview_generation=1
    message=result([(0.,0.),(1.025,1.025)])
    q=message.result.path.poses[-1].pose.orientation
    q.z,q.w=math.sin(.463647609/2),math.cos(.463647609/2)
    f=Future();f.set_result(message)
    b.preview_finished(f,1,b.map.revision)
    assert b.preview_target[:2]==[1.025,1.025]
    assert abs(b.preview_target[2]-.463647609)<1e-9
    b.clear_preview()
    assert b.preview_target is None
    assert not b.control.mock_calls
