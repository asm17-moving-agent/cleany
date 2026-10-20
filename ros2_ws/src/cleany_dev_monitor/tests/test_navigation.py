"""Protect observation boundaries, timing, large messages and durable replay."""
import math
import struct
from types import SimpleNamespace as NS

import pytest

from cleany_dev_monitor.navigation import NavigationObserver, cloud_points, transform_xyz
from cleany_dev_monitor.core import Hub, Recorder


def runtime(operation="op-13", state="RETURN_HOME"):
    return {"state": state, "navigation": {"operation_id": operation, "target": "home", "frame": "map", "goal_pose": {"x": 1., "y": 0., "yaw": 0.}}}


def tick(observer, t, speed=.01, *, sim=None, moving=True):
    if moving:
        observer.ingest("odometry", {"velocity": [speed, 0., 0.]}, 100+t, t, sim)
    for topic in observer.velocity_topics:
        observer.ingest("cmd:"+topic, {"velocity": [.1, 0., 0.]}, 100+t, t)
    return observer.snapshot(100+t, t, sim, [t*.01, 0., 0.], runtime(), True)[0]


def test_measured_creep_is_not_stop_and_observation_only_pan_has_no_veto():
    observer = NavigationObserver(["/nav2/cmd_vel", "/cmd_vel"])
    observer.ingest("pan", {"reason": "PAN_TARGET_OUT_OF_RANGE", "motion_authority": False}, 100, 0)
    first = tick(observer, 0)
    assert first["motion"] == "CREEPING"
    assert first["reason_source"] is None
    second = tick(observer, .2)
    assert second["active"]["creep_s"] == pytest.approx(.2)
    assert second["active"]["stationary_s"] == 0
    third = tick(observer, .4, 0.)
    fourth = tick(observer, .6, 0.)
    assert fourth["active"]["stationary_s"] == pytest.approx(.2)
    assert fourth["active"]["stationary_episodes"] == 1
    assert third["active"]["unobserved_s"] == pytest.approx(.2)


def test_stale_motion_and_observer_gaps_are_not_counted_as_stationary():
    observer = NavigationObserver(["/cmd_vel"])
    tick(observer, 0, speed=0.)
    result = tick(observer, 1., speed=0., moving=False)
    assert result["motion"] == "UNOBSERVED"
    assert result["active"]["stationary_s"] == 0
    assert result["active"]["unobserved_s"] == 1.
    assert result["history"][-1][-2] is None


def test_actual_rtf_clock_reset_and_result_do_not_restart_completed_operation():
    observer = NavigationObserver(["/cmd_vel"])
    for i in range(6):
        data = tick(observer, i*.2, sim=10+i*.4)
    assert data["rtf"] == pytest.approx(2.)
    assert data["active"]["rtf"] == pytest.approx(2.)
    reset = tick(observer, 1.2, sim=0.)
    assert reset["rtf"] is None
    assert reset["runs"][0]["result"] == "CLOCK_RESET_INCOMPLETE"
    assert reset["active"] is None
    next_runtime = runtime("op-new")
    observer.snapshot(102, 2, 1., [0,0,0], next_runtime, True)
    observer.finish("OK", 102.2)
    result, _ = observer.snapshot(102.3, 2.3, 1.6, [0,0,0], next_runtime, True)
    assert result["active"] is None
    assert result["runs"][-1]["result"] == "OK"


def test_stamped_future_and_stale_measurements_are_unavailable():
    observer = NavigationObserver(["/cmd_vel"])
    observer.ingest("ground_truth", {"velocity": [0.,0.,0.]}, 100., 0., 1.)
    data, _ = observer.snapshot(100., 0., 5., None, {}, False)
    assert data["motion"] == "UNOBSERVED"
    observer.ingest("ground_truth", {"velocity": [0.,0.,0.]}, 100., 0., 6.)
    data, _ = observer.snapshot(100., 0., 5., None, {}, False)
    assert data["motion"] == "UNOBSERVED"


def test_cloud_padding_big_endian_nonfinite_and_bounded_sampling():
    fields = [NS(name=name, offset=i*4, datatype=7, count=1) for i,name in enumerate(("x","y","z"))]
    message = NS(width=2,height=2,point_step=12,row_step=28,fields=fields,is_bigendian=True,
                 data=struct.pack('>ffffff',1,2,3,4,5,6)+b'pad!'+struct.pack('>ffffff',7,8,9,math.nan,0,0)+b'pad!')
    points, count = cloud_points(message, 16)
    assert count == 4 and points == [[1,2,3],[4,5,6],[7,8,9]]
    message.data = message.data[:-1]
    with pytest.raises(ValueError, match="layout"):
        cloud_points(message, 16)
    huge = NS(width=100_000,height=1,point_step=12,row_step=1_200_000,fields=fields,is_bigendian=False,
              data=struct.pack('<fff',1,2,3)*100_000)
    assert len(cloud_points(huge, 32)[0]) <= 32
    rotated = transform_xyz([[1.,0.,0.]], [2.,3.,4.], [0., math.sqrt(.5), 0., math.sqrt(.5)])
    assert rotated[0] == pytest.approx([2.,3.,3.])


def test_navigation_replay_never_reads_future_and_comparison_retains_context(tmp_path):
    recorder = Recorder(tmp_path)
    hub = Hub()
    hub.put("config", "monitor", {"source_revision": "abc", "stationary_speed_mps": .005})
    before = hub.put("navigation", "summary", {"motion": "CREEPING", "runs": []})
    recorder.append(hub.snapshot())
    hub.put("navigation", "summary", {"motion": "STATIONARY", "runs": [{"operation_id": "13", "result": "OK", "stationary_s": 1.}]})
    recorder.append(hub.snapshot(recorder.last))
    replay = recorder.replay(recorder.session_id, before["received_at"])
    summary = next(row["data"] for row in replay["rows"] if row["kind"] == "navigation")
    assert summary["motion"] == "CREEPING" and not summary["runs"]
    runs = recorder.navigation_runs()
    assert runs[0]["configuration"]["source_revision"] == "abc"
    assert runs[0]["session_id"] == recorder.session_id
    recorder.close()


def test_motion_timeline_survives_frequent_bt_status_updates():
    observer = NavigationObserver(["/cmd_vel"])
    observer.note("navigation_decision", {"decision":"STOP_OBSTACLE"}, 99.)
    for _ in range(2000):
        observer.bt([{"node":"IsPathValid","status":"SUCCESS"}])
    data = tick(observer, 0.)
    assert data["timeline"][0]["data"]["decision"] == "STOP_OBSTACLE"
    assert data["bt_status"]["IsPathValid"] == "SUCCESS"
