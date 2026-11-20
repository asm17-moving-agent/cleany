"""Coordinate and map persistence checks for the parallel extension tool."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import math

SCRIPT = Path(__file__).parents[2]/'tools/validation/map_extension/extend.py'
spec = importlib.util.spec_from_file_location('map_extension', SCRIPT)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_initial_pose_uses_saved_trajectory_origin():
    result = m.relative_pose(2, 4, math.pi, {'x': 2, 'y': 3, 'yaw': math.pi/2})
    assert abs(result[0]-1) < 1e-9
    assert abs(result[1]) < 1e-9
    assert abs(result[2]-math.pi/2) < 1e-9


def test_map_rows_and_unknown_values(tmp_path):
    grid = S(info=S(width=2, height=2, resolution=.05, origin=S(position=S(x=1,y=2),orientation=S(x=0,y=0,z=0,w=1))), data=[0,100,-1,50])
    m.write_grid(grid,tmp_path)
    assert (tmp_path/'map.pgm').read_bytes() == b'P5\n2 2\n255\n'+bytes([205,127,254,0])
    assert 'origin: [1, 2, 0.0]' in (tmp_path/'map.yaml').read_text()
    assert not list(tmp_path.glob('*.tmp'))


def test_view_tf_composes_same_time_poses():
    spec = importlib.util.spec_from_file_location('map_extension_view', SCRIPT.parent/'view_tf.py')
    view = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(view)
    actual = view.correction((3, 4, math.pi/2), (1, 0, 0))
    for got, expected in zip(actual, (3, 3, math.pi/2)):
        assert abs(got-expected) < 1e-9


def test_odom_guard_rejects_origin_reset_and_gap():
    guard = m.OdomContinuity()
    assert guard.check(10, 100, -.49, -.95, .57) is None
    assert guard.check(10.02, 100.02, -.49, -.95, .57) is None
    assert 'position jumped' in guard.check(10.04, 100.04, 0, 0, 0)
    assert 'input gap' in guard.check(15, 105, -.49, -.95, .57)


def test_odom_guard_accepts_wrapped_heading():
    guard = m.OdomContinuity()
    assert guard.check(10, 100, 0, 0, math.pi-.01) is None
    assert guard.check(10.02, 100.02, .001, 0, -math.pi+.01) is None


def test_odom_guard_uses_sensor_time_for_queued_samples():
    guard = m.OdomContinuity()
    assert guard.check(10,100,0,0,0) is None
    assert guard.check(10.02,105,0,0,0) is None


def test_trajectory_view_emphasizes_new_nodes_without_mutating_source():
    spec = importlib.util.spec_from_file_location('trajectory_view', SCRIPT.parent/'view_tf.py')
    view = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(view)
    from visualization_msgs.msg import Marker, MarkerArray
    sent = []
    marker = Marker(ns='Trajectory 1')
    marker.color.a = .25
    source = MarkerArray(markers=[marker])
    owner = S(trajectory_out=S(publish=sent.append))
    view.ViewTF.trajectory(owner, source)
    assert source.markers[0].color.a == .25
    assert sent[0].markers[0].color.a == 1.
    assert sent[0].markers[0].color.r == 1.
