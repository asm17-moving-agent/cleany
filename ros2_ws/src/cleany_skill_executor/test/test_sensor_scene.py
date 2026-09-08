from pathlib import Path
from types import SimpleNamespace
from std_msgs.msg import Header

import pytest

from cleany_skill_executor.core.sensor_scene import validate_sensor_scene
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    NearestPregraspCoordinator,
)


def test_populated_fresh_scene_passes():
    validate_sensor_scene(0.5, 2.0, 'OcTree', 500)


def test_attachment_barrier_waits_for_a_new_processed_capture(monkeypatch):
    import cleany_skill_executor.nearest_pregrasp_coordinator as module
    node = SimpleNamespace(_scene_cloud_stamp_ns=10,
                           _check_sensor_scene=lambda: None,
                           get_logger=lambda: SimpleNamespace(info=lambda _: None))
    spins = []
    def update(*_, **__):
        spins.append(True)
        node._scene_cloud_stamp_ns = 21
    monkeypatch.setattr(module.rclpy, 'spin_once', update)
    NearestPregraspCoordinator._wait_for_sensor_scene(node, 1., after_stamp_ns=20)
    assert len(spins) == 1


def test_fresh_but_pre_attachment_capture_cannot_pass_barrier(monkeypatch):
    node = SimpleNamespace(_scene_cloud_stamp_ns=20,
                           _check_sensor_scene=lambda: None,
                           get_logger=lambda: SimpleNamespace(info=lambda _: None))
    with pytest.raises(RuntimeError, match='after the attachment'):
        NearestPregraspCoordinator._wait_for_sensor_scene(node, 0., after_stamp_ns=20)


def test_receipt_preserves_capture_age_and_missing_receipt_does_not_refresh_it():
    node = SimpleNamespace(_scene_cloud_stamp_ns=None)
    message = Header(frame_id='camera')
    message.stamp.sec = 7
    NearestPregraspCoordinator._on_scene_cloud_receipt(node, message)
    assert node._scene_cloud_stamp_ns == 7_000_000_000
    received = node._scene_commit_received_at
    NearestPregraspCoordinator._on_scene_cloud_receipt(node, message)
    assert node._scene_commit_received_at == received  # replay cannot renew it
    NearestPregraspCoordinator._on_scene_cloud_receipt(node, Header())
    assert node._scene_cloud_stamp_ns == 7_000_000_000
    with pytest.raises(RuntimeError, match='fresh'):
        validate_sensor_scene((10_000_000_000-node._scene_cloud_stamp_ns)/1e9,
                              2., 'OcTree', 500)


@pytest.mark.parametrize('age', [-0.1, 2.1, float('inf'), float('nan')])
def test_missing_or_stale_camera_fails_closed(age):
    with pytest.raises(RuntimeError, match='fresh'):
        validate_sensor_scene(age, 2., 'OcTree', 500)


@pytest.mark.parametrize('tree_id,size', [
    ('', 0), ('OcTree', 0), ('other', 100),
])
def test_missing_map_fails_closed(tree_id, size):
    with pytest.raises(RuntimeError, match='OctoMap'):
        validate_sensor_scene(0., 2., tree_id, size)


def test_sensor_launch_defaults_no_static_geometry_and_no_execution():
    package = Path(__file__).resolve().parents[1]
    launch = (package / 'launch' /
              'study_cafe_nearest_grasp_demo.launch.py').read_text()
    assert "'sensor_scene', default_value='true'" in launch
    assert 'condition=UnlessCondition(sensor_scene)' in launch
    assert "'plan_only', default_value='true'" in launch
    source = (package / 'cleany_skill_executor' /
              'nearest_pregrasp_coordinator.py').read_text()
    guard = source.index("if bool(self.get_parameter('plan_only').value):")
    assert guard < source.index('self._execute_pregrasp(selected, attempt)')


def test_plan_only_never_executes_even_with_reachable_candidate():
    values = {'startup_timeout_sec': 1., 'execute_grasp_and_lift': False,
              'require_sensor_scene': True, 'plan_only': True}
    service = SimpleNamespace(wait_for_server=lambda **_: True,
                              wait_for_service=lambda **_: True)
    events = []
    attempt = SimpleNamespace(object_id=1, label='cup', distance_m=0.5)
    selection = SimpleNamespace(
        selected_candidate_index=0, selected_arm='left'
    )
    node = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _inspection=service, _grasp=service, _selection=service,
        _move_group=service, _wait_for_joint_state=lambda _: None,
        _hold=lambda _: None,
        _wait_for_sensor_scene=lambda _: events.append('scene_ready'),
        _detect_objects=lambda: SimpleNamespace(detections=SimpleNamespace(
            detections=[], snapshot_id='snapshot')),
        _attempts=lambda _: [attempt],
        _inspect_selected=lambda *_: object(),
        _plan_grasps=lambda *_: SimpleNamespace(candidates=[object()]),
        _publish_grasp_overlay=lambda *_, **__: None,
        _select_reachable=lambda *_: selection,
        _execute_pregrasp=lambda *_: pytest.fail('Unexpected arm motion'),
        _open_gripper=lambda *_: pytest.fail('Unexpected gripper motion'),
    )
    node._wait_for_pipeline = lambda: (
        NearestPregraspCoordinator._wait_for_pipeline(node))
    node._run_nearest = lambda: NearestPregraspCoordinator._run_nearest(node)
    NearestPregraspCoordinator.run(node)
    assert events == ['scene_ready']


def test_map_failure_prevents_perception_and_motion():
    node = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value={
            'startup_timeout_sec': 1., 'execute_grasp_and_lift': False,
            'require_sensor_scene': True,
        }[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _wait_for_joint_state=lambda _: None, _hold=lambda _: None,
        _detect_objects=lambda: pytest.fail('Perception bypassed gate'),
    )
    service = SimpleNamespace(wait_for_server=lambda **_: True,
                              wait_for_service=lambda **_: True)
    for name in ('_inspection', '_grasp', '_selection', '_move_group'):
        setattr(node, name, service)

    def unavailable(_):
        raise RuntimeError('No sensor scene')

    node._wait_for_sensor_scene = unavailable
    node._wait_for_pipeline = lambda: (
        NearestPregraspCoordinator._wait_for_pipeline(node))
    node._run_nearest = lambda: NearestPregraspCoordinator._run_nearest(node)
    with pytest.raises(RuntimeError, match='No sensor scene'):
        NearestPregraspCoordinator.run(node)


def test_scene_diffs_do_not_refresh_old_map_and_full_empty_scene_clears_it():
    from moveit_msgs.msg import PlanningScene
    node = SimpleNamespace(
        _sensor_map_id='', _sensor_map_bytes=0,
        _sensor_map_received_at=float('-inf'),
    )
    scene = PlanningScene()
    scene.world.octomap.octomap.id = 'OcTree'
    scene.world.octomap.octomap.data = [1, 2]
    NearestPregraspCoordinator._on_sensor_scene(node, scene)
    timestamp = node._sensor_map_received_at
    assert node._sensor_map_bytes == 2
    state_diff = PlanningScene(is_diff=True)
    NearestPregraspCoordinator._on_sensor_scene(node, state_diff)
    assert node._sensor_map_bytes == 2
    assert node._sensor_map_received_at == timestamp
    NearestPregraspCoordinator._on_sensor_scene(node, PlanningScene())
    assert node._sensor_map_bytes == 0


@pytest.mark.parametrize('map_age', [0.1, 3.0])
def test_fresh_cloud_cannot_hide_stale_map(map_age, monkeypatch):
    import cleany_skill_executor.nearest_pregrasp_coordinator as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    node = SimpleNamespace(
        _scene_cloud_stamp_ns=1_000_000_000,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_100_000_000)),
        _sensor_map_received_at=10. - map_age,
        _sensor_map_id='OcTree', _sensor_map_bytes=2,
        get_parameter=lambda _: SimpleNamespace(value=2.),
    )
    if map_age > 2.:
        with pytest.raises(RuntimeError):
            NearestPregraspCoordinator._check_sensor_scene(node)
    else:
        NearestPregraspCoordinator._check_sensor_scene(node)


def test_new_post_integration_receipt_renews_static_populated_map_only(monkeypatch):
    import cleany_skill_executor.nearest_pregrasp_coordinator as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    node = SimpleNamespace(_scene_cloud_stamp_ns=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_100_000_000)),
        _sensor_map_received_at=0., _sensor_map_id='OcTree', _sensor_map_bytes=2,
        get_parameter=lambda _: SimpleNamespace(value=2.))
    message = Header(frame_id='camera')
    message.stamp.sec = 1
    NearestPregraspCoordinator._on_scene_cloud_receipt(node, message)
    NearestPregraspCoordinator._check_sensor_scene(node)
    node._sensor_map_bytes = 0
    with pytest.raises(RuntimeError):
        NearestPregraspCoordinator._check_sensor_scene(node)
