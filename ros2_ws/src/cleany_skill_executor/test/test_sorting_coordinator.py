from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest


@pytest.mark.parametrize('angle,valid', [(1.1, True), (6., False)])
def test_direct_vertical_lift_uses_verified_joint_endpoint(monkeypatch, angle, valid):
    import math
    from geometry_msgs.msg import Pose
    target, actual = Pose(), Pose()
    target.position.z = actual.position.z = .5
    target.orientation.w = 1.
    actual.orientation.x = math.sin(math.radians(angle)/2)
    actual.orientation.w = math.cos(math.radians(angle)/2)
    node = object.__new__(SortingCoordinator)
    node._direct_vertical_lift = True
    node._feedback_state = lambda: object()
    node._transport_adapter = SimpleNamespace(set_current_state=lambda _: None,
        solve_position_ik=lambda *args: SimpleNamespace(names=['joint'], positions=[.1]))
    node._tcp_pose = lambda *args: actual
    values = {'lin_position_tolerance_m': .001, 'corridor_orientation_tolerance_deg': 5.}
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    calls = []
    monkeypatch.setattr(NearestPregraspCoordinator, '_execute_linear',
        lambda self, arm, pose, label, **kwargs: calls.append((pose, kwargs)) or False)
    if valid:
        node._execute_linear('left', target, 'vertical grasp lift', velocity_scaling=.4)
        assert calls[0][0] is actual
        assert list(calls[0][1]['joint_target'].position) == [.1]
    else:
        with pytest.raises(ValueError, match='endpoint mismatch'):
            node._execute_linear('left', target, 'vertical grasp lift', velocity_scaling=.4)
        assert not calls


@pytest.mark.parametrize('destination', ['zone', 'handoff_center'])
def test_table_transport_uses_separate_lower_release_clearance(destination):
    centers = []
    obj = SimpleNamespace(obb_size=SimpleNamespace(x=.08, y=.08, z=.095),
                          obb_pose=SimpleNamespace(position=SimpleNamespace(x=.5, y=0.)))
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm='left',
                           selected_candidate=SimpleNamespace(target_object=obj)))
    zone = SimpleNamespace(kind='table_zone', center_xy=(.5, .45),
                           outside_size=(.7, .3, .3), top_z=.34)
    values = {'sorting_release_clearance_m': .06, 'sorting_table_release_clearance_m': .015}
    node = SimpleNamespace(_require_held_contact=lambda _: None, _bins={destination: zone},
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        _placed_footprints=[], _observed_footprints=[], _current={},
        _held_center_joints=lambda _, center, tolerance: centers.append(center) or object(),
        _move_to=lambda *args: None, _verify_feedback=lambda _: None)
    SortingCoordinator.transport(node, held, destination)
    assert centers[0][2] == pytest.approx(.34 + (.08**2+.08**2+.095**2)**.5/2 + .015)
    if destination == 'handoff_center':
        assert centers[0][:2] == pytest.approx(zone.center_xy)
    values['sorting_table_release_clearance_m'] = .01
    with pytest.raises(ValueError, match='10mm'):
        SortingCoordinator.transport(node, held, destination)
from cleany_interfaces.action import InspectScene
from cleany_interfaces.srv import ObserveObjectReference, ObserveWristTarget
from cleany_interfaces.msg import GraspCandidate
from rclpy.time import Time
from rclpy.serialization import deserialize_message

from cleany_skill_executor.sorting_coordinator import SortingCoordinator
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    LiftRedetectionError, NearestPregraspCoordinator,
)
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from cleany_skill_executor.core.grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.core.sorting import load_sorting_policy
from cleany_skill_executor.core.sorting import Category
from cleany_mujoco_sim.sorting_scene import load_bins


@pytest.mark.parametrize('old_stamp,fresh_stamp,category,error,refresh', [
    (90, 99, 'lost_item', None, False),
    (10, 99, 'lost_item', None, True),
    (0, 99, 'lost_item', 'Invalid head reference', False),
    (110, 99, 'lost_item', 'Invalid head reference', False),
    (10, 20, 'lost_item', 'expired during replanning', True),
    (10, 99, 'trash', 'classification changed', True),
    (10, 99, 'review', 'classification changed', True),
])
def test_aged_wrist_prior_is_reobserved_without_restamping_or_weakening_checks(
        old_stamp, fresh_stamp, category, error, refresh):
    def selection(stamp):
        return SimpleNamespace(selected_candidate=SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp, nanosec=0))))
    old, new = selection(old_stamp), selection(fresh_stamp)
    attempt = ObjectAttempt(1, 'wallet', .9, .6, 'lost_item', 'personal item')
    fresh = ObjectAttempt(2, 'wallet', .9, .6, category, 'fresh classification')
    events = []
    def reobserve(*args):
        assert args == (old, attempt)
        events.append('refresh')
        return new, fresh
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=30.),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100*10**9)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _switch_camera=lambda camera: events.append(camera),
        _refresh_selected_grasp=reobserve,
        _policy=SimpleNamespace(classify_model=lambda label, confidence, category, reason:
            SimpleNamespace(category=Category(category), destination=category)))
    if error:
        with pytest.raises(RuntimeError, match=error):
            SortingCoordinator._ensure_fresh_head_before_wrist(node, old, attempt)
    else:
        result = SortingCoordinator._ensure_fresh_head_before_wrist(node, old, attempt)
        assert result == ((new, fresh) if refresh else (old, attempt))
    assert events == (['head', 'refresh'] if refresh else [])
    assert old.selected_candidate.header.stamp.sec == old_stamp


@pytest.mark.parametrize('initial_contact, responses, expected', [
    (True, [], (True, .0)),
    (False, [True], (True, -.1)),
    (False, [False, True], (True, -.2)),
    (False, [False, False, False], (False, -.3)),
])
def test_bounded_contact_retry_preserves_limits_and_initial_open_reference(
    initial_contact, responses, expected,
):
    calls = []
    values = dict(gripper_contact_retry_step_rad=.1, gripper_close_position_rad=-.3,
                  gripper_contact_retry_motion_sec=1.2)
    def command(arm, position, operation, **kwargs):
        calls.append(position)
        assert arm == 'left' and operation == 'close'
        assert kwargs == dict(allow_contact_stall=True, motion_seconds=1.2, contact_start=1.4)
        assert position >= -.3
        return responses[len(calls)-1]
    node = SimpleNamespace(_gripper_retry_steps=5, _grasp_close_override=None,
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None), _command_gripper=command)
    contact, position = NearestPregraspCoordinator._retry_gripper_contact(
        node, 'left', 1.4, .0, initial_contact)
    assert contact == expected[0] and position == pytest.approx(expected[1])
    assert len(calls) == len(responses)
    if calls:
        assert NearestPregraspCoordinator._candidate_close_position(node, None) == position


def test_pipeline_artifacts_preserve_original_ros_result(tmp_path):
    node = SimpleNamespace(_artifact_directory=tmp_path, _artifact_sequence=0)
    result = InspectScene.Result(success=True, message='sensor result')
    for _ in range(2):
        SortingCoordinator._record_pipeline_message(node, 'inspection', result)
    paths = sorted(tmp_path.glob('*.cdr'))
    assert [p.name for p in paths] == ['001_inspection.cdr', '002_inspection.cdr']
    assert deserialize_message(paths[0].read_bytes(), InspectScene.Result) == result


def test_tcp_fk_uses_configured_timeout_and_joint_feedback():
    from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionFK
    response = GetPositionFK.Response()
    response.error_code.val = 1
    response.pose_stamped = [PoseStamped()]
    requests = []
    def future(token, timeout, label):
        assert timeout == 5.0 and label == 'left TCP FK'
        return response
    node = SimpleNamespace(
        _joint_positions={name: .1 for name in REQUIRED_JOINT_NAMES},
        _fk=SimpleNamespace(wait_for_service=lambda **_: True,
                            call_async=lambda request: requests.append(request)),
        get_parameter=lambda key: SimpleNamespace(value=5.0), _future=future)
    assert NearestPregraspCoordinator._tcp_pose(node, 'left') == response.pose_stamped[0].pose
    assert requests[0].fk_link_names == ['left_grasp_tcp']
    assert list(requests[0].robot_state.joint_state.position) == [.1] * len(REQUIRED_JOINT_NAMES)


def test_disabled_artifact_recording_does_not_serialize():
    node = SimpleNamespace(_artifact_directory=None, _artifact_sequence=0)
    SortingCoordinator._record_pipeline_message(node, 'inspection', object())
    assert node._artifact_sequence == 0


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_pregrasp_opens_only_selected_gripper_in_same_checked_motion(monkeypatch, arm):
    candidate = GraspCandidate(snapshot_id='original')
    selected = SimpleNamespace(selected_arm=arm, selected_candidate=candidate,
        pregrasp_joint_state=JointState(name=list(ARM_JOINT_NAMES[arm]), position=[0.]*5),
        grasp_joint_state=JointState(name=list(ARM_JOINT_NAMES[arm]), position=[.1]*5))
    events = []
    def move(selected_arm, state, label):
        assert selected_arm == arm
        assert state.name == [*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint']
        assert list(state.position) == [0.]*5 + [1.4]
        events.append('combined_move')
    node = SimpleNamespace(get_parameter=lambda _: SimpleNamespace(value=1.4),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        _execution_scene=SimpleNamespace(begin=lambda *_: events.append('scene'),
            disallow_target_contacts=lambda: events.append('forbid_contact'), restore=lambda: events.append('restore')),
        _move_to=move, _verify_feedback=lambda state: events.append(('feedback', len(state.name))))
    # Use a real subclass instance so the super() scene transaction is exercised.
    actual = object.__new__(SortingCoordinator)
    actual.__dict__.update(node.__dict__)
    actual._execute_pregrasp(selected, SimpleNamespace(object_id=1))
    assert events == ['scene', 'forbid_contact', 'combined_move', ('feedback', 6)]
    assert len(selected.pregrasp_joint_state.name) == len(selected.grasp_joint_state.name) == 5


@pytest.mark.parametrize('pending', [[], ['lego brick']])
def test_sorting_start_does_not_send_initial_gripper_commands(pending):
    events = []
    values = dict(sorting_use_reference_observation=False, sorting_maximum_objects=2,
                  sorting_required_categories=[], sorting_empty_confirmations=2)
    node = SimpleNamespace(_wrist_enabled=False, _completed=[], _pending_handoffs=pending,
        _stage=events.append, _wait_for_pipeline=lambda: None,
        _verification=SimpleNamespace(wait_for_service=lambda **k: True),
        _register_bins=lambda: None, _arm_joint_state=lambda _: JointState(),
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        _detect_objects=lambda: SimpleNamespace(detections=SimpleNamespace(detections=[])),
        _attempts=lambda _: [],
        _record_pipeline_message=lambda *_: None,
        _prepare_gripper=lambda *_: pytest.fail('Initial opening removed'),
        _open_gripper=lambda *_: pytest.fail('Initial opening removed'))
    if pending:
        with pytest.raises(RuntimeError,match='Pending handoff'):
            SortingCoordinator.run(node)
        assert events == ['starting','search']
        return
    SortingCoordinator.run(node)
    assert events == ['starting', 'search', 'empty_confirmation', 'empty_confirmation', 'mission_complete']


def test_visible_unclassified_object_cannot_be_reported_as_clear():
    events = []
    root = Path(__file__).parents[2]
    obj = SimpleNamespace(obb_pose=Pose(),obb_size=SimpleNamespace(x=.03,y=.02,z=.01))
    obj.obb_pose.position.x, obj.obb_pose.position.z = .4,.36
    obj.obb_pose.orientation.w = 1.
    attempt = ObjectAttempt(1,'cup',.95,.5)
    values = dict(sorting_use_reference_observation=False,sorting_maximum_objects=2,sorting_empty_confirmations=2)
    node = SimpleNamespace(_wrist_enabled=False,_completed=[],
        _stage=events.append,_wait_for_pipeline=lambda:None,
        _verification=SimpleNamespace(wait_for_service=lambda **k:True),
        _register_bins=lambda:None,_arm_joint_state=lambda _:JointState(),
        get_parameter=lambda k:SimpleNamespace(value=values[k]),
        _detect_objects=lambda:SimpleNamespace(detections=SimpleNamespace(detections=[object()],snapshot_id='fresh')),
        _attempts=lambda _:[attempt],_record_pipeline_message=lambda *_:None,
        _inspect_selected=lambda *_:SimpleNamespace(objects=SimpleNamespace(objects=[obj])),
        _policy=load_sorting_policy(root/'cleany_skill_executor/config/table_sorting_policy.yaml'),
        _bins={b.name:b for b in load_bins(root/'cleany_mujoco_sim/config/robot_top_bins.yaml')})
    with pytest.raises(RuntimeError,match='1 unresolved'):
        SortingCoordinator.run(node)
    assert events == ['starting','search','review']


def test_missing_previously_seen_item_is_not_an_empty_workspace(monkeypatch):
    root=Path(__file__).parents[2]
    attempts=[ObjectAttempt(1,'cup',.95,.5,'trash','Disposable'),
              ObjectAttempt(2,'wallet',.95,.6,'lost_item','Personal belonging')]
    detections=iter([attempts,[]])
    obj=SimpleNamespace(obb_pose=Pose(),obb_size=SimpleNamespace(x=.03,y=.02,z=.01))
    obj.obb_pose.position.x,obj.obb_pose.position.z=.4,.36
    obj.obb_pose.orientation.w=1.
    values=dict(sorting_use_reference_observation=False,sorting_maximum_objects=2,sorting_empty_confirmations=2)
    node=SimpleNamespace(_wrist_enabled=False,_completed=[],_pending_handoffs=[],
        _stage=lambda *_:None,_wait_for_pipeline=lambda:None,
        _verification=SimpleNamespace(wait_for_service=lambda **k:True),
        _register_bins=lambda:None,_arm_joint_state=lambda _:JointState(),
        get_parameter=lambda k:SimpleNamespace(value=values[k]),
        _detect_objects=lambda:SimpleNamespace(detections=SimpleNamespace(detections=next(detections),snapshot_id='fresh')),
        _attempts=lambda items:items,_record_pipeline_message=lambda *_:None,
        _inspect_selected=lambda *_:SimpleNamespace(objects=SimpleNamespace(objects=[obj])),
        _plan_grasps=lambda *_:SimpleNamespace(candidates=[]),_select_reachable=lambda *a,**k:object(),
        _policy=load_sorting_policy(root/'cleany_skill_executor/config/table_sorting_policy.yaml'),
        _bins={b.name:b for b in load_bins(root/'cleany_mujoco_sim/config/robot_top_bins.yaml')})
    monkeypatch.setattr('cleany_skill_executor.sorting_coordinator.execute_sort',lambda *_:True)
    with pytest.raises(RuntimeError,match='Previously observed'):
        SortingCoordinator.run(node)
    assert len(node._completed)==1


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('label,group', [('pregrasp', 'pregrasp_open'), ('return from trash_left', 'return_close')])
def test_combined_pregrasp_goal_contains_arm_and_jaw_without_relaxing_limits(monkeypatch, arm, label, group):
    def base(*_):
        goal = MoveGroup.Goal()
        goal.request.max_velocity_scaling_factor = .08
        goal.request.max_acceleration_scaling_factor = .08
        return goal
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', base)
    node = object.__new__(SortingCoordinator)
    node._held_object = None
    joints = JointState(name=[*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint'], position=[0.]*5 + [1.4])
    goal = SortingCoordinator._execution_goal(node, arm, joints, label)
    assert goal.request.group_name == f'{arm}_{group}'
    assert goal.request.max_velocity_scaling_factor == .08
    assert goal.request.max_acceleration_scaling_factor == .08
    node._held_object = object()
    with pytest.raises(ValueError, match='empty arm'):
        SortingCoordinator._execution_goal(node, arm, joints, label)


@pytest.mark.parametrize('reference_enabled', [False, True])
def test_pick_does_not_reopen_prepared_gripper(reference_enabled):
    events = []
    candidate = SimpleNamespace(snapshot_id='fresh', target_object=SimpleNamespace(object_id=1))
    selected = SimpleNamespace(selected_arm='left', selected_candidate=candidate)
    target = SimpleNamespace(selected=selected, attempt=object())
    held = SimpleNamespace(target=None)

    def refresh(*_):
        events.append('refresh')
        return selected, target.attempt

    def pin(request):
        assert request.operation == ObserveObjectReference.Request.PIN
        assert request.source_snapshot_id == 'fresh'
        events.append('pin')
        return object()

    def execute(*_):
        events.append('grasp_and_lift')
        node._held_object = held

    node = SimpleNamespace(
        _execute_pregrasp=lambda *_: events.append('pregrasp'),
        _open_gripper=lambda *_: pytest.fail('duplicate gripper opening'),
        _refresh_selected_grasp=refresh,
        get_parameter=lambda _: SimpleNamespace(value=reference_enabled),
        _reference_request=pin,
        _execute_grasp_and_lift=execute,
    )
    assert SortingCoordinator.pick(node, target) is held
    assert held.target is target
    assert events == ['pregrasp', 'refresh'] + (
        ['pin'] if reference_enabled else []) + ['grasp_and_lift']


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('handoff_ok', [True, False])
def test_wrist_pick_switches_only_after_pregrasp_and_stops_on_failed_handoff(arm, handoff_ok):
    events = []
    selected = SimpleNamespace(selected_arm=arm)
    target = SimpleNamespace(selected=selected, attempt=object())
    held = SimpleNamespace(target=None)
    def observe(selection, operation):
        assert selection is selected and operation == ObserveWristTarget.Request.HANDOFF
        events.append('handoff')
        if not handoff_ok:
            raise RuntimeError('handoff failed')
        return object()
    def grasp(selection, attempt):
        assert selection is selected and attempt is target.attempt
        events.append('grasp')
        node._held_object = held
    node = SimpleNamespace(_wrist_enabled=True,
        _ensure_fresh_head_before_wrist=lambda selected, attempt: (selected, attempt),
        _execute_pregrasp=lambda *_: events.append('pregrasp'),
        _wait_arm_stationary=lambda a: events.append(f'stationary:{a}'),
        _switch_camera=lambda a: events.append(f'camera:{a}'),
        _observe_wrist=observe, _execute_grasp_and_lift=grasp,
        _execution_scene=SimpleNamespace(allow_contacts_for=lambda a: events.append(f'contacts:{a}')),
        _refresh_selected_grasp=lambda *_: pytest.fail('Unexpected head reinspection'),
        _open_gripper=lambda *_: pytest.fail('Duplicate gripper opening'))
    if handoff_ok:
        assert SortingCoordinator.pick(node, target) is held
    else:
        with pytest.raises(RuntimeError, match='handoff failed'):
            SortingCoordinator.pick(node, target)
    assert events == ['pregrasp', f'stationary:{arm}', f'camera:{arm}', 'handoff'] + (
        [f'contacts:{arm}', 'grasp'] if handoff_ok else [])


@pytest.mark.parametrize('fault', ['', 'source', 'frame', 'stamp', 'reference'])
@pytest.mark.parametrize('operation', [ObserveWristTarget.Request.HANDOFF, ObserveWristTarget.Request.CHECK])
def test_wrist_response_must_preserve_source_arm_and_fresh_timestamp(fault, operation):
    candidate = GraspCandidate(snapshot_id='head-1')
    candidate.header.frame_id = 'base_link'
    candidate.header.stamp.sec = 1
    candidate.target_object.object_id = 1
    candidate.target_object.label = 'cup'
    candidate.target_object.confidence = .8
    selected = SimpleNamespace(selected_arm='right', selected_candidate=candidate)
    response = ObserveWristTarget.Response(success=True, reference_id='wrist-1',
        source_snapshot_id='head-1', source_object_id=1)
    response.header.frame_id = 'right_wrist_rgb_optical_frame'
    response.header.stamp.sec = 3
    if fault == 'source': response.source_object_id = 2
    if fault == 'frame': response.header.frame_id = 'left_wrist_rgb_optical_frame'
    if fault == 'stamp': response.header.stamp.sec = 2
    if fault == 'reference': response.reference_id = ''
    def request(req):
        assert req.expected_pose.header == candidate.header
        assert req.after_stamp_ns == 2_000_000_000 and req.arm == 'right'
        return response
    node = SimpleNamespace(_wrist_reference=(None if operation == 0 else SimpleNamespace(reference_id='wrist-1')),
        _lift_completion_stamp_ns=2_000_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=2 if operation == 0 else 5)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        _record_pipeline_message=lambda *_: None,
        _wrist_client=SimpleNamespace(call_async=request),
        _future=lambda result, *_: result)
    if fault:
        with pytest.raises(RuntimeError):
            SortingCoordinator._observe_wrist(node, selected, operation)
    else:
        assert SortingCoordinator._observe_wrist(node, selected, operation) is response


@pytest.mark.parametrize('completed', [None, 0, 6_000_000_000])
def test_wrist_check_rejects_missing_or_future_lift_barrier(completed):
    selected = SimpleNamespace(selected_arm='left', selected_candidate=GraspCandidate())
    node = SimpleNamespace(_wrist_reference=None, _lift_completion_stamp_ns=completed,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=5)))
    with pytest.raises(RuntimeError, match='freshness barrier'):
        SortingCoordinator._observe_wrist(node, selected, ObserveWristTarget.Request.CHECK)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_wrist_retreat_restores_head_only_after_feedback(arm):
    events = []
    home = JointState(name=list(ARM_JOINT_NAMES[arm]), position=[.1]*5)
    def move(selected_arm, joints, label):
        assert selected_arm == arm
        assert joints.name == [*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint']
        assert list(joints.position) == [.1]*5 + [-.3]
        assert label == 'return from trash_left'
        events.append('return')
    node = SimpleNamespace(_wrist_enabled=True, _home={arm: home}, _held_object=None,
        get_parameter=lambda _: SimpleNamespace(value=-.3),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        _move_to=move,
        _verify_feedback=lambda *_: events.append('feedback'),
        _switch_camera=lambda camera: events.append(camera))
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm=arm))
    SortingCoordinator.retreat(node, held, 'trash_left')
    assert events == ['return', 'feedback', 'head']
    assert len(home.name) == 5


@pytest.mark.parametrize('arm,y,destination', [
    ('left', .49, 'trash_left'), ('right', -.49, 'lost_items_right')])
def test_transport_uses_fixed_base_bin_without_camera_inference(arm, y, destination):
    size = SimpleNamespace(x=.04, y=.04, z=.04)
    held = SimpleNamespace(offset_in_tcp=np.zeros(3), selected=SimpleNamespace(selected_arm=arm,
        selected_candidate=SimpleNamespace(target_object=SimpleNamespace(obb_size=size))))
    center = np.array([.08, y, .26 + np.linalg.norm([.04]*3)/2 + .06])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.w = 1.
    events = []
    def solve(selected_arm, target, seed):
        assert selected_arm == arm
        np.testing.assert_allclose(target, center)
        events.append('ik')
        return SimpleNamespace(names=(f'{arm}_shoulder_yaw_joint',), positions=(0.,))
    node = SimpleNamespace(
        _bins={destination: SimpleNamespace(center_xy=(.08, y), top_z=.26, kind='bin')},
        _require_held_contact=lambda *_: events.append('contact'),
        get_parameter=lambda _: SimpleNamespace(value=.06),
        _tcp_pose=lambda *_: pose, _feedback_state=lambda: object(),
        _pose_position=NearestPregraspCoordinator._pose_position,
        _transport_adapter=SimpleNamespace(set_current_state=lambda *_: None,
            solve_position_ik=solve, state_is_valid=lambda *_: True),
        _move_to=lambda *_: events.append('move'),
        _verify_feedback=lambda *_: events.append('feedback'),
        _camera_info=None,
        _observe_wrist=lambda *_: pytest.fail('Transport must not wait for bin image'),
        _inspect_selected=lambda *_: pytest.fail('Transport must not redetect the bin'))
    SortingCoordinator.transport(node, held, destination)
    assert events == ['contact', 'ik', 'move', 'feedback', 'contact']


def test_contact_offset_uses_actual_tcp_not_commanded_endpoint():
    tcp = Pose()
    tcp.orientation.w = 1.
    tcp.position.x, tcp.position.z = .4, .5
    obb = Pose()
    obb.position.x, obb.position.z = .38, .46
    selected = SimpleNamespace(selected_arm='left', selected_candidate=SimpleNamespace(
        target_object=SimpleNamespace(obb_pose=obb)))
    node = SimpleNamespace(_tcp_pose=lambda arm: tcp,
                           _pose_position=NearestPregraspCoordinator._pose_position)
    SortingCoordinator._on_grasp_contact(node, selected, object())
    np.testing.assert_allclose(node._held_object.offset_in_tcp, (-.02, 0., -.04))


@pytest.mark.parametrize('label,acceleration', [('refreshed grasp approach', 1.), ('refreshed retreat', .4)])
def test_acceleration_increase_is_only_for_empty_grasp_approach(monkeypatch, label, acceleration):
    def base(*_):
        goal = MoveGroup.Goal()
        goal.request.max_acceleration_scaling_factor = .4
        return goal
    monkeypatch.setattr(NearestPregraspCoordinator, '_joint_corridor_goal', base)
    node = object.__new__(SortingCoordinator)
    node.get_parameter = lambda _: SimpleNamespace(value=1.)
    goal = node._joint_corridor_goal('left', None, None, None, label, .7)
    assert goal.request.max_acceleration_scaling_factor == acceleration


def test_async_lift_hold_is_removed_but_other_holds_remain(monkeypatch):
    events = []
    monkeypatch.setattr(NearestPregraspCoordinator, '_hold', lambda _, p: events.append(p))
    node = object.__new__(SortingCoordinator)
    node._async_carry = True
    node._check_motion_guard = lambda: events.append('guard')
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    node._hold('lift_hold_sec')
    node._hold('grasp_settle_sec')
    assert events == ['guard', 'grasp_settle_sec']


def test_return_cannot_close_jaw_before_release_completed():
    node = SimpleNamespace(_held_object=object())
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm='left'))
    with pytest.raises(RuntimeError, match='before release completes'):
        SortingCoordinator.retreat(node, held, 'trash_left')


def test_guarded_payload_disables_moveit_internal_restart(monkeypatch):
    def base(*_):
        goal = MoveGroup.Goal()
        goal.planning_options.replan = True
        goal.request.max_velocity_scaling_factor = .08
        goal.request.max_acceleration_scaling_factor = .08
        return goal
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', base)
    node = object.__new__(SortingCoordinator)
    node._async_carry, node._held_object = True, object()
    node.get_parameter = lambda _: SimpleNamespace(value=.02)
    goal = node._execution_goal('left', JointState(name=list(ARM_JOINT_NAMES['left']), position=[0.]*5), 'carry')
    assert not goal.planning_options.replan
    assert goal.request.max_acceleration_scaling_factor == .02


def test_async_lift_keeps_geometry_and_contact_but_never_calls_wrist_check():
    events = []
    pose = Pose()
    pose.orientation.w, pose.position.z = 1., .5
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm='left'), offset_in_tcp=np.zeros(3))
    node = SimpleNamespace(_wrist_enabled=True, _async_carry=True, _held_object=held,
        _require_held_contact=lambda _: events.append('contact'),
        _tcp_pose=lambda _: pose, _pose_position=NearestPregraspCoordinator._pose_position,
        _check_motion_guard=lambda: events.append('guard'),
        _observe_wrist=lambda *a, **k: pytest.fail('No blocking CHECK in async carry'),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    SortingCoordinator._verify_lift_height(node, object(), minimum_center_z_m=.4)
    assert events == ['contact', 'guard']
    with pytest.raises(RuntimeError, match='clearance'):
        SortingCoordinator._verify_lift_height(node, object(), minimum_center_z_m=.6)


def test_expected_release_disarms_only_after_bin_and_guard_checks():
    events = []
    pose = Pose()
    pose.orientation.w, pose.position.y, pose.position.z = 1., .49, .4
    held = SimpleNamespace(offset_in_tcp=np.zeros(3), selected=SimpleNamespace(selected_arm='left',
        selected_candidate=SimpleNamespace(target_object=SimpleNamespace(
            obb_size=SimpleNamespace(x=.04, y=.04, z=.04)))))
    node = SimpleNamespace(_async_carry=True, _wrist_enabled=False, _held_object=held,
        _pinned_reference=None, _tcp_pose=lambda _: pose,
        _pose_position=NearestPregraspCoordinator._pose_position,
        _bins={'bin': SimpleNamespace(center_xy=(0., .49), outside_size=(.26, .24, .22), wall=.008, top_z=.26, kind='bin')},
        _check_motion_guard=lambda: events.append('guard'),
        _carry_guard=SimpleNamespace(disarm=lambda: events.append('disarm')),
        _open_gripper=lambda _: events.append('open'),
        _hold=lambda _: events.append('settle'),
        _execution_scene=SimpleNamespace(restore=lambda: events.append('detach')),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    SortingCoordinator.release(node, held, 'bin')
    assert events == ['guard', 'disarm', 'open', 'settle', 'detach']
    assert node._held_object is None
    events.clear()
    pose.position.z = .2
    with pytest.raises(RuntimeError, match='bin opening'):
        SortingCoordinator.release(node, held, 'bin')
    assert events == []


@pytest.mark.parametrize('age', [.2, 2.])
def test_carry_monitor_requires_fresh_gripper_feedback_without_waiting(monkeypatch, age):
    events = []
    monkeypatch.setattr('cleany_skill_executor.sorting_coordinator.time.monotonic', lambda: 20.)
    values = dict(sorting_joint_feedback_max_age_sec=1., gripper_open_position_rad=1.4)
    node = SimpleNamespace(_async_carry=True,
        _carry_guard=SimpleNamespace(armed=True, check=lambda *a: events.append('tracking')),
        _held_object=SimpleNamespace(selected=SimpleNamespace(selected_arm='left', selected_candidate=None)),
        _carry_joint_received={'left_gripper_joint': 20.-age},
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)),
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        _candidate_close_position=lambda _: .6,
        _gripper_contact_stalled=lambda *a, allow_closing_motion: allow_closing_motion,
        _contact_loss_guard=SimpleNamespace(check=lambda *a: events.append('contact')))
    if age < 1.:
        SortingCoordinator._check_motion_guard(node)
        assert events == ['tracking', 'contact']
    else:
        with pytest.raises(RuntimeError, match='feedback stale'):
            SortingCoordinator._check_motion_guard(node)
        assert events == ['tracking']


def recovery_node():
    node = object.__new__(SortingCoordinator)
    values = dict(sorting_reobserve_after_lift=True, sorting_held_association_tolerance_m=.03,
                  sorting_use_reference_observation=False)
    node.get_parameter = lambda name: SimpleNamespace(value=values[name])
    pose = Pose()
    pose.orientation.w = 1.
    pose.position.z = .5
    node._tcp_pose = lambda arm: pose
    node._held_object = SimpleNamespace(offset_in_tcp=np.zeros(3), selected=SimpleNamespace(
        selected_arm='left', selected_candidate=SimpleNamespace(header=SimpleNamespace(frame_id='base_link'))))
    node._record_pipeline_message = lambda *_: None
    return node, values, SimpleNamespace(objects=SimpleNamespace(
        header=SimpleNamespace(frame_id='base_link'), objects=[SimpleNamespace(obb_pose=pose)]))


def test_missing_detection_gets_one_move_then_same_height_verification(monkeypatch):
    node, _, inspected = recovery_node()
    events = []
    def verify(_self, attempt, *, minimum_center_z_m):
        events.append(('verify', minimum_center_z_m))
        if len(events) == 1:
            raise LiftRedetectionError('missing')
        return inspected
    monkeypatch.setattr(NearestPregraspCoordinator, '_verify_lift_height', verify)
    node._reobserve_held_object = lambda minimum: events.append(('move', minimum))
    assert node._verify_lift_height(object(), minimum_center_z_m=.46) is inspected
    assert events == [('verify', .46), ('move', .46), ('verify', .46)]


@pytest.mark.parametrize('fault', ['', 'identity', 'stale', 'height', 'contact', 'confidence'])
def test_reference_height_keeps_provenance_height_and_contact_gates(fault):
    node, values, _ = recovery_node()
    values['lift_min_center_z_m'] = .38
    response = ObserveObjectReference.Response(success=True, reference_id='pinned',
        source_snapshot_id='source', source_object_id=1, source_capture_stamp_ns=100,
        source_label='cup', source_confidence=.315, valid_depth_points=300)
    response.header.stamp = Time(nanoseconds=300).to_msg()
    response.header.frame_id = 'base_link'
    response.observed_center.z = .5
    node._pinned_reference = ObserveObjectReference.Response(success=True, reference_id='pinned',
        source_snapshot_id='source', source_object_id=1, source_capture_stamp_ns=100,
        source_label='cup', source_confidence=.315)
    if fault == 'identity':
        response.source_object_id = 2
    elif fault == 'stale':
        response.header.stamp = Time(nanoseconds=200).to_msg()
    elif fault == 'height':
        response.observed_center.z = .4
    elif fault == 'confidence':
        response.source_confidence = .9
    events = []
    node._wait_arm_stationary = lambda arm: events.append('stationary')
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=200))
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    def request(message):
        assert message.operation == message.OBSERVE and message.reference_id == 'pinned'
        assert message.after_stamp_ns == 200
        events.append('observe')
        return response
    node._reference_request = request
    def contact(held):
        events.append('contact')
        if fault == 'contact':
            raise RuntimeError('lost contact')
    node._require_held_contact = contact
    if fault:
        with pytest.raises(RuntimeError):
            node._verify_reference_height(SimpleNamespace(label='cup'), minimum_center_z_m=.46)
    else:
        assert node._verify_reference_height(SimpleNamespace(label='cup'), minimum_center_z_m=.46) is response
        assert events == ['stationary', 'observe', 'contact']


def test_reference_mask_failure_gets_only_one_physical_reobservation():
    node, values, _ = recovery_node()
    values['sorting_use_reference_observation'] = True
    calls = []
    def missing(*args, **kwargs):
        calls.append('observe')
        raise LiftRedetectionError('clipped mask')
    node._verify_reference_height = missing
    node._reobserve_held_object = lambda minimum: calls.append(('move', minimum))
    with pytest.raises(LiftRedetectionError):
        node._verify_lift_height(SimpleNamespace(label='cup'), minimum_center_z_m=.46)
    assert calls == ['observe', ('move', .46), 'observe']


@pytest.mark.parametrize('failure', [RuntimeError('object fell'), ValueError('invalid height')])
def test_recovery_does_not_swallow_non_detection_failures(monkeypatch, failure):
    node, _, _ = recovery_node()
    moves = []
    def verify(*_, **kwargs):
        raise failure
    monkeypatch.setattr(NearestPregraspCoordinator, '_verify_lift_height', verify)
    node._reobserve_held_object = moves.append
    with pytest.raises(type(failure), match=str(failure)):
        node._verify_lift_height(object(), minimum_center_z_m=.46)
    assert moves == []


@pytest.mark.parametrize('enabled', [True, False])
def test_failed_reobservation_cannot_retry_forever_or_claim_lift(monkeypatch, enabled):
    node, values, _ = recovery_node()
    values['sorting_reobserve_after_lift'] = enabled
    moves = []
    def verify(*_, **kwargs):
        raise LiftRedetectionError('missing')
    monkeypatch.setattr(NearestPregraspCoordinator, '_verify_lift_height', verify)
    node._reobserve_held_object = moves.append
    with pytest.raises(LiftRedetectionError):
        node._verify_lift_height(object(), minimum_center_z_m=.46)
    assert moves == ([.46] if enabled else [])


def test_distant_same_label_observation_is_not_proof_of_held_object(monkeypatch):
    node, _, inspected = recovery_node()
    node._held_object.offset_in_tcp = np.array((.1, 0., 0.))
    monkeypatch.setattr(NearestPregraspCoordinator, '_verify_lift_height', lambda *a, **k: inspected)
    with pytest.raises(RuntimeError, match='does not match held geometry'):
        node._verify_lift_height(object(), minimum_center_z_m=.46)


@pytest.mark.parametrize('valid_ik', [True, False])
@pytest.mark.parametrize('contact_after', [True, False])
def test_reobservation_checks_scene_ik_execution_and_contact(valid_ik, contact_after):
    node, values, _ = recovery_node()
    values.update(sorting_reobserve_geometry_padding_m=.01, lift_min_center_z_m=.38,
                  sorting_reobserve_margin_px=24., sorting_reobserve_max_translation_m=.4,
                  sorting_reobserve_max_lowering_m=0., sorting_reobserve_height_clearance_m=.02,
                  require_sensor_scene=True, attachment_scene_timeout_sec=5.)
    node._held_object.selected.selected_candidate.target_object = SimpleNamespace(
        obb_size=SimpleNamespace(x=.04, y=.04, z=.04))
    node._camera_info = SimpleNamespace(width=640, height=480,
        k=[500., 0., 319.5, 0., 500., 239.5, 0., 0., 1.],
        header=SimpleNamespace(frame_id='camera_optical'))
    node._tf_buffer = SimpleNamespace(lookup_transform=lambda base, optical, stamp:
        SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=.2, y=0., z=.8),
            rotation=SimpleNamespace(x=1., y=0., z=0., w=0.))))
    events = []
    def contact(_):
        if not contact_after and events:
            raise RuntimeError('lost contact')
        events.append('contact')
    node._require_held_contact = contact
    node._wait_arm_stationary = lambda _: events.append('stationary')
    node._wait_for_sensor_scene = lambda _: events.append('scene')
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    def ik(*_):
        events.append('ik')
        return object() if valid_ik else None
    node._held_center_joints = ik
    node._move_to = lambda *_: events.append('execute')
    node._verify_feedback = lambda _: events.append('feedback')
    node._hold = lambda _: events.append('hold')
    if not valid_ik or not contact_after:
        with pytest.raises(RuntimeError, match='No collision-valid|lost contact'):
            node._reobserve_held_object(.46)
    else:
        node._reobserve_held_object(.46)
    if valid_ik:
        assert events == ['contact', 'stationary', 'scene', 'ik', 'execute', 'feedback', 'hold'] + (
            ['contact'] if contact_after else [])
    else:
        assert events == ['contact', 'stationary', 'scene', 'ik', 'ik', 'ik']


@pytest.mark.parametrize('holding', [True, False])
@pytest.mark.parametrize('requested', [.01, .5])
def test_payload_scaling_only_tightens_joint_transit_and_preserves_goal(monkeypatch, holding, requested):
    node, values, _ = recovery_node()
    if not holding:
        node._held_object = None
    values.update(sorting_payload_velocity_scaling=requested,
                  sorting_payload_acceleration_scaling=requested)
    def base(*_):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'left_grasp_arm'
        goal.request.max_velocity_scaling_factor = .08
        goal.request.max_acceleration_scaling_factor = .05
        goal.request.start_state.is_diff = True
        return goal
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', base)
    goal = node._execution_goal('left', JointState(), 'payload move')
    assert goal.request.group_name == 'left_grasp_arm'
    assert goal.request.start_state.is_diff
    assert not goal.planning_options.plan_only
    assert goal.request.max_velocity_scaling_factor == (min(.08, requested) if holding else .08)
    assert goal.request.max_acceleration_scaling_factor == (min(.05, requested) if holding else .05)


@pytest.mark.parametrize('requested', [0., -1., 1.1, float('nan'), float('inf')])
def test_invalid_payload_scaling_cannot_reach_execution(monkeypatch, requested):
    node, values, _ = recovery_node()
    values.update(sorting_payload_velocity_scaling=requested,
                  sorting_payload_acceleration_scaling=.01)
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', lambda *_: MoveGroup.Goal())
    with pytest.raises(ValueError, match='Payload motion scaling'):
        node._execution_goal('left', JointState(), 'payload move')
