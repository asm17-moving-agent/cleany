from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
import numpy as np
import pytest
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import JointState

from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import GraspCandidate
from cleany_interfaces.srv import ObserveObjectReference, ObserveWristTarget
from cleany_mujoco_sim.sorting_scene import load_bins
from cleany_skill_executor.core.grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.core.sorting import Category, load_sorting_policy
from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    LiftRedetectionError,
    NearestPregraspCoordinator,
)
from cleany_skill_executor.sorting_coordinator import SortingCoordinator


@pytest.mark.parametrize('wrist', [True, False])
@pytest.mark.parametrize('fails', [True, False])
def test_grasp_depth_boost_is_scoped_and_restored(monkeypatch, wrist, fails):
    node = object.__new__(SortingCoordinator)
    node._wrist_enabled = wrist
    events = []
    node._set_head_depth_boost = lambda enabled: events.append(enabled)
    def execute(self, selected, attempt):
        events.append('grasp-and-lift')
        if fails:
            raise RuntimeError('stale depth')
    monkeypatch.setattr(NearestPregraspCoordinator, '_execute_grasp_and_lift', execute)
    if fails:
        with pytest.raises(RuntimeError, match='stale depth'):
            node._execute_grasp_and_lift(None, None)
    else:
        node._execute_grasp_and_lift(None, None)
    assert events == ([True, 'grasp-and-lift', False] if wrist else ['grasp-and-lift'])


def test_disabled_age_refresh_does_not_access_clock_or_reinvoke_detector():
    node = SimpleNamespace(get_parameter=lambda _: SimpleNamespace(value=0.0))
    selected, attempt = object(), object()
    assert SortingCoordinator._ensure_fresh_head_before_wrist(node, selected, attempt) == (selected, attempt)


def test_return_detection_is_consumed_once_without_duplicate_request():
    events = []
    node = SimpleNamespace(
        _return_detection=('handle', 'future'),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _finish_object_detection=lambda pending: events.append(pending) or 'prefetched',
        _detect_objects=lambda: events.append('fresh') or 'fresh')
    assert SortingCoordinator._next_sorting_detection(node) == 'prefetched'
    assert node._return_detection is None
    assert SortingCoordinator._next_sorting_detection(node) == 'fresh'
    assert events == [('handle', 'future'), 'fresh']


def test_grasp_does_not_start_if_depth_boost_is_rejected(monkeypatch):
    node = object.__new__(SortingCoordinator)
    node._wrist_enabled = True
    def rejected(enabled):
        raise RuntimeError('Head depth refresh request rejected')
    node._set_head_depth_boost = rejected
    calls = []
    monkeypatch.setattr(NearestPregraspCoordinator, '_execute_grasp_and_lift',
                        lambda *args: calls.append(True))
    with pytest.raises(RuntimeError, match='rejected'):
        node._execute_grasp_and_lift(None, None)
    assert not calls


def test_depth_cleanup_failure_preserves_original_motion_error(monkeypatch):
    node = object.__new__(SortingCoordinator)
    node._wrist_enabled = True
    errors = []
    node.get_logger = lambda: SimpleNamespace(error=errors.append)

    def boost(enabled):
        if not enabled:
            raise RuntimeError('camera service lost')

    def execute(*args):
        raise RuntimeError('payload contact lost')

    node._set_head_depth_boost = boost
    monkeypatch.setattr(NearestPregraspCoordinator, '_execute_grasp_and_lift', execute)
    with pytest.raises(RuntimeError, match='payload contact lost'):
        node._execute_grasp_and_lift(None, None)
    assert len(errors) == 1 and 'camera service lost' in errors[0]


@pytest.mark.parametrize('accepted', [True, False])
def test_depth_boost_parameter_does_not_switch_active_camera(accepted):
    requests = []
    node = SimpleNamespace(
        _camera_client=SimpleNamespace(wait_for_service=lambda **kwargs: True,
            call_async=lambda request: requests.append(request)),
        _future=lambda *args: SimpleNamespace(results=[SimpleNamespace(successful=accepted)]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    if accepted:
        SortingCoordinator._set_head_depth_boost(node, True)
    else:
        with pytest.raises(RuntimeError, match='rejected'):
            SortingCoordinator._set_head_depth_boost(node, True)
    assert len(requests[0].parameters) == 1
    assert requests[0].parameters[0].name == 'head_depth_boost'
    assert requests[0].parameters[0].value.bool_value is True


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


@pytest.mark.parametrize('requested', [.12, 0., -1., 1.1, float('nan')])
def test_return_scaling_caps_only_return_and_rejects_invalid(monkeypatch, requested):
    def base(*_):
        goal = MoveGroup.Goal()
        goal.request.max_velocity_scaling_factor = .30
        goal.request.max_acceleration_scaling_factor = .50
        return goal
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', base)
    node = object.__new__(SortingCoordinator)
    node._held_object = None
    node.get_parameter = lambda name: SimpleNamespace(value=requested)
    if requested == .12:
        goal = node._execution_goal('left', JointState(), 'return from lost_items_left')
        assert goal.request.max_velocity_scaling_factor == .12
        assert goal.request.max_acceleration_scaling_factor == .12
    else:
        with pytest.raises(ValueError, match='Return motion scaling'):
            node._execution_goal('left', JointState(), 'return from lost_items_left')
    goal = node._execution_goal('left', JointState(), 'pregrasp')
    assert goal.request.max_velocity_scaling_factor == .30
    assert goal.request.max_acceleration_scaling_factor == .50


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


def sorting_loop(attempts=(), *, maximum=2, stage=None, **overrides):
    """One loop harness; each scenario supplies only its observations and effects."""
    root = Path(__file__).parents[2]
    values = dict(sorting_use_reference_observation=False, sorting_maximum_objects=maximum,
                  sorting_required_categories=[], sorting_empty_confirmations=2)
    node = SimpleNamespace(
        _wrist_enabled=False, _completed=[], _pending_handoffs=[],
        _stage=stage or (lambda *_: None), _wait_for_pipeline=lambda: None,
        _verification=SimpleNamespace(wait_for_service=lambda **_: True),
        _register_bins=lambda: None, _arm_joint_state=lambda _: JointState(),
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        _next_sorting_detection=lambda: SimpleNamespace(detections=SimpleNamespace(
            detections=list(attempts), snapshot_id='fresh')),
        _attempts=lambda items: items, _record_pipeline_message=lambda *_: None,
        _prepare_gripper=lambda *_: pytest.fail('Unexpected initial gripper command'),
        _open_gripper=lambda *_: pytest.fail('Unexpected initial gripper command'),
        _policy=load_sorting_policy(root/'cleany_skill_executor/config/table_sorting_policy.yaml'),
        _bins={b.name: b for b in load_bins(root/'cleany_mujoco_sim/config/robot_top_bins.yaml')})
    node.__dict__.update(overrides)
    return node


@pytest.mark.parametrize('pending', [[], ['lego brick']])
def test_sorting_start_does_not_send_initial_gripper_commands(pending):
    events = []
    node = sorting_loop(stage=events.append, _pending_handoffs=pending)
    if pending:
        with pytest.raises(RuntimeError,match='Pending handoff'):
            SortingCoordinator.run(node)
        assert events == ['starting','search']
        return
    SortingCoordinator.run(node)
    assert events == ['starting', 'search', 'empty_confirmation', 'empty_confirmation', 'mission_complete']


def test_operator_observation_does_not_wait_for_placement_service():
    events = []
    node = sorting_loop(stage=events.append, _placement_verification_enabled=False,
        _verification=SimpleNamespace(wait_for_service=lambda **k: pytest.fail('Verification must not block')))
    SortingCoordinator.run(node)
    assert events[-1] == 'mission_complete_unverified'


@pytest.mark.parametrize('stage,verified', [('complete', True), ('complete_unverified', False)])
def test_placement_status_distinguishes_operator_observation(stage, verified):
    import json
    published = []
    node = SimpleNamespace(_current={'label': 'cup'}, _completed=[], _artifact_directory=None,
        _status=SimpleNamespace(publish=lambda message: published.append(json.loads(message.data))),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    SortingCoordinator._stage(node, stage)
    assert published[0]['placement_verified'] is verified
    assert node._current['placement_verified'] is verified


def test_visible_unclassified_object_cannot_be_reported_as_clear():
    events = []
    obj = SimpleNamespace(obb_pose=Pose(),obb_size=SimpleNamespace(x=.03,y=.02,z=.01))
    obj.obb_pose.position.x, obj.obb_pose.position.z = .4,.36
    obj.obb_pose.orientation.w = 1.
    attempt = ObjectAttempt(1,'cup',.95,.5)
    node = sorting_loop([attempt], stage=events.append,
        _inspect_selected=lambda *_: SimpleNamespace(objects=SimpleNamespace(objects=[obj])))
    with pytest.raises(RuntimeError,match='1 unresolved'):
        SortingCoordinator.run(node)
    assert events == ['starting','search','review']


@pytest.mark.parametrize('target_y,arms', [(.2, ['left', 'right']), (-.2, ['right', 'left']), (0., ['left', 'right'])])
@pytest.mark.parametrize('label,category', [('cup', 'trash'), ('wallet', 'lost_item')])
@pytest.mark.parametrize('success_index', [0, 1, None])
def test_sorting_tries_target_near_arm_then_other_without_changing_destination(
        monkeypatch, target_y, arms, label, category, success_index):
    attempt = ObjectAttempt(1, label, .99, .5, category, 'test')
    obj = SimpleNamespace(obb_pose=Pose(), obb_size=SimpleNamespace(x=.03, y=.02, z=.01))
    obj.obb_pose.position.x, obj.obb_pose.position.y, obj.obb_pose.position.z = .4, target_y, .36
    calls = []
    selected = object()
    def select(*args, required_arm):
        calls.append(required_arm)
        return selected if len(calls) - 1 == success_index else None
    destination = 'trash_right' if category == 'trash' else 'lost_items_left'
    def execute(target, decision, node, stage):
        assert decision.destination == destination
        assert target.selected is selected
        raise RuntimeError('selection verified')
    monkeypatch.setattr('cleany_skill_executor.sorting_coordinator.execute_sort', execute)
    node = sorting_loop([attempt], maximum=1,
        _inspect_selected=lambda *_: SimpleNamespace(objects=SimpleNamespace(objects=[obj])),
        _plan_grasps=lambda *_: SimpleNamespace(candidates=[]), _select_reachable=select)
    with pytest.raises(RuntimeError, match='unresolved' if success_index is None else 'selection verified'):
        SortingCoordinator.run(node)
    assert calls == (arms[:success_index + 1] if success_index is not None else arms)


@pytest.mark.parametrize('first_failure', [None, 'inspection', 'planning', 'reachability'])
def test_fixed_bins_reconstruct_candidates_only_until_one_is_reachable(monkeypatch, first_failure):
    attempts = [ObjectAttempt(i, 'object', .95, .4+i*.1, 'trash', 'discarded')
                for i in (1, 2, 3)]
    obj = SimpleNamespace(obb_pose=Pose(), obb_size=SimpleNamespace(x=.03, y=.02, z=.01))
    obj.obb_pose.position.x, obj.obb_pose.position.z = .4, .36
    obj.obb_pose.orientation.w = 1.
    calls = []
    def inspect(snapshot, attempt):
        assert snapshot == 'fresh'
        calls.append(attempt.object_id)
        if first_failure == 'inspection' and attempt.object_id == 1:
            return None
        return SimpleNamespace(objects=SimpleNamespace(objects=[obj]))
    def plan(observed, attempt):
        return None if first_failure == 'planning' and attempt.object_id == 1 else SimpleNamespace(candidates=[])
    def select(candidates, attempt, **kwargs):
        return None if first_failure == 'reachability' and attempt.object_id == 1 else object()
    def execute(target, *args, **kwargs):
        assert target.attempt.object_id == (2 if first_failure else 1)
        raise RuntimeError('selected candidate executed')
    monkeypatch.setattr('cleany_skill_executor.sorting_coordinator.execute_sort', execute)
    node = sorting_loop(attempts, maximum=1,
        _inspect_selected=inspect, _plan_grasps=plan, _select_reachable=select)
    with pytest.raises(RuntimeError, match='selected candidate executed'):
        SortingCoordinator.run(node)
    assert calls == ([1, 2] if first_failure else [1])


def test_missing_previously_seen_item_is_not_an_empty_workspace(monkeypatch):
    attempts=[ObjectAttempt(1,'cup',.95,.5,'trash','Disposable'),
              ObjectAttempt(2,'wallet',.95,.6,'lost_item','Personal belonging')]
    detections=iter([attempts,[]])
    obj=SimpleNamespace(obb_pose=Pose(),obb_size=SimpleNamespace(x=.03,y=.02,z=.01))
    obj.obb_pose.position.x,obj.obb_pose.position.z=.4,.36
    obj.obb_pose.orientation.w=1.
    node = sorting_loop(attempts,
        _next_sorting_detection=lambda: SimpleNamespace(detections=SimpleNamespace(
            detections=next(detections), snapshot_id='fresh')),
        _inspect_selected=lambda *_: SimpleNamespace(objects=SimpleNamespace(objects=[obj])),
        _plan_grasps=lambda *_: SimpleNamespace(candidates=[]), _select_reachable=lambda *a, **k: object())
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
    node.get_parameter = lambda name: SimpleNamespace(value=.12)
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
@pytest.mark.parametrize('fails', [False, True])
def test_wrist_retreat_starts_detection_before_motion(arm, fails):
    events = []
    home = JointState(name=list(ARM_JOINT_NAMES[arm]), position=[.1]*5)
    def move(selected_arm, joints, label):
        assert selected_arm == arm
        assert joints.name == [*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint']
        assert list(joints.position) == [.1]*5 + [-.3]
        assert label == 'return from trash_left'
        events.append('return')
        if fails:
            raise RuntimeError('return motion failed')
    pending = (SimpleNamespace(cancel_goal_async=lambda: events.append('cancel')), 'future')
    node = SimpleNamespace(_wrist_enabled=True, _home={arm: home}, _held_object=None,
        get_parameter=lambda _: SimpleNamespace(value=-.3),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        _move_to=move,
        _begin_object_detection=lambda: events.append('detect') or pending,
        _verify_feedback=lambda *_: events.append('feedback'),
        _switch_camera=lambda camera: events.append(camera))
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm=arm))
    if fails:
        with pytest.raises(RuntimeError, match='return motion failed'):
            SortingCoordinator.retreat(node, held, 'trash_left')
        assert events == ['head', 'detect', 'return', 'cancel']
        assert node._return_detection is None
    else:
        SortingCoordinator.retreat(node, held, 'trash_left')
        assert events == ['head', 'detect', 'return', 'feedback']
        assert node._return_detection == pending
    assert len(home.name) == 5


@pytest.mark.parametrize('arm,y,destination', [
    ('left', .49, 'trash_left'), ('right', -.49, 'lost_items_right')])
@pytest.mark.parametrize('use_waypoint', [False, True])
def test_transport_uses_fixed_base_bin_without_camera_inference(arm, y, destination, use_waypoint):
    size = SimpleNamespace(x=.04, y=.04, z=.04)
    held = SimpleNamespace(offset_in_tcp=np.zeros(3), selected=SimpleNamespace(selected_arm=arm,
        selected_candidate=SimpleNamespace(target_object=SimpleNamespace(obb_size=size))))
    center = np.array([.08, y, .26 + np.linalg.norm([.04]*3)/2 + .06])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.w = 1.
    events = []
    def solve(selected_arm, low, high, offset, **kwargs):
        assert selected_arm == arm
        assert low[2] == pytest.approx(center[2])
        assert high[2] > low[2]
        np.testing.assert_allclose(offset, held.offset_in_tcp)
        events.append('ik')
        return SimpleNamespace(names=(f'{arm}_shoulder_yaw_joint',), positions=(0.,))
    node = SimpleNamespace(
        _bins={destination: SimpleNamespace(center_xy=(.08, y), top_z=.26, kind='bin',
                                           outside_size=(.18,.17,.12), wall=.008)},
        _require_held_contact=lambda *_: events.append('contact'),
        get_parameter=lambda name: SimpleNamespace(value={
            'sorting_release_clearance_m': .06, 'sorting_release_maximum_clearance_m': .21,
            'sorting_release_edge_margin_m': .005, 'sorting_release_ik_attempts': 16,
            'sorting_release_ik_iterations': 80}[name]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _tcp_pose=lambda *_: pose, _feedback_state=lambda: object(),
        _pose_position=NearestPregraspCoordinator._pose_position,
        _transport_adapter=SimpleNamespace(set_current_state=lambda *_: None,
            solve_held_region_ik=solve, state_is_valid=lambda *_: True),
        _move_to=lambda *_: events.append('move'),
        _verify_feedback=lambda *_: events.append('feedback'),
        _camera_info=None,
        _observe_wrist=lambda *_: pytest.fail('Transport must not wait for bin image'),
        _inspect_selected=lambda *_: pytest.fail('Transport must not redetect the bin'))
    if use_waypoint:
        node._common_waypoint = np.array([-.35, 0., .6])
        node._common_waypoint_joints = lambda _: events.append('waypoint_ik') or object()
        node._stage = lambda stage: events.append(stage)
        node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    SortingCoordinator.transport(node, held, destination)
    prefix = (['waypoint_ik', 'common_waypoint', 'move', 'feedback', 'contact']
              if use_waypoint else [])
    assert events == ['contact'] + prefix + ['ik', 'move', 'feedback', 'contact']


@pytest.mark.parametrize('failure', ['ik', 'collision', 'fk', None])
def test_common_waypoint_requires_ik_collision_and_fk_checks(failure):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = -.35, 0., .6
    if failure == 'fk':
        pose.position.x += .02
    solution = SimpleNamespace(names=['right_shoulder_yaw_joint'], positions=[0.])
    node = SimpleNamespace(_common_waypoint=np.array([-.35, 0., .6]),
        _feedback_state=lambda: object(), _tcp_pose=lambda *_: pose,
        _pose_position=NearestPregraspCoordinator._pose_position,
        _transport_adapter=SimpleNamespace(set_current_state=lambda _: None,
            solve_position_ik=lambda *_: None if failure == 'ik' else solution,
            state_is_valid=lambda *_: failure != 'collision'))
    if failure:
        with pytest.raises(RuntimeError):
            SortingCoordinator._common_waypoint_joints(node, 'right')
    else:
        assert SortingCoordinator._common_waypoint_joints(node, 'right').name == solution.names


def test_common_waypoint_preflight_failure_prevents_pick_motion():
    def reject(_):
        raise RuntimeError('No common waypoint IK')
    node = SimpleNamespace(_common_waypoint=np.array([-.35, 0., .6]),
        _common_waypoint_joints=reject,
        _execute_pregrasp=lambda *_: pytest.fail('Must not start pick after failed preflight'))
    target = SimpleNamespace(selected=SimpleNamespace(selected_arm='right'))
    with pytest.raises(RuntimeError, match='No common waypoint IK'):
        SortingCoordinator.pick(node, target)


def test_contact_offset_uses_actual_tcp_not_commanded_endpoint():
    tcp = Pose()
    tcp.orientation.w = 1.
    tcp.position.x, tcp.position.z = .4, .5
    obb = Pose()
    obb.position.x, obb.position.z = .38, .46
    selected = SimpleNamespace(selected_arm='left', selected_candidate=SimpleNamespace(
        target_object=SimpleNamespace(obb_pose=obb)))
    node = SimpleNamespace(_tcp_pose=lambda arm: tcp,
                           _arm_joint_state=lambda arm: JointState(
                               name=['left_wrist_pitch_joint', 'left_wrist_roll_joint'],
                               position=[.3, -.2]),
                           get_parameter=lambda _: SimpleNamespace(value=[2., 5., 10., 20., 30.]),
                           _pose_position=NearestPregraspCoordinator._pose_position)
    SortingCoordinator._on_grasp_contact(node, selected, object())
    np.testing.assert_allclose(node._held_object.offset_in_tcp, (-.02, 0., -.04))
    assert node._carry_wrist_reference == {'left_wrist_roll_joint': -.2}
    assert node._carry_wrist_tolerance == pytest.approx(np.deg2rad(2.))


@pytest.mark.parametrize('succeed_at', [2., 10., None])
def test_carry_wrist_ik_relaxes_only_to_configured_bound(succeed_at):
    calls = []
    def solve(*args, **kwargs):
        low, high = kwargs['joint_bounds']['right_wrist_roll_joint']
        assert (low+high)/2 == pytest.approx(.4)
        calls.append(np.rad2deg((high-low)/2))
        return object() if succeed_at is not None and calls[-1] >= succeed_at-1e-6 else None
    params = {'sorting_carry_wrist_tolerances_deg': [2., 5., 10., 20., 30.],
              'sorting_release_ik_attempts': 16, 'sorting_release_ik_iterations': 80}
    node = SimpleNamespace(
        _carry_wrist_reference={'right_wrist_roll_joint': .4}, _carry_wrist_tolerance=np.deg2rad(2.),
        _feedback_state=lambda: None, get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _transport_adapter=SimpleNamespace(set_current_state=lambda _: None, solve_held_region_ik=solve))
    if succeed_at is None:
        with pytest.raises(RuntimeError, match='No carry IK'):
            SortingCoordinator._carry_region_ik(node, 'right', (0,0,0), (1,1,1), np.zeros(3))
        assert calls == pytest.approx([2.,5.,10.,20.,30.])
    else:
        assert SortingCoordinator._carry_region_ik(node, 'right', (0,0,0), (1,1,1), np.zeros(3))
        assert calls[-1] == pytest.approx(succeed_at)


@pytest.mark.parametrize('held', [True, False])
def test_carry_wrist_constraints_only_while_holding(monkeypatch, held):
    from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', lambda *args: MoveGroup.Goal())
    node = object.__new__(SortingCoordinator)
    node._held_object = object() if held else None
    node._carry_wrist_reference = {'right_wrist_roll_joint': .4}
    node._carry_wrist_tolerance = .05
    node.get_parameter = lambda _: SimpleNamespace(value=.5)
    goal = SortingCoordinator._execution_goal(node, 'right', JointState(), 'transport')
    constraints = goal.request.path_constraints.joint_constraints
    assert len(constraints) == int(held)
    if held:
        assert constraints[0].position == .4
        assert constraints[0].tolerance_above == .05


def test_carry_cartesian_rejects_midpath_wrist_excursion():
    trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=['right_wrist_roll_joint'], points=[
            SimpleNamespace(positions=[value]) for value in (.4, .7, .4)]))
    node = SimpleNamespace(_held_object=object(),
        _carry_wrist_reference={'right_wrist_roll_joint': .4}, _carry_wrist_tolerance=.05)
    with pytest.raises(RuntimeError, match='wrist bound exceeded'):
        SortingCoordinator._validate_cartesian_plan(node, 'right', trajectory, None, None, 'lift')


@pytest.mark.parametrize('failure', ['planning', 'infrastructure', 'exhausted'])
def test_carry_replans_endpoint_and_retries_only_plan_failures(monkeypatch, failure):
    from cleany_skill_executor.seeded_cartesian import CartesianPlanningError
    calls = []
    def plan(node, arm, target, label, **kwargs):
        calls.append((node._carry_wrist_tolerance, kwargs['joint_target'].position))
        if failure == 'infrastructure':
            raise RuntimeError('FK service unavailable')
        if len(calls) == 1 or failure == 'exhausted':
            raise CartesianPlanningError('no constrained path')
        return 'validated trajectory'
    monkeypatch.setattr(NearestPregraspCoordinator, '_plan_linear_motion', plan)
    node = object.__new__(SortingCoordinator)
    node._held_object = object()
    node._carry_wrist_reference = {'right_wrist_pitch_joint': .3}
    node._carry_wrist_tolerance = np.deg2rad(2.)
    params = {'sorting_carry_wrist_tolerances_deg': [2.,5.,10.], 'sorting_release_ik_attempts': 16,
              'sorting_release_ik_iterations': 80, 'lin_position_tolerance_m': .001,
              'corridor_orientation_tolerance_deg': 5., 'sorting_carry_cartesian_rotation_limit_deg': 30.}
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node._wait_arm_stationary = lambda _: None
    node._check_motion_guard = lambda: None
    node._feedback_state = lambda: None
    node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    node._transport_adapter = SimpleNamespace(set_current_state=lambda _: None,
        solve_held_region_ik=lambda *args, **kwargs: SimpleNamespace(
            names=['right_wrist_pitch_joint'], positions=[.3]))
    pose = Pose()
    pose.orientation.w = 1.
    node._tcp_pose = lambda *args: pose
    kwargs = dict(joint_target=JointState(name=['right_wrist_pitch_joint'], position=[1.]),
                  velocity_scaling=.5)
    if failure == 'infrastructure':
        with pytest.raises(RuntimeError, match='FK service unavailable'):
            node._plan_linear_motion('right', pose, 'reverse grasp retreat', **kwargs)
        assert len(calls) == 1
    elif failure == 'exhausted':
        with pytest.raises(CartesianPlanningError, match='no bounded carry path'):
            node._plan_linear_motion('right', pose, 'reverse grasp retreat', **kwargs)
        assert [a for a, _ in calls] == pytest.approx(np.deg2rad([2.,5.,10.]))
    else:
        assert node._plan_linear_motion('right', pose, 'reverse grasp retreat', **kwargs) == 'validated trajectory'
        assert [a for a, _ in calls] == pytest.approx(np.deg2rad([2.,5.]))
        assert all(list(q) == [.3] for _, q in calls)  # old 1-radian goal is never reused


def test_carry_execution_error_is_never_retried(monkeypatch):
    calls = []
    def execute(*args, **kwargs):
        calls.append(True)
        raise RuntimeError('controller failed')
    monkeypatch.setattr(NearestPregraspCoordinator, '_execute_linear', execute)
    node = object.__new__(SortingCoordinator)
    node._held_object = object()
    node._carry_wrist_reference = {'right_wrist_pitch_joint': .3}
    with pytest.raises(RuntimeError, match='controller failed'):
        node._execute_linear('right', Pose(), 'lift', velocity_scaling=.5)
    assert len(calls) == 1


@pytest.mark.parametrize('valid', [True, False])
def test_fixed_release_cache_always_rechecks_current_scene(valid):
    arm, point, offset = 'right', np.zeros(3), np.zeros(3)
    wrist = {'right_wrist_pitch_joint': .3, 'right_wrist_roll_joint': -.4}
    solution = SimpleNamespace(names=list(wrist), positions=tuple(wrist.values()))
    pose = Pose()
    pose.orientation.w = 1.
    events = []
    def solve(*args, **kwargs):
        events.append('solve')
        assert kwargs['fixed_joints'] == wrist
        return None
    node = SimpleNamespace(
        _fixed_release_cache={(arm, tuple(point), tuple(offset), tuple(sorted(wrist.items()))): solution},
        _feedback_state=lambda: None, _tcp_pose=lambda *_: pose,
        _pose_position=NearestPregraspCoordinator._pose_position,
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        get_parameter=lambda key: SimpleNamespace(value=16 if key.endswith('attempts') else 80),
        _transport_adapter=SimpleNamespace(set_current_state=lambda _: events.append('state'),
            state_is_valid=lambda *_: events.append('collision') or valid, solve_held_region_ik=solve))
    result = SortingCoordinator._fixed_release_ik(node, arm, point, offset, wrist)
    assert result is (solution if valid else None)
    assert events == (['state', 'collision'] if valid else ['state', 'collision', 'solve'])


def test_direct_release_ik_failure_prevents_motion():
    params = {'sorting_release_clearance_m': .06, 'sorting_release_maximum_clearance_m': .21,
              'sorting_release_edge_margin_m': .005, 'sorting_carry_wrist_tolerances_deg': [2.,5.],
              'sorting_release_ik_attempts': 16, 'sorting_release_ik_iterations': 80}
    node = SimpleNamespace(
        _bins={'trash': SimpleNamespace(center_xy=(-.405,-.105), outside_size=(.18,.17,.12), wall=.008, top_z=.3)},
        _fixed_release_points={'trash': np.array([-.395,-.105,.52])},
        _wait_arm_stationary=lambda _: None,
        _arm_joint_state=lambda _: JointState(name=['left_wrist_pitch_joint', 'left_wrist_roll_joint'], position=[.3,.4]),
        _fixed_release_ik=lambda *a: None,
        _common_waypoint=np.array([-.35,0.,.6]), _carry_wrist_tolerance=np.deg2rad(2.),
        _carry_wrist_reference={'left_wrist_pitch_joint': .3, 'left_wrist_roll_joint': .4},
        _feedback_state=lambda: None, _require_held_contact=lambda _: None,
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _transport_adapter=SimpleNamespace(set_current_state=lambda _: None, solve_held_region_ik=lambda *a, **k: None),
        _move_to=lambda *a: pytest.fail('No motion before destination preflight'),
        _stage=lambda *a: pytest.fail('No waypoint stage before destination preflight'))
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm='left'), offset_in_tcp=np.zeros(3))
    with pytest.raises(RuntimeError, match='Direct fixed release unreachable'):
        SortingCoordinator._transport_fixed_release(node, held, 'trash', .04)


def test_direct_mode_skips_waypoint_preflight_before_pick():
    def begin(*args):
        raise RuntimeError('pregrasp reached')
    node = SimpleNamespace(_fixed_release_enabled=True, _common_waypoint=np.array([-.35,0.,.6]),
        _common_waypoint_joints=lambda *_: pytest.fail('No waypoint preflight in direct mode'),
        _execute_pregrasp=begin)
    target = SimpleNamespace(selected=SimpleNamespace(selected_arm='right'), attempt=object())
    with pytest.raises(RuntimeError, match='pregrasp reached'):
        SortingCoordinator.pick(node, target)


def test_fixed_release_goal_holds_post_lift_wrist_without_relaxation(monkeypatch):
    from cleany_skill_executor.grasp_execution_demo import GraspExecutionDemo
    monkeypatch.setattr(GraspExecutionDemo, '_execution_goal', lambda *args: MoveGroup.Goal())
    node = object.__new__(SortingCoordinator)
    node._held_object = object()
    node._carry_wrist_reference = {'right_wrist_roll_joint': .4}
    node._carry_wrist_tolerance = .5
    node._fixed_release_wrist = {'right_wrist_roll_joint': .5}
    node._fixed_release_hold_tolerance = np.deg2rad(.5)
    node.get_parameter = lambda _: SimpleNamespace(value=.5)
    constraints = node._execution_goal('right', JointState(), 'transport').request.path_constraints.joint_constraints
    assert len(constraints) == 1
    assert constraints[0].joint_name == 'right_wrist_roll_joint'
    for constraint in constraints:
        assert constraint.position == node._fixed_release_wrist[constraint.joint_name]
        assert constraint.tolerance_above == pytest.approx(np.deg2rad(.5))


def test_fixed_transfer_goes_directly_to_classified_bin_without_waypoint():
    arm = 'right'
    names = [f'{arm}_wrist_pitch_joint', f'{arm}_wrist_roll_joint']
    drop = SimpleNamespace(names=names, positions=(.3, .4))
    params = {'sorting_release_clearance_m': .06, 'sorting_release_maximum_clearance_m': .21,
              'sorting_release_edge_margin_m': .005, 'sorting_carry_wrist_tolerances_deg': [2.,5.],
              'sorting_release_ik_attempts': 16, 'sorting_release_ik_iterations': 80}
    events = []
    def solve(*args, **kwargs):
        pytest.fail('Must not solve shared waypoint IK')
    def final(arm, point, offset, wrist):
        events.append('measured wrist recheck')
        assert wrist == dict(zip(names, (.301, .399)))
        return drop
    node = SimpleNamespace(
        _bins={'lost': SimpleNamespace(center_xy=(-.405,.105), outside_size=(.18,.17,.12), wall=.008, top_z=.3)},
        _fixed_release_points={'lost': np.array([-.395,.105,.52])}, _fixed_release_cache={},
        _common_waypoint=np.array([-.35,0.,.6]), _carry_wrist_tolerance=np.deg2rad(2.),
        _carry_wrist_reference=dict(zip(names, drop.positions)),
        _feedback_state=lambda: None, _require_held_contact=lambda _: None,
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _transport_adapter=SimpleNamespace(set_current_state=lambda _: None, solve_held_region_ik=solve),
        _move_to=lambda arm, joints, label: events.append(label),
        _stage=lambda _: pytest.fail('Must not enter common waypoint stage'), _verify_feedback=lambda _: None,
        _wait_arm_stationary=lambda _: None,
        _arm_joint_state=lambda _: JointState(name=names, position=[.301,.399]), _fixed_release_ik=final)
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm=arm), offset_in_tcp=np.zeros(3))
    SortingCoordinator._transport_fixed_release(node, held, 'lost', .04)
    assert events == ['measured wrist recheck', 'transport to lost']


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


@pytest.mark.parametrize('axis', ['x', 'y', 'z'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_release_rejects_nonfinite_center_before_opening_or_disarming(axis, value):
    pose = Pose()
    pose.orientation.w = 1.0
    setattr(pose.position, axis, value)
    held = SimpleNamespace(offset_in_tcp=np.zeros(3),
                           selected=SimpleNamespace(selected_arm='left'))
    node = SimpleNamespace(_tcp_pose=lambda _: pose,
                          _pose_position=NearestPregraspCoordinator._pose_position,
                          _open_gripper=lambda _: pytest.fail('Invalid release must not open'))
    with pytest.raises(RuntimeError, match='center must be finite'):
        SortingCoordinator.release(node, held, 'bin')


@pytest.mark.parametrize('radius', [float('nan'), float('inf'), 0.0, -0.01])
def test_release_rejects_invalid_payload_radius(monkeypatch, radius):
    pose = Pose()
    pose.orientation.w = 1.0
    held = SimpleNamespace(offset_in_tcp=np.zeros(3),
                           selected=SimpleNamespace(selected_arm='left'))
    node = SimpleNamespace(_tcp_pose=lambda _: pose,
                          _pose_position=NearestPregraspCoordinator._pose_position,
                          _bins={'bin': object()})
    monkeypatch.setattr('cleany_skill_executor.sorting_coordinator.held_bounding_radius',
                        lambda *args: radius)
    with pytest.raises(RuntimeError, match='radius must be positive and finite'):
        SortingCoordinator.release(node, held, 'bin')


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
