from types import SimpleNamespace

import pytest
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.nearest_object import (
    ObjectAttempt,
    rank_object_attempts,
)
from cleany_skill_executor.core.grasp_pipeline import (
    ObservedGrasp, ReinspectionLimits,
)
from cleany_skill_executor.nearest_pregrasp_coordinator import (
    NearestPregraspCoordinator,
)


def _attempt(
    object_id: int,
    distance_m: float,
    confidence: float,
) -> ObjectAttempt:
    return ObjectAttempt(
        object_id=object_id,
        label=f'object-{object_id}',
        confidence=confidence,
        distance_m=distance_m,
    )


@pytest.mark.parametrize('already_aligned', [True, False])
def test_refreshed_pregrasp_is_executed_and_reobserved_before_contact(already_aligned):
    events = []
    current, goal = Pose(), Pose()
    current.position.z, goal.position.z = 0.55, 0.4
    current.position.x = 0.0 if already_aligned else 0.10
    selected = SimpleNamespace(selected_arm='left', selected_candidate=object(),
                               grasp_joint_state=object(), pregrasp_joint_state=object())
    node = SimpleNamespace(
        _tcp_pose=lambda arm, override=None: goal if override else current,
        _pose_position=NearestPregraspCoordinator._pose_position,
        _observed_grasp=lambda *args: SimpleNamespace(approach=(0., 0., -1.)),
        get_parameter=lambda _: SimpleNamespace(value=10.),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _execution_scene=SimpleNamespace(
            disallow_target_contacts=lambda: events.append('disallow'),
            allow_contacts_for=lambda arm: events.append('allow')),
        _move_to=lambda *args: events.append('move'),
        _verify_feedback=lambda _: events.append('feedback'),
        _confirm_unchanged_target=lambda *args: events.append('reobserve'),
    )
    NearestPregraspCoordinator._align_refreshed_pregrasp(
        node, selected, ObjectAttempt(1, 'cup', 0.8, 0.5))
    assert events == ([] if already_aligned else
                      ['disallow', 'move', 'feedback', 'reobserve', 'allow'])


@pytest.mark.parametrize('shift,stamp,count,frame,flipped,accepted', [
    (0.003, 2, 1, 'base_link', False, True),
    (0.020, 2, 1, 'base_link', False, False),
    (0., 0, 1, 'base_link', False, False),
    (0., 2, 0, 'base_link', False, False),
    (0., 2, 2, 'base_link', False, False),
    (0., 2, 1, 'camera', False, False),
    (0., 2, 1, 'base_link', True, True),
])
def test_post_reposition_confirmation_is_fresh_unique_and_geometry_bounded(
        shift, stamp, count, frame, flipped, accepted):
    from copy import deepcopy
    from geometry_msgs.msg import Vector3
    original = SimpleNamespace(obb_pose=Pose(), obb_size=Vector3(x=0.1, y=0.08, z=0.12))
    original.obb_pose.orientation.w = 1.0
    fresh = deepcopy(original)
    fresh.obb_pose.position.x = shift
    if flipped:
        fresh.obb_pose.orientation.w = 0.
        fresh.obb_pose.orientation.z = 1.
    selected = SimpleNamespace(selected_arm='left', selected_candidate=SimpleNamespace(
        target_object=original, header=SimpleNamespace(frame_id='base_link')))
    detection = SimpleNamespace(object_id=1, label='cup', confidence=0.8,
                                distance_m=0.5, distance_valid=True)
    inspected = SimpleNamespace(objects=SimpleNamespace(objects=[fresh], header=SimpleNamespace(
        frame_id=frame, stamp=SimpleNamespace(sec=stamp, nanosec=0))))
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        _wait_arm_stationary=lambda _: None,
        _detect_objects=lambda: SimpleNamespace(detections=SimpleNamespace(
            snapshot_id='fresh', detections=[detection]*count)),
        _inspect_selected=lambda *args: inspected,
        _obb_corners=NearestPregraspCoordinator._obb_corners,
        get_parameter=lambda _: SimpleNamespace(value=0.005),
    )
    if accepted:
        NearestPregraspCoordinator._confirm_unchanged_target(
            node, selected, ObjectAttempt(1, 'cup', 0.8, 0.5))
    else:
        with pytest.raises(RuntimeError, match='target geometry'):
            NearestPregraspCoordinator._confirm_unchanged_target(
                node, selected, ObjectAttempt(1, 'cup', 0.8, 0.5))


def test_attempts_rank_by_distance_confidence_then_object_id():
    ranked = rank_object_attempts(
        (
            _attempt(3, 0.8, 0.99),
            _attempt(2, 0.4, 0.7),
            _attempt(1, 0.4, 0.9),
            _attempt(4, 0.4, 0.9),
        )
    )

    assert [item.object_id for item in ranked] == [1, 4, 2, 3]


@pytest.mark.parametrize('extent', [-0.1, float('nan')])
def test_confirmation_rejects_invalid_box_geometry(extent):
    from geometry_msgs.msg import Vector3
    box = SimpleNamespace(obb_pose=Pose(), obb_size=Vector3(x=extent, y=0.1, z=0.1))
    box.obb_pose.orientation.w = 1.0
    with pytest.raises(ValueError, match='positive extents'):
        NearestPregraspCoordinator._obb_corners(box)


@pytest.mark.parametrize('reachable', [True, False])
@pytest.mark.parametrize('compatible', [True, False])
def test_refresh_hands_off_old_obb_before_selector_transaction(reachable, compatible):
    events = []
    candidate = SimpleNamespace(snapshot_id='new')
    incompatible = SimpleNamespace(snapshot_id='bad')
    selected = SimpleNamespace(
        selected_arm='left', selected_candidate=candidate,
        selected_candidate_index=0,
    )
    detection = SimpleNamespace(object_id=1, label='cup', confidence=0.8,
                                distance_valid=True, distance_m=0.5,
                                sorting_category='trash', sorting_reason='disposable cup')
    pose = Pose()
    pose.position.x, pose.position.z = 0.4, 0.4
    inspected = SimpleNamespace(objects=SimpleNamespace(
        objects=[SimpleNamespace(obb_pose=pose)]))

    def select(*args, **kwargs):
        assert args[0] == [candidate]
        # The selector may now register exactly one new target collision OBB.
        assert events == ['restore']
        events.append('select')
        return selected if reachable else None

    node = SimpleNamespace(
        _wait_arm_stationary=lambda _: None,
        _detect_objects=lambda: SimpleNamespace(detections=SimpleNamespace(
            detections=[detection], snapshot_id='new')),
        _inspect_selected=lambda *args: inspected,
        _observed_grasp=lambda item, key: ObservedGrasp(
            key, (0.4, 0., 0.4), (0., 0., -1.),
            (0., 1., 0.) if item.snapshot_id == 'bad' else (1., 0., 0.)),
        _reinspection_limits=lambda: ReinspectionLimits(),
        _plan_grasps=lambda *args: SimpleNamespace(
            candidates=[incompatible, candidate] if compatible else [incompatible]),
        _select_reachable=select,
        _publish_grasp_overlay=lambda *args, **kwargs: None,
        _execution_scene=SimpleNamespace(
            restore=lambda: events.append('restore'),
            begin=lambda *args: events.append('begin'),
            allow_contacts_for=lambda _: events.append('allow'),
        ),
    )
    attempt = ObjectAttempt(1, 'cup', 0.8, 0.5)
    if not compatible:
        with pytest.raises(RuntimeError, match='no continuity-compatible'):
            NearestPregraspCoordinator._refresh_selected_grasp(node, selected, attempt)
        assert events == []
    elif reachable:
        result, refreshed = NearestPregraspCoordinator._refresh_selected_grasp(
            node, selected, attempt)
        assert result is selected
        assert refreshed.sorting_category == 'trash'
        assert refreshed.sorting_reason == 'disposable cup'
        assert events == ['restore', 'select', 'begin', 'allow']
    else:
        with pytest.raises(RuntimeError, match='not reachable'):
            NearestPregraspCoordinator._refresh_selected_grasp(
                node, selected, attempt)
        assert events == ['restore', 'select']


def test_attempts_reject_duplicate_object_ids():
    with pytest.raises(ValueError, match='unique'):
        rank_object_attempts((_attempt(1, 0.2, 0.8), _attempt(1, 0.3, 0.7)))


def test_coordinator_ignores_invalid_depth_detections():
    detections = [
        SimpleNamespace(
            object_id=1,
            label='invalid-near',
            confidence=1.0,
            distance_valid=False,
            distance_m=0.0,
        ),
        SimpleNamespace(
            object_id=2,
            label='far',
            confidence=0.9,
            distance_valid=True,
            distance_m=0.8,
        ),
        SimpleNamespace(
            object_id=3,
            label='near',
            confidence=0.7,
            distance_valid=True,
            distance_m=0.4,
        ),
    ]

    attempts = NearestPregraspCoordinator._attempts(detections)

    assert [item.object_id for item in attempts] == [3, 2]


def test_grasp_debug_image_is_republished_when_available():
    published = []
    image = object()
    coordinator = SimpleNamespace(
        _last_grasp_debug_image=image,
        _grasp_debug_publisher=SimpleNamespace(publish=published.append),
    )

    NearestPregraspCoordinator._republish_grasp_debug_image(coordinator)

    assert published == [image]


def test_transient_gripper_contact_does_not_attach_or_lift():
    events = []
    start, goal = Pose(), Pose()
    start.position.z, goal.position.z = 0.55, 0.4
    selected = SimpleNamespace(selected_arm='left', grasp_joint_state=object(),
                               selected_candidate=object())
    node = SimpleNamespace(
        _align_refreshed_pregrasp=lambda *args: None,
        _tcp_pose=lambda arm, override=None: goal if override else start,
        _pose_position=NearestPregraspCoordinator._pose_position,
        _observed_grasp=lambda *args: SimpleNamespace(approach=(0., 0., -1.)),
        get_parameter=lambda name: SimpleNamespace(value=10. if
                                                   name == 'lin_alignment_tolerance_deg'
                                                   else 0.2),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _execute_linear=lambda *args, **kwargs: events.append('approach'),
        _joint_positions={'left_gripper_joint': 1.2},
        _candidate_close_position=lambda _: 0.5,
        _command_gripper=lambda *args, **kwargs: True,
        _hold=lambda _: events.append('settle'),
        _gripper_contact_stalled=lambda *args: False,
        _on_grasp_contact=lambda *_: events.append('contact_reference'),
        _execution_scene=SimpleNamespace(
            restore=lambda: events.append('restore'),
            attach_to=lambda _: events.append('attach'),
        ),
    )
    node._retry_gripper_contact = lambda *args: NearestPregraspCoordinator._retry_gripper_contact(node, *args)
    with pytest.raises(RuntimeError, match='did not persist'):
        NearestPregraspCoordinator._execute_grasp_and_lift(
            node, selected, ObjectAttempt(1, 'cup', 0.8, 0.5))
    assert events == ['approach', 'settle', 'restore']


def test_full_close_retains_existing_lower_command_without_width_override():
    values = {'gripper_force_full_close': True, 'gripper_close_position_rad': -.3}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))
    assert NearestPregraspCoordinator._candidate_close_position(node, object()) == -.3


def test_grasp_debug_republish_waits_for_first_image():
    published = []
    coordinator = SimpleNamespace(
        _last_grasp_debug_image=None,
        _grasp_debug_publisher=SimpleNamespace(publish=published.append),
    )

    NearestPregraspCoordinator._republish_grasp_debug_image(coordinator)

    assert published == []


def test_lin_goal_is_pilz_plan_only_pose_goal():
    values = {
        'pilz_pipeline_id': 'pilz_industrial_motion_planner',
        'pilz_planner_id': 'LIN',
        'lin_acceleration_scaling': 0.4,
        'lin_position_tolerance_m': 0.001,
        'lin_orientation_tolerance_rad': 0.01,
    }

    def execution_goal(arm, *_):
        goal = MoveGroup.Goal()
        goal.request.group_name = f'{arm}_grasp_arm'
        return goal

    coordinator = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        _execution_goal=execution_goal,
        _arm_joint_state=lambda _arm: JointState(),
    )
    coordinator._pose_constraint = lambda arm, target, label: (
        NearestPregraspCoordinator._pose_constraint(
            coordinator, arm, target, label
        )
    )
    target = Pose()
    target.orientation.w = 1.0

    goal = NearestPregraspCoordinator._linear_goal(
        coordinator, 'left', target, 'test', 0.2
    )

    assert goal.request.pipeline_id == 'pilz_industrial_motion_planner'
    assert goal.request.planner_id == 'LIN'
    assert goal.request.group_name == 'left_grasp_arm'
    assert goal.request.max_velocity_scaling_factor == 0.2
    assert goal.request.max_acceleration_scaling_factor == 0.4
    assert goal.planning_options.plan_only is True
    constraint = goal.request.goal_constraints[0]
    assert constraint.position_constraints[0].link_name == 'left_grasp_tcp'
    assert constraint.orientation_constraints[0].link_name == 'left_grasp_tcp'


def test_controller_contact_uses_reference_minus_feedback():
    state = JointTrajectoryControllerState()
    state.reference.positions = [0.2]
    state.feedback.positions = [0.05]
    state.feedback.velocities = [0.01]
    current = Pose()
    target = Pose()
    coordinator = SimpleNamespace(
        _controller_states={'left': state},
        get_parameter=lambda name: SimpleNamespace(
            value={
                'contact_effort_threshold': 0.0,
                'contact_min_joint_error_rad': 0.10,
                'contact_max_joint_velocity_rad_s': 0.05,
            }[name]
        ),
        _tcp_pose=lambda _arm: current,
        _pose_position=NearestPregraspCoordinator._pose_position,
    )

    sample = NearestPregraspCoordinator._contact_sample(
        coordinator, 'left', target
    )

    assert sample is not None
    assert sample.position_errors_rad == pytest.approx((0.15,))
    assert sample.tcp_distance_m == 0.0


@pytest.mark.parametrize('sensor_ready', [True, False])
@pytest.mark.parametrize('count_retreat', [True, False])
@pytest.mark.parametrize('retreat_rise', [0.0, 0.14])
@pytest.mark.parametrize('direct_vertical', [False, True])
def test_post_refresh_order_is_approach_grip_retreat_then_lift(sensor_ready, count_retreat, retreat_rise, direct_vertical):
    events = []
    start = Pose()
    start.orientation.w = 1.0
    start.position.z = retreat_rise
    grasp = Pose()
    grasp.position.x = 0.10
    grasp.orientation.w = 1.0
    poses = iter((start, grasp, grasp, grasp if direct_vertical else start))
    scene = SimpleNamespace(
        attach_to=lambda arm: events.append(('attach', arm)),
        restore=lambda: events.append(('restore',)),
    )
    values = {
        'lin_alignment_tolerance_deg': 10.0,
        'approach_velocity_scaling': 0.2,
        'retreat_velocity_scaling': 0.4,
        'lift_distance_m': 0.06,
        'count_retreat_as_lift': count_retreat,
        'require_sensor_scene': True,
        'attachment_scene_timeout_sec': 5.0,
    }
    def scene_barrier(timeout, *, after_stamp_ns):
        assert timeout == 5.0 and after_stamp_ns == 7_000_000_000
        events.append(('scene_barrier',))
        if not sensor_ready:
            raise RuntimeError('No new depth after attach')
    coordinator = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=7_000_000_000)),
        _wait_for_sensor_scene=scene_barrier,
        _direct_vertical_lift=direct_vertical,
        _align_refreshed_pregrasp=lambda *args: None,
        _tcp_pose=lambda *_: next(poses),
        _pose_position=NearestPregraspCoordinator._pose_position,
        _observed_grasp=lambda *_: SimpleNamespace(
            approach=(0.10, 0.0, -retreat_rise)
        ),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        _execute_linear=lambda arm, target, label, **kwargs: (
            events.append((label, kwargs['velocity_scaling'], target.position.z))
            or False
        ),
        _command_gripper=lambda *_args, **_kwargs: (
            events.append(('gripper',)) or True
        ),
        _candidate_close_position=lambda _candidate: 0.0,
        _joint_positions={'left_gripper_joint': 1.2},
        _gripper_contact_stalled=lambda *args: True,
        _hold=lambda name: events.append(('hold', name)),
        _execution_scene=scene,
        _on_grasp_contact=lambda *_: events.append(('contact_reference',)),
        _on_lift_motion_complete=lambda: events.append(('lift_complete',)),
        _verify_lift_height=lambda _attempt, **kwargs: events.append(('verify', kwargs)),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    selected = SimpleNamespace(
        selected_arm='left',
        selected_candidate=SimpleNamespace(target_object=SimpleNamespace(
            obb_pose=Pose())),
        grasp_joint_state=JointState(),
    )
    coordinator._retry_gripper_contact = lambda *args: NearestPregraspCoordinator._retry_gripper_contact(coordinator, *args)
    attempt = ObjectAttempt(1, 'box', 1.0, 0.5)

    if not sensor_ready:
        with pytest.raises(RuntimeError, match='No new depth'):
            NearestPregraspCoordinator._execute_grasp_and_lift(coordinator, selected, attempt)
        assert [event[0] for event in events] == [
            'refreshed grasp approach', 'gripper', 'hold', 'contact_reference', 'attach', 'scene_barrier']
        return
    NearestPregraspCoordinator._execute_grasp_and_lift(coordinator, selected, attempt)

    needs_vertical = direct_vertical or not count_retreat or retreat_rise < 0.06
    assert [event[0] for event in events] == [
        'refreshed grasp approach',
        'gripper',
        'hold',
        'contact_reference',
        'attach',
        'scene_barrier',
        *([] if direct_vertical else ['reverse grasp retreat']),
        *(['vertical grasp lift'] if needs_vertical else []),
        'lift_complete',
        'hold',
        'verify',
    ]
    assert events[0][1] == 0.2
    assert events[6][1] == 0.4
    if needs_vertical:
        lift_event = next(e for e in events if e[0] == 'vertical grasp lift')
        assert lift_event[1] == 0.4
        assert lift_event[2] == pytest.approx(0.06 + (
            0.0 if count_retreat or direct_vertical else retreat_rise))
    assert events[-1][1] == ({'minimum_center_z_m': 0.06} if count_retreat else {})


@pytest.mark.parametrize('observed_z', [0.40, 0.50])
def test_relative_lift_verification_requires_object_rise_not_just_tcp_motion(observed_z):
    detection = SimpleNamespace(label='cup', distance_valid=True, object_id=2,
                                confidence=0.8, distance_m=0.5)
    inspected = SimpleNamespace(objects=SimpleNamespace(objects=[SimpleNamespace(
        obb_pose=SimpleNamespace(position=SimpleNamespace(z=observed_z)))]))
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=0.38),
        _detect_objects=lambda: SimpleNamespace(detections=SimpleNamespace(
            detections=[detection], snapshot_id='fresh')),
        _inspect_selected=lambda *_: inspected,
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    attempt = ObjectAttempt(1, 'cup', 0.8, 0.5)
    if observed_z < 0.46:
        with pytest.raises(RuntimeError, match='was not retained'):
            NearestPregraspCoordinator._verify_lift_height(node, attempt, minimum_center_z_m=0.46)
    else:
        NearestPregraspCoordinator._verify_lift_height(node, attempt, minimum_center_z_m=0.46)
