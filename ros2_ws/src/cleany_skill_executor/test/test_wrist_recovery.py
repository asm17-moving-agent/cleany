from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time

from cleany_interfaces.msg import GraspCandidate
from cleany_interfaces.srv import ObserveWristTarget as Service
from cleany_skill_executor import sorting_coordinator as module
from cleany_skill_executor.nearest_pregrasp_coordinator import NearestPregraspCoordinator
from cleany_skill_executor.sorting_coordinator import SortingCoordinator, WristTargetNotDetected


@pytest.mark.parametrize(('attempts', 'misses'), [(1, 0), (1, 1), (3, 0), (3, 1), (3, 2), (3, 3)])
def test_check_retries_new_frames_and_requires_motor_confirmation_before_and_after(attempts, misses):
    calls, contacts, warnings = [], [], []
    observed = object()
    held = SimpleNamespace(selected=object())

    def observe(selected, operation, *, held, after_stamp_ns):
        assert selected is target.selected and held and operation == Service.Request.CHECK
        calls.append(after_stamp_ns)
        if len(calls) <= misses:
            raise WristTargetNotDetected('no target', len(calls) * 10**9)
        return observed

    target = held
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=attempts),
        _confirm_held_contact=lambda held: contacts.append(held), _observe_wrist=observe,
        get_logger=lambda: SimpleNamespace(info=lambda _: None, warning=warnings.append))
    result = SortingCoordinator._verify_wrist_or_contact(node, held)
    count = min(misses + 1, attempts)
    assert result is (None if misses == attempts else observed)
    assert calls == [None, 10**9, 2 * 10**9][:count]
    assert contacts == [held] * (2 * count)
    assert len(warnings) == misses + (misses == attempts)
    if misses == attempts:
        assert 'visual grasp confirmation unavailable' in warnings[-1]


@pytest.mark.parametrize('failure', ['sensor', 'before_contact', 'after_contact', 'retry_contact', 'observed_contact_loss'])
def test_camera_recovery_does_not_override_motor_loss_or_infrastructure_errors(failure):
    contacts, frames = [], []
    fail_at = {'before_contact': 1, 'after_contact': 2, 'retry_contact': 3, 'observed_contact_loss': 2}.get(failure)

    def confirm(_):
        contacts.append(True)
        if len(contacts) == fail_at:
            raise RuntimeError('lost motor feedback/contact')

    def observe(*args, **kwargs):
        frames.append(True)
        if failure == 'sensor':
            raise RuntimeError('camera unavailable')
        if failure == 'observed_contact_loss':
            return object()
        raise WristTargetNotDetected('no target', 10**9)

    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=3),
        _confirm_held_contact=confirm, _observe_wrist=observe,
        get_logger=lambda: SimpleNamespace(warning=lambda _: None))
    with pytest.raises(RuntimeError, match='unavailable|lost motor'):
        SortingCoordinator._verify_wrist_or_contact(node, SimpleNamespace(selected=None))
    assert len(frames) == (0 if failure == 'before_contact' else 1)


@pytest.mark.parametrize('fault', [None, 'clock_lag', 'duplicate', 'old', 'stale', 'future', 'gap', 'empty', 'moving', 'nan'])
def test_motor_confirmation_requires_recent_distinct_sustained_contact(monkeypatch, fault):
    clock = {'ros': 10 * 10**9, 'wall': 0.0}
    frames = [(0.1, 0.1, .464, 0.), (0.2, 0.2, .464, 0.), (0.4, 0.4, .464, 0.)]
    if fault == 'clock_lag':
        frames = [(.2, .1, .464, 0.), (.2, .2, .464, 0.), (.3, .3, .464, 0.), (.5, .5, .464, 0.)]
    elif fault == 'duplicate':
        frames = [frames[0]]
    elif fault == 'old':
        frames = [(0., .1, .464, 0.)]
    elif fault == 'stale':
        frames = [(.1, .9, .464, 0.)]
    elif fault == 'future':
        frames = [(.2, .1, .464, 0.)]
    elif fault == 'gap':
        frames[1] = (.8, .8, .464, 0.)
    elif fault == 'empty':
        frames[1] = (.2, .2, -.3, 0.)
    elif fault == 'moving':
        frames[1] = (.2, .2, .464, .1)
    elif fault == 'nan':
        frames[1] = (.2, .2, .464, float('nan'))
    params = dict(sorting_wrist_contact_confirmation_sec=.3,
        sorting_gripper_feedback_max_age_sec=.5, gripper_contact_feedback_timeout_sec=1.,
        gripper_open_position_rad=1.2, gripper_contact_min_motion_rad=.05,
        gripper_contact_min_residual_rad=.05, gripper_contact_max_velocity_rad_s=.05)
    joint = 'left_gripper_joint'
    node = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock['ros'])),
        _gripper_feedback_stamps={joint: clock['ros']},
        _joint_positions={joint: .464}, _joint_velocities={joint: 0.},
        _candidate_close_position=lambda _: -.3)
    node._gripper_contact_stalled = lambda *a: NearestPregraspCoordinator._gripper_contact_stalled(node, *a)
    node._require_held_contact = lambda held: SortingCoordinator._require_held_contact(node, held)
    index = 0

    def spin(*args, **kwargs):
        nonlocal index
        clock['wall'] += .1
        stamp, now, actual, velocity = frames[min(index, len(frames) - 1)]
        index += 1
        clock['ros'] = 10 * 10**9 + int(now * 10**9)
        node._gripper_feedback_stamps[joint] = 10 * 10**9 + int(stamp * 10**9)
        node._joint_positions[joint] = actual
        node._joint_velocities[joint] = velocity

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock['wall'])
    held = SimpleNamespace(selected=SimpleNamespace(selected_arm='left', selected_candidate=None))
    if fault not in (None, 'clock_lag'):
        with pytest.raises(RuntimeError, match='feedback|contact'):
            SortingCoordinator._confirm_held_contact(node, held)
    else:
        SortingCoordinator._confirm_held_contact(node, held)
        assert index == (4 if fault == 'clock_lag' else 3)


@pytest.mark.parametrize('fault', [None, 'source', 'frame', 'stamp', 'reference', 'future', 'error', 'handoff'])
def test_only_identified_fresh_check_misses_are_recoverable(fault):
    candidate = GraspCandidate(snapshot_id='head-1')
    candidate.header.frame_id = 'base_link'
    candidate.target_object.object_id = 1
    selected = SimpleNamespace(selected_arm='left', selected_candidate=candidate)
    response = Service.Response(status=Service.Response.NOT_DETECTED, success=False,
        message='no matching detection', reference_id='wrist-1',
        source_snapshot_id='head-1', source_object_id=1)
    response.header.frame_id = 'left_wrist_rgb_optical_frame'
    response.header.stamp.sec = 4
    if fault == 'source': response.source_object_id = 2
    if fault == 'frame': response.header.frame_id = 'right_wrist_rgb_optical_frame'
    if fault == 'stamp': response.header.stamp.sec = 3
    if fault == 'reference': response.reference_id = 'another-target'
    if fault == 'future': response.header.stamp.sec = 6
    if fault == 'error': response.status = response.ERROR
    requests = []
    node = SimpleNamespace(_wrist_reference=SimpleNamespace(reference_id='wrist-1'),
        _lift_completion_stamp_ns=2 * 10**9,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=5)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        _record_pipeline_message=lambda *_: None,
        _wrist_client=SimpleNamespace(call_async=lambda request: requests.append(request) or response),
        _future=lambda result, *_: result)
    operation = Service.Request.HANDOFF if fault == 'handoff' else Service.Request.CHECK
    with pytest.raises(RuntimeError) as caught:
        SortingCoordinator._observe_wrist(node, selected, operation, after_stamp_ns=3 * 10**9)
    if fault is None:
        assert isinstance(caught.value, WristTargetNotDetected)
        assert caught.value.stamp_ns == 4 * 10**9
    else:
        assert not isinstance(caught.value, WristTargetNotDetected)
    if operation == Service.Request.CHECK:
        assert requests[0].after_stamp_ns == 3 * 10**9


def test_motor_fallback_still_requires_kinematic_lift_clearance():
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.z = .3
    pose.orientation.w = 1.
    node = SimpleNamespace(_wrist_enabled=True,
        _held_object=SimpleNamespace(selected=SimpleNamespace(selected_arm='left'), offset_in_tcp=np.zeros(3)),
        _require_held_contact=lambda _: None, _tcp_pose=lambda _: pose,
        _pose_position=NearestPregraspCoordinator._pose_position,
        _verify_wrist_or_contact=lambda _: pytest.fail('No fallback below required lift height'))
    with pytest.raises(RuntimeError, match='lift clearance'):
        SortingCoordinator._verify_lift_height(node, object(), minimum_center_z_m=.4)
