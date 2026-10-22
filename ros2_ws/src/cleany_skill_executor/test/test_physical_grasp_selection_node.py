from types import SimpleNamespace
import threading

import pytest
from cleany_interfaces.msg import GraspCandidate
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.physical_grasp_selection import (
    InfrastructureError,
    JointSolution,
    Selection,
)
from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode


def test_startup_warmup_runs_once_and_never_overlaps_an_active_request():
    from std_srvs.srv import Trigger
    calls = []
    parameters = {'curobo_ik_warmup_timeout_sec': 180.,
        'curobo_ik_warmup_arms': ['right', 'left'], 'curobo_ik_warmup_include_aim': True}
    adapter = SimpleNamespace(_description='runtime',
        set_current_state=lambda state: calls.append(('state', state)),
        set_deadline=lambda deadline: None, set_progress_callback=lambda callback: None,
        warmup=lambda arms, **kwargs: calls.append(('warmup', arms, kwargs)))
    logger = SimpleNamespace(info=lambda _: None, error=lambda _: None, warning=lambda _: None)
    node = SimpleNamespace(_warmup_ready=False, _warmup_error='', _description_ready=True,
        _description_error='', _goal_active=True, _adapter_lock=threading.RLock(), _adapter=adapter,
        _current_joint_state=lambda: 'fresh state', get_parameter=lambda key: SimpleNamespace(value=parameters[key]),
        get_logger=lambda: logger)
    GraspSelectionNode._try_warmup(node)
    assert not calls
    node._goal_active = False
    GraspSelectionNode._try_warmup(node)
    assert calls[0] == ('state', 'fresh state')
    assert calls[1][:2] == ('warmup', ['right', 'left'])
    assert calls[1][2]['include_aim'] is True
    continue_work = calls[1][2]['continue_work']
    assert continue_work()
    node._goal_active = True
    assert not continue_work()
    node._goal_active = False
    assert node._warmup_ready and node._warmup_description == 'runtime'
    GraspSelectionNode._try_warmup(node)
    assert len(calls) == 2
    assert GraspSelectionNode._warmup_status(node, Trigger.Request(), Trigger.Response()).success


def test_cancel_while_waiting_for_gpu_lock_finishes_action_and_releases_goal_ownership():
    from cleany_interfaces.action import SelectReachableGrasp
    lock = threading.Lock()
    lock.acquire()
    node = SimpleNamespace(_adapter_lock=lock, _goal_lock=threading.Lock(), _goal_active=True,
        _wait_for_description=lambda *_: None,
        get_parameter=lambda _: SimpleNamespace(value=120.),
        _set_failure=GraspSelectionNode._set_failure,
        _infrastructure_error_code=GraspSelectionNode._infrastructure_error_code)
    terminals = []
    goal = SimpleNamespace(is_cancel_requested=True, canceled=lambda: terminals.append('canceled'),
        abort=lambda: terminals.append('abort'))
    try:
        result = GraspSelectionNode._execute(node, goal)
    finally:
        lock.release()
    assert result.error_code == SelectReachableGrasp.Result.ERROR_CANCELED
    assert terminals == ['canceled'] and not node._goal_active


def test_unused_pregrasp_is_transported_as_empty_joint_state():
    message = GraspSelectionNode._joint_message(None)
    assert message.name == [] and len(message.position) == 0


def _waiting_node(timeout=1.0):
    return SimpleNamespace(_requires_description=True, _description_ready=False,
        _description_error='', get_parameter=lambda _: SimpleNamespace(value=timeout))


def test_hybrid_waits_until_runtime_model_is_applied(monkeypatch):
    node = _waiting_node()
    applied = []
    node._adapter = SimpleNamespace(set_robot_description=applied.append)
    def receive(_):
        GraspSelectionNode._on_description(node, SimpleNamespace(data='runtime model with head joints'))
    monkeypatch.setattr('cleany_skill_executor.physical_grasp_selection_node.time.sleep', receive)
    GraspSelectionNode._wait_for_description(node, SimpleNamespace(is_cancel_requested=False), float('inf'))
    assert applied == ['runtime model with head joints']
    assert node._description_ready


@pytest.mark.parametrize('deadline', [.5, float('inf')])
def test_runtime_model_wait_obeys_startup_timeout_and_action_deadline(monkeypatch, deadline):
    times = iter([0.0, 1.0])
    monkeypatch.setattr('cleany_skill_executor.physical_grasp_selection_node.time.monotonic', lambda: next(times))
    with pytest.raises(InfrastructureError, match='startup timeout'):
        GraspSelectionNode._wait_for_description(_waiting_node(),
            SimpleNamespace(is_cancel_requested=False), deadline)


def test_runtime_model_wait_can_be_canceled():
    with pytest.raises(InterruptedError):
        GraspSelectionNode._wait_for_description(_waiting_node(),
            SimpleNamespace(is_cancel_requested=True), float('inf'))


def test_invalid_runtime_model_stops_wait_immediately():
    node = _waiting_node()
    node._description_error = 'Invalid runtime joint limits'
    with pytest.raises(InfrastructureError, match='Invalid runtime'):
        GraspSelectionNode._wait_for_description(node,
            SimpleNamespace(is_cancel_requested=False), float('inf'))


def test_moveit_keeps_its_existing_optional_model_behavior():
    GraspSelectionNode._wait_for_description(SimpleNamespace(_requires_description=False), None, 0.0)


def _candidate(frame_id='base_link'):
    message = GraspCandidate()
    message.header.frame_id = frame_id
    message.snapshot_id = 'snapshot'
    message.object_id = 1
    message.target_object.object_id = 1
    message.target_object.obb_size.x = 0.1
    message.target_object.obb_size.y = 0.1
    message.target_object.obb_size.z = 0.2
    message.target_object.obb_pose.orientation.w = 1.0
    message.tcp_pose.position.x = 0.5
    message.tcp_pose.position.y = 0.2
    message.tcp_pose.position.z = 0.8
    message.approach_direction.y = -1.0
    message.required_opening_m = 0.04
    message.score = 1.0
    return message


def test_candidates_must_use_configured_planning_frame():
    with pytest.raises(ValueError, match='configured planning_frame'):
        GraspSelectionNode._validate_candidates(
            [_candidate('camera_frame')],
            'base_link',
        )


def test_candidate_quaternion_and_approach_axis_must_agree():
    value = _candidate()
    value.approach_direction.y = 0.0
    value.approach_direction.x = 1.0
    with pytest.raises(ValueError, match='TCP local -Y'):
        GraspSelectionNode._validate_candidates([value], 'base_link')


class _GoalHandle:
    def __init__(self, candidate):
        self.request = SimpleNamespace(candidates=[candidate], required_arm='')
        self.is_cancel_requested = False
        self.terminal_state = ''

    def publish_feedback(self, _message):
        pass

    def succeed(self):
        self.terminal_state = 'succeeded'

    def canceled(self):
        if not self.is_cancel_requested:
            raise RuntimeError('CANCELED requires a cancellation request')
        self.terminal_state = 'canceled'

    def abort(self):
        self.terminal_state = 'aborted'


class _Scene:
    active = False

    def begin(self, candidate, object_id):
        pass

    def restore(self):
        raise InfrastructureError('failed to restore planning scene')


class _Selector:
    def select(self, candidates, **_kwargs):
        candidate = candidates[0]
        solution = JointSolution(('left_joint',), (0.1,))
        return Selection(0, 'left', candidate, solution, solution)


def test_restore_failure_overrides_success_and_aborts_action():
    selected_successfully = []
    parameters = {'action_timeout_sec': 120.0, 'grasp_approach_offset_m': 0.0,
        'deeper_grasp_labels': [''], 'deeper_grasp_offsets_m': [0.0],
        'grasp_use_aperture_centering': False, 'grasp_lateral_offset_m': 0.0,
        'asymmetric_gripper_enabled': False}
    logger = SimpleNamespace(error=lambda _message: None, info=lambda _message: None)
    node = SimpleNamespace(
        _current_joint_state=lambda: JointState(),
        _sensor_scene=SimpleNamespace(wait=lambda **_: None, check=lambda: None),
        _description_error='',
        _wait_for_description=lambda *_: None,
        _visibility_tf=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10)),
        _adapter=SimpleNamespace(
            set_current_state=lambda _state: None,
            set_visibility_constraint=lambda _: None,
            cancel_active=lambda: None,
        ),
        _scene=_Scene(),
        _scene_port=SimpleNamespace(reset=lambda: None),
        _selector=_Selector(),
        _goal_lock=threading.Lock(),
        _goal_active=True,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: logger,
        _joint_message=GraspSelectionNode._joint_message,
        _set_failure=GraspSelectionNode._set_failure,
        _set_metrics=lambda result, _: selected_successfully.append(result.success),
        _infrastructure_error_code=(
            GraspSelectionNode._infrastructure_error_code
        ),
    )
    goal = _GoalHandle(_candidate())

    result = GraspSelectionNode._execute(node, goal)

    assert selected_successfully == [True]
    assert goal.terminal_state == 'aborted'
    assert result.success is False
    assert result.error_code == result.ERROR_PLANNING_SCENE
    assert result.selected_candidate_index == -1
    assert result.selected_arm == ''
    assert 'Failed to restore planning scene' in result.message
    assert node._goal_active is False


@pytest.mark.parametrize('cancel_requested, expected_state', [(False, 'aborted'), (True, 'canceled')])
def test_interrupted_selection_returns_valid_result_and_ros_terminal_state(
        cancel_requested, expected_state):
    def interrupted(*args):
        raise InterruptedError('evaluation deadline exceeded')
    node = SimpleNamespace(
        _wait_for_description=interrupted,
        get_parameter=lambda _: SimpleNamespace(value=120.0),
        _adapter=SimpleNamespace(set_visibility_constraint=lambda _: None),
        _goal_lock=threading.Lock(), _goal_active=True,
        _set_failure=GraspSelectionNode._set_failure)
    goal = _GoalHandle(_candidate())
    goal.is_cancel_requested = cancel_requested
    result = GraspSelectionNode._execute(node, goal)
    assert goal.terminal_state == expected_state
    assert not result.success and result.error_code == result.ERROR_CANCELED
    assert 'evaluation deadline exceeded' in result.message
    assert not node._goal_active
