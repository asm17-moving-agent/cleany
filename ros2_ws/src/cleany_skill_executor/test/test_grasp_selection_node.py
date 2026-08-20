from types import SimpleNamespace
import threading

import pytest
from cleany_interfaces.msg import GraspCandidate
from sensor_msgs.msg import JointState

from cleany_skill_executor.core.grasp_selection import (
    InfrastructureError,
    JointSolution,
    Selection,
)
from cleany_skill_executor.grasp_selection_node import GraspSelectionNode


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
    message.approach_direction.x = 1.0
    message.score = 1.0
    return message


def test_candidates_must_use_configured_planning_frame():
    with pytest.raises(ValueError, match='configured planning_frame'):
        GraspSelectionNode._validate_candidates(
            [_candidate('camera_frame')],
            'base_link',
        )


class _GoalHandle:
    def __init__(self, candidate):
        self.request = SimpleNamespace(candidates=[candidate])
        self.is_cancel_requested = False
        self.terminal_state = ''

    def publish_feedback(self, _message):
        pass

    def succeed(self):
        self.terminal_state = 'succeeded'

    def canceled(self):
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
    logger = SimpleNamespace(error=lambda _message: None, info=lambda _message: None)
    node = SimpleNamespace(
        _current_joint_state=lambda: JointState(),
        _adapter=SimpleNamespace(
            set_current_state=lambda _state: None,
            cancel_active=lambda: None,
        ),
        _scene=_Scene(),
        _scene_port=SimpleNamespace(reset=lambda: None),
        _selector=_Selector(),
        _goal_lock=threading.Lock(),
        _goal_active=True,
        get_parameter=lambda _name: SimpleNamespace(value=120.0),
        get_logger=lambda: logger,
        _joint_message=GraspSelectionNode._joint_message,
        _set_failure=GraspSelectionNode._set_failure,
        _infrastructure_error_code=(
            GraspSelectionNode._infrastructure_error_code
        ),
    )
    goal = _GoalHandle(_candidate())

    result = GraspSelectionNode._execute(node, goal)

    assert goal.terminal_state == 'aborted'
    assert result.success is False
    assert result.error_code == result.ERROR_PLANNING_SCENE
    assert result.selected_candidate_index == -1
    assert result.selected_arm == ''
    assert 'Failed to restore planning scene' in result.message
    assert node._goal_active is False
