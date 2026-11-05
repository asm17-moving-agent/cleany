"""Ranked Hybrid checks preserve winners and retain interruption/scene guards."""
from types import SimpleNamespace
import threading

import pytest
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PlanningScene

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, InfrastructureError, JointSolution, PoseAlignment, PoseValue,
)
from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter
from cleany_skill_executor.hybrid_adapter import _CandidateGpuAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


def alignment(index, roll, approach=0., position=.001):
    return PoseAlignment(
        JointSolution(ARM_JOINT_NAMES['left'], (index*.01, 0., 0., 0., roll)),
        PoseValue((0., 0., 0.), (0., 0., 0., 1.)), position, approach, 0.)


def adapter(valid):
    calls = []
    gpu = _CandidateGpuAdapter.__new__(_CandidateGpuAdapter)
    gpu._config = MoveItAdapterConfig()
    gpu._current_state = JointState(name=list(ARM_JOINT_NAMES['left']), position=[0.]*5)
    gpu._owner = SimpleNamespace(
        _check_active=lambda: None,
        state_is_valid=lambda arm, q: calls.append(q) is None and valid[q])
    return gpu, calls


@pytest.mark.parametrize('roll', [None, -.5, .5])
def test_ranked_validation_matches_eager_winner_with_ties_and_rejections(roll):
    rejected = alignment(0, 0.)
    tied_a, tied_b = alignment(1, .5), alignment(2, .5)
    wrong = alignment(3, 0., position=.1)
    far = alignment(4, -.8)
    groups = [[far, rejected, tied_a, tied_b, wrong], [], [tied_b, tied_a]]
    valid = {item.solution: item is not rejected for group in groups for item in group}
    gpu, calls = adapter(valid)
    gpu._valid_batch = lambda arm, solutions: [valid[q] for q in solutions]
    eager = CuRoboBatchAdapter._choose_batch(gpu, 'left', groups, [roll]*3)
    chosen = gpu._choose_batch('left', groups, [roll]*3)
    assert all(a is b for a, b in zip(eager, chosen))
    assert len(calls) < sum(gpu._feasible_alignment(item) for group in groups for item in group)
    assert gpu.last_batch_diagnostics[1]['collision_checked'] == 0
    assert gpu.last_batch_diagnostics[2]['collision_unchecked'] == 1


def test_invalid_group_is_fully_checked_and_single_target_uses_same_policy():
    group = [alignment(0, -.5), alignment(1, .5)]
    gpu, calls = adapter({item.solution: False for item in group})
    assert gpu._choose_valid('left', group) is None
    assert calls == [item.solution for item in group]
    diagnostic = gpu.last_batch_diagnostics[0]
    assert diagnostic['collision_valid_count_complete']
    assert diagnostic['collision_checked'] == 2 and diagnostic['collision_unchecked'] == 0


def test_grouped_gpu_proposals_validate_each_actual_gripper_opening():
    first, second = alignment(0, 0.), alignment(1, 0.)
    gpu, _ = adapter({first.solution: True, second.solution: True})
    current, calls = [None], []
    gpu.validation_openings = [.4, .8]
    gpu._owner.set_gripper_position = lambda arm, opening: current.__setitem__(0, opening)
    def validate(arm, solution):
        calls.append((solution, current[0]))
        return current[0] == (.4 if solution == first.solution else .8)
    gpu._owner.state_is_valid = validate
    assert gpu._choose_batch('left', [[first], [second]]) == [first, second]
    assert calls == [(first.solution,.4), (second.solution,.8)]


@pytest.mark.parametrize('reason', ['cancel', 'deadline'])
def test_interruption_between_checks_stops_without_choosing_a_state(reason):
    group = [alignment(0, 0.), alignment(1, .5)]
    gpu, calls = adapter({item.solution: False for item in group})
    owner = MoveItGraspAdapter.__new__(MoveItGraspAdapter)
    owner._monotonic = lambda: 10.
    owner._deadline, owner._canceled = 20., False
    def validate(arm, solution):
        calls.append(solution)
        if reason == 'cancel':
            owner._canceled = True
        else:
            owner._deadline = 10.
        return False
    owner.state_is_valid = validate
    gpu._owner = owner
    with pytest.raises(InterruptedError):
        gpu._choose_valid('left', group)
    assert calls == [group[0].solution]


def test_rpc_failure_propagates_instead_of_falling_back_to_lower_rank():
    group = [alignment(0, 0.), alignment(1, .5)]
    gpu, calls = adapter({item.solution: True for item in group})
    def failure(arm, q):
        calls.append(q)
        raise InfrastructureError('state validity timed out')
    gpu._owner.state_is_valid = failure
    with pytest.raises(InfrastructureError, match='timed out'):
        gpu._choose_valid('left', group)
    assert calls == [group[0].solution]


def test_scene_change_is_rejected_even_after_first_state_passes():
    group = [alignment(0, 0.), alignment(1, .5)]
    gpu, calls = adapter({item.solution: True for item in group})
    gpu._cancel_event = threading.Event()
    gpu._monotonic, gpu._deadline = lambda: 0., float('inf')
    gpu._active_scene = None
    first, changed = PlanningScene(), PlanningScene()
    changed.allowed_collision_matrix.entry_names = ['changed_link']
    scenes = iter([first, changed])
    gpu._snapshot = lambda: next(scenes)
    with pytest.raises(InfrastructureError, match='scene changed'):
        with gpu._scene_operation():
            assert gpu._choose_valid('left', group) is group[0]
    assert len(calls) == 1 and gpu._active_scene is None
