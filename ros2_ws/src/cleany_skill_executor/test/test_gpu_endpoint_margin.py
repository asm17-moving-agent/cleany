"""GPU endpoint limits stay separate from physical state/path bounds."""
from contextlib import nullcontext
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, JointSolution, PoseAlignment, PoseValue,
)
from cleany_skill_executor.curobo_model import RobotModel
from cleany_skill_executor.hybrid_adapter import HybridConfig, HybridGraspAdapter, _CandidateRobotModel


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_gpu_clip_does_not_change_physical_limits_or_locked_joints(monkeypatch, arm):
    names = [*ARM_JOINT_NAMES['left'], *ARM_JOINT_NAMES['right'], 'left_gripper_joint']
    state = dict.fromkeys(names, 1.95)
    limits = dict.fromkeys(names, (-2., 2.))
    original = {'kinematics': {'cspace': {'joint_names': names, 'retract_config': list(state.values()),
        'position_limit_clip': 0.}, 'lock_joints': {
            name: value for name, value in state.items() if name not in ARM_JOINT_NAMES[arm]}}}
    monkeypatch.setattr(RobotModel, 'robot_config', lambda *_: deepcopy(original))
    model = _CandidateRobotModel.__new__(_CandidateRobotModel)
    model._exclude_opposite_arm_cost, model._endpoint_margin_rad = False, .1
    model.limits = deepcopy(limits)
    result = model.robot_config('runtime.urdf', arm, state, arm+'_grasp_tcp')['kinematics']
    assert result['cspace']['position_limit_clip'] == .1
    assert result['lock_joints'] == original['kinematics']['lock_joints']
    assert model.limits == limits and set(state.values()) == {1.95}
    assert result['cspace']['retract_config'] == [
        1.9 if name in ARM_JOINT_NAMES[arm] else 1.95 for name in names]


@pytest.mark.parametrize('angle,accepted', [(1.89, True), (1.95, False)])
def test_final_gpu_proposal_respects_endpoint_margin_before_dispatch(angle, accepted):
    adapter = HybridGraspAdapter.__new__(HybridGraspAdapter)
    adapter._hybrid_config = HybridConfig(endpoint_margin_rad=.1)
    adapter._check_active = lambda: None
    adapter._feasible_alignment = lambda _: True
    adapter._planning_position_bounds = lambda _: (-2., 2.)
    adapter._progress = NS(stage=lambda *_: nullcontext())
    target = PoseValue((.3, .1, .4), (0., 0., 0., 1.))
    adapter.forward_kinematics = lambda *_: target
    checked = []
    adapter.state_is_valid = lambda *args: checked.append(args) or True
    result = PoseAlignment(JointSolution(ARM_JOINT_NAMES['left'], (angle, 0., 0., 0., 0.)),
                           target, 0., 0., 0.)
    actual = adapter._accept_alignment('left', target, result)
    assert (actual is not None) is accepted
    assert bool(checked) is accepted


@pytest.mark.parametrize('margin', [-.1, float('nan'), float('inf')])
def test_gpu_endpoint_margin_rejects_invalid_values(margin):
    with pytest.raises(ValueError, match='GPU IK endpoint margin'):
        HybridConfig(endpoint_margin_rad=margin)
