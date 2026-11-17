"""GPU endpoint limits stay separate from physical state/path bounds."""
from contextlib import nullcontext
from copy import deepcopy
from types import SimpleNamespace as NS
from xml.etree import ElementTree as ET

import pytest

from cleany_skill_executor.core.grasp_approach_bounds import GraspApproachYawBounds
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


@pytest.mark.parametrize('arm,expected', [('right', (0., 1.7)), ('left', (-1.8, 0.))])
def test_gpu_proposal_urdf_uses_approach_yaw_for_only_the_active_arm(monkeypatch, tmp_path, arm, expected):
    original = tmp_path/'runtime.urdf'
    names = (*ARM_JOINT_NAMES['left'], *ARM_JOINT_NAMES['right'])
    xml = ET.Element('robot')
    for name in names:
        ET.SubElement(ET.SubElement(xml, 'joint', name=name), 'limit', lower='-2', upper='2')
    description = ET.tostring(xml, encoding='unicode')
    original.write_text(description)
    locked = {name: 1.95 for name in names if name not in ARM_JOINT_NAMES[arm]}
    robot = {'kinematics': {'urdf_path': str(original), 'lock_joints': locked,
        'cspace': {'joint_names': names, 'retract_config': [1.95]*len(names)}}}
    monkeypatch.setattr(RobotModel, 'robot_config', lambda *_: deepcopy(robot))
    model = _CandidateRobotModel.__new__(_CandidateRobotModel)
    model.xml, model.limits = description, dict.fromkeys(names, (-2., 2.))
    model._exclude_opposite_arm_cost, model._endpoint_margin_rad = False, .1
    model._grasp_approach_yaw = GraspApproachYawBounds((-1.8, 0.), (0., 1.7))

    kin = model.robot_config(str(original), arm, locked, arm+'_grasp_tcp')['kinematics']
    proposal = ET.parse(kin['urdf_path'])
    for joint in proposal.findall('joint'):
        limit = joint.find('limit')
        expected_range = expected if joint.get('name') == ARM_JOINT_NAMES[arm][0] else (-2., 2.)
        assert (float(limit.get('lower')), float(limit.get('upper'))) == expected_range
    assert original.read_text() == description
    assert model.limits == dict.fromkeys(names, (-2., 2.)) and kin['lock_joints'] == locked
    assert kin['cspace']['retract_config'][names.index(ARM_JOINT_NAMES[arm][0])] == pytest.approx(expected[1]-.1)


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
