from types import SimpleNamespace as NS
import numpy as np
import pytest
from cleany_interfaces.msg import GraspCandidate
from sensor_msgs.msg import JointState
from cleany_skill_executor.core.fixed_jaw_clearance import Bounds, ClearanceConfig
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, Candidate, GraspSelectionConfig, GraspSelector
from cleany_skill_executor.core.hardware_execution import CollisionValidationError
from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode
from cleany_skill_executor.hardware_execution import HardwareExecution


JAW = Bounds((.008, -.006, -.005), (.016, .06, .005))


def candidate_message():
    message = GraspCandidate()
    message.target_object.obb_pose.orientation.w = 1.
    message.target_object.obb_size.x = .04
    message.target_object.obb_size.y = .03
    message.target_object.obb_size.z = .04
    return message


def selection_node(maximum=32):
    config = GraspSelectionConfig(asymmetric_gripper_enabled=True,
        fixed_jaw_tcp_offset_m=.008, fixed_jaw_additional_bias_m=.010,
        maximum_candidates=maximum)
    logs = []
    node = NS(_selector=GraspSelector(NS(), config),
              _fixed_jaw_config=ClearanceConfig(.005, .010),
              _fixed_jaw_models={'left': JAW, 'right': JAW},
              get_logger=lambda: NS(info=logs.append))
    return node, logs


def candidate(index=0, x=0., score=1.):
    return Candidate((x, 0., 0.), (0., -1., 0.), score, index, required_opening_m=.04)


def prepare(node, candidates, messages, arm='', canceled=lambda: False):
    return GraspSelectionNode._prepare_fixed_jaw_candidates(
        node, candidates, messages, arm, lambda *args: None, canceled)


def test_correction_preserves_source_and_shifts_pregrasp_and_grasp_equally():
    node, logs = selection_node()
    value = candidate(x=-.008)
    corrected, = prepare(node, [value], [candidate_message()])
    assert corrected.source_index == value.source_index
    assert corrected.position == value.position  # object/candidate reference stays fixed
    before = node._selector.grasp_tcp_position(value)
    after = node._selector.grasp_tcp_position(corrected)
    assert after[0] - before[0] == pytest.approx(.003)
    pre_before = node._selector.pregrasp_position(value, .08, before)
    pre_after = node._selector.pregrasp_position(corrected, .08, after)
    assert np.subtract(pre_after, pre_before) == pytest.approx(np.subtract(after, before))
    assert 'additional_correction_m' in logs[0]


def test_existing_ten_mm_bias_is_not_added_twice():
    node, _ = selection_node()
    value = candidate()
    corrected, = prepare(node, [value], [candidate_message()], 'right')
    assert corrected.lateral_offset_m == pytest.approx(.022)
    assert node._selector.grasp_tcp_position(corrected) == node._selector.grasp_tcp_position(value)


def test_fixed_finger_clearance_does_not_require_the_object_between_fingers():
    node, _ = selection_node()
    # Jaw and object do not overlap even though the object misses the aperture.
    value = candidate(x=-.2)
    corrected, = prepare(node, [value], [candidate_message()], 'right')
    assert corrected.lateral_offset_m == pytest.approx(.022)


def test_rejected_correction_does_not_promote_candidates_outside_the_ik_budget():
    node, logs = selection_node(maximum=1)
    values = [candidate(0, -.02, 2.), candidate(1, 0., 1.)]
    assert prepare(node, values, [candidate_message(), candidate_message()], 'right') == []
    assert 'accepted": false' in logs[0]


def test_correction_cancellation_stops_before_batch_ik():
    node, _ = selection_node()
    with pytest.raises(InterruptedError):
        prepare(node, [candidate()], [candidate_message()], canceled=lambda: True)


def test_guard_receives_retimed_samples_and_mandatory_hold_bridge_before_goal():
    names = ARM_JOINT_NAMES['right']
    measured = JointState(name=[*names, 'right_gripper_joint'], position=[0.] * 6)
    trace = []
    def guard(q, index, phase):
        trace.append((index, phase, q[names[0]], q['right_gripper_joint']))
        if index == 1:
            raise CollisionValidationError('fixed finger contact', phase=phase, sample_index=index)
    real = NS(monitor=lambda: None, _approach_clearance_guard=guard, validity=object(),
              node=NS(call=lambda *args: NS(valid=True)))
    samples = [NS(positions=(x, 0., 0., 0., 0.)) for x in (.02, .04, .06)]
    with pytest.raises(CollisionValidationError) as error:
        HardwareExecution.validate_collision_samples(real, 'right', names, samples,
            dict(zip(measured.name, measured.position)), measured, hold_bridge_samples=2)
    assert trace == [(0, 'hold_bridge', .02, 0.), (1, 'hold_bridge', .04, 0.)]
    assert error.value.entry_blocked
