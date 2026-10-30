"""Direction ordering must reduce redundant planning without bypassing validation."""

from dataclasses import replace
import math

import pytest

from cleany_skill_executor.core.alignment import tcp_quaternion_for_approach
from cleany_skill_executor.core.grasp_direction import (
    GraspDirectionConfig, grasp_direction_pairs, order_grasp_directions,
)
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, Candidate, GraspSelectionConfig, GraspSelector,
    JointSolution, PoseAlignment, PoseValue,
)


def orientation(angle_degrees):
    angle = math.radians(angle_degrees)
    return tcp_quaternion_for_approach((1., 0., 0.), (0., math.cos(angle), math.sin(angle)))


def candidates():
    return tuple(Candidate((.5, .2, .8), (1., 0., 0.), .8, index,
                           orientation_xyzw=orientation(angle))
                 for index, angle in enumerate((0., 180.)))


class DirectionPort:
    prepares_pregrasps_lazily = True

    def __init__(self, current_angles=None, failure=''):
        self.current_angles = current_angles or {'left': 180., 'right': 0.}
        self.failure = failure
        self.calls = []
        self.index = -1
        self.requests = []

    def current_tcp_pose(self, arm):
        self.calls.append(('current_fk', arm))
        return PoseValue((.1, .2, .3), orientation(self.current_angles[arm]))

    def prepare_pregrasps(self, requests):
        self.requests = requests

    def set_ik_context(self, index, arm, target, *, pregrasp):
        self.index = index

    def set_target_contacts(self, arm):
        self.calls.append(('contacts', arm))

    def set_gripper_position(self, arm, value):
        self.calls.append(('opening', arm, value))

    def _solve(self, stage, arm, target):
        self.calls.append((stage, arm, self.index, target))
        if self.index == 1 and self.failure == stage:
            return None
        solution = JointSolution(ARM_JOINT_NAMES[arm], (0.,) * 5)
        return PoseAlignment(solution, target, 0., 0., 0.)

    def solve_pregrasp_ik(self, arm, target, seed):
        return self._solve('pregrasp_ik', arm, target)

    def solve_aligned_ik(self, arm, target, seed):
        return self._solve('grasp_ik', arm, target)

    def state_is_valid(self, arm, solution):
        self.calls.append(('valid', arm, self.index))
        return not (self.index == 1 and self.failure == 'valid')

    def gripper_sweep_is_valid(self, arm, *args):
        self.calls.append(('sweep', arm, self.index))
        return not (self.index == 1 and self.failure == 'sweep')

    def plan(self, arm, goal, start):
        stage = 'plan_current' if start is None else 'plan_grasp'
        self.calls.append((stage, arm, self.index))
        return not (self.index == 1 and self.failure == stage)

    def wrist_motion_cost(self, *args):
        pytest.fail('First-feasible ordering must not fully evaluate both directions')


def selection_config(**kwargs):
    return GraspSelectionConfig(
        stop_after_first_feasible=True, require_gripper_closure_clearance=True,
        asymmetric_gripper_enabled=True, fixed_jaw_tcp_offset_m=.0098,
        fixed_jaw_additional_bias_m=.01,
        direction_preference=GraspDirectionConfig(enabled=True), **kwargs)


@pytest.mark.parametrize('pregrasp', [True, False])
@pytest.mark.parametrize('arm,index', [('left', 1), ('right', 0)])
def test_near_direction_uses_each_arms_measured_pose_without_extra_ik_or_plan(pregrasp, arm, index):
    port = DirectionPort()
    config = selection_config(pregrasp_motion_enabled=pregrasp)
    selected = GraspSelector(port, config).select(candidates(), required_arm=arm)
    assert selected.candidate_index == index
    assert [call for call in port.calls if call[0] == 'current_fk'] == [('current_fk', arm)]
    assert [call[2] for call in port.calls if call[0] == 'grasp_ik'] == [index]
    assert sum(call[0] == 'pregrasp_ik' for call in port.calls) == int(pregrasp)
    assert sum(call[0].startswith('plan_') for call in port.calls) == 1 + int(pregrasp)
    assert all(call[2] == index for call in port.calls if call[0] in ('valid', 'sweep'))
    assert sum(call[0] == 'sweep' for call in port.calls) == 1
    if pregrasp:
        assert [request.candidate_index for request in port.requests] == [index, 1 - index]
    else:
        assert port.requests == []
    # The selected orientation still gets its own asymmetric TCP translation.
    expected = GraspSelector(port, config).grasp_tcp_position(candidates()[index])
    assert selected.grasp_alignment.pose.position == pytest.approx(expected)


@pytest.mark.parametrize('failure', ['pregrasp_ik', 'grasp_ik', 'valid', 'sweep', 'plan_current', 'plan_grasp'])
def test_preferred_direction_failure_checks_reverse_on_same_arm_before_other_arm(failure):
    port = DirectionPort(failure=failure)
    selected = GraspSelector(port, selection_config()).select(candidates())
    assert selected.candidate_index == 0 and selected.arm == 'left'
    assert [call[2] for call in port.calls if call[0] == 'pregrasp_ik'] == [1, 0]
    assert all(call[1] == 'left' for call in port.calls if call[0].endswith('_ik'))


@pytest.mark.parametrize('angle,expected', [(92., 0), (93., 1), (90., 0), (179., 1)])
def test_left_fixed_jaw_breaks_only_estimated_movement_ties(angle, expected):
    port = DirectionPort({'left': angle})
    selected = GraspSelector(port, selection_config()).select(candidates(), required_arm='left')
    assert selected.candidate_index == expected


def test_left_preference_uses_the_configured_physical_fixed_jaw_side():
    pair = candidates()
    ordered = order_grasp_directions(pair, orientation(90.), GraspDirectionConfig(), -1.)
    assert ordered[0].source_index == 1


def test_direction_pair_is_ordered_before_candidate_budget_is_applied():
    port = DirectionPort()
    selected = GraspSelector(port, selection_config(maximum_candidates=1)).select(
        candidates(), required_arm='left')
    assert selected.candidate_index == 1
    assert len(port.requests) == 1
    assert [call[2] for call in port.calls if call[0] == 'pregrasp_ik'] == [1]


@pytest.mark.parametrize('change', [
    {'position': (.5001, .2, .8)}, {'required_opening_m': .05}, {'depth_m': .01},
    {'approach_offset_m': .01}, {'lateral_offset_m': .01}, {'score': .7},
    {'orientation_xyzw': orientation(160.)}, {'orientation_xyzw': orientation(0.)},
    {'orientation_xyzw': tcp_quaternion_for_approach((0., 0., -1.), (0., -1., 0.))},
])
def test_distinct_grasps_are_not_reordered_or_sent_for_current_fk(change):
    first, second = candidates()
    second = replace(second, **change)
    port = DirectionPort()
    selected = GraspSelector(port, selection_config()).select([first, second], required_arm='left')
    assert selected.candidate_index == 0
    assert not any(call[0] == 'current_fk' for call in port.calls)


def test_grasp_quality_and_original_source_indices_survive_pairing():
    first, second = candidates()
    highest = replace(first, score=.9, source_index=20, position=(.6, .2, .8))
    another = replace(first, source_index=30, position=(.7, .2, .8))
    port = DirectionPort()
    selector = GraspSelector(port, selection_config())
    order = selector._evaluation_order([highest, first, another, second], 'left', lambda: None)
    assert [item.source_index for item, _ in order] == [20, 1, 0, 30]
    assert [item.score for item, _ in order] == [.9, .8, .8, .8]
    assert len([call for call in port.calls if call[0] == 'current_fk']) == 1


def test_degenerate_current_closing_projection_keeps_original_order():
    current = tcp_quaternion_for_approach((0., -1., 0.), (1., 0., 0.))
    pair = candidates()  # Their approach is +X, parallel to current closing.
    assert order_grasp_directions(pair, current, GraspDirectionConfig(), 1.) == pair


def test_pairing_obeys_small_transport_rounding_tolerances():
    first, second = candidates()
    rounded = replace(second, position=(.500001, .2, .8), score=.8 + 1e-8,
                      required_opening_m=.040000001, orientation_xyzw=orientation(179.9))
    assert grasp_direction_pairs([first, rounded], GraspDirectionConfig()) == [(first, rounded)]


def test_scene_port_forwards_current_pose_without_changing_target_contacts():
    from types import SimpleNamespace
    from cleany_skill_executor.planning_scene import SceneAwarePort
    def unexpected(*args):
        pytest.fail('Direction ordering must not mutate target contact permissions')
    adapter = DirectionPort()
    port = SceneAwarePort(adapter, SimpleNamespace(
        allow_contacts_for=unexpected, disallow_target_contacts=unexpected))
    result = port.current_tcp_pose('left')
    assert result.orientation_xyzw == orientation(180.)
    assert adapter.calls == [('current_fk', 'left')]


@pytest.mark.parametrize('values', [
    {'enabled': 1}, {'movement_tie_degrees': -1.}, {'movement_tie_degrees': 90.},
    {'movement_tie_degrees': math.nan}, {'pair_position_tolerance_m': 0.},
    {'pair_position_tolerance_m': math.inf}, {'pair_axis_tolerance_degrees': 90.},
    {'pair_axis_tolerance_degrees': 0.}, {'pair_score_tolerance': -1.},
    {'pair_score_tolerance': math.nan},
])
def test_invalid_direction_preferences_are_rejected(values):
    with pytest.raises(ValueError):
        GraspDirectionConfig(**values)
