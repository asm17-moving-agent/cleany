from __future__ import annotations

import pytest
from dataclasses import replace

from cleany_skill_executor.core.grasp_selection import (
    Candidate,
    EvaluationStage,
    GraspSelectionConfig,
    GraspSelector,
    JointSolution,
    directed_axis_error_deg,
    unsigned_axis_error_deg,
)


def candidate(index, *, y, score=1.0, approach=(1.0, 0.0, 0.0)):
    return Candidate((0.5, y, 0.8), approach, score, index)


class FakePort:
    def __init__(self, failures=()):
        self.failures = set(failures)
        self.calls = []

    def set_target_contacts(self, arm):
        self.calls.append(('contacts', arm))

    def solve_position_ik(self, arm, position, seed):
        stage = 'pre_ik'
        self.calls.append((stage, arm, position, seed))
        if (stage, arm, round(position[1], 2)) in self.failures:
            return None
        return JointSolution((f'{arm}_joint',), (position[0],))

    def solve_aimed_pregrasp_ik(
        self,
        arm,
        grasp_position,
        approach_direction,
        closing_direction,
        pregrasp_position,
        seed,
    ):
        self.calls.append(
            (
                'aim_ik',
                arm,
                grasp_position,
                approach_direction,
                closing_direction,
                pregrasp_position,
                seed,
            )
        )
        if ('aim_ik', arm) in self.failures:
            return ()
        return (
            seed
            or JointSolution((f'{arm}_joint',), (pregrasp_position[0],)),
        )

    def solve_grasp_ik(
        self,
        arm,
        grasp_position,
        approach_direction,
        closing_direction,
        seed,
    ):
        self.calls.append(
            (
                'grasp_ik',
                arm,
                grasp_position,
                approach_direction,
                closing_direction,
                seed,
            )
        )
        if ('grasp_ik', arm) in self.failures:
            return ()
        return (JointSolution((f'{arm}_joint',), (grasp_position[0],)),)

    def state_is_valid(self, arm, solution):
        self.calls.append(('valid', arm, solution))
        return ('valid', arm) not in self.failures

    def open_grasp_is_valid(self, arm, solution, opening):
        self.calls.append(('open_clearance', arm, opening))
        return ('open_clearance', arm) not in self.failures

    def gripper_sweep_is_valid(self, arm, solution, opening, closing, step):
        self.calls.append(('closure_clearance', arm, opening, closing, step))
        return ('closure_clearance', arm) not in self.failures

    def pregrasp_is_visible(self, arm, solution):
        self.calls.append(('visible', arm, solution))
        return ('visible', arm) not in self.failures

    def plan(self, arm, goal, start):
        stage = 'plan_current' if start is None else 'plan_grasp'
        self.calls.append((stage, arm, goal, start))
        return (stage, arm) not in self.failures


def test_open_target_overlap_rejects_arm_before_grasp_plan():
    port = FakePort(failures={('open_clearance', 'left')})
    selector = GraspSelector(port, GraspSelectionConfig(open_gripper_position_rad=1.4))
    selected = selector.select([candidate(0, y=.2)])
    assert selected is not None and selected.arm == 'right'
    assert ('open_clearance', 'left', 1.4) in port.calls
    assert not any(c[0] == 'plan_grasp' and c[1] == 'left' for c in port.calls)


def test_closure_collision_rejects_arm_before_grasp_plan():
    port = FakePort(failures={('closure_clearance', 'left')})
    selector = GraspSelector(port, GraspSelectionConfig(
        open_gripper_position_rad=1.4, closed_gripper_position_rad=-.3))
    selected = selector.select([candidate(0, y=.2)])
    assert selected is not None and selected.arm == 'right'
    assert ('closure_clearance', 'left', 1.4, -.3, .05) in port.calls
    assert not any(c[0] == 'plan_grasp' and c[1] == 'left' for c in port.calls)


def test_pregrasp_is_fourteen_centimeters_opposite_normalized_approach():
    value = GraspSelector.pregrasp_position(candidate(0, y=0.2, approach=(2.0, 0.0, 0.0)))
    assert value == pytest.approx((0.36, 0.2, 0.8))


def test_candidate_centering_overrides_both_old_lateral_offsets():
    port = FakePort()
    item = replace(candidate(0,y=.2), lateral_offset_m=.004)
    selector = GraspSelector(port,GraspSelectionConfig(
        grasp_lateral_offset_m=.03,grasp_execution_lateral_offset_m=.03))
    assert selector.select([item]) is not None
    expected = GraspSelector.grasp_position(item,0,.004)
    assert next(c for c in port.calls if c[0]=='grasp_ik')[2] == pytest.approx(expected)


def test_occluded_pregrasp_falls_back_before_planning():
    port = FakePort({('visible', 'left')})
    selector = GraspSelector(port, GraspSelectionConfig(require_pregrasp_visibility=True))
    result = selector.select([candidate(0, y=.2)])
    assert result.arm == 'right'
    assert not any(c[0].startswith('plan') and c[1] == 'left' for c in port.calls)
    assert sum(c[0] == 'visible' for c in port.calls) == 2  # Never at grasp endpoint.


def test_visibility_default_does_not_change_generic_selection():
    port = FakePort({('visible', 'left')})
    result = GraspSelector(port).select([candidate(0, y=.2)])
    assert result.arm == 'left'
    assert not any(c[0] == 'visible' for c in port.calls)


def test_candidate_preserves_normalized_pose_and_rejects_mismatched_approach():
    value = Candidate(
        (0.5, 0.2, 0.8),
        (1.0, 0.0, 0.0),
        1.0,
        orientation=(0.0, 0.0, 2**-0.5, 2**-0.5),
    )
    assert value.orientation == pytest.approx(
        (0.0, 0.0, 2**-0.5, 2**-0.5)
    )
    assert value.closing_direction == pytest.approx((0.0, 1.0, 0.0))
    with pytest.raises(ValueError, match='local -Y'):
        Candidate(
            (0.5, 0.2, 0.8),
            (0.0, 1.0, 0.0),
            1.0,
            orientation=(0.0, 0.0, 2**-0.5, 2**-0.5),
        )


def test_directed_approach_and_unsigned_closing_axis_errors():
    assert directed_axis_error_deg((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)) == 180.0
    assert unsigned_axis_error_deg((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)) == 0.0


def test_candidate_approach_is_passed_to_direction_aware_ik():
    port = FakePort()
    selected = GraspSelector(port).select([
        candidate(0, y=0.2, approach=(0.0, -2.0, 0.0))
    ])

    aimed = next(call for call in port.calls if call[0] == 'aim_ik')
    assert selected is not None
    assert aimed[2] == (0.5, 0.2, 0.8)
    assert aimed[3] == (0.0, -1.0, 0.0)
    assert aimed[4] == pytest.approx((-1.0, 0.0, 0.0))
    assert aimed[5] == pytest.approx((0.5, 0.34, 0.8))
    seed = next(call for call in port.calls if call[0] == 'pre_ik')
    assert seed[2] == pytest.approx((0.5, 0.28, 0.8))


def test_tcp_calibration_keeps_pregrasp_and_grasp_laterally_aligned() -> None:
    port = FakePort()
    item = candidate(0, y=0.2, approach=(0.0, -1.0, 0.0))
    selector = GraspSelector(
        port,
        GraspSelectionConfig(
            grasp_approach_offset_m=0.025,
            grasp_lateral_offset_m=0.027,
        ),
    )

    selected = selector.select([item])

    aimed = next(call for call in port.calls if call[0] == 'aim_ik')
    grasp = next(call for call in port.calls if call[0] == 'grasp_ik')
    assert selected is not None
    expected_aim = tuple(
        position + 0.027 * closing
        for position, closing in zip(
            item.position, item.closing_direction, strict=True
        )
    )
    assert aimed[2] == pytest.approx(expected_aim)
    assert aimed[5] == pytest.approx(
        tuple(
            position - 0.14 * approach
            for position, approach in zip(
                expected_aim, item.approach_direction, strict=True
            )
        )
    )
    assert grasp[2] == pytest.approx(
        tuple(
            position + 0.025 * approach + 0.027 * closing
            for position, approach, closing in zip(
                item.position,
                item.approach_direction,
                item.closing_direction,
                strict=True,
            )
        )
    )
    delta = tuple(
        grasp_value - pregrasp_value
        for grasp_value, pregrasp_value in zip(
            grasp[2], aimed[5], strict=True
        )
    )
    assert delta == pytest.approx(
        tuple(
            (0.14 + 0.025) * approach
            for approach in item.approach_direction
        )
    )


def test_direction_aware_ik_failure_falls_back_to_other_arm():
    selected = GraspSelector(FakePort({('aim_ik', 'left')})).select([
        candidate(3, y=0.2)
    ])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (3, 'right')


def test_score_order_and_target_y_choose_arm_order():
    selected = GraspSelector(FakePort()).select([
        candidate(0, y=-0.2, score=0.2), candidate(1, y=0.2, score=0.9)
    ])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (1, 'left')


def test_required_arm_disables_bilateral_fallback():
    port = FakePort()
    selected = GraspSelector(port).select(
        [candidate(0, y=0.2)], required_arm='right'
    )
    assert selected is not None and selected.arm == 'right'
    assert not any(len(call) > 1 and call[1] == 'left' for call in port.calls)
    with pytest.raises(ValueError, match='required_arm'):
        GraspSelector(FakePort()).select(
            [candidate(0, y=0.2)], required_arm='middle'
        )


@pytest.mark.parametrize('failure', [('valid', 'left'), ('plan_current', 'left'), ('plan_grasp', 'left')])
def test_pair_failure_falls_back_to_other_arm(failure):
    selected = GraspSelector(FakePort({failure})).select([candidate(3, y=0.2)])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (3, 'right')


def test_seed_ik_failure_tries_aimed_ik_from_current_state():
    port = FakePort({('pre_ik', 'left', 0.2)})

    selected = GraspSelector(port).select([candidate(3, y=0.2)])

    assert selected is not None
    assert selected.arm == 'left'
    aimed = next(call for call in port.calls if call[0] == 'aim_ik')
    assert aimed[-1] is None


def test_failed_high_score_candidate_falls_back_to_next_candidate():
    port = FakePort()
    calls = 0

    def restore_validity(arm, solution):
        nonlocal calls
        calls += 1
        return calls > 2

    port.state_is_valid = restore_validity
    selected = GraspSelector(port).select([
        candidate(0, y=0.2, score=1.0), candidate(1, y=-0.2, score=0.5)
    ])
    assert selected is not None
    assert selected.candidate_index == 1


def test_second_plan_uses_pregrasp_as_explicit_start():
    port = FakePort()
    selected = GraspSelector(port).select([candidate(0, y=0.2)])
    grasp_plan = next(call for call in port.calls if call[0] == 'plan_grasp')
    assert selected is not None and grasp_plan[3] == selected.pregrasp


def test_target_contact_is_disallowed_for_pregrasp_and_allowed_for_grasp():
    port = FakePort()
    GraspSelector(port).select([candidate(0, y=0.2)])

    contacts = [call for call in port.calls if call[0] == 'contacts']
    assert contacts == [
        ('contacts', None),
        ('contacts', None),
        ('contacts', 'left'),
    ]


def test_planning_failure_tries_next_ik_solution_before_other_arm():
    port = FakePort()
    first = JointSolution(('left_joint',), (0.1,))
    second = JointSolution(('left_joint',), (0.2,))
    port.solve_aimed_pregrasp_ik = lambda *_: (first, second)

    def plan(arm, goal, start):
        stage = 'plan_current' if start is None else 'plan_grasp'
        port.calls.append((stage, arm, goal, start))
        return goal != first

    port.plan = plan
    selected = GraspSelector(port).select([candidate(0, y=0.2)])

    assert selected is not None
    assert selected.arm == 'left'
    assert selected.pregrasp == second
    pregrasp_plans = [call for call in port.calls if call[0] == 'plan_current']
    assert [call[2] for call in pregrasp_plans] == [first, second]


def test_all_pairs_failed_returns_none_and_cancel_stops_immediately():
    assert GraspSelector(FakePort({('valid', 'left'), ('valid', 'right')})).select([candidate(0, y=0.2)]) is None
    with pytest.raises(InterruptedError):
        GraspSelector(FakePort()).select([candidate(0, y=0.2)], cancel_requested=lambda: True)


def test_cancel_after_blocking_stage_prevents_later_stages():
    port = FakePort()

    def canceled():
        return any(call[0] == 'pre_ik' for call in port.calls)

    with pytest.raises(InterruptedError):
        GraspSelector(port).select(
            [candidate(0, y=0.2)],
            cancel_requested=canceled,
        )

    assert not any(call[0] == 'grasp_ik' for call in port.calls)


def test_cancel_after_final_plan_prevents_success_result():
    port = FakePort()

    def canceled():
        return any(call[0] == 'plan_grasp' for call in port.calls)

    with pytest.raises(InterruptedError):
        GraspSelector(port).select(
            [candidate(0, y=0.2)],
            cancel_requested=canceled,
        )


def test_feedback_exposes_each_stage():
    updates = []
    GraspSelector(FakePort()).select([candidate(0, y=0.2)], feedback=lambda *args: updates.append(args))
    assert {item[2] for item in updates} == set(EvaluationStage)
