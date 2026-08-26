from __future__ import annotations

import pytest

from cleany_skill_executor.core.grasp_selection import (
    Candidate,
    EvaluationStage,
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

    def plan(self, arm, goal, start):
        stage = 'plan_current' if start is None else 'plan_grasp'
        self.calls.append((stage, arm, goal, start))
        return (stage, arm) not in self.failures


def test_pregrasp_is_fourteen_centimeters_opposite_normalized_approach():
    value = GraspSelector.pregrasp_position(candidate(0, y=0.2, approach=(2.0, 0.0, 0.0)))
    assert value == pytest.approx((0.36, 0.2, 0.8))


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
