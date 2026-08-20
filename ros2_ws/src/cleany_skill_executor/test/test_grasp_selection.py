from __future__ import annotations

import pytest

from cleany_skill_executor.core.grasp_selection import Candidate, EvaluationStage, GraspSelector, JointSolution


def candidate(index, *, y, score=1.0, approach=(1.0, 0.0, 0.0)):
    return Candidate((0.5, y, 0.8), approach, score, index)


class FakePort:
    def __init__(self, failures=()):
        self.failures = set(failures)
        self.calls = []

    def set_target_contacts(self, arm):
        self.calls.append(('contacts', arm))

    def solve_position_ik(self, arm, position, seed):
        stage = 'pre_ik' if seed is None else 'grasp_ik'
        self.calls.append((stage, arm, position, seed))
        if (stage, arm, round(position[1], 2)) in self.failures:
            return None
        return JointSolution((f'{arm}_joint',), (position[0],))

    def state_is_valid(self, arm, solution):
        self.calls.append(('valid', arm, solution))
        return ('valid', arm) not in self.failures

    def plan(self, arm, goal, start):
        stage = 'plan_current' if start is None else 'plan_grasp'
        self.calls.append((stage, arm, goal, start))
        return (stage, arm) not in self.failures


def test_pregrasp_is_eight_centimeters_opposite_normalized_approach():
    value = GraspSelector.pregrasp_position(candidate(0, y=0.2, approach=(2.0, 0.0, 0.0)))
    assert value == pytest.approx((0.42, 0.2, 0.8))


def test_score_order_and_target_y_choose_arm_order():
    selected = GraspSelector(FakePort()).select([
        candidate(0, y=-0.2, score=0.2), candidate(1, y=0.2, score=0.9)
    ])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (1, 'left')


@pytest.mark.parametrize('failure', [('pre_ik', 'left', 0.2), ('valid', 'left'), ('plan_current', 'left'), ('plan_grasp', 'left')])
def test_pair_failure_falls_back_to_other_arm(failure):
    selected = GraspSelector(FakePort({failure})).select([candidate(3, y=0.2)])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (3, 'right')


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
        ('contacts', 'left'),
        ('contacts', None),
        ('contacts', 'left'),
        ('contacts', None),
        ('contacts', 'left'),
    ]


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
