from __future__ import annotations

import math
import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    Candidate,
    EvaluationStage,
    GraspSelectionConfig,
    GraspSelector,
    JointSolution,
    PlannedPath,
    PoseAlignment,
    TrajectoryAlignment,
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


@pytest.mark.parametrize('primary_succeeds', [False, True])
def test_candidate_retry_pass_runs_only_after_primary_candidates_are_exhausted(primary_succeeds):
    from dataclasses import replace
    port = FakePort()
    passes = []
    retry = [False]
    def begin(value):
        retry[0] = value
        passes.append(value)
        return True
    port.begin_candidate_pass = begin
    original = port.solve_position_ik
    def solve(arm, position, seed):
        if seed is None and not retry[0] and not (primary_succeeds and position[0] > .6):
            return None
        return original(arm, position, seed)
    port.solve_position_ik = solve
    selector = GraspSelector(port)
    updates = []
    result = selector.select([candidate(0, y=.2),
        replace(candidate(1, y=.2), position=(.7, .2, .8))], required_arm='left',
        feedback=lambda *args: updates.append(args))
    assert result is not None
    assert result.candidate_index == (1 if primary_succeeds else 0)
    assert passes == ([False] if primary_succeeds else [False, True])
    assert all(0 <= update[0] < 2**32 for update in updates)


class DirectPort(FakePort):
    def solve_pregrasp_ik(self, arm, target, seed):
        self.calls.append(('pregrasp_ik', arm))
        return PoseAlignment(JointSolution((f'{arm}_joint',), (target.position[0],)), target, 0., 0., 0.)

    def solve_aligned_ik(self, arm, target, seed):
        self.calls.append(('grasp_ik', arm, target, seed))
        if ('ik', arm) in self.failures:
            return None
        return PoseAlignment(JointSolution((f'{arm}_joint',), (target.position[0],)), target, 0., 0., 0.)

    def prepare_pregrasps(self, requests):
        self.calls.append(('prepare_pregrasps', requests))

    def plan(self, arm, goal, start):
        self.calls.append(('plan', arm, goal, start))
        if goal.positions[0] < .49 or ('plan', arm) in self.failures:
            return None  # Reproduce an unreachable current-to-pregrasp route.
        return PlannedPath(goal.names, ((0.,), goal.positions))

    def trajectory_alignment(self, *args):
        pytest.fail('A direct joint move must not require intermediate grasp orientation')

    def validate_approach_path(self, *args):
        pytest.fail('A direct joint move must not require a pregrasp approach corridor')

    def pregrasp_is_visible(self, *args):
        pytest.fail('An unused pregrasp must not require visibility')


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_direct_grasp_selection_survives_unused_pregrasp_plan_failure(arm):
    value = candidate(0, y=.2)
    legacy = DirectPort()
    legacy.trajectory_alignment = lambda *_: TrajectoryAlignment(0., 0., 1)
    assert GraspSelector(legacy).select([value], required_arm=arm) is None
    assert any(call[0] == 'pregrasp_ik' for call in legacy.calls)
    direct = DirectPort()
    config = GraspSelectionConfig(pregrasp_motion_enabled=False,
                                  require_approach_corridor=True, require_pregrasp_visibility=True)
    selected = GraspSelector(direct, config).select([value], required_arm=arm)
    assert selected is not None and selected.arm == arm
    assert selected.pregrasp is selected.pregrasp_alignment is selected.pregrasp_path is None
    assert selected.grasp_alignment is not None and selected.grasp_path is not None
    assert not any(call[0] in ('pregrasp_ik', 'prepare_pregrasps') for call in direct.calls)
    plans = [call for call in direct.calls if call[0] == 'plan']
    assert len(plans) == 1 and plans[0][2] == selected.grasp and plans[0][3] is None
    assert 'plan_direct_grasp' in dict(selected.latencies)
    assert 'pregrasp_ik' not in dict(selected.latencies)


@pytest.mark.parametrize('failure', ['ik', 'valid', 'plan'])
def test_direct_grasp_keeps_endpoint_and_path_checks_and_arm_fallback(failure):
    port = DirectPort({(failure, 'left')})
    selector = GraspSelector(port, GraspSelectionConfig(pregrasp_motion_enabled=False))
    selected = selector.select([candidate(0, y=.2)])
    assert selected is not None and selected.arm == 'right'
    assert selector.select([candidate(0, y=.2)], required_arm='left') is None


def test_direct_grasp_still_rejects_gripper_closure_collision():
    port = DirectPort()
    port.gripper_sweep_is_valid = lambda *args: False
    selector = GraspSelector(port, GraspSelectionConfig(pregrasp_motion_enabled=False,
                                                       require_gripper_closure_clearance=True))
    assert selector.select([candidate(0, y=.2)]) is None
    assert not any(call[0] == 'plan' for call in port.calls)


@pytest.mark.parametrize('key', ['pregrasp_motion_enabled', 'direct_grasp_shortcut_enabled'])
@pytest.mark.parametrize('value', [0, None, 'false'])
def test_selection_motion_policy_requires_boolean(key, value):
    with pytest.raises(ValueError, match=key+' must be boolean'):
        GraspSelectionConfig(**{key: value})


@pytest.mark.parametrize(('pregrasp_error', 'grasp_error', 'accepted'), [
    (.009, .001, True),
    (.010, .002, True),
    (.010001, .001, False),
    (.009, .003, False),
])
def test_pregrasp_tolerance_does_not_relax_final_grasp(
    pregrasp_error, grasp_error, accepted,
):
    class AlignedPort(FakePort):
        def solve_pregrasp_ik(self, arm, target, seed):
            self.calls.append(('pregrasp_alignment', arm))
            return PoseAlignment(JointSolution((f'{arm}_joint',), (target.position[0],)),
                                 target, pregrasp_error, 0., 0.)

        def solve_aligned_ik(self, arm, target, seed):
            assert seed is not None
            return PoseAlignment(JointSolution((f'{arm}_joint',), (target.position[0],)),
                                 target, grasp_error, 0., 0.)

    port = AlignedPort()
    config = GraspSelectionConfig(position_tolerance_m=.002,
                                  pregrasp_position_tolerance_m=.010)
    result = GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    assert (result is not None) is accepted
    assert ('pregrasp_alignment', 'left') in port.calls


def test_fixed_jaw_tcp_shift_uses_object_width_and_local_closing_axis():
    config = GraspSelectionConfig(
        asymmetric_gripper_enabled=True,
        fixed_jaw_closing_axis_sign=1.0,
        fixed_jaw_tcp_offset_m=.009849299681842694,
        fixed_jaw_additional_bias_m=.010,
    )
    selector = GraspSelector(FakePort(), config)
    width = .06367146223783493
    expected = width/2-.009849299681842694+.010
    identity = Candidate(
        (.5, .2, .8), (0., -1., 0.), 1., 0,
        required_opening_m=width,
    )
    quarter_turn = Candidate(
        (.5, .2, .8), (1., 0., 0.), 1., 0,
        orientation_xyzw=(0., 0., math.sqrt(.5), math.sqrt(.5)),
        required_opening_m=width,
    )

    assert config.fixed_jaw_tcp_shift_m(width) == pytest.approx(expected)
    assert selector.grasp_tcp_position(identity) == pytest.approx(
        (.5+expected, .2, .8))
    assert selector.grasp_tcp_position(quarter_turn) == pytest.approx(
        (.5, .2+expected, .8))


def test_selector_keeps_fixed_jaw_shift_from_pregrasp_through_grasp():
    port = FakePort()
    config = GraspSelectionConfig(
        asymmetric_gripper_enabled=True,
        fixed_jaw_closing_axis_sign=1.0,
        fixed_jaw_tcp_offset_m=.01,
    )
    value = Candidate(
        (.5, .2, .8), (0., -1., 0.), 1., 0,
        required_opening_m=.04,
    )

    selected = GraspSelector(port, config).select([value], required_arm='left')

    assert selected is not None
    pregrasp = next(call for call in port.calls if call[0] == 'pre_ik')
    grasp = next(call for call in port.calls if call[0] == 'grasp_ik')
    assert pregrasp[2] == pytest.approx((.51, .28, .8))
    assert grasp[2] == pytest.approx((.51, .2, .8))


def test_invalid_fixed_jaw_configuration_is_rejected():
    with pytest.raises(ValueError, match='sign'):
        GraspSelectionConfig(fixed_jaw_closing_axis_sign=0.)
    with pytest.raises(ValueError, match='offset'):
        GraspSelectionConfig(
            fixed_jaw_closing_axis_sign=1., fixed_jaw_tcp_offset_m=-.01)
    with pytest.raises(ValueError, match='additional bias'):
        GraspSelectionConfig(fixed_jaw_additional_bias_m=-.001)


def test_score_order_and_target_y_choose_arm_order():
    selected = GraspSelector(FakePort()).select([
        candidate(0, y=-0.2, score=0.2), candidate(1, y=0.2, score=0.9)
    ])
    assert selected is not None
    assert (selected.candidate_index, selected.arm) == (1, 'left')


def test_required_arm_cannot_fall_back_to_the_other_arm():
    port = FakePort({('pre_ik', 'left', .2)})
    assert GraspSelector(port).select([candidate(0, y=.2)], required_arm='left') is None
    assert not any('right' in call for call in port.calls)


def test_closure_collision_rejects_reachable_endpoints_before_planning():
    port = FakePort()
    port.gripper_sweep_is_valid = lambda *_: False
    result = GraspSelector(port, GraspSelectionConfig(require_gripper_closure_clearance=True)).select(
        [candidate(0, y=.2)])
    assert result is None
    assert not any(call[0].startswith('plan') for call in port.calls)


@pytest.mark.parametrize('closure_sweep', [False, True])
def test_open_grasp_collision_is_rejected_even_without_optional_closure_sweep(closure_sweep):
    port = FakePort()
    openings = []
    port.set_gripper_position = lambda arm, position: openings.append((arm, position))
    valid = port.state_is_valid
    port.state_is_valid = lambda arm, solution: (
        valid(arm, solution) and solution.positions[0] < .49)
    config = GraspSelectionConfig(require_gripper_closure_clearance=closure_sweep)
    value = Candidate((.5, .2, .8), (1., 0., 0.), 1., 0, required_opening_m=.04)

    assert GraspSelector(port, config).select([value], required_arm='left') is None
    assert openings == [('left', config.opening_to_joint_position(.04))]
    assert len([call for call in port.calls if call[0] == 'valid']) == 2
    assert not any(call[0].startswith('plan') for call in port.calls)


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


def test_wrist_motion_is_ranked_before_detector_score_across_feasible_candidates():
    class WristRankedPort(FakePort):
        def wrist_motion_cost(self, _arm, pregrasp, grasp):
            return abs(pregrasp.positions[0]) + abs(
                grasp.positions[0] - pregrasp.positions[0]
            )

    selected = GraspSelector(WristRankedPort()).select([
        candidate(0, y=0.2, score=1.0),
        Candidate((0.2, 0.2, 0.8), (1.0, 0.0, 0.0), 0.5, 1),
    ])

    assert selected is not None
    assert selected.candidate_index == 1


def test_first_feasible_policy_finishes_after_both_paths_pass():
    class WristRankedPort(FakePort):
        def wrist_motion_cost(self, *args):
            pytest.fail('First feasible policy must finish before comparing later candidates')
    port = WristRankedPort()
    selected = GraspSelector(port, GraspSelectionConfig(stop_after_first_feasible=True)).select([
        candidate(0,y=.2,score=1.), candidate(1,y=.3,score=.5)])
    assert selected.candidate_index == 0
    assert [c[0] for c in port.calls].count('plan_current') == 1
    assert [c[0] for c in port.calls].count('plan_grasp') == 1
    assert all(c[2][1] == .2 for c in port.calls if c[0] in ('pre_ik','grasp_ik'))


def test_lazy_scene_port_does_not_report_unstarted_candidate_batches():
    from types import SimpleNamespace
    from cleany_skill_executor.planning_scene import SceneAwarePort

    port = FakePort()
    port.prepares_pregrasps_lazily = True
    prepared, updates = [], []
    port.prepare_pregrasps = prepared.extend
    wrapped = SceneAwarePort(port, SimpleNamespace(disallow_target_contacts=lambda: None))
    with pytest.raises(InterruptedError):
        GraspSelector(wrapped).select(
            [candidate(0, y=.2), candidate(1, y=.3)],
            cancel_requested=lambda: bool(prepared),
            feedback=lambda *args: updates.append(args))

    assert len(prepared) == 4
    assert not updates


def test_approach_plan_uses_pregrasp_as_explicit_start():
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
    ]
    for call in port.calls:
        if call[0] == 'contacts':
            current_contacts = call[1]
        elif call[0] == 'plan_current':
            assert current_contacts is None
        elif call[0] == 'plan_grasp':
            assert current_contacts == 'left'


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


class AlignedFallbackPort:
    def __init__(self):
        self.last_goal_x = 0.0

    def set_target_contacts(self, _arm):
        pass

    def set_gripper_position(self, _arm, _position):
        pass

    def solve_aligned_ik(self, arm, target, _seed):
        solution = JointSolution((f'{arm}_joint',), (target.position[0],))
        approach_error = 40.0 if target.position[0] < 0.4 else 0.0
        return PoseAlignment(solution, target, 0.0, approach_error, 0.0)

    def state_is_valid(self, _arm, solution):
        return round(solution.positions[0], 2) not in (0.42, 0.5)

    def plan(self, _arm, goal, _start):
        self.last_goal_x = goal.positions[0]
        return PlannedPath(goal.names, (goal.positions,))

    def trajectory_alignment(self, _arm, _path, _orientation, _maximum):
        error = 50.0 if 0.55 < self.last_goal_x < 0.65 else 0.0
        return TrajectoryAlignment(error, 0.0, 1)


def test_opening_orientation_collision_and_trajectory_failures_fallback():
    candidates = [
        Candidate((0.3, 0.2, 0.8), (1.0, 0.0, 0.0), 1.0, 0,
                  required_opening_m=0.096),
        Candidate((0.4, 0.2, 0.8), (1.0, 0.0, 0.0), 0.9, 1),
        Candidate((0.5, 0.2, 0.8), (1.0, 0.0, 0.0), 0.8, 2),
        Candidate((0.6, 0.2, 0.8), (1.0, 0.0, 0.0), 0.7, 3),
        Candidate((0.7, 0.2, 0.8), (1.0, 0.0, 0.0), 0.6, 4),
    ]

    selected = GraspSelector(AlignedFallbackPort()).select(candidates)

    assert selected is not None
    assert selected.candidate_index == 4
    assert selected.trajectory_alignment.sampled_waypoints == 1


def test_float32_safe_limit_opening_is_accepted():
    config = GraspSelectionConfig()

    position = config.opening_to_joint_position(0.09500000067)

    assert position < config.gripper_open_joint_rad


class GraspFirstPort(FakePort):
    def __init__(self, failure=''):
        super().__init__()
        self.failure = failure
        self.contacts = None

    def set_target_contacts(self, arm):
        super().set_target_contacts(arm)
        self.contacts = arm

    def prepare_grasps(self, requests):
        self.calls.append(('prepare_grasps', requests))

    def solve_aligned_ik(self, arm, target, seed):
        assert self.contacts == arm
        assert seed is None
        self.calls.append(('grasp_first', arm, target, seed))
        first_candidate = target.position[0] == .5
        if self.failure == 'grasp_ik' and first_candidate:
            return None
        error = .1 if self.failure == 'grasp_alignment' and first_candidate else 0.
        return PoseAlignment(JointSolution((f'{arm}_joint',), (0. if first_candidate else 1.,)),
                             target, error, 0., 0.)

    def solve_pregrasp_ik(self, arm, target, seed):
        assert self.contacts is None
        self.calls.append(('seeded_pregrasp', arm, target, seed))
        if self.failure == 'pregrasp_ik' and seed.positions == (0.,):
            return None
        error = .1 if self.failure == 'pregrasp_alignment' and seed.positions == (0.,) else 0.
        return PoseAlignment(JointSolution(seed.names, (seed.positions[0] + .1,)),
                             target, error, 0., 0.)

    def state_is_valid(self, arm, solution):
        self.calls.append(('valid', arm, solution))
        return not (self.failure == 'collision' and solution.positions == (.1,))

    def plan(self, arm, goal, start):
        self.calls.append(('plan_current' if start is None else 'plan_grasp', arm, goal, start))
        if self.failure == 'current_path' and goal.positions == (.1,):
            return None
        if self.failure == 'approach_path' and goal.positions == (0.,):
            return None
        return True


def test_grasp_first_registers_final_targets_and_seeds_pregrasp_from_grasp():
    port = GraspFirstPort()
    value = candidate(17, y=.2)
    selected = GraspSelector(port, GraspSelectionConfig(grasp_ik_first=True)).select(
        [value], required_arm='left')
    assert selected is not None
    registered = next(c[1] for c in port.calls if c[0] == 'prepare_grasps')
    assert len(registered) == 1 and registered[0].target.position == value.position
    grasp_call = next(c for c in port.calls if c[0] == 'grasp_first')
    pregrasp_call = next(c for c in port.calls if c[0] == 'seeded_pregrasp')
    assert port.calls.index(grasp_call) < port.calls.index(pregrasp_call)
    assert pregrasp_call[2].position == pytest.approx((.42, .2, .8))
    assert pregrasp_call[3] == selected.grasp
    plans = [c for c in port.calls if c[0].startswith('plan_')]
    assert plans == [('plan_grasp', 'left', selected.grasp, selected.pregrasp),
                     ('plan_current', 'left', selected.pregrasp, None)]


@pytest.mark.parametrize('preparation_offset', [.08, .06, .04])
def test_twenty_millimeter_insertion_keeps_preparation_and_plans_deeper_goal(preparation_offset):
    from dataclasses import replace
    config = GraspSelectionConfig(
        grasp_ik_first=True, pregrasp_fallback_offsets_m=(.06, .04),
        asymmetric_gripper_enabled=True, fixed_jaw_tcp_offset_m=.009849299681842694,
        fixed_jaw_additional_bias_m=.020)
    original = candidate(0, y=.2, approach=(.6, .8, 0.))
    baseline = GraspSelector(GraspFirstPort(), config).grasp_tcp_position(original)
    inserted = replace(original, approach_offset_m=.020)
    port = GraspFirstPort()
    solve = port.solve_pregrasp_ik

    def prepare(arm, target, seed):
        result = solve(arm, target, seed)
        return result if math.dist(target.position, baseline) <= preparation_offset + 1e-9 else None

    port.solve_pregrasp_ik = prepare
    selected = GraspSelector(port, config).select([inserted], required_arm='left')
    assert selected is not None
    expected_goal = tuple(p + .020*a for p, a in zip(baseline, original.approach_direction))
    assert selected.grasp_alignment.pose.position == pytest.approx(expected_goal)
    preparations = [c[2] for c in port.calls if c[0] == 'seeded_pregrasp']
    offsets = [.08, .06, .04][:1+[.08, .06, .04].index(preparation_offset)]
    for pose, offset in zip(preparations, offsets, strict=True):
        assert pose.position == pytest.approx(
            GraspSelector.pregrasp_position(original, offset, baseline))
    assert math.dist(preparations[-1].position, expected_goal) == pytest.approx(preparation_offset+.020)
    registered = next(c[1] for c in port.calls if c[0] == 'prepare_grasps')
    assert registered[0].target.position == pytest.approx(expected_goal)
    assert ('valid', 'left', selected.grasp) in port.calls
    assert ('plan_grasp', 'left', selected.grasp, selected.pregrasp) in port.calls


def test_grasp_first_skips_pregrasp_and_planning_if_grasp_has_no_solution():
    port = GraspFirstPort('grasp_ik')
    assert GraspSelector(port, GraspSelectionConfig(grasp_ik_first=True)).select(
        [candidate(0, y=.2)], required_arm='left') is None
    assert not any(c[0] == 'seeded_pregrasp' or c[0].startswith('plan_') for c in port.calls)


@pytest.mark.parametrize('failure', ['no_ik', 'alignment'])
@pytest.mark.parametrize('collision_free', [True, False])
def test_shorter_pregrasp_fallback_reuses_grasp_and_keeps_collision_checks(failure, collision_free):
    from dataclasses import replace
    port = GraspFirstPort()
    solve = port.solve_pregrasp_ik

    def prepare(arm, target, seed):
        result = solve(arm, target, seed)
        if target.position[0] < .43:
            return None if failure == 'no_ik' else replace(result, approach_error_degrees=31.)
        return result

    port.solve_pregrasp_ik = prepare
    port.state_is_valid = lambda *_: collision_free
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offset_m=.06)
    selected = GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    assert (selected is not None) == collision_free
    preparations = [c for c in port.calls if c[0] == 'seeded_pregrasp']
    assert [c[2].position[0] for c in preparations] == pytest.approx([.42, .44])
    assert preparations[0][3] == preparations[1][3]
    assert len([c for c in port.calls if c[0] == 'grasp_first']) == 1
    assert bool([c for c in port.calls if c[0].startswith('plan_')]) == collision_free


def test_shorter_pregrasp_fallback_is_not_used_when_primary_succeeds():
    port = GraspFirstPort()
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offset_m=.06)
    assert GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    preparations = [c for c in port.calls if c[0] == 'seeded_pregrasp']
    assert len(preparations) == 1
    assert preparations[0][2].position[0] == pytest.approx(.42)


@pytest.mark.parametrize('fallbacks', [(.06,), (.06, .04)])
def test_shorter_pregrasp_fallback_uses_selected_endpoint_for_corridor_validation(fallbacks):
    port = GraspFirstPort()
    solve = port.solve_pregrasp_ik
    port.solve_pregrasp_ik = lambda arm, target, seed: (
        solve(arm, target, seed) if target.position[0] > .5-fallbacks[-1]-.001 else None)
    port.plan = lambda arm, goal, start: PlannedPath(goal.names, (goal.positions,))
    port.trajectory_alignment = lambda *_: TrajectoryAlignment(0., 0., 1)
    observed = []
    port.validate_approach_path = lambda arm, path, start, end: observed.append((start, end)) or True
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offsets_m=fallbacks,
                                  require_approach_corridor=True)
    assert GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    assert observed[0][0].position == pytest.approx((.5-fallbacks[-1], .2, .8))
    assert observed[0][1].position == pytest.approx((.5, .2, .8))


def test_shorter_pregrasp_fallback_preserves_cancellation_between_attempts():
    port = GraspFirstPort('pregrasp_ik')
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offset_m=.06)
    with pytest.raises(InterruptedError):
        GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left',
            cancel_requested=lambda: any(c[0] == 'seeded_pregrasp' for c in port.calls))
    assert len([c for c in port.calls if c[0] == 'seeded_pregrasp']) == 1
    assert not any(c[0].startswith('plan_') for c in port.calls)


@pytest.mark.parametrize('valid', [False, True])
def test_approach_planner_hook_owns_validation_and_preserves_selected_endpoints(valid):
    port = GraspFirstPort()
    port.plan = lambda arm, goal, start: PlannedPath(goal.names, (goal.positions,))
    port.trajectory_alignment = lambda *_: TrajectoryAlignment(0., 0., 1)
    observed = []

    def plan_approach(arm, goal, start, start_pose, target_pose):
        observed.append((start_pose, target_pose))
        return PlannedPath(goal.names, (start.positions, goal.positions)) if valid else None

    port.plan_approach = plan_approach
    port.validate_approach_path = lambda *_: pytest.fail('Validated hook must not repeat corridor validation')
    config = GraspSelectionConfig(grasp_ik_first=True, require_approach_corridor=True)
    result = GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    assert (result is not None) is valid
    assert observed[0][0].position == pytest.approx((.42, .2, .8))
    assert observed[0][1].position == pytest.approx((.5, .2, .8))


@pytest.mark.parametrize('approach_ok,alignment_ok', [(False, True), (True, False), (True, True)])
def test_scene_wrapper_checks_approach_before_spending_current_path_plan(approach_ok, alignment_ok):
    from types import SimpleNamespace
    from cleany_skill_executor.planning_scene import SceneAwarePort

    port = GraspFirstPort()
    phases, contacts = [], [None]
    port.set_gripper_position = lambda *_: None
    def approach(arm, goal, start, start_pose, target_pose):
        assert contacts[0] == arm
        phases.append('approach')
        return PlannedPath(goal.names, (start.positions, goal.positions)) if approach_ok else None
    def alignment(*args):
        phases.append('alignment')
        return TrajectoryAlignment(0., 0., 1) if alignment_ok else None
    def current_path(arm, goal, start):
        assert start is None and contacts[0] is None
        phases.append('current_path')
        return PlannedPath(goal.names, ((0.,), goal.positions))
    port.plan_approach, port.plan, port.trajectory_alignment = approach, current_path, alignment
    port.validate_approach_path = lambda *_: pytest.fail('Validated approach was checked twice')
    def set_contacts(arm):
        contacts[0] = arm
        port.set_target_contacts(arm)
    scene = SimpleNamespace(disallow_target_contacts=lambda: set_contacts(None),
                            allow_contacts_for=set_contacts)
    result = GraspSelector(SceneAwarePort(port, scene), GraspSelectionConfig(
        grasp_ik_first=True, require_approach_corridor=True)).select(
            [candidate(0, y=.2)], required_arm='left')
    assert (result is not None) is (approach_ok and alignment_ok)
    assert phases == (['approach'] + (['alignment'] if approach_ok else [])
                      + (['current_path'] if approach_ok and alignment_ok else []))


@pytest.mark.parametrize('offset', [-.01, .08, .09, float('nan'), float('inf')])
def test_shorter_pregrasp_fallback_rejects_invalid_distance(offset):
    with pytest.raises(ValueError, match='pregrasp fallback'):
        GraspSelectionConfig(pregrasp_fallback_offset_m=offset)


@pytest.mark.parametrize('first_success', [.08, .06, .04, None])
@pytest.mark.parametrize('failure', ['no_ik', 'alignment'])
def test_pregrasp_distance_sequence_stops_at_first_valid_ik(first_success, failure):
    from dataclasses import replace
    port = GraspFirstPort()
    solve = port.solve_pregrasp_ik

    def prepare(arm, target, seed):
        result = solve(arm, target, seed)
        distance = round(.5-target.position[0], 2)
        if first_success is None or distance > first_success:
            return None if failure == 'no_ik' else replace(result, approach_error_degrees=31.)
        return result

    port.solve_pregrasp_ik = prepare
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offsets_m=(.06, .04))
    selected = GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left')
    assert (selected is not None) == (first_success is not None)
    preparations = [c for c in port.calls if c[0] == 'seeded_pregrasp']
    expected = [.08, .06, .04][:1+[.08, .06, .04].index(first_success)] if first_success else [.08, .06, .04]
    assert [.5-c[2].position[0] for c in preparations] == pytest.approx(expected)
    assert all(c[3] == preparations[0][3] for c in preparations)
    assert len([c for c in port.calls if c[0] == 'grasp_first']) == 1


def test_four_centimeter_pregrasp_keeps_collision_rejection():
    port = GraspFirstPort()
    solve = port.solve_pregrasp_ik
    port.solve_pregrasp_ik = lambda arm, target, seed: (
        solve(arm, target, seed) if target.position[0] > .45 else None)
    port.state_is_valid = lambda *_: False
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offsets_m=(.06, .04))
    assert GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left') is None
    assert not any(c[0].startswith('plan_') for c in port.calls)


def test_cancellation_after_six_centimeters_prevents_four_centimeter_attempt():
    port = GraspFirstPort('pregrasp_ik')
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_fallback_offsets_m=(.06, .04))
    with pytest.raises(InterruptedError):
        GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left',
            cancel_requested=lambda: len([c for c in port.calls if c[0] == 'seeded_pregrasp']) >= 2)
    assert len([c for c in port.calls if c[0] == 'seeded_pregrasp']) == 2


@pytest.mark.parametrize('offsets', [(.08,), (.04, .06), (.06, .06), (.06, 0.),
                                   (.06, -.01), (.06, float('nan')), (float('inf'),)])
def test_pregrasp_fallback_sequence_rejects_invalid_distances(offsets):
    with pytest.raises(ValueError, match='pregrasp fallback offsets'):
        GraspSelectionConfig(pregrasp_fallback_offsets_m=offsets)


def test_grasp_first_does_not_revisit_failed_candidates_when_port_offers_retry():
    from dataclasses import replace
    port = GraspFirstPort()
    passes, visited = [], []
    def begin(retry):
        passes.append(retry)
        return True
    def fail(arm, target, seed):
        visited.append(target.position)
        return None
    port.begin_candidate_pass = begin
    port.solve_aligned_ik = fail
    first = candidate(0, y=.2)
    second = replace(candidate(1, y=.2, score=.9), position=(.6,.2,.8))
    selected = GraspSelector(port, GraspSelectionConfig(grasp_ik_first=True)).select(
        [first, second], required_arm='left')
    assert selected is None
    assert visited == [first.position, second.position]
    assert passes == [False]
    assert not any(c[0] == 'seeded_pregrasp' or c[0].startswith('plan_') for c in port.calls)


@pytest.mark.parametrize('failure', [
    'grasp_ik', 'grasp_alignment', 'pregrasp_ik', 'pregrasp_alignment',
    'collision', 'current_path', 'approach_path',
])
def test_grasp_first_failure_moves_to_next_candidate_without_extra_grasp_solutions(failure):
    from dataclasses import replace
    port = GraspFirstPort(failure)
    first = candidate(0, y=.2)
    second = replace(candidate(1, y=.2, score=.9), position=(.6,.2,.8))
    selected = GraspSelector(port, GraspSelectionConfig(grasp_ik_first=True)).select(
        [first, second], required_arm='left')
    assert selected is not None and selected.candidate_index == 1
    grasps = [c for c in port.calls if c[0] == 'grasp_first']
    assert [c[2].position for c in grasps] == [first.position, second.position]
    seeds = [c[3] for c in port.calls if c[0] == 'seeded_pregrasp']
    assert [seed.positions for seed in seeds] == ([(1.,)] if failure.startswith('grasp_')
                                                else [(0.,), (1.,)])


def test_grasp_first_cancellation_prevents_pregrasp():
    config = GraspSelectionConfig(grasp_ik_first=True)
    port = GraspFirstPort()
    with pytest.raises(InterruptedError):
        GraspSelector(port, config).select([candidate(0, y=.2)], required_arm='left',
            cancel_requested=lambda: any(c[0] == 'grasp_first' for c in port.calls))
    assert not any(c[0] == 'seeded_pregrasp' for c in port.calls)


def test_grasp_first_setting_is_validated():
    with pytest.raises(ValueError):
        GraspSelectionConfig(grasp_ik_first=1)


def test_zero_margin_accepts_100mm_without_exceeding_open_joint_limit():
    import pytest
    config = GraspSelectionConfig(maximum_gripper_width_m=.1, gripper_safety_margin_m=0.)
    for opening in (.1, .10000000149):
        assert config.opening_to_joint_position(opening) == pytest.approx(config.gripper_open_joint_rad)
        assert config.opening_to_joint_position(opening) <= config.gripper_open_joint_rad
    assert config.opening_to_joint_position(.098) < config.gripper_open_joint_rad
    with pytest.raises(ValueError, match='exceeds'):
        config.opening_to_joint_position(.1001)


def test_invalid_gripper_margins_are_rejected():
    import pytest
    for margin in (-.001, .1, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='margin'):
            GraspSelectionConfig(gripper_safety_margin_m=margin)
