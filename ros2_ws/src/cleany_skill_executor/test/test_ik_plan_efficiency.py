"""Fast proposals keep the existing pose, collision and fallback contracts."""
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, Candidate, GraspSelectionConfig, GraspSelector, InfrastructureError,
    JointSolution, PoseValue,
)
from cleany_skill_executor.hybrid_adapter import HybridConfig
from cleany_skill_executor.curobo_batch import CuRoboBatchAdapter
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
from test_execution_optimization import approach_fixture
from test_hybrid_adapter import make_adapter
from test_physical_grasp_selection import DirectPort, candidate


def seeded_adapter():
    return make_adapter(
        hybrid_config=HybridConfig(seeded_local_ik=True),
        moveit_config=MoveItAdapterConfig(use_verified_local_fk=True,
            pose_refinement_iterations=10, validate_motion_paths=True))


def test_seeded_local_ik_avoids_gpu_and_still_checks_live_scene():
    adapter, trace = seeded_adapter()
    seed = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    target = PoseValue((.42, .28, .3), (0., 0., 0., 1.))
    adapter.set_gripper_position('left', .4)
    states = []
    adapter._validity_client.callback = lambda request: states.append(request.robot_state) or NS(valid=True)
    result = adapter.solve_pregrasp_ik('left', target, seed)
    assert result is not None and result.position_error_m < .002
    assert not any(isinstance(item, dict) for item in trace)
    assert len(states) == 1
    state = dict(zip(states[0].joint_state.name, states[0].joint_state.position))
    assert state['left_gripper_joint'] == .4 and state['right_gripper_joint'] == 0.
    assert adapter.last_ik_statistics['seeded_local_ik_calls'][0]['accepted']


@pytest.mark.parametrize('enabled', [False, True])
def test_selector_wrapper_uses_fast_ik_and_both_validated_connections(enabled):
    from cleany_skill_executor.planning_scene import SceneAwarePort
    adapter, trace = make_adapter(
        hybrid_config=HybridConfig(seeded_local_ik=enabled, progressive_pregrasp=True),
        moveit_config=MoveItAdapterConfig(use_verified_local_fk=True,
            pose_refinement_iterations=10, validate_motion_paths=True,
            pregrasp_joint_shortcut_enabled=True, approach_joint_shortcut_enabled=enabled))
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    selector = GraspSelector(SceneAwarePort(adapter, scene), GraspSelectionConfig(
        grasp_ik_first=True, require_approach_corridor=True, stop_after_first_feasible=True))
    result = selector.select([Candidate((.4, .2, .3), (0., -1., 0.), 1.)], required_arm='left')
    assert result is not None and result.pregrasp_path and result.grasp_path
    assert len([item for item in trace if isinstance(item, dict)]) == (1 if enabled else 2)
    assert trace.count('cpu_rrtconnect_request') == (0 if enabled else 1)
    assert adapter.last_ik_statistics['approach_calls'][0]['route'] == (
        'direct_approach' if enabled else 'rrt')


@pytest.mark.parametrize('failure', ['collision', 'pose'])
def test_seeded_local_failure_preserves_gpu_search(failure):
    adapter, trace = seeded_adapter()
    seed = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    target = PoseValue((.42, .28, .3), (0., 0., 0., 1.))
    adapter.set_gripper_position('left', .4)
    if failure == 'collision':
        checks = iter([False, True])
        adapter._validity_client.callback = lambda _: NS(valid=next(checks))
    else:
        original = adapter._refine_seed
        adapter._refine_seed = lambda *args: replace(original(*args), position_error_m=1.)
    result = adapter.solve_pregrasp_ik('left', target, seed)
    assert result is not None
    assert len([item for item in trace if isinstance(item, dict)]) == 1
    assert not adapter.last_ik_statistics['seeded_local_ik_calls'][0]['accepted']


@pytest.mark.parametrize('failure', ['canceled', 'fk_mismatch', 'wrong_arm'])
def test_seeded_local_infrastructure_errors_do_not_fall_through_to_gpu(failure):
    adapter, trace = seeded_adapter()
    seed = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    target = PoseValue((.42, .28, .3), (0., 0., 0., 1.))
    adapter.set_gripper_position('left', .4)
    if failure == 'canceled':
        adapter.cancel_active()
    elif failure == 'fk_mismatch':
        adapter._remote_forward_kinematics = lambda *_: PoseValue((1., 1., 1.), target.orientation_xyzw)
    else:
        seed = JointSolution(ARM_JOINT_NAMES['right'], seed.positions)
    with pytest.raises((InfrastructureError, InterruptedError)):
        adapter.solve_pregrasp_ik('left', target, seed)
    assert not any(isinstance(item, dict) for item in trace)


def test_direct_approach_avoids_cartesian_and_rrt_but_checks_final_cubic():
    adapter, _, args, calls = approach_fixture()
    adapter._config = replace(adapter._config, approach_joint_shortcut_enabled=True)
    result = adapter.plan_approach(*args)
    assert result.points == (args[2].positions, args[1].positions)
    assert result.times and result.velocities
    assert [kind for kind, _ in calls] == ['collision_batch']
    assert adapter.last_plan_route == 'direct_approach'


@pytest.mark.parametrize('failure', ['corridor', 'collision', 'orientation', 'joint_limit'])
def test_direct_approach_rejections_retain_cartesian_and_rrt(failure):
    import math
    adapter, _, args, calls = approach_fixture()
    adapter._config = replace(adapter._config, approach_joint_shortcut_enabled=True)
    if failure == 'collision':
        adapter._batch_validity.check = lambda *_: NS(first_invalid_index=0)
    elif failure == 'joint_limit':
        adapter._joint_limits[args[1].names[0]] = (-2., .15, 1.)
    else:
        def pose(arm, q):
            middle = .04 < q.positions[0] < .16
            return PoseValue((q.positions[0], .2 if middle and failure == 'corridor' else 0., 0.),
                (0., 0., math.sin(.5), math.cos(.5))
                if middle and failure == 'orientation' else (0., 0., 0., 1.))
        adapter.forward_kinematics = pose
    assert adapter.plan_approach(*args) is None
    assert any(kind == 'cartesian' for kind, _ in calls)
    assert any(kind == 'rrt' for kind, _ in calls)


def test_rrt_retry_does_not_repeat_failed_direct_joint_check():
    adapter, trace = make_adapter(moveit_config=MoveItAdapterConfig(
        validate_motion_paths=True, pregrasp_joint_shortcut_enabled=True))
    goal = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    direct = []
    adapter._direct_joint_path = lambda *args: direct.append(args) or None
    # Reject both final RRT cubics. The stochastic planner still gets two attempts.
    adapter._validated_cubic_path = lambda *_: None
    assert adapter.plan('left', goal, None) is None
    assert len(direct) == 1
    assert trace.count('cpu_rrtconnect_request') == 2


@pytest.mark.parametrize('enabled', [False, True])
def test_exact_candidate_dedup_keeps_distinct_openings_poses_and_arms(enabled):
    first = candidate(0, y=.2)
    values = [first, replace(first, source_index=1, score=.9),
              replace(first, source_index=2, score=.8, required_opening_m=.041),
              replace(first, source_index=3, score=.7, position=(.500001, .2, .8))]
    port = DirectPort({('plan', 'left'), ('plan', 'right')})
    selector = GraspSelector(port, GraspSelectionConfig(
        pregrasp_motion_enabled=False, deduplicate_candidates=enabled))
    assert selector.select(values) is None
    assert len([row for row in port.calls if row[0] == 'plan']) == (6 if enabled else 8)
    # No result is retained across selections or changes in the live scene.
    port.failures.clear()
    selected = selector.select(values)
    assert selected is not None and selected.candidate_index == 0


def test_approach_shortcut_requires_validation():
    with pytest.raises(ValueError, match='full path validation'):
        MoveItAdapterConfig(approach_joint_shortcut_enabled=True)


def test_identical_gripper_override_keeps_fk_but_changes_invalidate_it():
    adapter, _ = seeded_adapter()
    seed = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    adapter.set_gripper_position('left', .4)
    remote = adapter._remote_forward_kinematics
    calls = []
    def fk(*args):
        calls.append(args)
        return remote(*args)
    adapter._remote_forward_kinematics = fk
    for _ in range(3):
        adapter.forward_kinematics('left', seed)
        adapter.set_gripper_position('left', .4)
    assert len(calls) == 1
    adapter.set_gripper_position('left', .400001)
    adapter.forward_kinematics('left', seed)
    assert len(calls) == 2
    adapter.set_gripper_position('right', .4)
    assert adapter._gripper_positions == {'right': .4}
    adapter.forward_kinematics('left', seed)
    assert len(calls) == 3
    adapter.set_current_state(adapter._current_state)
    adapter.forward_kinematics('left', seed)
    assert len(calls) == 4


def test_fk_scoring_evaluates_each_exact_joint_state_once_per_batch():
    adapter = CuRoboBatchAdapter.__new__(CuRoboBatchAdapter)
    adapter._config = MoveItAdapterConfig(align_wrist_roll=True)
    adapter._joint_limits = {ARM_JOINT_NAMES['left'][-1]: (-2., 2., 1.)}
    adapter._compute = lambda fn, *args: fn(*args)
    evaluated = []
    def fk(arm, tip, solutions, state):
        evaluated.extend(solutions)
        return [PoseValue((q.positions[0], state['head_pan_joint'], 0.), (0., 0., 0., 1.))
                for q in solutions]
    adapter._engine = NS(fk=fk)
    adapter._aligned_wrist_variants = lambda q, *_: [q]
    q = JointSolution(ARM_JOINT_NAMES['left'], (0.,)*5)
    targets = [PoseValue((0., 0., 0.), (0., 0., 0., 1.)),
               PoseValue((1., 0., 0.), (0., 0., 0., 1.))]
    result = adapter._score_batch('left', targets, [[q]*16, [q]*16], {'head_pan_joint': 0.})
    assert len(evaluated) == len(set(evaluated)) == 5
    assert len(result[0]) == len(result[1]) == 5
    assert all(item.position_error_m == 0. for item in result[0])
    assert all(item.position_error_m == 1. for item in result[1])
    evaluated.clear()
    result = adapter._score_batch('left', targets, [[q], [q]], {'head_pan_joint': 1.})
    assert len(evaluated) == 5
    assert all(item.position_error_m == 1. for item in result[0])
