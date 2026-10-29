"""Candidate scheduling tests use fake planning responses, never robot goals."""
from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, Candidate, GraspSelectionConfig, GraspSelector, InfrastructureError,
    JointSolution, PlannedPath, PoseAlignment, TrajectoryAlignment,
)
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig
from cleany_skill_executor.planning_scene import SceneAwarePort
from test_execution_optimization import approach_fixture
from test_hybrid_adapter import make_adapter


class CandidatePort:
    def __init__(self, blocked=(.2,), failed_stage='approach'):
        self.blocked, self.failed_stage = blocked, failed_stage
        self.fast = False
        self.calls = []
        self.valid = True

    @contextmanager
    def planning_pass(self, *, fast_only):
        previous, self.fast = self.fast, fast_only
        try:
            yield
        finally:
            self.fast = previous

    def set_target_contacts(self, arm):
        self.calls.append(('contacts', arm))

    def solve_aligned_ik(self, arm, target, seed):
        self.calls.append(('ik', target.position[1], seed is None))
        return PoseAlignment(JointSolution(('q',), (target.position[1],)), target, 0., 0., 0.)

    def state_is_valid(self, arm, solution):
        self.calls.append(('valid', solution.positions[0]))
        return self.valid

    def gripper_sweep_is_valid(self, *args):
        self.calls.append(('sweep',))
        return self.valid

    def plan_approach(self, arm, goal, start, *_):
        return self._path('approach', goal)

    def plan(self, arm, goal, start):
        return self._path('entry', goal)

    def _path(self, stage, goal):
        value = goal.positions[0]
        self.calls.append((stage, value, self.fast))
        if self.fast and value in self.blocked and stage == self.failed_stage:
            return None
        return PlannedPath(goal.names, ((0.,), goal.positions))

    def trajectory_alignment(self, *args):
        return TrajectoryAlignment(0., 0., 2)


def selection_fixture(port=None, **options):
    port = port or CandidatePort()
    config = GraspSelectionConfig(grasp_ik_first=True, stop_after_first_feasible=True,
        fast_candidate_paths_first=True, require_approach_corridor=True,
        require_gripper_closure_clearance=True, **options)
    candidates = [Candidate((.5, y, .8), (1., 0., 0.), 1., source_index=i)
                  for i, y in enumerate((.2, .3))]
    return port, GraspSelector(port, config), candidates


@pytest.mark.parametrize('stage', ['approach', 'entry'])
def test_fast_candidate_avoids_earlier_rrt_for_both_connections(stage):
    port, selector, candidates = selection_fixture(CandidatePort(failed_stage=stage))
    selected = selector.select(candidates, required_arm='left')
    assert selected.candidate_index == 1
    paths = [row for row in port.calls if row[0] in ('approach', 'entry')]
    assert all(row[2] for row in paths)
    assert paths[0][1] == .2 and paths[-1][1] == .3
    assert not port.fast


def test_fast_candidate_fallback_reuses_only_ik_and_rechecks_live_scene():
    port, selector, candidates = selection_fixture(CandidatePort(blocked=(.2, .3)))
    selected = selector.select(candidates, required_arm='left')
    assert selected.candidate_index == 0
    assert [row for row in port.calls if row[0] == 'approach'] == [
        ('approach', .2, True), ('approach', .3, True), ('approach', .2, False)]
    assert sum(row[0] == 'ik' for row in port.calls) == 4
    assert sum(row[0] == 'valid' for row in port.calls) == 6
    assert sum(row[0] == 'sweep' for row in port.calls) == 3
    selector.select(candidates, required_arm='left')
    assert sum(row[0] == 'ik' for row in port.calls) == 8  # No cross-request cache.


@pytest.mark.parametrize('stop', ['collision', 'cancel'])
def test_fast_candidate_changed_scene_and_cancellation_block_deferred_rrt(stop):
    port, selector, candidates = selection_fixture(CandidatePort(blocked=(.2, .3)))
    canceled = False

    def feedback(*args):
        nonlocal canceled
        if args[-1] == 'fast paths exhausted; trying deferred RRT':
            if stop == 'collision':
                port.valid = False
            else:
                canceled = True

    if stop == 'cancel':
        with pytest.raises(InterruptedError):
            selector.select(candidates, required_arm='left', feedback=feedback,
                            cancel_requested=lambda: canceled)
    else:
        assert selector.select(candidates, required_arm='left', feedback=feedback) is None
    assert not any(row[0] in ('approach', 'entry') and not row[2] for row in port.calls)
    assert not port.fast


def test_fast_candidate_legacy_policy_remains_candidate_first():
    port, selector, candidates = selection_fixture()
    selector._config = replace(selector._config, fast_candidate_paths_first=False)
    assert selector.select(candidates, required_arm='left').candidate_index == 0
    assert not any(row[0] == 'ik' and row[1] == .3 for row in port.calls)


def test_fast_candidate_requires_scoped_port_and_compatible_policy():
    with pytest.raises(ValueError):
        GraspSelectionConfig(fast_candidate_paths_first=True)
    port, selector, candidates = selection_fixture()
    port.planning_pass = None
    with pytest.raises(InfrastructureError, match='scoped planning port'):
        selector.select(candidates, required_arm='left')


def test_fast_candidate_cartesian_fallback_is_deferred_and_scope_restored():
    adapter, _, args, calls = approach_fixture(fraction=.9)
    with adapter.planning_pass(fast_only=True):
        assert adapter.plan_approach(*args) is None
        assert adapter.last_plan_route == 'rrt_deferred'
    assert [kind for kind, _ in calls] == ['cartesian']
    assert adapter.plan_approach(*args) is not None
    assert sum(kind == 'rrt' for kind, _ in calls) == 1
    with pytest.raises(InterruptedError):
        with adapter.planning_pass(fast_only=True):
            adapter._canceled = True
            adapter.plan_approach(*args)
    assert not adapter._fast_path_only


def test_fast_candidate_hybrid_wrapper_does_not_repeat_direct_check_or_send_rrt():
    adapter, trace = make_adapter(moveit_config=MoveItAdapterConfig(
        validate_motion_paths=True, pregrasp_joint_shortcut_enabled=True))
    scene = NS(disallow_target_contacts=lambda: None, allow_contacts_for=lambda arm: None)
    wrapped = SceneAwarePort(adapter, scene)
    direct = []
    adapter._direct_joint_path = lambda *args: direct.append(args) or None
    adapter._validated_cubic_path = lambda *_: None
    goal = JointSolution(ARM_JOINT_NAMES['left'], (.4, .28, .3, 0., 0.))
    with wrapped.planning_pass(fast_only=True):
        assert wrapped.plan('left', goal, None) is None
    assert len(direct) == 1 and trace.count('cpu_rrtconnect_request') == 0
    assert wrapped.plan('left', goal, None) is None
    assert trace.count('cpu_rrtconnect_request') == 2
