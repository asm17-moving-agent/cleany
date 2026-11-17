"""Shared CPU/GPU selection policy tests without DDS or motor access."""
from contextlib import contextmanager
import math

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, Candidate, GraspSelectionConfig, GraspSelector,
    JointSolution, PoseAlignment,
)


class Port:
    def __init__(self):
        self.prepared = []
        self.pregrasp_targets = []
        self.grasp_targets = []
        self.openings = []
        self.plans = []
        self.fast_only = False
        self.blocked = False
        self.minimum_pregrasp_x = -math.inf

    def set_target_contacts(self, arm):
        pass

    def set_gripper_position(self, arm, opening):
        self.openings.append(opening)

    def prepare_grasps(self, requests):
        self.prepared.extend(requests)

    prepare_pregrasps = prepare_grasps

    def solve_aligned_ik(self, arm, target, seed):
        self.grasp_targets.append(target)
        return PoseAlignment(JointSolution(ARM_JOINT_NAMES[arm], (target.position[0], 0., 0., 0., 0.)),
                             target, 0., 0., 0.)

    def solve_pregrasp_ik(self, arm, target, seed):
        self.pregrasp_targets.append(target)
        if target.position[0] < self.minimum_pregrasp_x:
            return None
        return PoseAlignment(JointSolution(ARM_JOINT_NAMES[arm], (target.position[0], 0., 0., 0., 0.)),
                             target, 0., 0., 0.)

    def state_is_valid(self, arm, solution):
        return not self.blocked

    def plan(self, arm, goal, start):
        self.plans.append((self.fast_only, goal.positions[0], start))
        return not self.fast_only

    @contextmanager
    def planning_pass(self, *, fast_only):
        old, self.fast_only = self.fast_only, fast_only
        try:
            yield
        finally:
            self.fast_only = old


@pytest.mark.parametrize('grasp_first', [True, False])
@pytest.mark.parametrize('arm', ['left', 'right'])
def test_full_open_and_z_offset_keep_final_tcp(grasp_first, arm):
    port = Port()
    value = Candidate((.5, .2, .8), (.6, 0., -.8), 1., required_opening_m=.02)
    config = GraspSelectionConfig(grasp_ik_first=grasp_first, pregrasp_z_offset_m=.010,
                                  gripper_open_joint_rad=2.4173120563240813)
    result = GraspSelector(port, config).select([value], required_arm=arm)
    assert result is not None
    baseline = GraspSelector.pregrasp_position(value)
    assert port.pregrasp_targets[0].position == pytest.approx(
        (baseline[0], baseline[1], baseline[2]+.010))
    assert port.grasp_targets[0].position == value.position
    assert port.prepared[0].gripper_position == config.gripper_open_joint_rad
    assert result.gripper_joint_position == config.gripper_open_joint_rad
    assert port.openings == [config.gripper_open_joint_rad]


def test_all_pregrasp_fallbacks_keep_the_same_z_shift():
    port = Port()
    port.minimum_pregrasp_x = .459
    value = Candidate((.5, .2, .8), (1., 0., 0.), 1.)
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_z_offset_m=.010,
                                  pregrasp_fallback_offsets_m=(.06, .04))
    assert GraspSelector(port, config).select([value], required_arm='left')
    assert len(port.pregrasp_targets) == 3
    for target, expected in zip(port.pregrasp_targets, [(.42, .2, .81), (.44, .2, .81), (.46, .2, .81)]):
        assert target.position == pytest.approx(expected)


def test_path_budget_preserves_deferred_rrt_for_already_counted_candidates():
    port = Port()
    values = [Candidate((.5+i*.01, .2, .8), (1., 0., 0.), 1.-i*.01, i) for i in range(6)]
    config = GraspSelectionConfig(grasp_ik_first=True, stop_after_first_feasible=True,
        fast_candidate_paths_first=True, maximum_candidates=6, maximum_path_candidates=4)
    selected = GraspSelector(port, config).select(values, required_arm='left')
    assert selected is not None and selected.candidate_index == 0
    fast = [p for p in port.plans if p[0]]
    assert len(fast) == 4
    assert len([p for p in port.plans if not p[0]]) == 2


def test_scene_collision_still_rejects_before_planning():
    port = Port()
    port.blocked = True
    config = GraspSelectionConfig(grasp_ik_first=True, pregrasp_z_offset_m=.010)
    assert GraspSelector(port, config).select(
        [Candidate((.5, .2, .8), (1., 0., 0.), 1.)], required_arm='left') is None
    assert not port.plans


@pytest.mark.parametrize('offset', [float('nan'), float('inf'), -float('inf')])
def test_pregrasp_z_rejects_nonfinite_values(offset):
    with pytest.raises(ValueError, match='pregrasp Z'):
        GraspSelectionConfig(pregrasp_z_offset_m=offset)
