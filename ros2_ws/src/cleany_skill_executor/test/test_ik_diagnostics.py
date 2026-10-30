"""Diagnostic evidence keeps pose errors separate from solver and scene failures."""
import json
import math

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    JointSolution, PoseAlignment, PoseValue, PregraspRequest,
)
from cleany_skill_executor.core.ik_diagnostics import (
    AlignmentLimits, alignment_errors, summarize_alignments,
)


LIMITS = AlignmentLimits(.010, 30., 30.)


def alignment(position=.001, approach=0., closing=0.):
    return PoseAlignment(JointSolution(('joint',), (0.,)),
                         PoseValue((0., 0., 0.), (0., 0., 0., 1.)),
                         position, approach, closing)


def test_separate_error_counts_do_not_claim_independent_minima_are_a_valid_pose():
    summary = summarize_alignments([
        alignment(.001, 42., 0.), alignment(.020, 2., 40.),
    ], LIMITS)
    assert summary['pose_feasible'] == 0
    assert summary['pose_rejection_counts'] == {
        'position_error_mm': 1, 'approach_error_degrees': 1, 'closing_error_degrees': 1}
    assert summary['minimum_errors'] == {
        'position_error_mm': 1., 'approach_error_degrees': 2., 'closing_error_degrees': 0.}
    closest = summary['closest_alignment']
    assert closest['approach_error_degrees'] == 42.
    assert closest['violations'] == ['approach_error_degrees']
    assert closest['joint_positions_rad'] == [0.]


def test_no_solution_has_missing_errors_instead_of_zero_error():
    summary = summarize_alignments([], LIMITS)
    assert summary['evaluated_variants'] == 0
    assert summary['closest_alignment'] is None
    assert all(value is None for value in summary['minimum_errors'].values())


def test_tolerance_boundary_matches_inclusive_acceptance():
    summary = summarize_alignments([alignment(.010, 30., 30.)], LIMITS)
    assert summary['pose_feasible'] == 1
    assert summary['closest_alignment']['violations'] == []


@pytest.mark.parametrize('error', [math.inf, math.nan])
def test_nonfinite_errors_are_json_null_and_rejected(error):
    data = alignment_errors(alignment(approach=error), LIMITS)
    assert data['approach_error_degrees'] is None
    assert data['violations'] == ['approach_error_degrees']
    json.dumps(data, allow_nan=False)


def test_nonfinite_fk_pose_can_be_logged_as_failure_evidence():
    item = alignment(approach=math.nan)
    item = PoseAlignment(item.solution, PoseValue((math.nan, 0., 0.),
                         (math.nan, 0., 0., 1.)), math.nan, math.nan, math.nan)
    json.dumps(alignment_errors(item, LIMITS), allow_nan=False)


def test_diagnostic_index_does_not_change_pregrasp_cache_identity():
    target = PoseValue((0., 0., 0.), (0., 0., 0., 1.))
    first = PregraspRequest('left', target, .4, 17)
    second = PregraspRequest('left', target, .4, 18)
    assert first == second
    assert {first: 'same pose'}[second] == 'same pose'
