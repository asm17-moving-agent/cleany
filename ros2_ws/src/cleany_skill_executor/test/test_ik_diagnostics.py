"""Diagnostic evidence keeps pose errors separate from solver and scene failures."""
import json
import math

import numpy as np

import pytest

from cleany_skill_executor.core.physical_grasp_selection import (
    JointSolution, PoseAlignment, PoseValue, PregraspRequest,
)
from cleany_skill_executor.core.ik_diagnostics import (
    AlignmentLimits, ScoredAlignments, alignment_errors, alignment_rank, merge_alignments,
    summarize_alignments,
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


@pytest.mark.parametrize('items', [
    [], [alignment(.010, 30., 30.)],
    [alignment(.001, 42., 0.), alignment(.020, 2., 40.)],
    [alignment(approach=math.nan), alignment(closing=math.inf)],
    [alignment(approach=1.), alignment(closing=1.), alignment(approach=1.)],
])
def test_compact_summary_matches_complete_witness_summary(items):
    errors = [alignment_errors(item, LIMITS) for item in items]
    minima = {}
    for name in LIMITS.as_dict():
        values = [value[name] for value in errors if value[name] is not None]
        minima[name] = min(values) if values else None
    expected = {
        'evaluated_variants': len(items),
        'pose_feasible': sum(not value['violations'] for value in errors),
        'pose_rejection_counts': {
            name: sum(name in value['violations'] for value in errors)
            for name in LIMITS.as_dict()},
        'minimum_errors': minima,
        'closest_alignment': min(errors, key=lambda value: alignment_rank(value, LIMITS))
                             if errors else None,
    }
    assert summarize_alignments(items, LIMITS) == expected


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


@pytest.mark.parametrize('preferred', [None, -.5, .5])
def test_compact_gpu_witnesses_preserve_selection_statistics_and_mixed_refinement(preferred):
    from types import SimpleNamespace
    from cleany_skill_executor.curobo_alignment import _alignment_from_row, _compact_group
    from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter

    names = tuple(f'joint_{index}' for index in range(5))
    generator = np.random.default_rng(20261108)
    rows = generator.uniform(-1., 1., (120, 15))
    rows[:, 8:12] = (0., 0., 0., 1.)
    rows[:, 12] = generator.choice([.001, .010, .011, math.nan, math.inf], len(rows))
    rows[:, 13:15] = generator.choice([0., 30., 31., math.nan, math.inf], (len(rows), 2))
    def feasible(item):
        return (item.position_error_m <= LIMITS.position_m
                and item.approach_error_degrees <= LIMITS.approach_degrees
                and item.closing_error_degrees <= LIMITS.closing_degrees)
    for subset in (rows[:0], rows[:1], rows, rows[np.isfinite(rows[:, 12:15]).all(axis=1)]):
        full = [_alignment_from_row(names, row) for row in subset]
        rank = MoveItGraspAdapter._alignment_sort_key(SimpleNamespace(_current_state=None), full, preferred)
        compact = _compact_group(names, subset, LIMITS, preferred)
        assert summarize_alignments(compact, LIMITS) == summarize_alignments(full, LIMITS)
        assert [item.solution for item in compact.candidates if feasible(item)] == [
            item.solution for item in full if feasible(item)]
        rejected = [item for item in full if not feasible(item)]
        retained = [item for item in compact.candidates if not feasible(item)]
        assert len(retained) == int(bool(rejected))
        if rejected:
            assert retained[0].solution == min(rejected, key=rank).solution
        middle = len(subset)//2
        first = _compact_group(names, subset[:middle], LIMITS, preferred)
        second = _compact_group(names, subset[middle:], LIMITS, preferred)
        for left, right in ((first, second), (full[:middle], second), (first, full[middle:])):
            combined = merge_alignments(left, right)
            assert isinstance(combined, ScoredAlignments)
            assert summarize_alignments(combined, LIMITS) == summarize_alignments(full, LIMITS)
            candidates = [item.solution for item in combined.candidates if feasible(item)]
            assert candidates == [item.solution for item in full if feasible(item)]
    with pytest.raises(ValueError, match='limits changed'):
        summarize_alignments(compact, AlignmentLimits(.005, 30., 30.))
