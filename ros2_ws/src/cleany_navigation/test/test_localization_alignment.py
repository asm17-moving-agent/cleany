"""Stationary geometric matching, uncertainty and temporal evidence boundaries."""
from dataclasses import replace

import numpy as np
import pytest

from cleany_navigation.localization_alignment import (
    AlignmentHealth, AlignmentMap, AlignmentScore, AlignmentSettings, search_pose, transform)


def scene():
    cells = np.zeros((200, 200), dtype=int)
    cells[5, 5:195] = cells[194, 5:195] = 100
    cells[5:195, 5] = cells[5:195, 194] = 100
    cells[40:110, 60] = 100
    cells[130, 100:160] = 100
    model = AlignmentMap(cells, .05, (0., 0., 0.))
    y, x = np.where(cells == 100)
    world = np.column_stack(((x+.5)*.05, (y+.5)*.05))[::3]
    pose = np.array([4., 4., .2])
    points = transform(world-pose[:2], np.array([0., 0., -pose[2]]))
    return model, points, pose


def test_alignment_detects_wrong_pose_despite_identical_points():
    model, points, pose = scene()
    settings = AlignmentSettings()
    assert model.score(points, pose, settings).acceptable(settings)
    assert not model.score(points, pose+[.5, -.4, .1], settings).acceptable(settings)


def test_unknown_and_outside_map_cannot_look_like_aligned_walls():
    model, points, pose = scene()
    model.cells[:] = -1
    score = model.score(points, pose, AlignmentSettings())
    assert score.known_ratio == 0 and score.inlier_ratio == 0
    assert not score.acceptable(AlignmentSettings())


def test_rotated_map_origin_is_respected():
    model, points, pose = scene()
    rotated = AlignmentMap(model.cells, model.resolution, (2., -1., .7))
    world_pose = transform(pose[None, :2], np.array([2., -1., .7]))[0]
    transformed_pose = np.array([*world_pose, pose[2]+.7])
    assert rotated.score(points, transformed_pose, AlignmentSettings()).inlier_ratio > .95


def test_bad_and_good_evidence_require_time_and_duplicates_do_not_advance():
    settings = AlignmentSettings()
    health = AlignmentHealth(settings)
    good = AlignmentScore(100, .9, .02, .01, 1.)
    bad = AlignmentScore(100, .05, .4, .5, 1.)
    for t in np.arange(10., 11.3, .2): health.observe(t, good)
    assert health.state == 'available'
    health.observe(11.4, bad)
    for _ in range(20):health.observe(11.4, bad)
    assert health.state == 'degraded'
    for t in np.arange(11.6, 12.5, .2):health.observe(t, bad)
    assert health.state == 'unavailable'
    health.observe(12.6, good)
    assert health.state == 'unavailable'
    health.observe(13.4, good)  # A data gap must restart stability.
    assert health.state == 'unknown'
    for t in np.arange(13.6, 14.8, .2):health.observe(t, good)
    assert health.state == 'available'


def test_mild_mismatch_keeps_verified_pose_then_stops_if_progress_stalls():
    health=AlignmentHealth(AlignmentSettings())
    good=AlignmentScore(100,.9,.02,.01,1.)
    mild=AlignmentScore(100,.4,.22,.2,1.)
    for t in np.arange(10.,11.3,.2):health.observe(t,good)
    for t in np.arange(11.4,15.3,.2):
        assert health.observe(t,mild)=='degraded'
    assert health.observe(15.6,mild)=='unavailable'
    assert health.reason=='reconvergence_stalled'


def test_reconvergence_retains_hysteresis_and_cannot_extend_forever():
    settings=replace(AlignmentSettings(),progress_distance_m=.004)
    health=AlignmentHealth(settings);good=AlignmentScore(100,.9,.02,.01,1.)
    for t in np.arange(10.,11.3,.2):health.observe(t,good)
    for i in range(42):
        health.observe(11.4+.2*i,AlignmentScore(100,.4,.32-.001*i,.3,1.))
    assert health.state=='unavailable'
    assert health.reason=='reconvergence_budget_exhausted'
    health.observe(19.8,good);assert health.state=='unavailable'
    for t in np.arange(20.,21.2,.2):health.observe(t,good)
    assert health.state=='available'


def test_startup_and_input_loss_never_gain_degraded_motion_permission():
    health=AlignmentHealth(AlignmentSettings())
    mild=AlignmentScore(100,.4,.22,.2,1.)
    for t in np.arange(10.,13.,.2):assert health.observe(t,mild)=='unknown'
    good=AlignmentScore(100,.9,.02,.01,1.)
    for t in np.arange(13.,14.3,.2):health.observe(t,good)
    assert health.state=='available'
    health.invalidate();assert health.observe(14.4,mild)=='unknown'


def test_brief_valid_matches_refresh_progress_without_resetting_total_budget():
    health = AlignmentHealth(AlignmentSettings())
    good = AlignmentScore(100, .9, .02, .01, 1.)
    mild = AlignmentScore(100, .4, .22, .2, 1.)
    for i in range(7):
        health.observe(10.+.25*i, good)
    for i in range(33):
        score = good if i % 2 else mild
        state = health.observe(12.+.25*i, score)
        if i < 32:
            assert state == 'degraded'
        else:
            assert state == 'unavailable'
    assert health.reason == 'reconvergence_budget_exhausted'


def test_total_budget_applies_while_a_brief_valid_match_is_being_verified():
    health = AlignmentHealth(AlignmentSettings())
    good = AlignmentScore(100, .9, .02, .01, 1.)
    mild = AlignmentScore(100, .4, .22, .2, 1.)
    for i in range(7):
        health.observe(10.+.25*i, good)
    for i in range(32):
        health.observe(12.+.25*i, good if i % 2 else mild)
    assert health.observe(20., good) == 'unavailable'
    assert health.reason == 'reconvergence_budget_exhausted'


def test_bounded_search_restores_asymmetric_pose_with_holdout_validation():
    model, points, pose = scene()
    guess = pose+[.4, -.3, .12]
    result = search_pose(model, points, guess, AlignmentSettings())
    assert result['accepted'], result
    assert np.linalg.norm(np.array(result['pose'])[:2]-pose[:2]) < .04
    assert abs(result['pose'][2]-pose[2]) < .02
    assert result['validation']['inlier_ratio'] > .9


def test_search_cannot_apply_an_unobservable_corridor_displacement():
    cells = np.zeros((240, 240), dtype=int)
    cells[:, 80] = cells[:, 160] = 100
    model = AlignmentMap(cells, .05, (0., 0., 0.))
    y = np.linspace(3., 9., 100)
    points = np.vstack((np.column_stack((np.full(100, 4.025), y)),
                        np.column_stack((np.full(100, 8.025), y)))) - [6., 6.]
    result = search_pose(model, points, np.array([6.4, 6., 0.]), AlignmentSettings())
    assert not result['accepted'] and result['reason'] == 'ambiguous_candidate'


def test_search_cancellation_and_invalid_settings():
    model, points, pose = scene()
    assert search_pose(model, points, pose, AlignmentSettings(), lambda: True)['reason'] == 'search_canceled'
    for key, value in [('search_radius', -1), ('min_inlier_ratio', 1.1), ('coarse_xy', 3.)]:
        with pytest.raises(ValueError):replace(AlignmentSettings(), **{key:value})
