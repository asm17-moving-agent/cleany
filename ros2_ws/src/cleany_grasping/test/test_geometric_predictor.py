import math

import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud
from cleany_grasping.geometric_predictor import (
    GeometricGraspConfig,
    GeometricGraspPredictor,
    _longitudinal_contacts,
)


def cloud(points):
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    return PointCloud(values, np.zeros_like(values))


def test_longitudinal_search_stays_on_support_plane_and_keeps_finger_footprint():
    target = np.array([(x, y, z) for x in (-.05, .05)
                       for y in (-.025, .025) for z in (0., .01)])
    center = np.array([0., 0., .005])
    config = GeometricGraspConfig(longitudinal_offset_fractions=(0., -.25, .25))
    contacts = _longitudinal_contacts(center, np.array([1., 0., .2]),
                                      np.array([0., 0., 1.]), target, config)
    assert len(contacts) == 3
    assert [p[0] for p in contacts] == pytest.approx([0., -.025, .025])
    for point in contacts:
        assert point[2] == pytest.approx(center[2])
        assert abs(point[0]) + config.finger_length_m/2 <= .05


@pytest.mark.parametrize('span,height', [(.03, .01), (.10, .06)])
def test_longitudinal_search_does_not_shift_short_or_tall_objects(span, height):
    target = np.array([(x, y, z) for x in (-span/2, span/2)
                       for y in (-.01, .01) for z in (0., height)])
    center = np.array([0., 0., height/2])
    contacts = _longitudinal_contacts(center, np.array([1., 0., 0.]),
        np.array([0., 0., 1.]), target,
        GeometricGraspConfig(longitudinal_offset_fractions=(0., -.25, .25),
                             longitudinal_contact_height_offset_m=.003))
    assert len(contacts) == 1
    assert np.array_equal(contacts[0], center)


def test_longitudinal_height_correction_only_applies_to_eligible_thin_objects():
    target = np.array([(x, y, z) for x in (-.05, .05)
                       for y in (-.025, .025) for z in (0., .02)])
    config = GeometricGraspConfig(longitudinal_offset_fractions=(0., -.25, .25),
                                  longitudinal_contact_height_offset_m=.003)
    contacts = _longitudinal_contacts(np.array([0., 0., .01]), np.array([1., 0., 0.]),
                                      np.array([0., 0., 1.]), target, config)
    assert len(contacts) == 3
    assert [p[2] for p in contacts] == pytest.approx([.013]*3)


@pytest.mark.parametrize('fractions,ratio', [((), .4), ((.5,), .4), ((float('nan'),), .4),
                                           ((0.,), 0.), ((0.,), float('nan'))])
def test_invalid_longitudinal_search_rejected(fractions, ratio):
    with pytest.raises(ValueError, match='longitudinal'):
        GeometricGraspConfig(longitudinal_offset_fractions=fractions,
                             longitudinal_max_height_ratio=ratio)


def test_approaches_follow_both_configured_shoulder_origins():
    target, context = tabletop_scene()
    predictor = GeometricGraspPredictor(GeometricGraspConfig(
        approach_tilt_options=(16.,), include_reverse_closing_axis=True,
        approach_reference_positions=((-0.3, .15, .4), (-0.3, -.15, .4)),
        maximum_candidates=24))
    result = predictor.predict(target, context, np.zeros((3,2)))
    assert result
    horizontal_y = [grasp.rotation[1,0] for grasp in result]
    assert min(horizontal_y) < -.05 and max(horizontal_y) > .05


def tabletop_scene(obstacle=False):
    x, y = np.meshgrid(np.linspace(-0.12, 0.12, 31), np.linspace(-0.12, 0.12, 31))
    plane = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    tx, ty, tz = np.meshgrid(
        np.linspace(-0.035, 0.035, 9),
        np.linspace(-0.018, 0.018, 7),
        np.linspace(0.025, 0.065, 5),
    )
    target = np.column_stack((tx.ravel(), ty.ravel(), tz.ravel()))
    context = np.vstack((plane, target))
    if obstacle:
        ox, oy, oz = np.meshgrid(
            np.linspace(-0.06, 0.06, 15),
            np.linspace(-0.06, 0.06, 15),
            np.linspace(0.07, 0.11, 5),
        )
        context = np.vstack((context, np.column_stack((ox.ravel(), oy.ravel(), oz.ravel()))))
    return cloud(target), cloud(context)


def test_upward_closing_preference_only_reorders_equal_quality_candidates():
    target, context = tabletop_scene()
    options = dict(approach_tilt_degrees=16., approach_tilt_direction=(1., 1., 0.),
                   include_reverse_closing_axis=True, maximum_candidates=40)
    plain = GeometricGraspPredictor(GeometricGraspConfig(**options)).predict(
        target, context, np.array([[-1., 1.]]*3))
    preferred = GeometricGraspPredictor(GeometricGraspConfig(
        **options, prefer_upward_closing_axis=True)).predict(
        target, context, np.array([[-1., 1.]]*3))
    assert preferred
    assert sorted(x.score for x in plain) == pytest.approx(sorted(x.score for x in preferred))
    assert preferred[0].rotation[2, 1] > 0


def test_thin_object_predictor_generates_nearer_contacts_without_score_inflation():
    target, context = tabletop_scene()
    target = cloud(target.points * (1., 1., .25))
    context = cloud(context.points * (1., 1., .25))
    options = dict(yaw_offsets_degrees=(0.,), robot_reference_position=(-1., 0., 0.))
    plain = GeometricGraspPredictor(GeometricGraspConfig(**options)).predict(
        target, context, np.zeros(6))
    shifted = GeometricGraspPredictor(GeometricGraspConfig(**options,
        longitudinal_offset_fractions=(0., -.25, .25))).predict(target, context, np.zeros(6))
    assert len(shifted) > len(plain)
    assert shifted[0].translation[0] < plain[0].translation[0] - .005
    assert shifted[0].score == pytest.approx(plain[0].score)
    assert shifted[0].width_m == pytest.approx(plain[0].width_m)
    assert shifted[0].translation[2] == pytest.approx(plain[0].translation[2])


@pytest.mark.parametrize('deferred', [False, True])
def test_plane_deferral_preserves_above_plane_context_obstacles(monkeypatch, deferred):
    import cleany_grasping.geometric_predictor as geometry
    target, context = tabletop_scene(obstacle=True)
    observed = []
    original = geometry._collides
    def inspect(translation, rotation, width, target, context, config):
        observed.append(context.copy())
        return original(translation, rotation, width, target, context, config)
    monkeypatch.setattr(geometry, '_collides', inspect)
    GeometricGraspPredictor(GeometricGraspConfig(
        defer_support_plane_collision=deferred)).predict(target, context, np.zeros(6))
    assert observed
    for points in observed:
        assert np.any(points[:, 2] >= .10)
        assert bool(np.any(np.abs(points[:, 2]) < 1e-9)) is not deferred


def test_generates_ranked_top_down_candidates_for_tabletop_box() -> None:
    target, context = tabletop_scene()
    predictor = GeometricGraspPredictor()

    candidates = predictor.predict(target, context, np.zeros(6))

    assert 1 <= len(candidates) <= 12
    assert [item.score for item in candidates] == sorted(
        (item.score for item in candidates), reverse=True
    )
    assert candidates[0].width_m < 0.05
    assert np.allclose(candidates[0].rotation[:, 0], (0.0, 0.0, -1.0), atol=1e-3)
    assert np.allclose(
        candidates[0].rotation.T @ candidates[0].rotation,
        np.eye(3),
        atol=1e-6,
    )
    contact = (
        candidates[0].translation
        + candidates[0].depth_m * candidates[0].rotation[:, 0]
    )
    assert np.allclose(contact, np.median(target.points, axis=0))


@pytest.mark.parametrize('height_scale', [.25, 2.5])
@pytest.mark.parametrize('depth', [.025, .035])
def test_top_contact_depth_raises_tall_grasps_without_raising_thin_objects(height_scale, depth):
    target, context = tabletop_scene()
    target = cloud(target.points * (1., 1., height_scale))
    context = cloud(context.points * (1., 1., height_scale))
    candidates = GeometricGraspPredictor(GeometricGraspConfig(
        maximum_top_contact_depth_m=depth)).predict(target, context, np.zeros(6))
    assert candidates
    item = candidates[0]
    contact = item.translation + item.depth_m * item.rotation[:, 0]
    expected = max(float(np.median(target.points[:, 2])),
                   float(np.percentile(target.points[:, 2], 99.5)) - depth)
    assert contact[2] == pytest.approx(expected, abs=1e-4)


def test_configured_approach_tilt_preserves_a_proper_grasp_rotation() -> None:
    target, context = tabletop_scene()
    predictor = GeometricGraspPredictor(
        GeometricGraspConfig(
            approach_tilt_degrees=12.0,
            approach_tilt_direction=(1.0, 0.0, 0.0),
        )
    )

    candidates = predictor.predict(target, context, np.zeros(6))

    assert candidates
    expected_approach = (
        math.sin(math.radians(12.0)),
        0.0,
        -math.cos(math.radians(12.0)),
    )
    assert np.allclose(
        candidates[0].rotation[:, 0],
        expected_approach,
        atol=1e-3,
    )
    assert np.allclose(
        candidates[0].rotation.T @ candidates[0].rotation,
        np.eye(3),
        atol=1e-6,
    )


def test_rejects_approach_tilt_pointing_away_from_robot() -> None:
    target, context = tabletop_scene()
    predictor = GeometricGraspPredictor(
        GeometricGraspConfig(
            approach_tilt_degrees=12.0,
            approach_tilt_direction=(-1.0, 0.0, 0.0),
            reject_robot_opposite_approach=True,
            robot_reference_position=(-0.4, 0.0, 0.0),
        )
    )

    assert predictor.predict(target, context, np.zeros(6)) == ()


def test_keeps_approach_tilt_pointing_from_robot_to_target() -> None:
    target, context = tabletop_scene()
    predictor = GeometricGraspPredictor(
        GeometricGraspConfig(
            approach_tilt_degrees=12.0,
            approach_tilt_direction=(1.0, 0.0, 0.0),
            reject_robot_opposite_approach=True,
            robot_reference_position=(-0.4, 0.0, 0.0),
        )
    )

    assert predictor.predict(target, context, np.zeros(6))


def test_rejects_candidates_when_palm_is_blocked() -> None:
    target, context = tabletop_scene(obstacle=True)
    predictor = GeometricGraspPredictor(
        GeometricGraspConfig(palm_depth_m=0.05)
    )

    assert predictor.predict(target, context, np.zeros(6)) == ()


def test_multi_tilt_search_keeps_diverse_approaches_under_candidate_limit():
    target, context = tabletop_scene()
    candidates = GeometricGraspPredictor(GeometricGraspConfig(
        approach_tilt_options=(0.0, 8.0, 16.0), maximum_candidates=6,
    )).predict(target, context, np.zeros(6))
    angles = {round(math.degrees(math.acos(-g.rotation[2, 0])))
              for g in candidates}
    assert len(candidates) == 6
    assert angles == {0, 8, 16}
    assert all(g.width_m <= 0.10 for g in candidates)


def test_robust_extent_ignores_sparse_depth_boundary_outlier() -> None:
    target, context = tabletop_scene()
    outlier = np.array(((0.0, 0.25, 0.04),))
    expanded_target = cloud(np.vstack((target.points, outlier)))
    expanded_context = cloud(np.vstack((context.points, outlier)))

    candidates = GeometricGraspPredictor().predict(
        expanded_target,
        expanded_context,
        np.zeros(6),
    )

    assert candidates
    assert candidates[0].width_m <= 0.10


def test_contact_uses_volume_center_for_surface_biased_depth_cloud() -> None:
    top_x, top_y = np.meshgrid(
        np.linspace(-0.04, 0.04, 17),
        np.linspace(-0.025, 0.025, 13),
    )
    top = np.column_stack(
        (top_x.ravel(), top_y.ravel(), np.full(top_x.size, 0.08))
    )
    side_y, side_z = np.meshgrid(
        np.linspace(-0.025, 0.025, 9),
        np.linspace(0.0, 0.08, 9),
    )
    side = np.column_stack(
        (
            np.full(side_y.size, -0.04),
            side_y.ravel(),
            side_z.ravel(),
        )
    )
    target = cloud(np.vstack((top, side)))
    plane_x, plane_y = np.meshgrid(
        np.linspace(-0.12, 0.12, 31),
        np.linspace(-0.12, 0.12, 31),
    )
    plane = np.column_stack(
        (plane_x.ravel(), plane_y.ravel(), np.zeros(plane_x.size))
    )
    context = cloud(np.vstack((plane, target.points)))

    candidates = GeometricGraspPredictor().predict(
        target,
        context,
        np.zeros(6),
    )

    assert np.median(target.points[:, 2]) > 0.07
    contact = (
        candidates[0].translation
        + candidates[0].depth_m * candidates[0].rotation[:, 0]
    )
    assert np.allclose(contact, (0.0, 0.0, 0.04), atol=0.006)
