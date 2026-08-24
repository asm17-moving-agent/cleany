import numpy as np

from cleany_grasping.core.models import PointCloud
from cleany_grasping.geometric_predictor import (
    GeometricGraspConfig,
    GeometricGraspPredictor,
)


def cloud(points):
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    return PointCloud(values, np.zeros_like(values))


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


def test_rejects_candidates_when_palm_is_blocked() -> None:
    target, context = tabletop_scene(obstacle=True)
    predictor = GeometricGraspPredictor(
        GeometricGraspConfig(palm_depth_m=0.05)
    )

    assert predictor.predict(target, context, np.zeros(6)) == ()


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
