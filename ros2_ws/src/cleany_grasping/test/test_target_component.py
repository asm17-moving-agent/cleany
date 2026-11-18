import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud
from cleany_grasping.core.target_component import dominant_body_mask
from cleany_grasping.geometric_predictor import GeometricGraspConfig, GeometricGraspPredictor


def cloud(points):
    return PointCloud(points, np.zeros_like(points))


def test_detached_return_stays_an_obstacle_when_absent_from_context():
    x, y = np.meshgrid(np.arange(-.03, .031, .005), np.arange(-.025, .026, .005))
    body = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .05)))
    stray = np.array([[.09, .04, .001], [.09, .045, .001]])
    target = cloud(np.vstack((stray, body)))
    table = body * [4., 4., 0.]
    predictor = GeometricGraspPredictor(GeometricGraspConfig(body_connectivity_radius_m=.009))
    scene = predictor._prepare_geometry(target, cloud(table))
    np.testing.assert_array_equal(scene.target, body)
    assert all(np.any(np.all(scene.obstacles == point, axis=1)) for point in stray)
    assert np.ptp(scene.target @ scene.minor) == pytest.approx(.05, abs=.001)


@pytest.mark.parametrize('radius,extra_x', [
    (0., [.20]),                    # Feature disabled.
    (.009, [.056]),                 # A single depth dropout at the edge.
    (.009, np.arange(15) * .005 + .20),  # Two substantial components.
])
def test_ambiguous_or_disabled_split_preserves_target(radius, extra_x):
    x = np.r_[np.arange(10) * .005, extra_x]
    points = np.column_stack((x, np.zeros((len(x), 2))))
    assert dominant_body_mask(points, radius, .90).all()


def test_body_is_independent_of_point_order_and_rigid_frame():
    x = np.r_[np.arange(21) * .005, .4, .404]
    points = np.column_stack((np.zeros(len(x)), -x, np.zeros(len(x)))) + [1., 2., .5]
    order = np.random.default_rng(2).permutation(len(x))
    np.testing.assert_array_equal(dominant_body_mask(points[order], .009, .90), x[order] < .2)


def test_curved_thin_target_keeps_its_ends_and_rejects_contact_on_detached_return():
    from cleany_grasping.core.models import RawGrasp
    from cleany_grasping.core.selector import GraspConfig, rank_grasps

    angles = np.linspace(0., np.pi, 41)
    cable = np.column_stack((.04 * np.cos(angles), .04 * np.sin(angles), np.full(41, .02)))
    # The stray lies inside the cable's bounding box, in the open part of the arc.
    stray = np.array([0., .004, .02])
    target = cloud(np.vstack((cable, stray)))
    config = GraspConfig()
    body, context, workspace = config.prediction_inputs(target, cloud(cable))
    np.testing.assert_array_equal(body.points, cable)
    assert np.count_nonzero(np.all(context.points == stray, axis=1)) == 1
    expected = np.column_stack((cable.min(axis=0) - config.workspace_margin_m,
                                cable.max(axis=0) + config.workspace_margin_m))
    np.testing.assert_allclose(workspace.reshape((3, 2)), expected)
    def candidate(p, score):
        return RawGrasp(np.eye(3), p, .035, 0., score)
    candidates = (candidate(stray, .99), candidate(cable[0], .9), candidate(cable[-1], .8))
    ranked = rank_grasps(candidates, body, config)
    np.testing.assert_allclose([g.translation for g in ranked], cable[[0, -1]])
    np.testing.assert_allclose([g.translation for g in rank_grasps(candidates, target, config)],
                               cable[[0, -1]])


def test_body_color_and_context_density_survive_outlier_separation():
    from cleany_grasping.core.target_component import BodyFilter

    points = np.zeros((31, 3))
    points[:30, 0] = np.arange(30) * .005
    points[-1, 1] = .1
    target = PointCloud(points, np.random.default_rng(3).random(points.shape))
    body = BodyFilter().target(target)
    np.testing.assert_array_equal(body.colors, target.colors[:-1])
    assert body.obstacle_context(target) is target
    augmented = body.obstacle_context(cloud(points[:2]))
    np.testing.assert_array_equal(augmented.colors[-1], target.colors[-1])


def test_two_visible_parts_separated_by_occlusion_keep_their_full_span():
    from cleany_grasping.core.selector import GraspConfig

    side = np.column_stack((np.arange(20) * .004, np.zeros((20, 2))))
    parts = cloud(np.vstack((side, side + [0., .055, 0.])))
    body, context, _ = GraspConfig().prediction_inputs(parts, parts)
    assert body.detached is None
    np.testing.assert_array_equal(body.points, parts.points)
    assert context is parts
