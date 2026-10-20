import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.core.selector import GraspConfig, rank_grasps


def cloud(points):
    points = np.asarray(points, dtype=float)
    return PointCloud(points, np.zeros_like(points))


def grasp(x, score, width=0.04, rotation=None, depth=0.0):
    return RawGrasp(
        np.eye(3) if rotation is None else rotation,
        np.asarray(x, dtype=float),
        width,
        depth,
        score,
    )


def test_selects_highest_scoring_target_contact_after_width_filter() -> None:
    target = cloud(((-0.02, -0.02, 0.5), (0.02, 0.02, 0.55)))
    candidates = (
        grasp((0.0, 0.0, 0.52), 0.8),
        grasp((0.0, 0.0, 0.52), 0.99, width=0.2),
        grasp((0.3, 0.0, 0.52), 0.9),
    )

    ranked = rank_grasps(candidates, target)

    assert len(ranked) == 1
    result = ranked[0]
    assert result.score == 0.8
    assert np.allclose(result.approach_direction, (1.0, 0.0, 0.0))


def test_canonical_rotation_is_converted_to_tcp_axes() -> None:
    target = cloud(((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)))
    conversion = np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))

    ranked = rank_grasps(
        (grasp((0.0, 0.0, 0.0), 0.7),),
        target,
        GraspConfig(canonical_to_tcp_rotation=conversion),
    )

    assert len(ranked) == 1
    result = ranked[0]
    assert np.allclose(result.rotation, conversion)
    assert np.allclose(result.approach_direction, (1.0, 0.0, 0.0))
    assert result.required_opening_m == 0.04


def test_cleany_negative_y_tcp_axis_preserves_canonical_approach() -> None:
    target = cloud(((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)))
    conversion = np.array(
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )

    ranked = rank_grasps(
        (grasp((0.0, 0.0, 0.0), 0.7),),
        target,
        GraspConfig(
            canonical_to_tcp_rotation=conversion,
            tcp_approach_axis=np.array((0.0, -1.0, 0.0)),
        ),
    )

    assert len(ranked) == 1
    result = ranked[0]
    assert np.allclose(
        result.rotation @ np.array((0.0, -1.0, 0.0)),
        result.approach_direction,
    )
    assert np.allclose(result.approach_direction, (1.0, 0.0, 0.0))


def test_returns_empty_ranking_instead_of_inventing_candidate() -> None:
    target = cloud(((-0.01, -0.01, 0.5), (0.01, 0.01, 0.52)))
    assert rank_grasps((), target) == ()


def test_nms_keeps_best_of_near_duplicate_candidates() -> None:
    target = cloud(((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)))
    ranked = rank_grasps(
        (grasp((0.0, 0.0, 0.0), 0.9), grasp((0.001, 0.0, 0.0), 0.8)),
        target,
    )
    assert len(ranked) == 1 and ranked[0].score == 0.9


def test_reflection_cannot_be_used_as_a_tcp_rotation():
    with pytest.raises(ValueError, match='right-handed'):
        GraspConfig(canonical_to_tcp_rotation=np.diag([-1., 1., 1.]))


def test_tcp_configuration_is_isolated_from_mutating_input_arrays():
    conversion = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
    axis = np.array([0., -1., 0.])
    config = GraspConfig(canonical_to_tcp_rotation=conversion, tcp_approach_axis=axis)
    conversion[:] = 0.
    axis[:] = [0., 0., 1.]
    target = cloud(((-.02, -.02, .5), (.02, .02, .55)))
    result = rank_grasps([grasp((0., 0., .52), .8)], target, config)[0]
    np.testing.assert_allclose(result.rotation, ((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
    np.testing.assert_allclose(result.approach_direction, [1., 0., 0.])
    with pytest.raises(ValueError):
        config.canonical_to_tcp_rotation[0, 0] = -1.
    with pytest.raises(ValueError):
        config.tcp_approach_axis[0] = 0.
