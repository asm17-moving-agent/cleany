"""Synthetic floor/obstacle checks independent of physical sensor validation."""

import numpy as np
import pytest

from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.depth_obstacles import ObstacleFilter, fit_floor, filter_obstacles


def test_floor_fit_ignores_a_small_low_obstacle():
    rng = np.random.default_rng(4)
    x, y = rng.uniform(.5, 2.8, 2000), rng.uniform(-1, 1, 2000)
    z = -.0955+.015*x+rng.normal(0, .002, 2000)
    z[:150] += .08
    result = fit_floor(np.column_stack((x, y, z)))
    assert result['z_at_origin'] == pytest.approx(-.0955, abs=.002)
    assert result['normal'][0] == pytest.approx(-.015, abs=.002)


def test_tabletop_cannot_be_used_as_floor():
    rng = np.random.default_rng(4)
    p = np.column_stack((rng.uniform(.5, 2.8, 1000), rng.uniform(-1, 1, 1000), np.full(1000, .7)))
    with pytest.raises(ValueError):
        fit_floor(p)


def test_floor_removed_but_supported_eight_cm_obstacle_retained():
    yy, zz = np.meshgrid(np.linspace(-.05, .05, 11), np.linspace(.07, .09, 5))
    object_points = np.column_stack((np.ones(yy.size), yy.ravel(), zz.ravel()))
    floor = np.column_stack((np.linspace(.5, 2, 50), np.zeros(50), np.zeros(50)))
    p = np.vstack((object_points, floor, [[2, 1, .8]], [[np.nan, 0, 1]]))
    out, counts = filter_obstacles(p, np.array([0, 0, 1]), [], ObstacleFilter((0, 0, 1), 0))
    assert len(out) >= 5
    assert np.all(out[:, 2] >= .07)
    assert np.all(out[:, 2] <= .09)
    assert counts['height_removed'] == 50
    assert counts['isolated_removed'] == 1


def test_self_occluded_hits_do_not_become_free_rays():
    box = BodyBox('fixed_rail', np.array([.5, 0, .5]), np.eye(3), np.array([.05, .1, .05]))
    p = np.array([[1, -.01, 0], [1, 0, 0], [1, .01, 0]])
    out, counts = filter_obstacles(p, np.array([0, 0, 1]), [box], ObstacleFilter((0, 0, 1), -.1))
    assert len(out) == 0
    assert counts['self_removed'] == 3


def test_invalid_floor_and_empty_input():
    with pytest.raises(ValueError):
        ObstacleFilter((0, 0, 2), 0)
    out, counts = filter_obstacles(np.empty((0, 3)), np.zeros(3), [], ObstacleFilter((0, 0, 1), 0))
    assert out.shape == (0, 3)
    assert counts['output'] == 0


def test_lidar_cutoff_is_fixed_in_base_frame_not_fitted_floor():
    points = np.array([[x, 0, z] for x in [.5, 2.5]
                       for z in [.08, .191, .192, .3]])
    config = ObstacleFilter((0.1, 0, np.sqrt(.99)), -.1,
                            min_base_z=.192, min_neighbors=1, voxel_size=.0001)
    out, counts = filter_obstacles(points, np.array([0, 0, 1]), [], config)
    assert len(out) == 4
    assert np.all(out[:, 2] >= np.float32(.192))
    assert counts['height_removed'] == 4
    assert set(out[:, 0]) == {.5, 2.5}


def test_lidar_cutoff_rejects_nonfinite_value():
    with pytest.raises(ValueError, match='cutoff'):
        ObstacleFilter((0, 0, 1), 0, min_base_z=np.nan)
