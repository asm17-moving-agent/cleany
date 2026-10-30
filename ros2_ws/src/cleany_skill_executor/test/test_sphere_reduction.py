import numpy as np
import pytest

from cleany_skill_executor.core.sphere_reduction import reduce_spheres, sphere_cover_error


def test_contained_and_coincident_spheres_keep_the_outer_volume():
    centers = np.array([[0., 0., 0.], [0., 0., 0.], [.2, 0., 0.]])
    radii = np.array([1., .3, .2])
    c, r = reduce_spheres(centers, radii, 1)
    np.testing.assert_array_equal(c, [[0., 0., 0.]])
    np.testing.assert_array_equal(r, [1.])
    assert sphere_cover_error(centers, radii, c, r) <= 1e-12


def test_separated_unequal_balls_are_fully_enclosed_not_only_their_centers():
    centers = np.array([[0., 0., 0.], [4., 0., 0.]])
    radii = np.array([1., 2.])
    c, r = reduce_spheres(centers, radii, 1)
    np.testing.assert_allclose(c, [[2.5, 0., 0.]])
    np.testing.assert_allclose(r, [3.5])
    assert sphere_cover_error(centers, radii, c, r) <= 1e-12


@pytest.mark.parametrize('maximum', [1, 4, 8, 40])
def test_repeated_merges_cover_every_original_ball_and_do_not_mutate_inputs(maximum):
    rng = np.random.default_rng(74)
    centers, radii = rng.uniform(-.1, .1, (40, 3)), rng.uniform(.002, .02, 40)
    old_centers, old_radii = centers.copy(), radii.copy()
    c, r = reduce_spheres(centers, radii, maximum)
    assert len(r) <= maximum
    assert sphere_cover_error(centers, radii, c, r) < 1e-12
    np.testing.assert_array_equal(centers, old_centers)
    np.testing.assert_array_equal(radii, old_radii)


@pytest.mark.parametrize('centers,radii,maximum', [
    ([], [], 2), ([[0., 0., float('nan')]], [1.], 1),
    ([[0., 0., 0.]], [0.], 1), ([[0., 0., 0.]], [1.], 0),
    ([[0., 0., 0.]], [1.], 1.5), ([[0., 0., 0.]], [1.], True)])
def test_invalid_geometry_and_budgets_are_rejected(centers, radii, maximum):
    with pytest.raises(ValueError):
        reduce_spheres(centers, radii, maximum)
