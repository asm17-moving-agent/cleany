"""Enclosure regressions include faces and volume, not only mesh vertices."""
import numpy as np
import pytest
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from cleany_skill_executor.core.collision_geometry import covering_spheres, enclosing_polytope
from cleany_skill_executor.core.sphere_reduction import reduce_spheres


def clearance(points, centers, radii):
    return (np.linalg.norm(points[:, None, :] - centers, axis=2) - radii).min(axis=1)


def test_long_rotated_arm_has_no_uncovered_surface_or_interior_after_merging():
    points = np.array([[x, y, z] for x in (-.16, .16) for y in (-.015, .015) for z in (-.025, .025)])
    rotation = Rotation.from_euler('xyz', [.3, -.7, .5]).as_matrix()
    points = points @ rotation.T + [.2, -.1, .3]
    centers, radii = covering_spheres(points, 16, .002)
    rng = np.random.default_rng(981)
    samples = rng.uniform(-1, 1, (10000, 3)) * [.16, .015, .025]
    # Include the complete box edges and face interiors, which a vertex-only
    # sphere fit can miss even while covering every vertex.
    for axis in range(3):
        samples[axis * 1000:(axis + 1) * 1000, axis] = [.16, .015, .025][axis]
    samples = np.vstack((samples @ rotation.T + [.2, -.1, .3], points))
    assert clearance(samples, centers, radii).max() <= -.00199
    reduced = reduce_spheres(centers, radii, 8)
    assert clearance(samples, *reduced).max() <= -.00199
    assert radii.max() < .04


def test_tapered_hull_cell_cuts_cover_face_interiors():
    points = np.array([[0., -.03, -.02], [0., .03, -.02], [0., 0., .04], [.3, 0., 0.]])
    centers, radii = covering_spheres(points, 8, 0.)
    rng = np.random.default_rng(129)
    samples = rng.dirichlet(np.ones(4), 10000) @ points
    hull = ConvexHull(points)
    for face in hull.simplices:
        samples = np.vstack((samples, rng.dirichlet(np.ones(3), 1000) @ points[face]))
    assert clearance(samples, centers, radii).max() <= 1e-10


def test_separate_components_do_not_fill_the_space_between_them():
    cube = np.array([[x, y, z] for x in (-.005, .005) for y in (-.005, .005) for z in (-.005, .005)])
    points = np.vstack((cube, cube + [.2, 0, 0]))
    faces = ConvexHull(cube).simplices
    centers, radii = covering_spheres(points, 8, .0005, np.vstack((faces, faces + 8)))
    assert clearance(points, centers, radii).max() < 0
    assert clearance(np.array([[.1, 0., 0.]]), centers, radii)[0] > .08


def test_gpu_reduction_encloses_rounded_box_with_bounded_expansion():
    rng = np.random.default_rng(261)
    points = rng.normal(size=(300, 3))
    points /= np.linalg.norm(points, axis=1, keepdims=True)
    points *= [.025, .02, .05]
    reduced = enclosing_polytope(points, .001)
    hull = ConvexHull(reduced)
    assert np.max(points @ hull.equations[:, :3].T + hull.equations[:, 3]) <= 1e-12
    assert len(reduced) < len(points)
    # Independent radial certificate against the input hull.
    source = ConvexHull(points)
    center = points.mean(axis=0)
    delta = reduced - center
    projection = delta @ source.equations[:, :3].T
    slack = -(source.equations[:, :3] @ center + source.equations[:, 3])
    fractions = np.full(projection.shape, np.inf)
    np.divide(slack, projection, out=fractions, where=projection > 0)
    bounds = np.linalg.norm(delta, axis=1) * (1 - np.minimum(1, fractions.min(axis=1)))
    assert bounds.max() <= .001
    np.testing.assert_array_equal(enclosing_polytope(points, 0), points[source.vertices])


@pytest.mark.parametrize('count,padding', [(0, .002), (1.5, .002), (4, -.1), (4, float('nan'))])
def test_bad_sphere_parameters_fail(count, padding):
    with pytest.raises(ValueError):
        covering_spheres(np.eye(3), count, padding)


@pytest.mark.parametrize('expansion', [-.1, float('nan')])
def test_bad_hull_parameters_fail(expansion):
    with pytest.raises(ValueError):
        enclosing_polytope(np.eye(3), expansion)
