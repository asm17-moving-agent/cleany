import numpy as np
import pytest

from cleany_grasping.core.collision_geometry import observed_convex_prism


def test_observed_prism_preserves_circular_footprint_and_infers_support_bottom():
    angles = np.linspace(0, 2*np.pi, 16, endpoint=False)
    points = np.column_stack((.04*np.cos(angles), .04*np.sin(angles), np.full(16, .08)))
    mesh = observed_convex_prism(points, np.array([0., 0., .04]), np.eye(3), .08)
    assert len(mesh.vertices) == 32
    assert mesh.vertices[:, 2].min() == pytest.approx(-.04)
    assert mesh.vertices[:, 2].max() == pytest.approx(.04)
    assert np.max(np.linalg.norm(mesh.vertices[:, :2], axis=1)) == pytest.approx(.04)
    # 방향을 제외한 모든 모서리를 서로 반대 방향으로 두 번 사용하므로 닫힌 면의 방향이 일관됩니다.
    directed = [(int(a), int(b)) for face in mesh.triangles
                for a, b in zip(face, np.roll(face, -1))]
    for a, b in directed:
        assert directed.count((a, b)) == directed.count((b, a)) == 1
    for face in mesh.triangles:
        triangle = mesh.vertices[face]
        normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
        assert normal @ triangle.mean(0) > 0


def test_mesh_local_coordinates_preserve_transformed_observations():
    local = np.array([[-.04,-.02,-.03], [.04,-.02,.04], [.04,.02,.03], [-.04,.02,.02]])
    rotation = np.array([[0.,-1.,0.], [1.,0.,0.], [0.,0.,1.]])
    center = np.array([.5,.2,.4])
    mesh = observed_convex_prism(local @ rotation.T+center, center, rotation, .05)
    assert mesh.vertices.min(0) == pytest.approx([-.04,-.02,-.03])
    assert mesh.vertices.max(0) == pytest.approx([.04,.02,.04])


@pytest.mark.parametrize('points', [np.zeros((3,3)), np.array([[0,0,0],[1,1,0],[2,2,0]]),
                                   np.full((3,3), np.nan), np.zeros((2,3))])
def test_invalid_cloud_fails_without_inventing_geometry(points):
    with pytest.raises(ValueError):
        observed_convex_prism(points, np.zeros(3), np.eye(3), .02)


def test_vertex_limit_fails_without_simplifying_away_observations():
    angles = np.linspace(0, 2*np.pi, 10, endpoint=False)
    points = np.column_stack((np.cos(angles), np.sin(angles), np.ones(10)))
    with pytest.raises(ValueError, match='oversized'):
        observed_convex_prism(points, np.zeros(3), np.eye(3), .02, maximum_vertices=8)
