"""Check complete surfaces, narrow openings and fail-closed reduction limits."""

from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def reduction(monkeypatch):
    pytest.importorskip("vtk")
    monkeypatch.syspath_prepend(str(ROOT / "tools"))
    import freecad_mesh_reduction

    return freecad_mesh_reduction


def grid(reduction, *, hole=False):
    vertices = np.array([[x / 1000, y / 1000, 0] for y in range(9) for x in range(9)])
    faces = []
    for y in range(8):
        for x in range(8):
            if hole and 3 <= x < 5 and 3 <= y < 5:
                continue
            a = y * 9 + x
            faces.extend([[a, a + 1, a + 10], [a, a + 10, a + 9]])
    return reduction.Mesh(vertices.astype(np.float32).astype(float), np.array(faces))


def test_planar_reduction_keeps_hole_and_boundaries(reduction):
    source = grid(reduction, hole=True)
    reduced, report = reduction.reduce_mesh(source, 0.0001)
    assert len(reduced.faces) < len(source.faces)
    assert report["boundary_and_nonmanifold_edges_preserved"]
    assert report["source_to_reduced"]["upper_bound_m"] <= 0.0001
    assert report["reduced_to_source"]["upper_bound_m"] <= 0.0001
    with pytest.raises(ValueError, match="deviation"):
        reduction.surface_bound(grid(reduction), reduced, 0.0001)
    triangles = reduced.vertices[reduced.faces]
    assert (
        np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])[
            :, 2
        ]
        > 0
    ).all()


def test_shared_vertices_do_not_certify_a_missing_triangle_interior(reduction):
    source = reduction.Mesh(
        np.array([[0.0, 0, 0], [1.0, 0, 0], [0, 1.0, 0]]), np.array([[0, 1, 2]])
    )
    # All source vertices lie on target segments, but its interior is absent.
    edges = reduction.Mesh(source.vertices, np.array([[0, 1, 1], [1, 2, 2], [2, 0, 0]]))
    with pytest.raises(ValueError, match="deviation"):
        reduction.surface_bound(source, edges, 0.0001)


def test_different_diagonals_and_small_parallel_offset_are_bounded(reduction):
    source = reduction.Mesh(
        np.array([[0.0, 0, 0], [0.001, 0, 0], [0.001, 0.001, 0], [0, 0.001, 0]]),
        np.array([[0, 1, 2], [0, 2, 3]]),
    )
    target = reduction.Mesh(
        source.vertices + [0, 0, 0.00002], np.array([[0, 1, 3], [1, 2, 3]])
    )
    for a, b in ((source, target), (target, source)):
        assert reduction.surface_bound(a, b, 0.0001)["upper_bound_m"] <= 0.0001
    with pytest.raises(ValueError, match="work limit"):
        reduction.surface_bound(source, target, 0.0001, max_cells=1)


@pytest.mark.parametrize("error", [0.0, -1.0, float("nan"), float("inf"), 0.002])
def test_invalid_reduction_error_is_rejected(reduction, error):
    with pytest.raises(ValueError, match="surface error"):
        reduction.reduce_mesh(grid(reduction), error)
