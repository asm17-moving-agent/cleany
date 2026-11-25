"""Collision approximation keeps fitted geometry and preserves material gaps."""
from pathlib import Path
import sys

import numpy as np
import pytest

trimesh = pytest.importorskip('trimesh')
pytest.importorskip('fast_simplification')

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation'))
from optimize_cad_collision import (Mesh, MeshInput, certificate, clean, close_bores,
                                    maximum_plane_violation, meshopt_surface, optimize_one, read_mesh, sphere_gap)
from scipy.spatial import ConvexHull


@pytest.mark.parametrize(('radius', 'length', 'closed'), [
    (.001, .003, True), (.002, .003, False), (.001, .012, False),
])
def test_only_short_narrow_bores_are_capped(radius, length, closed):
    source = trimesh.creation.annulus(r_min=radius, r_max=.004, height=length, sections=24)
    original = clean(Mesh(source.vertices, source.faces))
    result, holes = close_bores(original, .00165, .006)
    assert bool(holes) is closed
    if not closed:
        assert result is original
        return
    fitted = trimesh.Trimesh(vertices=result.vertices, faces=result.faces, process=False)
    assert fitted.is_volume and fitted.euler_number == 2
    assert fitted.volume > source.volume
    assert len(result.faces) < len(original.faces)
    assert len(holes) == 1
    assert holes[0]['radius_m'] == pytest.approx(radius)
    assert holes[0]['length_m'] == pytest.approx(length)
    assert np.array_equal(fitted.bounds, source.bounds)


def test_closed_enclosure_is_smaller_and_contains_serialized_source(tmp_path):
    sphere = trimesh.creation.icosphere(subdivisions=2, radius=.01)
    path = tmp_path/'source.glb'
    sphere.export(path)
    result = optimize_one(MeshInput(path, tmp_path, True, .00035, .001, 0.))
    assert result['after']['vertices'] < result['before']['vertices']
    candidate = read_mesh(tmp_path/result['result'])
    assert maximum_plane_violation(read_mesh(path).vertices, ConvexHull(candidate.vertices).equations) <= 0


def test_surface_certificate_rejects_removed_external_face():
    box = trimesh.creation.box(extents=(.02, .01, .004))
    mesh = Mesh(box.vertices, box.faces)
    with pytest.raises(ValueError, match='surface distance limit'):
        certificate(mesh, Mesh(box.vertices, box.faces[2:]), .00035)


def test_sphere_coverage_checks_unsupported_face_interior():
    mesh = Mesh(np.eye(3)*.001, np.array([[0, 1, 2]]))
    spheres = ((.001, 0., 0., .0001), (0., .001, 0., .0001), (0., 0., .001, .0001))
    assert sphere_gap(mesh, spheres) > .0007


def test_alternate_simplifier_must_keep_closed_surface(monkeypatch):
    cube = trimesh.creation.box(extents=(.03, .01, .006))
    vertices, faces = trimesh.remesh.subdivide(cube.vertices, cube.faces)
    dense = clean(Mesh(vertices, faces))
    monkeypatch.setattr('optimize_cad_collision.meshopt_simplify',
                        lambda *_: Mesh(cube.vertices, cube.faces[2:]))
    assert meshopt_surface(dense, dense, .00035, (), Path('unused')) is None
    monkeypatch.setattr('optimize_cad_collision.meshopt_simplify',
                        lambda *_: Mesh(cube.vertices, cube.faces))
    candidate, report = meshopt_surface(dense, dense, .00035, (), Path('unused'))
    assert len(candidate.faces) == 12
    assert report['method'] == 'bounded_meshoptimizer'


def test_refit_verification_preserves_both_sides_of_the_enclosure_budget():
    from verify_cad_collision_refit import verify_enclosure

    box = trimesh.creation.box(extents=(.02, .01, .008))
    source = Mesh(box.vertices, box.faces)
    offset = np.sign(box.vertices)*.0002
    outer = Mesh(box.vertices + offset, box.faces)
    proof = verify_enclosure(source, outer, .0004)
    assert proof['maximum_expansion_m'] == pytest.approx(np.sqrt(3)*.0002, abs=2e-9)
    with pytest.raises(ValueError, match='expands'):
        verify_enclosure(source, outer, .0003)
    with pytest.raises(ValueError, match='cuts into'):
        verify_enclosure(source, Mesh(box.vertices - offset, box.faces), .0004)
