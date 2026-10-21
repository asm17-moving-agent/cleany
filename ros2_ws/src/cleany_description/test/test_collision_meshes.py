"""Geometry containment and model-scope checks for the optional arm meshes."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import ConvexHull


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / 'ros2_ws/src/cleany_description'
sys.path.insert(0, str(ROOT / 'tools'))
spec = importlib.util.spec_from_file_location('cleany_mesh_optimizer', ROOT / 'tools/optimize_arm_collision_meshes.py')
optimizer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = optimizer
spec.loader.exec_module(optimizer)
SETTINGS = json.loads((ROOT / 'configs/robot/arm_collision_meshes.json').read_text())


def test_enclosure_preserves_extremes_that_inscribed_decimation_would_lose():
    rng = np.random.default_rng(341)
    vertices = rng.normal(size=(200, 3))
    vertices /= np.linalg.norm(vertices, axis=1, keepdims=True)
    vertices *= [.03, .02, .07]
    hull = ConvexHull(vertices)
    source = optimizer.Mesh(vertices, hull.simplices)
    output, report = optimizer.enclosing_hull(source,
        max_expansion=.00025, margin=.000001, max_iterations=64, planes_per_iteration=16)
    equations = ConvexHull(output.vertices).equations
    assert optimizer.maximum_plane_violation(vertices, equations) <= 0
    assert report['max_expansion_from_component_hull_m'] <= .00025
    assert report['output_volume_m3'] >= report['source_convex_hull_volume_m3']


def test_disconnected_solids_do_not_bridge_the_gap():
    vertices = np.array([[x, y, z] for x in (-.01, .01) for y in (-.01, .01) for z in (-.01, .01)])
    faces = ConvexHull(vertices).simplices
    source = optimizer.Mesh(np.vstack((vertices, vertices + [.1, 0, 0])), np.vstack((faces, faces + 8)))
    output, reports = optimizer.optimize(source, SETTINGS)
    parts = optimizer.components(output)
    assert len(parts) == len(reports) == 2
    for part in parts:
        eq = ConvexHull(part.vertices).equations
        assert np.max(np.array([.05, 0, 0]) @ eq[:, :3].T + eq[:, 3]) > 0


@pytest.mark.parametrize('filename', SETTINGS['source_meshes'])
def test_committed_assets_meet_component_geometry_bounds(filename):
    source_path = PACKAGE / 'meshes' / filename
    output_path = PACKAGE / SETTINGS['output_directory'] / filename
    manifest = json.loads((output_path.parent / 'arm_meshes.json').read_text())['meshes'][filename]
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['source_sha256']
    assert hashlib.sha256(output_path.read_bytes()).hexdigest() == manifest['output_sha256']
    source = optimizer.read_stl(source_path)
    output = optimizer.read_stl(output_path)
    originals = optimizer.components(source)
    reduced = optimizer.components(output)
    assert len(originals) == len(reduced) == len(manifest['components'])
    maximum_fraction = .08 if filename in SETTINGS.get('surface_components', {}) else (.02 if '_5cm' in filename else .15)
    assert len(output.vertices) < len(source.vertices) * maximum_fraction
    assert len(output.faces) < len(source.faces) * maximum_fraction
    used = set()
    for component_index, part in enumerate(originals):
        surface = SETTINGS.get('surface_components', {}).get(filename, {}).get(str(component_index))
        # Nested CAD parts can lie inside another component's hull. Match by
        # both containment and the independently measured expansion limit.
        original_hull = ConvexHull(part.vertices)
        matches = []
        for index, candidate in enumerate(reduced):
            if surface:
                bounds_difference = np.max(np.abs(np.r_[part.vertices.min(0), part.vertices.max(0)] -
                    np.r_[candidate.vertices.min(0), candidate.vertices.max(0)]))
                if bounds_difference <= surface['max_surface_distance_m']:
                    matches.append(index)
                continue
            if optimizer.maximum_plane_violation(part.vertices, ConvexHull(candidate.vertices).equations) > 0:
                continue
            bounds, _ = optimizer.radial_bounds(candidate.vertices, original_hull, part.vertices.mean(axis=0))
            if bounds.max() <= SETTINGS['max_hull_expansion_m']:
                matches.append(index)
        assert len(matches) == 1
        index = matches[0]
        assert index not in used
        used.add(index)
        candidate = reduced[index]
        if surface:
            optimizer.directed_bound(part, candidate, surface['max_surface_distance_m'])
            optimizer.directed_bound(candidate, part, surface['max_surface_distance_m'])
            def euler(mesh):
                edges = np.sort(mesh.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
                return len(mesh.vertices) - len(np.unique(edges, axis=0)) + len(mesh.faces)
            assert euler(candidate) == euler(part)
        else:
            bounds, _ = optimizer.radial_bounds(candidate.vertices, original_hull, part.vertices.mean(axis=0))
            assert bounds.max() <= SETTINGS['max_hull_expansion_m']
        edges = np.sort(candidate.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        assert np.all(counts == 2)  # Closed two-manifold after serialization.
        triangles = candidate.vertices[candidate.faces]
        normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        if not surface:
            assert np.all(np.einsum('ij,ij->i', normal, triangles.mean(axis=1) - candidate.vertices.mean(axis=0)) > 0)
        # Enclosures contain complete source triangles; the surface branch
        # instead certifies both complete surfaces by adaptive subdivision.


@pytest.mark.parametrize('entrypoint', ['cleany.urdf.xacro', 'cleany_control.urdf.xacro'])
def test_mesh_toggle_changes_only_selected_collision_resources(entrypoint):
    def expand(value):
        root = ET.fromstring(subprocess.check_output([
            'xacro', str(PACKAGE / 'urdf' / entrypoint), f'optimized_arm_collision_meshes:={value}',
        ]))
        for element in root.iter():
            element.text = element.tail = None
        return root
    original, reduced = expand('false'), expand('true')
    altered = []
    for mesh in reduced.findall('./link/collision/geometry/mesh'):
        filename = mesh.get('filename')
        if '/meshes/collision/' in filename:
            altered.append(filename)
            mesh.set('filename', filename.replace('/meshes/collision/', '/meshes/'))
    assert len(altered) == 2 * len(SETTINGS['source_meshes'])
    assert sorted(path.split('/')[-1] for path in altered) == sorted(SETTINGS['source_meshes'] * 2)
    assert ET.tostring(original) == ET.tostring(reduced)


def test_bad_enclosure_settings_fail():
    with pytest.raises(ValueError, match='settings'):
        optimizer.enclosing_hull(optimizer.Mesh(np.eye(3), np.array([[0, 1, 2]])),
            max_expansion=.0001, margin=.001, max_iterations=1, planes_per_iteration=1)


def test_surface_distance_index_matches_brute_force():
    from collision_surface_geometry import TriangleIndex, distances
    rng = np.random.default_rng(131)
    triangles = rng.normal(size=(70, 3, 3))
    points = rng.normal(size=(200, 3))
    brute = distances(points, np.broadcast_to(triangles, (len(points), *triangles.shape)))
    indexed = TriangleIndex(triangles).query(points, np.full(len(points), np.inf))
    np.testing.assert_allclose(indexed, brute, atol=1e-12)


def test_surface_certificate_accepts_small_shift_and_rejects_large_shift():
    vertices = np.array([[0., 0., 0.], [.001, 0., 0.], [0., .001, 0.], [0., 0., .001]])
    mesh = optimizer.Mesh(vertices, ConvexHull(vertices).simplices)
    small = optimizer.Mesh(vertices + .00001, mesh.faces)
    optimizer.directed_bound(mesh, small, .00025)
    with pytest.raises(ValueError, match='Surface distance'):
        optimizer.directed_bound(mesh, optimizer.Mesh(vertices + .01, mesh.faces), .00025)


def test_point_triangle_distance_covers_face_edge_vertex_and_degeneracy():
    from collision_surface_geometry import distances
    triangle = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    points = np.array([[.2, .2, 1.], [.5, -1., 0.], [-1., -1., 0.]])
    actual = distances(points, np.broadcast_to(triangle, (3, 1, 3, 3)))
    np.testing.assert_allclose(actual, [1., 1., np.sqrt(2.)], atol=1e-12)
    degenerate = np.zeros((1, 1, 3, 3))
    np.testing.assert_allclose(distances(np.array([[1., 0., 0.]]), degenerate), [1.])
