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
    manifest = json.loads((PACKAGE / SETTINGS['output_directory'] / 'arm_meshes.json').read_text())['meshes'][filename]
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == manifest['source_sha256']
    assert hashlib.sha256(output_path.read_bytes()).hexdigest() == manifest['output_sha256']
    source = optimizer.read_stl(source_path)
    output = optimizer.read_stl(output_path)
    scale = SETTINGS.get('mesh_scales', {}).get(filename, 1.)
    source = optimizer.Mesh(source.vertices * scale, source.faces)
    output = optimizer.Mesh(output.vertices * scale, output.faces)
    originals = optimizer.source_pieces(source, SETTINGS, filename)
    reduced = optimizer.components(output)
    assert len(originals) == len(reduced) == len(manifest['components'])
    maximum_fraction = .35 if filename in SETTINGS.get('mesh_scales', {}) else (.02 if '_5cm' in filename else .15)
    assert len(output.vertices) < len(source.vertices) * maximum_fraction
    assert len(output.faces) < len(source.faces) * maximum_fraction
    used = set()
    for component_index, part in originals:
        # Nested CAD parts can lie inside another component's hull. Match by
        # both containment and the independently measured expansion limit.
        original_hull = ConvexHull(part.vertices)
        matches = []
        for index, candidate in enumerate(reduced):
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
        bounds, _ = optimizer.radial_bounds(candidate.vertices, original_hull, part.vertices.mean(axis=0))
        assert bounds.max() <= SETTINGS['max_hull_expansion_m']
        edges = np.sort(candidate.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        assert np.all(counts == 2)  # Closed two-manifold after serialization.
        triangles = candidate.vertices[candidate.faces]
        normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        assert np.all(np.einsum('ij,ij->i', normal, triangles.mean(axis=1) - candidate.vertices.mean(axis=0)) > 0)
        unit = normal / np.linalg.norm(normal, axis=1, keepdims=True)
        assert (candidate.vertices @ unit.T - np.einsum('ij,ij->i', unit, triangles[:, 0])).max() <= 1e-9


@pytest.mark.parametrize('entrypoint', ['cleany.urdf.xacro', 'cleany_control.urdf.xacro'])
@pytest.mark.parametrize('head_meshes', ['true', 'false'])
def test_mesh_toggle_changes_only_selected_collision_resources(entrypoint, head_meshes):
    def expand(value):
        root = ET.fromstring(subprocess.check_output([
            'xacro', str(PACKAGE / 'urdf' / entrypoint), f'optimized_arm_collision_meshes:={value}',
            'include_head_camera:=true', f'head_collision_meshes:={head_meshes}',
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
    expected = [mesh.get('filename') for mesh in original.findall('./link/collision/geometry/mesh')
                if mesh.get('filename').split('/meshes/')[1] in SETTINGS['source_meshes']]
    assert sorted(path.replace('/meshes/collision/', '/meshes/') for path in altered) == sorted(expected)
    if head_meshes == 'true' and entrypoint == 'cleany.urdf.xacro':
        assert len(altered) == 28
        assert {path.split('/meshes/')[1] for path in expected} == set(SETTINGS['source_meshes'])
        remaining = [mesh.get('filename').split('/meshes/')[1]
                     for mesh in original.findall('./link/collision/geometry/mesh')
                     if mesh.get('filename') not in expected]
        assert set(remaining) == set(SETTINGS['already_planar_meshes'])
    assert ET.tostring(original) == ET.tostring(reduced)


@pytest.mark.parametrize('entrypoint', ['cleany.urdf.xacro', 'cleany_control.urdf.xacro'])
def test_default_model_uses_the_optimized_collision_resources(entrypoint):
    command = ['xacro', str(PACKAGE / 'urdf' / entrypoint), 'include_head_camera:=true']
    default = ET.fromstring(subprocess.check_output(command))
    explicit = ET.fromstring(subprocess.check_output(command + [
        'optimized_arm_collision_meshes:=true', 'head_collision_meshes:=true']))
    def resources(root):
        return [m.attrib for m in root.findall('./link/collision/geometry/mesh')]
    assert resources(default) == resources(explicit)
    assert any('/meshes/collision/' in m['filename'] for m in resources(default))


def test_partition_retains_lower_arm_fork_and_complete_source_surface():
    filename = 'Lower_Arm_5cm.stl'
    source = optimizer.components(optimizer.read_stl(PACKAGE / 'meshes' / filename))[2]
    pieces = optimizer.partition_surface(source, SETTINGS['partition_components'][filename]['2'])
    assert len(pieces) == 3

    def area(mesh):
        triangles = mesh.vertices[mesh.faces]
        return np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                      triangles[:, 2] - triangles[:, 0]), axis=1).sum() / 2

    assert sum(area(piece) for piece in pieces) == pytest.approx(area(source), abs=1e-12)
    point_in_fork_gap = np.array([.065, -.032, 0.])
    for piece in pieces:
        reduced, _ = optimizer.enclosing_hull(piece, max_expansion=SETTINGS['max_hull_expansion_m'],
            margin=SETTINGS['enclosure_margin_m'], max_iterations=1024, planes_per_iteration=1)
        equations = ConvexHull(reduced.vertices).equations
        assert (point_in_fork_gap @ equations[:, :3].T + equations[:, 3]).max() > .01


def test_clipping_through_a_vertex_preserves_area_without_degenerate_faces():
    triangle = optimizer.Mesh(np.array([[-1., 0., 0.], [0., 1., 0.], [1., 0., 0.]]), np.array([[0, 1, 2]]))
    areas = []
    for sign in (-1, 1):
        part = optimizer.clip_surface(triangle, 0, 0., sign)
        assert len(part.faces) == 1
        assert len(np.unique(part.faces)) == 3
        a, b, c = part.vertices[part.faces[0]]
        areas.append(np.linalg.norm(np.cross(b - a, c - a)) / 2)
    assert sum(areas) == pytest.approx(1.)


def test_bad_enclosure_settings_fail():
    with pytest.raises(ValueError, match='settings'):
        optimizer.enclosing_hull(optimizer.Mesh(np.eye(3), np.array([[0, 1, 2]])),
            max_expansion=.0001, margin=.001, max_iterations=1, planes_per_iteration=1)
    with pytest.raises(ValueError, match='Surface decimation'):
        optimizer.optimize(optimizer.Mesh(np.eye(3), np.array([[0, 1, 2]])),
            {**SETTINGS, 'surface_components': {'retired.stl': {'0': {}}}})
