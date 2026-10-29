"""Regression checks for solid certificates and independently owned housings."""
import itertools
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation'))
from audit_joint_spheres import mesh_region_coverage
from prune_sphere_fit import certify_cell, tetra_volume


def test_vertex_cover_does_not_accept_an_interior_hole():
    tetra = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    witnesses = []
    assert certify_cell(tetra, tetra.copy(), np.full(4, .1), .01,
                        failure_points=witnesses) is None
    assert witnesses
    assert all((np.linalg.norm(point-tetra, axis=1)-.09).min() > 0 for point in witnesses)


def test_certificate_preserves_volume_and_padding():
    cube = np.array(list(itertools.product((-.1, .1), repeat=3)))
    centers, radii, padding = np.zeros((1, 3)), np.array([.2]), .0005
    tetra, owners = certify_cell(cube, centers, radii, padding)
    assert tetra_volume(tetra).sum() == pytest.approx(.008)
    assert (np.linalg.norm(tetra-centers[owners, None], axis=2)-radii[owners, None]).max() <= -padding


def test_budget_exhaustion_is_not_an_uncovered_point():
    cube = np.array(list(itertools.product((-.1, .1), repeat=3)))
    witnesses = []
    assert certify_cell(cube, np.array([[-.05, 0., 0.], [.05, 0., 0.]]),
                        np.full(2, .151), .0005, max_nodes=0,
                        failure_points=witnesses) is None
    assert witnesses == []


def test_housing_cannot_borrow_coverage_from_another_regions_ball(tmp_path):
    path = tmp_path/'housing.stl'
    trimesh.creation.box(extents=(.02, .02, .02)).export(path)
    link = ET.fromstring(f'<link name="head"><collision><geometry><mesh filename="{path}"/>'
                         '</geometry></collision></link>')
    selector = [{'mesh': path.name, 'expected_components': 1}]
    own = {'center': [0., 0., 0.], 'radius': .015}
    neighbour = {'center': [.001, 0., 0.], 'radius': .03}
    assert mesh_region_coverage(link, selector, [own, neighbour], .0005)
    with pytest.raises(ValueError, match='Rigid mesh region is not covered: head'):
        mesh_region_coverage(link, selector, [own], .0005)
    own['radius'] = .02
    assert mesh_region_coverage(link, selector, [own], .0005)
