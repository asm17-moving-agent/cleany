"""Continuous primitive coverage for additive wrist mount spheres."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import ConvexHull
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation'))
from audit_joint_spheres import primitive_coverage, primitive_union_coverage
from fit_joint_spheres import fit_housing_pair, primitive_components, refine_housing_ball


def test_rotated_cylinder_enclosure_contains_between_polygon_vertices():
    link = ET.fromstring('''<link name="wrist">
      <collision name="mount_lens"><origin xyz=".1 -.2 .3" rpy=".3 -.4 .8"/>
        <geometry><cylinder radius=".006" length=".003"/></geometry></collision>
      </link>''')
    parts = primitive_components(link, {'collision_name_contains': 'mount_', 'expected_components': 1})
    theta = np.linspace(0., 2*np.pi, 513)
    points = np.concatenate([np.column_stack((.006*np.cos(theta), .006*np.sin(theta),
                            np.full(len(theta), z))) for z in (-.0015, .0015)])
    rotation = trimesh.transformations.euler_matrix(.3, -.4, .8)[:3, :3]
    hull = ConvexHull(parts[0])
    assert ((points @ rotation.T+[.1, -.2, .3]) @ hull.equations[:, :3].T
            + hull.equations[:, 3]).max() < 1e-12
    with pytest.raises(ValueError, match='selection changed'):
        primitive_components(link, {'collision_name_contains': 'mount_', 'expected_components': 2})


def test_analytic_cylinder_check_rejects_a_gap_missed_by_polygon_vertices():
    link = ET.fromstring('''<link name="wrist">
      <collision name="mount_box"><geometry><box size=".01 .01 .01"/></geometry></collision>
      <collision name="mount_lens"><geometry><cylinder radius="1" length=".2"/></geometry></collision>
      </link>''')
    selector = {'collision_name_contains': 'mount_', 'expected_components': 2}
    center, padding = np.array([.2, .1, 0.]), .0005
    angles = np.arange(16)*2*np.pi/16
    polygon = np.column_stack((np.cos(angles), np.sin(angles), np.full(16, .1)))
    radius = float(np.linalg.norm(polygon-center, axis=1).max()+padding)
    with pytest.raises(AssertionError, match='mount_lens'):
        primitive_coverage(link, selector, {'center': center, 'radius': radius}, padding)
    radius = float(np.hypot(np.linalg.norm(center[:2])+1., .1)+padding+1e-9)
    checks, _ = primitive_coverage(link, selector, {'center': center, 'radius': radius}, padding)
    assert len(checks) == 2


def test_primitive_pair_covers_solid_and_rejects_a_gap_between_balls():
    link = ET.fromstring('''<link name="wrist">
      <collision name="mount_box"><geometry><box size=".01 .1 .01"/></geometry></collision>
      </link>''')
    selector = {'collision_name_contains': 'mount_', 'expected_components': 1}
    parts = primitive_components(link, selector)
    padding = .0005
    balls, _ = fit_housing_pair(parts, 1, padding)
    spheres = [{'center': c, 'radius': r+padding+1e-9} for c, r in balls]
    assert len(primitive_union_coverage(link, selector, spheres, padding)) == 1
    # The eight source corners are enclosed, but the solid between them is not.
    bad = [{'center': [0., y, 0.], 'radius': .02} for y in (-.05, .05)]
    with pytest.raises(AssertionError, match='solid is not covered'):
        primitive_union_coverage(link, selector, bad, padding)


def test_wrist_lower_refinement_keeps_upper_ball_and_certifies_the_overlap():
    source = Path(__file__).resolve().parents[2]/'ros2_ws/src/cleany_description/urdf/wrist_mount_v8.xacro'
    # Collision dimensions and origins are numeric; only names use the side macro.
    link = ET.parse(source).getroot().find('{http://www.ros.org/wiki/xacro}macro')
    selector = {'collision_name_contains': 'wrist_mount_v8_', 'expected_components': 15}
    parts = primitive_components(link, selector)
    padding = .0005
    original, _ = fit_housing_pair(parts, 1, padding)
    refined, report = refine_housing_ball(parts, original, 1, padding, .000001, 64)
    np.testing.assert_array_equal(refined[0][0], original[0][0])
    assert refined[0][1] == original[0][1]
    assert refined[1][1]+padding < .027
    assert refined[1][1] < original[1][1]-.002
    assert refined[1][0][2] < original[1][0][2]-.004
    assert report['iterations'] > 1
    spheres = [{'center': c, 'radius': r+padding+1e-9} for c, r in refined]
    assert len(primitive_union_coverage(link, selector, spheres, padding)) == 15
    with pytest.raises(ValueError, match='complete solid coverage'):
        refine_housing_ball(parts, original, 1, padding, .000001, 1)
