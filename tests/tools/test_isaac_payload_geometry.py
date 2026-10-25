from pathlib import Path
import math

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def cup(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    monkeypatch.syspath_prepend(str(ROOT/'ros2_ws/src/cleany_mujoco_sim'))
    from payload_geometry import PayloadGeometry
    return PayloadGeometry.from_payload(dict(visual_type='paper_cup',
        collision={'size_m': [.0612, .095]}, geometry=dict(
            bottom_diameter_m=.0396, wall_thickness_m=.000504, segments=32)))


def test_upright_cup_has_original_radius_and_support_bottom(monkeypatch):
    geometry = cup(monkeypatch)
    center = np.array([.5, -.2, .8])
    low, high = geometry.bounds(center, np.eye(3))
    assert low == pytest.approx(center-np.array([.0306, .0306, .0475]))
    assert high == pytest.approx(center+np.array([.0306, .0306, .0475]))


def test_tilted_taper_uses_surface_extrema_instead_of_box_corners(monkeypatch):
    geometry = cup(monkeypatch)
    c = math.sqrt(.5)
    rotation = np.array([[c, 0., c], [0., 1., 0.], [-c, 0., c]])
    low, high = geometry.bounds(np.zeros(3), rotation)
    # The lower edge has the smaller bottom radius; the upper edge is wider.
    assert low[2] == pytest.approx(-c*(.0475+.0198))
    assert high[2] == pytest.approx(c*(.0475+.0306))
    vertices = geometry.vertices @ rotation.T
    assert np.all(vertices >= low-1e-12) and np.all(vertices <= high+1e-12)
    old_low = -(np.abs(rotation) @ np.array([.0306, .0306, .0475]))
    assert low[2]-old_low[2] > .007


def test_bottom_cylinder_extrema_are_preserved_between_polygon_vertices(monkeypatch):
    geometry = cup(monkeypatch)
    angle = math.pi/32
    c, s = math.cos(angle), math.sin(angle)
    # Tip the cylinder axis horizontally and rotate it between polygon angles.
    rotation = np.array([[0., 0., 1.], [c, -s, 0.], [s, c, 0.]])
    low, high = geometry.bounds(np.zeros(3), rotation)
    assert low[0] == pytest.approx(-.0475)
    assert high[0] == pytest.approx(.0475)
    assert low[1] <= -.0198 and high[1] >= .0198
