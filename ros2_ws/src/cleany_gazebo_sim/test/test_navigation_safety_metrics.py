import math

import pytest

from cleany_gazebo_sim.navigation_safety_metrics import rectangle_clearance


@pytest.mark.parametrize('pose, expected', [
    ((0.0, 0.0, 0.0), 0.5),
    ((0.0, 0.0, math.pi/2), 0.6),
    ((0.6, 0.0, 0.0), 0.0),
    ((0.5, 0.0, 0.0), 0.0),
])
def test_rotated_base_contact_and_clearance(pose, expected):
    assert rectangle_clearance(pose, (0.3, 0.2), (1.0, 0.0), (0.2, 0.2)) == pytest.approx(expected)


def test_diagonal_clearance_uses_edges_and_is_translation_invariant():
    expected = math.hypot(0.5, 0.6)
    for x, y in ((0, 0), (-10, 21)):
        assert rectangle_clearance((x, y, 0), (0.3, 0.2), (x+1, y+1), (0.2, 0.2)) == pytest.approx(expected)
