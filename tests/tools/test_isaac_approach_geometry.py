from pathlib import Path
import math

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_rotated_collision_bounds_include_all_chair_parts(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from approach_geometry import chair_bounds
    transform = [[0., -1., 0., 2.], [1., 0., 0., 3.],
                 [0., 0., 1., 1.], [0., 0., 0., 1.]]
    parts = [dict(kind='collision', model='office_chair_28', transform=transform,
                  shape=dict(type='box', size=[.1, .6, .8])),
             dict(kind='collision', model='office_chair_28', transform=transform,
                  shape=dict(type='cylinder', radius=.4, length=.2))]
    parts.append(dict(parts[0], kind='visual', model='ignored_chair'))
    bounds = chair_bounds(parts, tmp_path)
    assert list(bounds) == ['office_chair_28']
    assert bounds['office_chair_28'][0] == pytest.approx([1.6, 2.6, .6])
    assert bounds['office_chair_28'][1] == pytest.approx([2.4, 3.4, 1.4])


def test_chair_overlap_uses_rotated_full_footprint_and_padding(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from approach_geometry import overlapping_chairs
    polygon = np.array([[-.48, -.4], [.48, -.4], [.48, .4], [-.48, .4]])
    chairs = {'chair': (np.array([-.05, .51, 0.]), np.array([.05, .61, 1.]))}
    assert overlapping_chairs(np.zeros(2), 0., polygon, .04, chairs) == []
    assert overlapping_chairs(np.zeros(2), math.pi/2, polygon, .04, chairs) == ['chair']
    assert overlapping_chairs(np.zeros(2), math.pi/2, polygon, 0., chairs) == []
    assert overlapping_chairs(np.array([1., 0.]), math.pi/2, polygon, .04, chairs) == []
