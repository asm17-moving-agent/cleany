"""Solid coverage and alignment of housing sphere pairs."""
import itertools
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation'))
from fit_joint_spheres import fit_housing_pair, sphere_pair_volume
from prune_sphere_fit import certify_cell


@pytest.mark.parametrize('separation,expected', [(0., 4*np.pi/3), (2., 8*np.pi/3),
                                                (1., 9*np.pi/4)])
def test_union_volume_accounts_for_containment_and_overlap(separation, expected):
    assert sphere_pair_volume(np.array([[0., 0., 0.], [separation, 0., 0.]]),
                              np.ones(2)) == pytest.approx(expected)


def test_axial_partition_stays_centered_and_covers_the_cut_face():
    housing = np.array(list(itertools.product((-.02, .02), (-.06, .06), (-.01, .01))))
    balls, report = fit_housing_pair([housing], 1, .0005)
    c = np.array([b[0] for b in balls])
    r = np.array([b[1]+.000500001 for b in balls])
    np.testing.assert_allclose(c[:, [0, 2]], 0., atol=1e-8)
    assert c[0, 1] < 0. < c[1, 1]
    assert max(r) < .038
    assert report['partition_volume_m3'] == pytest.approx(report['source_volume_m3'])
    assert certify_cell(housing, c, r, .0005) is not None


def test_partition_axis_must_be_explicit():
    with pytest.raises(ValueError, match='partition_axis'):
        fit_housing_pair([], None, .0005)
