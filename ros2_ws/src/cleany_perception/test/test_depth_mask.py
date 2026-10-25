import numpy as np
import pytest
from cleany_perception.core.depth_mask import dominant_depth_mask


def clean(depth, mask):
    return dominant_depth_mask(depth, mask, minimum_depth_m=.1, maximum_depth_m=3.,
                               maximum_jump_m=.02, minimum_fraction=.7)


def test_adjacent_foreground_contamination_is_excluded_but_sloped_object_retained():
    depth = np.ones((10, 12), dtype=float)
    depth[:, :10] += np.arange(10)*.01
    depth[:, 10:] = .5
    mask = np.ones_like(depth, dtype=bool)
    result = clean(depth, mask)
    assert result[:, :10].all() and not result[:, 10:].any()
    assert mask.all()  # Input observation is not mutated.


def test_two_comparable_surfaces_fail_instead_of_silently_selecting_one():
    depth = np.ones((10, 10))
    depth[:, 5:] = .5
    with pytest.raises(ValueError, match='dominant'):
        clean(depth, np.ones_like(depth, dtype=bool))


def test_invalid_depth_is_not_a_surface_and_empty_depth_fails():
    depth = np.ones((10, 10))
    depth[0, :3] = [np.nan, np.inf, 0.]
    result = clean(depth, np.ones_like(depth, dtype=bool))
    assert result.sum() == 97
    with pytest.raises(ValueError, match='dominant'):
        clean(np.zeros((3, 3)), np.ones((3, 3), dtype=bool))
