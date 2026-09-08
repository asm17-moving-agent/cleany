import numpy as np
import pytest
from cleany_skill_executor.core.pose_refinement import refine_pose


def test_bounded_refinement_converges_and_never_queries_outside_limits():
    def residual(q):
        assert np.all(q >= -1) and np.all(q <= 1)
        return np.array([q[0]-.4, np.sin(q[1])-.3])
    result = refine_pose(residual, [-.8,.8], [(-1,1),(-1,1)])
    assert result == pytest.approx((.4,np.arcsin(.3)),abs=1e-4)


def test_unreachable_residual_stays_bounded_and_nan_rejected():
    assert refine_pose(lambda q:q-3,[0.],[(-1,1)])[0] <= 1
    with pytest.raises(ValueError):
        refine_pose(lambda q:np.array([np.nan]),[0.],[(-1,1)])
