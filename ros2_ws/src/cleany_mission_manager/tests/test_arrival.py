import math
import pytest
from cleany_mission_manager.core.arrival import arrival_error


def test_path_and_controller_tolerances_cannot_stack_into_arrival_success():
    # A planner endpoint displaced 12 cm plus a controller stopping 12 cm
    # earlier is not an arrival at the original requested desk pose.
    error = arrival_error((1., 2., math.pi/2), (1., 1.76, math.pi/2))
    assert not error.within(.08, .15)
    assert not arrival_error((0., 0., 0.), (.01, .01, .2)).within(.08, .15)


def test_yaw_wrap_and_finite_precision_boundary():
    assert arrival_error((0., 0., math.pi-.01), (.08, 0., -math.pi+.01)).within(.08, .15)
    with pytest.raises(ValueError):
        arrival_error((0., 0., 0.), (math.nan, 0., 0.))
    with pytest.raises(ValueError):
        arrival_error((0., 0., 0.), (0., 0., 0.)).within(math.inf, .15)
