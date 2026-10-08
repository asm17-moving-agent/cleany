import math
import pytest
from stimuli import obstacle_ahead


def test_omni_lateral_motion_places_obstacle_on_travel_path_not_body_heading():
    current={'x':2.,'y':3.,'yaw':math.pi/2}
    position=obstacle_ahead(current,{'x':1.,'y':3.},1.)
    assert position==[3.,3.,0.]


def test_unknown_motion_direction_is_not_used_for_obstacle_trial():
    with pytest.raises(ValueError):
        obstacle_ahead({'x':1.,'y':2.},{'x':1.,'y':2.},1.)
