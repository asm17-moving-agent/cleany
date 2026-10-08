import math
import pytest
from stimuli import obstacle_ahead,scene_model_names


def test_omni_lateral_motion_places_obstacle_on_travel_path_not_body_heading():
    current={'x':2.,'y':3.,'yaw':math.pi/2}
    position=obstacle_ahead(current,{'x':1.,'y':3.},1.)
    assert position==[3.,3.,0.]


def test_unknown_motion_direction_is_not_used_for_obstacle_trial():
    with pytest.raises(ValueError):
        obstacle_ahead({'x':1.,'y':2.},{'x':1.,'y':2.},1.)


def test_scene_names_ignore_nested_links_models_and_quoted_braces():
    scene='model { name: "chair_01" link { name: "chair_link" visual { name: "{visual}" } } model { name: "nested" } } model { name: "robot" }'
    assert scene_model_names(scene)=={'chair_01','robot'}


@pytest.mark.parametrize('text',['','model { name: "chair"','model { name: "chair" } model { name: "chair" }','model { id: 1 }'])
def test_incomplete_or_ambiguous_scene_fails_closed(text):
    with pytest.raises(ValueError):scene_model_names(text)
