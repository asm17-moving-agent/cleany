import numpy as np
import pytest
from cleany_gazebo_sim.body_geometry import BodyBox, nearest_body, swept_hits

def box(center=(0,0,0), half=(.1,.1,.1), rotation=None):
    return BodyBox('part', np.array(center), np.eye(3) if rotation is None else rotation, np.array(half))

def test_height_and_gap_are_preserved():
    parts=[box((0,0,0)),box((0,0,1))]
    distance,_=nearest_body(np.array([[0,0,.5],[0,0,.05],[.13,0,1]]),parts)
    np.testing.assert_allclose(distance,[.4,0,.03])

def test_oriented_part_distance_and_margin():
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    distances,_=nearest_body(np.array([[0,.3,0],[.2,0,0]]),[box(half=(.4,.1,.1),rotation=rotation)])
    np.testing.assert_allclose(distances,[0,.1])

def test_translation_and_rotation_sweep_detect_future_contact():
    point=np.array([[.4,0,0]])
    assert not swept_hits(point,[box()],(0,0,0),.02,.1,1,1)[0]
    assert swept_hits(point,[box()],(.5,0,0),.02,.1,1,1)[0]
    arm=box((.5,0,0),(.05,.05,.05))
    point=np.array([[0,.5,0]])
    assert swept_hits(point,[arm],(0,0,1.5),.01,.1,1,1)[0]

def test_invalid_geometry_and_parameters_fail_closed():
    with pytest.raises(ValueError):box(half=(-.1,.1,.1))
    with pytest.raises(ValueError):box(rotation=np.zeros((3,3)))
    with pytest.raises(ValueError):box(center=(np.nan,0,0))
    with pytest.raises(ValueError):nearest_body(np.array([[np.nan,0,0]]),[box()])
    with pytest.raises(ValueError):nearest_body(np.empty((0,3)),[])
    with pytest.raises(ValueError):swept_hits(np.zeros((1,3)),[box()],(0,0,0),.02,0,0,1)


def test_merged_box_contains_rotated_parts_without_extra_margin():
    from itertools import product
    from cleany_gazebo_sim.body_geometry import merge_body_boxes
    angle = .6
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    parts = [box((.2, -.1, .3), (.1, .04, .06), rotation),
             box((-.1, .1, .3), (.03, .07, .06))]
    frame = np.eye(4)
    frame[:3, :3] = rotation
    frame[:3, 3] = [.1, -.2, .3]
    merged = merge_body_boxes(parts, frame, 'wheel/merged')
    corners = np.concatenate([np.array(list(product((-1., 1.), repeat=3)))
                              *b.half_size@b.rotation.T+b.center for b in parts])
    np.testing.assert_allclose(merged.distances(corners), 0, atol=1e-12)
    local = (corners-merged.center)@merged.rotation
    np.testing.assert_allclose(local.min(axis=0), -merged.half_size, atol=1e-12)
    np.testing.assert_allclose(local.max(axis=0), merged.half_size, atol=1e-12)
