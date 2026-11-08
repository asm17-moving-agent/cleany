import math
import pytest
from cleany_teleop.map_view import MapView


def test_rotated_map_coordinates_and_unknown_rejection():
    m=MapView(2,2,1.,10.,20.,math.pi/2,[0,100,-1,0])
    assert m.validate_goal(dict(map_id=m.revision,x=9.5,y=20.5,yaw=0.))==(9.5,20.5,0.)
    for x,y in [(9.5,21.5),(8.5,20.5),(11.,20.)]:
        with pytest.raises(ValueError):m.validate_goal(dict(map_id=m.revision,x=x,y=y,yaw=0.))


def test_revision_and_invalid_values():
    m=MapView(1,1,.05,0.,0.,0.,[0]); old=m.revision
    for values in [dict(map_id='old',x=.01,y=.01,yaw=0),dict(map_id=old,x=float('nan'),y=0,yaw=0)]:
        with pytest.raises(ValueError):m.validate_goal(values)
    m.cells[0]=100
    assert m.revision!=old
