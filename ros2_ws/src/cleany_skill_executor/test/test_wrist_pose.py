import math
import pytest
from geometry_msgs.msg import Quaternion
from cleany_skill_executor.core.wrist_pose import carried_orientation


def test_relative_carried_orientation_and_invalid_quaternion():
    identity=Quaternion(w=1.)
    yaw=Quaternion(z=math.sin(.4),w=math.cos(.4))
    q=carried_orientation(identity,yaw,identity)
    assert q.z==pytest.approx(yaw.z) and q.w==pytest.approx(yaw.w)
    q=carried_orientation(yaw,yaw,identity)
    assert q.w==pytest.approx(1.)
    with pytest.raises(ValueError): carried_orientation(Quaternion(w=0.),yaw,identity)
