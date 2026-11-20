import math

import pytest

from cleany_telemetry.pose import PoseFreshness, quaternion_yaw


def test_source_time_frame_and_clock_reset():
    gate = PoseFreshness(1.5)
    assert gate.accept('odom', 10, 10)
    assert not gate.accept('odom', 10, 10.1)
    assert not gate.accept('odom', 9.9, 10.2)
    assert not gate.accept('odom', 10.1, 12)
    assert not gate.accept('odom', 13, 12)
    assert gate.accept('map', 11, 12)
    assert gate.accept('map', 1, 1)  # ROS clock reset, not out-of-order data.
    assert not gate.accept('map', 1, 1.1)
    assert gate.accept('map', 1.2, 1.2)


@pytest.mark.parametrize('frame,stamp,now', [('', 1, 1), (' ', 1, 1), ('map', 0, 1),
                                           ('map', math.nan, 1), ('map', 1, math.inf)])
def test_invalid_source_metadata(frame, stamp, now):
    assert not PoseFreshness().accept(frame, stamp, now)


def test_orientation_normalization():
    assert quaternion_yaw(0, 0, 2, 2) == pytest.approx(math.pi / 2)
    assert quaternion_yaw(0, 0, 0, 0) is None
