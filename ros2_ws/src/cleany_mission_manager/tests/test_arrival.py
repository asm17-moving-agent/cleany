import math
import pytest
from cleany_mission_manager.core.arrival import arrival_error


def test_path_and_controller_tolerances_cannot_stack_into_arrival_success():
    # 계획 끝점이 12cm 이동하고 제어기가 12cm 먼저 멈췄다면
    # 원래 요청한 책상 자세에 도착한 것이 아닙니다.
    error = arrival_error((1., 2., math.pi/2), (1., 1.76, math.pi/2))
    assert not error.within(.08, .15)
    assert not arrival_error((0., 0., 0.), (.01, .01, .2)).within(.08, .15)


def test_yaw_wrap_and_finite_precision_boundary():
    assert arrival_error((0., 0., math.pi-.01), (.08, 0., -math.pi+.01)).within(.08, .15)
    with pytest.raises(ValueError):
        arrival_error((0., 0., 0.), (math.nan, 0., 0.))
    with pytest.raises(ValueError):
        arrival_error((0., 0., 0.), (0., 0., 0.)).within(math.inf, .15)
