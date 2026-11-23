import math

import pytest

from cleany_teleop.gamepad import GamepadControl, Mapping
from cleany_teleop.web_control import Control


def controller(speed=0.5, axis_limit=0.5):
    profile = {"geometry": {"wheel_radius_m": 0.0635},
               "limits": {"linear_x_mps": axis_limit, "linear_y_mps": axis_limit, "wheel_rad_s": 10.0}}
    return GamepadControl(Mapping(speed=speed), profile)


def sample(pad, t, *, forward=0, left=0, lb=0, rb=0, right_stick=0, lt=0, rt=0):
    return pad.sample([left, forward, lt, right_stick, 0, rt], [0, 0, 0, 0, lb, rb], t, 0, t)


def enable(pad):
    assert sample(pad, 1) is None
    assert sample(pad, 1.03, forward=1) == ("press", [0, 0, 0])


def test_stick_drives_without_button_and_neutral_stops_immediately():
    pad = controller()
    enable(pad)
    assert sample(pad, 1.04, forward=0.8) == ("update", [0.8, 0, 0])
    assert sample(pad, 1.05, forward=0.03) == ("stop", None)
    assert sample(pad, 1.06, rb=1) is None
    assert sample(pad, 1.07, left=-1) is None
    assert sample(pad, 1.131, left=-1)[0] == "press"
    assert sample(pad, 1.15, left=-1) == ("update", [0, -1, 0])


@pytest.mark.parametrize("turn", [-1., -.25, .25, 1.])
def test_right_stick_turns_proportionally_alone_and_during_translation(turn):
    pad = controller()
    assert sample(pad, 1, right_stick=turn) is None
    assert sample(pad, 1.01, lb=1) is None
    assert sample(pad, 1.02, rb=1, right_stick=.04) is None
    assert sample(pad, 1.03, right_stick=turn)[0] == "press"
    assert sample(pad, 1.04, right_stick=turn) == ("update", [0, 0, turn])
    assert sample(pad, 1.12, forward=-.5, right_stick=turn) == ("update", [-.5, 0, turn])
    assert sample(pad, 1.13, forward=-.5, lb=1) == ("update", [-.5, 0, 0])
    assert sample(pad, 1.14, rb=1) == ("stop", None)
    assert sample(pad, 1.15, lb=1, rb=1) is None


def test_steering_requires_physical_startup_neutral_and_ignores_vertical_axis():
    pad = controller()
    def conditioned(t, turn, raw_turn, tilt):
        axes = [0, 0, 0, turn, tilt, 0, 0, 0, raw_turn, tilt]
        return pad.sample(axes, [0]*11, t, 0, t)

    assert conditioned(1, 0, .8, 0) is None
    assert conditioned(1.02, .8, .8, 0) is None
    assert conditioned(1.04, 0, 0, 1) is None
    assert conditioned(1.06, -.5, -.5, 1)[0] == "press"
    assert conditioned(1.08, -.5, -.5, 0) == ("update", [0, 0, -.5])
    assert conditioned(1.09, 0, .3, 0) == ("stop", None)


def test_release_ringing_does_not_reclaim_wheels_even_if_center_report_is_skipped():
    pad = controller()
    enable(pad)
    assert sample(pad, 1.04, forward=.97, left=.40)[0] == "update"
    assert sample(pad, 1.06, forward=-.62, left=-.30) == ("stop", None)
    for t, f, l in [(1.08, .37, .16), (1.10, -.15, -.10), (1.12, 0, 0),
                    (1.14, 0, 0), (1.16, 0, 0), (1.181, 0, 0)]:
        assert sample(pad, t, forward=f, left=l) is None
    assert sample(pad, 1.19, forward=-.12)[0] == "press"
    assert sample(pad, 1.20, forward=-.12) == ("update", [-.12, 0, 0])


def test_neutral_required_at_start_and_only_fresh_input_can_resume():
    pad = controller()
    assert sample(pad, .96, forward=1) is None
    enable(pad)
    assert pad.timeout(1.19) == ("stop", None)
    assert pad.sample([0, 0, 0, 1, 0, 0], [0]*6, 1.17, .03, 1.20) is None
    assert sample(pad, 1.22, right_stick=.5)[0] == "press"
    assert pad.interrupt(1.23) == ("stop", None)
    assert pad.sample([0, 0, 0, 1, 0, 0], [0]*6, 1.225, .01, 1.235) is None


@pytest.mark.parametrize(
    "axes,buttons,age",
    [
        ([], [], 0),
        ([0] * 6, [0] * 5, 0),
        ([0] * 9, [0] * 11, 0),
        ([math.nan] * 6, [0] * 6, 0),
        ([1.1] * 6, [0] * 6, 0),
        ([0] * 6, [2] * 6, 0),
        ([0, 0, 0, math.nan, 0, 0], [0] * 6, 0),
        ([0] * 6, [1] * 6, -0.1),
    ],
)
def test_invalid_or_buffered_input_stops_owner(axes, buttons, age):
    pad = controller()
    enable(pad)
    assert pad.sample(axes, buttons, 1.06, age, 1.06) == ("stop", None)
    assert pad.sample([0, 1, 0, 0, 0, 0], [0]*6, 1.04, .08, 1.12) is None


def test_diagonal_forward_only_and_expiry_even_if_timer_is_late():
    pad = controller()
    enable(pad)
    assert sample(pad, 1.11, forward=1, left=1)[1] == pytest.approx(
        [2**-0.5, 2**-0.5, 0]
    )
    pad.forward_only = True
    assert sample(pad, 1.19, left=1, right_stick=-1)[1] == [0, 0, -1]
    assert sample(pad, 1.5, right_stick=-1) == ("stop", None)
    assert sample(pad, 1.51, right_stick=-1)[0] == "press"


def test_trigger_speed_while_moving_and_release():
    pad = controller(speed=2)
    enable(pad)
    for t, lt, rt, expected in [(1.04, 0, 0, .127), (1.06, 0, .5, .5),
                                 (1.08, 1, 1, .1), (1.10, 1, 0, .1), (1.20, 0, .49, .127)]:
        cmd = sample(pad, t, forward=1, lt=lt, rt=rt)
        assert pad.command_speed * .0635 == pytest.approx(expected)
        if cmd is not None:
            assert cmd == ("update", [1, 0, 0])
    assert sample(pad, 1.22, rt=1) == ("stop", None)
    assert sample(pad, 1.25, rt=1) is None


def test_triggers_leave_rotation_at_base_speed_and_respect_lower_profile():
    pad = controller(speed=2, axis_limit=.3)
    assert sample(pad, 1, rt=1) is None
    assert sample(pad, 1.03, right_stick=1, rt=1)[0] == "press"
    assert sample(pad, 1.05, right_stick=1, rt=1) == ("update", [0, 0, 1])
    assert pad.command_speed == 2
    _, vector = sample(pad, 1.13, forward=.5, right_stick=1, rt=1)
    assert vector[0] * pad.command_speed * .0635 == pytest.approx(.15)
    assert vector[2] * pad.command_speed == pytest.approx(2)
    assert sample(pad, 1.16, forward=1, rt=math.nan) == ("stop", None)


def test_filtered_zero_cannot_rearm_drive_and_partial_release_bypasses_rate_limit():
    pad = controller()
    buttons = [0] * 11
    def feed(t, x, y, raw_x, raw_y):
        return pad.sample([x, y, 0, 0, 0, 0, raw_x, raw_y, 0, 0], buttons, t, 0, t)

    assert feed(1, 0, 0, -.6, 0) is None and not pad.startup_neutral_seen
    assert feed(1.02, -.6, 0, -.6, 0) is None
    assert feed(1.04, 0, 0, .01, .02) is None and pad.startup_neutral_seen
    assert feed(1.06, .4, .7, .4, .7)[0] == "press"
    assert feed(1.08, .4, .7, .4, .7) == ("update", [.7, .4, 0])
    assert feed(1.081, 0, .7, -.3, .7) == ("update", [.7, 0, 0])
    assert feed(1.09, 0, 0, -.3, -.3) == ("stop", None)
    pad.timeout(1.3)
    assert feed(1.31, 0, 0, -.3, -.3) is None
    assert feed(1.33, 0, 0, math.nan, 0) is None


def test_late_gamepad_command_and_scoped_release_do_not_stop_phone():
    packets = []
    ctrl = Control(packets.append, now=lambda: 10)
    ctrl.owner = "phone"
    with pytest.raises(ValueError, match="만료"):
        ctrl.request(
            dict(client="pad", sequence=1, action="gamepad_press", expires_at=9)
        )
    ctrl.request(dict(client="pad", sequence=2, action="release"))
    assert ctrl.owner == "phone" and packets == []
    ctrl.owner = "pad"
    ctrl.request(dict(client="pad", sequence=3, action="release"))
    assert ctrl.owner is None and packets[-1].mode == ctrl.STOP
    with pytest.raises(ValueError, match="이전"):
        ctrl.request(
            dict(client="pad", sequence=2, action="gamepad_press", expires_at=10.1)
        )


@pytest.mark.parametrize("phase", ["preparing", "active", "stopping"])
def test_gamepad_cannot_preempt_navigation(phase):
    from types import SimpleNamespace

    from cleany_teleop.web import check_gamepad_navigation

    navigation = SimpleNamespace(phase=phase)
    for action in ("gamepad_press", "gamepad_update"):
        with pytest.raises(ValueError, match="자동 이동"):
            check_gamepad_navigation(navigation, action)
    check_gamepad_navigation(navigation, "release")
    check_gamepad_navigation(navigation, "stop")
    check_gamepad_navigation(SimpleNamespace(phase="idle"), "gamepad_press")
