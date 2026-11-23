import json
import os
import signal
import sys
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from cleany_teleop.gamepad_head import HeadServos, HeadStick
from cleany_teleop import gamepad_head


class FakeBus:
    def __init__(self):
        self.r = {}
        self.writes = []
        for motor, pos, lo, hi in [(13, 2027, 8, 4001), (14, 958, 44, 1101)]:
            for addr, val in {3: 777, 9: lo, 11: hi, 33: 0, 40: 0, 41: 0, 42: 0,
                              44: 0, 46: 0, 48: 1000, 56: pos, 63: 30, 65: 0}.items():
                self.r[motor, addr] = val

    def read(self, motor, addr, size):
        return self.r[motor, addr]

    def write(self, motor, addr, size, value):
        self.writes.append((motor, addr, value))
        self.r[motor, addr] = value


def profile():
    return {'motor_ranges_ticks': {'13': [8, 4001], '14': [44, 1101]},
            'direction_verification': {'pan_left_raw_decreases': True, 'tilt_up_raw_decreases': True}}


def test_neutral_after_start_stale_or_invalid_right_stick():
    s = HeadStick()
    def sample(x, y, t, age=0):
        s.sample([1, 1, 0, x, y], round(t * 1e9), age, t)
        return s.current(t)
    assert sample(1, 1, 1) == (0, 0)
    assert sample(.02, 0, 1.01) == (0, 0)
    assert sample(1, -.5, 1.02) == (0, -.5)
    assert s.current(1.18) == (0, 0)
    assert sample(1, 1, 1.2) == (0, 1)
    sample(0, 0, 1.21)
    assert sample(1, 0, 1.22, .16) == (0, 0)
    assert sample(1, 0, 1.23) == (0, 0)


def test_head_holds_through_diagonal_release_and_accepts_a_sustained_reversal():
    stick = HeadStick()

    def axes(t_ms, x, y):
        stick.sample([0, 0, 0, x, y], t_ms * 1_000_000, 0, t_ms / 1000)
        return stick.current(t_ms / 1000)

    assert axes(1000, 0, 0) == (0, 0)
    assert axes(1020, -.97, -.41) == (0, -.41)
    assert axes(1040, 0, 0) == (0, 0)
    assert axes(1060, .62, .3) == (0, 0)
    assert axes(1080, -.37, -.16) == (0, 0)
    assert axes(1100, .4, .3) == (0, 0)
    assert axes(1140, .7, .5) == (0, 0)
    assert axes(1161, .8, .6) == (0, .6)


def test_hold_from_measured_pose_limit_lookahead_and_cleanup():
    b = FakeBus()
    h = HeadServos(b, profile())
    step = lambda axes: h.step(axes, .05, 1, lambda: 0)
    step((0, 0))
    assert not b.writes
    step((1, 0))
    assert b.writes.index((13, 42, 2027)) < b.writes.index((13, 40, 1))
    assert b.r[13, 46] == 341 and b.r[13, 42] == 2027 - 17
    assert all(m == 13 for m, _, _ in b.writes)
    for _ in range(20):
        step((1, 0))
    assert b.r[13, 42] == 2027 - 51
    b.r[13, 56] = 2020
    b.r[14, 56] = 46
    step((0, 1))
    assert (b.r[13, 42], b.r[14, 42]) == (2020, 44)
    assert b.r[14, 46] == 341
    step((-1, -1))
    assert b.r[13, 42] > 2020 and b.r[14, 42] > 46
    h.step((1, 1), .05, 0, lambda: 1)
    assert (b.r[13, 42], b.r[14, 42]) == (2020, 46)
    h.close()
    assert all(b.r[m, 40] == 0 and b.r[m, 48] == 1000 for m in (13, 14))


def test_head_requires_physical_neutral_and_does_not_delay_conditioned_input_twice():
    stick = HeadStick()
    def feed(ms, x, y, raw_x, raw_y):
        stick.sample([0, 0, 0, x, y, 0, 0, 0, raw_x, raw_y], ms * 10**6, 0, ms / 1000)
        return stick.current(ms / 1000)

    assert feed(1000, 0, 0, 0, .6) == (0, 0) and not stick.armed
    assert feed(1020, 0, .6, 0, .6) == (0, 0)
    assert feed(1040, 1, 0, 1, .01) == (0, 0) and stick.armed
    assert feed(1060, -.7, .8, -.7, .8) == (0, .8)
    assert feed(1061, 0, .8, .35, .8) == (0, .8)
    assert feed(1122, .7, .8, .7, .8) == (0, .8)
    assert feed(1300, 0, 0, .4, -.4) == (0, 0)
    stick.sample([0] * 9, 1320 * 10**6, 0, 1.32)
    assert stick.current(1.32) == (0, 0) and not stick.expires


def test_identity_and_calibration_before_writes_and_partial_acquire_cleanup():
    b = FakeBus()
    b.r[14, 11] = 4095
    with pytest.raises(ValueError):
        HeadServos(b, profile())
    assert not b.writes
    b.r[14, 11] = 1101
    h = HeadServos(b, profile())
    write = b.write
    def fail_after_enable(m, addr, n, val):
        write(m, addr, n, val)
        if addr == 40 and val == 1:
            raise RuntimeError("lost ack")
    b.write = fail_after_enable
    with pytest.raises(RuntimeError):
        h.step((1, 1), .05, 1, lambda: 0)
    h.close()
    assert b.r[13, 40] == 0 and b.r[13, 48] == 1000 and b.r[14, 40] == 0


@pytest.mark.parametrize('addr,value', [(56, 4002), (65, 32), (63, 50)])
def test_servo_fault_requires_healthy_feedback_then_new_neutral(addr, value):
    bus = FakeBus()
    servos = HeadServos(bus, profile())
    stick = HeadStick()
    log = Mock()

    def update(t, axis):
        stick.sample([0, 0, 0, axis, axis], round(t * 1e9), 0, t)
        return servos.update(stick, .05, log, lambda: t)

    update(.01, 0)
    update(.02, 1)
    assert servos.owned == {14}
    original = bus.r[13, addr]
    bus.r[13, addr] = value
    assert update(.03, 1) is None
    assert 'ID 13' in log.error.call_args.args[0]
    assert not servos.owned and all(bus.r[m, 40] == 0 for m in (13, 14))
    assert all(bus.r[m, 48] == 1000 for m in (13, 14))
    writes = len(bus.writes)
    bus.r[13, addr] = original
    update(.1, 0)
    bus.r[13, addr] = value
    assert update(.9, 0) is None
    bus.r[13, addr] = original
    for tick in range(10, 21):
        assert update(tick / 10, 1) == [2027, 958]
    update(2.01, 1)
    assert not servos.faulted and not stick.armed and len(bus.writes) == writes
    update(2.02, 0)
    update(2.03, 1)
    assert servos.owned == {14}


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_shutdown_finishes_serial_transaction_then_releases_owned_torque(
        monkeypatch, tmp_path, stop_signal):
    bus = FakeBus()
    mapping = dict(profile(), reference_ticks=[2009, 425], encoder_directions=[-1, 1],
                   encoder_scale_rad_per_tick=.001534)
    (tmp_path / 'head_preview_joints.json').write_text(json.dumps(mapping))
    (tmp_path / 'head_board_view.json').write_text(json.dumps(
        dict(motor_ids={'pan': 13, 'tilt': 14}, serial_port='mock', baudrate=1000000)))
    monkeypatch.setitem(sys.modules, 'cleany_robot_interface.arm_torque',
                        SimpleNamespace(connect=lambda *_: nullcontext(bus)))
    node = Mock()
    node.declare_parameter.return_value.value = str(tmp_path)
    node.get_clock.return_value.now.return_value.to_msg.return_value = gamepad_head.JointState().header.stamp
    monkeypatch.setattr(gamepad_head, 'Node', lambda _: node)
    ros = Mock()
    monkeypatch.setattr(gamepad_head, 'rclpy', ros)
    handler = signal.getsignal(stop_signal)
    finished = []

    def step_with_signal(servos, *args):
        servos.acquire(13)
        os.kill(os.getpid(), stop_signal)
        finished.append(True)
        return [2027, 958]

    monkeypatch.setattr(HeadServos, 'update', step_with_signal)
    ros.spin_once.side_effect = lambda *a, **kw: node.create_timer.call_args.args[1]()
    gamepad_head.main()
    assert finished == [True]
    assert bus.r[13, 40] == 0 and bus.r[13, 48] == 1000
    assert signal.getsignal(stop_signal) == handler
