"""Hold ordering, partial failures and ownership; no real motor access."""
from contextlib import contextmanager
import os
from pathlib import Path
import pty
import subprocess
from unittest.mock import Mock

import pytest
import yaml
import cleany_robot_interface.arm_torque as module

ROOT = Path(__file__).resolve().parents[4]
PROFILE = yaml.safe_load((ROOT / 'configs/robot/feetech_operational_left_arm.yaml').read_text())
RIGHT_PROFILE = yaml.safe_load((ROOT / 'configs/robot/feetech_operational_right_arm.yaml').read_text())


class Bus:
    def __init__(self, side='left'):
        self.events = []
        self.fail_write = self.fail_read = None
        self.values = {m: {3: 2825 if (m - 1) % 6 in (1, 2) else 777,
            9: 0, 11: 4095, 13: 80, 16: 1000, 33: 0, 40: 0, 42: 100,
            56: 2000 + m, 63: 25, 65: 0} for m in module.arm_selection(side)[0]}

    def read(self, motor, address, size):
        self.events.append(('read', motor, address))
        if self.fail_read == (motor, address):
            raise RuntimeError('READ failed')
        return self.values[motor].get(address, 0)

    def write(self, motor, address, size, value):
        self.events.append(('write', motor, address, value))
        if self.fail_write == (motor, address):
            raise RuntimeError('WRITE failed')
        self.values[motor][address] = value


def test_right_preloads_only_ids_7_to_11_and_repeat_is_noop():
    bus = Bus('right')
    module.enable(bus, RIGHT_PROFILE, side='right')
    first_on = next(i for i, e in enumerate(bus.events) if e[0] == 'write' and e[2] == 40)
    assert {e[1] for e in bus.events[:first_on] if e[0] == 'write' and e[2] == 42} == set(range(7, 12))
    assert {e[1] for e in bus.events} == set(range(7, 12))
    assert all(r[40] == 1 and r[42] == r[56] and r[48] == 800 for r in bus.values.values())
    assert all(e[2] >= 40 for e in bus.events if e[0] == 'write')
    bus.events.clear()
    module.enable(bus, RIGHT_PROFILE, side='right')
    assert not any(e[0] == 'write' for e in bus.events)


def test_right_uses_right_joint_overrides():
    bus = Bus('right')
    profile = {**RIGHT_PROFILE, 'joint_overrides': {'right_wrist_pitch_joint': {'torque_limit_raw': 650}}}
    module.enable(bus, profile, side='right')
    assert bus.values[10][48] == 650
    assert bus.values[9][48] == 800


@pytest.mark.parametrize('register,value', [(33, 1), (65, 1), (63, 80), (56, 4096), (3, 123)])
def test_right_late_preflight_failure_never_writes(register, value):
    bus = Bus('right')
    bus.values[11][register] = value
    with pytest.raises(RuntimeError):
        module.enable(bus, RIGHT_PROFILE, side='right')
    assert not any(e[0] == 'write' for e in bus.events)


def test_right_off_continues_after_partial_failure():
    bus = Bus('right')
    for row in bus.values.values():
        row[40] = 1
    bus.fail_write, bus.fail_read = (8, 40), (9, 40)
    assert len(module.disable(bus, side='right')) == 2
    assert [e[1] for e in bus.events if e[0] == 'write'] == list(range(7, 12))
    assert all(e[2] == 40 for e in bus.events)
    assert bus.values[11][40] == 0


@pytest.mark.parametrize('operation', ['on', 'off'])
def test_right_main_dispatch_and_profile(monkeypatch, capsys, operation):
    bus = Bus('right')
    @contextmanager
    def fake_connect(*args):
        yield bus
    monkeypatch.setattr(module, 'connect', fake_connect)
    monkeypatch.setattr('sys.argv', ['right-arm-torque-' + operation])
    enabled = Mock(wraps=module.enable)
    monkeypatch.setattr(module, 'enable', enabled)
    assert module.main(operation, ROOT, side='right') == 0
    assert 'RIGHT ARM' in capsys.readouterr().out
    assert {e[1] for e in bus.events} == set(range(7, 12))
    if operation == 'on':
        assert enabled.call_args.args[1] == RIGHT_PROFILE
        assert enabled.call_args.kwargs == {'side': 'right'}


def test_right_partial_on_preserves_hold_and_reports_correct_ids(monkeypatch, capsys):
    bus = Bus('right')
    bus.fail_write = (9, 40)
    @contextmanager
    def fake_connect(*args):
        yield bus
    monkeypatch.setattr(module, 'connect', fake_connect)
    monkeypatch.setattr('sys.argv', ['right-arm-torque-on'])
    assert module.main('on', ROOT, side='right') == 1
    assert [bus.values[m][40] for m in range(7, 12)] == [1, 1, 0, 0, 0]
    assert not any(e[0] == 'write' and e[2:] == (40, 0) for e in bus.events)
    output = capsys.readouterr().out
    assert 'ID 7: torque=1' in output and 'ID 11:' in output and 'ID 1:' not in output


def test_invalid_arm_rejected_before_access():
    bus = Bus()
    with pytest.raises(ValueError, match='Select'):
        module.enable(bus, PROFILE, side='invalid')
    assert bus.events == []

def test_preload_all_goals_before_on_and_repeat_is_noop():
    bus = Bus()
    module.enable(bus, PROFILE)
    first_on = next(i for i, e in enumerate(bus.events) if e[0] == 'write' and e[2] == 40)
    assert {e[1] for e in bus.events[:first_on] if e[0] == 'write' and e[2] == 42} == set(module.IDS)
    assert {e[1] for e in bus.events} == set(module.IDS)
    assert all(r[40] == 1 and r[42] == r[56] for r in bus.values.values())
    assert all(r[48] == 800 and r[46] == 800 and r[41] == 254 for r in bus.values.values())
    assert all(e[2] >= 40 for e in bus.events if e[0] == 'write')
    bus.events.clear()
    module.enable(bus, PROFILE)
    assert not any(e[0] == 'write' for e in bus.events)


@pytest.mark.parametrize('register,value', [(33, 1), (65, 1), (63, 80), (56, 4096), (3, 123), (16, 500)])
def test_late_preflight_failure_never_writes(register, value):
    bus = Bus()
    bus.values[5][register] = value
    with pytest.raises(RuntimeError):
        module.enable(bus, PROFILE)
    assert not any(e[0] == 'write' for e in bus.events)


def test_distant_enabled_goal_rejected():
    bus = Bus()
    bus.values[2][40] = 1
    with pytest.raises(RuntimeError, match='distant'):
        module.enable(bus, PROFILE)
    assert not any(e[0] == 'write' for e in bus.events)


def test_mixed_state_preserves_already_enabled_joint():
    bus = Bus()
    bus.values[2].update({40: 1, 42: bus.values[2][56]})
    module.enable(bus, PROFILE)
    assert not any(e[0] == 'write' and e[1] == 2 for e in bus.events)


@pytest.mark.parametrize('drift', [False, True])
def test_goal_verification_or_drift_prevents_activation(drift):
    bus = Bus()
    original = bus.write
    def write(motor, address, size, value):
        if not drift and address == 42:
            return
        original(motor, address, size, value)
        if drift and motor == 5 and address == 42:
            bus.values[1][56] += 4
    bus.write = write
    with pytest.raises(RuntimeError):
        module.enable(bus, PROFILE)
    assert all(r[40] == 0 for r in bus.values.values())


def test_partial_on_reports_state_without_automatic_off(monkeypatch, capsys):
    bus = Bus()
    bus.fail_write = (3, 40)
    @contextmanager
    def fake_connect(*args):
        yield bus
    monkeypatch.setattr(module, 'connect', fake_connect)
    monkeypatch.setattr('sys.argv', ['left-arm-torque-on'])
    assert module.main('on', ROOT) == 1
    assert [bus.values[m][40] for m in module.IDS] == [1, 1, 0, 0, 0]
    assert not any(e[0] == 'write' and e[2:] == (40, 0) for e in bus.events)
    output = capsys.readouterr().out
    assert 'ON incomplete' in output and 'ID 1: torque=1' in output


def test_off_continues_after_failures_and_only_touches_torque():
    bus = Bus()
    for row in bus.values.values():
        row[40] = 1
    bus.fail_write, bus.fail_read = (2, 40), (3, 40)
    assert len(module.disable(bus)) == 2
    assert [e[1] for e in bus.events if e[0] == 'write'] == list(module.IDS)
    assert all(e[2] == 40 for e in bus.events)
    assert bus.values[5][40] == 0


def test_off_readback_mismatch_is_failure():
    bus = Bus()
    bus.values[1][40] = 1
    bus.write = lambda *args: None
    assert 'not confirmed' in module.disable(bus)[0]


def test_pty_ownership_detects_other_owner():
    master, slave = pty.openpty()
    try:
        device = os.ttyname(slave)
        module.check_port_unused(device)
        child = subprocess.Popen(['/usr/bin/python3', '-c',
            'import os,sys; fd=os.open(sys.argv[1],os.O_RDWR); print("ready",flush=True); sys.stdin.read()', device],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        try:
            assert child.stdout.readline().strip() == 'ready'
            with pytest.raises(RuntimeError, match='in use'):
                module.check_port_unused(device)
        finally:
            child.communicate(timeout=5)
    finally:
        os.close(master)
        os.close(slave)


def test_sdk_connection_excludes_new_open_and_cleans_up():
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    try:
        with pytest.raises(RuntimeError, match='test failure'):
            with module.connect(device, 1000000):
                with pytest.raises(OSError):
                    fd = os.open(device, os.O_RDWR)
                    os.close(fd)
                raise RuntimeError('test failure')
        reopened = os.open(device, os.O_RDWR)
        os.close(reopened)
    finally:
        os.close(master)
        os.close(slave)


def test_busy_port_never_opens_sdk(monkeypatch):
    import scservo_sdk
    opened = Mock()
    monkeypatch.setattr(scservo_sdk, 'PortHandler', opened)
    monkeypatch.setattr(module, 'check_port_unused', Mock(side_effect=RuntimeError('busy')))
    with pytest.raises(RuntimeError, match='busy'):
        with module.connect('/unused', 1000000):
            pass
    opened.assert_not_called()
