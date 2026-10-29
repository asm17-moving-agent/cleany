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
PROFILE = yaml.safe_load((ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_left_arm.yaml').read_text())
RIGHT_PROFILE = yaml.safe_load((ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_right_arm.yaml').read_text())
ARM_CASES = [('left', PROFILE, 1), ('right', RIGHT_PROFILE, 7)]


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


@pytest.mark.parametrize('side,profile,first_id', ARM_CASES)
def test_preload_all_goals_before_on_and_repeat_is_noop(side, profile, first_id):
    bus = Bus(side)
    ids = set(range(first_id, first_id + 5))
    module.enable(bus, profile, side=side)
    first_on = next(i for i, e in enumerate(bus.events) if e[0] == 'write' and e[2] == 40)
    assert {e[1] for e in bus.events[:first_on] if e[0] == 'write' and e[2] == 42} == ids
    assert {e[1] for e in bus.events} == ids
    assert all(r[40] == 1 and r[42] == r[56] for r in bus.values.values())
    assert all(r[48] == 800 and r[46] == 800 and r[41] == 254 for r in bus.values.values())
    assert all(e[2] >= 40 for e in bus.events if e[0] == 'write')
    bus.events.clear()
    module.enable(bus, profile, side=side)
    assert not any(e[0] == 'write' for e in bus.events)


def test_right_uses_right_joint_overrides():
    bus = Bus('right')
    profile = {**RIGHT_PROFILE, 'joint_overrides': {'right_wrist_pitch_joint': {'torque_limit_raw': 650}}}
    module.enable(bus, profile, side='right')
    assert bus.values[10][48] == 650
    assert bus.values[9][48] == 800


@pytest.mark.parametrize('side,profile,first_id', ARM_CASES)
@pytest.mark.parametrize('register,value', [(33, 1), (65, 1), (63, 80), (56, 4096), (3, 123), (16, 500)])
def test_late_preflight_failure_never_writes(side, profile, first_id, register, value):
    bus = Bus(side)
    bus.values[first_id + 4][register] = value
    with pytest.raises(RuntimeError):
        module.enable(bus, profile, side=side)
    assert not any(e[0] == 'write' for e in bus.events)


@pytest.mark.parametrize('side,first_id', [('left', 1), ('right', 7)])
def test_off_continues_after_failures_and_only_touches_torque(side, first_id):
    bus = Bus(side)
    for row in bus.values.values():
        row[40] = 1
    bus.fail_write, bus.fail_read = (first_id + 1, 40), (first_id + 2, 40)
    assert len(module.disable(bus, side=side)) == 2
    assert [e[1] for e in bus.events if e[0] == 'write'] == list(range(first_id, first_id + 5))
    assert all(e[2] == 40 for e in bus.events)
    assert bus.values[first_id + 4][40] == 0


@pytest.mark.parametrize('operation', ['on', 'off'])
def test_right_main_dispatch_and_profile(monkeypatch, capsys, operation):
    bus = Bus('right')
    @contextmanager
    def fake_connect(*args):
        yield bus
    monkeypatch.setattr(module, 'connect', fake_connect)
    monkeypatch.setattr('sys.argv', ['right-arm-torque-' + operation, *(['--control-config', str(ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_right_arm.yaml')] if operation == 'on' else [])])
    enabled = Mock(wraps=module.enable)
    monkeypatch.setattr(module, 'enable', enabled)
    assert module.main(operation, ROOT, side='right') == 0
    assert 'RIGHT ARM' in capsys.readouterr().out
    assert {e[1] for e in bus.events} == set(range(7, 12))
    if operation == 'on':
        assert enabled.call_args.args[1] == RIGHT_PROFILE
        assert enabled.call_args.kwargs == {'side': 'right'}


@pytest.mark.parametrize('side,first_id', [('left', 1), ('right', 7)])
def test_partial_on_preserves_hold_and_reports_correct_ids(monkeypatch, capsys, side, first_id):
    bus = Bus(side)
    bus.fail_write = (first_id + 2, 40)
    @contextmanager
    def fake_connect(*args):
        yield bus
    monkeypatch.setattr(module, 'connect', fake_connect)
    monkeypatch.setattr('sys.argv', [side + '-arm-torque-on', '--control-config',
        str(ROOT / f'ros2_ws/src/cleany_robot_interface/test/data/feetech_operational_{side}_arm.yaml')])
    assert module.main('on', ROOT, side=side) == 1
    assert [bus.values[m][40] for m in range(first_id, first_id + 5)] == [1, 1, 0, 0, 0]
    assert not any(e[0] == 'write' and e[2:] == (40, 0) for e in bus.events)
    output = capsys.readouterr().out
    assert 'ON incomplete' in output and f'ID {first_id}: torque=1' in output
    assert f'ID {first_id + 4}:' in output
    if side == 'right':
        assert 'ID 1:' not in output


def test_invalid_arm_rejected_before_access():
    bus = Bus()
    with pytest.raises(ValueError, match='Select'):
        module.enable(bus, PROFILE, side='invalid')
    assert bus.events == []


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


def test_torque_on_requires_local_limits_before_opening_a_bus(monkeypatch):
    monkeypatch.setattr('sys.argv', ['left-arm-torque-on'])
    monkeypatch.setattr(module, 'connect', lambda *_: pytest.fail('No profile may open a bus'))
    with pytest.raises(SystemExit) as stopped:
        module.main('on', ROOT)
    assert stopped.value.code == 2
