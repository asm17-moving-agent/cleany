"""Verify whole-robot torque checks and release without opening real hardware."""
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest
import yaml


loader = SourceFileLoader('feetech_torque', str((Path(__file__).resolve().parents[2] / 'tools').joinpath('feetech-torque')))
module = module_from_spec(spec_from_loader(loader.name, loader))
sys.modules[loader.name] = module
loader.exec_module(module)


@pytest.fixture
def transport(monkeypatch):
    port = Mock()
    port.setBaudRate.return_value = True
    packet = Mock()
    packet.read1ByteTxRx.return_value = (0, 0, 0)
    packet.write1ByteTxRx.return_value = (0, 0)
    opened = Mock(return_value=port)
    monkeypatch.setattr(module, 'PortHandler', opened)
    monkeypatch.setattr(module, 'PacketHandler', Mock(return_value=packet))
    return opened, port, packet


@pytest.mark.parametrize('operation', ['check-off', 'off'])
def test_all_fourteen_motors_share_one_serial_owner(monkeypatch, transport, operation):
    opened, port, packet = transport
    monkeypatch.setattr(sys, 'argv', ['feetech-torque', operation])
    assert module.main() == 0
    assert opened.call_count == 1
    arm_config = Path(__file__).resolve().parents[2] / 'ros2_ws/src/cleany_robot_interface/config/arm_preview.yaml'
    assert opened.call_args.args == (yaml.safe_load(arm_config.read_text())['port'],)
    assert [call.args for call in packet.read1ByteTxRx.call_args_list] == [
        (port, motor, 40) for motor in range(1, 15)]
    if operation == 'off':
        assert [call.args for call in packet.write1ByteTxRx.call_args_list] == [
            (port, motor, 40, 0) for motor in range(1, 15)]
    else:
        packet.write1ByteTxRx.assert_not_called()
    port.closePort.assert_called_once()


@pytest.mark.parametrize('motor', [13, 14])
def test_head_failure_prevents_whole_robot_off_confirmation(monkeypatch, transport, motor):
    _, port, packet = transport
    packet.read1ByteTxRx.side_effect = lambda _, observed, address: (
        (0, -1, 0) if observed == motor else (0, 0, 0))
    monkeypatch.setattr(sys, 'argv', ['feetech-torque', 'off'])
    assert module.main() == 1
    assert len(packet.write1ByteTxRx.call_args_list) == 14
    port.closePort.assert_called_once()
