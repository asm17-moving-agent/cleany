from unittest.mock import Mock

import pytest

from cleany_robot_interface.reader import ReadOnlyBus


def test_bus_only_calls_read_instructions():
    bus = ReadOnlyBus.__new__(ReadOnlyBus)
    bus.port = object()
    bus.packet = Mock(spec=['read1ByteTxRx', 'read2ByteTxRx'])
    bus.packet.read1ByteTxRx.return_value = (0, 0, 0)
    bus.packet.read2ByteTxRx.return_value = (2048, 0, 0)
    assert bus.read(1, 40, 1) == 0
    assert bus.read(12, 56, 2) == 2048
    assert len(bus.packet.mock_calls) == 2


@pytest.mark.parametrize('result', [(0, -1, 0), (2048, 0, 1)])
def test_bus_rejects_communication_and_motor_errors(result):
    bus = ReadOnlyBus.__new__(ReadOnlyBus)
    bus.port = object()
    bus.packet = Mock(spec=['read2ByteTxRx'])
    bus.packet.read2ByteTxRx.return_value = result
    with pytest.raises(RuntimeError):
        bus.read(1, 56, 2)
