import os
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

import rclpy
from cleany_base_interfaces.msg import WheelCommand
from cleany_base_driver.mock_node import MockMcu


def _command(mock, mode, sequence, *, boot=None, session=None, deadline=None, values=None):
    msg = WheelCommand()
    msg.protocol_version = 1
    msg.boot_id = mock.boot if boot is None else boot
    msg.session_id = mock.session if session is None else session
    msg.sequence = sequence
    msg.mode = mode
    msg.velocity_rad_s = [0.] * 4 if values is None else values
    msg.valid_until_us = int(time.monotonic() * 1e6) + 100000 if deadline is None else deadline
    mock.command(msg)
    return msg


def test_mock_gate_validates_stops_retires_and_watchdogs_without_feedback():
    rclpy.init()
    mock = MockMcu()
    try:
        mock.last_command = time.monotonic() - 1
        mock.publish()
        assert not mock.faults  # an idle, disarmed MCU is not watchdog-faulted

        _command(mock, WheelCommand.BEGIN_SESSION, 0, boot=0, session=12)
        assert mock.session == 0
        expired = int(time.monotonic() * 1e6)
        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=12, deadline=expired)
        assert mock.session == 0

        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=12)
        assert mock.session == 12 and not mock.armed
        _command(mock, WheelCommand.ARM, 1, session=12, boot=mock.boot + 1)
        assert not mock.armed
        _command(mock, WheelCommand.ARM, 1, session=12)
        assert mock.armed
        accepted_at = mock.last_command
        _command(mock, WheelCommand.VELOCITY, 1, session=12,
                 values=[1., 1., 1., 1.])
        assert mock.last_command == accepted_at  # duplicate sequence cannot refresh
        _command(mock, WheelCommand.VELOCITY, 2, session=12,
                 deadline=int(time.monotonic() * 1e6) + 300000,
                 values=[1., 1., 1., 1.])
        assert mock.armed and mock.last_command == accepted_at
        _command(mock, WheelCommand.STOP, 0)
        _command(mock, WheelCommand.BEGIN_SESSION, 3, session=12)
        assert not mock.armed and mock.session == 0  # A live-deadline replay is rejected.
        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=14)
        _command(mock, WheelCommand.ARM, 1, session=14)
        assert mock.armed
        _command(mock, WheelCommand.STOP, 99, boot=0, session=0)
        assert mock.session == 0 and not mock.armed
        _command(mock, WheelCommand.BEGIN_SESSION, 4, session=12)
        assert mock.session == 0  # Multiple recent retired sessions are protected.
        _command(mock, WheelCommand.BEGIN_SESSION, 4, session=14)
        assert mock.session == 0

        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=13)
        _command(mock, WheelCommand.ARM, 1, session=13)
        mock.connected = False
        mock.last_command = time.monotonic() - .3
        mock.publish()
        assert not mock.armed and mock.session == 0 and mock.faults & 1
    finally:
        mock.destroy_node()
        rclpy.shutdown()


def test_mock_deadline_expiry_and_retirement_capacity(monkeypatch):
    import cleany_base_driver.mock_node as module
    rclpy.init()
    mock = MockMcu()
    now = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    try:
        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=1)
        _command(mock, WheelCommand.ARM, 1, session=1)
        _command(mock, WheelCommand.BEGIN_SESSION, 2, session=1)
        assert mock.armed  # Even a newer BEGIN cannot reset its current session.
        _command(mock, WheelCommand.VELOCITY, 2, session=1, deadline=1050000,
                 values=[1., 2., 3., 4.])
        now[0] = 1.05
        mock.publish()
        assert not mock.armed and mock.faults & 1
        for session in range(2, 10):
            _command(mock, WheelCommand.BEGIN_SESSION, 0, session=session)
        _command(mock, WheelCommand.BEGIN_SESSION, 0, session=10)
        assert mock.session == 9
        _command(mock, WheelCommand.STOP, 0)
        _command(mock, WheelCommand.BEGIN_SESSION, 1, session=10)
        assert mock.session == 0
        now[0] = 1.30
        _command(mock, WheelCommand.BEGIN_SESSION, 2, session=10)
        assert mock.session == 10 and not mock.armed
    finally:
        mock.destroy_node()
        rclpy.shutdown()
