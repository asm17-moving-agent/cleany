from concurrent.futures import Future
from types import SimpleNamespace
import pytest
from cleany_teleop.driver_transport import Geometry, DriverTransport, body_velocity
from cleany_teleop.web_control import Command, Control


def test_inverse_kinematics_preserves_wheel_order():
    g = Geometry(0.0635, 0.350, 0.590)
    assert body_velocity((1, 1, 1, 1), g) == pytest.approx((0.0635, 0, 0))
    assert body_velocity((-1, 1, 1, -1), g) == pytest.approx((0, 0.0635, 0))
    assert body_velocity((-1, 1, -1, 1), g) == pytest.approx((0, 0, 0.0635 / 0.47))


def test_permission_reply_is_not_an_mcu_ack_and_rpc_is_not_repeated():
    calls = []
    future = Future()
    transport = DriverTransport(
        lambda *v: (calls.append(v), future)[1], lambda _: None, Geometry(0.1, 0.2, 0.3)
    )
    command = Command(Control.ENABLE, 45)
    transport.publish(command)
    transport.publish(command)
    assert len(calls) == 1
    state = SimpleNamespace(boot_id=4, last_command_sequence=10)
    assert not transport.acknowledged(command, state)
    future.set_result(SimpleNamespace(accepted=True, boot_id=4, command_sequence=11))
    assert not transport.acknowledged(command, state)
    state.last_command_sequence = 11
    assert transport.acknowledged(command, state)
    state.boot_id = 5
    assert not transport.acknowledged(command, state)


def test_stop_supersedes_late_enable_reply():
    futures = []
    calls = []

    def request(*values):
        calls.append(values)
        future = Future()
        futures.append(future)
        return future

    transport = DriverTransport(request, lambda _: None, Geometry(0.1, 0.2, 0.3))
    enable = Command(Control.ENABLE, 1)
    stop = Command(Control.STOP, 2)
    transport.publish(enable)
    transport.publish(stop)
    futures[0].set_result(SimpleNamespace(accepted=True, boot_id=1, command_sequence=1))
    state = SimpleNamespace(boot_id=1, last_command_sequence=1)
    assert not transport.acknowledged(stop, state)
    assert calls[1][1] > calls[0][1] and not calls[1][2]
    futures[1].set_result(SimpleNamespace(accepted=True, boot_id=1, command_sequence=2))
    state.last_command_sequence = 2
    assert transport.acknowledged(stop, state)
