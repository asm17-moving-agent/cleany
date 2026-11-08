from concurrent.futures import Future
from types import SimpleNamespace
import pytest
from cleany_teleop.driver_transport import Geometry, DriverTransport, body_velocity, limit_wheel_velocity
from cleany_teleop.web_control import Command, Control


def test_inverse_kinematics_preserves_wheel_order():
    g = Geometry(0.0635, 0.350, 0.590)
    assert body_velocity((1, 1, 1, 1), g) == pytest.approx((0.0635, 0, 0))
    assert body_velocity((-1, 1, 1, -1), g) == pytest.approx((0, 0.0635, 0))
    assert body_velocity((-1, 1, -1, 1), g) == pytest.approx((0, 0, 0.0635 / 0.47))


@pytest.mark.parametrize('axes', [(.2, 0., 0.), (0., -.2, 0.),
                                  (0., 0., -.5), (.127, .127, .27)])
def test_combined_navigation_twist_respects_each_wheel_limit(axes):
    g = Geometry(.0635, .35, .59)
    limited = limit_wheel_velocity(axes, g, 2.)
    x, y, yaw = limited
    turn = (g.wheelbase + g.separation) * yaw / 2
    wheels = [(x-y-turn)/g.radius, (x+y+turn)/g.radius,
              (x+y-turn)/g.radius, (x-y+turn)/g.radius]
    assert max(abs(v) for v in wheels) == pytest.approx(2.)
    assert max(abs(v * .25) for v in wheels) == pytest.approx(.5)
    ratios = [new / old for new, old in zip(limited, axes) if old]
    assert ratios == pytest.approx([ratios[0]] * len(ratios))


@pytest.mark.parametrize('axes', [(0., 0., 0.), (.01, 0., .01)])
def test_wheel_limit_never_increases_small_or_stopped_commands(axes):
    assert limit_wheel_velocity(axes, Geometry(.0635, .35, .59), 2.) == axes


@pytest.mark.parametrize('limit', [0., -1., float('nan'), float('inf')])
def test_invalid_navigation_wheel_limit_is_rejected(limit):
    with pytest.raises(ValueError):
        limit_wheel_velocity((.1, 0., 0.), Geometry(.0635, .35, .59), limit)


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


def test_startup_stop_waits_for_a_real_boot_identity_and_new_ack():
    futures, calls = [], []
    def request(*values):
        calls.append(values)
        future = Future(); futures.append(future)
        return future
    transport = DriverTransport(request, lambda _: None, Geometry(.1, .2, .3))
    stop = Command(Control.STOP, 1)
    transport.publish(stop)
    futures[0].set_result(SimpleNamespace(accepted=True, boot_id=0, command_sequence=1))
    state = SimpleNamespace(boot_id=42, last_command_sequence=80)
    assert not transport.acknowledged(stop, state)
    assert len(calls) == 2 and not any(call[2] for call in calls)
    futures[1].set_result(SimpleNamespace(accepted=True, boot_id=42, command_sequence=81))
    assert not transport.acknowledged(stop, state)
    state.last_command_sequence = 81
    assert transport.acknowledged(stop, state)


@pytest.mark.parametrize(
    "kind", ["rejected", "future_exception", "request_exception", "empty"]
)
def test_permission_failures_keep_reason_and_are_reported_once(kind):
    from cleany_teleop.driver_transport import DriverControl

    future = Future()
    expected = "stale permission sequence"
    if kind == "rejected":
        future.set_result(SimpleNamespace(accepted=False, reason=expected))
    elif kind == "empty":
        future.set_result(None)
        expected = "Base permission service returned no response"
    else:
        future.set_exception(ConnectionError(expected))

    def request(*args):
        if not args[-1]:
            return Future()
        if kind == "request_exception":
            raise ConnectionError(expected)
        return future

    transport = DriverTransport(request, lambda _: None, Geometry(0.1, 0.2, 0.3))
    control = DriverControl(transport)
    control.pending = Command(Control.ENABLE, 1)
    control.state = SimpleNamespace(boot_id=1, last_command_sequence=1)
    transport.publish(control.pending)
    for _ in range(50):
        assert not control.ack()
    failures = [
        event for event in control.events if event["kind"] == "permission_failure"
    ]
    assert len(failures) == 1 and failures[0]["reason"] == expected
    control.state = None
    control.tick()
    assert transport.last_permission_error["reason"] == expected
    previous = dict(transport.last_permission_error)
    transport.publish(Command(Control.STOP, 2))
    assert transport.last_permission_error == previous
    assert previous["client"] == transport.client and previous["request_sequence"] == 1
    if "exception" in kind:
        assert previous["exception_type"] == "ConnectionError"
        assert (
            "ConnectionError" in previous["traceback"]
            and expected in previous["traceback"]
        )
