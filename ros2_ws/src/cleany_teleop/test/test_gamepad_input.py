from types import SimpleNamespace

from evdev import InputEvent, ecodes as ec
import pytest

from cleany_teleop.gamepad import StickSnapbackFilter
from cleany_teleop.gamepad_input import AXES, STICK_CENTER_BAND, GamepadInput, receiver_path


def test_auto_selects_only_a_unique_xpad_event(tmp_path):
    def named_event(event, name):
        device = tmp_path / event / "device"
        device.mkdir(parents=True)
        (device / "name").write_text(name + "\n")

    named_event("event8", "LiteEMF Controller Keyboard")
    with pytest.raises(FileNotFoundError):
        receiver_path("auto", tmp_path)
    named_event("event7", "LeadJoy Xeno Plus")
    assert receiver_path("auto", tmp_path) == "/dev/input/event7"
    named_event("event15", "LeadJoy Xeno Plus")
    with pytest.raises(ValueError):
        receiver_path("auto", tmp_path)
    assert receiver_path("/dev/input/by-path/pad", tmp_path) == "/dev/input/by-path/pad"


@pytest.fixture
def reader():
    reports = []
    events = []
    node = SimpleNamespace(
        device=SimpleNamespace(read=lambda: events),
        axes=dict.fromkeys(AXES, 0),
        keys=set(),
        report=False,
        last_buttons=None,
        last_publish=0,
        last_report=None,
        left_stick=StickSnapbackFilter(STICK_CENTER_BAND),
        right_stick=StickSnapbackFilter(STICK_CENTER_BAND),
        publish=lambda stamp, axes, buttons: reports.append((stamp, axes, buttons)),
    )

    def frame(micros, changes, marker=True):
        data = list(changes)
        if marker:
            data.append((ec.EV_MSC, ec.MSC_TIMESTAMP, micros))
        data.append((ec.EV_SYN, ec.SYN_REPORT, 0))
        events[:] = [InputEvent(1, micros, typ, code, val) for typ, code, val in data]
        GamepadInput.poll(node)

    return frame, reports


def test_report_marker_preserves_held_input_and_button_edges(reader):
    frame, reports = reader
    frame(0, [], marker=False)
    assert not reports
    frame(1000, [(ec.EV_ABS, ec.ABS_Y, -32768), (ec.EV_KEY, ec.BTN_TL, 1)])
    frame(2000, [])
    frame(22000, [])
    frame(23000, [(ec.EV_KEY, ec.BTN_TL, 0)])
    assert len(reports) == 3
    assert reports[0][1][1] == 1.0
    assert [r[2][4] for r in reports] == [1, 1, 0]
    assert reports[-1][0] == 1023000000


@pytest.mark.parametrize("phase_ms", range(20))
def test_reader_stops_both_sticks_through_recorded_release_at_any_publish_phase(reader, phase_ms):
    frame, reports = reader
    # Replay the measured Jetson right-stick release on both sticks.
    changes = {40: (-.9688, -.3047), 97: (-.0898, -.0586), 98: (0, 0),
               100: (.2305, .0899), 103: (.625, .2969), 111: (.0195, .0235),
               116: (-.3672, -.1601), 122: (.1094, .0547), 124: (.2812, .1289),
               129: (0, 0), 134: (-.1836, -.0234), 138: (.1523, .0352), 142: (0, 0)}
    for ms in range(phase_ms, 242):
        events = []
        if ms in changes or ms == 219:
            x, y = changes.get(ms, (.75, -.30))
            events = [(ec.EV_ABS, axis, round(-v * (32768 if v > 0 else 32767)))
                      for axis, v in ((ec.ABS_X, x), (ec.ABS_Y, y),
                                      (ec.ABS_RX, x), (ec.ABS_RY, y))]
        if ms in (103, 104):
            events.append((ec.EV_KEY, ec.BTN_TR, int(ms == 103)))
        frame(ms * 1000, events)
    by_ms = {(stamp - 10**9) // 10**6: (a, b) for stamp, a, b in reports}
    assert any(a[0] < -.08 and a[3] < -.08 for ms, (a, _) in by_ms.items() if ms < 98)
    assert 98 in by_ms  # Stop on the USB report that first observes neutral.
    assert all((a[0], a[1], a[3], a[4]) == (0, 0, 0, 0)
               for ms, (a, _) in by_ms.items() if 98 <= ms < 219)
    peak, buttons = by_ms[103]
    assert peak[6:] == pytest.approx([.625, .2969, .625, .2969], abs=.00002)
    assert buttons[5] and not by_ms[104][1][5]
    assert reports[-1][1][:2] == pytest.approx([.75, -.3], abs=.00002)


def test_reader_one_axis_reverses_while_orthogonal_axis_stays_held(reader):
    frame, reports = reader
    frame(0, [])
    frame(20000, [(ec.EV_ABS, ec.ABS_X, -26000), (ec.EV_ABS, ec.ABS_Y, -29400),
                  (ec.EV_ABS, ec.ABS_RX, -26000), (ec.EV_ABS, ec.ABS_RY, -29400)])
    frame(21000, [(ec.EV_ABS, ec.ABS_X, 11000), (ec.EV_ABS, ec.ABS_RX, 11000)])
    frame(41000, [])
    frame(62000, [])
    frame(83000, [])
    assert reports[2][0] == 1021000000
    for _, axes, _ in reports[2:-1]:
        assert axes[0] == axes[3] == 0 and axes[6] == axes[8] < -.33
        assert axes[1] == axes[4] == axes[7] == axes[9] > .89
    assert reports[-1][1][0] == reports[-1][1][3] < -.33


def test_diagonal_kick_on_previously_idle_axis_is_suppressed(reader):
    frame, reports = reader
    frame(0, [])
    frame(20000, [(ec.EV_ABS, ec.ABS_X, -26000)])
    frame(21000, [(ec.EV_ABS, ec.ABS_X, 11000), (ec.EV_ABS, ec.ABS_Y, 6000)])
    assert reports[-1][1][:2] == [0, 0]
    assert all(v < -.18 for v in reports[-1][1][6:8])


def test_center_drift_stays_zero_even_while_another_axis_is_held(reader):
    frame, reports = reader
    frame(0, [(ec.EV_ABS, ec.ABS_X, 1280), (ec.EV_ABS, ec.ABS_Y, -24000)])
    frame(20000, [(ec.EV_ABS, ec.ABS_X, -1024)])
    for _, axes, _ in reports:
        assert axes[0] == 0 and .03 < abs(axes[6]) < .05
        assert axes[1] == axes[7] > .7


def test_dropped_events_force_disconnect_before_next_frame():
    closed = []
    event = InputEvent(1, 0, ec.EV_SYN, ec.SYN_DROPPED, 0)
    node = SimpleNamespace(
        device=SimpleNamespace(read=lambda: [event]),
        disconnect=lambda: closed.append(True),
    )
    GamepadInput.poll(node)
    assert closed == [True]
