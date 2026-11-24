from concurrent.futures import Future

from cleany_teleop.gamepad import GamepadControl, Mapping
from cleany_teleop.gamepad_relay import GamepadRelay


class PendingIO:
    def __init__(self):
        self.sent = []

    def submit(self, exchange, request):
        reply = Future()
        self.sent.append((request, reply))
        return reply


def feed(relay, t, forward, left=0):
    axes = [left, forward, 0, 0, 0, 0, left, forward, 0, 0]
    relay.sample(axes, [0] * 11, t, 0, t)


def acquired():
    profile = {"geometry": {"wheel_radius_m": .0635},
               "limits": {"linear_x_mps": .5, "linear_y_mps": .5, "wheel_rad_s": 10}}
    relay = GamepadRelay(GamepadControl(Mapping(), profile), "", lambda message: None)
    relay.workers.shutdown()
    relay.workers = io = PendingIO()
    feed(relay, 1, 0)
    feed(relay, 1.02, 1)
    feed(relay, 1.04, 1)
    assert [r.action for r, _ in io.sent] == ["press"]
    io.sent[0][1].set_result(False)
    feed(relay, 1.06, 1)
    return relay, io.sent


def test_late_update_does_not_block_new_direction_or_reset_ownership():
    relay, sent = acquired()
    for t in (1.08, 1.10, 1.14):
        feed(relay, t, .65)
    assert [r.action for r, _ in sent] == ["press", "update", "update"]
    assert sent[-1][0].vector == [.65, 0, 0]
    sent[2][1].set_result(False)
    feed(relay, 1.17, .4)
    sent[1][1].set_exception(TimeoutError("discarded"))
    for t in (1.20, 1.22):
        feed(relay, t, .4)
    assert sent[-1][0].vector == [.4, 0, 0]
    assert relay.control.active and relay.control.startup_neutral_seen and relay.last_ack == 3
    assert all(r.action != "stop" for r, _ in sent)


def test_release_overtakes_two_pending_updates_and_old_reply_cannot_restart():
    relay, sent = acquired()
    feed(relay, 1.14, .9)
    feed(relay, 1.15, 0)
    req, release = sent[-1]
    assert req.action == "stop" and req.sequence == 4
    feed(relay, 1.16, .7)
    assert len(sent) == 4
    release.set_result(False)
    sent[1][1].set_result(True)
    sent[2][1].set_exception(ValueError("old denial"))
    feed(relay, 1.18, .6)
    assert sent[-1][0].action == "press" and sent[-1][0].vector == [0, 0, 0]
    assert not relay.control.forward_only


def test_lost_owner_or_lost_reports_stop_then_reacquire_from_fresh_deflection():
    for cause in ("denied", "disconnect", "silent", "no_ack"):
        relay, sent = acquired()
        if cause == "denied":
            sent[1][1].set_exception(ValueError("lost owner"))
            feed(relay, 1.08, 1)
        elif cause == "disconnect":
            relay.sample([], [], 1.08, 0, 1.08)
        elif cause == "silent":
            relay.tick(1.22)
        else:
            for t in (1.12, 1.14, 1.19, 1.24, 1.28):
                feed(relay, t, 1)
        assert sent[-1][0].action == "stop" and not relay.control.active
        for _, f in sent:
            if not f.done():
                f.set_result(False)
        relay.tick(1.30)
        assert sent[-1][0].action == "stop"
        feed(relay, 1.32, -.6)
        assert sent[-1][0].action == "press" and sent[-1][0].vector == [0, 0, 0]
        feed(relay, 1.34, -.6)
        assert sent[-1][0].action == "press"
        sent[-1][1].set_result(False)
        feed(relay, 1.36, -.6)
        assert sent[-1][0].action == "update" and sent[-1][0].vector == [-.6, 0, 0]


def test_no_nonzero_command_before_forward_only_admission():
    relay, sent = acquired()
    sent[-1][1].set_result(True)
    feed(relay, 1.15, .7, .4)
    assert sent[-1][0].vector == [.7, 0, 0]


def test_busy_controller_retries_only_on_fresh_input_after_backoff():
    relay, sent = acquired()
    sent[-1][1].set_exception(ValueError("lost owner"))
    feed(relay, 1.08, 1)
    sent[-1][1].set_result(False)
    feed(relay, 1.10, 1)
    assert sent[-1][0].action == "press"
    sent[-1][1].set_exception(ValueError("busy"))
    feed(relay, 1.12, 1)
    assert sent[-1][0].action == "stop"
    sent[-1][1].set_result(False)
    for t in (1.14, 1.24, 1.34):
        feed(relay, t, 1)
    assert sent[-1][0].action == "stop"
    relay.tick(1.38)
    assert sent[-1][0].action == "stop"
    feed(relay, 1.40, .5)
    assert sent[-1][0].action == "press"
