from cleany_control_bridge.observation_relay import ObservationRelay


def value(seq, kind="joints", boot="one"):
    return dict(boot_id=boot, sequence=seq, source_age_seconds=0, sample=dict(kind=kind))


def test_latest_only_order_age_ack_and_restart():
    now = [10.0]
    relay = ObservationRelay("ws://localhost/test", "unused", clock=lambda: now[0])
    relay.update(value(1))
    relay.update(value(2, "battery"))
    relay.update(value(3))
    key, frame = relay.next_frame()
    assert key == "battery" and frame["sequence"] == 2
    relay.acknowledge(key, frame)
    key, frame = relay.next_frame()
    assert frame["sequence"] == 3
    relay.update(value(4))
    relay.acknowledge(key, frame)
    assert relay.next_frame()[1]["sequence"] == 4
    now[0] = 14
    assert relay.next_frame() is None
    relay.update(value(1, boot="two"))
    assert relay.next_frame()[1]["boot_id"] == "two"


def test_event_retries_are_same_id_and_bounded():
    relay = ObservationRelay("ws://localhost/test", "unused", event_capacity=2)
    for seq in range(3):
        relay.update(value(seq, "supervisor_event"))
    assert relay.dropped_events == 1
    first = relay.next_frame()
    again = relay.next_frame()
    assert first[1]["sequence"] == again[1]["sequence"] == 1
    relay.acknowledge(*first)
    assert relay.next_frame()[1]["sequence"] == 2
