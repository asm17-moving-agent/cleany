import math

import pytest

from cleany_telemetry.robot_observation import ObservationBuffer, battery_sample


def test_unknown_values_are_null_and_provenance_is_explicit():
    assert battery_sample(math.nan, math.nan, 0) == dict(
        kind="battery", percentage=None, voltage=None, charging="UNKNOWN")
    assert battery_sample(.75, 24, 1)["percentage"] == 75
    assert battery_sample(-1, 24, 4)["percentage"] is None
    with pytest.raises(ValueError):
        ObservationBuffer("")


def test_partial_joints_keep_independent_age_and_expire():
    buffer = ObservationBuffer("sim")
    assert buffer.accept("/joint_states", 10, 10, dict(kind="joints", names=["arm"], positions=[1]))
    buffer.drain(10)
    assert buffer.accept("/joint_states", 12, 12, dict(kind="joints", names=["wheel"], positions=[2]))
    frame = buffer.drain(12)[0]
    assert frame["sample"]["names"] == ["arm", "wheel"]
    assert frame["sample"]["ages_seconds"] == [2, 0]
    assert buffer.accept("/joint_states", 14, 14, dict(kind="joints", names=["wheel"], positions=[3]))
    assert buffer.drain(14)[0]["sample"]["names"] == ["wheel"]
    assert not buffer.accept("/joint_states", 14, 14, dict(kind="joints", names=["wheel"], positions=[3]))


def test_source_time_reset_future_and_old_data():
    buffer = ObservationBuffer("real")
    value = dict(kind="base")
    assert not buffer.accept("/odom", 10, 20, value)
    assert not buffer.accept("/odom", 21, 20, value)
    assert buffer.accept("/odom", 20, 20, value)
    boot = buffer.boot
    assert buffer.accept("/odom", 1, 1, value)
    assert boot != buffer.boot
    assert buffer.drain(5) == []


def test_targets_remain_separate_and_events_bounded():
    buffer = ObservationBuffer("mock", event_capacity=2)
    for kind in ("joints", "joint_targets"):
        assert buffer.accept("/"+kind, 1, 1, dict(kind=kind, names=["arm"], positions=[.1]))
    for index in range(3):
        assert buffer.accept("/events", 1, 1, dict(kind="supervisor_event", event_sequence=index))
    frames = buffer.drain(1)
    assert len(frames) == 4 and buffer.dropped_events == 1
    assert {f["sample"]["kind"] for f in frames} == {"joints", "joint_targets", "supervisor_event"}
    assert [f["sequence"] for f in frames] == [1, 2, 3, 4]
