from types import SimpleNamespace as NS

import pytest

from stack_readiness import ReadinessState


def message(stamp_ns):
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)),
              pose=NS(pose=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.))),
              ranges=[1.] * 100, range_min=.15, range_max=12.,
              timestamp_us=stamp_ns // 1000, enabled=False, fault_bits=0,
              target_rad_s=[0.] * 4, commanded_rad_s=[0.] * 4, pwm_percent=[0] * 4)


def filled_state():
    state = ReadinessState(.15, 2., .45)
    for i in range(3):
        msg = message(10**9 + i * 20_000_000)
        state.receive_odom(msg, 1 + .02 * i, 10**9 + i * 20_000_000)
        state.receive_scan(msg, 1 + .02 * i, 10**9 + i * 20_000_000)
        state.receive_wheel(msg, 1 + .02 * i)
    return state


PUBLISHERS = {"cmd_vel": 0, "wheel_command": 1, "wheel_state": 1, "odom": 1, "scan": 1}


def test_graph_alone_is_not_readiness():
    state = ReadinessState(.15, 2., .45)
    assert not state.snapshot(1., PUBLISHERS, True)["ready"]


def test_three_fresh_advancing_samples_and_service_are_required():
    state = filled_state()
    assert state.snapshot(1.04, PUBLISHERS, True)["ready"]
    assert not state.snapshot(1.3, PUBLISHERS, True)["ready"]
    assert not state.snapshot(1.04, PUBLISHERS, False)["ready"]
    assert not state.snapshot(1.04, {**PUBLISHERS, "cmd_vel": 1}, True)["ready"]
    assert not state.snapshot(1.04, {**PUBLISHERS, "scan": 2}, True)["ready"]


@pytest.mark.parametrize("stream", ["odom", "scan", "wheel"])
def test_repeated_stamps_rejected(stream):
    state = filled_state()
    msg = message(1_040_000_000)
    args = (msg, 1.05) if stream == "wheel" else (msg, 1.05, 1_050_000_000)
    getattr(state, f"receive_{stream}")(*args)
    assert not state.snapshot(1.05, PUBLISHERS, True)["ready"]


def test_enabled_or_faulty_mcu_rejected():
    state = filled_state()
    msg = message(1_060_000_000)
    msg.enabled = True
    state.receive_wheel(msg, 1.06)
    assert not state.snapshot(1.06, PUBLISHERS, True)["ready"]


def test_profile_only_scan_is_not_ready():
    state = filled_state()
    msg = message(1_060_000_000)
    msg.ranges = [.30] * 100
    state.receive_scan(msg, 1.06, 1_060_000_000)
    snapshot = state.snapshot(1.06, PUBLISHERS, True)
    assert snapshot["scan_fresh"]
    assert not snapshot["ready"]


def test_old_ros_stamps_cannot_become_fresh_just_by_receiving():
    state = filled_state()
    msg = message(1_060_000_000)
    state.receive_odom(msg, 10., 10_000_000_000)
    state.receive_scan(msg, 10., 10_000_000_000)
    assert not state.snapshot(10., PUBLISHERS, True)["ready"]


def test_probe_owns_executor_context_without_default_ros_init(monkeypatch):
    pytest.importorskip("rclpy")
    pytest.importorskip("cleany_base_interfaces.msg")
    from stack_readiness import RosReadinessProbe
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    probe = RosReadinessProbe("/wheel_eval_test_missing_odom", .15, 2., .45)
    try:
        probe.spin(.02)
        assert not probe.snapshot()["ready"]
        # The observer owns no motion publishers.
        assert {name for name, _ in probe.node.get_publisher_names_and_types_by_node(
            probe.node.get_name(), probe.node.get_namespace()
        )} == {"/parameter_events", "/rosout"}
    finally:
        probe.close()
