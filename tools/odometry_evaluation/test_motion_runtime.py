"""Offline ROS-adapter tests: no node initialization or hardware commands."""

from types import SimpleNamespace
import math

import pytest
from nav_msgs.msg import Odometry

import evaluator_core as core
import motion_runtime as runtime
from motion_control import MotionConfig


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, interval):
        self.now += interval


class Publisher:
    def __init__(self):
        self.commands = []

    def publish(self, msg):
        self.commands.append((msg.linear.x, msg.linear.y, msg.angular.z))


def odometry(clock, x=0.0, yaw=0.0, speed=0.0, angular_speed=0.0):
    msg = Odometry()
    total_ns = round(clock.now * 1e9)
    msg.header.stamp.sec = 100 + total_ns // 1_000_000_000
    msg.header.stamp.nanosec = total_ns % 1_000_000_000
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.twist.twist.linear.x = speed
    msg.twist.twist.angular.z = angular_speed
    return msg


@pytest.fixture
def setup(monkeypatch):
    clock = Clock()
    log = SimpleNamespace(error=lambda _: None)
    enables, asynchronous_disables = [], []
    node = SimpleNamespace(
        guard=core.OdomFreshnessGuard(), current_pose=None, aborted=False,
        cmd_vel_pub=Publisher(), get_logger=lambda: log,
        set_base_enabled=lambda enable: enables.append(enable) or True,
        enable_client=SimpleNamespace(call_async=lambda request: asynchronous_disables.append(request.data)),
    )
    runtime.init_motion_runtime(node, MotionConfig.from_json())
    monkeypatch.setattr(core, "time", clock)
    monkeypatch.setattr(runtime, "time", clock)
    ros = SimpleNamespace(ok=lambda: True, spin_once=None)
    monkeypatch.setattr(runtime, "rclpy", ros)
    return clock, node, ros, enables, asynchronous_disables


def test_passive_resume_after_plot_waits_without_latching(setup):
    clock, node, ros, enables, _ = setup
    runtime.receive_odometry(node, odometry(clock))
    clock.now = 1.0
    calls = 0

    def spin(*args, **kwargs):
        nonlocal calls
        calls += 1
        clock.sleep(.02)
        if calls > 1:
            runtime.receive_odometry(node, odometry(clock, x=.1))

    ros.spin_once = spin
    pose = runtime.await_fresh_pose(node)
    assert calls == 2 and pose.x_m == .1
    assert not node.guard.is_tripped
    assert enables == [] and node.cmd_vel_pub.commands == []


def test_stale_in_motion_disables_and_publishes_only_zero_after_trip(setup):
    clock, node, ros, enables, disables = setup

    def spin(*args, **kwargs):
        clock.sleep(.01)
        if clock.now < .07:
            runtime.receive_odometry(node, odometry(clock))

    ros.spin_once = spin
    success, result = runtime.execute_motion(node, .2, 0, 0, .5, False, .1, 1)
    assert not success and node.guard.is_tripped and node.aborted
    assert enables == [True, False] and disables == [False]
    assert clock.now < .25
    commands = node.cmd_vel_pub.commands
    assert any(x > 0 for x, _, _ in commands)
    first_final_zero = next(i for i, command in enumerate(commands) if command == (0, 0, 0))
    assert all(command == (0, 0, 0) for command in commands[first_final_zero:])
    assert "timeout" in result["failure_reason"].lower()


def test_post_dwell_drift_cannot_report_success(setup):
    clock, node, ros, enables, _ = setup
    calls = 0

    def spin(*args, **kwargs):
        nonlocal calls
        calls += 1
        clock.sleep(.01)
        x = 0.0 if calls == 1 else (.5 if clock.now < .65 else .55)
        runtime.receive_odometry(node, odometry(clock, x=x))

    ros.spin_once = spin
    success, result = runtime.execute_motion(node, .2, 0, 0, .5, False, .4, 2)
    assert not success and not result["reached_tolerance"]
    assert result["actual_displacement"] == pytest.approx(.55)
    assert result["overshoot_mm"] == pytest.approx(50)
    assert enables[-1] is False


@pytest.mark.parametrize("sign", [-1, 1])
def test_full_turn_runtime_unwraps_and_finishes_stopped(setup, sign):
    clock, node, ros, enables, _ = setup
    yaw = 0.0
    angular = 0.0

    def spin(*args, **kwargs):
        nonlocal yaw, angular
        dt = .03
        clock.sleep(.01)
        angular = node.cmd_vel_pub.commands[-1][2] if node.cmd_vel_pub.commands else 0.0
        yaw += angular * dt
        runtime.receive_odometry(node, odometry(clock, yaw=yaw, angular_speed=angular))

    ros.spin_once = spin
    success, result = runtime.execute_motion(node, 0, 0, sign * .5, 2 * math.pi, True, .1, 30)
    assert success and result["reached_tolerance"], result["failure_reason"]
    assert result["actual_displacement"] == pytest.approx(2 * math.pi, abs=math.radians(1))
    assert enables == [True, False]
    assert node.cmd_vel_pub.commands[-1] == (0, 0, 0)


def test_nan_velocity_requests_callback_safe_disable(setup):
    clock, node, _, _, disables = setup
    runtime.receive_odometry(node, odometry(clock, speed=float("nan")))
    assert node.aborted and disables == [False]
    assert node.cmd_vel_pub.commands == [(0, 0, 0)]


def test_old_queued_header_does_not_refresh_receipt_watchdog(setup):
    clock, node, _, _, _ = setup
    old = odometry(clock)
    runtime.receive_odometry(node, old)
    clock.now = .2
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)
    )
    runtime.receive_odometry(node, old)
    with pytest.raises(core.OdomGuardError, match="timeout"):
        node.guard.check_freshness()


def test_duplicate_named_command_publisher_prevents_enable(setup):
    clock, node, ros, enables, _ = setup
    ros.spin_once = lambda *a, **k: (
        clock.sleep(.01), runtime.receive_odometry(node, odometry(clock))
    )
    node.get_name = lambda: "evaluation"
    node.get_publishers_info_by_topic = lambda _: [
        SimpleNamespace(node_name="evaluation"), SimpleNamespace(node_name="evaluation")
    ]
    success, result = runtime.execute_motion(node, .2, 0, 0, .5, False, .1, 1)
    assert not success and enables == [False]
    assert "publisher" in result["failure_reason"]


def test_backwards_stamp_is_latched_and_cannot_resume():
    guard = core.OdomFreshnessGuard()
    guard.update(100, 0, 0, 0, 0, mono_time=0)
    with pytest.raises(core.OdomGuardError, match="backwards"):
        guard.update(99, 0, 1, 0, 0, mono_time=.01)
    with pytest.raises(core.OdomGuardError, match="already tripped"):
        guard.update(101, 0, 0, 0, 0, mono_time=.02)


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_invalid_watchdog_settings_rejected(timeout):
    with pytest.raises(ValueError):
        core.OdomFreshnessGuard(timeout_s=timeout)
