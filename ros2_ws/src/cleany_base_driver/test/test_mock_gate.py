from copy import deepcopy
import time

import rclpy
import pytest
from cleany_base_interfaces.msg import WheelCommand, WheelState
from cleany_base_driver.driver_node import BaseDriver
from cleany_base_driver.mock_node import MockMcu
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger


def _command(mock, mode, sequence, *, deadline=None, values=None, version=2):
    msg = WheelCommand()
    msg.protocol_version = version
    msg.sequence, msg.mode = sequence, mode
    msg.velocity_rad_s = [0.] * 4 if values is None else values
    msg.valid_until_us = int(time.monotonic() * 1e6) + 100000 if deadline is None else deadline
    mock.command(msg)


def test_mock_enable_sequence_stop_and_watchdog():
    rclpy.init()
    mock = MockMcu()
    try:
        _command(mock, WheelCommand.ENABLE, 1)
        assert mock.enabled
        _command(mock, WheelCommand.ENABLE, 2)
        assert mock.enabled and mock.last_accepted_seq == 1
        _command(mock, WheelCommand.VELOCITY, 2, values=[1.] * 4)
        assert mock.target == [1.] * 4
        _command(mock, WheelCommand.STOP, 3)
        assert not mock.enabled and mock.last_accepted_seq == 3
        _command(mock, WheelCommand.ENABLE, 2)
        assert not mock.enabled
        _command(mock, WheelCommand.ENABLE, 4, version=1)
        assert not mock.enabled
        _command(mock, WheelCommand.ENABLE, 4, deadline=int(time.monotonic()*1e6))
        assert not mock.enabled
        _command(mock, WheelCommand.ENABLE, 4)
        assert mock.enabled
        mock.valid_until_us = int(time.monotonic()*1e6) - 1
        mock.publish()
        assert not mock.enabled
    finally:
        mock.destroy_node()
        rclpy.shutdown()


def test_mock_expiry_precedes_velocity_and_stop_watermark_never_rewinds(monkeypatch):
    import cleany_base_driver.mock_node as module
    rclpy.init()
    mock = MockMcu()
    now = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    try:
        # MCU와 동일하게 초기 명령의 sequence 0을 허용합니다.
        _command(mock, WheelCommand.ENABLE, 0, deadline=10_050_000)
        assert mock.enabled and mock.have_command_sequence
        now[0] = 10.051
        _command(mock, WheelCommand.VELOCITY, 1, deadline=10_200_000, values=[3.] * 4)
        assert not mock.enabled and mock.last_accepted_seq == 0
        assert mock.faults & WheelState.FAULT_COMMAND_TIMEOUT
        # 무효한 메타데이터로 STOP sequence의 최댓값을 되돌릴 수 없습니다.
        _command(mock, WheelCommand.STOP, 0, version=1)
        assert mock.last_accepted_seq == 0
        _command(mock, WheelCommand.ENABLE, 0, deadline=10_200_000)
        assert not mock.enabled
        _command(mock, WheelCommand.ENABLE, 1, deadline=10_200_000)
        assert mock.enabled and mock.faults == 0
    finally:
        mock.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def peers(monkeypatch, request):
    """executor 없이 결정적인 시계로 실제 콜백을 검사합니다."""
    rclpy.init()
    params = {
        'geometry.wheel_radius_m': .05, 'geometry.wheelbase_m': .30,
        'geometry.wheel_separation_m': .30, 'limits.linear_x_mps': .4,
        'limits.linear_y_mps': .4, 'limits.angular_z_rad_s': .5,
        'limits.wheel_rad_s': 10., 'limits.command_timeout_s': .4,
    }
    params.update(getattr(request, 'param', {}))
    driver = BaseDriver(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    mock = MockMcu()
    now, commands, states = [10.], [], []
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(driver.publisher, 'publish', commands.append)
    monkeypatch.setattr(mock.pub, 'publish', states.append)

    def feedback():
        mock.publish()
        driver.on_state(states[-1])

    feedback()
    try:
        yield driver, mock, now, commands, feedback
    finally:
        driver.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()


def _enable(driver):
    response = driver.on_enable(SetBool.Request(data=True), SetBool.Response())
    assert response.success


def test_driver_fixed_enable_retry_idle_and_fresh_commands(peers):
    driver, mock, now, commands, feedback = peers
    twist = Twist()
    twist.linear.x = .1
    driver.on_twist(twist)
    _enable(driver)
    original = commands[-1]
    assert driver.latest_twist is None
    mock.drop_enable_count = 1
    mock.command(original)
    assert not mock.enabled
    now[0] += .02
    feedback()
    driver.tick()
    assert commands[-1] == original  # sequence와 마감 시각 모두 증가하지 않습니다.
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert driver.pending_enable is None and mock.enabled
    # 구동 허가 직후 타이머와 경합하지 않고 목표 0인 허가 상태를 유지할 수 있습니다.
    for _ in range(20):
        driver.tick()
        mock.command(commands[-1])
        assert commands[-1].mode == WheelCommand.VELOCITY
        assert mock.target == [0.] * 4
        now[0] += .02
        feedback()
    count = len(commands)
    _enable(driver)
    assert len(commands) == count  # 이미 허가된 상태에서의 허가 요청은 멱등적입니다.
    driver.on_twist(twist)
    driver.tick()
    mock.command(commands[-1])
    assert mock.target == [2.] * 4


def test_driver_enable_delivery_expiry_does_not_renew_permission(peers):
    driver, mock, now, commands, feedback = peers
    _enable(driver)
    original = commands[-1]
    for offset in (.02, .10, .15):
        now[0] = 10. + offset
        feedback()
        driver.tick()
        assert commands[-1] == original
    now[0] = original.valid_until_us / 1e6
    feedback()
    driver.tick()
    assert not driver.enabled and driver.pending_enable is None
    assert commands[-1].mode == WheelCommand.STOP
    mock.command(commands[-1])
    replay = deepcopy(original)
    replay.valid_until_us += 200000
    mock.command(replay)
    assert not mock.enabled
    _enable(driver)
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert mock.enabled and driver.pending_enable is None


@pytest.mark.parametrize('peers', [{'limits.command_timeout_s': .3}], indirect=True)
def test_three_tenths_input_timeout_keeps_mcu_deadlines_bounded(peers):
    driver, mock, now, commands, feedback = peers
    _enable(driver)
    assert commands[-1].valid_until_us == driver.latest.timestamp_us + 200000
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert driver.pending_enable is None
    twist = Twist()
    twist.linear.x = .1
    driver.on_twist(twist)
    receipt = now[0]
    for tick in range(15):
        now[0] = receipt + tick * .02
        feedback()
        driver.tick()
        msg = commands[-1]
        assert driver.enabled and msg.mode == WheelCommand.VELOCITY
        assert msg.valid_until_us == driver.latest.timestamp_us + 200000
        mock.command(msg)
        assert mock.enabled and mock.target[0] > 0.
    now[0] = receipt + .32
    feedback()
    driver.tick()
    assert not driver.enabled and commands[-1].mode == WheelCommand.STOP
    assert driver.diagnostic_error == 'cmd_vel timeout'
    assert driver.fault_reason == ''  # 명시적인 안전 해제는 베이스 사용 불가 상태가 아닙니다.
    mock.command(commands[-1])
    assert not mock.enabled and mock.target == [0.] * 4


def test_driver_fault_recovery_input_timeout_and_feedback_validation(peers):
    driver, mock, now, commands, feedback = peers
    _enable(driver)
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    receipt = driver.last_state_receipt
    duplicate = deepcopy(driver.latest)
    now[0] += .02
    driver.on_state(duplicate)
    assert driver.enabled and driver.last_state_receipt == receipt
    gap = deepcopy(duplicate)
    gap.sequence += 20
    gap.timestamp_us = int(now[0] * 1e6)
    driver.on_state(gap)
    assert driver.enabled and driver.last_state_receipt == now[0]
    mock.sequence = gap.sequence + 1
    # 늦게 도착한 유효 속도 명령으로 만료된 MCU 허가를 복구할 수 없습니다.
    now[0] = mock.valid_until_us / 1e6
    feedback()
    assert not driver.enabled and mock.faults
    assert driver.fault_reason  # 만료된 MCU 허가는 다시 허가될 때까지 실제 고장으로 남습니다.
    mock.command(commands[-1])
    _enable(driver)
    now[0] += .02
    feedback()  # ENABLE 대기 중에는 이전 시간 초과 고장이 계속 도착할 수 있습니다.
    assert driver.enabled and driver.pending_enable is not None
    driver.tick()
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert not mock.faults and driver.pending_enable is None
    assert driver.fault_reason == ''  # ENABLE의 수신 확인으로 해제합니다.
    twist = Twist()
    twist.linear.x = .1
    driver.on_twist(twist)
    for _ in range(22):
        driver.tick()
        mock.command(commands[-1])
        now[0] += .02
        feedback()
    driver.tick()
    mock.command(commands[-1])
    assert not driver.enabled and not mock.enabled
    assert driver.latest_twist is None and driver.diagnostic_error == 'cmd_vel timeout'


def test_driver_reboot_restart_and_feedback_loss_rebase(peers):
    driver, mock, now, commands, feedback = peers
    # 드라이버가 재시작하면 실행 중인 상대를 정지시킬 수 있고 명령 sequence가 래핑될 수 있습니다.
    mock.enabled = mock.have_command_sequence = True
    mock.last_accepted_seq = 0xffffffff
    mock.valid_until_us = int(now[0] * 1e6) + 200000
    driver.command_sequence = None
    now[0] += .02
    feedback()
    assert commands[-1].mode == WheelCommand.STOP and commands[-1].sequence == 0
    mock.command(commands[-1])
    _enable(driver)
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert driver.enabled and driver.pending_enable is None
    # 재부팅 시 STOP 전송 전에 송신 sequence 기준을 다시 잡습니다.
    driver.command_sequence = 100
    mock.reboot(Trigger.Request(), Trigger.Response())
    now[0] += .02
    feedback()
    assert not driver.enabled and commands[-1].sequence == 1
    mock.command(commands[-1])
    _enable(driver)
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert driver.enabled
    before = list(driver.encoder.totals)
    now[0] += .30
    driver.tick()
    mock.command(commands[-1])
    assert not driver.enabled and driver.latest is None
    mock.encoder_integers = [1000] * 4
    feedback()
    assert driver.encoder.totals == before and not driver.enabled
    assert driver.latest_twist is None


def test_feedback_in_flight_before_stop_is_not_a_fault_but_ignored_stop_is(peers):
    driver, mock, now, commands, feedback = peers
    _enable(driver)
    mock.command(commands[-1])
    now[0] += .02
    feedback()
    assert driver.enabled and driver.pending_enable is None
    driver._disable('stopped by request')  # STOP을 보냈지만 MCU가 아직 처리하지 않은 상태입니다.

    def enabled_feedback():
        state = deepcopy(driver.latest)
        state.sequence += 1
        state.timestamp_us += 20000
        state.enabled = True
        driver.on_state(state)
    now[0] += .02
    enabled_feedback()
    assert driver.fault_reason == ''
    now[0] += .3
    enabled_feedback()
    assert driver.fault_reason == 'MCU enabled without host permission'
