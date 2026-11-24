from rclpy.serialization import deserialize_message, serialize_message

from cleany_base_interfaces.msg import WheelCommand, WheelDriveState, WheelState


def test_fixed_command_roundtrip_preserves_sequence_deadline_and_wheel_order():
    command = WheelCommand(
        protocol_version=2,
        sequence=0xFFFFFFFF, valid_until_us=2**50,
        mode=WheelCommand.VELOCITY, velocity_rad_s=[1.0, -2.0, 3.0, -4.0],
    )
    payload = serialize_message(command)
    output = deserialize_message(payload, WheelCommand)
    assert output == command
    assert list(output.velocity_rad_s) == [1.0, -2.0, 3.0, -4.0]
    assert len(payload) < 512
    assert (WheelCommand.STOP, WheelCommand.ENABLE, WheelCommand.VELOCITY) == (0, 1, 2)
    assert 'session_id' not in command.get_fields_and_field_types()
    assert 'boot_id' not in command.get_fields_and_field_types()


def test_fixed_state_roundtrip_preserves_signed_counters_and_pwm():
    state = WheelState(
        protocol_version=2, boot_id=1, sequence=3,
        last_command_sequence=4, timestamp_us=2**50,
        counts_per_revolution=3172, max_velocity_rad_s=10.0,
        watchdog_ms=250, command_age_ms=65535, control_mode=WheelState.TIMEOUT_STOP,
        enabled=False, fault_bits=WheelState.FAULT_COMMAND_TIMEOUT,
        encoder_counts=[-(2**31), 2**31 - 1, -3, 4],
        velocity_rad_s=[1.0, 2.0, 3.0, 4.0],
        target_rad_s=[0.0] * 4, commanded_rad_s=[0.0] * 4,
        pwm_percent=[-100, 100, -1, 0],
    )
    payload = serialize_message(state)
    assert deserialize_message(payload, WheelState) == state
    assert len(payload) < 512
    assert WheelState.PROTOCOL_VERSION == WheelCommand.PROTOCOL_VERSION == 2
    assert 'session_id' not in state.get_fields_and_field_types()


def test_drive_state_roundtrip_preserves_signed_duty_and_pending_times():
    drive = WheelDriveState(
        boot_id=1, sequence=3, timestamp_us=2**50, pwm_period_ticks=2048,
        pwm_frequency_hz=20000, duty_ticks=[2040, -2040, 1, -1],
        requested_pwm_percent=[100.0, -100.0, 0.125, -0.125],
        first_sequence=0xffffffff, received_us=2**50 - 5000,
        start_counts=[-(2**31), 2**31 - 1, 0, 7],
        first_us=[0, 5000, -1, -1], fifth_us=[5000, -1, -1, -1],
        onset_ticks=[0, -96, 0, 0],
    )
    payload = serialize_message(drive)
    assert deserialize_message(payload, WheelDriveState) == drive
    assert len(payload) < 512
