from rclpy.serialization import deserialize_message, serialize_message

from cleany_base_interfaces.msg import WheelCommand, WheelState


def test_fixed_command_roundtrip_preserves_identifiers_deadline_and_wheel_order():
    command = WheelCommand(
        protocol_version=1, boot_id=0xFFFFFFFF, session_id=0xDEADBEEF,
        sequence=0xFFFFFFFF, valid_until_us=2**50,
        mode=WheelCommand.VELOCITY, velocity_rad_s=[1.0, -2.0, 3.0, -4.0],
    )
    payload = serialize_message(command)
    output = deserialize_message(payload, WheelCommand)
    assert output == command
    assert list(output.velocity_rad_s) == [1.0, -2.0, 3.0, -4.0]
    assert len(payload) < 512
    assert (WheelCommand.STOP, WheelCommand.BEGIN_SESSION,
            WheelCommand.ARM, WheelCommand.VELOCITY) == (0, 1, 2, 3)


def test_fixed_state_roundtrip_preserves_signed_counters_and_pwm():
    state = WheelState(
        protocol_version=1, boot_id=1, session_id=2, sequence=3,
        last_command_sequence=4, timestamp_us=2**50,
        counts_per_revolution=3172, max_velocity_rad_s=10.0,
        watchdog_ms=250, command_age_ms=65535, control_mode=WheelState.WATCHDOG_STOP,
        armed=False, fault_bits=WheelState.FAULT_COMMAND_TIMEOUT,
        encoder_counts=[-(2**31), 2**31 - 1, -3, 4],
        velocity_rad_s=[1.0, 2.0, 3.0, 4.0],
        target_rad_s=[0.0] * 4, commanded_rad_s=[0.0] * 4,
        pwm_percent=[-100, 100, -1, 0],
    )
    payload = serialize_message(state)
    assert deserialize_message(payload, WheelState) == state
    assert len(payload) < 512
    assert WheelState.PROTOCOL_VERSION == WheelCommand.PROTOCOL_VERSION == 1
