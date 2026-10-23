from rclpy.serialization import deserialize_message, serialize_message

from cleany_base_interfaces.msg import ImuState, WheelCommand, WheelState


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


def test_fixed_imu_state_roundtrip_preserves_si_samples_and_device_identity():
    state = ImuState(
        protocol_version=2, device_mac=[0x90, 0xe5, 0xb1, 0xd5, 0x3f, 0x34],
        boot_id=123, sequence=0xffffffff, timestamp_us=2**50, who_am_i=0x68,
        fault_bits=ImuState.FAULT_SATURATED,
        angular_velocity_rad_s=[0.1, -0.2, 0.3], acceleration_mps2=[0., 0., 9.80665],
        temperature_c=27.5,
        calibration_state=ImuState.FALLBACK, calibration_reason=ImuState.REASON_TIMEOUT,
        corrected_angular_velocity_rad_s=[.09, -.18, .27], gyro_bias_rad_s=[.01, -.02, .03],
        gyro_covariance_rad2_s2=[1e-6, 0., 0., 0., 2e-6, 0., 0., 0., 3e-6],
    )
    payload = serialize_message(state)
    assert deserialize_message(payload, ImuState) == state
    assert len(payload) < 256
    assert ImuState.PROTOCOL_VERSION == 2
    assert (ImuState.CALIBRATING, ImuState.RUNNING, ImuState.FALLBACK) == (0, 1, 2)
    assert (ImuState.FAULT_SENSOR, ImuState.FAULT_SATURATED) == (1, 2)
    assert all('string' not in value and 'sequence' not in value
               for value in state.get_fields_and_field_types().values())
