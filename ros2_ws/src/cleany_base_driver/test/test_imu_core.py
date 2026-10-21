"""Transport checks and receipt-anchored timestamp mapping."""
from types import SimpleNamespace

import pytest

from cleany_base_driver.imu_core import ImuAdapter, ImuValidationError


def message(seq=1, timestamp=1000, boot=7, **changes):
    values = dict(protocol_version=2, device_mac=[2, 3, 4, 5, 6, 7], boot_id=boot,
                  sequence=seq, timestamp_us=timestamp, who_am_i=0x68, fault_bits=0,
                  angular_velocity_rad_s=[1., 2., 3.], acceleration_mps2=[0., 0., 9.8],
                  temperature_c=25., calibration_state=1, calibration_reason=0,
                  corrected_angular_velocity_rad_s=[.9, 1.8, 2.7],
                  gyro_bias_rad_s=[.1, .2, .3], gyro_covariance_rad2_s2=[0.] * 9)
    values.update(changes)
    return SimpleNamespace(**values)


def test_mcu_intervals_sequence_wrap_and_rejected_duplicate():
    adapter = ImuAdapter()
    first = adapter.stamp(message(0xffffffff), 10_000_000_000, 5_000_000_000)
    second = adapter.stamp(message(0, 11000), 10_015_000_000, 5_015_000_000)
    assert second - first == 10_000_000
    with pytest.raises(ImuValidationError):
        adapter.stamp(message(0, 11000), 10_016_000_000, 5_016_000_000)
    assert adapter.stamp(message(1, 12000), 10_017_000_000, 5_017_000_000) == 10_011_000_000


def test_backlog_clock_jump_reanchor_and_reboot_retirement():
    adapter = ImuAdapter()
    adapter.stamp(message(), 1_000_000_000, 1_000_000_000)
    with pytest.raises(ImuValidationError, match="backlog"):
        adapter.stamp(message(2, 2000), 1_200_000_000, 1_200_000_000)
    with pytest.raises(ImuValidationError, match="clock"):
        adapter.stamp(message(2, 2000), 1_200_000_000, 1_001_000_000)
    assert adapter.stamp(message(3, 3000), 1_201_000_000, 1_002_000_000) == 1_201_000_000
    assert adapter.stamp(message(1, 100, 8), 1_210_000_000, 1_011_000_000) == 1_210_000_000
    with pytest.raises(ImuValidationError, match="retired"):
        adapter.stamp(message(4, 4000, 7), 1_220_000_000, 1_021_000_000)


@pytest.mark.parametrize("changes", [
    {"protocol_version": 1}, {"who_am_i": 0x69}, {"fault_bits": 1},
    {"boot_id": 0}, {"timestamp_us": 0}, {"device_mac": [1, 2, 3, 4, 5, 6]},
    {"device_mac": [0] * 6},
])
def test_transport_fault_rejected_without_advancing_state(changes):
    adapter = ImuAdapter()
    adapter.stamp(message(), 1_000_000_000, 1_000_000_000)
    with pytest.raises(ImuValidationError):
        adapter.stamp(message(2, 2000, **changes), 1_001_000_000, 1_001_000_000)
    assert adapter.stamp(message(2, 2000), 1_001_000_000, 1_001_000_000) == 1_001_000_000


def test_identity_expected_mac_and_timing_configuration():
    with pytest.raises(ImuValidationError, match="unexpected"):
        ImuAdapter(expected_device_mac="02:03:04:05:06:08").stamp(message(), 1, 1)
    adapter = ImuAdapter()
    adapter.stamp(message(), 1_000_000_000, 1_000_000_000)
    with pytest.raises(ImuValidationError, match="identity changed"):
        adapter.stamp(message(2, 2000, device_mac=[2, 4, 4, 5, 6, 7]), 1_001_000_000, 1_001_000_000)
    for mac in ("bad", "00:00:00:00:00:00", "01:03:04:05:06:07"):
        with pytest.raises(ValueError):
            ImuAdapter(expected_device_mac=mac)
    for timing in (0., -1., float("nan"), float("inf")):
        with pytest.raises(ValueError):
            ImuAdapter(max_transport_delay_s=timing)


def test_calibration_result_is_untouched_measurement_contract():
    sample = message(gyro_covariance_rad2_s2=[0.] * 9)
    assert ImuAdapter().stamp(sample, 1_000_000, 1_000_000) == 1_000_000
    assert sample.corrected_angular_velocity_rad_s == [.9, 1.8, 2.7]
    assert sample.gyro_bias_rad_s == [.1, .2, .3]
    assert sample.gyro_covariance_rad2_s2 == [0.] * 9
