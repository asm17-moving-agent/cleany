from dataclasses import replace
from math import pi

import pytest

from cleany_base_odometry.encoder_http import EncoderSample
from cleany_base_odometry.encoder_odometry import (
    EncoderAngleTracker, EncoderCalibration, RejectedEncoderSample,
)
from cleany_base_odometry.mecanum_odometry import MecanumGeometry, MecanumOdometry


CALIBRATION = EncoderCalibration((3172.0,) * 4, (1, -1, -1, 1))
EPOCH = 1700000000000000000
BOOT = '0123456789abcdef'


def sample(ticks=(0, 0, 0, 0), seq=0, us=0, boot=BOOT, rtt=0.02):
    return EncoderSample(ticks, rtt, boot, seq, us)


def update(tracker, message, jitter_ns=0):
    return tracker.update(message, EPOCH + message.sample_time_us * 1000 + 10000000 + jitter_ns)


def test_one_output_revolution_moves_one_wheel_circumference():
    tracker = EncoderAngleTracker(CALIBRATION, max_gap_sec=2.0)
    odom = MecanumOdometry(MecanumGeometry(0.0635, 0.30, 0.51))
    baseline = update(tracker, sample((100, 200, 300, 400)))
    assert baseline.baseline_reason == 'initial'
    assert odom.update(baseline.positions, baseline.stamp_ns * 1e-9) is None
    angles = update(tracker, sample((3272, -2972, -2872, 3572), seq=1, us=1000000))
    assert angles.positions.as_tuple() == pytest.approx((2 * pi,) * 4)
    estimate = odom.update(angles.positions, angles.stamp_ns * 1e-9)
    assert estimate.x_m == pytest.approx(2 * pi * 0.0635)
    assert estimate.y_m == pytest.approx(0.0)
    assert estimate.yaw_rad == pytest.approx(0.0)


def test_rear_wheel_order_and_direction_are_mapped_to_ros():
    tracker = EncoderAngleTracker(CALIBRATION)
    update(tracker, sample())
    result = update(tracker, sample((10, -20, -30, 40), seq=1, us=100000))
    assert result.positions.as_tuple() == pytest.approx(tuple(v * 2 * pi / 3172 for v in (10, 20, 40, 30)))


def test_count_and_sequence_wrap_are_continuous():
    tracker = EncoderAngleTracker(CALIBRATION)
    update(tracker, sample((2**31 - 1, -(2**31), 0, 0), seq=2**32 - 1))
    result = update(tracker, sample((-(2**31), 2**31 - 1, 0, 0), seq=0, us=100000))
    assert result.positions.as_tuple() == pytest.approx((2 * pi / 3172, 2 * pi / 3172, 0, 0))
    assert not result.baseline_reason


def test_dropped_samples_keep_cumulative_distance_and_mcu_timing():
    tracker = EncoderAngleTracker(CALIBRATION)
    first = update(tracker, sample())
    result = update(tracker, sample((100, -100, -100, 100), seq=4, us=200000), jitter_ns=80000000)
    assert result.positions.as_tuple() == pytest.approx((100 * 2 * pi / 3172,) * 4)
    assert result.stamp_ns - first.stamp_ns == 200000000


@pytest.mark.parametrize('bad', [sample(seq=0, us=100000), sample(seq=2**32 - 1, us=100000), sample(seq=1, us=0)])
def test_duplicate_or_old_sample_does_not_replace_baseline(bad):
    tracker = EncoderAngleTracker(CALIBRATION)
    update(tracker, sample())
    with pytest.raises(RejectedEncoderSample):
        update(tracker, bad)
    result = update(tracker, sample((10, -10, -10, 10), seq=1, us=100000))
    assert result.positions.front_left == pytest.approx(10 * 2 * pi / 3172)


def test_reboot_retains_pose_and_rejects_retired_boot_packets():
    tracker = EncoderAngleTracker(CALIBRATION)
    odom = MecanumOdometry(MecanumGeometry(0.0635, 0.30, 0.51))
    def integrate(message, arrival):
        result = tracker.update(message, arrival)
        if result.baseline_reason:
            odom.reset_baseline()
        return odom.update(result.positions, result.stamp_ns * 1e-9)
    integrate(sample((1000, -1000, -1000, 1000)), EPOCH)
    before = integrate(sample((1100, -1100, -1100, 1100), seq=1, us=100000), EPOCH + 100000000)
    assert integrate(sample(boot='fedcba9876543210'), EPOCH + 200000000) is None
    with pytest.raises(RejectedEncoderSample):
        integrate(sample(seq=2, us=300000), EPOCH + 300000000)
    after = integrate(sample(boot='fedcba9876543210', seq=1, us=100000), EPOCH + 300000000)
    assert after.x_m == pytest.approx(before.x_m)
    assert after.linear_x_mps == pytest.approx(0.0)


@pytest.mark.parametrize('reason, message, jitter', [
    ('sample_gap', sample((999, 0, 0, 0), seq=1, us=1000000), 0),
    ('tick_jump', sample((1000000, 0, 0, 0), seq=1, us=100000), 0),
    ('clock_changed', sample((100, 0, 0, 0), seq=1, us=100000), 1000000000),
    ('clock_changed', sample((100, 0, 0, 0), seq=1, us=100000), -1000000000),
])
def test_discontinuity_rebaselines_without_inventing_motion(reason, message, jitter):
    tracker = EncoderAngleTracker(CALIBRATION)
    update(tracker, sample())
    result = update(tracker, message, jitter)
    assert result.baseline_reason == reason
    assert result.positions.as_tuple() == (0.0,) * 4


@pytest.mark.parametrize('message', [
    sample(boot=''), sample(boot='invalid'), sample(ticks=(0, 0, 0)),
    sample(ticks=(2**31, 0, 0, 0)), sample(rtt=-1), sample(rtt=float('nan')),
    sample(rtt=1.0), replace(sample(), sample_time_us=-1),
])
def test_missing_or_bad_metadata_is_rejected(message):
    with pytest.raises(RejectedEncoderSample):
        update(EncoderAngleTracker(CALIBRATION), message)
