from math import isclose, pi, sqrt
import pytest
from unittest import TestCase

from cleany_base_odometry.mecanum_odometry import (
    MecanumGeometry,
    MecanumOdometry,
    OdometryEstimate,
    WheelPositions,
)


GEOMETRY = MecanumGeometry(
    wheel_radius_m=0.0635,
    wheelbase_m=0.30,
    wheel_separation_m=0.51,
)
ZERO = WheelPositions(0.0, 0.0, 0.0, 0.0)


def estimate_after_one_second(
    positions: WheelPositions,
) -> OdometryEstimate:
    odometry = MecanumOdometry(GEOMETRY)
    assert odometry.update(ZERO, 1.0) is None
    estimate = odometry.update(positions, 2.0)
    assert estimate is not None
    return estimate


class MecanumOdometryTest(TestCase):
    def test_equal_positive_wheel_rotation_moves_forward(self) -> None:
        estimate = estimate_after_one_second(
            WheelPositions(1.0, 1.0, 1.0, 1.0)
        )

        self.assertTrue(isclose(estimate.x_m, GEOMETRY.wheel_radius_m))
        self.assertTrue(isclose(estimate.y_m, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(estimate.yaw_rad, 0.0, abs_tol=1e-12))
        self.assertTrue(
            isclose(estimate.linear_x_mps, GEOMETRY.wheel_radius_m)
        )

    def test_mecanum_wheel_pattern_moves_left(self) -> None:
        estimate = estimate_after_one_second(
            WheelPositions(-1.0, 1.0, 1.0, -1.0)
        )

        self.assertTrue(isclose(estimate.x_m, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(estimate.y_m, GEOMETRY.wheel_radius_m))
        self.assertTrue(isclose(estimate.yaw_rad, 0.0, abs_tol=1e-12))
        self.assertTrue(
            isclose(estimate.linear_y_mps, GEOMETRY.wheel_radius_m)
        )

    def test_left_back_right_forward_rotates_counter_clockwise(self) -> None:
        estimate = estimate_after_one_second(
            WheelPositions(-1.0, 1.0, -1.0, 1.0)
        )
        expected_yaw = (
            GEOMETRY.wheel_radius_m / GEOMETRY.center_to_wheel_sum_m
        )

        self.assertTrue(isclose(estimate.x_m, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(estimate.y_m, 0.0, abs_tol=1e-12))
        self.assertTrue(isclose(estimate.yaw_rad, expected_yaw))
        self.assertTrue(isclose(estimate.angular_z_rps, expected_yaw))

    def test_non_increasing_timestamp_restarts_sample_baseline(self) -> None:
        odometry = MecanumOdometry(GEOMETRY)
        self.assertIsNone(odometry.update(ZERO, 1.0))
        self.assertIsNone(
            odometry.update(WheelPositions(1.0, 1.0, 1.0, 1.0), 1.0)
        )

        estimate = odometry.update(
            WheelPositions(2.0, 2.0, 2.0, 2.0), 2.0
        )
        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertTrue(isclose(estimate.x_m, GEOMETRY.wheel_radius_m))


def test_count_window_reduces_lateral_encoder_alias_without_losing_pose():
    instant = MecanumOdometry(GEOMETRY)
    windowed = MecanumOdometry(GEOMETRY, velocity_window_s=.125)
    quantum = 2*pi/3172
    phases = (.23, .71, .41, .89)
    instantaneous_x, windowed_x, windowed_y = [], [], []
    for index in range(1001):
        stamp = 1. + index*.001
        positions = WheelPositions(*(
            round(phase + sign*.008/GEOMETRY.wheel_radius_m*(stamp-1.)/quantum)*quantum
            for phase, sign in zip(phases, (-1., 1., 1., -1.))
        ))
        raw = instant.update(positions, stamp)
        filtered = windowed.update(positions, stamp)
        if filtered:
            assert raw is not None
            instantaneous_x.append(raw.linear_x_mps)
            windowed_x.append(filtered.linear_x_mps)
            windowed_y.append(filtered.linear_y_mps)
            # Count integration and pose publication are independent of twist
            # averaging after warmup; no encoder motion is discarded.
            assert (filtered.x_m, filtered.y_m, filtered.yaw_rad) == (raw.x_m, raw.y_m, raw.yaw_rad)
    rms = lambda values: sqrt(sum(v*v for v in values)/len(values))
    assert len(windowed_x) > 850  # publish every input after warmup, not once per window
    assert rms(windowed_x) < rms(instantaneous_x)/10
    assert abs(sum(windowed_y)/len(windowed_y)-.008) < .0001
    assert filtered is not None and abs(filtered.y_m-.008) < .0002


def test_window_requires_observable_velocity_after_clock_reset_and_data_gap():
    odometry = MecanumOdometry(GEOMETRY, velocity_window_s=.125)
    assert odometry.update(ZERO, 1.) is None
    assert odometry.update(WheelPositions(.1, .1, .1, .1), 1.05) is None
    estimate = odometry.update(WheelPositions(.2, .2, .2, .2), 1.125)
    assert estimate is not None
    assert isclose(estimate.x_m, GEOMETRY.wheel_radius_m*.2)
    assert odometry.update(ZERO, .5) is None
    assert odometry.update(ZERO, .55) is None
    assert odometry.update(ZERO, .625) is not None
    assert odometry.update(ZERO, 2.) is None
    assert odometry.update(ZERO, 2.05) is None
    assert odometry.update(ZERO, 2.125) is not None
    # Once stopped for the full window, a moving historical sample cannot
    # survive and report motion indefinitely.
    stopped = odometry.update(ZERO, 2.2)
    assert stopped is not None and stopped.linear_x_mps == 0.


@pytest.mark.parametrize('window', [-1., float('nan'), float('inf')])
def test_invalid_velocity_window_is_rejected(window):
    with pytest.raises(ValueError):
        MecanumOdometry(GEOMETRY, velocity_window_s=window)


@pytest.mark.parametrize('window', [0.0, 0.125])
def test_yaw_calibration_preserves_encoder_translation_and_rotates_later_motion(window):
    from math import cos, sin
    odometry = MecanumOdometry(GEOMETRY, velocity_window_s=window, yaw_scale=1.2)
    odometry.update(ZERO, 1.0)
    for step in range(1, 21):
        t = step / 20
        result = odometry.update(WheelPositions(-t, t, -t, t), 1.+t)
    expected = 1.2 * GEOMETRY.wheel_radius_m / GEOMETRY.center_to_wheel_sum_m
    assert result.yaw_rad == pytest.approx(expected)
    assert result.angular_z_rps == pytest.approx(expected)
    assert result.x_m == pytest.approx(0.)
    assert result.y_m == pytest.approx(0.)
    for step in range(1, 21):
        t = step / 20
        result = odometry.update(WheelPositions(-1.+t, 1.+t, -1.+t, 1.+t), 2.+t)
    assert result.x_m == pytest.approx(GEOMETRY.wheel_radius_m*cos(expected))
    assert result.y_m == pytest.approx(GEOMETRY.wheel_radius_m*sin(expected))
    assert result.linear_x_mps == pytest.approx(GEOMETRY.wheel_radius_m)
    assert result.angular_z_rps == pytest.approx(0.)


@pytest.mark.parametrize('scale', [0., -1., float('nan'), float('inf')])
def test_invalid_yaw_calibration_is_rejected(scale):
    with pytest.raises(ValueError, match='Yaw scale'):
        MecanumOdometry(GEOMETRY, yaw_scale=scale)
