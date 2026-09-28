from math import isclose
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
