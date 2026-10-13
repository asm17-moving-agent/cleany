from math import isclose
from unittest import TestCase

from cleany_gazebo_sim.odometry_error import (
    OdometryErrorParameters,
    Pose2D,
    StatefulOdometryError,
)


ZERO = Pose2D(0.0, 0.0, 0.0)


class StatefulOdometryErrorTest(TestCase):
    def test_ideal_profile_passes_pose_through_exactly(self) -> None:
        model = StatefulOdometryError(OdometryErrorParameters())
        model.update(Pose2D(2.0, -1.0, 0.2), 1.0)
        raw = Pose2D(2.4, -0.7, 0.5)

        estimate = model.update(raw, 2.0)

        self.assertEqual(estimate.pose, raw)
        self.assertFalse(estimate.slip_active)

    def test_axis_scales_change_relative_motion(self) -> None:
        model = StatefulOdometryError(
            OdometryErrorParameters(
                forward_scale=1.1,
                lateral_scale=0.8,
                rotation_scale=1.2,
            )
        )
        model.update(ZERO, 1.0)

        estimate = model.update(Pose2D(1.0, 0.0, 0.5), 2.0)

        self.assertTrue(isclose(estimate.linear_x_mps, 1.1))
        self.assertTrue(isclose(estimate.linear_y_mps, 0.0))
        self.assertTrue(isclose(estimate.angular_z_rps, 0.6))
        self.assertTrue(isclose(estimate.pose.yaw_rad, 0.6))

    def test_yaw_drift_accumulates_during_forward_motion(self) -> None:
        model = StatefulOdometryError(
            OdometryErrorParameters(yaw_drift_rad_per_forward_m=0.1)
        )
        model.update(ZERO, 1.0)

        estimate = model.update(Pose2D(1.0, 0.0, 0.0), 2.0)

        self.assertTrue(isclose(estimate.pose.yaw_rad, 0.1))
        self.assertTrue(isclose(estimate.angular_z_rps, 0.1))

    def test_random_model_is_seeded_and_does_not_drift_at_rest(self) -> None:
        parameters = OdometryErrorParameters(
            forward_noise_stddev_per_sqrt_m=0.1,
            yaw_bias_walk_stddev_per_sqrt_m=0.1,
            max_abs_yaw_bias_rad_per_m=0.2,
            random_seed=7,
        )
        first = StatefulOdometryError(parameters)
        second = StatefulOdometryError(parameters)
        for model in (first, second):
            model.update(ZERO, 1.0)

        first_moving = first.update(Pose2D(1.0, 0.0, 0.0), 2.0)
        second_moving = second.update(Pose2D(1.0, 0.0, 0.0), 2.0)
        first_still = first.update(Pose2D(1.0, 0.0, 0.0), 3.0)

        self.assertEqual(first_moving, second_moving)
        self.assertEqual(first_still.pose, first_moving.pose)
        self.assertEqual(
            first_still.yaw_bias_rad_per_m,
            first_moving.yaw_bias_rad_per_m,
        )

    def test_slip_is_a_persistent_motion_state(self) -> None:
        model = StatefulOdometryError(
            OdometryErrorParameters(
                slip_events_per_second=1e9,
                slip_duration_min_sec=0.5,
                slip_duration_max_sec=0.5,
                slip_gain_min=1.1,
                slip_gain_max=1.1,
            )
        )
        model.update(ZERO, 1.0)

        started = model.update(Pose2D(1.0, 0.0, 0.0), 1.1)
        continuing = model.update(Pose2D(2.0, 0.0, 0.0), 1.2)
        stopped = model.update(Pose2D(2.0, 0.0, 0.0), 1.7)

        self.assertTrue(started.slip_active)
        self.assertTrue(continuing.slip_active)
        self.assertTrue(isclose(started.linear_x_mps, 11.0))
        self.assertFalse(stopped.slip_active)
        self.assertEqual(stopped.pose, continuing.pose)

    def test_invalid_slip_range_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            OdometryErrorParameters(
                slip_gain_min=1.2,
                slip_gain_max=1.1,
            )
