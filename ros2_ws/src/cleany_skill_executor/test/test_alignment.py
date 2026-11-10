import math

import pytest

from cleany_skill_executor.core.alignment import (
    axis_errors_degrees,
    sampled_indices,
    tcp_axes,
    tcp_quaternion_for_approach,
)
from cleany_skill_executor.core.physical_grasp_selection import GraspSelectionConfig


def test_tcp_uses_local_minus_y_approach_and_plus_x_closing() -> None:
    approach, closing = tcp_axes((0.0, 0.0, 0.0, 1.0))
    assert approach == pytest.approx((0.0, -1.0, 0.0))
    assert closing == pytest.approx((1.0, 0.0, 0.0))


def test_quaternion_constructor_preserves_requested_approach() -> None:
    requested = (0.0, -0.2, 0.98)
    quaternion = tcp_quaternion_for_approach(requested)
    approach, closing = tcp_axes(quaternion)
    norm = math.sqrt(sum(value * value for value in requested))
    assert approach == pytest.approx(tuple(value / norm for value in requested))
    assert closing == pytest.approx((1.0, 0.0, 0.0))


def test_axis_errors_distinguish_approach_and_wrist_closing_rotation() -> None:
    half = math.sqrt(0.5)
    approach, closing = axis_errors_degrees(
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, half, half),
    )
    assert approach == pytest.approx(90.0)
    assert closing == pytest.approx(90.0)


def test_simulation_opening_mapping_reserves_five_millimeters() -> None:
    config = GraspSelectionConfig()
    expected = config.gripper_closed_joint_rad + 0.95 * (
        config.gripper_open_joint_rad - config.gripper_closed_joint_rad
    )
    assert config.opening_to_joint_position(0.095) == pytest.approx(expected)
    with pytest.raises(ValueError, match='safe simulation limit'):
        config.opening_to_joint_position(0.0951)


def test_trajectory_sampling_keeps_endpoints_and_caps_twenty() -> None:
    indices = sampled_indices(101, 20)
    assert len(indices) == 20
    assert indices[0] == 0
    assert indices[-1] == 100
