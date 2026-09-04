import numpy as np
import pytest
from cleany_interfaces.msg import DetectedObject3D, GraspCandidate
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.can_grasp_execution_demo import (
    EXECUTION_CAN_ID,
    CanGraspExecutionDemo,
)
from cleany_skill_executor.core.can_rgbd import SegmentedCanCloud


def target_object() -> DetectedObject3D:
    target = DetectedObject3D()
    target.obb_pose.position.x = 0.44
    target.obb_pose.position.y = 0.16
    target.obb_pose.position.z = 0.395
    target.obb_pose.orientation.w = 1.0
    target.obb_size.x = target.obb_size.y = 0.070
    target.obb_size.z = 0.100
    return target


def test_execution_collision_is_detected_can_cylinder() -> None:
    target = target_object()

    collision = CanGraspExecutionDemo._execution_collision(target)

    assert collision.id == EXECUTION_CAN_ID
    assert collision.header.frame_id == 'base_link'
    assert collision.operation == CollisionObject.ADD
    assert collision.primitives[0].type == SolidPrimitive.CYLINDER
    assert collision.primitives[0].dimensions == pytest.approx([0.100, 0.035])
    assert collision.primitive_poses[0] == target.obb_pose


def test_execution_collision_uses_box_dimensions_for_box_target() -> None:
    target = target_object()
    target.label = 'box'
    target.obb_size.x = 0.050
    target.obb_size.y = 0.050
    target.obb_size.z = 0.080

    collision = CanGraspExecutionDemo._execution_collision(target)

    assert collision.primitives[0].type == SolidPrimitive.BOX
    assert collision.primitives[0].dimensions == pytest.approx(
        [0.050, 0.050, 0.080]
    )


def test_attachment_moves_can_from_world_to_selected_gripper() -> None:
    scene = CanGraspExecutionDemo._attachment_scene(target_object(), 'left')

    assert scene.is_diff
    assert scene.robot_state.is_diff
    assert not scene.world.collision_objects
    attached = scene.robot_state.attached_collision_objects[0]
    assert attached.link_name == 'left_gripper_frame'
    assert attached.object.id == EXECUTION_CAN_ID
    assert attached.object.operation == CollisionObject.ADD
    assert attached.touch_links == [
        'left_gripper_frame',
        'left_moving_jaw_link',
    ]


def test_observed_can_center_uses_trimmed_vertical_extent() -> None:
    points = np.asarray(
        [
            [0.44, 0.16, 0.50],
            [0.44, 0.16, 0.54],
            [0.44, 0.16, 0.56],
            [0.44, 0.16, 0.60],
        ],
        dtype=float,
    )
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    cloud = SegmentedCanCloud(points, colors, points.copy(), colors.copy())

    center = CanGraspExecutionDemo._observed_can_center_z(cloud)

    assert center == pytest.approx(0.55)


def test_fixed_jaw_centering_offset_matches_can_geometry() -> None:
    offset = CanGraspExecutionDemo._fixed_jaw_centering_offset(
        0.070, 0.008
    )

    assert offset == pytest.approx(0.027)


def test_execution_tcp_combines_guarded_depth_and_jaw_centering() -> None:
    candidate = GraspCandidate()
    candidate.tcp_pose.position.x = 0.44
    candidate.tcp_pose.position.y = 0.16
    candidate.tcp_pose.position.z = 0.42
    candidate.tcp_pose.orientation.w = 1.0
    candidate.approach_direction.y = -1.0

    target = CanGraspExecutionDemo._execution_tcp_position(
        candidate,
        approach_offset_m=0.010,
        lateral_offset_m=0.040,
    )

    assert target == pytest.approx((0.480, 0.150, 0.42))


def test_required_opening_selects_width_aware_close_position() -> None:
    position = CanGraspExecutionDemo._opening_to_gripper_position(
        required_opening_m=0.080,
        opening_reduction_m=0.010,
        reference_aperture_m=0.050,
        reference_position_rad=0.30,
        aperture_m_per_rad=0.10,
        minimum_position_rad=0.30,
        maximum_position_rad=1.20,
    )

    assert position == pytest.approx(0.50)


def test_width_aware_close_position_is_clamped_to_safe_limit() -> None:
    position = CanGraspExecutionDemo._opening_to_gripper_position(
        required_opening_m=0.040,
        opening_reduction_m=0.010,
        reference_aperture_m=0.050,
        reference_position_rad=0.30,
        aperture_m_per_rad=0.10,
        minimum_position_rad=0.30,
        maximum_position_rad=1.20,
    )

    assert position == pytest.approx(0.30)


def test_gripper_stall_is_contact_only_after_closing_motion() -> None:
    detected = CanGraspExecutionDemo._is_gripper_contact_stall(
        start=1.20,
        actual=0.78,
        command=0.30,
        velocity=0.01,
        minimum_motion=0.10,
        minimum_residual=0.05,
        maximum_velocity=0.05,
    )
    still_moving = CanGraspExecutionDemo._is_gripper_contact_stall(
        start=1.20,
        actual=0.78,
        command=0.30,
        velocity=0.20,
        minimum_motion=0.10,
        minimum_residual=0.05,
        maximum_velocity=0.05,
    )
    barely_moved = CanGraspExecutionDemo._is_gripper_contact_stall(
        start=1.20,
        actual=1.15,
        command=0.30,
        velocity=0.0,
        minimum_motion=0.10,
        minimum_residual=0.05,
        maximum_velocity=0.05,
    )

    assert detected
    assert not still_moving
    assert not barely_moved
