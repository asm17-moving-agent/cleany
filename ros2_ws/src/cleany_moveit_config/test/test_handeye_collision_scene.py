from pathlib import Path

import pytest

from cleany_moveit_config.collision_scene import (
    build_planning_scene,
    load_collision_scene,
)


PACKAGE_ROOT = Path(__file__).parents[1]


def test_collision_object_config_builds_explicit_primitive_poses() -> None:
    spec = load_collision_scene(
        PACKAGE_ROOT / 'config' / 'handeye_collision_objects.yaml'
    )
    assert spec.planning_frame == 'base_link'
    assert tuple(item.id for item in spec.objects) == (
        'handeye_table',
        'handeye_target_stand',
        'charuco_target',
    )
    scene = build_planning_scene(spec)
    assert scene.is_diff
    assert scene.robot_state.is_diff
    assert len(scene.world.collision_objects) == 3
    for collision_object in scene.world.collision_objects:
        assert collision_object.header.frame_id == 'base_link'
        assert len(collision_object.primitives) == len(
            collision_object.primitive_poses
        )
        assert len(collision_object.primitives) > 0
        assert all(
            primitive.type == primitive.BOX
            for primitive in collision_object.primitives
        )

    target = scene.world.collision_objects[-1]
    assert target.id == 'charuco_target'
    assert target.primitives[0].dimensions == pytest.approx(
        (0.210, 0.150, 0.006)
    )
    target_pose = target.primitive_poses[0]
    assert (
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
    ) == pytest.approx((0.653, 0.180, 0.400))
    assert (
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ) == pytest.approx((0.5, -0.5, -0.5, 0.5))
