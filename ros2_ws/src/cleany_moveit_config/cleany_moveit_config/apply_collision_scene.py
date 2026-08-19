#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node

from cleany_moveit_config.collision_scene import (
    CollisionSceneError,
    build_planning_scene,
    load_collision_scene,
)


def _default_config_path() -> str:
    return str(
        Path(get_package_share_directory('cleany_moveit_config'))
        / 'config'
        / 'handeye_collision_objects.yaml'
    )


class CollisionSceneApplier(Node):
    def __init__(self) -> None:
        super().__init__('handeye_collision_scene_applier')
        self.declare_parameter('scene_config', _default_config_path())
        self.declare_parameter('service_wait_timeout_sec', 60.0)

    def apply(self) -> bool:
        config_path = Path(
            self.get_parameter('scene_config').get_parameter_value().string_value
        )
        timeout = (
            self.get_parameter('service_wait_timeout_sec')
            .get_parameter_value()
            .double_value
        )
        try:
            spec = load_collision_scene(config_path)
        except CollisionSceneError as error:
            self.get_logger().error(str(error))
            return False
        client = self.create_client(ApplyPlanningScene, spec.apply_service)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f'timed out waiting for {spec.apply_service}'
            )
            return False
        request = ApplyPlanningScene.Request()
        request.scene = build_planning_scene(spec)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            self.get_logger().error('ApplyPlanningScene request failed')
            return False
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error('MoveIt rejected hand-eye collision objects')
            return False
        self.get_logger().info(
            'Applied hand-eye collision objects: '
            + ', '.join(item.id for item in spec.objects)
        )
        return True


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CollisionSceneApplier()
    try:
        succeeded = node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not succeeded:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
