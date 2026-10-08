#!/usr/bin/env python3

"""Keep the hand-eye executable's original default collision scene."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from cleany_moveit_config.apply_collision_scene import main


if __name__ == '__main__':
    main(
        scene_config_default=str(
            Path(get_package_share_directory('cleany_moveit_config'))
            / 'config'
            / 'handeye_collision_objects.yaml'
        )
    )
