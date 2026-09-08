from pathlib import Path

import pytest

from cleany_moveit_config.collision_scene import load_collision_scene


PACKAGE_ROOT = Path(__file__).parents[1]


def test_study_cafe_grasp_collision_scene_keeps_target_dynamic() -> None:
    spec = load_collision_scene(
        PACKAGE_ROOT / 'config' / 'study_cafe_grasp_collision_objects.yaml'
    )

    objects = {item.id: item for item in spec.objects}
    assert set(objects) == {
        'study_cafe_robot_desk',
        'study_cafe_partition',
        'study_cafe_monitor',
        'study_cafe_cup_obstacle',
        'study_cafe_wallet_obstacle',
        'study_cafe_tissue_obstacle',
    }
    assert 'study_cafe_lego' not in objects
    assert objects['study_cafe_robot_desk'].primitives[
        0
    ].dimensions_m == pytest.approx(
        (0.77, 1.20, 0.04)
    )
