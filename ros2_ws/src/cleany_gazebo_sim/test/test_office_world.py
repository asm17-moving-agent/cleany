from pathlib import Path
from xml.etree import ElementTree

from cleany_gazebo_sim.world_generator import (
    materialize_husarion_office_world,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
OFFICE_PACKAGE = PACKAGE_ROOT.parent / 'husarion_gz_worlds'
OFFICE_WORLD = OFFICE_PACKAGE / 'worlds' / 'husarion_office.sdf'
ROBOT_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'


def test_office_dependency_contains_expected_world_and_assets() -> None:
    assert OFFICE_WORLD.is_file()
    assert (OFFICE_PACKAGE / 'models' / 'Surfaces').is_dir()
    assert (OFFICE_PACKAGE / 'LICENSE').is_file()


def test_office_world_replaces_demo_robots_with_cleany(
    tmp_path: Path,
) -> None:
    generated = materialize_husarion_office_world(
        OFFICE_WORLD,
        ROBOT_WORLD,
        tmp_path / 'cleany_office.sdf',
        simulator='harmonic',
    )
    root = ElementTree.parse(generated).getroot()
    world = root.find("world[@name='cleany_husarion_office']")
    assert world is not None
    model_names = {
        model.get('name', '') for model in world.findall('model')
    }

    assert 'cleany_mecanum' in model_names
    assert 'Surfaces' in model_names
    assert 'OpenRobotics/AdjTable' in model_names
    assert not any(
        name.startswith('OpenRobotics/_Rosbot') for name in model_names
    )

    cleany = world.find("model[@name='cleany_mecanum']")
    assert cleany is not None
    assert cleany.findtext('pose') == (
        '5.49526 -8.97241 0.38 0.0 0.0 2.7409'
    )
    assert cleany.find("joint[@name='lidar_mount']") is not None
    assert cleany.find(
        "plugin[@filename='gz-sim-mecanum-drive-system']"
    ) is not None
    assert world.find(
        "plugin[@filename='gz-sim-sensors-system']"
    ) is not None
    assert world.find(
        "plugin[@filename='ignition-gazebo-sensors-system']"
    ) is None


def test_office_launch_uses_navigation_bridge_and_resource_paths() -> None:
    source = (
        PACKAGE_ROOT / 'launch' / 'gazebo_office.launch.py'
    ).read_text(encoding='utf-8')

    assert "get_package_share_directory('husarion_gz_worlds')" in source
    assert "'navigation_bridge_harmonic.yaml'" in source
    assert "'gazebo_harmonic.launch.py'" in source
    assert "'GZ_SIM_RESOURCE_PATH'" in source
    assert "SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0')" in source
