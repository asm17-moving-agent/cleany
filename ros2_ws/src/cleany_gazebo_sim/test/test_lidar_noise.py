from pathlib import Path
from xml.etree import ElementTree

import pytest

from cleany_gazebo_sim.lidar_noise import load_lidar_noise_profile
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world
from cleany_gazebo_sim.world.generator import materialize_study_cafe_world


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILES_PATH = PACKAGE_ROOT / 'config' / 'lidar_noise_profiles.yaml'
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_fortress.sdf'


@pytest.mark.parametrize(
    ('name', 'stddev'),
    [('measured', 0.0025), ('stress', 0.01)],
)
def test_noise_profile_materializes_expected_gaussian_noise(
    name: str, stddev: float
) -> None:
    profile = load_lidar_noise_profile(PROFILES_PATH, name)
    world_path = materialize_mecanum_wheel_world(WORLD_PATH, profile)
    root = ElementTree.parse(world_path).getroot()
    noise = root.find(
        ".//link[@name='lidar_link']/sensor[@name='rplidar_a1']/lidar/noise"
    )
    assert noise is not None
    assert noise.findtext('type') == 'gaussian'
    assert float(noise.findtext('mean', 'nan')) == 0.0
    assert float(noise.findtext('stddev', 'nan')) == stddev


def test_unknown_noise_profile_names_available_choices() -> None:
    with pytest.raises(ValueError, match='measured, stress'):
        load_lidar_noise_profile(PROFILES_PATH, 'unknown')


def test_study_cafe_world_keeps_selected_noise_profile(tmp_path: Path) -> None:
    profile = load_lidar_noise_profile(PROFILES_PATH, 'stress')
    world_path = materialize_study_cafe_world(
        WORLD_PATH,
        tmp_path / 'study_cafe.sdf',
        lidar_noise=profile,
    )
    root = ElementTree.parse(world_path).getroot()
    noise = root.find(
        ".//link[@name='lidar_link']/sensor[@name='rplidar_a1']/lidar/noise"
    )
    assert noise is not None
    assert float(noise.findtext('stddev', 'nan')) == 0.01
