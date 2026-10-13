from pathlib import Path

import pytest
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[5]
CONFIG_ROOT = REPOSITORY_ROOT / 'ros2_ws/src/cleany_gazebo_sim/config'
TOOLS_ROOT = REPOSITORY_ROOT / 'tools/slam_evaluation'


def parameters(name: str) -> dict[str, float]:
    document = yaml.safe_load(
        (CONFIG_ROOT / name).read_text(encoding='utf-8')
    )
    return document['simulated_odometry_error']['ros__parameters']


@pytest.mark.parametrize(
    ('middle_name', 'fraction'),
    (
        ('odometry_error_level1.yaml', 1.0 / 3.0),
        ('odometry_error_level2.yaml', 2.0 / 3.0),
    ),
)
def test_intermediate_profiles_interpolate_ideal_and_stress(
    middle_name: str, fraction: float
) -> None:
    ideal = parameters('odometry_error_ideal.yaml')
    middle = parameters(middle_name)
    stress = parameters('odometry_error_stress.yaml')
    ignored = {'input_topic', 'output_topic', 'random_seed'}

    for key in ideal.keys() - ignored:
        expected = float(ideal[key]) + fraction * (
            float(stress[key]) - float(ideal[key])
        )
        assert float(middle[key]) == pytest.approx(expected, abs=1e-9)
    assert middle['random_seed'] == ideal['random_seed'] == stress['random_seed']


def test_odometry_noise_tools_define_the_two_by_four_matrix() -> None:
    materializer = (
        TOOLS_ROOT / 'materialize_odometry_noise_bags.py'
    ).read_text(encoding='utf-8')
    runner = (TOOLS_ROOT / 'run_odometry_noise_comparison.sh').read_text(
        encoding='utf-8'
    )
    renderer = (
        TOOLS_ROOT / 'render_odometry_noise_map_matrix.py'
    ).read_text(encoding='utf-8')

    for level in ('level0', 'level1', 'level2', 'level3'):
        assert level in materializer
        assert level in runner
        assert level in renderer
    assert 'slam_toolbox cartographer' in runner
    assert 'map_matrix_2x4.png' in renderer
    assert 'refusing to overwrite' in materializer
    assert '--source-odom-topic' in materializer
    assert 'ODOMETRY_EXPERIMENT_NAME' in runner
    assert 'evaluation_slam_toolbox_live_replay.launch.py' in runner
    assert 'extract_latest_occupancy_map.py' in runner
