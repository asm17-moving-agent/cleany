from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FORTRESS_BRIDGE = PACKAGE_ROOT / 'config' / 'navigation_bridge.yaml'
HARMONIC_BRIDGE = (
    PACKAGE_ROOT / 'config' / 'navigation_bridge_harmonic.yaml'
)
FORTRESS_LAUNCH = PACKAGE_ROOT / 'launch' / 'gazebo_sim.launch.py'
HARMONIC_LAUNCH = PACKAGE_ROOT / 'launch' / 'gazebo_harmonic.launch.py'
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]


def _bridge_topics(path: Path) -> set[str]:
    return {
        line.split(':', 1)[1].strip().strip('"')
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.startswith('- ros_topic_name:')
    }


def test_navigation_bridges_only_enable_runtime_contract_topics():
    expected_topics = {
        '/gazebo_cmd_vel',
        '/gazebo_odom',
        '/clock',
        '/scan',
        '/imu/data',
    }

    assert _bridge_topics(FORTRESS_BRIDGE) == expected_topics
    assert _bridge_topics(HARMONIC_BRIDGE) == expected_topics
    assert '/camera/' not in FORTRESS_BRIDGE.read_text(encoding='utf-8')
    assert '/camera/' not in HARMONIC_BRIDGE.read_text(encoding='utf-8')


def test_navigation_bridges_use_profile_specific_transport_types():
    fortress = FORTRESS_BRIDGE.read_text(encoding='utf-8')
    harmonic = HARMONIC_BRIDGE.read_text(encoding='utf-8')

    assert 'ignition.msgs.' in fortress
    assert 'gz.msgs.' not in fortress
    assert 'gz.msgs.' in harmonic
    assert 'ignition.msgs.' not in harmonic


def test_launch_profiles_accept_a_bridge_config_override():
    for launch_path in (FORTRESS_LAUNCH, HARMONIC_LAUNCH):
        launch = launch_path.read_text(encoding='utf-8')
        assert "DeclareLaunchArgument(\n        'bridge_config'" in launch
        assert "LaunchConfiguration('bridge_config')" in launch


def test_makefile_exposes_navigation_runtime_test():
    makefile = (REPOSITORY_ROOT / 'Makefile').read_text(encoding='utf-8')

    assert 'test-gazebo-nav-runtime:' in makefile
    assert 'test_runtime_navigation.py' in makefile
    assert '--run-sim-runtime' in makefile
