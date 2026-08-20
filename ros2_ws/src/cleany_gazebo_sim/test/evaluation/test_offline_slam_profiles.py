from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
NAVIGATION_PACKAGE_ROOT = PACKAGE_ROOT.parent / 'cleany_navigation'


def test_offline_launches_republish_bag_odometry_as_tf_only():
    for name in (
        'evaluation_slam_toolbox_replay.launch.py',
        'evaluation_cartographer_replay.launch.py',
        'evaluation_rtabmap_replay.launch.py',
    ):
        launch = (PACKAGE_ROOT / 'launch' / name).read_text(encoding='utf-8')
        assert "'input_topic': '/odom'" in launch
        assert "'publish_odometry': False" in launch


def test_localization_replay_loads_a_fixed_posegraph():
    launch = (
        PACKAGE_ROOT
        / 'launch/evaluation_slam_toolbox_localization_replay.launch.py'
    ).read_text(encoding='utf-8')
    config = (
        NAVIGATION_PACKAGE_ROOT
        / 'config/slam/slam_toolbox_localization.yaml'
    ).read_text(encoding='utf-8')

    assert "executable='localization_slam_toolbox_node'" in launch
    assert "'map_file_name': LaunchConfiguration('posegraph')" in launch
    assert "'map_start_pose': [0.0, 0.0, 0.0]" in launch
    assert 'mode: localization' in config
    assert 'do_loop_closing: false' in config


def test_algorithm_profiles_use_external_wheel_odometry():
    cartographer = (
        NAVIGATION_PACKAGE_ROOT / 'launch/cartographer_mapping.launch.py'
    ).read_text(encoding='utf-8')
    rtabmap = (
        PACKAGE_ROOT / 'launch/evaluation_rtabmap_replay.launch.py'
    ).read_text(encoding='utf-8')

    assert "remappings=[('scan', '/scan'), ('imu', '/imu/data')]" in cartographer
    assert "'odom_frame_id': 'odom'" in rtabmap
    assert "'subscribe_scan': True" in rtabmap
    assert "remappings=[('scan', '/scan'), ('odom', '/odom')]" in rtabmap
