from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cartographer_profiles_differ_only_by_imu_use():
    base = (PACKAGE_ROOT / 'config/cartographer_2d.lua').read_text(
        encoding='utf-8'
    )
    imu = (PACKAGE_ROOT / 'config/cartographer_2d_imu.lua').read_text(
        encoding='utf-8'
    )

    assert 'use_odometry = true' in base
    assert 'published_frame = "odom"' in base
    assert 'use_imu_data = false' in base
    assert 'constraint_builder.max_constraint_distance = 1.0' in base
    assert 'cartographer_2d.lua' in imu
    assert 'TRAJECTORY_BUILDER_2D.use_imu_data = true' in imu


def test_offline_launches_republish_bag_odometry_as_tf_only():
    for name in (
        'slam_toolbox_replay.launch.py',
        'cartographer_mapping.launch.py',
        'rtabmap_mapping.launch.py',
    ):
        launch = (PACKAGE_ROOT / 'launch' / name).read_text(encoding='utf-8')
        assert "'input_topic': '/odom'" in launch
        assert "'publish_odometry': False" in launch


def test_localization_replay_loads_a_fixed_posegraph():
    launch = (
        PACKAGE_ROOT / 'launch/slam_toolbox_localization_replay.launch.py'
    ).read_text(encoding='utf-8')
    config = (
        PACKAGE_ROOT / 'config/slam_toolbox_localization.yaml'
    ).read_text(encoding='utf-8')

    assert "executable='localization_slam_toolbox_node'" in launch
    assert "'map_file_name': LaunchConfiguration('posegraph')" in launch
    assert "'map_start_pose': [0.0, 0.0, 0.0]" in launch
    assert 'mode: localization' in config
    assert 'do_loop_closing: false' in config


def test_algorithm_profiles_use_external_wheel_odometry():
    cartographer = (
        PACKAGE_ROOT / 'launch/cartographer_mapping.launch.py'
    ).read_text(encoding='utf-8')
    rtabmap = (PACKAGE_ROOT / 'launch/rtabmap_mapping.launch.py').read_text(
        encoding='utf-8'
    )

    assert "remappings=[('scan', '/scan'), ('imu', '/imu/data')]" in cartographer
    assert "'odom_frame_id': 'odom'" in rtabmap
    assert "'subscribe_scan': True" in rtabmap
    assert "remappings=[('scan', '/scan'), ('odom', '/odom')]" in rtabmap
