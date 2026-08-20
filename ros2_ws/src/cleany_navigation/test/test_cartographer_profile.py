from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cartographer_profiles_differ_only_by_imu_use() -> None:
    base = (PACKAGE_ROOT / 'config/slam/cartographer_2d.lua').read_text(
        encoding='utf-8'
    )
    imu = (PACKAGE_ROOT / 'config/slam/cartographer_2d_imu.lua').read_text(
        encoding='utf-8'
    )

    assert 'use_odometry = true' in base
    assert 'published_frame = "odom"' in base
    assert 'use_imu_data = false' in base
    assert 'constraint_builder.max_constraint_distance = 1.0' in base
    assert 'cartographer_2d.lua' in imu
    assert 'TRAJECTORY_BUILDER_2D.use_imu_data = true' in imu
