from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[5]
SCRIPT = REPOSITORY_ROOT / 'tools' / 'slam_evaluation' / 'record_slam_input.sh'
RUN_SCRIPT = (
    REPOSITORY_ROOT
    / 'tools'
    / 'slam_evaluation'
    / 'run_slam_algorithm_comparison.sh'
)
SEGMENTED_SCRIPT = (
    REPOSITORY_ROOT
    / 'tools'
    / 'slam_evaluation'
    / 'record_segmented_slam_input.sh'
)
MERGE_SCRIPT = (
    REPOSITORY_ROOT
    / 'tools'
    / 'slam_evaluation'
    / 'merge_segmented_bags.py'
)


def test_recording_script_separates_noise_inputs_and_uses_two_ms_physics(
) -> None:
    source = SCRIPT.read_text(encoding='utf-8')

    assert '{measured|stress}' in source
    assert 'algorithm_compare_inputs/$noise_profile/' in source
    assert 'lidar_noise_profile:="$noise_profile"' in source
    assert 'physics_max_step_size:=0.002' in source
    assert 'physics_real_time_factor:=2.0' in source
    assert 'GAZEBO_SENSOR_RENDER_ENGINE:-ogre2' in source
    assert 'GAZEBO_SERVER_RENDER_ENGINE:-ogre2' in source
    assert 'invalid LiDAR scan: range spread' in source
    assert '--storage sqlite3' in source
    assert 'rosbag recorder failed to start' in source


def test_slam_run_script_separates_noise_outputs_and_records_sqlite() -> None:
    source = RUN_SCRIPT.read_text(encoding='utf-8')

    assert 'input_root/$noise_profile/input_' in source
    assert 'run_root/$noise_profile/$algorithm/' in source
    assert '--storage sqlite3' in source
    assert '--topics /map' not in source
    assert 'rosbag recorder failed to start' in source


def test_segmented_recording_restarts_each_route_edge_and_merges() -> None:
    source = SEGMENTED_SCRIPT.read_text(encoding='utf-8')
    merge_source = MERGE_SCRIPT.read_text(encoding='utf-8')

    assert 'record_slam_input.sh' in source
    assert 'expected 16 completed segments' in source
    assert 'ROBOT_SPAWN_POSE=' in source
    assert 'ROUTE_CONFIG=' in source
    assert 'SEGMENT_GAP_NS = 2_000_000' in merge_source
    assert "topic == '/clock'" in merge_source
    assert "topic == '/tf_static'" in merge_source
