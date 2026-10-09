from pathlib import Path

import pytest
import yaml

from prepare_profile import prepare


def test_preparation_preserves_source_symlink_configs_and_unlisted_guards(tmp_path: Path):
    base = tmp_path/'baseline'
    (base/'config').mkdir(parents=True)
    original = tmp_path/'original-navigation.yaml'
    safety = {'controller_server': {'ros__parameters': {
        'goal_checker': {'xy_goal_tolerance': 0.03}, 'FollowPath': {'max_vel_x': 0.15}}},
        'reactive_guard': {'ros__parameters': {'input_timeout': 0.6}}}
    original.write_text(yaml.safe_dump(safety))
    (base/'config/nav2_safety.yaml').symlink_to(original)
    (base/'config/wheel_odometry.yaml').write_text('wheel_odometry: {ros__parameters: {wheel_radius_m: 0.0635}}')
    (base/'world.sdf').write_text('<sdf/>')
    profile = tmp_path/'profile.yaml'
    profile.write_text(yaml.safe_dump({
        'config/wheel_odometry.yaml': {'wheel_odometry': {'ros__parameters': {'yaw_scale': 1.047}}},
        'config/nav2_safety.yaml': {'controller_server': {'ros__parameters': {'FollowPath': {'PathProgress.scale': 20.0}}}}}))
    before = {p: p.read_bytes() for p in (original, base/'config/wheel_odometry.yaml', base/'world.sdf')}
    output = tmp_path/'prepared'
    prepare(base, output, profile)
    assert all(path.read_bytes() == raw for path, raw in before.items())
    assert not (output/'config/nav2_safety.yaml').is_symlink()
    prepared = yaml.safe_load((output/'config/nav2_safety.yaml').read_text())
    del prepared['controller_server']['ros__parameters']['FollowPath']['PathProgress.scale']
    assert prepared == safety
    assert (output/'world.sdf').resolve() == (base/'world.sdf').resolve()
    with pytest.raises(FileExistsError):
        prepare(base, output, profile)
    assert all(path.read_bytes() == raw for path, raw in before.items())


def test_bad_profile_cannot_create_output_or_write_outside_configs(tmp_path: Path):
    base = tmp_path/'baseline'
    base.mkdir()
    profile = tmp_path/'profile.yaml'
    profile.write_text('../outside.yaml: {overwrite: true}')
    output = tmp_path/'prepared'
    with pytest.raises(ValueError):
        prepare(base, output, profile)
    assert not output.exists()
    with pytest.raises(ValueError):
        prepare(base, base/'nested', profile)
