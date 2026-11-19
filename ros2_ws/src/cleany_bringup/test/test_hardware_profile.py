from pathlib import Path
import copy
import math
import pytest
import yaml
from cleany_bringup.hardware_profile import load_profile

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "cleany_bringup/config/mapping_legacy.yaml"


def share(name):
    return ROOT / name


def test_profiles_remain_distinct_and_resolve_by_package_index(tmp_path):
    legacy = load_profile(PROFILE, state_root=tmp_path, package_share=share)
    measured = load_profile(
        PROFILE.with_name("mapping_evaluation_20261015.yaml"),
        state_root=tmp_path,
        package_share=share,
    )
    assert legacy["mount"]["yaw_deg"] == -7.85
    assert measured["mount"]["yaw_deg"] == pytest.approx(79.77007003626845)
    assert (
        legacy["lidar"]["frame_id"] == "lidar_link"
        and measured["lidar"]["frame_id"] == "laser"
    )
    assert legacy["scan_filter"]["front_center_rad"] == pytest.approx(
        math.radians(7.85)
    )
    assert Path(legacy["base_config"]).is_file()
    assert legacy["recordings_directory"] == str(tmp_path / "recordings")


def test_invalid_mount_cannot_enter_operational_profile(tmp_path):
    data = yaml.safe_load(PROFILE.read_text())
    data["mount"]["yaw_deg"] = float("nan")
    broken = tmp_path / "invalid.yaml"
    broken.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_profile(broken, package_share=share)


def test_mock_always_selects_synthetic_geometry(tmp_path):
    profile = load_profile(PROFILE, mock=True, state_root=tmp_path, package_share=share)
    assert Path(profile["base_config"]).name == "base_synthetic.yaml"


@pytest.mark.parametrize('session,total,valid', [
    (9536, 14336, True), (10240, 10240, True),
    (10241, 14336, False), (9536, 1024, False), (True, 10240, False),
])
def test_recording_quota_profile(tmp_path, session, total, valid):
    data = yaml.safe_load(PROFILE.read_text())
    data.update(recording_limit_mib=session, recording_total_limit_mib=total)
    path = tmp_path / 'quota.yaml'
    path.write_text(yaml.safe_dump(data))
    if valid:
        assert load_profile(path, package_share=share)['recording_limit_mib'] == session
    else:
        with pytest.raises(ValueError):
            load_profile(path, package_share=share)


@pytest.mark.parametrize('half_angle,valid', [(90, True), (180, True), (0, False), (181, False), (float('nan'), False)])
def test_scan_coverage_profile(tmp_path, half_angle, valid):
    data = yaml.safe_load(PROFILE.read_text())
    data['scan_filter']['half_angle_deg'] = half_angle
    path = tmp_path / 'coverage.yaml'
    path.write_text(yaml.safe_dump(data))
    if valid:
        assert load_profile(path, package_share=share)['scan_filter']['half_angle_deg'] == half_angle
    else:
        with pytest.raises(ValueError):
            load_profile(path, package_share=share)
