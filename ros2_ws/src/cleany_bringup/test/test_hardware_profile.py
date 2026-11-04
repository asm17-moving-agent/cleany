from pathlib import Path
import copy
import json
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


def test_cad_lidar_front_profile_matches_description_coordinates(tmp_path):
    profile = load_profile(
        PROFILE.with_name("mapping_cad_lidar_front.yaml"),
        state_root=tmp_path,
        package_share=share,
    )
    manifest = json.loads(
        (ROOT / "cleany_description/config/geometry/freecad_final.json").read_text()
    )
    lidar = next(p for p in manifest["parts"] if p["name"] == "RPLidarA1M8")
    x, y, z = profile["mount"]["translation_m"]
    lower, upper = lidar["bounds_base_link_m"]
    assert 0 < lower[0] < x < upper[0]
    assert (x, y, z) == pytest.approx((0.139275, 0.0, 0.0))
    assert profile["mount"]["yaw_deg"] == 180.0
    assert profile["scan_filter"]["front_center_rad"] == pytest.approx(-math.pi)
    assert manifest["source"]["sha256"] in profile["mount"]["source"]
