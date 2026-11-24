"""Verify which field inputs survive preparation and release changes."""

import runpy
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
prepare = runpy.run_path(str(ROOT / "tools/ros2/floor19_session/prepare.py"))["prepare"]


def test_template_keeps_calibration_and_pins_release(tmp_path):
    source = ROOT / "ros2_ws/src/cleany_bringup/config/mapping_cad_lidar_front.yaml"
    original = yaml.safe_load(source.read_text())
    release = tmp_path / "release"
    release.mkdir()
    current = tmp_path / "current"
    current.symlink_to(release)
    target = tmp_path / "survey"
    prepare(source, current, target)
    saved = yaml.safe_load((target / "config/profile.yaml").read_text())
    assert saved["lidar"] == original["lidar"]
    assert saved["mount"] == original["mount"]
    assert saved["scan_filter"]["half_angle_deg"] == 180
    assert (target / "release.txt").read_text().strip() == str(release)
    for name in [
        "session",
        "mapping.launch.py",
        "config/cartographer_hardware_2d.lua",
        "config/vslam-realsense.launch.py",
        "config/record-topics.txt",
    ]:
        assert (target / name).is_file()
    with pytest.raises(FileExistsError):
        prepare(source, current, target)
