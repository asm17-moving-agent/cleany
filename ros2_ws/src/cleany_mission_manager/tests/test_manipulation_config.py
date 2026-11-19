"""Reject mismatched logical desk fixtures before composing a simulation."""

from pathlib import Path

import pytest
import yaml

from cleany_mission_manager.adapters.config import load_runtime_parameters, validate_manipulation_fixture


CONFIG = Path(__file__).parents[1] / "config"


def test_gazebo_action_profile_matches_shared_desk_fixture():
    parameters = load_runtime_parameters(str(CONFIG / "runtime_gazebo_manipulation_mock.yaml"))
    fixture = validate_manipulation_fixture(parameters, str(CONFIG / "manipulation_server_fixture.yaml"),
                                            "success")
    assert parameters["navigation_backend"] == "nav2"
    assert parameters["manipulation_backend"] == "action_mock"
    assert fixture["snapshots"]["mock-snapshot-004"]["object_ids"] == []


def test_observation_timing_fits_operation_cleaning_and_cancel_budgets():
    parameters = load_runtime_parameters(str(CONFIG / "runtime_gazebo_manipulation_mock.yaml"))
    fixture = yaml.safe_load((CONFIG / "manipulation_server_fixture.yaml").read_text())
    recovery = {"RELEASING_IN_PLACE", "RECOVERING_ARM"}
    collection_duration = sum(value for stage, value in fixture["stage_durations_sec"].items()
                              if stage not in recovery)
    assert parameters["operation_timeout"] > collection_duration + fixture["stop_timeout_sec"]
    assert parameters["cleaning_timeout"] > 2 * collection_duration
    assert parameters["cancel_timeout"] > fixture["motion_timeout_sec"] + fixture["stop_timeout_sec"]


@pytest.mark.parametrize("change", ["snapshot", "destination", "scenario", "wire_id", "empty"])
def test_mismatched_fixture_is_rejected(tmp_path, change):
    parameters = load_runtime_parameters(str(CONFIG / "runtime_gazebo_manipulation_mock.yaml"))
    fixture = yaml.safe_load((CONFIG / "manipulation_server_fixture.yaml").read_text())
    scenario = "missing" if change == "scenario" else "success"
    if change == "snapshot":
        parameters["manipulation_mock_snapshot_ids"][1] = "unregistered"
    if change == "destination":
        fixture["destination_id"] = "other_bin"
    if change == "wire_id":
        fixture["snapshots"]["mock-snapshot-001"]["object_ids"] = [True, 2]
    if change == "empty":
        fixture["snapshots"]["mock-snapshot-004"]["object_ids"] = [2]
    path = tmp_path / "fixture.yaml"
    path.write_text(yaml.safe_dump(fixture))
    with pytest.raises(ValueError):
        validate_manipulation_fixture(parameters, str(path), scenario)
