"""Versioned deployment target files never promote UI drafts to robot goals."""
import hashlib
import json

import pytest

from cleany_mission_manager.adapters.config import load_targets


def write_catalog(tmp_path, **changes):
    data = dict(schema_version=2, map_id="slam-test", frame_id="map", source_sha256="a"*64,
                layout_revision=3, validation_status="field_verified", home=[0., 0., 0.],
                seats={"seat-01": [1., 2., 0.], "seat-a1-01": [-2., 3., 1.5],
                       "seat-m1-01": [4., -3., -1.5]})
    data.update(changes)
    data["target_digest"] = hashlib.sha256(json.dumps(
        data, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()
    path = tmp_path / "targets.json"
    path.write_text(json.dumps(data))
    return path, data


def test_load_three_representative_targets_and_relay_exact_binding(tmp_path):
    from cleany_control_bridge.session import BridgeSession
    path, data = write_catalog(tmp_path)
    targets = load_targets(str(path))
    assert targets.seats["seat-01"].y == 2
    assert targets.seats["seat-a1-01"].x == -2
    assert targets.seats["seat-m1-01"].yaw == -1.5
    assert targets.binding["target_digest"] == data["target_digest"]
    session = BridgeSession(str(tmp_path / "bridge.db"))
    session.observe({"boot_id": "catalog-test", "robot_state": "IDLE", "ready": True,
                     "active_request": None, "phase": "", "state": "IDLE", "bt_stage": "",
                     "supported_seat_ids": list(targets.seats), "last_result": None,
                     "execution_profile": dict.fromkeys(("navigation", "perception", "planning", "execution"), "mock"),
                     "target_map": targets.binding})
    assert session.sync_event()["payload"]["target_map"] == targets.binding
    session.close()


def test_draft_and_missing_home_are_rejected(tmp_path):
    path, _ = write_catalog(tmp_path, validation_status="draft", home=None)
    with pytest.raises(ValueError, match="field verification"):
        load_targets(str(path))
    path, _ = write_catalog(tmp_path, home=None)
    with pytest.raises(ValueError, match="pose must be finite"):
        load_targets(str(path))


def test_coordinate_tampering_rejected_before_navigation(tmp_path):
    path, data = write_catalog(tmp_path)
    data["seats"]["seat-01"][0] = 20
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="digest mismatch"):
        load_targets(str(path))


def test_browser_json_integer_roundtrip_preserves_pose_digest(tmp_path):
    path, data = write_catalog(tmp_path)
    data["home"] = [0, 0, 0]
    data["seats"]["seat-01"] = [1, 2, 0]
    path.write_text(json.dumps(data))
    assert load_targets(str(path)).home.x == 0
