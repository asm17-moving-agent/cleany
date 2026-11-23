"""Cold boot checks run without USB devices, DDS or a production database."""

from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import sqlite3

import pytest


@pytest.fixture
def field(tmp_path, monkeypatch):
    spec = spec_from_file_location("field_service", Path(__file__).resolve().parents[2] / "tools/ros2/field_service.py")
    field = module_from_spec(spec)
    spec.loader.exec_module(field)
    for key in ("STATE", "CONFIG", "RUN"):
        monkeypatch.setattr(field, key, tmp_path / key)
    monkeypatch.setattr(field, "TARGETS", field.CONFIG / "field-targets.json")
    monkeypatch.setattr(field, "COLLECTION", field.CONFIG / "field-collection.json")
    monkeypatch.setattr(field, "TOKEN", field.CONFIG / "token")
    monkeypatch.setenv("CLEANY_DATABASE_PATH", str(tmp_path / "backend/db"))
    return field


def test_missing_field_settings_do_not_start_mission(field):
    assert len(field.blockers("mission")) == 3
    field.write_status("mission", field.blockers("mission"))
    record = json.loads((field.RUN / "mission.json").read_text())
    assert record["updated"] > 0 and record["blockers"] == field.blockers("mission")


def test_empty_grippers_and_both_measured_stow_poses_required(field):
    arm = {"home_joint_rad": [0] * 5, "bins": {"demo_trash_bin": [0] * 5}, "gripper": {"open_joint_rad": 2}}
    cfg = {"arms": {"left": arm, "right": arm}, "operation_mode": "full_collection",
           "commissioning_verified": True, "label_destinations": {"trash": "demo_trash_bin"}}
    parked = field.park_pose(cfg)
    assert len(parked) == 12 and parked["right_gripper_joint"] == 2
    arm["gripper"]["open_joint_rad"] = float("nan")
    with pytest.raises(ValueError):
        field.park_pose(cfg)


def test_arm_boot_waits_for_verified_interrupted_seat(field):
    backend = field.dashboard_database()
    backend.parent.mkdir(parents=True)
    field.STATE.mkdir()
    with sqlite3.connect(backend) as db:
        db.executescript("""
            CREATE TABLE missions(mission_id, robot_id, active, payload);
            CREATE TABLE field_recovery_confirmations(mission_id);
            INSERT INTO missions VALUES('seat02','cleany-01',1,'{}');
        """)
    with pytest.raises(ValueError):
        field.check_arm_journals()
    with sqlite3.connect(backend) as db:
        db.execute("UPDATE missions SET active=0,payload=?", (json.dumps({"outcome": "INTERRUPTED"}),))
    with sqlite3.connect(field.STATE / "desk-field-missions.db") as db:
        db.execute("CREATE TABLE metadata(key,value)")
        db.execute("CREATE TABLE missions(mission_id,report)")
        db.execute("INSERT INTO missions VALUES('seat02',?)", (json.dumps({"outcome": "INTERRUPTED"}),))
    with pytest.raises(ValueError):
        field.check_arm_journals()
    with sqlite3.connect(backend) as db:
        db.execute("INSERT INTO field_recovery_confirmations VALUES('seat02')")
    field.check_arm_journals()
    with sqlite3.connect(field.STATE / "desk-field-missions.db") as db:
        db.execute("INSERT INTO metadata VALUES('error_code','E_STOP')")
    with pytest.raises(ValueError, match="비상정지"):
        field.check_arm_journals()


def test_missing_recovery_table_only_blocks_active_or_interrupted(field):
    backend = field.dashboard_database()
    backend.parent.mkdir(parents=True)
    with sqlite3.connect(backend) as db:
        db.execute("CREATE TABLE missions(mission_id, robot_id, active, payload)")
    field.check_arm_journals()
    with sqlite3.connect(backend) as db:
        db.execute("INSERT INTO missions VALUES(?, ?, 0, ?)",
                   ("seat02", "cleany-01", json.dumps({"outcome": "INTERRUPTED"})))
    with pytest.raises(ValueError, match="현장에서"):
        field.check_arm_journals()


def test_arm_power_up_requires_the_arm_owners_exact_recovery_record(field):
    from dataclasses import replace
    from cleany_manipulation_bt.store import BTExecutionStore
    from cleany_skill_executor.manipulation.models import Goal, ObjectState, Record, RecordState

    path = field.STATE / "manipulation_jetson/executions.sqlite3"
    store = BTExecutionStore(str(path))
    goal = Goal("mission-02", "trash", "action-1", "pick", "scene-1", 1, "bin")
    running = Record(goal=goal, execution_profile="jetson")
    try:
        store.save(running, create=True)
        with pytest.raises(ValueError, match="도중 중단"):
            field.check_execution_journal()
        held = replace(running, record_state=RecordState.INTERRUPTED, object_state=ObjectState.HELD,
                       revision=running.revision + 1)
        store.save(held)
        with pytest.raises(ValueError, match="현장 팔 복구"):
            field.check_execution_journal()
        store.confirm_recovery(held, "field inspection")
        field.check_execution_journal()
        store.save(replace(held, revision=held.revision + 1))
        with pytest.raises(ValueError, match="현장 팔 복구"):
            field.check_execution_journal()
    finally:
        store.close()


def test_rgbd_selects_the_calibrated_serial_before_launch(field, monkeypatch, tmp_path):
    monkeypatch.setattr(field, "REPO", tmp_path)
    assert field.blockers("rgbd")
    profile = tmp_path / "ros2_ws/src/cleany_perception/config"
    profile.mkdir(parents=True)
    (profile / "head_rgb_intrinsics.yaml").write_text("serial_no: '12345'")
    (profile / "realsense_pregrasp.yaml").touch()
    field.CONFIG.mkdir()
    (field.CONFIG / "head-rgb-intrinsics.json").touch()
    assert field.blockers("rgbd") == []
    assert "serial_no:=12345" in field.command("rgbd")


@pytest.mark.parametrize("role", ["rgbd", "perception"])
def test_native_sensor_units_are_only_observed(field, monkeypatch, role):
    observed = []
    monkeypatch.setattr(field.sys, "argv", ["field-service", role, "--observe-native-sensors"])
    monkeypatch.setattr(field.os, "environ", {"CYCLONEDDS_URI": "obsolete-profile"})
    monkeypatch.setattr(field, "FIELD_RMW", "rmw_fastrtps_cpp")
    monkeypatch.setattr(field, "blockers", lambda _: [])
    monkeypatch.setattr(field, "watch", observed.append)
    monkeypatch.setattr(field, "assert_no_owner", lambda _: pytest.fail("Tried to claim a native sensor"))
    monkeypatch.setattr(field.subprocess, "Popen", lambda *a, **kw: pytest.fail("Started a second driver"))
    assert field.main() == 0
    assert observed == [role]
    assert field.os.environ["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert field.os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] == str(field.DDS)
    assert "CYCLONEDDS_URI" not in field.os.environ


def test_native_monitoring_keeps_navigation_ownership_check(field, monkeypatch):
    def occupied(role):
        assert role == "navigation"
        raise ValueError("existing driver")

    monkeypatch.setattr(field.sys, "argv", ["field-service", "navigation", "--observe-native-sensors"])
    monkeypatch.setattr(field.os, "environ", {})
    monkeypatch.setattr(field, "blockers", lambda _: [])
    monkeypatch.setattr(field, "assert_no_owner", occupied)
    assert field.main() == 1
    assert json.loads((field.RUN / "navigation.json").read_text())["blockers"] == ["existing driver"]

def test_field_interlock_failure_prevents_real_control_start(tmp_path):
    import subprocess

    runtime = Path(__file__).resolve().parents[2] / "tools/collection-runtime"
    functions, dispatch, _ = runtime.read_text().partition('\ncommand="${1:-}"\n')
    assert dispatch
    check = tmp_path / "arm-check"
    check.write_text("#!/bin/sh\ntest \"$*\" = 'collection --arm-check' || exit 42\nexit 7\n")
    check.chmod(0o755)
    invocation = functions + """
start_group() { exit 43; }
CLEANY_FIELD_PARK_JOINTS=measured
CLEANY_FIELD_ARM_CHECK=$1
start_real_control /dev/null
"""
    result = subprocess.run(["bash", "-c", invocation, str(runtime), str(check)])
    assert result.returncode == 7
