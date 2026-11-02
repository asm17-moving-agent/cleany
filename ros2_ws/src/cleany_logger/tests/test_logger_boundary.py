"""Use the recording engine without the web application or ROS Python runtime."""

import subprocess
import sys
from pathlib import Path

from cleany_logger import Recorder, RecordingControls
from cleany_logger.recording import defaults


def test_logger_runs_without_monitor_http_or_ros_on_the_import_path(tmp_path):
    source = Path(__file__).resolve().parents[1]
    subprocess.run(
        [
            sys.executable, "-I", "-S", "-c",
            "import sys; sys.path.insert(0, sys.argv[1]); "
            "from cleany_logger import Recorder, RecordingControls, RawMessage; "
            "from cleany_logger.manipulation import ManipulationObserver; "
            "from cleany_logger.observation import ObservationBuffer; "
            "recorder = Recorder(sys.argv[2], controls=RecordingControls()); "
            "assert recorder.db is not None; recorder.close(); "
            "assert not any(name in sys.modules for name in "
            "('cleany_dev_monitor', 'aiohttp', 'rclpy', 'rosbag2_py', "
            "'cleany_mission_manager'))",
            str(source), str(tmp_path),
        ],
        check=True,
    )


def test_producer_snapshots_reopen_with_the_same_session_and_limits(tmp_path):
    recorder = Recorder(tmp_path)
    row = dict(seq=1, kind="event", key="mission", received_at=12.0,
               ros_time=2.0, data={"kind": "action_result", "result": "FAILED"})
    recorder.append(dict(sequence=1, rows=[row]))
    session = recorder.session_id
    recorder.close()

    # Another embedding application can read the archive without the old Hub.
    reader = Recorder(tmp_path)
    assert reader.replay(session, 12.0)["rows"] == [row]
    assert reader.replay(session, 11.0)["rows"] == []
    evidence = tmp_path / "bags" / session / "large.db3"
    evidence.parent.mkdir(parents=True)
    evidence.write_bytes(b"x" * 100_000)
    reader.total_limit = reader.disk_bytes()
    reader.append(dict(sequence=1, rows=[row]))
    assert "size limit" in reader.reason
    assert evidence.read_bytes() == b"x" * 100_000
    assert reader.replay(session, 12.0)["rows"] == [row]
    reader.close()


def test_policy_and_session_rotation_are_owned_by_the_logger(tmp_path):
    controls = RecordingControls()
    recorder = Recorder(tmp_path, controls=controls)
    previous_session = recorder.session_id
    settings = defaults()
    settings["limit_mib"] = 20
    settings["sources"]["path_topic"] = "/custom/plan"
    recorder.persist_settings(settings)
    recorder.restore_settings({})
    assert controls.settings["sources"]["path_topic"] == "/custom/plan"
    assert recorder.session_limit == 20 * 1024**2
    controls.writer_error = "previous writer unavailable"
    with controls.transition():
        recorder.append_raw()
        recorder.start_session()
    assert recorder.session_id != previous_session
    assert recorder.session_limit == 20 * 1024**2
    assert controls.writer_error == recorder.reason == ""
    assert (tmp_path / f"{previous_session}.sqlite3").is_file()
    assert not (tmp_path / "recording-settings.tmp").exists()
    recorder.close()

    restored = Recorder(tmp_path, controls=RecordingControls())
    restored.restore_settings({})
    assert restored.controls.settings["sources"]["path_topic"] == "/custom/plan"
    restored.close()
