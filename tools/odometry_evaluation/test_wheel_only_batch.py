import json
from pathlib import Path
import signal

import pytest

import run_wheel_only_eval as batch


def config(tmp_path):
    motion = Path(__file__).with_name("motion_settings.json")
    extrinsic = batch.ROOT / "configs/robot/lidar_extrinsic_20261015.json"
    data = {
        "schema_version": 1, "repetitions": 3, "ros_domain_id": 0,
        "patterns": ["surge", "sway", "rotation"],
        "linear_speeds_mps": [.2, .35, .5], "angular_speeds_rad_s": [.3, .6, 1.],
        "translation_distance_m": .5, "rotation_angle_deg": 90.,
        "odom_topic": "/wheel/odom", "odom_timeout_s": .15, "lidar_min_range_m": .45,
        "extrinsic_config": str(extrinsic.relative_to(batch.ROOT)),
        "runtime_config": "configs/robot/wheel_evaluation_runtime.json",
        "motion_config": str(motion.relative_to(batch.ROOT)), "plots": True,
        "output_dir": str((tmp_path / "out").relative_to(batch.ROOT)) if tmp_path.is_relative_to(batch.ROOT) else str(tmp_path / "out"),
    }
    return data


def test_commands_and_dry_run_no_side_effects(tmp_path, monkeypatch, capsys):
    data = config(tmp_path)
    cfg = tmp_path / "batch.json"
    cfg.write_text(json.dumps(data))
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(batch.subprocess, "Popen", lambda *a, **k: pytest.fail("spawned"))
    monkeypatch.setattr(batch, "RosReadinessProbe", lambda *a, **k: pytest.fail("ROS initialized"))
    monkeypatch.setattr(batch, "EvaluationStack", lambda *a, **k: pytest.fail("stack initialized"))
    assert batch.main(["--config", str(cfg), "--dry-run"]) == 0
    output = capsys.readouterr().out
    assert "--odom-topic /wheel/odom" in output
    assert "--lidar-min-range 0.45" in output
    assert "rplidar_ros rplidar_node" in output
    assert "__node:=wheel_eval_lidar" in output
    assert not (tmp_path / "out").exists()


def test_repository_default_config_has_requested_plan(monkeypatch, capsys):
    monkeypatch.setattr(batch.subprocess, "Popen", lambda *a, **k: pytest.fail("spawned"))
    assert batch.main(["--dry-run"]) == 0
    output = capsys.readouterr().out
    assert "ROS domain: 0" in output
    assert "Runs: 3; pattern/speed round trips: 45; directional legs: 90" in output
    assert "--patterns surge sway rotation diagonal_left diagonal_right" in output
    assert "Evaluation-only distance/yaw P: X=6; Y=6; yaw=6; diagonal=6" in output
    assert "--distance 0.5" in output
    assert "--rotation-angle-deg 90.0" in output
    assert "--odom-topic /wheel/odom" in output
    assert not Path(output.splitlines()[0].removeprefix("Batch: ")).exists()


@pytest.mark.parametrize("mutate", [
    lambda c: c.update(extra=True), lambda c: c.pop("plots"),
    lambda c: c.update(repetitions=True), lambda c: c.update(odom_timeout_s=float("nan")),
    lambda c: c.update(patterns=["surge", "surge"]), lambda c: c.update(linear_speeds_mps=[0]),
    lambda c: c.update(lidar_min_range_m=10),
])
def test_invalid_schema_fails_before_output(tmp_path, capsys, mutate):
    c = config(tmp_path)
    mutate(c)
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(c))
    with pytest.raises(SystemExit):
        batch.main(["--config", str(path)])
    assert not (tmp_path / "out").exists()


def test_referenced_config_validation(tmp_path):
    c = config(tmp_path)
    c["motion_config"] = "does-not-exist.json"
    path = tmp_path / "config.json"
    path.write_text(json.dumps(c))
    with pytest.raises(FileNotFoundError):
        batch.load_config(path)


class FakeProbe:
    def __init__(self, *args):
        self.closed = False

    def close(self):
        self.closed = True


class FakeStack:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def start(self):
        return {"snapshot": {"ready": True}}

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    monkeypatch.setattr(batch, "RosReadinessProbe", FakeProbe)
    monkeypatch.setattr(batch, "EvaluationStack", FakeStack)


class Child:
    def __init__(self, rc, cmd=None):
        self.rc = rc
        self.signals = []
        self.cmd = cmd

    def wait(self):
        return self.rc

    def poll(self):
        return None

    def send_signal(self, sig):
        self.signals.append(sig)


def setup_run(tmp_path, monkeypatch, codes):
    c = config(tmp_path)
    path = tmp_path / "config.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(c))
    children = []

    def popen(cmd, **kwargs):
        child = Child(codes[len(children)], cmd)
        children.append(child)
        assert kwargs["start_new_session"] is True
        assert kwargs["env"]["ROS_DOMAIN_ID"] == "0"
        assert kwargs["env"]["PYTHONNOUSERSITE"] == "1"
        return child
    monkeypatch.setattr(batch.subprocess, "Popen", popen)
    return c, path, children


@pytest.mark.parametrize("child_rc,batch_rc", [(7, 7), (-signal.SIGTERM, 128 + signal.SIGTERM)])
def test_first_failure_stops_and_success_counts(tmp_path, monkeypatch, child_rc, batch_rc):
    c, path, children = setup_run(tmp_path, monkeypatch, [child_rc, 0, 0])
    assert batch.main(["--config", str(path)]) == batch_rc
    assert len(children) == 1
    dirs = list((tmp_path / "out").glob("batch_*"))
    summary = json.loads((dirs[0] / "batch_summary.json").read_text())
    assert summary["runs"][0]["return_code"] == child_rc
    assert summary["return_code"] == batch_rc
    c, path, children = setup_run(tmp_path / "second", monkeypatch, [0, 0, 0])
    assert batch.main(["--config", str(path)]) == 0
    assert len(children) == 3
    run_dir = next((tmp_path / "second" / "out").glob("batch_*"))
    assert (run_dir / "motion_config.json").exists()
    assert (run_dir / "extrinsic_config.json").exists()
    snap = json.loads((run_dir / "config_snapshot.json").read_text())
    summary = json.loads((run_dir / "batch_summary.json").read_text())
    assert snap["motion_config_sha256"] == summary["motion_config_sha256"]
    for child in children:
        assert child.cmd[child.cmd.index("--motion-config") + 1] == str(run_dir / "motion_config.json")
        assert child.cmd[child.cmd.index("--extrinsic-config") + 1] == str(run_dir / "extrinsic_config.json")
    assert (run_dir / "runtime_config.json").is_file()
    assert (run_dir / "base_hardware.yaml").is_file()


def test_spawn_error_recorded_and_no_next_run(tmp_path, monkeypatch):
    c, path, children = setup_run(tmp_path, monkeypatch, [])
    monkeypatch.setattr(batch.subprocess, "Popen", lambda *a, **k: (_ for _ in ()).throw(OSError("spawn failed")))
    assert batch.main(["--config", str(path)]) == 1
    assert children == []
    run_dir = next((tmp_path / "out").glob("batch_*"))
    summary = json.loads((run_dir / "batch_summary.json").read_text())
    assert summary["failure_reason"] == "Could not start run_1: spawn failed"
    assert summary["runs"][0]["return_code"] is None


def test_signal_during_popen_forwarded_once(tmp_path, monkeypatch):
    _, path, children = setup_run(tmp_path, monkeypatch, [128 + signal.SIGTERM])
    original_signal = batch.signal.signal
    registered = {}
    def signal_register(sig, handler):
        registered[sig] = handler
        return original_signal(sig, handler)
    monkeypatch.setattr(batch.signal, "signal", signal_register)
    original_popen = batch.subprocess.Popen
    def racing_popen(*args, **kwargs):
        child = original_popen(*args, **kwargs)
        registered[signal.SIGTERM](signal.SIGTERM, None)
        return child
    monkeypatch.setattr(batch.subprocess, "Popen", racing_popen)
    assert batch.main(["--config", str(path)]) == 128 + signal.SIGTERM
    assert len(children) == 1
    assert children[0].signals == [signal.SIGTERM]
    run_dir = next((tmp_path / "out").glob("batch_*"))
    summary = json.loads((run_dir / "batch_summary.json").read_text())
    assert summary["runs"][0]["return_code"] == 128 + signal.SIGTERM


def test_signal_forwarded_once(tmp_path, monkeypatch):
    _, path, children = setup_run(tmp_path, monkeypatch, [128 + signal.SIGTERM])
    original = batch.signal.signal
    handlers = {}
    def intercept(sig, handler):
        handlers[sig] = handler
        result = original(sig, handler)
        return result
    monkeypatch.setattr(batch.signal, "signal", intercept)
    real_popen = batch.subprocess.Popen
    def interrupting_popen(*args, **kwargs):
        child = real_popen(*args, **kwargs)
        child.wait = lambda: (handlers[signal.SIGTERM](signal.SIGTERM, None),
                              handlers[signal.SIGTERM](signal.SIGTERM, None),
                              128 + signal.SIGTERM)[-1]
        return child
    monkeypatch.setattr(batch.subprocess, "Popen", interrupting_popen)
    assert batch.main(["--config", str(path)]) == 128 + signal.SIGTERM
    assert len(children) == 1
    assert children[0].signals == [signal.SIGTERM]


def test_runtime_failure_prevents_evaluator_and_persists_cause(tmp_path, monkeypatch):
    _, path, children = setup_run(tmp_path, monkeypatch, [])
    stack = FakeStack()
    def fail():
        raise RuntimeError("runtime stack startup timed out: odom: no samples received")
    stack.start = fail
    monkeypatch.setattr(batch, "EvaluationStack", lambda *args, **kwargs: stack)
    assert batch.main(["--config", str(path)]) == 1
    assert not children
    assert stack.closed
    summary = json.loads(next((tmp_path / "out").glob("batch_*/batch_summary.json")).read_text())
    assert summary["failed_stage"] == "runtime_startup"
    assert "odom: no samples" in summary["failure_reason"]
    assert summary["runs"] == []


def test_runtime_stays_alive_until_evaluator_exit(tmp_path, monkeypatch):
    _, path, children = setup_run(tmp_path, monkeypatch, [0, 0, 0])
    stack = FakeStack()
    monkeypatch.setattr(batch, "EvaluationStack", lambda *args, **kwargs: stack)
    original = batch.subprocess.Popen
    def launch(*args, **kwargs):
        assert not stack.closed
        return original(*args, **kwargs)
    monkeypatch.setattr(batch.subprocess, "Popen", launch)
    assert batch.main(["--config", str(path)]) == 0
    assert len(children) == 3
    assert stack.closed


def test_runtime_interruption_cleans_up_without_motion(tmp_path, monkeypatch):
    _, path, children = setup_run(tmp_path, monkeypatch, [])
    stack = FakeStack()
    def start():
        batch.signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        raise RuntimeError("evaluation stack startup cancelled")
    stack.start = start
    monkeypatch.setattr(batch, "EvaluationStack", lambda *args, **kwargs: stack)
    assert batch.main(["--config", str(path)]) == 128 + signal.SIGTERM
    assert stack.closed
    assert not children


def test_configured_domain_applied_to_probe_and_restored(tmp_path, monkeypatch):
    import os
    c, path, _ = setup_run(tmp_path, monkeypatch, [0, 0, 0])
    monkeypatch.setenv("ROS_DOMAIN_ID", "173")
    def probe(*args):
        assert os.environ["ROS_DOMAIN_ID"] == str(c["ros_domain_id"])
        return FakeProbe()
    monkeypatch.setattr(batch, "RosReadinessProbe", probe)
    assert batch.main(["--config", str(path)]) == 0
    assert os.environ["ROS_DOMAIN_ID"] == "173"
