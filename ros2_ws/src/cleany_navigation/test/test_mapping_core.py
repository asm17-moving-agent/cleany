import math
from pathlib import Path
from types import SimpleNamespace as Obj
import pytest
import yaml
from cleany_navigation.mapping_storage import save_grid, planar_origin
from cleany_navigation.scan_filter import mask_front_scan
from cleany_navigation.mapping_session import SessionController, Busy


def test_map_origin_yaw_and_image_row_order_survive_save(tmp_path):
    yaw = 0.71
    pose = Obj(
        position=Obj(x=1.0, y=-2.0),
        orientation=Obj(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
    )
    origin = planar_origin(pose)
    result = save_grid(tmp_path, 2, 2, 0.05, origin, [0, 100, -1, 50])
    assert result["origin"] == pytest.approx([1.0, -2.0, yaw])
    document = yaml.safe_load((tmp_path / "map.yaml").read_text())
    assert (
        (tmp_path / document["image"]).read_bytes().endswith(bytes([205, 127, 254, 0]))
    )
    assert (tmp_path / "map.pgm").read_bytes() == (
        tmp_path / document["image"]
    ).read_bytes()
    save_grid(tmp_path, 1, 1, 0.1, (0.0, 0.0, 0.0), [100])
    assert len(list(tmp_path.glob("map-*.pgm"))) == 1


def test_map_save_rejects_invalid_data_without_replacing_previous(tmp_path):
    save_grid(tmp_path, 1, 1, 0.1, (0.0, 0.0, 0.0), [0])
    before = (tmp_path / "map.yaml").read_bytes()
    with pytest.raises(ValueError):
        save_grid(tmp_path, 1, 2, 0.1, (0.0, 0.0, 0.0), [0])
    with pytest.raises(ValueError):
        save_grid(tmp_path, 1, 1, math.nan, (0.0, 0.0, 0.0), [0])
    assert (tmp_path / "map.yaml").read_bytes() == before


def test_filter_keeps_raw_intact_and_masks_near_and_rear():
    source = [2.0, 0.3, 2.0, 2.0, 2.0]
    result = mask_front_scan(source, -math.pi, math.pi / 2)
    assert source == [2.0, 0.3, 2.0, 2.0, 2.0]
    assert math.isnan(result[0]) and math.isnan(result[1]) and math.isnan(result[4])
    assert result[2:4] == [2.0, 2.0]
    assert mask_front_scan([2.0], 0.0, 0.0, center=math.pi) == [math.nan] or math.isnan(
        mask_front_scan([2.0], 0.0, 0.0, center=math.pi)[0]
    )


class Backend:
    def __init__(self):
        self.running = False
        self.confirmed = True
        self.fail_save = False
        self.calls = []

    def devices(self):
        return {"lidar": True, "esp32": True}

    def has_control(self):
        return True

    def active(self):
        return self.running

    def confirm_stop(self):
        self.calls.append("confirm")
        return self.confirmed

    def stop(self):
        self.calls.append("save")
        if self.fail_save:
            raise RuntimeError("save failed")
        self.running = False

    def start(self, output):
        self.calls.append("start")
        self.running = True


def test_reset_serialization_and_stop_ack_before_start(tmp_path):
    backend = Backend()
    controller = SessionController(backend, tmp_path)
    first = controller.request_reset("one")
    assert controller.request_reset("one")["operation_id"] == first["operation_id"]
    with pytest.raises(Busy):
        controller.request_reset("two")
    controller.perform_reset()
    assert backend.calls == ["confirm", "save", "start"]
    assert controller.snapshot()["generation"] == 1
    backend.confirmed = False
    controller.request_reset("two")
    controller.perform_reset()
    assert controller.snapshot()["phase"] == "failed"
    assert backend.running
    assert backend.calls[-1] == "confirm"


def test_save_failure_preserves_old_session_and_blocks_new_start(tmp_path):
    backend = Backend()
    controller = SessionController(backend, tmp_path)
    controller.request_reset("one")
    controller.perform_reset()
    old = controller.snapshot()["output"]
    backend.fail_save = True
    controller.request_reset("two")
    controller.perform_reset()
    assert (
        controller.snapshot()["phase"] == "failed"
        and controller.snapshot()["output"] == old
    )
    assert backend.calls.count("start") == 1


def test_interrupted_yaml_replace_preserves_previous_map(tmp_path, monkeypatch):
    from cleany_navigation import mapping_storage

    save_grid(tmp_path, 1, 1, 0.1, (0.0, 0.0, 0.0), [0])
    previous = (tmp_path / "map.yaml").read_bytes()
    replace = mapping_storage.os.replace

    def fail_yaml(source, target):
        if Path(target).name == "map.yaml":
            raise OSError("injected write failure")
        replace(source, target)

    monkeypatch.setattr(mapping_storage.os, "replace", fail_yaml)
    with pytest.raises(OSError):
        save_grid(tmp_path, 2, 1, 0.2, (1.0, 2.0, 0.3), [100, 0])
    assert (tmp_path / "map.yaml").read_bytes() == previous
    document = yaml.safe_load(previous)
    assert (tmp_path / document["image"]).is_file()


def test_process_ledger_rejects_reused_pid_identity():
    import os
    from cleany_navigation.mapping_process import identity, live_recorded

    pid = os.getpid()
    token = identity(pid)
    assert token is not None
    assert live_recorded({str(pid): token}) == [pid]
    assert live_recorded({str(pid): "reused-" + token}) == []


def test_late_periodic_save_cannot_write_into_a_new_generation(tmp_path, monkeypatch):
    from concurrent.futures import Future
    import sys
    import types
    from cleany_navigation.mapping_backend import MappingBackend

    future = Future()
    requests = []
    service = types.ModuleType("cartographer_ros_msgs.srv")
    service.WriteState = Obj(Request=lambda **values: Obj(**values))
    monkeypatch.setitem(
        sys.modules, "cartographer_ros_msgs", types.ModuleType("cartographer_ros_msgs")
    )
    monkeypatch.setitem(sys.modules, "cartographer_ros_msgs.srv", service)

    def call(request):
        requests.append(request)
        return future

    node = Obj(write_client=Obj(service_is_ready=lambda: True, call_async=call))
    backend = MappingBackend(node, {}, tmp_path / "profile", tmp_path)
    old = tmp_path / "old"
    new = tmp_path / "new"
    old.mkdir()
    new.mkdir()
    backend.output = old
    backend.process = backend.logger_process = Obj(poll=lambda: None)
    backend.periodic_save()
    backend.output = new
    Path(requests[0].filename).write_bytes(b"old generation")
    future.set_result(Obj(status=Obj(code=0)))
    assert (old / "latest.pbstream").read_bytes() == b"old generation"
    assert not (new / "latest.pbstream").exists()


def test_start_failure_exposes_artifacts_and_persists_full_traceback(tmp_path):
    import json

    class FailedBackend(Backend):
        def start(self, output):
            output.mkdir()
            (output / "cartographer.log").write_text("startup details")
            raise RuntimeError("missing scan-time TF")

    controller = SessionController(FailedBackend(), tmp_path)
    controller.request_reset("failed-start")
    controller.perform_reset()
    status = controller.snapshot()
    assert status["phase"] == "failed" and status["generation"] == 0
    assert (Path(status["output"]) / "cartographer.log").exists()
    diagnostic = json.loads(Path(status["error_file"]).read_text())
    assert diagnostic["error_code"] == "MAPPING_STARTING_FAILED"
    assert (
        diagnostic["request_id"] == "failed-start"
        and diagnostic["output"] == status["output"]
    )
    assert (
        "missing scan-time TF" in diagnostic["traceback"]
        and "in start" in diagnostic["traceback"]
    )
    controller.request_reset("retry")
    assert controller.snapshot()["error_code"] is None


def test_diagnostic_storage_failure_keeps_original_error(tmp_path, monkeypatch):
    import cleany_navigation.mapping_session as module

    def unavailable(*_):
        raise OSError("disk full")

    monkeypatch.setattr(module, "atomic_json", unavailable)
    controller = SessionController(Backend(), tmp_path)
    controller.request_reset("failure")
    try:
        raise RuntimeError("original RPC failure")
    except RuntimeError as error:
        controller.fail(error)
    status = controller.snapshot()
    assert status["phase"] == "failed" and status["error_file"] is None
    assert (
        "original RPC failure" in status["message"] and "disk full" in status["message"]
    )


def test_failed_start_preserves_original_and_cleanup_exceptions(tmp_path, monkeypatch):
    import json
    import sys
    from cleany_navigation.mapping_backend import MappingBackend

    monkeypatch.setitem(
        sys.modules, "std_srvs.srv", Obj(Trigger=Obj(Request=lambda: None))
    )
    profile = tmp_path / "profile.yaml"
    profile.write_text("{}")

    class FailedBackend(MappingBackend):
        def stop(self):
            raise RuntimeError("logger flush failed")

    backend = FailedBackend(
        Obj(
            get_node_names_and_namespaces=lambda: [("cleany_logger", "/")],
            get_clock=lambda: Obj(now=lambda: Obj(nanoseconds=1)),
        ),
        {},
        profile,
        tmp_path,
    )
    output = tmp_path / "session"
    with pytest.raises(RuntimeError) as captured:
        backend.start(output)
    formatted = "".join(
        __import__("traceback").format_exception(
            type(captured.value), captured.value, captured.value.__traceback__
        )
    )
    assert "existing Logger" in formatted and "logger flush failed" in formatted
    assert "startup failed" in str(captured.value)
    assert json.loads((output / "session.json").read_text())["profile"] == {}
