import importlib.util
from pathlib import Path
import pytest

SCRIPT = Path(__file__).with_name("micro_ros_setup.py")
SPEC = importlib.util.spec_from_file_location("micro_ros_setup", SCRIPT)
assert SPEC and SPEC.loader
setup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(setup)


def test_lock_contains_immutable_pins():
    import json

    lock = json.loads(setup.LOCK.read_text())
    assert set(lock) == {"component", "agent", "micro_ros_msgs", "component_sources"}
    for entry in (lock[name] for name in ("component", "agent", "micro_ros_msgs")):
        assert len(entry["revision"]) == 40
        assert set(entry["revision"]) <= set("0123456789abcdef")
    for repository, revision in lock["component_sources"].values():
        assert repository.startswith("https://")
        assert len(revision) == 40
        assert set(revision) <= set("0123456789abcdef")


def test_checkout_refuses_a_wrong_resolved_revision(tmp_path, monkeypatch):
    monkeypatch.setattr(setup, "run", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        setup.subprocess, "check_output", lambda *_args, **_kwargs: "0" * 40 + "\n"
    )
    with pytest.raises(RuntimeError, match="expected"):
        setup.checkout("https://example.invalid/repo", "1" * 40, tmp_path)


def test_checkout_checks_out_requested_commit(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(setup, "run", lambda *args, **_kwargs: calls.append(args))
    monkeypatch.setattr(
        setup.subprocess,
        "check_output",
        lambda *_args, **_kwargs: next(revisions),
    )
    revisions = iter(("b" * 40 + "\n", "a" * 40 + "\n"))
    setup.checkout("https://example.invalid/repo", "a" * 40, tmp_path)
    assert calls[-1][-3:] == ("checkout", "--detach", "FETCH_HEAD")


def test_clone_wrapper_replaces_branch_with_immutable_source():
    spec = importlib.util.spec_from_file_location("micro_ros_git", SCRIPT.with_name("micro_ros_git.py"))
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.clone_spec(
        ["clone", "-b", "humble", "https://github.com/ros2/rclc", "src/rclc"],
        {"rclc": ["https://github.com/ros2/rclc", "a" * 40]},
    ) == ("https://github.com/ros2/rclc", "a" * 40, "src/rclc")
    with pytest.raises((KeyError, ValueError)):
        module.clone_spec(["clone", "-b", "humble", "https://evil/rclc", "src/rclc"],
                          {"rclc": ["https://github.com/ros2/rclc", "a" * 40]})


def test_fingerprint_changes_with_message_source(tmp_path, monkeypatch):
    root = tmp_path
    controller = root / "motor_controller"
    controller.mkdir()
    interfaces = root / "ros2_ws/src/cleany_base_interfaces"
    (interfaces / "msg").mkdir(parents=True)
    for file in ("app-colcon.meta", "sdkconfig.microros.defaults", "platformio.ini", "lock.json"):
        (controller / file).write_text(file)
    for file in ("CMakeLists.txt", "package.xml", "msg/WheelCommand.msg"):
        (interfaces / file).write_text(file)
    monkeypatch.setattr(setup, "ROOT", root)
    monkeypatch.setattr(setup, "CONTROLLER", controller)
    monkeypatch.setattr(setup, "LOCK", controller / "lock.json")
    first = setup.component_fingerprint(controller, "runtime")
    (interfaces / "msg/WheelCommand.msg").write_text("uint32 changed")
    assert first != setup.component_fingerprint(controller, "runtime")
    assert first != setup.component_fingerprint(controller, "smoke")


def test_component_refresh_invalidates_cmake_on_environment_switch(tmp_path, monkeypatch):
    import hashlib

    controller = tmp_path / "motor_controller"
    controller.mkdir()
    interfaces = tmp_path / "ros2_ws/src/cleany_base_interfaces"
    (interfaces / "msg").mkdir(parents=True)
    for file in ("app-colcon.meta", "sdkconfig.microros.defaults", "platformio.ini", "lock.json"):
        (controller / file).write_text(file)
    for file in ("CMakeLists.txt", "package.xml", "msg/WheelCommand.msg"):
        (interfaces / file).write_text(file)
    output = controller / "micro_ros"
    component = output / "micro_ros_espidf_component"
    (component / "include").mkdir(parents=True)
    (component / "libmicroros.a").write_text("old library")
    (component / ".cleany-build-fingerprint").write_text("previous smoke environment")
    cache = controller / ".pio/build/runtime/CMakeCache.txt"
    cache.parent.mkdir(parents=True)
    cache.write_text("previous runtime configure")
    (controller / ".pio/cleany-sdkconfig-defaults").write_text(hashlib.sha256(
        (controller / "sdkconfig.microros.defaults").read_bytes()).hexdigest())
    monkeypatch.setattr(setup, "ROOT", tmp_path)
    monkeypatch.setattr(setup, "CONTROLLER", controller)
    monkeypatch.setattr(setup, "LOCK", controller / "lock.json")
    monkeypatch.setattr(setup, "OUTPUT", output)
    setup.refresh_component(controller, "runtime")
    assert not cache.exists()
    assert not (component / "libmicroros.a").exists()
    assert not (component / "include").exists()
    cache.write_text("matching runtime configure")
    setup.refresh_component(controller, "runtime")
    assert cache.exists()  # Matching fingerprints retain a valid build cache.
