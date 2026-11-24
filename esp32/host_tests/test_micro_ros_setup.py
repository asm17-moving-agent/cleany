import importlib.util
from pathlib import Path
import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/micro_ros_setup.py"
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


@pytest.mark.parametrize('location', ['component', 'child', 'outside'])
def test_git_wrapper_applies_pins_at_component_root_and_children(tmp_path, monkeypatch, location):
    import json

    spec = importlib.util.spec_from_file_location("micro_ros_git", SCRIPT.with_name("micro_ros_git.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    component = tmp_path/'esp32/micro_ros/micro_ros_espidf_component'
    child = component/'build'
    child.mkdir(parents=True)
    (tmp_path/'esp32/micro_ros.lock.json').write_text(json.dumps({
        'component_sources': {'rclc': ['https://github.com/ros2/rclc', 'a'*40]}}))
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(module, 'COMPONENT', component)
    monkeypatch.chdir({'component': component, 'child': child, 'outside': tmp_path}[location])
    monkeypatch.setattr(module.sys, 'argv', ['git', 'clone', '-b', 'humble',
                                           'https://github.com/ros2/rclc', 'src/rclc'])
    calls = []
    monkeypatch.setattr(module.subprocess, 'run', lambda args, **kwargs: calls.append(args))

    class Delegated(Exception):
        pass

    def delegate(*args):
        raise Delegated()

    monkeypatch.setattr(module.os, 'execv', delegate)
    if location == 'outside':
        with pytest.raises(Delegated):
            module.main()
        assert not calls
    else:
        module.main()
        assert calls[0][2] == '--no-checkout'
        assert calls[1][-1] == 'a'*40
        assert calls[2][-2:] == ['--detach', 'a'*40]


def test_fingerprint_changes_with_message_source(tmp_path, monkeypatch):
    root = tmp_path
    controller = root / "esp32"
    controller.mkdir()
    interfaces = root / "ros2_ws/src/cleany_base_interfaces"
    (interfaces / "msg").mkdir(parents=True)
    for file in ("app-colcon.meta", "sdkconfig.defaults", "platformio.ini", "lock.json"):
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

    controller = tmp_path / "esp32"
    controller.mkdir()
    interfaces = tmp_path / "ros2_ws/src/cleany_base_interfaces"
    (interfaces / "msg").mkdir(parents=True)
    for file in ("app-colcon.meta", "sdkconfig.defaults", "platformio.ini", "lock.json"):
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
    (controller / ".pio/cleany-sdkconfig-defaults-runtime").write_text(hashlib.sha256(
        (controller / "sdkconfig.defaults").read_bytes()).hexdigest())
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
    assert cache.exists()  # 입력 지문이 같으면 유효한 빌드 캐시를 유지합니다.
    incomplete = component / "micro_ros_src/install"
    incomplete.mkdir(parents=True)
    (incomplete / "partial-marker").write_text("failed colcon configure")
    setup.refresh_component(controller, "runtime")
    assert not incomplete.exists()
    assert cache.exists()
    (component / "libmicroros.a").write_text("complete library")
    (component / "include").mkdir()
    incomplete.mkdir()
    setup.refresh_component(controller, "runtime")
    assert incomplete.exists()  # 필요한 생성물이 모두 있으면 강제로 재생성하지 않습니다.


def test_tof_type_support_is_regenerated_only_for_integrated_firmware(tmp_path, monkeypatch):
    controller = tmp_path / "esp32"
    controller.mkdir()
    for name in ("app-colcon.meta", "sdkconfig.defaults", "platformio.ini", "lock.json"):
        (controller / name).write_text(name)
    base = tmp_path / "ros2_ws/src/cleany_base_interfaces"
    base.mkdir(parents=True)
    subset = controller / "mcu_interfaces/cleany_interfaces"
    (subset / "msg").mkdir(parents=True)
    for directory in (base, subset):
        for name in ("CMakeLists.txt", "package.xml"):
            (directory / name).write_text(name)
    message = tmp_path / "TofFrame.msg"
    message.write_text("bool sensor_ready\n")
    (subset / "msg/TofFrame.msg").symlink_to(message)
    output = controller / "micro_ros"
    generated = output / "micro_ros_espidf_component/micro_ros_src/src/extra_packages"
    generated.mkdir(parents=True)
    monkeypatch.setattr(setup, "ROOT", tmp_path)
    monkeypatch.setattr(setup, "CONTROLLER", controller)
    monkeypatch.setattr(setup, "LOCK", controller / "lock.json")
    monkeypatch.setattr(setup, "OUTPUT", output)
    integrated = "esp32-s3-integrated"
    legacy = "esp32-s3-microros"
    legacy_fingerprint = setup.component_fingerprint(controller, legacy)
    setup.refresh_component(controller, integrated)
    link = controller / "extra_ros_packages/cleany_interfaces"
    copied = generated / "cleany_interfaces/msg/TofFrame.msg"
    assert link.resolve() == subset
    assert copied.read_text() == message.read_text()
    message.write_text("uint32 sequence\n")
    setup.refresh_component(controller, integrated)
    assert copied.read_text() == message.read_text()
    assert setup.component_fingerprint(controller, legacy) == legacy_fingerprint
    setup.refresh_component(controller, legacy)
    assert not link.is_symlink()
    assert not copied.exists()


def test_generated_sdkconfig_link_migrates_only_known_previous_source(tmp_path):
    source = tmp_path / "sdkconfig.defaults"
    source.write_text("CONFIG_ESP_CONSOLE_NONE=y")
    previous = tmp_path / "previous.defaults"
    link = tmp_path / "generated.defaults"
    link.symlink_to(previous)  # 통합 과정에서 이전 소스 위치를 제거했습니다.
    setup.link_generated_config(link, source, previous)
    assert link.resolve() == source
    setup.link_generated_config(link, source, previous)  # 다시 실행해도 결과가 같습니다.
    link.unlink()
    link.write_text("unexpected local config")
    with pytest.raises(RuntimeError, match="unexpected generated link"):
        setup.link_generated_config(link, source, previous)


@pytest.mark.parametrize('old_path', ['tools/micro_ros_git.py', 'tools/micro_ros/micro_ros_git.py'])
def test_git_wrapper_migrates_known_old_paths(tmp_path, monkeypatch, old_path):
    monkeypatch.setattr(setup, 'ROOT', tmp_path)
    wrapper = tmp_path/'git'
    wrapper.symlink_to(tmp_path/old_path)
    setup.link_git_wrapper(wrapper)
    assert wrapper.resolve() == SCRIPT.with_name('micro_ros_git.py').resolve()
    setup.link_git_wrapper(wrapper)
    assert wrapper.resolve() == SCRIPT.with_name('micro_ros_git.py').resolve()


def test_git_wrapper_creates_link_and_preserves_unknown_target(tmp_path):
    wrapper = tmp_path/'git'
    setup.link_git_wrapper(wrapper)
    assert wrapper.resolve() == SCRIPT.with_name('micro_ros_git.py').resolve()
    wrapper.unlink()
    unknown = tmp_path/'custom-git'
    wrapper.symlink_to(unknown)
    with pytest.raises(RuntimeError, match='unexpected generated link'):
        setup.link_git_wrapper(wrapper)
    assert wrapper.readlink() == unknown


def test_only_micro_ros_firmware_and_console_free_board_defaults():
    import configparser

    config = configparser.ConfigParser()
    config.read(setup.CONTROLLER / "platformio.ini")
    assert config.sections() == ["platformio", "env:esp32-s3-microros", "env:esp32-s3-integrated"]
    assert config["platformio"]["default_envs"] == "esp32-s3-integrated"
    integrated = config["env:esp32-s3-integrated"]
    assert integrated["extends"] == "env:esp32-s3-microros"
    assert "-DCLEANY_ENABLE_BNO055=0" in integrated["build_flags"].split()
    assert "-DCLEANY_TOF_I2C_HZ=400000" in integrated["build_flags"].split()
    assert config["env:esp32-s3-microros"]["platform"] == "espressif32@6.12.0"
    defaults = dict(
        line.split("=", 1) for line in
        (setup.CONTROLLER / "sdkconfig.defaults").read_text().splitlines()
        if line and not line.startswith("#")
    )
    assert defaults["CONFIG_ESP_CONSOLE_NONE"] == "y"
    assert defaults["CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG"] == "n"
    assert defaults["CONFIG_ESP_CONSOLE_SECONDARY_NONE"] == "y"
    assert defaults["CONFIG_MICRO_ROS_ESP_UART_TRANSPORT"] == "n"
    assert defaults["CONFIG_FREERTOS_HZ"] == "1000"
    assert defaults["CONFIG_ESPTOOLPY_FLASHSIZE_32MB"] == "y"
    assert defaults["CONFIG_SPIRAM_MODE_OCT"] == "y"


def test_cmake_restores_pinned_generators_after_platformio_clears_pythonpath(tmp_path):
    import os
    import shutil
    import subprocess

    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("CMake is not installed")
    controller = tmp_path / "esp32"
    helper = controller / "cmake/microros_python.cmake"
    helper.parent.mkdir(parents=True)
    helper.write_text((setup.CONTROLLER / "cmake/microros_python.cmake").read_text())
    script = tmp_path / "check.cmake"
    script.write_text(f'include("{helper}")\nmessage(STATUS "PYTHONPATH=$ENV{{PYTHONPATH}}")\n')
    env = dict(os.environ, PYTHONPATH="retained")
    missing = subprocess.run([cmake, "-P", str(script)], env=env, text=True, capture_output=True)
    assert missing.returncode != 0
    assert "make firmware-setup" in missing.stderr
    generator = controller / ".venv/lib/python3.10/site-packages"
    (generator / "catkin_pkg").mkdir(parents=True)
    result = subprocess.run([cmake, "-P", str(script)], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert f"{generator}:retained" in result.stdout
    for source in ("CMakeLists.txt", "microros_smoke/CMakeLists.txt"):
        contents = (setup.CONTROLLER / source).read_text()
        assert contents.index("microros_python.cmake") < contents.index("project(")
