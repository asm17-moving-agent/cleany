"""Standalone project isolation and authoritative configuration checks."""
from __future__ import annotations

import configparser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / "esp32/imu_calibration"


def run_configuration_script(project: Path) -> None:
    class Environment:
        def subst(self, key: str) -> str:
            return {"$PROJECT_DIR": str(project), "$PIOENV": "test"}[key]

    script = PROJECT / "scripts/configure.py"
    exec(compile(script.read_text(), str(script), "exec"),
         {"Import": lambda _: None, "env": Environment()})


def test_defaults_changes_regenerate_only_derived_configuration(tmp_path):
    (tmp_path / "main").mkdir()
    (tmp_path / "main/Kconfig.projbuild").write_text("config TEST\n")
    (tmp_path / "sdkconfig.defaults").write_text("CONFIG_TEST=1\n")
    cache = tmp_path / ".pio/build/test/CMakeCache.txt"
    cache.parent.mkdir(parents=True)
    config = tmp_path / "sdkconfig.test"
    config.write_text("old")
    cache.write_text("old")
    run_configuration_script(tmp_path)
    assert not config.exists() and not cache.exists()
    config.write_text("generated")
    cache.write_text("generated")
    run_configuration_script(tmp_path)
    assert config.read_text() == "generated"
    assert cache.read_text() == "generated"
    (tmp_path / "sdkconfig.defaults").write_text("CONFIG_TEST=2\n")
    run_configuration_script(tmp_path)
    assert not config.exists() and not cache.exists()
    assert (tmp_path / "sdkconfig.defaults").read_text() == "CONFIG_TEST=2\n"


def test_kconfig_changes_regenerate_derived_configuration(tmp_path):
    (tmp_path / "main").mkdir()
    kconfig = tmp_path / "main/Kconfig.projbuild"
    kconfig.write_text("config TEST\n")
    (tmp_path / "sdkconfig.defaults").write_text("")
    run_configuration_script(tmp_path)
    config = tmp_path / "sdkconfig.test"
    config.write_text("generated")
    kconfig.write_text("config TEST_NEW\n")
    run_configuration_script(tmp_path)
    assert not config.exists()


def test_project_has_explicit_diagnostic_environment_and_no_micro_ros():
    config = configparser.ConfigParser()
    config.read(PROJECT / "platformio.ini")
    assert config["platformio"]["default_envs"] == "esp32-s3-imu-calibration"
    assert config["platformio"]["src_dir"] == "main"
    env = config["env:esp32-s3-imu-calibration"]
    assert env["platform"] == "espressif32@6.12.0"
    assert env["board_upload.flash_size"] == "32MB"
    assert env["board_build.flash_mode"] == "dout"
    assert "micro_ros" not in (PROJECT / "CMakeLists.txt").read_text()
    main = (PROJECT / "main/CMakeLists.txt").read_text()
    assert "../../src/mpu6050.cpp" in main and "micro_ros" not in main
    assert "../../src/main.cpp" not in main


def test_thresholds_start_unreviewed_without_invented_limits():
    kconfig = (PROJECT / "main/Kconfig.projbuild").read_text()
    sections = kconfig.split("\nconfig ")
    reviewed = next(s for s in sections if s.startswith("CLEANY_IMU_LIMITS_REVIEWED"))
    assert "default n" in reviewed
    for name in (
        "CLEANY_IMU_MAX_GYRO_STD_MICRORAD_S",
        "CLEANY_IMU_MAX_ACCEL_STD_MM_S2",
        "CLEANY_IMU_MAX_VALIDATION_MEAN_MICRORAD_S",
        "CLEANY_IMU_MAX_TEMPERATURE_SPAN_MILLIC",
    ):
        section = next(s for s in sections if s.startswith(name))
        assert "default 0" in section


def test_sensor_and_serial_have_distinct_pinned_owners():
    main = (PROJECT / "main/main.cpp").read_text()
    header = (PROJECT / "main/diagnostic_tasks.hpp").read_text()
    imu = (PROJECT / "main/imu_task.cpp").read_text()
    serial = (PROJECT / "main/serial_io.cpp").read_text()
    assert "kImuCore = 1" in header and "kSerialCore = 0" in header
    assert "portNUM_PROCESSORS >= 2" in header
    assert main.count("xTaskCreatePinnedToCore(") == 2
    assert "cleany::diagnostic::kImuCore" in main
    assert "cleany::diagnostic::kSerialCore" in main
    assert "usb_serial_jtag" not in imu and "cJSON" not in imu
    assert "mpu6050.hpp" not in serial and "readSample(" not in serial
    assert "xQueueOverwrite(channels_.latest" in imu
    assert "xQueuePeek(channels_.latest" in serial
    assert "xQueueSend(channels_.restarts" in serial
