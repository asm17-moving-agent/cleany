"""Check camera host deployment without changing the host or opening devices."""

from __future__ import annotations

import configparser
import importlib.util
from pathlib import Path
import shutil
import stat
import subprocess
import sys
from unittest.mock import Mock

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "camera_install", ROOT / "tools/camera/install.py"
)
assert SPEC is not None and SPEC.loader is not None
installer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(installer)


def test_checkout_payload_installs_and_uninstalls_in_isolated_cli(
    tmp_path: Path,
) -> None:
    root = tmp_path / "stage"
    command = [sys.executable, "-I", str(ROOT / "tools/camera/install.py")]
    subprocess.run(
        [*command, "install", "--root", str(root)],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    for source, target, mode in installer.FILES:
        assert (root / target).read_bytes() == (ROOT / source).read_bytes()
        assert stat.S_IMODE((root / target).stat().st_mode) == mode
    subprocess.run(
        [*command, "uninstall", "--root", str(root)],
        check=True,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert all(not (root / target).exists() for _, target, _ in installer.FILES)


@pytest.fixture
def sources(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    source_root = tmp_path / "source"
    for source, _, _ in installer.FILES:
        path = source_root / source
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"source: {source}\n")
    monkeypatch.setattr(installer, "REPO_ROOT", source_root)
    return source_root


def test_staged_install_is_repeatable_without_host_commands(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, sources: Path
) -> None:
    def forbidden(*_args: object, **_kwargs: object) -> None:
        pytest.fail("Staging must not run privileged host operations.")

    monkeypatch.setattr(installer, "require_jetson_host", forbidden)
    monkeypatch.setattr(installer.subprocess, "run", forbidden)
    destination = tmp_path / "root"
    for _ in range(2):
        assert installer.main(["install", "--root", str(destination)]) == 0
    for source, target, mode in installer.FILES:
        installed = destination / target
        assert installed.read_bytes() == (sources / source).read_bytes()
        assert stat.S_IMODE(installed.stat().st_mode) == mode


def test_staged_uninstall_only_removes_managed_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, sources: Path
) -> None:
    run = Mock()
    monkeypatch.setattr(installer.subprocess, "run", run)
    destination = tmp_path / "root"
    installer.install(destination)
    unrelated = destination / "etc/systemd/system/unrelated.service"
    unrelated.write_text("preserve")
    for _ in range(2):
        assert installer.main(["uninstall", "--root", str(destination)]) == 0
    assert unrelated.read_text() == "preserve"
    assert all(
        not (destination / target).exists() for _, target, _ in installer.FILES
    )
    run.assert_not_called()


@pytest.mark.parametrize("kind", ["file", "directory"])
def test_symlink_destination_is_rejected_before_any_copy(
    tmp_path: Path, sources: Path, kind: str
) -> None:
    destination = tmp_path / "root"
    outside = tmp_path / "outside"
    outside.mkdir()
    if kind == "file":
        target = destination / installer.HELPER
        target.parent.mkdir(parents=True)
        original = outside / "original"
        original.write_text("unchanged")
        target.symlink_to(original)
    else:
        destination.mkdir()
        (destination / "usr").symlink_to(outside, target_is_directory=True)
    assert installer.main(["install", "--root", str(destination)]) == 1
    assert not (destination / installer.RULE).exists()
    if kind == "file":
        assert original.read_text() == "unchanged"
    else:
        assert list(outside.iterdir()) == []


def test_missing_source_is_preflighted_before_replacement(
    tmp_path: Path, sources: Path
) -> None:
    destination = tmp_path / "root"
    previous = destination / installer.HELPER
    previous.parent.mkdir(parents=True)
    previous.write_text("old helper")
    (sources / installer.FILES[-1][0]).unlink()
    assert installer.main(["install", "--root", str(destination)]) == 1
    assert previous.read_text() == "old helper"
    assert not (destination / installer.UNIT).exists()


def test_symlink_is_also_rejected_on_uninstall(
    tmp_path: Path, sources: Path,
) -> None:
    destination = tmp_path / "root"
    installer.install(destination)
    target = destination / installer.HELPER
    outside = tmp_path / "unrelated"
    outside.write_text("preserve")
    target.unlink()
    target.symlink_to(outside)
    assert installer.main(["uninstall", "--root", str(destination)]) == 1
    assert outside.read_text() == "preserve"
    assert (destination / installer.RULE).is_file()


def test_staging_root_must_be_a_directory(
    tmp_path: Path, sources: Path
) -> None:
    destination = tmp_path / "file"
    destination.write_text("unchanged")
    assert installer.main(["install", "--root", str(destination)]) == 1
    assert destination.read_text() == "unchanged"


def test_live_install_requires_privilege_before_writing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(installer.os, "geteuid", lambda: 1000)
    install = Mock()
    monkeypatch.setattr(installer, "install", install)
    assert installer.main(["install"]) == 1
    install.assert_not_called()


@pytest.mark.parametrize(
    ("release", "machine"),
    [
        ({"ID": "ubuntu", "VERSION_ID": "20.04"}, "aarch64"),
        ({"ID": "ubuntu", "VERSION_ID": "22.04"}, "x86_64"),
        ({"ID": "fedora", "VERSION_ID": "44"}, "aarch64"),
    ],
)
def test_live_install_rejects_unsupported_platform(
    monkeypatch: pytest.MonkeyPatch, release: dict[str, str], machine: str
) -> None:
    monkeypatch.setattr(installer.os, "geteuid", lambda: 0)
    monkeypatch.setattr(installer.platform, "freedesktop_os_release", lambda: release)
    monkeypatch.setattr(installer.platform, "machine", lambda: machine)
    with pytest.raises(RuntimeError, match="Ubuntu 22.04 aarch64"):
        installer.require_jetson_host()


@pytest.mark.parametrize(
    ("compatible", "pid_one", "missing", "error"),
    [
        ("nvidia,tegra194\0", "systemd", None, "Jetson Orin"),
        ("nvidia,tegra234\0", "bash", None, "native systemd"),
        ("nvidia,tegra234\0", "systemd", "udevadm", "udevadm"),
    ],
)
def test_live_install_checks_board_host_and_commands(
    monkeypatch: pytest.MonkeyPatch,
    compatible: str,
    pid_one: str,
    missing: str | None,
    error: str,
) -> None:
    monkeypatch.setattr(installer.os, "geteuid", lambda: 0)
    monkeypatch.setattr(
        installer.platform,
        "freedesktop_os_release",
        lambda: {"ID": "ubuntu", "VERSION_ID": "22.04"},
    )
    monkeypatch.setattr(installer.platform, "machine", lambda: "aarch64")
    values = {
        "/proc/device-tree/compatible": compatible,
        "/proc/1/comm": pid_one,
    }
    monkeypatch.setattr(Path, "read_text", lambda path: values[str(path)])
    monkeypatch.setattr(
        installer.shutil, "which",
        lambda command: None if command == missing else f"/usr/bin/{command}",
    )
    with pytest.raises(RuntimeError, match=error):
        installer.require_jetson_host()


@pytest.mark.parametrize(
    "compatible",
    [
        "nvidia,p3767-0000\0nvidia,tegra234\0",
        "nvidia,p3701-0000\0nvidia,tegra234\0",
        "vendor,custom-carrier\0nvidia,tegra234\0",
    ],
)
def test_orin_including_custom_carrier_passes_host_preflight(
    monkeypatch: pytest.MonkeyPatch, compatible: str,
) -> None:
    monkeypatch.setattr(installer.os, "geteuid", lambda: 0)
    monkeypatch.setattr(
        installer.platform,
        "freedesktop_os_release",
        lambda: {"ID": "ubuntu", "VERSION_ID": "22.04"},
    )
    monkeypatch.setattr(installer.platform, "machine", lambda: "aarch64")
    values = {
        "/proc/device-tree/compatible": compatible,
        "/proc/1/comm": "systemd\n",
    }
    monkeypatch.setattr(Path, "read_text", lambda path: values[str(path)])
    monkeypatch.setattr(
        installer.shutil, "which", lambda command: f"/usr/bin/{command}"
    )
    installer.require_jetson_host()


def test_live_install_reloads_without_starting_camera(
    monkeypatch: pytest.MonkeyPatch, sources: Path
) -> None:
    check = Mock()
    copy = Mock()
    run = Mock()
    monkeypatch.setattr(installer, "check_destination", check)
    monkeypatch.setattr(installer, "install_file", copy)
    monkeypatch.setattr(installer.subprocess, "run", run)
    installer.install(Path("/"))
    assert check.call_count == len(installer.FILES)
    assert copy.call_count == len(installer.FILES)
    assert [call.args[0] for call in run.call_args_list] == [
        ["systemctl", "daemon-reload"],
        ["udevadm", "control", "--reload-rules"],
    ]


def test_reload_failure_is_reported(
    monkeypatch: pytest.MonkeyPatch, sources: Path,
) -> None:
    monkeypatch.setattr(installer, "require_jetson_host", Mock())
    monkeypatch.setattr(installer, "check_destination", Mock())
    monkeypatch.setattr(installer, "install_file", Mock())
    monkeypatch.setattr(
        installer.subprocess, "run",
        Mock(side_effect=subprocess.CalledProcessError(1, "systemctl")),
    )
    assert installer.main(["install"]) == 1


def test_live_uninstall_stops_helpers_before_removing_them(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    managed = {}
    for target in (installer.RULE, installer.UNIT, installer.HELPER):
        path = tmp_path / target
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("installed")
        managed[target] = path
    monkeypatch.setattr(
        installer, "check_destination", lambda _root, relative: managed[relative]
    )
    commands = []

    def run(command: list[str], *, check: bool) -> None:
        assert check
        assert not managed[installer.RULE].exists()
        if command[0:2] == ["systemctl", "stop"]:
            assert managed[installer.HELPER].exists()
            assert managed[installer.UNIT].exists()
        commands.append(command)

    monkeypatch.setattr(installer.subprocess, "run", run)
    installer.uninstall(Path("/"))
    assert commands == [
        ["udevadm", "control", "--reload-rules"],
        ["systemctl", "stop", "cleany-camera-orientation@*.service"],
        ["systemctl", "daemon-reload"],
    ]
    assert all(not path.exists() for path in managed.values())


def test_failed_stop_preserves_helper_and_unit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    managed = {}
    for target in (installer.RULE, installer.UNIT, installer.HELPER):
        path = tmp_path / target
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("installed")
        managed[target] = path
    monkeypatch.setattr(
        installer, "check_destination", lambda _root, relative: managed[relative]
    )

    def run(command: list[str], *, check: bool) -> None:
        assert check
        if command[0:2] == ["systemctl", "stop"]:
            raise subprocess.CalledProcessError(1, command)

    monkeypatch.setattr(installer.subprocess, "run", run)
    with pytest.raises(subprocess.CalledProcessError):
        installer.uninstall(Path("/"))
    assert not managed[installer.RULE].exists()
    assert managed[installer.HELPER].is_file()
    assert managed[installer.UNIT].is_file()


def test_unit_is_a_bounded_usb_parent_oneshot() -> None:
    unit = configparser.ConfigParser(interpolation=None)
    unit.optionxform = str
    unit.read(ROOT / "configs/host/systemd/cleany-camera-orientation@.service")
    service = unit["Service"]
    assert service["Type"] == "oneshot"
    assert service["RemainAfterExit"] == "no"
    assert service["Restart"] == "no"
    assert int(unit["Unit"]["StartLimitIntervalSec"]) == 0
    assert int(service["TimeoutStartSec"]) == 35
    assert service["ExecStart"] == (
        "/usr/bin/python3 -I /usr/local/libexec/cleany/camera-orientation.py "
        "--usb-device %i --roll 3"
    )
    assert "Install" not in unit
    assert service["User"] == "root"
    assert service.get("PrivateDevices", "no") == "no"
    assert service["DeviceAllow"] == "char-usb_device rw"
    assert service["ReadWritePaths"] == "/run/lock"
    assert service["ProtectHome"] == "yes"


def test_rule_matches_only_the_intended_usb_device_add() -> None:
    content = (
        ROOT / "configs/host/udev/70-cleany-camera-orientation.rules"
    ).read_text()
    rules = [
        line for line in content.splitlines()
        if line and not line.startswith("#")
    ]
    assert len(rules) == 1
    rule = rules[0]
    for match in (
        'ACTION=="add"', 'SUBSYSTEM=="usb"', 'ENV{DEVTYPE}=="usb_device"',
        'ATTR{idVendor}=="0bda"', 'ATTR{idProduct}=="5844"',
        'TAG+="systemd"',
        'ENV{SYSTEMD_WANTS}+="cleany-camera-orientation@%k.service"',
    ):
        assert match in rule
    assert "video4linux" not in rule
    assert "RUN" not in rule
    assert "ATTRS" not in rule


def test_systemd_unit_syntax() -> None:
    command = shutil.which("systemd-analyze")
    if command is None:
        pytest.skip("systemd-analyze is not installed")
    subprocess.run(
        [
            command, "verify",
            str(ROOT / "configs/host/systemd/cleany-camera-orientation@.service"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
