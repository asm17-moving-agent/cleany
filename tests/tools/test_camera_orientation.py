"""Hardware-free USB ABI, safety-guard and lifecycle tests."""

import ctypes
import errno
import importlib.util
import os
from pathlib import Path
import signal
import stat
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[2] / "tools/camera/orientation.py"
SPEC = importlib.util.spec_from_file_location("camera_orientation", SOURCE)
orientation = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = orientation
SPEC.loader.exec_module(orientation)


@pytest.fixture(autouse=True)
def prohibit_hardware(monkeypatch):
    original_open = os.open

    def safe_open(path, *args, **kwargs):
        assert not str(path).startswith(("/dev/", "/sys/", "/proc/")), path
        return original_open(path, *args, **kwargs)

    def no_ioctl(*args, **kwargs):
        pytest.fail("An ioctl escaped the fake USB backend")

    monkeypatch.setattr(orientation.os, "open", safe_open)
    monkeypatch.setattr(orientation.fcntl, "ioctl", no_ioctl)


class Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakeCamera:
    def __init__(self, tmp_path, monkeypatch):
        self.paths = orientation.Paths(
            tmp_path / "sys", tmp_path / "dev", tmp_path / "proc", tmp_path / "locks"
        )
        self.name = "1-2.3"
        self.device = self.paths.usb / self.name
        self.interface = self.device / f"{self.name}:1.0"
        self.driver = tmp_path / "drivers/uvcvideo"
        self.driver.mkdir(parents=True)
        self.interface.mkdir(parents=True)
        self.driver_link = self.interface / "driver"
        self.driver_link.symlink_to(self.driver)
        for key, value in {
            "idVendor": "0bda", "idProduct": "5844", "busnum": "1",
            "devnum": "7", "bConfigurationValue": "1",
        }.items():
            (self.device / key).write_text(value)
        for key, value in {
            "bInterfaceNumber": "00", "bInterfaceClass": "0e", "bInterfaceSubClass": "01",
        }.items():
            (self.interface / key).write_text(value)
        self.paths.dev.mkdir()
        self.nodes = {}
        for index in (8, 9):
            video = self.interface / "video4linux" / f"video{index}"
            video.mkdir(parents=True)
            (video / "dev").write_text(f"81:{index}")
            node = self.paths.dev / video.name
            node.touch()
            self.nodes[str(node)] = os.makedev(81, index)
        self.usb_node = self.paths.dev / "bus/usb/001/007"
        self.usb_node.parent.mkdir(parents=True)
        descriptor = bytearray(18)
        descriptor[:2] = b"\x12\x01"
        struct.pack_into("<HH", descriptor, 8, 0x0BDA, 0x5844)
        self.usb_node.write_bytes(descriptor)
        self.paths.locks.mkdir()
        proc = self.paths.proc
        (proc / "self/ns").mkdir(parents=True)
        (proc / "1/ns").mkdir(parents=True)
        (proc / "1/task/1/fd").mkdir(parents=True)
        (proc / "self/ns/pid").symlink_to("pid:[123]")
        (proc / "1/ns/pid").symlink_to("pid:[123]")
        (proc / "self/status").write_text("Name:\ttest\nNSpid:\t1\n")
        (proc / "self/mountinfo").write_text(
            f"30 20 0:1 / {proc} rw,nosuid - proc proc rw\n"
        )
        self.log = []
        self.failures = {}
        self.hooks = {}
        self.current = 0
        self.readback = None
        self.responses = {
            orientation.GET_INFO: 3, orientation.GET_MIN: 0,
            orientation.GET_MAX: 3, orientation.GET_RES: 1, orientation.GET_DEF: 0,
        }
        self.usb_fd = None
        self.usb_rdev = os.makedev(189, 6)
        self.clock = Clock()
        monkeypatch.setattr(orientation.time, "monotonic", self.clock.now)
        monkeypatch.setattr(orientation.time, "sleep", self.clock.sleep)
        monkeypatch.setattr(orientation, "check_platform", lambda: None)

        original_stat = Path.stat
        original_fstat = os.fstat
        original_open = os.open
        original_close = os.close
        original_pread = os.pread

        def fake_stat(path, *, follow_symlinks=True):
            info = original_stat(path, follow_symlinks=follow_symlinks)
            target = os.path.realpath(path)
            if follow_symlinks and target in self.nodes:
                return SimpleNamespace(st_mode=stat.S_IFCHR | 0o660, st_rdev=self.nodes[target])
            return info

        def fake_fstat(fd):
            info = original_fstat(fd)
            if fd == self.usb_fd:
                return SimpleNamespace(st_mode=stat.S_IFCHR | 0o660, st_rdev=self.usb_rdev)
            return SimpleNamespace(
                st_mode=info.st_mode, st_uid=0, st_nlink=info.st_nlink
            )

        def fake_open(path, *args, **kwargs):
            fd = original_open(path, *args, **kwargs)
            if Path(path) == self.usb_node:
                self.log.append("open")
                self.usb_fd = fd
            return fd

        def fake_close(fd):
            if fd == self.usb_fd:
                self.log.append("close")
                self.usb_fd = None
            return original_close(fd)

        def fake_pread(fd, size, offset):
            assert fd == self.usb_fd
            self.log.append("descriptor")
            return original_pread(fd, size, offset)

        monkeypatch.setattr(Path, "stat", fake_stat)
        monkeypatch.setattr(orientation.os, "fstat", fake_fstat)
        monkeypatch.setattr(orientation.os, "open", fake_open)
        monkeypatch.setattr(orientation.os, "close", fake_close)
        monkeypatch.setattr(orientation.os, "pread", fake_pread)
        monkeypatch.setattr(orientation.fcntl, "ioctl", self.ioctl)

    def event(self, name):
        self.log.append(name)
        if name in self.hooks:
            self.hooks[name]()
        if name in self.failures:
            raise self.failures[name]

    def ioctl(self, fd, request, argument, mutate=True):
        assert fd == self.usb_fd
        if request == orientation.USB_DRIVER_IOCTL:
            assert (argument.interface, argument.data) == (0, None)
            if argument.request == orientation.USB_DISCONNECT:
                self.event("detach")
                self.driver_link.unlink()
            else:
                assert argument.request == orientation.USB_CONNECT
                self.event("connect")
                self.driver_link.symlink_to(self.driver)
            return 0
        if request in (orientation.USB_CLAIM, orientation.USB_RELEASE):
            assert argument == b"\x00\x00\x00\x00"
            self.event("claim" if request == orientation.USB_CLAIM else "release")
            return 0
        assert request == orientation.USB_CONTROL
        assert (argument.value, argument.index, argument.timeout) == (0x0F00, 0x0100, 1000)
        if argument.request == orientation.SET_CUR:
            assert (argument.request_type, argument.length) == (0x21, 2)
            value = ctypes.string_at(argument.data, 2)
            assert value in (b"\x00\x00", b"\x03\x00")
            self.event("set")
            self.current = int.from_bytes(value, "little", signed=True)
        else:
            assert argument.request_type == 0xA1
            self.event(argument.request)
            if argument.request == orientation.GET_CUR:
                value = self.current if self.readback is None or "set" not in self.log else self.readback
            else:
                value = self.responses[argument.request]
            assert argument.length == (1 if argument.request == orientation.GET_INFO else 2)
            payload = value.to_bytes(argument.length, "little", signed=argument.length == 2)
            ctypes.memmove(argument.data, payload, len(payload))
        return argument.length

    def run(self, desired=3):
        return orientation.orient(self.name, desired, self.paths)


@pytest.fixture
def camera(tmp_path, monkeypatch):
    return FakeCamera(tmp_path, monkeypatch)


def test_ioctl_abi_matches_linux_64_bit_headers():
    assert ctypes.sizeof(ctypes.c_void_p) == 8
    assert ctypes.sizeof(orientation.ControlTransfer) == 24
    assert ctypes.sizeof(orientation.DriverIoctl) == 16
    assert [getattr(orientation.ControlTransfer, field).offset for field in
            ("request_type", "request", "value", "index", "length", "timeout", "data")] == [
                0, 1, 2, 4, 6, 8, 16
            ]
    assert orientation.DriverIoctl.data.offset == 8
    assert (
        orientation.USB_CONTROL, orientation.USB_DRIVER_IOCTL,
        orientation.USB_CLAIM, orientation.USB_RELEASE,
        orientation.USB_DISCONNECT, orientation.USB_CONNECT,
    ) == (0xC0185500, 0xC0105512, 0x8004550F, 0x80045510, 0x5516, 0x5517)


@pytest.mark.parametrize("name", [
    "", "../1-2", "/sys/1-2", "1-2:1.0", "1/2", "1\\x2d2", "usb1",
    "0-1", "1-0", "01-2", "1-02", "1-2\n", "1-2.3.4.5.6.7.8", "1-" + "2" * 65,
])
def test_invalid_name_is_permanent_without_io(name):
    with pytest.raises(orientation.Unsupported):
        orientation.orient(name, 3)


@pytest.mark.parametrize("name", ["1-2", "1-2.3", "12-14.2.3.4.5.6"])
def test_valid_raw_kernel_name(name):
    assert orientation.validate_name(name) == name


@pytest.mark.parametrize("desired", [1, 2, -1, 4])
def test_unverified_desired_value_never_opens_usb(camera, desired):
    with pytest.raises(orientation.Unsupported):
        camera.run(desired)
    assert camera.log == []


@pytest.mark.parametrize("desired,initial,changed", [(3, 0, True), (0, 3, True), (3, 3, False), (None, 0, False)])
def test_full_lifecycle_and_idempotence(camera, desired, initial, changed):
    camera.current = initial
    result = camera.run(desired)
    assert result.current == (initial if desired is None else desired)
    assert result.changed == changed
    requests = [orientation.GET_INFO, orientation.GET_MIN, orientation.GET_MAX,
                orientation.GET_RES, orientation.GET_DEF, orientation.GET_CUR]
    if changed:
        requests += ["set", orientation.GET_CUR]
    assert camera.log == [
        "open", "descriptor", "detach", "claim", *requests, "release", "connect", "close"
    ]
    assert camera.driver_link.resolve() == camera.driver
    assert camera.usb_fd is None


@pytest.mark.parametrize("key,value", [
    ("idVendor", "ffff"), ("idProduct", "1234"), ("busnum", "2"),
    ("devnum", "0"), ("devnum", "128"), ("devnum", "bad"),
    ("bConfigurationValue", "2"),
])
def test_sysfs_guards_before_usb_open(camera, key, value):
    (camera.device / key).write_text(value)
    with pytest.raises(orientation.Unsupported):
        camera.run()
    assert camera.log == []


@pytest.mark.parametrize("key,value", [
    ("bInterfaceNumber", "01"), ("bInterfaceClass", "ff"), ("bInterfaceSubClass", "02"),
])
def test_interface_guards(camera, key, value):
    (camera.interface / key).write_text(value)
    with pytest.raises(orientation.Unsupported):
        camera.run()
    assert camera.log == []


def test_unknown_driver_is_not_detached(camera):
    other = camera.driver.parent / "other"
    other.mkdir()
    camera.driver_link.unlink()
    camera.driver_link.symlink_to(other)
    with pytest.raises(orientation.Unsupported, match="unexpected driver"):
        camera.run()
    assert camera.log == []


@pytest.mark.parametrize("mutation,error_type", [
    ("wrong_identity", orientation.Unsupported),
    ("short", orientation.TemporaryError),
    ("malformed", orientation.TemporaryError),
    ("wrong_node", orientation.Unsupported),
])
def test_opened_fd_must_match_before_detach(camera, mutation, error_type):
    descriptor = bytearray(camera.usb_node.read_bytes())
    if mutation == "wrong_identity":
        descriptor[8:12] = b"\xff" * 4
    elif mutation == "short":
        descriptor = descriptor[:10]
    elif mutation == "malformed":
        descriptor[1] = 2
    else:
        camera.usb_rdev = os.makedev(189, 7)
    camera.usb_node.write_bytes(descriptor)
    with pytest.raises(error_type):
        camera.run()
    assert "detach" not in camera.log
    assert camera.log[-1] == "close"


def test_sysfs_replacement_after_open_refuses_detach(camera, monkeypatch):
    verify = orientation.verify_open_device

    def verify_then_replace(fd, snapshot):
        verify(fd, snapshot)
        (camera.device / "devnum").write_text("8")

    monkeypatch.setattr(orientation, "verify_open_device", verify_then_replace)
    with pytest.raises(orientation.TemporaryError, match="changed before detach"):
        camera.run()
    assert camera.log == ["open", "descriptor", "close"]


@pytest.mark.parametrize("missing", ["driver", "video_directory", "node"])
def test_cold_add_readiness_timeout_is_bounded(camera, missing):
    if missing == "driver":
        camera.driver_link.unlink()
    elif missing == "node":
        (camera.paths.dev / "video9").unlink()
    else:
        (camera.interface / "video4linux").rename(camera.interface / "not-ready")
    with pytest.raises(orientation.TemporaryError, match="readiness timeout"):
        camera.run()
    assert orientation.READY_TIMEOUT <= camera.clock.value <= orientation.READY_TIMEOUT + 0.2
    assert camera.log == []


def test_wait_for_driver_then_apply(camera, monkeypatch):
    camera.driver_link.unlink()

    def sleep(seconds):
        camera.clock.sleep(seconds)
        if camera.clock.value >= 0.3 and not camera.driver_link.is_symlink():
            camera.driver_link.symlink_to(camera.driver)

    monkeypatch.setattr(orientation.time, "sleep", sleep)
    assert camera.run().current == 3


@pytest.mark.parametrize("control_request,value", [
    (orientation.GET_INFO, 0), (orientation.GET_INFO, 1), (orientation.GET_INFO, 2),
    (orientation.GET_INFO, 7), (orientation.GET_MIN, -1), (orientation.GET_MAX, 4),
    (orientation.GET_RES, 0), (orientation.GET_RES, 2), (orientation.GET_DEF, 1),
])
def test_unverified_capability_never_sets_and_restores(camera, control_request, value):
    camera.responses[control_request] = value
    with pytest.raises(orientation.Unsupported):
        camera.run()
    assert "set" not in camera.log
    assert camera.log[-3:] == ["release", "connect", "close"]


def test_out_of_range_current_is_rejected(camera):
    camera.current = -1
    with pytest.raises(orientation.Unsupported):
        camera.run()
    assert "set" not in camera.log


def test_readback_mismatch_is_temporary_and_restores(camera):
    camera.readback = 0
    with pytest.raises(orientation.TemporaryError, match="readback mismatch"):
        camera.run()
    assert camera.log.count("set") == 1
    assert camera.log[-3:] == ["release", "connect", "close"]


@pytest.mark.parametrize("stage,tail", [
    ("detach", ["close"]),
    ("claim", ["connect", "close"]),
    (orientation.GET_INFO, ["release", "connect", "close"]),
    (orientation.GET_CUR, ["release", "connect", "close"]),
    ("set", ["release", "connect", "close"]),
])
def test_failures_preserve_primary_and_cleanup(camera, stage, tail):
    primary = OSError(errno.EIO, "primary USB failure")
    camera.failures[stage] = primary
    with pytest.raises(OSError) as caught:
        camera.run()
    assert caught.value is primary
    assert camera.log[-len(tail):] == tail
    assert camera.usb_fd is None


def test_release_failure_does_not_prevent_connect(camera):
    camera.failures["release"] = OSError(errno.EIO, "release failed")
    with pytest.raises(orientation.TemporaryError, match="release failed"):
        camera.run()
    assert camera.log[-3:] == ["release", "connect", "close"]


def test_disconnect_during_query_preserves_primary(camera, capsys):
    primary = OSError(errno.ENODEV, "disconnected")
    camera.failures.update({
        orientation.GET_MIN: primary,
        "release": OSError(errno.ENODEV, "gone"),
        "connect": OSError(errno.ENODEV, "gone"),
    })
    with pytest.raises(OSError) as caught:
        camera.run()
    assert caught.value is primary
    assert camera.log[-3:] == ["release", "connect", "close"]
    assert "primary error preserved" in capsys.readouterr().err
    assert camera.clock.value <= orientation.RESTORE_TIMEOUT + 0.3


def test_connect_failure_is_not_success(camera):
    camera.failures["connect"] = OSError(errno.EIO, "connect failed")
    with pytest.raises(orientation.TemporaryError, match="connect failed"):
        camera.run()
    assert camera.log[-1] == "close"


def test_missing_restored_node_is_not_success(camera):
    camera.hooks["connect"] = lambda: (camera.paths.dev / "video9").unlink()
    with pytest.raises(orientation.TemporaryError, match="restoration"):
        camera.run()
    assert camera.usb_fd is None


def test_restoration_requires_original_video_node_count(camera):
    camera.hooks["connect"] = lambda: (
        camera.interface / "video4linux/video9"
    ).rename(camera.interface / "missing-video9")
    with pytest.raises(orientation.TemporaryError, match="restoration"):
        camera.run()
    assert camera.usb_fd is None


@pytest.mark.parametrize("stage,signum", [
    ("detach", signal.SIGTERM), ("claim", signal.SIGINT),
    (orientation.GET_CUR, signal.SIGTERM),
])
def test_signals_during_acquisition_and_query_restore(camera, stage, signum):
    camera.hooks[stage] = lambda: os.kill(os.getpid(), signum)
    previous = signal.getsignal(signum)
    with orientation.termination_handlers():
        with pytest.raises(orientation.TemporaryError, match="signal"):
            camera.run()
    assert signal.getsignal(signum) == previous
    assert "connect" in camera.log
    assert camera.log[-1] == "close"
    if stage == "claim":
        assert "release" in camera.log


def test_signal_during_cleanup_cannot_skip_close(camera):
    camera.hooks["release"] = lambda: os.kill(os.getpid(), signal.SIGTERM)
    with orientation.termination_handlers():
        with pytest.raises(orientation.TemporaryError, match="signal"):
            camera.run()
    assert camera.log[-3:] == ["release", "connect", "close"]


def test_signal_at_cleanup_entry_cannot_skip_restoration(camera, monkeypatch):
    cleanup = orientation.cleanup

    def stop_then_cleanup(actions):
        os.kill(os.getpid(), signal.SIGTERM)
        return cleanup(actions)

    monkeypatch.setattr(orientation, "cleanup", stop_then_cleanup)
    with orientation.termination_handlers():
        with pytest.raises(orientation.TemporaryError, match="signal"):
            camera.run()
    assert camera.log[-3:] == ["release", "connect", "close"]


def test_signal_during_cleanup_does_not_mask_primary(camera):
    primary = OSError(errno.EIO, "primary query error")
    camera.failures[orientation.GET_CUR] = primary
    camera.hooks["release"] = lambda: os.kill(os.getpid(), signal.SIGTERM)
    with orientation.termination_handlers():
        with pytest.raises(OSError) as caught:
            camera.run()
    assert caught.value is primary
    assert camera.log[-3:] == ["release", "connect", "close"]


def test_keyboard_interrupt_runs_cleanup(camera):
    camera.failures[orientation.GET_INFO] = KeyboardInterrupt()
    with pytest.raises(KeyboardInterrupt):
        camera.run()
    assert camera.log[-3:] == ["release", "connect", "close"]


@pytest.mark.parametrize("thread", ["1", "99"])
def test_active_video_fd_in_any_thread_refuses_detach(camera, thread):
    fd_directory = camera.paths.proc / f"1/task/{thread}/fd"
    fd_directory.mkdir(parents=True, exist_ok=True)
    (fd_directory / "42").symlink_to(camera.paths.dev / "video8")
    with pytest.raises(orientation.TemporaryError, match="Camera in use"):
        camera.run()
    assert camera.log == []


def test_unrelated_device_fd_is_not_a_camera_reader(camera):
    other = camera.paths.dev / "video99"
    other.touch()
    camera.nodes[str(other)] = os.makedev(81, 99)
    (camera.paths.proc / "1/task/1/fd/42").symlink_to(other)
    assert camera.run().current == 3


def test_uninspectable_individual_fd_fails_closed(camera, monkeypatch):
    target = camera.paths.proc / "1/task/1/fd/42"
    target.symlink_to(camera.paths.dev / "video8")
    original = Path.stat

    def denied(path, **kwargs):
        if path == target:
            raise PermissionError(errno.EACCES, "fd access denied")
        return original(path, **kwargs)

    monkeypatch.setattr(Path, "stat", denied)
    with pytest.raises(orientation.TemporaryError, match="reliably inspect"):
        camera.run()
    assert camera.log == []


def test_reader_appearing_after_first_scan_is_refused(camera, monkeypatch):
    scan = orientation.refuse_active_readers
    calls = []

    def inspect(snapshot, paths):
        calls.append(1)
        if len(calls) == 2:
            (camera.paths.proc / "1/task/1/fd/42").symlink_to(camera.paths.dev / "video9")
        scan(snapshot, paths)

    monkeypatch.setattr(orientation, "refuse_active_readers", inspect)
    with pytest.raises(orientation.TemporaryError, match="Camera in use"):
        camera.run()
    assert len(calls) == 2
    assert camera.log == ["open", "descriptor", "close"]


@pytest.mark.parametrize("failure", [PermissionError(errno.EACCES, "denied"), FileNotFoundError()])
def test_uninspectable_process_fails_closed(camera, monkeypatch, failure):
    original = Path.iterdir

    def iterdir(path):
        if path == camera.paths.proc / "1/task/1/fd":
            raise failure
        return original(path)

    monkeypatch.setattr(Path, "iterdir", iterdir)
    with pytest.raises(orientation.TemporaryError, match="reliably inspect"):
        camera.run()
    assert camera.log == []


@pytest.mark.parametrize("kind", ["process", "thread"])
def test_exited_process_or_thread_does_not_block_initialization(
    camera, monkeypatch, kind
):
    if kind == "process":
        directory = camera.paths.proc / "2"
        (directory / "task/2/fd").mkdir(parents=True)
        inspected = directory / "task"
    else:
        directory = camera.paths.proc / "1/task/99"
        (directory / "fd").mkdir(parents=True)
        inspected = directory / "fd"
    original = Path.iterdir

    def iterdir(path):
        if path == inspected and directory.exists():
            directory.rename(camera.paths.proc / "exited")
            raise FileNotFoundError(errno.ENOENT, "task exited")
        return original(path)

    monkeypatch.setattr(Path, "iterdir", iterdir)
    assert camera.run().current == 3


def test_configured_timeouts_fit_service_budgets():
    start_budget = (
        orientation.READY_TIMEOUT + 2 * orientation.SCAN_TIMEOUT
        + 8 * orientation.TRANSFER_TIMEOUT_MS / 1000
        + orientation.RESTORE_TIMEOUT + 1
    )
    stop_budget = (
        orientation.TRANSFER_TIMEOUT_MS / 1000 + orientation.RESTORE_TIMEOUT + 0.5
    )
    assert start_budget < 35
    assert stop_budget < 5


@pytest.mark.parametrize("restriction", ["hidepid", "nested", "missing_status", "bad_mount"])
def test_restricted_proc_fails_closed(camera, restriction):
    proc = camera.paths.proc
    if restriction == "hidepid":
        mount = proc / "self/mountinfo"
        mount.write_text(mount.read_text().replace("proc proc rw", "proc proc rw,hidepid=2"))
    elif restriction == "nested":
        (proc / "self/status").write_text("NSpid:\t123\t1\n")
    elif restriction == "missing_status":
        (proc / "self/status").unlink()
    else:
        (proc / "self/mountinfo").write_text("not valid\n")
    with pytest.raises(orientation.TemporaryError):
        camera.run()
    assert camera.log == []


def test_bounded_process_scan(camera, monkeypatch):
    original = Path.iterdir

    def iterdir(path):
        if path == camera.paths.proc / "1/task/1/fd":
            camera.clock.sleep(orientation.SCAN_TIMEOUT)
        return original(path)

    monkeypatch.setattr(Path, "iterdir", iterdir)
    with pytest.raises(orientation.TemporaryError, match="inspection timed out"):
        camera.run()
    assert camera.log == []


def test_lock_contention_never_opens_usb(camera):
    with orientation.device_lock(camera.name, camera.paths):
        with pytest.raises(orientation.TemporaryError, match="holds the lock"):
            camera.run()
    assert camera.log == []
    assert camera.run().current == 3


def test_unsafe_lock_permissions_are_rejected(camera):
    lock = camera.paths.locks / f"cleany-camera-orientation-{camera.name}.lock"
    lock.touch(mode=0o666)
    lock.chmod(0o666)
    with pytest.raises(orientation.Unsupported, match="Unsafe"):
        camera.run()
    assert camera.log == []


@pytest.mark.parametrize("control_request,value", [
    (0x85, None), (0x99, None), (orientation.SET_CUR, None),
    (orientation.SET_CUR, 1), (orientation.SET_CUR, 2), (orientation.GET_CUR, 3),
])
def test_transfer_whitelist_rejects_unverified_operations(control_request, value):
    with pytest.raises(orientation.Unsupported):
        orientation.roll_transfer(-1, control_request, value)


@pytest.mark.parametrize("control_request,size", [
    (orientation.GET_INFO, 1), (orientation.GET_CUR, 2), (orientation.SET_CUR, 2),
])
def test_short_transfers_are_errors(monkeypatch, control_request, size):
    monkeypatch.setattr(orientation.fcntl, "ioctl", lambda *_: size - 1)
    with pytest.raises(orientation.TemporaryError, match="Short Roll"):
        orientation.roll_transfer(
            -1, control_request, 3 if control_request == orientation.SET_CUR else None
        )


def test_get_values_are_signed_little_endian(monkeypatch):
    def ioctl(_fd, _request, packet, _mutate):
        ctypes.memmove(packet.data, b"\x85\xff", 2)
        return 2

    monkeypatch.setattr(orientation.fcntl, "ioctl", ioctl)
    assert orientation.roll_transfer(-1, orientation.GET_MIN) == -123


@pytest.mark.parametrize("argv", [[], ["--roll", "3"], ["--usb-device", "1-2", "--roll", "1"]])
def test_cli_argument_errors_exit_two(argv):
    with pytest.raises(SystemExit) as caught:
        orientation.main(argv)
    assert caught.value.code == 2


def test_isolated_cli_process_rejects_invalid_name_without_device_access():
    result = subprocess.run(
        [sys.executable, "-I", str(SOURCE), "--usb-device", "../1-2", "--roll", "3"],
        capture_output=True, text=True, timeout=5, check=False,
    )
    assert result.returncode == 2
    assert "raw USB device kernel name" in result.stderr
    assert result.stdout == ""


@pytest.mark.parametrize("error,expected,text", [
    (orientation.Unsupported("wrong camera"), 2, "wrong camera"),
    (orientation.TemporaryError("busy"), 75, "busy"),
    (OSError(errno.ENODEV, "gone"), 75, "disconnected"),
    (OSError(errno.EACCES, "denied"), 75, "denied"),
    (KeyboardInterrupt(), 75, "Interrupted"),
])
def test_cli_error_codes(monkeypatch, capsys, error, expected, text):
    def fail(*_):
        raise error

    monkeypatch.setattr(orientation, "orient", fail)
    assert orientation.main(["--usb-device", "1-2", "--roll", "3"]) == expected
    assert text in capsys.readouterr().err


@pytest.mark.parametrize("extra,desired", [([], None), (["--roll", "3"], 3), (["--roll", "0"], 0)])
def test_cli_success_passes_raw_name_and_optional_roll(monkeypatch, capsys, extra, desired):
    calls = []

    def run(name, value):
        calls.append((name, value))
        return orientation.RollState(3, 0, 3, 1, 0, value or 0, value is not None)

    monkeypatch.setattr(orientation, "orient", run)
    assert orientation.main(["--usb-device", "1-2.3", *extra]) == 0
    assert calls == [("1-2.3", desired)]
    assert "uvcvideo restored" in capsys.readouterr().out


@pytest.mark.parametrize("machine", ["aarch64", "x86_64"])
def test_supported_platforms(monkeypatch, machine):
    monkeypatch.setattr(orientation.platform, "machine", lambda: machine)
    monkeypatch.setattr(orientation.os, "geteuid", lambda: 0)
    orientation.check_platform()


def test_non_root_is_configuration_error(monkeypatch):
    monkeypatch.setattr(orientation.os, "geteuid", lambda: 1000)
    with pytest.raises(orientation.Unsupported, match="root"):
        orientation.check_platform()
