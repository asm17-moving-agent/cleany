#!/usr/bin/env python3
"""Apply the image-verified DECXIN 0bda:5844 UVC Roll value.

Run as root on the host, before capture services, with unrestricted host /proc.
Roll 3 is the tested 180-degree orientation; Roll 0 restores the original.
Omitting --roll queries only. Both modes briefly detach and restore uvcvideo.
The per-device lock coordinates these helpers, not camera applications. Reader
checks are snapshots, not an atomic exclusion of new opens: deployment must
order capture startup after this helper, including on USB replug.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes
from dataclasses import dataclass
import errno
import fcntl
import os
from pathlib import Path
import platform
import re
import signal
import stat
import struct
import sys
import time
from typing import Callable, Iterator, Sequence


class OrientationError(Exception):
    exit_code = 75


class Unsupported(OrientationError):
    exit_code = 2


class TemporaryError(OrientationError):
    pass


class ControlTransfer(ctypes.Structure):
    _fields_ = [
        ("request_type", ctypes.c_uint8), ("request", ctypes.c_uint8),
        ("value", ctypes.c_uint16), ("index", ctypes.c_uint16),
        ("length", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
        ("data", ctypes.c_void_p),
    ]


class DriverIoctl(ctypes.Structure):
    _fields_ = [
        ("interface", ctypes.c_int), ("request", ctypes.c_int),
        ("data", ctypes.c_void_p),
    ]


def ioctl_number(direction: int, number: int, size: int = 0) -> int:
    # asm-generic ioctl encoding shared by Linux aarch64 and x86_64.
    return (direction << 30) | (size << 16) | (ord("U") << 8) | number


USB_CONTROL = ioctl_number(3, 0, ctypes.sizeof(ControlTransfer))
USB_DRIVER_IOCTL = ioctl_number(3, 18, ctypes.sizeof(DriverIoctl))
USB_CLAIM = ioctl_number(2, 15, ctypes.sizeof(ctypes.c_uint))
USB_RELEASE = ioctl_number(2, 16, ctypes.sizeof(ctypes.c_uint))
USB_DISCONNECT = ioctl_number(0, 22)
USB_CONNECT = ioctl_number(0, 23)
SET_CUR, GET_CUR, GET_MIN, GET_MAX, GET_RES, GET_INFO, GET_DEF = (
    0x01, 0x81, 0x82, 0x83, 0x84, 0x86, 0x87
)
# Leave margin in the service's 35-second start budget for host /proc scans.
# Driver-bind ioctls have no userspace timeout; target rebind timing is separate.
READY_TIMEOUT = 10.0
RESTORE_TIMEOUT = 3.0
SCAN_TIMEOUT = 3.0
POLL_INTERVAL = 0.1
TRANSFER_TIMEOUT_MS = 1000
STOP_SIGNALS = {signal.SIGINT, signal.SIGTERM}
_stop_requested: int | None = None


@dataclass(frozen=True)
class Paths:
    usb: Path = Path("/sys/bus/usb/devices")
    dev: Path = Path("/dev")
    proc: Path = Path("/proc")
    locks: Path = Path("/run/lock")


@dataclass(frozen=True)
class Camera:
    name: str
    bus: int
    address: int
    inode: tuple[int, int]
    videos: tuple[tuple[str, int], ...]


@dataclass(frozen=True)
class RollState:
    info: int
    minimum: int
    maximum: int
    resolution: int
    default: int
    current: int
    changed: bool


def validate_name(name: str) -> str:
    if len(name) > 64 or not re.fullmatch(
        r"[1-9][0-9]*-[1-9][0-9]*(?:\.[1-9][0-9]*){0,5}", name
    ):
        raise Unsupported("Use a raw USB device kernel name, e.g. 1-2.3 (systemd %i).")
    return name


def check_platform() -> None:
    if (
        sys.platform != "linux" or platform.machine() not in ("aarch64", "x86_64")
        or ctypes.sizeof(ctypes.c_void_p) != 8 or sys.byteorder != "little"
    ):
        raise Unsupported("Supported platforms: little-endian Linux aarch64/x86_64.")
    if os.geteuid() != 0:
        raise Unsupported("Run as root with unrestricted host /proc access.")


def check_interruption() -> None:
    if _stop_requested is not None:
        raise TemporaryError(f"Interrupted by signal {_stop_requested}.")


def camera_snapshot(name: str, paths: Paths) -> Camera:
    """Resolve only the named USB parent and its interface-0 video nodes."""
    device = paths.usb / name
    identity = tuple((device / key).read_text().strip().lower()
                     for key in ("idVendor", "idProduct"))
    if identity != ("0bda", "5844"):
        raise Unsupported(f"{name}: expected USB 0bda:5844, found {identity}.")
    try:
        bus, address = (int((device / key).read_text())
                        for key in ("busnum", "devnum"))
    except ValueError as error:
        raise Unsupported("Malformed USB bus/address attributes.") from error
    if not (1 <= bus <= 999 and 1 <= address <= 127) or bus != int(name.split("-")[0]):
        raise Unsupported("Invalid or inconsistent USB bus/address.")
    if (device / "bConfigurationValue").read_text().strip() != "1":
        raise Unsupported("Expected USB configuration 1.")
    interface = device / f"{name}:1.0"
    for key, expected in (
        ("bInterfaceNumber", "00"), ("bInterfaceClass", "0e"),
        ("bInterfaceSubClass", "01"),
    ):
        if (interface / key).read_text().strip().lower() != expected:
            raise Unsupported("Expected UVC VideoControl interface 0.")
    driver = (interface / "driver").resolve(strict=True).name
    if driver != "uvcvideo":
        raise Unsupported(f"Refusing to detach unexpected driver {driver}.")
    videos = []
    for entry in (interface / "video4linux").iterdir():
        if not re.fullmatch(r"video[0-9]+", entry.name):
            continue
        try:
            major, minor = map(int, (entry / "dev").read_text().strip().split(":"))
            number = os.makedev(major, minor)
        except (ValueError, OverflowError) as error:
            raise Unsupported("Malformed video device number.") from error
        node = (paths.dev / entry.name).stat()
        if not stat.S_ISCHR(node.st_mode) or node.st_rdev != number:
            raise TemporaryError(f"{entry.name}: device node is not ready.")
        videos.append((entry.name, number))
    if not videos:
        raise TemporaryError(f"{name}: waiting for video nodes.")
    info = device.stat()
    return Camera(name, bus, address, (info.st_dev, info.st_ino), tuple(sorted(videos)))


def wait_ready(name: str, paths: Paths, previous: Camera | None = None) -> Camera:
    """Wait boundedly for a stable driver/node snapshot, including after CONNECT."""
    deadline = time.monotonic() + (READY_TIMEOUT if previous is None else RESTORE_TIMEOUT)
    last = None
    reason: Exception = TemporaryError("Device not ready.")
    while True:
        if previous is None:
            check_interruption()
        try:
            current = camera_snapshot(name, paths)
            if previous is not None and (
                current.inode != previous.inode or current.bus != previous.bus
                or current.address != previous.address
            ):
                raise TemporaryError("Camera disconnected or replaced during operation.")
            if current == last and (
                previous is None or len(current.videos) >= len(previous.videos)
            ):
                return current
            last = current
        except (FileNotFoundError, TemporaryError) as error:
            reason = error
            last = None
        if time.monotonic() >= deadline:
            raise TemporaryError(f"{name}: readiness timeout: {reason}") from reason
        time.sleep(POLL_INTERVAL)


@contextmanager
def blocked_signals() -> Iterator[None]:
    """Keep successful resource acquisition and its bookkeeping indivisible."""
    old = signal.pthread_sigmask(signal.SIG_BLOCK, STOP_SIGNALS)
    try:
        yield
    finally:
        signal.pthread_sigmask(signal.SIG_SETMASK, old)


def cleanup(actions: Sequence[tuple[str, Callable[[], object]]]) -> None:
    """Attempt every cleanup action and preserve an exception already in flight."""
    primary = sys.exc_info()[1]
    errors = []
    try:
        with blocked_signals():
            for label, action in actions:
                try:
                    action()
                except BaseException as error:
                    errors.append(f"{label}: {error}")
    except BaseException as error:
        errors.append(f"cleanup interrupted: {error}")
    if errors:
        message = "; ".join(errors)
        if primary is None:
            raise TemporaryError(f"Cleanup failed: {message}")
        print(f"Cleanup warning (primary error preserved): {message}", file=sys.stderr)


@contextmanager
def device_lock(name: str, paths: Paths) -> Iterator[None]:
    fd = None
    try:
        with blocked_signals():
            fd = os.open(
                paths.locks / f"cleany-camera-orientation-{name}.lock",
                os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600,
            )
        info = os.fstat(fd)
        if (
            not stat.S_ISREG(info.st_mode) or info.st_uid != 0
            or info.st_nlink != 1 or info.st_mode & 0o022
        ):
            raise Unsupported("Unsafe per-device lock file.")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise TemporaryError(f"{name}: another orientation helper holds the lock.") from error
        yield
    finally:
        if fd is not None:
            # Never unlink: waiters must continue to share the same inode.
            cleanup([("close lock", lambda: os.close(fd))])


def check_proc_visibility(proc: Path) -> None:
    """Reject restricted proc mounts and visibly nested PID namespaces."""
    mounts = (proc / "self/mountinfo").read_text().splitlines()
    found = False
    for line in mounts:
        before, after = line.split(" - ", 1)
        fields, fs = before.split(), after.split()
        if fields[4] == str(proc):
            found = fields[3] == "/" and fs[0] == "proc"
            options = fields[5].split(",") + fs[2].split(",")
            if any(opt.startswith("hidepid=") and opt != "hidepid=0" for opt in options):
                raise TemporaryError("Cannot establish reader visibility with hidepid.")
    if not found:
        raise TemporaryError("Cannot verify an unrestricted proc mount.")
    status = (proc / "self/status").read_text().splitlines()
    namespace = next((line.split()[1:] for line in status if line.startswith("NSpid:")), [])
    if len(namespace) != 1:
        raise TemporaryError("Reader inspection requires the host PID namespace.")
    if os.readlink(proc / "self/ns/pid") != os.readlink(proc / "1/ns/pid"):
        raise TemporaryError("Reader inspection requires the host PID namespace.")


def refuse_active_readers(camera: Camera, paths: Paths) -> None:
    """Refuse any open video fd, including aliases and non-shared thread fd tables.

    Inaccessible live tasks fail closed. Exited tasks and closed individual fds
    may disappear normally. This scan cannot prevent an open after the scan.
    """
    try:
        check_proc_visibility(paths.proc)
        deadline = time.monotonic() + SCAN_TIMEOUT
        targets = {number for _, number in camera.videos}
        processes = [p for p in paths.proc.iterdir() if p.name.isdecimal()]
        if not processes:
            raise TemporaryError("No processes visible for reader inspection.")
        for process in processes:
            check_interruption()
            try:
                tasks = list((process / "task").iterdir())
            except FileNotFoundError:
                if not process.exists():
                    continue
                raise
            if not tasks:
                raise TemporaryError(f"Cannot inspect tasks for PID {process.name}.")
            for task in tasks:
                try:
                    descriptors = list((task / "fd").iterdir())
                except FileNotFoundError:
                    if not task.exists():
                        continue
                    raise
                for fd in descriptors:
                    check_interruption()
                    if time.monotonic() >= deadline:
                        raise TemporaryError("Reader inspection timed out.")
                    try:
                        info = fd.stat()
                    except FileNotFoundError:
                        continue
                    if stat.S_ISCHR(info.st_mode) and info.st_rdev in targets:
                        raise TemporaryError(
                            f"Camera in use by PID {process.name}, TID {task.name}, fd {fd.name}."
                        )
                if time.monotonic() >= deadline:
                    raise TemporaryError("Reader inspection timed out.")
    except (OSError, ValueError, IndexError) as error:
        raise TemporaryError(f"Cannot reliably inspect camera readers: {error}") from error


def verify_open_device(fd: int, camera: Camera) -> None:
    info = os.fstat(fd)
    expected = os.makedev(189, (camera.bus - 1) * 128 + camera.address - 1)
    if not stat.S_ISCHR(info.st_mode) or info.st_rdev != expected:
        raise Unsupported("Opened USB node does not match the selected bus/address.")
    # usbfs read returns cached descriptors; no class or vendor request is sent.
    descriptor = os.pread(fd, 18, 0)
    if len(descriptor) != 18 or descriptor[:2] != b"\x12\x01":
        raise TemporaryError("Missing or malformed opened USB device descriptor.")
    if struct.unpack_from("<HH", descriptor, 8) != (0x0BDA, 0x5844):
        raise Unsupported("Opened USB device is not 0bda:5844; refusing detach.")


def driver_ioctl(fd: int, request: int) -> None:
    fcntl.ioctl(fd, USB_DRIVER_IOCTL, DriverIoctl(0, request, None), True)


def roll_transfer(fd: int, request: int, value: int | None = None) -> int:
    check_interruption()
    writing = request == SET_CUR
    if (
        request not in (SET_CUR, GET_CUR, GET_MIN, GET_MAX, GET_RES, GET_INFO, GET_DEF)
        or writing != (value is not None) or (writing and value not in (0, 3))
    ):
        raise Unsupported("Only verified Roll requests and SET values 0 or 3 are allowed.")
    size = 1 if request == GET_INFO else 2
    buffer = ctypes.create_string_buffer(size)
    if writing:
        buffer.raw = struct.pack("<h", value)
    packet = ControlTransfer(
        0x21 if writing else 0xA1, request, 0x0F00, 0x0100,
        size, TRANSFER_TIMEOUT_MS, ctypes.addressof(buffer),
    )
    count = fcntl.ioctl(fd, USB_CONTROL, packet, True)
    check_interruption()
    if count != size:
        raise TemporaryError(f"Short Roll USB transfer: {count}/{size} bytes.")
    return int.from_bytes(buffer.raw, "little", signed=size == 2)


def query_and_apply(fd: int, desired: int | None) -> RollState:
    info = roll_transfer(fd, GET_INFO)
    if info & 3 != 3 or info & 4:
        raise Unsupported(f"Roll must support GET/SET and be enabled (INFO=0x{info:02x}).")
    low, high, step = (roll_transfer(fd, request) for request in (GET_MIN, GET_MAX, GET_RES))
    if (low, high, step) != (0, 3, 1):
        raise Unsupported(f"Unverified Roll range: MIN={low} MAX={high} RES={step}.")
    default, current = (roll_transfer(fd, request) for request in (GET_DEF, GET_CUR))
    if default != 0 or not low <= current <= high:
        raise Unsupported(f"Unverified Roll default/current: {default}/{current}.")
    changed = desired is not None and desired != current
    if changed:
        roll_transfer(fd, SET_CUR, desired)
        current = roll_transfer(fd, GET_CUR)
        if current != desired:
            raise TemporaryError(f"Roll readback mismatch: expected {desired}, got {current}.")
    return RollState(info, low, high, step, default, current, changed)


def orient(name: str, desired: int | None = None, paths: Paths = Paths()) -> RollState:
    """Query/apply Roll, restoring the interface on errors and catchable signals."""
    validate_name(name)
    if desired is not None and desired not in (0, 3):
        raise Unsupported("Only Roll 0 and 3 have been image-verified.")
    check_platform()
    with device_lock(name, paths):
        camera = wait_ready(name, paths)
        refuse_active_readers(camera, paths)
        fd = None
        detached = claimed = False
        try:
            with blocked_signals():
                fd = os.open(
                    paths.dev / f"bus/usb/{camera.bus:03d}/{camera.address:03d}",
                    os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW,
                )
            verify_open_device(fd, camera)
            if camera_snapshot(name, paths) != camera:
                raise TemporaryError("Camera identity or video nodes changed before detach.")
            refuse_active_readers(camera, paths)
            check_interruption()
            with blocked_signals():
                driver_ioctl(fd, USB_DISCONNECT)
                detached = True
            check_interruption()
            with blocked_signals():
                fcntl.ioctl(fd, USB_CLAIM, struct.pack("@I", 0))
                claimed = True
            check_interruption()
            result = query_and_apply(fd, desired)
        finally:
            # All stages are attempted even if RELEASE fails or USB disappears.
            actions = []
            if claimed:
                actions.append(("release interface 0", lambda: fcntl.ioctl(
                    fd, USB_RELEASE, struct.pack("@I", 0))))
            if detached:
                actions.append(("reattach uvcvideo", lambda: driver_ioctl(fd, USB_CONNECT)))
            if fd is not None:
                actions.append(("close USB fd", lambda: os.close(fd)))
            if detached:
                actions.append(("verify driver/video restoration", lambda: wait_ready(
                    name, paths, previous=camera)))
            cleanup(actions)
        check_interruption()
        return result


@contextmanager
def termination_handlers() -> Iterator[None]:
    global _stop_requested

    def stop(signum: int, _frame: object) -> None:
        # Cooperative cancellation cannot raise between entering finally and
        # protecting cleanup. Transfers finish within their USB timeout first.
        global _stop_requested
        _stop_requested = signum

    old = {number: signal.getsignal(number) for number in STOP_SIGNALS}
    previous_request = _stop_requested
    _stop_requested = None
    try:
        for number in STOP_SIGNALS:
            signal.signal(number, stop)
        yield
    finally:
        for number, handler in old.items():
            signal.signal(number, handler)
        _stop_requested = previous_request


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usb-device", required=True, metavar="NAME",
                        help="Raw USB parent kernel name (systemd %%i), e.g. 1-2.3")
    parser.add_argument("--roll", type=int, choices=(0, 3),
                        help="Set verified raw Roll 0 or 3; omit for query-only")
    args = parser.parse_args(argv)
    try:
        with termination_handlers():
            state = orient(args.usb_device, args.roll)
        action = "set/read back" if state.changed else "unchanged"
        print(f"{args.usb_device} 0bda:5844 Roll={state.current} ({action}); "
              f"MIN={state.minimum} MAX={state.maximum} RES={state.resolution} "
              f"DEF={state.default} INFO=0x{state.info:02x}; uvcvideo restored")
        return 0
    except OrientationError as error:
        print(f"Error: {error}", file=sys.stderr)
        return error.exit_code
    except OSError as error:
        detail = "USB device disconnected" if error.errno in (errno.ENODEV, errno.ESHUTDOWN) else str(error)
        print(f"Temporary error: {detail}", file=sys.stderr)
        return 75
    except KeyboardInterrupt:
        print("Interrupted; interface cleanup attempted.", file=sys.stderr)
        return 75


if __name__ == "__main__":
    sys.exit(main())
