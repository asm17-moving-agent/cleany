"""Bounded JSONL host client for the standalone IMU calibration protocol."""
from __future__ import annotations

import argparse
import json
import math
import secrets
import tempfile
import time
from pathlib import Path
from typing import Any, Callable

SCHEMA = "cleany.imu_calibration/v1"
MAX_LINE_BYTES = 65536


class CaptureError(RuntimeError):
    """Serial, protocol, or deadline failure."""


def _validate_result(message: dict[str, Any]) -> dict[str, Any]:
    if message.get("type") != "result":
        return message
    if message.get("phase") not in ("completed", "failed") or not isinstance(message.get("accepted"), bool):
        raise CaptureError("malformed result phase/accepted")
    if not _integer(message.get("run_id")) or message["run_id"] < 0:
        raise CaptureError("malformed result run_id")
    if not isinstance(message.get("error"), str) or not isinstance(message.get("config"), dict):
        raise CaptureError("malformed result error/config")
    for key in ("candidate_bias_rad_s", "bias_rad_s", "validation_residual_rad_s"):
        value = message.get(key)
        if value is not None and (not isinstance(value, list) or len(value) != 3 or
                                  any(not _finite(x) for x in value)):
            raise CaptureError(f"malformed result {key}")
    if not isinstance(message.get("calibration"), dict) or not isinstance(message.get("validation"), dict):
        raise CaptureError("malformed result windows")
    for window in (message["calibration"], message["validation"]):
        if not _integer(window.get("samples")) or window["samples"] < 0:
            raise CaptureError("malformed window samples")
        for key in ("first_timestamp_us", "last_timestamp_us", "duration_s",
                    "temperature_min_c", "temperature_max_c"):
            if not _finite(window.get(key)):
                raise CaptureError(f"malformed window {key}")
        for key, n in (("gyro_mean_rad_s", 3), ("accel_mean_m_s2", 3),
                       ("gyro_covariance_rad2_s2", 9), ("accel_covariance_m2_s4", 9)):
            v = window.get(key)
            if not isinstance(v, list) or len(v) != n or any(not _finite(x) for x in v):
                raise CaptureError(f"malformed window {key}")
    if message["accepted"]:
        if (message["phase"] != "completed" or message["error"] != "none"
                or message["config"].get("limits_reviewed") is not True
                or any(message.get(k) is None for k in
                       ("bias_rad_s", "candidate_bias_rad_s", "validation_residual_rad_s"))):
            raise CaptureError("accepted result lacks reviewed limits or valid bias statistics")
    elif message.get("bias_rad_s") is not None:
        raise CaptureError("rejected result must not contain a usable bias")
    return message


def _finite(value: Any) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def create_serial_port() -> Any:
    """Create a POSIX port with atomic DTR/RTS updates during open.

    pySerial normally deasserts DTR before RTS. On native ESP32-S3 USB,
    the intervening control-line state can reset the MCU even when both
    attributes were set False before opening.
    """
    import fcntl
    import struct
    import termios
    import serial

    class AtomicSerial(serial.Serial):
        def _update_control_lines(self) -> None:
            try:
                # open() calls these hooks before is_open=True; use its valid fd,
                # not fileno(), which rejects that intermediate lifecycle state.
                flags = struct.unpack(
                    "I", fcntl.ioctl(self.fd, termios.TIOCMGET, struct.pack("I", 0))
                )[0]
                for bit, enabled in ((termios.TIOCM_DTR, self.dtr),
                                     (termios.TIOCM_RTS, self.rts)):
                    flags = flags | bit if enabled else flags & ~bit
                fcntl.ioctl(self.fd, termios.TIOCMSET, struct.pack("I", flags))
            except OSError as exc:
                raise CaptureError(f"atomic serial control-line update failed: {exc}") from exc

        def _update_dtr_state(self) -> None:
            self._update_control_lines()

        def _update_rts_state(self) -> None:
            self._update_control_lines()

    return AtomicSerial()


def capture(ser: Any, timeout: float = 60.0, calibrate: bool = False,
            clock: Callable[[], float] = time.monotonic,
            request_id: str | None = None) -> dict[str, Any]:
    """Query retained status, or start a fresh command run, until terminal/deadline."""
    if not _finite(timeout) or timeout <= 0:
        raise CaptureError("timeout must be finite and positive")
    if calibrate:
        request_id = request_id or secrets.token_hex(8)
        if (len(request_id) != 16
                or any(value not in "0123456789abcdef" for value in request_id)):
            raise CaptureError("request_id must be 16 lowercase hex digits")
    deadline = clock() + timeout
    buf = bytearray()
    run_id = None
    acknowledged = not calibrate
    calibrate_sent = False
    last_query = float("-inf")
    while clock() < deadline:
        if clock() - last_query >= 1.0:
            command = (f"calibrate {request_id}\n".encode("ascii")
                       if calibrate and not calibrate_sent else b"status\n")
            offset = 0
            calls = 0
            while offset < len(command):
                if clock() >= deadline or calls >= len(command):
                    raise CaptureError("serial write deadline/progress bound exceeded")
                try:
                    n = ser.write(command[offset:])
                except Exception as exc:
                    raise CaptureError(f"serial write failed: {exc}") from exc
                calls += 1
                if not _integer(n) or n <= 0 or n > len(command) - offset:
                    raise CaptureError("serial write returned invalid byte count")
                offset += n
            if command.startswith(b"calibrate "):
                calibrate_sent = True
            last_query = clock()
        try:
            chunk = ser.read(256)
        except Exception as exc:
            raise CaptureError(f"serial read failed: {exc}") from exc
        if not chunk:
            continue
        for byte in chunk:
            if byte == 10:
                line = bytes(buf).rstrip(b"\r")
                buf.clear()
                try:
                    msg = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, ValueError, RecursionError):
                    continue
                if not isinstance(msg, dict) or msg.get("schema") != SCHEMA:
                    continue
                kind = msg.get("type")
                if kind == "started":
                    if (calibrate and msg.get("trigger") == "command"
                            and msg.get("request_id") == request_id
                            and _integer(msg.get("run_id")) and msg["run_id"] >= 0):
                        acknowledged, run_id = True, msg["run_id"]
                elif kind == "result":
                    msg = _validate_result(msg)
                    if calibrate and (not acknowledged or msg["run_id"] != run_id):
                        continue
                    return msg
                elif kind == "status" and calibrate and isinstance(msg.get("run_id"), int):
                    # Status cannot establish that this is the newly commanded run.
                    continue
            else:
                if len(buf) >= MAX_LINE_BYTES:
                    buf.clear()
                    raise CaptureError("protocol line exceeds maximum length")
                buf.append(byte)
    raise CaptureError("capture timed out")


def save_report(path: str | Path, report: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=target.name + ".", suffix=".tmp", dir=target.parent)
    try:
        with open(fd, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(report, indent=2, allow_nan=False) + "\n")
            stream.flush()
        Path(tmp_name).replace(target)
    finally:
        Path(tmp_name).unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--calibrate", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    try:
        port = create_serial_port()
        port.port, port.baudrate = args.port, 115200
        port.timeout = port.write_timeout = min(0.2, args.timeout)
        port.dtr = False
        port.rts = False
        port.exclusive = True
        try:
            port.open()
            report = capture(port, args.timeout, args.calibrate)
        finally:
            port.close()
        save_report(args.output, report)
        print(json.dumps(report, indent=2))
        return 0 if report["accepted"] else 2
    except (CaptureError, OSError, ImportError) as exc:
        parser.exit(1, f"capture failed: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
