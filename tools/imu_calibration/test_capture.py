import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest

MODULE = Path(__file__).with_name("capture.py")
spec = importlib.util.spec_from_file_location("imu_capture", MODULE)
capture_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture_module)


def result(run_id=7, accepted=True):
    window = {"samples": 3, "first_timestamp_us": 1, "last_timestamp_us": 3, "duration_s": 0.02,
              "gyro_mean_rad_s": [0.0] * 3, "accel_mean_m_s2": [0.0] * 3,
              "gyro_covariance_rad2_s2": [0.0] * 9, "accel_covariance_m2_s4": [0.0] * 9,
              "temperature_min_c": 20, "temperature_max_c": 20}
    return {"schema": capture_module.SCHEMA, "type": "result", "run_id": run_id,
            "phase": "completed", "accepted": accepted, "error": "none" if accepted else "limits_not_reviewed",
            "candidate_bias_rad_s": [0.0] * 3, "bias_rad_s": [0.0] * 3 if accepted else None,
            "validation_residual_rad_s": [0.0] * 3, "config": {"limits_reviewed": accepted},
            "calibration": window, "validation": window}


class FakeSerial:
    def __init__(self, lines=(), chunksize=13):
        self.pending = bytearray(b"".join(json.dumps(x).encode() + b"\n" for x in lines))
        self.chunksize = chunksize
        self.writes = []
        self.transmitted = bytearray()
        self.write_limit = None

    def write(self, data):
        self.writes.append(bytes(data))
        size = min(len(data), self.write_limit) if self.write_limit else len(data)
        self.transmitted.extend(data[:size])
        return size

    def read(self, size):
        n = min(size, self.chunksize, len(self.pending))
        out = bytes(self.pending[:n])
        del self.pending[:n]
        return out


def test_retained_accepted_and_rejected_reports(tmp_path):
    for accepted in (True, False):
        report = result(accepted=accepted)
        ser = FakeSerial([report])
        assert capture_module.capture(ser)["accepted"] is accepted
        target = tmp_path / f"{accepted}.json"
        capture_module.save_report(target, report)
        assert json.loads(target.read_text()) == report


def test_calibrate_requires_command_ack_and_matching_run():
    nonce = "0123456789abcdef"
    ser = FakeSerial([
        {"schema": capture_module.SCHEMA, "type": "started", "trigger": "boot", "run_id": 2},
        result(2),
        {"schema": capture_module.SCHEMA, "type": "started", "trigger": "command",
         "run_id": 9, "request_id": "aaaaaaaaaaaaaaaa"},
        result(9),
        {"schema": capture_module.SCHEMA, "type": "started", "trigger": "command",
         "run_id": 3, "request_id": nonce},
        result(3),
    ])
    assert capture_module.capture(ser, calibrate=True, request_id=nonce)["run_id"] == 3
    assert ser.writes[0] == f"calibrate {nonce}\n".encode()
    assert all(not w.startswith(b"calibrate ") for w in ser.writes[1:])


def test_ignores_binary_and_wrong_schema_and_handles_partial_reads():
    ser = FakeSerial(chunksize=1)
    ser.pending.extend(b"\xffnot json\n" + b'{"schema":"other"}\n' + json.dumps(result()).encode() + b"\n")
    assert capture_module.capture(ser)["accepted"]


def test_valid_schema_malformed_result_and_oversized_line_fail():
    malformed = result()
    del malformed["config"]
    with pytest.raises(capture_module.CaptureError, match="config"):
        capture_module.capture(FakeSerial([malformed]))
    ser = FakeSerial()
    ser.pending.extend(b"x" * (capture_module.MAX_LINE_BYTES + 1))
    with pytest.raises(capture_module.CaptureError, match="maximum"):
        capture_module.capture(ser)


def test_timeout_is_bounded():
    class Clock:
        now = 0.0
        def __call__(self):
            self.now += 0.25
            return self.now
    with pytest.raises(capture_module.CaptureError, match="timed out"):
        capture_module.capture(FakeSerial(), timeout=1, clock=Clock())


def test_short_writes_and_invalid_or_failing_writes():
    ser = FakeSerial([result()])
    ser.write_limit = 2
    assert capture_module.capture(ser)["accepted"]
    assert ser.transmitted == b"status\n"
    for writer in (lambda data: 0, lambda data: (_ for _ in ()).throw(OSError("broken"))):
        ser = FakeSerial([result()])
        ser.write = writer
        with pytest.raises(capture_module.CaptureError, match="write"):
            capture_module.capture(ser)


@pytest.mark.parametrize("timeout", [float("nan"), float("inf"), -1, 0])
def test_capture_rejects_invalid_timeout(timeout):
    with pytest.raises(capture_module.CaptureError, match="timeout"):
        capture_module.capture(FakeSerial(), timeout=timeout)


def test_invalid_accepted_and_rejected_bias_envelopes():
    accepted = result()
    accepted["config"]["limits_reviewed"] = False
    with pytest.raises(capture_module.CaptureError, match="accepted result"):
        capture_module._validate_result(accepted)
    rejected = result(accepted=False)
    rejected["bias_rad_s"] = [0.0] * 3
    with pytest.raises(capture_module.CaptureError, match="rejected result"):
        capture_module._validate_result(rejected)


def test_cli_configures_serial_and_exit_codes(monkeypatch, tmp_path):
    class Port(FakeSerial):
        def __init__(self, report):
            super().__init__([report])
            self.opened = self.closed = False
            self.dtr = self.rts = True

        def open(self):
            self.opened = True

        def close(self):
            self.closed = True

    path = tmp_path / "report.json"
    port = Port(result())
    monkeypatch.setattr(capture_module, "create_serial_port", lambda: port)
    assert capture_module.main(["--port", "fake", "--output", str(path)]) == 0
    assert port.opened and port.closed and not port.dtr and not port.rts
    assert port.exclusive is True and port.baudrate == 115200
    assert port.write_timeout == 0.2 and port.timeout <= 0.2
    assert json.loads(path.read_text())["accepted"]

    port = Port(result(accepted=False))
    monkeypatch.setattr(capture_module, "create_serial_port", lambda: port)
    assert capture_module.main(["--port", "fake", "--output", str(path)]) == 2
    assert not json.loads(path.read_text())["accepted"]

    port = Port(result())
    port.pending.clear()
    monkeypatch.setattr(capture_module, "create_serial_port", lambda: port)
    with pytest.raises(SystemExit) as error:
        capture_module.main(["--port", "fake", "--output", str(path), "--timeout", "0.1"])
    assert error.value.code == 1 and port.closed


def test_cli_rejects_nonfinite_timeout():
    with pytest.raises(SystemExit) as error:
        capture_module.main(["--port", "fake", "--output", "unused", "--timeout", "nan"])
    assert error.value.code == 2


def test_sensor_failure_with_empty_windows_is_saved(tmp_path):
    report = result(accepted=False)
    report["phase"] = "failed"
    report["error"] = "sensor"
    report["candidate_bias_rad_s"] = None
    report["validation_residual_rad_s"] = None
    for window in (report["calibration"], report["validation"]):
        window["samples"] = 0
        for key in ("first_timestamp_us", "last_timestamp_us", "duration_s",
                    "temperature_min_c", "temperature_max_c"):
            window[key] = 0
    captured = capture_module.capture(FakeSerial([report]))
    target = tmp_path / "failure.json"
    capture_module.save_report(target, captured)
    assert json.loads(target.read_text())["error"] == "sensor"


@pytest.mark.parametrize("mutation", [
    lambda r: r.update(run_id=True),
    lambda r: r["calibration"].update(samples=-1),
    lambda r: r["calibration"].update(gyro_covariance_rad2_s2=[True] * 9),
    lambda r: r["validation"].update(temperature_max_c=float("nan")),
    lambda r: r.update(bias_rad_s=[10**400, 0, 0]),
])
def test_invalid_numeric_fields_are_protocol_errors(mutation):
    report = result()
    mutation(report)
    with pytest.raises(capture_module.CaptureError, match="malformed"):
        capture_module.capture(FakeSerial([report]))


def test_atomic_save_cleans_failed_temporary_file(tmp_path):
    with pytest.raises(ValueError):
        capture_module.save_report(tmp_path / "result.json", {"bad": float("nan")})
    assert list(tmp_path.iterdir()) == []


def test_fresh_command_gets_a_random_nonce(monkeypatch):
    nonce = "aaaaaaaaaaaaaaaa"
    monkeypatch.setattr(capture_module.secrets, "token_hex", lambda _: nonce)
    ser = FakeSerial([
        {"schema": capture_module.SCHEMA, "type": "started", "trigger": "command",
         "run_id": 12, "request_id": nonce},
        result(12),
    ])
    assert capture_module.capture(ser, calibrate=True)["run_id"] == 12
    assert ser.transmitted.startswith(f"calibrate {nonce}\n".encode())


@pytest.mark.parametrize("hook", ["_update_dtr_state", "_update_rts_state"])
def test_atomic_control_lines_clear_both_bits_before_open_finishes(monkeypatch, hook):
    import fcntl
    import struct
    import termios

    class Backend:
        fd = 123
        dtr = rts = False
        is_open = False

        def fileno(self):
            raise AssertionError("fileno is not valid during pySerial open")

    calls = []
    unrelated = 0x100

    def ioctl(fd, operation, data):
        assert fd == Backend.fd
        calls.append((operation, struct.unpack("I", data)[0]))
        return struct.pack("I", unrelated | termios.TIOCM_DTR | termios.TIOCM_RTS)

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=Backend))
    monkeypatch.setattr(fcntl, "ioctl", ioctl)
    port = capture_module.create_serial_port()
    getattr(port, hook)()
    assert calls == [(termios.TIOCMGET, 0), (termios.TIOCMSET, unrelated)]


def test_atomic_control_failure_is_not_silently_ignored(monkeypatch):
    import fcntl

    class Backend:
        fd = 123
        dtr = rts = False

    def failed(*args):
        raise OSError("unsupported ioctl")

    monkeypatch.setitem(sys.modules, "serial", types.SimpleNamespace(Serial=Backend))
    monkeypatch.setattr(fcntl, "ioctl", failed)
    port = capture_module.create_serial_port()
    with pytest.raises(capture_module.CaptureError, match="atomic serial"):
        port._update_dtr_state()
