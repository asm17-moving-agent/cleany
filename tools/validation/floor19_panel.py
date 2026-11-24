"""Laptop controls for the live Floor 19 field mapping session (GTK 4 over SSH)."""

import fcntl
import json
import os
import re
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from contextlib import closing, contextmanager
from datetime import datetime
from pathlib import Path

MAPPING = "cleany-floor19-mapping.service"
RECORDERS = "cleany-floor19-record*"
VIEW = "cleany-floor19-rviz.service"
OVERLAYS = "cleany-floor19-view-overlays.service"
RUNS = Path("/var/opt/cleany/runs/mapping")
CAPTURE = "/opt/cleany/tools/floor19-capture"
GIB = 1024 ** 3
DISPLAY_RESERVE = 10 * GIB
SENSOR_TOPICS = {
    "/camera/sdk/color/image_raw/compressed": "RGB",
    "/camera/aligned_depth_to_color/image_raw": "Depth",
    "/camera/infra1/image_rect_raw": "왼쪽 IR",
    "/camera/infra2/image_rect_raw": "오른쪽 IR",
    "/floor19/scan": "LiDAR",
}


def missing_sensors(path: Path, now_ns: int) -> list[str] | None:
    """Check recent recorder timestamps across the current and previous SQLite splits."""
    suffix = re.compile(re.escape(path.name) + r"_(\d+)\.db3")
    parts = sorted((int(m[1]), f) for f in path.glob("*.db3") if (m := suffix.fullmatch(f.name)))
    recent = set()
    readable = False
    for _, part in parts[-2:]:
        try:
            with closing(sqlite3.connect(part.as_uri() + "?mode=ro", uri=True, timeout=0.15)) as db:
                rows = db.execute(
                    "SELECT DISTINCT t.name FROM messages AS m JOIN topics AS t ON t.id=m.topic_id "
                    "WHERE m.timestamp BETWEEN ? AND ?", (now_ns - 8_000_000_000, now_ns))
                recent.update(row[0] for row in rows)
                readable = True
        except sqlite3.OperationalError:
            continue
    return [label for topic, label in SENSOR_TOPICS.items() if topic not in recent] if readable else None


def command(*args: str, timeout: int = 30) -> str:
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        tail = (result.stderr or result.stdout).strip().splitlines()
        raise RuntimeError(tail[-1] if tail else f"{args[0]} 종료 코드 {result.returncode}")
    return result.stdout.strip()


def unit_state() -> dict[str, str]:
    data = command("systemctl", "--user", "show", MAPPING, "-p", "ActiveState",
                   "-p", "MainPID", "-p", "InvocationID")
    return dict(line.split("=", 1) for line in data.splitlines())


def recorder_directory(pid: int) -> Path | None:
    """Find the output selected by the live ros2 bag record process."""
    proc = Path(f"/proc/{pid}")
    argv = os.fsdecode((proc / "cmdline").read_bytes()).strip("\0").split("\0")
    for index, arg in enumerate(argv):
        if arg in ("-o", "--output") and index + 1 < len(argv):
            return (proc / "cwd" / argv[index + 1]).resolve()
        if arg.startswith("--output="):
            return (proc / "cwd" / arg.partition("=")[2]).resolve()
    # With an automatically named bag, the writer holds an open storage file.
    for fd in (proc / "fd").iterdir():
        dest = fd.resolve()
        if dest.suffix in (".db3", ".mcap") and dest.is_file():
            return dest.parent
    return None


def recorder_mode(invocation: str) -> str | None:
    """Read the state acknowledged by this recorder invocation on the Jetson."""
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    try:
        mode = (runtime / f"floor19_recorder_{invocation}.state").read_text().strip()
    except FileNotFoundError:
        return None
    return mode if mode in ("recording", "paused") else None


def recording_status() -> dict:
    """Measure the newest active Floor 19 recorder and its destination filesystem."""
    info = command("systemctl", "--user", "show", RECORDERS,
                   "-p", "Id", "-p", "ActiveState", "-p", "MainPID", "-p", "InvocationID",
                   "-p", "ExecMainStartTimestampMonotonic")
    units = [dict(line.split("=", 1) for line in block.splitlines())
             for block in info.split("\n\n") if block]
    active = [u for u in units if u["ActiveState"] == "active" and int(u["MainPID"])]
    now = time.monotonic()
    bag = None
    disk = RUNS
    if active:
        unit = max(active, key=lambda u: int(u["ExecMainStartTimestampMonotonic"]))
        start = int(unit["ExecMainStartTimestampMonotonic"]) / 1_000_000
        bag = dict(id=unit["InvocationID"], unit=unit["Id"], seconds=max(0, now - start),
                   path=None, size=None, mode=recorder_mode(unit["InvocationID"]))
        try:
            path = recorder_directory(int(unit["MainPID"]))
            if path and path.is_dir():
                disk = path
                bag["path"] = str(path)
                bag["size"] = sum(f.stat().st_size for f in path.iterdir() if f.is_file())
        except FileNotFoundError:
            pass  # Recorder exit or storage rotation during the sample.
    fs = os.statvfs(disk)
    return dict(bag=bag, sample=now, free=fs.f_bavail * fs.f_frsize,
                disk=str(disk), recorder_count=len(active))


class RecordingRate:
    """Estimate bytes per second from one recorder's recent bag growth."""

    def __init__(self):
        self.identity = None
        self.samples = deque()

    def update(self, bag: dict | None, now: float) -> float | None:
        if not bag or bag["size"] is None:
            self.samples.clear()
            self.identity = None
            return None
        size = bag["size"]
        if (bag["id"] != self.identity or
                (self.samples and (size < self.samples[-1][1] or now <= self.samples[-1][0]))):
            self.samples.clear()
        self.identity = bag["id"]
        while self.samples and now - self.samples[0][0] > 60:
            self.samples.popleft()
        self.samples.append((now, size))
        elapsed = now - self.samples[0][0]
        growth = size - self.samples[0][1]
        return growth / elapsed if elapsed >= 15 and growth > 0 else None


def byte_label(value: int) -> str:
    if value >= GIB:
        return f"{value / GIB:.2f} GiB"
    return f"{value / 1024 ** 2:.1f} MiB"


def duration_label(seconds: float) -> str:
    hours, remainder = divmod(int(seconds), 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def estimate_label(free: int, rate: float | None) -> str:
    if free <= DISPLAY_RESERVE:
        return "0분"
    if not rate:
        return "속도 측정 중"
    minutes = int((free - DISPLAY_RESERVE) / rate / 60)
    if minutes < 1:
        return "1분 미만"
    hours, minutes = divmod(minutes, 60)
    return f"약 {hours}시간 {minutes}분" if hours else f"약 {minutes}분"


def save_map(run: Path) -> Path:
    """Publish one consistent Cartographer snapshot with a portable PGM/YAML pair."""
    checkpoints = run / "checkpoints"
    checkpoints.mkdir(exist_ok=True)
    target = checkpoints / datetime.now().strftime("%Y-%m-%d_%H%M%S_%f")
    session = str(run / "session")
    with tempfile.TemporaryDirectory(dir=checkpoints, prefix=".saving-") as tmp:
        stage = Path(tmp)
        state = stage / "map.pbstream"
        request = json.dumps(dict(filename=str(state), include_unfinished_submaps=True))
        reply = command(session, "exec", "ros2", "service", "call", "/floor19/write_state",
                        "cartographer_ros_msgs/srv/WriteState", request, timeout=90)
        if "code=0," not in reply or not state.is_file() or state.stat().st_size == 0:
            raise RuntimeError(f"Cartographer 지도 저장 실패 {reply[-300:]}")
        command(session, "exec", "ros2", "run", "cartographer_ros",
                "cartographer_pbstream_to_ros_map", "-pbstream_filename", str(state),
                "-map_filestem", str(stage / "map"), "-resolution", "0.05", timeout=90)
        image = stage / "map.pgm"
        meta = stage / "map.yaml"
        if image.stat().st_size < 12 or not meta.stat().st_size:
            raise RuntimeError("지도 내보내기 결과가 비어 있습니다.")
        text = meta.read_text()
        if str(image) not in text:
            raise RuntimeError("지도 YAML의 이미지 경로를 확인할 수 없습니다.")
        meta.write_text(text.replace(str(image), "map.pgm", 1))
        # Rename the directory as one unit so readers see all three files together.
        os.replace(stage, target)
    return target


@contextmanager
def pinned_unit(unit: str):
    """Keep a transient recorder's exact unit definition loaded across stop/start."""
    import dbus

    bus = dbus.SessionBus()
    manager = dbus.Interface(bus.get_object("org.freedesktop.systemd1", "/org/freedesktop/systemd1"),
                             "org.freedesktop.systemd1.Manager")
    manager.RefUnit(unit)
    try:
        yield
    finally:
        manager.UnrefUnit(unit)


def bag_files(path: Path, prefix: str) -> list[Path]:
    """Accept only the SQLite bag's own split files, sidecars and metadata."""
    storage = re.compile(re.escape(prefix) + r"_\d+\.db3")
    member = re.compile(storage.pattern + r"(?:-wal|-shm|-journal)?")
    files = list(path.iterdir())
    if (path.is_symlink() or not any(storage.fullmatch(f.name) for f in files)
            or any(f.is_symlink() or not f.is_file()
                   or (f.name != "metadata.yaml" and not member.fullmatch(f.name)) for f in files)):
        raise RuntimeError(f"bag 외의 파일이 있거나 SQLite bag을 확인할 수 없습니다. {path}")
    return files


def wait_for_new_bag(unit: str, old_id: str, run: Path, timeout: float = 25) -> Path:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        replacement = recording_status()["bag"]
        if (not replacement or replacement["unit"] != unit or replacement["id"] == old_id
                or not replacement["path"]):
            time.sleep(0.5)
            continue
        path = Path(replacement["path"])
        if path.parent != run:
            raise RuntimeError(f"새 bag의 저장 경로가 바뀌었습니다. {path}")
        for part in path.glob("*.db3"):
            try:
                with closing(sqlite3.connect(part.as_uri() + "?mode=ro", uri=True, timeout=0.3)) as db:
                    if db.execute("SELECT 1 FROM messages LIMIT 1").fetchone():
                        return path
            except sqlite3.OperationalError:
                pass  # The recorder may still be creating the database.
        time.sleep(0.3)
    raise RuntimeError("새 bag에 저장된 메시지를 확인할 수 없습니다.")


def restart_mapper(run: Path, expected: str) -> None:
    if unit_state()["InvocationID"] != expected:
        raise RuntimeError("매핑 세션이 바뀌었습니다.")
    resume = run / "resume.pbstream"
    if os.path.lexists(resume):
        resume.rename(run / f"previous-resume-{time.time_ns()}.pbstream")
    command("systemctl", "--user", "restart", MAPPING, timeout=125)
    new = unit_state()
    if new["ActiveState"] != "active" or new["InvocationID"] == expected:
        raise RuntimeError("새 매핑 서비스가 시작되지 않았습니다.")
    # An empty new trajectory exists even while wheel odometry is still offline.
    reply = command(str(run / "session"), "exec", "ros2", "service", "call",
                    "/floor19/get_trajectory_states", "cartographer_ros_msgs/srv/GetTrajectoryStates",
                    "{}", timeout=35)
    if not all(field in reply for field in ("code=0,", "trajectory_id=[0]", "trajectory_state=[0]")):
        raise RuntimeError("빈 지도에서 시작한 새 궤적을 확인할 수 없습니다.")


def confirmed_bag(expected: str) -> dict:
    status = recording_status()
    bag = status["bag"]
    if not bag or status["recorder_count"] != 1 or bag["id"] != expected or not bag["path"]:
        raise RuntimeError("녹화 세션이 바뀌었습니다. 현재 bag을 다시 확인해 주세요.")
    return bag


def control_bag(action: str, expected: str) -> dict:
    bag = confirmed_bag(expected)
    path = Path(bag["path"])
    run = path.parent
    if run.parent != RUNS or path.resolve() != path or not (run / "session").is_file():
        raise RuntimeError("19층 녹화 경로를 확인할 수 없습니다.")
    if action == "stop-bag":
        # Honor a deliberate stop even if the boot watcher later restarts.
        (RUNS / ".floor19-recording-stopped").touch()
        command("systemctl", "--user", "stop", bag["unit"], timeout=110)
        meta = path / "metadata.yaml"
        if not meta.is_file() or not meta.stat().st_size:
            raise RuntimeError(f"녹화 서비스 종료 후 metadata.yaml을 확인할 수 없습니다. {path}")
    else:
        mode = "paused" if action == "pause-bag" else "recording"
        if bag.get("mode") is None:
            raise RuntimeError("이 녹화는 일시중지를 지원하지 않습니다.")
        if bag["mode"] != mode:
            reply = command(str(run / "session"), "exec", "ros2", "service", "call",
                            f"/floor19_recorder_{expected}/set_paused", "std_srvs/srv/SetBool",
                            json.dumps({"data": mode == "paused"}), timeout=12)
            if "success=True" not in reply or recorder_mode(expected) != mode:
                raise RuntimeError("녹화 상태 변경을 확인할 수 없습니다.")
    return dict(path=str(path))


def reset_bag(run: Path, expected: str, expected_bag: str) -> dict:
    bag = confirmed_bag(expected_bag)
    path = Path(bag["path"])
    if path.parent != run or path.resolve() != path:
        raise RuntimeError("선택한 bag이 현재 매핑 실행 폴더에 없습니다.")
    bag_files(path, path.name)
    unit = bag["unit"]
    with pinned_unit(unit):
        restart_mapper(run, expected)
        confirmed_bag(expected_bag)
        command("systemctl", "--user", "stop", unit, timeout=100)
        # The original unit reopens the same output path with all its original options.
        previous = run / f"{path.name}-before-reset-{time.time_ns()}"
        path.rename(previous)
        try:
            command("systemctl", "--user", "start", unit)
            new_path = wait_for_new_bag(unit, expected_bag, run)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"새 녹화 확인 실패. 기존 bag 보관 위치: {previous}\n{exc}") from exc
        try:
            for file in bag_files(previous, path.name):
                file.unlink(missing_ok=True)
            previous.rmdir()
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(f"새 bag 녹화 중. 이전 bag 삭제 미완료: {previous}\n{exc}") from exc
    return dict(path=str(new_path))


def remote(action: str, expected: str, expected_bag: str = "") -> dict:
    state = unit_state()
    if action == "status":
        status = recording_status()
        bag = status["bag"]
        if bag and bag["path"] and bag.get("mode") != "paused":
            bag["missing"] = missing_sensors(Path(bag["path"]), time.time_ns())
        return dict(mapping=state["ActiveState"], token=state["InvocationID"], **status)
    if action == "start-bag":
        if state["ActiveState"] != "active" or state["InvocationID"] != expected:
            raise RuntimeError("매핑 세션이 바뀌었습니다.")
        if recording_status()["recorder_count"]:
            raise RuntimeError("이미 녹화 중입니다.")
        run = Path(f"/proc/{int(state['MainPID'])}/cwd").resolve(strict=True)
        # The capture tool takes the same lock and rechecks the mapper under it.
        return dict(path=command(CAPTURE, "start-bag", str(run), expected, timeout=145))
    if action not in ("save", "reset", "reset-bag", "pause-bag", "resume-bag", "stop-bag"):
        raise ValueError(action)
    lock = RUNS / ".floor19-panel.lock"
    with lock.open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if action in ("pause-bag", "resume-bag", "stop-bag"):
            return control_bag(action, expected_bag)
        state = unit_state()
        if state["ActiveState"] != "active" or state["InvocationID"] != expected:
            raise RuntimeError("매핑 세션이 바뀌었습니다. 상태를 새로 확인해 주세요.")
        run = Path(f"/proc/{int(state['MainPID'])}/cwd").resolve(strict=True)
        if run.parent != RUNS or not (run / "session").is_file():
            raise RuntimeError("19층 매핑 서비스의 실행 경로를 확인할 수 없습니다.")
        if action == "reset-bag":
            return reset_bag(run, expected, expected_bag)
        checkpoint = save_map(run)
        if action == "reset":
            try:
                restart_mapper(run, expected)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError(f"이전 지도 저장됨 {checkpoint}\n새 지도 확인 실패 {exc}") from exc
        return dict(path=str(checkpoint))


def jetson(action: str, token: str, bag_token: str = "") -> dict:
    import shlex

    args = ["python3", "-", "--remote", action, token, bag_token]
    if action in ("reset-bag", "start-bag", "stop-bag"):
        args = ["systemd-run", "--user", "--quiet", "--wait", "--collect", "--pipe", "--", *args]
    invocation = shlex.join(args)
    result = subprocess.run(
        ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
         "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
         os.environ.get("CLEANY_JETSON_SSH", "jetson-orin-via-n100"), invocation],
        input=Path(__file__).read_text(), capture_output=True, text=True,
        timeout=380 if action != "status" else 12)
    if result.returncode:
        raise RuntimeError(result.stderr.strip()[-620:] or "Jetson 연결을 확인해 주세요.")
    return json.loads(result.stdout)


def desktop() -> None:
    import gi
    gi.require_version("Gtk", "4.0")
    from gi.repository import GLib, Gtk

    app = Gtk.Application(application_id="dev.cleany.Floor19MapPanel")

    def build(application):
        if application.get_active_window():
            application.get_active_window().present()
            return
        window = Gtk.ApplicationWindow(application=application, title="19층 지도 관리")
        window.set_default_size(350, -1)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        for edge in ("top", "bottom", "start", "end"):
            getattr(box, f"set_margin_{edge}")(14)
        window.set_child(box)
        status = Gtk.Label(label="Jetson 연결 중…", xalign=0)
        box.append(status)
        bag_name = Gtk.Label(xalign=0, max_width_chars=32)
        bag_name.add_css_class("dim-label")
        box.append(bag_name)
        metrics = Gtk.Grid(column_spacing=16, row_spacing=6)
        box.append(metrics)
        values = []
        for index, title in enumerate(("녹화 세션 경과", "현재 bag 용량", "디스크 여유", "예상 녹화 가능 시간")):
            metrics.attach(Gtk.Label(label=title, xalign=0), 0, index, 1, 1)
            value = Gtk.Label(label="—", xalign=1, hexpand=True)
            metrics.attach(value, 1, index, 1, 1)
            values.append(value)
        elapsed, size, free, remaining = values
        elapsed.set_tooltip_text("일시중지를 포함한 세션 경과 시간")
        basis = Gtk.Label(label="현재 bag 기록 속도 · 10 GiB 여유 기준", xalign=0)
        basis.add_css_class("dim-label")
        box.append(basis)
        recording_row = Gtk.Box(spacing=8, homogeneous=True)
        box.append(recording_row)
        record = Gtk.Button(label="녹화 시작")
        stop = Gtk.Button(label="녹화 종료")
        stop.set_tooltip_text("현재 bag을 저장하고 녹화를 종료합니다.")
        recording_row.append(record)
        recording_row.append(stop)
        box.append(Gtk.Separator())
        row = Gtk.Box(spacing=8, homogeneous=True)
        box.append(row)
        save = Gtk.Button(label="지도 저장")
        save.add_css_class("suggested-action")
        reset = Gtk.Button(label="새 지도 시작…")
        for button in (save, reset):
            row.append(button)
        discard = Gtk.Button(label="현재 bag 삭제하고 새로 시작…")
        box.append(discard)
        view = Gtk.Button(label="RViz 시점 복원")
        box.append(view)
        note = Gtk.Label(xalign=0, wrap=True, max_width_chars=36, selectable=True)
        note.set_visible(False)
        box.append(note)
        token = ""
        current_bag = None
        recorder_count = None
        active = False
        busy = False
        checking = False
        generation = 0
        rate = RecordingRate()
        started = None

        def clock_tick():
            if started is not None:
                elapsed.set_label(duration_label(time.monotonic() - started))
            return True

        def show_metrics(report):
            nonlocal started
            bag = report.get("bag")
            paused = bag and bag.get("mode") == "paused"
            bps = rate.update(None if paused else bag, report.get("sample", 0))
            started = time.monotonic() - bag["seconds"] if bag else None
            elapsed.set_label("—")
            clock_tick()
            path = bag["path"] if bag else None
            bag_name.set_label(Path(path).name if path else "")
            bag_name.set_tooltip_text(path)
            bag_name.set_visible(bool(path))
            size.set_label(byte_label(bag["size"]) if bag and bag["size"] is not None else "—")
            free.set_label(byte_label(report["free"]) if "free" in report else "—")
            free.set_tooltip_text(report.get("disk"))
            can_estimate = path and not paused and report["recorder_count"] == 1
            remaining.set_label(estimate_label(report["free"], bps) if can_estimate else "—")
            basis.set_visible(bool(can_estimate))

        def feedback(message):
            note.set_label(message)
            note.set_tooltip_text(None)
            note.set_visible(bool(message))

        def controls():
            mode = current_bag.get("mode") if current_bag else None
            record.set_label(("녹화 재개" if mode == "paused" else "녹화 일시중지")
                             if current_bag else "녹화 시작")
            record.set_sensitive(not busy and (mode in ("paused", "recording")
                                               or (active and recorder_count == 0)))
            stop.set_sensitive(bool(current_bag and not busy))
            save.set_sensitive(active and not busy)
            reset.set_sensitive(active and not busy)
            discard.set_sensitive(bool(active and current_bag and not busy))
            view.set_sensitive(not busy)

        def perform(action, expected=None, bag_token=""):
            nonlocal busy, checking, generation
            if busy or (action == "status" and checking):
                return
            if action == "status":
                checking = True
            else:
                generation += 1
                busy = True
                controls()
                feedback({"save": "지도 저장 중…", "reset": "기존 지도 저장 후 새 지도 시작 중…",
                          "reset-bag": "새 지도와 녹화 시작 중…",
                          "start-bag": "새 bag 녹화 시작 중…", "stop-bag": "bag 저장 중…",
                          "pause-bag": "녹화 일시중지 중…", "resume-bag": "녹화 재개 중…",
                          "view": "RViz 시점 복원 중…"}[action])
            current = token if expected is None else expected
            request_generation = generation

            def done(result, error):
                nonlocal token, current_bag, recorder_count, active, busy, checking
                if action == "status":
                    checking = False
                    if request_generation != generation:
                        if not busy:
                            perform("status")
                        return False
                else:
                    busy = False
                    current_bag = recorder_count = None
                if action == "status":
                    active = not error and result["mapping"] == "active"
                    token = "" if error else result["token"]
                    recorder_count = result.get("recorder_count")
                    current_bag = result.get("bag") if recorder_count == 1 else None
                    if current_bag and not current_bag["path"]:
                        current_bag = None
                    paused = current_bag and current_bag.get("mode") == "paused"
                    recording_label = "녹화 일시중지" if paused else ("녹화 중" if result.get("bag") else "녹화 종료")
                    status.set_label("Jetson 연결 확인 필요" if error else
                                     f"매핑 {'실행 중' if active else '중지'}    ·    {recording_label}")
                    status.set_tooltip_text(error)
                    if current_bag and not paused:
                        missing = current_bag.get("missing")
                        if missing is None:
                            status.set_label(status.get_label() + "\n센서 기록 확인 중")
                        elif missing:
                            status.set_label(status.get_label() + "\n최근 8초간 기록 없음: " + ", ".join(missing))
                        else:
                            status.set_label(status.get_label() + "\nRGB·Depth·좌우 IR·LiDAR 기록 중")
                    show_metrics(result)
                elif error:
                    feedback(error)
                elif action == "view":
                    feedback("RViz 기본 시점을 열었습니다.")
                elif action in ("start-bag", "pause-bag", "resume-bag", "stop-bag"):
                    message = {"start-bag": "새 bag 녹화 시작", "pause-bag": "녹화 일시중지됨",
                               "resume-bag": "같은 bag에 녹화 재개", "stop-bag": "bag 저장 및 녹화 종료"}[action]
                    feedback(f"{message} · {Path(result['path']).name}")
                    note.set_tooltip_text(result["path"])
                elif action == "reset-bag":
                    feedback("이전 bag 삭제 완료 · 새 지도와 녹화 시작")
                    note.set_tooltip_text(result["path"])
                else:
                    label = "새 지도 시작됨" if action == "reset" else "지도 저장됨"
                    stamp = datetime.strptime(Path(result["path"]).name, "%Y-%m-%d_%H%M%S_%f")
                    feedback(f"{label}  ·  {stamp:%H:%M:%S}")
                    note.set_tooltip_text(result["path"])
                controls()
                if action != "status":
                    perform("status")
                return False

            def worker():
                try:
                    if action == "view":
                        command("systemctl", "--user", "restart", VIEW, OVERLAYS, timeout=90)
                        result = {}
                    else:
                        result = jetson(action, current, bag_token)
                        if action in ("reset", "reset-bag"):
                            try:
                                command("systemctl", "--user", "try-restart", OVERLAYS, VIEW, timeout=90)
                            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                                raise RuntimeError(f"Jetson 초기화 완료. RViz 화면 갱신 실패: {exc}") from exc
                    GLib.idle_add(done, result, None)
                except Exception as exc:
                    GLib.idle_add(done, {}, str(exc))

            threading.Thread(target=worker, daemon=True).start()

        def confirm(_button, action):
            confirmed_token = token
            bag_token = ""
            title = "현재 지도를 저장하고 처음부터 시작할까요?"
            detail = "현재 로봇 위치에서 새 지도를 그립니다. 주행 기록은 계속됩니다."
            accept = "저장 후 새로 시작"
            if action == "reset-bag":
                if not current_bag:
                    return
                bag_token = current_bag["id"]
                title = "현재 bag을 영구 삭제하고 새로 시작할까요?"
                amount = byte_label(current_bag["size"]) if current_bag["size"] is not None else "용량 확인 중"
                detail = (f"{current_bag['path']}\n{amount} · 모든 분할 파일\n\n"
                          "이번 bag의 카메라·LiDAR 등 센서 기록을 지우고 새 지도와 새 bag을 시작합니다. "
                          "따로 저장한 지도와 다른 bag은 보관합니다.")
                accept = "bag 삭제하고 새로 시작"
            dialog = Gtk.MessageDialog(transient_for=window, modal=True,
                                       text=title)
            dialog.set_property("secondary-text", detail)
            dialog.add_button("취소", Gtk.ResponseType.CANCEL)
            accepted = dialog.add_button(accept, Gtk.ResponseType.ACCEPT)
            if action == "reset-bag":
                accepted.add_css_class("destructive-action")
            dialog.set_default_response(Gtk.ResponseType.CANCEL)

            def respond(widget, response):
                widget.destroy()
                if response == Gtk.ResponseType.ACCEPT:
                    perform(action, confirmed_token, bag_token)

            dialog.connect("response", respond)
            dialog.present()

        def on_record(_button):
            if not current_bag:
                perform("start-bag")
            else:
                action = "resume-bag" if current_bag.get("mode") == "paused" else "pause-bag"
                perform(action, bag_token=current_bag["id"])

        record.connect("clicked", on_record)
        stop.connect("clicked", lambda _: perform("stop-bag", bag_token=current_bag["id"]))
        save.connect("clicked", lambda _: perform("save"))
        reset.connect("clicked", confirm, "reset")
        discard.connect("clicked", confirm, "reset-bag")
        view.connect("clicked", lambda _: perform("view"))
        controls()
        window.present()
        perform("status")

        def refresh():
            perform("status")
            return True

        GLib.timeout_add_seconds(5, refresh)
        GLib.timeout_add_seconds(1, clock_tick)

    app.connect("activate", build)
    app.run(None)


if __name__ == "__main__":
    if sys.argv[1:2] == ["--remote"]:
        try:
            print(json.dumps(remote(*sys.argv[2:])))
        except Exception as error:
            sys.exit(str(error))
    else:
        desktop()
