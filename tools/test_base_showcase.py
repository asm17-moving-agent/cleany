"""Device-free CLI and console safety tests for base_showcase.sh."""

from pathlib import Path
import os
import subprocess

import pytest


SCRIPT = Path(__file__).with_name("base_showcase.sh")


def _cli(*args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment.pop("CLEANY_ESP_PORT", None)
    environment.update(env or {})
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        text=True,
        capture_output=True,
        check=False,
        env=environment,
    )


def test_help() -> None:
    result = _cli("--help")
    assert result.returncode == 0
    assert "Usage:" in result.stdout


@pytest.mark.parametrize(
    ("args", "mode", "domain"),
    [
        (("--dry-run", "--port", "/dev/example"), "hardware", "0"),
        (("--dry-run", "--mock"), "mock", "173"),
    ],
)
def test_dry_run_has_disabled_startup_and_distinct_domains(
    args: tuple[str, ...], mode: str, domain: str
) -> None:
    result = _cli(*args)
    assert result.returncode == 0, result.stderr
    assert f"Mode: {mode}" in result.stdout
    assert f"ROS_DOMAIN_ID: {domain}" in result.stdout
    assert "Startup: disabled;" in result.stdout
    assert "enter demo" in result.stdout


@pytest.mark.parametrize("option", ["--port", "--workspace", "--container", "--role", "--session-dir"])
def test_value_options_require_values(option: str) -> None:
    result = _cli(option)
    assert result.returncode != 0
    assert f"{option} requires a value" in result.stderr


def test_hardware_requires_port() -> None:
    result = _cli("--dry-run")
    assert result.returncode != 0
    assert "--port or CLEANY_ESP_PORT is required" in result.stderr


def test_mock_conflicts_with_port() -> None:
    result = _cli("--dry-run", "--mock", "--port", "/dev/example")
    assert result.returncode != 0
    assert "--mock cannot be combined" in result.stderr


def test_dry_run_quotes_option_values() -> None:
    result = _cli("--dry-run", "--mock", "--workspace", "Kitchen 'test'", "--container", "box name")
    assert result.returncode == 0, result.stderr
    assert "Workspace: Kitchen 'test'" in result.stdout
    if not Path("/opt/ros/humble/setup.bash").is_file():
        assert "box\\ name" in result.stdout


def test_mock_ignores_inherited_hardware_port() -> None:
    result = _cli("--dry-run", "--mock", env={"CLEANY_ESP_PORT": "/dev/example"})
    assert result.returncode == 0, result.stderr
    assert "Agent:" not in result.stdout


def test_hardware_can_use_port_environment() -> None:
    result = _cli("--dry-run", env={"CLEANY_ESP_PORT": "/dev/example"})
    assert result.returncode == 0, result.stderr
    assert "Agent: native USB /dev/example" in result.stdout


@pytest.mark.parametrize("value", ["", "--mock"])
def test_missing_option_value_is_not_consumed(value: str) -> None:
    result = _cli("--port", value)
    assert result.returncode != 0
    assert "--port requires a value" in result.stderr


def test_negative_enable_response_prevents_publication(tmp_path: Path) -> None:
    log = tmp_path / "calls"
    result = subprocess.run(
        [
            "bash", "-c",
            'source "$1"; timeout() { echo "$*" >>"$LOG"; echo "success=False"; }; '
            'drive 0.05 0 0',
            "_", str(SCRIPT),
        ],
        text=True, capture_output=True, check=False,
        env={"PATH": "/usr/bin:/bin", "LOG": str(log)},
    )
    assert result.returncode != 0
    assert "/cmd_vel" not in log.read_text()


def test_enable_failure_prevents_velocity_publication(tmp_path: Path) -> None:
    log = tmp_path / "calls"
    result = subprocess.run(
        [
            "bash", "-c",
            'source "$1"; timeout() { echo "$*" >>"$LOG"; return 1; }; ros2() { echo "ros2 $*" >>"$LOG"; }; drive 0.05 0 0',
            "_", str(SCRIPT),
        ],
        text=True, capture_output=True, check=False, env={"LOG": str(log)},
    )
    assert result.returncode != 0
    calls = log.read_text().splitlines()
    assert len(calls) == 1
    assert "/base/enable" in calls[0]
    assert "/cmd_vel" not in calls[0]


def test_successful_finite_drive_enables_publishes_then_stops(tmp_path: Path) -> None:
    log = tmp_path / "calls"
    result = subprocess.run(
        [
            "bash", "-c",
            'source "$1"; timeout() { echo "$*" >>"$LOG"; if [[ $* == *"/base/enable"* ]]; then echo "success=True"; return 0; fi; return 124; }; ros2() { :; }; drive 0.05 0 0',
            "_", str(SCRIPT),
        ],
        text=True, capture_output=True, check=False,
        env={"PATH": "/usr/bin:/bin", "LOG": str(log)},
    )
    assert result.returncode == 0, result.stderr
    calls = log.read_text().splitlines()
    assert len(calls) == 3
    assert "/base/enable" in calls[0] and "data: true" in calls[0]
    assert "/cmd_vel" in calls[1]
    assert "/base/enable" in calls[2] and "data: false" in calls[2]


def test_publish_failure_stops_and_returns_failure(tmp_path: Path) -> None:
    log = tmp_path / "calls"
    result = subprocess.run(
        [
            "bash", "-c",
            'source "$1"; timeout() { echo "$*" >>"$LOG"; if [[ $* == *"/base/enable"* && $* == *"data: true"* ]]; then echo "success=True"; return 0; fi; if [[ $* == *"/cmd_vel"* ]]; then return 2; fi; return 0; }; ros2() { :; }; drive 0.05 0 0',
            "_", str(SCRIPT),
        ],
        text=True, capture_output=True, check=False,
        env={"PATH": "/usr/bin:/bin", "LOG": str(log)},
    )
    assert result.returncode != 0
    calls = log.read_text().splitlines()
    assert len(calls) == 3
    assert "data: true" in calls[0]
    assert "/cmd_vel" in calls[1]
    assert "data: false" in calls[2]


def test_mock_launcher_lifecycle_places_workspace_and_stops_before_cleanup(
    tmp_path: Path,
) -> None:
    log = tmp_path / "lifecycle.log"
    result = subprocess.run(
        [
            "bash",
            "-c",
            r'''
source "$1"
log=$2
export XDG_RUNTIME_DIR=$3 XDG_CACHE_HOME=$3 SWAYSOCK=fake
record() { printf '%s\n' "$*" >>"$log"; }
swaymsg() {
    record "sway $*"
    if [[ $* == "-t get_tree" ]]; then
        if [[ -f $log.app ]]; then
            printf '{"nodes":[{"id":101,"app_id":"%s"},{"id":102,"pid":99999998}]}\n' "$(<"$log.app")"
        else
            printf '{"nodes":[],"floating_nodes":[]}\n'
        fi
    fi
}
kitty() {
    record "kitty $*"
    local session=
    while (($#)); do
        case "$1" in
            --session) session=$2; shift 2 ;;
            --class) printf '%s\n' "$2" > "$log.app"; shift 2 ;;
            *) shift ;;
        esac
    done
    cp "$session" "$log.kitty"
    printf '99999999\n' > "$(dirname "$session")/driver.pid"
    sleep 2
}
bash() {
    record "runner $*"
    local args="$*"
    if [[ $args == *"--role monitor"* ]]; then
        local session=${args##*--session-dir }
        printf '99999998\n' > "$session/monitor.pid"
        sleep 2
    elif [[ $args == *"--role stop"* ]]; then
        record STOP
    fi
}
distrobox() {
    while (($#)) && [[ $1 != -- ]]; do shift; done
    shift
    "$@"
}
stop_group() { record "owned-process-cleanup $1"; }
main --mock --workspace "Test Sweep"
''',
            "_",
            str(SCRIPT),
            str(log),
            str(tmp_path),
        ],
        text=True,
        capture_output=True,
        check=False,
        env={**os.environ, "XDG_RUNTIME_DIR": str(tmp_path), "XDG_CACHE_HOME": str(tmp_path)},
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
    events = log.read_text().splitlines()
    assert 'sway workspace "Test Sweep"; layout splith' in events
    assert 'sway [con_id=102] focus; move left; resize set width 65 ppt' in events
    kitty_script = Path(f"{log}.kitty").read_text()
    assert "new_tab Driver" in kitty_script
    assert "new_tab Odom" in kitty_script
    assert "new_tab Commands" in kitty_script
    assert "new_tab Agent" not in kitty_script
    assert events.index("STOP") < next(
        i for i, event in enumerate(events) if event.startswith("owned-process-cleanup ")
    )
