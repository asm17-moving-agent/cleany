#!/usr/bin/env bash
# ROS 2 base showcase: Kitty tabs + the saved rqt dashboard in one Sway workspace.
# shellcheck disable=SC1091
set -eo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
script="$root/tools/base_showcase.sh"
mode=hardware
port=${CLEANY_ESP_PORT:-}
port_explicit=false
workspace='Cleany sweep'
container=${ROS2_CONTAINER:-ros2-humble}
dry_run=false
role=
session_dir=

usage() {
    cat <<'EOF'
Usage: bash tools/base_showcase.sh --port /dev/serial/by-id/DEVICE [options]
       bash tools/base_showcase.sh --mock [options]

  --port PATH        ESP32-S3 native USB device (or set CLEANY_ESP_PORT)
  --mock             Device-free demo in ROS_DOMAIN_ID=173 (hardware uses 0)
  --workspace NAME   Sway workspace (default: Cleany sweep)
  --container NAME   Distrobox used when native Humble is absent (ros2-humble)
  --dry-run          Print the launch plan without opening windows or ROS
  -h, --help         Show help

Starts disabled. In the Commands tab: forward, backward, left, right,
yaw_left, yaw_right, demo, stop, status, quit. Motion lasts 3 seconds per step.
Hardware: lift all wheels, secure the robot, and prepare the physical E-stop.
Close Kitty or the dashboard, or press Ctrl+C here, to stop and shut down.
EOF
}

fail() { printf 'base_showcase: %s\n' "$*" >&2; exit 1; }

parse_args() {
    while (($#)); do
        case "$1" in
            --mock) mode=mock; shift ;;
            --port|--workspace|--container|--role|--session-dir)
                (($# >= 2)) && [[ -n $2 && $2 != --* ]] || fail "$1 requires a value"
                case "$1" in
                    --port) port=$2; port_explicit=true ;;
                    --workspace) workspace=$2 ;;
                    --container) container=$2 ;;
                    --role) role=$2 ;;
                    --session-dir) session_dir=$2 ;;
                esac
                shift 2 ;;
            --dry-run) dry_run=true; shift ;;
            -h|--help) usage; exit 0 ;;
            *) fail "unknown argument: $1" ;;
        esac
    done
}

stop_group() {
    local pid=$1
    kill -0 "$pid" 2>/dev/null || return 0
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..30}; do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.1
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
    sleep 0.5
    kill -KILL -- "-$pid" 2>/dev/null || true
}

setup_ros() {
    # ROS setup scripts reference unset variables; intentionally do not use set -u.
    source /opt/ros/humble/setup.bash
    if [[ $mode == hardware ]]; then
        source "$root/esp32/micro_ros/agent/install/local_setup.bash"
    fi
    source "$root/ros2_ws/install/local_setup.bash"
    export ROS_DOMAIN_ID=$domain
    cd "$root"
}

stop_base() {
    timeout 5s ros2 service call /base/enable std_srvs/srv/SetBool '{data: false}'
}

run_process() {
    setsid "$@" &
    child=$!
    printf '%s\n' "$child" > "$session_dir/$role.pid"
    trap 'stop_group "$child"; rm -f "$session_dir/$role.pid"' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM HUP
    wait "$child"
}

drive() {
    local response result=0
    [[ -z $session_dir || ! -e $session_dir/stopping ]] || return 1
    response=$(timeout 5s ros2 service call /base/enable std_srvs/srv/SetBool \
        '{data: true}') || { printf '%s\n' "$response"; return 1; }
    printf '%s\n' "$response"
    [[ $response == *'success=True'* || $response == *'success: true'* ]] || return 1
    timeout --signal=INT --kill-after=2s 3s ros2 topic pub -r 50 \
        /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $1, y: $2}, angular: {z: $3}}" || result=$?
    stop_base || return 1
    # GNU timeout returns 124 for the expected end of the finite motion step.
    [[ $result == 0 || $result == 124 ]]
}

console() {
    local command
    trap 'stop_base >/dev/null 2>&1 || true' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM HUP
    printf 'Waiting for wheel feedback...\n'
    timeout 30s ros2 topic echo /base/wheel_state cleany_base_interfaces/msg/WheelState --once \
        --qos-reliability best_effort || return 1
    printf '\nCommands: forward backward left right yaw_left yaw_right demo stop status quit\n'
    printf 'Each motion step lasts 3 seconds. Hardware wheels must be lifted.\n'
    while read -er -p 'cleany> ' command; do
        case "$command" in
            forward) drive 0.05 0 0 || printf 'Motion aborted.\n' ;;
            backward) drive -0.05 0 0 || printf 'Motion aborted.\n' ;;
            left) drive 0 0.05 0 || printf 'Motion aborted.\n' ;;
            right) drive 0 -0.05 0 || printf 'Motion aborted.\n' ;;
            yaw_left) drive 0 0 0.1 || printf 'Motion aborted.\n' ;;
            yaw_right) drive 0 0 -0.1 || printf 'Motion aborted.\n' ;;
            demo)
                drive 0.05 0 0 && sleep 1 &&
                    drive -0.05 0 0 && sleep 1 &&
                    drive 0 0.05 0 && sleep 1 &&
                    drive 0 0 0.1 || printf 'Demo aborted.\n' ;;
            stop) stop_base || true ;;
            status)
                timeout 5s ros2 topic echo /base/wheel_state cleany_base_interfaces/msg/WheelState --once \
                    --qos-reliability best_effort || true ;;
            quit) return 0 ;;
            '') ;;
            *) printf 'Unknown command. Use demo, stop, status, or a motion name.\n' ;;
        esac
    done
}

main() {
parse_args "$@"
if [[ -n $role ]]; then
    [[ -f $session_dir/environment ]] || fail "missing session environment"
    # This private file is generated by this launcher, not user shell configuration.
    source "$session_dir/environment"
    setup_ros
    case "$role" in
        check)
            for package in cleany_base_driver cleany_base_odometry rqt_gui \
                rqt_plot rqt_robot_monitor; do
                ros2 pkg prefix "$package" >/dev/null
            done
            if [[ $mode == hardware ]]; then
                ros2 pkg prefix micro_ros_agent >/dev/null
            fi
            services=$(ros2 service list)
            [[ $services != *'/base/enable'* ]] || fail "a base driver is already running in domain $domain"
            for topic in /base/wheel_command /cmd_vel; do
                info=$(ros2 topic info "$topic" 2>/dev/null || true)
                if grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$info"; then
                    fail "$topic already has a publisher in domain $domain"
                fi
            done ;;
        agent) run_process ros2 run micro_ros_agent micro_ros_agent serial --dev "$port" -b 115200 ;;
        driver)
            if [[ $mode == mock ]]; then
                run_process ros2 launch cleany_base_driver base_driver.launch.py mock:=true
            else
                run_process ros2 launch cleany_base_driver base_driver.launch.py \
                    mock:=false config:="$root/configs/robot/base_hardware.yaml"
            fi ;;
        monitor)
            timeout 30s ros2 topic echo /base/wheel_state cleany_base_interfaces/msg/WheelState --once \
                --qos-reliability best_effort >/dev/null
            share=$(ros2 pkg prefix cleany_base_driver)/share/cleany_base_driver
            # Launch rqt directly so its process ID can place only this new window.
            run_process "$(ros2 pkg prefix rqt_gui)/lib/rqt_gui/rqt_gui" \
                --force-discover --perspective-file "$share/config/base_monitor.perspective" \
                --ros-args -r /diagnostics_agg:=/diagnostics ;;
        control) console ;;
        odom) run_process ros2 topic echo /odom nav_msgs/msg/Odometry ;;
        stop) stop_base ;;
        *) fail "unknown role: $role" ;;
    esac
    exit
fi

[[ -n $workspace && $workspace != *$'\n'* && $workspace != *$'\r'* ]] || fail "invalid workspace name"
domain=0
[[ $mode == mock ]] && domain=173
[[ $mode == mock || -n $port ]] || fail "--port or CLEANY_ESP_PORT is required for hardware"
[[ $mode != mock || $port_explicit == false ]] || fail "--mock cannot be combined with a hardware port"
[[ $mode != mock ]] || port=
runner=()
tab_runner=()
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    runner=(distrobox enter --name "$container" --no-tty --clean-path --)
    tab_runner=(distrobox enter --name "$container" --clean-path --)
fi

if $dry_run; then
    printf 'Workspace: %s\nMode: %s\nROS_DOMAIN_ID: %s\n' "$workspace" "$mode" "$domain"
    printf 'ROS runner: '; printf '%q ' "${runner[@]}" bash "$script"; printf '\n'
    [[ $mode == mock ]] || printf 'Agent: native USB %s, baud 115200\n' "$port"
    printf 'Kitty tabs: %sDriver, Commands, Odom\n' "$([[ $mode == hardware ]] && printf 'Agent, ')"
    printf 'Dashboard: base_monitor.perspective (speed / PWM / diagnostics)\n'
    printf 'Startup: disabled; enter demo in Commands to request motion\n'
    exit 0
fi

for tool in swaymsg kitty python3 setsid flock timeout; do
    command -v "$tool" >/dev/null || fail "$tool is required"
done
if ((${#runner[@]})); then
    command -v distrobox >/dev/null || fail "native Humble or Distrobox is required"
fi
[[ -n ${SWAYSOCK:-} ]] || fail "run this launcher inside Sway"
[[ $mode == mock || -c $port ]] || fail "not a serial device: $port"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/cleany-base-showcase-$UID-$domain.lock"
flock -n 9 || fail "a showcase already owns domain $domain"

mkdir -p "${XDG_CACHE_HOME:-$HOME/.cache}"
session_dir=$(mktemp -d "${XDG_CACHE_HOME:-$HOME/.cache}/cleany-base-showcase.XXXXXX")
printf 'root=%q\nmode=%q\nport=%q\ndomain=%q\n' "$root" "$mode" "$port" "$domain" \
    > "$session_dir/environment"
terminal_pid=
monitor_launcher=

cleanup() {
    trap - EXIT INT TERM HUP
    touch "$session_dir/stopping"
    # Stop while our driver and Agent still run, before terminating owned groups.
    if [[ -f $session_dir/driver.pid ]]; then
        "${runner[@]}" bash "$script" --role stop --session-dir "$session_dir" \
            >"$session_dir/stop.log" 2>&1 || true
    fi
    for role in monitor odom driver agent; do
        if [[ -f $session_dir/$role.pid ]]; then
            stop_group "$(<"$session_dir/$role.pid")"
        fi
    done
    [[ -z $terminal_pid ]] || kill -TERM "$terminal_pid" 2>/dev/null || true
    [[ -z $monitor_launcher ]] || kill -TERM "$monitor_launcher" 2>/dev/null || true
    printf '\nShowcase stopped. Session logs: %s\n' "$session_dir"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
"${runner[@]}" bash "$script" --role check --session-dir "$session_dir"

# Do not retile a workspace that already contains unrelated windows.
occupied=$(swaymsg -t get_tree | python3 -c '
import json, sys
name = sys.argv[1]
def occupied(node):
    if node.get("type") == "workspace" and node.get("name") == name:
        return bool(node.get("nodes") or node.get("floating_nodes"))
    return any(occupied(child) for child in node.get("nodes", []) + node.get("floating_nodes", []))
print(int(occupied(json.load(sys.stdin))))
' "$workspace")
[[ $occupied == 0 ]] || fail "workspace $workspace is occupied; choose another --workspace"

# Use one Kitty window with tabs, leaving most of the workspace for the dashboard.
for role in agent driver odom control; do
    [[ $mode != mock || $role != agent ]] || continue
    case "$role" in
        agent) tab=Agent ;;
        driver) tab=Driver ;;
        odom) tab=Odom ;;
        control) tab=Commands ;;
    esac
    {
        printf 'new_tab %s\n' "$tab"
        printf 'launch '
        printf '%q ' "${tab_runner[@]}" bash "$script" --role "$role" --session-dir "$session_dir"
        printf '\n'
    } >> "$session_dir/kitty.session"
done
printf 'focus\n' >> "$session_dir/kitty.session"

# JSON string quoting also handles spaces/quotes in the workspace name for Sway.
quoted_workspace=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1]))' "$workspace")
swaymsg "workspace $quoted_workspace; layout splith" >/dev/null
app_id="cleany-base-showcase-$$"
kitty --class "$app_id" --title 'Cleany base commands / logs' \
    --session "$session_dir/kitty.session" >"$session_dir/kitty.log" 2>&1 &
terminal_pid=$!
"${runner[@]}" bash "$script" --role monitor --session-dir "$session_dir" \
    >"$session_dir/monitor.log" 2>&1 &
monitor_launcher=$!

window_id() {
    swaymsg -t get_tree | python3 -c '
import json, sys
kind, value = sys.argv[1:]
def visit(node):
    props = node.get("window_properties") or {}
    matches = (node.get("app_id") == value or props.get("class") == value) if kind == "app" else str(node.get("pid")) == value
    if matches:
        print(node["id"])
    for child in node.get("nodes", []) + node.get("floating_nodes", []):
        visit(child)
visit(json.load(sys.stdin))
' "$1" "$2"
}

terminal_id=
dashboard_id=
for _ in {1..90}; do
    kill -0 "$terminal_pid" 2>/dev/null || fail "Kitty exited; see $session_dir/kitty.log"
    kill -0 "$monitor_launcher" 2>/dev/null || fail "monitor exited; see $session_dir/monitor.log"
    [[ -n $terminal_id ]] || terminal_id=$(window_id app "$app_id")
    if [[ -f $session_dir/monitor.pid && -z $dashboard_id ]]; then
        dashboard_id=$(window_id pid "$(<"$session_dir/monitor.pid")")
    fi
    if [[ -n $terminal_id && -n $dashboard_id ]]; then
        break
    fi
    sleep 0.5
done
[[ -n $terminal_id && -n $dashboard_id ]] || fail "windows did not become ready; see $session_dir"
swaymsg "[con_id=$terminal_id] move container to workspace $quoted_workspace" >/dev/null
swaymsg "[con_id=$dashboard_id] move container to workspace $quoted_workspace" >/dev/null
swaymsg "[con_id=$dashboard_id] focus; move left; resize set width 65 ppt" >/dev/null
swaymsg "[con_id=$terminal_id] focus" >/dev/null
printf 'Ready in %s. Enter demo in the Commands tab. Ctrl+C here shuts down.\n' "$workspace"
printf 'Session logs: %s\n' "$session_dir"
while kill -0 "$terminal_pid" 2>/dev/null && kill -0 "$monitor_launcher" 2>/dev/null; do
    sleep 1
done
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
    main "$@"
fi
