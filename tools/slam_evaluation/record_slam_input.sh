#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
result_root="$ros_workspace/slam_results"
gazebo_pid="" recorder_pid="" route_pid=""

usage() {
  echo "usage: $0 {16p5|26|45|70}" >&2
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

height=$1
case "$height" in
  16p5)
    display_height=16.5
    lidar_profile=floor_16p5cm
    domain_id=151
    ;;
  26)
    display_height=26
    lidar_profile=floor_26cm
    domain_id=152
    ;;
  45)
    display_height=45
    lidar_profile=floor_45cm
    domain_id=153
    ;;
  70)
    display_height=70
    lidar_profile=floor_70cm
    domain_id=154
    ;;
  *)
    echo "unsupported height: $height" >&2
    usage
    exit 2
    ;;
esac

input="$result_root/algorithm_compare_inputs/input_${height}cm_trial1"
environment="$result_root/algorithm_comparison/${height}cm_environment"

profile_shell=$(python3 "$workspace_root/tools/gazebo_profile.py" --shell)
eval "$profile_shell"
source "$CLEANY_ROS_SETUP"
source "$ros_workspace/$CLEANY_INSTALL_BASE/setup.bash"

study_cafe_launch=gazebo_study_cafe.launch.py
bridge_config="$ros_workspace/src/cleany_gazebo_sim/config/bridge/navigation_bridge.yaml"
export ROS_DOMAIN_ID=$domain_id

stop_group() {
  local pid=${1:-}
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return
  fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || true
}
trap 'stop_group "$route_pid"; stop_group "$recorder_pid"; stop_group "$gazebo_pid"' EXIT

if [[ -e "$input" || -e "$environment" ]]; then
  echo "refusing to overwrite existing ${display_height} cm input or environment" >&2
  exit 1
fi
mkdir -p "$environment"
setsid ros2 launch cleany_gazebo_sim "$study_cafe_launch" \
  headless:=true lidar_profile:="$lidar_profile" \
  physics_max_step_size:=0.004 physics_real_time_factor:=2.5 \
  bridge_config:="$bridge_config" \
  >"$environment/gazebo.log" 2>&1 &
gazebo_pid=$!

scan_sample=""
frame_id=""
for _ in {1..120}; do
  kill -0 "$gazebo_pid"
  scan_sample=$(timeout 2 ros2 topic echo --once /scan 2>/dev/null || true)
  frame_id=$(awk '/frame_id:/{print $2; exit}' <<<"$scan_sample")
  if [[ -n "$frame_id" ]]; then
    break
  fi
  sleep 0.5
done
kill -0 "$gazebo_pid"
if [[ "$frame_id" != "lidar_link" ]]; then
  echo "unexpected lower LiDAR frame: $frame_id" >&2
  exit 1
fi

setsid ros2 bag record -o "$input" --storage mcap --topics \
  /scan /imu/data /odom /ground_truth/odom /tf_static /clock \
  /cmd_vel /gazebo_cmd_vel >"$environment/recorder.log" 2>&1 &
recorder_pid=$!
sleep 2
setsid ros2 launch cleany_gazebo_sim evaluation_study_cafe_route.launch.py \
  >"$environment/route.log" 2>&1 &
route_pid=$!

completed=false
for _ in {1..900}; do
  kill -0 "$gazebo_pid"
  kill -0 "$route_pid"
  if grep -q 'evaluation route completed' "$environment/route.log"; then
    completed=true
    break
  fi
  sleep 1
done
if [[ "$completed" != true ]]; then
  echo "${display_height} cm route did not complete" >&2
  exit 1
fi
sleep 2
stop_group "$route_pid"; route_pid=""
stop_group "$recorder_pid"; recorder_pid=""
stop_group "$gazebo_pid"; gazebo_pid=""
echo "completed ${display_height} cm input bag"
