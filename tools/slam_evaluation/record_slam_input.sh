#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
result_root="$ros_workspace/slam_results"
gazebo_pid="" recorder_pid="" route_pid=""
server_render_engine=${GAZEBO_SERVER_RENDER_ENGINE:-ogre2}
sensor_render_engine=${GAZEBO_SENSOR_RENDER_ENGINE:-ogre2}

usage() {
  echo "usage: $0 {16p5|26|30|45|70} {measured|stress}" >&2
}

if [[ $# -ne 2 ]]; then
  usage
  exit 2
fi

height=$1
noise_profile=$2
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
  30)
    display_height=30
    lidar_profile=floor_30cm
    domain_id=155
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

case "$noise_profile" in
  measured) domain_id=$((domain_id + 10)) ;;
  stress) domain_id=$((domain_id + 20)) ;;
  *)
    echo "unsupported LiDAR noise profile: $noise_profile" >&2
    usage
    exit 2
    ;;
esac
input=${SLAM_INPUT_PATH:-$result_root/algorithm_compare_inputs/$noise_profile/input_${height}cm_trial1}
environment=${SLAM_ENVIRONMENT_PATH:-$result_root/algorithm_comparison/$noise_profile/${height}cm_environment}
robot_spawn_pose=${ROBOT_SPAWN_POSE:-}
route_config=${ROUTE_CONFIG:-}

requested_profile=${GAZEBO_PROFILE:-harmonic}
profile_shell=$(GAZEBO_PROFILE="$requested_profile" \
  python3 "$workspace_root/tools/gazebo_profile.py" --shell)
eval "$profile_shell"
source "$CLEANY_ROS_SETUP"
source "$ros_workspace/$CLEANY_INSTALL_BASE/setup.bash"

case "$CLEANY_GAZEBO_PROFILE" in
  fortress)
    study_cafe_launch=gazebo_study_cafe_fortress.launch.py
    bridge_config="$ros_workspace/src/cleany_gazebo_sim/config/bridge/navigation_bridge.yaml"
    ;;
  harmonic)
    study_cafe_launch=gazebo_study_cafe.launch.py
    bridge_config="$ros_workspace/src/cleany_gazebo_sim/config/bridge/navigation_bridge_harmonic.yaml"
    ;;
  *)
    echo "unsupported Gazebo profile: $CLEANY_GAZEBO_PROFILE" >&2
    exit 2
    ;;
esac
export ROS_DOMAIN_ID=$domain_id
transport_partition="cleany_slam_input_${domain_id}_$$"
export GZ_PARTITION=$transport_partition
export IGN_PARTITION=$transport_partition

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
mkdir -p "$(dirname "$input")" "$environment"
setsid ros2 launch cleany_gazebo_sim "$study_cafe_launch" \
  headless:=true lidar_profile:="$lidar_profile" \
  lidar_noise_profile:="$noise_profile" \
  server_render_engine:="$server_render_engine" \
  sensor_render_engine:="$sensor_render_engine" \
  robot_spawn_pose:="$robot_spawn_pose" \
  physics_max_step_size:=0.002 physics_real_time_factor:=2.0 \
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
clock_sample=$(timeout 5 ros2 topic echo --once /clock 2>/dev/null || true)
if [[ -z "$clock_sample" ]]; then
  echo "Gazebo clock bridge did not publish /clock" >&2
  exit 1
fi
scan_spread=$(python3 -c '
import sys, yaml
message = next(
    item for item in yaml.safe_load_all(sys.stdin.read())
    if isinstance(item, dict) and "ranges" in item
)
ranges = [
    float(value) for value in message["ranges"]
    if isinstance(value, (int, float))
]
print(max(ranges) - min(ranges))
' <<<"$scan_sample")
if ! python3 -c 'import sys; sys.exit(float(sys.argv[1]) < 0.01)' "$scan_spread"; then
  echo "invalid LiDAR scan: range spread is only ${scan_spread} m" >&2
  exit 1
fi

setsid ros2 bag record -o "$input" --storage sqlite3 \
  --topics \
  /scan /imu/data /odom /ground_truth/odom /tf_static /clock \
  /cmd_vel /gazebo_cmd_vel >"$environment/recorder.log" 2>&1 &
recorder_pid=$!
sleep 2
if ! kill -0 "$recorder_pid" 2>/dev/null; then
  echo "rosbag recorder failed to start; see $environment/recorder.log" >&2
  exit 1
fi
route_arguments=()
if [[ -n "$route_config" ]]; then
  route_arguments+=(route_config:="$route_config")
fi
setsid ros2 launch cleany_gazebo_sim evaluation_study_cafe_route.launch.py \
  "${route_arguments[@]}" \
  >"$environment/route.log" 2>&1 &
route_pid=$!

completed=false
for _ in {1..900}; do
  kill -0 "$gazebo_pid"
  if grep -q 'evaluation route completed' "$environment/route.log"; then
    completed=true
    break
  fi
  kill -0 "$route_pid"
  sleep 1
done
if [[ "$completed" != true ]]; then
  echo "${display_height} cm route did not complete" >&2
  exit 1
fi
sleep 2
stop_group "$route_pid"; route_pid=""
stop_group "$recorder_pid"; recorder_pid=""
python3 "$workspace_root/tools/slam_evaluation/prepare_humble_bag.py" \
  "$input" >>"$environment/recorder.log" 2>&1
stop_group "$gazebo_pid"; gazebo_pid=""
echo "completed $noise_profile ${display_height} cm input bag"
