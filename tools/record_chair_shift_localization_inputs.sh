#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
result_root="$ros_workspace/slam_results/chair_shift_localization"
source /opt/ros/jazzy/setup.bash
source "$ros_workspace/install-harmonic/setup.bash"

gazebo_pid="" recorder_pid="" route_pid=""
stop_group() {
  local pid=${1:-}
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then return; fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || true
}
cleanup() {
  stop_group "$route_pid"
  stop_group "$recorder_pid"
  stop_group "$gazebo_pid"
}
trap cleanup EXIT

record_one() {
  local height=$1 domain=$2 expected_frame
  local environment="$result_root/environments/${height}cm_shifted"
  local input="$result_root/inputs/${height}cm_shifted"
  case "$height" in
    16p5)
      expected_frame=lidar_link
      ;;
    26)
      expected_frame=lidar_link
      ;;
    *) echo "unsupported height: $height" >&2; return 2 ;;
  esac
  if [[ -f "$input/metadata.yaml" ]]; then
    echo "skip completed shifted input ${height}cm"
    return
  fi
  if [[ -e "$input" || -e "$environment" ]]; then
    echo "refusing to overwrite incomplete ${height}cm artifacts" >&2
    return 1
  fi
  mkdir -p "$(dirname "$input")" "$(dirname "$environment")"
  export ROS_DOMAIN_ID=$domain
  # Gazebo Transport is independent of ROS_DOMAIN_ID. Partition it as well so
  # another Harmonic simulation cannot leak /clock, scan, or command topics.
  export GZ_PARTITION="cleany_chair_shift_${height}_${domain}"
  python3 "$workspace_root/tools/prepare_chair_shift_localization_world.py" \
    "$height" "$environment"

  setsid ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py \
    world:="$environment/world.sdf" headless:=true \
    sensor_config:="$environment/sensor_tf.yaml" \
    >"$environment/gazebo.log" 2>&1 &
  gazebo_pid=$!
  local scan_sample="" frame_id=""
  for _ in {1..120}; do
    kill -0 "$gazebo_pid"
    scan_sample=$(timeout 2 ros2 topic echo --once /scan 2>/dev/null || true)
    frame_id=$(awk '/frame_id:/{print $2; exit}' <<<"$scan_sample")
    [[ -n "$frame_id" ]] && break
    sleep 0.5
  done
  if [[ "$frame_id" != "$expected_frame" ]]; then
    echo "unexpected ${height}cm scan frame: $frame_id" >&2
    return 1
  fi

  setsid ros2 bag record -o "$input" --storage mcap --topics \
    /scan /imu/data /odom /ground_truth/odom /tf_static /clock \
    /cmd_vel /gazebo_cmd_vel >"$environment/recorder.log" 2>&1 &
  recorder_pid=$!
  sleep 2
  setsid ros2 launch cleany_gazebo_sim evaluation_study_cafe_route.launch.py \
    >"$environment/route.log" 2>&1 &
  route_pid=$!
  local completed=false
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
    echo "${height}cm shifted route did not complete" >&2
    return 1
  fi
  sleep 2
  stop_group "$route_pid"; route_pid=""
  stop_group "$recorder_pid"; recorder_pid=""
  stop_group "$gazebo_pid"; gazebo_pid=""
  echo "completed shifted input ${height}cm"
}

heights=(16p5 26)
if [[ $# -ge 1 ]]; then heights=("$1"); fi
for index in "${!heights[@]}"; do
  record_one "${heights[$index]}" $((171 + index))
done
