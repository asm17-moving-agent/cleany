#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
result_root="$ros_workspace/slam_results"
study_root="$result_root/chair_shift_localization"
rate=${SLAM_REPLAY_RATE:-2.5}
source /opt/ros/jazzy/setup.bash
source "$ros_workspace/install-harmonic/setup.bash"

launch_pid="" recorder_pid=""
stop_group() {
  local pid=${1:-}
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then return; fi
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..40}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
}
cleanup() {
  stop_group "$recorder_pid"
  stop_group "$launch_pid"
}
trap cleanup EXIT

run_one() {
  local height=$1 condition=$2 domain=$3 input
  local posegraph="$result_root/algorithm_compare_runs/slam_toolbox/${height}cm/posegraph_final"
  local output="$study_root/runs/${height}cm/$condition"
  if [[ "$condition" == baseline ]]; then
    input="$result_root/algorithm_compare_inputs/input_${height}cm_trial1"
  else
    input="$study_root/inputs/${height}cm_shifted"
  fi
  if [[ -f "$output/run_complete" ]]; then
    echo "skip completed ${height}cm $condition"
    return
  fi
  if [[ ! -f "$input/metadata.yaml" || ! -f "$posegraph.posegraph" ]]; then
    echo "missing input or posegraph for ${height}cm $condition" >&2
    return 1
  fi
  if [[ -e "$output" ]]; then
    echo "refusing to overwrite incomplete output: $output" >&2
    return 1
  fi
  mkdir -p "$output"
  export ROS_DOMAIN_ID=$domain
  setsid ros2 launch cleany_gazebo_sim \
    slam_toolbox_localization_replay.launch.py posegraph:="$posegraph" \
    >"$output/processing.log" 2>&1 &
  launch_pid=$!
  sleep 5
  kill -0 "$launch_pid"
  if grep -Eq 'Failed to deserialize|Caught exception|process has died' \
    "$output/processing.log"; then
    echo "localization startup failed for ${height}cm $condition" >&2
    return 1
  fi
  setsid ros2 bag record -o "$output/result_bag" --storage mcap \
    --topics /map /map_metadata /tf \
    >"$output/recorder.log" 2>&1 &
  recorder_pid=$!
  sleep 1
  /usr/bin/time -v -o "$output/resource_usage.txt" \
    ros2 bag play "$input" --rate "$rate" \
    --topics /scan /odom /ground_truth/odom /tf_static /clock \
    >"$output/playback.log" 2>&1
  sleep 4
  stop_group "$recorder_pid"; recorder_pid=""
  stop_group "$launch_pid"; launch_pid=""
  date --iso-8601=seconds >"$output/run_complete"
  echo "completed localization ${height}cm $condition"
}

heights=(12 16p5 26)
conditions=(baseline shifted)
if [[ $# -ge 1 ]]; then heights=("$1"); fi
if [[ $# -ge 2 ]]; then conditions=("$2"); fi
domain=181
for height in "${heights[@]}"; do
  for condition in "${conditions[@]}"; do
    run_one "$height" "$condition" "$domain"
    domain=$((domain + 1))
  done
done
