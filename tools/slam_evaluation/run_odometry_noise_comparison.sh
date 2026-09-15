#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
experiment_name=${ODOMETRY_EXPERIMENT_NAME:-odometry_noise}
input_root="$ros_workspace/slam_results/$experiment_name/inputs"
run_root="$ros_workspace/slam_results/$experiment_name/runs"
rate=${SLAM_REPLAY_RATE:-2.5}

requested_profile=${GAZEBO_PROFILE:-harmonic}
profile_shell=$(GAZEBO_PROFILE="$requested_profile" \
  python3 "$workspace_root/tools/gazebo_profile.py" --shell)
eval "$profile_shell"
source "$CLEANY_ROS_SETUP"
source "$ros_workspace/$CLEANY_INSTALL_BASE/setup.bash"
set -u

stop_process() {
  local pid=${1:-}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.1
    done
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
}

run_one() {
  local algorithm=$1
  local level=$2
  local input="$input_root/$level/input_30cm_trial1"
  local output="$run_root/$algorithm/$level"
  local launch_pid="" recorder_pid=""
  local algorithm_domain level_number

  case "$algorithm" in
    slam_toolbox) algorithm_domain=180 ;;
    cartographer) algorithm_domain=190 ;;
    *) echo "unknown algorithm: $algorithm" >&2; return 2 ;;
  esac
  case "$level" in
    level0) level_number=0 ;;
    level1) level_number=1 ;;
    level2) level_number=2 ;;
    level3) level_number=3 ;;
    *) echo "unknown odometry level: $level" >&2; return 2 ;;
  esac
  export ROS_DOMAIN_ID=$((algorithm_domain + level_number))

  if [[ ! -f "$input/metadata.yaml" ]]; then
    echo "missing input bag: $input" >&2
    return 1
  fi
  if [[ -f "$output/run_complete" ]]; then
    echo "skip completed $algorithm $level"
    return
  fi
  if [[ -e "$output" ]]; then
    echo "refusing to overwrite incomplete output: $output" >&2
    return 1
  fi
  mkdir -p "$output"
  trap 'stop_process "$recorder_pid"; stop_process "$launch_pid"' RETURN

  case "$algorithm" in
    slam_toolbox)
      setsid ros2 launch cleany_gazebo_sim \
        evaluation_slam_toolbox_live_replay.launch.py \
        >"$output/processing.log" 2>&1 &
      ;;
    cartographer)
      setsid ros2 launch cleany_gazebo_sim \
        evaluation_cartographer_replay.launch.py \
        configuration_basename:=cartographer_2d.lua \
        >"$output/processing.log" 2>&1 &
      ;;
  esac
  launch_pid=$!
  sleep 4
  kill -0 "$launch_pid"

  setsid ros2 bag record -o "$output/result_bag" --storage sqlite3 \
    /map /map_metadata /tf /tracked_pose /submap_list \
    >"$output/recorder.log" 2>&1 &
  recorder_pid=$!
  sleep 2
  if ! kill -0 "$recorder_pid" 2>/dev/null; then
    echo "rosbag recorder failed; see $output/recorder.log" >&2
    return 1
  fi

  /usr/bin/time -v -o "$output/resource_usage.txt" \
    ros2 bag play "$input" --rate "$rate" --topics \
      /scan /odom /ground_truth/odom /tf_static /clock \
      >"$output/playback.log" 2>&1
  sleep 5

  case "$algorithm" in
    slam_toolbox)
      ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: '$output/map_final'}}" >"$output/save.log"
      ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '$output/posegraph_final'}" >>"$output/save.log"
      ;;
    cartographer)
      ros2 service call /finish_trajectory \
        cartographer_ros_msgs/srv/FinishTrajectory \
        "{trajectory_id: 0}" >"$output/save.log"
      ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
        "{filename: '$output/map_final.pbstream', include_unfinished_submaps: true}" \
        >>"$output/save.log"
      ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
        -pbstream_filename="$output/map_final.pbstream" \
        -map_filestem="$output/map_final" -resolution=0.05 \
        >>"$output/save.log" 2>&1
      sed -i 's|^image: .*|image: map_final.pgm|' "$output/map_final.yaml"
      ;;
  esac

  stop_process "$recorder_pid"
  recorder_pid=""
  stop_process "$launch_pid"
  launch_pid=""
  if [[ ! -f "$output/map_final.pgm" ]]; then
    python3 "$workspace_root/tools/slam_evaluation/extract_latest_occupancy_map.py" \
      "$output/result_bag" "$output/map_final" \
      >>"$output/save.log" 2>&1
  fi
  python3 - "$output/map_final.pgm" <<'PY'
from pathlib import Path
import sys
from PIL import Image

path = Path(sys.argv[1])
Image.open(path).save(path.with_suffix('.png'))
PY
  date --iso-8601=seconds >"$output/run_complete"
  echo "completed $algorithm $level"
}

algorithms=(slam_toolbox cartographer)
levels=(level0 level1 level2 level3)
if [[ $# -ge 1 ]]; then
  algorithms=("$1")
fi
if [[ $# -ge 2 ]]; then
  levels=("$2")
fi
for algorithm in "${algorithms[@]}"; do
  for level in "${levels[@]}"; do
    run_one "$algorithm" "$level"
  done
done
