#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
input_root="$ros_workspace/slam_results/algorithm_compare_inputs"
run_root="$ros_workspace/slam_results/algorithm_compare_runs"
rate=${SLAM_REPLAY_RATE:-2.5}

source /opt/ros/jazzy/setup.bash
source "$ros_workspace/install-harmonic/setup.bash"
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
  local height=$2
  local input="$input_root/input_${height}cm_trial1"
  local output="$run_root/$algorithm/${height}cm"
  local launch_pid="" recorder_pid=""
  local algorithm_domain height_domain

  case "$algorithm" in
    slam_toolbox) algorithm_domain=100 ;;
    cartographer) algorithm_domain=110 ;;
    cartographer_imu) algorithm_domain=120 ;;
    rtabmap) algorithm_domain=130 ;;
    *) echo "unknown algorithm: $algorithm" >&2; return 2 ;;
  esac
  case "$height" in
    16p5) height_domain=1 ;;
    26) height_domain=2 ;;
    45) height_domain=3 ;;
    70) height_domain=4 ;;
    *) echo "unknown height: $height" >&2; return 2 ;;
  esac
  export ROS_DOMAIN_ID=$((algorithm_domain + height_domain))

  if [[ -f "$output/run_complete" ]]; then
    echo "skip completed $algorithm ${height}cm"
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
      setsid ros2 launch cleany_gazebo_sim evaluation_slam_toolbox_replay.launch.py \
        >"$output/processing.log" 2>&1 &
      ;;
    cartographer)
      setsid ros2 launch cleany_gazebo_sim evaluation_cartographer_replay.launch.py \
        configuration_basename:=cartographer_2d.lua \
        >"$output/processing.log" 2>&1 &
      ;;
    cartographer_imu)
      setsid ros2 launch cleany_gazebo_sim evaluation_cartographer_replay.launch.py \
        configuration_basename:=cartographer_2d_imu.lua \
        >"$output/processing.log" 2>&1 &
      ;;
    rtabmap)
      setsid ros2 launch cleany_gazebo_sim evaluation_rtabmap_replay.launch.py \
        database_path:="$output/rtabmap.db" \
        >"$output/processing.log" 2>&1 &
      ;;
    *) echo "unknown algorithm: $algorithm" >&2; return 2 ;;
  esac
  launch_pid=$!
  sleep 4
  kill -0 "$launch_pid"

  setsid ros2 bag record -o "$output/result_bag" \
    --topics /map /map_metadata /tf /tracked_pose /submap_list \
    >"$output/recorder.log" 2>&1 &
  recorder_pid=$!
  sleep 1

  local topics=(/scan /odom /ground_truth/odom /tf_static /clock)
  if [[ "$algorithm" == cartographer_imu ]]; then
    topics+=(/imu/data)
  fi
  /usr/bin/time -v -o "$output/resource_usage.txt" \
    ros2 bag play "$input" --rate "$rate" --topics "${topics[@]}" \
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
    cartographer|cartographer_imu)
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
    rtabmap)
      : >"$output/save.log"
      ;;
  esac

  stop_process "$recorder_pid"
  recorder_pid=""
  stop_process "$launch_pid"
  launch_pid=""
  if [[ "$algorithm" == rtabmap ]]; then
    python3 "$workspace_root/tools/analyze_slam_algorithm_comparison.py" \
      --extract-rtabmap-run "$output" --input-bag "$input" \
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
  echo "completed $algorithm ${height}cm"
}

algorithms=(slam_toolbox cartographer cartographer_imu rtabmap)
heights=(16p5 26 45 70)
if [[ $# -ge 1 ]]; then
  algorithms=($1)
fi
if [[ $# -ge 2 ]]; then
  heights=($2)
fi
for algorithm in "${algorithms[@]}"; do
  for height in "${heights[@]}"; do
    run_one "$algorithm" "$height"
  done
done
