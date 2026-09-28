#!/usr/bin/env bash
set -eo pipefail

workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$workspace_root/ros2_ws"
result_root="$ros_workspace/slam_results"

if [[ $# -ne 2 ]]; then
  echo "usage: $0 {16p5|26|45|70} {measured|stress}" >&2
  exit 2
fi

height=$1
noise_profile=$2
final_input="$result_root/algorithm_compare_inputs/$noise_profile/input_${height}cm_trial1"
final_environment="$result_root/algorithm_comparison/$noise_profile/${height}cm_environment"
work_root="$result_root/segmented_recording/$noise_profile/${height}cm"
segment_inputs="$work_root/inputs"
segment_environments="$work_root/environments"
route_segments="$work_root/routes"
route_config="$ros_workspace/src/cleany_gazebo_sim/config/study_cafe/study_cafe_route.yaml"

if [[ -e "$final_input" || -e "$final_environment" ]]; then
  echo "refusing to overwrite final input or environment for ${height} cm" >&2
  exit 1
fi

mkdir -p "$segment_inputs" "$segment_environments"
python3 "$workspace_root/tools/slam_evaluation/prepare_route_segments.py" \
  "$route_config" "$route_segments"

while IFS=$'\t' read -r index spawn config; do
  input="$segment_inputs/segment_$index"
  environment="$segment_environments/segment_$index"
  if [[ -f "$input/metadata.yaml" ]]; then
    echo "reusing completed segment $index"
    continue
  fi
  if [[ -e "$input" || -e "$environment" ]]; then
    timestamp=$(date +%Y%m%d-%H%M%S)
    [[ ! -e "$input" ]] || mv "$input" "${input}_failed_$timestamp"
    [[ ! -e "$environment" ]] || mv "$environment" "${environment}_failed_$timestamp"
  fi
  echo "recording segment $index"
  ROS_DOMAIN_ID=$((180 + 10#$index)) \
  ROBOT_SPAWN_POSE="$spawn" ROUTE_CONFIG="$config" \
  SLAM_INPUT_PATH="$input" SLAM_ENVIRONMENT_PATH="$environment" \
    "$workspace_root/tools/slam_evaluation/record_slam_input.sh" \
      "$height" "$noise_profile"
done <"$route_segments/manifest.tsv"

mapfile -t segments < <(
  find "$segment_inputs" -mindepth 1 -maxdepth 1 -type d \
    -name 'segment_[0-9][0-9]' | sort
)
if [[ ${#segments[@]} -ne 16 ]]; then
  echo "expected 16 completed segments, found ${#segments[@]}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ros_workspace/install/setup.bash"
python3 "$workspace_root/tools/slam_evaluation/merge_segmented_bags.py" \
  "$final_input" "${segments[@]}"
mkdir -p "$final_environment"
cp -a "$segment_environments" "$final_environment/segments"
cp -a "$route_segments" "$final_environment/routes"
echo "completed merged $noise_profile ${height} cm input bag"
