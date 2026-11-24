#!/usr/bin/env bash
# Replay a navigation-only bag and render its finalized pose graph at 5 cm.
set -eu -o pipefail
if [[ $# != 4 && $# != 5 ]]; then
    echo "Usage: $0 INPUT_DIR OUTPUT_DIR MAX_CONSTRAINT_DISTANCE_M ROS_DOMAIN_ID [LUA_OVERRIDES_FILE]" >&2
    exit 2
fi
input=$(realpath "$1")
output=$(realpath -m "$2")
distance=$3
domain=$4
[[ $distance =~ ^[0-9]+([.][0-9]+)?$ && $domain =~ ^[1-9][0-9]*$ && $domain -lt 233 ]]
test ! -e "$output/map.pbstream"
mkdir -p "$output/config"
cp "$input/config"/cartographer{_hardware,}_2d.lua "$output/config/"
cat > "$output/config/replay.lua" <<LUA
include "cartographer_hardware_2d.lua"
POSE_GRAPH.constraint_builder.max_constraint_distance = $distance
LUA
if [[ $# == 5 ]]; then cat "$5" >> "$output/config/replay.lua"; fi
echo "return options" >> "$output/config/replay.lua"
export ROS_DOMAIN_ID=$domain ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
bin="$(ros2 pkg prefix cartographer_ros)/lib/cartographer_ros"
nice -n 10 "$bin/cartographer_offline_node" \
    "-configuration_directory=$output/config" -configuration_basenames=replay.lua \
    "-bag_filenames=$input/navigation" -use_bag_transforms=true \
    "-save_state_filename=$output/map.pbstream" > "$output/replay.log" 2>&1
(cd "$output" && "$bin/cartographer_pbstream_to_ros_map" \
    -pbstream_filename=map.pbstream -map_filestem=map -resolution=0.05 > export.log 2>&1)
test -s "$output/map.pgm" && test -s "$output/map.yaml"
