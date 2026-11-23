#!/usr/bin/env bash
# Build the standalone offline worker in an existing ROS 2 Humble environment.
set -euo pipefail
out=${1:?usage: build.sh output_binary}
prefix=${RTABMAP_PREFIX:-/opt/ros/${ROS_DISTRO:-humble}}
arch=$(gcc -print-multiarch)
lib="$prefix/lib/$arch"
read -r -a opencv <<< "$(pkg-config --cflags --libs opencv4)"
if [[ -n ${PCL_INCLUDE_DIR:-} ]]; then
    pcl=("-I$PCL_INCLUDE_DIR")
else
    read -r -a pcl <<< "$(pkg-config --cflags pcl_common)"
fi
includes=("$prefix"/include/rtabmap-*)
[[ -d "${includes[0]}" ]] || { echo "RTAB-Map headers missing in $prefix" >&2; exit 1; }
mkdir -p "$(dirname "$out")"
g++ -O2 -std=c++17 "$(dirname "$0")/visual_slam.cpp" -o "$out" \
    "-I${includes[0]}" -I/usr/include/eigen3 "${pcl[@]}" \
    -L"$lib" "-Wl,-rpath,$lib" "-Wl,-rpath-link,$lib" \
    "-Wl,-rpath-link,/opt/ros/${ROS_DISTRO:-humble}/lib" \
    "-Wl,-rpath-link,/opt/ros/${ROS_DISTRO:-humble}/lib/$arch" \
    -lrtabmap_core -lrtabmap_utilite "${opencv[@]}"
