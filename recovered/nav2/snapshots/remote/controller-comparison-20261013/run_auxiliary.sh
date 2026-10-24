#!/usr/bin/env bash
set -eo pipefail
experiment_root=/home/donglee/workspace/cleany/controller-comparison-20261013
source /opt/ros/humble/setup.bash
source "$experiment_root/cleany-data/builds/source/install/setup.bash"
source /tmp/cleany-teb-humble/install/setup.bash
source "$experiment_root/measurement_ws/install/setup.bash"
export CLEANY_COMPARISON_REPO="$experiment_root/source" CLEANY_BENCHMARK_TIMING=1
for candidate in dwb rpp teb; do
  case "$candidate" in
    dwb) output_group=aux_dwb ;;
    rpp) output_group=isolated_rpp ;;
    teb) output_group=isolated_teb ;;
  esac
  python3 "$experiment_root/run_comparison.py" --candidate "$candidate" \
    --output "$experiment_root/$output_group" --domain 225 --rtf 1 --goal-index 0 \
    > "$experiment_root/$output_group.log" 2>&1
done
