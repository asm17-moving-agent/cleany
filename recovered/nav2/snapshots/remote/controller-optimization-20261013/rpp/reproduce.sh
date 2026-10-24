#!/bin/bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK_PHASE="${1:-}"
case "$TASK_PHASE" in
 conservative) TASK_RUNNER="$TASK_ROOT/sol61_20261013_0620/run.sh"; TASK_TUNE=curvature_lookahead18 ;;
 relaxed) TASK_RUNNER="$TASK_ROOT/relaxed_sol61_20261013/run.sh"; TASK_TUNE=relaxed ;;
 *) echo 'Usage: reproduce.sh {conservative|relaxed}' >&2; exit 2 ;;
esac
TASK_OUTPUT="$TASK_ROOT/replay_${TASK_PHASE}_$(date -u +%Y%m%dT%H%M%S)_$(python3 -c 'import uuid; print(uuid.uuid4().hex[:8])')"
distrobox enter cleany-ros-humble-cdi -- bash -c 'cd /home/donglee/workspace/cleany/controller-optimization-20261013/rpp-source; make build-gazebo CLEANY_BUILD_ROOT=/home/donglee/workspace/cleany/controller-optimization-20261013/rpp-build-v2'
distrobox enter cleany-ros-humble-cdi -- bash "$TASK_RUNNER" --tune "$TASK_TUNE" --output "$TASK_OUTPUT" --domain 228
printf 'Results: %s\n' "$TASK_OUTPUT"
