#!/bin/bash
set -euo pipefail
RUN="/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a"
COMMON=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/common
while [[ ! -f "$COMMON/READY" ]]; do sleep 30; done
python3 "$RUN/freeze_common.py"
podman exec ros2-humble "$RUN/execute.sh" --output "$RUN/final_validation" --domain 226 --rtf 1 > "$RUN/campaign.supervisor.log" 2>&1
