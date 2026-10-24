# MPPI 18F 8-goal reproduction

실행 환경: a16-fedora ARM64, Podman ros2-humble, ROS2 Humble/Nav2 1.1.20/Fortress. reused MPPI binary/config hashes는 provenance.json과 각 목표 result.json에 보존한다. raw previous 5-goal evidence는 별도 경로에 그대로 있다. 이번 캠페인은 기존 월드/5개 목표/15cm를 사용하지 않는다.

## READY snapshot과 실행

공통 DWB preflight가 goals.locked.json과 READY를 만든 후 이 디렉토리의 freeze_common.py를 host에서 실행했다. common_snapshot은 공유 fixture, 목표별 canonical world와 matching map, 최종 goal JSON의 읽기 전용 사본이다. common_snapshot_manifest.json의 hash를 확인한다. 공유 common을 수정하지 않는다.

새 결과 폴더를 사용한다. 아래 RUN은 이 문서가 있는 디렉토리의 절대 경로다.

```bash
RUN="/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a"
podman exec ros2-humble "$RUN/execute.sh" --output "$RUN/reproduction-new" --domain 226 --rtf 1
```

다른 실행이 domain226을 사용 중이면 시작하지 않는다. IGN_PARTITION/GZ_PARTITION은 목표마다 UUID로 분리하고 loopback discovery를 사용한다. 한 목표만 확인하려면 --goal-index 0..7을 추가한다. 원래 캠페인은 목표마다 fresh world/home으로 한 번씩 총8회이며, 재현은 새 캠페인으로 구분한다. GUI나 물리 로봇을 실행하지 않는다.

각 목표 generated run.launch.py/config/world.sdf/map.yaml은 실제 실행한 파일이다. URI는 common_snapshot/fixture 경로로 materialize했다. 각 목표 scene_validation.json에서 target chair만 제거된 canonical scene/model hash와 나머지 의자 보존을 확인한다. server_scene_info.pbtxt 및 launch.log에 실제 server scene/mesh 로딩 근거를 남긴다.

## 설정과 계측

final_overrides.yaml은 이전 최종 완화 MPPI 설정 그대로다. SimpleGoalChecker .30m/10deg, controller20Hz/model_dt=.05, coupled wheel5rad/s를 실제 ROS parameters로 확인한다. 원래 축별 박스를 복원하지 않는다. 외부 pan/depth/freshness/CollisionMonitor/voxel guard 최종 출력은 shadow이고 내부 collision footprint는 유지한다. controller command .5sim초 watchdog은 별도 유지한다. hard1200sim/1800wall, RTF 요청1; 실제 period/RTF는 측정한다.

/ground_truth/odom을 월드 정렬 map 좌표 그대로 판정한다. body-frame GT velocity를20Hz 보간, centered5-point smoothing, 양 끝 .25s 제거 후 acceleration/jerk를 계산한다. 정지는 |v_xy|<.01 AND |wz|<.02이며 yaw/vy 반전은 deadband .02/.01이다. contact는 모두 bag 및 contacts.jsonl에 보존한다. JSONL streaming은 1200초 제한의 RAM 누적을 방지하는 계측 변경이며 샘플을 버리지 않는다. ground/self는 환경 접촉에서 구분한다. 최소 footprint clearance와 제공되지 않는 contact depth는 null+이유다.

computeVelocityCommands steady wall median/p95/p99/예외/50ms 초과, proc CPU/RSS는 활성 navigation 구간이다. timing wrapper 파일 I/O는 측정 구간 밖이다. 다른 CPU/호스트 절대 성능 순위를 계산하지 않는다. static preflight와 실제 dynamic depth/memory runtime는 다른 검증 범위다. 실패/timeout도 포함하고 합격점수나 목표별 맞춤 튜닝을 하지 않는다.

```bash
RUN="/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a"
podman exec ros2-humble python3 "$RUN/analyze.py"
podman exec ros2-humble python3 "$RUN/audit_final.py"
podman exec ros2-humble python3 "$RUN/collect_boundary_evidence.py"
podman exec ros2-humble python3 "$RUN/generate_report.py"
```

analysis는 기존 비교의 analyze_comparison.py를 import하여 동일 jerk 및 proc 지표를 사용한다. contacts.jsonl은 streaming 집계한다. 원시 result.json/bags/log는 덮어쓰지 않는다. 종료 후 remaining_owned_processes.json에서 소유 domain/partition 프로세스가 없음을 확인한다.

## Reused pinned MPPI overlay 재빌드

mppi-source.patch는 이번에 재사용한 Nav2 1.1.20 완화 source patch다. SHA와 upstream tar provenance는 reused_build_provenance.json 및 repository의 prepare_overlay.py에 있다. 새 workspace를 사용한다.

```bash
WT=/home/ehdrms/.codex/worktrees/mppi-optimization/main
RUN="/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a"
podman exec -w "$WT" ros2-humble make test-mppi-experiment \
  MPPI_EXPERIMENT_WS="$RUN/rebuilt-mppi-ws" \
  MPPI_EXPERIMENT_PATCH="$RUN/mppi-source.patch"
```

새 overlay를 사용할 때 execute.sh 복사본의 MPPI install 경로를 새 install/local_setup.bash로 변경한다. Cleany install과 timing measurement overlay 경로는 execute.sh에 명시되어 있다. 기존 simulation proof는 actual loaded hashes를 기준으로 읽는다. reused_validation은 이전 optimizer16/smoke3 결과의 사본이며 이번 runtime 검증과 구분한다.
