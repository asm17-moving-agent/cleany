# 再現 / 재현

실행 당시 host는 a16-fedora ARM64, Podman 컨테이너는 ros2-humble(ROS Humble/Nav2 1.1.20/Fortress 6.18)이다. 모든 경로는 host와 컨테이너에 동일하게 mount되어 있다. 이 문서의 완화 실행은 Gazebo 전용이다. 물리 로봇에 실행하지 않는다.

## 코드와 최소 테스트

```bash
WT=/home/ehdrms/.codex/worktrees/mppi-optimization/main
RUN=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013
podman exec -w "$WT" ros2-humble make test-mppi-experiment \
  MPPI_EXPERIMENT_WS="$RUN/reproduce-conservative-ws"
podman exec -w "$WT" ros2-humble make test-mppi-experiment \
  MPPI_EXPERIMENT_WS="$RUN/relaxed/reproduce-ws" \
  MPPI_EXPERIMENT_PATCH="$WT/tools/navigation_evaluation/mppi_humble_experiment/nav2-1.1.20-relaxed.patch"
```

새 workspace를 사용한다. tar 및 patch hash는 source_provenance.json에 남는다. `reproduction_source_check.json`은 원본 tar에 완화 patch를 적용한 source가 실제 빌드 source와 같았음을 기록한다. 위 명령은 production 설정을 변경하지 않는다.

## 보존된 overlay로 실행

`execute.sh`는 시스템 ROS, 별도 Cleany install, 해당 단계 MPPI install, timing wrapper를 순서대로 source한다. 설치 경로는 스크립트에 명시되어 있다. 재빌드한 overlay로 검증하려면 해당 스크립트의 MPPI install 경로를 복사본에서 변경한다. 실제 사용한 라이브러리 경로와 SHA는 각 목표 result.json에 저장되며 controller_server.yaml은 적용 dump다.

```bash
RUN=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013
# 새로운 출력 폴더를 사용하고 domain 226을 다른 실행과 공유하지 않는다.
podman exec -e MPPI_OVERRIDE="$RUN/final_conservative_overrides.yaml" ros2-humble \
  "$RUN/execute.sh" --output "$RUN/reproduce-conservative"
# 완화 runner의 생성 launch는 stage script를 output의 두 단계 위에서 찾으므로 아래 구조를 유지한다.
podman exec -e MPPI_OVERRIDE="$RUN/relaxed/final_overrides.yaml" ros2-humble \
  "$RUN/relaxed/execute.sh" --output "$RUN/relaxed/reproduce-final"
# 대표 단일 목표만 필요하면 --goal-index 0..4를 추가한다.
```

기준 Cleany commit은 b3ca816ba420452ee7960837bc16ec74148e07b0이다. 최종 YAML은 baseline effective controller dump에 merge하고 최종 robot safety overlay에도 적용한다. runtime parameter 검증, lifecycle 검증, `/cmd_vel` publisher 검증이 실패하면 실행을 infrastructure error로 저장한다. GT 도착 허용은 15cm/10deg 그대로다. 최종 목표 각 1회는 final_validation 폴더에 보존한다.

## 재집계

진행 중인 실험의 CPU 부하에 영향을 주지 않도록 실행 종료 후 집계한다.

```bash
RUN=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013
podman exec ros2-humble python3 "$RUN/analyze.py"
podman exec ros2-humble python3 "$RUN/generate_report.py"
```

analyze.py는 기존 comparison evidence의 analyze_comparison.py를 import한다. 원본 raw result.json 및 bags/logs는 수정하지 않는다. compute는 활성 navigation 구간 steady wall time, GT derivative는 sim time 20Hz/centered 5-point/양끝 .25초 제외다. Null reason은 미계측 접촉, packet depth 부재 등과 함께 저장한다. 원래 baseline 위치는 provenance.json에 있다. 중간 대표 후보 및 infrastructure error도 all_attempts.json에 남는다.
