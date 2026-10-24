# MPPI 인계 — 사용자 취소로 중단 (2026-10-13 KST)

기존 개선/실험 요청이 취소되어 중단했다. 새 개선 실험, 제품 코드 변경, 최종 5개 목표 검증은 수행하지 않았다. 기존 main의 dirty 변경과 기준 evidence를 보존했고 push/commit/reset/삭제는 하지 않았다.

## 소유 경로와 상태

- checkout: `/home/ehdrms/.codex/worktrees/mppi-optimization/main`
- HEAD: `b3ca816ba420452ee7960837bc16ec74148e07b0`, detached HEAD, git status clean. Codex 관리 worktree로 이 채팅에 첨부되어 있다.
- 전용 build/install/log: `/home/ehdrms/workspace/cleany/cleany-data/builds/mppi-optimization/{build,install,log}`
- 계측 plugin 전용 source/build/install: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/measurement_ws`
- output: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi`
- 원래 evidence: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-20261013`
- container: `ros2-humble` (Podman, ARM64). Nav2 MPPI 1.1.20, Fortress 6.18.0 확인.

`make build-scenario-runtime`는 7개 패키지 빌드 성공. 계측 plugin도 전용 디렉토리에서 빌드 성공. 기준 install을 build underlay로 읽었지만 덮어쓰지 않았다. 런타임은 ROS Humble + 전용 install + 전용 measurement install을 source했다.

## 중단된 대표 재현

`diagnostic_baseline/1_turn_arrival`에 변경 없는 baseline을 계측 wrapper로 재현했다. domain 232, partition `cleany_comparison_880946ec6cd74db284a1189b56213374`, RTF 요청 1. 사용자가 중단을 요청해 약 102 wall초 후 runner PID 149062에 SIGINT를 보냈다. 결과는 `completed:false`, `KeyboardInterrupt`이며 도착 결과로 해석하면 안 된다.

원시 bag (~31 MB), launch.log, commands.csv, velocity.csv, joints.json, resources.json, compute_timing.csv, diagnostics.json, plans.json, config, scenario, runtime_verified 및 runtime guard 전이 로그를 보존했다. Python ROS context가 SIGINT에서 먼저 무효화되어 node.finish/save에 cleanup_error가 기록되었다. samples.csv는 없고 bag/commands/diagnostics는 저장됐다. bag metadata는 생성됐다.

runner와 launch/Gazebo/Nav2/bag 자식은 종료됐다. 제가 CLI 조회로 만든 domain 232 ros2 daemon도 종료했다. 최종 `/proc` 확인에서 소유 domain/partition 프로세스는 없음: `remaining_owned_processes.json`은 `[]`. 다른 채팅의 프로세스는 건드리지 않았다. 빌드 세션도 완료됐고 전역 find 조회 세션도 종료 확인했다.

## 발견사항과 미검증 제안

1. 원래 official_mppi turn_arrival samples.csv에서 약 sim 60초 이후 raw_wz는 +0.1398인데 safe_wz가 거의 0이었다. yaw critic 미작동으로 단정할 수 없다.
2. 새 부분 재현의 마지막 pan 진단: sim 90.454, requested `[0.00231,0.01004,0.13980]`, pan target 1.34488 vs actual 0.71824 rad, `TURNING_PAN`, output `[0,0,0]`. 미세한 XY 보정이 카메라 travel bearing을 바꾸며 회전 요청까지 막는 증거다. 전체 진단에서 TURNING_PAN 905, WAIT_FRESH_DEPTH 254, PASS 572였다. startup/중단과 wall/clock timeout도 포함되므로 이 수치를 정식 결과 비율로 쓰지 말 것.
3. 3D guard에서도 Command timeout 491, local voxel window coverage 오류 3건 관측. pan gate 차단 및 전달 중단과 분리해 시간순으로 분석할 필요가 있다.
4. 중요한 설정 merge: 저장된 comparison_motion.yaml은 time_steps 56, PathAlign weight10, PathFollow/Goal weight5지만 실제 runtime_verified와 param dump는 time_steps80, PathAlign3, PathFollow/Goal15이다. **원래 공식 baseline runtime_verified도 80/3/15**이므로 새 재현만의 drift는 아니다. Gazebo wrapper의 robot safety overlay가 motion overlay 뒤에 merge되어 controller 설정을 덮는다. `launch_runtime.py:80`의 robot merge, Gazebo `amcl_nav2_safety.launch.py`의 robot=safety 경로 확인. 다음 튜닝은 출력 YAML만 바꾸지 말고 실제 적용 파라미터를 검증해야 한다. `baseline_effective_controller.yaml`은 ros2 param dump 결과.
5. Humble 1.1.20의 MPPI motion model/optimizer 공식 소스는 ax_max/ay_max/az_max를 읽거나 가속 제한을 적용하지 않는다. 기존 YAML에 해당 값이 존재해도 제한이 작동한다고 주장할 수 없다.
6. Humble TwirlingCritic은 `cost_weight`/`cost_power`를 읽는다. 기준 YAML의 twirling_cost_weight/power는 해당 API 이름과 다르다. 기본 weight10이 적용되고 goal XY tolerance 내에서는 TwirlingCritic이 비활성이다. GoalAngleCritic은 transformed path 마지막 yaw를 사용한다. StraightFirstPlanner는 goal XY의 회전 pose들과 최종 goal yaw를 보존한다. plans.json으로 실제 endpoint를 추가 확인할 것.

아직 코드/튜닝 변경은 없다. 후보는 MPPI controller 내부의 goal tolerance 내 불필요한 XY 출력을 0으로 만드는 arrival phase 또는 노이즈/critic 조정이지만 구현/검증하지 않았다. pan gate나 freshness/footprint/safety 검사를 완화하지 말 것. 공통 계층 변경이 필요하면 controller 전용 개선과 분리하고 다른 후보에도 공정 비교 적용 필요성을 명시할 것.

공식 소스 확인:
- https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/goal_angle_critic.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/twirling_critic.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/optimizer.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp

## 재개 방법 (사용자 새 요청 후)

새 채팅에서 worktree를 attach하거나 필요한 source를 별도 checkout으로 옮겨 쓰고 경로를 검증한다. `execute.sh`는 위 전용 경로를 고정한다. `run_optimization.py`는 기존 remote runner를 복사해 intermediate command topics, pan/3D diagnostics, plans 계측 및 MPPI_OVERRIDE hook을 추가했다. override는 shallow merge이며 robot safety overlay에 의해 재덮어쓸 수 있어 아직 튜닝 용도로 완성되지 않았다.

새 출력 디렉토리로 실행 예시:

```bash
podman exec ros2-humble /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/execute.sh --goal-index 1 --output /home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/resume_diagnostic_01
```

기존 diagnostic_baseline은 덮어쓰지 않는다. 다음 단계는 중단 로그의 command stage/time/reason 정렬, 실제 parameter merge 확인, 한 가지 원인별 수정과 대표 검증이다. 이후 최종 설정으로 공통 5개 목표 각 1회, 정확도/시간/compute/RTF/CPU/RSS/wheel/jerk 계측 및 REPORT.md/JSON/CSV/YAML/patch 정리 작업이 남았다. 물리 로봇은 범위 밖이다.
