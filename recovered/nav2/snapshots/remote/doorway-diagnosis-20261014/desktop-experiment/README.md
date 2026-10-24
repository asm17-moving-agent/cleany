# Smac Lattice + MPPI 문 통과 실험

이 디렉터리는 원래 작업 트리와 노트북 실행을 보존하는 독립 실험이다. 최종 성능 판단은 `REPORT.md`와 각 실행의 `metrics.json`을 기준으로 한다. 현재 작업 중인 실행을 성공으로 해석하지 않는다.

## 최신: 수동 경유점 없는 공통 8경로 검증

최신 결과는 [generalization/REPORT.md](generalization/REPORT.md)다. 수동 경유점을 제거한 첫 묶음은 7/8이었고, 가까운 전역 경로 지점들로 연결하는 후보 다양화와 막힌 후퇴 후 bounded 재계획을 적용한 최종 묶음은 8/8이었다. 모든 최종 실행에 `candidate_diverse`, `generalization/common.yaml`, `generalization/recovery_replan.xml`을 동일 적용했다. 각 경로 1회 검증이며 반복 성공률을 보장하지 않는다.

`run_campaign.py --profile general`이 최신 기본값이다. 아래 M1 수동 경유점과 `initial`/`final` 프로필 설명은 이전 실험의 보존 기록이며 최신 적용 설정이 아니다. 최신 재현 명령과 전체 JSON/CSV는 위 보고서를 따른다.

## 환경과 실행

호스트에서 `distrobox enter cleany-ros-humble-cdi`로 기존 Ubuntu/ROS Humble 환경에 들어간다. `run.sh`는 ROS, 비교용 Cleany 런타임, 선택한 MPPI, TimedController 순으로 source한다. 기존 다른 실험과 ROS domain/IGN partition을 공유하지 않는다. 실행기는 매번 새 UUID partition을 만들고 자신이 시작한 프로세스만 종료한다.

```bash
cd /home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
distrobox enter cleany-ros-humble-cdi -- bash build.sh candidate_v2

# 개선 전: 실제 노트북 최적화 MPPI + 읽기 전용 계측
# output은 존재하지 않는 경로여야 한다.
distrobox enter cleany-ros-humble-cdi -- bash run.sh \
  --output "$PWD/reproduce_baseline" --goal 7 --domain 211

# 개선 후: 경로 후보 + 출력 검증 + 진행량 timeout + M1 사전 정렬
distrobox enter cleany-ros-humble-cdi -- bash -lc '
  export MPPI_INSTALL=candidate_v2/install MPPI_EXPERIMENT_MODE=pathseed
  export MPPI_CONFIG_OVERRIDE=$PWD/prealign_v2.yaml
  export MPPI_APPROACH_CONFIG=$PWD/portal_approaches_v3.yaml
  export MPPI_BT_OVERRIDE=$PWD/recovery_keep_path.xml
  unset MPPI_TEST_HOLD
  bash run.sh --output "$PWD/reproduce_final_m1" --goal 7 --domain 212
'

# 결과 집계와 실제 Gazebo GT 기록으로 GIF 만들기
distrobox enter cleany-ros-humble-cdi -- python3 summarize_run.py reproduce_final_m1
distrobox enter cleany-ros-humble-cdi -- python3 render_drive.py reproduce_final_m1
```

`prepare_experiment.py`는 canonical 18F world를 목표별로 확인하고 해당 의자만 제거됐는지 검사한다. 2.5cm 공통 정적 지도를 원래 노트북 방식(2×2 min/occupied union)으로 5cm 변환한다. 기존 별도 5cm 지도는 사용하지 않는다. 초기 HOME와 최종 목표는 공통 잠금 파일을 사용한다. GT는 기록만 하며 AMCL 초기화 외 주행 중 위치 주입은 없다.

`MPPI_APPROACH_CONFIG`는 지도별 mission waypoint YAML이다. M1은 문 밖에서 yaw=180°를 완료하고 문을 통과한 뒤 원래 최종 목표로 이동한다. 각 단계도 동일한 stock Smac Lattice와 MPPI를 사용한다. 최종 M1 경유점의 중간 checker는 2cm/2°이며, 최초 개선(v1)은 3.5cm/3°였다. 최종 checker는 기존30cm/10°다. 문 안의 짧은30cm 구간은 초기 Smac 경로를 유지하고 실패 때 재계획한다(`hold_path:true`, `stage_hold_path.xml`). 이 구간의 정체12초/충돌 검사는 유지한다. 단계 실패를 전체 실패로 기록하며 완료 상태와 GT 오차를 별도로 판정한다. 경유점은 범용 문 인식기나 자동 문 탐지 결과가 아니다.

## 제어 변경

`candidate_v2/src/nav2_mppi_controller`는 `provenance/mppi-original`의 복사본에 다음을 추가했다.

- `path_seed_count`, `path_seed_max_speed`: Omni의 일부 후보에 연속적인 경로 SE(2) 보간 입력을 넣는다. 나머지 Gaussian 후보와 함께 기존 모든 critic으로 평가한다. 모든 후보와 출력은 실제 메카넘 휠 결합 제한을 따른다.
- `validate_output`: weighted mean/Savitzky–Golay 이후 출력 시퀀스를 실제 footprint로 최대 1cm/1° 간격 검사한다. 무효일 때 이미 평가한 후보를 비용순으로 검사해 유효 후보를 선택한다. 모두 무효면 실패를 반환한다. 충돌 비용/footprint를 완화하지 않는다.
- `path_progress_timeout`, `path_progress_distance`: 같은 목적지에는 고정 참조 경로의 arc 진행량을 사용한다. 주기적 재계획은 정체 시간을 초기화하지 않는다. 최종 XY 영역 내 yaw 개선은 진행으로 인정한다. recovery 후 controller 호출 공백이 1초보다 길면 새 시도로 초기화한다.
- `MPPI_TRACE_FILE`: 2sim초마다 후보 yaw와 critic별 증분 비용을 CSV로 저장한다. MPPI 제어 정규화 이전 critic 비용이며 전체 목적함수와 구분한다.

기존 최적화 소스의 `goal_rotate_enter_radius`/`exit_radius`는 일반 프로필0.025/0.030m, 최종 M1 프로필0.010/0.015m로 각각 중간 checker보다 작게 적용한다. 최종 tolerance를 넓히지 않는다. `PathAlignCritic.offset_from_furthest=2`를 유지한다. 0 설정에서 발생한 원래 SIGSEGV와 주행 정체를 동일 원인으로 취급하지 않는다.

`recovery_bt.xml`: 5sim초 주기 Smac 재계획, controller 연속 실패 허용 0.5초, route 진행 0.08m/12초, 보조 SimpleProgressChecker 0.10m/20초. 실패 시 충돌 검사된 BackUp 0.35m/0.10m/s(8초 제한) 후 재계획, 최대 4회. 막힌 후퇴는 성공으로 처리하지 않는다. 전체 실행 제한은 300sim초 또는 1800wall초다.

## 재계획 경합 및 짧은 문 구간의 경로 유지

`recovery_keep_path.xml`은 재계획 결과를 별도 blackboard 값으로 받고, 성공한 결과만 현재 경로에 복사한다. Nav2 ComputePathToPose는 abort/cancel/halt 때 자신의 출력 경로를 비우므로 단순 ForceSuccess만으로 실패를 무시하면 기존 유효 경로도 소실된다. 초기 계획 성공은 필수이며, 주기적 재계획 실패 시에는 기존 경로를 계속 검증하면서 따른다. 컨트롤러 충돌/진행량 실패는 기존 bounded BackUp으로 넘어간다. `MPPI_BT_OVERRIDE=$PWD/recovery_keep_path.xml`로 선택한다. 공통8회 시험 도중 원래 BT를 덮어쓰지 않는다.

실제 Nav2 PipelineSequence/RecoveryNode와 BehaviorTree.CPP SetBlackboard를 사용하는 `native/test_replan_tree.cpp`로 기존 경합 실패, 수정된 경로 유지와 도착 성공, 초기 계획 실패/컨트롤러 실패의 복구 진입을 검사했다. 회전 도착 및 정체주입 라이브 재시험 결과를 REPORT에 별도로 기록한다. `stage_hold_path.xml`은 짧은 문 구간에서 주기적 재계획에 의한 lattice loop가 새로 삽입되는 것을 방지한다. 초기 계획과 실패 후 재계획은 반드시 stock Smac을 거친다.

## 기록과 해석

- `result.json`: Nav2, GT 오차, scene/실제 로드 라이브러리/실제 파라미터 검증, sim/wall/RTF, 명령 wheel 최대, GT smoothness.
- `metrics.json`: 위 결과 + joint wheel 최대, compute median/p95/p99/max, 관찰된 접촉. Nav2 status=4와 GT≤0.30m/10°를 동시에 요구한다.
- `navigation_stages.json`: 중간 목표별 status, 시작/종료 sim 시간과 GT.
- `candidate_costs.csv`: 후보별 yaw 궤적 knot, critic별 비용.
- `snapshots/`: 서비스로 읽은 원시 uint8 local/global costmap, TF, 실제 계획. 일반 costmap 토픽의 occupancy 재인코딩과 구분한다.
- `contacts.jsonl`: 모든 collision link 센서, 비바닥/비자체 접촉 모두 보존, 바닥 접촉은 양성 대조로 샘플링. 관찰 없음은 전신 무충돌의 독립적 보장이 아니다.
- `drive.gif`: 실제 Gazebo GT 주행 기록을 지도 위에 재생한 영상이며 GUI 화면 녹화가 아니다.

정체하거나 일찍 실패한 주행의 낮은 jerk/짧은 주행시간을 개선으로 해석하지 않는다. 명령 휠 제한과 Gazebo joint 응답의 순간 overshoot는 따로 기록한다. 단일 실행으로 성공률이나 통계적 성능 우위를 주장하지 않는다. 공통 8목표는 각각 1회이며 추가 5회 반복은 하지 않는다.

소스 패치는 원본 MPPI package 디렉터리에서 `patch -p1 --dry-run < candidate-v2.patch`로 확인한 뒤 적용할 수 있다. 원본 복사본에 대한 dry-run 결과는 `provenance/patch-dry-run.txt`에 저장했다. 기존 dirty 트리에 자동으로 적용하지 않았다.

## 8목표 재현 프로필

기록된 최초8회는 `--profile initial`이다. 발견한회귀수정후에는 목표1과M1만개별재검증했고, 전체8개재반복이나5회반복은하지않았다. `--profile final`은최종재현설정(일반BT경합수정, M1짧은경로유지)이며 나머지6개경로를이BT로다시검증했다는뜻은아니다.

```bash
distrobox enter cleany-ros-humble-cdi -- python3 run_campaign.py \
  --profile final --output "$PWD/reproduce_final8" --domain-base 220
```

기존 결과 경로를 덮어쓰지 않으며, JSON/CSV 통합은 `assemble_results.py`로 생성한다. `RESULTS.json`은 정상8회, 개발중실패, 별도정지주입시험을명시적으로구분한다.
