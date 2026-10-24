# MPPI Omni 개선 및 Gazebo 검증 — 2026-10-13 KST

기준 source commit은 `b3ca816ba420452ee7960837bc16ec74148e07b0`이다. ARM64 a16-fedora의 `ros2-humble` Podman 컨테이너, ROS2 Humble/Nav2 1.1.20/Fortress 6.18에서 실행했다. 물리 로봇 검증은 하지 않았다. 점수나 통계적 성공률을 계산하지 않는다.

보수 최종에서는 straight/turn_arrival/chair_gap/far_heading이 Nav2 도착, distant가 abort였다. 보수 chair_gap은 실제 XY 허용을 벗어났다. 완화 최종은 모든 목표에서 Nav2 도착 상태였지만 chair_gap의 실제 XY 오차는 여전히 허용 밖이다. 완화 활성 navigation 구간의 이름 확인 가능한 접촉 기록에서 ground/self를 제외한 환경 접촉은 관측되지 않았다. 최소 clearance 및 물리 검증은 미측정이다.

## 원래 공식 비교

Nav2 status 4는 추정 pose의 도착 결정이며, status 6은 abort이다. 실패 위치의 작은 XY 오차를 도착 정확도라고 해석하지 않는다.

| 목표 | Nav2 status | GT XY cm | GT yaw deg | sim s | 선형 jerk RMS | 각 jerk RMS |
| --- | --- | --- | --- | --- | --- | --- |
| straight | 4 | 13.422 | 0.986 | 22.088 | 3.446 | 0.613 |
| turn_arrival | 6 | 4.828 | 90.954 | 124.066 | 1.689 | 1.014 |
| chair_gap | 6 | 125.384 | 20.144 | 154.716 | 2.177 | 1.101 |
| far_heading | 6 | 4.584 | 87.747 | 148.486 | 1.342 | 1.057 |
| distant | 6 | 433.739 | 45.795 | 154.990 | 1.998 | 0.633 |

## 원인별 개선

1. 인계된 중단 재현은 완료 측정에서 제외했다. `/plan`의 endpoint `[4,0,pi/2]`가 보존됐고 MPPI raw wz≈0.1398이 존재했다. 반면 작은 XY 요청의 bearing 변화가 `TURNING_PAN`을 반복시켜 회전까지 막았다.
2. 도착 전용 탐색 영역: 10cm 내에 들어오면 XY 샘플/선택 sequence/history를 0으로 제한하고 13cm 밖에서 해제한다. goal checker의 XY tolerance 이하일 때만 활성화한다. measured 초기 속도는 rollout에 보존한다. 회전 궤적은 기존 전체 critic/footprint 검사에 들어가며 fallback retry에도 이 영역을 유지한다. 출력 뒤에 검사를 우회하는 회전 command를 만들어 넣지 않는다.
3. Humble의 Savitzky-Golay 필터가 제한을 넘는 값을 만들었다. 대표 실행 raw vx=0.15266, 최종 vx=0.15081을 확인했다. 필터 뒤에 기존 constraint를 다시 적용했다. 해당 대표 결과를 최종 제한 준수 결과로 사용하지 않는다.
4. 낮은 sampling std만 적용한 chair-gap은 jerk가 낮아졌으나 -29.88deg 자세에서 모든 rollout이 충돌 판정을 받아 abort했다. 고정 heading 경로와 Omni 이동의 목적에 맞게 PathAngleCritic을 비활성화하고 PathAlignCritic의 orientation 사용 및 offset=2를 적용했다. 내부 CostCritic, consider_footprint와 footprint/costmap은 유지했다. 이 후보는 도착했지만 200.65sim초가 필요했다.
5. 낮은 std의 제어 effort 비용 증가를 gamma=0.002로 보정했다. 나머지 설정을 유지한 먼 목표 대표 실행은 도착, 97.30sim초였다. 이후 고정한 최종 보수 후보로 5개 목표를 각 1회 실행했다.

원래 motion YAML의 time_steps=56/PathAlign=10/Goal=5 대신 실제 robot overlay가 80/3/15를 적용했음이 확인됐다. 최적화 러너는 실제 baseline dump를 시작점으로 삼아 controller override를 마지막 robot overlay에도 명시하고, 실제 ROS parameter dump 전체를 비교한다. 이는 계측/설정 적용 수정이다. 공통 planner, 지도, 모델, 센서, 안전 알고리즘을 변경한 실험으로 섞지 않는다.

`ax_max/ay_max/az_max`는 이 Humble 구현에서 읽지 않는다. 해당 YAML 존재를 native acceleration 제한 증거로 사용하지 않았다. TwirlingCritic은 `cost_weight/cost_power` 이름을 사용하며 실제 baseline weight=10을 유지했다.

## 보수 제한 최종 검증

| 목표 | Nav2 status | GT XY cm | GT yaw deg | sim s | wall s | 실제 RTF |
| --- | --- | --- | --- | --- | --- | --- |
| straight | 4 | 11.847 | 0.279 | 15.560 | 16.800 | 0.926 |
| turn_arrival | 4 | 6.919 | 8.238 | 45.614 | 48.600 | 0.939 |
| chair_gap | 4 | 17.465 | 0.758 | 43.396 | 46.500 | 0.933 |
| far_heading | 4 | 6.737 | 8.576 | 57.606 | 61.503 | 0.937 |
| distant | 6 | 301.790 | 0.364 | 71.410 | 76.902 | 0.929 |

| 목표 | compute median ms | p95 ms | p99 ms | 예외 | 50ms 초과 | CPU one-core % | RSS p95 MiB |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 5.561 | 10.679 | 13.041 | 0 | 0 | 29.455 | 56.712 |
| turn_arrival | 6.803 | 11.860 | 14.899 | 0 | 0 | 24.412 | 55.044 |
| chair_gap | 9.589 | 27.059 | 28.841 | 0 | 0 | 39.160 | 55.948 |
| far_heading | 6.528 | 12.140 | 14.485 | 0 | 0 | 25.287 | 55.070 |
| distant | 7.262 | 23.550 | 25.837 | 1 | 0 | 32.987 | 57.527 |

| 목표 | 선형 jerk RMS | p95 | 각 jerk RMS | p95 | 전체 정지 s | 재출발 | yaw 반전 | vy 반전 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 0.512 | 0.952 | 0.167 | 0.309 | 1.600 | 1 | 0 | 0 |
| turn_arrival | 0.253 | 0.172 | 0.268 | 0.367 | 1.650 | 1 | 0 | 0 |
| chair_gap | 0.844 | 2.092 | 0.286 | 0.504 | 2.450 | 9 | 1 | 1 |
| far_heading | 0.167 | 0.174 | 0.233 | 0.336 | 1.650 | 1 | 0 | 0 |
| distant | 0.846 | 1.602 | 0.280 | 0.377 | 3.250 | 8 | 1 | 0 |

| 목표 | controller wheel rad/s | 최종 command wheel | joint wheel | controller sim 주기 mean | p95 |
| --- | --- | --- | --- | --- | --- |
| straight | 2.544 | 2.544 | 2.544 | 0.046 | 0.060 |
| turn_arrival | 2.612 | 2.612 | 2.612 | 0.047 | 0.060 |
| chair_gap | 4.022 | 4.022 | 4.022 | 0.047 | 0.060 |
| far_heading | 2.788 | 2.788 | 2.788 | 0.047 | 0.060 |
| distant | 3.976 | 3.976 | 3.976 | 0.047 | 0.060 |

보수 단계는 |vx|≤0.15, |vy|≤0.10, |wz|≤0.14, 원래 240sim초/wall360초 제한과 모든 외부 안전 가드를 유지한다. controller_frequency=20Hz, model_dt=0.05, RTF 요청=1이다. 최종 표의 box 위반 수는 machine-readable 결과에 별도 기록했다(부동소수 비교 허용 1e-6).

chair-gap 최종 Nav2 추정 XY 오차는 samples.csv 마지막 TF 기준 14.49cm, GT는 17.47cm였다. goal checker의 15cm/10deg를 바꾸지 않았으며 localization 차이를 controller 성공으로 덮지 않는다. 실제 접촉 계측은 보수 단계에서 없으므로 collision은 null이며 미계측이다.

보수 distant는 71.410sim초에서 status 6으로 abort했다. GT `[6.4523,-3.1059,.00636]`에서 목표까지 3.0179m였다. 활성 navigation compute 예외 1회가 sim 84.066s에 기록됐고 launch.log는 `Optimizer fail to compute path` 뒤 wait/replan, `no_valid_lattice_path` 및 NavigateToPose abort를 보였다. controller와 planner가 보고한 실패 경계이며 실제 환경 접촉 또는 원자적 costmap 원인까지 증명하지 못한다. 대표 gamma 후보의 earlier 도착을 이 최종 실패 대신 사용하지 않는다.

## Gazebo 전용 완화 최종 검증

| 목표 | Nav2 status | GT XY cm | GT yaw deg | sim s | wall s | 실제 RTF |
| --- | --- | --- | --- | --- | --- | --- |
| straight | 4 | 12.585 | 0.279 | 12.168 | 13.204 | 0.922 |
| turn_arrival | 4 | 8.730 | 8.751 | 35.334 | 37.901 | 0.932 |
| chair_gap | 4 | 16.709 | 9.854 | 31.858 | 35.101 | 0.908 |
| far_heading | 4 | 7.339 | 8.290 | 44.706 | 48.601 | 0.920 |
| distant | 4 | 12.783 | 1.241 | 65.252 | 70.301 | 0.928 |

| 목표 | compute median ms | p95 ms | p99 ms | 예외 | 50ms 초과 | CPU one-core % | RSS p95 MiB |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 5.742 | 10.213 | 13.103 | 0 | 0 | 21.077 | 55.734 |
| turn_arrival | 7.538 | 13.159 | 14.972 | 0 | 0 | 23.799 | 55.454 |
| chair_gap | 9.240 | 25.522 | 37.490 | 0 | 0 | 35.061 | 55.906 |
| far_heading | 7.027 | 11.872 | 14.699 | 0 | 0 | 22.435 | 55.833 |
| distant | 5.770 | 21.231 | 25.843 | 0 | 0 | 27.230 | 58.203 |

| 목표 | 선형 jerk RMS | p95 | 각 jerk RMS | p95 | 전체 정지 s | 재출발 | yaw 반전 | vy 반전 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 0.473 | 0.294 | 0.182 | 0.374 | 1.450 | 1 | 0 | 0 |
| turn_arrival | 0.226 | 0.193 | 0.280 | 0.349 | 1.700 | 1 | 0 | 1 |
| chair_gap | 0.272 | 0.343 | 0.246 | 0.436 | 1.200 | 1 | 3 | 0 |
| far_heading | 0.215 | 0.197 | 0.277 | 0.372 | 1.400 | 1 | 1 | 3 |
| distant | 0.499 | 0.262 | 0.181 | 0.354 | 1.350 | 2 | 4 | 2 |

| 목표 | controller wheel rad/s | 최종 command wheel | joint wheel | controller sim 주기 mean | p95 |
| --- | --- | --- | --- | --- | --- |
| straight | 3.593 | 3.593 | 3.593 | 0.046 | 0.060 |
| turn_arrival | 3.758 | 3.758 | 3.758 | 0.047 | 0.060 |
| chair_gap | 3.955 | 3.955 | 3.952 | 0.046 | 0.060 |
| far_heading | 3.737 | 3.737 | 3.737 | 0.046 | 0.060 |
| distant | 3.961 | 3.961 | 3.961 | 0.047 | 0.060 |

| 목표 | 환경 접촉 관측 samples | 접촉 episodes | max depth m | ground heartbeat samples | 이름 미확인 | 240s 경과 |
| --- | --- | --- | --- | --- | --- | --- |
| straight | 0 | 0 | null | 24072 | 0 | False |
| turn_arrival | 0 | 0 | null | 69349 | 0 | False |
| chair_gap | 0 | 0 | null | 62654 | 0 | False |
| far_heading | 0 | 0 | null | 87854 | 0 | False |
| distant | 0 | 0 | null | 128397 | 0 | False |

## 완화 튜닝과 남은 문제

처음 넓은 sampling std(.075/.06/.20), gamma=.008을 사용한 의자 사이 후보는 45.20sim초 도착했지만 wheel 최대 2.565rad/s로 결합 영역을 충분히 쓰지 못했다. 초기 contact bridge 방향 오류로 그 후보 접촉값은 null이다. bridge를 GZ→ROS로 수정한 뒤 낮은 std(.035/.025/.07), gamma=.002 직선은 13.536sim초/wheel 최대 3.418, 선형 jerk RMS .415였다. gamma만 .0003으로 낮춘 직선은 12.162sim초/wheel 최대 3.647, 선형 jerk RMS .459였다. 이 설정을 고정하고 최종 다섯 목표를 각각 한 번 검증했다. 완화 후보의 작은 jerk는 다른 외부 가드/계측 조건에서도 측정되므로 보수 결과와 단일 원인 효과로 해석하지 않는다.

의자 사이 최종 완화는 Nav2 status 4지만 GT XY 16.71cm로 도착 허용을 벗어났고 yaw 9.85deg였다. 보수 단계에서도 GT 17.47cm였다. localization과 GT 차이는 미해결이며 최종 tolerance를 넓히거나 재실행으로 대체하지 않았다. contact packet의 depth 배열은 비어 있어 실제 penetration depth는 null로 남긴다. 접촉 관측이 없는 경우에도 최소 장애물 clearance는 별도로 계측하지 않아 null이다.

## 완화 프로파일 경계

Gazebo 전용이다. 원래 속도 박스를 대신하여 r=.0635, wheelbase=.35, separation=.6038, lever=.4769 및 각 wheel≤5rad/s의 결합 영역을 적용한다. 순수 직진 한계는 .3175m/s, 순수 회전 한계는 약 .66576rad/s이다. 모든 축을 동일 비율로 축소해 전체 twist 비율을 유지한다. MPPI sampled control을 rollout 전에, optimized sequence를 update와 최종 필터 뒤에 projection한다. 최종 `/cmd_vel` projector가 같은 기구학으로 다시 검사한다.

실제 경로는 `/nav2/cmd_vel -> coupled wheel projector -> /cmd_vel`이다. pan gate, CollisionMonitor 및 serial voxel/freshness 경로는 계속 실행하고 최종 출력만 `/shadow/cmd_vel`로 분리한다. 이 가드의 STOP/PASS 수는 반사실 shadow 관측이며 실제 정지 횟수가 아니다. controller 내부 footprint/feasibility 검사는 유지한다. 별도 controller command loss watchdog .5sim초는 유지하고 `/wheel_limit/state`에 원인을 기록한다. 센서 freshness 또는 pan alignment를 최종 구동 권한으로 쓰지 않는다.

progress allowance=1500sim초(실험 hard cap보다 큼), controller failure_tolerance=-1(무기한 controller retry를 hard watchdog이 한정)을 적용한다. 240sim초는 soft 이벤트, hard cap=1200sim초/wall1800초이다. 예외/정체/timeout을 그대로 기록한다. 제한 완화가 원인을 해결했다는 증거로 쓰지 않는다.

완화 world에는 Fortress Contact system과 실제 모든 collision을 가진 6개 robot link의 contact sensor/ROS bridge만 추가한다. 기존 robot model을 sensor 추가 전후 비교하여 모든 collision/pose/joint 형상 불변을 확인한다. ground와 self contact는 환경 충돌과 구분한다. 관측된 contact packet 수는 접촉 횟수 자체가 아니므로 unique sim stamp에서 50ms 이상 끊긴 구간을 episode로 별도 계산한다. ground heartbeat 또는 collision 이름이 없으면 환경 접촉값은 null과 이유를 남긴다. 외부 가드 우회 및 추가 contact 계측은 보수 프로파일과 다른 실험 조건이다. 다른 controller와 완화 비교에도 같은 변경을 적용해야 한다.

## 계측 및 해석

GT message twist의 body-frame velocity를 20Hz sim time으로 보간하고 centered 5-point moving average 후 acceleration/jerk를 계산했다. 양 끝 .25s를 제외했다. 전체 정지 시간은 XY≤.01m/s이고 yaw≤.02rad/s인 샘플이다. 원래 비교의 stationary_s는 XY 기준이라 회전 중도 포함되므로 전체 정지와 혼동하지 않는다. acceleration RMS/p95(선형 m/s², 각 rad/s²), jerk RMS/p95(선형 m/s³, 각 rad/s³), 정지/재출발 및 축 반전은 JSON/CSV에 있다. body velocity 변화이며 물리 승차감/가속도 계측으로 해석하지 않는다. 실패/장기 정지로 낮아진 jerk를 개선으로 판정하지 않는다.

computeVelocityCommands는 steady wall 시간이며 파일 I/O는 측정 구간 밖이다. median/p95/p99, 예외 및 50ms 초과를 활성 navigation 구간에서 집계했다. CPU는 controller_server의 Linux proc 카운터 delta, RSS는 p95다. 이 데이터는 같은 ARM64 호스트 내부의 기술 지표다. 다른 CPU 호스트와 절대 CPU/compute 순위를 비교하지 않는다. 다른 세션의 동시 부하와 완화의 contact 계측 부하는 결과의 한계다. 원래 공식 MPPI 5회에는 per-cycle compute가 없고 원래 straight profile p95=10.75ms는 별도 참고 값이다.

## 테스트

보수 optimizer unit 15개와 smoke 3개가 통과했다. upstream critics suite는 23개 중 PathAlignCritic의 float 합산 assertion 1개가 실패했다(6599.984375 vs 6600, 허용 .01; 차이 .015625). 수정하지 않은 `/opt/ros/humble` controller/critics 라이브러리로 같은 테스트 바이너리를 실행해 동일 실패를 재현했다. 해당 한계를 보존하고 assertion을 완화하지 않았다. 완화 optimizer 16개와 smoke 3개도 통과했다. 추가 projection 테스트는 8개 twist 부호 조합에서 모든 wheel≤5 및 축 비율 보존, 순수 직진 .3175와 작은 twist 불변을 확인했다. 결과는 validation 디렉토리와 로그에 보존한다.

## 아티팩트와 재현

결과 루트: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi/sol_resume_20261013`

- `conservative_results.json/csv`, `relaxed_gazebo_only_results.json/csv`, `results.json/csv`: 최종 5개 목표씩. `all_attempts.json`은 startup 실패와 대표 튜닝을 포함한다.
- `final_validation/*`: 보수 raw bag/log/commands/velocity/joints/resources/compute/diagnostics/plans/runtime dump.
- `relaxed/final_validation/*`: 완화 동일 원시 자료 및 contacts.json, shadow command, wheel-limit state.
- `final_conservative_overrides.yaml`, `relaxed/final_overrides.yaml`, 각 목표 `controller_server.yaml`: 요청 설정과 실제 적용 설정.
- `nav2-1.1.20.patch`, `relaxed/nav2-1.1.20-relaxed.patch`: pinned upstream에 대한 코드/테스트 patch.
- `cleany_experiment.patch`: Makefile, package README와 실험 도구 전체의 통합 patch.
- `provenance.json`, `validation/`, `relaxed/validation/`, `REPRODUCE.md`: 환경/바이너리/검증/재현 근거.

기존 dirty main, 원래 baseline evidence 및 인계 중단 evidence를 보존했다. 소유 실험만 종료하고 push하지 않았다. 최종 프로세스 검사는 `remaining_owned_processes.json`에 저장한다.

공식 API/계측 구현 근거: [Nav2 1.1.20 optimizer](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/optimizer.cpp), [GoalAngleCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/goal_angle_critic.cpp), [PathAngleCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_angle_critic.cpp), [Fortress Contact system](https://raw.githubusercontent.com/gazebosim/gz-sim/ign-gazebo6/src/systems/contact/Contact.cc).
