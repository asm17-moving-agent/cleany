# MPPI Omni — 18F facility 8개 목표 — 2026-10-14 KST

각 목표 fresh world/home에서1회씩 실행했다. Nav2 status와 GT 오차는 별도 관측이며 합격 점수나 통합 순위를 계산하지 않는다. 이전5개 목표/15cm/옛 study-cafe 실험을 이번 결과에 혼합하지 않는다. Gazebo 전용이고 물리 검증은 없다.

## 공통 조건과 잠금

시작 map/world pose `[-1.865,-4.705,1.5708]`, 도착 XY .30m/yaw10deg, wheel 최대5rad/s, controller20Hz/model_dt=.05, RTF요청1, hard1200sim/1800wall이다. StraightFirstPlanner 및 actual footprint/padding/internal collision을 유지한다. 공통 DWB 담당이 footprint+planner/scene을 사전 검증한 READY와 locked JSON snapshot만 캠페인에 사용한다. 공유 common을 수정하지 않았다. 목표 의자만 제거하고 나머지 의자/형상을 canonical hash로 확인했다. URI는 소유 snapshot fixture에 materialize하고 실제 server scene/로그를 남긴다.

| index | 목표 | 잠금 map/world pose | 제거 의자 | 좌표 조정 |
| --- | --- | --- | --- | --- |
| 0 | straight | [-1.865, -3.705, 1.5708] | null | null |
| 1 | turn_arrival | [-1.865, -2.8, 0] | null | null |
| 2 | chair_gap | [0.575, -1.625, 1.570796] | office_chair_29 | null |
| 3 | far_heading | [-1.865, -0.3, -1.570796] | null | null |
| 4 | distant | [-5.525, 4.775, -1.570796] | office_chair_01 | null |
| 5 | lateral | [-1.265, -4.705, 1.5708] | null | null |
| 6 | narrow_seat | [-3.125, -4.475, 1.570796] | office_chair_43 | null |
| 7 | other_room | [-16.775, 1.775, 1.570796] | facility_chair_M14 | null |

## 최종 목표별 관측

| 목표 | 결과 | Nav2 status | GT XY cm | GT yaw deg | sim s | wall s | 실제 RTF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight | arrival | 4 | 27.349 | 0.306 | 5.991 | 19.701 | 0.304 |
| turn_arrival | arrival | 4 | 10.861 | 8.571 | 21.363 | 92.802 | 0.230 |
| chair_gap | arrival | 4 | 29.137 | 3.986 | 25.128 | 110.501 | 0.227 |
| far_heading | arrival | 4 | 6.395 | 7.079 | 43.156 | 199.102 | 0.217 |
| distant | arrival | 4 | 13.790 | 6.216 | 76.953 | 341.801 | 0.225 |
| lateral | arrival | 4 | 27.565 | 0.298 | 6.203 | 27.201 | 0.228 |
| narrow_seat | arrival | 4 | 33.309 | 0.760 | 7.856 | 35.800 | 0.219 |
| other_room | timeout | 5 | 1345.473 | 48.106 | 392.631 | 1800.011 | 0.218 |

| 목표 | compute median ms | p95 | p99 | 예외 | 50ms 초과 | CPU one-core % | RSS p95 MiB | command sim mean s | p95 s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 8.942 | 14.317 | 15.266 | 0 | 0 | 28.041 | 59.113 | 0.015 | 0.022 |
| turn_arrival | 6.879 | 11.552 | 13.783 | 0 | 0 | 24.326 | 60.266 | 0.012 | 0.017 |
| chair_gap | 6.034 | 15.115 | 22.607 | 0 | 0 | 23.979 | 60.422 | 0.011 | 0.017 |
| far_heading | 6.844 | 12.566 | 14.191 | 0 | 0 | 24.370 | 60.684 | 0.011 | 0.016 |
| distant | 5.393 | 10.911 | 13.485 | 0 | 0 | 21.367 | 62.301 | 0.011 | 0.016 |
| lateral | 7.685 | 13.169 | 15.267 | 0 | 0 | 27.346 | 60.316 | 0.011 | 0.017 |
| narrow_seat | 7.783 | 20.995 | 24.824 | 0 | 0 | 29.933 | 60.504 | 0.011 | 0.016 |
| other_room | 17.167 | 26.879 | 32.527 | 0 | 1 | 39.725 | 64.645 | 0.013 | 0.016 |

| 목표 | call sim median s | p95 | p99 | call wall median s | p95 | p99 | command wall mean s | p95 s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 0.015 | 0.022 | 0.026 | 0.050 | 0.050 | 0.050 | 0.050 | 0.055 |
| turn_arrival | 0.011 | 0.017 | 0.022 | 0.050 | 0.050 | 0.051 | 0.050 | 0.055 |
| chair_gap | 0.011 | 0.017 | 0.020 | 0.050 | 0.050 | 0.050 | 0.050 | 0.055 |
| far_heading | 0.011 | 0.016 | 0.018 | 0.050 | 0.050 | 0.050 | 0.050 | 0.055 |
| distant | 0.011 | 0.016 | 0.018 | 0.050 | 0.050 | 0.051 | 0.050 | 0.055 |
| lateral | 0.011 | 0.016 | 0.018 | 0.050 | 0.050 | 0.050 | 0.050 | 0.055 |
| narrow_seat | 0.011 | 0.016 | 0.018 | 0.050 | 0.050 | 0.050 | 0.050 | 0.054 |
| other_room | 0.011 | 0.016 | 0.018 | 0.050 | 0.050 | 0.050 | 0.059 | 0.059 |

| 목표 | 선형 jerk RMS | p95 | 각 jerk RMS | p95 | 전체 정지 s | 재출발 | yaw 반전 | vy 반전 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 0.705 | 0.633 | 0.260 | 0.522 | 0.500 | 1 | 0 | 0 |
| turn_arrival | 0.215 | 0.440 | 0.444 | 0.761 | 0.350 | 1 | 0 | 1 |
| chair_gap | 0.297 | 0.385 | 0.233 | 0.430 | 0.350 | 1 | 1 | 1 |
| far_heading | 0.150 | 0.270 | 0.366 | 0.744 | 0.350 | 1 | 0 | 0 |
| distant | 0.185 | 0.331 | 0.296 | 0.557 | 0.350 | 1 | 1 | 1 |
| lateral | 0.367 | 0.399 | 0.209 | 0.417 | 0.400 | 1 | 0 | 0 |
| narrow_seat | 0.514 | 0.461 | 0.236 | 0.456 | 0.350 | 1 | 0 | 0 |
| other_room | 0.551 | 0.616 | 0.327 | 0.661 | 298.300 | 474 | 20 | 25 |

| 목표 | controller wheel rad/s | 최종 command wheel | joint wheel | 환경 contact samples | episodes | ground heartbeat samples | 이름 미확인 | max depth m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 2.807 | 2.807 | 2.807 | 0 | 0 | 23713 | 0 | null |
| turn_arrival | 3.711 | 3.711 | 3.711 | 0 | 0 | 85262 | 0 | null |
| chair_gap | 3.921 | 3.921 | 3.921 | 0 | 0 | 100205 | 0 | null |
| far_heading | 3.714 | 3.714 | 3.714 | 0 | 0 | 172264 | 0 | null |
| distant | 4.111 | 4.111 | 4.111 | 0 | 0 | 306898 | 0 | null |
| lateral | 1.232 | 1.232 | 1.232 | 0 | 0 | 24751 | 0 | null |
| narrow_seat | 3.371 | 3.371 | 3.371 | 0 | 0 | 31389 | 0 | null |
| other_room | 3.664 | 3.664 | 3.664 | 0 | 0 | 1566126 | 0 | null |

관측한 실패와 경계:

- narrow_seat: GT XY가30cm 밖. raw log/command_reason_timeline.json 참조.
- other_room: Nav2 status=5, outcome=timeout; GT XY가30cm 밖; GT yaw가10deg 밖. raw log/command_reason_timeline.json 참조.

## GT/TF 경계와 다른 방 진단

좁은 좌석의 마지막 navigation 표본(sim=17.876, 종료 전 0.019s)은 TF 목표 오차 28.51cm, GT 목표 오차 33.31cm, TF–GT 거리 7.48cm다. Nav2 status4와 GT30cm 초과를 별도 기록한다. 이 비동기 표본만으로 localization의 원인을 확정하지 않는다.

다른 방의 최종 status는 5, timeout=True, error=None다. static preflight의 경로 존재가 runtime depth/memory costmap과 BT 경로 처리를 인증하지는 않는다.

- `Failed to transform from base_link to `: 133회. 최초: `[bt_navigator-20] [ERROR] [1791908938.034733522] [transformPoseInTargetFrame]: Failed to transform from base_link to `
- `Failed to lookup current robot pose`: 133회. 최초: `[bt_navigator-20] [WARN] [1791908938.034790137] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to lookup current robot pose for bt_0_1_0_0_0_0__TruncatePathLocal`
- `Failed to get result for compute_path_to_pose in node halt`: 2회. 최초: `[bt_navigator-20] [ERROR] [1791908937.178201105] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to get result for compute_path_to_pose in node halt!`
- `Lattice plan rejected:`: 3회. 최초: `[planner_server-18] [WARN] [1791908889.168163156] [planner_server]: Lattice plan rejected: no_valid_lattice_path, attempts=3 elapsed=69.002 budget=80.000`
- `Goal was canceled`: 13회. 최초: `[controller_server-17] [INFO] [1791908820.166002375] [controller_server]: Goal was canceled. Stopping the robot.`
- `Optimizer reset`: 9회. 최초: `[controller_server-17] [INFO] [1791908765.242202149] [controller_server]: Optimizer reset`
- `Control loop missed its desired rate`: 1회. 최초: `[controller_server-17] [WARN] [1791909678.479227897] [controller_server]: Control loop missed its desired rate of 20.0000Hz`

이 로그는 관측한 오류 연쇄이며 정체의 단일 원인 증명이 아니다. shadow 가드 STOP과 실제 actuator 차단을 구분해 command_reason_timeline.json/commands.csv/bag를 확인한다. 정체 구간의 낮은 jerk를 성능 개선으로 해석하지 않는다.

계측 runner SHA, 공통 snapshot/source 118개 파일 SHA 및 모든8개 bag의 SQLite quick_check/message count를 확인했다. `boundary_evidence.json`에 검증과 표본을 보존했다.

## 공통 사전 검증 변경

이번 공통 입력은 최초5cm raster에서 다른 방 M1 경로를 찾지 못한 뒤 같은 물리 scene을2.5cm raster로 만들고, grid_resolution metadata만2.5cm인 대응 lattice(primitive metric poses/physical radius.5 불변)를 사용했다. 모든8개 지도/global costmap에 동일 적용하며 local costmap 기존resolution은 유지한다. shared planner는 탐색 예산80초/heading32개, holonomic_direct_path/require_exact_goal=true, lattice_collision_padding=.05이다. 물리 footprint/padding.02/internal collision을 축소하지 않는다. canonical physics step .001s를 유지한다. goals.locked.json과 common snapshot에서 원인/변경/검증을 확인한다. 별도 heading 진단을 최종 goal로 혼합하지 않는다. seat43은 preflight1.28m 거의 고정heading 경로이므로 multi-turn이라고 부르지 않는다.

## MPPI 설정과 안전 우회 범위

2026-10-13 최종 개선/완화 MPPI 1.1.20 빌드 및 YAML을 재사용하며 이번 controller 맞춤 튜닝은 없다. goal rotation domain10cm/해제13cm, sampling std .035/.025/.07, gamma .0003, PathAngle 비활성, PathAlign orientation 사용/offset2, time_steps80를 유지했다. 실제 controller dump 모든 FollowPath 값과 .30m/10deg 및 frequency20을 확인한다. 원래 .15/.10/.14 축별 보수 박스를 복원하지 않았다.

결합 영역은 `|vx|+|vy|+.4769|wz| <= .0635*5`이다. twist 전체 축을 동일 비율로 축소한다. sampled control을 rollout/collision 평가 전에, selected sequence를 update 및 Savitzky-Golay 뒤에 제한한다. 최종 wheel projector도 같은 IK를 적용한다. 순수 직진 상한 .3175m/s다. wheel command와 joint max는 별도 보고한다. controller 내부 footprint/feasibility/CostCritic을 유지하고 geometry 축소는 없다. Humble MPPI는 ax_max/ay_max/az_max를 읽지 않으므로 native acceleration 제한을 주장하지 않는다.

실제 chain: `/nav2/cmd_vel → coupled wheel projector → /cmd_vel`. 외부 pan alignment/머리 depth freshness/CollisionMonitor stop-slow/serial voxel-freshness 최종 출력은 `/shadow/cmd_vel`이다. 계속 실행되는 이 가드의 reason/STOP/PASS는 shadow 관측이며 실제 개입으로 해석하지 않는다. .5sim초 controller command-loss watchdog은 유지한다. progress allowance1500sim 및 failure_tolerance=-1은 hard1200sim/1800wall watchdog이 한정한다. depth/memory costmap과 internal collision은 유지한다. Gazebo 외부 우회 프로파일은 물리 로봇용 설정이 아니다.

## 계측과 불명값

GT pose는 world-aligned 좌표 그대로이며 옛 spawn-relative 변환을 제거했다. GT twist body velocity를20Hz sim time으로 보간하고 centered5-point moving average, 양끝 .25s 제거, 두 번 미분하여 jerk RMS/p95를 계산했다. acceleration RMS/p95는 JSON/CSV에 있다. 단위는 선형 m/s²/m/s³ 및 각 rad/s²/rad/s³이다. body velocity 변화 지표이며 물리 승차감/가속도 측정이 아니다. 공통 정지는 `|v_xy|<.01 AND |wz|<.02rad/s`이다. 재출발 및 yaw/vy 반전(deadband .02/.01)은 기술 지표다. 정체/실패로 jerk가 낮아져도 개선으로 판정하지 않는다.

Contact system과 모든 collision을 가진6개 robot link sensor/ROS bridge를 추가했다. 물리 collision/pose/joint geometry 불변을 확인했다. 모든 contact를 contacts.jsonl과 bag에 남긴다. JSONL streaming은 긴1200초 실행의 무한 RAM 누적을 피하는 계측 변경이며 샘플을 버리지 않는다. ground/self는 환경 접촉과 구분하고 이름 미확인 또는 ground heartbeat 없으면 null+이유다. packet 수는 접촉 횟수 자체가 아니므로 unique sim stamp에서50ms 이상 끊긴 구간을 episode로 계산한다. depth 배열이 제공되지 않으면 penetration depth는 null이다. minimum true footprint clearance는 별도 계측하지 않아 null이다. 비접촉 관측은 물리 안전 증명이 아니다.

computeVelocityCommands는 steady wall 시간이며 timing-file I/O는 측정 구간 밖이다. 활성 navigation 구간 median/p95/p99/exception/50ms 초과를 집계한다. CPU는 controller_server Linux proc delta로 one-core %, RSS는p95 MiB이다. command period와 RTF는sim/wall을 구분한다. ARM64 a16-fedora와 다른 x86 호스트의 absolute compute/CPU 순위를 계산하지 않는다. 병렬 다른 세션 부하, rendering 및 contact streaming 부하는 해석 한계다.

controller_frequency20/model_dt.05는 설정값이다. 실제 호출 및 command 간격은 sim/wall 두 시계로 보고하며, 낮은 RTF에서 설정20Hz를20Hz sim cadence로 해석하지 않는다. timeout/정체 결과의 오차는 성공 도착 정확도 점수가 아니다.

## 검증과 원시 증거

재사용한 완화 optimizer16개/smoke3개는 이전 검증에서 통과했다. upstream critic float assertion1개는 원본 시스템 라이브러리에서도 재현된 기존 한계이며 전체 suite 통과라고 쓰지 않는다. 이번에는 source/controller binary를 바꾸지 않았으므로 동일 build를 재사용하고 parameter/scene/lifecycle/loaded-library SHA를 실제 runtime에서 검증했다. prepare-only 검증과 캠페인 goal은 분리한다. run마다 초기 truth와 actual30cm/10deg,20Hz, modeldt 및 wheel cap을 확인한 뒤 goal을 보낸다. 다른 목표를 추가하거나 반복하지 않는다.

- `REPORT.md`, `results.json/csv`: 최종8개 목표 관측. 실패/timeout 포함.
- `final_validation/*`: actual config/dumps, result.json, raw bag/launch.log, GT velocity/commands/joints/resources/compute, contacts.jsonl, reason timeline, plans, scene metadata.
- `final_overrides.yaml`, `execute.sh`, `run_facility.py`, `REPRODUCE.md`: 설정/실제 runner/재현 명령.
- `common_snapshot`, `common_snapshot_manifest.json`, `provenance.json`: locked goals/canonical worlds/matching maps/환경 및 immutable hashes.
- `render_startup_preflight`: READY 전 navigation goal을 보내지 않은 actual parameter/센서 준비 검증.
- `remaining_owned_processes.json`: 종료 후 소유 domain/partition 정리 확인. 다른 세션 프로세스는 보존한다.

기존 dirty main과 이전 실험 증거를 보존하고 push하지 않았다. 다른 채팅으로 메시지를 보내지 않았다.

결과 루트: `/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a`
