# TEB 보수 공통 제한 최적화 — 2026-10-13

동결한 설정으로 공통 5개 목표를 각각 한 번 주행했다. Nav2 도착 2/5, 도착 중 GT 15cm/10도 범위 관측 1/5다. 합격기준이나 통합점수를 정의하지 않았다. 중단·timeout 행의 오차는 종료 시 남은 거리와 방향 차이며 도착 정확도가 아니다.

최종 native 변경은 min_obstacle_dist: 0 → 0.05m 한 가지다. 기존 expanded TEB polygon, costmap footprint/padding, 내부 feasibility, pan/2D/3D 안전 가드, planner, map/world, 속도 박스, 15cm/10도 goal checker를 유지했다. 최종 후보수 5, inner/outer 반복 5/4, dt_ref=.3, 제어 20Hz, RTF 목표1, physics dt=.002, 목표별240sim/360wall초다.

기존 정식 기준선은 Nav2 도착 1/5였고 compute p95 58.9–65.0ms였다. 이번 단일 대표 pilot에서 5cm 간격 설정이 직선 도착과 reset 제거를 보였지만, 다른 실행의 변동과 최종 캠페인을 함께 봐야 한다. 반복 캠페인이나 확률적 성공률 추정은 하지 않았다.

## 원래 정식 기준선

| 목표 | Nav2 상태 | sim s | GT XY cm | GT yaw ° | compute p95 ms |
| --- | --- | --- | --- | --- | --- |
| straight | aborted | 42.566 | 66.325 | 22.147 | 58.928 |
| turn_arrival | arrived | 125.044 | 14.363 | 5.437 | 64.690 |
| chair_gap | aborted | 130.996 | 276.854 | 27.177 | 61.040 |
| far_heading | aborted | 26.118 | 524.752 | 133.013 | 65.045 |
| distant | aborted | 191.178 | 565.591 | 3.175 | 60.703 |

동일 목표의 아래 동결 결과와 비교한다. 실패 잔여 오차와 성공 도착 오차는 같은 의미가 아니며, 호스트 부하와 AMCL/Gazebo 확률성이 다르므로 일반 성공률을 추정하지 않는다.

## 최종 정확도와 시간

| 목표 | Nav2 상태 | sim s | wall s | GT XY cm | GT yaw ° | GT 범위 관측 | 실제 RTF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight | aborted | 115.666 | 119.801 | 47.800 | 13.729 | False | 0.965 |
| turn_arrival | arrived | 86.972 | 89.400 | 15.683 | 4.528 | False | 0.973 |
| chair_gap | aborted | 155.458 | 160.900 | 142.971 | 2.869 | False | 0.966 |
| far_heading | arrived | 85.056 | 87.900 | 10.652 | 5.645 | True | 0.968 |
| distant | timeout | 240.000 | 247.201 | 401.935 | 14.453 | False | 0.971 |

Nav2 상태와 GT 오차는 독립 항목이다. final_nav2_map_xy는 AMCL/TF 좌표이며 GT와 다를 수 있다. stateful SimpleGoalChecker가 XY를 latch할 수 있으므로 Nav2 도착만으로 GT 허용오차 충족을 주장하지 않는다. 시작 map pose [0,0,0]은 기존 시뮬레이션 spawn 좌표계로 변환하며 world 원점과 다르다.

## 연산량과 주기

| 목표 | median ms | p95 ms | p99 ms | 예외 수 | budget ms | budget 초과 | CPU %/core | RSS p95 MiB | Nav2 sim주기 평균 s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 8.952 | 24.968 | 33.499 | 0 | 50.000 | 6 | 27.383 | 54.586 | 0.051 |
| turn_arrival | 25.308 | 33.228 | 39.782 | 0 | 50.000 | 1 | 52.554 | 55.799 | 0.049 |
| chair_gap | 25.142 | 33.774 | 47.030 | 0 | 50.000 | 27 | 54.411 | 56.111 | 0.050 |
| far_heading | 21.687 | 29.988 | 39.325 | 0 | 50.000 | 7 | 47.112 | 55.328 | 0.049 |
| distant | 21.239 | 31.038 | 45.319 | 1 | 50.000 | 36 | 47.941 | 57.199 | 0.050 |

computeVelocityCommands 내부 steady wall 측정이며 정상 반환과 예외를 모두 포함한다. 성공/예외별 통계와 max도 JSON에 있다. 파일 기록은 측정 밖이다. CPU는 controller_server 프로세스의 one-core 100% 기준이고 RSS는 그 프로세스 표본이다. 호스트는 기준선과 같은 Ryzen 5 9600X x86_64; 다른 CPU 호스트와 절대값 순위를 비교하지 않는다. 다른 소유자의 기존 Gazebo 프로세스를 관측했고 종료하지 않았다. 변동하는 병렬 호스트 부하는 측정에 포함된다.

## GT 부드러움

| 목표 | linear jerk RMS/p95 | angular jerk RMS/p95 | 병진 정지 s | motion stop 수 | forward/lateral/yaw 반전 |
| --- | --- | --- | --- | --- | --- |
| straight | 1.851/5.313 | 1.787/4.883 | 102.850 | 51 | 0/1/3 |
| turn_arrival | 3.991/6.636 | 3.174/5.570 | 51.200 | 162 | 1/2/33 |
| chair_gap | 2.196/5.411 | 2.096/4.933 | 124.150 | 119 | 1/4/19 |
| far_heading | 3.239/6.794 | 2.900/5.633 | 42.750 | 109 | 1/2/45 |
| distant | 3.560/6.605 | 2.208/4.824 | 171.050 | 368 | 3/4/120 |

GT 속도를 sim time 20Hz에 보간하고 centered 5-point smoothing 뒤 acceleration/jerk를 계산하며 양끝 .25초를 제외했다. stationary는 병진 .01m/s 미만이고 제자리 회전을 포함한다. 장시간 정지한 실패의 낮은 전체 jerk를 좋은 주행으로 해석하지 않는다. raw command stop 수와 GT motion stop 수는 정의가 다르며 둘 다 JSON에 있다.

## 제한·접촉 검증

| 목표 | 최종 cmd wheel max | joint wheel max | 종료 GT 속도 | 실측 contact episodes |
| --- | --- | --- | --- | --- |
| straight | 4.967 | 4.967 | [0.0, 0.0, 0.0] | None |
| turn_arrival | 4.985 | 4.985 | [0.0, 0.0, 0.0] | None |
| chair_gap | 4.988 | 4.988 | [0.0, 0.0, 0.0] | None |
| far_heading | 4.988 | 4.988 | [0.0, 0.0, 0.0] | None |
| distant | 4.988 | 4.988 | [0.0, 0.0, 0.0] | None |

보수 단계에는 physics contact sensor가 없으므로 actual_contact_count=null이다. 실제 충돌 0회로 추론하지 않는다. command_stage_audit는 모든 단계의 속도 박스 초과 수, 마지막 명령과 수신 시각을 포함한다. 마지막 최종 명령과 실제 속도 정지를 별도로 확인했다.

## 기록된 경로의 정적 투영 간격

| 목표 | 최소 XY 투영 간격 m | 투영 overlap 표본 | 지원 static collision 수 |
| --- | --- | --- | --- |
| straight | 0.208 | 0 | 175 |
| turn_arrival | 0.207 | 0 | 175 |
| chair_gap | 0.168 | 0 | 175 |
| far_heading | 0.210 | 0 | 175 |
| distant | 0.171 | 0 | 175 |

CAD physical footprint와 static collision의 XY 투영을 기록된 GT 표본에서 비교했다. full-height prism과 mesh convex hull은 서로 다른 높이/틈을 채울 수 있고 inter-sample swept contact를 검증하지 않는다. 이 간격은 물리 Contact 센서 결과와 별도 지표다. geometry_summary.json에 모델·지원 범위·한계·입력 hash를 보존했다.

## 대표 구간 튜닝과 원인

| 실험 | 목표 | 상태 | sim s | GT XY cm | compute p95 ms | reset 수 |
| --- | --- | --- | --- | --- | --- | --- |
| 02_release_diagnostic | straight | aborted | 43.868 | 131.229 | 64.329 | 155 |
| 03_positive_clearance | straight | arrived | 22.038 | 2.224 | 25.696 | 0 |
| 04_classes2 | straight | aborted | 115.738 | 32.967 | 27.116 | 0 |
| 05_inner3 | straight | aborted | 116.600 | 35.853 | 22.577 | 0 |
| 06_single_band | straight | aborted | 136.070 | 45.097 | 7.946 | 0 |
| 07_dt4 | straight | aborted | 20.054 | 48.196 | 20.585 | 30 |
| 01_baseline_resume_0512 | straight | aborted | 103.838 | 93.917 | 50.678 | 437 |

초기 min_obstacle_dist=0는 AddEdgesObstacles의 force-inclusion/cutoff 거리도 0으로 만든다. 양수 거리 장애물이 association에서 제외되어 polygon이 lethal cell을 향해 갈 수 있다. 진단 pilot의 거절은 모두 Trajectory Hits Obstacle였다. min_obstacle_dist=.05만 추가한 pilot은 22.038sim초 도착, GT 2.224cm/6.562°, reset 0, p95 25.696ms였다. 후보수2/inner3/single-band는 연산량을 낮췄지만 명령 방향 변화, TURNING_PAN/WAIT_FRESH_DEPTH, progress 실패가 남아 채택하지 않았다. dt_ref=.4는 lethal footprint 거절을 다시 만들었다.

이후 동결 검증에서 같은 설정의 재현성과 구조적 명령-gate 제약을 평가했다. 원시 command_transitions.json과 pan_gate_reason_duration_sim_s, pan_requested_bearing_step_rad, body_guard_stop_reason_duration_sim_s로 각 원인을 추적한다. 3D guard의 늦은 Command timeout을 최초 controller 실패나 장애물 대기로 오인하지 않는다. 더 강한 공통 안전 계층 완화는 본 단계에 섞지 않고 별도 승인된 완화 단계로 진행한다.

## 호환성·재현성·한계

Pinned Cleany b3ca816ba420452ee7960837bc16ec74148e07b0, Nav2 Humble1.1.20, Fortress6.18.0, TEB humble-devel630a22e88dc9fd45be726a762edbb5b776bef231이다. latest-TF lookup 및 null converter cleanup 호환 패치를 유지했고 dedicated build/Gazebo에서 검증했다. diagnostics.patch는 거절 이유와 실패 pose/index만 추가한다. 기존 CMake cache의 build type은 비었지만 CMakeLists가 Release를 강제해 기존/새 빌드 모두 -O3 -DNDEBUG였으므로 Release 성능 개선이라고 주장하지 않는다.

목표를 발행하기 전 인프라 실패 1개는 infrastructure_results.json/csv에 별도 보존했다. startup retry는 같은 목표의 중복 주행이 아니다. 모든 주행 실패/timeout은 results에 포함했다. 각 목표 한 회이며 Gazebo/AMCL의 같은 random seed를 보장하지 않는다. 단일 pilot의 인과 효과를 확률적 일반 성능 개선으로 확대하지 않는다.

results.json/csv, pilot_results.json/csv, all_runs.json, infrastructure_results.json/csv, final_controller.yaml, frozen_overrides.json, 원시 final/*/bag 및 log/CSV/JSON, REPRO.md를 함께 보존한다. 기존 사용자 worktree/다른 controller 파일을 수정하지 않았고 push하지 않았다. cleanup.json은 이 실험의 정확한 partition/PID만 검사한다.
