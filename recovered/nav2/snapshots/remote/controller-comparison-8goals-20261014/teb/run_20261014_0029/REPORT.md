# TEB — 18F 전체 시설 8목표, 각1회 — 2026-10-14

Nav2 도착 7/8, 도착 중 GT30cm/10° 범위 관측 7/8이다. 모든 실패/timeout/인프라 실패도 목표별8행에 보존했다. 목표별 맞춤 튜닝, 반복 캠페인, 통합 점수나 합격기준은 적용하지 않았다.

이번 입력은 DWB 세션의 공통 사전검증 후 READY로 봉인된 goals.locked.json 및 목표별18F full-facility canonical 월드/일치 지도다. 초기 map/world pose[-1.865,-4.705,1.5708]이며 GT도 world 정렬 좌표로 기록했다. 옛 study-cafe 월드는 사용하지 않았다. 목적지 의자만 해당 목표의 canonical 월드에서 제거했고 다른 모델 보존·mesh 로딩을 실제 Gazebo scene/info와 server log로 확인했다. 목표가 좌석에서 수정되었거나 경로 생성이 불가능하다는 사전검증은 locked_snapshot.json에 남긴 공통 근거 그대로다. 목표 이름만으로 협소/다중회전을 주장하지 않는다.

공통 사전검증에서는 목표 좌표를 바꾸지 않고 global map/costmap 해상도2.5cm, planner budget80초, heading bins32를 고정했다. local costmap 해상도와 padded CAD footprint는 유지했다. 모든204개 공통 입력hash와 목표별 world/map-image hash를 재확인했다. 세 텍스처 파일의 부모호스트 경로는 pinned source의 해당 이미지로 연결해 byte hash를 남겼다. Fortress가 비ASCII mesh의 file:// 경로를 읽지 못한 시작 시도는 목표 발행 전에 중단했고, 같은 asset을 절대 filesystem 경로로 연결한 뒤 재검증했다. 이 인프라 시도는 주행 반복에 포함되지 않으며 infrastructure_uri_startup/에 보존했다. 공통 canonical 파일은 수정하지 않았다.

## 목표별 결과

| 목표 | Nav2 상태 | sim s | wall s | GT XY cm | GT yaw ° | GT범위 관측 | RTF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| straight | arrived | 3.394 | 8.300 | 21.711 | 0.685 | True | 0.409 |
| turn_arrival | arrived | 9.643 | 23.200 | 7.599 | 6.842 | True | 0.416 |
| chair_gap | arrived | 24.817 | 59.900 | 23.521 | 1.261 | True | 0.414 |
| far_heading | arrived | 25.016 | 60.300 | 2.141 | 1.253 | True | 0.415 |
| distant | arrived | 71.170 | 167.000 | 18.297 | 7.360 | True | 0.426 |
| lateral | arrived | 1.830 | 4.400 | 16.754 | 1.559 | True | 0.416 |
| narrow_seat | arrived | 5.546 | 13.600 | 10.974 | 8.392 | True | 0.408 |
| other_room | aborted | 358.175 | 869.846 | 1335.147 | 6.912 | False | 0.412 |

GT오차는 진단값이다. 실패 행은 종료 시 잔여 거리/방향이며 도착 정확도가 아니다. Nav2 stateful SimpleGoalChecker의 실제30cm/10° 파라미터를 런타임에서 조회/검증했다. Nav2 도착과 GT 허용범위 관측은 서로 다른 항목이다. Unknown 항목은 null이며 error/주석에 이유를 보존했다.

이 TEB port의 XY 내부 허용거리는 computeVelocityCommands에서 실제 GoalChecker.getTolerances() 값으로 갱신된다. 별도 FollowPath.xy_goal_tolerance/yaw_goal_tolerance ROS 파라미터는 선언하지 않으므로 해당 조회값은 null이다. Yaw 도착 판단은 Nav2 checker의10° 값으로 수행한다. source_evidence/의 코드와 주석에 근거를 남겼으며 미선언 YAML 키를 실제 로딩 근거로 사용하지 않았다.

파라미터 계측 제약: 원래8회 TEB 일괄조회는 미선언 키 때문에 빈 응답이었고 raw result의 verified_teb_parameters={}를 보존했다. goal checker, planner, footprint/costmap, wheel odometry 및 limiter의 당시 조회는 정상이다. 캠페인 종료 후 Gazebo·activation·주행 목표 없이 동일 controller binary/YAML을 configure하고 개별조회로 TEB 파라미터를 확인했다. 모든8회의 명시 알고리즘 설정과 로딩된 binary hash가 이 configuration replay와 일치한다. Replay는 당시 내부 상태를 소급 조회한 것으로 취급하지 않는다. artifact_checks에는 원래 runtime query 부재8건과 장거리 contact 부재1건을 확인되지 않은 항목으로 표시했다.

## 고정 TEB 및 명령 경로

| 목표 | lattice 실패 시도 | body-heading grid 경로 | planner handle abort | FollowPath client 취소 |
| --- | --- | --- | --- | --- |
| straight | 0 | 0 | 0 | 0 |
| turn_arrival | 0 | 0 | 0 | 0 |
| chair_gap | 0 | 0 | 0 | 0 |
| far_heading | 0 | 0 | 0 | 0 |
| distant | 0 | 0 | 0 | 0 |
| lateral | 0 | 0 | 0 | 0 |
| narrow_seat | 0 | 0 | 0 | 0 |
| other_room | 30 | 32 | 10 | 28 |

Nav2 로그 사건 수이며 실패 원인을 단정하는 지표가 아니다. 내부 FollowPath 취소/재계획은 새로운 NavigateToPose 목표 발행과 구분한다. 최종 watchdog 취소도 로그 사건 수에 포함될 수 있다.

이전 최종 Gazebo완화 프로파일을 유지했다: singleband, homotopy/multithreading false,10Hz, inner/outer5/4, dt_ref.1/hysteresis.02, min_obstacle_dist.05, lookahead3m. 기존 TEB padded polygon 및 costmap footprint/padding과 내부 feasibility/include_costmap_obstacles를 유지했고 추가 축소하지 않았다. max_vel_x/back/y=.3175m/s, max_vel_theta=.665758rad/s, acc_lim_x/y=.5m/s², acc_lim_theta=.8rad/s²다.

실제 전달은 Nav2 → twist전체 uniform wheel-envelope 축소 → convex wheel slew limiter → cmd_vel → Gazebo guard다. r=.0635, wheelbase=.35, separation=.6038, 물리 wheel cap5rad/s, 수치 output cap4.99, wheel slew8rad/s²,20Hz output을 유지했다. 기존 축별 보수 박스는 되살리지 않았다. Gazebo의 독립 body축 가속 제한은 이전 단계와 같이 제거했고 geometry는 바꾸지 않았다. 외부 pan/fresh-depth, Collision Monitor, voxel guard의 출력만 ignored/cmd_vel로 우회하며 counterfactual 진단 및 costmap 관측은 계속한다. 물리 로봇 설정이 아니다.

Progress allowance1200sim초, failure_tolerance=-1, BT retries10000, 목표 hard watchdog1200sim/1800wall초다.240sim초는 소프트 표식이다. Headless server/EGL rendering이며 GUI는 열지 않았다. RTFtarget1, physics step은 canonical 월드 값을 그대로 사용했다. config/wheel_odometry.yaml은 이번 공통 fixture의 센서/odometry 캘리브레이션을 사용했고 로딩값을 기록했다. 이전5goal환경 대비 변경과 원격 호스트 overlay/NVIDIA환경 복구는 비교 조건 차이다.

공통 wheel odometry는 velocity_window_s=.125 및 yaw_scale=1.047로 실제 로딩됐다. SDF frame chain으로 계산한 nominal LiDAR 높이는.30m, 공통 sensor TF 높이는.26m이며4cm 차이가 있다. 두 공통 입력을 유지했다. GT 최종오차와 종료 속도는 action 종료 후2sim초 관찰한 값이며, 시간·jerk·접촉 집계는 기록된 action 주행 창에 한정한다.

## 연산·실제 주기·자원

| 목표 | median ms | p95 ms | p99 ms | 예외 | 100ms 초과 | control sim주기 평균 s | control wall주기 평균 s | CPU %/core | RSS p95 MiB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| straight | 3.378 | 4.413 | 4.608 | 0 | 0 | 0.041 | 0.100 | 8.681 | 58.879 |
| turn_arrival | 5.218 | 11.734 | 13.330 | 0 | 0 | 0.042 | 0.100 | 11.001 | 59.906 |
| chair_gap | 10.681 | 12.635 | 13.767 | 0 | 0 | 0.041 | 0.100 | 15.102 | 60.625 |
| far_heading | 9.618 | 23.347 | 25.727 | 0 | 0 | 0.042 | 0.100 | 15.992 | 61.858 |
| distant | 13.148 | 16.050 | 17.498 | 0 | 0 | 0.043 | 0.100 | 18.216 | 63.402 |
| lateral | 2.991 | 3.494 | 4.052 | 0 | 0 | 0.042 | 0.100 | 8.437 | 59.185 |
| narrow_seat | 4.730 | 6.656 | 7.056 | 0 | 0 | 0.041 | 0.100 | 10.074 | 59.859 |
| other_room | 16.928 | 94.568 | 98.778 | 0 | 4 | 0.476 | 1.157 | 7.319 | 71.953 |

computeVelocityCommands 내부 steady-wall 시간이며 성공과 예외 모두 포함하고 파일 I/O는 측정 밖이다. 실제 주기는 timing CSV의 모든 호출 시작 시각에서 계산하며 명령 발행 주기도 별도 보존했다. CPU는 controller_server의 one-core100% 기준, RSS는 해당 프로세스 표본이다. localhost domain227/UUID partition으로 격리했지만 변동하는 호스트 부하는 포함된다. local ARM과 remote x86의 절대 compute/CPU 순위 비교는 금지한다.

## GT jerk·정지·반전

| 목표 | linear jerk RMS/p95 | angular jerk RMS/p95 | 정지 sim s | motion stop 수 | forward/lateral/yaw 반전 |
| --- | --- | --- | --- | --- | --- |
| straight | 1.352/2.347 | 1.843/3.863 | 0.100 | 0 | 0/5/5 |
| turn_arrival | 0.783/1.657 | 0.908/1.973 | 0.150 | 0 | 0/5/1 |
| chair_gap | 0.677/1.319 | 1.058/2.251 | 0.200 | 0 | 2/9/14 |
| far_heading | 0.874/1.670 | 1.195/2.533 | 0.150 | 0 | 5/22/14 |
| distant | 0.603/1.190 | 0.889/1.904 | 0.200 | 0 | 8/19/23 |
| lateral | 1.234/2.138 | 1.102/2.218 | 0.150 | 0 | 2/0/1 |
| narrow_seat | 0.853/1.594 | 1.147/2.120 | 0.150 | 0 | 2/0/3 |
| other_room | 0.247/0.369 | 0.191/0.140 | 335.650 | 7 | 12/1/7 |

공통 정지는 hypot(vx,vy)<.01m/s AND abs(wz)<.02rad/s이다.20Hz보간, centered5point smoothing, 양끝.25초 제외 뒤 jerk를 계산했다. raw command와 GT stop은 별도 기록했다. 실패/정체의 낮은 jerk를 유용한 주행의 부드러움 개선으로 해석하지 않는다.

## 물리 접촉·wheel 및 입력 검증

| 목표 | cmd wheel max rad/s | joint wheel max rad/s | joint5 초과 표본 | 물리 contact episodes | 종료 GT속도 |
| --- | --- | --- | --- | --- | --- |
| straight | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| turn_arrival | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| chair_gap | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| far_heading | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| distant | 4.990 | 4.990 | 0 | None | [0.0, 0.0, 0.0] |
| lateral | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| narrow_seat | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |
| other_room | 4.990 | 4.990 | 0 | 0 | [0.0, 0.0, 0.0] |

실측 접촉은 full rosbag2의 collision name/ID/depth 메시지에서 floor 및 self-contact를 제외한 collision-pair episode(.25sim초 간격)다. floor 접촉을 positive control로 확인하며 attribution이 불명확하면 null+이유다. 접촉 센서는 canonical robot의 모든collision을 포함하는지 확인했다. asset_materialization.json, world.canonical.sdf, scene.raw.pbtxt/scene.json/scene_validation.json, contact_coverage.json, raw_contact_summary.json에 입력/실측 근거를 남겼다.

계측 제약 — distant: Incomplete navigation contact coverage: recorder stopped after live SQLite read lock; no navigation retry. 기록된 contact 창[8.371, 37.768], 실제 주행 창[8.371, 79.54100000000001]sim초다. Live observer의 GT/명령/joint 기록 및 전체 compute CSV는 별도로 유지됐지만 이 자료로 누락된 full-bag 접촉을0으로 추정하지 않았다. SQLite metadata는 recorder 종료 후 offline reindex로 복원했고 원본DB byte hash를 확인했다. 목표 재실행은 하지 않았다.

## 증거와 재현

results.json/csv, infrastructure_results.json/csv, all_runs.json, final_controller.yaml, tested_runtime_parameters.yaml, locked_snapshot.json, common 입력hash, environment.json, 환경복구 기록, 소스/호환·진단patch 및 build archive, raw final8/*의 bag/log/CSV/JSON을 보존했다. REPRO.md의 명령으로 새 출력 그룹에서 재현한다. 각 목표 한 회라 확률적 성공률을 추정하지 않는다. 기존/새 TEB는 모두-O3 -DNDEBUG였으므로 Release 성능 개선으로 주장하지 않는다. 전용 source는 clean pinned 상태이며 다른 owner 프로세스와 dirty checkout을 보존했고 git push하지 않았다. cleanup.json은 이번 소유partition/PID만 검사한다.

![목표별 GT 경로](/home/donglee/workspace/cleany/controller-comparison-8goals-20261014/teb/run_20261014_0029/trajectories.png)
