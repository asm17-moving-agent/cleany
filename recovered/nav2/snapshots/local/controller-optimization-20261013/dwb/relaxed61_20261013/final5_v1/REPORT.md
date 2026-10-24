# DWB 완화 단계 최종 결과 — 2026-10-13

고정 설정으로 각 목표를 1회 실행했다. 완료 기록 5/5, Nav2 status4 5/5. Nav2 도착은 추정 pose에 대한 결과이며 GT15cm/10도 충족이나 collision-free를 뜻하지 않는다. source local commit `44c083abe2ebeb0077879e4419660b15fe4dd91b`; navigation tests194개 통과. push 및 물리 로봇 실험 없음.

보수적 결과는 상위 `conservative_REPORT.md`, `final61_20261013_v1`에 보존했다. 원래 baseline은 common command box 위반이 있었으므로 역사적 참고치다. 완화 단계는 속도 envelope와 외부 safety delivery 조건이 달라 보수적 설정과 동일 제약 A/B 비교가 아니다. 각1회 관측이고 AMCL/scheduler/동시 부하가 달라 통계적 성공률이나 인과 효과로 해석하지 않는다.
최종 soft240초 초과: []; hard timeout: []; compute exception 총0, >50ms 총0.

| 목표 | Nav2 status | baseline / 보수적 / 완화 sim초 | GT XY cm | GT yaw ° | map XY cm | GT-map cm |
|---|---|---|---|---|---|---|
| straight | 4 | 29.43 / 19.91 / 9.13 | 13.14 | 0.00 | 15.07 | 5.59 |
| turn_arrival | 4 | 38.92 / 50.63 / 18.49 | 12.32 | 7.46 | 14.19 | 2.83 |
| chair_gap | 4 | 51.30 / 64.35 / 20.83 | 16.36 | 0.00 | 13.93 | 2.90 |
| far_heading | 4 | 57.93 / 57.77 / 25.47 | 9.81 | 4.30 | 12.74 | 5.06 |
| distant | 4 | 168.59 / 142.82 / 45.69 | 11.67 | 0.00 | 14.19 | 4.96 |


표의 terminal 오차는 action 후2sim초 settled snapshot이다. GT와 map TF는 비동기 샘플이며 goal-checker가 사용한 pose의 원자적 캡처가 아니다. straight의 map snapshot15.07cm는 이 관측 경계를 포함한다. action/settled source timestamps와 errors를 JSON에 보존했다.

목표, GT 초기 [0,0,0], StraightFirstPlanner, world/map/robot assets, 물리 collision geometry와 실제 footprint를 유지했다. r=.0635m, base=.35m, separation=.6038m, k=.4769m이다. 독립 common box .15/.10/.14를 제거하고 |wheel|≤5rad/s 결합 제약을 적용했다. nominal straight5rad/s는 .3175m/s이며 수치/feedback 여유를 위해 command4.99rad/s(.316865m/s)를 쓴다. 각 rollout에서 vx/vy/yaw를 동일 비율로 축소하고 정상 critic이 그 궤적을 평가한다. joint feedback은 별도 계측한다.

선택 설정:5×5×7 samples, goal-directed radius4.5m, target-bearing ray preference, xy acc/decel±.4m/s², yaw±.6rad/s², control20Hz wall timer, full collision rollout2s/discretization.05s, 기존 critics/padding.02/inflation.8 유지. RotateToGoal score lookahead만1s다. 15cm/10도 goal checker는 동일하다. required progress radius/angle.001, allowance1200s, failure_tolerance=-1; soft240sim초 초과는 기록만 하고 hard1200sim/1800wall초로 제한한다. 새 점수/합산 점수는 없다.

Gazebo 전용 bypass: pan/freshness, Collision Monitor, voxel guard는 계속 실행되지만 `/shadow/cmd_vel`에 출력한다. actual `/cmd_vel`의 sole publisher는 wheel_cap_relay이며 Nav2 raw를 coupled cap4.99로 전달한다. nonfinite/stale5wall초는 zero. controller 내부 collision checks는 유지한다. 기존 Gazebo 전달 guard의 .3m/s box가 pilot01c를 잘라낸 것을 발견해 외부 runtime YAML만 x/y1m/s, yaw2rad/s, timeout5s로 넓혔다. 유효 wheel envelope는 여전히 relay/source에서 제한된다. 실제 guard source나 production safety profile은 수정하지 않았다. 센서/pan nodes는 계속 실행되어 비용도 남아 있다.

| 목표 | compute median/p95/p99 ms | 예외 / >50ms | RTF | cmd dt median/p95 s | CPU % / RSS MiB | command / joint wheel max rad/s |
|---|---|---|---|---|---|---|
| straight | 5.95/11.89/14.14 | 0 / 0 | 0.923 | 0.040/0.060 | 20.5 / 48.6 | 4.990000 / 4.992796 |
| turn_arrival | 5.83/12.10/13.73 | 0 / 0 | 0.924 | 0.040/0.060 | 20.4 / 48.5 | 4.990000 / 4.990000 |
| chair_gap | 6.28/12.71/14.55 | 0 / 0 | 0.926 | 0.040/0.060 | 21.2 / 49.9 | 4.990000 / 4.990000 |
| far_heading | 5.50/12.45/14.56 | 0 / 0 | 0.929 | 0.040/0.060 | 20.0 / 49.1 | 4.990000 / 4.990000 |
| distant | 8.01/14.02/17.83 | 0 / 0 | 0.931 | 0.040/0.060 | 21.6 / 50.0 | 4.990000 / 4.990000 |


compute는 steady clock의 wrapped computeVelocityCommands 구간이며 CSV I/O/lock은 제외한다. CPU는 같은 ARM64 호스트의 controller process deltaCPU/wall, 한 코어 기준이며 동시 부하가 다를 수 있다. ROS2 Humble/Nav2 1.1.20, Fortress6.18. simulation interval과 wall20Hz는 구분한다. 실제 live params, source/library hashes, loaded library `/proc/maps`, backend params와 publishers를 각 run에 저장했다.

| 목표 | linear accel RMS/p95 m/s² | linear jerk RMS/p95 m/s³ | angular jerk RMS/p95 rad/s³ | 정지 sim초 / start / stop | yaw 반전 | 환경 / body-ground 접촉 |
|---|---|---|---|---|---|---|
| straight | 0.119/0.266 | 0.476/1.017 | 0.000/0.000 | 1.25 / 1 / 1 | 0 | 0 / 0 |
| turn_arrival | 0.086/0.254 | 0.260/0.658 | 0.797/1.605 | 4.40 / 1 / 1 | 0 | 0 / 0 |
| chair_gap | 0.084/0.177 | 0.378/0.843 | 0.000/0.000 | 1.40 / 1 / 1 | 0 | 0 / 0 |
| far_heading | 0.061/0.169 | 0.178/0.471 | 0.955/0.473 | 4.50 / 1 / 1 | 0 | 0 / 0 |
| distant | 0.105/0.175 | 0.641/0.694 | 0.000/0.000 | 1.50 / 2 / 2 | 0 | 0 / 0 |


smoothness는 GT twist를20Hz sim-time 보간, centered5point smoothing, gradient2회, 양쪽.25s trim으로 계산한다. translation speed≤.01m/s를 정지로 센다. 정상 heading rotation도 translation 정지에 포함된다. 각 축 반전/가속/angular metric과 raw sample timestamps는 JSON에 있다. 성공/중단 결과를 숨기지 않는다.

환경 접촉은 실제31개 robot collider ContactSystem/sensor에서 얻었다. wheel-floor heartbeat를 확인했고 별도 body–시험 장애물 overlap에서1509개 raw positive contact가 검출돼 경로를 확인했다. 이 positive-control world는 비교5회에서 사용하지 않았다. sensor update_rate20Hz 설정에도 persistent physics contact는.002s timestep 약500Hz로 발행됐다. wheel-ground JSON만20Hz로 줄이고 환경/body 및 raw bag은 줄이지 않았다. DDS loss/짧은 transient 누락 가능성이 있으며0건은 보고된 접촉0건이다. 절대 연속 collision-free나 physical proof가 아니다. GT는 최근 receive sample이라 원자적 접촉 pose가 아니다. ROS bridge depths가 빈 배열이므로 penetration depth=null(이유 기록), continuous minimum true footprint clearance=null(미계측)이다. 보수적 단계에는 contact sensor가 없었다.

대표 탐색: straight9.654s는 .3m/s backend clip이 남은 pilot, 이를 제거한 chair-gap20.80s/GT16.62cm/joint4.99018이었다. full2s RotateToGoal terminal yaw plateau를 진단 중단했고 .5s 후보도 map yaw 약90도에서0명령을 반복해 중단했다. 1s 후보는 turn_arrival19.566s/GT12.04cm/compute exception1로 도착했다. 중단2회는 completed outcome이나 성공률에 넣지 않으며 raw bag/log와 manual_stop_diagnosis.json을 보존했다. invalid numeric contact topics와 remap delivery 오류2회는 infrastructure 진단이다.

RotateToGoal lookahead는 scoreRotation의 평가 pose를 선택하며 collision rollout 길이를 줄이지 않는다. [Nav2 1.1.20 upstream 구현](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)을 확인했다. .5s에서의 경로 방향 penalty 우세는 local PathOrientation full-horizon score와 관측 command를 함께 본 추론이다.

GT arrival 오차는 common localization/15cm goal-checker의 한계가 남는다. map 추정에 대한 stop과 실제GT 중심 도착은 별개이며 odom–GT와 map–GT 차이를 metrics에서 분리한다. goal window 내부 RotateToGoal translation 제한 때문에 controller candidate만으로 checker 이후 GT correction을 계속할 수 없다. common arrival/localization 변경은 하지 않았다.

재현은 `../REPRODUCE.md`, 설정 `../final_patch.yaml`, `../final.yaml`, source `../final.patch` 및 `../source_manifest.json`. 원시 bag/log, events, GT/map/odom snapshots, commands.csv(모든 전달 단계 receive sim/wall stamp), joint feedback, compute CSV, resources와 support snapshots는 각 goal directory에 있다. metadata/geometry 동등성, bag integrity와 소유 process 종료는 별도 verification JSON이다.
