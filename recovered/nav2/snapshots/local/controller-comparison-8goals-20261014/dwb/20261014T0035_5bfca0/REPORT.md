# DWB 공통8 goals — descriptive report

**Raw bag2개 tail 누락 및 실제 joint cap 위반으로 완전한 원자료/제한 조건을 갖춘 캠페인은 아니다.**

완료된 action 측정 8/8회. 알고리즘 점수나 다른 host와의 절대 순위는 만들지 않았다. 18F full facility/CAD robot/headless Gazebo, world/map start [-1.865, -4.705, 1.5708], 목표 좌표 변경0. 각 목표는 새 world에서1회 주행했다. action 상태와 GT 오차는 별도 지표다.

**실제 wheel5rad/s 조건 미충족.** command cap4.99와 별개로 actual joint 최대 5.128498rad/s가 관측됐다. 위반 목표: turn_arrival(5.015522), far_heading(5.006476), distant(5.128498). 원래 고정1회 프로토콜을 지켜 완료된 목표를 다시 실행하거나 margin을 사후 변경하지 않았다. 이 profile을 실제 joint cap을 만족하는 결과로 분류할 수 없다. wheel_cap_violations.json에 위반 sample을 저장했다.

## Raw bag coverage

|goal|bag clock first/last sim s|navigation last sim s|전체 주행 구간 포함|
|---|---|---|---|
|straight|7.277/15.673|13.652|True|
|turn_arrival|7.623/22.751|20.708|True|
|chair_gap|7.955/102.533|445.970|False|
|far_heading|7.469/32.669|30.602|True|
|distant|7.465/367.620|546.497|False|
|lateral|7.677/13.570|11.552|True|
|narrow_seat|7.986/15.901|13.862|True|
|other_room|7.656/122.350|120.158|True|

독립 Probe velocity/command 기록의 전체 주행 시간 구간은 independent_probe_coverage.json에서8/8 확인했다. 이 시간 coverage는 모든 DDS packet의 무손실 수신을 뜻하지 않는다. Raw bag 누락을 Probe로 복원했다고 주장하지 않는다.

## Action 결과와 실제 위치

status4=SUCCEEDED,5=CANCELED,6=ABORTED. 아래 GT는 action 종료 후2sim seconds의 settled snapshot이며, GT는 Gazebo raw world pose(초기 home 값으로 확인)이고 wheel/odom은 start-local이므로 home rotation/translation을 적용한 뒤 비교했다. callback/TF/AMCL은 비동기이므로 goal checker가 평가한 순간과 동일 샘플은 아니다.

|goal|action status|timeout|GT xy cm|GT yaw °|map TF xy cm|action 순간 GT xy cm|sim s|wall s|RTF|
|---|---|---|---|---|---|---|---|---|---|
|straight|4|False|26.40|0.00|29.74|26.40|5.506|24.500|0.225|
|turn_arrival|4|False|27.35|3.72|26.59|27.35|12.566|53.200|0.236|
|chair_gap|5|True|186.16|0.05|186.14|186.16|437.859|1800.103|0.243|
|far_heading|4|False|21.96|2.74|20.05|21.96|22.511|102.500|0.220|
|distant|5|True|610.46|118.10|612.05|610.46|538.396|1800.101|0.299|
|lateral|4|False|29.19|0.00|30.14|29.19|3.397|10.001|0.340|
|narrow_seat|4|False|32.90|0.00|27.96|32.90|5.745|16.801|0.342|
|other_room|6|False|1580.66|0.00|1577.29|1580.66|112.009|324.201|0.345|

GT30cm/10° 범위 밖의 관측: chair_gap, distant, narrow_seat, other_room (해당 없으면 빈 목록). Nav2 goal checker 오차/TF-AMCL-GT 차이는 진단값이며 점수로 합치지 않았다.

## Compute, 실제 주기, resource, wheel

20Hz configured target. timed wrapper는 computeVelocityCommands 내부 steady wall duration을 기록한다. 실패 주기의 짧은 compute 시간이 섞일 수 있어 metrics.json에 successful/exception compute 분포도 따로 넣었다. 수신 raw command의 wall/sim 간격은 별도이고 DDS callback 지연을 포함한다. CPU는 owned controller process의 wall-window CPU/one core, RSS는 해당 process peak다. 같은 로컬 host의 동시 작업/RTF 영향을 포함하며 원격 TEB와 성능 순위를 만들 수 없다.

|goal|compute median/p95/p99 ms|exception/>50ms|raw wall dt median/p95/p99 ms|raw sim dt median ms|controller CPU %|RSS MiB|command wheel max rad/s|joint wheel max rad/s|joint≤5|
|---|---|---|---|---|---|---|---|---|---|
|straight|5.842/11.455/12.302|0/0|49.975/54.132/59.065|11.000|19.46|52.25|4.212334|4.212334|True|
|turn_arrival|4.842/11.519/13.376|1/0|50.023/54.359/63.434|12.000|19.01|52.43|4.990000|5.015522|False|
|chair_gap|2.876/8.120/10.248|34613/0|49.995/53.746/60.983|12.000|15.33|56.64|4.990000|4.990000|True|
|far_heading|6.846/12.296/14.413|6/0|50.018/54.633/60.496|11.000|20.18|53.26|4.990000|5.006476|False|
|distant|3.897/10.040/13.200|32439/0|49.999/56.150/61.733|14.000|15.84|56.42|4.990000|5.128498|False|
|lateral|7.334/11.741/12.860|0/0|50.005/54.509/58.602|17.000|22.26|51.71|3.098013|3.098013|True|
|narrow_seat|5.578/11.536/13.054|0/0|50.025/55.763/58.140|17.000|19.48|52.27|4.990000|4.990253|True|
|other_room|6.270/13.225/15.226|0/0|49.882/56.673/63.933|17.000|8.68|53.13|3.061046|3.061046|True|

## Motion and contact observations

공통 정지 정의는 `|v_xy|<0.01m/s AND |wz|<0.02rad/s`. GT20Hzsim resampling, 5-point centered smoothing, 두 차분, 양끝.25sim seconds trim. 방향 reversal은 longitudinal/lateral.01m/s, yaw.02rad/s deadband를 넘는 부호 변화 수다. 횟수는 sampling/threshold에 의존한다.

|goal|stops/starts|stationary sim s|linear jerk RMS/p95 m/s³|angular jerk RMS/p95 rad/s³|longitudinal/lateral/yaw reversals|environment contact samples/episodes|body-ground samples|
|---|---|---|---|---|---|---|---|
|straight|1/1|0.350|0.780/0.435|0.000/0.000|0/0/0|0/0|0|
|turn_arrival|0/1|0.200|0.135/0.203|0.839/0.473|0/0/0|0/0|0|
|chair_gap|1/1|422.350|0.118/0.000|0.009/0.000|0/0/0|0/0|0|
|far_heading|0/1|0.200|0.065/0.148|0.712/0.473|0/0/0|0/0|0|
|distant|2/2|486.050|0.137/0.058|0.296/0.155|7/5/2|0/0|0|
|lateral|1/1|0.550|1.000/2.369|0.000/0.000|0/0/0|0/0|0|
|narrow_seat|1/1|0.550|0.919/1.239|0.000/0.000|0/0/0|0/0|0|
|other_room|4/4|105.900|0.379/0.072|0.000/0.000|0/0/0|0/0|0|

31개 actual robot collider의 contact sensor를 재사용하고 ContactSystem plugin1개를 관측용으로 추가했다. 물리 timestep.001s, sensor setting20Hz이나 raw wheel-ground stream의 실제 수신률은 contact_analysis.json에 저장했다. floor JSON은20Hz subsample이며 정상 raw bag과 environment/body JSON은 unthrottled다. chair_gap/distant raw bag은 recorder 오류 이후 tail이 없다. 전체 구간 contact 관측은 독립 Probe JSON을 사용하며 wheel-ground는20Hz subsample이다. wheel-ground heartbeat가 있으면 reported0 events를 해석할 수 있지만 DDS loss/missed transient 가능성이 있다. 보고된 contact0은 연속 무충돌 증명이 아니다. penetration depth=null (bridge depths empty), minimum continuous physical clearance=null (계측하지 않음). 실물 검증 없음.

## 공통 입력과 controller 고정 설정

- lock SHA256: `296c9abae047f5ba4f27ced4e2d80bd159565d5b370460fee1ce4d29e58022ac`
- full base330models/82chairs, seat 목표는 해당 chair1개만 제거하여329models/81chairs. 각 actual scene과 map SHA/normalized world geometry는 input_and_shutdown_audit.json에 저장했다.
- common LiDAR-plane map.30m/2.5cm, global inflation.8m/padding.02, 전체 footprint 유지. StraightFirstPlanner max_planning_time80s/body_heading_grid_bins32/common2.5cm lattice metadata. 원래 물리 primitive와 turning radius.5m 유지.
- common tolerance xy.30m/yaw10° 실제로드. 초기 AMCL covariance는 common lock [.25²,.25²,(15°)²]이며, 초기 pose publish 후 no-motion update5회와 각1sim second 대기를 적용했다. 8개 좌표/회전은 common lock 그대로. labels는 static actual planner path 기반이며 narrow_seat를 다중회전 경로라고 부르지 않는다.
- source commit44c083abe2ebeb0077879e4419660b15fe4dd91b, baseline b3ca816ba420452ee7960837bc16ec74148e07b0. 빌드/source/library SHA 및 loaded plugin 경로는 각 provenance.json/loaded_controller_libraries.json. 기존 source 단위 검증194개 통과 기록을 재사용했고 이번에는 source 변경 없이 외부 setup/analysis script syntax와 실제 runtime을 검증했다.
- DWB5x5x7 samples, goal-directed radius4.5m/target-translation preference, full collision rollout2s/.05m, RotateToGoal.lookahead1s, xy acceleration.4/yaw.6, integrated mecanum cap4.99rad/s. 독립.15/.10/.14 boxes 적용 안 함. 실제 joint hard cap5rad/s를 별도로 확인한다.
- Gazebo 외부pan/freshness/Collision Monitor/voxel chain은 관찰용 shadow/cmd_vel로 유지한다. wheel-cap relay가 sole /cmd_vel publisher다. backend guard1/1/2, stale timeout5wall seconds. DWB 내부 ObstacleFootprint/full footprint와 costmap sensor/depth/memory 계층은 유지한다. 추가 safety/footprint 축소 없음.
- target RTF1, headless only, domain225/ports22551/22552/unique partitions. 각 goal hard1200sim/1800wall timeout, soft240sim crossing timestamp만 기록.

## 재현과 자료

`cd /home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/dwb/20261014T0035_5bfca0` 후 `distrobox enter ros2-humble -- bash ./run.sh ./final_patch.yaml --output <새 디렉터리>`로 재현한다. 진단 SQLite 읽기 도구는 result.json이 존재하는 종료된 goal에서만 사용한다. live 진단은 ROS subscriber/별도 snapshot을 사용한다. run.sh가 ROS underlay/전용 navigation/measurement overlay와 domain/port/common path를 고정한다. common/READY가 필요하다. `final.yaml`은 instrumentation wrapper/trace 출력 경로를 제거한 최종 controller/planner 설정이다. scenario.json의 legacy spawn_pose=[0,0,0,0,0,0]는 이전 상대좌표 변환을 없애기 위한 값이며, 실제 CAD spawn은 locked canonical world/home pose이고 초기 GT/scene로 확인했다. 실제 benchmark wrapper/출력 경로, AMCL/safety/backend parameter files는 각 campaign goal/config에 보존된다. `distrobox enter ros2-humble -- python3 analyze_relaxed.py campaign`, 이어 `python3 build_report.py`로 파생 metrics/report를 재생성한다.

metrics.json/metrics.csv에 전체 raw-command stage, wheel limits, compute failures, guard/pan/relay diagnostics, trajectory counts, stop/reversal/jerk, actual periods를 저장했다. 각 goal에 raw rosbag, commands.csv, velocity.csv, joints/poses/contacts JSON, compute_timing.csv, server log, scene.info.pbtxt, actual parameters, source/config snapshot, provenance를 보존했다.

seat-29/seat-01 stall_snapshot의 비동기 local footprint 검사에서 각각 lethal cell2개/1개가 padded full footprint에 겹쳤다. 모든 rejection의 원인이나 특정 layer 책임을 단정하지 않는다. 검사 당시 raw contact 메시지는 네 wheel-ground topic에서만 관측됐다. runtime_environment_provenance.json은 실제 controller/planner plugin library 경로와 SHA를 추가 확인한다.

M1 runtime에서는 no_valid_lattice_path와 body-heading-grid route가 번갈아 기록되고 FollowPath cancellation/recovery를 거쳐 NavigateToPose가 status6으로 끝났다. 이는 static preflight와 별도의 runtime 실패다.

chair_gap/distant recorder가 SQLite database is locked 오류로 주행 중 중단됐고 raw bag은 prefix만 남았다. 에이전트의 live SQLite 진단 읽기와 오류 시점이 맞으며, 이 읽기 방식은 writer를 방해할 수 있어 제거했다. 이 두 구간의 recorder 종료는 host IO/부하에도 영향을 줄 수 있다. 독립 Probe의 전체 GT/command/joint/pose/contact JSON·CSV 및 compute/resource 기록은 남았지만 누락 raw 메시지는 복구할 수 없다. metadata.yaml은 offline ros2 bag reindex로 복구했고 DB 바이트 SHA는 전후 동일, quick_check는 통과했다. 재색인은 metadata 복구일 뿐 tail 복구가 아니다. final_validation.json에 bag clock 범위와 주행 구간 coverage를 저장했다. 고정1회 범위를 지켜 재주행하지 않았다.

첫 infrastructure 기동은 기존 contact 센서를 중복 추가해 Gazebo가 로드 실패했다. action을 전송하지 않았고 startup_failed_duplicate_contact에 보존했다. 중복 추가를 방지한 뒤 동일 geometry/settings로 캠페인을 실행했다. 공통 자료/잠금은 수정하지 않았다. 이전5goal/15cm 실험과는 world/map/tolerance/physics-step가 달라 causal 성능 개선으로 비교할 수 없다.

로컬/원격 common READY와204파일 SHA transfer 검증은 root remote_common_verification.final.json/READY.payload.json에 있다. 전용 checkout clean, main dirty tree 보존, no push.
