# cleany_navigation

SCRUM-425: 실제 로봇과 Gazebo가 공유하는 saved-map localization, Nav2 및 안전
runtime이다. SLAM은 외부 `slam_toolbox`/`cartographer_ros`, 주행은 외부 Nav2를
사용한다. 공통 노드는 Gazebo에 의존하지 않는다. Gazebo world/bridge, encoder,
카메라 TF adapter, rosbag 및 평가 도구는 `cleany_gazebo_sim`에 남는다.

## Saved-map navigation

```bash
make build-navigation
source /opt/ros/humble/setup.bash
source ros2_ws/install-navigation/setup.bash
ros2 launch cleany_navigation navigation.launch.py \
  map:=/absolute/path/map.yaml \
  robot_params_file:=/absolute/path/robot_params.yaml
```

`map`과 `robot_params_file`은 필수다. 기본은 `use_sim_time:=false`,
`autostart:=true`, `axis_motion:=false`, pan/memory/voxel 모두 활성화다.
`config/robot_params.template.yaml`, `config/body_profile.template.yaml`을 외부
로봇 설정 디렉터리로 복사해 null 항목을 채운다. Footprint, STOP/SLOW polygon,
바닥/높이, park posture와 최대 속도·반응시간·제동 감속도를 명시해야 한다.
Body profile 경로는 robot 설정 파일의 위치를 기준으로 해석한다.
설치된 CAD URDF의 관절 제한을 적용하며, Gazebo 관절 제한 override와 +Inf
clearing은 실제 시간 모드에서 거부한다. Controller 속도/가속도와 depth 범위도
backend에 맞게 조정한다. 제공된 숫자 설정은 제어 튜닝값이며 실물 안전 인증값이 아니다.

AMCL은 임의의 초기 위치를 주입하지 않는다. 활성화 후 RViz의 2D Pose Estimate
또는 `/initialpose`로 초기 자세를 제공한다. 저장된 지도에서
`/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) 목표를 받는다.
Omni DWB/lattice와 경로 검사·즉시 재계획 BT를 사용한다.
Humble/Jazzy별 plugin ID, progress checker, BT 포트와 Collision Monitor point
threshold 차이는 launch에서 변환한다. STOP/SLOW의 3-point 경계는 두 환경에서 같다.

## ROS 계약과 직렬 안전 경로

```text
Nav2 -> nav2/cmd_vel -> pan gate -> pan_ready/cmd_vel
     -> Collision Monitor -> safety_2d/cmd_vel -> voxel guard -> cmd_vel
```

모든 속도는 `geometry_msgs/Twist`의 x/y/yaw다. 최종 `cmd_vel` publisher는
voxel guard 한 개다. Backend는 아래 입력/TF를 제공한다.

| 입력 | 계약 |
| --- | --- |
| `scan` | LaserScan, 실제 LiDAR frame과 ROS stamp |
| `odom` | Odometry, `child_frame_id=base_link`, 실제 측정 x/y/yaw 속도 |
| `joint_states` | head_pan_joint의 position과 velocity, 유효한 stamp |
| `camera/head/depth/image_raw` | Rectified depth, 32FC1 m 또는 16UC1 mm; 0/NaN/-Inf는 관측 없음 |
| `camera/head/depth/camera_info` | Image와 동일 크기/optical frame, 유효한 K 또는 monocular P, D=0 |
| TF | Backend: odom→base_link와 base→LiDAR/head/optical; AMCL: map→odom |

Depth projection은 optical Z-depth를 원래 image stamp의 TF로 output frame에
변환한다. TF·calibration이 없으면 cloud를 발행하지 않는다. Park posture의 팔과 고정 몸체·카메라 기둥 형상으로
센서 원점부터 hit까지의 선분이 몸체를 통과하는 ray를 제거한다. 레일 뒤로 튀는
depth 경계 픽셀도 제거하며, 몸체에 가리지 않은 외부 장애물은 유지한다.
자기 필터는 각 OBB의 세 축에 대해 닫힌 선분 교차 구간을 누적해 계산한다.
박스마다 큰 임시 배열을 만들고 축약하던 비용을 줄였으며, 평행 ray,
표면 접촉, 센서가 박스 내부인 경우의 보수적인 제거 기준은 유지한다.
회전하는 head 부품은 정적 자기 필터에 포함하지 않으며,
몸체에 가린 ray를 free-space로 바꾸지 않는다. `depth_clearance.self_filter_margin_m`의 기본 1mm
여유는 depth/CAD 표면의 수치 오차를 처리한다. 실제 로봇에서는 보정 오차에
맞게 검증해야 한다. 공통 pan gate는
`head_pan/command` Float64를 발행하고 TF를 소유하지 않는다. Backend의 head
actuator adapter와 실제 joint feedback이 필요하다. 공통 노드의 topic/frame은
ROS parameter로 설정 가능하며 TF edge owner는 중복되면 안 된다.
`pan_min_position_rad`/`pan_max_position_rad`는 position actuator에 보낼 수 있는
관절 위치 범위다. 기본값은 기존 명령 범위인 [-π, π]이며 실제 하드웨어의
보정된 범위를 증명하지 않는다. Backend의 검증된 한계를 명시해야 한다.
목표 방위각과 정확히 같은 방향의 위치(2π의 정수 배 차이) 중 실제 pan에서
가장 가까운 범위 내 값을 명령한다. +π/−π 경계의 불필요한 한 바퀴 회전을
방지하되, 기계적 한계를 넘는 짧은 경로는 선택하지 않는다. 범위 안에 목표
방위각을 나타낼 수 없으면 `PAN_TARGET_OUT_OF_RANGE`로 속도를 차단한다.
정렬 tolerance와 fresh depth 조건은 동일하며 목표각을 clipping하지 않는다.

기본 주행은 전진·후진과 제자리 회전이다. 횡이동은 기본 차단한다.
PoseProgressChecker는 XY 0.15m 이동 또는 yaw 0.25rad 회전을 진행으로 인정하며,
20초 동안 둘 다 없으면 실패로 처리한다.
Pan은 자유 이동 모드에서 방향 정렬과 정렬 후 fresh depth를 확인한다. 축 제한 모드에서는 정지·settle도 확인한다. 후진 시 카메라를
뒤쪽(π rad)으로 돌리고 정지 후 새 depth를 받아야 속도를 통과시킨다.
뒤쪽 시야가 확보되었다는 물리 검증은 별도로 필요하며, 후진을 제외하려면
`pan_motion_gate.allow_reverse: false`로 설정한다. LiDAR/depth를 Collision
Monitor와 장애물 메모리에 전달하고, 3D voxel guard는 몸체 OBB와 예측 제동
sweep를 검사한다. 필수 입력 단절, stale/future stamp, TF 누락, localization
jump, voxel freeze 및 계산 시간 초과 시 최종 속도를 0으로 만든다. Pan/voxel의
steady timer는 `/clock`이 멈춘 상황에서도 정지 출력을 유지한다.
Swept-volume 검사는 각 예상 자세에서 `BodyBox`를 다시 생성·검증하지 않고
검증된 몸체의 변환과 SAT 축을 계산한다. 회전/병진 샘플 간격, voxel 크기,
몸체 padding과 제동 horizon은 동일하며, 측정 속도와 요청 속도가 정확히 같을 때만
동일한 제동 sweep의 중복 계산을 생략한다. 속도가 0이어도 관성으로 움직이는
측정 속도에 대한 검사는 유지한다.

몸체 guard는 **설정된 park posture**를 가정한다. 실시간 팔 FK 감시나 팔 자세
유지는 이 패키지에 구현되지 않았다. 관측한 voxel만 검사하며 unknown 공간을
free로 인증하지 않는다. 물리 E-stop, 드라이버 watchdog와 실측 제동 검증은
로봇 통합 작업에서 별도로 수행해야 한다.

Global/local costmap 모두 `obstacle_memory/grid`를 max 병합하여 planner와
controller가 동일한 누적 장애물을 검사한다.

`obstacle_memory/grid`, `obstacle_memory/voxels`, `obstacle_memory/voxel_status`,
`body_guard_3d/state`, `head_pan/state`를 진단에 사용한다. Memory의
`obstacle_memory/save`, `obstacle_memory/set_enabled` 서비스를 유지한다.
`output_directory`는 기본 빈 문자열이다. 지정했을 때만 memory/guard 로그와
스냅샷을 각각 하위 디렉터리에 기록한다. 디스크 기록 없이도 runtime이 동작한다.

`amcl_nav2_safety.launch.py`는 위 full runtime의 alias다. 안전 기능을 제외한
기본 Nav2 실험은 `amcl_nav2.launch.py map:=...`로 명시적으로 실행한다.
Full runtime의 `pan_enabled`, `memory_enabled`, `voxel_enabled`를 끌 수 있으나
voxel을 사용하려면 memory가 필요하다. 비활성화한 단계에 따라 속도 경로를
다시 구성하며, 기본값은 모든 단계 활성화다.

## 카메라 관찰과 주행 분리: reactive 시뮬레이션 모드

`safety_mode:=reactive voxel_enabled:=false use_sim_time:=true`는 다음 경로를 사용한다.
Gazebo safety wrapper의 기본 모드다. 공통 실제 로봇 entrypoint는 기존 `serial`을
유지하며, 실물 제동·관측 범위가 검증되지 않은 reactive 실제 시간 실행은 거부한다.

```text
Nav2 -> nav2/cmd_vel -> Collision Monitor -> safety_2d/cmd_vel
     -> reactive_guard -> cmd_vel
                  pan tracker -> head_pan/command (비동기 관찰)
```

`pan_motion_gate.observation_only=true`에서는 Twist publisher 자체를 만들지 않는다.
주행 방위가 이전 목표와 10° 이상 다를 때 관절 범위 내 관찰 방향으로 추적하고,
30°→25° 같은 변화와 0.01m/s 미만의 미세 움직임에서는 목표각을 유지한다.
`target_change_rad`, `tracking_minimum_speed`로 관찰 정책을 조절한다. 이는
충돌 허용 각도가 아니다. pan 정렬·settle·정렬 이후 depth를 주행 조건으로 쓰지 않는다.
`head_pan/state`는 `motion_authority=false`인 관찰 상태다.

`reactive_guard`가 유일한 최종 속도 publisher다. 기존 Collision Monitor의
STOP/SLOW polygon과 3-point 조건을 유지하고, 별도로 직접 LiDAR와 depth
표면을 전체 parked body의 XY 외접 사각형과 비교한다. 요청 속도와 측정 속도
양쪽에 기존 반응시간·감속도·margin·샘플 간 padding을 적용해 제동 경로를
검사하고, 센서 관측 나이도 반응시간에 더한다. 가까운 위험은 정지, 먼 위험은
기존 0.35 비율 감속이다. 높이 구간은 Collision Monitor와 공유한다.
2D 외접 형상은 높이별 틈을 통과하는 3D 판정보다 보수적일 수 있다.

Humble Collision Monitor는 [source 수집 코드](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp)에서
유효하지 않은 source를 건너뛸 수 있으므로 최종
guard가 직접 데이터 유효성을 확인한다. 원래 stamp의 `odom←sensor` TF로
관측을 변환하고, 신선한 `odom←base_link` 자세로 보정한다. TF를 기다리는
동안 이전 유효 관측은 기존 0.5초 freshness 이내에서만 사용한다. 필수 scan,
depth, odom, 동적 pan feedback의 손실/잘못된 입력, 명령 또는 monitor 출력
단절, 시계 정지, 계산 시간 초과는 0 출력이다. 시계 역행과 localization jump는
재시작까지 정지한다. raw 정지 명령을 늦은 monitor 출력이 되살리지 않는다.
물리 드라이버 watchdog/E-stop은 여전히 별도 계층이다.

Voxel map/status 및 pan alignment는 최종 guard 입력이 아니다. 장애물 메모리는
선택적으로 costmap에 계속 사용한다. 기존 3D cube 진단 발행과 depth marking/
clearing 밀도·이력도 유지하며, 그 발행이 속도 허가 조건은 아니다.
장애물 marking/clearing은 유지한다. `shared_voxel_costmap: true`이면 depth
VoxelLayer를 공유 world 투영으로 대체하며, false일 때만 기존 VoxelLayer를 사용한다.
`reactive_guard/state`에서 decision, source age, 처리 시간, 요청/출력 속도를 기록한다.
관측한 표면에 대한 감시이므로, 카메라 밖의 낮은 물체·돌출물·낙차나 동적 장애물을
전부 보장하지 않는다. 아직 없는 ToF를 보호 수단으로 계산하지 않는다. 사람이 없는
사전 확인된 통제 구역, 감독자/E-stop, 고정된 park 자세라는 데모 전제를 유지한다.

ROS 수신·TF·steady timer를 포함한 장애 경계는 다음으로 검증한다. 실제 actuator를
띄우지 않고 domain 225의 별도 test topic으로 명령·센서·시계 장애를 주입한다.

```bash
CLEANY_REACTIVE_PROFILE=/absolute/path/simulation_body.yaml \
  python3 -m pytest -c /dev/null --rootdir=. \
  ros2_ws/src/cleany_navigation/runtime_test/test_reactive_guard_ros.py -q
```

## C++ navigation plugins

아래 축 제한 설명은 `axis_motion:=true` 호환 모드에만 적용된다.
DWB `trajectory_generator_name: cleany_axis_controller::AxisGenerator`로 사용한다.
플러그인 기본값은 전후 X 또는 제자리 회전 속도만 가진 후보와 정지 후보를
컨트롤러 안에서 생성한다. 출력 이후 임의로 축을 자르지 않는다.
`PathHeadingCritic`은 경로의 1.2m 앞 방향에 몸체를 먼저 정렬시킨다.
대각선 보정이 꺼져 있을 때 방향 오차가 0.025rad 이하가 되면 병진 구간을 유지하고, 0.06rad를 넘으면 다시
정렬한다. 정렬된 구간에서는 작은 각도 개선만을 위한 회전 후보를 거부한다.
미정렬 회전 후보에는 연속적인 각도 오차 비용을 준다. `vtheta_samples: 31`로
작은 방향 오차에서도 정지보다 나은 회전 후보를 제공한다. 경로 방향이 몸체 전방에서 135도를 초과해 벗어날 때 후진을
선택한다. 측면 목표는 몸체를 돌려 전진하므로 90도 경계의 작은 각도 오차 때문에
전진/후진 선택이 뒤집히지 않는다. `reverse_angle_threshold`로 설정한다. 목표 XY 허용 범위
안에서는 `RotateToGoal`에 최종 yaw 정렬을 맡긴다. 횡이동 확장에서는 이 critic을
비활성화한다. lookahead와 방향 허용 오차는 `FollowPath.PathHeading`에서 설정한다.
DWB의 footprint 비용 평가와 가속·감속 rollout을 사용하며,
`discretize_by_time: true`, `time_granularity: 0.05`로 제동 궤적을 샘플링한다.

움직이는 동안은 측정된 현재 축과 정지 후보만 허용한다. 다른 축을
사용하려면 controller의 canonical `odom` 속도가 선형 0.01m/s, 회전 0.02rad/s 이하로
0.3초 연속 유지되어야 한다. 같은 축 재출발은 기다리지 않으며, 경로 갱신이
측정된 정지 이력을 초기화하지 않는다. 시계 역행과 비정상 피드백은 이력을 무효화한다. 병진과 회전이 섞인 움직임이나 비정상 측정은 정지만
허용한다. 제한 내 대각선 병진은 병진 동작으로 취급한다. `axis_linear_stopped`, `axis_angular_stopped`, `axis_settle_s`로
임계값을 설정한다. 물리적 관성·접촉·양자화로 실제 속도가 완벽하게
단일 축일 것을 보장하지 않으며 실제 하드웨어 안전 인증은 아니다.

Gazebo 없이 `make build-navigation`, `make test-navigation`으로 빌드·검증한다.
`axis_motion:=true` 호환 모드에서는 `cleany_navigation/config/nav2_axis_motion.yaml`을 사용한다.
공통 설정은 `axis_translation_angle_deg: 10.0`, Y 범위 ±0.0265m/s,
`axis_allow_lateral: false`로 전후 방향에서 ±10° 대각선 보정을 허용한다.
정지, 제자리 회전, 전후 병진(0°, ±5°, ±10°)을 후보로 평가한다.
순수 횡이동과 병진 중 회전은 허용하지 않는다. 합성 속도와 Y 한계를 함께 검사한다.
`axis_translation_angle_deg: 0.0`이면 기존 전후진 전용 동작으로 복귀한다.
`respect_path_orientation: true`에서는 lattice의 몸체 방향 허용대를 유지하며, 병진 후보의 실제 이동 방향을
별도로 평가한다. launch는 같은 제한을 pan gate에 전달한다. 작은 대각선 보정 중 카메라는
전방/후방을 유지하며, 후진 전환의 정지·settle·fresh depth 조건은 동일하다.
측정된 대각선 병진은 하나의 병진 동작으로 취급하고 회전 전환은 기존 정착 대기를 유지한다.
이는 simulation 설정이며 실제 카메라 가림과 횡방향 slip/제동 검증은 별도로 필요하다.
`cleany_navigation::StraightFirstPlanner`는 시작/목표의 제자리 회전과 직선 이동을
map 해상도의 절반 간격으로 검사한다. 현재 global costmap의 padded footprint 경계와
내부에 lethal/unknown 셀이 있거나 중심이 inscribed 셀에 들어가면 직선 계획을 거부하고
Smac으로 우회한다. 뒤쪽 목표도 후진 몸체 방향을 보존하도록 Smac에 위임한다.
검사 도중 취소도 처리한다. 안전 노드는 기존대로 최종 속도를 검사한다.
Smac 우회 탐색은 geometry와 cost 배열을 함께 복사한 전용 지도에서 수행하며,
탐색 중에는 live costmap 잠금을 보유하지 않는다. 반환 직전에는 최신 live 지도에서
연속적인 padded footprint sweep를 다시 검사하여 새 장애물이 생긴 경로를 거부한다.
`GridBased.max_planning_time`은 선택적 여유 거리 0.05/0.025/0 m의 세 탐색이
공유하는 양의 유한 시간 예산이다. 남은 시도에 시간을 균등 배분하며,
Nav2 내부 시간 확인 간격과 초기화/최종 검사 때문에 소폭 초과할 수 있는 soft deadline이다.
Humble의 planner API에는 실행 중 취소 checker가 없으므로 시간 예산으로 탐색을
종료하고, 취소 checker를 지원하는 Nav2에서는 취소와 deadline을 함께 전달한다.
Smac은 설치된 diff lattice에서 직선과 제자리 회전 primitive만 선택한
임시 lattice와 후진 확장을 사용한다. 곡선 analytic shortcut도 비활성화한다. AMCL은 실제 Mecanum
odometry의 횡방향 오차를 모델링하도록 OmniMotionModel을 유지한다.
횡이동 확장은 별도 설정 파일에서 `FollowPath.axis_allow_lateral: true`,
Y 속도 범위와 샘플 수, `pan_motion_gate.allow_lateral: true`를 함께 지정한다.
이때 launch는 Omni lattice를 선택하며, 명시한 `lattice_filepath`가 있으면
그 파일을 우선 사용한다. 기본 주행과 확장 주행 모두 이동과 회전을 혼합하지 않는다.
`make build-gazebo`에도 포함된다. 활성 프로필의 install 환경을 source한 뒤
`ros2_ws/build-harmonic/cleany_navigation/test_axis_generator`로
기본 횡이동 차단, 확장 시 횡이동 후보, 정지 후 축 전환, 혼합 입력 제동 및 제동 궤적을 검증한다.
`make test-gazebo-navigation-pipeline GAZEBO_PROFILE=harmonic`은 일치하는 방/지도에서
전진, 후진, 90도 방향 전환 후 이동의 목표 완료와 모든 속도 단계의 Y=0을 검사한다.
이 fixture는 정확한 spawn, ideal odometry와 초기 pose 표준편차 5mm/0.5도,
목표 허용 오차 5cm/0.08rad를 사용한다. 기본 runtime 목표 허용 오차는 12cm/0.15rad다.
실제 로봇의 localization 정밀도를 검증하는 시험은 아니다.

`cleany_yield_wait_bt_node` 라이브러리는 Nav2 Wait action을 사용하는
`YieldWait` BT 노드를 제공한다. `wait_duration`은 양의 유한 초 단위이며
의도적인 양보에는 recovery count를 증가시키지 않는다. 취소와 결과 처리는
Nav2 BtActionNode를 사용한다. navigator의 `plugin_lib_names`에 등록한다.
플러그인 등록 헤더는 Nav2가 제공하는 Humble의 BehaviorTree.CPP v3 경로와
Jazzy의 v4 경로를 지원한다. factory 헤더는 등록 구현 파일에서만 포함한다.

AxisGenerator, PathHeadingCritic과 YieldWait는 이 패키지에서 C++ 공유 라이브러리로 빌드한다.
기존 설정 호환을 위해 `cleany_axis_controller` C++ namespace와 플러그인
식별자는 유지하지만, 별도의 ROS 패키지는 필요하지 않다. 헤더는
`include/cleany_navigation/`에 설치한다.

## Mapping

```bash
ros2 launch cleany_navigation slam_mapping.launch.py
ros2 launch cleany_navigation slam_mapping.launch.py use_sim_time:=true
ros2 launch cleany_navigation cartographer_mapping.launch.py
ros2 launch cleany_navigation cartographer_mapping.launch.py \
  configuration_basename:=cartographer_2d_imu.lua
```

Mapping은 주행 runtime과 별도다. 기존 scan/odom/TF 계약을 사용한다.

## 검증

```bash
make test-navigation
make test-navigation-runtime
make test-gazebo
make test-gazebo-nav-runtime
make test-gazebo-navigation-pipeline GAZEBO_PROFILE=harmonic
```

앞의 두 명령은 Gazebo 없이 navigation과 description dependency만 빌드한다.
Unit test는 형상·depth decoding·pan·memory·voxel·parameter/BT 호환성을 검사한다.
Opt-in runtime test는 합성 backend와 저장 지도에서 lifecycle, 목표 수락, 직렬
속도 출력, 장애물 STOP, cancel, 입력 단절 및 clock pause를 검사한다. 합성
backend의 설정은 테스트 fixture이며 실물 calibration이 아니다.
합성 카메라는 프레임 내부에서 모든 광선이 self-filter에 가려지지 않도록
합성 base 기준 높이 1.0m에 배치한다. 실제 backend의 카메라 mount를 변경하는 설정은 아니다.
Gazebo sensor 회귀 테스트는 backend의 sensor/odom/TF 계약을 확인한다. Harmonic pipeline test는
CAD backend, 실시간 depth/pan/voxel과 일치하는 방 지도에서 0.6m 직선 목표의 완료와
주행 후 정지를 확인한다. 장거리·혼잡 환경의 성능이나 실물 검증을 대신하지 않는다.

Jazzy에서는 `ROS_SETUP=/opt/ros/jazzy/setup.bash`와
`NAVIGATION_BUILD_BASE=build-navigation-jazzy`,
`NAVIGATION_INSTALL_BASE=install-navigation-jazzy`,
`NAVIGATION_LOG_BASE=log-navigation-jazzy`를 make에 함께 전달해 Humble 산출물과
분리한다. 관련 RViz 설정은 `rviz/navigation_costmap.rviz`,
`rviz/voxel_navigation.rviz`다.

Depth 바닥 제외: `/camera/head/depth/points`는 바닥을 포함한 관측 ray를 유지하고,
`/camera/head/depth/obstacle_points`는 `base_link` 기준 `marking_min_z`–`marking_max_z`
안의 점만 제공합니다. `shared_voxel_costmap: false`의 VoxelLayer는 전자를 clearing 전용, 후자를 marking 전용으로
사용합니다. 기본 시뮬레이션 바닥은 z=-0.38 m이고 marking 최소 높이는 -0.33 m입니다.
CAD 카메라 주행 자세는 수평에서 아래로 20°입니다. 실물 높이 기준은 로봇 parameter로
맞춰야 합니다.

BT는 100ms 주기로 앞 1.5m 경로를 검사하며, FollowPath에는 원본 경로를 전달한다.
이동에 따른 앞부분 제거는 DWB가 수행하므로 매 pruning마다 새 목표/정지 이력을 만들지 않는다.
앞으로 1.5m 경로가 무효화되면 고정 대기 없이 즉시 재계획한다. Planner/controller 실패 시에만 0.2초 backoff 후 재시도하며, 장애물 지도를 지우지 않는다.

Depth는 최대 `tf_wait_timeout_s`(기본 0.15초) 동안 최신 한 프레임만 보관하며
비동기 재시도로 원래 시각의 TF를 기다린다. TF를 추정하거나 latest TF로 대체하지 않고,
대기 만료 시 버린다. 기존 VoxelLayer 경로에서는 marking에 obstacle_points,
clearing에 전체 points를 사용한다. 공유 world 경로에서는 self-filtered 전체 points를
voxel memory에서 처리한다.

The axis controller follows the body orientation stored in the lattice path
(`PathHeading.respect_path_orientation`). Bounded diagonal compensation does
not expand the tolerance for the body's planned heading. Repeated XY poses
represent an in-place rotation; the last orientation in that group is used
within `PathHeading.rotation_position_tolerance_m` of its position. This keeps
the executed rectangular footprint aligned with the shape checked by the planner.

The lattice fallback uses Nav2 Smac search with a private footprint padded by
`GridBased.lattice_collision_padding` (default 0.05 m), 0.5-degree collision
orientation bins, and full footprint checks. When that additional margin cannot
yield a valid path, search retries with half a cell less extra margin, then
with the configured robot footprint alone. The mandatory costmap footprint
padding remains in every attempt. The returned path is checked
through translation and rotation between poses at at most half a costmap cell
of swept distance, including obstacles inside the footprint. Invalid sweeps
are rejected. This search margin is separate from costmap/voxel resolution.
`GridBased.cost_penalty` is 5.0 to prefer room around obstacles.


기본 주행은 `axis_motion:=false`와 `config/nav2_omni_motion.yaml`을 사용한다.
DWB의 `StandardTrajectoryGenerator`가 전후·횡·대각선 이동과 회전을 동시에
평가하며 ±10° 제한과 사용자 정의 `PathHeadingCritic`을 적용하지 않는다.
최대 병진 속도는 0.15 m/s이며 planner는 전체 omni lattice를 사용한다.
`axis_motion:=true`는 기존 제한 모드를 재현할 때만 선택한다.

카메라 게이트의 `allow_mixed_motion: true`는 `atan2(vy, vx)` 방향으로 pan을
추적한다. 카메라가 이동 방향과 맞지 않으면 속도 명령을 차단하고,
정렬 후 새 depth가 도착해야 이동한다. 자유 이동 모드는 몸체·카메라의 정지와
고정 0.2초 안정화 대기를 요구하지 않는다. Cloud는 획득 시각의 TF로 변환하므로
카메라가 움직이는 동안에도 사용할 수 있다. 이미 관측한 방향에서의
작은 방향 보정은 계속 이동하며 수행한다. 회전만 할 때는
pan 방향을 유지한다. 명령 timeout과 2D·3D 충돌 검사는 그대로 적용된다.

Gazebo 주행 전용 profile의 `simulation_excluded_link_roots`는 양쪽 어깨부터
팔·gripper·손목 카메라 subtree를 제외한다. 동일 profile을 모델 변환, 몸체
충돌 형상 및 depth 자기 몸체 필터에 사용해야 한다. 실시간 로봇 설정에서는
이 옵션을 거부한다. 팔 없는 시뮬레이션 결과는 실제 팔 장착 로봇의 검증이 아니다.

### Holonomic paths, arrival precision and map reloads

The omni profile explicitly enables `GridBased.holonomic_direct_path`. Direct candidates translate at the existing body yaw and turn at the destination, or turn first and translate at the goal yaw. Rear and lateral translation are allowed only in this profile. Both strategies pass the complete continuous padded-footprint sweep; blocked candidates still fall back to the conservative lattice. Axis profiles retain heading-aligned direct paths. No costmap obstacle, body geometry or guard threshold is removed.

The omni controller and DWB RotateToGoal use the same .05 m XY tolerance, with goal yaw .10 rad and planner endpoint tolerance .05 m. The mission navigator additionally checks the original requested pose. DWB's `Twirling` critic (scale 1.0) discourages unnecessary yaw when a safe non-rotating holonomic trajectory exists. It does not prohibit rotations needed to avoid obstacles or align at the goal. Hardware precision and manipulation reach remain uncalibrated.

A `/map` content reload with identical frame, origin, resolution and extent updates obstacle memory's map identity and saved snapshot while preserving all observed evidence. A geometry change freezes mapping and publishes disabled voxel status; enabling recording cannot override that fault. Restart with the correct new map geometry before motion. Changing a static map does not erase evidence of a removed chair; clearing must be supported by sensor rays or an explicit simulation-fixture reset.

`PathOrientation` (`cleany_navigation::PathOrientationCritic`, scale 8.0 in
the omni profile) follows the planned body yaw independently of translation
direction. Same-position path pose groups retain their explicit planned
rotation. This supplies a yaw reference throughout travel rather than relying
only on RotateToGoal near the endpoint. `GridBased.require_exact_goal` adds
the requested goal to a tolerance-found lattice path and validates its full
connecting body sweep; an obstructed exact approach is rejected.

`HolonomicGenerator` retains the standard velocity grid and acceleration-aware
rollout, then adds translations scaled by `fine_translation_scale` (.25) only
for zero-yaw samples. This provides .0125/.025/.0375 m/s samples with the
7-by-7 XY grid, instead of multiplying every yaw sample by a denser XY grid.
Commands remain within configured velocity/speed bounds and pass the same
critics and serial 2-D/3-D safety gates. Near-zero floating-point components
are normalized to exact zero so RotateToGoal's nonrotation check works.
Fine approach is needed because a .05 m/s command over a 2 s rollout advances
roughly .10 m, while the tightened .05 m goal and grid distance scores can
otherwise favor a stationary trajectory near the endpoint.

The `cleany_navigation::GoalDirectedController` is a DWB
subclass that supplies the current control-call pose and the original global
goal transformed into that pose's frame to `HolonomicGenerator`. It delegates
planning, pruning, critic preparation and command scoring to DWB. With
`goal_directed_radius: 0.75`, the generator adds at most three zero-yaw
translations along the exact body-frame goal vector within 0.75 m. Target
speeds are 1/4, 1/2 and 1 times the lesser of distance/prediction-horizon and
the configured directional component/XY speed limits. The inherited
acceleration/braking rollout and all critics still apply; these are candidates,
not forced commands or a stopping-distance guarantee. Context is consumed
once and cleared on reset and control-call exit, including exceptions. Missing
or untransformable goal context adds no candidates. A radius of zero (the
generator's default) disables the feature. The shipped omni profile uses this
controller with radius 0.75, matching the full-facility GUI 43/home/13/home
validation profile. This is simulation evidence, not physical robot validation.
At a speed cap, the candidate speed is rounded inward by two floating-point
representable steps; strict component/XY limits remain enforced. Tests cover
frame rotation, speed/acceleration budgets, max-speed round-off,
disabled/invalid/expired context, forward-only limits and controller plugin
loading. Timing and ground-truth arrival/return errors must still be checked
for the loaded map, geometry and sensor profile.

Single-config builds with no explicit `CMAKE_BUILD_TYPE` now default to
`RelWithDebInfo` (`-O2 -g -DNDEBUG`), keeping debug symbols while optimizing
trajectory scoring, footprint sweeps and lattice-template code. An explicit
Debug/Release setting and multi-config generators retain their chosen mode.
For a debug build, pass `--cmake-args -DCMAKE_BUILD_TYPE=Debug` to colcon.
No fast-math or relaxed floating-point/velocity checks are enabled. The
optimized package still requires the same regression and runtime checks;
offline compute improvement is not proof of navigation success.

The StraightFirstPlanner adapter links only `nav2_smac_planner_lattice`.
Humble exports hybrid, 2D and lattice libraries together, and each contains
the same NodeLattice static heuristic tables. Linking hybrid and lattice
together caused duplicate static destruction and a process-exit double free.
The search regression now calls planner/costmap cleanup and ROS shutdown,
then uses normal process exit so static destructors are tested. An isolated
real planner-server configure → cleanup → SIGINT test also verifies a zero
exit code and that only the lattice implementation is loaded. This verifies
the identified planner exit defect; it does not certify all other nodes or
active planning cancellation during shutdown. The subprocess regression sets its own
`RLIMIT_CORE` to zero: a deliberately reproduced failure must not launch
a host crash analyzer during live validation. Signal/exit status and stderr
still fail the test, and normal static destruction remains exercised.


Planner failure evidence: `GridBased.failure_snapshot_directory` defaults to empty (disk capture disabled). Safety launch `output_directory` sets it to `<output_directory>/planner_failures`. A failed lattice fallback saves the exact immutable search grid (`search.bin`), a later live grid copied under its mutex (`live.bin`), and `failure.json` containing start/goal, footprint, parameters, attempt iteration counts/times and failure category. These grids are distinct times, not an atomic two-map observation. Files overwrite the previous failure, bounding disk usage. Binary cells are raw uint8 row-major costs; geometry is in the JSON. Check each map's decimal FNV-1a 64-bit hash against its bytes before replay: individual files are atomically renamed and the manifest is written last, so a concurrent reader must reject a mixed generation. Recording runs after a failed search and outside its planning deadline; disk I/O can delay the failure response, so leave capture disabled for timing benchmarks. A capture error logs a warning and never permits an invalid path. Successful direct/lattice paths do not write files. Attempt logs are available even without disk capture.

Controller failure evidence: the omni profile uses `cleany_navigation::SweptFootprintCritic` under the existing `ObstacleFootprint` name. It retains stock scores/rejections and adds the conservative swept-cell veto described below. Its `ObservedFootprintCritic` base records both stock and swept rejections; `collision_model` identifies the active model. When every evaluated candidate is rejected by that critic, it saves the first failed cycle of the episode to `<output_directory>/controller_failures/failure.json`. The single atomically replaced manifest contains the exact local costmap row-major uint8 costs, geometry/frame, scored pose/velocity, footprint, transformed path, candidate rejection counts/reasons and first rejected trajectory. DWB's costmap mutex remains held through failure debrief, so these values describe one scoring cycle; a later GetCostmap service response or latest `/evaluation` subscriber sample does not provide that guarantee. A cycle with any obstacle-valid candidate ends the episode, even if another critic subsequently rejects navigation. Other failure categories therefore require the enabled DWB rejection-detail logs. `FollowPath.ObstacleFootprint.failure_snapshot_directory` defaults to empty; capture errors warn without changing scores or rejection. Synchronous diagnostic disk writes can delay a failure response and costmap updates; disable capture for timing benchmarks. One file bounds retained disk usage, and repeated all-rejected cycles do not overwrite the first-cycle evidence.

The omni profile sets `FollowPath.limit_vel_cmd_in_traj: true` and `use_elapsed_command_time: true`. `HolonomicGenerator` splits the first rollout interval using ROS-clock time elapsed between control iterations, capped at the original first rollout interval. Nav2 publishes the resulting first limited velocity. For an actual 0.02 s interval, angular acceleration 0.6 rad/s² permits an initial 0.012 rad/s rather than the old fixed-0.05 s result of 0.03. Startup/new-path reset, repeated clock time and backward jumps grant zero acceleration budget; nonzero measured velocity is held on that first zero-duration step. Subsequent iterations resume their normal budget. A long gap cannot grant a larger step than the original rollout interval. Remaining steps still cover the complete prediction horizon and model acceleration/braking; no candidate-specific or per-candidate clock reads change the budget. `use_elapsed_command_time=false` is available for explicit legacy/offline reproduction. Emergency/cancellation stages still issue zero independently. This is a configured motion model, not calibrated physical braking or a guarantee against stale odometry/downstream command changes.

`CommandBearingCritic` adds a soft cost for changing the body-frame translation bearing from the last selected DWB command. This addresses repeated camera target changes such as forward ↔ 45° diagonal while acceleration-limited corrections are small. It does not prohibit lateral/reverse motion, alter the footprint, or bypass pan alignment/depth freshness. Zero commands retain the last bearing; a new path resets the preference. The cost is zero within `CommandBearing.free_goal_distance` (default 0.1 m), then fades linearly to full weight at `CommandBearing.near_goal_distance` (default 0.3 m). Final lateral/reverse corrections therefore incur no bearing cost. The omni profile uses scale 2.0. These are runtime tuning parameters, not physically calibrated safety limits; full mission performance must be measured with the loaded profile.


Initial `ComputePathToPose` now shares the bounded patience policy: `InitialPlanWithPatience` makes at most four planning attempts (initial plus three retries), with a 0.2-second `YieldWait` between failures. Each attempt takes a new planner snapshot and retains its own configured shared search deadline. No path is followed until planning succeeds; repeated failure terminates navigation without clearing observed obstacles. Cancellation during the backoff halts the tree before another attempt. This handles transient initial planning failure, but does not authorize motion from an occupied pose or repair a colliding controller trajectory.


Lattice start connection: the closest discretized heading can collide even when the original continuous pose is free. Fallback now considers headings in angular-distance order at the same snapped grid cell and accepts a seed only after both its conservative lattice collision check and a continuous padded-body sweep from the real pose (prefer rotation then translation; also check translation then rotation and interpolated translation/rotation when necessary) pass. The real pose and checked connection remain at the beginning of the returned plan. No start cost is cleared, pose is teleported, or occupied original pose allowed. Seed selection shares the same total planning deadline; each returned route is still vetoed against the latest live map.


Near-goal control: omni `GoalDist.class` is `cleany_navigation::PreciseGoalDistCritic`. It retains stock GoalDist grid scoring and failures, adding endpoint Euclidean distance divided by map resolution only inside `GoalDist.near_goal_distance` (0.3 m), with a linear weight that reaches zero at that boundary. DWB applies the existing GoalDist scale, including its stock resolution factor. This resolves score ties between stationary and closing trajectories in the same goal cell; the actual 5 cm goal checker and independent mission arrival gate remain authoritative. CommandBearing costs are zero within `free_goal_distance` (0.1 m), fading to full weight at 0.3 m, so prior camera bearing cannot penalize final lateral/reverse corrections against a free zero command. Parameter distances must be finite, near distance positive, and free distance in [0, near distance). These changes do not bypass footprint, path-grid, camera freshness or 3D safety checks. The subcell plateau regression is tested against stock Nav2 scoring/rejection. Simulation arrival and complete home return must be revalidated after the change.

Controller onset evidence now includes `previous_cycle`: a bounded in-memory copy of the most recent preceding DWB debrief with at least one footprint-valid candidate (or zero candidates), including grid geometry/bytes, pose, measured velocity, actual DWB command, goal/path/footprint, counters and sim-clock stamp. This does not imply another critic accepted that cycle or that downstream safety gates issued the same command. The first all-footprint-rejected cycle writes the preceding copy alongside its own snapshot; subsequent failed cycles remain latched. New paths reset the latch and prior copy. Recording is disabled with an empty failure_snapshot_directory. Enabled capture copies a local grid and path each preceding cycle under the DWB map mutex; JSON/disk writing still occurs only on failure, so disable evidence capture for timing benchmarks. This lets a future run distinguish map changes from motion between consecutive evaluations; existing failure artifacts without previous_cycle do not support that distinction.


Swept collision model: `SweptFootprintCritic` first runs Nav2 ObstacleFootprint scoring. Each adjacent pose interval then uses the convex hull of the endpoint oriented footprints, expanded conservatively for rotating vertices (`max_vertex_radius * delta_yaw² / 8`) and the center curvature of a constant issued body-frame twist relative to the Euler translation (`segment_length * abs(delta_yaw) / 2`). SAT tests all lethal/unknown **cell squares**, including body interior, against that enclosing region. Thus the check does not rely on choosing a temporal sampling frequency. Off-map, empty/degenerate/nonfinite geometry is rejected. Valid trajectories retain their stock scores; added vetoes are included in the same coherent failure evidence. This deliberately strengthens spatial occupancy semantics as well as intermediate-motion checks: unlike stock boundary raster scoring, a partially intersected cell square or interior occupied cell is an obstacle. Conservative extra vetoes are possible, especially for large yaw intervals or coarse cells. The planner/path tracking must maintain clearance under this model; no occupied-current-pose escape is added. Bounds apply to the configured polygon, costmap and piecewise rollout/constant-twist assumptions, not sensor uncertainty, actual contact, arbitrary physical dynamics or later downstream modifications.

The cropped `test/data/return_sweep_alias.json` fixture retains the observed pre-failure pose/velocity/footprint and lethal/unknown cells. Its recorded legacy candidate passes stock sampled scoring but is rejected by the new swept model; soft costs are omitted without changing stock collision validity. Additional tests cover crossing an obstacle between free endpoints, rotating into an obstacle between free endpoints, a bowed vertex arc outside the endpoint hull, unknown/body-interior cells, bounds, and clear-map score equality. Runtime CPU, route availability and full 43/13 arrival/return require actual verification after this change.

Swept-check performance: `prepare` builds a summed-area count of lethal/unknown cells from the costmap held under the DWB scoring mutex. A swept bounding rectangle certified to contain zero such cells returns free without hull/SAT work; nonempty rectangles run the complete geometry test. The cache is rebuilt for each cycle, including changed map cells/origin/size. It never replaces stock scoring and is not reused across unlocked map updates. This avoids the initial expensive all-free-map implementation. The offline batch benchmark includes preparation and all 523 candidates, but excludes generation, other critics, executor load and diagnostic I/O; it is not a measurement of end-to-end runtime CPU or a worst-case bound.

Holonomic translation ramp: `preserve_translation_direction=true` (default and omni profile) uses one X/Y interpolation fraction, chosen as the smallest fraction allowed by the inherited per-component acceleration/deceleration projections. Angular motion remains independently limited. From rest to a fixed translation target, and from a fixed translation velocity to zero, the translation bearing stays constant; a general velocity change follows the straight segment between its current and requested vector. Each component still respects its original budget, possibly using less than the maximum. This prevents unequal X/Y targets from initially becoming a 45-degree command and changing bearing again when one component reaches its target—a feedback that can restart pan alignment after the body is stopped. The same projection runs in published first velocity and all rollout steps. Explicit false is reserved for reproducing recorded legacy component-wise rollouts; stored pre-change replay parameters must disable both this option and use_elapsed_command_time. This model does not constrain a downstream actuator's actual acceleration or certify physical response. Camera freshness/alignment criteria, path/footprint/voxel checks and navigation deadline were not relaxed. Runtime performance remains to be verified after the direction change.

Optional `FollowPath.timing_trace_file` (default empty) records at most 10,000
generator iterations in memory. It records current/previous ROS timestamps,
reset generation, raw/used command delta, measured velocity, actual first-step
range and counts of nonzero targets/zero first commands. On clean generator
destruction it writes JSON through a temporary file and rename; abrupt process
kill may lose the trace. There is no trace file I/O in the control loop and no
change to sampling, acceleration, scoring or emitted trajectories. Inspect
`dropped` before treating a trace as complete. `after_reset` includes initial
startup; `clock_unchanged` and `clock_backward` require a preceding iteration.
A zero first command may still predict subsequent motion and is not evidence
that a stationary target won. Trace records are controller-cycle inputs, while
asynchronously received sensor/plan messages are separate context.

Holonomic lattice fallback: when `GridBased.holonomic_direct_path=true`, a
blocked direct path first receives an XY grid search at the requested goal
body yaw, then at the start body yaw if needed. A safe initial rotation is
mandatory for the first strategy. This keeps the preferred body orientation
tied to the mission goal rather than adopting a new current yaw at each
replan. This searches for clearance for the chosen body
orientation instead of simply reorienting a tangent-yaw lattice route that
hugs a corner. Eight-neighbor edges, exact start/goal connections and endpoint
rotations use the full-cell swept-footprint model, with the configured robot
footprint and safety padding. Lethal/unknown cells and map boundaries veto
sweeps; inflated center costs also penalize search via `GridBased.cost_penalty`.
The search uses the private map snapshot. A returned route must independently
pass the same swept check on the latest locked live map. Live verification
checks the exact selected poses; it never reorients the route or chooses a
different yaw strategy. Each heading search
gets at most a quarter of `max_planning_time`, leaving roughly half for the
original rotating lattice fallback; all checks share cancellation/deadline.
This search is restricted to the existing holonomic option. It does not
smooth XY corners or prove downstream tracking/camera alignment performance.

If fixed-heading XY search cannot return a valid route, a
valid lattice XY route also receives a fixed-body-heading alternative. It
tries preserving the start yaw and rotating only at the goal, then preserving
the goal yaw after an initial rotation. The searched XY points and exact
endpoint poses remain. Every intervening translation/rotation is checked with
the conservative full-cell swept-footprint model, using the robot footprint
including its configured costmap safety padding on both the private search map
and the latest locked live map. `lattice_collision_padding` guides the lattice
search with optional extra clearance; it is not an additional acceptance
requirement for the fixed-heading alternative. This matches the required
footprint used to accept the original rotating route. Required padding is
never removed, and lethal/unknown cells and map boundaries still veto a sweep.
These checks share the existing cancellation/deadline budget. If neither
alternative passes, the original lattice route remains subject to its original
live veto. This does not certify tracking or physical clearance: downstream
controller/pan/collision/voxel guards remain active. Nonholonomic profiles
retain the original route. Live 43/13 progress must be verified separately.


### 가속 시뮬레이션의 장애물 메모리 TF 처리

`obstacle_memory`와 TF listener는 같은 단일 스레드 executor를 사용한다.
관측 callback에서 TF를 blocking wait하면 TF 수신 자체가 지연되고, 가속된
clock에서 update timer가 밀려 관측을 한 건도 반영하지 못할 수 있다.
TF lookup은 원래 센서 timestamp로 즉시 조회하며, 아직 없으면 다음 update에서
다시 시도한다. 최신 TF로 대체하거나 `source_timeout_s`를 늘리지 않는다.
원인을 구분할 수 있도록 조회 실패의 센서 timestamp와 예외를 제한된 빈도로
기록한다. 관측이 신선하지 않으면 기존 voxel guard의 정지를 유지한다.

전체 시설의 302,400셀 격자 및 depth/voxel PointCloud2는 ROS의 typed array로
직렬화한다. int8 격자의 unknown/free/occupied 값(-1/0/100), float32 XYZ 바이트,
센서 timestamp와 발행 주기는 보존하며 셀/바이트별 Python 검증 반복만 피한다.
이 비용도 TF 수신을 밀리게 하므로 nonblocking lookup만으로 해결됐다고
판단하지 않는다. 전송 회귀는 실제 ROS 직렬화 왕복, 지연 TF 재시도와 stale
관측 거부를 확인한다.

### 주행 방향 연속성과 관측 범위 진단

`HolonomicGenerator.continuity_samples`는 측정된 이동 방향에 최대 속도의
25%, 50%, 100% 후보를 추가한다. `GoalDirectedController`는 같은 frame의 직전
선택 속도를 현재 body 방향으로 변환해 한 제어 주기에만 추가로 전달한다.
`FollowPath.continuity_timeout_s`(기본 0.5초), 새 경로, 실패, 시계 역행으로
이전 선택의 재사용이 제한된다. 이 후보들도 기존 가속도 rollout과 모든 critic을
거친다. 충돌 검사 뒤 최종 속도를 보간하지 않는다.

`CommandBearing.switch_cost`는 방향 변경에 주는 추가 soft cost다.
`switch_angle`까지 선형 증가하고 그 뒤 포화하며, 기존 near/free goal 거리에서
감소해 목표 근처 미세 보정을 허용한다. omni 설정은 각각 0.5, 0.04rad를 사용한다.
이 값은 통과 허용 각도가 아니며 `pan_motion_gate.angle_tolerance=0.04`와
센서 freshness, 속도, 제동, 도착 및 미션 timeout은 변경하지 않는다.

`depth_coverage.ros__parameters.enabled: true`를 motion/robot overlay에 추가하면
`/head_pan/coverage`에 **shadow 진단만** 발행한다. 노드는 cmd_vel이나 pan 명령을
발행하지 않는다. launch에서 기존 body profile, braking 값, optical frame 및
pan source timeout을 공유한다. 원본 rectified depth image와 촬영 시점의 정확한
`odom ← optical` TF를 보관하고, 요청/측정 속도 양쪽의 body sweep를 검사한다.
현재 body 안에 완전히 포함된 셀만 제외하며, 나머지 셀의 전체 영상 투영 사각형에
유효하고 더 먼 depth가 있어야 관측됐다고 판정한다. 장애물 뒤, 영상 밖, NaN/inf,
빈 pixel, stale/future 영상은 빈 공간으로 간주하지 않는다. 관측은 최대 8장,
기본 0.5초 동안만 유지하고, 시계 역행과 odom 자세 점프에서 지운다. 새 영상의
장애물/무효 pixel이 오래된 빈 공간 판정보다 우선하며, 셀 수/연산 시간 초과도
미검증으로 처리한다.

현재 고정된 카메라/몸체 구성은 차체 근접 사각지대를 포함하므로 이 진단을
기존 각도 가드의 대체 권한으로 사용하지 않는다. 0.1m 셀과 보수적인 영상
사각형은 실제 관측 범위보다 작게 평가할 수 있고, pixel 모델은 실물 센서의
오차·혼합 pixel이나 움직이는 장애물에 대한 안전 보증이 아니다.
이 진단의 실패를 통과로 바꾸지 않는다. 위 reactive 시뮬레이션 모드는 별도의
직접 센서 감시 구조이며, 전체 sweep 관측을 증명했다는 의미가 아니다.

2026-10-04 전체 시설 GUI에서 같은 조건을 각각 2회 실행한 결과, 43/13 왕복
평균 wall 시간은 36.5/157.2초에서 30.0/121.4초로 줄었다. 13번 복귀의
GT 속도 기준 정지는 평균 29.0→12.9초였다. 모든 미션이 도착·복귀를 완료했지만
실측 RTF는 개선 후 1.37–1.41로 2배속에 못 미쳤고, 43번 홈 GT 오차는
평균 7.5→8.5cm, 13번 도착 GT 오차는 3.4→4.2cm로 일부 악화됐다.
관측 범위 진단은 한 번도 전체 sweep를 검증하지 못해 주행 권한을 부여하지 않았다.
142개 패키지 테스트를 통과했다. 로컬 비교 근거는
`artifacts/navigation-continuity-20261004/RESULTS.md`와 `validation.json`이다.


2026-10-04 reactive GUI 비교: 동일 화면 각 1회에서 43/13 왕복 wall 시간은
31.98/110.65→21.41/89.30초, 13번 복귀 정지 수준 시간은 9.57→2.15초였다.
155개 패키지 테스트 및 별도 ROS 장애/복구 27개 경우를 통과했다. 실측 RTF
1.43–1.45, 13번 홈 GT 오차 12.32cm로 2배속과 위치 오차 문제는 남는다.
추가 작은 GUI 실행에서는 SlowZone과 DWB 명령의 상호작용으로 약 0.01m/s
creep가 관측됐다. 검증 범위와 원본 근거는
`artifacts/navigation-reactive-20261004/RESULTS.md`를 참고한다.

### Nav2 실행 트리 관측

안전 주행 launch는 distro별 변환을 마친 `wait_before_detour.xml`의 각 노드에 고유한 instance name을 부여합니다. 노드 순서·포트·재시도·안전 정책은 유지합니다. 같은 디렉터리의 `bt_manifest.json`에 부모 관계, 순서, 타입, 포트와 최종 XML SHA256을 기록합니다. 이 경로의 기본 NavigateToPose 트리만 지원하며 SubTree/include 확장은 지원하지 않습니다.

`nav2_bt_trace`는 BehaviorTreeLog, navigator lifecycle, NavigateToPose 상태를 구독하고 `/navigation/bt_trace` (`std_msgs/String`, reliable/transient-local)에 4Hz 전체 스냅샷을 발행합니다. 최대 256개 노드와 128개 최근 전이를 유지합니다. 새 목표·lifecycle 전환·ROS 시계 역행에서 초기화하며 미관측 상태는 UNKNOWN입니다. 전이의 previous/current 불일치는 누락 가능성으로 표시합니다. 이벤트에 sequence/goal UUID가 없으므로 모든 누락을 검출하거나 목표 소속을 증명할 수는 없습니다. 목표 ID는 수신 시점 문맥입니다. 외부 XML의 알 수 없는 노드는 구조 불일치로 표시합니다.


### Voxel world 공유와 관측 만료

`config/obstacle_memory.yaml`의 `shared_voxel_costmap: true`는 기존 센서별
voxel world를 local/global costmap과 serial 3D 충돌 검증에 공유하는 실험 설정이다.
LiDAR obstacle layer와 정적 지도, inflation, 기존 Collision Monitor는 유지하며,
중복 depth VoxelLayer는 costmap plugin 목록에서 제외한다. `false`로 되돌리면
기존 depth VoxelLayer와 `/obstacle_memory/grid` 결합 경로를 사용한다.
serial guard는 어느 모드에서도 빠른 local_voxels를 사용한다.
높이 저장 범위는 기존 바닥 위 0.05~2m를 유지한다. 투영 높이는 robot 설정의
`depth_layer.min_obstacle_height/max_obstacle_height`를 따른다.

- `/obstacle_memory/navigation_grid`: 5Hz 주행용 높이 투영. 만료된 장애물 위치는
  내부적으로 미확인이지만 Nav2에는 재관측 전까지 occupied로 전달한다.
  unknown을 무시하는 StaticLayer 결합에서 빈 통로로 바뀌는 것을 방지한다.
- `/obstacle_memory/local_voxels`: 5Hz 로봇 주변 3D 점군. 점유 voxel과 만료된
  미확인 voxel을 함께 전달하고 serial guard가 기존 제동 sweep으로 검사한다.
  cloud/status 시각과 주변 범위가 맞지 않거나 sweep을 포함하지 못하면 정지한다.
- `/obstacle_memory/voxels`, `/grid`, `/lidar`, `/depth`: 전체 지도 진단은
  `world_publish_period`(기본 1초)마다 발행한다. 전체 높이는 보존한다.
- `near_radius`(3m) 안은 `update_period`(0.2초)로 처리하고, 그 밖은
  `far_update_period`(1초)로 처리한다. 먼 점의 광선도 가까운 구간에서는 계속
  clearing에 사용한다. 로봇 자세 TF와 기존 센서 신선도 제한은 유지한다.
- `memory_ttl_s`(300초)는 ROS 시간 기준 voxel별 마지막 **장애물 관측**의
  만료 시간이다. 0이면 만료를 끈다. 한 센서의 만료/clearing은 다른 센서의
  유효한 증거나 미확인 증거를 지우지 않는다. 미확인은 같은 센서의 실제 광선이
  해당 voxel을 통과하거나 다시 장애물을 관측해야 해제된다. 정적 지도는 만료하지 않는다.
- snapshot schema 2는 schema 1도 읽는다. 재시작 시 과거 관측은 미확인으로
  복원하고 현재 시각으로 갱신하지 않는다. ROS clock 역행 시 mapping을 차단한다.

거리, 주기, TTL은 하드웨어에서 확정한 안전값이 아닌 검증용 설정이다.
만료는 통로를 자동으로 열어주는 정책이 아니며, 재관측할 수 없는 장애물의 위치는
계속 차단될 수 있다. RAM의 기본 dense 배열은 유지하고, voxel별 관측 시각과
미확인 상태는 점유/만료 voxel의 sparse 자료로 추가 관리한다.

검증: `make test-navigation` 및 `make test-navigation-runtime`.
Gazebo 43/13 왕복, CPU/RSS, 실제 RTF, 센서 나이, 도착 오차와 최종 속도는 별도
실행 결과로 확인한다. core 처리 시간 측정은 전체 스택/실물 성능을 보증하지 않는다.
