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
`autostart:=true`, `axis_motion:=true`, pan/memory/voxel 모두 활성화다.
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
Single-axis DWB/Omni lattice, 경로 대기 BT와 YieldWait는 기존 동작을 공유한다.
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
자기 관측점을 제거한다. 회전하는 head 부품은 정적 자기 필터에 포함하지 않으며,
몸체에 가린 ray를 free-space로 바꾸지 않는다. `depth_clearance.self_filter_margin_m`의 기본 1mm
여유는 depth/CAD 표면의 수치 오차를 처리한다. 실제 로봇에서는 보정 오차에
맞게 검증해야 한다. 공통 pan gate는
`head_pan/command` Float64를 발행하고 TF를 소유하지 않는다. Backend의 head
actuator adapter와 실제 joint feedback이 필요하다. 공통 노드의 topic/frame은
ROS parameter로 설정 가능하며 TF edge owner는 중복되면 안 된다.

기본 주행은 전진·후진과 제자리 회전이다. 횡이동은 기본 차단한다.
PoseProgressChecker는 XY 0.15m 이동 또는 yaw 0.25rad 회전을 진행으로 인정하며,
20초 동안 둘 다 없으면 실패로 처리한다.
Pan은 방향 정렬·정지·settle·fresh depth를 확인한다. 후진 시 카메라를
뒤쪽(π rad)으로 돌리고 정지 후 새 depth를 받아야 속도를 통과시킨다.
뒤쪽 시야가 확보되었다는 물리 검증은 별도로 필요하며, 후진을 제외하려면
`pan_motion_gate.allow_reverse: false`로 설정한다. LiDAR/depth를 Collision
Monitor와 장애물 메모리에 전달하고, 3D voxel guard는 몸체 OBB와 예측 제동
sweep를 검사한다. 필수 입력 단절, stale/future stamp, TF 누락, localization
jump, voxel freeze 및 계산 시간 초과 시 최종 속도를 0으로 만든다. Pan/voxel의
steady timer는 `/clock`이 멈춘 상황에서도 정지 출력을 유지한다.

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

## C++ navigation plugins

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
`cleany_navigation/config/nav2_axis_motion.yaml`이 공통 설정을 소유한다.
공통 설정은 `axis_translation_angle_deg: 10.0`, Y 범위 ±0.0265m/s,
`axis_allow_lateral: false`로 전후 방향에서 ±10° 대각선 보정을 허용한다.
정지, 제자리 회전, 전후 병진(0°, ±5°, ±10°)을 후보로 평가한다.
순수 횡이동과 병진 중 회전은 허용하지 않는다. 합성 속도와 Y 한계를 함께 검사한다.
`axis_translation_angle_deg: 0.0`이면 기존 전후진 전용 동작으로 복귀한다.
PathHeading은 보정 각도만큼 정렬 허용대를 확장하고 병진 후보의 실제 이동 방향을
평가한다. launch는 같은 제한을 pan gate에 전달한다. 작은 대각선 보정 중 카메라는
전방/후방을 유지하며, 후진 전환의 정지·settle·fresh depth 조건은 동일하다.
측정된 대각선 병진은 하나의 병진 동작으로 취급하고 회전 전환은 기존 정착 대기를 유지한다.
이는 simulation 설정이며 실제 카메라 가림과 횡방향 slip/제동 검증은 별도로 필요하다.
`cleany_navigation::StraightFirstPlanner`는 시작/목표의 제자리 회전과 직선 이동을
map 해상도의 절반 간격으로 검사한다. 현재 global costmap의 padded footprint 경계와
내부에 lethal/unknown 셀이 있거나 중심이 inscribed 셀에 들어가면 직선 계획을 거부하고
Smac으로 우회한다. 뒤쪽 목표도 후진 몸체 방향을 보존하도록 Smac에 위임한다.
검사 도중 취소도 처리한다. 안전 노드는 기존대로 최종 속도를 검사한다.
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
backend의 설정은 테스트 fixture이며 실물 calibration이 아니다. Gazebo sensor 회귀
테스트는 backend의 sensor/odom/TF 계약을 확인한다. Harmonic pipeline test는
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
안의 점만 제공합니다. VoxelLayer는 전자를 clearing 전용, 후자를 marking 전용으로
사용합니다. 기본 시뮬레이션 바닥은 z=-0.38 m이고 marking 최소 높이는 -0.33 m입니다.
CAD 카메라 주행 자세는 수평에서 아래로 20°입니다. 실물 높이 기준은 로봇 parameter로
맞춰야 합니다.

BT는 100ms 주기로 앞 1.5m 경로를 검사하며, FollowPath에는 원본 경로를 전달한다.
이동에 따른 앞부분 제거는 DWB가 수행하므로 매 pruning마다 새 목표/정지 이력을 만들지 않는다.
경로 실패 시 최대 15초 기다린 후 재계획하는 정책은 유지한다.

Depth는 최대 `tf_wait_timeout_s`(기본 0.15초) 동안 최신 한 프레임만 보관하며
비동기 재시도로 원래 시각의 TF를 기다린다. TF를 추정하거나 latest TF로 대체하지 않고,
대기 만료 시 버린다. marking은 obstacle_points, clearing은 전체 points를 사용한다.

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
