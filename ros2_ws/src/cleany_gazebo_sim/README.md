# cleany_gazebo_sim

ROS 2 Humble / Gazebo Fortress 기반의 Cleany mobile-base simulation
backend입니다. `cmd_vel -> odom / TF` 계약과 LiDAR·IMU·camera sensor를
headless 및 GUI 환경에서 검증합니다.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress 6

설치와 renderer 진단은
[`DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따릅니다.

```bash
source /opt/ros/humble/setup.bash
make check-gazebo-env
```

## 빠른 실행

저장소 루트에서 headless simulation을 실행합니다.

```bash
make sim-gazebo
```

48석 study-cafe GUI를 실행합니다. Sensor server는 항상 OGRE2를
사용하고 GUI renderer는 machine별로 선택합니다.

```bash
# 기본 GUI renderer: OGRE1
make sim-gazebo-study-cafe

# OGRE2가 필요한 machine
GAZEBO_GUI_RENDER_ENGINE=ogre2 make sim-gazebo-study-cafe
```

직접 launch할 때는 `gui_render_engine:=ogre|ogre2`를 지정합니다.

```bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false gui_render_engine:=ogre2
```

## ROS interface

| Direction | ROS topic | Type / role |
| --- | --- | --- |
| Input | `/cmd_vel` | 사용자 차체 속도 명령 |
| Internal | `/gazebo_cmd_vel` | guard를 통과한 Gazebo 명령 |
| Output | `/clock` | simulation clock |
| Output | `/odom` | `odom -> base_link` 기준 pose |
| Evaluation | `/ground_truth/odom` | 평가 전용 simulator pose |
| Output | `/joint_states` | 4개 drive wheel joint |
| Output | `/scan` | 360-sample GPU LiDAR |
| Output | `/imu/data` | `imu_link`, 50 Hz simulation IMU |

Camera profile은 head RGB·depth와 좌·우 wrist RGB를 다음 topic으로
발행합니다.

- `/camera/head/color/image_raw`
- `/camera/head/depth/image_raw`
- `/camera/left_wrist/color/image_raw`
- `/camera/right_wrist/color/image_raw`

### TF ownership

- `gazebo_odom_tf_publisher`: `odom -> base_link`
- `gazebo_sensor_tf_publisher`: `base_link -> lidar_link / imu_link`
- Camera optical frame: REP-103 `_optical_frame`

Stock Fortress `MecanumDrive` 플러그인은 odometry message를 발행하지
않습니다. 따라서 Fortress profile은 `OdometryPublisher`의 ground-truth
출력을 `/gazebo_odom`과 `/ground_truth/odom`에 동시에 bridge합니다.
현재 `/odom`은 wheel drift나 slip이 반영된 odometry가 아닙니다.

## Sensor profiles

`sensor_profile` launch argument로 rendering sensor 부하를 선택합니다.
차체, clock, odometry, joint state, IMU bridge는 모든 profile에서 실행됩니다.

| Profile | LiDAR | Head RGB | Head depth | Left wrist | Right wrist |
| --- | --- | --- | --- | --- | --- |
| `lidar_nav` (기본값) | O | X | X | X | X |
| `head_rgbd` | X | O | O | X | X |
| `left_wrist` | X | X | X | O | X |
| `right_wrist` | X | X | X | X | O |
| `all_cameras` | X | O | O | O | O |

```bash
ros2 launch cleany_gazebo_sim gazebo_fortress.launch.py \
  headless:=true sensor_profile:=head_rgbd
```

`bridge_config` launch argument를 지정하면 sensor profile 대신 해당 bridge
설정 하나를 사용합니다. 선택되지 않은 rendering sensor는
`always_on=false`와 bridge subscriber 부재로 lazy 상태를 유지합니다.

## Study-cafe world

`gazebo_study_cafe.launch.py`는 12.26×10.94 m, 48석 study-cafe 평가
공간을 생성합니다. 로봇 spawn, 방 크기, 책상·의자 배치는
`config/study_cafe/study_cafe_layout.yaml`이 관리하며 생성된 world는
`/tmp/cleany_study_cafe.sdf`에 기록됩니다.

LiDAR 높이는 `lidar_profile` argument로 선택합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false lidar_profile:=floor_26cm
```

World의 반복 가구는 primitive collision을 사용하고, 로봇 visual과
LiDAR에 별도 visibility mask를 적용해 self-hit를 방지합니다.
의자 visual은 OpenRobotics Gazebo Fuel `OfficeChairGrey` (CC BY 4.0)를
사용하며 최초 실행 시 network가 필요할 수 있습니다.

## SLAM evaluation

LiDAR 높이별 bag 기록, slam_toolbox·Cartographer·RTAB-Map 비교,
의자 이동 localization 실험은
[`tools/slam_evaluation/README.md`](../../../tools/slam_evaluation/README.md)를
참고합니다. 실험 생성물은 `ros2_ws/slam_results/`에 저장하며
커밋하지 않습니다.

## Validation

정적 world·bridge·TF·command guard 계약을 검증합니다.

```bash
make test-gazebo
```

SLAM 실험과 study-cafe 형상 계약을 검증합니다.

```bash
make test-gazebo-evaluation
```

LiDAR·IMU·odometry·TF와 차체 구동을 실제 Fortress runtime에서
검증합니다.

```bash
make test-gazebo-nav-runtime
```

모든 camera stream의 해상도·encoding·frame·timestamp와 실제 image 변화를
검증하려면 opt-in runtime test를 실행합니다.

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_rendering_sensors.py \
  --run-sim-runtime --sim-profile=fortress \
  --sensor-profile=all_cameras
```

## Configuration map

- `worlds/cleany_mecanum_fortress.sdf`: robot, physics, sensor system
- `config/base.yaml`: command guard와 TF publisher parameter
- `config/bridge/`: Gazebo transport / ROS bridge
- `config/lidar_mount_profiles.yaml`: LiDAR 높이 후보
- `config/study_cafe/`: study-cafe layout과 평가 route
- `launch/gazebo_fortress.launch.py`: core Fortress backend
- `launch/gazebo_study_cafe.launch.py`: study-cafe scenario
- `launch/evaluation_*.launch.py`: SLAM replay·visualization·route 평가
- `test/evaluation/`: 실험 전용 계약 테스트

## Known limitations

- Mobile base 주행에 초점을 두며 arm controller와 manipulation은 포함하지
  않습니다. 양팔은 접은 대기 자세로 고정됩니다.
- Mecanum roller는 visual로만 표현하고 contact는 anisotropic friction으로
  단순화합니다.
- Simulation IMU에 stochastic noise와 bias model이 없습니다.
- Fortress odometry fallback은 wheel drift와 slip을 모사하지 않습니다.
- GPU LiDAR와 camera는 headless 실행에서도 OpenGL rendering context가
  필요합니다.

## 경로 접근 속도 설정

`ground_truth_route_follower`의 `position_gain` parameter는 목표점까지 남은
거리와 곱해 접근 속도를 정한다. 기본값 1.0은 기존 동작을 유지하며 낮은 값은
최대 속도 제한 안에서 목표점에 더 천천히 접근하게 한다. 양의 유한값만 허용한다.

## 전체 18층 시설과 ROLY 의자

사용자가 확인한 [시설 기준 도면](../../../docs/assets/facility/facility-18f-reference.png)을
바탕으로 전체 벽체·열린 출입구·둥근 모서리와 82석을 생성한다. D-HUB는 기존
48석 좌표를 유지하고 ROLY P1G210M 참고 모델을 사용한다. A1–A4는 각 4석,
M1–M3는 각 6석이며 이 34석에는 기존 OfficeChairGrey 모델을 사용한다.
A/M 의자가 같은 제품이라는 근거는 없다.

```bash
# 저장소 루트, Ubuntu 22.04 / ROS 2 Humble 환경
make sim-gazebo-facility GAZEBO_GUI_RENDER_ENGINE=ogre2

# 빌드와 overlay source 이후
ros2 launch cleany_gazebo_sim gazebo_facility_18f.launch.py \
  headless:=false gui_render_engine:=ogre2 lidar_profile:=floor_26cm
# 기존 의자로 비교
ros2 launch cleany_gazebo_sim gazebo_facility_18f.launch.py chair_model:=legacy
# D-HUB 단독에서도 새 의자를 비교할 수 있다.
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py chair_model:=roly
```

- `facility_layout_config`: `config/facility_18f/facility_layout.yaml`.
  대시보드 map 좌표의 벽 선분·곡선·출입구·A/M 가구와 검증 동선을 관리한다.
- `layout_config`: 기존 D-HUB 방·가구 설정. `chair_model`은 `roly|legacy`,
  `chair_config`는 `config/furniture/roly_p1g210m.yaml`이다.
- `world_output`: 기본 `/tmp/cleany_facility_18f/world.sdf`. 옆에 `.seats.json`
  좌석 ID/모델/pose 목록, sensor TF YAML과 생성용 SDF가 생긴다.
  `roly_meshes/`에는 smooth normal·UV를 포함한 재질별 COLLADA mesh 6개를 생성한다.
  모든 D-HUB 의자가 같은 asset을 공유하며 설정·소스 hash로 캐시를 구분한다.
  SDF를 복사할 때 mesh 경로도 함께 유지해야 한다. 소스 asset은 수정하지 않는다.
- `run_route:=true`: ground-truth 기반 제한된 검증 이동을 시작한다. 기본은 false다.
  D-HUB 서쪽 출입구→공용 복도→M3 입구→THE GROND→A2 입구를 따라간다.
  속도·오차 기준은 시설 YAML의 `route_control`로 관리한다.
  좁은 출입구에서 먼저 방향을 맞추고 목표 근처에서는 감속한다.
  이 옵션은 Nav2 장애물 회피나 실제 navigation 정확도 검증이 아니다.

### 좌표와 모델 가정

D-HUB 중심이 world 원점이며 오른쪽이 +X, 도면 위쪽이 +Y다.
`map_x = 776 + world_x * (400/12.26)`,
`map_y = 8 + (5.47-world_y) * (400/12.26)`를 사용한다.
D-HUB의 내부 치수 12.26×10.94 m와 기존 가구 pose를 유지하기 위해 해당 외곽
벽은 기준선에서 바깥쪽으로 벽 두께의 절반만큼 배치한다.

시설 전체 치수·문 폭·A/M 가구 치수는 도면과 기존 D-HUB 치수에서 추정했다.
시설 전체 바닥은 회색 펠트 질감으로 통일했다. 로컬 반복 texture와 무광 PBR
(roughness 1, metalness 0)을 사용하며 `floor.texture_tile_m`으로 질감 크기를
조절한다. 색상·섬유 크기는 시각적 근사이고 마찰·탄성 실측값은 반영하지 않았다.
기존 바닥 collision과 마찰 설정은 유지한다. 생성 SDF 옆의 `.floor.dae`도
월드와 함께 보존해야 한다.
벽 높이 2.5 m, 두께 0.16 m는 시뮬레이션 기본값이며 현장 실측값이 아니다.
대시보드의 회색 구역도 형상에는 포함하지만 초기 평가 동선은 흰색 운영 구역이다.
회색 표시를 collision으로 만들지는 않는다. 계단 시작에는 갈색의 명시적
simulation guard를 배치하며 계단 하강·엘리베이터 운행은 모델링하지 않는다.

ROLY는 제품 사진을 참고한 정적 근사 모델이다. 색상은 대표 사진의 흰 프레임,
회색 등판, 녹색 좌판이며 시설 설치 색상은 미확인이다. 공식 카탈로그의 전체 치수와 좌고 범위를 YAML에 참고값으로 기록했으며,
부품별 치수는 `photo_estimate`로 표시했다. 둥근 좌판, 연속 U자 하부 프레임,
고정 팔걸이, 뒤쪽 요추 패드와 테이퍼 베이스를 곡면으로 재구성했다. 5발 베이스·캐스터·팔걸이에는 개별 collision을
사용하며 기존의 꽉 찬 원판 collision을 사용하지 않는다. 좌판 collision은 box,
곡면 등판 collision은 5개 panel로 근사한다. 등판 메쉬의 광학 투과·반사는
재현하지 않는다. 세부 사항은 [모델 설명](models/roly_p1g210m/README.md)을 참고한다.

Fortress `gpu_lidar`는 **visual geometry**를 관측한다. ROLY visual은 visibility
flag `0x01`로 관측 가능하며 로봇의 기존 self-filter `0x02`는 유지한다.
로컬 직물 texture를 사용하고, 같은 재질의 static 부품은 한 mesh로 합치며 collision은
개별 형상을 유지한다. `/joint_states`의 Gazebo 입력은 world 이름에 독립적인
`/model/cleany_mecanum/joint_state`로 지정했다. ROS topic 이름은 동일하다.

### 재현 가능한 검증

```bash
make test-gazebo
python3 tools/facility/render_layout_overlay.py
# 아래는 저장소 루트에서 ROS overlay를 source한 뒤 실행한다.
python3 tools/facility/validate_runtime.py --scene chair --chair-model roly \
  --output "$PWD/artifacts/facility/roly-26cm"
python3 tools/facility/validate_runtime.py --scene facility --gui --route \
  --timeout 3600 --output "$PWD/artifacts/facility/full-route"
```

`render_layout_overlay.py`는 Pillow와 PyYAML을 사용해 도면 위에 청록색 벽,
초록색 출입구, 갈색 계단 guard, 분홍색 동선을 표시한 SVG를 생성한다.
`headless_rendering:=true`는 headless 서버에서 EGL 렌더링을 선택한다.
기본 launch는 기존 X11 방식을 유지하고 검증 도구의 headless 실행은 EGL을 사용한다.
`validate_runtime.py`는 테스트용 ROS domain과 Gazebo partition에서 simulator를
띄우고 PNG·scan·이동 궤적·RTF·로그를 출력 디렉터리에 저장한 후 종료한다.
`--physics-step` 기본값은 0.001 s이며 변경 시 결과 JSON에도 기록한다.
이 기본값으로 먼저 확인한다. 가속 스텝은 동일한 물리 안정성을 보장하지 않는다.
`--lidar-height`는 0.165, 0.26, 0.45, 0.70 m를 지원하고
`--chair-model legacy`로 같은 조건의 비교 결과를 얻을 수 있다.
실행에는 runtime capture용 Pillow가 필요하다. 센서 이미지 확인과 시뮬레이션
이동 성공은 실제 시설·실물 의자 검증과 구분한다.

### 2026-10-02 확인 결과

- Humble Distrobox에서 관련 3개 package build 성공. `make test-gazebo`는
  50 passed, 2 skipped이며 경로 제어 evaluation 테스트 4개도 통과했다.
  기본 suite에서 제외되는 두 runtime 항목과 별도로 위 capture 도구를 실행했다.
- 전체 시설의 실제 Gazebo 상부 화면과 기준 도면 overlay를 비교했다.
  ROLY 정면·측면·상부 화면, 16.5/26/45/70 cm LiDAR scan을 확보했다.
  단품 26 cm에서 기존/ROLY 모두 중앙 기둥을 관측하므로 바닥 collision 원판
  제거만으로 해당 높이의 scan이 개선됐다고 해석하지 않는다.
- 기본 1 ms 스텝, 26 cm LiDAR, 상부 camera, 8.143초 simulation 구간에서
  전체 시설의 legacy/ROLY RTF는 각각 약 0.24/0.15였다. 두 경우 scan 주기는
  simulation 시간 기준 약 5.53 Hz였다. 다른 시뮬레이션이 실행 중인 호스트의
  짧은 관측이므로 통제된 성능 benchmark는 아니다. 현재 실시간 속도를 달성하지
  못하며, 재질별 visual 병합 후에도 상세 collision의 비용이 남는다.
- 3 ms 가속 스텝 주행은 D-HUB→공용 복도→M3 입구까지 도달했다.
  simulation 223.35초 후 clock/scan이 멈춰 중단했다. 전체 경로 완주와
  기본 1 ms의 장시간 주행 안정성은 미검증이다. 가속 스텝을 기본값으로 바꾸지 않았다.
- SLAM map 생성·Nav2 주행·실물 센서 비교는 수행하지 않았다.

원시 근거는 저장소 루트 `artifacts/facility/`의 `tests-final.log`,
`layout_overlay.svg`, `final-roly-*/`, `final-legacy-0.26/`,
`performance-{legacy,roly}/`, `route-conservative/`에 있다.
각 runtime 디렉터리는 PNG, `result.json`, `runtime.log`를 포함한다.

같은 날짜의 ROLY 재설계 후에도 build와 50개 테스트가 통과했다(선택 항목 2개 제외).
`roly-redesign-final/`에 5개 시점과 26 cm scan,
`roly-redesign-70cm/`에 70 cm scan, `roly-redesign-facility/`에
전체 시설 렌더링과 sensor 발행 근거를 저장했다. 위의 장시간 경로·성능 수치는
재설계 전 모델의 기록이며 새 모델의 완주 결과로 해석하지 않는다.

### 시설 시뮬레이터의 CAD 로봇

전체 시설 launch의 `robot_model` 기본값은 `cad_frame`이다. `feat-navigation-runtime`
커밋 `71f8d6c`의 canonical `cleany_description` CAD mesh와 URDF를 사용한다.
기존 D-HUB launch의 기본값은 `legacy`이며 시설에서도 `robot_model:=legacy`로
이전 모델을 선택할 수 있다. 모델의 출처, 외형을 포함한 collision bounds와
카메라 변환은 생성된 world 옆 `.model.json`에 기록한다.

Fortress 어댑터는 URDF의 팔을 설정된 주차 자세로 고정하고, head pan과 4개
메카넘 구동 관절을 유지한다. 주차 자세의 wrist limit override는
`config/cad_frame.yaml`의 시뮬레이션 설정이며 실제 로봇 URDF limit을 변경하지 않는다.
LiDAR, IMU, 손목/헤드 카메라와 기존 구동·odometry topic을 연결한다.
URDF 변환 helper는 이 패키지 내부에 두어 navigation 패키지 전체의 변경 없이
CAD 모델을 사용할 수 있게 했다.

CAD 시설에서 package build, 렌더링, LiDAR·odometry·관절 상태 발행을 확인했다.
근거는 `artifacts/facility/cad-facility/`에 있다. 새 모델의 폭과 충돌 형상이
달라졌으므로 위 이전 모델의 경로 검증은 새 모델의 주행 검증을 대신하지 않는다.

### THE GROUND 테이블·유리 구획·천장

`config/facility_18f/facility_layout.yaml`에서 중앙 사각형은 벽이 아니라
높이 0.80 m의 통짜 테이블이다. 도면의 평면 범위를 사용하고 바닥부터
상판까지 채운 box visual/collision을 둔다. 계단 부근의 기존 벽과 중복되던
갈색 시뮬레이션용 차단벽 두 개는 제거했다. A1–A4, M1–M3와 THE GROUND 사이의 전면 구획은
유리 패널과 얇은 프레임으로 구성한다. 공간 사이의 내부 칸막이는 기존 벽이다.
해당 유리문은 경첩 기준으로 실내 방향으로 90도 열린 static 모델이며,
`doors[].open_angle_deg`로 생성 시 각도를 바꿀 수 있다. 동적인 개폐 제어는 없다.
유리의 transparency는 화면 표현이며 실제 LiDAR의 유리 투과·반사를 재현하지 않는다.
유리와 프레임에는 collision이 있어 통과 가능한 장애물로 취급하지 않는다.

사용자가 선택한 천장 높이는 2.7 m이며 구획 벽도 같은 높이로 맞췄다.
천장 visual은 아래쪽을 향한 단면 mesh로 만든다. 따라서 천장 위에서 내려다보면
내부가 보이고 아래에서 올려다보면 천장이 보인다. 충돌용 얇은 box는 별도로
유지하며, 상부 조명에 의한 실내 암전을 피하기 위해 천장 그림자는 끈다.
생성된 world 옆 `.ceiling.dae`도 world와 함께 유지해야 한다.

`validate_runtime.py --scene facility`는 상부·THE GROUND·천장 화면을 저장한다.
이번 변경은 build 및 53개 테스트(선택 runtime 2개 제외)와 실제 Fortress
렌더링/scan·odometry·관절 상태 발행으로 확인했다.
근거는 `artifacts/facility/glass-table-ceiling/`이다. 변경 후 전체 동선 주행은 미검증이다.
