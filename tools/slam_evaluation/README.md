# SLAM evaluation

Gazebo study-cafe에서 LiDAR 높이, 2D SLAM 알고리즘, 가구 변화
localization을 반복 비교하는 도구입니다. 모든 명령은 저장소
루트에서 실행합니다.

Simulator의 일반 실행·topic·sensor profile은
[`cleany_gazebo_sim` README](../../ros2_ws/src/cleany_gazebo_sim/README.md)를
참고합니다.

## Prerequisites

```bash
source /opt/ros/humble/setup.bash
make build-gazebo
source ros2_ws/install/setup.bash
```

실험 생성물은 `ros2_ws/slam_results/`에 저장되며 커밋하지
않습니다. 기록 script는 기존 input이나 미완료 output을 덮어쓰지
않습니다.

## Evaluation contract

- 지원 LiDAR 높이: `16p5`, `26`, `45`, `70` cm
- 공통 frame: `lidar_link`
- 공통 input: `/scan`, `/odom`, `/tf_static`, `/clock`
- IMU algorithm input: `/imu/data`
- 평가 전용 reference: `/ground_truth/odom`
- 공통 경로: 17-waypoint study-cafe closed loop
- LiDAR noise: mean 0 m, measured stddev 0.0025 m, stress stddev 0.01 m
- 비교 후보: slam_toolbox, Cartographer 2D, Cartographer 2D + IMU,
  RTAB-Map 2D

Ground truth는 입력 bag에 보존하지만 SLAM node input으로 연결하지
않습니다. 모든 알고리즘은 동일한 높이별 bag을 replay해 주행
편차를 제거합니다.

## Interactive mapping

Gazebo와 `slam_toolbox` mapping을 각각 실행합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false lidar_profile:=floor_26cm \
  bridge_config:=ros2_ws/src/cleany_gazebo_sim/config/bridge/navigation_bridge.yaml
```

```bash
ros2 launch cleany_navigation slam_mapping.launch.py use_sim_time:=true
```

반복 구조에서 loop closure 영향을 분리할 때는 baseline YAML을
바꾸지 않고 launch override를 사용합니다.

```bash
ros2 launch cleany_navigation slam_mapping.launch.py \
  use_sim_time:=true do_loop_closing:=false
```

지도와 scan을 함께 표시합니다. ARM Adreno Mesa에서는 RViz map
shader 대신 occupancy-grid marker fallback을 사용합니다.

```bash
ros2 launch cleany_gazebo_sim evaluation_slam_visualization.launch.py
```

## Record common input bags

실험용 Jazzy + Harmonic 환경에서 높이·LiDAR noise profile별
study-cafe 경로를 주행하고 비교용 bag을 기록합니다. Physics는 2 ms
timestep과 목표 RTF 2.0을 사용합니다. Script는 LiDAR frame, Gazebo
process, recorder, route 완료 여부를 검사합니다. 제품 edge runtime의
Humble 기준은 변경하지 않습니다. Jazzy가 기록한 SQLite bag의 QoS metadata는
원본을 `metadata.jazzy.yaml`로 보존하고 Humble 호환 metadata v5로 변환합니다.
OGRE2 장시간 sensor rendering이 중단되는 환경에서는 각 route edge를
별도 Gazebo process로 기록하고 simulation stamp를 보정해 병합합니다.

```bash
for noise in measured stress; do
  for height in 16p5 45; do
    GAZEBO_PROFILE=harmonic \
      ./tools/slam_evaluation/record_slam_input.sh "$height" "$noise"
  done
done
```

```bash
./tools/slam_evaluation/record_segmented_slam_input.sh 16p5 measured
```

입력은 다음 경로에 생성됩니다.

```text
ros2_ws/slam_results/algorithm_compare_inputs/<noise>/input_<height>cm_trial1/
```

## Run algorithm comparison

기본 실험 행렬인 noise 2종×알고리즘 2종×높이 2종, 총 8개 조합을
2.5배속으로 replay합니다.

```bash
./tools/slam_evaluation/run_slam_algorithm_comparison.sh
```

알고리즘, 높이, LiDAR noise profile 하나씩 선택할 수 있습니다.

```bash
./tools/slam_evaluation/run_slam_algorithm_comparison.sh \
  cartographer 16p5 measured
```

Replay 배속은 `SLAM_REPLAY_RATE`로 변경합니다.

```bash
SLAM_REPLAY_RATE=1.0 \
  ./tools/slam_evaluation/run_slam_algorithm_comparison.sh \
    slam_toolbox 26 measured
```

Run output은 다음 경로에 생성됩니다.

```text
ros2_ws/slam_results/algorithm_compare_runs/<noise>/<algorithm>/<height>cm/
```

공통 생성물은 `map_final.pgm/.png/.yaml`, `result_bag/`, log,
`resource_usage.txt`입니다. 알고리즘 고유 생성물은 다음과 같습니다.

- slam_toolbox: `.posegraph`, `.data`
- Cartographer: `.pbstream`
- RTAB-Map: `.db`

## Analyze results

ATE는 scale을 고정한 SE(2) rigid alignment 후 계산하고, RPE는 1초
간격 상대 pose로 계산합니다. 상면 overlay는 전용 top-view camera와
map YAML의 resolution/origin을 사용해 world coordinate로 투영합니다.

```bash
python3 tools/slam_evaluation/analyze_slam_algorithm_comparison.py
python3 tools/slam_evaluation/capture_gazebo_top_view.py
python3 tools/slam_evaluation/render_slam_algorithm_overlays.py
```

높이 후보의 주요 비교 지표는 다음과 같습니다.

- ATE translation RMSE
- translation / rotation RPE RMSE
- map coverage와 구조 왜곡
- valid scan ratio와 scan rate
- real-time factor와 resource usage
- 사각, 가림, 벽 왜곡, loop closure 정성 관찰

## Moved-chair localization

가구 변화에 대한 fixed-map localization 강건성을 16.5 cm와 26 cm
LiDAR에서 평가합니다. 의자 12개를 책상 방향으로 0.20 m 이동하고
교대로 ±10° 회전한 world를 사용합니다.

```bash
./tools/slam_evaluation/record_chair_shift_localization_inputs.sh
./tools/slam_evaluation/run_chair_shift_localization.sh
python3 tools/slam_evaluation/analyze_chair_shift_localization.py
```

기준 배치에서 계산한 map-to-world alignment를 이동 배치에도 그대로
적용해 전역 localization 오차를 숨기지 않습니다. 결과는 다음
경로에 생성됩니다.

```text
ros2_ws/slam_results/chair_shift_localization/
```

## Validate experiment contracts

SLAM 평가 CLI, world materialization, study-cafe geometry, route 계약을
검증합니다.

```bash
make test-gazebo-evaluation
```
