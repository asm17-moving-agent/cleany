# cleany_perception

동기화된 RGB-D snapshot에서 Gemini 2D bbox를 한 번 검출하고, 사용자가 선택한 객체
하나만 SAM2와 3D reconstruction으로 정밀 검사하는 package다. Perception은 객체와 위치
후보만 제공하며 수거·보관 등 최종 행동을 결정하지 않는다.

## 처리 경계

```text
aligned RGB-D → capture-time TF → Gemini bbox + 번호
→ bounded snapshot cache → 사용자 선택
→ selected bbox만 SAM2 → support plane → base_link 3D OBB
```

순수 NumPy core는 ROS, Gemini, SAM2와 MuJoCo를 import하지 않는다. `DetectorPort`,
`SegmenterPort`, `TransformPort` 뒤의 adapter를 교체하면 detector와 segmenter에
독립적으로 기하 계산을 재사용할 수 있다. 시뮬레이션 GT topic은 입력으로 사용하지
않는다.

## 모델 준비

workspace dependency와 별도 SAM2 설치는 `docs/DEVELOPMENT_SETUP.md`를 따른다.

- Gemini API key: `GEMINI_API_KEY` 환경변수
- Gemini model ID: `gemini_model` parameter
- Jetson 검증 SDK: `google-genai==2.18.0`
- SAM2 model config/checkpoint/device: launch argument 또는 parameter
- API key, checkpoint와 model weight는 commit하지 않는다.

1차 detector-only action은 SAM2와 checkpoint를 로드하거나 호출하지 않는다. Gemini SDK
또는 API key가 없으면 `ERROR_DETECTOR_API`를 반환한다. 2차 선택 요청에서 처음으로 SAM2를
lazy load하며 dependency 또는 checkpoint 문제가 있으면 `ERROR_MASK`를 반환한다.

기본 detector는 `gemini-robotics-er-2-preview`다. Robotics ER 계열은 공식
Interactions API와 업로드된 RGB snapshot을 사용하고, 요청이 끝나면 원격 임시 파일을
삭제한다. 그 외 Gemini model ID는 기존 `generateContent` 경로를 사용한다. Robotics ER
API는 제한이 설정된 API key가 필요할 수 있다.

## 실행

1차 detector-only 단계만 확인할 때는 SAM2 인자가 필요 없다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_perception inspect_scene.launch.py
```

2차 selected-object 단계까지 실행할 때는 SAM2를 함께 설정한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_perception inspect_scene.launch.py \
  sam2_model_config:=configs/sam2.1/sam2.1_hiera_s.yaml \
  sam2_checkpoint:=/absolute/path/to/sam2.1_hiera_small.pt \
  sam2_device:=cpu
```

입력 topic 기본값:

- `camera/color/image_raw`: `rgb8` 또는 `bgr8`
- `camera/color/camera_info`
- `camera/depth/image_raw`: meter `32FC1` 또는 scale parameter를 적용한 `16UC1`
- `camera/depth/camera_info`

네 메시지는 exact timestamp로 조립한다. RGB와 depth의 해상도와 intrinsics가 일치해야
한다. 해당 timestamp의 `target_frame <- depth optical frame` TF는 snapshot 직후 Gemini
전에 확보하고 RGB-D 및 detections와 함께 cache에 보관한다. 기본 cache는 최근 2개
snapshot을 120초 동안 유지하며 parameter로 조정한다.

### Jetson D435 입력 관문

Jetson의 native ROS 2 Humble에서 D435를 color/depth `640x480x15`로 실행하고 depth를
color optical frame에 정렬한다.

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true
```

실제 wrapper topic은 `/camera/camera/color/*`와
`/camera/camera/aligned_depth_to_color/*`이다. 다음 명령은 네 topic의 exact timestamp,
해상도, optical frame, intrinsics, depth scale과 유효 depth를 5분 동안 검사한다. 기본
최대 공백 2초는 `snapshot_timeout_seconds`와 같다.

```bash
python3 tools/realsense_rgbd_check.py \
  --duration 300 \
  --output /tmp/realsense-rgbd-5m.json
```

hardware config에서는 위 네 실제 topic을 perception 입력으로 remap하고, handheld
검증의 `target_frame`은 검사 결과의 aligned-depth optical frame을 사용한다.

## ROS API

Action:

```text
perception/inspect_scene  cleany_interfaces/action/InspectScene
```

빈 query는 `default_query` parameter를 사용한다. 1차 요청은 `snapshot_id=''`,
`selected_object_id=0`이며, 동시에 하나의 goal만 실행한다. detection이 없으면 빈 2D
detection 배열로 성공한다. 2차 요청은 1차 결과의 두 값을 함께 전달한다.

`success`는 action 처리 상태이며 detection 존재 여부와는 별개다. `success: true`여도
요청한 물체가 RGB frame에 없으면 `detections`는 빈 배열이다.

```bash
ros2 action list
ros2 action info /perception/inspect_scene
ros2 action send_goal \
  /perception/inspect_scene \
  cleany_interfaces/action/InspectScene \
  "{query: 'Detect the box and can on the table.', snapshot_id: '', selected_object_id: 0}" \
  --feedback
```

2차 요청 예시:

```bash
ros2 action send_goal \
  /perception/inspect_scene \
  cleany_interfaces/action/InspectScene \
  "{query: '', snapshot_id: 'rgbd-...', selected_object_id: 2}" \
  --feedback
```

성공 시 같은 snapshot을 다음 topic에도 발행한다.

- `perception/detections_2d`: 번호가 부여된 `DetectedObject2DArray`
- `perception/objects`: 후속 선택 단계에서 사용할 `DetectedObject3DArray` topic

선택 객체 inspection action 결과에는 grasp 입력용 `target_cloud`와
`context_cloud`도 포함한다. 두 `PointCloud2`는 capture RGB optical frame/timestamp와
`x`, `y`, `z`, packed `rgb` field를 공유한다. target은 SAM2 mask 내부이고 context는
선택 bbox 주변 crop이며 `grasp_cloud_voxel_size_m`와 각각의 최대 점 개수 parameter로
payload를 제한한다.
- `perception/debug_image`: rqt용 `BEST_EFFORT`, `VOLATILE` debug image
- `perception/debug_image_latched`: 마지막 결과를 보관하는 `RELIABLE`,
  `TRANSIENT_LOCAL` debug image

debug image는 각 단계가 성공했을 때 생성된다. 1차 결과는 모든 bbox와 번호, 2차 결과는
선택 bbox와 SAM2 mask를 표시한다. rqt의 큰 best-effort sample 유실과
subscriber discovery 지연을 흡수하기 위해 live topic에는 기본 0.25초 간격으로 총 5회 같은
snapshot을 제한 재발행한다. 횟수와 간격은 `debug_republish_count`와
`debug_republish_period_seconds`로 조정한다. latched topic은 마지막 성공 결과 한 장만
보관한다. detector 또는 SAM2 단계에서 실패하면 이전 결과를 재발행하지 않는다.

```bash
ros2 topic echo /perception/detections_2d --once
ros2 topic echo /perception/debug_image_latched --once --field encoding \
  --qos-reliability reliable --qos-durability transient_local
```

## 실패 처리

- RGB-D timeout: `ERROR_RGBD_TIMEOUT`
- Gemini API/auth/network/timeout: `ERROR_DETECTOR_API`
- JSON/schema/bbox 오류: `ERROR_DETECTOR_RESPONSE`
- 잘못된 selected-object 요청: `ERROR_INVALID_SELECTION`
- 없거나 만료된 snapshot: `ERROR_SNAPSHOT_NOT_FOUND`
- SAM2 dependency/checkpoint/mask 오류: `ERROR_MASK`
- depth encoding, shape 또는 유효 point 부족: `ERROR_DEPTH`
- support plane 실패 또는 base-frame tilt 초과: `ERROR_PLANE`
- capture timestamp TF 실패: `ERROR_TF`
- cancel: `ERROR_CANCELLED`

1차 결과의 `snapshot_id`는 후속 선택 단계가 같은 RGB-D와 촬영 시점 TF를 사용하기 위한
opaque key다. cache 크기와 TTL을 넘긴 snapshot은 후속 단계에서 사용할 수 없다.

## 검증

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to cleany_perception
source install/setup.bash
python3 -m pytest -q src/cleany_perception/test
python3 -m flake8 src/cleany_perception
colcon test --packages-select cleany_perception
colcon test-result --verbose
```

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
