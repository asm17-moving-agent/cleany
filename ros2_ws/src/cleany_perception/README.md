# cleany_perception

동기화된 RGB-D snapshot에서 Gemini bbox, SAM2 mask와 3D OBB를 계산하는 one-shot
inspection package다. Perception은 객체와 위치 후보만 제공하며 수거·보관 등 최종 행동을
결정하지 않는다.

## 처리 경계

```text
aligned RGB-D → Gemini bbox → SAM2 mask → support plane
→ optical-frame OBB → capture-time TF → base_link OBB
```

순수 NumPy core는 ROS, Gemini, SAM2와 MuJoCo를 import하지 않는다. `DetectorPort`,
`SegmenterPort`, `TransformPort` 뒤의 adapter를 교체하면 detector와 segmenter에
독립적으로 기하 계산을 재사용할 수 있다. 시뮬레이션 GT topic은 입력으로 사용하지
않는다.

## 모델 준비

workspace dependency와 별도 SAM2 설치는 `docs/DEVELOPMENT_SETUP.md`를 따른다.

- Gemini API key: `GEMINI_API_KEY` 환경변수
- Gemini model ID: `gemini_model` parameter
- SAM2 model config/checkpoint/device: launch argument 또는 parameter
- API key, checkpoint와 model weight는 commit하지 않는다.

SAM2와 checkpoint가 없어도 package build와 mock/core test는 실행된다. 실제 action에서
dependency 또는 checkpoint가 없으면 `ERROR_MASK`를 반환한다. Gemini SDK 또는 API key가
없으면 `ERROR_DETECTOR_API`를 반환한다.

기본 detector는 `gemini-robotics-er-2-preview`다. Robotics ER 계열은 공식
Interactions API와 업로드된 RGB snapshot을 사용하고, 요청이 끝나면 원격 임시 파일을
삭제한다. 그 외 Gemini model ID는 기존 `generateContent` 경로를 사용한다. Robotics ER
API는 제한이 설정된 API key가 필요할 수 있다.

## 실행

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
한다. 해당 timestamp의 `target_frame <- depth optical frame` TF는 snapshot 직후, Gemini와
SAM2 추론 전에 확보하여 결과가 나올 때까지 보관한다. 따라서 추론 시간이 TF buffer
수명을 초과해도 동일한 촬영 시점의 좌표 변환을 유지한다. `tf_cache_seconds`(기본 60초)는
snapshot 도착 전의 scheduler 지연과 일시적인 부하를 흡수하는 안전 여유다.

## ROS API

Action:

```text
perception/inspect_scene  cleany_interfaces/action/InspectScene
```

빈 query는 `default_query` parameter를 사용한다. 동시에 하나의 goal만 실행하며 이후
goal은 reject한다. detection이 없으면 빈 객체 배열로 성공한다.

```bash
ros2 action list
ros2 action info /perception/inspect_scene
ros2 action send_goal \
  /perception/inspect_scene \
  cleany_interfaces/action/InspectScene \
  "{query: 'Detect the box and can on the table.'}" \
  --feedback
```

성공 시 같은 snapshot을 다음 topic에도 발행한다.

- `perception/objects`: `cleany_interfaces/DetectedObject3DArray`
- `perception/debug_image`: rqt용 `BEST_EFFORT`, `VOLATILE` debug image
- `perception/debug_image_latched`: 마지막 결과를 보관하는 `RELIABLE`,
  `TRANSIENT_LOCAL` debug image

debug image는 pipeline 전체가 성공했을 때 생성된다. rqt의 큰 best-effort sample 유실과
subscriber discovery 지연을 흡수하기 위해 live topic에는 기본 0.25초 간격으로 총 5회 같은
snapshot을 제한 재발행한다. 횟수와 간격은 `debug_republish_count`와
`debug_republish_period_seconds`로 조정한다. latched topic은 마지막 성공 결과 한 장만
보관한다. detector 또는 SAM2 단계에서 실패하면 이전 결과를 재발행하지 않는다.

```bash
ros2 topic echo /perception/objects --once
ros2 topic echo /perception/debug_image_latched --once --field encoding \
  --qos-reliability reliable --qos-durability transient_local
```

## 실패 처리

- RGB-D timeout: `ERROR_RGBD_TIMEOUT`
- Gemini API/auth/network/timeout: `ERROR_DETECTOR_API`
- JSON/schema/bbox 오류: `ERROR_DETECTOR_RESPONSE`
- SAM2 dependency/checkpoint/mask 오류: `ERROR_MASK`
- depth encoding, shape 또는 유효 point 부족: `ERROR_DEPTH`
- support plane 실패 또는 base-frame tilt 초과: `ERROR_PLANE`
- capture timestamp TF 실패: `ERROR_TF`
- cancel: `ERROR_CANCELLED`

일부 객체만 조용히 누락하지 않는다. 한 detection의 mask 또는 3D 복원이 실패하면 해당
snapshot 전체를 실패 처리한다.

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
