# cleany_perception

`enable_wrist_observation=true`는 `/perception/observe_wrist_target`를 추가한다.
기존 YOLOE/SAM2 모델 및 서비스 요청 busy lock을 공유하며 손목별 RGB/CameraInfo를 exact stamp로
짝지어 최신 frame에서 처리한다. 손목 HANDOFF는 head RGB-D에서 전달한 예상 OBB를
capture-time TF로 투영한 box를 SAM2 prompt로 사용한다. label/confidence는 원래
head 검출의 값이며 새로운 손목 YOLOE 검출 결과가 아니다.
`wrist_handoff_require_redetection=true`를 명시하면 같은 label의 단일 YOLOE
검출과 투영 box의 IoU도 요구한다. 기본값은 false다.

`wrist_continuous_tracking=true`(기본)는 HANDOFF의 **검증된 SAM2 mask**로
한 번 초기화한 video state를 백그라운드에서 유지한다. 접근·파지·상승·운반 중
직전 추론이 끝날 때마다 선택 손목의 최신 exact-pair RGB만 처리한다.
입력 FIFO를 쌓지 않으며 카메라 FPS와 추론 FPS는 다르다. 새 RGB만 encoding하고
조건부 seed와 최근 `wrist_streaming_memory_frames=32`개 결과 메모리를 유지한다.
JPEG는 초기화에 한 장만 사용하며 이후 RGB는 메모리에서 정규화한다.
CPU에서는 추적 세션 동안 `wrist_streaming_cpu_threads=4`로 PyTorch intra-op
thread 수를 제한하고 종료 시 이전 값을 복원한다. 이는 perception **프로세스 전체**에
적용되는 PyTorch 설정이며 모델/해상도 변경은 아니다. Ultralytics CPU 초기화가
환경의 OMP thread 설정을 덮어쓸 수 있어 명시적으로 제어한다. 동시에 수행되는
MuJoCo/제어 작업의 CPU 경합을 줄이기 위한 설정이며 GPU 모드에는 적용하지 않는다.
백그라운드 추적과 기존 batch 추적은 adapter의 별도 모델 lock으로 직렬화한다.
CHECK는 과거 영상을 재처리하지 않고 요청의 `after_stamp_ns`(상승 완료) 이후 촬영된 결과를 최대
`wrist_tracking_check_timeout_seconds=12`초 기다린다. 선택 당시 frame age는 기존
0.75초 이하이고, 계산 후 결과 age는 `wrist_maximum_tracking_result_age_seconds=6`초
이하여야 한다(서로 다른 제한). 원본 capture timestamp, reference/source ID,
capture-time TF 투영 및 mask 검사는 보존한다. CLEAR/교체/종료는 해당 worker를
중단하며 실패·오래된 결과를 성공으로 대체하지 않는다.
추론 완료마다 `/perception/wrist_tracking_status` (`WristTrackingStatus`, reliable depth 10)를
발행한다. 원본 촬영 Header와 arm/reference/source/object ID를 보존하며,
mask 면적이 설정 범위 안이면 `valid=true, visible=true`다. 빈/과대 mask는
`visible=false`, worker 오류는 `valid=false`와 reason으로 발행한다(오류에 촬영 시각을
꾸며 넣지 않는다). CLEAR 이후 이전 worker의 결과/오류는 발행하지 않는다.
이는 RGB 면적 기반 존재 신호로 독립 ID/3D 높이/낙하 증거가 아니다.
Sorting의 기본 비동기 carry monitor는 이 topic과 관절 접촉 추정을 이동 중 확인하여
이상 시 active action을 취소하며, 상승 후 CHECK를 요청하지 않는다.
기존 CHECK API와 그 freshness/TF 투영 검증은 비교 모드에서 유지한다.
실시간 visual servo/안전 센서가 아니며 CPU 약 0.2 Hz 추론으로는 즉각 낙하 정지를 보장하지 않는다.

`wrist_continuous_tracking=false`는 비교용 기존 batch 경로다. RGB를 2초 간격으로
최대 32장 보관하고 CHECK 때 seed + 중간 최대 4장 + 현재 frame(총 최대 6장)을
한꺼번에 추적한다. 두 경로 모두 reference box에서 물체를 다시 선택하지 않는다.
Streaming adapter는 설치된 Meta SAM2 commit
`2b90b9f5ceec907a1c18123530e92e794ad901a4`의 video predictor state layout을 사용한다.
공개 append-frame API가 없으므로 이 의존은 `Sam2Stream`에 격리했고 vendor 코드는
수정하지 않는다. SAM2 버전 변경 시 상태/전처리/메모리 보존 계약과 실모델 추적을
다시 검증해야 한다. 고정 FPS나 Jetson 실시간 성능은 보장하지 않는다.
RGB로 metric depth를 만들어 내지 않는다.
손목 frame TF, 유효 시야, timestamp, calibration, reference ID 또는 mask 검증 실패는
성공으로 처리하지 않으며 head RGB-D로 자동 fallback하지 않는다.
`config/inspect_scene.yaml`의 `wrist_*` ROS parameters로 freshness/TTL, confidence,
segmentation score, 투영 시야 및 mask 면적/연관성 한계를 설정한다. 기본 head prior
최대 나이는 60초, 손목 입력 frame 최대 나이는 0.75초다. HANDOFF는 요청 시각,
CHECK는 상승 동작 완료 시각을 `after_stamp_ns`로 사용하며 그보다 새 frame을 요구한다.
손목 마스크는 2D 일관성 확인일 뿐 독립적 물체 ID/3D 위치나 visual-servo 보장이 아니다.

## 필터링 점군 수신 확인

`scene_cloud_receipt_node`는 MoveIt의 `/perception/scene_cloud_filtered`를
받고, 완전한 비어 있지 않은 PointCloud2의 원본 Header만
`/perception/scene_cloud_receipt` (`std_msgs/Header`, RELIABLE depth 1)로
전달한다. 촬영 시각을 현재 시각으로 바꾸거나 timer로 재발행하지 않는다.
기본 입력은 BEST_EFFORT depth 1이며, 입력 손실/중단 시 확인 메시지도 중단된다.
`filtered_cloud_topic`, `receipt_topic`을 ROS parameter로 설정한다.

목적은 인식/action을 기다리는 coordinator에서 큰 점군의 수신·복사를
분리하는 것이다. 점군 좌표나 지도는 바꾸지 않고, mapper에는 기존 전체
점군이 그대로 입력된다. receipt만으로 지도 갱신/안전을 보장하지 않으며
coordinator의 capture-age와 populated OctoMap 수신-age 검사도 필요하다.
이 경로는 센서 전용 파이프라인 launch에서 함께 시작된다.

동기화된 RGB-D snapshot에서 Gemini 또는 YOLOE 2D bbox를 한 번 검출하고, 사용자 또는 외부
coordinator가 선택한 객체 하나만 SAM2와 3D reconstruction으로 정밀 검사하는 package다.
Perception은 객체와 위치
후보만 제공하며 수거·보관 등 최종 행동을 결정하지 않는다.

## 처리 경계

```text
aligned RGB-D → capture-time TF → detector bbox + 번호
→ bbox 중앙 depth로 base_link 거리 산정 및 가까운 순 정렬
→ bounded snapshot cache → 사용자 선택
→ selected bbox만 SAM2 → support plane → base_link 3D OBB
```

순수 NumPy core는 ROS, Gemini, YOLOE, SAM2와 MuJoCo를 import하지 않는다. `DetectorPort`,
`SegmenterPort`, `TransformPort` 뒤의 adapter를 교체하면 detector와 segmenter에
독립적으로 기하 계산을 재사용할 수 있다. 시뮬레이션 GT topic은 입력으로 사용하지
않는다.

## 모델 준비

workspace dependency와 별도 SAM2 설치는 `docs/DEVELOPMENT_SETUP.md`를 따른다.

- Gemini API key: `GEMINI_API_KEY` 환경변수
- Gemini model ID: `gemini_model` parameter
- YOLOE checkpoint/classes/device/text encoder directory: `yoloe_*` parameter
- SAM2 model config/checkpoint/device: launch argument 또는 parameter
- API key, checkpoint와 model weight는 commit하지 않는다.

1차 detector-only action은 SAM2와 checkpoint를 로드하거나 호출하지 않는다. Gemini SDK
또는 API key가 없으면 `ERROR_DETECTOR_API`를 반환한다. 2차 선택 요청에서 처음으로 SAM2를
lazy load하며 dependency 또는 checkpoint 문제가 있으면 `ERROR_MASK`를 반환한다.

기본 detector는 `gemini-robotics-er-2-preview`다. Robotics ER 계열은 공식
Interactions API와 업로드된 RGB snapshot을 사용하고, 요청이 끝나면 원격 임시 파일을
삭제한다. 그 외 Gemini model ID는 기존 `generateContent` 경로를 사용한다. Robotics ER
API는 제한이 설정된 API key가 필요할 수 있다.

YOLOE-26은 Ultralytics `8.4.0` 이상이 필요하다. 공식 배포 weight는 segmentation
checkpoint인 `yoloe-26n-seg.pt`이며 별도 `yoloe-26n-det.pt`는 없다. YOLOE detector
adapter는 이 checkpoint의 bbox/class/confidence만 사용하고 자체 mask는 버린다. 선택된
bbox의 최종 mask는 항상 configured segmenter, 예를 들면 SAM2-tiny가 만든다. Text
prompt 경로는 checkpoint 외에 Ultralytics CLIP package와 `mobileclip2_b.ts`가 필요하다.
`yoloe_text_encoder_directory`는 이 파일이 있는 디렉터리를 가리켜야 하며 model과
encoder는 저장소에 commit하지 않는다.

`DetectorPort`의 RGB 입력은 YOLOE adapter 내부에서 contiguous BGR로 변환한다.
Ultralytics의 NumPy 입력은 BGR로 해석되므로 RGB를 그대로 전달하면 색 채널이 뒤바뀐다.
반환 bbox는 원본 RGB 해상도의 픽셀 좌표다.

## 실행

Jetson에서는 perception을 `172.30.0.11` 전용 GPU service로 실행한다. AnyGrasp의 고정
MAC, license와 checkpoint는 perception container에 mount하지 않는다. 자세한 절차는
[`containers/vision`](../../../containers/vision/README.md)을 따른다. 아래 명령은 native
개발환경용이다.

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

YOLOE-26n bbox와 SAM2.1 tiny를 함께 사용할 때는 다음처럼 실행한다. `yoloe_classes`는
ROS string array이며, 배포 환경의 대상 label에 맞춰 명시한다.

```bash
ros2 launch cleany_perception inspect_scene.launch.py \
  detector_type:=yoloe segmenter_type:=sam2 \
  yoloe_model_path:=/absolute/path/to/yoloe-26n-seg.pt \
  yoloe_text_encoder_directory:=/absolute/path/to/yoloe-model-directory \
  yoloe_classes:="[cup, wallet, crumpled tissue, lego brick]" \
  yoloe_device:=cpu minimum_detection_confidence:=0.05 \
  sam2_model_config:=configs/sam2.1/sam2.1_hiera_t.yaml \
  sam2_checkpoint:=/absolute/path/to/sam2.1_hiera_tiny.pt \
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

1차 detection은 bbox 중앙 영역의 유효 depth 중앙값으로 대표 3D point를 만들고,
촬영 시점 TF를 적용해 configured target frame 원점(기본 `base_link`)까지의 거리를
계산한다. `nearest_object_central_bbox_fraction`과
`nearest_object_minimum_depth_pixels`로 배경 혼입과 depth hole을 제한한다. 결과는
유효 거리 오름차순, confidence 내림차순, detector 원본 순서로 번호를 부여하며,
유효 depth가 부족한 detection은 `distance_valid=false`로 배열 뒤에 남긴다. 자동 조작
coordinator는 `distance_valid=true`인 객체만 가까운 순서로 시도해야 한다.

선택 물체의 grasp용 cloud에서는 3D 복원 때 추정한 지지 평면과의 높이가
5 mm 미만인 점을 제외한다. SAM2 mask 경계에 섞인 책상 점이 jaw 폭과
접근 방향을 왜곡하지 않도록 하는 처리이며, 평면도 같은 depth에서 추정한다.
주변 context cloud와 전체 환경 cloud에서는 지지면을 지우지 않는다.

`simulation_color_profile:=study_cafe`는 과거의 파란 컵/지갑/휴대폰/지우개 장면용
결정적 회귀 fixture다. RGB 색상만 사용하며 simulator pose/ground-truth를 읽지
않지만, 현재 종이컵/레고/휴지 장면과는 호환되지 않는다. 현재 장면은 아래의 YOLOE-s
+ SAM2-tiny 프로필을 사용하며 색상 fallback은 없다. 기본 `legacy` color profile은
기존 box/can 데모와 호환된다.

## ROS API

Action:

```text
perception/inspect_scene  cleany_interfaces/action/InspectScene
```

빈 query는 `default_query` parameter를 사용한다. 1차 요청은 `snapshot_id=''`,
`selected_object_id=0`이며, 동시에 하나의 goal만 실행한다. detection이 없으면 빈 2D
detection 배열로 성공한다. 2차 요청은 1차 결과의 두 값을 함께 전달한다.

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
`context_cloud`도 포함한다. 두 `PointCloud2`는 객체 OBB와 같은 configured target
frame(기본 `base_link`) 및 capture timestamp를 사용하고 `x`, `y`, `z`, packed `rgb`
field를 공유한다. target은 SAM2 mask 내부이고 context는
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

MuJoCo 통합 데모는 `detector_type=simulation_color`,
`segmenter_type=simulation_color`로 시뮬레이션 전용 adapter를 선택한다. 이 adapter는
렌더링된 RGB의 빨강/파랑 픽셀에서 bbox와 mask를 계산할 뿐, 물체 pose나 합성 점을
주입하지 않는다. 거리와 3D geometry는 운영 경로와 동일하게 실제 depth image,
CameraInfo 및 촬영 시점 TF에서 계산한다. 색상 규칙은 통합 검증용이므로 실제 배포
perception 대체물로 사용하지 않는다.

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

## YOLOE-s + SAM2-tiny 공통 실행 프로필

설치되는 `config/fastdds_rgbd.xml`은 Fast DDS 2.6/Humble용 UDP + participant별
16 MiB 공유 메모리 설정이다. 큰 RGB-D/PointCloud2의 동일 호스트 전달 실험에서 사용했다.
Study-cafe 전체 launch가 모든 child에 적용하며, 별도 프로세스들은
`FASTRTPS_DEFAULT_PROFILES_FILE`에 이 파일 경로를 지정한다. `learned_rgbd.launch.py`
단독 실행은 외부 카메라/MoveIt의 middleware 환경을 변경하지 않는다. QoS나 2초 지도
freshness 검증을 완화하는 설정이 아니며 Jetson에서도 무손실이라는 보장은 없다.

공통 프로필은 `enable_reference_observation=true`로
`/perception/observe_object_reference`도 제공한다. 집기 전 YOLOE snapshot/object를
`PIN`으로 고정한 뒤, `OBSERVE`에서 새 동기화 RGB-D를 받아 capture TF를 먼저 확보하고
SAM2 video predictor로 기준/현재 두 영상만 처리한다. 검출 cache(max 2)와 별도로
한 reference만 보존하며 TTL은 120초(단조 시계), 해제는 `CLEAR`다. inspector action과
service는 상호 배타적이며 busy면 실패한다. 일반 inspector 기본값은 비활성화다.

공식 SAM2 video API에 맞춰 두 RGB를 private 임시 디렉터리의 JPEG quality=100,
subsampling=0으로 변환한다(무손실 raw 입력이라고 주장하지 않음). 디렉터리는 요청 후
자동 정리하며 checkpoint를 바꾸거나 추가 학습하지 않는다. preload 시 image/video
predictor를 각각 준비하므로 기존보다 메모리가 늘어난다. Jetson 메모리/속도는 미검증이다.

현재 mask는 최소 30점, 이미지 최대 50%, border margin 2px, 유효 depth 비율 80%를
검증한다. 설정은 `minimum_object_points`, `reference_maximum_mask_fraction`,
`reference_border_margin_px`, `reference_minimum_depth_fraction`이다. 기존 depth 범위를
유지하며 current depth를 deproject/transform한 표면의 축별 1–99% bounds를 반환한다
(`reference_trim_fraction=0.01`). **탁자 평면으로 바닥을 채우지 않으며 완전한 물체 OBB로
취급하지 않는다.** 원래 검출 confidence는 `source_confidence`에만 보존한다.
reference 실패에 GT/color fallback은 없다. mask와 현재 point cloud는 service 결과에
포함되어 실행 artifact로 저장할 수 있다. 테스트는 `test_reference_*`,
`test_sam2_reference_tracker.py`와 `make test-grasp-pregrasp`에 포함된다.

`config/yoloe_s_sam2_tiny.yaml`은 MuJoCo 파이프라인과 일반 RGB-D 실행에서
명시적으로 YOLOE를 선택할 때 사용하는 로컬 모델 프로필이다. YOLOE-26s segmentation checkpoint의 bbox만
검출 결과로 사용하고 마스크는 SAM2.1 tiny로 만든다. 클래스는 cup/wallet/
crumpled tissue/lego brick, YOLOE 입력 크기는 640, confidence 0.25다.
이는 장면 교체에 맞춘 text prompt 설정이며 새 물체의 검출 정확도를 보증하지 않는다.

현재 MuJoCo study-cafe 기본 검출기는 `config/gemini_flash_lite_sam2_tiny.yaml`의
`gemini-3.1-flash-lite`다. [공식 모델 ID와 이미지/구조화 출력 지원](https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-lite)을
기준으로 기존 Gemini `generate_content` adapter를 사용한다. RGB PNG와 prompt를 외부
API에 보내 bbox를 받고, SAM2.1-tiny는 로컬에서 mask/추적을 담당한다. `GEMINI_API_KEY`
환경 변수가 필요하며 키 자체는 ROS parameter/로그/레포에 저장하지 않는다. `prepare()`는
API 호출 없이 인증 값의 존재와 client 생성만 확인한다. API 접근 권한·quota·bbox 정확도는
실제 요청에서 별도로 확인해야 하며, 실패 시 YOLOE나 색상 검출기로 자동 대체하지 않는다.
Gemini confidence는 자기 보고 점수로 YOLOE confidence와 직접 동등 비교할 수 없다.
일반 `learned_rgbd.launch.py`도 `gemini_flash_lite_sam2_tiny.yaml`을 기본으로
사용한다. `model_profile:=<yaml>`로 대체 프로필을 명시할 수 있다.
SAM2 reference/wrist service의 시작 조건은 YOLOE와 Gemini 검출기를 모두 지원한다.
Gemini도 같은 RGB snapshot의 bbox/label을 reference seed로 전달하며 이후 연속 추적은
로컬 SAM2가 담당한다. reference가 없는 모드나 색상 검출기를 이 경로로 허용하지 않는다.

```bash
ros2 launch cleany_perception learned_rgbd.launch.py
# 다른 모델 저장 위치 / 장치
ros2 launch cleany_perception learned_rgbd.launch.py \
  model_directory:=/models device:=cuda:0
# MuJoCo 연결 + tracking 중단 (단일 이미지 segmentation 유지)
ros2 launch cleany_perception learned_rgbd.launch.py \
  model_directory:=/models device:=cuda:0 use_sim_time:=true sam2_tracking_enabled:=false
```

이 launch는 카메라/로봇 driver를 시작하지 않는다. 기본 camera 입력은
`/camera/color/image_raw`, `/camera/color/camera_info`,
`/camera/aligned_depth_to_color/image_raw`이며 실제 토픽은 인자로 변경한다.
정류·정합 RGB-D와 CameraInfo의 frame/크기/intrinsics/timestamp 계약을
맞춰야 한다. 현재 snapshot buffer는 exact timestamp sync이고 실제 D435
driver 조합의 동기화/보정 검증을 대체하지 않는다.

모델 파일 경로는 `model_directory`(기본 `CLEANY_MODEL_DIR` 또는 `~/models`)
기준이며 절대 경로도 허용한다. detector/segmenter 생성 시 asset 존재와
device를 확인한다. `auto`만 CUDA/CPU를 선택하고, 명시한 CUDA가 없으면
실패한다. `preload_models=true`는 action server 공개 전에 두 모델을 로딩한다.
YOLOE checkpoint와 text prompt를 선택 장치에 준비하고 SAM2 predictor를
생성한다. 첫 이미지 추론까지 실행하는 warmup과는 다르다. load 실패 시
색상 fixture나 다른 모델로 대체하지 않는다.

기존 `inspect_scene.launch.py`의 Gemini/테스트용 선택 기능은 유지한다.
실로봇 지향 실행은 `learned_rgbd.launch.py` 또는 skill executor의 기본
파이프라인을 사용해야 한다. 모델 dependency 설치 기준은
`docs/DEVELOPMENT_SETUP.md`를 따른다.

## 전체 depth 환경 포인트클라우드

`depth_scene_node`는 검출 bbox나 SAM mask와 무관하게 전체 depth를
`/perception/scene_cloud` (`PointCloud2`, XYZ, 원본 optical frame/stamp)로
투영한다. 환경 물체 pose, 크기, MuJoCo mesh/정답 상태는 읽지 않는다.
설정은 `config/depth_scene.yaml`이다. 기본값은 0.1–2.0 m, pixel stride 2,
2 Hz이며, 1초보다 오래되거나 미래 timestamp인 depth는 발행하지 않는다.
0/NaN/Inf depth를 빈 공간이나 임의 표면으로 채우지 않는다.

입력은 **정류된 depth와 그 영상에 맞는 CameraInfo**여야 한다. RGB로
정렬된 depth라면 RGB optical frame/intrinsics를 사용한다. 왜곡 계수가
남아 있거나 frame/해상도가 다르면 거부한다. `32FC1`은 m, `16UC1`은
`depth_16u_scale_m`(기본 0.001)로 환산하며 endian과 row padding을 처리한다.
CameraInfo는 같은 보정/해상도가 유지된다는 전제로 최신 값을 재사용한다.

```bash
ros2 run cleany_perception depth_scene_node --ros-args \
  --params-file ros2_ws/src/cleany_perception/config/depth_scene.yaml \
  -p depth_image_topic:=/camera/depth/image_rect_raw \
  -p depth_info_topic:=/camera/depth/camera_info
```

이 출력은 RViz 표시와 MoveIt occupancy updater 입력용이다. 로봇 자체
표면 제거는 MoveIt의 URDF/TF 기반 self-filter가 담당한다. 검출되지 않은
물체도 depth가 관측되면 포함되지만, 가려진 표면은 복원하지 않는다.

## 관련 KB

### Gemini 분류 메타데이터 (2026-09-08)

Gemini structured response에 `sorting_category`(trash/lost_item/review),
`sorting_reason`을 추가해 `DetectedObject2D`까지 보존한다. 기존 bbox만 있는
응답은 파싱 가능하지만 category는 빈 값이며 sorting 실행에서 검토 대상으로
처리한다. 위험하거나 용도가 불확실한 물체는 review를 요청한다. 소유권 판단은
확정 사실이 아니며 현재 정책은 감독하의 시뮬레이션 검증용이다.

연속 손목 tracking의 120초 참조 TTL은 마지막으로 유효한 새 관측 이후의 유휴
시간이다. fresh/visible/증가하는 capture timestamp가 모두 만족할 때만 갱신한다.
오래된 결과 재발행, 빈 mask, 카메라 정지로는 갱신하지 않으며 CLEAR 시 종료한다.
따라서 CPU 시뮬레이션에서 한 동작이 오래 걸려도 정상 추적을 임의로 만료시키지 않는다.

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
