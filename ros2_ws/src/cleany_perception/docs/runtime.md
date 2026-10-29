# cleany_perception

## 물체 종류명과 검출 클래스

`yoloe_gemini`는 Gemini가 판단한 짧은 영어 종류명을 `label`로 반환한다.
색상, 브랜드와 상태 설명은 이름에 붙이지 않는다. YOLOE와 다른 종류나
YOLOE 34종 밖의 이름도 허용하며, `detector_label`에는 원본 YOLOE 클래스를
보존한다. bbox, instance mask와 confidence는 YOLOE 결과다.

`semantic_label_aliases` ROS parameter는 `source=type` 문자열 배열이다.
대소문자와 연속 공백을 정규화한 뒤 명시한 별칭만 적용한다. 기본값은
`mouse`, `wireless mouse`를 `computer mouse`로, `paper cup`을 `cup`으로
통일한다. 새로운 종류를 기존 종류로 추측해서 바꾸지 않으며, 순환하거나
중복된 별칭 설정은 거절한다. `unknown object`는 항상 `review`로 반환한다.
종류명 정규화는 쓰레기와 분실물 분류를 대신하지 않는다.

Gemini 분류 응답은 객체별 `id`, `label`, `sorting_category`만 요청한다.
분류 사유 문장은 생성하거나 메시지로 전달하지 않는다. 처리 허가는 category와
기존 위험 물체, 검출 신뢰도, 깊이, 크기 및 목적지 조건으로 검사한다.

고정 베이스 추적과 head reference 재관측은 원본 검출 클래스와 공간 정보를
사용한다. 최종 종류명 교정만으로 새 track ID를 만들지 않는다. `detector_label`이
없는 비-YOLOE 입력은 `label`을 사용한다. Monitor 승인 재확인은
[승인 대상 재확인 서비스](interfaces.md)로 원본 이름과 분류를 유지하고
새 YOLOE 및 깊이 관측을 검사한다. 제외 목록, 목적지와 종류별 파지 보정의
설정 키는 정규화된 최종 종류명이다.

필드와 호환성은 [관측 계약](../../cleany_interfaces/docs/observation-contracts.md)을 따른다.

`depth_scene_node`의 Depth/CameraInfo 구독은 best-effort KEEP_LAST(1)이다.
부하가 높을 때 과거 입력을 순차 처리하지 않고 최신 수신 프레임을 사용한다.
촬영 timestamp와 `maximum_depth_age_sec` 검사는 유지하며 오래된 Depth를
현재 시각으로 재표기하지 않는다.

`enable_wrist_observation=true`는 `/perception/observe_wrist_target`를 추가한다.
현재 sorting의 `yoloe_gemini` + `yoloe_seg` 모드에서는 손목 RGB에 YOLOE-seg를
실행한다. `wrist_yoloe_model_path`는 왼손목 바닥 위 물체의 HANDOFF,
`wrist_right_yoloe_model_path`는 오른손목 HANDOFF,
`wrist_check_yoloe_model_path`는 왼손목 파지 후 CHECK,
`wrist_mouse_check_yoloe_model_path`는 왼손목 마우스 CHECK,
`wrist_right_check_yoloe_model_path`는 오른손목 CHECK 체크포인트다. 경로를 비워 두면
앞 단계 모델을 공유한다. Head RGB-D에서 전달한 예상 OBB를 촬영 시점 TF로 손목 영상에 투영하고,
별도 오른손목 CHECK 모델을 지정했다면
`wrist_right_check_yoloe_class_confidence_thresholds`로 그 모델의 클래스별 검출
기준을 설정할 수 있다. 노드 매개변수의 빈 목록은 head와 같은 기준을 쓴다.
스터디카페 기본 프로필은 실제 오른손목 컵 파지 자세로 만든 전용 CHECK 모델을
사용하며 컵 기준만 0.08로 설정한다. 추가 학습한 v2 모델은 서로 다른 독립
합성 영상 묶음에서 컵 마스크 IoU 0.5 검출이 각각 20/20이었다. 실제 파지
영상 한 장에서도 컵 검출 신뢰도가 약 0.69였다. 미검출과 관측 오류는 아래 응답 상태로 구분한다.
별도로 전달한 원본 `detector_label`, 투영 box와의 IoU, 단일 물체 여부 및 instance
mask 위치·면적을 HANDOFF와 CHECK마다 확인한다. 손목 영상은 로컬 YOLOE-seg로 검증한다. 연속 추적 대신 두 시점의 새 영상을 독립적으로 검출하며,
같은 위치·마스크의 중복 예측만 하나로 취급한다.
오른손목 HANDOFF 전용 모델이 투영 위치에서 물체를 찾지 못하면, 이미 로드된
왼손목/양손목 통합 모델로 같은 RGB를 한 번 더 검사한다. 이 경우에도 같은
클래스·투영 box·마스크 검증을 통과해야 하며, 정상 검출 시 추가 추론은 없다.
오른손목 CHECK 전용 모델이 놓친 경우에는 왼손목 파지 후 통합 모델로 같은
검증을 한 번 더 수행한다. 마우스 CHECK만 별도 모델을 쓰므로, 통합 모델을
휴지 CHECK의 추가 메모리 비용 없이 재사용한다.
headless 인식 진단 시 `wrist_failure_image_directory`를 지정하면 YOLOE가
예상 손목 물체를 찾지 못한 프레임만 PNG로 저장한다. 기본값은 빈 문자열이라
영상을 저장하지 않는다.
HANDOFF 이후 RGB·mask 이력을 보관하지 않는다. 손목 카메라는 RGB만 제공하므로
새 3D 위치나 독립적인 물체 ID는 산출하지 않는다. Head 모델과 다른 손목
체크포인트를 지정하면 그만큼 모델 메모리가 추가된다.

손목 RGB와 CameraInfo는 exact timestamp로 짝지어 처리한다. 촬영 시점 TF,
보정값, source/reference ID와 mask 검증 실패는 성공으로 처리하지 않는다.
`wrist_*` 매개변수로 freshness/TTL, confidence, 투영 시야와 mask 면적 한계를 설정한다.
HANDOFF와 CHECK는 `after_stamp_ns`보다 새로운 촬영 영상을 요구한다.
`ObserveWristTarget` 응답은 `OK`, `NOT_DETECTED`, `ERROR`를 구분한다.
정상 프레임에서 대응 후보가 없으면 `NOT_DETECTED`이며 `success=false`다.
CHECK 미검출도 실제 촬영 시각과 기존 reference/source ID를 반환하고,
그 프레임을 다시 CHECK에 사용하면 거부한다. 새 프레임 재시도와 그리퍼 피드백을
이용한 진행 여부는 skill executor가 판단한다. 센서·TF·모델 오류, 후보 모호성,
잘못된 mask 및 비정상 confidence는 `ERROR`로 반환하며 미검출로 숨기지 않는다.

## 기존 snapshot 조회

`/perception/get_scene_snapshot` (`cleany_interfaces/srv/GetSceneSnapshot`)은
`snapshot_id`의 캐시된 2D 관측을 읽는다. `found`, `detections`, `message`를 반환하고,
TTL과 촬영 시각을 갱신하거나 새 모델 추론을 시작하지 않는다. 누락/만료는 `found=false`다.
동일 cache를 `InspectScene` 선택 물체 복원에서도 사용한다.

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

## 처리 흐름 및 경계

인식 파이프라인의 처리 순서는 다음과 같습니다:

```text
정렬된(Aligned) RGB-D 수신
  └─► 촬영 시점의 TF 좌표 변환
  └─► 2D 검출기(YOLOE) Bounding Box 및 객체 식별 번호 부여
  └─► Bounding Box 중앙 Depth를 기준으로 로봇(base_link)과의 거리 계산 및 가까운 순 정렬
  └─► 크기 제한이 적용된 스냅샷 캐시(Bounded Snapshot Cache) 저장
  └─► 대상 물체 선정 (자동 추천 또는 사용자 승인)
  └─► 선택된 객체의 인스턴스 마스크(Mask) 추출
  └─► 바닥 지지 평면(Support Plane) 추정 및 분리
  └─► 로봇 기준계(base_link) 3D 지향 경계 상자(OBB) 및 파지 영역 생성
```

## 모델 준비 및 Gemini 설정

- **사전 준비**: 상세 의존성 및 모델 다운로드 안내는 [개발환경 가이드](../../../../docs/DEVELOPMENT_SETUP.md)를 참고하세요.
- **가중치 파일 위치**: YOLOE 체크포인트와 `mobileclip2_b.ts` 텍스트 인코더는 `CLEANY_MODEL_DIR` 또는 `~/models` 아래에 준비합니다.
- **Gemini 분류 연동**:
  - `yoloe_gemini` 모드 사용 시 환경 변수 `GEMINI_API_KEY`를 필수로 주입해야 합니다.
  - Gemini API로 전송되는 이미지는 원본 해상도의 **JPEG(`image/jpeg`) 포맷**으로 인코딩됩니다.
  - 압축 품질은 ROS 파라미터 `gemini_jpeg_quality`로 지정하며 (기본값: 95, 범위: 1~100), 색차 서브샘플링 없는 고품질 4:4:4 형식을 사용합니다.
  - 전송 이미지에는 YOLOE가 검출한 객체 번호와 박스를 시각적으로 오버레이하여 VLM의 공간 인식을 보조합니다. (ROS 2 내부의 PointCloud2 원본 데이터나 3D OBB는 압축 손실 없이 보존됨)

## 실행

Jetson에서는 perception을 `172.30.0.11` 전용 GPU service로 실행한다. AnyGrasp의 고정
MAC, license와 checkpoint는 perception container에 mount하지 않는다. 자세한 절차는
[`containers/vision`](../../../../containers/vision/README.md)을 따른다. 아래 명령은 native
개발환경용이다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_perception inspect_scene.launch.py
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

1차 결과의 `snapshot_id`는 후속 선택 단계가 같은 RGB-D와 촬영 시점 TF를 사용하기 위한
opaque key다. cache 크기와 TTL을 넘긴 snapshot은 후속 단계에서 사용할 수 없다.

## 모델 로딩 로그

기본 `INFO` 출력은 모델 로딩 시작과 준비 완료·사용 장치를 알린다.
개별 detector 로딩과 모델·text encoder 경로는 `DEBUG`에서 확인한다.
MuJoCo 수거·BT launch에서는 `log_level:=debug`, 단독 노드 실행에서는
`--ros-args --log-level perception_inspector:=debug`로 상세 로그를 켠다.

깊이 점군과 receipt 노드는 ROS context 종료에 따른 executor 중단과 점군 변환
중단을 정상 종료로 처리한다. ROS가 실행 중일 때의 `RuntimeError`는 다시
발생시켜 실제 변환 실패가 숨겨지지 않게 한다.

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

## 스터디카페 MuJoCo 전용 YOLOE-seg 학습

기본 YOLOE-26s-seg의 범용 text prompt는 현재 MuJoCo 종이컵·마우스·휴지·레고를
안정적으로 검출하지 못했다. `tools/study_cafe_yoloe_dataset.py`는 동일한 study-cafe
scene의 head RGB 영상을 여러 물체 위치·회전과 밝기 값으로 생성하고, **오프라인 학습
라벨에만** MuJoCo segmentation geom ID를 쓴다. 실행 중 인식 노드는 RGB-D 픽셀과
YOLOE 출력만 읽으며 geom ID나 물체 pose를 사용하지 않는다. 생성 데이터와 모델은
저장소 밖의 모델 디렉터리에 둔다.

손목 시점 학습에는 `--camera left_wrist_rgb` 또는 `right_wrist_rgb`와
`--joint-positions-json`으로 실제 pregrasp의 관절값을 전달할 수 있다. 손목 시야에
네 물체가 모두 들어오지 않으면 `--allow-partial-objects`를 사용해 보이는 물체만
라벨링한다. 서로 다른 카메라 데이터는 같은 클래스 순서로 합쳐 학습하고, 별도 seed의
holdout 영상으로 head와 양손목 성능을 각각 평가한다. 이 geom ID와 관절값은 오프라인
학습에만 쓰며 실행 중 인식에는 전달하지 않는다.
`tools/combine_study_cafe_yoloe_datasets.py`는 클래스 순서를 검사하고 이미지·라벨을
복사하지 않고 심볼릭 링크로 합친다. 예를 들어 head와 두 손목의 `dataset.yaml`을
`--source head=... --source left_wrist=... --source right_wrist=...`로 전달한다.
파지 후 시점을 학습할 때는 `--attached-object lego --attachment-site left_grasp_tcp`처럼
오프라인 장면의 물체를 TCP 부근에 배치할 수 있다. `--attachment-jitter-m`으로 위치를
흔들며, 렌더링한 물체 geom에서만 학습 라벨을 만든다. 이는 실제 런타임 물체 위치를
읽는 기능이 아니며, 생성 자세 밖의 손목 시점 성능을 보장하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
export MUJOCO_GL=egl
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
python3 ros2_ws/src/cleany_perception/tools/study_cafe_yoloe_dataset.py \
  --output ~/models/yoloe/study_cafe_sim_v1_dataset \
  --train-count 64 --val-count 16
python3 ros2_ws/src/cleany_perception/tools/finetune_study_cafe_yoloe.py \
  --dataset ~/models/yoloe/study_cafe_sim_v1_dataset/dataset.yaml \
  --checkpoint ~/models/yoloe/yoloe-26s-seg.pt \
  --text-encoder-directory ~/models/yoloe \
  --project ~/models/yoloe/study_cafe_sim_v1_training \
  --epochs 8 --batch 4 --device cpu
python3 ros2_ws/src/cleany_perception/tools/evaluate_study_cafe_yoloe.py \
  --dataset ~/models/yoloe/study_cafe_sim_v1_dataset/dataset.yaml \
  --checkpoint ~/models/yoloe/study_cafe_sim_v1_training/fit/weights/best.pt \
  --text-encoder-directory ~/models/yoloe \
  --confidence 0.08 --class-thresholds 0.25 0.25 0.25 0.08
cp ~/models/yoloe/study_cafe_sim_v1_training/fit/weights/best.pt \
  ~/models/yoloe/study_cafe_sim_yoloe26s_seg.pt
```

실행 기본값은 YOLOE 추론 confidence 0.08과 클래스별 컵/마우스/휴지 0.25,
레고 0.08이다. 클래스별 문턱값은 모델 출력 뒤, Gemini 분류 전에 적용한다.
마스크 평가는 IoU 0.5를 사용한다. 학습에 쓰지 않은 별도 난수 시드 20장(80개 물체)에서
범용 모델은 6개, 이 모델과 설정은 71개를 맞췄다. 클래스별로 컵 20/20,
마우스 18/20, 휴지 18/20, 레고 15/20이었다. 모델 출력 중 GT mask와 IoU 0.5로
매칭되지 않은 검출은 21개였다. 검증 영상도 같은 scene의 다른 무작위 배치이므로
실물 인식 성능의 근거로 사용할 수 없다.

## YOLOE-seg 공통 실행 프로필

`config/yoloe_seg.yaml`은 로컬 YOLOE 검출의 instance mask를 선택 객체 분할에
그대로 사용한다. `config/yoloe_seg_gemini.yaml`은 같은 마스크에 Gemini의
label/category를 결합하며 `learned_rgbd.launch.py`의 기본 프로필이다.
`model_profile:=<yaml>`로 명시적으로 다른 프로필을 선택할 수 있다.
`enable_wrist_observation:=true`로 YOLOE-seg 손목 HANDOFF/CHECK 서비스를 켠다.
모델 생성 시 로컬 파일과 장치를 검사하고 `preload_models=true`는 action 공개 전에
모델을 준비한다. 실패하면 다른 검출기나 색상 fixture로 자동 대체하지 않는다.

```bash
ros2 launch cleany_perception learned_rgbd.launch.py \
  model_directory:=/models device:=cuda:0 use_sim_time:=true
```

카메라와 로봇 driver는 별도로 실행한다. 기본 입력은 `/camera/color/image_raw`,
`/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw`이며
RGB-D와 CameraInfo의 크기·frame·intrinsics·exact timestamp 계약이 필요하다.
`auto`는 CUDA/CPU를 선택하고 명시한 CUDA를 사용할 수 없으면 실패한다.

## 전체 depth 환경 포인트클라우드

`depth_scene_node`는 검출 bbox나 SAM mask와 무관하게 전체 depth를
`/perception/scene_cloud` (`PointCloud2`, XYZ, 원본 optical frame/stamp)로
투영한다. 환경 물체 pose, 크기, MuJoCo mesh/정답 상태는 읽지 않는다.
설정은 `config/depth_scene.yaml`이다. 기본값은 0.1–2.0 m, pixel stride 2,
2 Hz이며, 1초보다 오래되거나 미래 timestamp인 depth는 발행하지 않는다.
cloud 구독자가 없으면 depth 투영 자체를 건너뛰며, 구독자가 생기면 다음 새 depth부터 발행한다.
현재 정렬 실행에서는 MoveIt의 OctoMap 최신성 검증 때문에 장면 cloud를 계속 갱신한다.
작업 단계별 생성 중단은 최신성 계약과 함께 재설계해야 하므로 적용하지 않았다.
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

## 고정 베이스 책상 물체 추적

`InspectScene.Goal.tracking_session_id`를 지정하면 새 관찰 성공 시 `core/tracking.py`가
같은 검출 클래스와 촬영 TF의 `base_link` 대표 위치로 물체를 연결한다. 거리 계산에 사용한 3D
위치를 함께 보존하며 `position_valid`, `track_id`, `tracking_state`를 반환한다.
RGB header frame과 대표 위치의 `representative_frame`은 서로 구분한다. 유일한 양방향
대응만 `TRACKED`이며 모호한 위치·큰 이동·깊이 실패는 새 ID로 우회하지 않는다.
사라진 물체의 마지막 참조는 `missing_objects`에 남는다. 최초부터 불확실했던 물체는
빈 track ID 참조로 남을 수 있다. 빈 장면이 불확실한 대상을 없던 것으로 만들지 않는다.

`inspect_scene.yaml`의 위치 차이 3cm·후보 차이 1cm, alias, 최대 32세션·TTL 600초는
조정 가능한 시뮬레이션 초기값이다. 컵 별칭과 `computer mouse`, `wireless mouse`를
각각 `cup`, `mouse`로 연결하며 레고는 `lego brick`을 유지한다. 별칭은 종류만 연결하고
쓰레기·분실물 분류를 결정하지 않는다. 실물 기준이 아니다. 세션 메모리 재생성·만료마다
epoch가 바뀌며 베이스 이동 시 호출자가 새 세션을 시작해야 한다. TF 실패·취소 관찰은
갱신하지 않는다. cache 조회·선택 복원은 기존 ID·epoch를 그대로 반환하고 TTL을 유지한다.
기존 빈 세션 호출은 `UNTRACKED`다.

## 관련 KB

### Gemini 분류 메타데이터

Gemini structured response의 `sorting_category`(trash/lost_item/review)를
`DetectedObject2D`까지 보존한다. 기존 bbox만 있는
응답은 파싱 가능하지만 category는 빈 값이며 sorting 실행에서 검토 대상으로
처리한다. 위험하거나 용도가 불확실한 물체는 review를 요청한다. 소유권 판단은
확정 사실이 아니며 현재 정책은 감독하의 시뮬레이션 검증용이다.

- [Technical Overview](../../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [Safety and Risk](../../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

## RGB-D 버퍼와 재구성 재사용

ROS 영상과 점군은 연속 NumPy 버퍼에서 ROS가 소유하는 byte array로 복사한다. 같은 snapshot의 평면과 OBB는 재사용하되 depth, frame, intrinsic와 설정이 달라지면 캐시를 무효화한다. 실패한 재구성은 성공 결과로 보관하지 않으며 capture 시각과 TTL을 갱신하지 않는다.

`snapshot_cleanup_period_seconds`는 만료된 snapshot과 reference 정리를 제어한다. `/perception/check_rgbd` (`std_srvs/Trigger`)는 새 RGB-D 입력의 timestamp와 freshness를 확인하며 모델 추론이나 과거 검출을 입력 정상성의 근거로 쓰지 않는다. 기존 tracking 필드와 함께 검출 메시지의 `median_depth_m`, `estimated_width_m`, `estimated_height_m`을 발행한다. 두 크기는 지지면 위의 footprint 추정이며 물체의 수직 높이나 완전한 형상이 아니다.

VLM 대기와 geometry 계산을 중첩하며 선택적 `/grasp/vlm_warmup` 예열을 요청할 수 있다. 응답 성공은 최종 분류와 기하 검증 이후에만 반환한다. 예열 서비스가 없거나 실패해도 그 자체로 물체를 성공 처리하지 않는다.

`vlm_anygrasp_prepare_service`의 기본값은 `/grasp/prepare_window`다. 이 준비 응답이
성공한 뒤 cuRobo 예열을 요청한다. VLM이 그 전에 끝나면 cuRobo를 새로 예열하지
않는다. 종료 통지는 cuRobo, AnyGrasp 순서로 보내며 perception은 해당 GPU 준비
완료를 기다리지 않는다. 빈 서비스 이름은 기존 cuRobo 통지만 사용한다.
