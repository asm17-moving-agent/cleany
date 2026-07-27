# cleany_perception

카메라 이미지를 받아 **객체 탐지 후보**(`vision_msgs/Detection2DArray`)를 publish하는 ROS 2 패키지.

Perception은 generic 후보(라벨·confidence·bbox)만 제공하고 행동을 판정하지 않는다.
`collect`/`skip` 같은 의미 판단은 Planner 몫이다 (AGENTS.md §3).

## 현재 범위: RGB 2D 탐지 + 인스턴스 세그멘테이션

- **RGB 2D 탐지 + 인스턴스 마스크**까지 한다. depth 융합(3D 위치)은 이후 단계.
- 기본 가중치는 `yolo11n-seg.pt`. plain detect 가중치(`yolo11n.pt`)를 지정하면
  마스크 없이 bbox만 나온다 (코드 변경 불필요).
- 마스크는 노드 내부 `Detection.mask`(원본 해상도 bool 배열)로만 유지하고
  별도 토픽으로 publish하지 않는다. 이후 depth 융합이 같은 노드 안에서
  마스크를 소비할 예정이라 메시지 전송이 필요 없기 때문 (YAGNI).
  시각 확인은 `publish_annotated`의 마스크 오버레이로 한다.
- 확장 대비로 `vision_msgs`의 `pose` 슬롯은 비워둔 채 남겨둔다 (메시지 교체 없이 depth 추가 가능).

## 구조 (core / node 분리)

ROS 무의존 core 로직과 얇은 ROS 노드를 분리해 core를 pytest로 빠르게 검증한다 (AGENTS.md §4).

| 파일 | 역할 | ROS 의존 |
|---|---|---|
| `detector.py` | `parse_boxes()`(순수, 마스크 포함) + `YoloDetector`(ultralytics lazy import) | 없음 |
| `imaging.py` | `Image`↔ndarray (numpy 직접, cv_bridge 미사용) | 메시지 타입만 |
| `detection_to_msg.py` | `Detection[]` → `vision_msgs/Detection2DArray` | 메시지 타입만 |
| `annotate.py` | bbox+label+마스크 오버레이 그리기 (선택 viz, cv2 lazy import) | 없음 |
| `detection_node.py` | 배선: `/image_raw` → detect → `/detections` | rclpy |

## 토픽

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| sub | `/image_raw` | `sensor_msgs/Image` (rgb8) | QoS = SensorDataQoS (sim reliable·실카메라 best-effort 겸용) |
| pub | `/detections` | `vision_msgs/Detection2DArray` | 탐지 0이어도 매 사이클 publish |
| pub | `/detections_image` | `sensor_msgs/Image` (rgb8) | `publish_annotated: true`일 때만 |

## 파라미터 (`config/detection.yaml`)

| param | 기본 | 설명 |
|---|---|---|
| `image_topic` | `/image_raw` | 입력 이미지 토픽 |
| `detections_topic` | `/detections` | 출력 탐지 토픽 |
| `weights` | `yolo11n-seg.pt` | 모델명(자동 다운로드) 또는 abs 경로 (`models/README.md`). `-seg` 가중치면 인스턴스 마스크 포함 |
| `conf` | `0.25` | confidence 임계값 |
| `classes` | `[]` | COCO class-id 필터 (빈=전체) |
| `device` | `''` | `''` auto / `cpu` / `cuda:0`·`0` / `mps` |
| `detection_rate_hz` | `5.0` | 추론 타이머 rate (카메라 rate와 분리) |
| `publish_annotated` | `false` | bbox/label overlay 이미지 publish 여부 |
| `annotated_topic` | `/detections_image` | overlay 토픽 |

설정은 코드에 하드코딩하지 않는다. 값 변경은 이 yaml 편집 또는 `params_file:=<경로>`로 다른 파일 지정.

## 빌드

```bash
cd ros2_ws
colcon build --packages-select cleany_mujoco_sim cleany_perception
source install/setup.bash
```

## 실행 (sim end-to-end)

호스트가 정하는 런타임 env 두 가지를 먼저 export한다 (코드/launch에 박지 않음):

- `MUJOCO_GL` — offscreen 렌더 백엔드: `egl`(GPU) / `osmesa`(SW 폴백, 느림) / `glfw`(온스크린 창).
- (필요시) `CUDA_VISIBLE_DEVICES=""` — GPU를 torch가 못 쓰는 경우 CPU 강제.

```bash
CUDA_VISIBLE_DEVICES="" MUJOCO_GL=egl \
  ros2 launch cleany_perception detection_sim.launch.py headless:=true
```

확인:

```bash
ros2 topic echo /detections            # 탐지 결과
# annotated 육안: config에서 publish_annotated: true 로 두고 재빌드 후
ros2 run rqt_image_view rqt_image_view # 드롭다운에서 /detections_image 선택
```

launch 인자는 `headless`, `params_file` 둘뿐이다. detection 튜닝은 전부 params-file로 모아
launch가 서로 값을 덮어쓰지 않게 한다.

## Python 의존 설치 (중요 — ROS 공존 핀)

ROS Humble은 numpy 1.x ABI로 빌드돼 있다. ultralytics를 그냥 설치하면 numpy 2.x로
올려 **ROS가 깨질 수 있다**. 개발 환경은 저장소의 단일 버전 세트를 사용한다:

```bash
python3 -m pip install --require-hashes -r requirements/ros2-humble-dev.txt
```

- Dockerfile은 hash가 포함된 이 lock만 설치한다. rosdep wrapper는 `mujoco`와
  `ultralytics` key를 건너뛰므로 lock을 덮어쓰지 않는다.
- Jetson은 `requirements/jetson-jp622-arm64.txt`를 사용한다. 일반 PyPI
  Torch로 JetPack용 aarch64 빌드를 덮어쓰면 안 된다.
- 개발 PC GPU가 PyTorch 빌드에서 지원되지 않으면 `device:=cpu`
  또는 `CUDA_VISIBLE_DEVICES=""`.

## weights

개발 중에는 모델명만 지정하면 Ultralytics가 자동 다운로드할 수 있다. Jetson
배포에서는 자동 다운로드를 허용하지 않으며 `models/`의 provenance와 SHA-256이
일치하는 파일만 TensorRT로 변환한다. `models/README.md`를 따른다.

## 테스트

```bash
cd ros2_ws/src/cleany_perception
python3 -m pytest test/ -q
```

YOLO 실추론(`detect()`)은 무거워 단위테스트에서 제외한다. `parse_boxes`(순수 로직), imaging 왕복,
msg 변환, 노드 배선(fake detector 주입)만 테스트한다. 실추론 검증은 sim e2e로 한다.

## 알려진 함정

- **cv_bridge 미사용**: ultralytics의 pip opencv와 ABI 충돌(`cv2_to_imgmsg` `KeyError: 16`) → numpy로 직접 변환.
- **빈 배열 파라미터**: rclpy가 `.value`로 `None`을 돌려준다 → `classes` 등은 `x or []`로 가드.
- **config 반영**: `config/*.yaml`은 빌드 때 install로 복사된다 → 편집 후 재빌드. 즉석 확인은 설치본 직접 편집.
- **네이티브 실행**: sim/추론은 네이티브(WSL/Linux)에서 돌린다. Docker는 빌드/버전 공유용.
