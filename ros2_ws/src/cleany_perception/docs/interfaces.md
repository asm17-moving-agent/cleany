# 인식 인터페이스

`inspection_node`의 기존 Action과 토픽은 [런타임 API](runtime.md#ros-api)를 따릅니다.

| 인터페이스 | 변경 계약 |
|---|---|
| `perception/check_rgbd` (`std_srvs/srv/Trigger`) | 요청 뒤 수신한 동기 RGB-D의 시각과 기하를 검사. 추론이나 모터 명령 없음 |
| `DetectedObject2D/3D.label` | 정규화한 최종 종류명. Gemini가 원본 검출 종류를 수정할 수 있음 |
| `DetectedObject2D/3D.detector_label` | 원본 검출 클래스. 미제공은 빈 문자열 |
| `DetectedObject2D.median_depth_m` | 유효 깊이의 중앙값, m |
| `DetectedObject2D.estimated_width_m / estimated_height_m` | 지지면의 두 수평 방향 크기 추정, m. 미측정은 0 |
| `DetectedObject2D.sorting_reason` | 기존 수거 소비자가 사용하는 모델 분류 사유 유지 |

신뢰도는 검출기의 값이며 Gemini 종류명과 처리 분류의 확률이 아닙니다.
동의어 설정은 종류명 정규화만 수행하며 수거 허가 정책을 변경하지 않습니다.
새 필드가 포함된 모든 메시지, Action, Service 소비자는 같은 버전으로 재빌드합니다.

기존 `ObserveWristTarget` 요청의 `detector_label`은 손목 영상의 클래스 대응에
사용합니다. 빈 값은 `label`로 대체하고, CHECK에서 최종 종류명이나 원본 클래스가
바뀌면 거절합니다. 손목 상태와 승인 재확인은 아래 별도 인터페이스를 사용합니다.


## 기준 물체 재관찰과 예열 알림

| 방향 | 기본 이름 | 타입과 조건 |
|---|---|---|
| 서비스 제공 | `perception/observe_object_reference` | `cleany_interfaces/srv/ObserveObjectReference`; `enable_reference_observation=true`일 때 PIN/OBSERVE/CLEAR |
| 발행 | `/perception/wrist_tracking_status` | `cleany_interfaces/msg/WristTrackingStatus`, depth 10; 손목 관찰 결과 |
| 서비스 호출 | `/grasp/vlm_warmup` | `std_srvs/srv/SetBool`; `vlm_ik_warmup_service` 기본값 |
| 서비스 호출 | `/grasp/prepare_window` | `std_srvs/srv/SetBool`; `vlm_anygrasp_prepare_service` 기본값 |

기준 참조는 원래 snapshot/object와 분류를 보존하고 새 RGB-D 관측을 사용한다.
인식, 추적은 같은 추론 잠금을 공유한다. TTL 만료나 새 관측 실패를 성공으로 대체하지 않는다.
예열 알림은 `vlm_ik_warmup_enabled=true`에서만 사용하며 서비스 준비를 기다려 인식을 막지 않는다.

## 승인 대상 재확인

`inspection_node`는 `perception/revalidate_target`에
`cleany_interfaces/srv/RevalidateTarget` 서비스를 제공한다. 원본 snapshot/object와
추적 session/ID를 확인하고 원본과 `after_stamp_ns`(ns)보다 새 RGB-D 한 장에서 대상을
측정한다. 원본 캐시, 좌표계나 카메라 보정이 달라지거나 대응 후보가 모호하면 실패한다.

물체 번호는 승인 사진 안에서만 유효하다. 동일 원시 클래스의 bbox가 여러 원본이나 여러 최신 검출과 겹치면 각 최신 후보가 원본 추적 위치와 마스크에 일대일로 대응해야 한다. 거리 한계로 `tracking_maximum_distance_m`, bbox와 마스크의 IoU 하한으로 `reference_minimum_detection_iou`를 사용한다. 승인 track이 사라지거나 대응이 모호하면 실패한다. 응답 한 건은 원본 object/track ID,
종류명, 처리 분류와 `sorting_reason`을 유지하고 신뢰도, 위치, 크기, 깊이는 최신 값을 쓴다.
Gemini 재분류, 추적 이력 갱신, 참조 고정이나 모터 동작은 수행하지 않는다.
진단 snapshot은 파지용 캐시에 저장하지 않으며 승인 실행은 원래 snapshot/object를 쓴다.
인식과 같은 busy lock을 사용하며 실패는 BUSY, RGB-D, MASK, DEPTH, TF, REFERENCE로 구분한다.
승인 유효 기간과 위치 변화 허용치는 [운영 제어](../../cleany_dev_monitor/docs/interfaces.md#승인-기반-작업-제안)가 검사한다.

## 보정된 헤드 RGB-D

`calibrated_rgbd`는 선택된 `intrinsics_file`의 RGB K와 D로 원본 RGB를
왜곡 보정한다. SDK 원본 깊이의 보정된 투영행렬과 depth-to-color TF를
사용해 깊이를 새 RGB K의 픽셀로 투영한다. 겹친 픽셀은 가까운 광학 Z를
선택하고 미관측 픽셀은 0으로 남긴다. 깊이 센서의 내부 보정을 덮어쓰지 않는다.

| 구분 | 기본 endpoint | 타입 및 규약 |
|---|---|---|
| 구독 | `/camera/sdk/color/image_raw` | `sensor_msgs/msg/Image`, 원본 RGB, sensor-data QoS |
| 구독 | `/camera/raw_depth/image_raw`, `/camera/raw_depth/camera_info` | 원본 rectified depth와 `sensor_msgs/msg/CameraInfo` |
| TF 구독 | `/tf`, `/tf_static` | SDK의 depth optical에서 color optical로의 변환 |
| 발행 | `/camera/color/image_raw`, `/camera/color/camera_info` | 왜곡 보정된 `rgb8`, 새 K, D=0 |
| 발행 | `/camera/depth/image_raw`, `/camera/depth/camera_info` | RGB에 정렬된 `32FC1`, 단위 m, RGB와 같은 K 및 프레임 |
| 발행 | `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`, JPEG preview |
| 발행 | `/camera_calibration/rgbd_status` | `std_msgs/msg/String`, 보정 경로, 해시, 원본 시각 차이, 유효 깊이 비율의 JSON |

RGB와 원본 깊이의 시각 차이는 `maximum_pair_skew_s` 이하일 때만 처리한다.
기본값은 0.025초이며 출력 네 메시지는 RGB 원본 시각을 공유한다. 원본의
시각 차이는 상태에 남기며, 출력 시각 일치가 하드웨어 동시 촬영을 뜻하지는
않는다. RGB 해상도가 보정값과 다르거나 깊이 투영행렬이 유효하지 않으면
처리하지 않는다. `realsense.launch.py`는 `rgb_intrinsics_file`을 우선한다. 설정에 등록된 일련번호와 `serial_no`가 일치하면 패키지에 설치된 보정 파일을 사용한다. 일치하는 설정이 없으면 SDK 보정을 사용한다. 실측 보정 파일을 선택하면 SDK RGB를
`/camera/sdk/color/*`로 옮기고 SDK 자동 정렬 대신 이 노드를 실행한다.

선택한 물체의 `context_cloud`에는 촬영 시각의 카메라 원점을 점군 좌표계의 미터로 담은 FLOAT32 필드 `vp_x`, `vp_y`, `vp_z`가 있다. 세 값은 한 메시지 안에서 동일하다. XYZ와 RGB는 기존 오프셋을 유지하며 소비자는 `point_step`을 따른다. 원점 필드가 없는 과거 기록도 깊이 기반 파지 입력으로 사용할 수 있다.

## 헤드캠 비식별 영상

`camera_privacy`는 센서 시각과 좌표계를 유지한 BGR 영상을 발행한다.

| 방향 | 기본 토픽 | 타입 | QoS |
|---|---|---|---|
| 구독 | `/camera/color/image_raw` | `sensor_msgs/msg/Image`, RGB8 또는 BGR8 | Best effort, volatile, depth 1 |
| 발행 | `/camera/head/color/image_redacted` | `sensor_msgs/msg/Image`, BGR8 | Best effort, volatile, depth 1 |

검출기는 각 프레임에서 사람, 얼굴, 화면, 문서와 글자 위치를 확인한다. 사람과 민감한 표면을 서로 다른 불투명 색으로 채운다. 화면 검출이 잠시 끊기면 배경의 움직임을 확인한 뒤 최대 0.48초 동안 이전 화면 영역을 보강한다. 구독자가 있을 때 기본 최대 5 Hz로 처리한다. 입력 시각이 오래됐거나 추론이 실패하거나 GPU 잠금을 얻지 못하면 해당 프레임은 발행하지 않는다.

영상에 클라우드 인식을 허용하지 않는 현장 구성에서는 `inspect_scene.launch.py`에 `allow_external_images:=false`를 전달한다. 이 설정에서 Gemini 어댑터를 선택하면 노드가 기동 시 중단된다. 기존 원본 ROS 토픽의 접근 권한은 배포 환경이 관리한다.

Gemini 탐지, 파일 업로드, 번호 분류와 Decision API는 기본적으로 비식별 RGB 스냅샷을 사용한다. 시연용 인식 서비스에 `CLEANY_GEMINI_IMAGE_MODE=original`을 지정하면 가리지 않은 스냅샷을 사용한다. 번호 분류에는 선택한 영상의 복사본에 검출 상자와 번호를 그린다. 이 설정의 적용 범위는 Gemini 이미지 요청이다.
