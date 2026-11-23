# 실행 가이드

할 일을 선택하고 준비 → 실행 → 관측 → 종료 순서로 진행합니다. 각 가이드는 같은 저장소의 명령과 설정을 기준으로 합니다.

| 하려는 일 | 가이드 | 확인할 결과 |
|---|---|---|
| 가상 환경에서 센서·주행 확인 | [시뮬레이션](how-to/simulation.md) | scan, odom, TF |
| LiDAR로 지도 생성·저장 | [매핑](how-to/mapping.md) | map.yaml, PGM, pbstream |
| Jetson 게임패드 수동 조종 | [리시버와 입력 시험](how-to/gamepad.md) | 스틱 중립과 입력 단절 후 MCU 정지 확인 |
| 저장 지도에서 목표 주행 | [휴대폰 주행](how-to/phone-navigation.md) | 경로 미리보기와 정지 응답 |
| 물체 관찰·후보 계획·조작 | [조작](how-to/manipulation.md) | snapshot, 계획, Action 결과 |
| Backend 요청과 실행 연결 | [Mission](how-to/mission.md) | 요청 수락, 허가, 결과 |
| 센서 입력과 장애물 관측 | [센서](how-to/sensors.md) | 영상·점군·좌표계·시각 |
| MCU와 베이스 연결 | [베이스](how-to/base.md), [펌웨어](../../esp32/README.md) | 휠 피드백과 STOP ACK |
| 기록과 화면 확인 | [관측·기록](how-to/monitoring.md) | Logger 세션과 Monitor 화면 |

## 공통 준비

명령은 저장소 루트의 Ubuntu 22.04 / Humble 터미널에서 실행합니다. [환경과 출력 경로](development-setup.md), [연동 계약](integration.md)을 확인한 뒤, 각 터미널에 같은 DDS domain과 같은 worktree overlay를 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
```

Navigation 전용 빌드는 `print-navigation-install-base`로 overlay를 조회합니다. 빌드 때 바꾼 Make 변수는 경로 조회 때도 동일하게 전달합니다.

Jetson에서 Dashboard와 실물 Runtime을 연결하는 순서는 [관제 통합](how-to/control-plane-integration.md)을 따릅니다.

### 헤드캠 비식별 프리뷰

Jetson의 모델 디렉터리는 `/var/opt/cleany/models/camera-privacy/active`다. YOLOE용 `privacy.engine`, YuNet용 `face.onnx`, 글자 위치 검출용 `text.plan`을 함께 둔다. 카메라와 같은 ROS 환경에서 native YOLOE Python으로 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /opt/cleany/current/ros/local_setup.bash
/opt/cleany/current/native/yolo-env/bin/python \
  "$(ros2 pkg prefix cleany_perception)/lib/cleany_perception/camera_privacy" \
  --ros-args -p gpu_lock_path:=/var/opt/cleany/state/cleany/gpu-work.lock
```

운영 프리뷰는 `cleany_telemetry camera_preview`에 `-p allowed_topic:=/camera/head/color/image_redacted`를 전달한다. 노드는 구독자가 생기면 추론을 시작한다. 기존 ROS 원본 구독자와 외부 장면 분류기 설정은 별도로 확인한다. [토픽 계약과 클라우드 입력 설정](../../ros2_ws/src/cleany_perception/docs/interfaces.md#헤드캠-비식별-영상)
