# 실물 상태와 컨트롤러 경계 인터페이스

본 문서는 실물 로봇의 엔코더 조회 노드, 제어기 안전 가드(Guard), 손목 카메라 및 웨이포인트 도구의 ROS 2 인터페이스를 설명합니다.

> [!CAUTION]
> 엔코더 상태 조회 노드(`reader.py`, `robot_state_reader.py`)와 실물 제어기(`real_control.launch.py`)가 동일한 시리얼 포트를 동시에 점유해서는 안 됩니다.

## 1. 노드별 인터페이스 목록

| 노드명 | 인터페이스 명칭 및 타입 | 역할 및 동작 상세 |
|---|---|---|
| `arm_state_reader` | `joint_states` (`sensor_msgs/JointState`) [발행] | 설정된 단일 팔의 실측 관절 위치 |
| `read_only_robot_state` | `/joint_states` (`sensor_msgs/JointState`) [발행] | 양팔 및 헤드 전체 실측 관절 위치 (기본 20 Hz) |
| `real_control_state_guard` | `joint_state_broadcaster/joint_states` [구독] | 컨트롤러가 발행하는 원본 관절 상태 |
| 〃 | `joint_states` (`sensor_msgs/JointState`) [발행] | 정상성과 신선도(최대 나이 0.25초)를 검증한 관절 상태 |
| 〃 | `real_control/status` (`std_msgs/String`) [발행] | 제어 상태 판정 결과 (`waiting/ready/fault`, 20 Hz, Transient Local) |
| 〃 | `real_control/stop` (`std_srvs/Trigger`) [서비스 제공] | 컨트롤러 비활성화 및 활성 Goal 즉시 취소 |
| 〃 | `controller_manager/switch_controller` [서비스 호출] | 비상 상황 시 제어기 강제 정지 전환 |
| `wrist_rgb_camera` | `/{arm}_wrist_camera/image_raw`, `camera_info` [발행] | 손목 카메라 원본 영상 (SensorData QoS) 및 카메라 내부 파라미터 |
| 〃 | `/{arm}_wrist_camera/preview/compressed` [발행] | 모니터링용 압축 미리보기 영상 (JPEG) |
| 〃 | `/{arm}_wrist_camera/set_enabled` (`std_srvs/SetBool`) [제공] | 손목 카메라 USB 장치 동적 On/Off 제어 |
| 〃 | `/{arm}_wrist_camera/enabled` (`std_msgs/Bool`) [발행] | 카메라 활성화 상태 알림 (1 Hz, Reliable, Transient Local) |
| `rgbd_frame_bridge` | `/tf_static` (`tf2_msgs/TFMessage`) [발행] | 지정된 헤드 RGB-D 설정에 따른 정적 좌표계(TF) 브리지 |
| `motor_telemetry_recorder` | `joint_state_broadcaster/dynamic_joint_states` [구독] | 모터 전류/온도/부하 텔레메트리를 stdout JSONL로 기록 (수동 구독) |

- **안전 가드 기준**: 상태 데이터의 최대 허용 나이는 0.25초이며, 기동 대기 시간은 30초입니다. 지연되거나 결측된 상태는 즉시 감지되어 차단됩니다.
- **손목 카메라 생명주기**: 손목 카메라는 기본적으로 비활성(`start_enabled=false`) 상태로 대기하며, 제어 취소나 오류 발생 시 카메라 종료가 모터 비상 정지를 지연시키지 않도록 설계되었습니다.

## 2. TCP 지점 저장 및 재방문 (Waypoints Tool)

`tools/tcp-waypoints` 도구와 `tcp_waypoints.launch.py`를 통해 티칭 지점(Waypoint)을 저장하고 재방문할 수 있습니다.

- **세션 초기화**:
  ```bash
  tools/tcp-waypoints init --config /absolute/path/to/profile.yaml
  ```
  카메라 캘리브레이션과 원본 캡처 정보가 정상일 때만 세션이 초기화됩니다.
- **제어 범위 및 안전**:
  - 선택한 팔의 5관절만 제어하며, 머리나 그리퍼는 움직이지 않습니다.
  - `allow_torque_on_recovery=true` 옵션으로 수동 조작 시의 홀드(Hold) 상태를 이어받아 경로를 계획합니다.
- **토크 수동 제어**:
  - `tools/{left,right}-arm-torque-on --control-config /absolute/path/to/limits.yaml`
  - `tools/{left,right}-arm-torque-off` (선택된 모터 토크 해제)
