# 실물 상태와 컨트롤러 경계 인터페이스

본 문서는 실물 로봇의 엔코더 조회 노드, 제어기 안전 가드(Guard), 손목 카메라의 ROS 2 인터페이스를 설명합니다.

> [!CAUTION]
> 엔코더 상태 조회 노드(`reader.py`, `robot_state_reader.py`)와 실물 제어기(`real_control.launch.py`)가 동일한 시리얼 포트를 동시에 점유해서는 안 됩니다.

## 1. 노드별 인터페이스 목록

| 노드명 | 인터페이스 명칭 및 타입 | 역할 및 동작 상세 |
|---|---|---|
| `arm_state_reader` | `joint_states` (`sensor_msgs/JointState`) [발행] | 설정된 단일 팔의 실측 관절 위치 |
| `read_only_robot_state` | `/joint_states` (`sensor_msgs/JointState`) [발행] | 양팔 및 헤드 전체 실측 관절 위치 (기본 20 Hz) |
| `real_control_state_guard` | `joint_state_broadcaster/dynamic_joint_states` (`control_msgs/msg/DynamicJointState`) [구독] | 최신 프레임 하나에서 모터 위치와 공통 취득 시각을 확인한다. QoS는 best effort다. |
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

가드는 실제 `feedback_stamp_ns`로 관절 상태를 발행한다. 발행 헤더만 새로워진
프레임은 신선도를 갱신하지 않는다. 그리퍼 위치가 같으면 CAD 계산을 재사용한다.
Feetech 드라이버가 오류를 고정하면 취득 시각의 갱신이 멈춘다. 가드는
프레임에 오류 비트가 있으면 함께 검사하고, `require_driver_fault=true`를
선택한 경우에는 해당 비트가 없는 프레임도 거부한다.

## 2. 토크 수동 제어

선택한 팔의 토크는 `tools/{left,right}-arm-torque-on --control-config /absolute/path/to/limits.yaml`
및 `tools/{left,right}-arm-torque-off`로 제어합니다. 상태 조회기나 컨트롤러가 같은 포트를 점유한 동안 실행하지 않습니다.

## 보정된 카메라 TF

`calibrated_camera_pose`는 `config_file`의 고정된 보정 모델과 관절 좌표 변환을
사용한다. `/joint_states`의 위치는 rad이며, 수신 시각 기준 0.5초 이상 지난
상태는 사용하지 않는다. 보정 모델, 관절 매핑과 모터 보정의 해시가 달라지면
초기화에 실패한다. 장치 연결과 모터 명령은 수행하지 않는다.

| 구분 | 기본 endpoint | 타입 | 의미 |
|---|---|---|---|
| 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | 운영 관절 위치, sensor-data QoS |
| TF | `/tf` | `tf2_msgs/msg/TFMessage` | `base_link`에서 설정된 광학 프레임으로의 동적 변환 |
| 발행 | `/camera_calibration/pose_status` | `std_msgs/msg/String` | 선택 파일, 해시, 갱신된 카메라와 상태 나이의 JSON |

상태 프로파일의 `camera_pose_config`가 선택되면 기존 광학 프레임의 고정
분기를 제거해 TF 소유권을 이 노드로 넘긴다. 로봇의 가동 관절이나 형상이
포함된 링크는 제거할 수 없다. 일부 팔만 존재할 때는 해당 팔의 상태로
계산할 수 있는 카메라만 발행한다.


## 제어기 완료 알림 주기

외부 Feetech 제어 프로파일의 `controller.type`은 제어기 plugin을 선택한다.
생략 시 `joint_trajectory_controller/JointTrajectoryController`를 사용한다.
Jetson 이벤트 전달 구성은 `cleany_trajectory_controller/JointTrajectoryController`를
명시한다. `controller.action_monitor_rate`는 유한한 1–100Hz이며 기본값은
20Hz다. 기존 제어기에서는 결과 확인과 피드백 주기이며, 이벤트 제어기에서는
진행 상태 주기다. 이벤트 구성은 20Hz를 사용하고 완료, 실패와 사용자 취소는
발생 시 executor를 깨워 전달한다. 제어 주기와 이동 제한은 별도다.
자세한 상태 전이는 [이벤트 제어기 계약](../../cleany_trajectory_controller/docs/interfaces.md)을 따른다.

운영 모델의 고정 카메라 마운트 형상 등록은 [공통 파지 설정](../../cleany_skill_executor/docs/shared-grasp-policy.md)을 따른다. 광학 TF와 제어 캘리브레이션은 기존 경로를 유지한다.

보정된 CAD 프로파일에서 `control_state_guard`의 `cad_calibration_json`은 기존
모터 피드백에 계산한 손가락 관절 16개를 추가한다. 입력 모터의 신선도와
정상성을 먼저 검사하며, 계산 관절은 모터 명령이나 추가 실측 피드백으로
취급하지 않는다. 연동 실패 시 상태 발행을 중단하고 정지 래치를 유지한다.
[CAD 파지 모델 계약](../../cleany_moveit_collision/docs/calibrated-cad.md)을 따른다.


arm_preview.launch.py는 서보 버스를 열지 않고 공통 CAD 표시로 연결한다. 토픽과 상태는 [공통 CAD 런타임](../../cleany_description/docs/runtime-model.md)을 따른다.
