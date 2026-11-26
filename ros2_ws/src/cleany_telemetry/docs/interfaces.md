# cleany_telemetry 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리합니다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/implementation/ros-interfaces.md)을 따릅니다. 이름 설정이 없는 항목은 ROS remap으로 변경합니다.

## 책임 경계

`cleany_telemetry`는 로봇 관측을 읽기 전용으로 제공합니다. 위치와 카메라는 Backend와
무관하게 실행하며, 센서 장치를 열거나 조종 명령, 인식과 추론, 원본 녹화를 수행하지
않습니다. Backend 계약과 인증, 재접속은
[pose_backend_bridge](../../cleany_control_bridge/docs/interfaces.md#pose_backend_bridge)가
소유합니다. dev_monitor와 다른 ROS 소비자는 같은 관측 토픽을 사용할 수 있습니다.
카메라 미리보기는 dev_monitor와 Dashboard가 공유하지만 인식과 Logger는 원본 ROS
토픽을 계속 사용합니다.

## pose_observer

실행 파일: `pose_observer`. 원본: [pose_node.py](../cleany_telemetry/pose_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | 조건 |
|---|---|---|---|
| 토픽 구독 | `/odom` | `nav_msgs/msg/Odometry` | `source=odom`; `odom_topic` |
| TF 구독 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `source=tf`; tf2 기본 QoS |
| 토픽 발행 | `/telemetry/pose` | `geometry_msgs/msg/PoseStamped` | `pose_topic`; sensor data, best effort, volatile |

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `source` | `odom` | `odom` 입력 또는 `tf` 조회 |
| `target_frame`, `base_frame` | `map`, `base_link` | TF 조회 좌표계 |
| `rate` | `5.0` | TF 조회 주기(Hz); odom 모드는 수신 시 처리 |
| `input_timeout` | `1.5` | 원본 ROS 시각 기준 최대 입력 나이(초) |
| `future_tolerance` | `0.1` | 허용하는 미래 원본 시각 오차(초) |

위치(m), 정규화된 quaternion, 원본 `header.frame_id`와 `header.stamp`를 제공합니다.
유한하지 않은 위치·회전, 길이가 0인 quaternion, 비어 있는 좌표계와 0인 시각은
발행하지 않습니다. 동일하거나 역순인 표본은 재발행하지 않으며 ROS 시계 자체가
되돌아가면 새 시각부터 다시 관측합니다. 시뮬레이션에서는 생산자와 소비자 모두
`use_sim_time=true`를 사용해야 합니다.

TF 모드는 원본 변환의 시각을 보존하며 위치 추정을 요청하거나 TF를 발행하지 않습니다.
TF가 없거나 오래되면 출력이 멈춥니다. odom을 map으로 이름만 바꾸거나 마지막 위치를
현재 시각으로 갱신하지 않습니다. 소비자는 별도로 수신 중단을 감지해야 합니다.

기존 `telemetry_node`는 `pose_observer`와 `cleany_control_bridge/pose_backend_bridge`의
두 프로세스로 대체했습니다. 기존 `odom_topic`은 관측 노드로, `url`과 전송 설정은
bridge로 이동합니다. Backend가 필요 없는 관측에는 bridge를 실행하지 않습니다.

## camera_preview

실행 파일: `camera_preview`. 원본: [camera_node.py](../cleany_telemetry/camera_node.py).

| 종류 / 방향 | 이름 | ROS 타입 | QoS와 수명 |
|---|---|---|---|
| 토픽 구독 | HTTP에서 요청한 영상 토픽 | `sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage` | Sensor data, best effort, volatile; 조회 lease 만료 시 해제 |

ROS 발행, 서비스와 action은 없습니다. 위치 관측과 별도 프로세스로 실행하고
관측 대상 ROS graph에서 발견한 영상 토픽만 구독합니다. 구독할 토픽은 읽기 요청으로
선택하며 카메라 드라이버를 시작하지 않습니다. 발행 토픽이 사라지거나 타입이 바뀌면
해당 캐시와 lease를 지웁니다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `host`, `port` | `127.0.0.1`, `8769` | HTTP 주소; loopback만 허용 |
| `camera_max_streams` | `2` | 전체 소비자가 공유하는 동시 토픽 한도, 1~4 |
| `camera_fps` | `5.0` | 원시 영상의 최대 인코딩 빈도, 1~10 Hz |
| `lease_seconds` | `3.0` | 마지막 조회 이후 구독 유지 시간 |
| `stale_seconds` | `2.0` | 최신 프레임 제공 기한; lease보다 작아야 함 |
| `discovery_seconds` | `0.5` | ROS graph와 구독 갱신 주기 |

### HTTP 계약

| GET 경로 | 응답 |
|---|---|
| `/api/cameras` | `topics` 객체: 영상 토픽명 → ROS 메시지 타입 |
| `/api/camera?topic=/...` | 최신 프레임 JPEG 또는 아래 상태 |

정상 응답은 `image/jpeg`, `Cache-Control: no-store`, `X-Frame-Age`(수신 후 경과 초),
`X-Camera-Topic`(실제 구독 토픽), `X-Image-Width`, `X-Image-Height`, `X-ROS-Stamp`(원본 메시지 시각)를 포함합니다.
202는 최초 영상 대기 또는 오래된 영상, 400은 알 수 없거나 미리보기를 지원하지 않는 영상 토픽, 422는 변환 오류,
429는 동시 토픽 한도 초과입니다. 과거 프레임을 연결 오류의 대체 영상으로 반환하지 않습니다.
동일 토픽을 여러 화면이 조회해도 ROS 구독과 프레임 캐시는 하나입니다.

입력은 16 MiB와 2 MP로 제한하며 RGB/BGR/RGBA/BGRA/mono8, JPEG와 PNG를 지원합니다.
깊이 영상은 미리보기 대상이 아닙니다. 출력은 최대 1280×720 JPEG 품질 75이며,
이 범위 내 2 MiB 이하 JPEG는 컨테이너 검사 후 재인코딩 없이 전달합니다.
조회·프레임은 저장하지 않습니다. 로컬 HTTP 자체에는 사용자 인증이 없으므로
Dashboard 외부 접근은 Backend에서 사용자·시설 권한을 검사해야 합니다.

헤드 RGB와 손목 원시 영상 요청은 대응하는 압축 미리보기 토픽이 발견되면 그 토픽을
구독합니다. 알려진 metric-depth 토픽은 lease 생성 전에 거부합니다. 이미 인코딩된
JPEG는 다시 인코딩하지 않고 최신 수신 프레임을 유지하며 `camera_fps`로 감속하지 않습니다.

운영 카메라의 `allowed_topic` 값은 `/camera/head/color/image_preview_redacted`다. 이 값이 설정되면 HTTP 목록과 영상 요청에는 지정한 토픽만 노출한다. ROS 개발용 다중 카메라 프리뷰는 이 매개변수를 비워 사용한다.

## robot_observer

실행 파일은 `robot_observer`입니다. [robot_node.py](../cleany_telemetry/robot_node.py)는
관절과 배터리, Odometry, Supervisor 정보를 읽으며 명령, 서비스, TF를 발행하지 않습니다.
출력은 `/telemetry/robot_observations`의 `std_msgs/msg/String` JSON이며 reliable volatile
깊이100입니다. 출처 `provenance`를 `real`, `sim`, `mock` 중 하나로 명시해야 시작합니다.

| 기본 구독 | 타입 | QoS | 의미 |
| --- | --- | --- | --- |
| `/joint_states` | sensor_msgs/msg/JointState | sensor_data | 실제 관절 위치 |
| `joint_targets_topic` (기본 비활성) | sensor_msgs/msg/JointState | sensor_data | 별도 명령 목표; 실측 대체 금지 |
| `/battery_state` | sensor_msgs/msg/BatteryState | sensor_data | percentage 0~1을 0~100으로 변환; NaN은 null |
| `/odom` | nav_msgs/msg/Odometry | sensor_data | 원본 frame, 위치, 정규화 회전, 속도 |
| `/system/state` | cleany_interfaces/msg/SystemState | reliable transient_local, depth1 | 운용 상태 |
| `/system/faults` | cleany_interfaces/msg/SystemFaults | reliable transient_local, depth1 | 장애 목록 |
| `/system/recoveries` | cleany_interfaces/msg/SystemRecoveries | reliable transient_local, depth1 | 복구 상태 |
| `/system/events` | cleany_interfaces/msg/SystemEvent | reliable volatile, depth100 | 주요 사건 |
| `/safety/status` | cleany_interfaces/msg/SafetyStatus | reliable volatile, depth10 | 보호, 물리 해제, 정지 관측 |

토픽 이름, 출력 주기(10Hz), 원본 관측 만료(관절/odom 3초, Supervisor/safety 5초,
배터리30초), 미래 허용 오차(0.1초), 이벤트 용량(100)은 ROS 파라미터입니다.
[기본 설정](../../cleany_control_bridge/config/robot_observations.yaml)을 사용합니다.
관절 이름과 단위는 Robot description의 회전 rad, 직선 m을 그대로 사용합니다.
부분 JointState는 관절별 원본 시각을 유지하여 다른 관절 입력으로 갱신되지 않습니다.
오래된 입력, 잘못된 길이와 NaN은 거부합니다. ROS clock이 역행하면 새 boot_id로
전환합니다. odom을 map으로 바꾸지 않으며 명령 목표가 없으면 생성하지 않습니다.

관측 JSON 계약의 Source of Truth는 control-plane 저장소
`packages/contracts/schemas/robot-observation.schema.json`입니다. ROS clock의 원본 시각과
현재까지 경과한 나이, 관절별 추가 나이, boot_id와 sequence를 포함합니다. Backend 인증과
ACK 처리는 [robot_observation_bridge](../../cleany_control_bridge/docs/interfaces.md#robot_observation_bridge)가 소유합니다.
