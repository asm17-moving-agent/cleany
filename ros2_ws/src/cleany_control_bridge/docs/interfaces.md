# cleany_control_bridge 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리합니다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/implementation/ros-interfaces.md)을 따릅니다. 이름 설정이 없는 항목은 ROS remap으로 변경합니다.

## control_bridge

실행 파일: `control_bridge`. 원본: [노드.py](../cleany_control_bridge/node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 서비스 호출 | `mission/offer` | `cleany_interfaces/srv/OfferMission` | Service 기본값 |
| 서비스 호출 | `mission/cancel` | `cleany_interfaces/srv/CancelMission` | Service 기본값 |
| 서비스 호출 | `mission/snapshot` | `cleany_interfaces/srv/GetRuntimeSnapshot` | Service 기본값 |

ROS 토픽이나 action을 직접 제공하지 않습니다. Backend WebSocket은 `url`로 설정하고 상태 동기화 간격은 `heartbeat_interval` 기본 1초입니다. [Gateway 계약](../../cleany_interfaces/docs/mission.md)을 따릅니다.

`mission/snapshot` 응답 JSON에 `target_map`이 있으면 `robot.snapshot` payload로 그대로
전달합니다. 이 값은 Runtime이 로드한 좌석 목표 파일의 지도 ID, 원본 해시, 배치 revision과
목표 digest입니다. Bridge가 디스크 파일만 읽어 Runtime의 적용 상태로 보고하지 않습니다.
필드가 없는 기존 Runtime에는 이전 snapshot 계약을 유지합니다.

## pose_backend_bridge

실행 파일: `pose_backend_bridge`. 원본: [pose_node.py](../cleany_control_bridge/pose_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS와 설정 |
|---|---|---|---|
| 토픽 구독 | `/telemetry/pose` | `geometry_msgs/msg/PoseStamped` | sensor data, best effort, volatile; `pose_topic` |

ROS 발행, 서비스, action과 TF는 없습니다. Mission의 영속 명령 bridge와 별도 프로세스로
실행합니다. 위치는 최신값 하나만 유지하며 재접속 때 과거 표본을 재생하지 않습니다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `url` | `ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws` | Backend pose endpoint |
| `device_token_file` | 빈 문자열 | 장치 토큰 파일; 설정 시 연결마다 읽어 Bearer 인증에 사용 |
| `map_id`, `map_frame` | 빈 문자열, `map` | 해당 좌표계에서만 보낼 Backend 지도 ID |
| `rate` | `5.0` | 최대 전송 빈도(Hz) |
| `input_timeout`, `future_tolerance` | `1.5`, `0.1` | 원본 시각의 최대 나이와 미래 허용 오차(초) |
| `reconnect_initial`, `reconnect_max` | `1.0`, `30.0` | 재접속 지수 backoff 범위(초) |

Backend payload는 `x`, `y`, `yaw`, `frame_id`와 선택적 `map_id`입니다. map_id는
입력 좌표계가 map_frame과 일치할 때만 포함합니다. 지도 ID는 실제 localization에
사용한 지도와 맞춰야 하며 이 설정은 좌표 변환을 수행하지 않습니다. Backend의
`received_at`은 서버 수신 시각입니다. 원본 관측 시각은 ROS 메시지에 유지되며 현재
Backend 계약에 별도 필드로 추가하지 않습니다.

원본 시각은 같은 ROS 시계로 검사하고, 이미 경과한 입력 나이를 단조 시계의 전송
유효기간에서 차감합니다. 같은 시각의 반복 입력은 유효기간을 연장하지 않습니다.
Backend 연결 실패는 관측 노드에 영향을 주지 않습니다. 토큰 파일 경로만 ROS
파라미터로 전달하며 내용은 로그에 기록하지 않습니다. 인증이 필요한 Backend에는
파일을 반드시 설정해야 합니다.

Gateway 인증을 사용하는 경우 `device_token_file`에 로봇 토큰 파일 경로를 지정합니다. Bridge는 WebSocket에 연결할 때마다 파일을 읽어 Bearer 인증 헤더를 전송합니다.
