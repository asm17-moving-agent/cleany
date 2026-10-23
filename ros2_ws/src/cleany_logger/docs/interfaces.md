# cleany_logger 노드 인터페이스

## cleany_logger

실행 파일은 `logger_node`이며 원본은 [node.py](../cleany_logger/node.py)다.
기본 노드 이름은 `/cleany_logger`다. 녹화 제어는 로봇 구동 명령을 생성하지 않는다.

| 종류 | 기본 이름 | 타입 | 계약 |
|---|---|---|---|
| 토픽 발행 | `~/status` | `cleany_interfaces/msg/RecordingStatus` | Reliable, Transient Local, depth 1, steady clock 2 Hz |
| 서비스 제공 | `~/get_config` | `cleany_interfaces/srv/RecordingControl` | 현재 설정과 writer 상태 조회 |
| 서비스 제공 | `~/configure` | 같은 타입 | 정지 상태에서만 전체 설정 적용, 선택적 revision 비교 |
| 서비스 제공 | `~/start` | 같은 타입 | 활성 상태에서는 같은 세션을 반환, 정지 후에는 새 UUID 생성 |
| 서비스 제공 | `~/stop` | 같은 타입 | 현재 `session_id` 필수, 다른 세션 거절, flush와 종료 후 응답 |
| 서비스 요청 | `/submap_query` | `cartographer_ros_msgs/srv/SubmapQuery` | 기존 가공 기록 호환을 위한 읽기 전용 Cartographer 조회 |

공유 관측기의 기본 ROS 입력과 QoS 선택은 [Monitor 입력 표](../../cleany_dev_monitor/docs/interfaces.md#cleany_dev_monitor)를
따른다. logger에는 브라우저 선택이나 카메라 preview lease가 없으며, 녹화 선택은
자체 discovery에 적용한다. 원본 CDR 허용 타입은 `recording.py`의 `CONTEXT`, `TYPES`에서
관리한다. 지도, 경로, 위치, 속도, Runtime, BT, 안전 진단, 로그와 SLAM 원문, 선택한
센서 원문을 기록하고 TF, clock, CameraInfo와 로봇 모델을 함께 요청한다.
Discovery된 타입을 읽더라도 원본 허용 타입과 다르면 오류를 표시하고 저장하지 않는다.

`RecordingConfig`는 preset, MiB 한도, category별 선택과 주기, source별 토픽과
선택과 주기를 담는다. `RecordingSource`의 topics, enabled와 rate_hz 배열은 같은
원소 순서를 쓰며, 내부 계산 출처는 빈 topics와 한 개의 enabled/rate_hz를 쓴다.
`rate_hz=-1`은 수신마다 기록한다는 뜻이다. 그 밖의 유한한 양수만 허용한다.

`RecordingControl` 요청의 `expected_revision=0`은 비교를 생략한다. 실패 응답에는
`success=false`, 사유와 현재 설정과 상태를 담는다. `request_id`는 호출 식별용이며
영속 중복 제거 키가 아니다. 시작과 정지는 상태와 session ID로 멱등 처리한다.

상태는 `IDLE`, `RECORDING`, `STOPPING`, `ERROR`를 구분한다. `boot_id`는 프로세스
수명, `session_id`는 기록 수명이며 `revision`은 정책과 큐 ticket 세대다. `updated_at`과
counter의 `received_at`은 wall 초, `stored_bytes`는 세션 SQLite와 bag 합계다.
관측 간격은 logger 설정값이며 Monitor의 호스트 측정 간격과 구분한다.
카운터는 새 설정과 새 녹화 시 초기화하고 정지 응답에는 마지막 누적값을 유지한다.

`record_on_start`, 저장 경로와 한도, 입력 토픽과 frame은 ROS parameter다. 기본값은
[logger.yaml](../config/logger.yaml)과 공유 관측기 [observer.py](../cleany_logger/observer.py)를
따른다. HTTP endpoint는 logger에 없다.

실물 서버의 heartbeat를 관측할 때는 `manipulation_observation_mode=server`와
실물 `manipulation_events_topic`을 함께 지정한다. 기본 `mission` 모드는 같은
revision의 반복 기록을 무시하며, `server` 모드는 동일한 활성 기록의 수신 시각만
갱신한다. 완료 기록, 서버 timestamp와 단계 이력은 heartbeat로 바꾸지 않는다.

## CLI

실행 파일 `logger`는 `/cleany_logger_cli_<임의 suffix>` 임시 ROS 노드를 사용한다.
[cli.py](../cleany_logger/cli.py)의 명령 처리는 저장 파일을 열지 않으며
[cli_client.py](../cleany_logger/cli_client.py)가 위 네 서비스를 호출한다. 별도 logger
노드가 같은 ROS domain에서 실행 중이어야 한다. 실행 예시는 [RUNNING](../../../../docs/RUNNING.md#독립-logger)을 따른다.

| 명령 | 계약 |
|---|---|
| `status` | `get_config`로 현재 상태, session, 저장 바이트와 counter를 조회 |
| `start` | 현재 revision을 확인하고 시작, 이미 녹화 중이면 같은 세션 |
| `stop` | 현재 session과 revision을 조회한 뒤 해당 세션만 정지, `--session`으로 명시 가능 |
| `topics`, `topics list` | 센서 원본과 가공 출처의 ID, 그룹, 실제 선택 여부, 토픽과 주기 조회 |
| `topics set <ID>` | 정지 상태에서 `--topic`, `--hz`, `--enable`, `--disable` 중 지정한 값만 변경 |

`--node`는 절대 logger 노드 경로이며 기본 `/cleany_logger`, `--timeout`은 RPC별
발견과 응답을 합한 제한이며 기본 3초다. `--json`은 성공 결과를 stdout JSON으로
출력한다. CLI 오류와 대용량 원본 안내는 stderr로 출력한다. 조회 결과의 `settings`는
전체 정책이며 `topics`의 `enabled`는 상위 그룹과 개별 선택을 함께 반영한다.

설정은 조회한 revision과 비교해 적용하므로 동시에 바뀐 정책을 덮어쓰지 않는다.
상위 그룹이 꺼져 있을 때 하나의 출처를 활성화하면 다른 출처는 꺼진 상태를 유지한다.
토픽 변경만으로 기록 선택을 켜지는 않는다. 설정은 logger가 저장하며 다음 실행에도
복원한다. 지원하는 ID는 logger의 출처 schema와 센서 원본 역할에서 제공한다.

| 종료 코드 | 의미 |
|---|---|
| `0` | 조회 또는 요청 완료, 시작은 RECORDING, 정지는 IDLE 확인 |
| `2` | 잘못된 인자나 설정, ROS Python 의존성 누락 |
| `3` | 서비스 발견 또는 응답 제한 초과 |
| `4` | 상태, session, revision 충돌 또는 logger의 요청 거부와 종료 실패 |
| `130` | 사용자 인터럽트 |

Timeout과 인터럽트는 요청 취소나 정지 완료를 뜻하지 않는다. 처리 중인 요청이
완료됐을 수 있으므로 `status`로 확인한다. CLI 종료는 별도 정지 요청을 보내지 않는다.
