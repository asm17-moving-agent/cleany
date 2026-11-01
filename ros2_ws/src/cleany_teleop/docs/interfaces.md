# 휴대폰 조종

`phone_remote`는 만료하는 브라우저 입력을 base driver에 전달한다. MCU 휠 명령은 발행하지 않는다.
단일 브라우저가 입력 lease를 소유하며 70ms 갱신, 250ms 입력 만료와 기존 해제 경계를 유지한다.

| 종류 | 기본 이름 | 타입 | 계약 |
|---|---|---|---|
| 발행 | `cmd_vel` | `Twist` | 차체 속도, 50Hz 상한. driver의 축별 한도와 휠 한도 적용 |
| 구독 | `base/wheel_state` | `WheelState` | sensor data QoS. 상태는 fresh MCU 응답 기준 |
| 서비스 호출 | `base/set_enabled` | `SetBaseEnabled` | 순서가 있는 구동 허가 요청. 응답만으로 구동 완료 판정 금지 |
| 서비스 제공 | `teleop/pause` | `SetBool` | true: 새 입력 차단과 STOP 요청. false: 차단 해제만 수행 |
| 발행 | `teleop/status` | `TeleopStatus` | Reliable depth 10. ready, pause, 연결, 정지 확인과 boot |
| 구독 | `mapping/status` | `MappingStatus` | Reliable transient local depth 1 |
| 서비스 호출 | `mapping/control` | `MappingControl` | 인증된 웹 초기화 요청을 매핑 관리자에 전달 |

`base/wheel_command` publisher와 `cmd_vel` publisher가 각각 하나여야 조종을 허용한다.
`--base-config`는 실행 중인 driver와 같은 설정이어야 한다. UI의 최대 휠 속도는 의도 상한이며
실제 전송 목표는 MCU `target_rad_s`다. 축별 한도 때문에 더 낮아질 수 있다.

브라우저 연결이 끊기면 정지한다. 같은 눌림을 유지하면 연결 복구 후 STOP 확인, 새 ENABLE과
새 입력으로 재개한다. 해제, STOP, pointer cancel, 화면 이탈은 자동 재개를 취소한다.
재시작한 서버는 이전 브라우저 lease를 복원하지 않으며 눌림 없는 자동 ENABLE은 하지 않는다.

| HTTP | 계약 |
|---|---|
| `GET /`와 `/assets/` | 설치된 shadcn UI. 자산 디렉터리 밖의 파일은 제공하지 않음 |
| `GET /api/status` | `X-Control-Token` 필수. 목표, 측정 속도, PWM, 진단과 최근 정지 사유 |
| `POST /api/control` | token과 동일 Origin, `ui_version=2`, client와 증가하는 sequence 필요 |
| `GET /api/mapping/status` | token 필수. 매핑 상태 조회 |
| `POST /api/reset` | token과 동일 Origin, `request_id` 필요. 202는 접수이며 running이 완료 |

키는 `#token=...`에서 읽고 저장하지 않는다. STOP 요청도 인증하며, stale request의 409를
최신 통신 단절과 구분한다. 초기화 응답이 지연되면 같은 request_id로 재시도한다.

`commission_wheels`는 기존 protocol-v2 독립 시험 CLI다. base driver와 함께 실행하지 않는다.
운영 launch와 휴대폰 조종은 이 경로를 사용하지 않는다. MCU 펌웨어는 이 패키지가 변경하지 않는다.

## 진단 상태와 이벤트 기록

`GET /api/mapping/status`는 마지막 ROS heartbeat의 `status_age_ms`와 `connected`를 제공한다.
수신 간격이 `--mapping-status-timeout`(기본 1초)을 넘으면 `phase=unavailable`을 반환한다.
마지막 세션 경로와 식별자는 유지하며 이전 단계는 `last_phase`에서 확인한다.
HTTP 연결이 유지되는 것만으로 매핑 관리자가 정상이라고 판단하지 않는다.

`GET /api/status`의 `permission`은 현재 권한 요청의 단계, client, request_sequence와
응답 boot_id, command_sequence를 제공한다. `last_permission_error`는 마지막 RPC 거절 또는
예외의 code, reason, exception_type과 traceback을 다음 요청 이후에도 보존한다. MCU ACK 대기와
boot 불일치는 각각 `mcu_ack_pending`, `mcu_boot_mismatch`로 구분한다.

조종 상태 전환과 정지 사건은 JSON Lines로 기록한다. 각 행에는 UTC, 단조 시각 `at`,
실행별 `run_id`, 증가하는 `event_id`, PID와 관련 요청 정보가 포함된다. 파일 쓰기는
별도 스레드가 처리하며 큐가 차거나 저장에 실패해도 조종 루프를 기다리게 하지 않는다.
API의 `event_log`에서 경로, 저장 수, 누락 수와 마지막 저장 오류를 확인한다.
`event_sink_error`는 이벤트 전달 자체의 예외를 나타낸다.

직접 실행 시 `--event-log`의 기본값은 `~/.local/state/cleany/teleop/events.jsonl`이다.
통합 launch는 state_root의 `teleop-events.jsonl`을 사용한다. 파일은 기본 2 MB마다 회전하며
백업 3개를 유지한다. 크기와 개수는 `--event-log-max-bytes`, `--event-log-backups`로 바꾼다.
기록은 장기 보관을 보장하지 않으며 강제 종료, 디스크 장애나 큐 포화 시 누락될 수 있다.
