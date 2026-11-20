# 휴대폰 조종과 자동 주행 인터페이스

`phone_remote`는 브라우저 입력을 base driver에 전달합니다. MCU 휠 명령은 발행하지 않으며 단일 브라우저가 입력 lease를 소유합니다.

```mermaid
flowchart LR
    UI["브라우저 / token + 입력 lease"] --> Server["phone_remote"]
    Server --> Driver["Base driver / 같은 base-config"]
    Driver --> MCU["MCU"]
    MCU -->|신선한 WheelState| Server
    Server -->|상태 / 정지 확인| UI
```

## 수동 조종과 ROS 연결

| 종류 | 기본 이름 | 타입 / 계약 |
|---|---|---|
| 발행 | `cmd_vel` | Twist, 50 Hz 상한; driver 축별 / 휠 상한 적용 |
| 구독 | `base/wheel_state` | WheelState, SensorData QoS; 신선한 MCU 응답으로 상태 판정 |
| 호출 | `base/set_enabled` | SetBaseEnabled; 순서 있는 허가 요청, 응답만으로 구동 완료 판정 금지 |
| 제공 | `teleop/pause` | SetBool; true는 입력 차단 + STOP, false는 차단 해제만 |
| 발행 | `teleop/status` | TeleopStatus, Reliable depth 10; ready / pause / 연결 / 정지 / boot |
| 구독 | `mapping/status` | MappingStatus, Reliable / Transient Local depth 1 |
| 호출 | `mapping/control` | MappingControl; 인증된 초기화 전달 |

| 수동 입력 조건 | 동작 |
|---|---|
| 갱신 / 만료 | 70 ms / 250 ms |
| Publisher 단일성 | `base/wheel_command`와 `cmd_vel` 각각 하나여야 허용 |
| 설정 | `--base-config`는 실행 driver와 동일 |
| 속도 해석 | UI 최대 휠 속도는 의도 상한, 실제 전송 목표는 MCU `target_rad_s` |

```mermaid
stateDiagram-v2
    [*] --> 대기
    대기 --> 조종: 새 눌림 / ENABLE / 새 입력
    조종 --> 정지확인: 연결 단절
    정지확인 --> 조종: 같은 눌림 유지 / 연결 복구 / STOP 확인 / 새 ENABLE과 입력
    조종 --> 대기: 해제 / STOP / pointer cancel / 화면 이탈
    정지확인 --> 대기: 해제 / STOP / pointer cancel / 화면 이탈
```

서버 재시작은 이전 lease를 복원하지 않고 눌림 없이 ENABLE하지 않습니다. `commission_wheels`는 protocol-v2 독립 시험 CLI이므로 base driver와 동시에 실행하지 않습니다.

### 전후진과 회전만 허용하기

| 항목 | 방법 / 의미 |
|---|---|
| 설정 파일 | `--token-file`과 같은 디렉터리의 `teleop-motion.json` |
| 활성 | `{"forward_only": true}` 저장 후 조종 서버 재시작 |
| 복원 | false 또는 파일 없음 + 재시작 |
| 허용 | 전후 스틱, 회전 스틱과 a/d 키 |
| 차단 | UI 횡이동 숨김, 서버도 횡이동 / 대각 명령 거부 |
| 상태 | `GET /api/status`의 `forward_only` |

키 이름은 forward_only지만 회전은 허용합니다. STOP·입력 만료·허가 검사는 유지하며 driver나 다른 controller는 변경하지 않습니다.

## HTTP와 인증

| HTTP | 계약 |
|---|---|
| `GET /`, `/assets/` | 설치 shadcn UI; asset 디렉터리 밖 파일 제공 안 함 |
| `GET /api/status` | X-Control-Token 필수; 목표 / 측정 속도 / PWM / 진단 / 최근 정지 사유 |
| `POST /api/control` | Token + 동일 Origin + ui_version=2 + client + 증가 sequence |
| `GET /api/mapping/status` | Token 필수 |
| `POST /api/reset` | Token + 동일 Origin + request_id; 202는 접수, running이 완료 |

Token은 `#token=...`에서 읽고 저장하지 않습니다. STOP도 인증합니다. Stale 요청 409는 통신 단절과 구분하고, 초기화 응답 지연은 같은 request_id로 재시도합니다.

| UI 갱신 | 동작 |
|---|---|
| `/` 요청 | 현재 index.html을 매번 읽어 새 해시 asset 반영 |
| 파일 교체 중 entrypoint 없음 | 503 |
| UI / asset 캐시 | no-store |
| HTTP LAN | Secure-context 전용 API에 의존하지 않는 요청 ID로 목적지 선택 / 이동 지원 |

## 진단 상태와 이벤트 기록

| 상태 필드 | 해석 |
|---|---|
| Mapping `status_age_ms`, `connected` | 마지막 ROS heartbeat 기준; HTTP 연결 유지와 구분 |
| `--mapping-status-timeout` | 기본 1 s 초과 시 phase=unavailable; 마지막 경로 / ID와 last_phase 보존 |
| `permission` | 요청 단계, client, request_sequence, 응답 boot_id / command_sequence |
| `last_permission_error` | 마지막 RPC 거절 / 예외의 code, reason, exception_type, traceback; 다음 요청 뒤에도 보존 |
| MCU 대기 / boot 불일치 | `mcu_ack_pending` / `mcu_boot_mismatch` |
| `event_log` | 경로, 저장 / 누락 수, 마지막 저장 오류 |
| `event_sink_error` | 이벤트 전달 자체의 예외 |

| JSONL 기록 | 계약 |
|---|---|
| 행 식별 | UTC, 단조 시각 at, run_id, 증가 event_id, PID와 요청 문맥 |
| 쓰기 | 별도 스레드; 큐 포화 / 저장 실패가 조종 루프를 막지 않음 |
| 직접 실행 기본 | `~/.local/state/cleany/teleop/events.jsonl` (`--event-log`) |
| 통합 launch | state_root의 `teleop-events.jsonl` |
| 회전 | 기본 2 MB, 백업 3개; `--event-log-max-bytes`, `--event-log-backups` |
| 한계 | 강제 종료 / 디스크 장애 / 큐 포화 시 누락 가능; 장기 보관 보장 아님 |

## 지도 클릭 주행 (명시적 선택)

`phone_remote --navigation`은 일반 수동 조종과 별도 선택입니다. 수락한 자동 목표는 화면 잠금·앱 전환 중에도 Supervisor와 센서 감시 아래 계속 실행됩니다. 수동 조종은 기존 입력 만료와 화면 이탈 정지를 유지합니다.

```mermaid
sequenceDiagram
    participant UI as 휴대폰
    participant B as NavigationBridge
    participant N as Nav2
    participant S as Supervisor
    UI->>B: preview / map_id, x, y, yaw, request_id
    B->>N: ComputePathToPose
    N-->>B: 경로와 양자화된 끝점
    B-->>UI: preview_target / 빨간 목표 갱신
    UI->>B: 표시된 자세로 goal 요청
    B->>S: NAVIGATION 허가
    B->>B: map TF / scan / 안전 연결 확인
    B->>N: navigate_to_pose
```

| HTTP | 입력 / 결과 |
|---|---|
| GET `/api/navigation/map`, `/status` | Token으로 보호된 지도 / 상태 |
| POST `/api/navigation/{initial-pose,goal,cancel,heartbeat}` | 기존 Origin / token 검사 |
| Goal | map_id, x/y(m), yaw(rad), request_id |
| Initial pose | 정지 상태에서만 설정; preview 양자화 변환 미적용 |
| Preview | ComputePathToPose만 호출; lease 취득 / ENABLE / 속도 발행 없음 |
| Preview 상태 | preview_request_id, phase, path, error, target(x/y/yaw) |
| Preview 무효화 | 목적지 / 지도 변경, 취소, 이동 시작; 늦은 응답은 generation으로 제외 |

도착 판정은 표시된 목표의 3 cm / 0.1 rad를 유지합니다. 초기 위치 설정과 수동 조종이 가능하더라도 자동 목표에는 새 위치 정합과 새 Supervisor 허가가 필요합니다.

### 자동 속도와 정지 경로

```mermaid
flowchart LR
    Nav["자동 intent / motion/cmd_vel"] --> CM["Collision Monitor"]
    CM --> Safe["navigation/safe_cmd_vel"]
    Safe --> Owner["phone_remote / 소유 goal과 구동 상태 확인"]
    Owner --> Cmd["cmd_vel / 최종 publisher 하나"]
    Manual["수동 전환 요청"] --> Cancel["자동 goal 취소 / 정지 확인"]
    Cancel --> Operator["기존 operator-controlled 경로"]
```

| 프로필 / 제한 | 값 |
|---|---|
| 하드웨어 Omni | `forward_only=false`, Omni MPPI; body +Y 좌측 이동 허용 |
| Longitudinal | 횡이동 차단 유지 |
| 자동 MPPI 상한 | 병진 합성 0.5 m/s, 회전 0.7 rad/s |
| 주황 영역 | 병진 0.16 m/s, 회전 0.3 rad/s |
| 최종 휠 상한 | `phone_navigation.yaml`의 `navigation.max_wheel_rad_s=10` |
| 혼합 명령 | 네 휠 최대 절댓값으로 같은 비율 축소; 작은 요청을 키우지 않음 |

실물 자동 주행은 별도 통합 검증 대상입니다. Plugin 로딩이나 mock 통과가 실물 구동 증거는 아닙니다.

### ROS 연결과 재기동 후 정지

| 방향 | Endpoint / 계약 |
|---|---|
| 구독 | `/map` OccupancyGrid transient-local, `/plan` Path reliable/volatile, `/scan` LaserScan, `/odom` Odometry, `/nav2/cmd_vel` Twist |
| 발행 / Action | `/initialpose`, `navigate_to_pose`, 기존 NAVIGATION lease |
| 정지 Action | `navigation/stop` StopMission; 소유 session의 epoch/lease 검증, MCU STOP 확인 뒤 성공 |
| 공통 제어 | 일반 조종과 자동 주행이 같은 DriverControl 사용 |
| 허가 전 연결 | Fresh map TF/scan, 안전 입력 subscriber, 단일 최종 publisher |

| 재기동 idle bridge | 처리 |
|---|---|
| 현재 epoch의 이전 lease 정지 요청 | 목표나 새 허가 없이 STOP 수락 |
| stopped 근거 | 신선한 MCU ACK, 휠 속도 < 0.05 rad/s, 출력 정지, 진행 중 goal 전송 / 취소 완료 |
| 다른 epoch / 새 목표와 무관한 lease | 거부 |
| STOP 응답 boot_id=0 | 실제 feedback 뒤 새 STOP; 0을 ACK로 인정하거나 ENABLE로 복구하지 않음 |

### 주행 상태 표시

| 상태 | 출처 / 처리 |
|---|---|
| `slip`, `traction_recovery` | navigation/slip, navigation/traction_recovery의 String JSON; 표시용이며 권한 추가 없음 |
| `localization` | localization/status String; 정합 미확인으로 허가 거부 시 이유 표시 |
| `tf_error` | 마지막 TF 조회 오류; 정상 새 TF에서 해제. 예상 밖 처리 오류는 위치를 즉시 무효화하고 5 s 간격 제한 오류 로그 출력 |
| `last_input_failure` | 센서 지연 취소의 입력별 수신 나이; event log의 navigation_input_failure와 함께 정지 뒤에도 보존 |
| 파란 전역 경로 | `/plan`의 유한한 map-좌표계 점; 빈 경로 / 지도 변경 / 새 목표 / 취소 시 제거 |
| 경로 상태 | global_path_topic, global_path_points, global_path_age_s; 없으면 “경로 생성 대기” |

Subscription 변경은 기존 서버 프로세스에 자동 반영되지 않으므로 코드 반영 뒤 서버를 재실행합니다.
