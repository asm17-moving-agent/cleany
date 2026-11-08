# 휴대폰 조종

NavigationBridge는 `navigation/slip`, `navigation/traction_recovery`의
`std_msgs/msg/String` JSON을 읽고 `/api/navigation/status`에 `slip`,
`traction_recovery`로 제공한다. 화면의 감지 및 재시도 표시는 이 상태를 사용하며
구동 권한이나 명령 소유권을 추가하지 않는다. 기존 서버 프로세스에는 새
subscription이 자동 추가되지 않으므로 코드 반영 시 해당 서버를 다시 실행한다.

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

임시 전후진 및 회전 모드는 `--token-file`과 같은 디렉터리의 `teleop-motion.json`에
`{"forward_only": true}`를 저장하고 조종 서버를 재시작하면 적용된다. 파일이 없으면
기존 전체 방향 모드다. 복원하려면 값을 false로 바꾸고 조종 서버를 재시작한다.
이 모드에서는 UI의 횡이동 입력을 숨기고 이동 스틱은 전후 축만 사용한다. 회전 스틱과 a/d 키는 유지한다.
서버도 횡이동과 대각이동 명령을 거부하므로 이전 UI에서도 제한이 적용된다.
기존 설정 키 이름 `forward_only`는 유지하며 회전은 허용한다.
`GET /api/status`의 `forward_only`가 적용 상태를 나타낸다. STOP, 입력 만료와
기존 구동 허가 검증은 유지하며, 이 설정은 base driver나 다른 제어기를 변경하지 않는다.

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

## 지도 클릭 주행 (명시적 선택)

`phone_remote --navigation`은 `/motion/cmd_vel`로 선택된 속도를 보내며 기존 안전 체인의 최종 `/cmd_vel`을 사용한다. 일반 조종 실행은 변경하지 않는다. `GET /api/navigation/map`, `GET /api/navigation/status`는 토큰으로 보호된 지도/상태 조회다. `POST /api/navigation/{initial-pose,goal,cancel,heartbeat}`는 기존 Origin/토큰 검증을 사용한다. goal은 map_id, x/y(m), yaw(rad), request_id를 받는다. 초기 위치는 정지 상태에서만 설정하며, goal은 Supervisor NAVIGATION 허가, fresh map TF/scan, 안전 입력 subscriber와 단일 최종 출력 publisher를 요구한다. 수락된 자동 목표는 화면 잠금·앱 전환과 무관하게 로봇 쪽 Supervisor와 센서 감시 아래 계속 실행된다. 수동 조종의 입력 만료와 화면 이탈 정지는 유지한다.

센서 갱신 지연으로 자동 목표가 취소되면 상태 응답의 `last_input_failure`와 event log의 `navigation_input_failure`에 입력별 수신 나이를 기록한다. 정지 완료 후에도 구체적인 취소 사유를 유지한다.

하드웨어 실행의 `phone_motion_profile=omni`는 상태의 `forward_only=false`와 Omni MPPI를 함께 선택한다. 수동 조이스틱과 자동 주행 모두 body frame의 +Y 좌측 이동을 허용하며, 전후·횡이동·회전이 섞인 자동 명령은 네 휠의 최대 절댓값을 기준으로 같은 비율로 제한한다. `longitudinal`은 기존 횡이동 차단을 유지한다.

실물 자동 주행은 아직 별도 통합 검증이 필요하다. 플러그인 로딩이나 mock 테스트만으로 실물 구동이 검증됐다고 해석하지 않는다.

ROS 연결: `/map` (OccupancyGrid, transient-local), `/plan` (Path), `/scan` (LaserScan), `/odom` (Odometry), `/nav2/cmd_vel` (Twist)을 구독한다. `/initialpose`를 발행하며, 자동 주행은 `navigate_to_pose` Action과 기존 Supervisor NAVIGATION lease를 사용한다. `navigation/stop` (StopMission Action)은 소유 session의 epoch/lease를 검증하고 MCU STOP 확인 뒤에만 성공한다. 일반 조종과 자동 주행은 같은 DriverControl을 사용한다.

재기동 직후 목표가 없는 idle bridge는 현재 Supervisor epoch의 이전 lease 정지 요청도
수락한다. 새 목표나 구동 허가를 만들지 않고 STOP을 요청하며, 신선한 MCU ACK,
측정 휠 속도 0.05rad/s 미만, 출력 정지와 진행 중인 목표 전송/취소 완료를 확인한
뒤에만 stopped를 반환한다. 다른 epoch나 새 목표 실행 중의 무관한 lease는 거부한다.
기동 직후 STOP 응답의 MCU boot_id가 아직 0이면 실제 피드백을 받은 뒤 새 STOP을
요청한다. boot_id 0 응답을 정지 ACK로 인정하거나 복구를 위해 ENABLE하지 않는다.

`localization/status`(std_msgs/String)를 구독해 지도 정합과 위치 복구 상태를
`/api/navigation/status`의 `localization`에 포함한다. Supervisor가 정합 미확인으로
NAVIGATION을 거부하면 화면에 그 원인을 표시한다. 수동 조종과 초기 위치 지정은
가능하지만 자동 목표는 새 위치 정합과 새 Supervisor 허가가 확인돼야 수락한다.

LiDAR 전용 시연에서 자동 intent는 `/motion/cmd_vel` → Collision Monitor → `/navigation/safe_cmd_vel`로 흐른다. `phone_remote`가 소유 자동 goal과 구동 상태를 확인해 최종 `/cmd_vel`로 전달한다. 수동 조작은 기존 operator-controlled 경로를 유지하고 자동 goal 취소·정지 확인 후에만 전환한다. 최종 `/cmd_vel` 발행자는 하나다.

Nav2 전역 계획 `/plan` (`nav_msgs/Path`, reliable/volatile)을 map 좌표의 파란 선으로 표시한다. 준비 중과 대기 중 계산된 경로도 수신한다. 다른 frame 또는 non-finite 점은 무시하고, 빈 경로·지도 변경·새 목표·취소는 기존 경로를 지운다. 상태 API는 `global_path_topic`, `global_path_points`, `global_path_age_s`를 함께 제공한다. 화면에서 경로 표시를 켜고 끌 수 있으며 계획이 없으면 “경로 생성 대기”로 표시한다.

`POST /api/navigation/preview`는 map_id, x/y(m), yaw(rad), request_id를 받아 `ComputePathToPose`만 호출한다. Supervisor lease 취득, base ENABLE, 속도 발행은 하지 않는다. 상태 API의 `preview_request_id`, `preview_phase`, `preview_path`, `preview_error`, `preview_target`(반환 경로 끝점의 x/y/yaw)로 결과를 제공한다. 화면은 미리보기 완료 시 빨간 목표를 반환 끝점으로 갱신하고 이 자세를 이동 요청에 사용한다. 순정 Lattice의 격자/방향 양자화 결과를 사용자가 이동 전에 확인하며, 도착 판정의 3cm/0.1rad 허용오차는 그대로 유지한다. 초기 위치 설정은 이 변환을 사용하지 않는다. 목적지 변경, 지도 변경, 취소와 실제 이동 시작은 이전 미리보기를 무효화한다. 늦은 응답은 요청 generation으로 배제한다.

휴대폰의 HTTP LAN 접속에서도 목적지 선택과 이동 요청을 지원한다. 요청 식별자는 secure context 전용 API에 의존하지 않는다.

`config/phone_navigation.yaml`의 `navigation.max_wheel_rad_s`는 자동 intent의 최종
휠 상한이다. 현재 10rad/s이며 혼합 명령도 같은 비율로 축소해 드라이버 구동 범위에
맞춘다. Omni의 주행 속도는 MPPI의 병진 합성 0.5m/s와 회전 0.7rad/s가 결정한다.
뒤의 `phone_collision_monitor`는 주황 영역에서 병진 0.16m/s와 회전 0.3rad/s를
적용한다. 작은 명령을 상한까지 키우지 않는다.

조종 UI의 `/` 응답은 요청마다 현재 `index.html`을 읽는다. 서버가 실행된 동안
UI를 다시 빌드해 해시가 포함된 asset 이름이 바뀌어도 이전 HTML을 계속 제공하지 않는다.
파일 교체 중 entrypoint가 없으면 503을 반환하며, UI와 asset 응답은 `no-store` 정책을 유지한다.
