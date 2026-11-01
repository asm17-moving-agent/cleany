# 실물 매핑 세션

`mapping_scan_filter`는 `scan_raw` 원본을 보존하고 base 전방 180도 중 0.30m 초과 값을
`scan`으로 발행한다. 후방과 가까운 finite 값은 NaN이며 clearing ray로 바꾸지 않는다.
센서 yaw를 이용해 전방을 센서 좌표로 변환한다.

`mapping_session`은 지도 세션과 Cartographer, Logger 자식 프로세스를 소유한다.
Agent, base, 조종 웹과 센서 프로세스는 bringup 수명이므로 지도 초기화로 재시작하지 않는다.

| ROS endpoint | 타입 | 계약 |
|---|---|---|
| `mapping/control` | `MappingControl` 서비스 | reset 또는 stop 접수. request_id와 operation 조합으로 수명 내 중복 제거 |
| `mapping/status` | `MappingStatus` | Reliable transient local depth 1. phase, generation, output과 이전 output |
| `teleop/status` | `TeleopStatus` 구독 | 정지 확인은 fresh connected, paused, disabled, stop_confirmed 모두 필요 |
| `teleop/pause` | `SetBool` client | 초기화와 종료 전 입력 차단. 준비 완료 뒤 차단만 해제 |
| `wheel_odometry/reset` | `Trigger` client | 이전 SLAM 종료와 STOP 확인 후 오도메트리 원점 재설정 |
| `write_state` | Cartographer `WriteState` client | 최종 pbstream 파일과 code=0 응답 모두 확인 |
| `/cleany_logger/*` | `RecordingControl` client | 자신이 시작한 session ID만 정지 |

상태는 idle, queued, stopping, waiting_usb, starting, running, failed다.
waiting_usb는 장치 파일만 검사하지 않고 실제 scan과 fresh MCU 연결을 기다린다.
새 세션은 scan 시각의 `map -> sensor` TF, fresh odom과 새 map을 확인해야 running이 된다.

초기화 순서는 입력 차단과 STOP 확인, 이전 지도/pbstream 저장, Logger flush, 이전 SLAM 종료,
odom reset, 새 Logger와 SLAM 시작이다. 저장 실패와 남은 자식은 다음 시작을 차단한다.
ESP32 연결 자체가 사라지면 STOP 미확인을 상태에 남기고 재연결을 기다린다.

`map.yaml`은 실제 origin yaw와 immutable PGM을 참조한다. 현재 지도 사본 `map.pgm`,
`latest.pbstream`, 선택 설정과 출처를 담은 `session.json`, Logger session을 가리키는
`recording.json`, 종료 결과를 세션별로 보존한다. 기존 세션 디렉터리를 덮어쓰지 않는다.
Map YAML과 이미지는 임시 파일과 원자적 교체로 갱신한다.

Cartographer는 map->odom, base relay는 odom->base_link, 단일 장착 TF는 base_link->sensor를
소유한다. AMCL이나 별도 SLAM, Gazebo canonical odom과 동시에 실행하지 않는다.
일반 Cartographer launch는 유지하고 실물 경로는 `cartographer_hardware_2d.lua`를 선택한다.
