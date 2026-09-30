# cleany_control_bridge

Control Plane Backend의 Gateway WebSocket을 Mission Runtime의 ROS service에 연결합니다.
Queue와 배차는 Backend, 실제 수락·FSM·작업·최종 보고는 Runtime이 소유합니다.
이 패키지는 Nav2나 `/cmd_vel`을 직접 호출하지 않습니다.

## 계약과 영속성

`ws(s)://<backend>/api/robots/cleany-01/gateway/ws`의 schema v1을 사용합니다.
`mission.offer: clean_seat`를 ROS `OfferMission: clean_desk, SEAT`로 변환하고,
`mission.cancel`은 `CancelMission`에 전달합니다. canonical 좌석 ID는 그대로 보존합니다.

연결·재연결·sync 요청마다 fresh snapshot을 보내며 **snapshot ACK 이후** 해당 연결의
mission feedback과 명령 실행을 허용합니다. 명령 ACK는 수신 확인입니다. 실제 실행 수락은
`mission.accepted`, 취소 완료는 checkpoint 이후 `mission.result: CANCELLED`로 확인합니다.
ROS RPC timeout은 수락 여부가 불명확하므로 거절을 합성하지 않고 재동기화합니다.

SQLite outbox는 event ID와 미션별 sequence, payload를 ACK까지 유지하고 재연결 시 재전송합니다.
최종 보고는 Runtime journal에도 남습니다. snapshot은 최근 100개 최종 보고를 포함합니다.
Runtime의 IDLE이라도 port가 준비되지 않았으면 가용 상태를 ERROR로 전달해 배차를 막습니다.
terminal 보고와 fresh 가용 상태를 모두 확인하는 Backend의 배차 규칙을 따릅니다.

실행 profile의 `sim/real/mock`를 그대로 전달합니다. Runtime 내부 `mock://` 관측 ID는
사진 자료가 아니므로 외부 `before_observation`, `after_observation`에는 `null`을 보냅니다.
작업 실패·복귀 실패·부분 성공·사람 확인·취소는 원래 outcome을 보존합니다.

네트워크 worker와 ROS executor를 분리하고 RPC·재연결 대기를 제한합니다.
Backend가 끊겨도 Runtime의 수락된 미션은 계속됩니다. Runtime 재시작은 자동 재개하지 않고
INTERRUPTED를 보고합니다. 동일 mission ID를 새 실행으로 처리하지 않습니다.

## 실행

```bash
make build-mission-runtime
source ros2_ws/install/setup.bash
ros2 run cleany_control_bridge control_bridge --ros-args \
  -p url:=ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws \
  -p journal_path:=/tmp/cleany-lab-bridge.db
```

Runtime이 별도로 필요합니다. [mission_runtime.launch.py](../cleany_mission_manager/launch/mission_runtime.launch.py)가
두 node를 함께 실행하므로 같은 robot에 bridge를 중복 실행하지 않습니다.

| ROS parameter | 기본값 |
|---|---|
| `url` | `ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws` |
| `journal_path` | `~/.local/state/cleany/control-bridge.db` |
| `rpc_timeout` | 3초 |
| `heartbeat_interval` | 1초; Backend sync 요청에서 갱신 |
| `reconnect_initial`, `reconnect_max` | 1초, 30초 |

`make test-mission-core`는 ROS 없이 영속 sequence, 재전송, duplicate 및 snapshot 계약을
검사합니다. `make test-mission-runtime`은 설치된 ROS package 경계까지 검사합니다.
Backend counterpart는 `cleany-control-plane`의
`docs/architecture/robot-gateway-integration.md`와
`packages/contracts/schemas/gateway-robot-message.schema.json` 및
`gateway-backend-message.schema.json`을 기준으로 확인합니다.
