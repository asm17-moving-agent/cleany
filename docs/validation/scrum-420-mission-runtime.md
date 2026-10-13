# SCRUM-420 Mission Runtime 구현 및 검증

2026-10-01 기준 작업 브랜치 `feat/mission-runtime`의 구현 기록이다. KB는 수정하지 않았다.
상위 FSM이 단일 미션 수락·이동·작업·복귀·보고를 소유하고 `py_trees` BT가
관측·계획·한 작업 실행·재관측·완료 확인을 반복한다. 실제 모듈은 typed async port로
교체할 수 있으며 현재 주행은 Nav2/Gazebo, 인식·계획·책상 청소는 Mock이다.

## 구현과 실행

- [Mission Manager](../../ros2_ws/src/cleany_mission_manager/README.md): FSM, BT, ROS 계약 및 오류 복구
- [Control Bridge](../../ros2_ws/src/cleany_control_bridge/README.md): Backend 계약, snapshot ACK 및 영속 재전송
- [Bringup](../../ros2_ws/src/cleany_bringup/README.md): 기존 Gazebo/AMCL/Nav2와 Runtime 합성
- [Workspace](../../ros2_ws/README.md): 공통 빌드·검사 명령

Backend Queue가 배차하고 Runtime은 하나만 수락한다. 중복 mission ID는 재실행하지 않는다.
최종 outcome은 영속 보존하며 주행/복귀 실패를 성공으로 바꾸지 않는다. 미완료 상태의
Runtime 재시작은 INTERRUPTED와 사람 확인 필요로 보고한다. 일반 취소는 Nav2 terminal 및
fresh 정지를 확인한 후 보고하고, 확인 실패 시 ERROR를 유지한다. 물리 안전 정지는 별도
안전 소유자의 책임이며 해당 모듈·실물은 이 시험에서 호출하지 않았다.

`post_mission=return_home|wait_for_next`는 실행 옵션이다. home 제품 정책은 미확정이다.
기본 home `[3,-1.2,0]`은 simulator 통로의 대기 후보이며 충전 dock이 아니다.
Nav2 goal의 frame은 `map`, Dashboard 지도 표시 relay는 `/ground_truth/odom` world 좌표이다.
표시용 ground truth를 wheel odometry·AMCL 정확도의 검증으로 해석하지 않는다.

## 자동 검사

2026-10-01 이어받은 호스트에서 다음을 다시 실행했다.

| 검사 | 결과 및 범위 |
|---|---|
| Humble `make test-mission-core`, `CLEANY_RUN_ROS_TESTS=1`, domain 82 | 50개 통과: 기존 11, 신규 core 26, bridge 10, ROS transport 3 |
| Humble `make test-mission-runtime` | interfaces, manager, bridge 3개 build 및 colcon test 통과 |
| Jazzy `make build-mission-sim` | Gazebo·Nav2·telemetry·bringup 포함 관련 10개 패키지 build 통과 |

ROS transport 검사는 fake Nav2 server의 실제 ROS action/service를 사용한다. 좌석·home goal,
중복 실행 방지, 실제 cancel terminal, 입력 freshness 거절과 GetResult 응답 유실 시
다음 미션 수락 금지를 확인했다. 이것을 Gazebo 경로 성공이나 실물 동작으로 해석하지 않는다.
전체 workspace의 무관한 perception·MoveIt·MuJoCo 검사는 실행하지 않았다.

Humble은 `/opt/ros/humble`, Jazzy는 `/opt/ros/jazzy`를 사용했다. host는 Fedora/aarch64이며
각 ROS 환경은 기존 Podman container에서 실행했다. `py_trees==2.2.3`을 사용했다.
이어받은 Jazzy에서는 `/tmp/cleany-420-deps`의 분리된 Python dependency를 사용했다.
빌드 경로는 `/tmp/cleany-420-{build,install,log}`와
`/tmp/cleany-420-jazzy-{build,install,log}`로 저장소 생성물과 분리했다.

## 이전 Gazebo 시험 기록과 증거 한계

같은 세션의 이전 실행에서 수락한 4건을 모두 아래에 남긴다. 호스트 재시작 후 `/tmp`의
원본 mission JSON, DB와 Nav2 로그가 소실됐다. 아래는 **이전 세션 기록의 요약**이며
현재 파일로 독립 재검증할 수 없다. seat-13 실패 결과의 Firefox/Niri
[UI 캡처](../../artifacts/mission_runtime/20261001/failed-return-ui.png)는 남아 있다.

| Mission ID | 좌석/종료 정책 | 이전 실행에서 관찰한 결과 |
|---|---|---|
| `7ac1f254-6714-4efe-9ccf-3d689de1012a` | 12 / return_home | target OK, Mock trash-1·2 완료, home `[0,0]` 경로 실패 → FAILED/NAVIGATION_FAIL |
| `1291f343-b20c-4bff-a56c-2a45eaee3065` | 13 / return_home | target OK, Mock trash-1·2 완료, home `[3,-1.2]` Nav2 aborted → FAILED/NAVIGATION_FAIL |
| `b61ebdb9-a0f9-4d23-b36c-f1107674fe9c` | 12 / wait_for_next | 주행 180초 TIMEOUT, Nav2 cancel 및 정지 확인 → FAILED/TIMEOUT, 작업 미실행 |
| `2ae78a48-f0bb-4056-96fc-4053ca6d2516` | 12 / wait_for_next | 새 Gazebo 실행에서 주행 180초 TIMEOUT, Nav2 cancel 및 정지 확인 → FAILED/TIMEOUT, 작업 미실행 |

4건 중 전체 SUCCESS는 0건이며 실패 건을 분모에서 빼지 않는다. 이 결과는 실물 청소
성공률이 아니다. 최초 두 건의 내부 Mock 관측 ID는 당시 외부 결과에도 포함됐으나,
현재 bridge는 실제 이미지가 없는 경우 외부 observation을 `null`로 보낸다.
미션 3·4의 timeout에 따른 중단은 사용자가 요청한 checkpoint CANCELLED 검증과 구분한다.
home 경로 계산 한 번의 성공도 실제 복귀 성공을 뜻하지 않는다.

## 이어받은 환경의 연동 시험

이전 DB를 재구성하거나 실패 결과를 변경하지 않았다. 새 격리 Backend DB
`/tmp/cleany-420-control.db`와 새 Runtime/bridge journal, 새 mission ID를 사용했다.
Backend는 port 8088, Mock 구조 시험은 ROS domain 84, Gazebo 시험은 domain 83이다.
UI는 Firefox에서 Backend가 제공하는 Dashboard를 열고 Niri window capture로 확인한다.

전체 Mock 모드의 `3aaea6da-1259-4ebb-a3fe-1a3ee8694fa0`은 seat-12, wait_for_next로
수락·NAVIGATING·WORKING·두 객체 처리·재관측·SUCCESS 보고 후 fresh IDLE을 확인했다.
실행 profile의 네 모듈은 모두 `mock`이며 실제 Nav2/Gazebo 성공의 증거가 아니다.

새 환경의 수락 건수는 총 3건이며 모두 최종 결과 및 fresh IDLE까지 확인했다.
아래 소요 시간은 요청부터 결과 수신까지이며 Queue와 통신 시간이 포함된다.

| Mission ID | 실행 출처 및 정책 | 최종 결과 | 소요 시간 |
|---|---|---|---|
| `3aaea6da-1259-4ebb-a3fe-1a3ee8694fa0` | 전체 Mock / wait_for_next | SUCCESS, trash-1·2 완료 | 약 3.6초 |
| `3017f92a-56b5-4168-92b6-7fe1b00a5ac7` | Nav2/Gazebo + Mock 작업 / wait_for_next | 이동 중 사용자 cancel → CANCELLED, 작업 미실행 | 약 11.5초 |
| `73c26abd-469b-4acc-8525-d57895c8785d` | Nav2/Gazebo + Mock 작업 / wait_for_next | 좌석 12 도착 → BT trash-1·2 → 재관측 → SUCCESS | 약 42.2초 |

취소 시험에서 Nav2 `Goal canceled`, controller 정지와 Runtime의 CANCELLING→IDLE을
확인했다. 이후 `/wheel/odom`의 linear 및 angular 속도가 0인 fresh sample도 저장했다.
완료 시험에서 Nav2 `Goal succeeded`, 실제 BT ObserveBefore/ExecuteOne/Reobserve/
VerifyCompletion 단계와 POST_MISSION→IDLE을 확인했다. 두 시험 모두 외부 관측 참조는
null이며 실제 책상 청소·home 복귀는 수행하지 않았다.
Firefox/Niri 결과 화면에서 취소·완료, 실행 출처 `시뮬레이션 · 작업 모의 실행`과
관측 자료 없음 표시를 확인했다. 새 시험의 성공을 이전 실패 4건의 대체 결과로 사용하지 않는다.

증거의 영속 위치는 `/home/ehdrms/.local/state/cleany/scrum-420/20261001-continue/`이다.
`tests.log`, `jazzy-build.log`, `mock-success.json`, `mock-success-fresh-idle.json`,
`gazebo-cancel.json`, `gazebo-wait.json`, `sim.log`, `stopped-odom.log`,
`missions.json`, `robots.json`, `pose.json`과 UI PNG를 보존한다.
`control.db`, `mock-runtime.db`, `mock-bridge.db`, `sim-runtime.db`, `sim-bridge.db`는
각 실행의 SQLite backup이며 `implementation-sha256.json`은 변경 파일의 해시 목록이다.
SQLite는 backup API로 복사해 WAL에 남은 변경까지 보존한다. 새 시험의 결과와 DB는
이전 4건의 소실된 기록을 대체하지 않는다.
검증 종료 시 활성 미션이 없는 fresh IDLE을 확인한 뒤 시험용 Gazebo/Runtime을 종료했다.
Backend port 8088은 결과 조회를 위해 남겨 두었으며 현재 Robot은 OFFLINE이다.
`robots.json`과 각 mission proof의 IDLE은 종료 전 완료 checkpoint의 기록이다.

구현 provenance는 Cleany base `0f726903b695848ccc8cc30bca88e85063c97af2`에 적용한 이 브랜치의
SCRUM-420 변경이다. Runtime/Gateway 구현은 `418d2df0a04b8b3fc4232333d8458b2e0f9055fc`에 있으며,
bringup 및 재현 도구는 같은 브랜치의 후속 커밋에 포함한다.
이전 Backend 시험은 base `a9f976ba48b9378d1c74765889709b16a696086a`의
Gateway 작업 변경을 사용했고, 이어받은 시험은 Backend commit
`0919d38f8b8aa5e68282945522b5696d074b6d85`를 사용한다. Backend 코드와 KB는 수정하지 않았다.

## 재현과 남은 작업

[gateway_smoke.py](../../tools/mission_runtime/gateway_smoke.py)는 실제 HTTP mission 요청을
전송하므로 격리 simulation Backend에만 실행한다. 실행 중인 Runtime의 지원 좌석과
모드를 먼저 확인하고 새 output 파일을 지정한다. 기존 mission 관찰은 `--mission-id`를
사용하면 새 요청을 만들지 않는다.

```bash
python3 tools/mission_runtime/gateway_smoke.py \
  --api-url http://127.0.0.1:8088 --seat seat-12 \
  --timeout 420 --output /tmp/new-mission-proof.json
# 이동 중 checkpoint 취소: NAVIGATING 상태가 된 뒤 10초 이후 요청
python3 tools/mission_runtime/gateway_smoke.py \
  --api-url http://127.0.0.1:8088 --seat seat-12 \
  --cancel-after 10 --cancel-phase NAVIGATING --expected-outcome CANCELLED \
  --timeout 60 --output /tmp/new-cancel-proof.json
```

스크립트는 기대 outcome과 Mock 작업 profile, terminal 이후 fresh Robot 가용 상태를
확인하고 실패 시에도 관찰 기록을 저장한다. timeout은 자동 성공·재배차를 만들지 않는다.
ERROR 기대 시험은 `--expected-robot-state ERROR`를 지정한다.

실환경 Perception/Planner/Executor adapter, 검증된 좌석 접근·home pose 및 home 주행 안정화는
후속 작업이다. 현재 코드로 물리 청소나 실제 charging dock 복귀 완료를 주장하지 않는다.
활성 미션 중 Backend 재시작에 대한 실제 Gazebo 재연결 시험도 아직 확인하지 않았다.

## 2026-10-02 단일 물체 Manipulation 계약 연결

기존 `2fc4b43` 이후 변경으로, 원격 `feat/manipulation-action-server`의
`46efa012cddb4ef0ff3f4dadb785550591bb6073`에서 action/message/service 세 interface를
동일하게 가져왔다. 서버 내부 BT와 나머지 원격 브랜치는 병합하지 않았다.
우리 청소 BT가 관찰 → 제안 검증 → 물체 하나의 요청 → 새 관찰을 소유하고,
팀원 Executor는 해당 물체의 접근·집기·운반·투입 및 결과를 소유한다.

- 요청: snapshot에 연결된 numeric object ID, destination, mission/task/execution ID.
  destination allowlist와 관측의 numeric ID를 검증하고 UUID 및 전체 요청을 전송 전에 저장한다.
- 성공: ROS terminal status와 payload를 구분하고, placement 확인·팔 복귀·정지 증거를 확인한다.
  `report_json.actions`에 요청 식별자와 전체 물리 상태 결과를 보존한다.
- 취소: 실제 action 결과를 기다린다. 정지 확인과 주행 가능 조건을 분리하여
  HELD/UNKNOWN, 미복귀 팔, 사람 확인 필요 상태는 home·다음 미션·오류 reset을 차단한다.
- 불확실한 전송·재시작: 동일 UUID 취소 및 조회만 수행한다. `found=false`를 미실행 증거로
  취급하지 않고 새 execution ID로 자동 재전송하지 않는다.

### 확인한 결과

Humble container에서 별도 `/tmp/cleany-420-manipulation` build/install을 사용했다.
팀원 서버는 위 SHA의 read-only snapshot을 별도 overlay로 빌드했고 코드를 수정하지 않았다.
DDS domain 185에서 실제 ROS action/service 통신으로 아래를 확인했다.

| 검사 | 결과 |
|---|---|
| 두 물체 요청 → 각각 placement 확인 → mock home | SUCCESS, 두 서로 다른 execution ID와 결과 보존 |
| TRANSPORTING 중 취소 | CANCELLED, HELD 및 stop_confirmed 보존, ERROR·사람 확인 필요, home·reset·다음 미션 차단 |
| 서버 ROS SUCCEEDED + BLOCKED payload | BLOCKED 보존, 완료 물체 없음, 팔 복귀 미확인으로 ERROR 유지 |
| 전체 Mission/Bridge pytest 및 opt-in ROS 검사 | **65 passed** (기존 Nav2 ROS 검사 3개와 신규 조작 ROS 검사 3개 포함) |
| 관련 3 ROS package build 및 colcon test | build 성공; **71 tests, 0 errors, 0 failures, 6 skipped** |

첫 BLOCKED 시험에서는 `BACKEND_NOT_READY`로 `stop_confirmed=false`까지 반환하도록 설정했다.
Runtime은 이 payload를 actions에 BLOCKED로 보존했고, 정지 확인 15초 초과 후 전체 미션을
FAILED/TIMEOUT 및 ERROR로 종료했다. 이는 기대한 안전 동작이며, payload BLOCKED만으로
전체 미션이 BLOCKED 종료될 것이라는 시험의 기대값이 잘못되었다. 최종 BLOCKED 시험은
정지를 확인하지만 팔 복귀는 확인하지 않는 `VERIFICATION_UNAVAILABLE`로 구성했다.

영속 증거는 다음 디렉터리에 저장했다.

`/home/ehdrms/.local/state/cleany/scrum-420/20261002-manipulation/`

`tests.log`, `build-colcon.log`, success/cancel/blocked 각각의 `*-report.json`,
`*-missions.db`, `*-executions.db`를 보존했다. DB는 SQLite backup API로 복사하고
`integrity_check=ok`를 확인했다. `implementation-sha256.json`은 변경 파일의 내용 hash를 기록한다.

### 적용 범위와 남은 경계

기본 로컬 mock 경로를 유지하고, `action_mock` 설정에서 외부 팀원 mock server에 연결한다.
실제 Perception·Planner·팔 backend를 연결한 검증은 아니다. 신규 시험의 주행은 mock이며
기존 Gazebo 주행 증거와 구분한다. Gazebo home 또는 실제 청소가 성공했다는 증거를 추가하지 않는다.

최초 팔 안전 상태는 새 mock DB에서만 명시 설정으로 가정한다. 저장된 위험/진행 상태를
이 설정으로 덮어쓰지 않는다. 실제 팔 도입에는 최신 readiness 및 수동 복구 완료 계약이
필요하다. 현재 peer service의 저장 오류/기록 없음 구분도 추가 확장 대상이며, 지금 adapter는
두 경우 모두 보수적으로 주행을 차단한다. KB는 수정하지 않았다.

설정 및 팀원 서버 실행 절차는
[Mission Manager README](../../ros2_ws/src/cleany_mission_manager/README.md#단일-물체-manipulation-연결)를 따른다.
