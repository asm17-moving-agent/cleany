# Cleany 개발용 관제보드

Mission FSM·청소 BT·주행·조작 진행을 관측하는 **읽기 전용 개발 도구**다.
미션 요청·취소는 운영 `cleany-control-dashboard` 또는 Mission ROS API를 사용한다.
이 보드는 미션 수락·취소·reset·속도 발행·임의 service/action 호출을 제공하지 않는다.

| 찾는 내용 | 안내 |
|---|---|
| 웹 열기·기존 VM 서비스 확인 | [설치와 실행](#설치와-실행) |
| BT SUCCESS·주행 확인 불가의 의미 | [실행 흐름과 상태 읽기](#실행-흐름과-상태-읽기) |
| 물체별 단계·시간·결과 | [ExecuteOne 상세](#executeone에서-조작-진행-펼치기), [전용 조작 화면](#매니퓰레이션-전용-화면) |
| 지도·Nav2·센서 진단 | [자율주행 관측](#자율주행-관측) |
| 과거 미션·원본 저장 | [기록과 재생](#기록과-재생) |
| 빌드·브라우저 검사 | [프런트엔드 구성](#프런트엔드-구성과-foxglove), [검증](#검증) |

## 설치와 실행

ROS 2 Humble, 같은 DDS domain과 공통 메시지 install이 필요하다.
[의존 설치](../../../docs/DEVELOPMENT_SETUP.md)와 [Bash/zsh 환경 안내](../../README.md#외부-데이터와-빌드-경로)를
먼저 확인하고 저장소 루트에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src/cleany_dev_monitor --ignore-src -r -y
make build-dev-monitor
ROS_DOMAIN_ID=186 make run-dev-monitor MONITOR_ROS_ARGS='-p use_sim_time:=true'
```

브라우저에서 **<http://127.0.0.1:8768>**을 연다. `use_sim_time`은 Gazebo/ROS clock을
사용할 때 지정한다. 이 명령은 simulator나 Runtime을 시작하지 않는다.

| 상황 | 설정 |
|---|---|
| 다른 포트 | `MONITOR_ROS_ARGS='-p port:=8769'` |
| `/robot1` Runtime | `MONITOR_ROS_ARGS='-p runtime_namespace:=/robot1'` |
| 원격 호스트 | localhost bind를 유지하고 `ssh -L 8768:127.0.0.1:8768 robot-host`로 전달 |
| 토픽·frame·저장 위치 변경 | [monitor.yaml](config/monitor.yaml)의 실제 remapping과 일치하도록 설정 |

### Mission Mock 통합 VM에서 확인

[기존 VM 활성화](../../README.md#기존-mission-mock-vm)를 사용한다.
아래 사용자 서비스는 별도로 실행한 경우에만 존재하며 상시 실행을 보장하지 않는다.
기존 `start-monitor.bash`는 **domain 185 / port 8768**을 사용하므로 Runtime도 domain을 맞춘다.

```bash
systemctl --user status cleany-mission-mock-monitor.service
journalctl --user -u cleany-mission-mock-monitor.service -n 30 --no-pager
```

서비스가 없거나 종료된 경우, 해당 스크립트가 있는 기존 VM에서 시작할 수 있다.

```bash
systemd-run --user --unit=cleany-mission-mock-monitor --collect \
  /usr/bin/bash /home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/start-monitor.bash
```

종료 명령은 `systemctl --user stop cleany-mission-mock-monitor.service`다.
Gazebo 미션 실행·요청은 [Bringup](../cleany_bringup/README.md#빠른-시작)을 따르며,
자동 시험을 domain 185에서 관측하려면 공통 검사 명령의 `BASE_TEST_ROS_DOMAIN_ID`도 185로 맞춘다.
Monitor 시작 이후 수신한 데이터만 기록하므로 먼저 실행해야 전체 미션을 재생할 수 있다.

## 제공 화면

| 화면 | 관측 내용 |
|---|---|
| 실행 흐름 | Mission FSM, 현재 상태와 전이 이벤트, 청소 py_trees 구조/상태/이번 tick 방문 노드, 모듈 준비/정지 보고, 주행 차단 이유, 작업 대상/목적지, 조작 결과 |
| ROS 탐색기 | topic/node/service/action 목록, 선택 토픽의 publisher→topic→subscriber 그래프, endpoint QoS, 수신 Hz, 메시지 미리보기 |
| 자율주행 | 주행 개요, 장애물·안전, 계획·제어, 센서·위치, 주행 비교의 5개 하위 탭. 지도/실제 궤적, 단계별 명령/실측 속도, guard/pan/BT 진단과 공통 타임라인 |
| 매니퓰레이션 | 조작 개요, 단계 진행, 실행 타임라인, 취소·주행 조건, 실행 비교. 물체·목적지·선택 팔, Action의 단계·세부 동작·결과, 관측 소요 시간과 Runtime 주행 차단 근거 |
| 카메라 | Image / CompressedImage 토픽 선택, 실시간 JPEG 미리보기, 일시정지, 수신 지연 표시 |
| 로그 | `/rosout`, FSM/BT/action 이벤트와 결과를 수신 시각순으로 조회. 노드/미션/문자열 및 로그 수준 필터 |
| 기록 & 재생 | 항목별 ON/OFF·프리셋·원본 주기·용량 제한, SQLite 세션 선택 및 재생·내보내기 |

## 실행 흐름과 상태 읽기

| 표시 | 읽는 방법 |
|---|---|
| 상단 `LIVE` | 웹과 Monitor가 연결됨. Runtime·Nav2 준비 완료를 뜻하지 않음 |
| `Runtime 대기` | Monitor는 연결됐지만 같은 domain의 Runtime 관측이 없음 |
| 주행 `확인 불가` | Runtime 수신이 3초 이상 끊겼거나 연결을 확인할 수 없음 |
| 마지막 청소 결과의 `SUCCESS` | 당시 BT 노드의 통과 결과. 현재 주행 허가와 별개 |
| Mock / sim | Runtime의 `execution_profile` 출처. 실물 실행 증거가 아님 |

### 청소 BT와 노드 의미

책상 도착 후 Runtime이 생성한 실제 트리를 표시한다. Sequence는 번호 순서대로
실행하며 `UntilVerified`가 `ActionCheckpoint` 주기를 반복한다. 반복 때 자식 상태가
INVALID로 초기화될 수 있다. 최근 tick 방문 점과 노드 선택 상세에서 흐름을 확인한다.

| 노드 | 호출·판정 역할 |
|---|---|
| `ObserveBefore`, `Reobserve` | perception port로 작업 전·후 관측 |
| `PlanNext` | planner port로 다음 행동 제안 요청 |
| `ValidateProposal` | Mission Manager가 snapshot·물체·행동·목적지 검사 |
| `ExecuteOne` | 승인된 한 물체를 조작 port로 실행. done·skip·human_review는 Action 없이 통과 가능 |
| `VerifyCompletion` | fresh 재관측으로 완료 여부 검사. 노드 SUCCESS여도 완료 미확정이면 다음 주기 진행 |

노드 SUCCESS는 미션 전체 성공이나 실제 수거 성공을 보장하지 않는다.
최종 조작 결과는 Action payload, 미션 결과는 `mission/result`로 확인한다.

snapshot/event는 Runtime boot ID·event sequence로 맞춘다. 이미 snapshot에 포함된
이벤트는 다시 적용하지 않고 이후 leaf tick으로 마지막 청소 단계를 갱신한다.
방문 표시는 root tick마다 초기화한다. 이벤트 반영으로 snapshot 수신 시각이나
주행 준비 보고를 새로 만들지 않는다.

### ExecuteOne에서 조작 진행 펼치기

`실행 흐름 → 청소 BT → ExecuteOne` 또는 ‘조작 단계 보기’를 누른다.
상세 패널에서 한국어 동작명, 물체·목적지·선택 팔, 완료 수, 경과 시간과
**9단계·17개 세부 동작**을 확인한다. 실패 위치·오류 코드는 별도로 강조한다.
UUID·원본 기록은 펼쳐 볼 수 있다. 패널을 열 때만 초점과 스크롤이 이동하고
수신 갱신은 독자의 위치를 바꾸지 않는다. 접으면 ExecuteOne으로 초점을 돌린다.

같은 미션의 실행만 선택한다. done·skip·human_review에서는 이번 주기의 Action
미실행과 이전 기록을 구분한다. 이 패널은 실제 BT의 자식 노드를 추가하지 않는다.

### 매니퓰레이션 전용 화면

| 탭 | 확인 내용 |
|---|---|
| 조작 개요 | 현재/마지막 동작, 확인된 진행률, 물체·목적지·결과와 현재 주행 조건 |
| 단계 진행 | 단계별 세부 동작, 완료·실패 위치 |
| 실행 타임라인 | 관측 구간별 소요 시간, ‘시점 보기’로 기록 재생 |
| 취소·주행 조건 | 취소 방식, 팔·물체 상태, Runtime의 준비·주행 차단 근거 |
| 실행 비교 | 최근 실행의 결과·시간·오류·복귀/정지 증거 비교 |

현재 미션·execution ID를 자동으로 따라간다. 이전 실행을 직접 선택하거나 자동
선택으로 돌아올 수 있다. ExecuteOne 상세와 동일한 단계 정의·관측 규칙을 사용한다.

### 조작 관측 규칙

| 항목 | 표시 근거 |
|---|---|
| 진행·결과 입력 | `/mock/manipulation/execution_events`의 `ManipulationExecutionRecord` |
| 완료 수·실패 위치·최종 결과 | `completed_substages`, `failed_stage/failed_substage`, `has_result/status` |
| 새 실행의 첫 기록 대기 | 이전 물체의 진행·성공이나 다른 미션의 결과로 대체하지 않음 |
| 현재 실행 강조·경과 시간 추정 | 최신 Runtime·graph, 같은 미션·Feedback execution ID와 현재 제안 ID 일치 |
| 종료·수신 중단·과거 실행 | 서버 기록 시각으로 시간 고정; 진행 애니메이션 중단 |
| 재생 | 선택한 시각까지 저장된 요약만 사용; 미래·실시간 결과를 섞지 않음 |
| 현재 주행 조건 | Runtime의 `safe_to_drive`, navigation 준비·베이스 정지·미션 수락 준비를 구분 |

현재 제안에 execution ID가 없는 과거 기록도 읽는다. 시간이나 BT SUCCESS만으로
조작 성공을 만들지 않으며, 과거 실행의 팔 복귀·정지가 현재 주행 판단을 대체하지 않는다.

기본 최근 실행 20개·실행별 구간 64개를 별도 요약으로 보관한다.
`manipulation_execution_limit`(1..50), `manipulation_interval_limit`(1..128)로 조절한다.
revision 중복·역순은 무시하고 늦은 접속·누락·보관 초과는 일부 구간 미관측으로 표시한다.
시계 역행 시 소요 시간은 표시하지 않는다. Monitor 재시작 시 retained 이벤트에서
관측한 범위만 복원하며, 과거 기록에 요약이 없으면 실행 기록 대기로 표시한다.

`manipulation_events_topic`이 절대 경로면 그대로, 상대 경로면 `runtime_namespace` 아래에서
구독한다. Mock 단계·안착·팔 복귀는 서버의 모의 보고이며 팔 애니메이션·실물 센서의
증거가 아니다. 서버 내부 BT를 추측해서 그리지 않는다.

## 자율주행 관측

| 하위 탭 | 제공 정보 |
|---|---|
| 주행 개요 | 전체 지도, 계획 경로와 실제 위치 궤적, 목표와 진행 상태, 실측 이동·저속·정지 및 배속, 현재 구간 경과 시간과 마지막 관측 목표 거리. 원인/명령 상세로 이동 |
| 장애물·안전 | 정지·감속 판단 요약, 로봇 주변 local costmap과 LiDAR/depth 표면 표본, 정지/감속 polygon과 위험점, 메모리 grid, reactive/voxel/collision/coverage 진단 및 감시·정지 이력 |
| 계획·제어 | 단계별 속도 명령과 실측 속도 그래프, 주변 계획/제어 경로와 DWB 선택·후보 궤적, Nav2 BT 노드 상태와 전이 이력, lifecycle 진단, 후보의 critic 점수 |
| 센서·위치 | 수신 주기/경과 시간/ROS stamp 나이, TF 지연, AMCL 위치와 covariance, odometry와 Gazebo ground truth 비교, 차체·이동 방향과 pan 역할, 카메라 화면 이동 |
| 주행 비교 | Runtime 작업 ID별 도착/복귀 관측 시간, 실측 RTF, 추정 위치 궤적 길이, 정지/저속/분류 불가 시간, 마지막 관측 목표 오차, 관측 설정과 코드 버전, 세션 간 전후 비교 |

- 개요 지도는 전체 시설을, 안전/제어 지도는 현재 로봇 또는 유효한 local costmap 주변을 표시한다. 주변 범위를 결정할 수 없으면 전체 지도와 안내를 표시한다. 각 화면의 레이어 선택은 독립적이며 숨겨진 다른 화면의 레이어는 그리지 않는다. 지도 수신 출처는 펼쳐서 확인한다.
- 작은 주행 진단과 odometry/AMCL/BT는 기본 수집한다. 큰 scan/depth/risk point,
  후보 궤적, 추가 grid/local path는 해당 하위 탭에서만 관측 선택을 요청한다.
  화면 이탈/재생 전환 시 자동 선택을 해제한다. 명시 선택 및 다른 브라우저가 같은
  토픽을 보고 있으면 공유 구독은 유지된다. 최대 선택 토픽 한도는 그대로 적용된다.
- 점군은 최대 `navigation_point_limit`개(기본 512, 상한 2048)를 직접 표본 추출하며,
  scan/점군/후보 변환은 최대 2 Hz다. 원본 수량/표본 여부를 남긴다. 화면의 표본은
  충돌 검증을 대체하지 않으며 미관측 공간을 빈 공간으로 인증하지 않는다.
- 주행 요약은 5 Hz, 속도 그래프는 최근 180개, 실제 궤적은 최근 600개,
  작업 요약은 최근 64개로 제한한다. 정지/guard/action 이벤트는 주행 요약에도 최근 128개를 별도로 보존한다.
  BT 상태는 모든 수신 이벤트로 갱신하며, 반복 조건 노드의 동일 상태 이벤트는
  `bt_event_repeat_interval_s`(기본 1초) 단위로 요약하고 반복 횟수를 남긴다.
  실패/ComputePath/FollowPath/Wait 전이는 즉시 이벤트로 기록한다.
- 속도 그래프 축은 관측값의 95백분위에 여유를 두어 정한다. 상한 밖의 표본은
  선을 끊고 상단 마커/실제 최대값으로 표시한다. 기록과 집계의 원본 값은 유지한다.
- 실측 속도는 fresh한 Gazebo ground truth를 우선하고, 없으면 wheel odometry를 쓴다.
  Twist의 `child_frame_id`가 설정된 `base_frame`과 다르면 실측으로 사용하지 않는다.
  명령만 수신된 상황을 이동/정지 실측으로 바꾸지 않는다. 도중에 출처가 바뀌면 `MIXED`로 남긴다.
- `stationary_speed_mps` 기본 0.005, `creep_speed_mps` 기본 0.03 및
  회전 0.02 rad/s는 관측 분류 기준이며 주행/안전 설정을 바꾸지 않는다.
  동일한 상태를 연속 관측한 구간만 정지/저속 시간에 넣는다. 0.5초보다 긴
  관측 공백 및 상태 전환 사이 시간은 분류 불가로 남긴다. 도착 후 작업/대기는
  주행 구간 집계에서 제외한다. 구간 끝은 실제 Runtime action_result를 우선한다.
  결과 이벤트를 못 받으면 `OBSERVATION_ENDED` 등으로 표시하며 성공을 추정하지 않는다.
- RTF는 `/clock` 증가량/steady wall 증가량의 실측이다. 요청 배속은
  `requested_rtf`를 명시한 경우만 표시한다. `/clock`이 없으면 실측 RTF도 미제공이다.
  clock 역행은 현재 구간을 불완전한 기록으로 종료하고 그래프를 초기화한다.
- stamped 입력은 ROS 시각과 수신 시각 모두의 freshness를 검사한다.
  `navigation_freshness_s`(기본 0.75초)는 보드 관측 기준이다. 실제 guard의
  safety timeout과 다르며 보드의 정상 표시가 주행 허가를 의미하지 않는다.
- 단계별 unstamped Twist는 같은 명령 ID가 없어 수신 시각 차이가 0.2초 이하일 때만
  감소 관측을 표시한다. 정확한 원인은 해당 모듈의 진단과 함께 확인한다.
  guard 비활성, 토픽 미수신, 오래된 진단, PASS를 서로 같은 상태로 취급하지 않는다.
  reactive guard가 현재 제공하지 않는 위험점/최소 거리/제동 영역은 미제공으로 표시한다.
- 주행 비교는 대상/단계/실측 출처/관측 분류 기준/map frame이 같을 때만 차이를 계산한다.
  `run_label`, `source_revision`, `requested_rtf`는 실행자가 명시한 값이다.
  보드 설정은 기록하지만 전체 Nav2/물리/guard 파라미터를 자동 수집하지 않는다.
  서로 다른 환경의 두 실행을 자동으로 개선 판정하지 않는다.
- 타임라인 선택은 해당 수신 시각의 SQLite 기록으로 이동한다. 기록이 중단되거나
  아직 저장되지 않은 시각은 오류를 표시한다. 큰 데이터는 당시 실제로 구독한 경우에만 재생된다.
  영상은 기존 카메라 탭에서 확인하며 카메라 원본은 녹화 설정에서 선택한 경우에만 별도 원본 테이블에 포함된다.

`config/monitor.yaml`의 각 `*_topic`을 실제 remapping에 맞춘다. DWB 기본 평가
토픽은 `/evaluation`이다. 실제 발행이 없거나 메시지 타입을 import하지 못하면 ROS
탐색기의 발행/오류 상태를 확인한다. 비교 요약은 읽기 전용 `GET /api/navigation/runs`로
최근 50개 세션의 마지막 작업 요약을 조회하며, 재생 화면은 해당 기록 시각의 요약만 사용한다.

### Nav2 BT 시각화 및 재생

`자율주행 → 계획·제어`에서 `/navigation/bt_trace`로 받은 최종 실행 XML 구조를 본다.
노드를 선택하면 역할·instance ID·입출력 포트·최근 전이를 확인한다. 세 ComputePathToPose
인스턴스를 구분하며 UNKNOWN은 미관측이다. 구조가 없으면 BehaviorTreeLog 노드 목록을 쓴다.
2초 이상 snapshot이 없으면 연결 끊김과 마지막 관측 상태를 표시한다.

구조·상태·XML SHA256·목표 문맥을 SQLite에 저장한다. 재생은 기록 당시 트리를 사용하고
현재 XML을 읽지 않는다. 목표 연결은 수신 시각 기준이며 누락 탐지는 연속 상태 불일치에
한정된다. Monitor는 BT를 실행하거나 주행 명령을 보내지 않는다.

## 실시간 카메라

카메라 메뉴에서 같은 DDS domain의 `sensor_msgs/msg/Image` 또는
`sensor_msgs/msg/CompressedImage` 토픽을 선택한다. Raw `rgb8`, `bgr8`, `rgba8`,
`bgra8`, `mono8`와 JPEG/PNG 압축 영상을 지원한다. depth/Bayer 영상은 미지원 사유를 표시한다.

- `GET /api/camera?topic={topic}`이 해당 영상 구독을 요청하며, 최근 JPEG 한 장만 반환한다.
  202는 대기/지연, 422는 디코딩 오류, 429는 동시 구독 한도다. Same-origin, no-store를 적용한다.
- 기본 최대 2개 토픽을 공유하며 `camera_max_streams`(1..4), `camera_fps`(1..10, 기본 5)로 조절한다.
  브라우저는 최대 5회/초 조회하며 원본 16 MiB / 2,097,152 pixel을 넘으면 거절한다.
  표시 해상도는 최대 1280×720, JPEG quality 75다. `Image.step`의 행 패딩을 반영한다.
- 마지막 영상 관측이 2초를 넘으면 이전 화면을 지우고 수신 지연으로 표시한다.
  메뉴 이동·일시정지·탭 숨김·기록 재생 시 영상 요청을 중단한다.
  마지막 요청 후 3초가 지나면 lease와 이미지 캐시를 제거하고, 다음 ROS discovery에서
  다른 관측자가 필요로 하지 않는 구독을 해제한다.
- 미리보기 JPEG는 SQLite/WS telemetry에 넣지 않는다. 녹화 설정에서 카메라 원본을 선택하면 전체 ROS 메시지를 SQLite `raw_messages`에 저장한다. 기록 재생 중에는 실시간 카메라를 표시하지 않는다.
  ROS 탐색기에서 따로 선택한 영상 토픽은 기존처럼 제한된 메시지 미리보기만 기록한다.
- 카메라 드라이버나 Gazebo는 자동 실행하지 않는다. 카메라 토픽이 없으면 해당 상태를 표시한다.

## 기록과 재생

기본 위치는 `~/.local/state/cleany/dev-monitor/`이며 Monitor 시작마다 새 UUID 세션을 만든다.
세션당 100 MiB, 전체 1 GiB를 넘으면 기록을 중단하고 이유를 표시한다. 기존 파일은
자동 삭제하지 않는다. 실시간 관측은 계속한다. 기록은 monitor 프로세스가 소유하며
미션/조작 journal을 열거나 수정하지 않는다.

지도/costmap은 내용이나 변환이 바뀔 때 저장한다. 그 외 데이터는 전송과 같은 갱신
상한을 적용한다. **전체 ROS 데이터의 무손실 기록은 아니다.** 기록 목록은 FSM 포함 여부를 표시하며, 재생 버튼은 첫 Runtime 기록 시점부터 재생을 시작한다.
Runtime이 없는 세션은 전체 기록 시작점부터 재생하고 실행 화면에 기록 없음을 표시한다.
재생은 해당 시각까지
저장된 최신 snapshot/레이어/메시지와 최근 이벤트를 합친다. snapshot 사이의 FSM 전이와
짧게 끝나는 BT 노드는 이벤트로 반영한다. 기록의 gap은 UI에 표시한다.

현재 기록은 **SQLite schema 3**이며 이전 schema 1/2도 읽는다.

| 저장 항목 | 내용 |
|---|---|
| `metadata.schema_version` | 현재 `3` |
| `records(seq, received, kind, key, payload)` | 관측 snapshot·레이어·이벤트. payload는 zlib 압축 JSON BLOB |
| `raw_messages` | 선택한 ROS 원본 CDR과 토픽·타입·수신/ROS 시각 |

`cleany_dev_monitor.core.decode_record()`는 현재 payload와 schema 1의 평문 JSON을 읽는다.
내보내기는 SQLite backup API를 사용해 기록 중인 세션도 일관된 사본으로 제공한다.

HTTP GET: `/api/snapshot`, `/api/recordings`,
`/api/recordings/{uuid}/replay?at={unix_seconds}`, `/api/recordings/{uuid}/export`.
`/ws` 입력은 `{"op":"select","topics":["/odom"]}`만 지원한다.
재생 기능은 ROS service/action을 호출하지 않는다.

### 녹화 항목 선택과 원본 저장

`기록 & 재생 → 녹화 설정`에서 기본 주행·장애물 분석·사용자 지정 프리셋과 항목별
ON/OFF를 선택한다. 화면 관측·카메라 preview lease와 녹화 구독은 독립적이다.

| 항목 | 설정·상한 |
|---|---|
| 기본 기록 | 위치·속도·미션, 경로·costmap·Nav2 BT·안전·로그. 청소 BT는 Runtime snapshot에 포함 |
| 기본 OFF | LiDAR, depth point cloud, Nav2 voxel layer, voxel world, 카메라 원본 |
| 주기·세션 용량 | 항목별 0.1~10 Hz, 세션 10~1024 MiB |
| TF 원본 | 원본 활성 시 `/tf`, `/tf_static`도 최대 30 Hz 기록 |
| 원본 한 건 | 최대 8 MiB, 전체 ROS 직렬화 CDR. 공간 점·픽셀을 줄이지 않음 |
| 원본 대기 큐 | 최대 32 MiB / 128건. 초과는 저장 오류·유실 건수·`raw_gap` metadata |
| 기본 voxel 토픽 | `/obstacle_memory/voxels`, `/local_costmap/voxel_grid`. 미발행은 미수신으로 표시 |
| 3D·영상 원본 재생 | 시각 직전 원본 다운로드 제공. 직접 재생 뷰어는 미제공 |

원본 압축·쓰기는 HTTP worker에서 수행한다. 녹화 OFF 후에도 다른 관측 목적의 구독은
유지될 수 있다. 용량 제한은 파일을 삭제하지 않고 기록을 중단하며 ‘새 녹화 시작’으로
새 세션을 만든다. 전체 저장 한도는 `total_limit_bytes`를 따른다.

설정은 `state_directory/recording-settings.json`, 변경 시점은 `recording_policy`로 저장한다.
일시정지 중에는 설정·녹화 상태만 기록한다. 재생은 당시 정책을 적용해 미녹화·미수신·
입력 중단·저장 오류를 구분한다. 원본은 SQLite 내보내기에 포함된다.

| 로컬 API | 역할 |
|---|---|
| `GET/PUT /api/recording/settings` | 녹화 설정 읽기·변경 |
| `POST /api/recording/session` | 새 세션 |
| `GET /api/recordings/{session}/raw?category=point_cloud&at=<wall-seconds>` | 시각 직전 원본 한 건 다운로드 |

설정 변경은 같은 출처에서만 허용하며 ROS 명령·로봇 설정을 변경하지 않는다.

## 구조와 데이터 계약

```text
MissionRuntime ─ debug_snapshot (2 Hz), runtime_events ─┐
ROS graph / selected topics / rosout / TF ──────────────┤
                                                      ▼
                 cleany_dev_monitor (rclpy, 관측만)
                         bounded Hub
                              │ 별도 worker thread
                        aiohttp + SQLite
                              │ HTTP / WebSocket
                        React + Vite + TypeScript + shadcn/ui
                         React Flow / Canvas
```

- ROS discovery 1 Hz, 선택 메시지/위치/웹 갱신 최대 10 Hz. 수신 주기는 발행 주기를
  보장하지 않으며 미구독 토픽은 `unmeasured`로 표시한다.
- 모든 토픽의 **메타데이터**를 찾지만 모든 메시지를 구독하지 않는다. 기본 관측 토픽,
  최대 32개 action status/feedback 토픽, 사용자가 선택한 최대 16개 토픽만 구독한다.
  최대 8개 브라우저가 구독을 공유한다. 마지막 선택자가 해제/종료하면 구독을 정리한다.
- QoS는 발견된 publisher endpoint에 맞춰 reliable/best-effort와
  transient-local/volatile을 선택한다. 혼합 publisher에는 호환 가능한 가장 약한
  설정을 사용한다. 타입 import 실패, QoS 오류, publisher 없음, 수신 없음, 오래된 수신을 구별한다.
- 메시지를 JSON으로 전부 펼친 뒤 자르지 않는다. 미리보기는 총 2,048개 항목,
  배열당 256개 항목, 깊이 10, 문자열당 4,096자 한도로 변환한다. `truncated`를 표시한다.
  PointCloud 원본 보기와 rosbag 전체 데이터 저장을 대체하지 않는다. 카메라 영상은 별도 경로로 제공한다.
- 지도는 기본 최대 524,288셀, 경로는 최대 4,096점이다. 초과 지도는 거절 사유를
  표시한다. 경로는 잘린 여부를 데이터에 남긴다. `max_grid_cells` 상한은 1,048,576이다.
- 2D 레이어는 메시지 시각의 TF를 이용해 `map_frame`으로 변환한다. 회전된 지도
  origin도 반영한다. TF가 없으면 레이어를 표시하지 않는다. 로봇 위치는 최신 TF를
  사용하고 ±2초 밖의 stamp는 오래된 값으로 처리한다. 정적 지도/마지막 경로는 유지된다.
- 최근 telemetry는 종류/토픽별 최신값으로 합치고, 이벤트/로그/결과는 별도 2,048개 ring에
  보존한다. 오래된 이벤트가 밀려나면 gap을 표시한다. 브라우저 로그는 최근 500개다.
  느린 브라우저의 전송은 2초 후 종료하고 재접속 시 초기 snapshot을 다시 제공한다.
- Runtime boot ID/sequence와 Monitor boot ID를 분리한다. 재접속, Runtime 재시작,
  이벤트 sequence gap을 표시한다. 데이터가 끊겨도 IDLE/성공/주행 안전으로 바꾸지 않는다.
- ROS stamp와 수신 wall/monotonic 시각을 함께 저장한다. 재생 축은 수신 wall 시각이다.
  `/clock` 역행은 이벤트로 남기며, 같은 clock 값 유지/수신 지연을 화면에 표시한다.
  호스트 wall clock 자체가 역행하는 환경의 정밀 재생은 지원하지 않는다.

## 프런트엔드 구성과 Foxglove

| 항목 | 구현·표시 |
|---|---|
| 구성 | React 19, Vite 6, TypeScript strict, Tailwind CSS 4, shadcn/ui (Radix) |
| 공통 UI | `web/components.json`, `src/components/ui/`; 버튼·배지·입력·탭·툴팁 |
| 스타일 | `src/theme.css`의 색상·반경 토큰, `src/style.css`의 레이아웃; `@/`는 `src/` alias |
| 상태 색상 | 선택·포커스 파랑, 정상·경고·오류 초록·노랑·빨강. 상태 설명·지연·유실 표시 병행 |
| 테마 | 해/달 버튼, 처음에는 시스템 설정. 직접 선택은 localStorage 보존. 지도 Canvas는 어두운 팔레트 유지 |
| 그래프 | React Flow의 FSM·BT·ROS 연결, 고정 노드 크기. 구조·화면 크기 변경 때만 자동 맞춤 |
| 탐색 | 접는 사이드바, 작은 화면 아이콘 탐색, ROS 하위 탭 방향키 이동 |

FSM 활성 강조는 관측 상태를 최소 **500ms**씩 표시한다. 빠른 전이는 시각적으로
지연될 수 있으나 실제 상태를 함께 표시한다. 상단 안전 정보와 Runtime 동작은
지연하지 않는다. 시각 탐색·세션 변경·Runtime 재시작 시 대기열을 초기화하며
미관측 상태를 복원하지 않는다.

<details>
<summary>FSM·ROS 그래프의 배치와 갱신 규칙</summary>

- 정상 FSM은 좌→우, 취소·오류는 아래에 배치한다. 높이·간격 편차는 기존의 20%로 제한한다.
- 정상 진행은 좌우 포트, 복귀는 위쪽, 취소·오류는 아래쪽 포트다. 같은 면의 입구·출구는
  42%/58% 위치로 분리하고 양방향 전이는 서로 다른 경로를 사용한다.
- FSM은 둥근 상자와 직각 화살표이며 화살표는 도착점에만 표시한다. 연결선은 가능한
  전이여서 활성 상태 색상을 사용하지 않는다. 노드 텍스트는 중앙 정렬한다.
- 전이 조건 요약은 설명이고 실제 판단·정의·활성 상태는 Runtime에서 온다. 노드 선택은
  모듈·작업 결과·실행 기록을 열며 미수신이면 대기를 표시한다.
- ROS 그래프는 같은 publisher/subscriber 노드를 하나로 표시한다. 주기·시각·QoS·endpoint
  순서만 바뀌면 DOM·배치·확대 위치를 유지하고, 실제 연결·선택 토픽이 바뀌면 재배치한다.

</details>

### Foxglove 연결

FSM·BT·미션 진단은 이 콘솔에서 보고 카메라·라이다·3D 분석용 Foxglove 링크도 제공한다.
**Foxglove Bridge 설치·실행·연결은 아직 검증하지 않았다.** 같은 ROS domain의 별도
Bridge가 필요하며 Monitor `/ws`는 Foxglove 프로토콜이 아니다. SQLite 관측 기록도
MCAP/rosbag과 다르다. 연결 방법은 [Foxglove 문서](https://docs.foxglove.dev/docs/visualization/connecting/live)를 따른다.

### 웹 개발과 분리 빌드

```bash
make build-dev-monitor-web
make build-dev-monitor-ros
```

Node.js 22 이상·pnpm 11과 ROS 빌드 환경이 다른 경우 위 순서로 실행한다.
웹을 다시 빌드하면 해시가 바뀐 자산이 install에 포함되도록 ROS 빌드도 실행한다.
개발 중에는 `web` 디렉터리에서 `pnpm dev`를 사용하고 브라우저는 Vite 주소로 접속한다.

## 검증

Core·ROS 공통 검사는 [workspace 명령](../../README.md#개발용-로봇-관제보드)을 따른다.
ROS 검사는 통합 install을 source한 별도 domain에서 활성화한다.

```bash
ROS_DOMAIN_ID=188 CLEANY_RUN_ROS_TESTS=1 make test-dev-monitor
```

브라우저 검사는 아래처럼 실행한다. 기본 실행은 Vite와 HTTP/WebSocket fixture를
사용하며 simulator나 로봇을 시작하지 않는다. 조작·ExecuteOne fixture는 녹화 설정도
자체 응답으로 제공한다. 웹 빌드는 미사용 import·변수·인자를 TypeScript 오류로 검사한다.

```bash
cd ros2_ws/src/cleany_dev_monitor/web
pnpm install --frozen-lockfile
pnpm build
pnpm exec playwright install chromium
pnpm test:e2e
```

실환경 브라우저 검사는 Runtime 관측과 지도가 있는 Monitor 주소를 지정한다.

```bash
MONITOR_URL=http://127.0.0.1:8768 NAV_MONITOR_URL=http://127.0.0.1:8769 pnpm test:e2e
```

실제 ROS/HTTP smoke는 같은 domain에서 Monitor를 먼저 시작한 뒤 실행한다.
아래 시나리오는 주행을 Mock으로 명시한다. `--held`는 같은 install의 Mock 서버를 사용한다.
저장소 루트로 돌아와 실행한다.

```bash
cd "$(git rev-parse --show-toplevel)"
ROS_DOMAIN_ID=187 python3 tools/dev_monitor/smoke.py --output /absolute/new-directory
ROS_DOMAIN_ID=187 python3 tools/dev_monitor/smoke.py --held --output /absolute/new-held-directory
```

smoke 명령은 저장소 루트에서 실행하고 `--output`에는 새 디렉터리를 지정한다.
다른 포트에서는 `--url http://127.0.0.1:<port>`를 추가한다. 물리 로봇에는 연결하지 않는다.

최신 검사 수·DB 무결성·Gazebo 실행과 빌드/병합 제한은
[통합 검증 결과](../cleany_mission_manager/docs/manipulation_mock_integration.md#최신-검증-결과)에서 관리한다.
