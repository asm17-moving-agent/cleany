# Cleany 개발용 관제보드

로봇 구현 레포 안의 독립적인 **읽기 전용 개발 도구**다. 운영용
`cleany-control-dashboard`의 미션 요청/취소 기능은 그대로 사용한다.
이 보드는 미션 수락, 취소, reset, 속도 발행, 임의 service/action 호출을 제공하지 않는다.

## 제공 화면

| 화면 | 관측 내용 |
|---|---|
| 실행 흐름 | Mission FSM, 현재 상태와 전이 이벤트, 청소 py_trees 구조/상태/이번 tick 방문 노드, 모듈 준비/정지 보고, 주행 차단 이유, 작업 대상/목적지, 조작 결과 |
| ROS 탐색기 | topic/node/service/action 목록, 선택 토픽의 publisher→topic→subscriber 그래프, endpoint QoS, 수신 Hz, 메시지 미리보기 |
| 자율주행 | 주행 개요, 장애물·안전, 계획·제어, 센서·위치, 주행 비교의 5개 하위 탭. 지도/실제 궤적, 단계별 명령/실측 속도, guard/pan/BT 진단과 공통 타임라인 |
| 카메라 | Image / CompressedImage 토픽 선택, 실시간 JPEG 미리보기, 일시정지, 수신 지연 표시 |
| 로그 | `/rosout`, FSM/BT/action 이벤트와 결과를 수신 시각순으로 조회. 노드/미션/문자열 및 로그 수준 필터 |
| 기록 & 재생 | 항목별 ON/OFF·프리셋·원본 주기·용량 제한, SQLite 세션 선택 및 재생·내보내기 |

청소 BT는 Runtime이 실제로 생성한 트리다. 책상 도착 전에는 트리 생성 대기 상태로
표시한다. Nav2 또는 팀원 조작 서버의 내부 트리를 추측해서 그리지 않는다.
자율주행의 Nav2 BT 화면은 최종 기본 XML에서 전달한 구조와 `BehaviorTreeLog`의 관측 상태를 표시한다.
구조가 없는 환경에서는 관측된 노드 목록만 제공하며 로그에서 부모 관계를 추측하지 않는다.
Nav2/조작 action의 status와 feedback은 ROS 탐색기로 확인하며, goal/result는 Runtime이나
조작 실행 이벤트에서 실제 제공한 것만 보여준다. 모듈의 mock/sim 표시는 Runtime의
`execution_profile`에서 가져온다. 현재 Runtime은 물리 로봇 실행을 지원하지 않는다.
청소 BT 화면에는 Runtime의 `manipulation_feedback`에 있는 조작 단계와 세부 동작을
별도로 표시한다. `ExecuteOne`의 내부 진행을 읽기만 하며 BT 노드를 추가하거나
완료·주행 조건을 판정하지 않는다. 오래된 정보에는 마지막 조작 관측임을 표시한다.

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

## 설치와 실행

ROS 2 Humble 환경, 같은 DDS domain, 같은 메시지 overlay가 필요하다. 의존 설치 기준은
[개발환경 문서](../../../docs/DEVELOPMENT_SETUP.md), 공통 명령은
[workspace README](../../README.md)를 따른다.

```bash
# 저장소 루트, ROS 환경의 터미널
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src/cleany_dev_monitor --ignore-src -r -y
make build-dev-monitor
ROS_DOMAIN_ID=187 make run-dev-monitor
```

브라우저에서 <http://127.0.0.1:8768>을 연다. 원격 ROS 호스트는 기본 localhost bind를
유지하고 `ssh -L 8768:127.0.0.1:8768 robot-host`로 전달한다. 공개 배포용 인증 서버는 아니다.

### Mission Mock 통합 VM에서 확인

`cleany-mission-manipulation-mock` 작업트리의 통합 install에 웹과 monitor를 포함해 빌드했다.
이 VM에서는 `cleany-mission-mock-monitor.service` 사용자 서비스가 domain 185,
`127.0.0.1:8768`에서 실행된다. 터미널을 닫아도 유지된다.

```bash
systemctl --user status cleany-mission-mock-monitor.service
journalctl --user -u cleany-mission-mock-monitor.service -n 30 --no-pager
# 관제보드 종료
systemctl --user stop cleany-mission-mock-monitor.service
```

서비스가 종료·회수되었거나 VM을 재시작했다면 다음 명령으로 다시 실행한다.

```bash
systemd-run --user --unit=cleany-mission-mock-monitor --collect \
  /usr/bin/bash /home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/start-monitor.bash
```

웹이 `Runtime 대기`를 표시하면 monitor 연결은 정상이고 같은 domain의 Runtime을 기다리는 상태다.
zsh 터미널에서 다음 시험을 실행하고 웹의 `실행 흐름 → 청소 BT`를 본다.

```zsh
cd /home/ubuntu/Documents/cleany-mission-manipulation-mock
source /home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/activate.zsh
make test-gazebo-manipulation BASE_TEST_ROS_DOMAIN_ID=185
```

시험은 종료 후 simulator와 Runtime을 닫는다. monitor가 미리 실행 중이었다면
`기록 & 재생`에서 해당 미션을 다시 볼 수 있다. 지금부터 수신한 데이터를 기록하며
monitor를 시작하기 전에 종료된 미션의 상태를 재구성하지 않는다.
상단 `LIVE`는 웹 관제보드 연결 상태다. Runtime 수신이 3초 이상 끊기면 현재 주행은
`확인 불가`로 표시하고 FSM/BT에는 마지막 관측임을 명시한다. 남아 있는 BT의 SUCCESS는
당시 실행 결과이며 현재 주행 준비 여부를 보장하지 않는다. 미션 종료 뒤 새 상태가
수신되는 경우에도 청소 BT는 `마지막 청소 결과`로 구분한다.

`config/monitor.yaml`에서 runtime namespace, map/base frame, 지도/경로/costmap/속도 토픽,
저장 위치와 용량을 설정한다. 토픽 이름은 실제 launch의 remapping과 맞춰야 한다.
예를 들어 safety chain 중간 토픽이 추가되면 `velocity_topics`에 관측 순서대로 추가한다.
기본값 자체가 특정 로봇의 실제 안전 체인을 확정하지 않는다.

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

```bash
# Gazebo/ROS clock을 사용하는 경우
ROS_DOMAIN_ID=189 make run-dev-monitor \
  MONITOR_ROS_ARGS='-p use_sim_time:=true -p port:=8769'
# namespace가 /robot1인 Runtime
make run-dev-monitor MONITOR_ROS_ARGS='-p runtime_namespace:=/robot1'
```

프런트엔드 빌드와 ROS 빌드 환경이 다른 경우:

```bash
# Node.js 22 이상 / pnpm 11: 프런트엔드 빌드
make build-dev-monitor-web
# ROS 2 환경: 빌드된 web/dist를 포함해 설치
make build-dev-monitor-ros
```

웹 파일을 다시 빌드하면 `make build-dev-monitor-ros`도 실행해야 해시가 바뀐 자산이
install에 포함된다. 개발 시 `web` 디렉터리의 `pnpm dev`도 사용할 수 있다.
Vite proxy와 API가 같은 origin을 사용하도록 브라우저는 Vite 주소로 접속한다.

## 프런트엔드 구성과 Foxglove

- React 19 / Vite 6 / TypeScript strict / Tailwind CSS 4 / shadcn/ui (Radix) 구성이다.
- `web/components.json`은 shadcn 설정, `src/components/ui/`는 소유하는 컴포넌트 소스다.
  버튼, 배지, 카드, 입력, 탭, 구분선, 툴팁을 공통 컴포넌트로 사용한다.
- 화면에는 상태·차단 이유·식별자·핵심 동작을 우선 배치한다. 반복 안내, 장식 문구,
  푸터 설명은 생략하며 실행 환경(mock 등), 데이터 지연과 유실 표시는 유지한다.
- 배경·패널은 무채색, 선택·포커스는 파란색, 정상·경고·오류는 초록·노랑·빨강으로 구분한다.
- 색상·반경 토큰은 `src/theme.css`, 콘솔 레이아웃은 `src/style.css`에서 관리한다.
  `@/`는 `src/` alias다. React Flow는 FSM/BT와 ROS 연결 그래프를 담당한다.
- 상단 해/달 버튼으로 라이트·다크 테마를 전환한다. 처음에는 시스템 설정을 따르며,
  직접 선택한 값은 브라우저 localStorage에 저장한다. FSM/BT/ROS 그래프와 진단 상태 색상도
  테마 토큰을 공유한다. 지도 Canvas는 레이어 판독을 위해 기존 어두운 팔레트를 유지한다.
- 실행 흐름은 상태 요약 한 줄과 FSM/청소 BT 전환 그래프를 중심으로 표시한다.
  정상 FSM 상태는 좌→우 방향으로 배치하고, 높이·간격 편차는 기존의 20%로 제한한다. 취소·오류는 아래에 배치한다.
  정상 진행은 좌우 포트, 복귀 전이는 위쪽, 취소·오류 전이는 아래쪽 포트로 분리한다. 연결선은 가능한 전이를 나타내므로 활성 상태 색상을 사용하지 않는다.
  FSM은 둥근 상태 박스와 직각 화살표로 표시하고 선 위에 전이 조건의 요약을 붙인다. 같은 면의 입구·출구는 42%/58% 위치로 분리하며 양방향 전이는 서로 다른 경로를 사용한다. 화살표는 도착 지점에만 표시한다. 조건 표시는 설명이며 실제 전이 판단은 Mission Runtime이 수행한다.
  노드 텍스트는 중앙 정렬하며 활성 상태는 색상과 접근성 라벨로 표시한다. 실제 전이 연결은 Runtime이 제공한다.
  그래프 관련 값이 변할 때만 렌더링하며 노드 크기는 고정한다. 자동 화면 맞춤은 구조/화면 크기 변경 시만 실행한다.
  노드를 선택하면 모듈·작업 결과·실행 기록을 한 상세 패널에서 확인한다. Runtime 미수신 시 대기 안내만 표시한다.
- 사이드바는 접을 수 있으며 작은 화면에서는 아이콘 탐색으로 전환한다.
  ROS 하위 탭은 방향키로 전환할 수 있다. 연결 끊김, 오래된 데이터, 기록 재생을 구분한다.
- FSM/BT·미션 진단은 이 콘솔, 카메라·라이다·3D 분석은 Foxglove를 병행하는 방향이다.
  상단 Foxglove 링크와 자율주행 화면의 연결 가이드 링크를 제공한다.
  **Foxglove Bridge 설치·실행·연결 검증은 아직 하지 않았다.** 기존 2D/ROS 화면도 유지한다.
- Foxglove는 같은 ROS domain의 별도 Foxglove Bridge에 연결한다. Monitor의 `/ws`는
  Foxglove 프로토콜이 아니며 SQLite 관측 기록도 MCAP/rosbag 파일이 아니다.
  현재 연결/확장 권한은 [Foxglove 문서](https://docs.foxglove.dev/docs/visualization/connecting/live)를 확인한다.

ROS 없이 프런트엔드 동작을 확인하려면:

```bash
cd ros2_ws/src/cleany_dev_monitor/web
pnpm install --frozen-lockfile
pnpm build
pnpm exec playwright install chromium
pnpm test:e2e
```

기본 브라우저 검사는 Vite를 임시 실행하고 테스트용 HTTP/WebSocket 응답으로
오래된 데이터의 주행 상태, 재생 격리, 키보드 탐색, 좁은 화면을 검사한다.
실제 ROS 검사는 `MONITOR_URL`, Gazebo 지도 검사는 `NAV_MONITOR_URL`을 명시한 경우만 실행한다.
검사가 시뮬레이터나 로봇을 시작하지는 않는다.

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

SQLite `metadata.schema_version=2`, `records(seq, received, kind, key, payload)` 구조다.
`payload`는 zlib 압축 UTF-8 JSON BLOB이며 `cleany_dev_monitor.core.decode_record()`로
읽는다. 버전 1의 평문 JSON 기록도 읽을 수 있다. 내보내기는 SQLite backup API를
사용하므로 기록 중인 세션도 일관된 사본을 받는다.

HTTP GET: `/api/snapshot`, `/api/recordings`,
`/api/recordings/{uuid}/replay?at={unix_seconds}`, `/api/recordings/{uuid}/export`.
`/ws` 입력은 `{"op":"select","topics":["/odom"]}`만 지원한다.
재생 기능은 ROS service/action을 호출하지 않는다.

## 검증

```bash
make test-mission-core test-dev-monitor
# ROS overlay source 후, 다른 실행과 분리된 domain
ROS_DOMAIN_ID=188 CLEANY_RUN_ROS_TESTS=1 make test-dev-monitor
# 정상 mock Runtime을 실제 ROS/HTTP로 검증 (같은 domain의 Monitor가 먼저 실행 중이어야 함)
ROS_DOMAIN_ID=187 python3 tools/dev_monitor/smoke.py --output /absolute/new-directory
# 팀원 cleany_skill_executor overlay까지 source한 경우, HELD 취소 검증
ROS_DOMAIN_ID=187 python3 tools/dev_monitor/smoke.py --held --output /absolute/new-held-directory
# 동작 중인 Monitor의 관측 데이터로 브라우저 통합 검사
cd ros2_ws/src/cleany_dev_monitor/web
pnpm exec playwright install chromium
MONITOR_URL=http://127.0.0.1:8768 NAV_MONITOR_URL=http://127.0.0.1:8769 pnpm test:e2e
```

`NAV_MONITOR_URL`은 지도/경로/두 costmap을 수신 중인 Gazebo monitor일 때만 지정한다.
검사 대상 미션/지도가 없는 빈 monitor에서는 브라우저 검사 사전조건이 충족되지 않는다.
물리 로봇에는 smoke 시나리오를 연결하지 않는다. 실제 수행한 검증과 제한은
[검증 기록](../../../docs/validation/README.md)에 정리한다.

FSM 활성 강조는 관측된 상태를 순서대로 최소 500ms씩 표시한다. 빠른 전이는 화면에서 지연될 수 있으며 이때 실제 상태를 함께 표시한다. 상단 상태·안전 정보와 실제 Runtime 동작에는 지연을 적용하지 않는다. 재생 시각 탐색·세션 변경·Runtime 재시작 시 표시 대기열을 초기화한다. 수신 전에 지나간 상태는 복원하지 않는다.

### Nav2 BT 시각화 및 재생

자율주행 → 계획·제어의 Nav2 BT 패널은 `/navigation/bt_trace`에서 수신한 최종 실행 XML의 구조를 표시합니다. 노드를 선택하면 역할 설명, instance ID, 입력·출력 포트와 최근 상태 전이를 확인합니다. 세 ComputePathToPose 인스턴스를 독립적으로 표시하며 UNKNOWN은 미관측입니다. 2초 이상 스냅샷이 없으면 연결 끊김과 마지막 관측 상태를 표시합니다. 구조가 미수신이면 기존 BehaviorTreeLog 노드 목록을 제공합니다.

구조·상태·XML SHA256·목표 문맥을 기존 SQLite 녹화에 함께 저장합니다. 재생은 해당 시점의 기록된 트리를 사용하며 현재 XML을 읽지 않습니다. 목표 연결은 수신 시점 기준이고, 누락 탐지는 연속 상태 불일치에 한정됩니다. 모니터는 기존처럼 읽기 전용이며 BT를 실행하거나 주행 명령을 보내지 않습니다.

### 녹화 항목 선택과 원본 저장

`기록 & 재생 → 녹화 설정`에서 기본 주행 / 장애물 분석 / 사용자 지정 프리셋과 항목별 ON/OFF를 선택합니다. 위치·속도·미션, 경로, costmap, Nav2 BT, 안전 진단, 로그를 독립적으로 선택하며 청소 BT는 미션 runtime snapshot에 포함됩니다. LiDAR, depth point cloud, Nav2 voxel layer, voxel world, 카메라 원본은 기본 OFF입니다. 화면 표시 및 카메라 preview lease와 녹화 구독은 독립적입니다. 센서별 토픽과 0.1~10Hz 주기, 세션 10~1024MiB 제한을 설정할 수 있습니다. 용량 제한은 이전 기록을 지우지 않고 중단하며 `새 녹화 시작`으로 새 세션을 만듭니다. 전체 저장 한도는 기존 `total_limit_bytes`를 따릅니다.

원본은 ROS 직렬화 CDR 전체 메시지로 SQLite `raw_messages`에 저장합니다. 공간 점/픽셀을 자르지 않으며 시간축에서 설정 주기로 표본화합니다. 원본 활성 시 `/tf`, `/tf_static`도 최대 30Hz로 함께 저장합니다. 메시지 최대 8MiB, 대기 큐 최대 32MiB/128건으로 제한하며 초과는 저장 오류·유실 건수 및 `raw_gap` metadata로 남깁니다. 원본 압축/쓰기 작업은 HTTP worker에서 수행합니다. 원본 구독은 토픽 발견 시 QoS에 맞춰 생성되고 OFF 시 해제되지만 화면 또는 다른 관측 목적의 구독은 유지됩니다. 현재 voxel world 기본 토픽은 `/obstacle_memory/voxels`, Nav2 voxel layer는 `/local_costmap/voxel_grid`이며 발행되지 않으면 미수신으로 표시합니다.

설정은 `state_directory/recording-settings.json`에 저장하고 변경 시점을 `recording_policy`로 녹화합니다. 일시정지 중에는 설정·녹화 상태만 기록합니다. 재생은 당시 정책으로 비활성 자료를 제외하고 미녹화·미수신·입력 중단·저장 오류를 구분합니다. 원본은 세션 내보내기(SQLite)에 포함되며 재생 시각 직전 원본 1건도 내려받을 수 있습니다. 3D 원본 및 카메라 영상의 직접 재생 뷰어는 아직 제공하지 않습니다. 기존 schema 1/2 기록은 그대로 읽고, 새 schema 3 기록은 원본 테이블을 추가합니다.

로컬 녹화 API: `GET/PUT /api/recording/settings`, `POST /api/recording/session`, `GET /api/recordings/{session}/raw?category=point_cloud&at=<wall-seconds>`. 설정 변경은 동일 출처에서만 허용하며 ROS 명령이나 로봇 설정을 변경하지 않습니다.
ROS 연결 그래프는 토픽·노드 연결에 따라 결정적인 정적 배치를 계산한다. Publisher/Subscriber 역할을 가진 동일 노드는 하나로 표시한다. 수신 주기·시각·QoS와 endpoint 순서가 갱신되어도 연결 관계가 같으면 그래프 DOM·배치·확대 위치를 유지한다. 선택 토픽이나 실제 연결 관계가 변경되면 배치를 다시 계산한다.
