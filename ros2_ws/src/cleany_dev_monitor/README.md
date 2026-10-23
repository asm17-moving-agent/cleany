# cleany_dev_monitor

ROS 그래프, 카메라, 조작 단계와 호스트 리소스를 웹으로 관측한다.
PR [#54](https://github.com/asm17-moving-agent/cleany/pull/54)와
[#57](https://github.com/asm17-moving-agent/cleany/pull/57)의
`067562f0651403f6cfaf6ec523f7d1ad2fcbc898`에서 모니터 패키지를 가져와
Jetson 실물 조작 서버에 연결했다.

실제 파지 작업의 메시지별 수신 시각과 단계별 부하를 기록하는 독립 도구는
[파지 중 IO 계측 절차](docs/manipulation-io-probe.md)를 따른다.
`ros2 run cleany_dev_monitor manipulation_io_probe --help`는 구독과 로컬 파일 기록만 수행하며 로봇을 시작하거나
Goal·토크·카메라 제어 요청을 보내지 않는다. 결과는 100ms 갱신 Hz, 메시지 간격,
원본 stamp 지연, 모터 피드백 신선도와 CPU/GPU 시계열로 내보낸다.

## 빌드와 실행

저장소 루트에서 실행한다. 웹 빌드는 Node.js 22 이상과 pnpm 11,
ROS 서버는 ROS 2 Humble과 `package.xml`의 의존성이 필요하다.
빌드한 웹 자산은 ROS 패키지에 설치되므로 서버 실행에는 Node.js가 필요하지 않다.
설치 기준은 [개발환경 안내](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

```bash
make build-dev-monitor
make dev-monitor-start
make dev-monitor-status
# 종료
make dev-monitor-stop
```

`dev-monitor-start`는 현재 사용자의 systemd 서비스로 실행하며 로그인 세션에 속한다.
로그는 `journalctl --user -u cleany-dev-monitor.service`에서 확인한다.
포그라운드 실행과 설정 변경은 다음과 같다.

```bash
make run-dev-monitor
make run-dev-monitor MONITOR_ROS_ARGS='-p host:=127.0.0.1 -p port:=8769'
```

두 실행 방식 중 하나를 선택한다. 기본 Jetson 프로필은 `0.0.0.0:8768`을 사용한다.
외부 PC에서 `http://<Jetson 주소>:8768/#manipulation`을 연다.
같은 네트워크 또는 Jetson의 Tailscale 주소로 접속할 수 있다.
관측 화면은 별도 계정 인증 없이 열며 HTTP/WebSocket은 같은 origin에서 사용한다.
선택적으로 켜는 실물 제어는 아래 조작키 인증이 필요하다.
Loopback만 사용하는 경우 PC에서 SSH 터널을 연결한다.

```bash
ssh -N -L 8768:127.0.0.1:8768 cleany@<Jetson 주소>
# PC 브라우저: http://127.0.0.1:8768/#manipulation
```

## 실물 매니퓰레이션 연결

`tools/dev-monitor`는 Cyclone DDS와 `CLEANY_COLLECTION_ROS_DOMAIN_ID`(기본 0)를
사용한다. `config/jetson_dds.xml`은 자동 선택한 네트워크 인터페이스에서 기존
vision bridge의 peer 주소를 조회하므로 GPU 컨테이너가 꺼져 있어도 웹은 실행된다.
`config/monitor.yaml` 위에 `config/jetson.yaml`을 적용해
`/manipulation/execution_events`의 `ManipulationExecutionRecord`를 구독한다.
실물 서버의 `execution_profile=jetson` 기록과 heartbeat로 현재 실행을 표시한다.
Mission Manager 없이 `make manipulation-run`으로 실행하는 작업도 관측한다.
모니터 시작은 로봇 런타임이나 조작 Goal을 시작하지 않는다.

`make manipulation-run`은 Action 이전에 런타임 준비, 초기 장면 인식과 자동 대상
선택을 수행한다. 웹의 `실행 요청 진행`은 이 구간부터 보여준다. 현재 요청의
execution ID가 서버 기록에 나타나기 전에는 이전 물체의 결과를 대신 표시하지 않는다.
서버의 `ValidateGoal`과 `PrepareTarget`은 준비된 포트와 승인 snapshot을 확인하는
짧은 작업이므로 라이브 화면에서 빠르게 지나갈 수 있다. 실제 관측 구간과 완료 동작은
타임라인과 단계 화면에 남긴다.

실행 기록을 수신하면 현재 동작, 완료 동작 수와 경과 시간을 준비 이력보다 먼저
표시한다. `실행 요청 진행`은 실패와 진행 상태를 별도 상태 영역으로 표시하며,
종료된 요청의 준비 단계는 `준비 단계 이력`을 펼쳐 확인한다. 이력의 `보고됨`은
해당 단계가 관측되었다는 뜻이며 조작 성공을 의미하지 않는다. 실행 흐름의
완료, 진행 중, 실패와 미관측 상태는 색상과 문자로 함께 구분한다.

ROS가 준비되기 전의 명령 진행은 호스트의
`~/.local/state/cleany/manipulation/request.json`에서 읽는다.
`CLEANY_MANIPULATION_PROGRESS_FILE`로 쓰기/읽기 위치를 함께 바꿀 수 있으며
서버의 ROS parameter는 `manipulation_request_file`이다. 동일 execution ID의
준비 이력을 원자적으로 교체하며, 명령 PID와 Linux 시작 시각으로 프로세스 종료와
PID 재사용을 구별한다. 이 관측은 물리 정지나 수거 완료의 증거가 아니다.
파일 쓰기 실패는 로봇 동작이나 취소를 중단하지 않는다.

실물 서버는 실행 중인 기록을 1 Hz로 재전송한다. 반복 기록은 liveness만 갱신하며
revision, 완료 동작, 서버 timestamp와 단계 구간을 바꾸지 않는다.
브라우저 연결, ROS discovery 또는 heartbeat가 3초 이상 오래되면 현재 진행으로
표시하지 않는다. 완료·실패·중단 기록은 heartbeat로 다시 활성화하지 않는다.
미션 모드(`manipulation_observation_mode=mission`)는 기존 Mission ID와 execution ID
일치 조건을 유지한다. 주행 허가와 Mission FSM은 실제 Mission Runtime 입력이
있을 때만 표시한다.

단계 진행과 실행 기록은 실물 서버의 보고다. `pick_return_release` 임시 모드의
`ASSUMED` 안착은 실측 `CONFIRMED`와 구별한다.
전체 실제 BT의 UID와 상태는 기존 1667번 monitor와
`/manipulation/bt_transitions`에서 제공한다. 웹의 9단계/17개 동작은 Action 진행
표시이며 실제 트리 topology를 재구성하지 않는다.

카메라 화면에서 실행 중인 이미지 토픽을 선택할 수 있다. 손목 카메라는 조작
런타임에서 활성화된 동안 수신한다. 리소스 화면은 서버가 실행되는 Jetson의
CPU, 메모리, 프로세스 등을 표시한다. Jetson GPU와 전력의 `tegrastats` 수집은
아직 연결하지 않았다.

## 선택적 웹 실물 제어와 되돌리기

기본값은 관측 전용이다. 최신 `cleany_manipulation_bt`의 상태 서비스와 모니터를
빌드한 뒤 유휴 실행기를 갱신하고 웹 제어를 켠다. 실행 중인 로봇 작업을 먼저
종료한 상태에서 실행한다. 기능을 켜는 명령 자체는 모터를 켜거나 Goal을 보내지 않는다.

```bash
make build-manipulation-bt build-dev-monitor
make manipulation-restart        # 기존 런타임에 실행기가 붙어 있는 경우, 유휴 상태에서
make dev-monitor-control-on
make dev-monitor-control-key     # Jetson 터미널에서 조회하여 웹 인증에 입력
```

`매니퓰레이션`의 `실물 조작 제어`에서 이름과 조작키로 연결한다. 기본 세션은
1시간이며 브라우저 하나만 제어권을 갖는다. 다른 브라우저에서 가져오려면 명시적으로
선택한다. 조작키는 URL이나 브라우저 저장소에 저장하지 않는다. HTTP를 사용하는
개발 화면이므로 기존 로컬 네트워크·Tailscale 또는 SSH 터널에서 사용한다.

상단 상태 영역은 실행기 연결, 제어기, 현재 작업과 실행 차단을 표시한다.
넓은 화면에서는 왼쪽에 조작과 중단 버튼, 오른쪽에 현재 실행 요약, 승인 대상과
단계별 결과를 나란히 배치한다. 좁은 화면은 실행 요약, 조작, 상세 관측 순으로
이어진다. `조작 권한 연결`을 펼쳐 인증하며 중단 버튼은 접지 않는다.
`복구 및 종료`와 `웹 명령 이력`은 필요할 때 펼친다.

`다음 작업 제안받기`로 대상을 검토한 뒤 승인하거나, `물체 1개 실행`으로 기존
자동 선택 명령을 실행한다. 인증 후 사용할 수 없는 버튼은 제한 이유를 함께
표시하며, 공통 이유는 한 번만 안내한다. 웹 제어가 꺼져 있으면 `웹 제어 꺼짐`
안내와 관측 화면을 유지한다. 라이트, 다크와 좁은 화면에서도 동일한 인증,
확인 절차와 최근 5개 명령 이력을 제공한다.

| 버튼 | 실제 동작과 확인 |
| --- | --- |
| 로봇 준비 | 기존 `collection-runtime start`와 `action-start`; 토크 활성화 확인 필요 |
| 장면 인식만 | `/perception/inspect_scene`의 detector 단계만 실행하여 물체 목록 표시 |
| 다음 작업 제안받기 | 새 장면에서 추적 가능한 적격 물체 하나를 추천하고 관측 이미지, 행동, 선정 이유 표시 |
| 승인하여 1회 실행 | 승인한 물체의 동일성과 유효성을 재관찰로 확인한 뒤 승인한 snapshot/object의 Action 한 번 실행 |
| 이 물체 건너뛰기 | 같은 추적 세션에서 해당 물체를 제외하고 새 제안 생성 |
| 물체 1개 실행 | 저장소 루트의 `make manipulation-run` 직접 호출; 초기 인식, 자동 대상 선택과 실물 Action 실행 |
| 즉시 중단 요청 / 현재 단계 후 중단 | 정확한 활성 execution ID의 기존 CancelManipulation; Action 전에는 웹 작업 또는 검증된 CLI 프로세스 그룹에 SIGINT |
| 팔 기본 자세 복귀 | 선택 그리퍼를 먼저 여는 기존 `arm-return`; 양팔·왼팔·오른팔 선택과 확인 필요 |
| 실행 차단 해제 | 정지·양쪽 빈 그리퍼·메모와 검토한 execution ID/revision으로 기존 복구 서비스 호출; 팔 이동 없음 |
| 로봇 종료·토크 해제 | 유휴 상태에서 기존 `collection-runtime stop`; 팔과 물체 지지 확인 필요 |

### 사용자 승인 기반 작업 제안 (Task Proposals)

Mission Manager를 통하지 않고도, 웹 UI 상에서 로봇이 감지한 물체 중 최적의 대상을 추천받아 사용자가 1회씩 직접 승인하며 작업을 진행할 수 있습니다.

1. **작업 제안 요청 (`다음 작업 제안받기`)**:
   - 카메라 뷰에 감지된 물체 중 로봇(base_link)과 가장 가깝고 유효한(크기/깊이 필터 통과) 객체를 추천합니다.
   - 화면에 초록색 테두리로 강조된 대상 물체, 추천 행동(`pick_and_dump`), 선정 이유가 표시됩니다.
2. **동일성 재검증 및 승인 실행 (`승인하여 1회 실행`)**:
   - 승인 버튼을 누르면 새 프레임을 촬영하여 해당 물체가 여전히 그 자리에 있는지 재검증합니다.
   - **재검증 기준**: 제안 생성 후 **90초 이내**, 위치 이동 **1.5cm(0.015m) 이내**, 바운딩 박스 **IoU 0.7 이상** 유지.
   - **주변 환경 변화 허용**: 다른 물체가 추가되거나 사라지더라도, **승인한 대상 물체 자체가 안정적으로 유지된다면 승인이 유지**됩니다. (주변 장애물과의 충돌 여부는 후속 경로 계획 단계에서 안전하게 검사됨)
   - 재검증 통과 시 해당 물체에 대한 1회 조작 Action Goal을 전송합니다.
3. **물체 건너뛰기 (`이 물체 건너뛰기`)**:
   - 현재 추천된 대상을 제외하고 다음으로 적합한 다른 물체를 추천받습니다.
4. **결과 확인 및 후속 제안**:
   - 동작이 완료되면 성공/실패 여부를 표시하고 다음 제안 대기 상태로 전환됩니다.
   - 사용자의 명시적인 승인 없이 **후속 작업이 자동으로 실행되지 않습니다**.

> [!NOTE]
> - `물체 1개 실행`: 제안 검토 단계를 건너뛰고 기존 자동 선택 규칙으로 즉시 1회 수거를 실행하는 단축 버튼입니다.
> - `pick_return_release` 모드: 물체를 집은 뒤 원래 위치에 되내려놓는 시험 모드입니다. 수거함 안착(`CONFIRMED`)과 구분하여 `ASSUMED` 상태로 기록됩니다.

중단은 소프트웨어 요청이며 물리 비상정지 버튼을 대체하지 않는다. 접수 ACK나
명령 프로세스의 종료를 실물 정지·안착·수거 성공으로 표시하지 않는다. 실물 결과는
기존 실행 기록에서 확인한다. 연결 끊김, 오래된 상태와 기록 재생에서는 동작 버튼을
잠근다. 실행기 상태를 조회할 수 없거나 제어기가 준비되지 않으면 이유를 표시한다.
기존 오류, 실제 보호 조건과 과거 차단을 웹에서 임의로 초기화하지 않는다.

복구 요청의 revision은 확인창을 열 때 고정하며, 전송·실행 시 서버가 다시 비교한다.
응답을 잃은 실행 요청은 자동 재전송하지 않는다. `같은 요청 다시 보내기`는 동일
UUID를 재사용하고 기존 작업을 조회하므로 새 동작을 중복 생성하지 않는다.
작업 프로세스가 사라지면 `결과 확인 필요`로 막고, 운영자가 실물과 결과를 확인한 뒤
`명령 기록 확인`에 메모를 남긴다. 이는 기록 확인이며 로봇 오류 해제가 아니다.

다음 명령으로 기존 관측 전용 동작으로 되돌린다. 기존 CLI 명령은 그대로 사용할 수 있다.

```bash
make dev-monitor-control-off
# 다시 켜기
make dev-monitor-control-on
```

토글은 모니터만 재시작하며 로봇 런타임은 종료하지 않는다. 시작·진행 중 웹 명령이나
CLI 작업이 잠금을 보유하면 토글을 거절하므로 먼저 해당 작업의 종료를 확인한다.
제어 모드 설정은 `~/.local/state/cleany/dev-monitor/operator/enabled`에 남아 다음
모니터 시작에도 적용된다. 재부팅 후 로봇 동작이나 웹 작업을 자동 재개하지 않는다.
모니터 서비스 자체의 부팅 자동 시작은 별도 설정이며 이 기능에서 변경하지 않는다.

웹 작업은 별도 systemd 사용자 scope에서 실행한다. 모니터 재시작·브라우저 종료만으로
로봇 프로세스가 종료되지 않는다. UI/CLI는 제어 기능이 켜진 동안 공통 `flock`으로
중복 시작을 막고, 실제 실행기는 기존 소유권 검사와 정지·복구 검사를 유지한다.
모든 프로세스를 일괄 kill하는 복구 API나 임의 셸 명령 API는 제공하지 않는다.

`operator/jobs/<UUID>.json`과 같은 이름의 `.log`에 명령·조작자·결과를 남긴다.
`물체 1개 실행`의 명령은 `make -C <저장소> manipulation-run`으로 고정한다.
Makefile의 기존 `tools/collection-runtime action-run`과 같은 인식·자동 대상 선택·
파지·복귀 절차를 사용하며 웹 전용 목표 선택기나 IK 설정을 두지 않는다.
웹의 인증·동시 작업 방지·실행 준비 확인은 이 명령 앞에서 수행한다.

`make dev-monitor-control-on`을 실행한 터미널의 운영 옵션을 0600 설정 파일에 저장해
다음 웹 작업에 전달한다. `COLLECTION_QUERY`, `CLEANY_COLLECTION_*`의 도메인·
계획 backend·grasp predictor·대기 시간·GUI·state directory,
`CLEANY_MANIPULATION_COLLECTION_CONFIG`와 진행 파일, 로컬 scene/MoveIt prefix를 포함한다.
터미널에서 별도 옵션을 사용했다면 같은 옵션으로 웹 제어를 활성화한다. 이후 터미널
환경변수를 바꾼 경우 유휴 상태에서 `make dev-monitor-control-on`으로 다시 반영한다.
다시 활성화할 때 현재 터미널에 없는 허용 옵션은 저장된 값을 유지하며, 명시한
환경변수만 덮어쓴다. 처음 활성화할 때는 로봇별 collection, control, motion-monitor
설정 경로를 지정해야 한다. 저장 옵션을 제거하려면 유휴 상태에서 0600 설정 파일의
해당 항목을 삭제한다.
API key 값은 웹 응답이나 명령 이력에 노출하지 않는다.
화면의 웹 명령 이력은 최근 5개만 표시하고 원본 기록은 보존한다. 실행이 실패하면
해당 명령 로그의 최종 `GetManipulationExecution` 결과를 읽어 실행 ID, 오류와 실패
단계를 표시한다. 예를 들어 양팔 모두 파지 자세를 찾지 못한 경우 위치·접근 방향을
확인하도록 안내한다. 이 로그 요약은 명령 이력 설명이며 새 동작의 허가나 실제
정지 판정에는 사용하지 않는다. 기존 실행기·guard의 검사는 그대로 적용한다.
`key`와 필요 시 보관하는 `runtime-settings.json`은 0600이다. 설정 디렉터리는
`XDG_STATE_HOME`을 따르며 ROS parameter는 `control_directory`,
`control_repository_root`, `control_enabled`와 `control_session_seconds`이다.
`control_enabled=true`와 `enabled` 파일이 모두 있어야 변경 요청을 허용한다.

HTTP API는 `/api/control/status`, `login`, `logout`, `jobs`, `cancel`,
`jobs/<UUID>/acknowledge`이다. 변경 API는 동일 origin과 HttpOnly/SameSite 세션을
검사한다. `jobs`는 `request_id`, 허용된 `action`, 엄격한 `args`만 받는다.
관측 collector에는 명령 publisher를 추가하지 않고 선택적 operator ROS 노드를 분리했다.
모니터와 명령 작업의 operator 노드는 프로세스·인스턴스마다 고유한 ROS 이름을 사용한다.
따라서 두 노드가 동시에 존재해도 기존 런타임의 중복 노드 검사를 통과하며, 실제
중복 실행기나 Action 서버를 거절하는 검사는 유지한다.

상태 조회는 한 번에 하나만 전송하고, 느린 응답도 다음 주기의 요청 때문에 버리지
않는다. HTTP 대기는 10초로 제한하며 오래된 상태에서는 기존 버튼 잠금을 유지한다.
녹화 목록은 HTTP 이벤트 루프 밖에서 조회하고 브라우저 간 갱신을 공유한다. 종료된
녹화의 요약은 파일 수정 시각과 크기가 바뀔 때 다시 읽는다. 기록 파일이 많아져도
목록 조회가 라이브 WebSocket과 조작 API의 응답을 막지 않도록 한다.
### 녹화 세션 관리 및 삭제

기록 탭의 `기록 관리, 내보내기` 목록에서 녹화본을 다운로드하거나 불필요한 세션을 안전하게 정리할 수 있습니다.

- **개별 세션 영구 삭제**:
  - 목록 오른쪽의 휴지통 아이콘을 누르면 해당 세션의 SQLite DB 파일, 저널/WAL 파일, `bags/<session>/` 원본 Rosbag 데이터가 함께 삭제됩니다.
  - 다른 녹화 세션, 모니터 운영 설정 및 웹 명령 이력은 보호됩니다.
- **삭제 시 안전 조건**:
  - 현재 녹화가 진행 중인 세션은 삭제할 수 없습니다. (`녹화 정지` 후 삭제 가능)
  - 용량 한도 초과나 센서 에러로 중단된 세션은 파일 락(Writer)이 안전하게 닫힌 후 삭제가 가능합니다.
  - 현재 재생 중인 세션을 삭제하면 자동으로 라이브 관측 모드로 복귀합니다.
- **인덱스 최적화**: 신규 녹화 시 `records(kind, received)` SQL 인덱스를 자동 생성하여 대용량 녹화본에서도 UI 지연 없이 고속 조회가 가능합니다.

이 적용 전 소스 백업은 이 Jetson의
`~/.local/state/cleany/dev-monitor/backups/before-controls-20261021-033539/`에 있다.
`sources.tar.gz`와 SHA256 `manifest.json`으로 기존 작업까지 보존했다. 일반적인
원복은 위 feature-off 명령으로 충분하다. 같은 디렉터리의 `controls-change.patch`는
이 기능에 한정한 소스 차이다. 소스까지 제거하려면 유휴 상태에서 제어를 끄고
저장소 루트에서 `git apply --reverse --check <패치 경로>`로 이후 변경과 충돌 여부를
먼저 확인한 뒤 `git apply --reverse <패치 경로>`를 적용하고 두 패키지를 다시 빌드한다.
기존 소스를 보존하기 위해 `git reset`이나 `git clean`을 사용하지 않는다.

## 검증

Linux/Jetson 호스트 리소스는 매 표본마다 `/proc/<PID>/stat`를 한 번 읽어
프로세스 이름, CPU 누적 시간, 시작 tick과 RSS를 함께 수집한다. 전체 프로세스와
호스트 지표의 기본 1초 수집 주기는 유지하며 PID 재사용 시 CPU 기준을 초기화한다.
다른 운영체제에서는 psutil을 사용한다. 녹화는 중복 제외가 필요한 지도 레이어만
내용 hash를 계산하고, 녹화 정책 조회와 화면용 상태 계산을 분리한다.
WebSocket과 주기 녹화는 새 sequence의 행만 가져온다. 최초 접속과 출처 변경에는
전체 최신 상태를 보내며 이벤트 ring의 누락 표시, 레이어 중복 제외와 저장 한도는 유지한다.

SQLite·journal·bag의 용량 검사는 `scandir`의 파일 정보를 사용해 중복 조회를
줄인다. 매 기록 전 현재 크기를 새로 확인하므로 다른 세션의 파일 증가도 한도에
반영한다. 녹화용 토픽과 CameraInfo 경로는 설정 변경 시 한 번 계산한다.
`RecordingControls.configure()`와 `source_config` 교체는 같은 lock 안에서
캐시를 갱신하고 이전 설정의 수신 ticket을 무효화한다. `source_config` 조회는
복사본을, 도출된 토픽 목록은 읽기 전용 객체를 반환한다.

합성 ROS 입력 51개 publisher, 16개 선택 토픽, WebSocket 1개와 2초 간격 HTTP 조회로
모니터 프로세스 CPU를 비교할 수 있다. 별도의 localhost DDS domain 180을 사용하며
실제 로봇 런타임을 시작하거나 조작 서비스를 호출하지 않는다. CPU는 한 코어를
100%로 계산하며 합성 publisher와 브라우저의 CPU는 포함하지 않는다.
시작 후 5초를 제외하고 기본 30초를 측정한다. 출력 경로는 새 디렉터리여야 한다.

```bash
make benchmark-dev-monitor-cpu MONITOR_CPU_OUTPUT=/absolute/new-monitor-cpu-run
# 백업된 패키지로 동일 부하 비교: 해당 경로 아래 cleany_dev_monitor 모듈 필요
make benchmark-dev-monitor-cpu MONITOR_CPU_OUTPUT=/absolute/new-baseline-run \
  MONITOR_CPU_SOURCE=/absolute/baseline
```

`report.json`에는 CPU, RSS, WebSocket 수신·누락, 상태 조회 지연과 SQLite/rosbag2
index 기록 건수가 남는다. 이 수치는 합성 관측 부하의 비교이며 실제 파지 작업의
최대 부하나 성공률 검증은 아니다.

CPU 비교의 원자료와 환경별 결과는 저장소 밖 데이터 루트에 보관한다. 현재 브랜치의 회귀 검증 범위와 원본 위치는 [EVIDENCE](../../../docs/EVIDENCE.md)를 따른다.

카메라 화면에서 손목 `image_raw`를 선택하면 해당 publisher의 5Hz
`preview/compressed` 토픽을 우선 구독한다. 헤드 컬러는 드라이버의
`image_raw/compressed`가 있으면 이를 사용한다. 헤드 JPEG 발행률은 카메라 설정을
따르며 웹 화면 갱신은 기본 5fps다. 원본을 기록하거나 토픽 진단에서 명시적으로 선택한
경우에는 원본 구독이 추가될 수 있다.
미리보기 API는 `X-Camera-Topic` 응답 헤더로 실제 구독 토픽을 알린다.
지원 크기의 JPEG는 재인코딩하지 않고 전달한다. JPEG 캐시는 원본 도착마다 최신
프레임으로 갱신하고, `camera_fps` 제한은 픽셀 인코딩이 필요한 입력에 적용한다.
웹 요청은 기본 200ms 간격이다. 알려진 원본 깊이와 compressedDepth는 미리보기
목록과 API에서 제외하지만 ROS 토픽 진단과 원본 기록에는 남긴다.
어떤 미리보기 선택도 손목 `set_enabled`를 호출하지 않는다.

```bash
make test-dev-monitor
make build-dev-monitor-web
cd ros2_ws/src/cleany_dev_monitor/web
pnpm test:e2e
```

ROS 검사는 localhost의 독립 domain 179에서 실행한다. 웹 검사는 Playwright Chromium이
필요하다. 실제 Jetson 서버 연결 검사는 모터 Goal을 보내지 않고 관측과 heartbeat를
검사한다. 기록은 기본 `~/.local/state/cleany/dev-monitor`에 저장하며 자동 삭제하지 않는다.

상세 규약은 [데이터](docs/data.md)와 [ROS 인터페이스](docs/interfaces.md)를 따른다.
