# cleany_bringup

Gazebo·AMCL/Nav2·Mission Runtime과 선택형 Mock 조작 서버를 함께 실행한다.
저장 지도로 localization하며 `cleany_navigation/amcl_nav2.launch.py`를 포함한다.
SLAM은 시작하지 않는다. 팀 표준은 **ROS 2 Humble / Gazebo Fortress**다.

| 실행 목적 | 안내 |
|---|---|
| Gazebo 주행 + 외부 Mock 조작 | [빠른 시작](#빠른-시작) |
| 기존 로컬 Mock 구성 | [기존 로컬 Mock 실행](#기존-로컬-mock-실행) |
| launch 설정과 backend 조합 | [실행 인자](#실행-인자) |
| 초기 위치·주행 설정 확인 | [주행 준비와 설정](#주행-준비와-설정) |
| 자동 통합 검사·검증 결과 | [검증](#검증) |

## 빠른 시작

주행은 Gazebo/Nav2, 인식·계획·팔 조작은 Mock이다. 서버 성공은 논리적 책상 목록을
갱신한다. Gazebo 팔·물체의 물리 수거를 실행하는 구성은 아니다.

### 1. 빌드와 환경

명령은 저장소 루트에서 실행한다. 아래는 Bash 기준이며 zsh와 기존 VM 활성화는
[workspace 환경 안내](../../README.md#외부-데이터와-빌드-경로)를 따른다.

```bash
make build-mission-sim
source "$(make -s print-install-base)/setup.bash"
export ROS_DOMAIN_ID=186
```

Mission·서버·interfaces는 같은 install을 사용한다. 기존 Gazebo/Nav2가 실행 중이면
[Runtime 단독 실행](../cleany_mission_manager/README.md#설정과-실행)을 사용한다.

### 2. Gazebo와 Mock 서버 실행

```bash
ros2 launch cleany_bringup mission_sim.launch.py \
  profile:=fortress headless:=true \
  runtime_config:=$PWD/ros2_ws/src/cleany_mission_manager/config/runtime_gazebo_manipulation_mock.yaml \
  start_manipulation_mock:=true manipulation_scenario:=success \
  post_mission:=wait_for_next enable_bridge:=false enable_telemetry:=false \
  journal_path:=$HOME/.local/state/cleany/mission-mock/missions.db \
  bridge_journal_path:=$HOME/.local/state/cleany/mission-mock/bridge.db \
  manipulation_database_path:=$HOME/.local/state/cleany/mission-mock/manipulation.sqlite3
```

첫 확인은 Gateway 없이 `wait_for_next`로 진행한다. 세 DB는 서로 분리하며, 재시작
복구를 확인할 때는 같은 DB를 보존한다. 정상 수거의 Mock 시간은 물체당 약 40초다.

### 3. 준비 상태 확인과 미션 요청

새 터미널에서도 같은 install과 `ROS_DOMAIN_ID=186`을 사용한다.

```bash
ros2 topic echo /mission/status
```

`ready: true`를 확인한 뒤 별도 터미널에서 요청한다. `NAV2_NOT_READY`이면
[주행 준비](#주행-준비와-설정)를 먼저 확인한다. 새 미션에는 새 `mission_id`를 사용한다.

```bash
ros2 service call /mission/offer cleany_interfaces/srv/OfferMission \
  "{mission_id: gazebo-mock-demo-001, mission_type: clean_desk, target_kind: SEAT, target_id: seat-12, requested_by: manual-test}"
```

수락 응답은 완료 결과가 아니다. 최종 결과와 보고는 다음 토픽에서 확인한다.

```bash
ros2 topic echo /mission/result
```

정상 시 `trash-1`, `trash-2`를 서로 다른 execution ID로 처리하고 추가 빈 관측까지
확인한다. 출처는 `navigation=sim`, `perception/planning/execution=mock`이다.
화면 관측은 [Dev Monitor](../cleany_dev_monitor/README.md#설치와-실행)를 사용한다.

## 기존 로컬 Mock 실행

`start_manipulation_mock`의 기본값은 `false`다. 기존 `runtime.yaml`을 사용하면
조작은 로컬 Mock이며 아래 명령처럼 Gateway·pose relay를 함께 연결할 수 있다.

```bash
ros2 launch cleany_bringup mission_sim.launch.py \
  profile:=fortress headless:=true post_mission:=wait_for_next \
  gateway_url:=ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws \
  journal_path:=$HOME/.local/state/cleany/missions.db \
  bridge_journal_path:=$HOME/.local/state/cleany/control-bridge.db
```

기존 pose producer가 있으면 `enable_telemetry:=false`를 지정한다. 기본 종료 정책은
`return_home`이며, `wait_for_next`는 현재 위치에서 다음 배차를 기다린다.

## 실행 인자

| 인자 | 기본값 | 용도 |
|---|---|---|
| `profile` | Humble: `fortress`, Jazzy: `harmonic` | simulator 선택 |
| `headless` | `true` | GUI 생략 |
| `map` | `study_cafe_26cm.yaml` | 저장 지도 |
| `targets_file` | `study_cafe_targets.yaml` | 지도 기준 home·좌석 pose |
| `runtime_config` | `runtime.yaml` | Runtime backend·timeout 설정 |
| `post_mission` | `return_home` | `return_home` 또는 `wait_for_next` |
| `gateway_url` | 로컬 Gateway WebSocket | Bridge 목적지 |
| `journal_path`, `bridge_journal_path` | 사용자 state 디렉터리의 별도 DB | Mission·Bridge 기록 |
| `enable_bridge`, `enable_telemetry` | `true` | Gateway·pose relay 시작 |
| `pose_url` | 빈 값 | 빈 값이면 Gateway host의 `/pose/ws` 사용 |
| `sim_partition` | `cleany-mission` | Gazebo transport 격리 |
| `start_manipulation_mock` | `false` | 외부 Mock 서버 동시 시작 |
| `manipulation_scenario` | `success` | 공유 fixture의 시나리오 |
| `manipulation_mock_fixture` | Mission의 `manipulation_server_fixture.yaml` | Runtime·서버 공통 입력 |
| `manipulation_database_path` | `~/.local/state/cleany/mission-mock/manipulation.sqlite3` | 서버 전용 DB |

| Runtime의 `manipulation_backend` | 서버 시작 | 처리 |
|---|---|---|
| `mock` | `false` | 기존 로컬 Mock |
| `action_mock` | `false` | 별도 서버 연결; 미준비 시 미션 수락 차단 |
| `action_mock` | `true` | `/mock` 서버를 함께 시작 |
| `mock` | `true` | 설정 오류로 시작 거절 |

서버 동시 시작 시 backend, snapshot wire ID 목록, 목적지, scenario·서버 설정과 DB
분리를 Gazebo 시작 전에 검사한다. 기본 옵션과 계약 원본은 [launch 파일](launch/mission_sim.launch.py)이다.

## 주행 준비와 설정

지도 파일명과 targets의 `map_id`가 일치해야 한다. 수동 실행에서 AMCL 초기 위치가
필요하면 [지도 기준 초기 pose](../cleany_gazebo_sim/config/nav2_amcl.yaml)를 사용한다.
SDF world 좌표를 그대로 `/initialpose`에 넣지 않는다. 자동 통합 검사는 초기 pose를 설정한다.
Nav2 lifecycle·도착·정지 조건은 [Mission Manager](../cleany_mission_manager/README.md#실행-흐름과-소유권)를 따른다.

`action_mock`의 추가 주행 구성은 다음과 같다.

| 항목 | 적용 내용 |
|---|---|
| 센서 | Gazebo `lidar_depth_nav` profile |
| 정지 체인 | `nav2_safety.yaml`의 collision monitor와 lifecycle manager; Nav2 명령을 monitor로 전달 |
| controller | `nav2_omni_motion.yaml`의 mecanum controller, 도착 허용 오차 3 cm |
| Mission 도착 검사 | 기존 8 cm gate 유지 |
| costmap | 기존 monitor envelope와 같은 footprint; Navfn planner 유지 |

home·좌석은 시뮬레이션 접근 후보이며 충전 dock·팔 도달성·실환경 좌표의 검증값이 아니다.
`/ground_truth/odom` pose relay도 AMCL 정확도의 증거가 아니다.
Jazzy/Harmonic은 별도 호환 환경이며 `ROS_SETUP`과 build/install/log 경로를 분리한다.

## 검증

[공통 검사 명령](../../README.md#missionmock-조작-검증)에서 Gazebo 검사와 home 복귀
검사를 선택한다. 기본 검사는 study-cafe의 `seat-12`, 새 분리 DB, Gateway 없는
`mission/offer`로 단일 clock, Nav2 도착·정지, 두 Action·추가 빈 관측·보고 형식을 확인한다.

성공한 실행 조건, 과거 home 시험과 현재 재빌드 제한은
[통합 검증 결과](../cleany_mission_manager/docs/manipulation_mock_integration.md#최신-검증-결과)를 확인한다.
