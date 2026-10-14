# Gazebo 주행과 Python Mock Manipulation 통합

PR #54 `98e9f6c`의 Mission Runtime 구조를 유지하고 PR #55 `86a6d85`에서 Python Mock
서버 의존 집합만 선별 포함한다. 전체 PR 병합 권고는 이 선별 포함 방침으로 대체한다.
제품 정책이나 실제 팔 안전 기준을 새로 확정하지 않는다.

## 책임과 구현 범위

| 구성 | 책임 |
|---|---|
| 기존 Mission FSM | 수락, 좌석 이동, 작업 진입, 취소, 종료 정책, 보고 |
| 기존 책상 청소 BT | 관찰 → 제안 → 검증 → 한 물체 실행 → 재관찰 → 완료 확인 |
| ManipulationPort | 전송 전 journal, 단일 실행 추적, 결과 검증, 취소·조회·복구 |
| ROSManipulationTransport | Action/service 변환, 동일 Goal UUID, 최신 Feedback 관측 |
| Python Mock 서버 | 조작 세부 단계와 모의 성공·실패·CHECKPOINT 취소 |
| Gazebo/Nav2 | 베이스 주행, 도착, odom·TF·clock과 정지 확인 |

FSM 상태·전이, 청소 BT의 노드·순서와 port의 start/poll/cancel/ready/stopped/safe_to_drive
계약을 유지한다. `collect_trash`만 Mission allowlist에 둔다. 서버의 GRASPING·TRANSPORTING은
Mission 노드가 아니다. 로컬 `manipulation_backend=mock`도 유지한다.

포함 파일은 Skill Executor의 `manipulation/{core,models,mock,ports,steps,store}.py`,
`manipulation_node.py`, 기본 설정과 Mock launch 및 core/ROS 검증 코드다.
실행 등록은 `manipulation_server`만 추가한다. MuJoCo 제어 실행기, native BT,
모니터·CLI와 무관한 인식·MoveIt 리팩터링은 포함하지 않는다.
기존 package dependency는 보존하므로 의존 패키지를 빌드할 수 있지만 이 launch에서 실행하지 않는다.

## 공통 interface와 보고 호환

조작 Action과 Record를 `86a6d85` 정의로 정렬하고 `CancelManipulation`을 등록한다.
GetManipulationExecution과 기존 Mission·관제 interface를 유지한다.
Mission과 서버는 같은 생성 타입과 install prefix를 사용한다. 별도 PR overlay를 겹치지 않는다.

Action transport의 `result`에는 기존 ManipulationOutcome 필드만 담는다.
추가 Result 필드 failed_substage/cancel_mode는 `details`로 분리한다.
조회에서는 substage/completed_substages도 진단으로 읽어 기존 manipulation_boundary journal의
선택적 details에 저장한다. details가 없는 fake transport·과거 journal도 읽는다.
Mission DB schema, ManipulationOutcome, RuntimeReport.actions, MissionResult와 Gateway payload는 유지한다.

Feedback은 Goal의 execution ID와 대조해 최신 값 하나만 보존한다.
새 실행에서 초기화하고, 이전·종료 실행의 늦은 메시지를 무시한다.
debug JSON의 manipulation_feedback/manipulation_details에서 관측한다.
기존 bt_stage는 ExecuteOne 등 청소 BT 단계다. Feedback·debug 오류와 수거 세부 단계는
완료·정지·주행 허가 판정에 영향을 주지 않는다. Mission core에는 ROS callback을 추가하지 않는다.

## 실행 구성과 공유 fixture

공통 빌드·검증은 [workspace README](../../../README.md),
통합 launch는 [bringup README](../../cleany_bringup/README.md),
서버 단독 실행은 [Skill Executor README](../../cleany_skill_executor/README.md)를 따른다.

| backend | 서버 시작 | 처리 |
|---|---|---|
| mock | false | 기존 로컬 Mock |
| action_mock | false | 외부 서버 연결; 미준비 시 수락 차단 |
| action_mock | true | `/mock` 서버를 통합 launch에서 시작 |
| mock | true | Gazebo 시작 전 설정 오류 |

runtime_gazebo_manipulation_mock.yaml은 nav2/action_mock, 목적지 mock_trash_bin,
작업 120초·취소 15초·청소 300초와 새 journal에서만 적용하는 초기 Mock 안전 가정을 명시한다.
launch는 snapshot wire ID 목록, 목적지, scenario·전체 서버 설정과 DB 분리를 먼저 검사한다.
action_mock에서는 기존 readiness의 필수 collision_monitor를 기존 시뮬레이터 설정으로
시작하고 Nav2 cmd_vel을 해당 monitor로 연결한다. readiness 조건을 완화하지 않는다.
monitor polygon은 기존 Humble/Jazzy 변환을 적용하고 Gazebo의 lidar_depth_nav로 필요한
센서를 발행한다. 기존 nav2_omni_motion.yaml의 mecanum controller와 3cm 도착 검사로
기존 8cm Mission gate를 충족하도록 구성한다. Navfn planner는 유지하고
costmap footprint는 기존 monitor envelope와 맞춘다.
기존 study-cafe 지도는 spawn을 원점으로 기록했으므로 SDF world 위치를
그대로 initialpose에 넣지 않는다. 기존 simulator AMCL의 지도 기준 초기 pose를 사용한다.
기존 launch 기본 종료 정책은 return_home이며 첫 시험은 wait_for_next를 명시한다.

공유 fixture는 다음 네 관찰을 사용한다. 표시 문자열에서 wire ID를 추정하지 않는다.

| snapshot | wire object 목록 | 의미 |
|---|---|---|
| mock-snapshot-001 | [1, 2] | 작업 전 |
| mock-snapshot-002 | [2] | 첫 수거 후 |
| mock-snapshot-003 | [] | 두 번째 수거 후 완료 제안 |
| mock-snapshot-004 | [] | 추가 빈 관찰로 완료 확인 |

수락 → Nav2 도착·정지 → 기존 BT 검증 → task/execution UUID 발급·저장 → 같은 UUID의
ROS Goal → 최종 Result 검증·저장 → 검증된 SUCCESS만 논리적 물체 제거 → 재관찰 순서다.
Gazebo가 유일한 simulator다. Mock 서버는 clock·TF·odom·카메라·joint_states를 발행하지 않으며
Mission과 Mock 서버의 steady-clock 처리를 유지한다. 물리적 팔·물체 애니메이션은 없다.
실행 출처는 navigation=sim, perception/planning/execution=mock이다.

공유 fixture는 관찰용으로 정상 단계에 총 40초/물체를 배정한다. 준비·검증보다
접근·운반·팔 복귀가 오래 걸리도록 설정하며 세부 단계는 최소 1초 유지한다.
실물 측정 시간은 아니며 [시간표](../README.md#mock-조작의-관찰-시간)는 구현 README에서 관리한다.
전역 `stage_durations_sec`보다 scenario별 시간이 우선하므로 고장·timeout fixture도 유지된다.
ROS 계약 회귀는 시간을 단축한 동일 논리 fixture, Gazebo 검증은 기본 관찰 시간을 사용한다.

## 실패·취소·재시작

ROS SUCCEEDED의 BLOCKED payload를 성공으로 처리하지 않는다. SUCCESS에는 execution ID,
mock 출처, placement 확인, 팔 복귀와 정지를 모두 요구한다. HELD/UNKNOWN, 팔 미복귀,
FATAL·정지 미확인은 기존 주행·미션 수락·오류 해제 차단 조건을 유지한다.
map_outcome과 safe_to_drive는 변경하지 않고 execution_profile=mujoco도 허용하지 않는다.

Mission 취소는 현재 UUID의 정확한 ROS Goal을 취소하며 서버 기본 CHECKPOINT를 사용한다.
취소 ACK는 정지 증거가 아니다. 잡고 있는 물체가 남으면 CANCELLED 보고와 ERROR·사람 확인
필요 상태를 보존한다. IMMEDIATE/RETURN_ARM을 선택하는 Mission API는 추가하지 않는다.

전송 전 journal 실패에서는 Action을 보내지 않는다. 거절·지연 수락·응답 유실은 같은 ID로
조회·취소하며 자동 재전송하지 않는다. found=false는 정지나 미실행 증거가 아니다.
미완료 미션의 재시작은 INTERRUPTED이며 자동 재개하지 않는다. 기존 위험 상태를 초기
Mock 안전 설정이나 로컬 Mock 전환으로 덮어쓰지 않는다. 두 DB를 분리하고 재시작 시 보존한다.
서버 추가 JSON 필드는 기본값으로 과거 기록을 읽고, 읽을 수 없는 기록은 오류로 처리한다.
DB 초기화로 계속 실행하는 자동 복구는 없다.

## 검증 순서

1. `make test-mission-core test-manipulation-core`: 기존 51개 회귀와 core·보고·journal 호환 검사.
2. `make test-mission-mock`: 공통 ROS 타입, 두 물체·네 관찰, BLOCKED, HELD 취소,
   응답 유실·동일 ID 조회, 양쪽 DB 재시작, Feedback과 debug 오류, 시작 전 설정 오류 검사.
3. `make test-gazebo-manipulation`: study-cafe 지도·seat-12로 주행하고 Gateway 없이
   mission/offer 요청. clock 단일 발행, 물리 토픽의 발행 주체, 도착·정지와 기존 보고 검증.
4. home 복귀는 CLEANY_GAZEBO_POST_MISSION=return_home으로 별도 주행 시험.
   Bridge core의 기존 payload 검사를 유지한다. 통합 ROS 검사는 실제 Bridge 프로세스와
   로컬 WebSocket Gateway fixture로 offer·ACK·terminal payload도 확인한다.
   운영 Backend·Dashboard 연결은 해당 외부 환경에서 별도 확인한다.

Mock 성공은 논리적 수거의 증거이며 실제 물리 청소·실물 팔 연결의 검증이 아니다.

## 2026-10-14 검증 결과

| 검사 | 결과 |
|---|---|
| 기존 Mission·Bridge core 회귀 | 변경 전 51개 모두 통과 |
| 초기 통합 test-mission-mock (0.5초 설정) | 199 passed, 1 skipped; Gazebo 시험은 별도 실행 |
| 공통 install 빌드 | Mock 11개 의존 패키지와 simulation 16개 패키지 빌드 성공 |
| 초기 Gazebo wait_for_next (0.5초 설정) | seat-12 주행·정지, 두 실행 UUID, 네 관찰, SUCCESS; 68.78초 |
| 초기 Gazebo return_home (0.5초 설정) | 두 물체 수거 후 home 복귀·정지, SUCCESS와 return_result=OK; 103.36초 |
| 실행 graph | clock publisher 1개, MuJoCo 없음, Mock 서버 물리 토픽 미발행 |
| 구조·보고 호환 | runtime.py, runtime_models.py, journal.py와 Bridge 코드는 98e9f6c와 동일 |

### 2026-10-15 관찰 시간 조정 검증

전역 단계 시간을 추가해 공유 fixture의 정상 실행은 40초/물체로 변경했다.
기존 단계·세부 단계 순서를 유지하고 scenario별 timeout/failure 시간은 전역 값보다 우선한다.
실물 측정값은 아니며 팔 이동을 확인보다 오래 유지하는 관찰용 설정이다.

| 검사 | 결과 |
|---|---|
| 조작 core | 76 passed; 모든 세부 단계의 관찰 시간, 정상 성공과 HELD 취소 확인 |
| ROS 통합 회귀 | 208 passed, 1 skipped; Gazebo는 별도 실행 |
| 웹 관측 회귀 | 2 passed; 단계 갱신, 기존 BT 노드 유지와 오래된 정보 표시 확인 |
| Gazebo wait_for_next | 1 passed, 139.43초; 주행 후 두 물체·네 관찰·SUCCESS |
| Mock 실행 측정 시간 | 첫 물체 40.779초, 두 번째 40.777초; 각 17개 세부 단계 완료 |
| 배포된 웹 | 청소 BT의 조작 단계 관측 표시 확인, page error 없음 |

이번 실행의 DB·launch 로그·웹 화면과 `verification.json`은
`cleany-data/runs/manipulation-timing-20261015-XwwCUn/`에 보존했다.

ROS 검사는 BLOCKED·불일치 결과, HELD CHECKPOINT 취소의 home·수락·reset 차단,
Goal 기록 실패의 미전송, 응답 유실의 동일 ID 조회·취소, 양쪽 DB를 보존한 재시작,
이전·종료 실행 Feedback 무시, debug 오류 격리와 로컬 WebSocket Gateway payload까지 포함한다.
서버 과거 JSON 기록의 기본값과 손상된 DB의 오류·보존도 검사했다.

VM에 빠진 Nav2·MoveIt perception dependency는 시스템을 수정하지 않고 사용자 prefix로
준비해 검증했다. 이 VM의 재실행 환경은
zsh에서는 `/home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/activate.zsh`,
Bash에서는 같은 경로의 `activate.bash`를 source한다. 두 파일은 현재 셸이 다르면 해당
셸용 파일로 전달하므로 기존 activate.bash 명령도 zsh에서 사용할 수 있다.
Cleany 서버·Runtime·interfaces는 모두
`/home/ubuntu/Documents/cleany-data/builds/cleany-mission-manipulation-mock/install` 하나에서 읽는다.
표준 새 환경의 dependency 설치는 [개발환경 문서](../../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

기본 DiffDrive profile은 목표 근처 12cm에서 멈추어 도착을 통과하지 못했다. 통합 구성은
기존 mecanum controller와 monitor에 맞춘 costmap footprint를 사용한다. 초기 world 좌표와
saved-map 좌표를 혼동하지 않으며 Mission gate를 완화하지 않았다.
검증 범위는 Humble/Fortress, study-cafe의 seat-12와 지정한 home이다.
다른 좌석·지도, 운영 Backend·Dashboard와 실물 팔은 이 결과의 범위에 포함하지 않는다.
