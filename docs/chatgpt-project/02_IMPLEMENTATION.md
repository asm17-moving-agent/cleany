# Cleany — 시스템 구조와 현재 구현

기준일: 2026-10-09. 로봇 구현 브랜치 `feat/runtime-integration`, 소스 기준 `c082d1d`, KB commit `9931007`. 초기 문서 기준 `16504ee`에 runtime cleanup 6개 커밋과 Nav2 BT 추적·설정형 기록 기능을 통합했다. 10/09 accuracy review·SCRUM-426 도구·ROS 그래프 개선·navigation 회귀 도구·공유 voxel 변경도 통합했다. 기존 주행 기록과 이번 통합 검사를 구분한다.

## 목표 구조와 실제 연결

```text
운영자 → 운영 Dashboard → HTTP 미션 요청 → Backend Queue
                                    ↕ SSE 상태·결과
Backend ↔ Gateway WebSocket ↔ Control Bridge ↔ Mission Runtime
                                                    ├─ Navigator → Nav2 → 안전 체인 → base
                                                    └─ 책상 작업 BT → 관찰 → 제안 → 실행 → 재관찰
센서·TF·미션 이벤트 → 읽기 전용 개발 모니터 → 기록·재생
```

제품 목표에서는 실제 Perception·Task Planner·Manipulation Skill이 책상 작업 BT에 연결된다. 현재 Runtime node는 `MockDesk`의 perception/planner를 연결한다. 별도 로봇 기능이 구현돼 있어도 Runtime에 연결됐다고 자동으로 간주하지 않는다.

| 구성 | 책임 | 현재 구현 상태 |
|---|---|---|
| 운영 Dashboard | 좌석 요청·취소, 진행·결과 표시 | 별도 React control-plane prototype |
| Backend | Queue·배차·외부 상태·결과 보존 | FastAPI/Pydantic·SQLite·SSE·Gateway 구현 |
| `cleany_control_bridge` | WebSocket과 ROS service 연결 | ACK·재동기화·영속 outbox 구현. 직접 주행 명령 없음 |
| `cleany_mission_manager` | 수락·상태 전이·취소·결과 | 비동기 Runtime FSM와 py_trees 책상 BT, journal 구현 |
| `cleany_navigation` | localization·planner/controller·안전 체인 | Nav2와 custom 전방향 경로/후보·센서 guards 구현 |
| `cleany_base_driver`, `cleany_base_odometry`, ESP32 | 명령→wheel, feedback→odom | ROS/micro-ROS/MCU 구현 및 무부하 실측 기록 |
| `cleany_grasping` | 점군에서 grasp 후보 생성·필터 | geometric 기본 predictor와 AnyGrasp adapter 구현 |
| `cleany_skill_executor` | 후보의 팔 도달성·충돌·경로 검증 | MoveIt plan-only action, 별도 MuJoCo 접근 데모 |
| `cleany_perception`, `cleany_planner`, `cleany_robot_interface` | 목표 인지·계획·공통 backend 경계 | 디렉터리 README만 있는 scaffold, ROS package/node 구현 없음 |
| `cleany_description`, `cleany_moveit_config` | 차체·joint/frame·팔 planning | CAD/Xacro·MoveIt 설정 구현 |
| `cleany_gazebo_sim`, `cleany_mujoco_sim` | 주행 / tabletop 조작 검증 | 역할이 다른 두 simulator의 구현 |
| `cleany_handeye_calibration` | wrist camera 외부변환 실험 | ChArUco·PnP·MoveIt 피드백/영상 시각 기반 기록·solver 구현 |
| `cleany_telemetry` | 지도용 robot pose relay | 별도 pose WebSocket. 물리 정확도 측정과 구분 |
| `cleany_dev_monitor` | 개발 관측·SQLite 기록·재생 | ROS 그래프·카메라·로그·주행 5개 탭. 읽기 전용 |

## 미션 상태와 책상 작업

현재 Runtime enum은 `IDLE`, `NAVIGATE_TO_TARGET`, `WORKING`, `POST_MISSION`, `RETURN_HOME`, `CANCELLING`, `ERROR`다. 정상 흐름은 IDLE→이동→WORKING→후속 정책→복귀 또는 대기→결과/IDLE이다. 사용하지 않던 core enum의 PERCEIVE/PLAN_TASKS/EXECUTE_TASKS/REPORT와 구 manager/mock 경로는 cleanup에서 제거됐다.

WORKING 안의 BT는 다음처럼 구성된다.

```text
ObserveBefore
→ 반복: PlanNext → ValidateProposal → ExecuteOne → Reobserve → VerifyCompletion
→ 검증된 완료
```

고수준 행동은 `collect_trash`, `skip`, `human_review`, `done`만 허용한다. proposal이 현재 snapshot을 참조하는지, object가 존재하는지, collect 대상인지, destination allowlist와 numeric object ID가 맞는지 확인한다. `store_lost_item`은 현재 Runtime allowlist에 없다.

각 실제 실행 요청에는 mission/task/execution ID를 붙인다. 새 Scene과 실행 결과로 다음 행동을 판단하고, 완료 제안만으로 성공을 만들지 않는다. typed SceneSnapshot·TaskProposal·ActionRecord·RuntimeReport가 있다. 현재 Scene 계약은 ID·관찰 시각·reference·객체 disposition 등을 가진 작은 mock 중심 모델이며 KB가 지향하는 mask·3D 품질까지 완성된 인지 계약은 아니다.

설정 파일 기준 deadline은 이동 180초·책상 작업 180초·단일 operation 30초·취소 5초, 최대 행동 30개·skill retry 2회다. 일부 시설 실험은 leg별 360초 override를 사용했다. 시험 설정과 기본 설정을 구분한다. deadline과 tick은 steady/monotonic 시간에 의존한다.

`post_mission=return_home|wait_for_next`가 있다. 구현 옵션을 제품 복귀 정책의 새 결정으로 해석하지 않는다. 현재 Runtime은 navigation을 `sim|mock`, manipulation을 `mock|action_mock`로 제한한다. action_mock도 mock 서버 계약이며 실물 수거 backend가 아니다.

### 수락·종료·복구

하나의 활성 미션만 수락한다. 대상·미션 타입·모듈 준비·정지 상태를 확인하고 동일 mission ID는 새 실행으로 처리하지 않는다. Nav2 action server뿐 아니라 map_server/amcl/planner/controller/bt_navigator/collision_monitor active를 확인한다.

Nav2 성공 뒤 fresh stopped wheel odometry와 요청 원본 pose에 대한 fresh map→base TF를 검사한다. Runtime 기본 도착 허용값은 위치 0.08m·yaw 0.15rad, pose age 0.5초다. 이는 localization 추정 pose 기준이며 실제 책상·팔 도달성이나 GT 정확도 보증이 아니다.

취소 ACK만으로 IDLE을 만들지 않는다. Nav2 terminal과 정지를 확인하며 확인 실패는 ERROR로 남긴다. SQLite journal에 수락·goal UUID·실행 출처·결과를 저장한다. 활성 미션 도중 재시작은 INTERRUPTED·사람 확인 대상으로 보고하고 자동 재개하지 않는다.

manipulation action 경계는 요청을 전송 전에 기록한다. 결과에는 object state, placement state, stop_confirmed, arm_recovered 등이 있으며 팔 정지·회복과 물체 상태가 안전해야 다시 base를 움직일 수 있다. 수락/종료가 불명확하면 성공이나 정지를 합성하지 않는다.

## Backend/Gateway 계약

현재 확인한 control-plane은 HIGH 우선·동순위 FIFO, 비선점 단일 슬롯의 Queue다. Dashboard는 HTTP로 요청하고 SSE로 갱신 알림을 받는다. SSE 재연결 때 HTTP snapshot을 다시 읽는다. Backend는 로봇 내부 FSM·motor 명령·물리 안전 해제를 소유하지 않는다.

Gateway는 schema v1 WebSocket으로 연결한다. envelope에는 event ID, robot ID, mission ID, sequence, event type, occurred_at과 payload가 있다. Backend의 `clean_seat` offer를 Bridge가 ROS의 `clean_desk`/`SEAT` 요청으로 변환하고 canonical 좌석 ID를 보존한다. 화면 pixel 좌표를 주행 목표로 역산하지 않는다.

- 연결 후 fresh snapshot과 ACK를 먼저 교환한다. 그 뒤 해당 연결의 명령·피드백을 처리한다.
- 명령 ACK는 수신 확인이다. 실행 수락은 `mission.accepted`, 취소 완료는 최종 CANCELLED 결과로 판단한다.
- `mission.phase`는 NAVIGATING/WORKING/RETURNING 등의 외부 의미다. 내부 BT/FSM를 모두 운영 상태로 노출하지 않는다.
- RPC/offer timeout은 미수락의 증거가 아니다. 재동기화하며 불명확한 실행 슬롯을 임의로 비우지 않는다.
- SQLite inbox/outbox·event ID·미션별 증가 sequence로 중복·재전송을 다룬다. terminal 결과는 DB 제약으로 불변이고 활성 미션은 unique index로 하나만 허용한다.
- 최종 결과와 fresh 가용 상태를 모두 확인한 뒤 다음 미션을 배차한다. Backend 연결이 끊겨도 수락된 로컬 미션은 계속한다.
- mock 관찰 reference는 사진이 아니므로 외부 before/after 자료에는 null을 보낸다. 실행 출처는 navigation/perception/planning/execution별 sim/real/mock로 표시한다.

## Navigation·센서·TF

사전 지도에서 AMCL이 위치를 추정하고 Nav2가 경로·속도를 만든다. 전방향 profile은 lateral/reverse 이동을 허용하되 padded footprint의 연속 sweep과 충돌 검사를 거친다. GoalDirectedController·DWB 후보·방향 연속성 비용으로 반복 정지를 줄이는 구현이 있다.

공통 실제 시간 entrypoint는 serial 안전 모드다. pan 정렬→settle→fresh depth 확인과 2D Collision Monitor·3D voxel guard를 연결한다. Gazebo wrapper의 reactive 모드는 Nav2→Collision Monitor→reactive_guard→cmd_vel로, pan 관찰을 주행 허가와 분리한다. reactive 실제 시간 실행은 거부하며 현재 실물 기본으로 간주하지 않는다.

reactive guard는 원본 sensor stamp의 TF, 현재 LiDAR/depth, 요청·측정 속도 및 센서 나이를 포함한 제동 경로를 검사한다. 센서/명령/TF 단절·계산 초과는 정지, 시계 역행/localization jump는 재시작까지 정지한다. 전체 depth coverage 진단은 shadow 전용으로 주행 권한이 없다.

`/cmd_vel`은 base_link 기준 x/y m/s·yaw rad/s다. `/wheel/odom`과 `/ground_truth/odom`은 서로 다른 정보다. TF 의미는 map→odom(localization), odom→base_link(활성 odometry backend), base→sensor/arm(robot description)이며 각 transform은 단일 소유자가 발행한다. Gazebo에서는 sim relay, 실물/mock base에서는 base_odom_relay가 canonical odom/TF를 소유한다.

모니터의 GT pose relay는 화면 표시용이고 AMCL 정확도를 입증하지 않는다. 센서 레이어는 메시지 시각 TF가 없으면 표시하지 않으며 최신 TF로 대체해 정상처럼 보이지 않는다.

## Grasp·조작과 AI의 실제 경계

geometric grasp predictor는 단순 강체의 target/context 점군에서 지지면·축·폭을 추정해 후보를 만들고 context 충돌·접촉·폭·NMS를 검사한다. 후보가 없으면 오류를 반환하고 가짜 pose를 만들지 않는다. AnyGrasp SDK/checkpoint/license는 외부 입력이며 Jetson 호환성을 이번에 실행 확인하지 않았다.

`grasp/select_reachable` action은 점수순 후보·가까운 팔부터 position-only IK, state validity, 현재→pregrasp→grasp plan을 검사한다. 운영 action은 trajectory/gripper를 실행하지 않는다. Planning Scene의 임시 target/ACM 변경을 복원한 뒤 결과를 확정한다.

별도 MuJoCo 데모는 선택된 trajectory를 controller로 접근 실행한다. can 데모는 시뮬 RGB-D에서 빨간 can을 분할하고 geometric 후보·MoveIt을 거쳐 gripper를 열고 방향을 맞춘 pregrasp까지 이동한다. close/attach/lift/운반/투입은 포함하지 않는다. 5축 position-only IK의 방향 한계를 보완하는 can 데모용 가상 조준점이 있으며 일반 action의 전체 TCP 방향 검증으로 확대하지 않는다.

AI 후보는 로컬 VLM/API VLM(Gemini Robotics ER 2 포함), YOLO+SAM 또는 YOLO segmentation이며 최종 채택은 미정이다. VLM은 고수준 의미·순서 제안, detector/mask/depth는 Scene, VLA는 Skill 내부 물리 action 후보라는 역할 구분을 유지한다. 현재 코드의 mock Planner를 실전 VLM·VLA 구현으로 부르지 않는다.

## 개발 모니터의 BT 추적·기록

Nav2가 실제 사용하는 최종 BT XML의 노드 인스턴스와 포트, 관측된 상태 전이·실패를 트리로 표시한다. `/navigation/bt_trace`에 XML SHA와 goal 문맥을 연결한다. 같은 이름의 노드도 인스턴스를 구분하며, 관측되지 않은 상태를 실행 완료로 만들지 않는다.

기록은 범주별 활성화·rate·일시정지·새 session과 export/download를 지원한다. 선택한 ROS 메시지의 raw CDR을 SQLite에 저장해 원본 stamp와 TF를 보존한다. 카메라 등 대용량 항목은 설정에 따라 기록한다. 모니터는 로봇 동작을 지시하지 않으며 기록 설정 제어만 제공한다.

cleanup은 미사용 FSM/mock/world 생성 경로를 제거하고 geometry·transport·mission·monitor·sim/base 공통 처리를 묶었다. shutdown·sticky error·실패 전달을 보강했으며 안전 기준이나 도착 허용치를 낮추지 않았다.

## 2026-10-09 추가 통합

공유 voxel 경로는 obstacle memory의 주행용 높이 투영을 local/global costmap에 연결하고 serial 3D guard에 로봇 주변 점군을 제공한다. LiDAR·정적 지도·Collision Monitor를 유지하며 중복 depth VoxelLayer를 대체한다. 관측 만료는 장애물을 빈 공간으로 바꾸지 않고 미확인으로 보존한다. 재시작의 과거 관측도 미확인으로 복원하며 ROS clock 역행은 mapping을 차단한다. sparse 점유 index 대신 voxel별 timestamp key를 투영에 재사용해 전체 volume 스캔을 피한다.

ideal quantized wheel 시뮬용 AMCL 보정 profile과 controller의 3cm 목표 허용 오차를 통합했다. Mission의 원래 목표 8cm 검사와 안전·freshness·deadline은 유지한다. 실물 보정값으로 해석하지 않는다. 이전 navigation 회귀 브랜치의 Twirling 64/RotateToGoal 128은 현재 critic 설정으로 덮어쓰지 않았다. 회귀 실행 도구를 통합했지만 새 전체 주행 결과는 별도 시험이 필요하다.

SCRUM-426 도구는 격리 Gazebo instance·좌석 inventory·병렬 처리 capacity·전체 survey coverage·후속 자극 시험 및 실측 대표 좌석 선정을 제공한다. 원격 survey/후속 시험이 진행 중이며 통합 자체가 전체 좌석 검증 완료를 뜻하지 않는다.
