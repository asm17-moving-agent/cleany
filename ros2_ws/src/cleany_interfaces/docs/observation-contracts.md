# cleany_interfaces

`ObservedObjectGeometry`는 `/grasp/collision_geometry`의 선택적 관측 형상
인터페이스다. header의 capture stamp/frame, snapshot_id, object_id로 기존
GraspCandidate와 연관하고, mesh_pose는 header frame 기준이며 Mesh vertices는
mesh_pose의 local 좌표다. convex footprint를 인식된 지지면까지 돌출한 형상으로,
실제 숨은 형상이나 시뮬레이터 정답을 의미하지 않는다. 별도 mesh 메시지이며,
다른 인터페이스의 필드 변경과 호환성은 각각의 계약을 확인한다.
메시 타입 의존성에 shape_msgs가 포함된다.

`ObserveWristTarget.srv`는 RGB-only 손목 인계/확인 서비스다.
HANDOFF는 head source ID, 최종 label, 원본 detector_label/confidence와 예상 base-frame OBB를 전달받아
손목 영상에서 segmentation reference를 만든다. 별도 손목 detection은 선택 옵션이다.
CHECK는 같은 arm/source/reference의 새 RGB mask를 반환한다. 최종 종류명이나 원본
검출 클래스가 HANDOFF 때와 다르면 거절한다. CLEAR는 참조를 해제한다.
`expected_pose`는 입력 추정치일 뿐 측정값이 아니며 응답은 촬영 header와 mask만
포함한다. 손목에서 측정한 depth/3D 높이를 주장하지 않는다.
HANDOFF의 `expected_pose.header.stamp`는 원본 head 관측 시각이다.
`after_stamp_ns`보다 새로운 손목 image/CameraInfo 쌍만 처리한다.

응답 `status`는 `ERROR=0`, `OK=1`, `NOT_DETECTED=2`를 구분한다.
`OK`만 `success=true`다. `NOT_DETECTED`는 정상적인 새 RGB/CameraInfo와 TF로
투영을 확인했지만 대응 물체가 검출되지 않은 경우다. CHECK 미검출에도 기존
reference/source ID와 실제 촬영 header를 반환하며 mask는 비어 있다.
호출자는 이 촬영 시각 이후의 프레임으로 재시도할 수 있다. 센서·TF·추론 오류,
참조 불일치, 중복 후보와 잘못된 mask는 `ERROR`이며 미검출로 취급하지 않는다.
HANDOFF 미검출은 파지 진행을 허용하는 참조를 만들지 않는다.
이 status 필드 추가로 wire/CDR 형식이 바뀌므로 제공자와 호출자를 함께 재빌드하고
재시작해야 한다. 이전 ObserveWristTarget CDR 기록은 이전 인터페이스로 읽는다.

## VerifyPlacement

`srv/VerifyPlacement.srv`는 label, destination_id, release 이후 기준
`after_stamp_ns`를 받아 success/message로 실제 놓기 결과를 응답한다.
그리퍼 열기 명령의 성공은 놓기 성공이 아니다. 현재 제공자는 시뮬레이션
평가용 oracle이며 조작 코드에는 물체 정답 pose를 반환하지 않는다.
실제 로봇에는 독립 센서 기반 검증 제공자가 필요하다.

Cleany perception snapshot, grasp candidate 생성과 MoveIt 도달 가능성 검증이
공유하는 ROS 2 interface package다. 구현 내부 model이나 provider별 응답은 wire
contract로 노출하지 않는다. 커스텀 메시지뿐 아니라 표준 ROS 메시지를 사용하는
프로젝트 공통 topic 계약도 이곳에 기록한다.

## 객체 메시지

`DetectedObject2D`와 `DetectedObject2DArray`는 Gemini detector가 반환한 RGB pixel
bounding box, snapshot-local 번호와 후속 선택 요청에 사용할 `snapshot_id`를 표현한다.
각 detection은 촬영 시점 depth와 TF로 계산한 configured target-frame 원점 기준
`distance_m`과 유효 여부를 포함한다. 유효한 후보는 거리, confidence, detector 원본
순서로 정렬되며 depth가 불충분한 후보는 자동 조작 대상으로 선택하지 않는다.
`DetectedObject3D`와 `DetectedObject3DArray`는 선택 객체의 OBB와 동일 snapshot 문맥을
표현한다.

| 필드 | 계약 |
|---|---|
| `DetectedObject2D/3D.label` | 최종 종류명. `yoloe_gemini`에서는 Gemini 이름을 정규화하며 YOLOE 이름을 접두어로 붙이지 않음 |
| `DetectedObject2D/3D.detector_label` | 원본 검출 클래스. 검출 재매칭용이며 미제공은 빈 문자열 |
| `confidence` | 검출기의 신뢰도. Gemini 종류명과 처리 분류의 확률이 아님 |
| `ObserveObjectReference.Response.source_label / source_detector_label` | PIN 당시의 최종 종류명과 원본 검출 클래스. 원래 snapshot이 cache에서 제거돼도 참조 TTL 동안 보존 |
| `ObserveWristTarget.Request.label / detector_label` | 최종 종류명과 손목 재검증에 사용할 원본 클래스. detector_label이 비어 있으면 label 사용 |

추적 ID가 유지돼도 최종 종류나 `sorting_category`가 바뀌면 기존 사용자 승인을
그대로 실행하지 않는다. 단순 동의어는 인식 노드의 `semantic_label_aliases`에서
정규화한다. 이름 기반 제외 목록과 목적지 설정은 최종 종류명을 사용한다.

원본 검출 클래스 필드 추가로 이 메시지를 포함한 Action, Service와 topic의
wire/CDR 형식도 바뀐다. native와 perception, grasp, motion 컨테이너의 인터페이스를
함께 재빌드한 뒤 같은 버전으로 실행해야 한다. 이전 CDR 및 rosbag은 기록 당시의
인터페이스로 읽는다. TensorRT 모델의 클래스와 출력은 바꾸지 않아 재변환하지 않는다.

`DetectedObject2D`에서 분류 사유 필드를 제거했다. Gemini 설명 문장을 요청하거나
수거 허가의 조건으로 사용하지 않는다. 이 제거도 CDR 변경이므로 위 재빌드 규칙을
따른다. 보존된 이전 관측과 rosbag은 기록 당시 인터페이스로 읽는다.

## Scene inspection action

`InspectScene` 1차 단계는 동기화된 RGB-D snapshot에서 2D detection만 수행하고,
2차 단계는 `snapshot_id`와 `selected_object_id`로 선택한 객체 하나의 YOLOE instance mask로 3D
복원한다. 성공한 2차 결과는 같은 configured target frame과 capture timestamp의
`target_cloud`, `context_cloud`와 선택 객체 OBB를 반환한다.

## Grasp planning과 선택

`PlanGrasp`는 score 내림차순 `GraspCandidate[] candidates`를 반환한다.
`SelectReachableGrasp` action은 같은 snapshot/object/frame/OBB 후보를 받아 양팔 IK,
state validity, 2구간 plan-only 검증 후 선택 index, arm, endpoint joint state를 반환한다.
trajectory는 현재 RobotState에 종속되므로 result에 포함하지 않는다.
`required_arm`은 `left`, `right` 또는 빈 문자열이며, 특정 팔을 유지해야 하는 재인식
단계에서는 해당 팔만 평가한다. 빈 문자열은 기존의 가까운 팔 우선 양팔 fallback을
그대로 사용하고 그 외 값은 `ERROR_INVALID_INPUT`이다.

팔 선택, IK, robot collision과 trajectory 계획은 `SelectReachableGrasp` 경계이며,
gripper 명령과 실제 trajectory 실행은 별도 Skill Executor coordinator가 담당한다.
초기 `nearest_pregrasp_coordinator`는 `DetectedObject2D.distance_valid`와 `distance_m`을
사용해 가까운 객체부터 시도한다.

## Contracts

- [Mobile base](../docs/mobile_base.md): `/cmd_vel` 차체 속도 명령

## Mission Runtime 계약

`OfferMission`은 `mission_id`, `mission_type=clean_desk`, `target_kind=SEAT`,
canonical `target_id`와 `requested_by`를 전달합니다. 응답의 `accepted`, `duplicate`,
`reason`과 선택적 `report_json`은 수락 여부이며 작업 완료는 아닙니다.
`CancelMission`도 취소 수락 응답이며 실제 종료는 최종 결과로 확인합니다.
`GetRuntimeSnapshot`은 현재 상태와 보존된 결과를 `snapshot_json`으로 반환합니다.

`MissionStatus`는 FSM 상태, 외부 phase, BT 단계, 준비 여부 및 진단 sequence를 제공합니다.
이 sequence는 Backend wire의 미션별 sequence와 구분합니다. `MissionResult`는 outcome,
실패 코드, 완료·skip 작업, 사람 확인 여부, 관측 참조, 주행·복귀 결과와 전체 `report_json`을
제공합니다. 내부 Mock 관측 ID는 실제 사진을 의미하지 않습니다.
공개 이름과 취소·복구 규칙은 [Mission Manager README](mission.md)를 따릅니다.


## 개발 관측 메시지

`DebugBTNode`는 노드 ID, 부모, 종류, 상태, 이번 tick 방문 여부와 operation ID를 전달한다. `RuntimeDebugSnapshot`은 header, boot/event sequence, mission/state/ready/reason, BT 노드와 versioned `snapshot_json`을 전달한다. `RuntimeEvent`는 boot/sequence, mission/task/execution 식별자와 kind, `data_json`을 전달한다.

기본 토픽은 `/mission/debug_snapshot`과 `/mission/runtime_events`다. 메시지는 명령 의미를 갖지 않는다. 이 패키지의 빌드만으로 producer가 실행되지는 않으며 발행이 없는 환경에서는 미수신으로 표시한다.

## 설정 및 검증

인터페이스를 추가하거나 변경할 때는 해당 의존 패키지, `package.xml`, 빌드 및 메시지
호환성 검증을 함께 갱신한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select cleany_interfaces
source install/setup.bash
colcon test --packages-select cleany_interfaces
colcon test-result --verbose
```

설치된 계약은 `ros2 interface show`로 `InspectScene`, `PlanGrasp`,
`SelectReachableGrasp`와 관련 message를 확인한다.

## 관련 KB

`DetectedObject2D`는 모델이 반환한 `sorting_category`를 포함한다.
category는 `trash`, `lost_item`, `review` 또는 미제공 빈 문자열이다.
빈 값은 이름 기반 폐기 허가가 아니다. 이 메시지 변경 후 모든 consumer를 재빌드한다.

- [Technical Overview](../../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Context](../../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [ROS 2 Software Architecture](../../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)

## 단일 물체 Manipulation action

실행·취소·조회 명령은 [Skill Executor 실행 안내](../../cleany_skill_executor/docs/manipulation_mock_usage.md)를
따른다. 이 절은 ROS 필드와 종료·기록 상태의 의미를 정리한다.

### 공개 인터페이스

| 타입 | 기본 ROS 이름 | 역할 |
|---|---|---|
| `ExecuteManipulationSkill.action` | `/mock/manipulation/execute_skill` | 승인된 `collect_trash` 또는 `collect_lost_item` 물체 하나의 요청·Feedback·Result·취소 |
| `GetManipulationExecution.srv` | `/mock/manipulation/get_execution` | execution_id를 받아 found와 최신 record 반환 |
| `CancelManipulation.srv` | `/mock/manipulation/cancel` | 활성 execution_id와 취소 mode를 받아 accepted와 message 반환 |
| `ManipulationExecutionRecord.msg` | `/mock/manipulation/execution_events` | 진행·종료·재시작 중단 이벤트 |

[MuJoCo BT 서버](../../cleany_manipulation_bt/README.md)는 같은 계약을 `/sim/manipulation`에서 제공한다.

이벤트 QoS는 Reliable·Transient Local, depth 100이다.
상태·단계·오류의 전체 목록은 [Action 명세](../../cleany_skill_executor/docs/02_execute_manipulation_skill_action_spec.md)를 따른다.

### Goal·Feedback·Result

| 구분 | 확인할 내용 |
|---|---|
| Goal | 호출자가 execution_id 발급. snapshot_id/object_id 조합으로 대상 지정 |
| Feedback | 큰 단계 `stage`, 세부 동작 `substage`, 선택 팔과 진행 설명 |
| `substage` | Feedback·기록의 현재 세부 동작. 큰 단계 `stage`의 취소·timeout 계약은 유지 |
| `completed_substages` | 기록에 저장된 완료 세부 동작 목록. 명령 완료와 물체 상태 확인을 구분 |
| `failed_substage` | Result·기록의 실패 세부 동작. 기존 기록에는 빈 값일 수 있음 |
| `cancel_mode` | Result·기록의 취소 정책. `IMMEDIATE`, `CHECKPOINT`, `RETURN_ARM`; 취소 요청이 없으면 빈 값 |
| Result | 종료 status, 오류, 물체·놓은 결과·팔 복귀·정지 근거 |
| `execution_profile` | 근거를 생성한 실행 환경. 모의 서버는 `mock`, BT MuJoCo 서버는 `mujoco` |

세부 진행 필드 추가 후 서버·클라이언트·모니터를 함께 재빌드하고 재시작한다.
이전 SQLite JSON 기록은 세부 진행 필드가 없어도 읽으며 빈 값으로 취급한다.

`CancelManipulation`은 기본 Action 취소를 내부에서 전달해 ROS 상태도 `CANCELING`으로
전환한다. 일반 Action cancel의 기본 정책은 `CHECKPOINT`다. 활성 실행 중 모드 선택은
멱등이며, 이미 요청한 모드를 변경하려면 `IMMEDIATE`로 선점한다. 즉시 정지는 이후
요청으로 완화되지 않는다. 잘못된 ID와 mode, 종료된 실행은 거절한다.

### 실행 기록 읽기

`ManipulationExecutionRecord`는 Goal, 최신 진행과 Result 필드를 함께 제공한다.

| 필드 | 의미 |
|---|---|
| `record_state` | 아래 표의 기록 상태 |
| `has_result` | 최종 Action Result 존재 여부 |
| `human_confirmation_required` | 물리 상태에 대한 사람 확인 필요 여부 |
| `revision` | 기록 갱신 버전 |
| `accepted_at_ns`, `updated_at_ns`, `evidence_at_ns` | 수락·갱신·관측 근거 시각. Unix nanoseconds |

| `record_state` | 해석 |
|---|---|
| `ACTIVE` | 진행 중 |
| `FINISHED` | 실행 종료와 최종 Result 확정. 디스크 저장 완료를 보장하지 않음 |
| `INTERRUPTED` | 재시작 시 발견한 미완료 실행. 가짜 Result를 만들지 않음 |
| `RECORDING_FAILED` | 이전 구현의 저장 실패 진단. 현재 서버는 새 기록에 사용하지 않음 |

`has_result=false`이면 status/error_code/failed_stage/retryable을 Result로 해석하지 않는다.
중단 기록도 마지막 물리 상태와 완료 단계를 보존하며 사람 확인 필요를 표시한다.
현재 서버의 조회는 메모리의 최신 기록을 반환한다. 실행 기록 저장 실패는 경고로 남기고
실제 실행의 status와 error_code를 유지하며, 조회 성공이 디스크 저장 완료를 보장하지 않는다.
단계 deadline은 기록 시각과 별개로 프로세스 단조 시계를 사용한다.

### 개발 관측 인터페이스

`RuntimeDebugSnapshot` (`mission/debug_snapshot`, reliable/transient-local depth 1)은
header, boot ID, 마지막 event sequence, mission ID, FSM state, ready/reason,
`DebugBTNode[]`, schema version 1의 `snapshot_json`을 전달한다. JSON에는 FSM 정의,
모듈별 ready/stopped/reason, 실행 profile, 현재 proposal, 조작 결과, navigation 진단이
포함된다. `DebugBTNode`는 id/parent/kind/status/visited/operation_id로 구성된다.

`RuntimeEvent` (`mission/runtime_events`, reliable/transient-local depth 2048)은
header, boot ID, 연속 sequence, mission/task/execution ID, kind, `data_json`을 전달한다.
kind는 transition, bt_tick, action_start, action_result, action_cancel_requested,
cancel_requested, mission_result다. 관측 데이터이며 명령 인터페이스가 아니다.
기존 `MissionStatus`, `MissionResult`, 미션 service 계약은 변경하지 않는다.

## 읽기 전용 snapshot 조회

`GetSceneSnapshot.srv`는 `/perception/get_scene_snapshot`에서 `snapshot_id`를 받아
`found`, `DetectedObject2DArray detections`, `message`를 반환한다. 기존 cache만 조회하고
TTL, 촬영 시각과 객체 번호를 갱신하지 않는다. 누락/만료는 `found=false`다.


## 관찰 간 추적

`InspectScene.Goal.tracking_session_id`는 같은 고정 베이스 책상 작업에 재사용한다.
2D 메시지의 `track_id`, `tracking_state`, `position_valid`, `representative_position`은
관찰 간 연결에 쓰이며 3D 대표 위치 frame은 배열의 `representative_frame`에 명시한다.
배열은 session/epoch와 마지막 참조를 보존한 `missing_objects`를 포함한다. 최초부터
불확실한 대상의 missing 참조에는 빈 track ID가 있을 수 있다. 선택 3D 결과에도 같은
track/session/epoch를 전달한다. RGB header frame·snapshot-local 번호와 구분한다.
세션을 비우면 추적을 사용하지 않는다. snapshot 조회·선택 복원은 저장된 ID를 유지한다.


## 개체별 배치 확인

`RegisterPlacementTarget.srv`는 `/sorting/register_placement_target`에서 execution ID,
목적지, 촬영 header와 관측 OBB·종류로 개체를 고정하고 불투명 `verification_id`를 반환한다.
`VerifyPlacement.verification_id`가 있으면 그 개체만 확인한다. 미등록·만료 ID는 실패하고
label로 fallback하지 않는다. 빈 ID의 label-only 경로는 종류당 물체 하나인 기존 장면에
유지한다. 종류 중복 장면은 등록 경로가 필요하다. 인터페이스 변경 후 consumer를 모두 재빌드한다.
