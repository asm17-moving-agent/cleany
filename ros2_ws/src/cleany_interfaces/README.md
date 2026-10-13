# cleany_interfaces

`ObservedObjectGeometry`는 `/grasp/collision_geometry`의 선택적 관측 형상
인터페이스다. header의 capture stamp/frame, snapshot_id, object_id로 기존
GraspCandidate와 연관하고, mesh_pose는 header frame 기준이며 Mesh vertices는
mesh_pose의 local 좌표다. convex footprint를 인식된 지지면까지 돌출한 형상으로,
실제 숨은 형상이나 시뮬레이터 정답을 의미하지 않는다. 기존 InspectScene,
PlanGrasp, GraspCandidate의 필드를 바꾸지 않아 기존 CDR 기록 형식은 유지한다.
메시 타입 의존성에 shape_msgs가 포함된다.

`ObserveWristTarget.srv`는 RGB-only 손목 인계/확인 서비스다.
HANDOFF는 head source ID, 원본 label/confidence와 예상 base-frame OBB를 전달받아
손목 영상에서 segmentation reference를 만든다. 별도 손목 detection은 선택 옵션이다.
CHECK는 같은 arm/source/reference의 새 RGB mask를 반환한다. CLEAR는 참조를 해제한다.
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

- [Mobile base](docs/mobile_base.md): `/cmd_vel` 차체 속도 명령

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
공개 이름과 취소·복구 규칙은 [Mission Manager README](../cleany_mission_manager/README.md)를 따릅니다.


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

`DetectedObject2D`는 모델이 반환한 `sorting_category`와 `sorting_reason`을
포함한다. category는 `trash`, `lost_item`, `review` 또는 미제공 빈 문자열이다.
빈 값은 이름 기반 폐기 허가가 아니다. 이 메시지 변경 후 모든 consumer를 재빌드한다.

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Context](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)

## 단일 물체 Manipulation action

`ExecuteManipulationSkill`은 승인된 물체 하나를 목적지로 옮기는 실행 계약입니다.
요청은 mission/task/execution ID, skill, snapshot ID, snapshot 범위의 uint32 object ID와
목적지 ID를 포함합니다. execution ID는 호출자가 발급하고 전송 전에 영속 저장합니다.
결과는 status/error, 실패·완료 stage, 물체·placement 상태, 선택한 팔,
정지 확인, 팔 복귀, retryable과 설명을 포함합니다. feedback stage는 완료 증거가 아닙니다.

`ManipulationExecutionRecord`는 동일 요청 및 영속 진행/결과와 revision, timestamp,
사람 확인 필요 여부를 전달합니다. `GetManipulationExecution`은 execution ID로 이 기록을
조회합니다. 현재 `found=false`는 서버 저장 오류와도 구분되지 않으므로 실행되지 않았다는
증거로 사용하지 않습니다. 상세 수락·성공·취소와 주행 gate는
[Mission Manager 계약](../cleany_mission_manager/README.md#단일-물체-manipulation-연결)을 따릅니다.

이 세 interface는 팀원 브랜치 `feat/manipulation-action-server`의
`46efa012cddb4ef0ff3f4dadb785550591bb6073`과 동일합니다. 서버 내부 BT 구조는 이 계약에 포함하지 않습니다.
