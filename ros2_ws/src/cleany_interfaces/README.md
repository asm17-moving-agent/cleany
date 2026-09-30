# cleany_interfaces

Perception, grasp 계획·선택과 Manipulation 실행이 공유하는 ROS 2 인터페이스 패키지다.
이 문서는 메시지·Action·Service와 프로젝트 공통 Topic 계약을 관리한다.

| 확인할 계약 | 바로가기 |
|---|---|
| 물체 하나의 수거 Action과 실행 기록 | [Manipulation](#manipulation-action과-실행-기록) |
| 선택 물체의 관측 형상 | [ObservedObjectGeometry](#관측-형상-observedobjectgeometry) |
| 손목 RGB 관측 | [ObserveWristTarget](#손목-관측-observewristtarget) |
| 놓은 결과 확인 | [VerifyPlacement](#verifyplacement) |
| 객체·snapshot 메시지 | [객체 메시지](#객체-메시지) |
| 장면 관찰과 파지 계획·선택 | [InspectScene](#scene-inspection-action), [Grasp](#grasp-planning과-선택) |
| 빌드·계약 검증 | [설정 및 검증](#설정-및-검증) |

## Manipulation Action과 실행 기록

실행·취소·조회 명령은 [Skill Executor 실행 안내](../cleany_skill_executor/docs/manipulation_mock_usage.md)를
따른다. 이 절은 ROS 필드와 종료·기록 상태의 의미를 정리한다.

### 공개 인터페이스

| 타입 | 기본 ROS 이름 | 역할 |
|---|---|---|
| `ExecuteManipulationSkill.action` | `/mock/manipulation/execute_skill` | 승인된 `collect_trash` 물체 하나의 요청·Feedback·Result·취소 |
| `GetManipulationExecution.srv` | `/mock/manipulation/get_execution` | execution_id를 받아 found와 최신 record 반환 |
| `ManipulationExecutionRecord.msg` | `/mock/manipulation/execution_events` | 진행·종료·재시작 중단 이벤트 |

이벤트 QoS는 Reliable·Transient Local, depth 100이다.
상태·단계·오류의 전체 목록은 [Action 명세](../cleany_skill_executor/docs/02_execute_manipulation_skill_action_spec.md)를 따른다.

### Goal·Feedback·Result

| 구분 | 확인할 내용 |
|---|---|
| Goal | 호출자가 execution_id 발급. snapshot_id/object_id 조합으로 대상 지정 |
| Feedback | 문자열 stage와 진행 설명 |
| Result | 종료 status, 오류, 물체·놓은 결과·팔 복귀·정지 근거 |
| `execution_profile` | 근거를 생성한 실행 환경. 현재 구현은 `mock` |

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
| `FINISHED` | 최종 Result 저장 완료 |
| `INTERRUPTED` | 재시작 시 발견한 미완료 실행. 가짜 Result를 만들지 않음 |
| `RECORDING_FAILED` | 저장 실패에 대한 메모리 진단 |

`has_result=false`이면 status/error_code/failed_stage/retryable을 Result로 해석하지 않는다.
중단 기록도 마지막 물리 상태와 완료 단계를 보존하며 사람 확인 필요를 표시한다.
저장 실패의 메모리 진단은 영속 저장을 주장하지 않는다.
단계 deadline은 기록 시각과 별개로 프로세스 단조 시계를 사용한다.

## 관측 형상: ObservedObjectGeometry

`ObservedObjectGeometry`는 `/grasp/collision_geometry`의 선택적 관측 형상
인터페이스다. header의 capture stamp/frame, snapshot_id, object_id로 기존
GraspCandidate와 연관하고, mesh_pose는 header frame 기준이며 Mesh vertices는
mesh_pose의 local 좌표다. convex footprint를 인식된 지지면까지 돌출한 형상으로,
실제 숨은 형상이나 시뮬레이터 정답을 의미하지 않는다. 기존 InspectScene,
PlanGrasp, GraspCandidate의 필드를 바꾸지 않아 기존 CDR 기록 형식은 유지한다.
메시 타입 의존성에 shape_msgs가 포함된다.

## 손목 관측: ObserveWristTarget

`ObserveWristTarget.srv`는 RGB-only 손목 인계/확인 서비스다.
HANDOFF는 head source ID, 원본 label/confidence와 예상 base-frame OBB를 전달받아
손목 영상에서 segmentation reference를 만든다. 별도 손목 detection은 선택 옵션이다.
CHECK는 같은 arm/source/reference의 새 RGB mask를 반환한다. CLEAR는 참조를 해제한다.
`expected_pose`는 입력 추정치일 뿐 측정값이 아니며 응답은 촬영 header와 mask만
포함한다. 손목에서 측정한 depth/3D 높이를 주장하지 않는다.
HANDOFF의 `expected_pose.header.stamp`는 원본 head 관측 시각이다.
`after_stamp_ns`보다 새로운 손목 image/CameraInfo 쌍만 처리한다.

## VerifyPlacement

`srv/VerifyPlacement.srv`는 label, destination_id, release 이후 기준
`after_stamp_ns`를 받아 success/message로 실제 놓기 결과를 응답한다.
그리퍼 열기 명령의 성공은 놓기 성공이 아니다. 현재 제공자는 시뮬레이션
평가용 oracle이며 조작 코드에는 물체 정답 pose를 반환하지 않는다.
실제 로봇에는 독립 센서 기반 검증 제공자가 필요하다.

## 객체 메시지

`DetectedObject2D`와 `DetectedObject2DArray`는 Gemini detector가 반환한 RGB pixel
bounding box, snapshot-local 번호와 후속 선택 요청에 사용할 `snapshot_id`를 표현한다.
각 detection은 촬영 시점 depth와 TF로 계산한 configured target-frame 원점 기준
`distance_m`과 유효 여부를 포함한다. 유효한 후보는 거리, confidence, detector 원본
순서로 정렬되며 depth가 불충분한 후보는 자동 조작 대상으로 선택하지 않는다.
`DetectedObject3D`와 `DetectedObject3DArray`는 선택 객체의 OBB와 동일 snapshot 문맥을
표현한다.

`DetectedObject2D`는 모델이 반환한 `sorting_category`와 `sorting_reason`을
포함한다. category는 `trash`, `lost_item`, `review` 또는 미제공 빈 문자열이다.
빈 값은 이름 기반 폐기 허가가 아니다. 이 메시지 변경 후 모든 consumer를 재빌드한다.

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

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Context](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)
