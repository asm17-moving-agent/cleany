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

`WristTrackingStatus.msg`는 백그라운드 SAM2 추적의 원본 촬영 Header,
arm/reference/source snapshot/object ID, 처리 유효성(`valid`), mask 면적 범위 기반
가시성(`visible`), mask/image pixel 수와 reason을 전달한다.
`/perception/wrist_tracking_status`는 reliable depth 10이며 새 추론 결과/오류에만 발행한다.
오류에는 `valid=false`를 사용하며 촬영 시각을 임의 생성하지 않는다.
`visible=false`는 가림/시야 이탈/추적 실패일 수도 있어 확정 낙하 판정이 아니다.

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
2차 단계는 `snapshot_id`와 `selected_object_id`로 선택한 객체 하나만 SAM2 및 3D
복원한다. 성공한 2차 결과는 같은 configured target frame과 capture timestamp의
`target_cloud`, `context_cloud`와 선택 객체 OBB를 반환한다.

## Grasp planning과 선택

`ObserveObjectReference` service는 검출 재실행과 구분된 관측 계약이다.
`PIN(1)`은 기존 `source_snapshot_id/source_object_id`를 단일 reference로 보존하고,
`OBSERVE(2)`는 reference ID로 요청 시점 이후의 RGB-D + capture TF를 관측한다.
`CLEAR(3)`은 명시적으로 해제한다. 유효기간은 inspector 설정이며 만료 시 실패한다.
`source_*`는 최초 detector의 의미·신뢰도·촬영 시각이다. `header`, `mask`,
`observed_cloud`, `observed_center/extent`는 **현재 관측된 표면**이다. center/extent는
완성된 OBB나 숨겨진 부피가 아니며 SAM2 확률 또는 새 YOLOE confidence를 만들지 않는다.
현재 RGB-D 유실, TF 실패, 잘린 mask, 부족한 depth와 busy 요청은 성공으로 처리하지 않는다.

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

`DetectedObject2D`는 모델이 반환한 `sorting_category`와 `sorting_reason`을
포함한다. category는 `trash`, `lost_item`, `review` 또는 미제공 빈 문자열이다.
빈 값은 이름 기반 폐기 허가가 아니다. 이 메시지 변경 후 모든 consumer를 재빌드한다.

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)
