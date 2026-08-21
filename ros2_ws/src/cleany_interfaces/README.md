# cleany_interfaces

Cleany perception snapshot, grasp candidate 생성과 MoveIt 도달 가능성 검증이
공유하는 ROS 2 interface package다. 구현 내부 model이나 provider별 응답은 wire
contract로 노출하지 않는다. 커스텀 메시지뿐 아니라 표준 ROS 메시지를 사용하는
프로젝트 공통 topic 계약도 이곳에 기록한다.

## 객체 메시지

`DetectedObject2D`와 `DetectedObject2DArray`는 Gemini detector가 반환한 RGB pixel
bounding box, snapshot-local 번호와 후속 선택 요청에 사용할 `snapshot_id`를 표현한다.
`DetectedObject3D`와 `DetectedObject3DArray`는 선택 객체의 OBB와 동일 snapshot 문맥을
표현한다.

## Scene inspection action

`InspectScene` 1차 단계는 동기화된 RGB-D snapshot에서 2D detection만 수행하고,
2차 단계는 `snapshot_id`와 `selected_object_id`로 선택한 객체 하나만 SAM2 및 3D
복원한다. 성공한 2차 결과는 같은 configured target frame과 capture timestamp의
`target_cloud`, `context_cloud`와 선택 객체 OBB를 반환한다.

## Grasp planning과 선택

`PlanGrasp`는 score 내림차순 `GraspCandidate[] candidates`를 반환한다.
`SelectReachableGrasp` action은 같은 snapshot/object/frame/OBB 후보를 받아 양팔 IK,
state validity, 2구간 plan-only 검증 후 선택 index, arm, endpoint joint state를 반환한다.
trajectory는 현재 RobotState에 종속되므로 result에 포함하지 않는다.

팔 선택, IK, robot collision과 trajectory 계획은 `SelectReachableGrasp` 경계이며,
gripper 명령과 실제 trajectory 실행은 별도 Skill Executor coordinator가 담당한다.

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
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)
