# cleany_interfaces

Cleany의 Perception, Grasping, Mission Manager, Skill Executor와 Dashboard
Bridge가 공유하는 ROS 2 interface package다. 구현 내부 model이나 provider별
응답을 wire contract로 노출하지 않는다. 커스텀 메시지뿐 아니라 표준 ROS 메시지를
사용하는 프로젝트 공통 topic 계약도 이곳에 기록한다.

`PlanGrasp`는 score 내림차순 `GraspCandidate[] candidates`를 반환한다.
`SelectReachableGrasp` action은 같은 snapshot/object/frame/OBB 후보를 받아 양팔 IK,
state validity, 2구간 plan-only 검증 후 선택 index, arm, endpoint joint state를 반환한다.
trajectory는 현재 RobotState에 종속되므로 result에 포함하지 않는다.

## Contracts

- [Mobile base](docs/mobile_base.md): `/cmd_vel` 차체 속도 명령

## 객체 메시지

`DetectedObject3D`는 하나의 oriented bounding box를 표현한다.

- `object_id`: snapshot 안에서 1부터 부여하는 사용자 선택 번호
- `label`: detector가 반환한 객체 label
- `confidence`: `[0, 1]` 범위의 normalized confidence
- `obb_pose`: OBB 중심과 방향
- `obb_size`: OBB local X/Y/Z 방향의 전체 길이(m)

`DetectedObject3DArray.header`가 모든 객체의 capture timestamp와 frame을 소유한다.
`snapshot_id`는 inspection 결과와 이후 planning 요청을 연결하는 opaque identifier다.
객체별로 서로 다른 frame을 사용하지 않는다.

## Scene inspection action

`InspectScene`은 동기화한 RGB-D snapshot에 detector, segmenter, 3D reconstruction과
TF 변환을 한 번 수행하는 cancel 가능한 action이다. 빈 `query`는 node의 기본 prompt를
사용한다. 결과가 정상적으로 비어 있는 경우에는 `success=true`, `ERROR_NONE`, 빈
`objects`를 반환한다.

오류 코드는 RGB-D timeout, detector API, detector response/JSON, mask, depth,
plane, TF, cancel, internal failure를 구분한다. feedback stage는 RGB-D 대기부터
target-frame 변환까지의 현재 단계를 나타낸다.

## Grasp planning service

`PlanGrasp` request는 선택한 객체의 snapshot/object ID, target/context point cloud와
`DetectedObject3D` OBB를 직접 전달한다. 따라서 planning server는 perception node의
숨은 object cache에 의존하지 않는다. geometric predictor나 AnyGrasp 같은 내부
provider가 score 내림차순 `GraspCandidate[]`를 생성한다.

## Reachable grasp selection action

`SelectReachableGrasp`는 `PlanGrasp`가 만든 후보들을 받아 양팔 IK, state validity와
pregrasp/grasp 2구간 plan-only 검증을 수행한다. 성공 result는 선택한 candidate index,
arm, candidate와 endpoint joint state를 반환한다. 실제 trajectory는 현재
`RobotState`에 종속되므로 result에 포함하지 않는다.

## 빌드와 검사

레포지토리 루트에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select cleany_interfaces
source install/setup.bash
colcon test --packages-select cleany_interfaces
colcon test-result --verbose
```

설치된 계약을 확인한다.

```bash
ros2 interface show cleany_interfaces/msg/DetectedObject3D
ros2 interface show cleany_interfaces/msg/DetectedObject3DArray
ros2 interface show cleany_interfaces/msg/GraspCandidate
ros2 interface show cleany_interfaces/action/InspectScene
ros2 interface show cleany_interfaces/action/SelectReachableGrasp
ros2 interface show cleany_interfaces/srv/PlanGrasp
```

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)
