# cleany_interfaces

Cleany perception snapshot과 위치 기반 grasp planning이 공유하는 ROS 2 interface
package다. 구현 내부 model이나 provider별 응답을 wire contract로 노출하지 않는다.

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

`PlanGrasp` request는 선택한 `DetectedObject3D`와 그 snapshot header를 직접
전달한다. 따라서 planning server는 perception node의 숨은 object cache에 의존하지
않는다. `arm_override`는 `ARM_AUTO`, `ARM_LEFT`, `ARM_RIGHT` 중 하나다.

성공 response는 다음 값을 제공한다.

- `selected_arm`: `ARM_LEFT` 또는 `ARM_RIGHT`
- `tcp_frame`: FK 검증에 사용한 nominal grasp TCP frame
- `grasp_point`: request header frame의 상단 중심 파지점
- `joint_target`: 관절 이름과 위치만 채운 planning 결과
- `tcp_position_error_m`: joint target을 FK한 TCP 위치 오차

`joint_target`은 robot command가 아니다. 이 service는 trajectory, collision
avoidance, gripper 자세와 실제 arm 전송을 수행하지 않는다.

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
ros2 interface show cleany_interfaces/action/InspectScene
ros2 interface show cleany_interfaces/srv/PlanGrasp
```

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [ROS 2 Software Architecture](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md)
