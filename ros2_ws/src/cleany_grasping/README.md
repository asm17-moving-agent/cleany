# cleany_grasping

선택 객체에 대해 AnyGrasp 후보를 필터링하고 최선의 `GraspCandidate` 하나만 반환한다.
팔 선택, IK, MoveIt collision, trajectory와 실행은 이 패키지의 범위가 아니다.

## 처리 경계

`grasp/plan` (`cleany_interfaces/srv/PlanGrasp`)은 perception 2단계 결과의 RGB
`target_cloud`, `context_cloud`, 원본 OBB를 받는다. context 전체를 SDK에 전달하되 target
bounds에 margin을 더한 workspace를 사용하고 SDK collision detection을 켠다. 결과는
score 정렬, 선택 객체 접촉, gripper 최대 폭과 NMS 필터를 통과해야 한다. 남은 후보가
없으면 `ERROR_NO_GRASP_CANDIDATE`이며 pose를 합성하지 않는다.

AnyGrasp import는 adapter가 첫 요청까지 지연한다. 따라서 ROS 비의존 core와 fake 테스트는
라이선스 SDK 없이 실행할 수 있다. `GraspPredictor` port를 구현하면 다른 predictor도 주입할
수 있다.

## 설정

`config/anygrasp.yaml`에서 다음 값을 배포 환경에 맞게 설정한다.

- `checkpoint_path`, `license_path`: Jetson-local AnyGrasp SDK 파일
- `planning_frame`: MoveIt planning frame (기본 `base_link`)
- `maximum_gripper_width_m`: 실제 gripper calibration 뒤 확정할 값
- `canonical_to_tcp_rotation`: GraspNet canonical frame에서 Cleany
  `*_grasp_tcp` frame으로의 3x3 row-major rotation
- `tcp_approach_axis`: Cleany TCP frame에서의 접근 축

점군은 동일 optical frame/timestamp여야 한다. node는 촬영 시점 TF를 조회해 최종 pose와
접근 방향만 planning frame으로 변환한다. OBB는 perception이 이미 planning frame에
제공한 원본을 그대로 반환한다.

## Jetson 첫 관문

현재 공식 AnyGrasp `dev` branch의 aarch64 지원은 시험 단계이므로 배포 전 아래를 Jetson
Orin NX에서 별도 확인해야 한다.

1. JetPack 6.2 / CUDA 12.6 호환 PyTorch와 SDK가 요구하는 수정 MinkowskiEngine 설치
2. 공식 example data로 SDK import, checkpoint/license load, collision-enabled inference
3. MuJoCo box/can RGB point cloud 각각에서 후보가 하나 이상 생성되는지 확인
4. `ros2 service call /grasp/plan ...`으로 planning-frame candidate와 RViz marker 검증

SDK와 license는 이 저장소에 vendor하지 않는다. 검증하지 않은 Jetson 호환성을 구현
사실로 간주하지 않는다.

## 빌드와 테스트

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to cleany_grasping
source install/setup.bash
colcon test --packages-select cleany_grasping
colcon test-result --verbose
ros2 run cleany_grasping grasp_server --ros-args \
  --params-file src/cleany_grasping/config/anygrasp.yaml
```
