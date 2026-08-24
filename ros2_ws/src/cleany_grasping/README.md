# cleany_grasping

선택 객체에 대해 geometric 또는 AnyGrasp 후보를 필터링하고 score 내림차순
`GraspCandidate[]`를 반환한다.
팔 선택, IK, MoveIt collision, trajectory와 실행은 이 패키지의 범위가 아니다.

## 처리 경계

`grasp/plan` (`cleany_interfaces/srv/PlanGrasp`)은 perception 2단계 결과의 RGB
`target_cloud`, `context_cloud`, 원본 OBB를 받는다. predictor는 target bounds에 margin을
더한 workspace에서 후보를 만들고 context를 이용해 충돌 후보를 제거한다. 결과는 score
정렬, 선택 객체 접촉, gripper 최대 폭과 NMS 필터를 통과해야 한다. 남은 후보가
없으면 `ERROR_NO_GRASP_CANDIDATE`이며 pose를 합성하지 않는다.

기본 `geometric` predictor는 별도 모델 없이 다음 순서로 top-down parallel-jaw 후보를
생성한다.

1. context 점군에서 RANSAC으로 지지면 법선을 추정한다.
2. target 점군을 지지면에 투영하고 최소 폭 방향을 탐색해 두 물체축을 구한다.
3. 축별 robust extent 중점으로 visible surface 편향을 보정한 3D 중심을 복원한다.
4. 각 주축과 설정된 yaw offset마다 gripper 폭과 pose를 계산한다.
5. finger/palm 부피가 target 외 context 점과 겹치는 후보를 제거한다.
6. 폭 여유, 주축 정렬과 짧은 축 선호도를 점수화한다.

이 방식은 상자·캔처럼 위에서 접근 가능한 단순 강체를 위한 deterministic MVP다. 복잡한
형상, 변형 물체, 측면 접근과 최종 로봇 충돌 검사는 별도 predictor 또는 MoveIt 단계가
필요하다.

AnyGrasp import는 adapter가 첫 요청까지 지연한다. `predictor_type: geometric`에서는
checkpoint와 SDK license가 필요 없다. `GraspPredictor` port를 구현하면 다른 predictor도
주입할 수 있다.

## 후보 이미지 확인

각 `grasp/plan` 요청은 `grasp/debug_image`에 800x800 RGB 이미지를 발행한다. 원본 RGB와
camera intrinsics가 grasp request에 포함되지 않으므로 pixel overlay가 아니라 추정
지지면 기준 top view를 사용한다. 주황색은 target, 회색은 context, 파란색 선은 후보 jaw,
초록색은 최고 점수 후보이며 숫자 뒤 값은 후보 점수다. topic은 `TRANSIENT_LOCAL`이므로
요청 뒤에 viewer를 열어도 마지막 결과를 받을 수 있다.

```bash
ros2 run rqt_image_view rqt_image_view /grasp/debug_image
```

## 설정

`config/anygrasp.yaml`에서 다음 값을 배포 환경에 맞게 설정한다.

- `predictor_type`: 기본 `geometric`, SDK 사용 시 `anygrasp`
- `debug_image_topic`: 후보 top-view 이미지 topic
- `geometric.*`: gripper 형상, 충돌 여유, RANSAC, depth 경계 outlier trim과 yaw 후보 설정
- `checkpoint_path`, `license_path`: Jetson-local AnyGrasp SDK 파일
- `planning_frame`: MoveIt planning frame (기본 `base_link`)
- `maximum_gripper_width_m`: 실제 gripper calibration 뒤 확정할 값
- `canonical_to_tcp_rotation`: GraspNet canonical frame에서 Cleany
  `*_grasp_tcp` frame으로의 3x3 row-major rotation
- `tcp_approach_axis`: Cleany TCP frame에서의 접근 축

점군은 동일 optical frame/timestamp여야 하며 요청의 OBB pose도 그 frame을 사용한다.
node는 촬영 시점 TF를 한 번 조회해 최종 pose, 접근 방향과 OBB pose를 모두 planning
frame으로 변환한다.

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
