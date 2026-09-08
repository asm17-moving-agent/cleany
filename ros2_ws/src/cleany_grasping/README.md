# cleany_grasping

sorting launch는 `publish_collision_geometry=true`로 같은 RGB-D target 점군의
관측 볼록 외곽을 `/grasp/collision_geometry`에 발행한다(reliable/transient-local,
depth 16). 일반 node 기본은 false다. target OBB local XY에서 모든 관측점을
포함하는 볼록 외곽을 만들고, 관측 Z와 OBB 바닥/상단을 포함하는 프리즘을 생성한다.
숨은 뒷면/구멍을 복원한 실제 mesh가 아니며, cup 내부도 비어 있다고 가정하지 않는다.
최대 512 vertices를 넘거나 퇴화한 형상은 축소하지 않고 요청을 실패시킨다.
선택 후보와 동일한 snapshot/object/capture/frame/OBB pose를 별도 메시지로
전달하여 기존 후보 메시지 및 과거 기록 포맷을 유지한다. collision geometry만
개선하며 물리 시뮬레이터의 물체 mesh, 질량, 마찰은 변경하지 않는다.

선택 객체에 대해 geometric 또는 AnyGrasp 후보를 필터링하고 score 내림차순
`GraspCandidate[]`를 반환한다.
팔 선택, IK, MoveIt collision, trajectory와 실행은 이 패키지의 범위가 아니다.

`geometric.prefer_upward_closing_axis`는 기본 false인 비대칭 그리퍼 진단 옵션이다.
동일 점수 후보끼리만 fixed-jaw 쪽(+closing)이 위를 향하는 방향을 먼저 둔다.
점수나 형상은 변경하지 않으며, 실제 파지 성공을 보장하지 않는다. sorting launch의
`prefer_upward_closing_axis:=true`로 시험할 수 있고 기본 실행은 기존 순서를 유지한다.

`geometric.search_longitudinal_contacts`는 기본 false, sorting launch에서 true다.
얇고 긴 객체는 중심 외에 긴 방향의 파지 지점도 생성한다. 점군의 지지면 높이/
길이 비가 `longitudinal_max_height_ratio`(0.4) 이하이고 길이가 finger length보다
클 때만 `longitudinal_offset_fractions`(0, -0.25, 0.25)를 적용한다. 전체 finger
길이가 robust 관측 범위 안에 남도록 이동량을 제한한다. 일반 기본은 높이 유지이며,
sorting의 `longitudinal_contact_height_offset_m=0.003`은 이 조건을 만족하는
얇고 긴 물체에만 지지면 법선 방향으로 3mm 보정한다. 레고처럼 finger length보다
짧거나 컵처럼 두꺼운 물체에는 높이 보정도 적용하지 않는다. 이 값은 비대칭 jaw의
닫힘 구간 지지면 여유를 확보하기 위한 시뮬레이션 시험값이며 실제 로봇 교정값은 아니다.
폭과 점수는 변경하지 않으며, 동점에서는 robot reference에 가까운 지점을 먼저
제안한다. context 충돌 검사, 후속 NMS/IK/MoveIt 검사는 그대로다. 물체 라벨이나
시뮬레이터 실제 위치를 사용하지 않으며, 파지 유지 성공은 별도 물리 검증이 필요하다.

sorting launch의 `geometric.defer_support_plane_collision=true`는 간이 대칭
손가락 모델에서 지지 평면의 inlier만 제외한다. 평면 위 장애물은 그대로 검사한다.
이 옵션은 반드시 아래의 실행 계약과 함께 사용해야 하며 단독 실행용이 아니다.
selector와 executor가 plane-aligned perception OBB의 바닥으로 유한 support
patch를 등록하고, 실제 로봇 형상으로 열린 jaw 및 전체 닫힘 구간을 검사한다.
기본 false는 기존 간이 지지면 충돌 검사다. sorting launch는 이 계약의 세 설정을
함께 활성화하며, 지지면을 무시하거나 MuJoCo 정답 책상 위치를 주입하지 않는다.

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
4. 설정된 support-normal tilt, 각 주축과 yaw offset마다 gripper 폭과 pose를 계산한다.
5. finger/palm 부피가 target 외 context 점과 겹치는 후보를 제거한다.
6. 폭 여유, 주축 정렬과 짧은 축 선호도를 점수화한다.

이 방식은 상자·캔처럼 위에서 접근 가능한 단순 강체를 위한 deterministic MVP다. 복잡한
형상, 변형 물체, 측면 접근과 최종 로봇 충돌 검사는 별도 predictor 또는 MoveIt 단계가
필요하다.

AnyGrasp import는 adapter가 첫 요청까지 지연한다. `predictor_type: geometric`에서는
checkpoint와 SDK license가 필요 없다. `GraspPredictor` port를 구현하면 다른 predictor도
주입할 수 있다.

## 물체별 접근각 탐색

`geometric.search_approach_tilts=true`이면
`geometric.approach_tilt_options`의 각도(기본 0/8/16/24도)를 평가한다.
기존 단일 `approach_tilt_degrees` 경로는 기본값으로 유지한다.
`include_reverse_closing_axis`는 비대칭 jaw 때문에 필요한 반대 closing
방향 후보도 생성한다. 분리 수거 launch에서만 두 옵션을 활성화한다.
각도 후보가 NMS에서 합쳐지지 않도록 이 launch의
`nms_rotation_threshold_degrees`는 5도로 설정한다(기존 기본 20도).
물리 그리퍼 최대 폭이나 충돌 검사는 완화하지 않는다.

## 후보 이미지 확인

각 `grasp/plan` 요청은 `grasp/debug_image`에 800x800 RGB 이미지를 발행한다. 원본 RGB와
camera intrinsics가 grasp request에 포함되지 않으므로 pixel overlay가 아니라 추정
지지면 기준 top view를 사용한다. 주황색은 target, 회색은 context, 파란색 선은 후보 jaw,
초록색은 최고 점수 후보이며 숫자 뒤 값은 후보 점수다. topic은 `TRANSIENT_LOCAL`이므로
요청 뒤에 viewer를 열어도 마지막 결과를 받을 수 있다.

```bash
ros2 run rqt_image_view rqt_image_view /grasp/debug_image
```

AnyGrasp import와 detector 생성은 adapter가 첫 요청까지 지연하므로 ROS 비의존 core와
fake 테스트는 라이선스 SDK 없이 실행할 수 있다. Jetson service에서는 별도 entrypoint가
network identity, pinned SDK Feature ID와 read-only mount를 먼저 검사하고, run command가
license validation과 detector 초기화를 마친 뒤 node를 시작한다. `GraspPredictor` port를
구현하면 다른 predictor도 주입할 수 있다. adapter는 2026 aarch64 `dev` SDK의
`create_detector()`와 region steering API를 사용한다.

## 설정

`config/anygrasp.yaml`에서 다음 값을 배포 환경에 맞게 설정한다.

- `predictor_type`: 기본 `geometric`, SDK 사용 시 `anygrasp`
- `debug_image_topic`: 후보 top-view 이미지 topic
- `geometric.*`: gripper 형상, 충돌 여유, RANSAC, depth 경계 outlier trim,
  yaw와 planning-frame 접근 tilt 설정. `reject_robot_opposite_approach`를
  활성화하면 `robot_reference_position`에서 물체로 향하는 수평 방향과 반대인
  approach 후보를 생성 단계에서 제거한다.
- `checkpoint_path`, `license_path`: Jetson-local AnyGrasp SDK 파일
- SDK 제약상 네 license 파일의 directory 이름은 `license`여야 한다.
- `gripper_height_m`: SDK collision model의 finger height
- `planning_frame`: MoveIt planning frame (기본 `base_link`)
- `maximum_gripper_width_m`: 실제 gripper calibration 뒤 확정할 값
- `canonical_to_tcp_rotation`: GraspNet canonical frame에서 Cleany
  `*_grasp_tcp` frame으로의 3x3 row-major rotation
- `tcp_approach_axis`: Cleany TCP frame에서의 접근 축

Cleany의 물리 TCP 접근축은 local `-Y`다. 기본 변환은 GraspNet canonical `+X`
approach와 `+Y` closing을 각각 Cleany TCP `-Y`와 `+X`에 대응시킨다. 따라서 geometric과
AnyGrasp 후보 모두 동일한 실제 gripper 접근축 기준의 pose와 방향을 반환한다.

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
사실로 간주하지 않는다. AnyGrasp 전용 고정 MAC service와 license 신청 절차는
[`containers/vision`](../../../containers/vision/README.md)을 따른다. host-native feature ID와
container feature ID는 서로 다르므로 실제 배포 container 안에서 검증한다. 현재 배포
기준 ID는 `N11176336906968411287`이며 다르면 node 시작을 거부한다.

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

`geometric.approach_reference_positions`는 planning frame의 기준점 xyz를
평탄화한 배열이다(기본 `[0.0]`은 비활성). 책상 정리 launch는 URDF의 두 어깨
위치로부터 관측 물체 중심을 향하는 수평 접근 방향을 각각 생성한다.
접근 tilt/어깨 기준점 후보를 교차 배분하며, 반대 closing-axis도 같은 short-axis
점수를 받아 비대칭 그리퍼의 180도 방향이 후보 제한에서 사라지지 않게 한다.
물체 위치는 RGB-D에서 오며 기준점은 로봇 기구학 정보다.

## 큰 물체의 접촉 높이

Sorting launch는 기하 후보를 최대 96개 생성한다(일반 demo 24개).
여러 접근 기울기와 closing 방향이 24개 상한에서 잘려 재관측 시 기존 방향과
호환되는 후보가 사라지는 경우를 보완한다. 실행 selector의 IK 후보 예산은
24개 그대로이며, 재관측에서는 기존 방향/중심 연속성 조건으로 먼저 선별한다.
생성 후보의 증가는 파지나 충돌 안전성의 보장이 아니며 기존 검사를 모두 거친다.

`geometric.maximum_top_contact_depth_m`은 기본 및 sorting에서 0(기존 체적 중심)이다.
Demo launch의 `grasp_maximum_top_contact_depth_m` argument로 같은 값을 전달할
수 있다(기본 0 유지). 현재 full-close/정밀 IK/관측 mesh 조건에서 0.035m
높이 제한을 별도 컵 진단으로 시험한다. 과거 0.025m 실패를 성공으로 바꾸어
기록하지 않으며, 검증 전에는 새로운 기본 운용값으로 채택하지 않는다.
0.025m 실험에서는 RGB-D target cloud의 support-normal 방향 상단 percentile을
기준으로 접촉점을 최대 25mm 아래에 둔다. 중심이 이미 더 높으면 그대로 유지해
레고 같은 얇은 물체를 과도하게 높게 잡지 않는다. 컵에 깊게 들어가 그리퍼 몸체가
윗부분을 누르던 경우를 줄이기 위한 후보 생성 설정이며 라벨/GT 좌표는 사용하지 않는다.
로봇 TCP 보정과 충돌 검사, 관절/파지 허용오차는 별도로 그대로 적용한다.
현재 컵에서는 움직이는 jaw가 빈 내부 공간으로 들어가 접촉하지 않아 이 실험 설정을
채택하지 않았다. 단순히 높게 잡는 것이 파지 신뢰성을 높이는 것은 아니다.
