# cleany_skill_executor

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 운영 action은 실제
trajectory와 gripper 명령을 실행하지 않는다. 별도 시뮬레이션 데모에서만 선택 결과를
MuJoCo arm controller로 실행할 수 있다. `nearest_pregrasp_coordinator`는 perception이
제공한 거리 유효 객체를 가까운 순서로 시도해 첫 reachable grasp의 pre-grasp까지만
실제 controller로 실행하는 초기 운영 coordinator다.

## Reachable grasp action

`grasp/select_reachable` (`SelectReachableGrasp`)는 가까운 팔부터 IK, state validity,
current→pre-grasp와 pre-grasp→grasp plan을 검사한다. pre-grasp는 접근 벡터 반대 방향
0.14 m다. 5축 position-only IK의 방향 손실을 막기 위해 먼저 TCP 위치 IK로 seed를 만든
뒤, TCP의 실제 `-Y` 접근축 앞 0.14 m에 있는 가상 aim tip을 grasp point에 맞춘다. 이로써
물리 gripper가 선택 객체를 바라보는 것은 강제된다. 위치 seed가 실패하면 현재 joint
state에서도 계속 시도하며, 전체 arm joint limit 안에 분산한 팔당 8개 seed에서 허용
오차를 만족하는 해를 자세 오차순으로 정렬한다. 5축 기구가 정확한
후보 방향을 만들 수 없는 경우에는 접근축 최대 15도, parallel-jaw 대칭을 고려한 closing
축 최대 30도 안에서 가장 가까운 실행 가능 방향을 허용한다. 그보다 큰 오차는 다음 arm
또는 후보로 fallback한다. grasp 위치에서도 TCP 위치·접근축·closing 축을 FK로 다시
검사한다. 한 해의 validity 또는 planning이 실패하면 같은 팔의 다음 grasp/pre-grasp
해를 먼저 시도한 뒤 반대 팔과 다음 후보로 넘어간다. 선택 오차는 selector 로그에 남긴다.

## 제공 계약

실행 결과는 Mission Manager가 상태 전이에 사용할 수 있는 성공·실패·차단 결과로 반환한다.
Skill Executor는 Mission Manager의 FSM 상태를 직접 변경하지 않는다.

## 설정 및 검증

입력 후보는 snapshot/object/frame/target OBB가 같고 frame은 설정된 `planning_frame`과
일치해야 한다. 현재 12개 arm/gripper joint는 완전하고 0.5초 이내여야 한다. action
동안 target OBB와 ACM은 임시 변경되고, 복원에 성공한 뒤에만 최종 성공·실패·취소
상태를 확정한다. 복원 실패는 planning-scene 오류로 반환한다. 한 번에 goal 하나만
처리한다.

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py
ros2 launch cleany_skill_executor grasp_selection.launch.py
pytest -q ros2_ws/src/cleany_skill_executor/test
```

planning frame, timeout, planning attempt/scaling, 최대 후보 수는
`config/grasp_selection.yaml`의 ROS parameter로 설정한다. timeout/cancel은 각 IK,
state-validity와 planning 단계 전후에 확인한다.

simulation 실행은 planning attempts 3, velocity/acceleration scaling 0.08,
MoveGroup replan 2회와 0.25초 delay를 사용한다. controller 실패 시 joint별 tracking
error를 기록하고 현재 상태에서 한 번만 같은 joint goal로 안전 재계획한다.

## 가까운 객체 자동 pre-grasp

`nearest_pregrasp_coordinator`는 다음 순서로 한 번의 작업을 수행한다.

```text
InspectScene 1차 detection
→ distance_valid 후보를 base_link 거리순 정렬
→ selected-object inspection
→ PlanGrasp
→ SelectReachableGrasp
→ target OBB를 충돌물로 유지하며 pre-grasp 실행
```

양쪽 gripper는 object evaluation 전에 open한다. 따라서 MoveIt 후보 검증과 실제 실행이
동일한 gripper joint state 및 collision geometry를 사용한다.

가장 가까운 객체가 segmentation, grasp 생성 또는 양팔 도달성 검사에서 실패하면 다음
가까운 객체로 fallback한다. depth가 불충분해 `distance_valid=false`인 detection은
자동 조작에서 제외한다. infrastructure, MoveIt 또는 controller 실패는 다른 객체로
넘기지 않고 전체 작업을 실패시킨다.

RGB-D camera, `perception/inspect_scene`, `grasp/plan`,
`grasp/select_reachable`, MoveGroup과 arm/gripper controller를 먼저 실행한 뒤 coordinator를
시작한다.

```bash
ros2 launch cleany_skill_executor nearest_pregrasp.launch.py
```

query, timeout, 속도/가속도 scaling과 gripper open 위치는
`config/nearest_pregrasp.yaml`에서 설정한다. pre-grasp joint target은 운영 selector가
aim-tip IK와 FK 방향 검증까지 통과한 결과를 사용한다. 성공한 target OBB는 완료 자세에서
Planning Scene에 유지하며, 실패하거나 coordinator가 종료될 때 기존 ACM과 함께 복원한다.

가까운 객체 선택을 실제 MuJoCo RGB-D 입력부터 확인하는 GUI 데모는 다음과 같이 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor nearest_rgbd_pregrasp_demo.launch.py
```

MuJoCo가 렌더링한 빨간 can과 파란 box의 실제 RGB 픽셀로 bbox/mask를 만들고, depth와
CameraInfo를 역투영한 뒤 capture-time TF로 `base_link` 거리를 계산한다. 유효 거리순으로
첫 객체를 선택해 geometric grasp, 양팔 MoveIt 검증 및 실제 controller pre-grasp 실행까지
이어진다. MuJoCo, RViz와 `/perception/debug_image_latched` Image View가 기본으로 열린다.
색상 detector/segmenter는 이 결정적 simulation 회귀 데모만을 위한 adapter이며 물체 pose나
가상 point cloud를 사용하지 않는다.

Gemini detection부터 SAM2 segmentation 및 pre-grasp 실행까지 확인하려면 키를 현재
shell의 환경변수로만 주입하고 다음처럼 실행한다. 키를 저장소 파일이나 launch 인자에
기록하지 않는다.

```bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_skill_executor nearest_rgbd_pregrasp_demo.launch.py \
  detector_type:=gemini segmenter_type:=sam2 \
  gemini_model:=gemini-robotics-er-2-preview \
  sam2_model_config:=configs/sam2.1/sam2.1_hiera_t.yaml \
  sam2_checkpoint:=/home/ubuntu/models/sam2/sam2.1_t.pt \
  sam2_device:=cpu
```

Gemini는 렌더 RGB에서 bbox와 label만 반환한다. 이후 mask는 SAM2, 거리와 3D geometry는
MuJoCo depth·CameraInfo·capture-time TF에서 계산하며, 이동은 동일한 MoveIt 및
`mujoco_ros2_control` 경로를 사용한다.

## MuJoCo 육안 확인 데모

아래 단일 launch는 실제 `mujoco_ros2_control` backend, MoveIt, RViz, grasp selector와
demo coordinator를 함께 시작한다. 기본값은 MuJoCo native viewer와 RViz를 모두
표시한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py
```

기본 synthetic reachable candidate는 같은 grasp point를 공유하는 실제 5축
pre-grasp/grasp FK 해에서 얻은 quaternion을 사용한다. 따라서 local `-Y` 접근축과
local `+X` closing 축을 selector가 보존하는지 데모 자체에서도 검증한다.

데모는 의도적으로 최고 점수의 도달 불가 후보를 먼저 검사한 뒤, 초록색 MuJoCo box와
정렬된 두 번째 후보를 왼팔로 선택한다. 선택 action에서 direction-aware pre-grasp IK,
두 endpoint의 collision/state validity, 두 구간 OMPL plan-only를 통과해야만 coordinator가
`left_arm_controller`로 pre-grasp와 grasp trajectory를 차례로 실행한다. 마지막에는
실제 `/joint_states`가 선택 결과에 수렴했는지도 검사한다. Gripper close, attach, lift는
아직 실행하지 않는다.

RViz의 `Grasp Candidates` display에서 구/화살표/상태 문구를 보고,
`MotionPlanning` display에서는 계획 궤적과 실제 joint state를 확인한다. MuJoCo 창의
초록색 box가 target이며 왼팔이 먼저 pre-grasp에서 멈춘 다음 box까지 접근한다. 창이
준비될 시간을 위해 평가 전 5초, 각 실행 구간 사이 3초를 기본 대기한다. 빠른 headless
회귀 검증은 다음처럼 실행한다.

```bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py \
  headless:=true use_rviz:=false \
  demo_start_delay_sec:=0.1 stage_hold_sec:=0.1
```

로그의 `Selected candidate=1 arm=left`, 두 개의
`MoveIt execution succeeded`, `DEMO COMPLETE`가 전체 성공 기준이다. 데모는 완료
자세와 marker를 유지하므로 종료는 `Ctrl-C`, 다시 보기는 launch 재실행으로 한다.

### 실제 RGB-D can 검출·이동 데모

다음 launch는 table, 파란 box, 빨간 can, 노란 경로 장애물과 고정 RGB-D 카메라가 있는
`mujoco_ros2_control` 장면을 연다. 시뮬레이터가 렌더링한 RGB-D에서 빨간 can을
분할하고 `base_link` 점군으로 투영한 뒤, geometric grasp 후보 생성과 MoveIt
양팔 검증을 거쳐 선택된 pre-grasp 자세까지 실제 controller로 이동한다. 실제 joint
feedback으로 도착을 확인한 뒤 그 위치에서 그리퍼를 연다. selector가 반환한 동일
joint goal을 실행하며 실행 직전 IK를 다시 계산하지 않는다. 그리퍼 전방 0.14 m의
가상 조준점은 selector에서 계산하고, can 데모는 FK로 그 결과만 다시 확인한다. 따라서 실제
TCP는 can에서 0.14 m 떨어져 있고, 그리퍼의 접근축은 can을 향한 자세에서 멈춘다.
Can demo는 기계적 limit에서 최소 0.02 rad 여유를 둔 접힌 초기 자세로 양팔을 소환한다.
이 초기값은 launch에서 ROS control description에 전달되며 다른 MuJoCo workflow의 기본
0 rad 초기 자세는 변경하지 않는다.
운영 `SelectReachableGrasp` action에서도 같은 보정과 FK 방향 검증을 수행한다. can 데모는
실행 직전에 한 번 더 FK 결과를 표시해 육안 확인용 marker와 로그를 제공한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor can_grasp_execution_demo.launch.py
```

기본으로 세 창이 열린다.

- MuJoCo: 갈색 table 위 노란 장애물을 우회해 빨간 can으로 이동하는 실제 simulation 상태
- RViz: 동일 위치의 Planning Scene 장애물과 검출 can 반투명 원통, 후보 구,
  선택 후보 초록 구,
  접근 방향 파란 화살표
- Image View: `/grasp/can_grasp_image`의 실제 RGB 영상 위 후보별 TCP, 접근 화살표,
  score, 접근 azimuth/elevation, 요구 opening과 MoveIt 선택 결과

로그에서 `RGB-D can segmented`, `GEOMETRIC GRASP COMPLETE`, `Selected generated
candidate`, `Direction-aware pre-grasp verified`,
`MoveIt execution succeeded: collision-checked aimed pre-grasp`, `gripper opened`,
`CAN PREGRASP DEMO COMPLETE`가 차례대로
나오면 전체 경로가 성공한 것이다. RGB-D 렌더링에는 OpenGL context가 필요하므로
이 데모의 `headless:=true`는 지원하지 않는다. table, can과 노란 장애물은 MuJoCo
물리 충돌체이며 같은 크기와 pose로 MoveIt Planning Scene에도 등록된다. 노란
장애물은 기존 관절 직선 보간 경로의 중간을 막으므로 성공한 실행은 OMPL이 충돌
구간을 우회했다는 뜻이다. 후보 선택 중에는 검출 can OBB를 검사하고, action 종료 뒤
실제 pre-grasp trajectory를 다시 계획할 때도 can cylinder를 Planning Scene에
contact permission 없이 유지한다. gripper close, attach, lift는 포함하지 않는다.

## 관련 KB

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
