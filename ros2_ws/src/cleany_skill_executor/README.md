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

### 실제 RGB-D can 검출·잡기·들기 데모

다음 launch는 table, 파란 box, 빨간 can과 고정 RGB-D 카메라가 있는
`mujoco_ros2_control` 장면을 연다. 시뮬레이터가 렌더링한 RGB-D에서 빨간 can을
분할하고 `base_link` 점군으로 투영한 뒤, geometric grasp 후보 생성과 MoveIt
양팔 검증을 거쳐 선택된 pre-grasp 자세까지 실제 controller로 이동한다. 실제 joint
feedback으로 도착을 확인한 뒤 그리퍼를 열고, selector가 이미 검증한 grasp joint goal로
전진해 그리퍼를 닫는다. 비대칭 gripper 때문에 원본 후보와 pre-grasp pose는 보존하되
최종 grasp IK에만 local 접근축 `+10 mm`와 물체 폭으로 계산한 closing `+X`축
보정을 적용한다. 범용 selector의 closing 축 최대 오차는 30도지만, 원통형 캔이
jaw 사이로 빠져나가지 않도록 이 demo에서만 15도로 제한한다. 범용 parallel-jaw
계약은 `+X/-X`를 같은 축으로 보지만, 고정 jaw 보정에는 방향이 있으므로 이
demo의 최종 grasp만 부호까지 일치해야 한다. 접근 중 controller가
멈추면 FK로 목표까지 남은 거리가 10 mm
이내인지 확인한 경우에만 물체 접촉으로 받아들이고 endpoint 재시도를 생략한다.
MoveIt에는 can을 선택 gripper의 attached collision object로 전환하고, 같은 pre-grasp
goal까지 되돌아가 약 14 cm 후퇴·상승한다. 실행 직전 IK를 다시
계산하지 않는다. MuJoCo can은 60 g free body이며 weld 없이 jaw 접촉과 마찰만으로 들어
올린다. 마지막에는 새 RGB-D frame에서 can 높이가 5 cm 이상 증가했는지 확인한다.
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

- MuJoCo: 갈색 table 위 빨간 can으로 이동하는 실제 simulation 상태
- RViz: Planning Scene table과 검출 can 반투명 원통, 후보 구,
  선택 후보 초록 구,
  접근 방향 파란 화살표
- Image View: `/grasp/can_grasp_image`의 실제 RGB 영상 위 후보별 TCP, 접근 화살표,
  score, 접근 azimuth/elevation, 요구 opening과 MoveIt 선택 결과

로그에서 `RGB-D can segmented`, `GEOMETRIC GRASP COMPLETE`, `Selected generated
candidate`, `Direction-aware pre-grasp verified`,
`MoveIt execution succeeded: collision-checked aimed pre-grasp`, gripper open,
`MoveIt execution succeeded: contact-enabled grasp`, gripper close,
`MoveIt execution succeeded: attached-can lift retreat`,
`Physical can lift verified from RGB-D`, `CAN GRASP DEMO COMPLETE`가 차례대로
나오면 전체 경로가 성공한 것이다. RGB-D 렌더링에는 OpenGL context가 필요하다.
`headless:=true`로 native viewer를 숨길 수는 있지만 `DISPLAY`가 없으면 카메라가
비활성화되므로 X11/Xvfb context가 필요하다. table과 can은 MuJoCo 물리 충돌체이며
같은 크기와 pose로 MoveIt Planning Scene에도 등록된다. 후보 선택 중에는 검출 can
OBB를 검사한다. 실제 최종 접근부터는 선택 팔의 fixed/moving jaw에만 can 접촉을
허용하고, 닫기 뒤 MoveIt collision cylinder를 선택 gripper에 attach한다. 이 attach는
RViz와 lift 충돌 계획용이며 MuJoCo 물체를 강제로 고정하지 않는다. 닫힘 명령은 후보의
`required_opening_m`을 jaw 각도로 환산한다. 50 mm aperture의 기준점은 `0.30 rad`,
기울기는 `0.10 m/rad`이며 후보 opening보다 10 mm 작게 명령해 접촉력을 만든다.
후보 opening이 없을 때만 `0.30 rad`를 fallback으로 쓴다. 따라서 비스듬한 자세에서
약 70 mm로 보이는 박스를 50 mm용 각도까지 억지로 조여 밀어내지 않는다. 실제 접촉 시
목표까지 닫히지 않으면 0.10 rad 이상 닫힌 뒤 명령과 0.05 rad 이상 차이가 난 저속
정지 상태를 contact로 판정한다. 안정화
시간은 1초다. arm path tolerance는 실제 추종 오차 `0.100364 rad`가 기존
`0.10 rad` 경계를 넘은 측정 결과를 반영해 이 demo controller에만 `0.12 rad`를
적용한다. 최종 성공 판정은 이 관절 정지가 아니라 lift 뒤 RGB-D 높이 변화다.
`gripper_force_full_close:=true`는 후보 폭 환산을 우회해
`gripper_close_position_rad`를 직접 명령하는 simulation 진단 옵션이며 기본값은
`false`다. 이 모드에서는 물체 접촉으로 jaw가 목표 전에 정지해야만 파지로 인정한다.
설정된 완전 닫힘 위치에 도달하면 물체가 jaw 사이를 벗어났거나 contact가 관통한
것이므로 lift 전에 실패 처리한다. 실제 물체에 쓰기 전에는 충돌·토크 한계를 별도로
검증해야 한다. 접촉이 검출되면 해당 close trajectory의 위치 tolerance만 지워
토크 제한이 적용된 닫힘 목표를 lift 동안 계속 유지한다. lift 직후 높이 검사에 더해
기본 3초 유지 뒤 RGB-D 높이를 다시 검사하므로 잠깐 들렸다 떨어지는 경우는 성공으로
판정하지 않는다.
느린 simulation controller가 짧은 최종 접근 trajectory의 기본 MoveIt 시간 상한에
걸리지 않도록 이 launch에만 execution-duration scaling `2.0`, goal margin `1.0초`를
적용한다. 다른 MoveIt workflow의 기본값 `1.2`/`0.5초`는 유지한다.

selector는 물체 위치로 먼저 구한 어깨·팔꿈치 자세를 우선 IK seed로 쓰고, 같은 자세의
wrist roll을 물체가 놓인 좌우 방향에 맞춰 한 번 더 검사한 다음 범용 seed로 넘어간다.
pre-grasp 거리는 기존과 동일하게 14 cm 하나만 사용하며 selector 전체 작업 재시도는
하지 않는다. `grasp_approach_offset_m`과 `grasp_lateral_offset_m`의 공통 기본값은
모두 0이다. 이 simulation launch는 접근 방향으로 10 mm를 적용하고, 비대칭 jaw의
좌우 중심 보정은 `max(target_width_m, target_depth_m) / 2 - 8 mm`로 계산한다.
따라서 70 mm 캔과 50×70 mm 박스는 모두 27 mm를 사용한다. 박스 후보가 회전해 긴
변을 물더라도 고정 손가락이 물체 안에서 출발하지 않게 하는 보수적인 값이다. 같은
값을 selector와 실행 시 접촉 검증에 전달하므로
계획 목표와 검증 목표가 어긋나지 않는다.
같은 좌우 보정을 pre-grasp에도 적용한다. 따라서 마지막 14 cm 접근에서 TCP가
옆으로 이동하며 박스를 쓸지 않고, pre-grasp와 grasp 사이 변위가 접근축과 평행하다.

### 무작위 headless pre-grasp 스트레스 검증

DISPLAY가 없는 환경에서는 `mujoco_ros2_control`의 GLFW RGB-D renderer를 사용할 수
없다. 다음 검증은 RGB-D detection/segmentation을 제외하고 headless MuJoCo controller,
MoveIt selector, FK 자세 오차, 충돌 검사와 실제 pre-grasp 실행을 반복한다. 각 반복에서
table 위 box와 can 위치를 seed 기반으로 독립 생성하고, 하나를 target으로 선택하는 동안
다른 하나도 Planning Scene 충돌체로 유지한다. target은 box와 can을 번갈아 사용하며 성공
후 선택된 팔을 초기 자세로 복귀시켜 각 반복의 시작 조건을 맞춘다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
CLEANY_RANDOM_STRESS_ITERATIONS=100 \
CLEANY_RANDOM_STRESS_SEED=20260826 \
CLEANY_RANDOM_STRESS_RESULT=/tmp/randomized_pregrasp_stress.json \
python3 -m pytest -q -s \
  ros2_ws/src/cleany_skill_executor/test/test_randomized_pregrasp_stress.py
```

반복 수를 지정하지 않으면 장시간 스트레스 테스트는 일반 pytest에서 skip된다. 결과 JSON은
반복별 물체 위치, 선택·실행·전체 시간, 선택 arm/candidate와 실패 stage/error code를 담는다.

## 관련 KB

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
