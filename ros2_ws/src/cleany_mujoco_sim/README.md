# cleany_mujoco_sim

## 로봇 후면 수거함 배치 미리보기

`ros2 launch cleany_mujoco_sim robot_top_bins_preview.launch.py`는 기존 책상
양끝 분류 영역/구분선을 넣지 않고, 로봇 뒤쪽에 청록 테두리 분실물함(로봇 왼쪽)과
주황 테두리 쓰레기함(오른쪽)을 배치한다. 본체는 저채도 네이비로 통일한다.
`config/robot_top_bins.yaml`에서 base_link 기준 위치와 크기를 변경한다.
기존 실행 호환을 위해 파일명은 유지하지만 현재 배치는 상단이 아닌 후면이다.
각 함의 중심은 초기 base_link 기준 x=-0.405 m, y=±0.105 m이며
바닥 z=0.18 m, 상단 z=0.30 m이다. `rear_shelf` 설정으로 320×460 mm
목재색 상판, 하단 선반, 금속색 다리 네 개를 추가한다. 바닥에서 상판까지
높이는 560 mm이며 수거함 바닥과 상판 윗면은 일치한다.
선반과 수거함은 초기 chassis 변환을 복사한 world 고정 body에 배치하므로
로봇을 이동해도 따라가지 않는다. 수거함은 정적 fixture이며 자유롭게 밀리거나
넘어지는 동역학은 모델링하지 않았다. `rear_shelf`가 없으면 기존 chassis
부착 동작을 유지한다. 이 후면 배치는 미리보기 전용이며, 이동 로봇의 수거
계획에 사용하려면 초기 base 좌표를 현재 base 좌표로 변환하는 연결이 필요하다.
각 함의 외형은 180×170×120 mm,
벽 두께 8 mm이며 바닥과 네 벽이 있는 열린 충돌 형상이다.
기존 홈 자세를 유지하는 MuJoCo GUI만 실행하며 인식·자동 수거는 시작하지 않는다.
이 설정은 배치 검토용 시뮬레이션 시안이며 실제 장착 강도/하중과 수거 경로는
검증되지 않았다. 자동 수거와 일반 파이프라인 launch도 동일한 후면 설정을
기본으로 사용한다. 배치 통일은 수거 경로 검증 완료를 뜻하지 않는다.
선택적 `rim_rgba`는 벽 두께 안쪽의 둥근 시각 테두리를 추가하며 질량/충돌은
추가하지 않는다. 물리 형상은 기존 바닥과 네 벽으로 유지한다.

`handeye_backend.launch.py scheduled_cameras:=true`는 sorting observer hardware의
카메라별 촬영 주기 제어를 켠다. sorting_bins_config가 있어야 해당 hardware를 사용한다.
일반 hand-eye 실행 기본값은 false이며 기존 카메라 계약은 변경하지 않는다.
`config/sorting_wrist_cameras.yaml`은 양팔 RGB optical frame의 nominal CAD 장착값이다.
수거 launch의 TF 입력이며 실제 로봇에서 측정한 hand-eye calibration이 아니다.

`scenes/study_cafe_grasp_execution.xml.in`에는 컵–양손 그리퍼의 10개 접촉쌍을
명시해 미끄럼 마찰을 기존 접촉값 3에서 6으로 높인 시험 설정을 둔다.
`friction="6 6 0.2 0.1 0.1"`이며 비틀림/구름 마찰과 기본 접촉 softness는
유지한다. 컵–책상, 다른 물체, 질량·모터 힘·형상은 바꾸지 않는다.
이는 실측 재질 계수가 아닌 시뮬레이션 민감도 시험이며 실제 패드 성능을 보장하지 않는다.

분리 수거용 `handeye_backend.launch.py`에서 수거함 config를 전달한 경우,
`sorting_contact_diagnostics:=true`로 출력 전용 접촉 진단을 켤 수 있다.
기본은 false이며 `/simulation/contact_diagnostics` 계약과 한계는
`cleany_mujoco_observer/README.md`를 따른다. 로봇/물체 물리 상태나 perception
입력을 바꾸지 않으며 일반 hand-eye 실행에는 observer를 추가하지 않는다.

## 분리 수거 fixture

`handeye_backend.launch.py sorting_bins_config:=<path>`는 materialized
scene의 chassis 기준으로 바닥과 네 벽을 갖는 좌우 수거함을 추가한다.
`config/robot_top_bins.yaml`의 +Y가 분실물함, -Y가 쓰레기함이다.
빈 인자(기본값)는 기존 장면을 그대로 사용한다. template/materializer 외의
canonical 로봇 mesh/scene asset은 변경하지 않는다.

수거함은 실제 MuJoCo 충돌 형상이다. 물체를 순간 이동하거나 그리퍼에
weld하는 기능은 추가하지 않았다. 설정 시
`cleany_mujoco_observer/ObservedMujocoSystem` hardware wrapper를 선택한다.
설치된 backend의 제어/물리 동작을 유지하며, 잠금된 `get_data` 복사로
평가용 관측을 수행한다. 동시 갱신되는 vendor plugin 배열을 직접 읽지 않는다.
`placement_verifier`는 `/simulation/sorting_ground_truth`를 관찰하고
`/sorting/verify_placement` 요청에 release 이후의 올바른 수거함 내부
안착 여부를 응답한다. 이는 시뮬레이션 평가용 정답 정보이며 RGB-D 인식이나
grasp/경로의 입력으로 쓰지 않는다. 동작 연결은 skill executor의
`make sim-mujoco-sorting`을 사용한다(전체 집기·놓기 검증은 진행 중).

현재 vendor renderer는 headless여도 GLFW 연결을 필요로 한다. 이 VM에서는
유효한 `DISPLAY=:0`을 전달해야 RGB-D가 발행된다. GUI 창을 켜야 한다는
뜻은 아니며 display가 없으면 모델 준비 완료 뒤에도 RGB-D timeout이 난다.

XLeRobot MuJoCo 시뮬레이션을 ROS 2 `ament_python` 패키지로 연결한다.

## 상태와 책임

이 패키지는 세 개의 배타적인 MuJoCo 실행 경로를 제공한다. 기존
`mujoco_sim_node`는 mobile-base와 sensor 개발용 custom bridge이고,
`rgbd_pick_demo.launch.py`는 이 bridge를 확장한 RGB-D 평가 경로다.
`handeye_backend.launch.py`는 좌·우 arm trajectory를 위한
`mujoco_ros2_control` backend다. 한 프로세스에서 여러 경로를 함께 실행하지 않는다.
운영 환경의 내비게이션 및 실제 하드웨어 adapter는 이 패키지의 범위에 포함하지
않는다.

## 실행과 테스트

아래 명령은 레포지토리 루트에서 실행한다.

```bash
make sim
make sim-mujoco-study-cafe
make test-mujoco
```

`sim-mujoco-study-cafe`는 Gazebo 개발공간의 12.26×10.94 m 방과 동일한 좌표에
벽 4개, 책상 48개, 파티션 24개, 모니터 48개와 의자 47개를 띄운다. 기존 Gazebo
spawn에서 가장 가까운 43번 의자만 제거하고, 조종 가능한 끌리니 1대를 그 위치에서
책상을 향하도록 배치한다. 의자 중심에서 책상 반대 방향으로 0.18 m 이동해 로봇
collision 외곽과 책상 사이에 약 0.02 m 여유를 둔다. 로봇 앞 43번 책상의 현재
물체는 손잡이 없는 종이컵, 빨간 2×4 레고 블록, 흰 휴지뭉치, 기존 지갑이다.
머그컵/지우개/휴대폰을 교체하되 배치 위치와 지갑 mesh는 유지한다.

| 물체 / body ID | 외형 크기 | 물리 모델 |
| --- | --- | --- |
| 종이컵 / `study_cafe_cup` | 윗지름 85, 밑지름 55, 높이 95, 벽 0.7 mm | 열린 원뿔대 벽 32개와 바닥, 8 g |
| 레고 / `study_cafe_lego` | 몸체 31.8×15.8×9.6 mm, 돌기 포함 높이 11.4 mm | 본체 box와 8개 돌기 cylinder, 2.3 g |
| 휴지뭉치 / `study_cafe_tissue` | 60×50×45 mm | 불규칙 삼각형 표면과 convex-hull collision, 1 g |

`cleany_mujoco_sim/tabletop_shapes.py`가 설정값에서 결정적으로 MJCF mesh/primitive를
생성한다. 네 물체 모두 free joint가 있으며 강제 고정/attach하지 않는다. 종이컵은
안쪽을 채운 cylinder가 아니므로 입구로 들어가는 접촉도 가능하다. 다만 종이컵과 휴지는
눌리거나 구겨지지 않는 **강체 근사**이며, 레고 밑면 결합 튜브와 로고는 생략했다.
질량과 종이컵/휴지 크기는 시뮬레이션 가정이며 실물 측정값이 아니다. 레고는
[실측 치수](https://www.cailliau.org/Alphabetical/L/Lego/Dimensions/More%20Dimensions/BBEditPreviewTemp.html)의
8 mm 돌기 간격, 4.8 mm 지름, 1.8 mm 돌기 높이를 사용한다.
파지 실행 launch의 우측 팔 mirrored home 자세는 wrist-roll을 `-1.58 rad`로
시작하는 기존 설정을 유지한다.
위치, 질량, 색상과 collision 크기는
`config/study_cafe_layout.yaml`에서 조정할 수 있다. 기본 위치는 640×480
`head_realsense_rgb` 카메라와 `head_tilt_joint=1.00 rad` 조건에서 네 물체의 전체
물체가 화면 경계 안에 들어오도록 맞춰져 있다.

실제 집기 실행은 `scenes/study_cafe_grasp_execution.xml.in`을 사용한다. 일반 관찰 장면과
같은 배치를 유지하면서 chassis를 world에 고정한다. 종이컵/레고와 양쪽 fixed/moving jaw
사이의 고마찰 contact pair는 복합 collision의 모든 부분으로 확장한다(컵 330개,
레고 90개). 컵의 sliding friction 6, 레고 5는 기존 실험 설정을 계승한 값이지
실제 종이/플라스틱의 측정 마찰계수가 아니다. 얇은 강체 컵 벽의 수치적 관통을
줄이기 위해 컵 pair는 `solref="0.002 1"`, `solimp="0.99 0.9999 0.0001"`을 사용한다.
이는 종이의 찌그러짐을 재현하는 설정이 아니며 실제 컵 파지력 검증을 대체하지 않는다.
휴지와 지갑 강체 proxy도 각각 양쪽 jaw와의 10개 pair에 같은 강성을 적용한다. 기본 혼합
마찰 `(3, 3, .2, .1, .1)`과 질량/형상/모터 한도는 유지한다. 부드러운 접촉에서
관측된 휴지 약 18mm, 지갑 약 2.1mm 메시 관통을 줄이기 위한 설정이며
실제 휴지나 지갑의 압축 모델은 아니다.
지갑은 기존 시각 메시를 collision mesh에도 공유한다(MuJoCo convex hull).
시각 메시의 최소 평면 폭 약 52mm와 맞지 않는 110×85mm 축 정렬 box 때문에
보이지 않는 영역을 손가락이 누르던 문제를 수정했다. 메시 파일/스케일, 질량과
마찰은 변경하지 않는다. 오목한 내부 장식까지 충돌 분해한 모델은 아니다.
머리 카메라는 materialization 시
`640×480`, `fovy=42°` 계약을 검사해 RGB-D driver의 1×1 기본 해상도 사용을 막는다.

조명은 `config/study_cafe_lighting.yaml`에서 설정하며 일반 관찰/집기 장면과 GUI/RGB-D
렌더에 공통 적용한다. 기존 canonical robot의 world light는 materialized copy에서만
제거하고, **작업 책상 및 주변 책상을 비추는 넓은 spotlight와 방 전체 directional
보조광**으로 대체한다. 주광원은 `(-3.13, -3.60, 2.45) m`, cutoff 55°,
exponent 2로, 상판 높이에서 대략 5 m 폭의 주변 구역을 비춘다. 약한 전역 보조광은
유지하므로 나머지 공간도 완전히 어두워지지 않는다. 주광원만 그림자를 만든다.
카메라 headlight는 ambient 0.12, diffuse 0.08, shadow map은 4096이며
spot shadow scale 1.0으로 해당 광원 범위를 사용한다. 전역 directional shadow 설정의
half extent 12 m는 보존하지만 현재 보조광의 그림자는 꺼져 있다. 이는 실측 조도나
실내 간접광을 계산하는 모델이 아닌 근사다.
기존 headlight와 기본 광원의 합산으로 흰 상판이 포화되고
그림자 대비가 사라지는 문제를 줄이기 위한 설정이지 실측 조도/카메라 보정값은 아니다.
[MuJoCo headlight는 자체적으로 그림자를 만들지 않는다](https://mujoco.readthedocs.io/en/stable/XMLreference.html#visual-headlight).
그림자 광원은 추가 렌더 패스를 사용하므로 렌더링 비용이 증가할 수 있다. 물체 크기,
마찰, 카메라 intrinsics, 검출 confidence 및 파지 안전 기준은 조명 변경으로 바꾸지 않는다.

Gazebo plugin·sensor는 옮기지 않으며, Fuel에서 내려받는 의자 mesh 대신 Gazebo
collision 치수와 같은 MuJoCo primitive를 visual/collision로 사용한다. 원본 배치는
`config/study_cafe_layout.yaml`에 독립 실행용으로 복제하며, 테스트에서 Gazebo 원본
YAML과의 일치 여부를 확인한다.

Tabletop asset 출처와 라이선스는 `config/study_cafe_assets.yaml`에 기록한다. 현재
외부 mesh는 Poly by Google의 Wallet(CC BY 3.0)만 사용하며 원본 material 대신
scene color를 사용한다. 과거 Coffee Mug(Michael Fuchs, CC BY 3.0)와
Smartphone(smallbigsquare, CC0)의 OBJ, URL, 변환 내역, SHA-256은 이력용으로
보존하지만 현재 장면에 로드하지 않는다.

새 형상의 장면 컴파일, 실물 크기, 열린 컵 충돌체, 물리 settle, 카메라 시야는
`test_study_cafe_scene.py`/`test_tabletop_shapes.py`와 `make test-grasp-pregrasp`로
검사한다. 이 검증은 YOLOE 검출이나 집기 성공 검증을 대신하지 않는다. 기존 머그컵의
수거 성공/시간 측정 결과를 새 종이컵·레고·휴지에 적용하지 않는다. 실제 크기 레고는
sorting의 기존 50 mm 최소 파지 후보 폭보다 작으므로 자동 집기 대상에서 제외될 수 있다.

직접 launch할 수도 있다.

```bash
ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=false
```

MuJoCo viewer가 필요하면 build 후 실행한다.

```bash
make build
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
```

RGB-D pick demo는 custom backend만 실행하며 hand-eye ros2_control backend와 함께
사용하지 않는다.

```bash
ros2 launch cleany_mujoco_sim rgbd_pick_demo.launch.py
```

이 장면의 table은 `1.20 x 0.77 x 0.03 m`, 중심은
`(0.635, -0.002, 0.710) m`이며 고정 box/can과 정렬된 RGB-D 및 평가용 OBB를
발행한다. 물체 동역학이나 실제 파지 실행을 검증하는 장면은 아니다.

Hand-eye arm controller backend는 별도로 실행한다. 이 launch는 기존
`mujoco_sim_node`를 include하거나 시작하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim handeye_backend.launch.py \
  headless:=true sim_speed_factor:=1.0
```

RGB-D pick-demo 장면은 EGL renderer, 아래로 `0.8 rad` 기울인 head, 비활성화된
laser scan으로 별도 노드를 실행한다.

```bash
make build
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim rgbd_pick_demo.launch.py
```

장면의 상판은 `1.20 x 0.77 x 0.03 m`, 중심은
`(0.635, -0.002, 0.710) m`, 상면은 world `Z=0.725 m`다. 평가 대상은 고정된
파란 상자와 빨간 캔이다. 이 첫 장면은 perception과 위치 기반 IK를 반복 가능하게
평가하기 위한 것으로 물체 동역학이나 실제 파지 동작을 검증하지 않는다.

`scenes/grasp_execution_demo.xml.in`은 같은 backend에서 MoveIt-selected trajectory를
육안 확인하기 위한 전용 장면이다. chassis를 world에 고정하고, 기본 demo grasp/OBB와
동일한 `base_link` 중심 `(0.09, 0.6696, 0.6158) m`, 크기 `0.03 m`의 초록색
고정 box를 둔다. MuJoCo world에서는 초기 chassis 원점 Z=0.38 m를 더한
`(0.09, 0.6696, 0.9958) m`에 배치되어 RViz/MoveIt target과 물리 위치가 일치한다.
이 box는 아직 gripper command/force closure가 없는 reach 데모에서 controller 접촉
오차를 만들지 않도록 MuJoCo에서는 render-only다. 동일 OBB의 collision 검사는
MoveIt planning scene에서 수행한다. 통합 실행은 `cleany_skill_executor`의
`grasp_execution_demo.launch.py`를 사용한다.

`scenes/can_grasp_execution_demo.xml.in`은 실제 렌더 RGB-D 기반 grasp 통합 장면이다.
chassis 기준 table 중심은 `(0.700, -0.002, 0.330) m`, 빨간 can 중심은
`(0.440, 0.160, 0.395) m`이고 `pick_demo_rgbd` 카메라는 640×480, vertical FOV
42°다. table과 can은 모두 MuJoCo 물리 충돌체다. 검출 OBB는 후보 검증 중 MoveIt에
등록한다. can은 60 g free body로 table 위에 놓이며, 선택된 gripper를 pre-grasp에서
연 뒤 grasp 위치로 전진하고 jaw 마찰로 압착한다. 이후 같은 pre-grasp joint goal로
복귀해 can을 들어 올린다. MuJoCo equality weld나 강제 attach는 사용하지 않는다.
통합 실행은 `cleany_skill_executor`의 `can_grasp_execution_demo.launch.py`를 사용한다.

`scenes/box_grasp_execution_demo.xml.in`은 기존 박스와 장애물을 제거하고 새
50×70×100 mm 테스트 블록 하나만 사용한다. 블록 자체에
`friction="2.0 0.12 0.06"`과
`condim="6"`을 명시하고, fixed/moving jaw contact pair에는 simulation 전용
고마찰 패드(`friction="5 5 0.4 0.15 0.15"`, `condim="6"`)를 적용한다. 이 값은
캔 장면이나 실제 로봇의 접촉 설정에는 영향을 주지 않는다. box demo tabletop에도
명시적으로 `friction="2.0 0.08 0.02"`, `condim="4"`를 적용해 접근 중 작은 충격에
박스가 쉽게 미끄러지지 않게 한다. jaw가 강한 위치 명령으로 블록을 관통하지 않도록
이 장면만 1 ms step, Newton 100회, no-slip 20회와 단단한 contact impedance를
사용한다.

이 backend의 기본값은 `scenes/handeye.xml.in`이다. 전용 scene은 canonical MJCF를
그대로 include하고 `chassis`를 world에 weld하며, 고정 table/stand와
7×5 ChArUco target을 추가한다. 기존 `mujoco_sim.launch.py`와 custom simulator의
기본 `scenes/default.xml.in` 경로는 그대로 유지된다.

Scene 계약과 printable board provenance는
`config/handeye_scene.yaml` (`cleany.handeye_scene/v1`)에 기록한다. Simulation은
30 mm square/15 mm marker nominal 값을 사용한다. Physical profile의 실측값은 현재
`not_measured`/`null`이며 임의 수치로 대체하지 않는다. 실제 인쇄물을 100% scale,
page fitting 비활성화로 출력하고 실측 provenance를 채우기 전에는 physical
preflight가 실패한다.

같은 manifest는 wrist camera render/public contract도 고정한다. 640×480,
vertical FOV 93°에서
`f=(height/2)/tan(fovy/2)=227.751496 px`, `cx=319.5`, `cy=239.5`를 사용한다.
Public `CameraInfo`는 `plumb_bob`, 5개 zero `D`, identity `R`, manifest의 exact
`K/P`를 사용한다. Width/height/FOV/formula/K/D/R/P 중 하나라도 바뀌면 simulation
preflight가 실패하고 adapter가 collection message를 발행하지 않는다.

```bash
ros2 run cleany_mujoco_sim handeye_scene_preflight --profile simulation
# 실측값 기록 전에는 의도적으로 exit 2
ros2 run cleany_mujoco_sim handeye_scene_preflight --profile physical
```

Printable SVG/PDF는 OpenCV 4.5.4 `CharucoBoard`의 7×5,
`DICT_5X5_100`, marker IDs 0–16 패턴을 같은 lossless vector run으로 표현한다.
파일별 SHA-256과 210×150 mm media 크기는 manifest에 고정되어 있다. Target GT는
OpenCV 4.5 object frame(인쇄면 좌하단 원점, +X 오른쪽, +Y 위, +Z 인쇄면 밖)의
`base_T_target`이며 평가 전용이다. PnP 후보 선택이나 solver 입력에는 사용하지 않는다.

전용 raw action/runtime 계약은 다음으로 검증한다.

```bash
cd ros2_ws
pytest -q -s \
  src/cleany_mujoco_sim/test/test_handeye_backend_runtime.py \
  src/cleany_mujoco_sim/test/test_handeye_camera_runtime.py
```

## ROS contract

### Custom simulation backend

`mujoco_sim_node`는 다음 topic을 발행한다.

- `joint_states` (`sensor_msgs/JointState`): drive-wheel joint만 노출하며 passive
  mecanum roller DOF는 내부에 유지한다.
- `odom` (`nav_msgs/Odometry`)
- `scan` (`sensor_msgs/LaserScan`)
- `publish_odom_tf=true`일 때 `tf` (`odom` -> `base_link`)
- laser scan이 활성화됐을 때 `tf_static` (`base_link` -> `laser`)

다음 topic을 구독한다.

- `~/joint_cmd` (`sensor_msgs/JointState`): controller가 아닌 시뮬레이션 시험용
  목표 관절 위치 명령
- `cmd_vel` (`geometry_msgs/msg/Twist`): 기본 namespace에서는 `/cmd_vel`로
  노출되는 mobile-base 차체 속도 명령

`cmd_vel`은 [`cleany_interfaces` mobile-base contract](../cleany_interfaces/docs/mobile_base.md)를
따른다. 지원 축은 `linear.x`, `linear.y`, `angular.z`이며, 잘못된 값은 거부하고
축별 속도 제한과 command timeout 정지를 적용한다. 검증된 차체 속도는 메카넘
역기구학으로 네 휠의 목표 각속도로 변환된다.

`rgbd_pick_demo.launch.py`의 `mujoco_rgbd_sim_node`는 추가로 color-aligned
`camera/color/image_raw`, `camera/depth/image_raw`, 두 CameraInfo와 평가 전용
`ground_truth/objects`를 같은 timestamp로 발행한다.

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

발행 중에는 전진·좌측 횡이동·반시계 회전이 함께 적용되고, 발행이 중단되면
command timeout 후 정지 목표를 적용한다.

### Hand-eye arm-control backend

`handeye_backend.launch.py`는 `mujoco_ros2_control/ros2_control_node`,
`robot_state_publisher`, `joint_state_broadcaster`와 side별
`joint_trajectory_controller`, camera contract adapter를 시작한다.

- `/left_arm_controller/follow_joint_trajectory`: left arm 5축만 claim
- `/right_arm_controller/follow_joint_trajectory`: right arm 5축만 claim
- `/joint_states`: 양팔 10축 position/velocity와 left/right gripper state. 기본
  hand-eye launch에서는 gripper가 read-only이고, can pre-grasp 데모는
  `enable_gripper_controllers:=true`로 side별 trajectory action을 추가한다.
- `/left_wrist_camera/image_raw`: 640×480 RGB, source simulation stamp,
  `left_wrist_rgb_optical_frame`
- `/left_wrist_camera/camera_info`: image와 같은 source stamp/frame 및 manifest의
  exact pinhole model

그리퍼는 MoveIt current-state completeness를 위해 상태만 내보내며 command
interface는 없다. 이 backend는 private `~/joint_cmd` topic을 만들거나 사용하지
않는다. Controller의 joint path/goal tolerance baseline은 각각 `0.05 rad`와
`0.01 rad`이며 `config/handeye_ros2_controllers.yaml`에서 관리한다. Grasp demo는 이
baseline을 바꾸지 않고 `config/grasp_demo_ros2_controllers.yaml`을 명시적으로 선택한다.
전용 profile의 arm path tolerance는 `0.12 rad`이고 gripper goal time은 3초다.
실제 can 최종 접근에서 shoulder tracking error가 기존 `0.08 rad` 경계를 반복적으로
약 `0.0008 rad` 초과한 측정 결과를 반영한 simulation 전용 값이다. hand-eye와 실제
로봇 controller 설정에는 적용하지 않는다.

ROS 2 Humble binary의 `mujoco_ros2_control` 0.0.3은 MuJoCo 3.4를 vendor하므로
canonical model의 MuJoCo 3.7 `dcmotor`를 읽을 수 없다. Default `.xml.in` scene을
hand-eye backend로 실행할 때 scene loader가 임시 model copy를 만들고 네 wheel
`dcmotor` actuator와 그 default만 제거한다. 이 임시 copy에는 0.0.3 plugin이
control contract 밖의 head actuator까지 finite command로 초기화하도록 startup
keyframe도 추가한다. Canonical MJCF와 기존 `mujoco_sim_node` materialization은
변경하지 않는다. 직접 `scene_path`에 완성된 `.xml`을 넘기면 호출자가 MuJoCo 3.4
호환성과 `handeye_ros2_control_home` keyframe을 보장해야 한다.

동일한 0.0.3 release에는 별도 `CameraPlugin`이 없다. 이 구현은 release에 실제로
포함된 `mujoco_ros2_control::MujocoCameras`가 `hardware_info.sensors`의
`frame_name`, `info_topic`, `image_topic`, `depth_topic`을 읽는 경로를 사용한다.
선택한 hardware sensor의 `/<camera_name>/*` 이름은 launch remap으로
`/cleany/internal/mujoco/left_wrist_camera/*` 아래에 격리되고, 작은 adapter만 위의
두 public topic을 발행한다. Adapter는 vendor image와 CameraInfo를 exact source
stamp로 pair한 뒤 public frame/model을 정규화하며 wall clock stamp를 만들지 않는다.
Hand-eye template marker가 있는 경우에만 scene loader가 임시 canonical include의
`left_wrist_rgb` resolution을 640×480으로 materialize한다. Source canonical MJCF,
default scene, custom simulator, Gazebo 경로의 bytes/동작은 바꾸지 않는다.

Simulation camera GT는 manifest의 `evaluation_ground_truth.camera_transform`과
`ground_truth_evaluation.py` pure accessor/metric에만 존재한다. 이는 compiled
`Fixed_Jaw` body에서 `left_wrist_rgb_optical_frame` site까지의 transform이며 solver
입력이나 canonical TF/topic으로 publish되지 않는다.

### RGB-D pick-demo backend

`rgbd_pick_demo.launch.py`의 `mujoco_rgbd_sim_node`가 추가로 발행하는 토픽:

- `camera/color/image_raw` (`sensor_msgs/Image`): `640x480`, `rgb8`
- `camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `camera/depth/image_raw` (`sensor_msgs/Image`): color에 정렬된 `640x480`,
  meter 단위 `32FC1`; far-plane과 유효하지 않은 pixel은 `NaN`
- `camera/depth/camera_info` (`sensor_msgs/CameraInfo`)
- `ground_truth/objects` (`cleany_interfaces/DetectedObject3DArray`):
  `base_link` 기준 box/can OBB. 인식 입력이 아닌 정량 평가 전용이다.

한 capture의 RGB, depth, 두 CameraInfo와 GT는 같은 timestamp를 사용한다. RGB와
depth optical frame은 각각 `head_camera_rgb_optical_frame`과
`head_camera_depth_optical_frame`이다. 두 stream은 같은 camera pose와 intrinsics로
렌더링하므로 pixel 정렬되어 있다.

실행 중 계약은 다음처럼 확인할 수 있다.

```bash
ros2 topic list
ros2 topic info /camera/color/image_raw --verbose
ros2 topic info /camera/depth/image_raw --verbose
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/image_raw --once --field encoding
ros2 topic echo /ground_truth/objects --once
```

## 베이스 구동 모델

각 휠은 독립적인 `PG42-4266-1270NE` output-shaft DC motor 모델을 사용한다.
actuator 제어 입력은 `rear_left_drive`, `rear_right_drive`, `front_left_drive`,
`front_right_drive`의 모터 단자 전압이다. `base_link +X`가 전방이며, 양의 yaw는
`+Z` 기준 반시계 방향이다.

- 정격 공급 전압: `12 V`
- 기어박스: `61:1`; 제조사 출력 토크에는 `72%` 효율이 반영되어 있다.
- 제조사 정격 출력: `103 rpm`에서 `2.94 N.m`
- 계산한 무부하 출력: `7000 / 61 rpm` (`12.017 rad/s`)
- 10% 마진을 적용한 시뮬레이션 운용 한계: `10.8 V`, `2.646 N.m`
- 디레이팅한 정격점: `92.7 rpm` (`9.708 rad/s`)
- 디레이팅한 무부하 평형점: `103.28 rpm` (`10.815 rad/s`)

MJCF actuator는 기어박스 출력축에서 직접 모델링하므로(`gear=1`) 감속비와 효율을
다시 적용하지 않는다. 매 physics step마다 실제 joint 속도를 읽고, feed-forward와
anti-windup을 포함한 휠별 PID controller로 `-10.8~10.8 V` 전압을 계산한다.
PID gain은 MuJoCo DC motor step response를 기준으로 한 시뮬레이션 값이며 실제
ESP32 controller에 그대로 사용하지 않는다.

## 팔 서보 모델

양팔은 Feetech 12 V 시리얼 서보를 사용한다.

- left/right shoulder pitch와 elbow pitch: `STS3250`
- shoulder rotation, wrist pitch/roll, jaw, head pan/tilt: `STS3215`

모델의 출력 한계에는 제조사 사양 대비 10% 운용 마진을 적용한다. 위치 actuator와
joint force 한계에는 최대 정지 토크의 90%를 적용한다. 전류, 발열 및 2초 과부하
차단 동작은 아직 모델링하지 않았다.

## 실행 인자

`mujoco_sim.launch.py`는 다음 launch argument를 제공한다.

- `scene_path`: MuJoCo scene XML 또는 `.xml.in` template. 기본값은
  `scenes/default.xml.in`이다.
- `publish_rate_hz`: simulation publish/timer rate. 기본값 `60.0`
- `headless`: viewer를 숨길지 여부. 기본값 `true`
- `scan_rate_hz`: laser scan 발행 주기. 기본값 `5.5`
- `scan_samples`: scan당 ray 수. `0`이면 `scan_sample_rate_hz`에서 계산
- `max_linear_x`, `max_linear_y`, `max_angular_z`: `cmd_vel` 축별 제한값
- `cmd_vel_timeout_sec`, `timeout_check_rate_hz`: 명령 timeout 설정
- `wheel_radius`, `wheelbase_length`, `track_width`, `max_wheel_speed`: 메카넘
  역기구학 설정
- `base_drive_enabled`, `wheel_kp`, `wheel_ki`, `wheel_kd`,
  `motor_voltage_limit`, `motor_no_load_speed`: 휠 drive controller 설정

`handeye_backend.launch.py`는 다음 launch argument를 제공한다.

- `scene_path`: control용 MuJoCo scene XML 또는 `.xml.in` template. 기본값은
  `scenes/handeye.xml.in`
- `headless`: native viewer 비활성화 여부. 기본값 `true`
- `sim_speed_factor`: wall time 대비 simulation speed. 기본값 `1.0`

Camera publish rate와 public model/topic은 launch override가 아니라
`config/handeye_scene.yaml`의 preflight 계약으로 관리한다.

ChArUco의 211개 vector ink box는 target geometry의 authoritative source로
유지한다. 640×480 wrist render에서는 sub-pixel box edge가 ArUco bit를 손상시킬 수
있으므로 `scene_loader`가 같은 box 좌표를 1400×1000 lossless grayscale PNG로
deterministic rasterize하고, materialized temporary scene의 non-collision render
surface에만 적용한다. 보드 바깥 10 mm 흰 quiet zone도 render-only/non-collision이며
210×150 mm object point와 planning-scene collision 형상은 바꾸지 않는다. Canonical
MJCF, `default.xml.in`, SVG/PDF source asset은 수정하지 않으며 temporary texture도
source tree에 기록하지 않는다.

`rgbd_pick_demo.launch.py`:

- `headless`: MuJoCo viewer 표시 여부. 기본값은 `true`다.
- `rgbd_rate_hz`: RGB, depth, CameraInfo와 GT 발행 주기. 기본값은 `5.0 Hz`다.
- launch가 renderer backend를 `MUJOCO_GL=egl`로 설정한다.

`MujocoSimNode`가 지원하는 추가 노드 파라미터:

- `base_body_name`: 로봇 베이스로 사용할 MuJoCo body. 기본값은 `chassis`다.
- `lidar_site_name`: 레이저 원점으로 사용할 MuJoCo site. 기본값은
  `lidar_site`다.
- `odom_frame_id`: 오도메트리 frame ID. 기본값은 `odom`이다.
- `base_frame_id`: 베이스 frame ID. 기본값은 `base_link`다.
- `laser_frame_id`: 레이저 frame ID. 기본값은 `laser`다.
- `publish_odom_tf`: 동적 odom-to-base transform 발행 여부. 기본값은 `true`다.
- `scan_enabled`: `scan`과 정적 레이저 transform 발행 여부. 기본값은 `true`다.
- `scan_sample_rate_hz`: `scan_samples`가 `0`일 때 사용할 가상 광선 샘플링 주기.
  기본값은 `8000.0`이다.
- `scan_range_min`: 유효한 최소 스캔 거리. 기본값은 `0.15`다.
- `scan_range_max`: 유효한 최대 스캔 거리. 기본값은 `12.0`이다.
- `initial_joint_names`: 시작할 때 설정할 actuator-backed scalar joint 이름 배열.
  기본값은 빈 배열이다.
- `initial_joint_positions`: `initial_joint_names`와 같은 순서의 초기 위치(rad 또는
  prismatic joint의 m) 배열. 기본값은 빈 배열이다. 두 배열이 비어 있으면 MJCF의
  초기 상태를 그대로 사용한다. 값은 유한해야 하고 관절 제한 안에 있어야 한다.

예를 들어 시작할 때 head를 아래로 기울이려면 node parameter YAML에 다음을 둔다.

```yaml
mujoco_sim:
  ros__parameters:
    initial_joint_names: [head_tilt_joint]
    initial_joint_positions: [1.0]
```

## 시뮬레이션 확장 API

같은 process의 sensor adapter는 `MujocoSimNode.simulation_context`에서
`MujocoSimulationContext`를 받아 MuJoCo model과 최신 data를 조회할 수 있다. 두
native handle은 렌더링 등 MuJoCo API 호출에 직접 사용할 수 있지만 adapter가 물리
상태를 변경하지 않는 read-only 계약을 따른다.

`MujocoSimNode.add_step_observer(observer)`에 `StepObserver` 구현을 등록하면 각 ROS
timer tick의 모든 물리 substep이 끝난 직후 한 번 `after_step(context, stamp)`가
호출된다. callback은 기존 ROS 상태 토픽 발행 전에 동기적으로 실행되며 예외를
숨기지 않는다. observer는 callback을 짧게 유지해야 한다.

## 범위

구현된 기능:

- `cleany_description`의 authoritative MJCF를 simulator-owned scene에 load
- 독립적인 PG42 motor를 사용하는 네 개의 5-inch mecanum drive wheel 시뮬레이션
- ROS timer로 simulator step, joint state·odometry·laser scan·TF 발행
- 시뮬레이션 시험용 직접 joint position 명령
- `/cmd_vel` 검증·제한·timeout과 메카넘 역기구학
- 휠별 PID, feed-forward, anti-windup, 전압 제한을 통한 actuator 제어
- 좌·우 5축별 표준 `FollowJointTrajectory` action과 12개 arm/gripper joint state를
  제공하는 배타적인 MuJoCo `ros2_control` backend
- Humble release `MujocoCameras` 기반 left wrist RGB/CameraInfo와 public contract
  normalizer
- 고정 pick-demo scene의 pixel-aligned RGB-D 동기 렌더링
- 평가 전용 ground-truth topic의 simulation object OBB 발행

아직 구현되지 않은 기능:

- `cleany_robot_interface`, `cleany_perception` 또는 Mission FSM 연동
- 실제 encoder noise, 전류, 열, 과부하 차단 동작
- 베이스 또는 매니퓰레이터용 하드웨어 현실성 기반 controller

## 공통 RGB-D 파이프라인 연결

`handeye_backend.launch.py`에 `color_image_topic`, `camera_info_topic`,
`depth_image_topic`을 명시하면 선택 MJCF camera 출력을 해당 ROS 토픽으로
직접 remap한다. 비워 두면 기존 hand-eye manifest의 internal 토픽을 유지한다.
custom 토픽 사용 시 `enable_camera_contract_adapter:=false`가 필요하다.
이 옵션은 영상 전달 경계만 바꾸며 simulator geometry/물체 정답을 알고리즘에
주입하지 않는다. head 고정 camera 보정 TF의 실제 로봇 정확성을 뜻하지 않는다.

레포 루트의 `make sim-mujoco-pipeline`은 학습 perception과 sensor-only
MoveIt 지도를 포함하는 plan-only GUI 실행이다. 장면만 띄우는
`make sim-mujoco-study-cafe` 및 legacy 테스트 launch와 구분한다.

## 관련 KB와 문서 갱신

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Navigation and Mapping](../../../docs/cleany-docs/20_TECHNICAL/05%20-%20Navigation%20and%20Mapping.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

### 단일 후면 수거함 fixture (2026-09-08)

책상 4분할 및 이전 좌우 수거함 YAML을 삭제했다. 책상 구분선 생성 코드도
제거했으며 `robot_top_bins.yaml`만 기본 배치로 사용한다. 과거 실행 기록은
진단 이력으로 보존하지만 새 실행의 장면 입력으로 사용하지 않는다.
발행 topic, launch parameter, 시뮬레이션 모델 가정 또는 테스트 명령이 바뀌면 이
README도 갱신한다. 시뮬레이션 하드웨어 파라미터를 관련 KB 결정 없이 확정된 실제
하드웨어 사양으로 표현하지 않는다.
