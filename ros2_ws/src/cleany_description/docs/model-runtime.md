# cleany_description

## URDF migration

`feat/migrate-cleany-description`의 형상 xacro와 참조 CAD frame 메시를 가져왔다.
`physical_properties.xacro`는 본체·팔·바퀴의 상세 visual/collision을 제공한다.
현재 브랜치의 두 control xacro는 카메라 스케줄링, hardware plugin 및 초기
관절값 설정 호환성을 위해 유지한다. 대응 MJCF도 같은 브랜치에서 가져왔으며,
스터디카페 접촉 pair 호환성을 위해 양팔 jaw collision geom 이름을 유지한다.
정합성 검사는 새 CAD 모델의 팔·TCP·그리퍼·헤드 카메라 좌표를 기준으로 한다.
모델 정합성 통과가 물체 파지나 전체 수거 성공을 보장하지는 않는다.

양팔 하단 `Base.stl`의 장착 위치는 원형 마운트
`cad_frame/035armbase_step__êäèë4.stl`의 체결 구멍 4개에 맞춘다.
기존 배치 대비 `base_link` 기준 이동량은 왼쪽
`(-0.099657, +11.114727, 0)` mm, 오른쪽
`(+0.099838, -11.114072, 0)` mm다. 팔 하단과 모터의 visual/collision,
어깨 관절 원점, MJCF의 전체 팔 body에 같은 이동량을 적용하고,
URDF에 합쳐진 고정 팔 하단의 질량 중심과 관성도 다시 계산했다.
수정 후 어깨 원점은 왼쪽 `(0.116200343, 0.196231727, 0.447797)` m,
오른쪽 `(0.116200838, -0.196177072, 0.447797)` m다.
이 수치는 기존 CAD 메시의 XY 체결 정렬값이며 실물 보정값이 아니다.
높이와 회전은 기존 값을 유지한다.
`test_arm_base_fasteners_align_with_cad_mounts`는 메시 좌표계에서 측정한
구멍 축을 각 배치로 변환해 XY 오차가 0.01 mm 이내인지 검사한다.
URDF/MJCF의 같은 오배치가 단순 모델 일치 검사를 통과하는 문제를 방지한다.

`base_link`의 고정 조립체 관성은 MJCF에 기록된 `chassis`, `Base`, `Base_2`의
질량·질량 중심·`fullinertia`를 같은 좌표계로 옮겨 평행축 정리로 합성한다.
관성 정합성 검사는 이 원본 텐서를 기준으로 절대 오차 `1e-9`를 유지하며,
MuJoCo 버전별 주관성축 분해의 수치 차이가 합성 기준에 섞이는 것을 방지한다.

`include_head_camera:=false`로 카메라 관절을 제외해도 고정 기둥
`top_base_link`/`top_base_joint`는 유지한다. MoveIt의 팔 전용 모델에서 기둥이
누락되어 복귀 중 팔이 실제 기둥에 부딪히던 문제를 방지한다.
움직이는 pan/tilt 헤드 전체의 충돌 반영은 해당 관절 feedback 연결이 별도 필요하다.
MJCF의 거친 head mount box가 pan joint를 사이에 둔 tilt housing과 겹쳐
카메라를 밀어내므로 `top_base_link`–`head_tilt_link` 한 쌍만 내부 조립체
접촉에서 제외한다. 헤드와 외부 환경의 충돌은 유지하며 head pose 유지 시험으로
검증한다. 이는 CAD 충돌 근사의 예외이지 실물 간섭이 없다는 보증은 아니다.
기본 description의 바퀴 관절은 continuous이며 고정 미리보기에서는
`include_wheel_joints:=false`로 확장할 수 있다.

control xacro의 `scheduled_cameras`(기본 false)는 observer hardware에 카메라별
촬영 스케줄러 사용 여부를 전달한다. `efficient_viewer`(기본 false)는 같은 worker에서
GUI도 그릴지 지정한다. launch가 이 GUI를 선택하면 vendor `headless=true`를 함께
전달한다. `efficient_viewer`는 hardware parser에 맞춰 소문자 true/false로 출력한다.
로봇 mesh/카메라 장착 형상은 변경하지 않는다.

Sorting의 analytical wrist-roll 후보 보정은 `wrist_roll_joint`의 local -Y
축과 `grasp_tcp_joint`의 `(0, -0.100, 0)` offset이 공선이라는 현재 모델
계약을 사용한다. `test_grasp_roll_alignment_matches_collinear_robot_geometry`
검사가 양쪽 URDF에서 이 전제를 확인한다. 축/도구 보정을 바꿀 때는 이
보정 알고리즘도 함께 검토해야 하며, 현재 TCP는 실측 하드웨어 보정값이 아니다.

Control xacro의 `mujoco_hardware_plugin` 기본값은 기존
`mujoco_ros2_control/MujocoSystemInterface`다. 분리 수거 backend는 이를
`cleany_mujoco_observer/ObservedMujocoSystem`으로 지정해 잠금된 평가 snapshot을
추가한다. 관절/command interface, controller 및 physics 설정은 바뀌지 않는다.

Authoritative robot description assets shared by MuJoCo, TF, and MoveIt.

## Files

- `urdf/cleany_geometry.xacro`: backend-neutral canonical physical model
- `urdf/cleany.urdf.xacro`: plugin-free dual-arm URDF used by
  `robot_state_publisher` and MoveIt
- `urdf/cleany_control.urdf.xacro`: control-backend extension entrypoint; it
  adds the MuJoCo `ros2_control` hardware interface for the ten arm joints and
  read-only state interfaces for both grippers
- `urdf/cleany_mujoco_ros2_control.xacro`: reusable MuJoCo hardware macro
- `urdf/head_camera.xacro`: physical head mount and optional measured RGB optical transform
- `mjcf/cleany.xml`: MuJoCo robot model included by simulator scenes
- `meshes/`: visual and collision CAD assets referenced by both descriptions
- `test/test_model_parity.py`: canonical joint, motor geometry, and randomized
  FK parity checks

Both URDF entrypoints include the five separate motor CAD meshes on each arm
as visuals and URDF collision shapes, at their MJCF body-local placements. Upper-arm
and lower-arm motors have respective offsets `(0, 0.05, 0)` and `(0, 0, 0.05)` m;
the other three have zero offsets. The RGB-D self-filter uses collision shapes,
so omitting these parts can leave the robot's own surfaces in the environment
OctoMap. The fixed-jaw motor remains visual-only in MJCF and has a matching URDF
collision mesh. The v8 wrist mounts use six visuals and fifteen collision
primitives per wrist in both backends, replacing the old camera-case meshes.
Their rigid geometry is attached to the gripper independently of optical TFs.
The simulated wrist cameras and optical sites/bodies use the v8 geometry
registration's lens centre and CV axes. The camera renderer reverses optical
Y/Z to use its -Z viewing convention. Sorting's nominal wrist transform config
matches this pose; the measured Jetson calibration profile remains independent.
Regression checks compare the original URDF collision model with MJCF, including
poses, dimensions and colors. The planning model separately preserves visuals,
inertia and joints; both original and optimized motor collision placements are
checked against MJCF. Motor translation comparisons allow `2e-6` m for CAD rounding.
Existing CAD files are reused without regenerating assets or changing joint
kinematics. This coverage does not by itself prove collision-free execution.

## Model contract

The MJCF moving-jaw body frames use the migrated URDF joint RPY `(pi, 0, 3.33)`
as quaternions. The visual/collision origins retain the CAD's approximately
21.59-degree compensation, and inertial properties use the same link frame.
Changing only the body rotation without this mesh compensation would alter
the contact geometry. Tests check both visual and collision mesh poses.
Randomized FK parity checks include both moving jaws over
arm and gripper configurations in both URDF entrypoints, in addition to the
fixed jaws and TCPs. The `1e-5` tolerance accommodates rounded MJCF CAD angles;
this is simulation-model parity, not a measured hardware calibration.

Run `make test` from the repository root to build the workspace and check
the model parity suite, including randomized arm/TCP,
moving-jaw and head-camera FK. The simulation tests also compare the configured
wrist optical transforms against the MJCF camera sites. URDF entrypoint geometry
is compared with `include_wheel_joints:=false` on both sides: the default mobile
description has continuous wheels, while the stationary arm-control and MoveIt
sorting descriptions use fixed wheels because their controllers do not publish
wheel states. These tests do not validate real-hardware calibration, Jetson
performance, navigation, or successful object retention.

- REP-103: `base_link` +X forward, +Y left, +Z up; wheel axes +Y.
  Optical frames use +X right, +Y down, +Z forward.
- Wheelbase **0.35 m**, track **0.6038 m**, wheel radius **0.0635 m**.
  URDF wheels are rigid 500 g assemblies; 48 passive roller joints are MJCF-only.
- `${side}_grasp_tcp` is fixed at `(0, -0.100, 0)` m in
  `${side}_gripper_frame`; the original hinged gripper is retained.
- Head RGB-D transforms are nominal by default. The explicit `jetson_20261009`
  calibration profile applies provisional measured RGB and wrist optical transforms.
  Depth remains nominal; simulation ground-truth wrist edges remain separate.
  See [model contract](model.md) and [calibration guide](../../../../docs/CALIBRATION.md).
- The control entrypoint commands ten arm joints and reads both grippers.
  Gripper commands are opt-in via `enable_gripper_command:=true`.
  Base, head, and rollers are outside this control contract.
- Arm-only MoveIt uses `include_head_camera:=false include_wheel_joints:=false`.
  These default to `true` in the full description; disabling wheel joints
  retains their geometry as fixed links.
- Fourteen legacy servo position actuators and four PG42 voltage actuators
  remain active. New servo DC-motor defaults are commented out pending the
  separate MuJoCo/ros2_control upgrade. The 3.4 backend removes only wheel
  actuators from its temporary scene; see [simulation setup](../../cleany_mujoco_sim/docs/scenes.md).

Mass properties are estimates: total **15.933871 kg**,
including a provisional **5.20 kg, 195×165×175 mm battery**, **0.8 kg per PG42
motor**, and estimated PLA parts.

## Use

After building and sourcing `ros2_ws`, publish the full description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```

Expand the control description from this package directory:

```bash
xacro urdf/cleany_control.urdf.xacro \
  mujoco_model:=/absolute/path/to/control_scene.xml headless:=true
```

The control entrypoint exposes position commands plus position and velocity
state for the five joints of each arm. Both gripper joints expose position and
velocity state so MoveIt receives a complete dual-arm model state. Gripper
commands are disabled by default and enabled with `enable_gripper_command:=true`
for sorting. The base, head, and passive roller joints remain outside this
control contract.

## Integration regression scope

Model-parity checks compare geometry and FK; they do not validate every existing
motion fixture after a CAD migration. Run `make test` with a working X display
for controller, planning and hand-eye regressions as well. Current findings and
fixture migration results and remaining dynamics issues are recorded in the
[quality review](../../../../docs/QUALITY_REVIEW_20260914.md).

Run geometry, color, mass, joint-contract, and randomized FK checks from `ros2_ws`:

```bash
python3 -m pytest src/cleany_description/test/test_model_parity.py
```

## Gazebo navigation adapter

`cleany_gazebo_sim` consumes this CAD description through Xacro / sdformat conversion.
Its navigation profile freezes the arms and head at configured park angles, retains
four driven wheel joints, and attaches Gazebo sensors and mecanum contact approximations.
It does not modify this package's movable joint contract. See that package's README
for the CAD launcher, wheel odometry geometry, and simulation limitations.

## 선택적 충돌 메시 평면화

`optimized_arm_collision_meshes`는 기존 이름을 유지하며 팔, 모터, 머리와 마스트의
경량 충돌 메시를 선택한다. 기본값은 `true`이고 `head_collision_meshes`도
`true`다. 공통 URDF 기본값이므로 Gazebo CAD 변환에도 최적화 충돌 형상이
전달된다. MuJoCo의 물리 접촉 계산은 `mjcf/cleany.xml`의 기존 형상을 유지한다.
시뮬레이션에서도 MoveIt 경로 계획에는 공통 URDF의 최적화 충돌 형상을 사용한다.
원본 URDF와 MJCF의 충돌 형상 대조 검사는 `optimized_arm_collision_meshes:=false`와
`head_collision_meshes:=false`를 모두 명시한다. 기본 계획 모델은 원본 모델과
시각 형상, 관성, 관절이 같은지 별도로 검사한다.
시각 모델, 관성, 관절과 원본 CAD 파일은 바뀌지 않는다.

| 대상 | `true`일 때의 형상 |
|---|---|
| 양팔의 상완, 하완, 연결부와 모터 11종 | `meshes/collision/`의 평면 메시, 좌우 합계 22개 인스턴스 |
| 머리와 마스트 CAD 6종 | `head_collision_meshes=true`이고 해당 링크가 포함될 때 경량 메시 적용 |
| 집게의 기존 볼록 메시 5종 | 이미 12–24개 삼각형으로 구성되어 원본 유지 |
| box, cylinder, sphere primitive | 기존 해석적 형상 유지 |

외접 메시의 각 조각은 담당하는 원본 표면 조각을 포함한다. 하완 포크, 어깨와 손목 연결부,
하완 모터, 마스트와 머리 브래킷은 먼저 평면으로 나누고 떨어진 표면 조각을
분리한다. 이후 각 조각을 지지 평면으로 감싸 불필요한 둥근 모서리와 작은
구멍을 단순화한다. 하완의 기존 8,000면 QEM 예외는 사용하지 않는다.
분할 축과 위치는 `configs/robot/arm_collision_meshes.json`의
`partition_components`에서 원본 메시 좌표계의 미터 단위로 지정한다.
밀리미터 CAD의 단위 변환은 `mesh_scales`가 관리한다.

`enclosure_variants`는 각 조각의 축 선택과 불필요한 지지 평면 제거 방식을
기록한다. STL의 float32 좌표로 반올림한 뒤 볼록 면을 다시 구성하고 원본
포함 및 팽창 상한을 검사한다. 최종 STL의 각 면도 같은 볼록 경계를 이루는지
검사해, 정점의 볼록 외피만 맞고 실제 삼각형은 안으로 접히는 경우를 막는다.

팽창 상한 1mm는 **분할 조각의 볼록 외피**를 기준으로 한다. 오목한 CAD
표면과의 거리 상한이 아니며, 조각 내부의 작은 빈 공간은 메워질 수 있다.
MoveIt은 `cleany/SolidFCL`로 닫힌 볼록 조각을 내부가 채워진 형상으로 읽는다.
삼각형 표면을 가로지르지 않고 조각 안에 완전히 들어간 작은 장애물도 검출한다.
외접 포함 조건뿐 아니라 실제 서비스의 내부 장애물 검출과 원본 CAD 대조를
검사한다. Gazebo와 MuJoCo의 물리 엔진에 이 MoveIt 플러그인을 적용하는 것은 아니다.

`make test-description-models`는 원본 모델의 MuJoCo 동등성, 기본 계획 모델의
공통 정보 보존과 최적화 자산을 검사한다. `test_collision_meshes.py`는 모든 출력 메시의 실제 STL 좌표에서 포함과 팽창
상한, 폐곡면, 분할된 포크의 틈, 단위와 Xacro 적용 범위를 검사한다.
재생성은 `python3 tools/optimize_arm_collision_meshes.py --write`를 명시적으로
실행하며, `--write`가 없으면 생성 결과와 저장된 자산의 일치를 확인한다.
