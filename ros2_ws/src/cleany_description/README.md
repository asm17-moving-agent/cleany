# cleany_description

## URDF migration

`feat/migrate-cleany-description`의 형상 xacro와 참조 CAD frame 메시를 가져왔다.
`physical_properties.xacro`는 본체·팔·바퀴의 상세 visual/collision을 제공한다.
현재 브랜치의 두 control xacro는 카메라 스케줄링, hardware plugin 및 초기
관절값 설정 호환성을 위해 유지한다. 대응 MJCF도 같은 브랜치에서 가져왔으며,
스터디카페 접촉 pair 호환성을 위해 양팔 jaw collision geom 이름을 유지한다.
정합성 검사는 새 CAD 모델의 팔·TCP·그리퍼·헤드 카메라 좌표를 기준으로 한다.
모델 정합성 통과가 물체 파지나 전체 수거 성공을 보장하지는 않는다.
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

control xacro의 `scheduled_cameras`(기본 false)는 sorting 전용 hardware에
카메라별 촬영 스케줄러 사용 여부를 전달한다. 로봇 mesh/카메라 장착 형상은 변경하지 않는다.

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

## Contents

- `urdf/cleany_geometry.xacro`: backend-neutral canonical physical model
- `urdf/cleany.urdf.xacro`: plugin-free dual-arm URDF used by
  `robot_state_publisher` and MoveIt
- `urdf/cleany_control.urdf.xacro`: control-backend extension entrypoint; it
  adds the MuJoCo `ros2_control` hardware interface for the ten arm joints and
  read-only state interfaces for both grippers
- `urdf/cleany_mujoco_ros2_control.xacro`: reusable MuJoCo hardware macro
- `urdf/head_camera.xacro`: nominal head pan/tilt and RGB-D frame tree
- `mjcf/cleany.xml`: MuJoCo robot model included by simulator scenes
- `meshes/`: visual and collision CAD assets referenced by both descriptions
- `test/test_model_parity.py`: canonical joint, motor geometry, and randomized
  FK parity checks

Both URDF entrypoints include the five separate motor CAD meshes on each arm
as visuals and URDF collision shapes, at their MJCF body-local placements. Upper-arm
and lower-arm motors have respective offsets `(0, 0.05, 0)` and `(0, 0, 0.05)` m;
the other three have zero offsets. The RGB-D self-filter uses collision shapes,
so omitting these parts can leave the robot's own surfaces in the environment
OctoMap. The fixed-jaw motor and the two rigid wrist-camera meshes remain
visual-only in MJCF, but have matching URDF collision meshes for conservative
planning and self-filter coverage. The migration initially omitted these URDF
collisions; they are restored using the exact visual origins/scales, without
changing MuJoCo contact physics. Regression checks compare both visual and
collision mesh scale and composed camera mesh poses
within `1e-5`; motor translation comparisons allow `2e-6` m for CAD rounding.
Existing CAD files are reused without regenerating assets or changing joint
kinematics. This coverage does not by itself prove collision-free execution.

## Coordinate and rotation convention

The MJCF moving-jaw body frames use the migrated URDF joint RPY `(pi, 0, 3.33)`
as quaternions. The visual/collision origins retain the CAD's approximately
21.59-degree compensation, and inertial properties use the same link frame.
Changing only the body rotation without this mesh compensation would alter
the contact geometry. Tests check both visual and collision mesh poses.
Randomized FK parity checks include both moving jaws over
arm and gripper configurations in both URDF entrypoints, in addition to the
fixed jaws and TCPs. The `1e-5` tolerance accommodates rounded MJCF CAD angles;
this is simulation-model parity, not a measured hardware calibration.

Run `make test-grasp-pregrasp` from the repository root to build the relevant
packages and check the entire model parity suite, including randomized arm/TCP,
moving-jaw and head-camera FK. The simulation tests also compare the configured
wrist optical transforms against the MJCF camera sites. URDF entrypoint geometry
is compared with `include_wheel_joints:=false` on both sides: the default mobile
description has continuous wheels, while the stationary arm-control and MoveIt
sorting descriptions use fixed wheels because their controllers do not publish
wheel states. These tests do not validate real-hardware calibration, Jetson
performance, navigation, or successful object retention.

The descriptions follow ROS REP-103:

- right-handed `base_link`: `+X` forward, `+Y` left, `+Z` up
- positive rotation follows the right-hand rule
- roll, pitch, and yaw rotate about `+X`, `+Y`, and `+Z`
- positive yaw is counter-clockwise when viewed from above
- wheel axes are `+Y`, so positive wheel rotation drives toward `+X`
- `_optical_frame` axes are `+X` right, `+Y` down, `+Z` forward

The public mobile-base joint contract contains only
`rear_left_wheel_joint`, `rear_right_wheel_joint`,
`front_left_wheel_joint`, and `front_right_wheel_joint`. MuJoCo retains named
passive roller degrees of freedom internally for contact physics. The names
allow `MujocoSystemInterface` to validate the MJCF, but these joints are not
listed in `ros2_control`, are not commandable, and are not published in the
control backend's `joint_states`.

The default head camera points toward `base_link +X`. Physical `+Y` is the
canonical left arm and physical `-Y` is the canonical right arm.

The nominal head camera frame tree is shared by URDF and MJCF:

```text
base_link
└── top_base_link
    └── head_pan_link
        └── head_tilt_link
            └── head_camera_link
                ├── head_camera_rgb_frame
                │   └── head_camera_rgb_optical_frame
                └── head_camera_depth_frame
                    └── head_camera_depth_optical_frame
```

RGB and aligned depth use colocated nominal optical origins. These fixed
transforms describe the current simulation assembly; they are not a measured
RealSense calibration. A real deployment must validate or replace them with
its calibration profile while preserving the public frame contract.

The nominal head RGB-D optical frames in the plugin-free description support
the perception demo. The arm-control entrypoint omits that head tree so its
current-state contract remains exactly ten arm plus two gripper joints.
Hand-eye evaluation uses its separately governed left-wrist camera profile.
MoveIt's real-backend launch expands `cleany.urdf.xacro` with
`include_head_camera:=false`; the regular description launch keeps the default
`true` so the perception-side `robot_state_publisher` can provide the head
camera TF tree. Detection results must be transformed into the configured
planning frame before they are passed to MoveIt.

Each arm exposes `${side}_grasp_tcp` as a fixed frame and MuJoCo site at
`(0, -0.100, 0) m` in `${side}_gripper_frame`. It is a nominal point near the
center of the jaw tips for position-only IK. Its orientation inherits the
gripper frame and is not a calibrated grasp orientation.

`cleany_control.urdf.xacro` registers the `left_wrist_rgb` MJCF camera as a
`ros2_control` sensor for the hand-eye MuJoCo backend. Its vendor topic names
and 10 Hz render rate are consumed by Humble `mujoco_ros2_control` 0.0.3; the
simulation package launch owns remapping and the public camera contract.

Publish the description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```

The default `cleany.urdf.xacro` remains plugin-free. The MuJoCo control
backend expands `cleany_control.urdf.xacro` with the materialized scene path
and runtime options:

```bash
xacro urdf/cleany_control.urdf.xacro \
  mujoco_model:=/absolute/path/to/control_scene.xml \
  headless:=true \
  sim_speed_factor:=1.0
```

The control entrypoint exposes position commands plus position and velocity
state for the five joints of each arm. Both gripper joints expose read-only
position and velocity state so MoveIt receives a complete dual-arm model
state; they have no command interface. The base, head, and passive roller
joints remain outside this control contract.

## Integration regression scope

Model-parity checks compare geometry and FK; they do not validate every existing
motion fixture after a CAD migration. Run `make test` with a working X display
for controller, planning and hand-eye regressions as well. Current findings and
fixture migration results and remaining dynamics issues are recorded in the
[quality review](../../../docs/QUALITY_REVIEW_20260914.md).
