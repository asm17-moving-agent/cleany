# cleany_gazebo_sim

Gazebo Fortress 기반의 Cleany mobile-base simulation backend입니다. 이 패키지는
MuJoCo의 motor-voltage dynamics를 복제하지 않고, ROS 차체 속도 계약의
`cmd_vel -> odom / TF` 경계를 headless에서 검증하는 데 초점을 둡니다.

## Scope

- Gazebo `MecanumDrive` system을 이용한 `linear.x`, `linear.y`, `angular.z` 이동
- `cmd_vel` 유효성 검사, 속도 제한, command timeout 정지
- `/clock`, `/odom`, `/joint_states`, `odom -> base_link` TF bridge
- MuJoCo의 head RGBD와 좌·우 wrist RGB camera image bridge
- `base_link +X`를 camera-forward 전면으로 사용하는 canonical 4-wheel/arm 배치
- controller UI와 `joint_states`에는 4개의 drive wheel joint만 노출

좌표와 회전은 ROS REP-103을 따릅니다. `base_link`는 `+X` 전방, `+Y` 좌측,
`+Z` 상방인 오른손 좌표계이며 양의 yaw는 위에서 볼 때 반시계 방향입니다.
wheel joint axis는 `base_link +Y`로 명시되어 양의 wheel 회전이 `+X` 전진을
뜻합니다. camera image의 frame id는 REP-103 `_optical_frame`
(`+X` right, `+Y` down, `+Z` forward) 이름을 사용합니다.

각 wheel은 12개의 고정 capsule roller visual을 사용합니다. 실제 접촉은 Gazebo
Fortress의 mecanum 예제와 같은 diagonal anisotropic friction sphere로 단순화해,
48개의 passive roller joint를 GUI나 ROS interface에 노출하지 않습니다.

Gazebo world는 `cleany_description/meshes/`를 resource path로 참조해 팀의
Cleany/RASKOG base, dual-arm, gripper visual mesh를 재사용합니다. arm/gripper의
joint pose, axis, limit과 extended-link mass/center-of-mass/full inertia tensor,
collision mesh도 Cleany description에서 가져왔습니다. 다만 arm/gripper controller가 아직
없어 arm link의 gravity는 비활성화한 상태입니다. camera, LiDAR, Nav2, MoveIt,
Mission Manager integration은 아직 포함하지 않습니다.

## Dependencies

Ubuntu 22.04 / ROS 2 Humble에서 Gazebo Fortress와 ROS bridge가 필요합니다.
저장소 루트에서 rosdep으로 workspace 의존성을 설치합니다.

```bash
make deps
```

## Run

저장소 루트에서 실행합니다.

```bash
make sim-gazebo
```

다른 terminal에서 명령을 보냅니다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

`/clock`, `/odom`, `/tf`와 guard output `/gazebo_cmd_vel`을 확인할 수 있습니다.
GUI는 `headless:=false`로 켤 수 있지만 WSLg/OGRE renderer 호환성은 host 환경에
따라 별도로 확인해야 합니다.

## Validation

저장소 루트에서 실행합니다.

```bash
make test-gazebo
```

세부 옵션이 필요하면 `ros2_ws/README.md`의 native 명령을 사용합니다.
