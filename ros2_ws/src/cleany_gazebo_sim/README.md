# cleany_gazebo_sim

Gazebo Fortress 기반의 Cleany mobile-base simulation backend입니다. 이 패키지는
MuJoCo의 motor-voltage dynamics를 복제하지 않고, ROS 차체 속도 계약의
`cmd_vel -> odom / TF` 경계를 headless에서 검증하는 데 초점을 둡니다.

## Scope

- Gazebo `MecanumDrive` system을 이용한 `linear.x`, `linear.y`, `angular.z` 이동
- `cmd_vel` 유효성 검사, 속도 제한, command timeout 정지
- `/clock`, `/odom`, `/joint_states`, `odom -> base_link` TF bridge
- MuJoCo의 head RGBD와 좌·우 wrist RGB camera image bridge
- 현재 MuJoCo 모델의 논리적인 4-wheel 배치와 joint 이름을 따르는 prototype

Gazebo world는 `cleany_mujoco_sim/hardware/assets/`를 resource path로 참조해 팀의
XLeRobot/RASKOG base, dual-arm, gripper visual mesh를 재사용합니다. arm/gripper의
joint pose, axis, limit과 mass/center-of-mass/full inertia tensor, convex collision
mesh도 MuJoCo body tree에서 가져왔습니다. 다만 arm/gripper controller가 아직 없어
arm link의 gravity는 비활성화한 상태입니다. camera, LiDAR, Nav2, MoveIt, Mission
Manager integration은 아직 포함하지 않습니다.

## Dependencies

Ubuntu 22.04 / ROS 2 Humble에서 Gazebo Fortress와 ROS bridge가 필요합니다.
저장소 루트에서 rosdep으로 workspace 의존성을 설치합니다.

```bash
./scripts/rosdep install --from-paths ros2_ws/src --ignore-src -r -y
```

## Run

ROS 환경을 source한 native terminal에서 실행합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_sim.launch.py headless:=true
```

다른 terminal에서 명령을 보냅니다.

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

`/clock`, `/odom`, `/tf`와 guard output `/gazebo_cmd_vel`을 확인할 수 있습니다.
GUI는 `headless:=false`로 켤 수 있지만 WSLg/OGRE renderer 호환성은 host 환경에
따라 별도로 확인해야 합니다.

## Validation

```bash
python3 -m pytest src/cleany_gazebo_sim/test/test_parameters.py
colcon build --packages-select cleany_gazebo_sim --symlink-install
```
