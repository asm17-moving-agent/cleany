# cleany_base_odometry

4개 Mecanum drive wheel의 누적 회전각으로 planar wheel odometry를 계산합니다.
시뮬레이션은 Gazebo `/joint_states`를 사용하고, 실제 로봇은 MCU encoder adapter가
같은 joint-state 계약을 제공하는 구성을 전제로 합니다.

## ROS 계약

- 입력: `/joint_states` (`sensor_msgs/msg/JointState`)
- 출력: `/wheel/odom` (`nav_msgs/msg/Odometry`)
- pose frame: `odom`
- child/twist frame: `base_link`
- TF: 발행하지 않음

노드는 휠 `position`의 이전 값과 현재 값의 차이를 적분합니다. `velocity` 필드는
필수 입력이 아닙니다. 휠 이름과 반경·wheelbase·wheel separation은
`config/wheel_odometry.yaml`에서 설정합니다.

Gazebo launch는 가상 encoder를 통해 tick 양자화와 선택적인 휠별 scale 및 tick
편차를 적용할 수 있습니다. `wheel_odometry_synthetic_error.yaml`은 nominal 값에서
휠 반경을 +1%, wheelbase와 wheel separation을 -1%로 둔 미보정 합성 profile입니다.
이 수치는 실측 보정값이 아니며 백래시, 추가 slip model과 통신 지연은 포함하지
않습니다. `/wheel/odom`은 평가 기준인 `/ground_truth/odom`과 구분해서 사용해야
합니다.

## 실행

```bash
ros2 launch cleany_base_odometry wheel_odometry.launch.py \
  use_sim_time:=true
```

Gazebo Cleany launch는 가상 encoder의 `/wheel_encoder/joint_states`를 입력으로
이 노드를 자동 실행합니다. 독립 실행과 실제 로봇에서는 기본 `/joint_states`를
입력으로 사용합니다. Gazebo launch의 기본 `odometry_source:=wheel`은 이 출력으로
canonical `/odom`과 `odom -> base_link` TF를 발행합니다. 해당 TF와 `/odom`의
publisher 소유권은 `cleany_gazebo_sim`에 남아 있습니다.

Gazebo에서 합성 odometry parameter 오차를 선택하려면 `wheel_odometry_config`에
`config/wheel_odometry_synthetic_error.yaml`을 지정합니다. encoder 합성 profile과
함께 사용하는 전체 명령은 `cleany_gazebo_sim/README.md`를 참고합니다.

## 검증

```bash
python3 -m pytest \
  ros2_ws/src/cleany_base_odometry/test/test_mecanum_odometry.py
```
