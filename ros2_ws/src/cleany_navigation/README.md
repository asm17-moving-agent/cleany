# cleany_navigation

Cleany의 실제 로봇과 simulation에서 공통으로 사용하는 mapping, localization,
navigation 통합 패키지입니다. 현재는 2D SLAM 설정과 launch를 제공하며 AMCL과 Nav2
구성은 검증 후 같은 패키지에 추가합니다. SLAM 알고리즘은 외부 `slam_toolbox`와
`cartographer_ros` 패키지가 제공하며, 이 패키지는 Cleany의 `/scan`, `/odom`, `/tf`
계약에 맞게 기동합니다.

Gazebo bridge, world, rosbag 기록과 평가 지표 계산은 이 패키지의 책임이 아닙니다.

## Mapping

실제 센서 시간을 사용할 때:

```bash
ros2 launch cleany_navigation slam_mapping.launch.py
```

simulation 또는 rosbag의 `/clock`을 사용할 때:

```bash
ros2 launch cleany_navigation slam_mapping.launch.py use_sim_time:=true
```

Cartographer 2D는 다음과 같이 실행합니다.

```bash
ros2 launch cleany_navigation cartographer_mapping.launch.py
ros2 launch cleany_navigation cartographer_mapping.launch.py \
  configuration_basename:=cartographer_2d_imu.lua
```
