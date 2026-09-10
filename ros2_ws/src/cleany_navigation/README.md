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

## 실물 LiDAR와 wheel odometry 표시

실물 mapping에는 `/scan`과 다음 TF 연결이 필요합니다.

```text
map --slam_toolbox--> odom --hardware_wheel_odometry--> base_link
                                                        |
                                               measured static TF
                                                        |
                                                    lidar_link
```

`cleany_base_odometry`의 `hardware_odometry.launch.py`가 실제 엔코더에서
`/wheel/odom`, `/odom`, `odom -> base_link`를 발행합니다. SLAM은 scan 시각의
TF로 odometry를 참조하므로 odom 토픽 등록만으로 연결을 확인하지 않습니다.
LiDAR driver의 `header.frame_id`와 장착 TF의 child frame을 일치시킵니다.
`base_link -> lidar_link`에는 실물 장착 위치/방향을 사용하며 simulation의
mount profile을 실측값으로 대체하지 않습니다. 임시 영점 TF를 사용한 화면은
장착 위치가 검증되지 않은 미리보기로 구분합니다.

센서와 TF가 준비된 상태에서 아래 명령을 각각 실행합니다.

```bash
ros2 launch cleany_navigation slam_mapping.launch.py use_sim_time:=false
rviz2 -d "$(ros2 pkg prefix --share cleany_navigation)/rviz/hardware_mapping.rviz"
```

`hardware_mapping.rviz`는 `/map`의 OccupancyGrid, `/scan`, `/wheel/odom`, TF를
`map` 좌표계에 표시합니다. scan은 best-effort, map은 transient-local QoS를
사용합니다. 원격 RViz는 SSH X11 forwarding으로 실행할 수 있습니다. 이 구성은
로봇 주행 명령, LiDAR driver 기동, 장착 TF 측정, 부팅 시 자동 실행을 포함하지
않습니다. 동일한 TF edge를 다른 SLAM, EKF, simulator와 중복 발행하지 않습니다.
