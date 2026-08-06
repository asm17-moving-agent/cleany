# cleany_mujoco_sim

XLeRobot MuJoCo 시뮬레이션을 ROS 2 `ament_python` 패키지로 연결한다.

## 상태와 책임

이 패키지는 구현된 시뮬레이션 브리지이며, 전체 로봇 인터페이스는 아니다.
MuJoCo 장면을 불러오고 시뮬레이션 상태를 발행하며, 테스트용 관절 위치 직접 명령을
받는다. 운영 환경의 내비게이션, 매니퓰레이션 및 하드웨어 어댑터는 이 패키지의
범위에 포함하지 않는다.

## 실행

아래 명령은 레포지토리 루트에서 실행한다.

헤드리스 시뮬레이션:

```bash
make sim
```

MuJoCo 뷰어를 포함한 시뮬레이션:

```bash
make build
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
```

## 테스트

레포지토리 루트에서 실행한다.

```bash
make test-mujoco
make build
```

## 토픽

`mujoco_sim_node`가 발행하는 토픽:

- `joint_states` (`sensor_msgs/JointState`) - actuator-backed robot joints only;
  passive mecanum roller DOFs stay internal
- `odom` (`nav_msgs/Odometry`)
- `scan` (`sensor_msgs/LaserScan`)
- `publish_odom_tf`가 `true`이면 `tf` (`odom` -> `base_link`)
- 레이저 스캔 발행이 활성화되면 `tf_static` (`base_link` -> `laser`)

`mujoco_sim_node`가 구독하는 토픽:

- `~/joint_cmd` (`sensor_msgs/JointState`): 목표 관절 위치를 직접 설정한다.
  컨트롤러가 아닌 단순한 시뮬레이션 테스트용 인터페이스다.

아직 `/cmd_vel` 기반 베이스 명령 인터페이스는 없다. MuJoCo 모델은 메카넘 휠 네 개에
각각 독립적인 DC 모터 전압 입력을 제공한다. 차체 속도 명령을 각 입력으로 변환하려면
ROS 메카넘 명령 어댑터가 추가로 필요하다.

## 베이스 구동 모델

Each wheel uses an independent `PG42-4266-1270NE` output-shaft DC motor model.
The actuator controls are terminal voltages named `rear_left_drive`,
`rear_right_drive`, `front_left_drive`, and `front_right_drive`.
`base_link +X` is front (the default head-camera heading), and equal positive
voltage on all four inputs drives the robot toward `+X`.
Wheel rotation is positive about `base_link +Y`. A positive yaw command is
counter-clockwise about `+Z`, and odometry twist is expressed in its
`base_link` child frame as required by `nav_msgs/Odometry`.

- 정격 공급 전압: `12 V`
- 기어박스: `61:1`. 제조사가 공개한 출력 토크에는 `72%` 효율이 이미 반영되어 있다.
- 제조사 정격 출력: `103 rpm`에서 `2.94 N.m`
- 계산한 무부하 출력: `7000 / 61 rpm` (`12.017 rad/s`)
- 10% 마진을 적용한 시뮬레이션 운용 한계: `10.8 V`, `2.646 N.m`
- 디레이팅한 정격점: `92.7 rpm` (`9.708 rad/s`)
- 디레이팅한 무부하 평형점: `103.28 rpm` (`10.815 rad/s`)

MJCF 액추에이터는 기어박스 출력축에서 직접 모델링하므로(`gear=1`) 감속비와 효율을
다시 적용하지 않는다. 모터 입력은 목표 휠 속도가 아니라 전압이다. 향후 베이스
어댑터가 휠 믹싱과 폐루프 속도 제어를 담당해야 한다.

## 팔 서보 모델

양팔은 다음과 같이 Feetech 12 V 시리얼 서보를 사용한다.

- left/right shoulder pitch and elbow pitch joints: `STS3250`
- Shoulder rotation, wrist pitch/roll, jaws, and head pan/tilt: `STS3215`

모델의 출력 한계에는 제조사 사양 대비 10% 운용 마진을 적용한다.

- `STS3215` (`ST-3215-C018`): 최대 한계 `2.648 N.m`, 디레이팅한 정격 토크
  `0.883 N.m`, 무부하 한계 `4.245 rad/s`
- `STS3250` (`ST-3250-C001`): 최대 한계 `4.413 N.m`, 디레이팅한 정격 토크
  `1.412 N.m`, 무부하 한계 `7.086 rad/s`

위치 액추에이터와 관절의 힘 한계에는 최대 정지 토크의 90%를 적용한다. 관절 감쇠는
포화 상태에서 각 무부하 속도의 90%를 재현하도록 보정했다. 기존 시뮬레이션 위치
루프 게인은 유지한다. Feetech는 PID를 설정할 수 있다고 명시하지만 고정된 공장 출하
게인 하나를 공개하지 않기 때문이다. 전류, 발열 및 2초 과부하 차단 동작은 아직
모델링하지 않았다.

## 실행 인자

`mujoco_sim.launch.py`:

- `scene_path` - MuJoCo scene XML or `.xml.in` template. Defaults to
  `scenes/default.xml.in`, which includes
  `cleany_description/mjcf/cleany.xml`.
- `publish_rate_hz` - simulation publish/timer rate. Defaults to `60.0`.
- `headless` - whether to hide the MuJoCo viewer. Defaults to `true`.
- `scan_rate_hz` - laser scan publish rate. Defaults to `5.5`.
- `scan_samples` - number of rays per scan. `0` derives the sample count from
  `scan_sample_rate_hz`.

`MujocoSimNode`가 추가로 지원하는 노드 파라미터:

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

이 패키지는 전체 로봇 인터페이스가 아니라 시뮬레이션 브리지다.

구현된 기능:

- Load the authoritative Cleany MJCF from `cleany_description` into a
  simulator-owned scene.
- Simulate four 5-inch mecanum drive wheels with independent PG42 motors and
  internal passive roller contact dynamics, while exposing only the four
  drive-wheel joints through ROS.
- Step the simulator on a ROS timer.
- Publish joint state, odometry, laser scan, and TF data.
- Apply direct joint position commands for simulation tests.

아직 구현되지 않은 기능:

- `/cmd_vel` 또는 Nav2 호환 베이스 명령 처리
- ROS 메카넘 명령 어댑터와 폐루프 휠 속도 컨트롤러
- `cleany_robot_interface`, `cleany_perception` 또는 Mission FSM 연동
- 베이스 또는 매니퓰레이터용 하드웨어 현실성 기반 컨트롤러

## 관련 KB와 문서 갱신

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Navigation and Mapping](../../../docs/cleany-docs/20_TECHNICAL/05%20-%20Navigation%20and%20Mapping.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

발행 토픽, launch 파라미터, 시뮬레이션 모델 가정 또는 테스트 명령이 바뀌면 이
README도 갱신한다. 시뮬레이션 하드웨어 파라미터를 관련 KB 결정 없이 확정된 실제
하드웨어 사양으로 표현하지 않는다.
