# Mobile base contract

MuJoCo, Gazebo, 실제 로봇의 mobile base는 다음 ROS 2 계약을 공통으로 따른다.

## Velocity command

- 기본 토픽 이름: `/cmd_vel`
- 메시지 타입: `geometry_msgs/msg/Twist`
- 기준 좌표계: `base_link`
- `linear.x`: 전진(+) 또는 후진(-) 속도, 단위 `m/s`
- `linear.y`: 좌측(+) 또는 우측(-) 횡이동 속도, 단위 `m/s`
- `angular.z`: 반시계(+) 또는 시계(-) 회전 속도, 단위 `rad/s`

`Twist`에는 frame ID가 없으므로 모든 값은 `base_link` 기준으로 해석한다.
구현에서는 namespace와 remapping을 지원할 수 있도록 상대 이름 `cmd_vel`을
구독하되, 별도 remapping이 없을 때 공개되는 기본 토픽은 `/cmd_vel`이다.

### Command handling

- `linear.z`, `angular.x`, `angular.y`는 지원하지 않으며 발행자는 `0`으로
  보내야 한다. 0이 아닌 값이 들어오면 수신자는 경고하고 해당 축을 무시한다.
- 여섯 축 중 하나라도 `NaN` 또는 무한대이면 메시지 전체를 폐기하고 정지
  목표를 적용한다.
- 수신자는 `linear.x`, `linear.y`, `angular.z`를 각각 설정된 최대 절댓값으로
  제한한다.
- 속도 제한값과 command timeout은 양의 유한 설정값이어야 한다. 구체적인
  수치는 실물 사양과 안전 검토 후 확정하며 공통 계약에서 하드코딩하지 않는다.
- timeout은 `Twist`에 timestamp가 없으므로 메시지를 수신한 monotonic time을
  기준으로 판단한다. 설정 시간 동안 새 명령이 없으면 정지 목표를 적용한다.
- 정지 목표는 제어 목표를 0으로 만드는 것을 뜻한다. 실제 감속 거리와 정지
  시간은 각 backend의 controller와 하드웨어 특성에 따른다.

이 토픽은 연속 명령 스트림이며 성공 응답을 반환하지 않는다. 발행자는 설정된
timeout보다 충분히 빠르게 명령을 반복해서 보내야 하고, 실제 이동 결과는
`/odom` 등의 상태 토픽으로 확인한다.

한 시점에는 하나의 명령원만 `/cmd_vel`을 제어해야 한다. Nav2, teleop, 시험
명령 사이의 선택과 우선순위는 별도 command mux에서 처리한다. Mission Manager는
`/cmd_vel`을 직접 발행하지 않고 Navigator/Nav2 계층에 주행을 위임한다.

## Internal controller boundary

`/cmd_vel`은 차체 속도까지만 정의하는 외부 계약이다. 메카넘 역기구학으로 만든
휠별 목표 속도, Jetson과 MCU 사이의 통신 형식, encoder feedback, PID, PWM은
backend 내부 계약으로 별도 정의한다.
