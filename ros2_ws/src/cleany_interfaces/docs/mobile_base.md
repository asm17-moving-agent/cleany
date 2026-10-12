# Mobile base 계약

MuJoCo·Gazebo·실물 base는 차체 속도를 같은 ROS 계약으로 받는다. `/cmd_vel`은 연속 명령 스트림이며 성공 응답을 반환하지 않는다. 실제 이동은 `/odom` 등의 상태로 확인한다.

## 인터페이스

| 항목 | 계약 |
|---|---|
| 기본 토픽 / 타입 | /cmd_vel / geometry_msgs/msg/Twist |
| 좌표계 | base_link; Twist에는 frame ID 없음 |
| 구독 이름 | 상대 이름 cmd_vel; namespace/remapping 허용 |
| linear.x / linear.y | 전진 / 좌측이 양수, m/s |
| angular.z | 반시계 회전이 양수, rad/s |

## 명령 처리

수신자는 x/y/yaw를 각 축의 최대 절댓값으로 제한한다. 제한값과 timeout은 양의 유한 설정값이며, 실물 사양과 안전 검토로 결정한다.

Twist에는 stamp가 없으므로 timeout은 수신 monotonic 시각으로 판단한다. 발행자는 timeout보다 충분히 짧은 주기로 명령을 반복해야 한다. 목표 0은 실제 정지 완료를 의미하지 않으며, 감속 거리와 시간은 backend별로 검증한다.

| 입력 조건 | 처리 |
|---|---|
| linear.z / angular.x / angular.y가 nonzero | 경고 후 해당 축 무시; 발행자는 0 사용 |
| 여섯 축 중 NaN/Inf 포함 | 메시지 전체 폐기, 정지 목표 적용 |
| Timeout 동안 새 명령 없음 | 정지 목표 적용 |

## 소유권

한 시점에 명령원 하나만 cmd_vel을 제어한다. Nav2·teleop·시험 명령의 선택은 별도 mux가 담당한다. Mission Manager는 cmd_vel을 직접 발행하지 않고 Navigator/Nav2에 주행을 위임한다.

이 계약은 차체 Twist까지만 정의한다. Wheel 목표·MCU 통신·encoder·PID·PWM은 backend 내부 계약이다.
