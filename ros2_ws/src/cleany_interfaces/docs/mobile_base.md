# Mobile base 계약

- 적용: MuJoCo·Gazebo·실물 base
- 기본 토픽: `/cmd_vel`, 타입: `geometry_msgs/msg/Twist`
- 좌표: base_link. Twist에 frame ID 없음
- 구독: 상대 이름 `cmd_vel`; namespace/remapping 허용

| 축 | 양수 방향 | 단위 |
|---|---|---|
| linear.x | 전진 | m/s |
| linear.y | 좌측 | m/s |
| angular.z | 반시계 회전 | rad/s |

## 명령 처리

- linear.z/angular.x/angular.y: 발행값 0. Nonzero 수신 시 경고·무시
- 여섯 축 중 NaN/Inf 포함: 메시지 전체 폐기·정지 목표 적용
- x/y/yaw: 각 축의 최대 절댓값으로 제한
- 제한·timeout: 양의 유한 설정값. 실물 사양·안전 검토로 결정
- Timeout: 수신 monotonic 기준. 새 명령 없으면 목표 0
- 목표 0: 실제 정지 완료 아님. 감속 거리·시간은 backend별 검증
- 발행 주기: timeout보다 충분히 짧게 반복
- 성공 응답 없음. 이동 결과는 `/odom` 등 상태로 확인

## 소유권

- 한 시점에 명령원 하나. Nav2/teleop/시험 명령 선택은 별도 mux
- Mission Manager: cmd_vel 직접 발행 없이 Navigator/Nav2에 위임
- 차체 Twist만 공통 계약. Wheel 목표·MCU 통신·encoder·PID·PWM은 backend 내부 계약
