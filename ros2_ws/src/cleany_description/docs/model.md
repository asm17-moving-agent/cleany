# Robot model 계약

CAD 모델은 ROS 좌표·팔 chain·시뮬레이션 형상을 공유한다. Geometry·질량·센서 TF에는 명목값과 추정값이 포함되므로 실물 검증값으로 취급하지 않는다.

## 좌표와 형상

| 항목 | 계약 |
|---|---|
| base_link, REP-103 | +X 전방 / +Y 좌측 / +Z 상방; wheel 축 +Y |
| Optical frame | +X 우측 / +Y 하방 / +Z 전방 |
| CAD wheelbase / track / radius | 0.35 / 0.6038 / 0.0635 m |
| Wheel 모델 | URDF: 500 g rigid assembly; MJCF만 passive roller joint 48개 |
| Grasp TCP | gripper_frame 기준 (0, -0.100, 0) m 고정; hinged gripper 유지 |
| Head RGB-D TF | 명목값, 실측 calibration 아님 |
| Wrist camera GT | hand-eye 수집을 위해 runtime TF에서 제외 |

## 제어 범위

Control entrypoint는 팔 10관절을 명령하고 gripper 2관절을 관측한다. Gripper 명령은 enable_gripper_command=true일 때만 허용하며 base·head·roller는 이 계약 밖이다.

Arm-only MoveIt은 include_head_camera=false, include_wheel_joints=false를 사용한다. Full description에서는 둘 다 기본 true이며, wheel false여도 fixed geometry는 유지한다.

기존 actuator는 servo position 14개와 PG42 voltage 4개다. 신규 servo DC-motor 기본값은 별도 upgrade 전까지 비활성이다. MuJoCo 3.4 backend는 임시 scene의 wheel actuator만 제거한다.

## 추정 질량과 Gazebo

| 항목 | 추정값 |
|---|---|
| 총 질량 | 15.933871 kg; PLA 추정 포함 |
| 배터리 | 5.20 kg, 195×165×175 mm |
| PG42 motor | 각 0.8 kg |

Gazebo navigation은 park 팔·head를 고정하고 wheel joint·센서를 부착한다. 이 설정은 실물 관성·센서 보정을 증명하지 않는다.

관련 안내: [MuJoCo backend](../../cleany_mujoco_sim/docs/backends.md).
