# Robot model 계약

## 좌표·형상

- REP-103: base_link +X 전방 / +Y 좌측 / +Z 상방; wheel 축 +Y
- Optical: +X 우측 / +Y 하방 / +Z 전방
- CAD geometry: wheelbase 0.35 m / track 0.6038 m / radius 0.0635 m
- URDF wheel: 500 g rigid assembly. MJCF만 passive roller joint 48개
- Grasp TCP: gripper_frame 기준 (0, -0.100, 0) m 고정. Hinged gripper 유지
- Head RGB-D TF: 명목값, 실측 calibration 아님
- Wrist camera GT: hand-eye 수집을 위해 runtime TF에서 제외

## 제어

- Control entrypoint: 팔 10관절 명령, gripper 2관절 관측
- Gripper 명령: `enable_gripper_command:=true`일 때만 허용
- Base·head·roller: arm control 계약 밖
- Arm-only MoveIt: include_head_camera=false, include_wheel_joints=false
- Full description: 위 옵션 기본 true. Wheel false도 fixed geometry 유지
- 기존 actuator: servo position 14개 + PG42 voltage 4개
- 신규 servo DC-motor 기본값: 별도 upgrade 전까지 비활성
- MuJoCo 3.4 backend: 임시 scene의 wheel actuator만 제거

## 추정값·시뮬레이션 한계

- 질량 추정: 총 15.933871 kg, 배터리 5.20 kg(195×165×175 mm), PG42당 0.8 kg, PLA 추정 포함
- Gazebo navigation: park 팔·head 고정, wheel joint·센서 부착
- 형상·관성·센서 TF: 실물 검증값으로 취급 금지
- 관련: [MuJoCo backend](../../cleany_mujoco_sim/docs/backends.md)
