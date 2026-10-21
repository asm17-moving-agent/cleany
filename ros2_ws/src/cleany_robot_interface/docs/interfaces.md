# 실물 상태와 controller 경계

`reader.py`와 `robot_state_reader.py`는 READ 명령으로 encoder 상태를 조회한다. 실물 제어는 `real_control.launch.py`가 선택한 `cleany_feetech_hardware`와 controller manager가 소유한다. 동시에 같은 serial port를 소유하는 reader와 controller를 시작하지 않는다.

| 노드 | 통신 | 의미 |
|---|---|---|
| `arm_state_reader` | `joint_states`, `sensor_msgs/JointState` 발행 | 설정된 팔의 실측 상태 |
| `read_only_robot_state` | `/joint_states`, `sensor_msgs/JointState` 발행 | 팔과 헤드의 실측 상태, 기본 20 Hz |
| `real_control_state_guard` | `joint_state_broadcaster/joint_states` 구독 | controller의 원본 상태 |
| 같은 guard | `joint_states`, `sensor_msgs/JointState` 발행 | 정상성과 신선도를 확인한 상태 |
| 같은 guard | `real_control/status`, `std_msgs/String` 발행 | transient-local 상태, 20 Hz 판정 |
| 같은 guard | `real_control/stop`, `std_srvs/Trigger` 제공 | controller 정지와 활성 Goal 취소 요청 |
| 같은 guard | `controller_manager/switch_controller` 호출 | controller 상태 전환 |
| `wrist_rgb_camera` | `/{arm}_wrist_camera/image_raw`, `camera_info` 발행 | SensorData QoS 영상과 보정 내참수 |
| 같은 카메라 | `/{arm}_wrist_camera/preview/compressed` 발행 | 미리보기 JPEG |
| 같은 카메라 | `/{arm}_wrist_camera/set_enabled`, `std_srvs/SetBool` 제공 | USB 장치 열기와 닫기 |
| 같은 카메라 | `/{arm}_wrist_camera/enabled`, `std_msgs/Bool` 발행 | reliable/transient-local, 1 Hz |
| `rgbd_frame_bridge` | `/tf_static` 발행 | 명시한 RGB-D 설정의 고정 frame 연결 |

Guard의 기본 상태 최대 나이는 0.25초이며 기동 대기는 30초다. 오래되거나 누락된 상태를 새 측정으로 바꾸지 않는다. 정지 요청 수락과 controller terminal 결과, 실제 정지 피드백은 서로 다른 근거다.

손목 카메라는 기본 `start_enabled=false`다. 상태 조회와 stop 진입을 막지 않으며, 취소와 실패 시 카메라 종료가 모터 정지를 지연시키지 않도록 실행기가 관리한다.


`motor_telemetry_recorder`는 `/joint_state_broadcaster/dynamic_joint_states` (`control_msgs/DynamicJointState`, SensorData QoS)를 읽어 stdout JSONL로 기록한다. serial 장치를 직접 읽거나 controller 명령을 보내지 않는다.

## TCP 지점 저장과 재방문

`tools/tcp-waypoints init`는 `--config`의 camera, board와 로봇별 operating profile을 읽어 세션 모델과 hash를 고정한다. 예시 정책의 operating profile은 `~/.config/cleany/` 아래 외부 YAML을 가리킨다. `init`는 장치에 명령하지 않으며 이후 hold, 저장과 재방문은 명시적인 명령이다. 카메라 intrinsics와 원본 capture manifest가 없으면 세션 준비를 거부한다.

`tcp_waypoints.launch.py`는 세션 모델의 native controller와 plan-only MoveIt을 연결한다. ROS transport는 `joint_states`, controller의 `FollowJointTrajectory`, MoveIt의 `GetMotionPlan`과 `GetStateValidity`를 사용한다. 선택한 팔 5관절 이외의 명령, 머리 명령과 gripper trajectory를 허용하지 않는다. 수동 hold 인수와 물리 정지 확인은 구분한다.

`tools/{left,right}-arm-torque-on`도 `--control-config /absolute/local-limits.yaml`을 요구한다. OFF 명령은 제어 profile 없이 선택한 모터의 torque 해제를 요청한다. 명령 성공은 보고된 register readback이며 외부 하중과 물리 정지의 증거를 대신하지 않는다.
