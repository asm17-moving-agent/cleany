# MuJoCo backend 계약

한 실행에서 다음 backend 중 하나만 구동한다. joint state·센서·TF·controller의 소유자를 중복시키지 않는다.

| Backend | 책임 |
|---|---|
| Custom simulation | base velocity, joint feedback, LiDAR/depth와 시뮬레이션 clock |
| Hand-eye ros2_control | MuJoCo 시스템과 팔 FollowJointTrajectory controller, calibration camera |
| RGB-D pick demo | 검출·후보 선택·접근 경로의 시각 확인 |

MuJoCo 결과는 실물 구동·gripper 집기·제동거리 검증이 아니다.
기하·센서 설정은 이 패키지 config/와 cleany_description 모델 원본이 관리한다.
CAD base의 wheelbase 0.350 m, separation 0.6038 m, radius 0.0635 m는 실물 측정값과 구분한다.
ROS sim time과 이미지 capture stamp는 backend가 발행한다. 소비자는 같은 use_sim_time 기준을 사용한다.
Hand-eye backend는 [frames/data](../../cleany_handeye_calibration/docs/frames-and-data.md)의 동기화 계약을 따른다.
MoveIt backend는 [planning](../../cleany_moveit_config/docs/planning.md)을 따른다.

실행 인자 원본은 launch/*.launch.py다. `ros2 launch ... --show-args`로 확인한다.
Custom backend와 ros2_control backend를 동시에 실행해 joint 명령을 경쟁시키지 않는다.
실행은 [RUNNING](../../../../docs/RUNNING.md), 테스트 범위는 [TESTING](../../../../docs/TESTING.md)를 따른다.
