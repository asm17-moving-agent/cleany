# MuJoCo backend 계약

한 실행에서는 backend 하나만 사용한다. Joint state·sensor·TF·controller의 소유자를 중복시키지 않는다.

## Backend 종류

| Backend | 책임 |
|---|---|
| Custom simulation | base velocity·joint feedback·LiDAR/depth·clock |
| Hand-eye ros2_control | MuJoCo·팔 FollowJointTrajectory·calibration camera |
| RGB-D pick demo | 검출·후보 선택·접근 경로 시각 확인 |

## 모델과 시간

기하·센서의 원본은 패키지 config/와 cleany_description 모델이다. CAD geometry는 wheelbase 0.350 m, separation 0.6038 m, radius 0.0635 m이며 실물 측정값과 구분한다.

Backend가 sim time과 capture stamp를 발행하고 소비자는 같은 use_sim_time을 사용한다. Hand-eye는 [동기화 계약](../../cleany_handeye_calibration/docs/frames-and-data.md), MoveIt은 [Planning 계약](../../cleany_moveit_config/docs/planning.md)을 따른다.

## 실행과 한계

Custom/ros2_control backend의 동시 실행은 joint 명령 경쟁을 일으키므로 허용하지 않는다. MuJoCo 결과는 실물 구동·집기·제동거리 검증이 아니다.

실행 인자는 launch/*.launch.py와 `ros2 launch ... --show-args`에서 확인한다.

관련 안내: [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md).
