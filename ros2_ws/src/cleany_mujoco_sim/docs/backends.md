# MuJoCo backend 계약

한 실행에서는 backend 하나만 사용한다. Joint state, sensor, TF, controller의 소유자를 중복시키지 않는다.

## Backend 종류

| Backend | 책임 |
|---|---|
| Custom simulation | base velocity, joint feedback, LiDAR/depth |
| Study-cafe ros2_control | MuJoCo, 팔 FollowJointTrajectory, 관찰 카메라와 sorting |
| Study-cafe manipulation | 검출, 후보 선택, 접근과 sorting |

## 모델과 시간

기하와 센서의 원본은 패키지 config/와 cleany_description 모델이다. CAD geometry는 wheelbase 0.350 m, separation 0.6038 m, radius 0.0635 m이며 실물 측정값과 구분한다.

Custom backend는 capture stamp를 node clock으로 만들며 자체 `/clock`을 발행하지 않는다. Study-cafe ros2_control backend는 simulation clock을 사용하고 소비자는 같은 `use_sim_time`을 사용한다. Hand-eye는 [동기화 계약](../../cleany_handeye_calibration/docs/frames-and-data.md), MoveIt은 [Planning 계약](../../cleany_moveit_config/docs/planning.md)을 따른다.

## 실행과 한계

Custom/ros2_control backend의 동시 실행은 joint 명령 경쟁을 일으키므로 허용하지 않는다. MuJoCo 결과는 실물 구동, 집기, 제동거리 검증이 아니다.

실행 인자는 launch/*.launch.py와 `ros2 launch ... --show-args`에서 확인한다.

관련 안내: [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md).

노드별 통신 목록: [인터페이스 표](interfaces.md).

현행 장면과 실행은 [Study-cafe 안내](scenes.md)를 따른다. 이전 hand-eye backend와 RGB-D pick demo는 제거됐다.
