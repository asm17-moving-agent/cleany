# Cleany 단일 축 Nav2 컨트롤러

DWB `trajectory_generator_name: cleany_axis_controller::AxisGenerator`로 사용한다.
전후 X, 좌우 Y, 제자리 회전 중 하나의 속도만 가진 후보와 정지 후보를
컨트롤러 안에서 생성한다. 출력 이후 임의로 축을 자르지 않는다.
DWB의 footprint 비용 평가와 가속·감속 rollout을 사용하며,
`discretize_by_time: true`, `time_granularity: 0.05`로 제동 궤적을 샘플링한다.

움직이는 동안은 측정된 현재 축과 정지 후보만 허용한다. 다른 축을
사용하려면 `/wheel/odom` 속도가 선형 0.01m/s, 회전 0.02rad/s 이하로
0.3초 연속 유지되어야 한다. 혼합 축 움직임이나 비정상 측정은 정지만
허용한다. `axis_linear_stopped`, `axis_angular_stopped`, `axis_settle_s`로
임계값을 설정한다. 물리적 관성·접촉·양자화로 실제 속도가 완벽하게
단일 축일 것을 보장하지 않으며 실제 하드웨어 안전 인증은 아니다.

`make build-gazebo`로 빌드한다. 활성 프로필의 install 환경을 source한 뒤
`ros2_ws/build-harmonic/cleany_axis_controller/test_axis_generator`로
단일 축 후보, 정지 후 축 전환, 혼합 입력 제동 및 제동 궤적을 검증한다.

2026-09-10 검증: 단위 테스트 3개 통과. Gazebo의 짧은 NavigateToPose
목표 3개(횡이동/전진/회전)가 action success로 종료됐다. Nav2 출력
635개, 최종 가드 출력 855개에서 혼합 축 명령은 0개였다.
이는 단일 축 동작 검증이며 전체 맵 목적지 도달이나 동적 사람 회피
성공을 의미하지 않는다. `make test-gazebo`에도 플러그인 테스트를 포함한다.

`cleany_yield_wait_bt_node` 라이브러리는 Nav2 Wait action을 사용하는
`YieldWait` BT 노드를 제공한다. `wait_duration`은 양의 유한 초 단위이며
의도적인 양보에는 recovery count를 증가시키지 않는다. 취소와 결과 처리는
Nav2 BtActionNode를 사용한다. navigator의 `plugin_lib_names`에 등록한다.
플러그인 등록 헤더는 Nav2가 제공하는 Humble의 BehaviorTree.CPP v3 경로와
Jazzy의 v4 경로를 지원한다. factory 헤더는 등록 구현 파일에서만 포함한다.
