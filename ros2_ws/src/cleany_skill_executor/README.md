# cleany_skill_executor

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 운영 action은 실제
trajectory와 gripper 명령을 실행하지 않는다. 별도 시뮬레이션 데모에서만 선택 결과를
MuJoCo arm controller로 실행할 수 있다.

## Reachable grasp action

`grasp/select_reachable` (`SelectReachableGrasp`)는 가까운 팔부터 position-only IK,
state validity, current→pre-grasp와 pre-grasp→grasp plan을 검사한다. pre-grasp는 접근
벡터 반대 방향 0.08 m다. 후보별 IK/충돌/plan 실패는 다음 arm 또는 후보로 fallback한다.

## 제공 계약

실행 결과는 Mission Manager가 상태 전이에 사용할 수 있는 성공·실패·차단 결과로 반환한다.
Skill Executor는 Mission Manager의 FSM 상태를 직접 변경하지 않는다.

## 설정 및 검증

입력 후보는 snapshot/object/frame/target OBB가 같아야 한다. 현재 12개 arm/gripper
joint는 완전하고 0.5초 이내여야 한다. action 동안 target OBB와 ACM은 임시 변경되고
모든 종료 경로에서 복원된다. 한 번에 goal 하나만 처리한다.

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py
ros2 launch cleany_skill_executor grasp_selection.launch.py
pytest -q ros2_ws/src/cleany_skill_executor/test
```

timeout, planning attempt/scaling, 최대 후보 수는 `config/grasp_selection.yaml`의 ROS
parameter로 설정한다.

## MuJoCo 육안 확인 데모

아래 단일 launch는 실제 `mujoco_ros2_control` backend, MoveIt, RViz, grasp selector와
demo coordinator를 함께 시작한다. 기본값은 MuJoCo native viewer와 RViz를 모두
표시한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py
```

데모는 의도적으로 최고 점수의 도달 불가 후보를 먼저 검사한 뒤, 초록색 MuJoCo box와
정렬된 두 번째 후보를 왼팔로 선택한다. 선택 action에서 position-only IK, 두 endpoint의
collision/state validity, 두 구간 OMPL plan-only를 통과해야만 demo coordinator가
`left_arm_controller`로 pre-grasp와 grasp trajectory를 차례로 실행한다. 마지막에는
실제 `/joint_states`가 선택 결과에 수렴했는지도 검사한다. Gripper close, attach, lift는
아직 실행하지 않는다.

RViz의 `Grasp Candidates` display에서 구/화살표/상태 문구를 보고,
`MotionPlanning` display에서는 계획 궤적과 실제 joint state를 확인한다. MuJoCo 창의
초록색 box가 target이며 왼팔이 먼저 pre-grasp에서 멈춘 다음 box까지 접근한다. 창이
준비될 시간을 위해 평가 전 5초, 각 실행 구간 사이 3초를 기본 대기한다. 빠른 headless
회귀 검증은 다음처럼 실행한다.

```bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py \
  headless:=true use_rviz:=false \
  demo_start_delay_sec:=0.1 stage_hold_sec:=0.1
```

로그의 `Selected candidate=1 arm=left`, 두 개의
`MoveIt execution succeeded`, `DEMO COMPLETE`가 전체 성공 기준이다. 데모는 완료
자세와 marker를 유지하므로 종료는 `Ctrl-C`, 다시 보기는 launch 재실행으로 한다.

## 관련 KB

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
