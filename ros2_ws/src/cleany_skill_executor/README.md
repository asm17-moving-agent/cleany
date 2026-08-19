# cleany_skill_executor

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 실제 trajectory와
gripper 명령은 실행하지 않는다.

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

## 관련 KB

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
