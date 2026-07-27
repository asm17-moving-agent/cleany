# cleany_planner

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 planner 구현은 아직 없다.

## 역할

Perception의 world state를 바탕으로 `collect`, `skip`, `store_lost_item`,
`human_review` 같은 high-level task와 skill sequence를 만든다. grasp pose, IK,
trajectory, gripper 제어는 담당하지 않는다.

## 제공 계약

초기에는 `RuleBasedPlanner`를 구현하고, 이후 `VLMPlanner` 또는 `VLAPlanner` adapter를
같은 인터페이스 뒤에 둘 수 있게 한다. Planner 결과는 Mission Manager가 해석할 수 있는
명시적 결과 계약으로 반환한다.

## 설정 및 검증

판단 정책, confidence 기준, retry와 제외 규칙은 설정 가능한 값으로 두고, planner core는
ROS 의존 없이 단위 테스트할 수 있게 유지한다.

## 관련 KB

- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
