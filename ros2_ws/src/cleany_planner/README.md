# cleany_planner

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 planner 구현은 아직 없다.

## 역할

Perception의 최신 Scene State와 이전 실행 결과를 바탕으로 다음 high-level 행동,
보류 또는 완료를 제안한다. Mission Manager가 허용 Capability와 인자를 검증한다.
분실물 후보의 물리적 처리 행동은 아직 미정이다. grasp pose, IK, trajectory와
gripper 제어는 Skill Executor 및 실행 backend가 담당한다.

## 제공 계약

초기에는 `RuleBasedPlanner`로 E2E 계약을 검증하고, 로컬 VLM 또는 API VLM adapter를
같은 고수준 제안 경계에서 비교한다. 최종 adapter는 미정이다. Planner 결과는 Mission
Manager가 검증할 수 있는 명시적 계약으로 반환한다. VLA policy는 Manipulation Skill
내부의 물리 실행 후보이며 고수준 Planner와 구분한다.

## 설정 및 검증

판단 정책, confidence 기준, retry와 제외 규칙은 설정 가능한 값으로 두고, planner core는
ROS 의존 없이 단위 테스트할 수 있게 유지한다.

## 관련 KB

- [Task Planning and Robot Capabilities](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
