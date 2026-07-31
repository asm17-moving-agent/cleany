# cleany_skill_executor

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 skill 구현은 아직 없다.

## 역할

`navigate_to`, `pick_object`, `place_object`, `push_object`, `inspect_area`,
`return_to_home` 같은 high-level skill을 세부 동작으로 분해하고, Robot Interface,
Nav2, MoveIt 또는 LeRobot adapter에 위임한다.

## 제공 계약

실행 결과는 Mission Manager가 상태 전이에 사용할 수 있는 성공·실패·차단 결과로 반환한다.
Skill Executor는 Mission Manager의 FSM 상태를 직접 변경하지 않는다.

## 설정 및 검증

안전 한계와 하드웨어 의존값은 설정 또는 adapter에 둔다. 구현 후에는 Mock 기반의
성공·실패·안전 차단 경로를 우선 검증한다.

## 관련 KB

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
