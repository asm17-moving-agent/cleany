# cleany_mission_manager

Mission Manager FSM과 mission lifecycle을 담당하는 ROS 2 패키지입니다.

상세 설계는 docs submodule의 [Mission Lifecycle](../../../docs/cleany-docs/20_TECHNICAL/09%20-%20Mission%20Lifecycle.md)을 기준으로 봅니다. 이 패키지 README는 구현 위치와 검증 명령만 유지합니다.

## 역할

- 외부 mission request를 받아 mission lifecycle을 시작합니다.
- Navigator, Perception, Planner, Skill Executor를 순서대로 호출합니다.
- 각 모듈이 반환한 `ModuleResult`를 해석해 FSM 상태, retry, report, error 전이를 결정합니다.
- 다른 모듈이 Mission Manager의 상태를 직접 변경하지 못하게 FSM 상태 전이 권한을 한 곳에 둡니다.

## 제공 계약

- 순수 core의 `MissionRequest`, `MissionReport`, `ModuleResult`와 module port protocol을 제공합니다.
- Navigator, Perception, Planner, Skill Executor는 결과를 반환하며, FSM 상태 전이는 이 패키지의 Manager만 수행합니다.
- 현재 공개 ROS topic, service, action은 아직 정의하지 않았습니다. 패키지 경계 인터페이스가 추가되면 `cleany_interfaces`에서 명시합니다.

현재 core는 한 번의 관측과 계획 뒤 plan의 skill sequence를 실행합니다.
최신 KB의 행동별 승인과 재관찰, 작업 후 관찰, 취소 경로 및 새 Manipulation Action
adapter는 추가 구현 대상입니다. 설계 문서가 갱신됐다고 이 흐름이 구현된 것은 아닙니다.

## 설정

retry 및 mission 정책은 향후 `configs/mission/` 또는 ROS parameter로 이동할 수 있게 유지합니다. 안전·범위 정책의 확정값을 core에 하드코딩하지 않습니다.

## 개발 명령

repo root에서 실행합니다.

```bash
make test-mission
make build
```

## 관련 KB와 갱신 규칙

- 패키지 core logic은 가능하면 ROS 의존 없이 유지해 pytest로 검증합니다.
- FSM 상태, 책임 경계, retry/report 구현을 바꾸면 이 README를 함께 갱신합니다.
  KB와 충돌하는 정책은 임의 확정하지 않으며 KB 수정은 명시 요청이 있을 때 수행합니다.
- MVP 범위나 안전 정책이 아직 검토 중이면 [기획 KB 안내](../../../docs/cleany-docs/README.md)와 [Planning Questions](../../../docs/cleany-docs/10_PLANNING/99%20-%20Questions.md)를 우선 확인합니다.
