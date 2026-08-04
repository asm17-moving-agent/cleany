# cleany_interfaces

## 상태

현재는 설계 경계를 드러내는 scaffold다. 아직 ROS 2 package manifest와 실제
`msg`·`srv`·`action` 정의는 추가되지 않았으므로, 런타임 인터페이스로 사용하면 안 된다.

## 역할

Perception, Mission Manager, Skill Executor와 향후 외부 연동 계층이 공유할 ROS 2
`msg`, `srv`, `action` 계약을 둔다.

## 제공 계약

구체 메시지 타입과 호환성 정책은 구현과 함께 정의한다. 패키지 경계를 넘는 데이터는
개별 구현체가 아니라 이 패키지의 명시적 인터페이스를 통해 전달하는 것을 목표로 한다.

## 설정 및 검증

인터페이스를 추가하거나 변경할 때는 해당 의존 패키지, `package.xml`, 빌드 및 메시지
호환성 검증을 함께 갱신한다.

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
