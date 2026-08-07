# cleany_interfaces

## 상태

Perception, Mission Manager, Skill Executor, Dashboard Bridge가 공유하는
인터페이스를 이 패키지에 둔다. 커스텀 메시지뿐 아니라 표준 ROS 메시지를 사용하는
프로젝트 공통 topic 계약도 이곳에 기록한다.

아직 ROS 2 package manifest와 실제 `msg`·`srv`·`action` 정의는 추가되지 않았으므로,
이 디렉터리를 런타임 인터페이스 패키지로 사용하면 안 된다.

## Contracts

- [Mobile base](docs/mobile_base.md): `/cmd_vel` 차체 속도 명령

## 설정 및 검증

인터페이스를 추가하거나 변경할 때는 해당 의존 패키지, `package.xml`, 빌드 및 메시지
호환성 검증을 함께 갱신한다.

## 관련 KB

- [Technical Overview](../../../docs/cleany-docs/20_TECHNICAL/00%20-%20Technical%20Overview.md)
- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
