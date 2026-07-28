# cleany_interfaces

ROS 2 `msg`, `srv`, `action` 공통 정의 패키지.

Perception, Mission Manager, Skill Executor, Dashboard Bridge가 공유하는 인터페이스를 이 패키지에 둔다.

커스텀 메시지뿐 아니라 표준 ROS 메시지를 사용하는 프로젝트 공통 토픽 계약도
이곳에 기록한다.

## Contracts

- [Mobile base](docs/mobile_base.md): `/cmd_vel` 차체 속도 명령
