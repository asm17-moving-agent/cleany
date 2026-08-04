# cleany_robot_interface

## 상태

현재는 설계 경계를 드러내는 scaffold다. ROS 2 package manifest와 adapter 구현은 아직 없다.

## 역할

Mock, Sim, Real 로봇을 같은 방식으로 다루기 위한 공통 인터페이스를 제공한다.
Mission Manager와 Skill Executor는 구체적인 하드웨어 구현에 직접 의존하지 않는다.

## 제공 계약

navigation, manipulation, sensor와 안전 제어의 구체 adapter는 이 인터페이스 뒤에 둔다.
실제 하드웨어 좌표계와 안전 한계는 공통 core에 넣지 않고 adapter 또는 설정으로 분리한다.

## 설정 및 검증

실제 장비 의존 설정은 `configs/robot/` 또는 명시적 adapter로 관리한다. 구현 시에는
Mock·Sim·Real adapter가 같은 계약을 지키는지 각각 검증한다.

## 관련 KB

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)
