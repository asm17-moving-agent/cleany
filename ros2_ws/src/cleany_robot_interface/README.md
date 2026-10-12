# cleany_robot_interface

Mock·Sim·Real backend를 공통 robot port 뒤에 연결하기 위한 예정 영역이다.
현재는 scaffold이며 ROS 2 package manifest와 adapter 구현은 없다.

Mission Manager와 Skill Executor가 구체 하드웨어에 직접 의존하지 않도록
navigation·manipulation·sensor·안전 제어 adapter의 경계를 둔다.
실물 좌표계와 안전 한계는 adapter 또는 설정에서 관리한다.
구현 시 Mock·Sim·Real 각각의 계약 준수를 검증한다.

목표 책임은 [KB 아키텍처](../../../docs/cleany-docs/20_TECHNICAL/11%20-%20ROS%202%20Software%20Architecture.md),
현재 구현된 연동 계약은 [INTEGRATION](../../../docs/INTEGRATION.md)을 따른다.
