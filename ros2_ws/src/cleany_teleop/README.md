# Cleany Teleop

폰 조종의 입력 유효시간, STOP과 ENABLE 확인 및 ROS 전송을 담당한다.
매핑 상태의 수신 지연, 권한 RPC 거절 사유와 조종 사건 기록은
[인터페이스와 진단 규약](docs/interfaces.md)에서 확인한다.

공통 [실행 절차](../../../docs/RUNNING.md)를 따라 실행하고,
저장소 루트에서 `make test-hardware-mapping-core`로 입력과 오류 처리 로직을 검증한다.
ROS 인터페이스를 변경한 경우 `make build-hardware-mapping`과
`make test-hardware-mapping`을 실행한다.
